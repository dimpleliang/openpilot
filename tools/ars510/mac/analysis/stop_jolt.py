import glob, json, os, sys, zstandard, numpy as np
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
def seg(p):
  try: return seg0(p)
  except Exception as e: return []
def seg0(p):
  from openpilot.cereal import log
  name = os.path.basename(os.path.dirname(p))
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  ACC = []; wall = None; v = a = 0.; en = False; brk = False; req = 0.; lcs = ''; ax = 0.; dec = ''; lead = None; R = []; last = -9
  for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'clocks' and wall is None and m.clocks.wallTimeNanos > 1.7e18: wall = m.clocks.wallTimeNanos / 1e9 - t
    elif w == 'selfdriveState': en = bool(m.selfdriveState.enabled)
    elif w == 'carControl': req = m.carControl.actuators.accel
    elif w == 'controlsState': lcs = str(m.controlsState.longControlState)
    elif w == 'accelerometer': ACC.append((t, -m.accelerometer.acceleration.v[2]))
    elif w == 'radarState':
      l = m.radarState.leadOne; lead = (l.dRel, l.vLead) if l.present else None
    elif w == 'logMessage' and 'decider' in m.logMessage:
      try: dec = json.loads(m.logMessage)['msg'].split('decider ')[1].split(' ')[0]
      except Exception: pass
    elif w == 'carState':
      c = m.carState; v = c.vEgo; a = c.aEgo; brk = c.brakePressed
      if t - last >= 0.019 and wall is not None:
        last = t; R.append((t + wall, v, a, float(en), float(brk), req, ax, lead[0] if lead else -1., lcs == 'stopping', hash(dec) % 1000, c.gasPressed))
        R[-1] = R[-1] + (dec,)
  if len(R) < 200: return []
  T = np.array([r[0] for r in R]); V = np.array([r[1] for r in R]); A = np.array([r[2] for r in R]); EN = np.array([r[3] for r in R]); BR = np.array([r[4] for r in R])
  RQ = np.array([r[5] for r in R]); AX = np.array([r[6] for r in R]); LD = np.array([r[7] for r in R]); DEC = [r[11] for r in R]; GAS = np.array([float(r[10]) for r in R])
  out = []
  lastst = -99
  for i in range(500, len(R) - 100):
    if V[i] < 0.05 and V[i - 1] >= 0.05 and V[i - 500:i].max() > 5.0 and T[i] - lastst > 10:
      lastst = T[i]
      w0 = i - 500; w1 = min(i + 75, len(R))
      sysstop = EN[i - 150:i + 25].min() > 0 and BR[i - 150:i + 25].max() == 0 and GAS[i - 150:i].max() == 0
      drvstop = BR[i - 50:i + 1].mean() > 0.9 and GAS[i - 150:i].max() == 0
      brk_sys = EN[i - 150:i - 50].min() > 0 and BR[i - 150:i - 50].max() == 0 and BR[i - 50:i + 1].max() > 0
      if not (sysstop or drvstop): continue
      Ac = np.array(ACC); Ac[:, 0] += wall
      ti = np.arange(T[i] - 5., min(T[i] + 1.6, Ac[-1, 0]), 0.01)
      raw = np.interp(ti, Ac[:, 0], Ac[:, 1]); sm = np.convolve(raw, np.ones(5) / 5, mode='same')
      rest = sm[(ti >= T[i] + 1.0) & (ti <= T[i] + 1.5)].mean() if (ti >= T[i] + 1.0).sum() > 20 else sm[-20:].mean()
      f = sm - rest; rel = ti - T[i]; jk = np.gradient(np.convolve(raw, np.ones(10) / 10, mode='same'), ti)
      def win(a, b): return (rel >= a) & (rel <= b)
      imu = dict(dec_last2s=float(f[win(-2., -0.3)].mean()), dec_peak_last1s=float(f[win(-1., 0.)].min()), at_stop=float(f[win(-0.05, 0.05)].mean()),
                 rebound=float(f[win(0., 0.6)].max()), jolt=float(f[win(-0.3, 0.6)].max() - f[win(-0.3, 0.6)].min()), jerk=float(np.abs(jk[win(-0.3, 0.6)]).max()),
                 ftrace=[[round(float(rel[j]), 2), round(float(f[j]), 2)] for j in range(0, len(rel), 10) if -3. <= rel[j] <= 1.5])
      pre = slice(i - 50, i + 1); end = slice(i - 15, i + 25)
      out.append(dict(seg=name, t=float(T[i]), kind='sys' if sysstop else ('sys>drv' if brk_sys else 'drv'), v3s=float(V[i - 150] * 3.6), v10=float(V[i - 500:i].max() * 3.6),
        a_v2=float(np.interp(2.0, V[w0:i + 1][::-1], A[w0:i + 1][::-1])), a_v1=float(np.interp(1.0, V[w0:i + 1][::-1], A[w0:i + 1][::-1])),
        imu=imu,
        req_v2=float(np.interp(2.0, V[w0:i + 1][::-1], RQ[w0:i + 1][::-1])), req_at_stop=float(RQ[i]), req_min=float(RQ[pre].min()), req_after=float(RQ[i + 25]) if i + 25 < len(RQ) else 0.,
        lead=float(LD[i]), dec=DEC[i], dec_pre=DEC[i - 50],
        trace=[[round(float(T[j] - T[i]), 2), round(float(V[j]), 2), round(float(RQ[j]), 2)] for j in range(i - 250, min(i + 50, len(R)), 5)]))
  return out
if __name__ == '__main__':
  files = []
  for r in sys.argv[2:]: files += glob.glob(ROOT + '/raw/' + r + '--*/rlog.zst')
  with Pool(4) as pool: res = pool.map(seg, sorted(files), chunksize=4)
  S = [s for r in res for s in r]
  json.dump(S, open(sys.argv[1], 'w'))
  print('DONE', len(S), sum(s['kind'] == 'sys' for s in S), sum(s['kind'] == 'drv' for s in S))
