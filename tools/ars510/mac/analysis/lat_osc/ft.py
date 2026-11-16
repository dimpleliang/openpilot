import glob, json, os, sys, zstandard, numpy as np
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
GAP = 6.; EXTRA = 0.4
def curv(y, r, T, v, at):
  v = max(v, 1.0); psi = np.interp(at, T, y) if at >= 0.3 else (at / 0.3) * np.interp(0.3, T, y); return 2 * psi / (v * at) - r[0] / v
def seg(p):
  try: return seg0(p)
  except Exception as e: return dict(seg=os.path.basename(os.path.dirname(p)), e=[], sec=0., err=str(e)[:80])
def seg0(p):
  from openpilot.cereal import log
  name = os.path.basename(os.path.dirname(p)); out = dict(seg=name, e=[], sec=0., chk=[])
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  v = ang = tq = 0.; o = cmd = None; lat = False; bl = False; lastprs = -99.; ld = 0.3; wall = None; R = []; prs = False
  for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'carState':
      c = m.carState; v = c.vEgo; ang = c.steeringAngleDeg; tq = c.steeringTorque; bl = c.leftBlinker or c.rightBlinker
      if c.steeringPressed: lastprs = t
    elif w == 'carControl': cmd = m.carControl.actuators.steeringAngleDeg; lat = bool(m.carControl.latActive)
    elif w == 'carOutput': o = m.carOutput.actuatorsOutput.steeringAngleDeg
    elif w == 'lateralDelay': ld = m.lateralDelay.lateralDelay
    elif w == 'clocks' and wall is None and m.clocks.wallTimeNanos > 1.7e18: wall = m.clocks.wallTimeNanos / 1e9 - t
    elif w == 'modelV2' and o is not None and cmd is not None:
      mv = m.modelV2; T = np.array(mv.orientation.t); y = np.array(mv.orientation.z); r = np.array(mv.orientationRate.z)
      if len(T) < 5: continue
      at = ld + 0.075; c0 = curv(y, r, T, v, at); c1 = curv(y, r, T, v, at + EXTRA)
      R.append((t, v * 3.6, ang, o, cmd, float(lat), t - lastprs, float(bl), abs(tq), mv.action.desiredCurvature, c0, c1))
  if len(R) < 40: return out
  A = np.array(R); kmh = A[:, 1]
  elig = (A[:, 5] > 0) & (kmh >= 25) & (kmh < 60)
  out['sec'] = float(elig.sum() * 0.05)
  ok = elig & (np.abs(A[:, 9]) > 5e-4)
  if ok.sum() > 20: out['chk'] = [float(np.median(np.abs(A[ok, 10] - A[ok, 9]) / np.abs(A[ok, 9]))), int(ok.sum())]
  trig = elig & (np.abs(A[:, 3] - A[:, 2]) > GAP)
  idx = np.where(trig)[0]; i = 0
  while i < len(idx):
    j = i
    while j + 1 < len(idx) and idx[j + 1] - idx[j] <= 4: j += 1
    a, b = idx[i], idx[j] + 1; S = A[a:b]
    dk = S[:, 11] - S[:, 10]; vm = S[:, 1] / 3.6
    # degrees per curvature from this stretch's own command (fallback: geometric)
    kk = np.abs(S[:, 9]) > 1e-3
    dpk = float(np.median(np.abs(S[kk, 4]) / np.abs(S[kk, 9]))) if kk.sum() >= 3 else 2112.
    dpk = float(np.clip(dpk, 1500., 4000.))
    out['e'].append(dict(seg=name, t=float(S[0, 0] + (wall or 0)), dur=float((b - a) * 0.05), v=float(S[:, 1].mean()), gap=float(np.abs(S[:, 3] - S[:, 2]).max()), ang=float(np.abs(S[:, 2]).max()),
                         since=float(S[:, 6].min()), bl=float(S[:, 7].max()), tq=float(S[:, 8].mean()), dang=float(np.abs(dk).max() * dpk), dang_med=float(np.median(np.abs(dk)) * dpk),
                         dlat=float((np.abs(dk) * vm ** 2).max()), cmdamp=float(np.abs(S[:, 4]).max()),
                         same=float(np.mean(np.sign(dk) == np.sign(S[:, 10]))), flips=int((np.diff(np.sign(S[:, 10][np.abs(S[:, 10]) > 5e-4])) != 0).sum())))
    i = j + 1
  return out
if __name__ == '__main__':
  files = sorted(glob.glob(ROOT + '/raw/*/rlog.zst'))
  if len(sys.argv) > 2: files = [f for f in files if any(('/raw/' + q) in f for q in sys.argv[2:])]
  with Pool(4) as pool: res = pool.map(seg, files, chunksize=8)
  with open(sys.argv[1], 'w') as f:
    for r in res: f.write(json.dumps(r) + chr(10))
  print('DONE', len(files), round(sum(r['sec'] for r in res) / 60.), sum(len(r['e']) for r in res), sum(1 for r in res if r.get('err')))
