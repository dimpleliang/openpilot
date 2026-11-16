import json, os, sys, zstandard, datetime, numpy as np
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
tz = datetime.timezone(datetime.timedelta(hours=8))
def load(name):
  from openpilot.cereal import log
  p = ROOT + '/raw/' + name + '/rlog.zst'
  if not os.path.exists(p): return []
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  return list(log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61))
def xlag(a, b, dt=0.01, mx=1.5):
  a = a - a.mean(); b = b - b.mean(); best = (0., -9)
  for k in range(0, int(mx / dt) + 1, 2):
    x = a[:len(a) - k] if k else a; y = b[k:]
    den = np.sqrt((x * x).sum() * (y * y).sum())
    c = float((x * y).sum() / den) if den > 0 else 0.
    if c > best[1]: best = (k * dt, c)
  return best
def one(e):
  try: return one0(e)
  except Exception as ex: return dict(e, err=str(ex)[:100])
def one0(e):
  r, s = e['seg'].rsplit('--', 1); ev = load(e['seg']) + load('%s--%d' % (r, int(s) + 1))
  off = [m.clocks.wallTimeNanos / 1e9 - m.logMonoTime / 1e9 for m in ev if m.which() == 'clocks' and m.clocks.wallTimeNanos > 1.7e18][0]
  t0 = e['t'] - off - 1.; t1 = t0 + e['dur'] + 2.
  cmd = out = None; ld = None; sad = None; ll = None; R = []; B = []
  for m in ev:
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'carControl': cmd = m.carControl.actuators.steeringAngleDeg
    elif w == 'carOutput': out = m.carOutput.actuatorsOutput.steeringAngleDeg
    elif w == 'liveDelay': ld = (m.liveDelay.lateralDelay, str(m.liveDelay.status))
    elif w == 'carParams': sad = m.carParams.steerActuatorDelay
    elif w == 'modelV2':
      mv = m.modelV2; L = mv.laneLines; pr = list(mv.laneLineProbs)
      if len(L) > 2 and t0 <= t <= t1: ll.append(((L[1].y[0] + L[2].y[0]) / 2., min(pr[1], pr[2]), mv.action.desiredCurvature)) if ll is not None else None
      if ll is None: ll = []
    elif w == 'carState' and cmd is not None and out is not None:
      c = m.carState
      if t0 <= t <= t1: R.append((cmd, out, c.steeringAngleDeg, c.vEgo, c.steeringTorque, c.steeringTorqueEps))
      elif c.vEgo > 6. and c.vEgo < 14.: B.append((t, cmd, out, c.steeringAngleDeg))
  A = np.array(R, dtype=float); res = dict(seg=e['seg'], t=e['t'], amp=e['amp'], half=e['half'], v=e['v'], tq=e['tq'], prs=e['prs'], pre_ang=e['pre_ang'], ld=ld, sad=sad)
  res['cmd_ang'] = xlag(A[:, 0], A[:, 2]); res['out_ang'] = xlag(A[:, 1], A[:, 2]); res['cmd_out'] = xlag(A[:, 0], A[:, 1])
  res['cmd_amp'] = float((A[:, 0].max() - A[:, 0].min()) / 2.); res['ang_amp'] = float((A[:, 2].max() - A[:, 2].min()) / 2.)
  if ll:
    Lc = np.array(ll, dtype=float); g = Lc[:, 1] > 0.5
    res['lane_pp'] = float(Lc[g, 0].max() - Lc[g, 0].min()) if g.sum() > 20 else None; res['lane_ok'] = float(g.mean())
    dc = Lc[:, 2]; res['dc_sign_changes'] = int((np.diff(np.sign(dc[np.abs(dc) > 5e-4])) != 0).sum())
  if len(B) > 1500:
    Bb = np.array(B, dtype=float); res['base_cmd_ang'] = xlag(Bb[:, 1], Bb[:, 3]); res['base_n'] = len(B)
  return res
if __name__ == '__main__':
  E = sorted([e for l in open('osc_out.jsonl') for e in json.loads(l)['e']], key=lambda e: e['t'])
  with Pool(4) as pool: res = pool.map(one, E)
  json.dump(res, open('lag_out.json', 'w'))
  for r in res:
    s = datetime.datetime.fromtimestamp(r['t'], tz).strftime('%m/%d %H:%M:%S')
    if r.get('err'): print(s, 'ERR', r['err']); continue
    b = r.get('base_cmd_ang')
    print(s, 'v%.0f amp%.0f half%d tq%.0f prs%.2f pre%.0f | cmd>ang %.2fs c%.2f out>ang %.2fs cmd>out %.2fs | cmdAmp%.0f angAmp%.0f | lane_pp %s ok%.2f dcflip %s | liveDelay %s sad %s | base %s' % (
      r['v'], r['amp'], r['half'], r['tq'], r['prs'], r['pre_ang'], r['cmd_ang'][0], r['cmd_ang'][1], r['out_ang'][0], r['cmd_out'][0], r['cmd_amp'], r['ang_amp'],
      ('%.2f' % r['lane_pp']) if r.get('lane_pp') is not None else '-', r.get('lane_ok', 0), r.get('dc_sign_changes'), r['ld'], r['sad'], ('%.2f c%.2f' % tuple(b)) if b else '-'))
