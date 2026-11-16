import glob, json, os, sys, zstandard, numpy as np
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
DT = 0.02; WIN = int(5. / DT); HOP = int(2.5 / DT); MX = int(1.5 / DT)
def xlag(a, b):
  a = a - a.mean(); b = b - b.mean(); best = (0, -9.)
  for k in range(0, MX + 1):
    x = a[:len(a) - k] if k else a; y = b[k:]
    den = np.sqrt((x * x).sum() * (y * y).sum())
    c = float((x * y).sum() / den) if den > 0 else 0.
    if c > best[1]: best = (k, c)
  return best[0] * DT, best[1]
def seg(p):
  try: return seg0(p)
  except Exception as e: return dict(seg=os.path.basename(os.path.dirname(p)), w=[], err=str(e)[:80])
def seg0(p):
  from openpilot.cereal import log
  name = os.path.basename(os.path.dirname(p)); out = dict(seg=name, w=[])
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  cmd = o = None; lat = False; R = []; last = -9; wall = None; lastprs = -99.; ld = 0.
  for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'carControl': cmd = m.carControl.actuators.steeringAngleDeg; lat = bool(m.carControl.latActive)
    elif w == 'carOutput': o = m.carOutput.actuatorsOutput.steeringAngleDeg
    elif w == 'lateralDelay': ld = m.lateralDelay.lateralDelay
    elif w == 'clocks' and wall is None and m.clocks.wallTimeNanos > 1.7e18: wall = m.clocks.wallTimeNanos / 1e9 - t
    elif w == 'carState' and cmd is not None and o is not None:
      c = m.carState
      if c.steeringPressed: lastprs = t
      if t - last >= DT - 0.001:
        last = t; R.append((t, c.vEgo, c.steeringAngleDeg, cmd, o, c.steeringTorque, c.steeringTorqueEps, lat, t - lastprs, c.leftBlinker or c.rightBlinker, c.steeringRateDeg, c.steeringPressed))
  if len(R) < WIN: return out
  A = np.array(R, dtype=float)
  for i in range(0, len(A) - WIN, HOP):
    S = A[i:i + WIN]
    if not (S[:, 7] > 0).all() or S[:, 1].min() < 3. or S[:, 11].mean() > 0.15: continue
    rng = S[:, 3].max() - S[:, 3].min()
    if rng < 4.: continue
    la, ca = xlag(S[:, 3], S[:, 2]); lo, co = xlag(S[:, 4], S[:, 2]); lc, cc_ = xlag(S[:, 3], S[:, 4])
    zc = int((np.diff(np.sign(S[:, 3] - S[:, 3].mean())) != 0).sum())
    out['w'].append([round(float(x), 3) for x in (S[0, 0] + (wall or 0), S[:, 1].mean(), la, ca, lo, co, lc, rng, np.abs(np.diff(S[:, 3])).max() / DT, np.abs(S[:, 5]).mean(), np.abs(S[:, 6]).max(),
                     S[:, 8].min(), np.abs(S[:, 2]).mean(), S[:, 9].max(), zc, ld, np.abs(S[:, 10]).max(), np.abs(S[:, 3] - S[:, 2]).max(), S[:, 11].mean())])
  return out
if __name__ == '__main__':
  files = sorted(glob.glob(ROOT + '/raw/*/rlog.zst'))
  if len(sys.argv) > 2: files = [f for f in files if any(('/raw/' + q) in f for q in sys.argv[2:])]
  with Pool(4) as pool: res = pool.map(seg, files, chunksize=8)
  with open(sys.argv[1], 'w') as f:
    for r in res: f.write(json.dumps(r) + chr(10))
  print('DONE', len(files), sum(len(r['w']) for r in res), sum(1 for r in res if r.get('err')))
