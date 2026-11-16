import glob, json, os, sys, zstandard, numpy as np
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
VB = [15, 30, 45, 60, 80, 200]; RB = [5, 10, 15, 20, 25, 35, 1000]
def seg(p):
  try: return seg0(p)
  except Exception as e: return None
def seg0(p):
  from openpilot.cereal import log
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  o = None; lat = False; R = []; last = -9; lastprs = -99.
  for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'carControl': lat = bool(m.carControl.latActive)
    elif w == 'carOutput': o = m.carOutput.actuatorsOutput.steeringAngleDeg
    elif w == 'carState' and o is not None:
      c = m.carState
      if c.steeringPressed: lastprs = t
      if t - last >= 0.0999:
        last = t; R.append((t, c.vEgo * 3.6, c.steeringAngleDeg, o, float(lat), t - lastprs, abs(c.steeringTorque), c.steeringTorqueEps))
  if len(R) < 50: return None
  A = np.array(R); n = len(A); S = {}
  for i in range(3, n - 13):
    if not A[i - 3:i + 13, 4].all(): continue
    v = A[i, 1]
    if v < VB[0]: continue
    orate = (A[i, 3] - A[i - 3, 3]) / 0.3
    if abs(orate) < RB[0] or abs(A[i, 2]) > 30.: continue
    if A[i + 3, 5] < 0.4 or A[i - 3:i + 4, 6].max() > 100.: continue
    s = np.sign(orate)
    r03 = s * (A[i + 3, 2] - A[i, 2]) / 0.3; r36 = s * (A[i + 6, 2] - A[i + 3, 2]) / 0.3
    err = s * (A[i, 3] - A[i, 2])
    vb = sum(v >= x for x in VB) - 1; rb = sum(abs(orate) >= x for x in RB) - 1
    pb = 0 if A[i, 5] < 5. else (1 if A[i, 5] < 15. else 2)
    away = int(s == np.sign(A[i, 2])) if abs(A[i, 2]) > 2. else 2
    S.setdefault((vb, rb, pb, away), []).append((abs(orate), r03, r36, err, A[i, 6], s * A[i, 7]))
  return {('%d,%d,%d,%d' % k): np.array(v).round(1).tolist() for k, v in S.items()}
if __name__ == '__main__':
  files = sorted(glob.glob(ROOT + '/raw/*/rlog.zst'))
  with Pool(4) as pool: res = pool.map(seg, files, chunksize=8)
  T = {}
  for r in res:
    if r:
      for k, v in r.items(): T.setdefault(k, []).extend(v)
  json.dump(T, open('rate_out.json', 'w')); print('DONE', sum(len(v) for v in T.values()))
