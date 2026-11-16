import glob, json, os, sys, zstandard, numpy as np
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
def seg(p):
  try: return seg0(p)
  except Exception as e: return dict(seg=os.path.basename(os.path.dirname(p)), min=0., e=[], err=str(e)[:80])
def seg0(p):
  from openpilot.cereal import log
  name = os.path.basename(os.path.dirname(p)); out = dict(seg=name, min=0., e=[])
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  lat = False; R = []; wall = None; last = -9; ver = None
  for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'carControl': lat = bool(m.carControl.latActive)
    elif w == 'initData': ver = m.initData.gitCommit[:7]
    elif w == 'clocks' and wall is None and m.clocks.wallTimeNanos > 1.7e18: wall = m.clocks.wallTimeNanos / 1e9 - t
    elif w == 'carState' and t - last >= 0.099:
      last = t; c = m.carState
      R.append((t, c.vEgo, c.steeringAngleDeg, c.steeringTorque, c.steeringPressed, lat, c.leftBlinker or c.rightBlinker, c.steeringTorqueEps))
  if len(R) < 100: return out
  A = np.array(R, dtype=float); t, v, ang, tq, prs, la, bl, eps = A.T
  out['min'] = float((la > 0).sum() * 0.1 / 60.); out['ver'] = ver
  n = len(t); k = 30
  cs = np.concatenate([[0.], np.cumsum(ang)]); lo = np.clip(np.arange(n) - k, 0, n); hi = np.clip(np.arange(n) + k + 1, 0, n)
  x = ang - (cs[hi] - cs[lo]) / (hi - lo)
  sg = np.sign(x); zc = [i for i in range(1, n) if sg[i] != 0 and sg[i] != sg[i - 1] and sg[i - 1] != 0]
  H = []
  for a, b in zip(zc[:-1], zc[1:]):
    dur = t[b] - t[a]; amp = float(np.abs(x[a:b]).max())
    ok = 1.2 <= dur <= 4.0 and amp >= 6. and (la[a:b] > 0).all() and v[a:b].min() > 6. and not (bl[a:b] > 0).any()
    H.append((a, b, amp, ok))
  i = 0
  while i < len(H):
    if not H[i][3]: i += 1; continue
    j = i
    while j + 1 < len(H) and H[j + 1][3]: j += 1
    if j - i + 1 >= 4:
      a, b = H[i][0], H[j][1]; pre = slice(max(a - 80, 0), a)
      opp = float(np.mean(np.sign(tq[a:b]) != np.sign(eps[a:b])))
      out['e'].append(dict(seg=name, ver=ver, t=float(t[a] + (wall or 0)), dur=float(t[b] - t[a]), half=j - i + 1, amp=float(np.mean([h[2] for h in H[i:j + 1]])), v=float(v[a:b].mean() * 3.6),
                           tq=float(np.abs(tq[a:b]).mean()), prs=float(prs[a:b].mean()), opp=opp, eps=float(np.abs(eps[a:b]).max()), pre_prs=float(prs[pre].mean()), pre_ang=float(np.abs(ang[pre]).max())))
    i = j + 1
  return out
if __name__ == '__main__':
  files = sorted(glob.glob(ROOT + '/raw/*/rlog.zst'))
  if len(sys.argv) > 2: files = [f for f in files if any(('/raw/' + q) in f for q in sys.argv[2:])]
  with Pool(4) as pool: res = pool.map(seg, files, chunksize=8)
  with open(sys.argv[1], 'w') as f:
    for r in res: f.write(json.dumps(r) + chr(10))
  print('DONE', len(files), round(sum(r['min'] for r in res)), sum(len(r['e']) for r in res), sum(1 for r in res if r.get('err')))
