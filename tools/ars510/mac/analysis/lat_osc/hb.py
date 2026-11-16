import glob, json, os, sys, zstandard, numpy as np
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
HB = 5.0; G = 0.5
def seg(p):
  try: return seg0(p)
  except Exception as e: return dict(seg=os.path.basename(os.path.dirname(p)), err=str(e)[:80], elig=0, hb=0, S=[], T=[])
def seg0(p):
  from openpilot.cereal import log
  name = os.path.basename(os.path.dirname(p))
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  v = ang = 0.; lat = False; lastprs = -99.; prs = False; wall = None; R = []
  for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'carState':
      c = m.carState; v = c.vEgo; ang = c.steeringAngleDeg; prs = c.steeringPressed
      if prs: lastprs = t
    elif w == 'carControl': lat = bool(m.carControl.latActive)
    elif w == 'clocks' and wall is None and m.clocks.wallTimeNanos > 1.7e18: wall = m.clocks.wallTimeNanos / 1e9 - t
    elif w == 'modelV2':
      mv = m.modelV2; T = np.array(mv.orientation.t); y = np.array(mv.orientation.z)
      if len(T) < 5: continue
      vv = max(v, 1.)
      kroad = (np.interp(3.0, T, y) - np.interp(1.5, T, y)) / (1.5 * vv)
      R.append((t, v * 3.6, ang, float(lat), t - lastprs, float(prs), mv.action.desiredCurvature, kroad))
  if len(R) < 40: return dict(seg=name, elig=0, hb=0, S=[], T=[])
  A = np.array(R); kmh = A[:, 1]
  elig = (A[:, 3] > 0) & (kmh >= 20) & (kmh < 60)
  hb = elig & (A[:, 4] < HB) & (A[:, 5] < 1)
  corr = A[:, 6] - A[:, 7]; dk = (1 - G) * corr; vm = kmh / 3.6
  S = np.stack([kmh, np.abs(dk) / 0.00035, np.abs(dk) * vm ** 2, np.abs(A[:, 2]), np.abs(A[:, 6]) * vm ** 2, np.abs(A[:, 7]) * vm ** 2, np.sign(A[:, 6]) * dk * vm ** 2], 1)[hb]
  # all eligible samples (not only handback) for the road-curvature sanity check
  T2 = np.stack([np.abs(A[:, 6]) * vm ** 2, np.abs(corr) * vm ** 2, np.abs(A[:, 7]) * vm ** 2], 1)[elig][::5]
  return dict(seg=name, wall=wall, elig=int(elig.sum()), hb=int(hb.sum()), S=S.round(4).tolist(), T=T2.round(4).tolist(), tt=[float(A[0, 0] + (wall or 0)), float(A[-1, 0] + (wall or 0))],
              P=[[float(a + (wall or 0)), float(b)] for a, b in zip(A[:, 0][::2], hb[::2])])
if __name__ == '__main__':
  files = sorted(glob.glob(ROOT + '/raw/*/rlog.zst'))
  with Pool(4) as pool: res = pool.map(seg, files, chunksize=8)
  S = np.array([x for r in res for x in r['S']]); T = np.array([x for r in res for x in r['T']])
  el = sum(r['elig'] for r in res); hb = sum(r['hb'] for r in res)
  np.save('hb_S.npy', S); np.save('hb_T.npy', T)
  json.dump({r['seg']: r.get('P', []) for r in res if r.get('P')}, open('hb_P.json', 'w'))
  print('DONE eligible min %.0f handback min %.0f (%.1f%%) errs %d' % (el * 0.05 / 60, hb * 0.05 / 60, 100. * hb / max(el, 1), sum(1 for r in res if r.get('err'))))
