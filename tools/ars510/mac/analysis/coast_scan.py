"""How fast does the car slow down by itself when the driver lifts off (no gas, no brake, openpilot not engaged)? (Mac)
  python3 coast_scan.py [--routes 33-4d]
Frames: carState gasPressed / brakePressed both off for >= SETTLE_S (the first second after a release is skipped), not
standstill, selfdriveState not enabled, |steering angle| < 30 deg. aEgo is corrected for the road's slope with the pitch of
liveLocationKalman (a_flat = aEgo + g sin(pitch), pitch nose-up positive). Reports the median per 10 km/h band.
"""
import glob, os, sys, zstandard, numpy as np
from multiprocessing import Pool

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log

SETTLE_S = 1.0
G = 9.81


def scan(seg):
  try:
    dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(seg + '/rlog.zst', 'rb').read())
  except Exception:
    return []
  out = []; free_since = None; en = False; pitch = float('nan')
  try:
    for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
      w = e.which()
      if w == 'selfdriveState':
        en = e.selfdriveState.enabled
      elif w == 'liveLocationKalman':
        try:
          o = e.liveLocationKalman.orientationNED
          if o.valid: pitch = float(o.value[1])
        except Exception:
          pass
      elif w == 'carState':
        c = e.carState; t = e.logMonoTime / 1e9
        free = not c.gasPressed and not c.brakePressed and not c.standstill and not en and abs(c.steeringAngleDeg) < 30
        if not free:
          free_since = None; continue
        if free_since is None: free_since = t
        if t - free_since >= SETTLE_S and c.vEgo > 3:
          out.append((c.vEgo, c.aEgo, pitch))
  except Exception:
    pass
  return out[::10]   # 10 Hz is plenty


def main():
  a = sys.argv[1:]
  lo, hi = (a[a.index('--routes') + 1].split('-') if '--routes' in a else ('33', 'ff'))
  segs = [s for s in sorted(glob.glob(os.path.join(ROOT, 'raw', '*--*--*')))
          if int(lo, 16) <= int(os.path.basename(s).split('--')[0], 16) <= int(hi, 16) and os.path.exists(s + '/rlog.zst')]
  with Pool(6) as p:
    rows = [r for rs in p.imap_unordered(scan, segs, chunksize=2) for r in rs]
  x = np.array(rows, float)
  if not len(x):
    print('no coasting frames'); return
  v = x[:, 0] * 3.6; a_raw = x[:, 1]; pit = x[:, 2]
  has_p = pit == pit
  a_flat = np.where(has_p, a_raw + G * np.sin(np.nan_to_num(pit)), np.nan)
  print(f'{len(segs)} segments, coasting samples {len(x)} ({len(x) / 10 / 60:.0f} min), with pitch {has_p.mean() * 100:.0f} %')
  print('km/h    n(min) | aEgo median p25 p75 | slope-corrected median p25 p75')
  for b in range(0, 130, 10):
    s = (v >= b) & (v < b + 10)
    if s.sum() < 100: continue
    f = s & has_p
    fs = f'{np.median(a_flat[f]):+.2f} {np.percentile(a_flat[f], 25):+.2f} {np.percentile(a_flat[f], 75):+.2f}' if f.sum() > 50 else 'n/a'
    print(f'{b:3d}-{b + 10:<3d} {s.sum() / 600:6.1f} | {np.median(a_raw[s]):+.2f} {np.percentile(a_raw[s], 25):+.2f} {np.percentile(a_raw[s], 75):+.2f} | {fs}')


if __name__ == '__main__':
  main()
