"""How fast does the car slow down by itself when the driver lifts off (no gas, no brake, openpilot not controlling the
gas / brake)? (Mac)
  python3 coast_scan.py [--routes 33-4d]
Frames: carState gasPressed / brakePressed both off for >= SETTLE_S, openpilot longitudinal not active (carControl.longActive;
11/09: lateral-only MADS driving is real coasting too and is now included, tagged "lat"; fully manual is "man") and not
active for SETTLE_S either, not standstill, |steering angle| < 30 deg. aEgo is corrected for the road's slope with the pitch
of liveLocationKalman calibratedOrientationNED (car frame; 11/09: the device-frame pitch carried the -1.26 deg mounting
pitch, -0.2 m/s^2 on every band) (a_flat = aEgo + g sin(pitch), pitch nose-up positive). Drive mode from GEAR_PACKET (0x3BC, 1 Hz):
SPORT_ON / ECON_ON / B_GEAR_ENGAGED (bit positions from the Toyota DBC, not yet confirmed on this car by toggling the
modes). Reports the median per 10 km/h band, per tag and per drive mode.
"""
import glob, os, sys, zstandard, numpy as np
from multiprocessing import Pool

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log

SETTLE_S = 2.0   # after a pedal release or the end of openpilot longitudinal control (11/09: 1.0 -> 2.0)
G = 9.81


def scan(seg):
  try:
    dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(seg + '/rlog.zst', 'rb').read())
  except Exception:
    return []
  out = []; free_since = None; en = False; mads = False; long_on = False; pitch = float('nan'); mode = 'normal'; bgear = False
  try:
    for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
      w = e.which()
      if w == 'selfdriveState':
        en = e.selfdriveState.enabled
      elif w == 'selfdriveStateSP':
        mads = e.selfdriveStateSP.mads.active
      elif w == 'carControl':
        long_on = e.carControl.longActive
      elif w == 'can':
        for m in e.can:
          if m.address == 956 and m.src == 0 and len(m.dat) >= 6:
            b = bytes(m.dat)
            mode = 'sport' if (b[0] >> 2) & 1 else 'eco' if b[5] & 1 else 'normal'
            bgear = bool((b[5] >> 1) & 1)
      elif w == 'liveLocationKalman':
        try:
          o = e.liveLocationKalman.calibratedOrientationNED   # car frame: the device mounting pitch (-1.26 deg) removed
          if o.valid: pitch = float(o.value[1])
        except Exception:
          pass
      elif w == 'carState':
        c = e.carState; t = e.logMonoTime / 1e9
        free = not c.gasPressed and not c.brakePressed and not c.standstill and not long_on and abs(c.steeringAngleDeg) < 30
        if not free:
          free_since = None; continue
        if free_since is None: free_since = t
        if t - free_since >= SETTLE_S and c.vEgo > 3:
          out.append((c.vEgo, c.aEgo, pitch, 1. if (en or mads) else 0., {'normal': 0., 'eco': 1., 'sport': 2.}[mode], 1. if bgear else 0.))
  except Exception:
    pass
  return out[::10]   # 10 Hz is plenty


def table(v, a_raw, a_flat, has_p, sel, title):
  print(f'--- {title}: {sel.sum() / 600:.1f} min')
  print('km/h    n(min) | aEgo median p25 p75 | slope-corrected median p25 p75')
  for b in range(0, 130, 10):
    s = sel & (v >= b) & (v < b + 10)
    if s.sum() < 100: continue
    f = s & has_p
    fs = f'{np.median(a_flat[f]):+.2f} {np.percentile(a_flat[f], 25):+.2f} {np.percentile(a_flat[f], 75):+.2f}' if f.sum() > 50 else 'n/a'
    print(f'{b:3d}-{b + 10:<3d} {s.sum() / 600:6.1f} | {np.median(a_raw[s]):+.2f} {np.percentile(a_raw[s], 25):+.2f} {np.percentile(a_raw[s], 75):+.2f} | {fs}')


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
  v = x[:, 0] * 3.6; a_raw = x[:, 1]; pit = x[:, 2]; lat = x[:, 3] > 0.5; mode = x[:, 4]; bg = x[:, 5] > 0.5
  has_p = pit == pit
  a_flat = np.where(has_p, a_raw + G * np.sin(np.nan_to_num(pit)), np.nan)
  print(f'{len(segs)} segments, coasting samples {len(x)} ({len(x) / 10 / 60:.0f} min), with pitch {has_p.mean() * 100:.0f} %, '
        f'manual {(~lat).sum() / 600:.0f} min, lateral-only (MADS) {lat.sum() / 600:.0f} min')
  allm = np.ones(len(x), bool)
  table(v, a_raw, a_flat, has_p, allm, 'all')
  table(v, a_raw, a_flat, has_p, ~lat, 'manual')
  table(v, a_raw, a_flat, has_p, lat, 'lateral-only (MADS)')
  for k, nm in ((1., 'ECO'), (2., 'SPORT')):
    if (mode == k).sum() >= 300: table(v, a_raw, a_flat, has_p, mode == k, nm)
  if bg.sum() >= 300: table(v, a_raw, a_flat, has_p, bg, 'B gear')
  print(f'drive mode samples: normal {(mode == 0).sum()}, eco {(mode == 1).sum()}, sport {(mode == 2).sum()}, B gear {bg.sum()}')


if __name__ == '__main__':
  main()
