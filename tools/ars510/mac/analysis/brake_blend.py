#!/usr/bin/env python3
"""ars510: regen vs friction blending at stops (hybrid). Reads rlogs, decodes from raw CAN (bus 0):
  0x0A6 BRAKE.BRAKE_FORCE           byte2 * 40 N   friction brake force (driver or ACC)
  0x127 GEAR_PACKET_HYBRID.FDRVREAL 11-bit signed (byte3 bits2..0, byte4) * 25 N   wheel drive force incl. regen
and carControl accel command, carState vEgo/aEgo. Forces become accelerations with MASS_KG.

  python3 brake_blend.py [--at HH:MM:SS] [--win 12] [--csv out.csv] rlog.zst [...]
  without --at: every stop (v from > 3 m/s to standstill) is summarised: friction onset speed / command / regen.
"""
import argparse
import os
import sys
import time

import numpy as np

MASS_KG = 1800.  # RAV4 hybrid AWD curb ~1700 kg + driver


def load(files):
  import capnp
  import zstandard
  capnp.remove_import_hook()
  root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..')
  d = os.path.join(root, 'openpilot', 'cereal')
  log = capnp.load(os.path.join(d, 'log.capnp'), imports=[d, os.path.join(root, 'opendbc_repo', 'opendbc', 'car')])
  bf, fd, cs, cc, gps = [], [], [], [], []
  for f in files:
    try:
      dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(f, 'rb').read())
      evs = log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61)
      for e in evs:
        w = e.which(); t = e.logMonoTime * 1e-9
        if w == 'can':
          for c in e.can:
            if c.src != 0:
              continue
            if c.address == 0xA6:
              b = bytes(c.dat); bf.append((t, b[2] * 40.))
            elif c.address == 0x127:
              b = bytes(c.dat); r = ((b[3] & 7) << 8) | b[4]
              fd.append((t, (r - 2048 if r & 0x400 else r) * 25.))
        elif w == 'carState':
          s = e.carState; cs.append((t, s.vEgo, s.aEgo, float(s.brakePressed), float(s.gasPressed)))
        elif w == 'carControl':
          c = e.carControl; cc.append((t, c.actuators.accel, float(c.longActive)))
        elif w in ('gpsLocationExternal', 'gpsLocation'):
          g = getattr(e, w)
          if g.unixTimestampMillis > 0 and g.hasFix:
            gps.append((t, g.unixTimestampMillis * 1e-3))
    except Exception as ex:
      print('read error', f, repr(ex), file=sys.stderr)
  a = lambda x, n: np.array(x) if x else np.zeros((0, n))
  return a(bf, 2), a(fd, 2), a(cs, 5), a(cc, 3), a(gps, 2)


def series(bf, fd, cs, cc, t):
  i = lambda arr, col: np.interp(t, arr[:, 0], arr[:, col])
  return dict(v=i(cs, 1), aEgo=i(cs, 2), brk=i(cs, 3), cmd=i(cc, 1), long=i(cc, 2),
              a_fric=-i(bf, 1) / MASS_KG, a_drv=i(fd, 1) / MASS_KG)


def stops(cs):
  v = cs[:, 1]; t = cs[:, 0]; out = []; armed = False
  for k in range(len(v)):
    if v[k] > 3.:
      armed = True
    elif armed and v[k] < 0.05:
      out.append(t[k]); armed = False
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--at'); ap.add_argument('--win', type=float, default=12.); ap.add_argument('--csv')
  ap.add_argument('--tz', type=float, default=8.); ap.add_argument('files', nargs='+')
  a = ap.parse_args()
  bf, fd, cs, cc, gps = load(a.files)
  if not len(bf) or not len(cs) or not len(cc):
    sys.exit('missing BRAKE / carState / carControl')
  off = float(np.median(gps[:, 1] - gps[:, 0])) if len(gps) else None
  clock = lambda t: time.strftime('%H:%M:%S', time.gmtime(t + off + a.tz * 3600)) if off is not None else f'{t:.1f}'
  print(f'BRAKE {len(bf)} frames, FDRVREAL {len(fd)}, span {clock(cs[0, 0])} - {clock(cs[-1, 0])}')
  if a.at:
    if off is None:
      sys.exit('no GPS time')
    hh, mm, ss = map(int, a.at.split(':'))
    day = time.gmtime(cs[0, 0] + off + a.tz * 3600)
    t_at = (time.mktime((day.tm_year, day.tm_mon, day.tm_mday, hh, mm, ss, 0, 0, 0)) - time.timezone) - a.tz * 3600 - off
    t = np.arange(t_at - a.win, t_at + 4., 0.1)
    s = series(bf, fd, cs, cc, t)
    print(f"{'time':>10s} {'v km/h':>7s} {'cmd':>6s} {'aEgo':>6s} {'fric':>6s} {'drive':>6s} long brk")
    for k in range(0, len(t), 2):
      print(f"{clock(t[k])}.{int((t[k] % 1) * 10)} {s['v'][k] * 3.6:7.1f} {s['cmd'][k]:+6.2f} {s['aEgo'][k]:+6.2f} "
            f"{s['a_fric'][k]:+6.2f} {s['a_drv'][k]:+6.2f} {int(s['long'][k])}    {int(s['brk'][k])}")
    if a.csv:
      np.savetxt(a.csv, np.column_stack([t - t_at] + [s[k] for k in ('v', 'cmd', 'aEgo', 'a_fric', 'a_drv', 'long', 'brk')]),
                 delimiter=',', header='t,v,cmd,aEgo,a_fric,a_drv,long,brk', comments='', fmt='%.3f')
    return
  print(f"{'stop':>8s} {'ACC':>3s} {'fric on @ km/h':>14s} {'cmd there':>9s} {'regen there':>11s} {'cmd step':>8s} "
        f"{'fric last 1s':>12s} {'regen last 1s':>13s}")
  for ts in stops(cs):
    t = np.arange(ts - 15., ts + 0.05, 0.05)
    s = series(bf, fd, cs, cc, t)
    acc = s['long'][-40:].mean() > 0.5 and s['brk'][-200:].max() < 0.5
    on = np.where(s['a_fric'] < -0.05)[0]
    k = on[0] if len(on) else None
    # friction onset of the final braking phase: last index where friction was ~0 before standstill
    if len(on):
      off_idx = np.where(s['a_fric'][:len(t) - 1] > -0.02)[0]
      k = off_idx[-1] + 1 if len(off_idx) and off_idx[-1] + 1 < len(t) else on[0]
    last = t > ts - 1.
    dcmd = np.max(np.abs(np.diff(s['cmd'][t > ts - 3.]))) / 0.05 if (t > ts - 3.).sum() > 2 else 0.
    print(f"{clock(ts)} {'Y' if acc else 'n':>3s} " + (f"{s['v'][k] * 3.6:14.1f} {s['cmd'][k]:+9.2f} {s['a_drv'][k]:+11.2f}" if k is not None
          else f"{'-':>14s} {'-':>9s} {'-':>11s}") + f" {dcmd:8.2f} {s['a_fric'][last].mean():+12.2f} {s['a_drv'][last].mean():+13.2f}")


if __name__ == '__main__':
  main()
