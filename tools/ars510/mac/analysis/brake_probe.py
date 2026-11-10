#!/usr/bin/env python3
"""ars510: decode the brake probe (opendbc/sunnypilot/car/toyota/brake_probe.py) from rlogs (Mac, offline).

The probe reads data identifiers from the brake ECU (answers on 0x7B8) and the hybrid / cruise ECU (0x7DA) at 10 Hz
while /data/ars510_brake_probe exists on the device. This prints, for every stop (or one moment with --at), what the
car's own computers did next to what openpilot asked:

  python3 brake_probe.py [--at HH:MM:SS] [--win 6] [--csv OUT] <route prefix> [...]

  cmd      m/s^2  openpilot's request (carControl.actuators.accel), 0 when the driver brakes himself
  ds_tgt   m/s^2  0x15ED driving support ECU target acceleration: what the hybrid ECU received
  cr_tgt   m/s^2  0x15ED cruise target acceleration: what it works to after its own processing
  grad     m/s^2  0x15EC gradient of road surface (its slope estimate, as an acceleration)
  f_req / f_out  kN  0x15ED cruise request driving force / 0x15EC output driving force
  mc_tq    Nm     0x1067 master cylinder control torque (brake torque the hybrid ECU asks of the brake system)
  mg_tq    Nm     0x10A1 target motor torque
  p_tgt    MPa    0x1022 target oil pressure, mean of the four wheels (FR FL RR RL in the csv)
  p_mc     MPa    0x1004 master cylinder pressure
  stroke   mm     0x104C brake pedal stroke (the driver's foot)
  g        m/s^2  0x1008 forward / rearward G of the brake ECU
  regen    Nm     0x1055 front regenerative operation
  op / bits       0x15EC operation condition byte, 0x15FE status bits (hex)
Scaling from 4piu/toyota-telemetry-XA50 signals.csv; identifiers verified on the car on 11/10.
"""
import argparse
import glob
import os
import sys
import time

import numpy as np
import zstandard

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log  # noqa: E402

s16 = lambda b: (b[0] << 8) | b[1]
i8 = lambda x: x - 256 if x > 127 else x

# (rx address, identifier) -> {column: value}
DECODE = {
  (0x7B8, 0x1022): lambda d: dict(p_fr=d[0] * 4 / 51, p_fl=d[1] * 4 / 51, p_rr=d[2] * 4 / 51, p_rl=d[3] * 4 / 51,
                                  p_tgt=sum(d[:4]) / 51),
  (0x7B8, 0x1004): lambda d: dict(p_mc=d[0] * 0.098 - 1.),
  (0x7B8, 0x104C): lambda d: dict(stroke=float(d[0])),
  (0x7B8, 0x1008): lambda d: dict(g=i8(d[1]) * 0.196133),
  (0x7B8, 0x1055): lambda d: dict(regen=s16(d) * 16.),
  (0x7DA, 0x15ED): lambda d: dict(ds_tgt=s16(d[4:6]) * 0.0004 - 13.1, cr_tgt=s16(d[6:8]) * 0.0004 - 13.1,
                                  f_req=d[8] * 0.078125 - 10., fb=float(d[9])),
  (0x7DA, 0x15EC): lambda d: dict(a_veh=s16(d[0:2]) * 0.0004 - 13.1, grad=s16(d[2:4]) * 0.001 - 32.76,
                                  f_out=d[4] * 0.078125 - 10., op=float(d[5])),
  (0x7DA, 0x10A1): lambda d: dict(mg_tq=s16(d[2:4]) * 0.125 - 4096.),
  (0x7DA, 0x1067): lambda d: dict(mc_tq=s16(d) * 0.125 - 4096.),
  (0x7DA, 0x15FE): lambda d: dict(bits=float(s16(d))),
}
COLS = ['cmd', 'ds_tgt', 'cr_tgt', 'grad', 'f_req', 'f_out', 'mc_tq', 'mg_tq', 'p_tgt', 'p_mc', 'stroke', 'g', 'regen', 'op', 'bits']


class IsoTp:
  """Reassembles the answers on one rx address (single frames and first + consecutive frames)."""
  def __init__(self):
    self.buf = None; self.need = 0

  def feed(self, b):
    t = b[0] >> 4
    if t == 0:
      return bytes(b[1:1 + (b[0] & 0xF)])
    if t == 1:
      self.need = ((b[0] & 0xF) << 8) | b[1]; self.buf = bytearray(b[2:8])
    elif t == 2 and self.buf is not None:
      self.buf += b[1:8]
      if len(self.buf) >= self.need:
        out = bytes(self.buf[:self.need]); self.buf = None
        return out
    return None


def load(route):
  S = {}; cs = []; cc = []; gps = []; tp = {0x7B8: IsoTp(), 0x7DA: IsoTp()}; n_req = 0
  for seg in sorted(glob.glob(os.path.join(ROOT, 'raw', route + '--*')), key=lambda s: int(s.rsplit('--', 1)[1])):
    p = seg + '/rlog.zst'
    if not os.path.exists(p):
      continue
    try:
      dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
      for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
        w = e.which(); t = e.logMonoTime * 1e-9
        if w == 'can':
          for c in e.can:
            if c.src == 0 and c.address in tp:
              m = tp[c.address].feed(bytes(c.dat))
              if m and len(m) >= 3 and m[0] == 0x62:
                f = DECODE.get((c.address, (m[1] << 8) | m[2]))
                if f:
                  try:
                    for k, v in f(m[3:]).items():
                      S.setdefault(k, []).append((t, v))
                  except IndexError:
                    pass
        elif w == 'sendcan':
          n_req += sum(1 for c in e.sendcan if c.address in (0x7B0, 0x7D2) and c.dat[0] == 0x03)
        elif w == 'carState':
          s = e.carState; cs.append((t, s.vEgoRaw, float(s.brakePressed), float(s.gasPressed)))
        elif w == 'carControl':
          cc.append((t, e.carControl.actuators.accel if e.carControl.longActive else 0., float(e.carControl.longActive)))
        elif w in ('gpsLocationExternal', 'gpsLocation'):
          g = getattr(e, w)
          if g.unixTimestampMillis > 0 and g.hasFix:
            gps.append(g.unixTimestampMillis * 1e-3 - t)
    except Exception as ex:
      print(f'  {os.path.basename(seg)}: {ex}', file=sys.stderr)
  S = {k: np.array(v) for k, v in S.items()}
  return S, np.array(cs), np.array(cc), (float(np.median(gps)) if gps else None), n_req


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--at'); ap.add_argument('--win', type=float, default=6.); ap.add_argument('--csv')
  ap.add_argument('--tz', type=float, default=8.); ap.add_argument('routes', nargs='+')
  a = ap.parse_args()
  for route in a.routes:
    S, cs, cc, off, n_req = load(route)
    if not len(cs):
      continue
    span = cs[-1, 0] - cs[0, 0]
    print(f"{route}: {span / 60:.0f} min, {n_req} requests sent; answers per identifier group: "
          + ', '.join(f"{k} {len(v) / max(span, 1):.1f}/s" for k, v in S.items() if k in ('p_tgt', 'p_mc', 'stroke', 'g', 'regen', 'cr_tgt', 'grad', 'mg_tq', 'mc_tq', 'bits')))
    if not S:
      print('  no probe answers in this route (flag file missing, or the firmware blocks the requests)')
      continue
    clock = lambda t: time.strftime('%H:%M:%S', time.gmtime(t + off + a.tz * 3600)) if off is not None else f'{t:.1f}'
    S['cmd'] = cc[:, :2]

    def at(k, t):
      v = S.get(k)
      if v is None or not len(v):
        return float('nan')
      i = min(np.searchsorted(v[:, 0], t), len(v) - 1)
      i = i - 1 if i > 0 and (i == len(v) or abs(v[i - 1, 0] - t) < abs(v[i, 0] - t)) else i
      return float(v[i, 1]) if abs(v[i, 0] - t) < 0.5 else float('nan')

    if a.at:
      hh, mm, ss = map(int, a.at.split(':'))
      day = time.gmtime(cs[0, 0] + off + a.tz * 3600)
      stops = [(time.mktime((day.tm_year, day.tm_mon, day.tm_mday, hh, mm, ss, 0, 0, 0)) - time.timezone) - a.tz * 3600 - off]
    else:
      v = cs[:, 1]; stops = []; k = 1
      while k < len(v):
        if v[k] < 0.05 <= v[k - 1] and v[max(k - 800, 0):k].max() > 0.5:
          stops.append(cs[k, 0]); k += 300
        k += 1
    rows = []
    for ts in stops:
      i = min(np.searchsorted(cc[:, 0], ts - 1.), len(cc) - 1)
      who = 'openpilot' if cc[i, 2] > 0.5 else 'driver'
      print(f"\nstop {clock(ts)} ({who})")
      print('   t     v  ' + ' '.join(f'{c:>6s}' for c in COLS))
      for t in np.arange(ts - a.win, ts + 1.01, 0.2):
        vals = [at(c, t) for c in COLS]
        vv = float(np.interp(t, cs[:, 0], cs[:, 1])) * 3.6
        fmt = lambda c, x: '     -' if x != x else (f'{int(x):6x}' if c in ('op', 'bits') else f'{x:6.0f}' if c in ('mc_tq', 'mg_tq', 'regen', 'stroke') else f'{x:+6.2f}')
        print(f"{t - ts:+5.1f} {vv:5.1f} " + ' '.join(fmt(c, x) for c, x in zip(COLS, vals)))
        rows.append([ts, t - ts, vv] + vals + [at(c, t) for c in ('p_fr', 'p_fl', 'p_rr', 'p_rl', 'a_veh', 'fb')])
    if a.csv and rows:
      np.savetxt(a.csv, np.array(rows), delimiter=',', fmt='%.4f', comments='',
                 header='stop_t,t,v_kmh,' + ','.join(COLS) + ',p_fr,p_fl,p_rr,p_rl,a_veh,fb')


if __name__ == '__main__':
  main()
