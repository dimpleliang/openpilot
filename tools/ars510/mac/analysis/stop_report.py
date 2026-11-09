#!/usr/bin/env python3
"""Per-stop report of openpilot stops (engaged, from > 15 km/h to standstill, no driver pedal), one row per stop (Mac, offline).

  python3 stop_report.py [--csv OUT] <route prefix> [...]      e.g. 0000005f--c2d08d263b

Columns
  scene     lead = a radar/vision lead within 40 m in the last 3 s before the stop, e2e = no lead (red light / stop line)
  v0        km/h where the stop started (last time above 15 km/h)
  pitch     deg, deviceMotion orientationNED pitch at the stop minus the device mounting pitch (median of
            liveLocationKalman orientationNED - calibratedOrientationNED; 11/09: about -1.26 deg on this car, which
            otherwise read as a -1.2 deg downhill everywhere)
  gap_v/gap_t/gap_a   regen -> friction hand-over gap: first stretch >= 0.2 s below 1.5 m/s where the sent ACCEL_CMD
            is < -0.3 but the real deceleration (raw IMU) is weaker than 0.15 m/s^2: speed at its start, length, weakest decel
  rel_v     speed where the pre-release started (from the 'ARS510 stop phase' log, else '')
  phases    stop sub-state sequence from the log (routes from 8126785 on)
  a_stop    m/s^2, raw IMU (sensor -z = forward, 80 ms EMA, standstill bias removed) at the wheel stop  -> the jolt
  a_stop_dm m/s^2, the same from deviceMotion (what controlsd's pose path sees, 20 Hz)
  bump      m/s^2, largest forward surge 0..1.5 s after the stop (rebound or roll-on)
  grab      m/s^2, strongest decel within 0.8 s after the bump (re-grab)
  verdict   clean | bump (> +0.12) | ROLL+GRAB (bump > +0.12 and grab < -0.15)
  cmd       ACCEL_CMD actually sent on CAN at the stop (incl. the Toyota PID)
  bp_stop / bp_max    BRAKE_MODULE.BRAKE_PRESSURE (0x226) at the stop / max over -1..+2 s (raw units)
  d_stop / d_end      lead distance at the stop / 3 s later (lead scenes)
  off       m, lead: d_stop - 4.5 (low-speed stop distance); e2e: travelled - model's planned stop distance at 10 km/h
"""
import csv
import glob
import json
import os
import sys

import numpy as np
import zstandard

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log  # noqa: E402

STOP_DISTANCE_LOW = 4.5
ACC_CONTROL = 0x343
BRAKE_MODULE = 0x226


def read(p):
  return log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read()),
                                       traversal_limit_in_words=2**61)


def load_route(route):
  segs = sorted(glob.glob(os.path.join(ROOT, 'raw', route + '--*')), key=lambda s: int(s.rsplit('--', 1)[1]))
  d = {k: [] for k in ('cs', 'en', 'imu', 'dm', 'cmd', 'bp', 'lead', 'plan', 'lm', 'mount')}
  for seg in segs:
    p = seg + '/rlog.zst'
    if not os.path.exists(p):
      continue
    try:
      evs = read(p)
      for e in evs:
        w = e.which()
        t = e.logMonoTime * 1e-9
        if w == 'carState':
          c = e.carState
          d['cs'].append((t, c.vEgo, c.vEgoRaw, c.gasPressed, c.brakePressed))
        elif w == 'selfdriveState':
          d['en'].append((t, float(e.selfdriveState.enabled)))
        elif w == 'accelerometer':
          a = e.accelerometer
          if a.which() == 'acceleration':
            d['imu'].append((t, -a.acceleration.v[2]))
        elif w == 'liveLocationKalman':
          L = e.liveLocationKalman
          if L.orientationNED.valid and L.calibratedOrientationNED.valid:
            d['mount'].append(L.orientationNED.value[1] - L.calibratedOrientationNED.value[1])
        elif w == 'deviceMotion':
          m = e.deviceMotion
          d['dm'].append((t, m.accelerationDevice.x, m.orientationNED.y if hasattr(m.orientationNED, 'y') else 0.))
        elif w == 'sendcan':
          for m in e.sendcan:
            if m.address == ACC_CONTROL and m.src == 0:
              b = m.dat
              d['cmd'].append((t, int.from_bytes(b[0:2], 'big', signed=True) * 0.001))
        elif w == 'can':
          for m in e.can:
            if m.address == BRAKE_MODULE and m.src == 0:
              b = m.dat
              d['bp'].append((t, ((b[0] & 1) << 8) | b[1]))
        elif w == 'radarState':
          l1 = e.radarState.leadOne
          d['lead'].append((t, float(l1.present), l1.dRel))
        elif w == 'modelV2':
          mv = e.modelV2
          vx = np.asarray(mv.velocity.x, float)
          px = np.asarray(mv.position.x, float)
          k = np.where(vx < 0.5)[0]
          d['plan'].append((t, float(px[k[0]]) if len(k) else np.nan))
        elif w == 'logMessage':
          s = e.logMessage
          if 'ARS510 stop phase' in s:
            try:
              msg = json.loads(s).get('msg', s)
            except Exception:
              msg = s
            d['lm'].append((t, str(msg)))
    except Exception as ex:
      print(f'  {os.path.basename(seg)}: {ex}', file=sys.stderr)
  out = {}
  for k, v in d.items():
    if k == 'lm':
      out[k] = v
    elif v:
      a = np.array(v, dtype=float)
      out[k] = a[np.argsort(a[:, 0], kind='stable')]
    else:
      out[k] = np.empty((0, 4))
  return out


def at(arr, col, tt):
  if not len(arr):
    return np.nan
  i = np.searchsorted(arr[:, 0], tt) - 1
  return arr[min(max(i, 0), len(arr) - 1), col]


def ema(t, x, tau=0.08):
  y = np.empty_like(x)
  if not len(x):
    return y
  y[0] = x[0]
  for k in range(1, len(x)):
    dt = max(t[k] - t[k - 1], 1e-3)
    y[k] = y[k - 1] + dt / (tau + dt) * (x[k] - y[k - 1])
  return y


def stops(R):
  cs, en = R['cs'], R['en']
  if not len(cs):
    return
  t, v, vraw, gas, brk = cs[:, 0], cs[:, 1], cs[:, 2], cs[:, 3], cs[:, 4]
  i = 1
  n = len(t)
  while i < n:
    if vraw[i - 1] >= 0.05 and vraw[i] < 0.05 and at(en, 1, t[i]) > 0.5 and not gas[i]:
      ts = t[i]
      j = i
      while j > 0 and ts - t[j] < 60 and at(en, 1, t[j]) > 0.5 and not gas[j] and v[j] < 4.2:
        j -= 1
      ok = v[j] >= 4.2 and not brk[j:i + 1].any()
      post = (t > ts) & (t < ts + 3.0)
      if ok and not gas[post].any() and not brk[post].any():
        yield t[j], ts, v[j]
      i += 100
    i += 1


def analyse(R, t_start, ts, v0):
  row = dict(t=round(ts, 1), v0=round(v0 * 3.6))
  # scene / lead
  ld = R['lead']
  w = (ld[:, 0] > ts - 3) & (ld[:, 0] <= ts) if len(ld) else np.array([], bool)
  lead = bool(w.any() and ((ld[w, 1] > 0.5) & (ld[w, 2] < 40)).any())
  row['scene'] = 'lead' if lead else 'e2e'
  row['d_stop'] = round(at(ld, 2, ts), 1) if lead and at(ld, 1, ts) > 0.5 else ''
  row['d_end'] = round(at(ld, 2, ts + 3), 1) if lead and at(ld, 1, ts + 3) > 0.5 else ''
  # stop offset
  if lead and row['d_stop'] != '':
    row['off'] = round(row['d_stop'] - STOP_DISTANCE_LOW, 1)
  else:
    cs = R['cs']
    k = np.where((cs[:, 0] >= t_start) & (cs[:, 0] <= ts) & (cs[:, 1] <= 2.78))[0]
    if len(k) and len(R['plan']):
      t10 = cs[k[0], 0]
      planned = at(R['plan'], 1, t10)
      seg = cs[(cs[:, 0] >= t10) & (cs[:, 0] <= ts)]
      travelled = float(np.sum(seg[1:, 1] * np.diff(seg[:, 0]))) if len(seg) > 1 else np.nan
      row['off'] = round(travelled - planned, 1) if np.isfinite(planned) else ''
    else:
      row['off'] = ''
  # pitch
  mount = float(np.median(R['mount'])) if len(R.get('mount', [])) else 0.
  row['pitch'] = round(np.degrees(at(R['dm'], 2, ts) - mount), 1) if len(R['dm']) else ''
  # IMU
  imu = R['imu']
  wb = (imu[:, 0] > ts + 2.5) & (imu[:, 0] < ts + 3.5)
  if not wb.any():
    return None
  bias = imu[wb, 1].mean()
  w = (imu[:, 0] > ts - 3.0) & (imu[:, 0] < ts + 2.0)
  tt = imu[w, 0] - ts
  ax = ema(imu[w, 0], imu[w, 1] - bias)
  row['a_stop'] = round(float(ax[max(np.searchsorted(tt, 0.0) - 1, 0)]), 2)
  dm = R['dm']
  if len(dm):
    wbd = (dm[:, 0] > ts + 2.5) & (dm[:, 0] < ts + 3.5)
    row['a_stop_dm'] = round(float(at(dm, 1, ts) - (dm[wbd, 1].mean() if wbd.any() else 0.)), 2)
  post = (tt > 0) & (tt < 1.5)
  bump = float(ax[post].max())
  tb = float(tt[post][np.argmax(ax[post])])
  after = (tt > tb) & (tt < tb + 0.8)
  grab = float(ax[after].min()) if after.any() else np.nan
  row.update(bump=round(bump, 2), t_bump=round(tb, 2), grab=round(grab, 2))
  row['verdict'] = 'ROLL+GRAB' if bump > 0.12 and grab < -0.15 else 'bump' if bump > 0.12 else 'clean'
  # regen -> friction gap (below 1.5 m/s, before the stop)
  cs = R['cs']
  cmd = R['cmd']
  wv = (tt < 0) & (tt > -3.0)
  gap = None
  run_start = None
  for k in np.where(wv)[0]:
    tk = ts + tt[k]
    vk = at(cs, 1, tk)
    weak = vk < 1.5 and at(cmd, 1, tk) < -0.3 and -ax[k] < 0.15
    if weak and run_start is None:
      run_start = k
    elif not weak and run_start is not None:
      if tt[k] - tt[run_start] >= 0.2:
        gap = (run_start, k)
        break
      run_start = None
  if gap:
    a, b = gap
    row.update(gap_v=round(at(cs, 1, ts + tt[a]), 2), gap_t=round(tt[b] - tt[a], 2), gap_a=round(float(-ax[a:b].max()), 2))
  else:
    row.update(gap_v='', gap_t='', gap_a='')
  # CAN
  row['cmd'] = round(at(cmd, 1, ts), 2) if len(cmd) else ''
  bp = R['bp']
  if len(bp):
    row['bp_stop'] = int(at(bp, 1, ts))
    wbp = (bp[:, 0] > ts - 1) & (bp[:, 0] < ts + 2)
    row['bp_max'] = int(bp[wbp, 1].max()) if wbp.any() else ''
  # phases from the log
  lm = [(t, m) for t, m in R['lm'] if t_start - 1 <= t <= ts + 3]
  seq, rel_v = [], ''
  for t, m in lm:
    try:
      tr = m.split('ARS510 stop phase ', 1)[1].split(' |', 1)[0]
      seq.append(f'{t - ts:+.1f}:{tr.split("->")[1]}')
      if tr.endswith('->RELEASE') and rel_v == '':
        rel_v = float(m.split(' v=', 1)[1].split()[0])
    except Exception:
      pass
  row['rel_v'] = rel_v
  row['phases'] = ' '.join(seq)
  return row


COLS = ['route', 't', 'scene', 'v0', 'pitch', 'gap_v', 'gap_t', 'gap_a', 'rel_v', 'a_stop', 'a_stop_dm', 'bump', 't_bump',
        'grab', 'verdict', 'cmd', 'bp_stop', 'bp_max', 'd_stop', 'd_end', 'off', 'phases']


def main():
  args = sys.argv[1:]
  out = None
  if '--csv' in args:
    k = args.index('--csv')
    out = args[k + 1]
    del args[k:k + 2]
  rows = []
  for route in args:
    R = load_route(route)
    for t0, ts, v0 in stops(R):
      r = analyse(R, t0, ts, v0)
      if r:
        r['route'] = route[-10:]
        rows.append(r)
  if not rows:
    print('no stops')
    return
  print(' '.join(f'{c:>9}' for c in COLS[:-1]) + '  phases')
  for r in rows:
    print(' '.join(f'{str(r.get(c, "")):>9}' for c in COLS[:-1]) + '  ' + str(r.get('phases', '')))
  if out:
    with open(out, 'w', newline='') as f:
      wr = csv.DictWriter(f, COLS, extrasaction='ignore')
      wr.writeheader()
      wr.writerows(rows)
  # summary
  def f(x):
    return np.array([float(v) for v in x if v != ''], float)
  print()
  for scene in ('lead', 'e2e', 'all'):
    g = [r for r in rows if scene == 'all' or r['scene'] == scene]
    if not g:
      continue
    a = f(r['a_stop'] for r in g)
    gv = [r for r in g if r['gap_v'] != '']
    print(f'{scene:5} n={len(g):3}  a_stop median {np.median(a):+.2f} P90 {np.percentile(a, 10):+.2f}  '
          f'bump {sum(r["verdict"] != "clean" for r in g)}  ROLL+GRAB {sum(r["verdict"] == "ROLL+GRAB" for r in g)}  '
          f'regen gap {len(gv)}' + (f' (v med {np.median(f(r["gap_v"] for r in gv)):.2f}, t med {np.median(f(r["gap_t"] for r in gv)):.2f})' if gv else ''))
  for lab, cond in (('gap', lambda r: r['gap_v'] != ''), ('no gap', lambda r: r['gap_v'] == '')):
    g = [r for r in rows if cond(r)]
    if g:
      print(f'{lab:7} n={len(g):3}  a_stop median {np.median(f(r["a_stop"] for r in g)):+.2f}  bump {sum(r["verdict"] != "clean" for r in g)}')
  p = f(r['pitch'] for r in rows)
  if len(p):
    for lab, lo, hi in (('pitch<-1', -90, -1), ('|pitch|<=1', -1, 1), ('pitch>1', 1, 90)):
      g = [r for r in rows if r['pitch'] != '' and lo <= float(r['pitch']) < hi + (1e-9 if hi == 1 else 0)]
      if g:
        print(f'{lab:10} n={len(g):3}  a_stop median {np.median(f(r["a_stop"] for r in g)):+.2f}  bump {sum(r["verdict"] != "clean" for r in g)}')


if __name__ == '__main__':
  main()
