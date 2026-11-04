"""Curve episodes of one route (Mac, ~/ars510_rlogs/raw): every stretch where openpilot steers (latActive) and the lateral
acceleration (controlsState.curvature x v^2) reaches >= THRESH m/s^2. Per episode: speed, actual / desired / model lateral
acceleration, the speed-only cap (drive_helpers.ramp_lat_accel: 3.4 at 40-70 km/h, 3.0 elsewhere), frames where the cap
cut the model, steering angle / rate, driver torque / steeringPressed, EPS torque, steer faults, lat drop-outs, GPS.

  python3 curve_scan.py 00000041--d75fc79ef7 [start HH:MM]   -> ~/ars510_rlogs/analysis/curve/<route>.txt / .csv
"""
import csv, glob, os, sys, zstandard, numpy as np
from multiprocessing import Pool

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log

THRESH = 1.5
COLS = ['t', 'v', 'curv', 'dcurv', 'model', 'lat', 'pressed', 'eps', 'drv', 'ang', 'rate', 'fault', 'roll', 'lat_deg', 'lon_deg']


def ramp_cap(v):
  lo, hi, fade = 40 / 3.6, 70 / 3.6, 5 / 3.6
  if lo <= v <= hi: return 3.4
  d = lo - v if v < lo else v - hi
  return 3.4 - 0.4 * min(d / fade, 1.)


def scan(seg):
  p = seg + '/rlog.zst'
  if not os.path.exists(p): return None
  try: d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  except Exception: return None
  out = []; st = dict(lat=0, pressed=0, v=0., eps=0., drv=0., ang=0., rate=0., fault=0, model=np.nan, roll=0., la=np.nan, lo=np.nan)
  try:
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      w = m.which()
      if w == 'carControl': st['lat'] = int(m.carControl.latActive)
      elif w == 'carState':
        c = m.carState
        st.update(v=c.vEgo, pressed=int(c.steeringPressed), eps=c.steeringTorqueEps, drv=c.steeringTorque, ang=c.steeringAngleDeg,
                  rate=c.steeringRateDeg, fault=int(c.steerFaultTemporary or c.steerFaultPermanent))
      elif w == 'modelV2': st['model'] = m.modelV2.action.desiredCurvature
      elif w == 'liveParameters': st['roll'] = m.liveParameters.roll
      elif w == 'gpsLocation' or w == 'gpsLocationExternal':
        g = getattr(m, w); st['la'], st['lo'] = g.latitude, g.longitude
      elif w == 'controlsState':
        c = m.controlsState
        out.append((m.logMonoTime / 1e9, st['v'], c.curvature, c.desiredCurvature, st['model'], st['lat'], st['pressed'], st['eps'],
                    st['drv'], st['ang'], st['rate'], st['fault'], st['roll'], st['la'], st['lo']))
  except Exception: pass
  return int(seg.rsplit('--', 1)[1]), np.array(out, dtype=float) if out else None


def main():
  route = sys.argv[1]
  h, mi = (int(x) for x in (sys.argv[2] if len(sys.argv) > 2 else '00:00').split(':'))
  segs = sorted(glob.glob(os.path.join(ROOT, 'raw', route + '--*')), key=lambda s: int(s.rsplit('--', 1)[1]))
  with Pool(6) as pool: res = [r for r in pool.map(scan, segs, chunksize=2) if r and r[1] is not None]
  res.sort()
  t0 = min(r[1][0, 0] - 60 * r[0] for r in res)        # route start, monotonic
  a = np.vstack([r[1] for r in res]); a = a[np.argsort(a[:, 0])]
  C = {k: i for i, k in enumerate(COLS)}
  v = a[:, C['v']]; act = np.abs(a[:, C['curv']]) * v ** 2; des = np.abs(a[:, C['dcurv']]) * v ** 2; mod = np.abs(a[:, C['model']]) * v ** 2
  cap = np.array([ramp_cap(x) for x in v])
  lat = a[:, C['lat']] == 1
  hot = lat & (act >= THRESH)
  # episodes: hot frames merged across gaps < 1 s
  eps_idx, i, n = [], 0, len(a)
  idx = np.flatnonzero(hot)
  if len(idx):
    s = idx[0]; prev = idx[0]
    for j in idx[1:]:
      if a[j, 0] - a[prev, 0] > 1.0: eps_idx.append((s, prev)); s = j
      prev = j
    eps_idx.append((s, prev))
  rows = []
  for s, e in eps_idx:
    s2 = max(0, s - 100); e2 = min(n - 1, e + 200)     # 1 s before, 2 s after
    w = slice(s2, e2 + 1); k = s + int(np.argmax(act[s:e + 1]))
    tt = a[k, 0] - t0; clock = (h * 60 + mi) * 60 + tt
    drop = bool((~lat[s:e2 + 1]).any())
    rows.append(dict(
      clock=f'{int(clock // 3600) % 24:02d}:{int(clock // 60) % 60:02d}:{int(clock % 60):02d}', seg=int(tt // 60), dur=round(a[e, 0] - a[s, 0], 1),
      kmh=round(v[k] * 3.6), kmh_min=round(v[s:e + 1].min() * 3.6), kmh_max=round(v[s:e + 1].max() * 3.6),
      act=round(act[k], 2), act_max=round(act[s:e + 1].max(), 2), des_max=round(np.nanmax(des[s:e + 1]), 2), model_max=round(np.nanmax(mod[s:e + 1]), 2),
      cap=round(cap[k], 2), capped_frames=int(((mod[s:e + 1] - des[s:e + 1]) > 0.05).sum()),
      ang_max=round(np.abs(a[s:e + 1, C['ang']]).max()), rate_max=round(np.abs(a[w, C['rate']]).max()),
      pressed=bool(a[w, C['pressed']].any()), drv_max=round(np.abs(a[w, C['drv']]).max()), eps_max=round(np.abs(a[w, C['eps']]).max()),
      fault=bool(a[w, C['fault']].any()), lat_drop=drop, roll_deg=round(np.degrees(a[k, C['roll']]), 1),
      gps=f'{a[k, C["lat_deg"]]:.5f},{a[k, C["lon_deg"]]:.5f}'))
  od = os.path.join(ROOT, 'analysis', 'curve'); os.makedirs(od, exist_ok=True)
  with open(os.path.join(od, route + '.csv'), 'w', newline='') as f:
    if rows:
      wr = csv.DictWriter(f, fieldnames=list(rows[0])); wr.writeheader(); wr.writerows(rows)
  steer_h = lat.sum() / 100 / 3600
  L = [f'{route}: {len(res)} segments, openpilot steering {steer_h:.2f} h, curve episodes (>= {THRESH} m/s^2): {len(rows)}']
  for lo, hi in ((0, 40), (40, 70), (70, 80), (80, 200)):
    s = lat & (v * 3.6 >= lo) & (v * 3.6 < hi)
    if s.any():
      L.append(f'  {lo:>3}-{hi:<3} km/h: {s.sum()/100/60:6.1f} min, actual p99 {np.percentile(act[s], 99):.2f} max {act[s].max():.2f}, '
               f'desired max {np.nanmax(des[s]):.2f}, model max {np.nanmax(mod[s]):.2f}, frames >2.5 {(act[s] > 2.5).sum()}, >3.0 {(act[s] > 3.0).sum()}, '
               f'cap cut model {((mod[s] - des[s]) > 0.05).sum()} frames')
  L.append(f'  episodes with driver press {sum(r["pressed"] for r in rows)}, lat drop-out {sum(r["lat_drop"] for r in rows)}, steer fault {sum(r["fault"] for r in rows)}')
  L.append('\nclock seg dur | km/h (min-max) | actual max desired model cap capped | angle rate | press drv EPS fault drop | roll gps')
  for r in sorted(rows, key=lambda r: -r['act_max'])[:40]:
    L.append(f'{r["clock"]} {r["seg"]:>3} {r["dur"]:>4} | {r["kmh"]:>3} ({r["kmh_min"]}-{r["kmh_max"]}) | {r["act_max"]:.2f} {r["des_max"]:.2f} {r["model_max"]:.2f} {r["cap"]:.1f} {r["capped_frames"]:>3} | '
             f'{r["ang_max"]:>4} {r["rate_max"]:>4} | {"P" if r["pressed"] else "-"} {r["drv_max"]:>4} {r["eps_max"]:>4} {"F" if r["fault"] else "-"} {"D" if r["lat_drop"] else "-"} | {r["roll_deg"]} {r["gps"]}')
  txt = '\n'.join(L); print(txt)
  open(os.path.join(od, route + '.txt'), 'w').write(txt)


if __name__ == '__main__':
  main()
