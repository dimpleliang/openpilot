#!/usr/bin/env python3
"""dimpleliang/ars510: lateral actuation check for a drive, following the method of sunnypilot/opendbc#299
(log the real maximum actuation per speed and compare it with the ISO 11270 limits comma uses: 3.0 m/s^2 lateral
acceleration, 5.0 m/s^3 lateral jerk).

Only frames where openpilot itself steers are counted: lateral active, no driver torque within +-0.5 s, no lane change,
above MIN_SPEED. For those, per speed band:
  - cmd: the LTA angle openpilot sent (carOutput), through the vehicle model with the live steer ratio / offset;
  - imu: measured yaw rate x speed (locationd), i.e. what the car actually did.
Jerk is the change of lateral acceleration over a centred 0.5 s window. The LTA command rate is also checked against
the ANGLE_LIMITS table.

On the device (parked):
  PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 /data/openpilot/tools/ars510/lat_limits.py <route or rlog> [...]
  e.g. lat_limits.py 00000058--0123456789
Prints a table and writes /data/ars510_lat_limits.json.
"""
import glob, json, math, os, sys
import numpy as np
import zstandard

try:
  from openpilot.cereal import log
except ImportError:  # running off-device with a capnp schema load
  import capnp
  capnp.remove_import_hook()
  SP = os.environ.get("SP_ROOT", "/home/claude/sp")
  log = capnp.load(f"{SP}/openpilot/cereal/log.capnp", imports=[f"{SP}/openpilot/cereal", f"{SP}/opendbc_repo/opendbc/car",
                                                               f"{SP}/openpilot", SP])
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.toyota.values import CarControllerParams

ISO_LAT_ACCEL = 3.0   # m/s^2
ISO_LAT_JERK = 5.0    # m/s^3
JERK_WINDOW = 0.5     # s
MIN_SPEED = 3.0       # m/s
PRESS_GUARD = 0.5     # s around driver torque
BANDS = [0, 20, 40, 60, 80, 100, 200]  # km/h
DT = 0.01


def rlog_paths(arg):
  if os.path.isfile(arg):
    return [arg]
  segs = glob.glob(f"/data/media/0/realdata/{arg}--*")
  segs.sort(key=lambda p: int(p.rsplit("--", 1)[1]))
  return [s + "/rlog.zst" for s in segs if os.path.isfile(s + "/rlog.zst")]


def hold(ts, vals, t):
  """last value at or before t for every t (zero-order hold)"""
  idx = np.searchsorted(ts, t, side="right") - 1
  idx = np.clip(idx, 0, len(vals) - 1)
  return np.asarray(vals)[idx]


def read(paths):
  cs, cc, co, vp, dm, lc = [], [], [], [], [], []
  CP = None
  wall0 = None
  for p in paths:
    try:
      data = zstandard.ZstdDecompressor().stream_reader(open(p, "rb")).read()
      msgs = log.Event.read_multiple_bytes(data, traversal_limit_in_words=2**61)
      for m in msgs:
        w = m.which(); t = m.logMonoTime / 1e9
        if w == "carState":
          s = m.carState
          cs.append((t, s.vEgo, s.steeringAngleDeg, s.steeringPressed, s.leftBlinker or s.rightBlinker))
        elif w == "carControl":
          cc.append((t, m.carControl.latActive))
        elif w == "carOutput":
          co.append((t, m.carOutput.actuatorsOutput.steeringAngleDeg))
        elif w == "vehicleParameters":
          v = m.vehicleParameters
          vp.append((t, v.steerRatio, v.stiffnessFactor, v.angleOffsetDeg, v.roll))
        elif w == "deviceMotion":
          dm.append((t, m.deviceMotion.angularVelocityDevice.z))
        elif w == "drivingModelData":
          lc.append((t, 0 if str(m.drivingModelData.meta.laneChangeState) == "off" else 1))
        elif w == "carParams" and CP is None:
          CP = m.carParams
        elif w == "clocks" and wall0 is None:
          wall0 = (m.clocks.wallTimeNanos / 1e9, t)
    except Exception as e:  # a truncated last segment still contributes what it has
      print(f"{p}: {str(e)[:80]}", file=sys.stderr)
  return CP, wall0, [np.array(x, dtype=float) if x else None for x in (cs, cc, co, vp, dm, lc)]


def main(args):
  paths = [p for a in args for p in rlog_paths(a)]
  if not paths:
    sys.exit("no rlogs found")
  CP, wall0, (cs, cc, co, vp, dm, lc) = read(paths)
  if CP is None or cs is None or co is None:
    sys.exit("missing carParams / carState / carOutput")
  VM = VehicleModel(CP)
  t = cs[:, 0]; v = cs[:, 1]; sa = cs[:, 2]; press = cs[:, 3] > 0
  lat = hold(cc[:, 0], cc[:, 1], t) > 0 if cc is not None else np.ones_like(t, bool)
  sa_out = hold(co[:, 0], co[:, 1], t)
  if vp is not None:
    sr, sf, off, roll = (hold(vp[:, 0], vp[:, i], t) for i in (1, 2, 3, 4))
  else:
    sr = np.full_like(t, CP.steerRatio); sf = np.ones_like(t); off = np.zeros_like(t); roll = np.zeros_like(t)
  yaw = hold(dm[:, 0], dm[:, 1], t) if dm is not None else np.full_like(t, np.nan)
  lane_change = hold(lc[:, 0], lc[:, 1], t) > 0 if lc is not None else np.zeros_like(t, bool)

  # curvature from angle; the vehicle model is linear in the angle, so one evaluation per sample is cheap enough
  a_cmd = np.empty_like(t); a_act = np.empty_like(t)
  for i in range(len(t)):
    VM.update_params(max(sf[i], 0.1), max(sr[i], 0.1))
    u = max(v[i], 0.1)
    a_cmd[i] = VM.calc_curvature(math.radians(sa_out[i] - off[i]), u, roll[i]) * u * u
    a_act[i] = VM.calc_curvature(math.radians(sa[i] - off[i]), u, roll[i]) * u * u
  a_imu = yaw * v

  # centred 0.5 s jerk on the (uniform 100 Hz) carState time base
  n = int(round(JERK_WINDOW / 2 / DT))
  def jerk(a):
    j = np.full_like(a, np.nan)
    dtw = t[2 * n:] - t[:-2 * n]
    j[n:-n] = (a[2 * n:] - a[:-2 * n]) / np.where(dtw > 0, dtw, np.nan)
    return j
  j_cmd, j_imu = jerk(a_cmd), jerk(a_imu)

  # mask: openpilot steering on its own
  press_near = np.zeros_like(press)
  g = int(PRESS_GUARD / DT)
  idx = np.flatnonzero(press)
  for i in idx:
    press_near[max(0, i - g):i + g + 1] = True
  ok = lat & ~press_near & ~lane_change & (v > MIN_SPEED)
  # windows that straddle an excluded frame are not valid for jerk
  ok_j = ok.copy()
  bad = np.flatnonzero(~ok)
  for i in bad:
    ok_j[max(0, i - n):i + n + 1] = False

  kmh = v * 3.6
  rows = []
  print(f"{'km/h':>9} {'s':>6} | {'cmd a max':>9} {'p99':>5} {'cmd j max':>9} {'p99':>5} | {'imu a max':>9} {'imu j max':>9} | >3.0 >5.0")
  for lo, hi in zip(BANDS[:-1], BANDS[1:]):
    m = ok & (kmh >= lo) & (kmh < hi); mj = ok_j & (kmh >= lo) & (kmh < hi)
    if m.sum() < 100:
      continue
    r = dict(band=f"{lo}-{hi}", seconds=round(float(m.sum()) * DT, 1),
             cmd_a_max=float(np.nanmax(np.abs(a_cmd[m]))), cmd_a_p99=float(np.nanpercentile(np.abs(a_cmd[m]), 99)),
             cmd_j_max=float(np.nanmax(np.abs(j_cmd[mj]))) if mj.any() else float("nan"),
             cmd_j_p99=float(np.nanpercentile(np.abs(j_cmd[mj]), 99)) if mj.any() else float("nan"),
             imu_a_max=float(np.nanmax(np.abs(a_imu[m]))), imu_j_max=float(np.nanmax(np.abs(j_imu[mj]))) if mj.any() else float("nan"),
             over_a=round(float((np.abs(a_cmd[m]) > ISO_LAT_ACCEL).sum()) * DT, 2),
             over_j=round(float((np.abs(j_cmd[mj]) > ISO_LAT_JERK).sum()) * DT, 2))
    rows.append(r)
    print(f"{r['band']:>9} {r['seconds']:6.0f} | {r['cmd_a_max']:9.2f} {r['cmd_a_p99']:5.2f} {r['cmd_j_max']:9.2f} {r['cmd_j_p99']:5.2f} | "
          f"{r['imu_a_max']:9.2f} {r['imu_j_max']:9.2f} | {r['over_a']:4.1f} {r['over_j']:4.1f}")

  # top jerk events (openpilot steering)
  def wall(ti):
    import datetime
    w = wall0[0] + (ti - wall0[1]) if wall0 else ti
    return datetime.datetime.fromtimestamp(float(w), datetime.timezone(datetime.timedelta(hours=8))).strftime("%H:%M:%S")
  order = np.argsort(-np.nan_to_num(np.abs(np.where(ok_j, j_cmd, np.nan)), nan=-1))
  events, last = [], -1e9
  for i in order:
    if not ok_j[i] or len(events) >= 6:
      break
    if t[i] - last < 3 and events:
      continue
    events.append(dict(wall=wall(t[i]), kmh=round(float(kmh[i]), 1), cmd_a=round(float(a_cmd[i]), 2), cmd_j=round(float(j_cmd[i]), 2),
                       imu_j=round(float(j_imu[i]), 2), angle=round(float(sa[i]), 1)))
    last = t[i]
  print("top openpilot jerk:", events)

  # LTA command rate vs ANGLE_LIMITS (the LTA command changes every 2nd 100 Hz frame: compare 20 ms steps)
  lim = CarControllerParams.ANGLE_LIMITS
  t100 = co[:, 0]; a100 = co[:, 1]
  d = a100[2:] - a100[:-2]; dt2 = t100[2:] - t100[:-2]
  v2 = hold(t, v, t100[2:])
  lat2 = hold(t, ok.astype(float), t100[2:]) > 0  # openpilot steering on its own (override follows the wheel, unlimited)
  up = (a100[:-2] * a100[2:] >= 0) & (np.abs(a100[2:]) > np.abs(a100[:-2]))
  allowed = np.where(up, np.interp(v2, *lim.ANGLE_RATE_LIMIT_UP), np.interp(v2, *lim.ANGLE_RATE_LIMIT_DOWN))
  reg = (dt2 > 0.015) & (dt2 < 0.03) & lat2
  if reg.any():
    rate = np.abs(d[reg]) / dt2[reg]
    excess = np.abs(d[reg]) - allowed[reg] * 1.02 - 0.01  # one LTA step per 20 ms whatever the log timing jitter
    print(f"LTA command rate while active: max {rate.max():.0f} deg/s, steps above the table: {(excess > 0).sum()}")
    if os.environ.get("LAT_DEBUG"):
      ii = np.flatnonzero(reg)[excess > 0][:10]
      for k in ii:
        print(f"  v {v2[k]*3.6:.1f} km/h step {d[k]:+.3f} allowed {allowed[k]:.3f} up {up[k]} dt {dt2[k]:.3f} prev {a100[k]:.2f}")

  out = dict(paths=len(paths), bands=rows, top=events, iso=dict(accel=ISO_LAT_ACCEL, jerk=ISO_LAT_JERK))
  dst = "/data/ars510_lat_limits.json" if os.path.isdir("/data") and os.access("/data", os.W_OK) else "ars510_lat_limits.json"
  json.dump(out, open(dst, "w"), indent=1)
  print("wrote", dst)


if __name__ == "__main__":
  main(sys.argv[1:])
