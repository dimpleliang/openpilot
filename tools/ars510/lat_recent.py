#!/usr/bin/env python3
"""ars510: how close did openpilot's steering come to the panda / control limits on recent routes.
Per route: LTA command step vs ANGLE_LIMITS (ratio to the allowed step), EPS torque vs the 2500 wind-down line,
lateral accel / jerk vs ISO 3.0 / 5.0, steer faults. Only frames where openpilot steers on its own (latActive, no
driver torque within 0.5 s, no lane change, v > 3 m/s).
Run on the device: PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 /data/lat_recent.py ROUTE [ROUTE...]
"""
import glob, json, math, sys
import numpy as np
import zstandard
sys.path.insert(0, "/data/openpilot")
from openpilot.cereal import log
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.toyota.values import CarControllerParams

BANDS = [0, 20, 40, 60, 80, 100, 200]
DT = 0.01


def hold(ts, vals, t):
  idx = np.clip(np.searchsorted(ts, t, side="right") - 1, 0, len(vals) - 1)
  return np.asarray(vals)[idx]


def analyse(route):
  segs = sorted(glob.glob(f"/data/media/0/realdata/{route}--*"), key=lambda p: int(p.rsplit("--", 1)[1]))
  cs, cc, co, dm, lc, vp = [], [], [], [], [], []
  CP = None; wall0 = None; events = []
  for sd in segs:
    try:
      data = zstandard.ZstdDecompressor().stream_reader(open(sd + "/rlog.zst", "rb")).read()
      for m in log.Event.read_multiple_bytes(data, traversal_limit_in_words=2**61):
        w = m.which(); t = m.logMonoTime / 1e9
        if w == "carState":
          s = m.carState
          cs.append((t, s.vEgo, s.steeringAngleDeg, s.steeringPressed, s.steeringTorqueEps, s.steerFaultTemporary, s.leftBlinker or s.rightBlinker))
        elif w == "carControl":
          cc.append((t, m.carControl.latActive))
        elif w == "carOutput":
          co.append((t, m.carOutput.actuatorsOutput.steeringAngleDeg))
        elif w == "deviceMotion":
          dm.append((t, m.deviceMotion.angularVelocityDevice.z))
        elif w == "drivingModelData":
          lc.append((t, 0 if str(m.drivingModelData.meta.laneChangeState) == "off" else 1))
        elif w == "vehicleParameters":
          v = m.vehicleParameters; vp.append((t, v.steerRatio, v.stiffnessFactor, v.angleOffsetDeg, v.roll))
        elif w == "carParams" and CP is None:
          CP = m.carParams
        elif w == "clocks" and wall0 is None:
          wall0 = (m.clocks.wallTimeNanos / 1e9, t)
        elif w == "onroadEvents":
          for e in m.onroadEvents:
            nm = str(e.name)
            if nm in ("steerSaturated", "steerTempUnavailable", "steerTempUnavailableSilent", "ldw", "fcw", "lowSpeedLockout"):
              if not events or events[-1][1] != nm or t - events[-1][0] > 2.0:
                events.append((t, nm))
    except Exception:
      pass
  if CP is None or len(cs) < 1000 or not co:
    return None
  cs = np.array(cs, float); cc = np.array(cc, float); co = np.array(co, float)
  t = cs[:, 0]; v = cs[:, 1]; sa = cs[:, 2]; press = cs[:, 3] > 0; eps = cs[:, 4]; fault = cs[:, 5] > 0
  lat = hold(cc[:, 0], cc[:, 1], t) > 0
  lane_change = hold(np.array(lc)[:, 0], np.array(lc)[:, 1], t) > 0 if lc else np.zeros_like(t, bool)
  press_near = np.zeros_like(press); g = 50
  for i in np.flatnonzero(press):
    press_near[max(0, i - g):i + g + 1] = True
  ok = lat & ~press_near & ~lane_change & (v > 3.0)
  kmh = v * 3.6

  # lateral accel from the command (vehicle model, live steer ratio) and from the IMU
  VM = VehicleModel(CP)
  sa_out = hold(co[:, 0], co[:, 1], t)
  if vp:
    vp = np.array(vp, float); sr, sf, off, roll = (hold(vp[:, 0], vp[:, i], t) for i in (1, 2, 3, 4))
  else:
    sr = np.full_like(t, CP.steerRatio); sf = np.ones_like(t); off = np.zeros_like(t); roll = np.zeros_like(t)
  a_cmd = np.empty_like(t)
  for i in range(len(t)):
    VM.update_params(max(sf[i], 0.1), max(sr[i], 0.1)); u = max(v[i], 0.1)
    a_cmd[i] = VM.calc_curvature(math.radians(sa_out[i] - off[i]), u, roll[i]) * u * u
  a_imu = hold(np.array(dm)[:, 0], np.array(dm)[:, 1], t) * v if dm else np.full_like(t, np.nan)
  n = 25
  j_cmd = np.full_like(t, np.nan); dtw = t[2 * n:] - t[:-2 * n]
  j_cmd[n:-n] = (a_cmd[2 * n:] - a_cmd[:-2 * n]) / np.where(dtw > 0, dtw, np.nan)
  ok_j = ok.copy()
  for i in np.flatnonzero(~ok):
    ok_j[max(0, i - n):i + n + 1] = False

  # LTA command steps vs the table (every 2nd 100 Hz frame)
  lim = CarControllerParams.ANGLE_LIMITS
  t100 = co[:, 0]; a100 = co[:, 1]
  d = a100[2:] - a100[:-2]; dt2 = t100[2:] - t100[:-2]
  v2 = hold(t, v, t100[2:]); ok2 = hold(t, ok.astype(float), t100[2:]) > 0
  up = (a100[:-2] * a100[2:] >= 0) & (np.abs(a100[2:]) > np.abs(a100[:-2]))
  allowed = np.where(up, np.interp(v2, *lim.ANGLE_RATE_LIMIT_UP), np.interp(v2, *lim.ANGLE_RATE_LIMIT_DOWN))
  reg = (dt2 > 0.015) & (dt2 < 0.03) & ok2 & (np.abs(d) > 0.005)
  ratio = np.abs(d) / allowed
  kmh2 = v2 * 3.6

  out = dict(route=route, segs=len(segs), active_s=round(float(ok.sum()) * DT, 0), bands=[])
  for lo, hi in zip(BANDS[:-1], BANDS[1:]):
    m = ok & (kmh >= lo) & (kmh < hi); mj = ok_j & (kmh >= lo) & (kmh < hi)
    m2 = reg & (kmh2 >= lo) & (kmh2 < hi)
    if m.sum() < 100:
      continue
    r = dict(band=f"{lo}-{hi}", s=round(float(m.sum()) * DT, 0),
             rate_ratio_max=round(float(ratio[m2].max()), 2) if m2.any() else None,
             rate_ratio_p99=round(float(np.percentile(ratio[m2], 99)), 2) if m2.any() else None,
             rate_sat_s=round(float((ratio[m2] > 0.95).sum()) * 0.02, 2) if m2.any() else 0,  # time at the rate limit
             eps_max=int(np.abs(eps[m]).max()), eps_p99=int(np.percentile(np.abs(eps[m]), 99)),
             eps_over2300_s=round(float((np.abs(eps[m]) > 2300).sum()) * DT, 2),
             eps_over2500_s=round(float((np.abs(eps[m]) > 2500).sum()) * DT, 2),
             a_cmd_max=round(float(np.abs(a_cmd[m]).max()), 2), a_imu_max=round(float(np.nanmax(np.abs(a_imu[m]))), 2),
             j_cmd_max=round(float(np.nanmax(np.abs(j_cmd[mj]))), 2) if mj.any() else None,
             fault_s=round(float((fault & m).sum()) * DT, 2))
    out["bands"].append(r)
  out["fault_total_s"] = round(float(fault.sum()) * DT, 2)
  import datetime
  tz = datetime.timezone(datetime.timedelta(hours=8))
  ev = []
  for te, nm in events:
    i0 = np.searchsorted(t, te - 1.5); i1 = np.searchsorted(t, te + 1.5)
    k0 = np.searchsorted(t100, te - 1.5); k1 = np.searchsorted(t100, te + 1.5)
    if i1 <= i0:
      continue
    wall = datetime.datetime.fromtimestamp(wall0[0] + te - wall0[1], tz).strftime("%H:%M:%S") if wall0 else round(te, 1)
    sl = slice(i0, i1)
    ev.append(dict(wall=wall, name=nm, kmh=round(float(kmh[sl].mean()), 1), angle_max=round(float(np.abs(sa[sl]).max()), 1),
                   cmd_max=round(float(np.abs(sa_out[sl]).max()), 1), lag_max=round(float(np.abs(sa_out[sl] - sa[sl]).max()), 1),
                   a_cmd_max=round(float(np.abs(a_cmd[sl]).max()), 2), a_imu_max=round(float(np.nanmax(np.abs(a_imu[sl]))), 2),
                   eps_max=int(np.abs(eps[sl]).max()), pressed=bool(press[sl].any()),
                   rate_ratio_max=round(float(np.nanmax(ratio[max(k0 - 2, 0):max(k1 - 2, 1)])), 2) if k1 > k0 + 2 else None))
  out["events"] = ev
  return out


if __name__ == "__main__":
  for r in sys.argv[1:]:
    res = analyse(r)
    print(json.dumps(res), flush=True)
