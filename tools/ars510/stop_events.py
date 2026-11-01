#!/usr/bin/env python3
"""ars510 on-device: every openpilot stop in the given routes, with the jolt at the moment the car really stops.

  PYTHONPATH=/data/openpilot python3 stop_events.py route_prefix [...]   (TRACE=1 prints a 20 Hz trace per stop)

A stop = long control in the stopping state while the wheel speed falls to 0 and stays there >= 1 s. Per stop:
  v_in     wheel speed when the stopping state began
  t_zero   time from stopping-state entry to the wheel speed reading 0
  a_zero   pose longitudinal acceleration (liveLocationKalman accelerationCalibrated x) at that moment
  a_min    most negative pose acceleration in the 1.5 s before the true stop (the final brake grab)
  rebound  most positive pose acceleration in the 1 s after a_min (the pitch-back)
  cmd_zero / cmd_min   brake request (carOutput accel) at t_zero and at a_min
  pre_rel  whether the request rose by >= 0.1 in the 0.6 s before the wheels stopped (pre-release ran)
  slope    road pitch at the stop in degrees (+ = uphill), liveLocationKalman pitch + mount pitch
  bl_off   time (s, relative to the wheels reading 0) when the PCM brake lights (ESP_CONTROL.BRAKE_LIGHTS_ACC) went off,
           bl_on when they came on again after the stop (the second grab); "-" = did not happen within +-1.5 s
  eng      engine running (ENGINE_RPM.ENGINE_RUNNING) at the stop
The TRACE adds columns bl (brake lights) and eng per 50 ms.
"""
import glob
import os
import sys

import numpy as np
import zstandard
from openpilot.cereal import log

REALDATA = os.environ.get("REALDATA", "/data/media/0/realdata")
TRACE = os.environ.get("TRACE") == "1"
STOPPING = "stopping"


def read(p):
  raw = open(p, "rb").read()
  try:
    return zstandard.ZstdDecompressor().stream_reader(raw).read()
  except Exception:
    return zstandard.ZstdDecompressor().decompressobj().decompress(raw)


def interp(ts, ys, t):
  return float(np.interp(t, ts, ys)) if len(ts) else float("nan")


def main():
  for r in sys.argv[1:]:
    segs = sorted(glob.glob(f"{REALDATA}/{r}--*"), key=lambda s: int(s.rsplit("--", 1)[1]))
    cs = []; co = []; pose = []; plan = []; pitch = []; bl = []; eng = []; calib = 0.0
    for seg in segs:
      p = os.path.join(seg, "rlog.zst")
      if not os.path.exists(p):
        continue
      try:
        d = read(p)
      except Exception:
        continue
      try:
        for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
          w = m.which(); t = m.logMonoTime / 1e9
          if w == "carState":
            c = m.carState
            cs.append((t, c.vEgoRaw, c.aEgo, c.standstill, c.cruiseState.enabled, c.brakePressed, c.gasPressed))
          elif w == "carOutput":
            a = m.carOutput.actuatorsOutput
            co.append((t, a.accel, str(a.longControlState)))
          elif w == "liveLocationKalman":
            ac = m.liveLocationKalman.accelerationCalibrated
            if ac.valid and len(ac.value) == 3:
              pose.append((t, ac.value[0]))
            if m.liveLocationKalman.orientationNED.valid:
              pitch.append((t, m.liveLocationKalman.orientationNED.value[1]))
          elif w == "longitudinalPlan":
            plan.append((t, m.longitudinalPlan.aTarget, m.longitudinalPlan.shouldStop))
          elif w == "extrinsicsCalibration" and len(m.extrinsicsCalibration.rpyCalib) == 3:
            calib = float(m.extrinsicsCalibration.rpyCalib[1])
          elif w == "can":
            for c in m.can:
              if c.src != 0:
                continue
              if c.address == 0x3B7 and len(c.dat) >= 3:
                bl.append((t, (c.dat[2] >> 2) & 1))
              elif c.address == 0x1C4 and len(c.dat) >= 4:
                eng.append((t, (c.dat[3] >> 3) & 1))
      except Exception:
        pass
    if not cs or not co:
      print(f"== {r}: no data"); continue
    cs_t = np.array([x[0] for x in cs]); v = np.array([x[1] for x in cs]); a_ego = np.array([x[2] for x in cs])
    brake = np.array([x[5] for x in cs]); gas = np.array([x[6] for x in cs])
    co_t = np.array([x[0] for x in co]); cmd = np.array([x[1] for x in co]); st = [x[2] for x in co]
    po_t = np.array([x[0] for x in pose]); po_a = np.array([x[1] for x in pose])
    pl_t = np.array([x[0] for x in plan]); pl_a = np.array([x[1] for x in plan])
    pi_t = np.array([x[0] for x in pitch]); pi_v = np.degrees(np.array([x[1] for x in pitch]) + calib)
    bl_t = np.array([x[0] for x in bl]); bl_v = np.array([x[1] for x in bl], dtype=float)
    en_t = np.array([x[0] for x in eng]); en_v = np.array([x[1] for x in eng], dtype=float)

    def edge(ts, vs, t_from, t_to, to_value):
      sel = (ts >= t_from) & (ts <= t_to)
      idx = np.where(sel)[0]
      for k in idx[1:]:
        if vs[k] == to_value and vs[k - 1] != to_value:
          return ts[k]
      return None
    # stopping-state runs
    runs = []; i = 0
    while i < len(st):
      if st[i] == STOPPING:
        j = i
        while j < len(st) and st[j] == STOPPING:
          j += 1
        runs.append((co_t[i], co_t[j - 1])); i = j
      else:
        i += 1
    t0 = co_t[0]
    print(f"== {r}: {len(runs)} stopping runs, pose {'yes' if len(po_t) else 'NO'}")
    print("   t(s)    v_in  t_zero  a_zero  a_min  rebound  cmd_zero cmd_min  pre_rel  slope  bl_off  bl_on  eng  note")
    for (ts, te) in runs:
      k0 = np.searchsorted(cs_t, ts); k1 = np.searchsorted(cs_t, te + 1.0)
      if k1 - k0 < 20:
        continue
      vv = v[k0:k1]; tt = cs_t[k0:k1]
      zero = np.where(vv < 0.01)[0]
      if not len(zero):
        continue  # never stopped (driver took over or plan released)
      kz = zero[0]; tz = tt[kz]
      if tz + 1.0 > te + 1.0 and te - tz < 1.0:
        pass
      if brake[k0:k1][:kz + 1].any() or gas[k0:k1][:kz + 1].any():
        note = "driver pedal"
      else:
        note = ""
      v_in = vv[0]
      # pose around the stop
      sel = (po_t >= tz - 1.5) & (po_t <= tz + 1.5)
      if sel.sum() < 5:
        a_zero = a_min = reb = float("nan"); t_min = tz
      else:
        a_zero = interp(po_t, po_a, tz)
        pre = (po_t >= tz - 1.5) & (po_t <= tz + 1.2)
        im = np.argmin(np.where(pre, po_a, np.inf)); a_min = po_a[im]; t_min = po_t[im]
        post = (po_t > t_min) & (po_t <= t_min + 1.0)
        reb = po_a[post].max() if post.any() else float("nan")
      cmd_zero = interp(co_t, cmd, tz); cmd_min = interp(co_t, cmd, t_min)
      cmd_before = interp(co_t, cmd, tz - 0.6)
      pre_rel = cmd_zero - cmd_before >= 0.1
      jolt = "" if np.isnan(reb) else ("JOLT" if reb >= 0.2 or a_min <= -0.35 else "ok")
      slope = interp(pi_t, pi_v, tz) if len(pi_t) else float("nan")
      off = edge(bl_t, bl_v, tz - 1.5, tz + 1.5, 0); on = edge(bl_t, bl_v, (off if off is not None else tz), tz + 1.5, 1)
      off_s = f"{off - tz:+5.2f}" if off is not None else "  -  "; on_s = f"{on - tz:+5.2f}" if on is not None else "  -  "
      eng_s = f"{int(interp(en_t, en_v, tz) > 0.5)}" if len(en_t) else "?"
      print(f"  {ts - t0:7.1f}  {v_in:5.2f}  {tz - ts:6.2f}  {a_zero:+6.2f} {a_min:+6.2f}   {reb:+5.2f}    {cmd_zero:+5.2f}   {cmd_min:+5.2f}   {int(pre_rel)}     {slope:+5.1f}  {off_s}  {on_s}   {eng_s}   {jolt} {note}")
      if TRACE and jolt == "JOLT":
        print("      dt      v    aEgo  pose_a   cmd   aTgt  state bl eng")
        for tq in np.arange(tz - 2.0, tz + 1.2, 0.05):
          kk = min(np.searchsorted(co_t, tq), len(st) - 1)
          b = int(interp(bl_t, bl_v, tq) > 0.5) if len(bl_t) else "?"; e = int(interp(en_t, en_v, tq) > 0.5) if len(en_t) else "?"
          print(f"    {tq - tz:+5.2f}  {interp(cs_t, v, tq):5.2f}  {interp(cs_t, a_ego, tq):+5.2f}  {interp(po_t, po_a, tq):+6.2f}  {interp(co_t, cmd, tq):+5.2f}  {interp(pl_t, pl_a, tq):+5.2f}  {st[kk][:4]}  {b}   {e}")


if __name__ == "__main__":
  main()
