#!/usr/bin/env python3
"""ars510: learn the car's lift-off (coast) deceleration per speed band from rlogs, slope removed.

  PYTHONPATH=/data/openpilot python3 tools/ars510/coast_learn.py route_prefix [...]

Coasting = no gas, no brake pedal (and none for 1.5 s), v > 3 m/s, |road pitch| < 0.02 rad, and either
  driver: openpilot not engaged, or
  engaged: openpilot asking for accel <= -0.2 (so the ECU has lifted the throttle, below hold-speed noise) and the ACC brake lights are off
           (ESP_CONTROL.BRAKE_LIGHTS_ACC, CAN 0x3B7) for the last 1 s.
Road pitch = liveLocationKalman device pitch plus the mount pitch (extrinsicsCalibration.rpyCalib[1]; verified sign: residual median 0.4 deg). D gear only. Prints per band the
measured deceleration (raw and slope-corrected, driver and engaged separately) next to a drag-only estimate (rho 1.2, Cd 0.32,
A 2.6 m2, 1900 kg, Crr 0.010); the gap to the estimate is what regen / engine braking adds.
"""
import glob
import math
import os
import sys

import numpy as np
import zstandard
from openpilot.cereal import log

REALDATA = os.environ.get("REALDATA", "/data/media/0/realdata")
BANDS = [(10, 30), (30, 50), (50, 70), (70, 90), (90, 110), (110, 130)]


def read(p):
  raw = open(p, "rb").read()
  try:
    return zstandard.ZstdDecompressor().stream_reader(raw).read()
  except Exception:
    return zstandard.ZstdDecompressor().decompressobj().decompress(raw)


def main():
  samples = []
  for r in sys.argv[1:]:
    for seg in sorted(glob.glob(f"{REALDATA}/{r}--*"), key=lambda s: int(s.rsplit("--", 1)[1])):
      p = os.path.join(seg, "rlog.zst")
      if not os.path.exists(p):
        continue
      try:
        d = read(p)
      except Exception:
        continue
      cs = ss = None; last_press = -99.; pitch = None; n = 0; calib = 0.0; last_acc_brake = -99.; eng_running = None
      try:
        for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
          w = m.which(); t = m.logMonoTime / 1e9
          if w == "carState":
            cs = m.carState
            if cs.gasPressed or cs.brakePressed:
              last_press = t
          elif w == "selfdriveState":
            ss = m.selfdriveState
          elif w == "livePose":
            try:
              pitch = float(m.livePose.orientationNED.y)
            except Exception:
              pass
          elif w == "liveLocationKalman":  # this fork logs the Kalman pose, not livePose
            llk = m.liveLocationKalman
            if llk.orientationNED.valid and len(llk.orientationNED.value) == 3:
              pitch = float(llk.orientationNED.value[1])
          elif w == "extrinsicsCalibration":
            if len(m.extrinsicsCalibration.rpyCalib) == 3:
              calib = float(m.extrinsicsCalibration.rpyCalib[1])
          elif w == "can":
            for c in m.can:
              if c.src != 0:
                continue
              if c.address == 0x3B7 and len(c.dat) >= 3 and (c.dat[2] >> 2) & 1:  # BRAKE_LIGHTS_ACC 18|1@0+
                last_acc_brake = t
              elif c.address == 0x1C4 and len(c.dat) >= 4:  # ENGINE_RUNNING 27|1@0+
                eng_running = bool((c.dat[3] >> 3) & 1)
          elif w == "carOutput" and cs is not None and pitch is not None:
            n += 1
            if n % 10:
              continue
            eng = ss.enabled if ss else False
            no_pedal = not cs.gasPressed and not cs.brakePressed and t - last_press > 1.5
            if eng:
              coasting = no_pedal and m.carOutput.actuatorsOutput.accel <= -0.2 and t - last_acc_brake > 1.0
            else:
              coasting = no_pedal
            road_pitch = pitch + calib
            if coasting and cs.vEgo > 3 and abs(road_pitch) < 0.02 and cs.gearShifter == "drive":
              samples.append((cs.vEgo, cs.aEgo, road_pitch, cs.aEgo + 9.81 * math.sin(road_pitch), eng,
                              -1 if eng_running is None else int(eng_running)))
      except Exception:
        pass
  a = np.array(samples)
  if not len(a):
    print("no samples"); return
  print(f"samples {len(a)} (10 Hz): driver {int((a[:, 4] == 0).sum())}, engaged {int((a[:, 4] == 1).sum())}; "
        f"road pitch median {np.degrees(np.median(a[:, 2])):+.2f} deg; engine running in {int((a[:, 5] == 1).sum())} samples")
  rho, cd, A, m, crr = 1.2, 0.32, 2.6, 1900, 0.010
  for name, sel in (("driver", a[:, 4] == 0), ("engaged", a[:, 4] == 1)):
    print(f"\n[{name}]  km/h      n   a_raw p25/p50/p75   slope-corr p50   drag-only")
    for lo, hi in BANDS:
      s = a[sel & (a[:, 0] * 3.6 >= lo) & (a[:, 0] * 3.6 < hi)]
      v = (lo + hi) / 2 / 3.6
      est = -(0.5 * rho * cd * A * v * v + crr * m * 9.81) / m
      if len(s) < 5:
        print(f"        {lo:3d}-{hi:3d}   n {len(s)}")
        continue
      print(f"        {lo:3d}-{hi:3d} {len(s):5d}  {np.percentile(s[:, 1], 25):+.2f}/{np.median(s[:, 1]):+.2f}/{np.percentile(s[:, 1], 75):+.2f}"
            f"     {np.median(s[:, 3]):+.2f}            {est:+.2f}")


if __name__ == "__main__":
  main()
