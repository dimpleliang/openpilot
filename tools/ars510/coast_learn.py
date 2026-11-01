#!/usr/bin/env python3
"""ars510: learn the car's lift-off (coast) deceleration per speed band from rlogs, slope removed.

  PYTHONPATH=/data/openpilot python3 tools/ars510/coast_learn.py route_prefix [...]

Coasting = no gas, no brake (and none for 1.5 s), openpilot either disengaged or asking for |accel| < 0.05, v > 3 m/s,
|pitch| < 0.02 rad (livePose, or liveLocationKalman on this fork). Prints per band the measured deceleration (raw and slope-corrected) next to a drag-only
estimate (rho 1.2, Cd 0.32, A 2.6 m2, 1900 kg, Crr 0.010); the gap to the estimate is what regen / engine braking adds.
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
      cs = ss = None; last_press = -99.; pitch = None; n = 0
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
          elif w == "carOutput" and cs is not None and pitch is not None:
            n += 1
            if n % 10:
              continue
            act = m.carOutput.actuatorsOutput
            eng = ss.enabled if ss else False
            coasting = (not cs.gasPressed and not cs.brakePressed and t - last_press > 1.5 and
                        ((not eng) or abs(act.accel) < 0.05))
            if coasting and cs.vEgo > 3 and abs(pitch) < 0.02:
              samples.append((cs.vEgo, cs.aEgo, pitch, cs.aEgo + 9.81 * math.sin(pitch), eng))
      except Exception:
        pass
  a = np.array(samples)
  print(f"samples {len(a)} (10 Hz); while engaged {int(a[:, 4].sum()) if len(a) else 0}")
  print("km/h      n   a_raw p25/p50/p75      slope-corrected p50   drag-only estimate")
  rho, cd, A, m, crr = 1.2, 0.32, 2.6, 1900, 0.010
  for lo, hi in BANDS:
    s = a[(a[:, 0] * 3.6 >= lo) & (a[:, 0] * 3.6 < hi)] if len(a) else a
    v = (lo + hi) / 2 / 3.6
    est = -(0.5 * rho * cd * A * v * v + crr * m * 9.81) / m
    if len(s) < 5:
      print(f"{lo:3d}-{hi:3d}   n {len(s)}")
      continue
    print(f"{lo:3d}-{hi:3d} {len(s):5d}  {np.percentile(s[:, 1], 25):+.2f}/{np.median(s[:, 1]):+.2f}/{np.percentile(s[:, 1], 75):+.2f}        "
          f"{np.median(s[:, 3]):+.2f}                 {est:+.2f}")


if __name__ == "__main__":
  main()
