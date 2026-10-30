#!/usr/bin/env python3
"""ars510 reportd: offroad only (manager). Every minute, generate the drive report for every route in realdata that has
none yet or has gained segments; newest first, at the lowest CPU priority. Reports: /data/ars510_reports/<route>/."""
import glob
import json
import os
import time

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ars510.report import REALDATA, OUT_ROOT, generate, route_segments


def pending_routes():
  routes = sorted({os.path.basename(p).rsplit("--", 1)[0] for p in glob.glob(os.path.join(REALDATA, "*--*--*"))}, reverse=True)
  out = []
  for r in routes:
    st = os.path.join(OUT_ROOT, r, "status.json")
    n = len(route_segments(r))
    if n == 0:
      continue
    if os.path.exists(st):
      try:
        if json.load(open(st)).get("segments", 0) >= n:
          continue
      except Exception:
        pass
    out.append(r)
  return out


def main():
  os.nice(19)
  os.makedirs(OUT_ROOT, exist_ok=True)
  time.sleep(20)  # let the device settle after ignition off
  while True:
    for r in pending_routes():
      t0 = time.monotonic()
      try:
        generate(r)
        cloudlog.info(f"ars510 report {r} in {time.monotonic() - t0:.0f} s")
      except Exception:
        cloudlog.exception(f"ars510 report {r} failed")
      time.sleep(2)
    time.sleep(60)


if __name__ == "__main__":
  main()
