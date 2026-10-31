#!/usr/bin/env python3
"""Replay the freeway incident alert (selfdrive/hwyalert/hwyalertd.py) on a recorded drive.

GPS comes from the route's qlogs (gpsLocation / gpsLocationExternal), the events from the 高公局 per-minute history
(LiveEvents/YYYYMMDD/LiveEventList_HHMM.xml.gz) at the drive's wall time. Prints the matched road and every change of
the alert list.
  PYTHONPATH=/data/openpilot python3 tools/ars510/hwyalert_replay.py <route prefix, e.g. 0000012a--1f2e3d4c5b>
"""
import glob
import gzip
import os
import sys
import time
import urllib.request

import zstandard
from openpilot.cereal import log
from openpilot.selfdrive.hwyalert import hwyalertd as H

REALDATA = os.environ.get("REALDATA", "/data/media/0/realdata")
HIST = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveEvents/{d}/LiveEventList_{hm}.xml.gz"


def gps_points(prefix):
  segs = sorted(glob.glob(os.path.join(REALDATA, prefix + "--*")), key=lambda s: int(s.rsplit("--", 1)[1]))
  for seg in segs:
    for name in ("qlog.zst", "qlog", "rlog.zst"):
      p = os.path.join(seg, name)
      if os.path.exists(p):
        break
    else:
      continue
    raw = open(p, "rb").read()
    if p.endswith(".zst"):
      try:
        raw = zstandard.ZstdDecompressor().stream_reader(raw).read()
      except Exception:
        raw = zstandard.ZstdDecompressor().decompressobj().decompress(raw)
    try:
      for m in log.Event.read_multiple_bytes(raw, traversal_limit_in_words=2**61):
        w = m.which()
        if w in ("gpsLocation", "gpsLocationExternal"):
          g = getattr(m, w)
          if g.hasFix and g.unixTimestampMillis > 0:
            yield g.unixTimestampMillis / 1000., g.longitude, g.latitude, g.bearingDeg, g.speed
    except Exception:
      continue


_cache = {}


def events_at(t):
  lt = time.gmtime(t + 8 * 3600)  # Taiwan time names the history files
  key = (time.strftime("%Y%m%d", lt), time.strftime("%H%M", lt))
  if key not in _cache:
    try:
      data = urllib.request.urlopen(HIST.format(d=key[0], hm=key[1]), timeout=20).read()
      _cache[key] = H.parse_feed(gzip.decompress(data), t)
    except Exception as e:
      print("fetch failed", key, e)
      _cache[key] = []
  return _cache[key]


def main():
  tr = H.Tracker(H.Sections())
  last_t, last_txt, n, on = 0., None, 0, 0
  for t, lon, lat, brg, v in gps_points(sys.argv[1]):
    if t - last_t < 1.0:
      continue
    last_t = t
    n += 1
    state, say = tr.step(t, (lon, lat, brg), v, events_at(t))
    on += bool(state["road"])
    line = " | ".join(a["text"] for a in state["alerts"]) + (f"   [語音 {'+'.join(say)}]" if say else "")
    if line != last_txt or (state["road"] and n % 120 == 0):
      road = f"{state['road']} {state['km']:6.2f}k" if state["road"] else "-"
      print(time.strftime("%H:%M:%S", time.gmtime(t + 8 * 3600)), f"{v * 3.6:5.1f} km/h", road, line, flush=True)
      last_txt = line
  print(f"gps {n} s, matched to a freeway {on} s")


if __name__ == "__main__":
  main()
