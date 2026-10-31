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
  S = H.Sections()
  last_t, streak, last_key, last_txt = 0., 0, None, None
  n = on = 0
  held, held_t = [], 0.
  for t, lon, lat, brg, v in gps_points(sys.argv[1]):
    if t - last_t < 1.0:
      continue
    last_t = t
    n += 1
    if v > H.MATCH_MIN_V:
      m = S.match(lon, lat, brg)
      if m:
        held, held_t = m, t
    else:
      m = held if held and t - held_t < H.HOLD_SLOW_S else []
    key = (m[0][1], m[0][2]) if m else None
    streak = streak + 1 if key is not None and key == last_key else (1 if key else 0)
    last_key = key
    txt = []
    if m and streak >= H.MATCH_FRAMES:
      on += 1
      txt = [H.alert_text(d, e) for d, e in H.ahead(m, events_at(t), lon, lat, brg)[:3]]
    road = f"{key[0]}{key[1]} {m[0][3]:6.2f}k" if m else "-"
    line = " | ".join(txt)
    if line != last_txt or (key and n % 120 == 0):
      print(time.strftime("%H:%M:%S", time.gmtime(t + 8 * 3600)), f"{v * 3.6:5.1f} km/h", road, line, flush=True)
      last_txt = line
  print(f"gps {n} s, matched to a freeway {on} s")


if __name__ == "__main__":
  main()
