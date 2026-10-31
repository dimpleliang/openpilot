#!/usr/bin/env python3
"""ars510: replay the display cut-in rule (ui/sunnypilot/onroad/ars510_tracks.py) on rlogs and score it.

Runs on the device (offroad) or anywhere with the openpilot tree:
  PYTHONPATH=/data/openpilot nice -n 19 python3 tools/ars510/merge_eval.py [route_prefix ...] [--seg a-b] [--from HH:MM:SS --to HH:MM:SS]

For each track the rule marks, an episode is TRUE when within HORIZON s the same track is in my lane (|offset| < IN_LANE_M)
or becomes the radar lead; otherwise FALSE. Reports, for the old rule (near only) and the new one (near + wide merge):
episodes, true / false, recall over real cut-ins (tracks that came from >= 1.8 m out into my lane at > 60 km/h),
the warning time before the car reached my lane, and what a planner coast (ceiling -0.5 while marked, > 60 km/h)
would have cost on false episodes (seconds coasting).
"""
import glob
import os
import struct
import sys
import zstandard
import numpy as np
from collections import defaultdict
from openpilot.cereal import log

REALDATA = "/data/media/0/realdata"
FMT = "<fffBH"
SZ = struct.calcsize(FMT)
HORIZON = 8.0
IN_LANE_M = 0.9
CUT_FROM_M = 1.8
MIN_V = 60 / 3.6

# keep in sync with ars510_tracks.py
WIN, RATE, OFF_MIN, OFF_MAX, D, VREL, HOLD, MIN_VG = 2.0, -0.15, 0.8, 2.6, (25., 90.), -1.5, 10, 2.0
W_OFF_MAX, W_RATE, W_VREL, W_D, RELEASE = 5.0, -0.3, -2.5, (25., 110.), 20


class Rule:
  def __init__(self, wide):
    self.wide = wide
    self.hist = defaultdict(list); self.hold = defaultdict(int); self.rel = defaultdict(int)

  def step(self, tid, frame, off, d, vrel, settling):
    h = self.hist[tid]; h.append((frame, off))
    while h and frame - h[0][0] > WIN * 20:
      h.pop(0)
    near = OFF_MIN < off < OFF_MAX and vrel < VREL and D[0] <= d <= D[1]
    wide = self.wide and OFF_MAX <= off < W_OFF_MAX and vrel < W_VREL and W_D[0] <= d <= W_D[1]
    ok = False
    if len(h) >= 6 and not settling and (near or wide):
      t = np.array([x[0] for x in h], float) / 20.; o = np.array([x[1] for x in h], float)
      slope = float(np.polyfit(t, o, 1)[0]) if t[-1] - t[0] > 0.5 else 0.
      ok = slope < (RATE if near else W_RATE)
    if ok:
      self.hold[tid] += 1
      if self.hold[tid] >= HOLD and self.wide:
        self.rel[tid] = RELEASE
    else:
      self.hold[tid] = 0
      if self.rel[tid] > 0:
        self.rel[tid] -= 1
    return self.hold[tid] >= HOLD or self.rel[tid] > 0


def segments(prefixes):
  out = []
  for p in prefixes:
    for s in glob.glob(os.path.join(REALDATA, p + "*")):
      if os.path.exists(os.path.join(s, "rlog.zst")):
        out.append(s)
  return sorted(out, key=lambda s: (s.rsplit("--", 1)[0], int(s.rsplit("--", 1)[1])))


def read(seg):
  raw = open(os.path.join(seg, "rlog.zst"), "rb").read()
  try:
    d = zstandard.ZstdDecompressor().stream_reader(raw).read()
  except Exception:
    d = zstandard.ZstdDecompressor().decompressobj().decompress(raw)
  try:
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      yield m
  except Exception:
    return


def main():
  args = [a for a in sys.argv[1:] if not a.startswith("--")]
  segs = segments(args or ["0000"])
  rules = {"old": Rule(False), "new": Rule(True)}
  marks = {k: {} for k in rules}           # rule -> (route, tid) -> [t_first, t_last, d0, off0, vrel0, v_ego, coast_s]
  tracks = defaultdict(list)               # (route, tid) -> [(t, d, off, is_lead, v)]
  km = 0.
  for seg in segs:
    route = os.path.basename(seg).rsplit("--", 1)[0]
    v = 0.; path = None; leads = []; frame = 0; last_t = None
    for m in read(seg):
      w = m.which(); t = m.logMonoTime / 1e9
      if w == "carState":
        if last_t is not None:
          km += m.carState.vEgo * (t - last_t) / 1000
        last_t = t; v = m.carState.vEgo
      elif w == "modelV2":
        path = (np.array(m.modelV2.position.x), np.array(m.modelV2.position.y))
      elif w == "radarState":
        leads = [(l.dRel, l.yRel) for l in (m.radarState.leadOne, m.radarState.leadTwo) if l.present and l.radar]
      elif w == "customReservedRawData0" and path is not None and len(path[0]):
        p = bytes(m.customReservedRawData0)
        if len(p) < 4 or p[:2] != b"A5" or p[2] != 2:
          continue
        frame += 1
        for i in range(p[3]):
          o = 4 + i * SZ
          if o + SZ > len(p):
            break
          d, y, vr, fl, tid = struct.unpack_from(FMT, p, o)
          if not (0.5 < d < 120):
            continue
          is_lead = any(abs(d - a) < 4 and abs(y - b) < 1.5 for a, b in leads)
          off = abs(-y - float(np.interp(d, path[0], path[1])))
          key = (route, tid)
          tracks[key].append((t, d, off, is_lead, v))
          if is_lead or vr + v <= MIN_VG:
            continue
          for name, r in rules.items():
            if r.step(tid, frame, off, d, vr, bool(fl & 2)):
              e = marks[name].setdefault(key, [t, t, d, off, vr, v, 0.])
              if v > MIN_V:
                e[6] += 1 / 20
              e[1] = t
  # real cut-ins: a track that was >= CUT_FROM_M out, then in my lane or the radar lead, at > 60 km/h, 15-110 m ahead
  real = {}
  for key, rows in tracks.items():
    out_t = None
    for t, d, off, is_lead, v in rows:
      if off >= CUT_FROM_M and not is_lead:
        out_t = t
      elif out_t is not None and (off < IN_LANE_M or is_lead) and v > MIN_V and 15 < d < 110 and t - out_t < 6:
        real[key] = t
        break
  print(f"segments {len(segs)}, {km:.0f} km, real cut-ins (>60 km/h) {len(real)}")
  for name in rules:
    tp = fp = 0; lead_t = []; fp_coast = 0.; hit = set()
    for key, (t0, t1, d0, off0, vr0, v0, coast) in marks[name].items():
      tin = real.get(key)
      ok = tin is not None and t0 - 1.0 <= tin <= t1 + HORIZON
      if not ok:
        # also true when the same track is in my lane within HORIZON (slow cut-ins)
        ok = any(t0 <= t <= t1 + HORIZON and (off < IN_LANE_M or is_lead) for t, d, off, is_lead, v in tracks[key])
      if ok:
        tp += 1; hit.add(key)
        if tin is not None:
          lead_t.append(tin - t0)
      else:
        fp += 1; fp_coast += coast
    rec = len(hit & set(real)) / max(len(real), 1)
    lt = sorted(lead_t)
    med = lt[len(lt) // 2] if lt else float("nan")
    print(f"[{name}] episodes {tp + fp}: true {tp}, false {fp} ({fp / max(tp + fp, 1) * 100:.0f}%), recall {rec * 100:.0f}%, "
          f"warning median {med:.1f}s, false-coast {fp_coast:.0f}s ({fp_coast / max(km, 1) * 100:.1f} s/100km)")
  if "--list" in sys.argv:
    for key, e in sorted(marks["new"].items(), key=lambda x: x[1][0]):
      old = key in marks["old"]
      print(f"  {key[0]} tid {key[1]:5d} d={e[2]:5.1f} off={e[3]:4.1f} vrel={e[4]:+5.1f} v={e[5] * 3.6:5.1f} dur={e[1] - e[0]:4.1f}s "
            f"{'REAL' if key in real else '----'} {'old' if old else 'new-only'}")


if __name__ == "__main__":
  main()
