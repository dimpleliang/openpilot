#!/usr/bin/env python3
"""ars510: cut-in candidate dataset from raw radar CAN. For every object-list car that sits beside / partly in the ego lane
(0.8-3.0 m from the model path), 20-100 m ahead, slower than us, write one JSON line with its whole trajectory and a label:
entered (it came within 0.9 m of the path) / became_lead (openpilot took it as the lead at that range).
Usage: cutin_dataset.py OUT.jsonl ROUTE [ROUTE...] [--realdata DIR]"""
import glob
import json
import os
import sys

import numpy as np
import zstandard

sys.path.insert(0, "/data/openpilot"); sys.path.insert(0, "/home/claude/sp/opendbc_repo"); sys.path.insert(0, "/home/claude/sp")
from openpilot.cereal import log
import opendbc.car.toyota.ars510.interface as I

REC = []
_orig = I.decode_native_slot
def _hook(i, s):
  o = _orig(i, s); REC.append(o); return o
I.decode_native_slot = _hook

D_MIN, D_MAX, OFF_MIN, OFF_MAX, V_MIN, CLOSING_MIN, MIN_RECORDS = 20., 100., 0.8, 3.0, 50 / 3.6, 1.5, 16


def scan_segment(path, out, route, seg):
  data = zstandard.ZstdDecompressor().stream_reader(open(path, "rb")).read()
  ri = I.Ars510NativeRadarInterface(I.NativeInterfaceConfig(radar_bus=1))
  v = 0.; pathxy = None; lead = None; leads = []; life = {}; tracks = {}; wall = None
  for m in log.Event.read_multiple_bytes(data, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == "clocks" and wall is None: wall = (m.clocks.wallTimeNanos / 1e9, t)
    elif w == "carState": v = m.carState.vEgo
    elif w == "modelV2": pathxy = (list(m.modelV2.position.x), list(m.modelV2.position.y))
    elif w == "radarState":
      l = m.radarState.leadOne; lead = (l.dRel, l.yRel, l.radar) if l.present else None
      leads.append((t, lead))
    elif w == "can" and pathxy is not None:
      for c in m.can:
        if c.src not in (0, 1) or c.address not in (0x80, 0xB4, 0x24): continue
        REC.clear(); ri.update_frame(t, c.src, c.address, bytes(c.dat))
        for o in REC:
          prev = life.get(o.slot)
          if prev is None or o.age < prev[0]: life[o.slot] = [o.age, (o.slot, round(t, 2))]
          else: life[o.slot][0] = o.age
          key = life[o.slot][1]
          if not (o.geometry_valid and o.lateral_valid and D_MIN <= o.d_rel <= D_MAX and v > V_MIN): continue
          py = float(np.interp(o.d_rel, pathxy[0], pathxy[1])) if o.d_rel <= pathxy[0][-1] else 0.
          off = o.y_rel - py; vr = o.v_long_ground - v
          tr = tracks.setdefault(key, dict(route=route, seg=seg, slot=o.slot, t0=t, samples=[], cand=False, lead_at_start=lead))
          is_lead = lead is not None and abs(o.d_rel - lead[0]) < 6
          tr["samples"].append([round(t - tr["t0"], 2), round(o.d_rel, 1), round(o.y_rel, 2), round(off, 2), round(vr, 2), round(o.v_lat_ground, 2),
                                int(o.raw_weight_state128 or 0), int(o.lane_crossing_code or 0), int(o.existence_pct or 0), int(o.object_class or 0), int(is_lead), round(v, 1)])
          if OFF_MIN < abs(off) < OFF_MAX and vr < -CLOSING_MIN and not is_lead and o.age >= 8 and (o.object_class or 0) in (2, 3):
            tr["cand"] = True
  for key, tr in tracks.items():
    if not tr["cand"] or len(tr["samples"]) < MIN_RECORDS: continue
    s = tr["samples"]
    entered = any(abs(x[3]) < 0.9 and x[1] > 15 for x in s[8:])
    became_lead = any(x[10] for x in s[8:])
    t_end = tr["t0"] + s[-1][0]
    # lead taken within 10 s after the track ended at about its last range
    after = [l for tl, l in leads if t_end < tl < t_end + 10 and l is not None and abs(l[0] - s[-1][1]) < 8]
    tr["entered"] = entered; tr["became_lead"] = became_lead or bool(after)
    tr["clock"] = __import__("datetime").datetime.fromtimestamp(wall[0] + tr["t0"] - wall[1], __import__("datetime").timezone(__import__("datetime").timedelta(hours=8))).strftime("%H:%M:%S") if wall else ""
    tr["dur"] = round(s[-1][0], 1); del tr["t0"]; tr["lead_at_start"] = list(tr["lead_at_start"]) if tr["lead_at_start"] else None
    tr["samples"] = s[::2]
    out.write(json.dumps(tr) + "\n")


if __name__ == "__main__":
  args = sys.argv[1:]; realdata = "/data/media/0/realdata"
  if "--realdata" in args:
    i = args.index("--realdata"); realdata = args[i + 1]; del args[i:i + 2]
  outp = args[0]; routes = args[1:]
  with open(outp, "a") as out:
    for r in routes:
      segs = sorted(glob.glob(os.path.join(realdata, f"{r}--*")), key=lambda p: int(p.rsplit("--", 1)[1]))
      for sd in segs:
        f = os.path.join(sd, "rlog.zst")
        if not os.path.exists(f): continue
        try:
          scan_segment(f, out, r, int(sd.rsplit("--", 1)[1])); out.flush()
        except Exception as e:
          print("fail", sd, e, file=sys.stderr)
        print("done", sd.rsplit("/", 1)[1], flush=True)
