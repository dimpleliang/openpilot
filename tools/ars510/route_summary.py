#!/usr/bin/env python3
"""ars510 on-device route summary: version, per-segment stats, ARS510 events. PYTHONPATH=/data/openpilot python3 route_summary.py ROUTE"""
import sys, glob, json, bisect
import zstandard
sys.path.insert(0, "/data/openpilot")
from openpilot.cereal import log
route = sys.argv[1]
segs = sorted(glob.glob(f"/data/media/0/realdata/{route}--*"), key=lambda p: int(p.rsplit("--", 1)[1]))
ev = []; plan = []; cs = []; lead = []; commit = None; T0 = None
print(f"== {route}: {len(segs)} segs", flush=True)
for sd in segs:
  try: data = zstandard.ZstdDecompressor().stream_reader(open(sd + "/rlog.zst", "rb")).read()
  except Exception: continue
  amin = 0; vmax = 0; brk = 0; n = 0; nl = 0; nr = 0; early = 0
  for m in log.Event.read_multiple_bytes(data, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if T0 is None: T0 = t
    if w == "initData" and commit is None: commit = m.initData.gitCommit[:7]
    elif w == "logMessage" and "ARS510" in m.logMessage:
      try: msg = json.loads(m.logMessage).get("msg", m.logMessage)
      except Exception: msg = m.logMessage
      ev.append((t, msg)); early += ("far lead early" in msg and " off" not in msg)
    elif w == "longitudinalPlan":
      a = m.longitudinalPlan.aTarget; amin = min(amin, a); plan.append((t, a))
    elif w == "carState":
      c = m.carState; vmax = max(vmax, c.vEgo); brk += c.brakePressed
      cs.append((t, c.vEgo, c.gasPressed, c.brakePressed))
    elif w == "radarState":
      l = m.radarState.leadOne; n += 1; nl += l.present; nr += (l.present and l.radar); lead.append((t, l.present, l.radar, l.dRel, l.vRel))
  print(f"seg {sd.rsplit('--',1)[1]:>2} +{(t-T0)/60:4.1f}min vmax {vmax*3.6:4.0f} aT_min {amin:5.2f} brake {brk:4d} lead {nl/max(n,1):.2f} radar {nr/max(n,1):.2f} early {early}", flush=True)
print("commit", commit)
pt = [p[0] for p in plan]; ct = [c[0] for c in cs]; lt = [l[0] for l in lead]
for t, msg in ev:
  if "far stopped" in msg or "far object" in msg: continue
  extra = ""
  if "far lead early" in msg and " off" not in msg:
    i0 = bisect.bisect_left(pt, t); i1 = bisect.bisect_left(pt, t + 12)
    amin = min((p[1] for p in plan[i0:i1]), default=0)
    j0 = bisect.bisect_left(ct, t); j1 = bisect.bisect_left(ct, t + 12)
    v0 = cs[j0][1] if j0 < len(cs) else 0; vmin = min((c[1] for c in cs[j0:j1]), default=v0)
    gas = any(c[2] for c in cs[j0:j1]); brk = any(c[3] for c in cs[j0:j1])
    extra = f" | 12s: aT min {amin:.2f}, v {v0*3.6:.0f}->{vmin*3.6:.0f}, gas{int(gas)} brk{int(brk)}"
  print(f"+{(t-T0)/60:5.2f}min {msg[:120]}{extra}")
