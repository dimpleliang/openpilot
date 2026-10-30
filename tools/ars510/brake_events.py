#!/usr/bin/env python3
"""ars510 on-device: every braking event (aTarget <= THR) in a route, with a compact trace and an automatic class.
PYTHONPATH=/data/openpilot python3 brake_events.py ROUTE [THR]"""
import sys, glob, json, bisect
import zstandard
sys.path.insert(0, "/data/openpilot")
from openpilot.cereal import log
route = sys.argv[1]; THR = float(sys.argv[2]) if len(sys.argv) > 2 else -1.3
segs = sorted(glob.glob(f"/data/media/0/realdata/{route}--*"), key=lambda p: int(p.rsplit("--", 1)[1]))
plan = []; cs = []; lead = []; cam = []; ev = []; T0 = None
for sd in segs:
  try: data = zstandard.ZstdDecompressor().stream_reader(open(sd + "/rlog.zst", "rb")).read()
  except Exception: continue
  for m in log.Event.read_multiple_bytes(data, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if T0 is None: T0 = t
    if w == "longitudinalPlan": plan.append((t, m.longitudinalPlan.aTarget, str(m.longitudinalPlan.longitudinalPlanSource)[:5]))
    elif w == "carState":
      c = m.carState; cs.append((t, c.vEgo, c.aEgo, c.gasPressed, c.brakePressed, c.cruiseState.enabled))
    elif w == "radarState":
      l = m.radarState.leadOne; lead.append((t, l.present, l.radar, l.dRel, l.vRel, l.aLeadK, l.radarTrackId, l.yRel))
    elif w == "modelV2":
      L = m.modelV2.leadsV3; cam.append((t, L[0].prob if len(L) else 0, L[0].x[0] if len(L) else 0))
    elif w == "logMessage" and "ARS510 far lead" in m.logMessage:
      try: msg = json.loads(m.logMessage).get("msg", m.logMessage)
      except Exception: msg = m.logMessage
      ev.append((t, msg))
pt = [p[0] for p in plan]; ct = [c[0] for c in cs]; lt = [l[0] for l in lead]; et = [e[0] for e in ev]
def at(arr, ts, t):
  i = bisect.bisect_right(ts, t) - 1
  return arr[max(i, 0)]
# events: local minima runs below THR, separated by >= 8 s
events = []; i = 0
while i < len(plan):
  if plan[i][1] <= THR:
    j = i
    while j < len(plan) and (plan[j][1] <= THR or plan[j][0] - plan[i][0] < 8): j += 1
    k = min(range(i, j), key=lambda n: plan[n][1]); events.append(k); i = j
  else: i += 1
print(f"== {route}: {len(events)} events with aTarget <= {THR}")
for k in events:
  t0, amin, src = plan[k]
  c0 = at(cs, ct, t0); eng = c0[5]
  brk = any(at(cs, ct, t0 + d)[4] for d in (-3, -2, -1, 0, 1)); gas = any(at(cs, ct, t0 + d)[3] for d in (-3, -2, -1, 0))
  L = [at(lead, lt, t0 + d) for d in (-6, -4, -2, -1, 0)]
  al_min = min(l[5] for l in L if l[1]) if any(l[1] for l in L) else 0
  ids = {l[6] for l in L if l[1] and l[2]}
  ds = [l[3] for l in L if l[1]]
  jump = any(ds[n] - ds[n + 1] > 10 for n in range(len(ds) - 1)) or len(ids) > 1
  nolead_before = not L[0][1] and L[-1][1]
  v6 = at(cs, ct, t0 - 6)[1]; v0 = c0[1]
  d6 = L[0][3] if L[0][1] else 0
  if not eng: cls = "not engaged"
  elif brk: cls = "driver brake"
  elif al_min < -1.0: cls = "lead braking"
  elif jump or nolead_before: cls = "cut-in / new lead"
  elif d6 > 60: cls = "FAR APPROACH"
  elif v0 < 8: cls = "stopping"
  else: cls = "close follow"
  early = [e[1][15:60] for e in ev if t0 - 12 <= e[0] <= t0 and " off" not in e[1]]
  print(f"+{(t0-T0)/60:5.2f}min aT={amin:5.2f} {src:5} v {v6*3.6:3.0f}->{v0*3.6:3.0f} km/h | lead -6s:{'%.0fm %+.1f'%(L[0][3],L[0][4]) if L[0][1] else 'none':12} -2s:{'%.0fm %+.1f a%.1f'%(L[2][3],L[2][4],L[2][5]) if L[2][1] else 'none':18} 0s:{'%.0fm %+.1f a%.1f %s'%(L[4][3],L[4][4],L[4][5],'R' if L[4][2] else 'C') if L[4][1] else 'none':20} | gas{int(gas)} | {cls} | early:{len(early)}")
