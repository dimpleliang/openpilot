#!/usr/bin/env python3
"""Lead detection range per log file: first-acquisition distance of model lead and radar lead at highway speed."""
import sys, glob, datetime, zstandard, capnp, numpy as np
capnp.remove_import_hook()
SP = "/home/claude/sp"
log = capnp.load(f"{SP}/openpilot/cereal/log.capnp", imports=[f"{SP}/openpilot/cereal", f"{SP}/opendbc_repo/opendbc/car", f"{SP}/openpilot", SP])
tz = datetime.timezone(datetime.timedelta(hours=8))
def scan(f):
  try: d = zstandard.ZstdDecompressor().stream_reader(open(f, "rb")).read()
  except Exception: return None
  v = 0; wall = None; mfirst = []; rfirst = []; m_last = -9; r_last = -9; m_x = []; r_x = []; trk_max = []; m_on = r_on = False
  for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == "clocks" and wall is None: wall = m.clocks.wallTimeNanos / 1e9
    elif w == "carState": v = m.carState.vEgo
    elif v < 70 / 3.6: continue
    elif w == "modelV2":
      L = m.modelV2.leadsV3
      on = len(L) > 0 and L[0].prob > 0.5
      if on:
        x = L[0].x[0]; m_x.append(x)
        if not m_on and t - m_last > 1.5: mfirst.append(x)
        m_last = t
      m_on = on
    elif w == "radarState":
      l = m.radarState.leadOne; on = l.present and l.radar
      if on:
        r_x.append(l.dRel)
        if not r_on and t - r_last > 1.5: rfirst.append(l.dRel)
        r_last = t
      r_on = on
    elif w in ("radarTracks", "liveTracks"):
      pts = m.radarTracks.points if w == "radarTracks" else []
      if len(pts): trk_max.append(max(p.dRel for p in pts))
  return wall, mfirst, rfirst, m_x, r_x, trk_max
groups = {}
for g in sys.argv[1:]:
  files = sorted(glob.glob(g + "/**/*.zst", recursive=True))
  agg = [None, [], [], [], [], []]
  for f in files:
    r = scan(f)
    if not r: continue
    if agg[0] is None and r[0]: agg[0] = r[0]
    for i in range(1, 6): agg[i] += r[i]
  wall = datetime.datetime.fromtimestamp(agg[0], tz).strftime("%m-%d %H:%M") if agg[0] else "?"
  def q(a, p): return f"{np.percentile(a, p):5.0f}" if len(a) else "    -"
  mf = [x for x in agg[1] if x > 40]; rf = [x for x in agg[2] if x > 40]
  print(f"{g.split('/')[-1]:13} {wall}  model 1st>40m n={len(mf):3} med={q(mf,50)} p90={q(mf,90)} | radar 1st>40m n={len(rf):3} med={q(rf,50)} p90={q(rf,90)} | "
        f"model x p99={q(agg[3],99)} max={q(agg[3],100)} | radarLead p99={q(agg[4],99)} max={q(agg[4],100)} | tracks max p99={q(agg[5],99)}")
