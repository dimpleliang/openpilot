#!/usr/bin/env python3
"""Episodes where the ARS510 ACC target (0x235/0x237) is far, in path and closing while openpilot has no lead.
Confirmed = openpilot later takes a lead at about that distance. Prints every episode."""
import sys, glob, datetime, zstandard, capnp
sys.path.insert(0, "/home/claude/sp/opendbc_repo")
capnp.remove_import_hook()
SP = "/home/claude/sp"
log = capnp.load(f"{SP}/openpilot/cereal/log.capnp", imports=[f"{SP}/openpilot/cereal", f"{SP}/opendbc_repo/opendbc/car", f"{SP}/openpilot", SP])
from opendbc.car.toyota.ars510.support import parse_acc_target_position, parse_acc_target_vrel
tz = datetime.timezone(datetime.timedelta(hours=8))
D_MIN, VREL_MAX, Y_MAX, V_MIN, HOLD = float(sys.argv[1]), -1.5, 1.5, 50 / 3.6, 1.0
tot = dict(n=0, conf=0)
for g in sys.argv[2:]:
  for f in sorted(glob.glob(g + "/**/*.zst", recursive=True)):
    d = zstandard.ZstdDecompressor().stream_reader(open(f, "rb")).read()
    wall = None; v = 0; acc = None; pos = None; lead = None; ep = None; eps = []; leads = []
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      w = m.which(); t = m.logMonoTime / 1e9
      if w == "clocks" and wall is None: wall = (m.clocks.wallTimeNanos / 1e9, t)
      elif w == "carState": v = m.carState.vEgo
      elif w == "radarState":
        l = m.radarState.leadOne; lead = (l.dRel, l.radar) if l.present else None
        if lead: leads.append((t, l.dRel, l.vRel))
        c = (acc is not None and pos is not None and t - acc[0] < 0.2 and v > V_MIN and lead is None and
             pos[1] >= D_MIN and acc[1] < VREL_MAX and abs(pos[2]) < Y_MAX)
        if c:
          if ep is None: ep = [t, pos[1], acc[1], v, t]
          ep[4] = t
        elif ep is not None:
          if ep[4] - ep[0] >= HOLD: eps.append(ep)
          ep = None
      elif w == "can":
        for x in m.can:
          if x.src != 1: continue
          if x.address == 0x235:
            dat = bytes(x.dat); acc = (t, parse_acc_target_vrel(dat)) if (dat[1] & 4) else None
          elif x.address == 0x237:
            dat = bytes(x.dat)
            if dat[2:] == bytes.fromhex("003E80000000"): pos = None
            else:
              p = parse_acc_target_position(dat); pos = (t, p[0], p[1])
    for t0, d0, vr0, v0, t1 in eps:
      conf = None
      for tl, dl, vl in leads:
        if t0 <= tl <= t0 + 10:
          dp = d0 + vr0 * (tl - t0)
          if abs(dl - dp) < 0.2 * dp + 5: conf = (tl - t0, dl); break
      tot["n"] += 1; tot["conf"] += conf is not None
      wt = datetime.datetime.fromtimestamp(wall[0] + t0 - wall[1], tz).strftime("%m-%d %H:%M:%S") if wall else "?"
      print(f"{wt} v={v0*3.6:5.1f} d={d0:6.1f} vRel={vr0:6.2f} held={t1-t0:4.1f}s ->", f"lead {conf[0]:.1f}s later at {conf[1]:.0f} m" if conf else "NOT confirmed", f"[{f.split('Downloads/')[-1]}]")
print("episodes", tot)
