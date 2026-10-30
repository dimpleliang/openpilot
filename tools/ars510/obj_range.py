import sys, glob, zstandard, capnp, numpy as np
sys.path.insert(0, "/home/claude/sp/opendbc_repo")
capnp.remove_import_hook()
SP = "/home/claude/sp"
log = capnp.load(f"{SP}/openpilot/cereal/log.capnp", imports=[f"{SP}/openpilot/cereal", f"{SP}/opendbc_repo/opendbc/car", f"{SP}/openpilot", SP])
import opendbc.car.toyota.ars510.interface as I
REC = []
_orig = I.decode_native_slot
def rec(i, s):
  o = _orig(i, s); REC.append(o); return o
I.decode_native_slot = rec
for g in sys.argv[1:]:
  births = []; dmax = []; v = 0
  for f in sorted(glob.glob(g + "/**/*.zst", recursive=True)):
    d = zstandard.ZstdDecompressor().stream_reader(open(f, "rb")).read()
    ri = I.Ars510NativeRadarInterface(I.NativeInterfaceConfig(radar_bus=1))
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      w = m.which()
      if w == "carState": v = m.carState.vEgo
      elif w == "can":
        t = m.logMonoTime / 1e9
        for c in m.can:
          if c.src not in (0, 1) or c.address not in (0x80, 0xB4, 0x24): continue
          REC.clear()
          ri.update_frame(t, c.src, c.address, bytes(c.dat))
          if REC and v > 70 / 3.6:
            ok = [o for o in REC if o.geometry_valid and o.lateral_valid and o.age >= 1]
            if ok: dmax.append(max(o.d_rel for o in ok))
            for o in ok:
              if o.age == 2 and o.d_rel > 30 and o.v_long_ground - v < -1 and abs(o.y_rel) < 6: births.append(o.d_rel)
  b = np.array(births); dm = np.array(dmax)
  pr = lambda a: np.percentile(a, [50, 90, 99, 100]).round(0) if len(a) else "-"
  print(g.split("/")[-1], "records", len(dm), "farthest object p50/p90/p99/max", pr(dm), "| new approaching objects (|y|<6 m, >30 m) n", len(b), "p50/p90/p99/max", pr(b))
