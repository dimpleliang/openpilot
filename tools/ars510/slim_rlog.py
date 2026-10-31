#!/usr/bin/env python3
"""ars510: shrink rlogs to what the cut-in / merge replay needs, so they can be pulled over the slow ssh relay.

On the device:  PYTHONPATH=/data/openpilot nice -n 10 python3 slim_rlog.py OUT_DIR route_prefix [route_prefix ...]
Per segment OUT_DIR/<segment>.pkl.zst (pickle, zstd) with
  can:   [(t, bus, addr, data)]  bus 1 (all radar frames) + bus 0 0xB4 / 0x24 (wheel speed, yaw rate for the object list)
  car:   [(t, vEgo, steeringAngleDeg, leftBlinker, rightBlinker)]
  model: [(t, x[33], y[33])]     modelV2 position (the path)
  lead:  [(t, (present, radar, dRel, yRel, vRel) x2)]  radarState leadOne / leadTwo
  disp:  [(t, payload)]          customReservedRawData0 (display rows, v2 has track ids)
  wall:  (wall_s, mono_s) from the first clocks message
About 5-8 % of the rlog size. Segments already done are skipped.
"""
import glob
import os
import pickle
import sys
import zstandard
from openpilot.cereal import log

REALDATA = os.environ.get("REALDATA", "/data/media/0/realdata")
BUS0_KEEP = {0xB4, 0x24}


def slim(seg):
  raw = open(os.path.join(seg, "rlog.zst"), "rb").read()
  try:
    d = zstandard.ZstdDecompressor().stream_reader(raw).read()
  except Exception:
    d = zstandard.ZstdDecompressor().decompressobj().decompress(raw)
  out = {"can": [], "car": [], "model": [], "lead": [], "disp": [], "wall": None}
  try:
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      w = m.which(); t = m.logMonoTime / 1e9
      if w == "can":
        for c in m.can:
          if c.src == 1 or (c.src == 0 and c.address in BUS0_KEEP):
            out["can"].append((t, c.src, c.address, bytes(c.dat)))
      elif w == "carState":
        c = m.carState; out["car"].append((t, c.vEgo, c.steeringAngleDeg, c.leftBlinker, c.rightBlinker))
      elif w == "modelV2":
        p = m.modelV2.position; out["model"].append((t, list(p.x), list(p.y)))
      elif w == "radarState":
        ls = []
        for l in (m.radarState.leadOne, m.radarState.leadTwo):
          ls.append((l.present, l.radar, l.dRel, l.yRel, l.vRel))
        out["lead"].append((t, ls))
      elif w == "customReservedRawData0":
        out["disp"].append((t, bytes(m.customReservedRawData0)))
      elif w == "clocks" and out["wall"] is None:
        out["wall"] = (m.clocks.wallTimeNanos / 1e9, t)
  except Exception:
    pass  # truncated segment: keep what was read
  return out


def main():
  out_dir = sys.argv[1]; prefixes = sys.argv[2:] or ["0000"]
  os.makedirs(out_dir, exist_ok=True)
  segs = []
  for p in prefixes:
    segs += [s for s in glob.glob(os.path.join(REALDATA, p + "*")) if os.path.exists(os.path.join(s, "rlog.zst"))]
  segs = sorted(set(segs), key=lambda s: (s.rsplit("--", 1)[0], int(s.rsplit("--", 1)[1])))
  cctx = zstandard.ZstdCompressor(level=10)
  for i, seg in enumerate(segs):
    name = os.path.basename(seg)
    dst = os.path.join(out_dir, name + ".pkl.zst")
    if os.path.exists(dst):
      continue
    data = slim(seg)
    tmp = dst + ".part"
    with open(tmp, "wb") as f:
      f.write(cctx.compress(pickle.dumps(data, protocol=4)))
    os.replace(tmp, dst)
    print(f"{i + 1}/{len(segs)} {name} {os.path.getsize(dst) / 1e6:.1f} MB", flush=True)
  print("SLIM_DONE", flush=True)


if __name__ == "__main__":
  main()
