#!/usr/bin/env python3
"""ars510: pull the model lane lines out of rlogs (for lane-line cut-in rules in merge_eval_slim.py).

On the device:  PYTHONPATH=/data/openpilot nice -n 10 python3 slim_lanes.py OUT_DIR route_prefix [...]
Per segment OUT_DIR/<segment>.lanes.pkl.zst with
  x:     laneLines x grid (model frame, from the camera, metres; the same for every message)
  lanes: [(t, probs[4], y_left[33], y_right[33])]  laneLines[1] / [2] y (right-positive), float16
Segments already done are skipped.
"""
import glob
import os
import pickle
import sys

import numpy as np
import zstandard
from openpilot.cereal import log

REALDATA = os.environ.get("REALDATA", "/data/media/0/realdata")


def lanes_of(seg):
  raw = open(os.path.join(seg, "rlog.zst"), "rb").read()
  try:
    d = zstandard.ZstdDecompressor().stream_reader(raw).read()
  except Exception:
    d = zstandard.ZstdDecompressor().decompressobj().decompress(raw)
  out = {"x": None, "lanes": []}
  try:
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      if m.which() != "modelV2":
        continue
      ll = m.modelV2.laneLines
      if len(ll) < 3:
        continue
      if out["x"] is None:
        out["x"] = np.array(ll[1].x, dtype=np.float32)
      out["lanes"].append((m.logMonoTime / 1e9, np.array(m.modelV2.laneLineProbs, dtype=np.float16),
                           np.array(ll[1].y, dtype=np.float16), np.array(ll[2].y, dtype=np.float16)))
  except Exception:
    pass  # truncated segment: keep what was read
  return out


def main():
  out_dir = sys.argv[1]
  os.makedirs(out_dir, exist_ok=True)
  segs = []
  for p in sys.argv[2:]:
    segs += [s for s in glob.glob(os.path.join(REALDATA, p + "*")) if os.path.exists(os.path.join(s, "rlog.zst"))]
  segs = sorted(set(segs), key=lambda s: (s.rsplit("--", 1)[0], int(s.rsplit("--", 1)[1])))
  cctx = zstandard.ZstdCompressor(level=10)
  for i, seg in enumerate(segs):
    dst = os.path.join(out_dir, os.path.basename(seg) + ".lanes.pkl.zst")
    if os.path.exists(dst):
      continue
    data = lanes_of(seg)
    with open(dst + ".part", "wb") as f:
      f.write(cctx.compress(pickle.dumps(data, protocol=4)))
    os.replace(dst + ".part", dst)
    print(f"{i + 1}/{len(segs)} {os.path.basename(seg)} {os.path.getsize(dst) / 1e3:.0f} kB", flush=True)
  print("LANES_DONE", flush=True)


if __name__ == "__main__":
  main()
