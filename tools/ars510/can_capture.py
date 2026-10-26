#!/usr/bin/env python3
"""dimpleliang/ars510: record live CAN from pandad for a labelled interval (e.g. nav_on / nav_off) and summarise the
Toyota head unit (AVN) messages from toyota_2017_ref_pt.dbc.

  PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 /data/openpilot/tools/ars510/can_capture.py nav_on 60
Writes /data/ars510_can_<label>.csv.gz (mono_time, src bus, address, data hex) and prints a summary.
"""
import collections, csv, gzip, sys, time
import openpilot.cereal.messaging as messaging

AVN = {900: "AVN1S01", 902: "AVN1S03", 905: "AVN1S08", 911: "AVN1S07", 933: "AVN1S13 (TBT/HUD)", 936: "AVN1S16",
       998: "AVN1S11", 999: "AVN1S17", 1000: "AVN1S18", 1001: "AVN1S19", 1043: "AVN1S31", 1076: "AVN1S20", 1077: "AVN1S21"}


def main(label: str, seconds: float) -> None:
  sock = messaging.sub_sock("can", conflate=False, timeout=100)
  out = f"/data/ars510_can_{label}.csv.gz"
  counts = collections.Counter()
  payloads = collections.defaultdict(collections.Counter)
  end = time.monotonic() + seconds
  with gzip.open(out, "wt") as f:
    w = csv.writer(f)
    w.writerow(["t", "src", "addr", "dat"])
    while time.monotonic() < end:
      for msg in messaging.drain_sock(sock, wait_for_one=True):
        t = msg.logMonoTime / 1e9
        for c in msg.can:
          dat = bytes(c.dat).hex()
          w.writerow([f"{t:.3f}", c.src, c.address, dat])
          counts[(c.src, c.address)] += 1
          if c.address in AVN:
            payloads[(c.src, c.address)][dat] += 1
  print(f"{label}: {seconds:.0f} s, {sum(counts.values())} frames, {len(counts)} (bus, id) pairs -> {out}")
  per_bus = collections.Counter()
  for (bus, _), n in counts.items():
    per_bus[bus] += n
  print("frames per bus:", dict(sorted(per_bus.items())))
  found = [(k, n) for k, n in sorted(counts.items()) if k[1] in AVN]
  if not found:
    print("no AVN head-unit messages on any bus")
  for (bus, addr), n in found:
    print(f"bus {bus} 0x{addr:03X} {AVN[addr]}: {n} frames ({n / seconds:.1f} Hz), payloads: {payloads[(bus, addr)].most_common(5)}")


if __name__ == "__main__":
  main(sys.argv[1] if len(sys.argv) > 1 else "capture", float(sys.argv[2]) if len(sys.argv) > 2 else 60.0)
