#!/usr/bin/env python3
"""Parked, READ-ONLY: find where the combination meter keeps its brightness setting (11/03).

The meter's brightness menu (steering-wheel arrow keys) is not on the comma's CAN bus, and 0x620 METER_DIMMED sent from the
comma does not reach the meter (meter_dim_test.py). The meter does answer UDS diagnostics from the comma's bus (openpilot's
FW query notes "Combination Meter (0x7c0) responds to UDS"). This tool only sends TesterPresent (0x3E) and
ReadDataByIdentifier (0x22) to 0x7C0 in the default session: nothing is written, no session or security change.

  scan:    read every DID in RANGES, keep the ones that answer -> /data/ars510_meter_dids_<tag>.json
  reread:  read only the DIDs found by a scan again (after the driver changed the brightness) and print what changed
  options: --full      every DID 0x0000-0xFFFF (~22 min) instead of RANGES
           --extended  enter the extended diagnostic session first (0x10 0x03: reads only; the meter drops back to the
                       default session by itself when the tester-present keep-alive stops), and also read the KWP-style
                       ReadDataByLocalIdentifier (0x21) 0x00-0xFF
           --base TAG  reread compares with this scan (default A)

Run only parked (P, READY, openpilot stopped: the panda has one owner); --parked = the caller checked carState first.
  PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo /usr/local/venv/bin/python3 meter_uds_scan.py scan A --parked
  ... change the meter brightness in its menu ...
  PYTHONPATH=... meter_uds_scan.py reread B --parked        (compares with A)
Then: sudo reboot
"""
import json
import sys
import time

from opendbc.car.structs import CarParams
from opendbc.car.uds import UdsClient, NegativeResponseError, MessageTimeoutError, SESSION_TYPE
from panda import Panda

METER = 0x7C0
RANGES = [(0x0000, 0x0300), (0x1000, 0x1300), (0x2000, 0x2300), (0xA000, 0xA400), (0xB000, 0xB400), (0xF100, 0xF200)]
ALLOUTPUT_PASSTHROUGH = 1


def be(d, sb, n):
  v, bit = 0, sb
  for _ in range(n):
    v = (v << 1) | ((d[bit // 8] >> (bit % 8)) & 1)
    bit = bit + 15 if bit % 8 == 0 else bit - 1
  return v


def meter_pct(p, wait=1.0):
  end = time.monotonic() + wait
  while time.monotonic() < end:
    for addr, dat, bus in p.can_recv():
      if bus == 0 and addr == 0x610 and len(dat) == 8:
        return be(dat, 30, 7)
  return None


def main():
  if len(sys.argv) < 3 or sys.argv[1] not in ("scan", "reread") or "--parked" not in sys.argv:
    sys.exit(__doc__)
  mode, tag = sys.argv[1], sys.argv[2]
  p = Panda()
  p.set_safety_mode(CarParams.SafetyModel.allOutput, ALLOUTPUT_PASSTHROUGH)
  time.sleep(0.2)
  if p.health()["safety_mode"] != CarParams.SafetyModel.allOutput:
    p.set_safety_mode(CarParams.SafetyModel.silent)
    sys.exit("panda refused allOutput")
  out = {}
  try:
    pct = meter_pct(p)
    print(f"meter brightness now: {pct} %", flush=True)
    uds = UdsClient(p, METER, bus=0, timeout=0.1)
    try:
      uds.tester_present()
      print("meter 0x7C0 answers TesterPresent", flush=True)
    except Exception as e:
      print(f"no TesterPresent answer ({e}); trying reads anyway", flush=True)
    extended = "--extended" in sys.argv
    if extended:
      uds.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)
      print("extended diagnostic session", flush=True)
    base_tag = sys.argv[sys.argv.index("--base") + 1] if "--base" in sys.argv else "A"
    if mode == "scan":
      dids = list(range(0x10000)) if "--full" in sys.argv else [d for lo, hi in RANGES for d in range(lo, hi)]
      lids = list(range(0x100)) if extended else []
    else:
      base = json.load(open(f"/data/ars510_meter_dids_{base_tag}.json"))
      dids = [int(k, 16) for k in base["dids"] if not k.startswith("L")]
      lids = [int(k[1:], 16) for k in base["dids"] if k.startswith("L")]
    nrc, tmo, t0, last_tp = {}, 0, time.monotonic(), time.monotonic()
    for lid in lids:   # Toyota / KWP-style ReadDataByLocalIdentifier
      try:
        out[f"L{lid:02x}"] = uds._uds_request(0x21,
                                              subfunction=None, data=bytes([lid])).hex()
      except NegativeResponseError as e:
        nrc[e.error_code] = nrc.get(e.error_code, 0) + 1
      except Exception:
        tmo += 1
    if lids:
      print(f"0x21 local identifiers: {sum(1 for k in out if k.startswith('L'))} answered", flush=True)
    for i, did in enumerate(dids):
      if extended and time.monotonic() - last_tp > 1.5:   # keep the extended session alive
        try:
          uds.tester_present()
        except Exception:
          pass
        last_tp = time.monotonic()
      try:
        out[f"{did:04x}"] = uds.read_data_by_identifier(did).hex()
      except NegativeResponseError as e:
        nrc[e.error_code] = nrc.get(e.error_code, 0) + 1
      except MessageTimeoutError:
        tmo += 1
      except Exception:
        tmo += 1
      if i % 4096 == 4095 or (len(dids) < 10000 and i % 512 == 511):
        print(f"  {i + 1}/{len(dids)} read, {len(out)} answered, {time.monotonic() - t0:.0f} s", flush=True)
    pct2 = meter_pct(p)
    print(f"done: {len(out)} DIDs answered, negative {nrc}, timeouts {tmo}, meter {pct2} %", flush=True)
    json.dump(dict(tag=tag, meter_pct=pct2, dids=out), open(f"/data/ars510_meter_dids_{tag}.json", "w"), indent=1)
    if mode == "reread":
      print(f"brightness {base_tag} {base.get('meter_pct')} % -> {tag} {pct2} %; DIDs that changed:", flush=True)
      for k, v in out.items():
        if base["dids"].get(k) != v:
          print(f"  {k}: {base['dids'].get(k)} -> {v}", flush=True)
  finally:
    p.set_safety_mode(CarParams.SafetyModel.silent)
    print("panda back to silent; reboot the device to restart openpilot", flush=True)


if __name__ == "__main__":
  main()
