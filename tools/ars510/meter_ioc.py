#!/usr/bin/env python3
"""Parked: look for the combination meter's Techstream "Active Test" (UDS InputOutputControlByIdentifier, 0x2F).

Toyota's service manual for the XX50 Avalon meter lists the Active Tests "Light Control Switch (UP)" / "(DOWN)": the
diagnostic equivalent of pressing the brightness up / down switch. If this RAV4's meter has them, the comma could step the
brightness the way the menu does (meter_uds_scan.py found no readable brightness value).

  probe            extended session; for every DID answered in /data/ars510_meter_dids_X.json (or --full: all 65536) send
                   only ReturnControlToECU (0x2F <DID> 0x00), which hands control back to the meter and changes nothing.
                   Any answer other than NRC 0x31 (requestOutOfRange) marks a DID that supports Active Test.
  pulse DID HEX S  ShortTermAdjustment (0x2F <DID> 0x03 <HEX>) for S seconds, then ReturnControlToECU; prints the meter's
                   reported brightness (0x610) before, during and after. Owner's go-ahead needed for each pulse.

Run parked (P, READY, openpilot stopped), --parked = the caller checked carState. Afterwards: sudo reboot.
"""
import json
import sys
import time

from opendbc.car.structs import CarParams
from opendbc.car.uds import UdsClient, NegativeResponseError, MessageTimeoutError, SESSION_TYPE
from panda import Panda

METER = 0x7C0
IOC = 0x2F
RETURN_CONTROL, SHORT_TERM_ADJUST = 0x00, 0x03


def be(d, sb, n):
  v, bit = 0, sb
  for _ in range(n):
    v = (v << 1) | ((d[bit // 8] >> (bit % 8)) & 1)
    bit = bit + 15 if bit % 8 == 0 else bit - 1
  return v


def meter_pct(p, wait=0.6):
  end = time.monotonic() + wait
  while time.monotonic() < end:
    for addr, dat, bus in p.can_recv():
      if bus == 0 and addr == 0x610 and len(dat) == 8:
        return be(dat, 30, 7)
  return None


def main():
  if len(sys.argv) < 2 or sys.argv[1] not in ("probe", "pulse") or "--parked" not in sys.argv:
    sys.exit(__doc__)
  p = Panda()
  p.set_safety_mode(CarParams.SafetyModel.allOutput, 1)   # passthrough: the camera keeps talking to the car
  time.sleep(0.2)
  if p.health()["safety_mode"] != CarParams.SafetyModel.allOutput:
    p.set_safety_mode(CarParams.SafetyModel.silent)
    sys.exit("panda refused allOutput")
  try:
    uds = UdsClient(p, METER, bus=0, timeout=0.1)
    uds.tester_present()
    uds.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)
    print(f"extended session; meter brightness {meter_pct(p)} %", flush=True)
    last_tp = time.monotonic()

    def keep_alive():
      nonlocal last_tp
      if time.monotonic() - last_tp > 1.5:
        try:
          uds.tester_present()
        except Exception:
          pass
        last_tp = time.monotonic()

    if sys.argv[1] == "probe":
      dids = list(range(0x10000)) if "--full" in sys.argv else \
        [int(k, 16) for k in json.load(open("/data/ars510_meter_dids_X.json"))["dids"] if not k.startswith("L")]
      found, nrc = {}, {}
      for i, did in enumerate(dids):
        keep_alive()
        try:
          resp = uds._uds_request(IOC, subfunction=None, data=bytes([did >> 8, did & 0xFF, RETURN_CONTROL]))
          found[f"{did:04x}"] = "ok " + resp.hex()
        except NegativeResponseError as e:
          nrc[e.error_code] = nrc.get(e.error_code, 0) + 1
          if e.error_code != 0x31:
            found[f"{did:04x}"] = f"NRC 0x{e.error_code:02x}"
        except MessageTimeoutError:
          nrc["timeout"] = nrc.get("timeout", 0) + 1
        if i % 4096 == 4095:
          print(f"  {i + 1}/{len(dids)}, candidates {len(found)}", flush=True)
      print(f"probe done: {len(dids)} DIDs, NRC counts {nrc}", flush=True)
      for k, v in found.items():
        print(f"  {k}: {v}", flush=True)
      json.dump(found, open("/data/ars510_meter_ioc_probe.json", "w"), indent=1)
    else:
      did, state, hold = int(sys.argv[2], 16), bytes.fromhex(sys.argv[3]), float(sys.argv[4])
      print(f"before: {meter_pct(p)} %", flush=True)
      try:
        resp = uds._uds_request(IOC, subfunction=None, data=bytes([did >> 8, did & 0xFF, SHORT_TERM_ADJUST]) + state)
        print(f"0x2F {did:04x} 03 {state.hex()} -> ok {resp.hex()}", flush=True)
      except NegativeResponseError as e:
        print(f"0x2F {did:04x} 03 {state.hex()} -> NRC 0x{e.error_code:02x}", flush=True)
      end = time.monotonic() + hold
      while time.monotonic() < end:
        keep_alive()
        print(f"  during: {meter_pct(p, 0.5)} %", flush=True)
      try:
        uds._uds_request(IOC, subfunction=None, data=bytes([did >> 8, did & 0xFF, RETURN_CONTROL]))
        print("returned control to the meter", flush=True)
      except Exception as e:
        print(f"return control: {e}", flush=True)
      print(f"after: {meter_pct(p, 1.0)} %", flush=True)
  finally:
    p.set_safety_mode(CarParams.SafetyModel.silent)
    print("panda back to silent; reboot the device to restart openpilot", flush=True)


if __name__ == "__main__":
  main()
