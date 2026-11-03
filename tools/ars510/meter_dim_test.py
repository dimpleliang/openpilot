#!/usr/bin/env python3
"""Parked experiment (11/03): does the combination meter follow 0x620 BODY_CONTROL_STATE.METER_DIMMED sent from the comma?

Logs (437 segments): lights off -> 0x610 METER_SLIDER_BRIGHTNESS_PCT 100; lights on with 0x620 METER_DIMMED = 1 -> the
slider level (17 %, darkest); lights on with METER_DIMMED = 0 (tunnel at noon) -> 66 %. So METER_DIMMED, from the body ECU, is
what makes the meter use the dark night level. 0x620 also carries doors / seat belt / parking brake, so the test copies the
latest real frame and clears only that bit (byte 4, 0x40), no checksum on this message.

The comma sits on the front-camera bus; the meter is behind the gateway (TBT test notes), so the frame may never reach it.
That is what this test answers. Nothing else is sent; ~TEST_S seconds; then the panda goes back to silent.

Run only parked: P, READY on, lights on, at night (0x620 METER_DIMMED = 1 before the test). openpilot must be stopped first
(the panda has one owner):   sudo systemctl stop comma
  PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo /usr/local/venv/bin/python3 tools/ars510/meter_dim_test.py --parked
Afterwards:                  sudo reboot        (brings openpilot back)
Watch the meter: does it brighten during the test phase, flicker every few seconds (the real frame), or show warnings?
"""
import sys
import time

from opendbc.car.structs import CarParams
from panda import Panda

BUS = 0
ADDR_BODY, ADDR_SLIDER, ADDR_GEAR = 0x620, 0x610, 0x3BC
DIM_BYTE, DIM_MASK = 4, 0x40          # METER_DIMMED 38|1 (Motorola) = byte 4 bit 6
OBSERVE_S, TEST_S, AFTER_S = 8.0, 15.0, 10.0
SEND_PERIOD_S = 0.1                   # our copy, 10 Hz, plus one right after every real frame
ALLOUTPUT_PASSTHROUGH = 1             # relay open, bus 0 <-> 2 forwarded: the camera keeps talking to the car


def be(d, sb, n):
  v, bit = 0, sb
  for _ in range(n):
    v = (v << 1) | ((d[bit // 8] >> (bit % 8)) & 1)
    bit = bit + 15 if bit % 8 == 0 else bit - 1
  return v


def offroad() -> bool:
  try:
    return open("/data/params/d/IsOffroad").read().strip() == "1"
  except OSError:
    return False


def main():
  # the meter needs the car ON (READY), so IsOffroad is 0; --parked = the caller checked carState (P, 0 km/h) right before
  # stopping openpilot
  if not (offroad() or "--parked" in sys.argv):
    sys.exit("not confirmed parked: check carState (gear P, vEgo 0) first, then run with --parked")
  try:
    p = Panda()
  except Exception as e:
    sys.exit(f"no panda ({e}): is openpilot still running? sudo systemctl stop comma")
  p.set_safety_mode(CarParams.SafetyModel.allOutput, ALLOUTPUT_PASSTHROUGH)
  time.sleep(0.2)
  mode = p.health()["safety_mode"]
  if mode != CarParams.SafetyModel.allOutput:
    p.set_safety_mode(CarParams.SafetyModel.silent)
    sys.exit(f"panda refused allOutput (safety mode {mode}): firmware without ALLOW_DEBUG, test not possible this way")

  last_body = None
  t0 = time.monotonic()
  next_send = 0.
  sent = 0
  phase = "observe"
  print(f"{OBSERVE_S:.0f} s observe, {TEST_S:.0f} s send METER_DIMMED=0, {AFTER_S:.0f} s observe")
  try:
    while True:
      now = time.monotonic() - t0
      phase = "observe" if now < OBSERVE_S else "TEST" if now < OBSERVE_S + TEST_S else "after" if now < OBSERVE_S + TEST_S + AFTER_S else None
      if phase is None:
        break
      real_now = False
      for addr, dat, bus in p.can_recv():
        if bus != BUS:
          continue
        if addr == ADDR_BODY and len(dat) == 8:
          last_body = bytearray(dat)
          real_now = True
          print(f"{now:6.2f} {phase:7s} real 0x620 {bytes(dat).hex()} METER_DIMMED={be(dat, 38, 1)}")
        elif addr == ADDR_SLIDER and len(dat) == 8:
          print(f"{now:6.2f} {phase:7s} 0x610 meter brightness {be(dat, 30, 7)} %  low={be(dat, 37, 1)} dimmed={be(dat, 38, 1)}")
      if phase == "TEST" and last_body is not None and (real_now or now >= next_send):
        out = bytearray(last_body)
        out[DIM_BYTE] &= ~DIM_MASK & 0xFF
        p.can_send(ADDR_BODY, bytes(out), BUS)
        sent += 1
        next_send = now + SEND_PERIOD_S
      time.sleep(0.005)
  finally:
    p.set_safety_mode(CarParams.SafetyModel.silent)
    print(f"done: {sent} frames sent, panda back to silent. Reboot the device to restart openpilot (sudo reboot).")


if __name__ == "__main__":
  main()
