#!/usr/bin/env python3
"""ars510 unlockd: offroad only (manager), started by manager when the ignition goes off.

The driver switches the car off and then opens the door; card is gone by then, so the "open the driver's door in P ->
unlock all doors" rule in opendbc/sunnypilot/car/toyota/auto_lock_unlock.py never fires. pandad keeps the CAN
transceivers on and the car's safety mode for OFFROAD_UNLOCK_WINDOW_S after ignition off (pandad.cc), and this process
watches BODY_CONTROL_STATE (0x620) for the driver's door and sends the same body-ECU active test through sendcan.
Opt-in with the rest of the door feature: /data/ars510_door_lock containing "unlock" (or empty). One unlock per power-off.

Tested 10/31 on the 2023 RAV4 HEV: after power off the bus stays awake for 1-2 minutes, 0x620 still reports the doors,
and 0x750/0x40 2F 2811 unlocks all four doors (positive response 0x758 6F 28 11).
"""
import time

from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.interfaces import _door_lock_mode
from opendbc.sunnypilot.car.toyota.auto_lock_unlock import SESSION_CMD, UNLOCK_CMD, UNLOCK_RETURN_CMD
from openpilot.cereal import messaging
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp

WINDOW_S = 85.            # a little under pandad's OFFROAD_UNLOCK_WINDOW_S
BODY_CONTROL_STATE = 0x620
DOOR_OPEN_FL_BYTE, DOOR_OPEN_FL_MASK = 5, 0x20  # toyota DBC: DOOR_OPEN_FL bit 45 (big endian)
UNLOCK_RESP = b"\x40\x05\x6f\x28\x11"            # body ECU positive response to UNLOCK_CMD
RETRIES = 3


def send(pm, data: bytes):
  pm.send("sendcan", can_list_to_can_capnp([CanData(0x750, data, 0)], msgtype="sendcan", valid=True))


def drain(sm, state):
  """Update door state from BODY_CONTROL_STATE and count the body ECU's positive unlock responses (0x758 40 05 6F 28 11)."""
  for msg in messaging.drain_sock(sm):
    for c in msg.can:
      state["n"] += 1
      if c.src != 0:
        continue
      if c.address == BODY_CONTROL_STATE and len(c.dat) > DOOR_OPEN_FL_BYTE:
        state["door"] = bool(c.dat[DOOR_OPEN_FL_BYTE] & DOOR_OPEN_FL_MASK)
      elif c.address == 0x758 and bytes(c.dat[:5]) == UNLOCK_RESP:
        state["ack"] += 1


def main():
  if "unlock" not in _door_lock_mode():
    while True:  # disabled: idle (an exit would make manager restart us)
      time.sleep(3600)
  sm = messaging.sub_sock("can", timeout=100)
  pm = messaging.PubMaster(["sendcan"])
  t0 = time.monotonic()
  st = {"door": None, "n": 0, "ack": 0}
  while time.monotonic() - t0 < WINDOW_S:
    drain(sm, st)
    if st["door"]:
      # the door is open (opened just now, or already open when we started): unlock the rest, same sequence as card.
      # Retry when the body ECU does not answer: right after ignition off the panda can stall or drop a send (10/31).
      t_door = time.monotonic() - t0
      for attempt in range(1, RETRIES + 1):
        send(pm, SESSION_CMD)
        time.sleep(0.05)
        send(pm, UNLOCK_CMD)
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.0:
          drain(sm, st)
        send(pm, UNLOCK_RETURN_CMD)
        if st["ack"]:
          break
        time.sleep(0.5)
      cloudlog.warning(f"ARS510 offroad unlock: door opened {t_door:.1f}s after ignition off, {attempt} attempt(s), "
                       f"body ECU ack={st['ack']}, can msgs={st['n']}")
      break
  else:
    cloudlog.info(f"ARS510 offroad unlock window closed, door never opened (can msgs: {st['n']})")
  # stay alive until manager stops us at the next ignition on: exiting would make manager restart us and unlock again
  while True:
    time.sleep(60)

if __name__ == "__main__":
  main()
