"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Automatic door locking and unlocking (sunnypilot/opendbc#354 by @rav4kumar, from @dragonpilot-community;
thanks to AlexandreSato & cydia2020, https://github.com/AlexandreSato/animalpilot/blob/personal/doors.py).
dimpleliang/ars510: ported to this fork (flags renumbered, enabled by DOOR_LOCK_FILE, see sunnypilot/car/interfaces.py).
"""

from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

GearShifter = structs.CarState.GearShifter

# 0x750 sub-address 0x40 (main body ECU). The dragonpilot frames (KWP IO control 0x30 / 0x11, 0x40 unlock, 0x80 lock)
# are refused by the 2023 RAV4 HEV body ECU (7F 30 11: service not supported). That ECU takes the GTS Active Tests
# instead (10/28, tested on the car): extended session, then UDS InputOutputControlByIdentifier 0x2F on DID 0x2810
# "Door Lock" / 0x2811 "Door Unlock", shortTermAdjustment 0x03, value 0x01, control-enable mask 0x80; returnControlToECU
# 0x00 afterwards. Each step is one single frame.
SESSION_CMD = b'\x40\x02\x10\x03\x00\x00\x00\x00'
LOCK_CMD = b'\x40\x06\x2f\x28\x10\x03\x01\x80'
UNLOCK_CMD = b'\x40\x06\x2f\x28\x11\x03\x01\x80'
LOCK_RETURN_CMD = b'\x40\x05\x2f\x28\x10\x00\x80\x00'
UNLOCK_RETURN_CMD = b'\x40\x05\x2f\x28\x11\x00\x80\x00'
SEQ_STEP_FRAMES = 5     # 50 ms between session and command
SEQ_RETURN_FRAMES = 100  # 1 s later: give control back
LOCK_AT_SPEED = 10 * CV.KPH_TO_MS
# 10/31 13:40, twice: in park the unlock sequence was sent 10 ms after the driver's door opened and the body ECU answered
# positively (6F 28 11), but the other doors stayed locked; the same sequence with all doors closed (ignition off test,
# 10/31) unlocked all four. So in park the unlock is also sent when the driver's seatbelt is unlatched (normally a few
# seconds before the door opens). One unlock per stop: whichever comes first, the seatbelt or the door; never repeated.


class AutoLockController:
  def __init__(self, CP_SP):
    self.auto_lock_enabled = bool(CP_SP.flags & ToyotaFlagsSP.AUTO_LOCK.value)
    self.auto_unlock_enabled = False  # 10/31 driver: unlock removed, whatever the flag
    self.last_gear = GearShifter.park
    self.last_door_open = False
    self.last_driver_door_open = True  # no edge at start-up: only a real change triggers
    self.lock_once = False
    self.last_belt_unlatched = True
    self.unlocked_in_park = False   # the seatbelt unlock fires once per stop in park
    self.pending: list[tuple[int, bytes]] = []  # (frames until send, data)

  def _queue(self, cmd: bytes, return_cmd: bytes):
    self.pending = [(0, SESSION_CMD), (SEQ_STEP_FRAMES, cmd), (SEQ_STEP_FRAMES + SEQ_RETURN_FRAMES, return_cmd)]

  @property
  def enabled(self) -> bool:
    return self.auto_lock_enabled or self.auto_unlock_enabled

  def update(self, CS) -> list[CanData]:
    can_sends: list[CanData] = []
    if not self.enabled:
      return can_sends

    current_gear = CS.out.gearShifter
    driver_door_open = bool(getattr(CS, "driver_door_open", False))
    belt_unlatched = bool(getattr(CS.out, "seatbeltUnlatched", False))
    if current_gear == GearShifter.park:
      self.lock_once = False  # re-arm the lock for the next drive
    # 10/31 driver: auto unlock removed (commands kept above and documented in ARS510_RADAR.md); lock only

    if self.auto_lock_enabled and current_gear != GearShifter.park:
      # auto lock once per drive: in drive, doors closed, above LOCK_AT_SPEED
      if current_gear == GearShifter.drive and not CS.out.doorOpen and not self.lock_once and CS.out.vEgo >= LOCK_AT_SPEED:
        self._queue(LOCK_CMD, LOCK_RETURN_CMD)
        self.lock_once = True

    self.last_gear = current_gear
    self.last_door_open = CS.out.doorOpen
    self.last_driver_door_open = driver_door_open
    self.last_belt_unlatched = belt_unlatched
    if self.pending:
      due = [d for f, d in self.pending if f <= 0]
      self.pending = [(f - 1, d) for f, d in self.pending if f > 0]
      can_sends.extend(CanData(0x750, d, 0) for d in due)
    return can_sends
