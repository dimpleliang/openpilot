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

# 0x750 sub-address 0x40 (main body ECU): IO control 0x30 / 0x11, 0x40 = unlock, 0x80 = lock
UNLOCK_CMD = b'\x40\x05\x30\x11\x00\x40\x00\x00'
LOCK_CMD = b'\x40\x05\x30\x11\x00\x80\x00\x00'
LOCK_AT_SPEED = 10 * CV.KPH_TO_MS


class AutoLockController:
  def __init__(self, CP_SP):
    self.auto_lock_enabled = bool(CP_SP.flags & ToyotaFlagsSP.AUTO_LOCK.value)
    self.auto_unlock_enabled = bool(CP_SP.flags & ToyotaFlagsSP.AUTO_UNLOCK.value)
    self.last_gear = GearShifter.park
    self.lock_once = False

  @property
  def enabled(self) -> bool:
    return self.auto_lock_enabled or self.auto_unlock_enabled

  def update(self, CS) -> list[CanData]:
    can_sends: list[CanData] = []
    if not self.enabled:
      return can_sends

    current_gear = CS.out.gearShifter
    if self.last_gear != current_gear and current_gear == GearShifter.park:
      # auto unlock when shifting to park
      if self.auto_unlock_enabled:
        can_sends.append(CanData(0x750, UNLOCK_CMD, 0))
      self.lock_once = False

    elif self.auto_lock_enabled:
      # auto lock once per drive: in drive, doors closed, above LOCK_AT_SPEED
      if current_gear == GearShifter.drive and not CS.out.doorOpen and not self.lock_once and CS.out.vEgo >= LOCK_AT_SPEED:
        can_sends.append(CanData(0x750, LOCK_CMD, 0))
        self.lock_once = True

    self.last_gear = current_gear
    return can_sends
