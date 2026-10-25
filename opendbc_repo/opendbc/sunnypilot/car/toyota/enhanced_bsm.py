"""
Enhanced BSM for Toyota: poll the two blind spot ECUs over UDS (0x750 sub-address 0x41 left / 0x42 right,
responses on 0x758) so the blind spot state updates every 0.2 s instead of the 1 Hz stock BSM broadcast.

Ported from sunnypilot/opendbc#502 (@arne182, @rav4kumar) for dimpleliang/ars510. Differences from the PR:
- the polled state is OR-ed with the stock BSM signal in carstate instead of replacing it
- debug mode is entered at any speed whenever the car is out of reverse / park (PR 502 only enters it above 6 m/s
  because the stock Toyota 360° camera stops working if it is switched on at low speed; this car's 360° camera is
  aftermarket, so that does not apply) and is kept through standstill, so the blind spot state keeps updating while
  stopped (the stock 1 Hz broadcast mostly reports nothing below ~16 km/h). It is left in reverse or park, where the
  stock rear cross traffic alert works, as before.
"""

from opendbc.car import structs
from opendbc.car.structs import CarState
from opendbc.car.can_definitions import CanData
from opendbc.car.toyota import toyotacan
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

LEFT_BLINDSPOT = b"\x41"
RIGHT_BLINDSPOT = b"\x42"
LEFT_SIDE = LEFT_BLINDSPOT[0]
RIGHT_SIDE = RIGHT_BLINDSPOT[0]

# BLINDSPOTD1/D2 aren't real distances despite the DBC name (PR 502): idle 0, a transitional noise band
# (1-31) and two real zone codes (~46-47 and ~50-53). Gate on a floor above the noise band.
BLINDSPOT_NOISE_FLOOR = 35

POLL_PERIOD = 20          # frames (100 Hz) between polls of one side -> 5 Hz per side, 10 UDS requests/s in total
SILENCE_TIMEOUT = 100     # frames without a response from a side before its state is cleared
GEAR_FRAMES = 20          # 0.2 s in / out of reverse or park before leaving / entering debug mode
START_FRAME = 200         # let the car settle after start before the first diagnostic request


class EnhancedBsm:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

  @property
  def enabled(self):
    return bool(self.CP_SP.flags & ToyotaFlagsSP.SP_ENHANCED_BSM)


class _BsmSideState:
  def __init__(self):
    self.blindspot = False
    self.counter = 0

  def update(self, distance_1, distance_2):
    # every fresh reading reflects the current occupancy; the counter is only a silence timeout
    self.blindspot = distance_1 > BLINDSPOT_NOISE_FLOOR or distance_2 > BLINDSPOT_NOISE_FLOOR
    self.counter = SILENCE_TIMEOUT

  def decay(self):
    self.counter = max(0, self.counter - 1)
    if self.counter == 0:
      self.blindspot = False


class EnhancedBsmCarState(EnhancedBsm):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self._sides = {LEFT_SIDE: _BsmSideState(), RIGHT_SIDE: _BsmSideState()}
    self._last_ts = 0

  def update(self, cp) -> tuple[bool, bool]:
    vl = cp.vl["DEBUG"]  # registers the message on first access (no frequency check)
    ts = cp.ts_nanos["DEBUG"]["BLINDSPOTSIDE"]
    if ts != self._last_ts:  # a new 0x758 frame arrived since the last call
      self._last_ts = ts
      side = int(vl["BLINDSPOTSIDE"])
      if side in self._sides:
        self._sides[side].update(vl["BLINDSPOTD1"], vl["BLINDSPOTD2"])

    for side_state in self._sides.values():
      side_state.decay()

    return self._sides[LEFT_SIDE].blindspot, self._sides[RIGHT_SIDE].blindspot


class EnhancedBsmCarController(EnhancedBsm):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.debug_enabled = False
    self.gear_frames = 0  # consecutive frames the gear has disagreed with the current debug mode state

  def update(self, CS, frame: int) -> list[CanData]:
    if frame <= START_FRAME:
      return []

    can_sends: list[CanData] = []
    want = CS.out.gearShifter not in (CarState.GearShifter.reverse, CarState.GearShifter.park)
    self.gear_frames = self.gear_frames + 1 if want != self.debug_enabled else 0

    if self.gear_frames >= GEAR_FRAMES:
      # DiagnosticSessionControl 0x60 (enter) / 0x01 (leave) once per side on the transition only: re-sending it
      # while already in the session makes the ECU intermittently drop its detection state (PR 502)
      can_sends.append(toyotacan.create_set_bsm_debug_mode(LEFT_BLINDSPOT, want))
      can_sends.append(toyotacan.create_set_bsm_debug_mode(RIGHT_BLINDSPOT, want))
      self.debug_enabled = want
      self.gear_frames = 0
      return can_sends

    if not self.debug_enabled:
      return can_sends

    if frame % POLL_PERIOD == 0:
      can_sends.append(toyotacan.create_bsm_polling_status(LEFT_BLINDSPOT))
    elif frame % POLL_PERIOD == POLL_PERIOD // 2:
      can_sends.append(toyotacan.create_bsm_polling_status(RIGHT_BLINDSPOT))

    return can_sends
