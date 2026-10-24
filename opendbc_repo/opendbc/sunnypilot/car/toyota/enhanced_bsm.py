"""
Enhanced BSM for Toyota: poll the two blind spot ECUs over UDS (0x750 sub-address 0x41 left / 0x42 right,
responses on 0x758) so the blind spot state updates every 0.2 s instead of the 1 Hz stock BSM broadcast.

Ported from sunnypilot/opendbc#502 (@arne182, @rav4kumar) for dimpleliang/ars510. Differences from the PR:
- the polled state is OR-ed with the stock BSM signal in carstate instead of replacing it
- debug mode is only entered above ENABLE_V and is left again after standing still (the 360° camera stops
  working while the BSM ECUs are in the extended diagnostic session at low speed)
"""

from opendbc.car import structs
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
ENABLE_V = 6.0            # m/s, enter debug mode above this speed
DISABLE_V = 1.0           # m/s, leave debug mode once below this speed for DISABLE_FRAMES
DISABLE_FRAMES = 200      # 2 s
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
    self.slow_frames = 0

  def update(self, CS, frame: int) -> list[CanData]:
    if frame <= START_FRAME:
      return []

    can_sends: list[CanData] = []
    v_ego = CS.out.vEgo

    if not self.debug_enabled:
      if v_ego > ENABLE_V:
        # DiagnosticSessionControl(0x60) once per side; re-sending it while already in the session makes the
        # ECU intermittently drop its detection state (PR 502), so it is sent only on the transition
        can_sends.append(toyotacan.create_set_bsm_debug_mode(LEFT_BLINDSPOT, True))
        can_sends.append(toyotacan.create_set_bsm_debug_mode(RIGHT_BLINDSPOT, True))
        self.debug_enabled = True
        self.slow_frames = 0
      return can_sends

    self.slow_frames = self.slow_frames + 1 if v_ego < DISABLE_V else 0
    if self.slow_frames >= DISABLE_FRAMES:
      can_sends.append(toyotacan.create_set_bsm_debug_mode(LEFT_BLINDSPOT, False))
      can_sends.append(toyotacan.create_set_bsm_debug_mode(RIGHT_BLINDSPOT, False))
      self.debug_enabled = False
      return can_sends

    if frame % POLL_PERIOD == 0:
      can_sends.append(toyotacan.create_bsm_polling_status(LEFT_BLINDSPOT))
    elif frame % POLL_PERIOD == POLL_PERIOD // 2:
      can_sends.append(toyotacan.create_bsm_polling_status(RIGHT_BLINDSPOT))

    return can_sends
