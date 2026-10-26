"""
dimpleliang/ars510: cluster turn-by-turn (TBT) bench test.

Sends AVN1S13 (0x3A5, the head unit's turn-by-turn / HUD guidance message from toyota_2017_ref_pt.dbc) on the car bus
to see whether the 12.3" meter shows a TBT pop-up when it receives it. Only active while the flag file exists on the
device, the car is at standstill in park and openpilot is not engaged. Off by default; create / remove the flag over
SSH:  touch /data/ars510_tbt_test   /   rm /data/ars510_tbt_test

The pattern cycles every 40 s through four 10 s phases so the person watching the meter can tell which values work:
  A: right (90 deg),  DT_GP 30, DT_UNIT 1
  B: left (270 deg),  DT_GP 30, DT_UNIT 2
  C: U-turn (180 deg), DT_GP 30, DT_UNIT 3
  D: straight (0),    DT_GP 30, DT_UNIT 0
The phase in effect is written to /data/ars510_tbt_test.phase.
"""
import os

from opendbc.car import structs
from opendbc.car.can_definitions import CanData

FLAG_FILE = "/data/ars510_tbt_test"
PHASE_FILE = "/data/ars510_tbt_test.phase"
CHECK_PERIOD = 100   # frames (1 s) between flag file checks
SEND_PERIOD = 20     # frames: 5 Hz
PHASE_FRAMES = 1000  # 10 s

PHASES = [
  ("A right 90 / unit 1", 90.0, 1),
  ("B left 270 / unit 2", 270.0, 2),
  ("C u-turn 180 / unit 3", 180.0, 3),
  ("D straight 0 / unit 0", 0.0, 0),
]


class TbtTest:
  def __init__(self):
    self.enabled = False
    self.start_frame = 0
    self.last_phase = None

  def update(self, packer, CC: structs.CarControl, CS, frame: int) -> list[CanData]:
    if frame % CHECK_PERIOD == 0:
      enabled = os.path.exists(FLAG_FILE)
      if enabled and not self.enabled:
        self.start_frame = frame
      self.enabled = enabled

    parked = CS.out.standstill and CS.out.gearShifter == structs.CarState.GearShifter.park
    if not self.enabled or CC.enabled or not parked or frame % SEND_PERIOD != 0:
      return []

    idx = ((frame - self.start_frame) // PHASE_FRAMES) % len(PHASES)
    name, dir_turn, dt_unit = PHASES[idx]
    if idx != self.last_phase:
      self.last_phase = idx
      try:
        with open(PHASE_FILE, "w") as f:
          f.write(name + "\n")
      except OSError:
        pass

    values = {
      "HUD_DISP": 1,
      "DRVSIDE": 0,
      "DT_GP": 30,
      "DT_UNIT": dt_unit,
      "DIR_TURN": dir_turn,
      "ROTARY": 0,
      "ROAD_TP1": 0,
      "ROAD_TP2": 0,
      "N_H_UP": 0,
      "COMPASS": 0,
      "DIR": 0,
    }
    return [packer.make_can_msg("AVN1S13", 0, values)]
