"""ars510: is my own car changing lanes? (11/01 driver: merging into another lane showed cut-in markers)

While I change lanes, the cars in the lane I move into slide toward my path on the radar and look like cut-ins
(display circle, confirmed lift-off). Active while a blinker is on, the model's lane-change state is not off, or my
car is within NEAR_LINE_M of a lane line (lane line at x = 0, probability >= MIN_PROB), and HOLD_S after.
Replay of 124 km (tools/ars510/merge_eval_lanes.py, REAL_NO_EGO_LC=1): 40 % of what the replay counted as real cut-ins
were my own lane changes; with this gate the predicted-marker false marks per 100 km 117/89/42 -> 68/75/35
(<60 / 60-90 / 90+ km/h) at the same recall (20/34/30 %).
"""
import numpy as np

NEAR_LINE_M = 0.7
MIN_PROB = 0.5
HOLD_S = 3.0


class EgoLaneChange:
  def __init__(self):
    self.last_t = -99.

  def update(self, car_state, model, now: float) -> bool:
    try:
      active = bool(car_state.leftBlinker or car_state.rightBlinker)
      if not active and int(model.meta.laneChangeState) != 0:
        active = True
      if not active and len(model.laneLines) >= 3 and len(model.laneLineProbs) >= 3:
        for i, sign in ((1, -1.), (2, 1.)):
          ll = model.laneLines[i]
          if model.laneLineProbs[i] >= MIN_PROB and len(ll.x) > 1 and \
             float(np.interp(0., ll.x, ll.y)) * sign < NEAR_LINE_M:
            active = True
    except Exception:
      active = False
    if active:
      self.last_t = now
    return now - self.last_t < HOLD_S
