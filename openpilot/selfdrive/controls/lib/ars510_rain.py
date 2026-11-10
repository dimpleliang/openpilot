"""ars510 rain mode (11/10): the wipers tell the planner it is raining, so it keeps more distance and accelerates less.

11/09 18:12 (downhill, rain, wipers AUTO): the ARS510 object list collapsed to about one object and 33 m median range
without any fault flag, the radar saw a stopped queue only at 71 m and the car braked to -2.2 m/s^2; the camera had the
slowing lead at 80-100 m. A wet road also needs a longer stop. A driver simply drives with more margin in rain.

Input: carStateSP.wiperLevel (Toyota 0x2F9: 0 off or AUTO between wipes, 1 INT / AUTO wiping, 2 LO, 3 HI) and
wiperMist (a single pull; wiping for MIST_IGNORE_S after it is ignored). In AUTO the level drops to 0 between wipes, so the mode goes by the
share of the last WINDOW_S the wipers were wiping, not by the instantaneous level:

  0 dry    -> 1 light : wiping share > ON_SHARE
  any      -> 2 heavy : LO / HI for HEAVY_ON_S without a break
  2 heavy  -> 1 light : no LO / HI for HEAVY_OFF_S
  1 light  -> 0 dry   : wiping share < OFF_SHARE for OFF_HOLD_S

Effects (driver's numbers, 11/10), all reached at a slow ramp so nothing steps:
  following time      + T_FOLLOW_EXTRA  (0 / 0.3 / 0.5 s)   -> the MPC and every ars510 rule that uses t_follow
                                                             (far-lead ease-off, cut-in response) start earlier
  acceleration        x ACCEL_SCALE     (1 / 0.8 / 0.7)     -> a_pos ceiling and e2e fill
  camera-only far lead ease-off cap      VISION_MAX_DECEL    -> ars510_long_rules far_early (-0.4 dry)
far_soft (lighter braking for a far lead) is NOT switched off in rain: firmer braking on a wet road is the wrong way;
the longer following time already makes everything start earlier (driver, 11/10).

Off: touch /data/ars510_no_rain_mode. cloudlog "ARS510 rain mode a -> b".
"""
import os
from collections import deque

from openpilot.common.swaglog import cloudlog

OFF_FILE = "/data/ars510_no_rain_mode"
WINDOW_S = 60.
ON_SHARE = 0.30
OFF_SHARE = 0.05
OFF_HOLD_S = 120.
HEAVY_ON_S = 10.         # a mist pull / washer cycle wipes at LO for about 4 s (11/03 capture): not rain
MIST_IGNORE_S = 8.       # wiping within this long after the mist flag is not counted at all
HEAVY_OFF_S = 60.
T_FOLLOW_EXTRA = (0., 0.3, 0.5)          # s
ACCEL_SCALE = (1., 0.8, 0.7)
VISION_MAX_DECEL = (-0.4, -0.5, -0.6)    # m/s^2, [0] must equal ars510_long_rules.FAR_EARLY_MAX_DECEL_VISION
T_FOLLOW_RATE = 0.1                      # s of following time per second
ACCEL_SCALE_RATE = 0.05                  # per second


class RainMode:
  def __init__(self, dt):
    self.dt = dt
    self.off = os.path.exists(OFF_FILE)
    self.win = deque(maxlen=int(WINDOW_S / dt))
    self.wiping_n = 0
    self.level = 0
    self.heavy_t = 0.        # s of uninterrupted LO / HI
    self.no_heavy_t = 0.     # s since the last LO / HI
    self.dry_t = 0.          # s the wiping share has been below OFF_SHARE
    self.mist_t = 0.         # s left of ignoring the wipers after a mist pull
    self.t_follow_extra = 0.
    self.accel_scale = 1.
    self.vision_max_decel = VISION_MAX_DECEL[0]

  def poll(self):
    self.off = os.path.exists(OFF_FILE)

  @property
  def share(self) -> float:
    return self.wiping_n / len(self.win) if self.win else 0.

  def update(self, wiper_level: int, wiper_mist: bool) -> int:
    """Once per planner cycle. Returns the level (0 dry, 1 light, 2 heavy)."""
    self.mist_t = MIST_IGNORE_S if wiper_mist else max(self.mist_t - self.dt, 0.)
    if self.mist_t > 0.:
      wiper_level = 0
    wiping = wiper_level >= 1
    if len(self.win) == self.win.maxlen:
      self.wiping_n -= self.win[0]
    self.win.append(1 if wiping else 0)
    self.wiping_n += 1 if wiping else 0
    heavy_now = wiper_level >= 2
    self.heavy_t = self.heavy_t + self.dt if heavy_now else 0.
    self.no_heavy_t = 0. if heavy_now else self.no_heavy_t + self.dt
    share = self.share
    self.dry_t = self.dry_t + self.dt if share < OFF_SHARE else 0.

    level = self.level
    if self.off:
      level = 0
    elif self.heavy_t >= HEAVY_ON_S:
      level = 2
    elif level == 2 and self.no_heavy_t >= HEAVY_OFF_S:
      level = 1
    elif level == 0 and share > ON_SHARE and len(self.win) * self.dt >= WINDOW_S * ON_SHARE:
      level = 1
    elif level == 1 and self.dry_t >= OFF_HOLD_S:
      level = 0
    if level != self.level:
      cloudlog.info(f"ARS510 rain mode {self.level} -> {level} wiper={wiper_level} share={share:.2f}")
      self.level = level

    # slow ramps: no step in the following distance or the acceleration ceiling
    step = T_FOLLOW_RATE * self.dt
    self.t_follow_extra += min(max(T_FOLLOW_EXTRA[level] - self.t_follow_extra, -step), step)
    step = ACCEL_SCALE_RATE * self.dt
    self.accel_scale += min(max(ACCEL_SCALE[level] - self.accel_scale, -step), step)
    self.vision_max_decel = VISION_MAX_DECEL[level]
    return level
