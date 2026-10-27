from collections import deque

import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# dimpleliang/ars510: soft stop. While the car is still rolling in the stopping state, ease the brake request toward
# SOFT_STOP_ACCEL instead of ramping it to stopAccel, so there is little deceleration left at the moment the wheels stop
# (IMU on 10/25: -0.2..-0.66 m/s^2 at the stop, then a +0.3 m/s^2 rebound, jerk 8-13 m/s^3). Once the car has
# (almost) stopped, ramp to stopAccel as before to hold it; adding brake at standstill does not pitch the car.
SOFT_STOP_V = 0.05             # m/s, below this (or at standstill) ramp to stopAccel
SOFT_STOP_MAX_V = 1.0          # m/s, only soften the last metre: if stopping is requested faster (e.g. model shouldStop), brake as before
SOFT_STOP_ACCEL = -0.3         # m/s^2, brake request to ease toward while still rolling
SOFT_STOP_RELEASE_RATE = 1.5   # m/s^3, how fast the brake is eased off
# The soft request is only kept while the car is really slowing down. At crawl speed the Toyota PCM barely brakes for
# small requests (idle creep: 10/27 route 4f, -0.2..-0.35 commanded gave 0..-0.1 m/s^2 for 1-3 s), so a fixed soft
# request can leave the car creeping. While rolling and not slowing, brake harder at the stock stopping rate instead.
SOFT_STOP_SLOWING_A = -0.1     # m/s^2, aEgo above this while rolling = not slowing
STOPPING_DECEL_RATE = 1.0      # m/s^3, stock rate toward stopAccel
# Toyota wheel speed reads 0 while the car still creeps at ~0.2-0.3 m/s (10/27 IMU: wheel speed 0.22 -> 0.03 m/s in
# 0.2 s, but the body kept decelerating for another ~0.7 s and the real stop came with -0.4 m/s^2 and a +0.2 jolt,
# because the ramp to stopAccel had already started). After the reading drops to standstill, hold the soft request for
# the time the car still needs to stop (last rolling speed / |SOFT_STOP_ACCEL| + margin), then ramp to stopAccel.
# Only once per stop: if the wheels report rolling again, the closed-loop rolling branch takes over.
SOFT_STOP_SETTLE_MARGIN = 0.2       # s
SOFT_STOP_SETTLE_RANGE = (0.3, 1.2) # s, min / max hold
# Creep stall: the planner only enters the stopping state below 0.3 m/s. If the PCM does not follow a small decel
# request, the car can hang at 0.3-0.6 m/s creeping toward the lead (10/27 15:04: 0.34-0.46 m/s for 3 s, gap 2.9 ->
# 1.6 m, driver had to brake). If the planner keeps asking to slow down but the speed has dropped less than
# STALL_DROP over the last STALL_WINDOW, treat it as a stop request until the planner wants to accelerate again.
STALL_MAX_V = 0.6              # m/s
STALL_A_TARGET = -0.05         # m/s^2, planner must ask for at least this much decel during the whole window
STALL_WINDOW = 1.0             # s
STALL_DROP = 0.1               # m/s
STALL_CLEAR_A_TARGET = 0.1     # m/s^2, same threshold as should_stop: the planner wants to move again
# True stop detection (when locationd's calibrated pose is valid; otherwise the timer-based settle hold above).
# 10/27 route 52 IMU: Toyota wheel speed stops updating at ~0.2 m/s, but the car kept rolling 0.45-1.3 s. In one stop
# the settle timer (0.75 s) ran out while the car was still rolling at ~0.1 m/s; the ramp to stopAccel then bit right
# at the real stop (-0.4 m/s^2, +0.23 rebound). Below SOFT_STOP_WHEEL_VALID_V the true speed is integrated from the
# pose acceleration (gravity compensated) and the request is held until the car has really stopped, then ramped.
# If the car is not slowing at all while held (creep), brake is added; never longer than SOFT_STOP_MAX_COAST.
SOFT_STOP_WHEEL_VALID_V = 0.22  # m/s, Toyota wheel speed below this is a decay artifact
SOFT_STOP_TRUE_STOP_V = 0.02    # m/s
SOFT_STOP_POSE_TAU = 0.2        # s, filter on the pose acceleration
SOFT_STOP_COAST_MIN_DECEL = 0.03  # m/s^2, filtered decel below this while held = not slowing: add brake
SOFT_STOP_MAX_COAST = 2.5       # s

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP_SP, active, long_control_state,
                             should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_SP.enableGasInterceptor

  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if should_stop:
        long_control_state = LongCtrlState.stopping

  return long_control_state

class LongControl:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.prev_v_ego = 0.0
    self.settle_frames = 0       # frames left to hold the soft request after the speed reading drops to standstill
    self.settle_used = False     # the settle hold runs once per stop
    self.was_rolling = False
    self.stall_v_hist = deque(maxlen=int(round(STALL_WINDOW / DT_CTRL)) + 1)
    self.stall_stop = False
    # pose-based soft stop
    self.v_est = 0.0
    self.coast_frames = 0
    self.pose_a_f = 0.0
    self.coast_active = False
    self.true_stopped = False

  def reset(self):
    self.pid.reset()

  def ramp_to_stop(self, output_accel):
    # stock stopping ramp: m/s^2/s toward stopAccel, never releasing a harder request
    # TODO: can we just go straight to stopAccel?
    if output_accel > self.CP.stopAccel:
      output_accel = max(output_accel - STOPPING_DECEL_RATE * DT_CTRL, self.CP.stopAccel)
    return output_accel

  def update_stall(self, active, CS, a_target):
    """Latch a stop request when the car hangs at crawl speed although the planner keeps asking to slow down."""
    if not active or CS.brakePressed or CS.gasPressed or a_target >= STALL_CLEAR_A_TARGET:
      self.stall_stop = False
    crawling = (active and self.long_control_state == LongCtrlState.pid and not CS.standstill and
                SOFT_STOP_V < CS.vEgo < STALL_MAX_V and a_target < STALL_A_TARGET)
    if not crawling:
      self.stall_v_hist.clear()
      return
    self.stall_v_hist.append(CS.vEgo)
    if len(self.stall_v_hist) == self.stall_v_hist.maxlen and self.stall_v_hist[0] - CS.vEgo < STALL_DROP:
      self.stall_stop = True
      self.stall_v_hist.clear()

  def update_v_est(self, CS, pose_accel):
    """True speed: wheel speed while it is valid, below that integrate the pose acceleration."""
    if pose_accel is None or CS.vEgoRaw >= SOFT_STOP_WHEEL_VALID_V:
      self.v_est = CS.vEgoRaw
      self.coast_frames = 0
    else:
      self.v_est = max(0.0, self.v_est + pose_accel * DT_CTRL)
      self.coast_frames += 1

  def coast_to_true_stop(self, output_accel, pose_accel):
    """Wheel speed reads standstill: hold the request until the pose says the car has really stopped."""
    if not self.coast_active:
      self.pose_a_f = pose_accel
      self.coast_active = True
    self.pose_a_f += (pose_accel - self.pose_a_f) * DT_CTRL / SOFT_STOP_POSE_TAU
    if self.true_stopped or self.v_est <= SOFT_STOP_TRUE_STOP_V or self.coast_frames * DT_CTRL >= SOFT_STOP_MAX_COAST:
      self.true_stopped = True
      return self.ramp_to_stop(output_accel)
    if -self.pose_a_f < SOFT_STOP_COAST_MIN_DECEL:
      # not slowing (idle creep): add brake at the stock rate until it does
      return self.ramp_to_stop(output_accel)
    return output_accel

  def update(self, active, CS, a_target, should_stop, accel_limits, pose_accel=None):
    """Update longitudinal control. This updates the state machine and runs a PID loop.
    pose_accel: calibrated longitudinal acceleration from locationd (m/s^2), None if not valid."""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]
    self.update_v_est(CS, pose_accel)

    self.update_stall(active, CS, a_target)
    self.long_control_state = long_control_state_trans(self.CP_SP, active, self.long_control_state,
                                                       should_stop or self.stall_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = min(self.last_output_accel, 0.0)
      rolling = SOFT_STOP_V < CS.vEgo < SOFT_STOP_MAX_V and not CS.standstill
      if rolling:
        if CS.aEgo < SOFT_STOP_SLOWING_A:
          # slowing down: ease the request toward SOFT_STOP_ACCEL (or brake gently toward it if coming from above)
          output_accel += float(np.clip(SOFT_STOP_ACCEL - output_accel, -STOPPING_DECEL_RATE * DT_CTRL, SOFT_STOP_RELEASE_RATE * DT_CTRL))
        else:
          # not slowing (idle creep / PCM not braking yet): brake harder at the stock rate until it does
          output_accel = self.ramp_to_stop(output_accel)
        self.was_rolling = True
      elif pose_accel is not None and (self.was_rolling or self.coast_active):
        # wheel speed reads standstill but the car may still be rolling: wait for the true stop (pose based)
        self.was_rolling = False
        output_accel = self.coast_to_true_stop(output_accel, pose_accel)
      else:
        if self.was_rolling and not self.settle_used:
          # the speed reading just dropped to standstill but the car is still creeping: keep the request until
          # it has really stopped, so the stop does not come with the ramp to stopAccel already under way
          settle_s = float(np.clip(self.prev_v_ego / abs(SOFT_STOP_ACCEL) + SOFT_STOP_SETTLE_MARGIN, *SOFT_STOP_SETTLE_RANGE))
          self.settle_frames = int(settle_s / DT_CTRL)
          self.settle_used = True
        self.was_rolling = False
        if self.settle_frames > 0:
          self.settle_frames -= 1
        else:
          output_accel = self.ramp_to_stop(output_accel)
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    if self.long_control_state != LongCtrlState.stopping:
      self.was_rolling = False
      self.settle_frames = 0
      self.settle_used = False
      self.coast_active = False
      self.true_stopped = False
    self.prev_v_ego = CS.vEgo
    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
