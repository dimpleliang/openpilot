import os
from collections import deque
from enum import IntEnum

import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
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
SOFT_STOP_ACCEL = -0.5         # m/s^2, brake request to ease toward while still rolling (= BRAKE_HOLD_ACCEL)
SOFT_STOP_RELEASE_RATE = 1.5   # m/s^3, how fast the brake is eased off
# The soft request is only kept while the car is really slowing down. At crawl speed the Toyota PCM barely brakes for
# small requests (idle creep: 10/27 route 4f, -0.2..-0.35 commanded gave 0..-0.1 m/s^2 for 1-3 s), so a fixed soft
# request can leave the car creeping. While rolling and not slowing, brake harder at the stock stopping rate instead.
SOFT_STOP_SLOWING_A = -0.1     # m/s^2, aEgo above this while rolling = not slowing
STOPPING_DECEL_RATE = 1.0      # m/s^3, stock rate toward stopAccel: after the true stop, or while not slowing at all
# 10/27 evening (route 56 21:40): the car was slowing at -0.09 m/s^2 at 0.29 m/s and would have stopped in ~3 s, but the
# rule above added brake at 1 m/s^3 (request -0.17 -> -0.77), the car stopped at -0.45 m/s^2 with a +0.30 rebound.
# Before the true stop, brake is added at SOFT_STOP_ADD_RATE instead: comma's former TSS2 stoppingDecelRate ("reach
# stopping target smoothly", upstream dropped per-car rates for a fixed 1.0; dragonpilot still uses 0.3). Only if the
# car does not slow down at all for SOFT_STOP_NO_DECEL_T (idle creep winning) is the stock rate used again.
SOFT_STOP_ADD_RATE = 0.3       # m/s^3
SOFT_STOP_NO_DECEL_A = -0.03   # m/s^2, deceleration (aEgo, or pose while coasting) above this = not slowing at all
SOFT_STOP_NO_DECEL_T = 1.5     # s
# 10/28 route 59: at crawl (0.2-0.3 m/s) the PCM brakes nothing for requests down to about -0.33 and then grabs: the
# wheels stopped when the request reached -0.44..-0.48 (added at 0.3 m/s^3 through the band in 0.4 s, lag 0.35-0.45 s),
# the body then decelerated at -0.35..-0.4 and rebounded +0.2. Inside the engagement band the request is therefore
# added at SOFT_STOP_BAND_RATE, so the lag overshoots the engagement point by ~0.04 instead of ~0.12, and the onset of
# braking is taken from the filtered pose acceleration (threshold SOFT_STOP_ONSET_A) instead of the wheel-speed aEgo.
SOFT_STOP_BAND_A = -0.33       # m/s^2, requests below this engage the friction brake at crawl speed
SOFT_STOP_BAND_RATE = 0.1      # m/s^3
SOFT_STOP_ONSET_A = -0.08      # m/s^2, filtered pose deceleration beyond this while rolling = the brake has engaged
# 10/28 route 5a, six stops: the three soft ones (-0.16..-0.22 at the stop, rebound <= 0.11) decelerated continuously
# from 0.6 m/s to 0 with the friction brake engaged the whole way. The three with a jolt (-0.29..-0.43, rebound
# 0.23..0.55) had a plateau at 0.4-0.5 m/s first: the planner eased its request to -0.05..-0.3, the PCM let the brake go
# (release at about -0.31), the car crept, and the brake had to re-engage at crawl speed, which it does with a grab
# (-0.5 m/s^2 for a -0.33 request). So below BRAKE_HOLD_V, while the planner still asks to slow down, the request is
# floored at BRAKE_HOLD_ACCEL to keep the brake engaged through the stop.
BRAKE_HOLD_V = 0.6             # m/s (10/28 route 5b: 0.8 with -0.33 stopped ~2 m short of the plan and still -0.37 at the stop)
POS_PID_MARGIN = 0.2  # m/s^2, ars510: PID may exceed a positive target by at most this (see update())
BRAKE_HOLD_A_TARGET = -0.1     # m/s^2, planner request below this = slowing to a stop (not following a creeping lead)
BRAKE_HOLD_ACCEL = -0.5        # m/s^2. 10/28 16:51 the driver's own stop: -0.9 at 0.4 m/s, then a linear release to 0
                               # over the last 0.4 s, no pitch at the stop (+0.16 rebound). A firm, continuous
                               # deceleration makes the stop time predictable for the pre-release below; -0.5 is a
                               # compromise with stopping distance (the planner asks ~-0.13 at 0.6 m/s).
# 11/09 13:01:47 (route 68): a gentle approach behind a stopped lead (plan -0.22, pose -0.21..-0.29) and the hold at
# 0.6 m/s stepped the request to -0.50; the last 1.3 s decelerated at -0.40..-0.49 (driver: "the last bit is still too
# hard"). The floor only has to keep the friction brake engaged (the PCM lets it go at about -0.31, SOFT_STOP_BAND_A):
# the hold now asks BRAKE_HOLD_MARGIN more than the plan, between BRAKE_HOLD_MIN (-0.35, = PRE_RELEASE_ACCEL, which held
# the brake through every stop since 11/08) and BRAKE_HOLD_ACCEL. A firmer plan still gets up to -0.50.
BRAKE_HOLD_MIN = -0.35         # m/s^2
# 11/10 (driver: the stop still ends with a small jolt). 15 stops of 11/09: over the last second the plan asked only
# -0.10..-0.22, the request sent was -0.35..-0.52 (this floor + margin + PID) and the IMU showed -0.34 at the stop: the
# floor, not the plan, sets the jolt. Whether the PCM keeps the brake on for a lighter request at crawl is not settled
# (0x0A6 BRAKE_FORCE says it does at 0.6-1.5 m/s; below 0.6 m/s there are too few frames), and -0.05 rolled on 11/07.
# So the floor is tried on the car without a new build: a number in STOP_FLOOR_FILE (m/s^2, clipped to STOP_FLOOR_RANGE)
# replaces BRAKE_HOLD_MIN and PRE_RELEASE_ACCEL; no file = the constants. Read once a second.
STOP_FLOOR_FILE = "/data/ars510_stop_floor"
STOP_FLOOR_RANGE = (-0.40, -0.15)


def read_stop_floor():
  try:
    with open(STOP_FLOOR_FILE) as f:
      return float(np.clip(float(f.read().strip()), *STOP_FLOOR_RANGE))
  except Exception:
    return None
BRAKE_HOLD_MARGIN = 0.0        # m/s^2. 11/10: was 0.1. With the landing rule (ars510_long_rules LAND_*) the plan over the last
                               # metres is already a steady -0.28..-0.5; another 0.1 on top at 0.6 m/s is the late extra brake
                               # the driver feels (stops with -0.41..-0.50 sent in the last second: -0.40..-0.46 at the stop,
                               # the one with a steady -0.30: -0.14). A lighter plan is still floored at the stop floor.
BRAKE_HOLD_ONSET_RATE = 0.5    # m/s^3: the hold builds up gradually from the current request (driver: "progressively
                               # heavier", 11/09) instead of stepping to the floor; -0.22 -> -0.35 in 0.26 s, -> -0.50 in 0.56 s
BRAKE_HOLD_CLEAR_A_TARGET = 0.1  # m/s^2, the hold is latched until the planner wants to move again (same as should_stop)
# The stopping state ignores the plan. With the gentle rate, follow the plan when it asks for clearly more braking than
# the current request (lead closer than expected), so the soft logic never brakes less than an urgent plan.
SOFT_STOP_URGENT_MARGIN = 0.3  # m/s^2
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
STALL_DROP = 0.05              # m/s (11/10: 0.1 called a car slowing at 0.09 m/s^2 stalled and added brake: -0.35 at the stop)
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
# 11/01 route 36 (20 stops): the pitch rebound is gone (<= 0.05 in 15 of 20), but in 5 stops the car moved forward again
# 0.5-0.6 s after it had stopped (wheel speed 0 -> 0.3 m/s, pose +0.25) and the brake had to grab a second time: the
# pre-release had taken the request to -0.05 and the true stop was only recognised when the filtered decel had been
# below SOFT_STOP_COAST_MIN_DECEL for PRE_RELEASE_FAIL_T, ~0.6 s after the stop, while the D-gear creep torque pushed
# the car off against almost no brake. The stop is now recognised from the raw pose: wheel speed 0 and |pose accel| <
# SOFT_STOP_STILL_A for SOFT_STOP_STILL_T, and the request then goes to BRAKE_HOLD_ACCEL at once (brake added at
# standstill does not pitch the car) before the usual ramp to stopAccel.
SOFT_STOP_STILL_A = 0.06        # m/s^2
SOFT_STOP_STILL_T = 0.12        # s
SOFT_STOP_STILL_WHEEL_V = 0.01  # m/s
# 11/02 review (Codex): zero acceleration alone cannot tell "stopped" from "creeping at a steady 0.1-0.2 m/s with the wheel
# speed already reading 0". So (1) the still timer only counts fresh valid pose samples (an invalid pose resets it); (2) the
# request does not jump to BRAKE_HOLD_ACCEL but ramps there at SOFT_STOP_STILL_RATE (0.18 s), so a car that was in fact
# still creeping gets a firm but not abrupt brake; (3) if the wheels report rolling again (> SOFT_STOP_STILL_UNDO_V) the
# stop verdict is withdrawn and the closed-loop rolling branch takes over.
SOFT_STOP_STILL_RATE = 2.5      # m/s^3
SOFT_STOP_STILL_UNDO_V = 0.2    # m/s
# Pre-release (10/28): a human stops without a jolt by easing the pedal in the last tenths of a second, so there is no
# deceleration left when the wheels stop. The PCM gives no light braking at crawl speed (either nothing or >= -0.3) and
# lags ~0.4 s, so instead the request is eased to PRE_RELEASE_ACCEL when the pose-based time to stop (v_est / decel)
# drops to PRE_RELEASE_LEAD: the release reaches the brakes about when the car stops. If the car then does not slow
# down for PRE_RELEASE_FAIL_T (released too early, or already stopped), brake is added again as usual, once per stop.
# Disabled when the file PRE_RELEASE_OFF_FILE exists.
# 10/28 route 62 (first test): the PCM follows a release almost at once (IMU decel -0.5 -> 0 within 0.1 s of the ramp),
# but the car had already stopped 0.15-0.25 s after the wheel speed read zero, i.e. ~0.25 s before the ramp ended, so it
# still stopped at -0.45 (+0.35 rebound). The Toyota wheel speed over-reads just before it drops out (v_est from it
# predicts the stop late). Trigger therefore at PRE_RELEASE_TRIGGER of predicted time to stop and ramp over
# PRE_RELEASE_LEAD, which ends about when the car really stops.
PRE_RELEASE_TRIGGER = 0.45      # s, predicted v_est / decel at which the release starts (0.8 with the uncorrected wheel
                                # speed released too early on route 63: the car crept 1.8 s and had to be braked again)
WHEEL_OVERREAD = 0.12           # m/s, wheel speed over-read taken off v_est as the reading approaches drop-out
WHEEL_OVERREAD_V = 0.5          # m/s, the correction fades in below this
PRE_RELEASE_LEAD = 0.4          # s, ramp duration (the driver's own release took ~0.4 s)
PRE_RELEASE_ACCEL = -0.35       # m/s^2, released to here linearly over PRE_RELEASE_LEAD. 10/28-11/07 at -0.05: the release
                                # let go of the friction brake entirely (PCM gives nothing above ~-0.31), and with the
                                # wheel speed reading 0 below 0.22 m/s the car was sometimes still rolling: 11/07 raw IMU,
                                # 2 of 19 stops rolled on (+0.27/+0.42) and were grabbed again (-0.35/-0.44) ~0.8 s later.
                                # -0.35 keeps the brake just engaged through the stop (Codex round 4: test from -0.35, not -0.30)
PRE_RELEASE_MAX_V = 0.45        # m/s, only in the very last stretch
PRE_RELEASE_MIN_DECEL = 0.12    # m/s^2, need a real deceleration to predict the stop
PRE_RELEASE_FAIL_T = 0.3        # s without deceleration -> give up (brake again)
PRE_RELEASE_MAX_T = 1.5         # s
PRE_RELEASE_OFF_FILE = "/data/ars510_no_pre_release"

LongCtrlState = car.CarControl.Actuators.LongControlState


# ---------------------------------------------------------------------------------------------------------------------
# Structure (G1 refactor, behaviour-identical to 462845c, verified frame by frame by tools/ars510/longcontrol_equiv.py)
#
# The external state is still LongCtrlState (off / pid / stopping), produced by long_control_state_trans(). Inside it,
# LongControl runs an explicit sub-state, StopPhase, chosen once per frame in one place (_select_stop_phase for the
# stopping state, _update_drive for pid) and recorded with a reason in self.stop_phase / self.last_transition:
#
#   OFF ──engage──> DRIVE <──> BRAKE_HOLD                              (LongCtrlState.pid)
#     └──engage──> stopping:  APPROACH ──wheel speed drops out──> CAPTURE ──true stop──> HOLD
#                                │   (no pose path)                  │ (pose lost)
#                                └──────────────> SETTLE <───────────┘
#                  RELEASE = pre-release overlay on top of APPROACH / CAPTURE (reported instead of the base phase)
#
# Speed / acceleration estimation is a separate component (SpeedEstimator). The safety arbitration (urgent plan) is
# applied first (to the starting request) and last (min with the plan) in _update_stopping.
# Lines marked "BEHAVIOUR KEPT (G1)" are known oddities kept on purpose for this step; see ARS510_RADAR.md › F › G1.
# ---------------------------------------------------------------------------------------------------------------------


class StopPhase(IntEnum):
  OFF = 0         # long control not active                                                  (LongCtrlState.off)
  DRIVE = 1       # PID on the plan                                                          (LongCtrlState.pid)
  BRAKE_HOLD = 2  # PID with the request floored at plan - BRAKE_HOLD_MARGIN in [BRAKE_HOLD_ACCEL, BRAKE_HOLD_MIN] (latched, see BRAKE_HOLD_*) (LongCtrlState.pid)
  APPROACH = 3    # stopping, wheels report rolling: soft-stop closed loop on aEgo / pose     (LongCtrlState.stopping)
  CAPTURE = 4     # stopping, wheel speed reads standstill, pose valid: hold until the pose says stopped
  HOLD = 5        # stopping, true stop recognised on the pose path: brake to BRAKE_HOLD_ACCEL, then stopAccel
  SETTLE = 6      # stopping, no pose path (pose invalid, or entered at standstill): timed settle hold, then ramp
  RELEASE = 7     # stopping, pre-release overlay easing the request just before the predicted true stop


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


class SpeedEstimator:
  """True speed and filtered pose acceleration for the soft stop.

  v_est: wheel speed (minus the drop-out over-read) while the wheel speed is valid or the pose is not; below
         SOFT_STOP_WHEEL_VALID_V with a valid pose, the pose acceleration integrated from the last value.
  coast_frames: frames v_est has been integrated from the pose (0 while it follows the wheel speed).
  pose_a_f: pose acceleration low-passed with SOFT_STOP_POSE_TAU; equals aEgo while the pose is invalid.
  """
  def __init__(self):
    self.v_est = 0.0
    self.coast_frames = 0
    self.pose_a_f = 0.0

  @property
  def coast_t(self):
    return self.coast_frames * DT_CTRL

  def update(self, v_ego_raw, a_ego, pose_accel):
    """True speed: wheel speed while it is valid, below that integrate the pose acceleration."""
    if pose_accel is None or v_ego_raw >= SOFT_STOP_WHEEL_VALID_V:
      # BEHAVIOUR KEPT (G1): with the pose invalid the speed is taken as the wheel reading, so below the Toyota drop-out
      # an unknown speed counts as 0 (no uncertainty grows while the pose is missing), and the coast time restarts.
      # the Toyota wheel speed over-reads just before it drops out (10/28 route 62: it read 0.22-0.23 m/s when the car,
      # by the IMU, had 0.15-0.25 s left at -0.47 m/s^2, i.e. ~0.1 m/s); take that off below WHEEL_OVERREAD_V
      over = WHEEL_OVERREAD * float(np.clip((WHEEL_OVERREAD_V - v_ego_raw) / (WHEEL_OVERREAD_V - SOFT_STOP_WHEEL_VALID_V), 0.0, 1.0))
      self.v_est = max(0.0, v_ego_raw - over)
      self.coast_frames = 0
    else:
      self.v_est = max(0.0, self.v_est + pose_accel * DT_CTRL)
      self.coast_frames += 1
    if pose_accel is not None:
      self.pose_a_f += (pose_accel - self.pose_a_f) * DT_CTRL / SOFT_STOP_POSE_TAU
    else:
      # BEHAVIOUR KEPT (G1): the filter state jumps to the wheel-speed aEgo while the pose is invalid
      self.pose_a_f = a_ego

  def mark_stopped(self):
    self.v_est = min(self.v_est, SOFT_STOP_TRUE_STOP_V)


class LongControl:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.prev_v_ego = 0.0
    # pose-based soft stop
    self.est = SpeedEstimator()
    self.stall_v_hist = deque(maxlen=int(round(STALL_WINDOW / DT_CTRL)) + 1)
    self.stall_stop = False
    self.brake_hold = False
    self.hold_floor = None       # ramped brake-hold floor (BRAKE_HOLD_ONSET_RATE), None outside a hold
    self.pre_release_enabled = not os.path.exists(PRE_RELEASE_OFF_FILE)
    self.stop_floor = read_stop_floor()   # None = BRAKE_HOLD_MIN / PRE_RELEASE_ACCEL (see STOP_FLOOR_FILE)
    self.stop_floor_poll = 0
    # BEHAVIOUR KEPT (G1): these three are only set when a pre-release triggers and are never reset between stops;
    # pre_release_start is read by the release-failed recovery (also after an urgent plan cancelled a stop that never
    # released, i.e. with the value of an earlier stop)
    self.pre_release_frames = 0
    self.pre_release_slow = 0
    self.pre_release_start = 0.0
    # sub-state bookkeeping (diagnostics only, never read by the control law)
    self.stop_phase = StopPhase.OFF
    self.last_transition = (StopPhase.OFF, StopPhase.OFF, "init")
    self._release_end_reason = ""
    self._brake_hold_clear_reason = ""
    self._entry_reason = ""      # why LongCtrlState changed this frame ("" if it did not)
    self._frame_transitions = []  # phase changes of this frame, logged at the end of update() (diagnostics only)
    self._reset_stop_latches()

  # --- compatibility views of the estimator --------------------------------------------------------------------------
  @property
  def v_est(self):
    return self.est.v_est

  @v_est.setter
  def v_est(self, v):
    self.est.v_est = v

  @property
  def coast_frames(self):
    return self.est.coast_frames

  @property
  def pose_a_f(self):
    return self.est.pose_a_f

  # --- per-stop latches -----------------------------------------------------------------------------------------------
  def _reset_stop_latches(self):
    """Everything that lives for one stop (cleared every frame outside LongCtrlState.stopping)."""
    self.was_rolling = False
    self.settle_frames = 0       # frames left to hold the soft request after the speed reading drops to standstill
    self.settle_used = False     # the settle hold runs once per stop
    self.coast_active = False    # CAPTURE/HOLD path entered in this stop (pose based true-stop wait)
    self.true_stopped = False
    self.no_decel_frames = 0
    self.brake_added = False
    self.pre_release = False
    self.pre_release_failed = False
    self.still_frames = 0
    self.still_set = False       # true_stopped was set by update_still (undo-able while the wheels still read 0)

  def _set_phase(self, phase, reason):
    """The one place the sub-state changes; every change is recorded with its reason."""
    if phase != self.stop_phase:
      if self._entry_reason:
        reason = f"{self._entry_reason}; {reason}"
      self.last_transition = (self.stop_phase, phase, reason)
      self._frame_transitions.append(self.last_transition)
      self.stop_phase = phase

  def reset(self):
    self.pid.reset()

  # --- helpers --------------------------------------------------------------------------------------------------------
  def ramp_to_stop(self, output_accel, rate=STOPPING_DECEL_RATE):
    # stopping ramp: m/s^2/s toward stopAccel, never releasing a harder request
    # TODO: can we just go straight to stopAccel?
    if output_accel > self.CP.stopAccel:
      output_accel = max(output_accel - rate * DT_CTRL, self.CP.stopAccel)
    return output_accel

  def add_brake_rate(self, decel_signal, output_accel):
    """Rate for adding brake before the true stop: gentle, slower still inside the brake engagement band, and the stock
    rate only if the car has not slowed down at all for a long while."""
    # BEHAVIOUR KEPT (G1): one counter shared by APPROACH (signal min(aEgo, pose_a_f)) and CAPTURE (pose_a_f); it is
    # updated every APPROACH frame even when the rate is not used (slowing branch), and only reset outside stopping or
    # by a failed pre-release
    self.no_decel_frames = self.no_decel_frames + 1 if decel_signal > SOFT_STOP_NO_DECEL_A else 0
    if self.no_decel_frames * DT_CTRL >= SOFT_STOP_NO_DECEL_T:
      return STOPPING_DECEL_RATE
    return SOFT_STOP_BAND_RATE if output_accel <= SOFT_STOP_BAND_A else SOFT_STOP_ADD_RATE

  def update_stall(self, active, CS, a_target):
    """Latch a stop request when the car hangs at crawl speed although the planner keeps asking to slow down."""
    if not active or CS.brakePressed or CS.gasPressed or a_target >= STALL_CLEAR_A_TARGET:
      self.stall_stop = False
    # BEHAVIOUR KEPT (G1): uses the LongCtrlState of the previous frame
    crawling = (active and self.long_control_state == LongCtrlState.pid and not CS.standstill and
                SOFT_STOP_V < CS.vEgo < STALL_MAX_V and a_target < STALL_A_TARGET)
    if not crawling:
      self.stall_v_hist.clear()
      return
    self.stall_v_hist.append(CS.vEgo)
    if len(self.stall_v_hist) == self.stall_v_hist.maxlen and self.stall_v_hist[0] - CS.vEgo < STALL_DROP:
      self.stall_stop = True
      self.stall_v_hist.clear()

  def update_still(self, CS, pose_accel):
    """True stop from the raw pose: wheels read 0 and no longitudinal acceleration for SOFT_STOP_STILL_T of valid samples."""
    if self.true_stopped and CS.vEgoRaw > SOFT_STOP_STILL_UNDO_V:
      # BEHAVIOUR KEPT (G1): the undo is partial: v_est stays clamped to SOFT_STOP_TRUE_STOP_V, brake_added /
      # pre_release_failed / settle_used / no_decel_frames keep their values, and it only fires above 0.2 m/s raw
      self.true_stopped = False   # the wheels say we were not stopped after all (whichever rule said so)
      self.still_set = False
      self.coast_active = False
    if pose_accel is None:
      self.still_frames = 0
      return
    if self.true_stopped:
      return
    # BEHAVIOUR KEPT (G1): zero acceleration with the wheel speed already reading 0 also matches a steady creep
    if CS.vEgoRaw < SOFT_STOP_STILL_WHEEL_V and abs(pose_accel) < SOFT_STOP_STILL_A:
      self.still_frames += 1
    else:
      self.still_frames = 0
    if self.still_frames * DT_CTRL >= SOFT_STOP_STILL_T:
      self.true_stopped = True
      self.still_set = True
      self.est.mark_stopped()

  def update_pre_release(self, pose_accel):
    """Ease the request just before the predicted true stop (see PRE_RELEASE_*). Returns True while releasing."""
    if pose_accel is None or not self.pre_release_enabled or self.true_stopped or self.pre_release_failed:
      if self.pre_release:
        self._release_end_reason = ("pose invalid" if pose_accel is None else "true stop" if self.true_stopped else
                                    "disabled" if not self.pre_release_enabled else "cancelled")
      self.pre_release = False
      return False
    decel = -self.est.pose_a_f
    if not self.pre_release:
      # BEHAVIOUR KEPT (G1): may trigger with brake_added set (creep / downhill brake latch), the release's max() then
      # eases that added brake too
      if 0.0 < self.est.v_est <= PRE_RELEASE_MAX_V and decel >= PRE_RELEASE_MIN_DECEL and self.est.v_est / decel <= PRE_RELEASE_TRIGGER:
        self.pre_release = True
        self.pre_release_frames = 0
        self.pre_release_slow = 0
        self.pre_release_start = min(self.last_output_accel, 0.0)
      return self.pre_release
    self.pre_release_frames += 1
    self.pre_release_slow = self.pre_release_slow + 1 if decel < SOFT_STOP_COAST_MIN_DECEL else 0
    if self.pre_release_slow * DT_CTRL >= PRE_RELEASE_FAIL_T or self.pre_release_frames * DT_CTRL >= PRE_RELEASE_MAX_T:
      # stopped (fine: brake is now added at standstill) or still creeping (brake is added again): either way, once
      self._release_end_reason = "no decel for PRE_RELEASE_FAIL_T" if self.pre_release_slow * DT_CTRL >= PRE_RELEASE_FAIL_T \
                                 else "PRE_RELEASE_MAX_T"
      self.pre_release = False
      self.pre_release_failed = True
      self.no_decel_frames = 0
      return False
    return True

  # --- stopping: sub-state selection (all stopping transitions happen here) ----------------------------------------------
  def _select_stop_phase(self, CS, pose_accel):
    """Pick APPROACH / CAPTURE / HOLD / SETTLE for this frame and apply the latches that go with the transition.
    The order of the tests is the priority of the original if/elif chain and must not change."""
    if SOFT_STOP_V < CS.vEgo < SOFT_STOP_MAX_V and not CS.standstill:
      # BEHAVIOUR KEPT (G1): true_stopped (e.g. from the still detector) is not consulted here: wheels reading
      # 0.05-0.2 m/s run the rolling branch with true_stopped still set (undo only above SOFT_STOP_STILL_UNDO_V)
      self.was_rolling = True
      return StopPhase.APPROACH, "wheels rolling"
    # BEHAVIOUR KEPT (G1): at vEgo >= SOFT_STOP_MAX_V (stopping requested fast, or speeding up downhill) the car is not
    # "rolling" either and lands in CAPTURE (if it was rolling / capturing, pose valid) or in SETTLE (stock-rate ramp)
    if pose_accel is not None and (self.was_rolling or self.coast_active):
      # wheel speed reads standstill but the car may still be rolling: wait for the true stop (pose based)
      reason = "wheel speed standstill, pose valid" if self.was_rolling else "capturing"
      self.was_rolling = False
      self.coast_active = True
      if self.true_stopped:
        return StopPhase.HOLD, ("still detector" if self.still_set else "true stop")
      if self.est.v_est <= SOFT_STOP_TRUE_STOP_V:
        self.true_stopped = True
        return StopPhase.HOLD, "v_est <= SOFT_STOP_TRUE_STOP_V"
      if self.est.coast_t >= SOFT_STOP_MAX_COAST:
        self.true_stopped = True
        return StopPhase.HOLD, "SOFT_STOP_MAX_COAST"
      return StopPhase.CAPTURE, reason
    # BEHAVIOUR KEPT (G1): losing the pose in CAPTURE/HOLD drops to SETTLE: was_rolling is already False there, so no
    # settle hold is armed and (settle_used False) the stock-rate ramp starts at once, i.e. an unknown speed is
    # treated as stopped. A true stop from the still detector with no prior roll-in also runs here (stock ramp).
    reason = "no pose" if pose_accel is None else "entered at standstill"
    if self.was_rolling and not self.settle_used:
      # the speed reading just dropped to standstill but the car is still creeping: keep the request until
      # it has really stopped, so the stop does not come with the ramp to stopAccel already under way
      settle_s = float(np.clip(self.prev_v_ego / abs(SOFT_STOP_ACCEL) + SOFT_STOP_SETTLE_MARGIN, *SOFT_STOP_SETTLE_RANGE))
      self.settle_frames = int(settle_s / DT_CTRL)
      self.settle_used = True
      reason = "wheel speed standstill, no pose: settle timer"
    self.was_rolling = False
    return StopPhase.SETTLE, reason

  # --- stopping: one method per sub-state (output only; no phase changes) ------------------------------------------------
  def _approach(self, CS, pose_accel, output_accel):
    # the pose acceleration shows the brake engaging ~0.3 s before the wheel-speed aEgo does
    decel = min(CS.aEgo, self.est.pose_a_f) if pose_accel is not None else CS.aEgo
    rate = self.add_brake_rate(decel, output_accel)
    if CS.aEgo < SOFT_STOP_SLOWING_A or (pose_accel is not None and self.est.pose_a_f < SOFT_STOP_ONSET_A):
      if not self.brake_added:
        # slowing down: ease the request toward SOFT_STOP_ACCEL (or brake gently toward it if coming from above)
        output_accel += float(np.clip(SOFT_STOP_ACCEL - output_accel, -SOFT_STOP_ADD_RATE * DT_CTRL, SOFT_STOP_RELEASE_RATE * DT_CTRL))
      # else: brake had to be added in this stop (creep / downhill): keep it, releasing it again would let the car
      # roll on and add it back, without ever stopping (simulated downhill + creep)
    else:
      # not slowing enough (idle creep / PCM not braking yet): add brake until it does
      output_accel = self.ramp_to_stop(output_accel, rate)
      self.brake_added = True
    return output_accel

  def _capture(self, output_accel):
    """Wheel speed reads standstill: hold the request until the pose says the car has really stopped."""
    rate = self.add_brake_rate(self.est.pose_a_f, output_accel)
    if -self.est.pose_a_f < SOFT_STOP_COAST_MIN_DECEL:
      # not slowing (idle creep): add brake (gently, at the stock rate if it keeps creeping) until it does
      return self.ramp_to_stop(output_accel, rate)
    return output_accel

  def _hold(self, output_accel):
    # stopped: engage the brake against the creep torque quickly (SOFT_STOP_STILL_RATE down to BRAKE_HOLD_ACCEL), then ramp
    # to stopAccel as before
    if output_accel > BRAKE_HOLD_ACCEL:
      return max(self.ramp_to_stop(output_accel, SOFT_STOP_STILL_RATE), BRAKE_HOLD_ACCEL)
    return self.ramp_to_stop(output_accel)

  def _settle(self, output_accel):
    if self.settle_frames > 0:
      self.settle_frames -= 1
      return output_accel
    # after a roll-in (no pose) the car may still be creeping: gentle ramp; entering at standstill: stock rate
    return self.ramp_to_stop(output_accel, SOFT_STOP_ADD_RATE if self.settle_used else STOPPING_DECEL_RATE)

  def _release(self, output_accel):
    # linear release from the request at the trigger to PRE_RELEASE_ACCEL over PRE_RELEASE_LEAD
    frac = min(1.0, (self.pre_release_frames + 1) * DT_CTRL / PRE_RELEASE_LEAD)
    end = PRE_RELEASE_ACCEL if self.stop_floor is None else self.stop_floor
    return max(output_accel, self.pre_release_start + (end - self.pre_release_start) * frac)

  def _rebrake_after_release(self, output_accel):
    # released too early: back to the request that was braking, at the gentle add rate (not in one frame: the review
    # found -0.05 -> -0.5 in a single step)
    # BEHAVIOUR KEPT (G1): the step is SOFT_STOP_ADD_RATE * 5 (1.5 m/s^3), measured from last_output_accel, so it can
    # brake less than the base phase computed this frame; it also runs after an urgent plan set pre_release_failed in
    # a stop that never released (pre_release_start from an earlier stop)
    if output_accel > self.pre_release_start:
      output_accel = max(self.pre_release_start, self.last_output_accel - SOFT_STOP_ADD_RATE * DT_CTRL * 5)
    return output_accel

  def _update_stopping(self, CS, a_target, pose_accel):
    output_accel = min(self.last_output_accel, 0.0)
    self.update_still(CS, pose_accel)

    # safety arbitration, part 1: an urgent plan replaces the request before any comfort shaping
    urgent = a_target < output_accel - SOFT_STOP_URGENT_MARGIN
    if urgent:
      output_accel = a_target
      self.brake_added = True
      # 11/02 review (Codex, round 2): the pre-release ran last and its max() overrode an urgent plan (a -2.0 request came
      # out as -0.37). An urgent plan cancels the release for this stop.
      if self.pre_release:
        self._release_end_reason = "urgent plan"
      self.pre_release = False
      self.pre_release_failed = True

    phase, reason = self._select_stop_phase(CS, pose_accel)
    if phase == StopPhase.APPROACH:
      output_accel = self._approach(CS, pose_accel, output_accel)
    elif phase == StopPhase.CAPTURE:
      output_accel = self._capture(output_accel)
    elif phase == StopPhase.HOLD:
      output_accel = self._hold(output_accel)
    else:
      output_accel = self._settle(output_accel)

    # RELEASE overlay (after the base phase, whose latches and counters have already run this frame)
    if self.update_pre_release(pose_accel):
      if self.stop_phase != StopPhase.RELEASE:
        reason = "predicted stop within PRE_RELEASE_TRIGGER"
      phase = StopPhase.RELEASE
      output_accel = self._release(output_accel)
    elif self.pre_release_failed and not self.true_stopped and not urgent:
      output_accel = self._rebrake_after_release(output_accel)
    if self.stop_phase == StopPhase.RELEASE and phase != StopPhase.RELEASE:
      reason = f"release ended ({self._release_end_reason}); {reason}"

    # safety arbitration, part 2:
    # the plan always has the last word on braking more: no comfort shaping above may brake less than an urgent plan
    if urgent:
      output_accel = min(output_accel, a_target)
    self.reset()
    self._set_phase(phase, reason)
    return output_accel

  def _state_change_reason(self, active, should_stop, CS):
    if not active:
      return "not active"
    if self.long_control_state == LongCtrlState.pid:
      return "starting condition"
    return ("should_stop" if should_stop else "creep stall" if self.stall_stop else "brake pressed" if CS.brakePressed
            else "cruise standstill")

  # --- pid / off ------------------------------------------------------------------------------------------------------
  def _update_drive(self, CS, a_target):
    error = a_target - CS.aEgo
    output_accel = self.pid.update(error, speed=CS.vEgo,
                                   feedforward=a_target)
    # ars510: the planner's positive-acceleration ceiling (longitudinal_planner A_POS_MAX) is meant to be felt; the PID
    # may add at most POS_PID_MARGIN on top of a positive target (uphill correction), never a second step of throttle.
    if a_target > 0.:
      output_accel = min(output_accel, a_target + POS_PID_MARGIN)
    # keep the friction brake engaged through the stop (see BRAKE_HOLD_*); latched, so a planner request that
    # hovers around the threshold while closing on a stopped lead does not release and re-grab the brake
    # BEHAVIOUR KEPT (G1): the latch is set before the clear conditions are checked (end of update()), so the hold is
    # applied for the frame in which gas / a positive plan / speed would clear it, and is re-set every frame while
    # vEgo < BRAKE_HOLD_V and a_target < BRAKE_HOLD_A_TARGET even with the gas pressed
    if CS.vEgo < BRAKE_HOLD_V and a_target < BRAKE_HOLD_A_TARGET:
      self.brake_hold = True
    if self.brake_hold:
      target = float(np.clip(a_target - BRAKE_HOLD_MARGIN, BRAKE_HOLD_ACCEL,
                             BRAKE_HOLD_MIN if self.stop_floor is None else self.stop_floor))
      if self.hold_floor is None:
        self.hold_floor = min(float(output_accel), 0.)
      self.hold_floor = target if target > self.hold_floor else max(target, self.hold_floor - BRAKE_HOLD_ONSET_RATE * DT_CTRL)
      output_accel = min(output_accel, self.hold_floor)
      self._set_phase(StopPhase.BRAKE_HOLD, "v < BRAKE_HOLD_V and plan < BRAKE_HOLD_A_TARGET"
                      if self.stop_phase == StopPhase.DRIVE else "pid")
    else:
      self._set_phase(StopPhase.DRIVE, self._brake_hold_clear_reason if self.stop_phase == StopPhase.BRAKE_HOLD else "pid")
    return output_accel

  def update(self, active, CS, a_target, should_stop, accel_limits, pose_accel=None):
    """Update longitudinal control. This updates the state machine and runs a PID loop.
    pose_accel: calibrated longitudinal acceleration from locationd (m/s^2), None if not valid."""
    self.pid.neg_limit = accel_limits[0]
    self.stop_floor_poll += 1
    if self.stop_floor_poll >= int(1. / DT_CTRL):
      self.stop_floor_poll = 0
      floor = read_stop_floor()
      if floor != self.stop_floor:
        cloudlog.info(f"ARS510 stop floor {self.stop_floor} -> {floor}")
        self.stop_floor = floor
    self.pid.pos_limit = accel_limits[1]
    self.est.update(CS.vEgoRaw, CS.aEgo, pose_accel)

    self.update_stall(active, CS, a_target)
    prev_state = self.long_control_state
    self.long_control_state = long_control_state_trans(self.CP_SP, active, self.long_control_state,
                                                       should_stop or self.stall_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    self._entry_reason = "" if self.long_control_state == prev_state else \
      self._state_change_reason(active, should_stop, CS)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.
      self._set_phase(StopPhase.OFF, "not active")
    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self._update_stopping(CS, a_target, pose_accel)
    else:  # LongCtrlState.pid
      output_accel = self._update_drive(CS, a_target)

    # end-of-frame latch maintenance (order kept from 462845c)
    if self.long_control_state != LongCtrlState.pid or CS.gasPressed or a_target > BRAKE_HOLD_CLEAR_A_TARGET or CS.vEgo > BRAKE_HOLD_V + 0.2:
      if self.brake_hold:
        self._brake_hold_clear_reason = ("not pid" if self.long_control_state != LongCtrlState.pid else "gas" if CS.gasPressed
                                         else "plan > BRAKE_HOLD_CLEAR_A_TARGET" if a_target > BRAKE_HOLD_CLEAR_A_TARGET
                                         else "v > BRAKE_HOLD_V + 0.2")
      self.brake_hold = False
    if not self.brake_hold:
      self.hold_floor = None
    if self.long_control_state != LongCtrlState.stopping:
      self._reset_stop_latches()
    self.prev_v_ego = CS.vEgo
    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    if self._frame_transitions:
      self._log_transitions(CS, a_target)
    return self.last_output_accel

  def _log_transitions(self, CS, a_target):
    """11/08: one rlog line per stop sub-state change (a few per stop) for the per-stop report
    (tools/ars510/mac/analysis/stop_report.py). Diagnostics only: never affects the control output."""
    for frm, to, reason in self._frame_transitions:
      try:
        cloudlog.info(f"ARS510 stop phase {frm.name}->{to.name} | {reason} | v={CS.vEgo:.3f} vraw={CS.vEgoRaw:.3f} "
                      f"v_est={self.est.v_est:.3f} pose_a_f={self.est.pose_a_f:.3f} a_target={a_target:.3f} "
                      f"out={float(self.last_output_accel):.3f} brake_added={int(self.brake_added)}")
      except Exception:
        pass
    self._frame_transitions = []
