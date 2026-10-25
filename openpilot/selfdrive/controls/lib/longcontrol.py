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
# The brake responds ~0.25-0.4 s late, so the release has to start before the stopping state (v < 0.3 m/s): in the
# final approach the decel request is capped by speed, like a driver easing off the pedal. The cap fades out
# continuously both with speed (gone at 3 m/s) and with how hard the plan brakes (full cap at -1.2, none at -1.8 m/s^2),
# so the request never jumps when either crosses a threshold. Simulated (0.25-0.4 s lag): decel at the stop
# -0.8..-1.3 -> about -0.35 m/s^2, at the cost of 0.15-0.7 m and up to ~1 s longer to stop.
SOFT_STOP_CAP_BP = [0.3, 0.8, 2.0, 3.0]   # m/s
SOFT_STOP_CAP_V = [0.3, 0.55, 1.2, 3.5]   # m/s^2, max decel request (3.5 = ACCEL_MIN, i.e. no cap)
SOFT_STOP_URGENT_BP = [-1.8, -1.2]        # m/s^2, a_target: cap fully released at -1.8, fully applied at -1.2

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

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP_SP, active, self.long_control_state,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if SOFT_STOP_V < CS.vEgo < SOFT_STOP_MAX_V and not CS.standstill:
        # still rolling: release toward SOFT_STOP_ACCEL (or brake gently toward it if coming from above)
        output_accel = min(output_accel, 0.0)
        output_accel += float(np.clip(SOFT_STOP_ACCEL - output_accel, -1.0 * DT_CTRL, SOFT_STOP_RELEASE_RATE * DT_CTRL))
      elif output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # TODO: can we just go straight to stopAccel?
        output_accel -= 1.0 * DT_CTRL  # m/s^2/s while trying to stop
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)
      if CS.vEgo < SOFT_STOP_CAP_BP[-1]:
        cap = float(np.interp(CS.vEgo, SOFT_STOP_CAP_BP, SOFT_STOP_CAP_V))
        weight = float(np.interp(a_target, SOFT_STOP_URGENT_BP, [0.0, 1.0]))  # 1 = apply the cap, 0 = urgent, no cap
        output_accel = weight * max(output_accel, -cap) + (1.0 - weight) * output_accel

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
