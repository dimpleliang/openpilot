#!/usr/bin/env python3
import os
import types
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource, \
  get_T_FOLLOW, STOP_DISTANCE
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
J_CRUISE_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = -0.5
# ars510: firmer cruise deceleration when the target speed (set speed, SCC-Vision, speed limit)
# is well below the current speed, so the car actually slows before a ramp or curve
# instead of coasting at -0.5 m/s^2. Indexed by (v_ego - v_cruise) in m/s.
A_CRUISE_MIN_BP = [1.0, 4.0, 8.0]
A_CRUISE_MIN_V = [A_CRUISE_MIN, -1.0, -1.5]
# ars510: no acceleration while a turn signal is on at low speed (turning / preparing to turn).
# 10/27: raised from 30 km/h; at 21:41 the car was at 31-32 km/h in a left turn when the lead left the view and it
# accelerated toward the set speed.
BLINKER_NO_ACCEL_V = 50. * CV.KPH_TO_MS
# ars510: no acceleration while the wheel is turned far (turning at a junction), with or without a signal, so losing
# the lead mid-turn does not make the car speed up. Enters above TURN_NO_ACCEL_ANGLE, leaves below TURN_NO_ACCEL_EXIT_ANGLE.
# Not below TURN_NO_ACCEL_MIN_V, so the car still pulls away from a stop with the wheel turned.
TURN_NO_ACCEL_ANGLE = 45.        # deg, steering wheel
TURN_NO_ACCEL_EXIT_ANGLE = 30.   # deg
TURN_NO_ACCEL_MIN_V = 3.         # m/s
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
# ars510: gentler braking for a far, newly-seen slower lead. 10/30 13:27: radar first saw a car 118 m ahead closing at
# 12 m/s (118 km/h vs 75 km/h); the MPC went to -1.9 m/s^2 within 2 s although about -0.9 m/s^2 would close the gap.
# While the radar lead is far and time to reach it is long, braking is limited to FAR_LEAD_K times the constant
# deceleration that removes the closing speed before a comfortable gap (follow distance + FAR_LEAD_MARGIN).
# The limit fades out as distance or time to contact shrink, so the MPC brakes as before when the lead is close.
# Off: touch /data/ars510_no_far_lead_soft.
FAR_LEAD_K = 1.25
FAR_LEAD_MARGIN = 5.           # m, added to t_follow * v_lead + STOP_DISTANCE
FAR_LEAD_D_BP = [50., 70.]     # m, dRel: no limit below 50 m, full limit above 70 m
FAR_LEAD_TTC_BP = [5., 7.]     # s, dRel / closing speed: no limit below 5 s, full limit above 7 s
FAR_LEAD_MIN_CLOSING = 1.0     # m/s
FAR_LEAD_MAX_LEAD_DECEL = -1.0  # m/s^2, lead braking harder than this: no limit
FAR_LEAD_MIN_DECEL = -0.3      # the limit never asks for less braking than this
FAR_LEAD_OFF_FILE = "/data/ars510_no_far_lead_soft"


def far_lead_accel_floor(lead, t_follow):
  """Lowest acceleration allowed for a far radar lead, and how much of it applies (0..1). None when not applicable."""
  if not (lead.present and lead.radar):
    return None
  closing = -lead.vRel
  if closing < FAR_LEAD_MIN_CLOSING or lead.aLeadK < FAR_LEAD_MAX_LEAD_DECEL:
    return None
  ttc = lead.dRel / closing
  w = float(np.interp(lead.dRel, FAR_LEAD_D_BP, [0., 1.]) * np.interp(ttc, FAR_LEAD_TTC_BP, [0., 1.]))
  if w <= 0.:
    return None
  gap = lead.dRel - (t_follow * max(lead.vLead, 0.) + STOP_DISTANCE + FAR_LEAD_MARGIN)
  if gap <= 1.:
    return None
  a_req = closing ** 2 / (2. * gap)
  floor = min(min(lead.aLeadK, 0.) - FAR_LEAD_K * a_req, FAR_LEAD_MIN_DECEL)
  return floor, w


# ars510: ease off early like an experienced driver (10/30 15:04: the camera saw a car 120 m ahead closing at 7 m/s,
# the MPC kept 107 km/h for 2 s more and then braked -0.5 m/s^2 for 12 s; about -0.35 from the first sight was enough).
# While a slower lead is far, acceleration is capped at FAR_EARLY_K x the constant deceleration that removes the closing
# speed before the comfortable gap (the same a_req as the floor above), so the car stops accelerating and starts a gentle
# coast-like slow-down as soon as the lead is seen instead of waiting for the MPC. The cap is reached at FAR_EARLY_JERK
# (no step), never asks more than FAR_EARLY_MAX_DECEL (camera-only lead: FAR_EARLY_MAX_DECEL_VISION), and fades out
# like the floor (50-70 m, 5-7 s). Same off file as the floor.
FAR_EARLY_K = 1.0
FAR_EARLY_JERK = 0.5                # m/s^3
FAR_EARLY_MAX_DECEL = -0.8          # m/s^2, radar lead
FAR_EARLY_MAX_DECEL_VISION = -0.4   # m/s^2, camera-only lead (its closing speed is often underestimated far away)
# Before openpilot has any lead, a radar object-list car the ARS510 interface reports 120+ m ahead in the ego lane and
# closing (radarTracks point FAR_OBJ_TRACK_ID, see ars510_radar_interface.py) eases off the same way but only down to
# FAR_RADAR_ONLY_MAX_DECEL: a coast. 15:04 that car was there 1.5 s before the camera's lead. Those objects are often
# never confirmed (4 of 6 on 10/23 + 10/30), so it stays a coast: a wrong one costs a second or two of lifting off.
FAR_OBJ_TRACK_ID = 1 << 28
FAR_RADAR_ONLY_MAX_DECEL = -0.3     # m/s^2
FAR_OBJ_HOLD = 10                   # planner cycles (0.5 s) the far object is kept after its last report
# 80-120 m: the radar's own ACC target (the car it holds as in-lane; acc_summaries publishes it with trackId < 64) is the
# radar-only source when openpilot has no lead (camera late at night / in rain). Beyond that the object-list far object.
FAR_ACC_TRACK_ID_MAX = 64
FAR_ACC_MIN_D = 80.                 # m
FAR_RADAR_ONLY_MIN_V = 70. / 3.6    # m/s, both radar-only sources
# 10/30 refinements: the cap also RISES at FAR_EARLY_JERK_UP (no throttle blip when the camera lead briefly reports a
# too-small closing speed between the radar object and the radar lead); a camera-only lead takes the closing speed of a
# radar object within FAR_FUSE_D_M of it; the radar object must lie within FAR_OBJ_PATH_Y_M of the model's path at its
# range (curves, FAR_OBJ_PATH_Y_*); a barely-slower lead far away in time still allows some acceleration (FAR_EARLY_ALLOW over TTC).
FAR_EARLY_JERK_UP = 0.3             # m/s^3
FAR_FUSE_D_M = 15.                  # m
FAR_OBJ_PATH_Y_BP = [80., 150.]     # m, dRel
FAR_OBJ_PATH_Y_V = [2.2, 3.0]       # m, |yRel - model path y at dRel|; the path's lateral error grows with range
FAR_EARLY_ALLOW_TTC_BP = [15., 40.]  # s, time to reach the lead
FAR_EARLY_ALLOW_V = [0., 0.4]       # m/s^2 of acceleration still allowed


def far_lead_accel_ceil(lead, t_follow):
  """Highest acceleration allowed for a far, slower lead, and how much of it applies (0..1). None when not applicable."""
  if not lead.present:
    return None
  closing = -lead.vRel
  if closing < FAR_LEAD_MIN_CLOSING:
    return None
  ttc = lead.dRel / closing
  w = float(np.interp(lead.dRel, FAR_LEAD_D_BP, [0., 1.]) * np.interp(ttc, FAR_LEAD_TTC_BP, [0., 1.]))
  if w <= 0.:
    return None
  gap = lead.dRel - (t_follow * max(lead.vLead, 0.) + STOP_DISTANCE + FAR_LEAD_MARGIN)
  if gap <= 1.:
    return None
  a_req = closing ** 2 / (2. * gap)
  ceil = min(lead.aLeadK, 0.) - FAR_EARLY_K * a_req + float(np.interp(ttc, FAR_EARLY_ALLOW_TTC_BP, FAR_EARLY_ALLOW_V))
  return max(ceil, FAR_EARLY_MAX_DECEL if lead.radar else FAR_EARLY_MAX_DECEL_VISION), w


def fuse_far_object(lead, fo):
  """A camera-only lead with a radar far object at about its range: the lead, with the radar's closing speed if faster."""
  if fo is None or not lead.present or lead.radar or abs(fo.dRel - lead.dRel) > FAR_FUSE_D_M or fo.vRel >= lead.vRel:
    return lead
  return types.SimpleNamespace(present=True, radar=False, dRel=lead.dRel, vRel=fo.vRel, vLead=fo.vLead, aLeadK=min(lead.aLeadK, 0.))


def get_cruise_accel(v_cruise, v_ego, a_cruise_prev, dt):
  a_min = float(np.interp(v_ego - v_cruise, A_CRUISE_MIN_BP, A_CRUISE_MIN_V))
  target_accel = np.clip(v_cruise - v_ego, a_min, ACCEL_MAX)
  j_cruise = np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.accel_boost = AccelBoost()

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False
    self.turning = False  # ars510: wheel turned far (junction turn), see TURN_NO_ACCEL_ANGLE
    self.far_lead_active = False  # ars510: far-lead braking limit in effect, see FAR_LEAD_K
    self.far_lead_off = os.path.exists(FAR_LEAD_OFF_FILE)
    self.far_lead_cnt = 0
    self.far_early_cap: float | None = None  # ars510: current early ease-off cap (ramped), see FAR_EARLY_K
    self.far_early_active: str | None = None
    self.far_obj = None  # ars510: radar-only far object (see far_object)
    self.far_obj_age = 0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    if self.CP.openpilotLongitudinalControl:
      self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    else:
      self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    self.a_cruise = get_cruise_accel(v_cruise, v_ego, self.a_cruise, self.dt)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    output_a_target_e2e = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop),
                  (output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e)]

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)

    # ars510: limit braking for a far radar lead (see FAR_LEAD_K); only when the MPC's lead0 decides and no FCW
    self.far_lead_cnt += 1
    if self.far_lead_cnt >= 20:
      self.far_lead_cnt = 0
      self.far_lead_off = os.path.exists(FAR_LEAD_OFF_FILE)
    far_active = False
    if self.CP.openpilotLongitudinalControl and not self.far_lead_off and not self.fcw and \
       self.mpc.source == LongitudinalPlanSource.lead0 and self.mpc.crash_cnt == 0:
      res = far_lead_accel_floor(sm['radarState'].leadOne, get_T_FOLLOW(sm['selfdriveState'].personality))
      if res is not None:
        floor, w = res
        if output_a_target < floor:
          output_a_target = output_a_target + w * (floor - output_a_target)
          far_active = True
    if far_active != self.far_lead_active:
      self.far_lead_active = far_active
      lead = sm['radarState'].leadOne
      cloudlog.info(f"ARS510 far lead soft {'on' if far_active else 'off'} d={lead.dRel:.1f} vRel={lead.vRel:.2f} a={output_a_target:.2f}")

    # ars510: ease off early for a far, slower lead (see FAR_EARLY_K); any plan source, not while the driver controls
    early = None
    radar_only = False; fo = None
    if self.CP.openpilotLongitudinalControl and not self.far_lead_off and not reset_state and not self.fcw:
      t_follow = get_T_FOLLOW(sm['selfdriveState'].personality)
      lead_one = sm['radarState'].leadOne
      fo = self.far_object(sm, v_ego)
      res = far_lead_accel_ceil(fuse_far_object(lead_one, fo), t_follow)
      if res is None and not lead_one.present and fo is not None:
        res = far_lead_accel_ceil(fo, t_follow)
        if res is not None:
          res = (max(res[0], FAR_RADAR_ONLY_MAX_DECEL), res[1])
          radar_only = True
      if res is not None:
        ceil, w = res
        target = output_a_target + w * (ceil - output_a_target) if output_a_target > ceil else output_a_target
        if self.far_early_cap is None:
          self.far_early_cap = max(a_prev, target)
        self.far_early_cap = float(np.clip(target, self.far_early_cap - FAR_EARLY_JERK * self.dt, self.far_early_cap + FAR_EARLY_JERK_UP * self.dt))
        if output_a_target > self.far_early_cap:
          output_a_target = self.far_early_cap
          early = ceil
    if early is None:
      self.far_early_cap = None
    state = None if early is None else ("radar-only" if radar_only else "lead")
    if state != self.far_early_active:
      self.far_early_active = state
      lead = sm['radarState'].leadOne
      fo_s = f" {fo.src} d={fo.dRel:.1f} vRel={fo.vRel:.2f} pathOff={fo.path_off:.1f}" if (radar_only and fo is not None) else ""
      cloudlog.info(f"ARS510 far lead early {state or 'off'} d={lead.dRel:.1f} vRel={lead.vRel:.2f} radar={lead.radar} "
                    f"a={output_a_target:.2f}{fo_s}")

    # ars510: hold speed (coast) instead of accelerating while signalling at low speed or turning at a junction
    CS = sm['carState']
    steer_angle = abs(CS.steeringAngleDeg)
    if steer_angle > TURN_NO_ACCEL_ANGLE:
      self.turning = True
    elif steer_angle < TURN_NO_ACCEL_EXIT_ANGLE:
      self.turning = False
    if ((CS.leftBlinker or CS.rightBlinker) and v_ego < BLINKER_NO_ACCEL_V) or \
       (self.turning and v_ego > TURN_NO_ACCEL_MIN_V):
      output_a_target = min(output_a_target, 0.0)

    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def far_object(self, sm, v_ego):
    """ars510: the radar-only far car from radarTracks, held FAR_OBJ_HOLD cycles, or None: the object-list far object
    (FAR_OBJ_TRACK_ID, 120 m+) or, failing that, the radar's ACC target (trackId < FAR_ACC_TRACK_ID_MAX) at FAR_ACC_MIN_D+.
    Both must lie near the model's path at their range."""
    if 'radarTracks' in sm.data and sm.updated['radarTracks'] and v_ego > FAR_RADAR_ONLY_MIN_V:
      pts = sm['radarTracks'].points
      cands = [p for p in pts if p.trackId == FAR_OBJ_TRACK_ID] or \
              [p for p in pts if p.trackId < FAR_ACC_TRACK_ID_MAX and p.dRel >= FAR_ACC_MIN_D and p.vRel < -FAR_LEAD_MIN_CLOSING]
      if cands:
        p = cands[0]
        pos = sm['modelV2'].position
        path_y = float(np.interp(p.dRel, pos.x, pos.y)) if len(pos.x) > 1 and p.dRel <= pos.x[-1] else 0.
        if abs(p.yRel - path_y) < float(np.interp(p.dRel, FAR_OBJ_PATH_Y_BP, FAR_OBJ_PATH_Y_V)):
          self.far_obj = types.SimpleNamespace(present=True, radar=True, dRel=float(p.dRel), vRel=float(p.vRel),
                                               vLead=float(v_ego + p.vRel), aLeadK=0., path_off=float(p.yRel - path_y),
                                               src="obj" if p.trackId == FAR_OBJ_TRACK_ID else "acc")
          self.far_obj_age = 0
    if self.far_obj is not None:
      self.far_obj_age += 1
      if self.far_obj_age > FAR_OBJ_HOLD:
        self.far_obj = None
    return self.far_obj

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
