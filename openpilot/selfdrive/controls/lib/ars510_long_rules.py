"""ars510 longitudinal rules: everything the ARS510 fork adds on top of the stock planner's acceleration, in one place.

The planner (longitudinal_planner.py) computes the stock candidates (MPC, cruise, model e2e), takes the lowest, then runs
RULES below in this fixed order; each rule gets the acceleration so far and returns a new one. The last rule that changed
the value is the "decider" (LongitudinalPlanner.decider, cloudlog "ARS510 decider ..." when it changes and holds 0.5 s).

  #  name        kind           what it may do                                             off / on file (/data/...)
  0  e2e_fill    candidate      raise the model's candidate while it is indifferent         ars510_no_e2e_fill
                                (still goes through the min with MPC / cruise)
  1  far_soft    SOFTENS        less braking for a far, newly seen radar lead (lead0 only)  ars510_no_far_lead_soft
  2  far_early   cap            ease off early for a far slower lead / radar-only object    (same as far_soft)
  3  near_stop   brake floor    firm enough to stop 4.5 m behind a stopped radar lead       ars510_no_near_stop
  4  lead_stop   brake floor    even braking to 4.5 m behind a lead slowing to a stop       ars510_no_lead_stop
  5  ci_confirm  cap >= -0.3    lift-off for a confirmed cut-in                             ars510_no_cutin_coast
  6  cutin_resp  cap >= -1.0    lift-off / light braking for a merging car                  ON only with ars510_cutin_predict
  7  hazard      cap = -0.5     lift-off for the radar's fast-closing hazard candidate      ars510_no_hazard_coast
  8  turn_hold   cap = 0        no acceleration while signalling < 50 km/h or turning
  9  a_pos       cap >= 0       unhurried positive acceleration (speed, set speed, lead)    ars510_accel_scale (0 = off)

Invariants (checked by the arbiter):
  * only far_soft may make braking lighter; it runs before the floors and caps, so a floor / cap always wins over it.
  * every other rule only lowers the acceleration (min); two rules never add up, the firmest one decides.
  * each rule ramps its own onset (its *_JERK constants); the release of the lift-off caps is ramped by the arbiter
    (RELEASE_RULES, RELEASE_JERK).
Off files are polled once a second (every POLL_CYCLES).

Shared distances: STOP_DISTANCE (6 m, long_mpc) is the MPC's following stop distance; the low-speed stop gap behind a
stopped / stopping lead is STOP_GAP_LOW (4.5 m, = long_mpc get_stop_distance at <= 10 m/s), used by near_stop and
lead_stop (ars510_lead_stop.GAP).
"""
import os
import types
import numpy as np

from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalPlanSource, STOP_DISTANCE
from openpilot.selfdrive.controls.lib.ars510_ego_lc import EgoLaneChange
from openpilot.selfdrive.controls.lib import ars510_cutin
from openpilot.selfdrive.controls.lib import ars510_lead_stop

# 11/10: the lift-off caps of far_early / ci_confirm / hazard used to vanish in one step when their condition ended (plan
# from e.g. -0.5 straight back to the MPC's value). The arbiter now lets the value rise from the last capped one at
# RELEASE_JERK until it meets the plan again (cutin_resp already did this itself). Only ever slows a rise.
RELEASE_RULES = ("far_early", "ci_confirm", "hazard")
RELEASE_JERK = 1.0        # m/s^3
POLL_CYCLES = 20          # off / on files and the accel scale are re-read once a second
DECIDER_HOLD = 10         # cycles a new decider must hold before it is logged
STOP_GAP_LOW = 4.5        # m, see above

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
FAR_RADAR_ONLY_MAX_CLOSING = 12.    # m/s
# cut-in coast (10/30): a radar object-list car (radarTracks CUTIN_TRACK_ID, see ars510_radar_interface.py) 25-90 m ahead,
# CUTIN_PATH_Y_M off the model's path (partly in the lane, not centred: that is the lead), slower by 2 m/s+, held CUTIN_HOLD
# cycles, above CUTIN_MIN_V: lift off to at most CUTIN_MAX_DECEL. 35 highway minutes: 12 of 16 such cars did cut in, 0-8 s
# later (median ~3 s); the 4 others cost a few seconds of coasting. The radar's own lane-crossing flag never fired.
E2E_FILL_MIN_V = 40. / 3.6          # m/s
E2E_FILL_INDIFFERENT = -0.15        # m/s^2: the model asking for less than this wants to slow -> untouched
E2E_FILL_K = 0.07                   # m/s^2 per m/s of speed deficit (12 km/h short -> 0.23)
E2E_FILL_A_MAX_BP = [15., 25., 35.]  # m/s
E2E_FILL_A_MAX_V = [0.3, 0.22, 0.18]  # m/s^2, unhurried (10/30 evening: 0.4/0.3/0.25 still felt too strong)
E2E_FILL_JERK = 0.2                 # m/s^3
# 11/10: the fill used to be dropped in one step when the model turned to slowing (below E2E_FILL_INDIFFERENT) or the set
# speed changed: 85 releases in 177 min, 11 with a plan step > 0.2 m/s^2, largest -0.41 (tools/ars510/mac/analysis/
# rule_toggles.py). It now ramps down at E2E_FILL_RELEASE_JERK. No ramp when the model asks for E2E_FILL_RELEASE_FLOOR or
# harder, or plans a stop: real braking is never delayed (the MPC lead candidate is in the min regardless).
E2E_FILL_RELEASE_JERK = 1.0         # m/s^3
E2E_FILL_RELEASE_FLOOR = -0.5       # m/s^2
E2E_FILL_LOG_HOLD = 20              # planner cycles (1 s) the fill must stay off before "off" is logged
E2E_FILL_OFF_FILE = "/data/ars510_no_e2e_fill"
# positive acceleration ceiling (see the block before the final clip)
# ars510: near-stop floor (10/31 11:51, red light behind a stopping car from 51 km/h: the MPC / model eased the request
# from -1.3 to -1.0 at 7 m and the PCM delivered less, the car stopped 2.3 m behind the lead and then crept to 1.8 m at a
# -0.5 hold; the low-speed stop distance is 4.5 m). While a radar lead within NEAR_STOP_D_MAX is stopped or nearly so,
# braking is floored at NEAR_STOP_K x the constant deceleration that stops the car NEAR_STOP_GAP behind it (never more
# than NEAR_STOP_MAX_DECEL). Closer than NEAR_STOP_TOO_CLOSE a firm NEAR_STOP_HOLD_ACCEL is requested even when the wheel
# speed reads 0 (Toyota wheel speed reads 0 while still creeping), which longcontrol follows as an urgent plan.
# Off: /data/ars510_no_near_stop.
NEAR_STOP_D_MAX = 20.          # m
NEAR_STOP_GAP = STOP_GAP_LOW   # m, = ars510_lead_stop.GAP
NEAR_STOP_VLEAD_MAX = 1.0      # m/s, lead (nearly) stopped
NEAR_STOP_MIN_V = 0.3          # m/s
NEAR_STOP_K = 1.15             # margin for actuator lag / PCM under-delivery
NEAR_STOP_MAX_DECEL = -2.0     # m/s^2
NEAR_STOP_TOO_CLOSE = 3.5      # m
NEAR_STOP_HOLD_ACCEL = -1.5    # m/s^2
NEAR_STOP_HOLD_VLEAD = 0.3     # m/s, the hold only behind a lead that has really stopped (not a crawling jam)
NEAR_STOP_OFF_FILE = "/data/ars510_no_near_stop"


def near_stop_accel_floor(lead, v_ego):
  """Braking floor for a (nearly) stopped radar lead close ahead, or None."""
  if not (lead.present and lead.radar) or lead.dRel > NEAR_STOP_D_MAX or lead.vLead > NEAR_STOP_VLEAD_MAX:
    return None
  if lead.dRel < NEAR_STOP_TOO_CLOSE and lead.vLead < NEAR_STOP_HOLD_VLEAD:
    return NEAR_STOP_HOLD_ACCEL
  if v_ego < NEAR_STOP_MIN_V:
    return None
  room = max(lead.dRel - NEAR_STOP_GAP, 0.5)
  return max(-NEAR_STOP_K * v_ego ** 2 / (2. * room), NEAR_STOP_MAX_DECEL)


# ars510: confirmed cut-in lift-off (10/31 driver: when a car has really cut in and is slower, ease off like a driver would).
# A radar track that was CONFIRM_FROM m or more off the model path within CONFIRM_WIN s and is now within CONFIRM_OFF of it,
# CONFIRM_D ahead and slower by CONFIRM_VREL (curves included since 10/31): acceleration is capped for CONFIRM_HOLD_S at a gentle lift-off,
# CONFIRM_K x closing speed down to CONFIRM_MAX_DECEL, reached at CONFIRM_JERK. Never brakes harder; the MPC does that.
# Same rule as the UI's flashing circle (ars510_tracks.py). Replay 241 km: 66 confirmations, 91 % really in my lane within
# 4 s, 2.5 false per 100 km. Off: /data/ars510_no_cutin_coast.
CONFIRM_OFF = 1.3
CONFIRM_D = (10., 60.)
CONFIRM_VREL = -0.5
CONFIRM_FROM = 1.8
CONFIRM_WIN = 3.0
CONFIRM_HOLD_S = 3.0
CONFIRM_K = 0.06                 # (m/s^2) per (m/s) of closing speed
CONFIRM_MAX_DECEL = -0.3         # m/s^2
CONFIRM_JERK = 0.5               # m/s^3
CONFIRM_MIN_V = 20. / 3.6
CONFIRM_SPECIAL_ID = 1 << 26     # HAZARD / CUTIN_TRACK_ID / FAR_OBJ_TRACK_ID / VRU ids are synthetic, not radar cars
CONFIRM_OFF_FILE = "/data/ars510_no_cutin_coast"
# ars510 cut-in response (11/04 driver: "keep my distance from a merging car and get off the throttle sooner; by my speed,
# the distance and its speed, sometimes only a lift-off, sometimes light braking"). The UI's amber-circle rule
# (controls/lib/ars510_cutin.py, the same code) picks the object; ars510_cutin.CutinResponse turns it into the deceleration
# needed to settle at my following distance behind it (closing^2 / 2 x room, less its own relative acceleration): at most a
# lift-off (-0.3) for a prediction, light braking (-1.0) once it is on / past its lane line, moving in fast, or confirmed.
# Harder braking stays with the MPC once the car is the lead. A line-hugging car counts (driver: a hazard too).
# Onset at CR_JERK_DOWN, release at CR_JERK_UP; nothing while the driver presses the gas.
# Until validated on replay and enabled by the driver: on only while /data/ars510_cutin_predict exists.
CR_ON_FILE = "/data/ars510_cutin_predict"
CR_JERK_DOWN = 0.8               # m/s^3
CR_JERK_UP = 1.0                 # m/s^3
CR_STALE_FRAMES = 10             # planner cycles without a radar display record: drop the objects
# ars510: hazard coast (11/01, driver: "at least lift off"). The radar interface's hazard candidate (radarTracks
# HAZARD_TRACK_ID, see ars510_radar_interface.py HAZARD_*: a tracked car closing fast on a short time-to-collision that
# the camera has not made the lead) within HAZARD_PATH_M of the model path, or within HAZARD_CROSS_M and moving toward
# my path faster than HAZARD_CROSS_VY (that check lives in the interface: RadarPoint carries no lateral speed here), for HAZARD_HOLD cycles -> acceleration capped at HAZARD_ACCEL (lift-off, no
# braking: the MPC brakes once radard takes the car), reached at HAZARD_JERK, kept HAZARD_KEEP_S after the candidate
# drops. Off: /data/ars510_no_hazard_coast. cloudlog "ARS510 hazard coast on/off".
HAZARD_TRACK_ID = 1 << 26
HAZARD_CROSS_M = 3.5
HAZARD_HOLD = 10                 # planner cycles (0.5 s)
HAZARD_FAST_TTC_S = 2.0          # closer than this (time-to-collision) the hold is skipped
HAZARD_ACCEL = -0.5
HAZARD_JERK = 1.0
HAZARD_KEEP_S = 1.0
HAZARD_OFF_FILE = "/data/ars510_no_hazard_coast"
# 11/01: no confirmation while I change lanes myself (ars510_ego_lc.py): the cars of the lane I move into are not cutting in


A_POS_MAX_BP = [0., 10., 20., 30., 35.]     # m/s
A_POS_MAX_V = [1.2, 0.8, 0.45, 0.28, 0.22]  # m/s^2
A_POS_JERK_BP = [10., 25.]                  # m/s
A_POS_JERK_V = [0.5, 0.15]                  # m/s^3, how fast the ceiling lets acceleration build (10/30: 0.25 -> 0.15, PCM still overshot 0.26 -> 0.80 for a second)
ACCEL_SCALE_FILE = "/data/ars510_accel_scale"  # 0.3-1.0 multiplier, 0 disables the ceiling; default 1.0
A_POS_TAU = 10.                            # s, time constant of the approach to the set speed (deficit shrinks to 37 % per tau)
# ars510 (10/31 driver: "like a magnet" -- with a lead pulling away the MPC accelerated to win the gap back, caught up
# in no time and then had to brake). With a lead ahead, positive acceleration is also capped by a speed-matching law,
# like the set-speed approach above: a <= (vLead - v) / LEAD_TAU(v) + LEAD_A_GAIN x max(aLead, 0) + a slow gap term, so
# the car only matches the lead's speed and lets a widening gap close slowly instead of chasing it. Braking is untouched.
LEAD_TAU_BP = [0., 8., 20.]                # m/s
LEAD_TAU_V = [3., 6., 10.]                 # s: quicker at a light (lead pulling away from a stop), unhurried at speed
LEAD_A_GAIN = 0.5                          # share of the lead's own acceleration that is followed
LEAD_GAP_K = 0.003                         # m/s^2 per m of gap beyond the desired one ...
LEAD_GAP_MAX = 0.1                         # m/s^2 ... at most (a 100 m gap at equal speed closes in about a minute)
LEAD_MAX_D = 150.                          # m


def read_accel_scale() -> float:
  try:
    with open(ACCEL_SCALE_FILE) as f:
      v = float(f.read().strip())
    return 0. if v <= 0. else float(np.clip(v, 0.3, 1.0))
  except Exception:
    return 1.0
CUTIN_TRACK_ID = 1 << 27
CUTIN_PATH_Y_M = (1.0, 2.6)
CUTIN_HOLD = 20                     # planner cycles (1 s)
CUTIN_MIN_V = 60. / 3.6
CUTIN_MAX_DECEL = -0.3
# 10/30 19:23-20:16 (54 min): 13 cut-in coasts, 1 real cut-in; the rest were adjacent-lane cars hugging the line (up to
# 6 s / -8 km/h). Logging only until a discriminator (lateral motion, duration) is found in the reports' clips.
CUTIN_COAST_ENABLED = False
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


def cutin_accel_ceil(obj, t_follow):
  """Coast cap for a cut-in candidate: the constant deceleration that would settle behind it, never below CUTIN_MAX_DECEL."""
  closing = -obj.vRel
  if closing < 2.0:
    return None
  gap = obj.dRel - (t_follow * max(obj.vLead, 0.) + STOP_DISTANCE + FAR_LEAD_MARGIN)
  a_req = closing ** 2 / (2. * gap) if gap > 1. else -CUTIN_MAX_DECEL
  return max(-a_req, CUTIN_MAX_DECEL), 1.0


def fuse_far_object(lead, fo):
  """A camera-only lead with a radar far object at about its range (FAR_FUSE_D_M or 15 % of range, the camera's
  distance error grows with range): the lead with the radar's closing speed if faster, and treated as radar-confirmed."""
  if fo is None or not lead.present or lead.radar or abs(fo.dRel - lead.dRel) > max(FAR_FUSE_D_M, 0.15 * lead.dRel) or fo.vRel >= lead.vRel:
    return lead
  return types.SimpleNamespace(present=True, radar=True, dRel=lead.dRel, vRel=fo.vRel, vLead=fo.vLead, aLeadK=min(lead.aLeadK, 0.), fused=True)




# ---------------------------------------------------------------------------------------------------------------------
# rules. Each: name, apply(c, a) -> new a (c = Ctx). State lives in the rule. Order and kinds: see the module docstring.

class Ctx:
  """Per-cycle inputs shared by the rules."""
  __slots__ = ("sm", "long", "v_ego", "v_cruise", "reset_state", "a_prev", "t_follow", "now", "fcw", "mpc", "dt", "poll")

  def __init__(self, **kw):
    for k in self.__slots__:
      setattr(self, k, kw.get(k))


class Rule:
  name = "rule"
  softens = False   # True only for far_soft

  def poll(self):
    pass

  def apply(self, c: Ctx, a: float) -> float:
    return a


class E2EFill(Rule):
  """0: the model's desired acceleration is always a candidate upstream and often sits near 0 well below the set speed
  (10/30 19:27: 101 km/h, set 113, model +0.00 for 100 s, speed lost on a 2 deg climb). When the model is merely
  indifferent (above E2E_FILL_INDIFFERENT, no stop) the e2e candidate is raised to a gentle speed-tracking value,
  E2E_FILL_K x (set speed - speed) capped by E2E_FILL_A_MAX(v), ramped at E2E_FILL_JERK. Whenever the model wants to
  slow (curve, exit, traffic, a sign) it keeps full authority; the MPC lead candidate stays in the min as before, so no
  lead-present switch is needed (the first version switched on "no lead" and surged whenever the camera lead flickered).
  Applied to the e2e candidate before the min (candidate(), not apply())."""
  name = "e2e_fill"

  def __init__(self):
    self.off = os.path.exists(E2E_FILL_OFF_FILE)
    self.active = False
    self.filled = 0.
    self.lifted = False     # last cycle's candidate was above the model's (fill or release ramp)
    self._logged, self._off_n = False, 0

  def poll(self):
    self.off = os.path.exists(E2E_FILL_OFF_FILE)

  def candidate(self, c: Ctx, a_e2e: float, stop_e2e: bool) -> float:
    fill = None
    if c.long and not self.off and not c.reset_state and c.v_ego > E2E_FILL_MIN_V and \
       a_e2e > E2E_FILL_INDIFFERENT and not stop_e2e and c.v_cruise > c.v_ego:
      fill = min(E2E_FILL_K * (c.v_cruise - c.v_ego), float(np.interp(c.v_ego, E2E_FILL_A_MAX_BP, E2E_FILL_A_MAX_V)))
    target = max(a_e2e, fill) if fill is not None else a_e2e
    # ramp: the filled value approaches its target at E2E_FILL_JERK from where the e2e candidate was last cycle
    if fill is not None and target > self.filled:
      target = min(target, self.filled + E2E_FILL_JERK * c.dt)
    # release ramp: what was lifted above the model comes down at E2E_FILL_RELEASE_JERK instead of in one step
    if self.lifted and c.long and not self.off and not c.reset_state and not stop_e2e and a_e2e > E2E_FILL_RELEASE_FLOOR:
      target = max(target, self.filled - E2E_FILL_RELEASE_JERK * c.dt)
    self.filled = target
    self.lifted = target > a_e2e + 1e-6
    active = fill is not None and target > a_e2e + 0.01
    self.active = active
    # log: "on" at once, "off" only after E2E_FILL_LOG_HOLD cycles off (it switched 4 times a minute)
    self._off_n = 0 if active else self._off_n + 1
    if (active and not self._logged) or (self._logged and self._off_n == E2E_FILL_LOG_HOLD):
      self._logged = active
      cloudlog.info(f"ARS510 e2e fill {'on' if active else 'off'} v={c.v_ego*3.6:.0f} set={c.v_cruise*3.6:.0f} e2e={a_e2e:.2f} fill={target:.2f}")
    return target


class FarLeadSoft(Rule):
  """1: limit braking for a far radar lead (see FAR_LEAD_K); only when the MPC's lead0 decides and no FCW. The only
  rule that makes braking lighter."""
  name = "far_soft"
  softens = True

  def __init__(self):
    self.off = os.path.exists(FAR_LEAD_OFF_FILE)
    self.active = False

  def poll(self):
    self.off = os.path.exists(FAR_LEAD_OFF_FILE)

  def apply(self, c, a):
    active = False
    if c.long and not self.off and not c.fcw and c.mpc.source == LongitudinalPlanSource.lead0 and c.mpc.crash_cnt == 0:
      res = far_lead_accel_floor(c.sm['radarState'].leadOne, c.t_follow)
      if res is not None:
        floor, w = res
        if a < floor:
          a = a + w * (floor - a)
          active = True
    if active != self.active:
      self.active = active
      lead = c.sm['radarState'].leadOne
      cloudlog.info(f"ARS510 far lead soft {'on' if active else 'off'} d={lead.dRel:.1f} vRel={lead.vRel:.2f} a={a:.2f}")
    return a


class FarLeadEarly(Rule):
  """2: ease off early for a far, slower lead (see FAR_EARLY_K); any plan source, not while the driver controls.
  Also the radar-only far object (FAR_OBJ_*) and the (disabled, logging only) cut-in coast candidate (CUTIN_*)."""
  name = "far_early"

  def __init__(self):
    self.off = os.path.exists(FAR_LEAD_OFF_FILE)
    self.cap: float | None = None
    self.active: str | None = None
    self.far_obj = None
    self.far_obj_age = 0
    self.cutin = None
    self.cutin_cnt = 0
    self.cutin_logged = False

  def poll(self):
    self.off = os.path.exists(FAR_LEAD_OFF_FILE)

  def apply(self, c, a):
    sm, v_ego = c.sm, c.v_ego
    early = None
    radar_only = False; fo = None
    if c.long and not self.off and not c.reset_state and not c.fcw:
      t_follow = c.t_follow
      lead_one = sm['radarState'].leadOne
      fo = self.far_object(sm, v_ego)
      fused = fuse_far_object(lead_one, fo)
      res = far_lead_accel_ceil(fused, t_follow)
      if fo is not None and not getattr(fused, "fused", False):
        # radar-only coast: no lead, or a lead the radar object does not match (another car)
        res_obj = far_lead_accel_ceil(fo, t_follow)
        if res_obj is not None:
          res_obj = (max(res_obj[0], FAR_RADAR_ONLY_MAX_DECEL), res_obj[1])
          if res is None or res_obj[0] * res_obj[1] < res[0] * res[1]:
            res = res_obj
            radar_only = True
      ci = self.cutin_object(sm, v_ego, lead_one)
      if (ci is not None) != self.cutin_logged:
        self.cutin_logged = ci is not None
        if ci is not None:
          cloudlog.info(f"ARS510 cut-in candidate on d={ci.dRel:.1f} vRel={ci.vRel:.2f} pathOff={ci.path_off:.1f} v={v_ego*3.6:.0f}")
        else:
          cloudlog.info("ARS510 cut-in candidate off")
      if ci is not None and CUTIN_COAST_ENABLED:
        res_ci = cutin_accel_ceil(ci, t_follow)
        if res_ci is not None and (res is None or res_ci[0] * res_ci[1] < res[0] * res[1]):
          res = res_ci
          radar_only = True
          fo = ci
      if res is not None:
        ceil, w = res
        target = a + w * (ceil - a) if a > ceil else a
        if self.cap is None:
          self.cap = max(c.a_prev, target)
        self.cap = float(np.clip(target, self.cap - FAR_EARLY_JERK * c.dt, self.cap + FAR_EARLY_JERK_UP * c.dt))
        if a > self.cap:
          a = self.cap
          early = ceil
    else:
      self.far_obj, self.far_obj_age = None, 0   # 11/10: nothing kept across a pause (it was not aged while paused)
    if early is None:
      self.cap = None
    state = None if early is None else (getattr(fo, "src", "radar-only") if radar_only else "lead")
    if state != self.active:
      self.active = state
      lead = sm['radarState'].leadOne
      fo_s = f" {fo.src} d={fo.dRel:.1f} vRel={fo.vRel:.2f} pathOff={fo.path_off:.1f}" if (radar_only and fo is not None) else ""
      cloudlog.info(f"ARS510 far lead early {state or 'off'} d={lead.dRel:.1f} vRel={lead.vRel:.2f} radar={lead.radar} "
                    f"a={a:.2f}{fo_s}")
    return a

  def far_object(self, sm, v_ego):
    """The radar-only far car from radarTracks, held FAR_OBJ_HOLD cycles, or None: the object-list far object
    (FAR_OBJ_TRACK_ID, 120 m+) or, failing that, the radar's ACC target (trackId < FAR_ACC_TRACK_ID_MAX) at FAR_ACC_MIN_D+.
    Both must lie near the model's path at their range."""
    if 'radarTracks' in sm.data and sm.updated['radarTracks'] and v_ego > FAR_RADAR_ONLY_MIN_V:
      pts = sm['radarTracks'].points
      # closing faster than FAR_RADAR_ONLY_MAX_CLOSING is not a slower car (oncoming / roadside / stopped: the far-stopped-car
      # path and the camera handle those) and is left alone (10/30 20:xx: 124 m, -16.9 m/s, gone 2 s later)
      cands = [p for p in pts if p.trackId == FAR_OBJ_TRACK_ID and -p.vRel <= FAR_RADAR_ONLY_MAX_CLOSING] or \
              [p for p in pts if p.trackId < FAR_ACC_TRACK_ID_MAX and p.dRel >= FAR_ACC_MIN_D and -FAR_LEAD_MIN_CLOSING >= p.vRel >= -FAR_RADAR_ONLY_MAX_CLOSING]
      if cands:
        p = cands[0]
        pos = sm['modelV2'].position
        path_y = float(np.interp(p.dRel, pos.x, pos.y)) if len(pos.x) > 1 and p.dRel <= pos.x[-1] else 0.
        # 11/10: radar yRel is left-positive, model y right-positive (as in radard / ci_confirm / hazard); this used
        # yRel - path_y, which in a curve dropped in-path cars and could accept the adjacent lane's
        path_off = -float(p.yRel) - path_y
        if abs(path_off) < float(np.interp(p.dRel, FAR_OBJ_PATH_Y_BP, FAR_OBJ_PATH_Y_V)):
          self.far_obj = types.SimpleNamespace(present=True, radar=True, dRel=float(p.dRel), vRel=float(p.vRel),
                                               vLead=float(v_ego + p.vRel), aLeadK=0., path_off=path_off,
                                               src="obj" if p.trackId == FAR_OBJ_TRACK_ID else "acc")
          self.far_obj_age = 0
    if self.far_obj is not None:
      self.far_obj_age += 1
      if self.far_obj_age > FAR_OBJ_HOLD:
        self.far_obj = None
    return self.far_obj

  def cutin_object(self, sm, v_ego, lead):
    """The cut-in candidate (CUTIN_TRACK_ID) once it has sat CUTIN_PATH_Y_M off the model path for CUTIN_HOLD cycles."""
    ok = False
    if 'radarTracks' in sm.data and sm.updated['radarTracks'] and v_ego > CUTIN_MIN_V:
      pts = [p for p in sm['radarTracks'].points if p.trackId == CUTIN_TRACK_ID]
      if pts:
        p = pts[0]
        pos = sm['modelV2'].position
        path_y = float(np.interp(p.dRel, pos.x, pos.y)) if len(pos.x) > 1 and p.dRel <= pos.x[-1] else 0.
        off = abs(-float(p.yRel) - path_y)   # 11/10: sign as in far_object
        closer_than_lead = not lead.present or p.dRel < lead.dRel - 6.  # a car beyond the lead cannot cut in front of us
        if CUTIN_PATH_Y_M[0] < off < CUTIN_PATH_Y_M[1] and closer_than_lead:
          ok = True
          self.cutin_cnt += 1
          self.cutin = types.SimpleNamespace(present=True, radar=True, dRel=float(p.dRel), vRel=float(p.vRel),
                                             vLead=float(v_ego + p.vRel), aLeadK=0., path_off=-float(p.yRel) - path_y, src="cut-in")
    if not ok:
      self.cutin_cnt = max(self.cutin_cnt - 2, 0)  # a missed cycle or two does not reset the hold
      if self.cutin_cnt == 0:
        self.cutin = None
    return self.cutin if self.cutin_cnt >= CUTIN_HOLD else None


class NearStop(Rule):
  """3: near-stop floor (see NEAR_STOP_K); applies whatever the source."""
  name = "near_stop"

  def __init__(self):
    self.off = os.path.exists(NEAR_STOP_OFF_FILE)
    self.active = False

  def poll(self):
    self.off = os.path.exists(NEAR_STOP_OFF_FILE)

  def apply(self, c, a):
    active = False
    if c.long and not self.off and not c.reset_state:
      ns = near_stop_accel_floor(c.sm['radarState'].leadOne, c.v_ego)
      if ns is not None and a > ns:
        a = ns
        active = True
    if active != self.active:
      self.active = active
      lead = c.sm['radarState'].leadOne
      cloudlog.info(f"ARS510 near stop {'on' if active else 'off'} d={lead.dRel:.1f} vLead={lead.vLead:.1f} v={c.v_ego*3.6:.1f} a={a:.2f}")
    return a


class LeadStop(Rule):
  """4: lead-stop floor (see ars510_lead_stop.py)."""
  name = "lead_stop"

  def __init__(self, dt):
    self.off = os.path.exists(ars510_lead_stop.OFF_FILE)
    self.f = ars510_lead_stop.LeadStopFloor(dt)

  def poll(self):
    self.off = os.path.exists(ars510_lead_stop.OFF_FILE)

  def apply(self, c, a):
    was = self.f.active
    lead = c.sm['radarState'].leadOne
    if c.long and not self.off and not c.reset_state:
      ls = self.f.update(lead, c.v_ego, c.sm.logMonoTime['radarState'] / 1e9, a)
      if ls is not None and a > ls:
        a = ls
    else:
      self.f.reset()
    if self.f.active != was:
      a_lead, x_stop = self.f.dbg or (0., 0.)
      cloudlog.info(f"ARS510 lead stop {'on' if self.f.active else 'off'} d={lead.dRel:.1f} vLead={lead.vLead:.1f} "
                    f"aLeadK={lead.aLeadK:.2f} a_lead={a_lead:.2f} x_stop={x_stop:.1f} v={c.v_ego*3.6:.1f} a={a:.2f}")
    return a


class CutinConfirm(Rule):
  """5: confirmed cut-in lift-off (see CONFIRM_*). Detection runs while engaged or not (track history)."""
  name = "ci_confirm"

  def __init__(self):
    self.off = os.path.exists(CONFIRM_OFF_FILE)
    self.out_t: dict[int, float] = {}   # track id -> last time far off the path
    self.ego_lc = EgoLaneChange()
    self.until = 0.                      # coast while time < this
    self.closing = 0.
    self.cap = None

  def poll(self):
    self.off = os.path.exists(CONFIRM_OFF_FILE)

  def apply(self, c, a):
    sm, now = c.sm, c.now
    if c.long and not self.off and 'radarTracks' in sm.data and c.v_ego > CONFIRM_MIN_V:
      px = np.array(sm['modelV2'].position.x); py = np.array(sm['modelV2'].position.y)
      curve = len(px) < 2  # 10/31: curves handled too (replay: 98 vs 66 confirmations, still 91 % real)
      if self.ego_lc.update(sm['carState'], sm['modelV2'], now):
        curve = True   # my own lane change: track the out-of-path times but confirm nothing
      if len(px) >= 2 and sm.updated['radarTracks']:
        for pnt in sm['radarTracks'].points:
          tid = int(pnt.trackId)
          if tid >= CONFIRM_SPECIAL_ID or tid < 0:
            continue
          off = abs(-float(pnt.yRel) - float(np.interp(pnt.dRel, px, py)))
          if off >= CONFIRM_FROM:
            self.out_t[tid] = now
          elif (not curve and off < CONFIRM_OFF and CONFIRM_D[0] < pnt.dRel < CONFIRM_D[1] and pnt.vRel < CONFIRM_VREL and
                c.v_ego + pnt.vRel > 2.0 and now - self.out_t.get(tid, -99.) < CONFIRM_WIN):   # 11/05: moving forward
            if now >= self.until:
              cloudlog.info(f"ARS510 cut-in confirmed: d={pnt.dRel:.1f} vRel={pnt.vRel:.1f} v={c.v_ego*3.6:.0f}")
            self.until = now + CONFIRM_HOLD_S
            self.closing = max(-float(pnt.vRel), 0.)
            self.out_t.pop(tid, None)
        if len(self.out_t) > 200:
          self.out_t = {k: t for k, t in self.out_t.items() if now - t < 10.}
    if now < self.until and not c.reset_state:
      target = max(-CONFIRM_K * self.closing, CONFIRM_MAX_DECEL)
      cap = max(c.a_prev, target) if self.cap is None else max(target, self.cap - CONFIRM_JERK * c.dt)
      self.cap = cap
      a = min(a, cap)
    else:
      self.cap = None
    return a


class CutinResponse(Rule):
  """6: cut-in response (see CR_* and ars510_cutin.py); on only while CR_ON_FILE exists."""
  name = "cutin_resp"

  def __init__(self):
    self.off = not os.path.exists(CR_ON_FILE)
    self.rule = ars510_cutin.CutinRule()
    self.resp = ars510_cutin.CutinResponse()
    self.points: list = []
    self.frame = 0
    self.cap = None
    self.on = False
    self.cam = 0.   # the UI's camera offset (CameraOffset, with a sunnypilot model bundle), read once
    try:
      from openpilot.common.params import Params
      from openpilot.sunnypilot.models.helpers import get_active_bundle
      if get_active_bundle():
        self.cam = float(Params().get("CameraOffset", return_default=True) or 0.)
    except Exception:
      pass

  def poll(self):
    self.off = not os.path.exists(CR_ON_FILE)

  def apply(self, c, a):
    sm, v_ego, now = c.sm, c.v_ego, c.now
    target = None
    svc = ars510_cutin.SERVICE
    if c.long and not self.off and not c.reset_state and svc in sm.data and \
       sm['selfdriveState'].enabled and not sm['carState'].gasPressed:
      if sm.updated[svc]:
        self.points = ars510_cutin.parse(bytes(sm[svc]))
      md = sm['modelV2']
      px = np.array(md.position.x); py = np.array(md.position.y) + self.cam
      if sm.frame - sm.recv_frame[svc] <= CR_STALE_FRAMES and self.points and len(px) >= 2:
        self.frame += 1
        objs = self.rule.update(sm['carState'], md, self.points, sm['radarState'], now, self.frame, sm.updated[svc],
                                sm.updated['modelV2'], lambda d: float(np.interp(d, px, py)), self.cam)
        best = self.resp.update(objs, v_ego, c.t_follow, now)
        if best is not None:
          target = -best[0]
          if not self.on:
            o = best[1]
            cloudlog.info(f"ARS510 cut-in response on: {o.kind} d={o.d:.1f} vRel={o.vrel:.1f} line={o.intr:.1f} "
                          f"toward={o.toward:.2f} strict={o.strict} need={best[2]:.2f} cap={target:.2f} v={v_ego*3.6:.0f}")
      else:
        self.rule.reset()
    else:
      self.rule.reset()
    if target is None and self.on:
      cloudlog.info("ARS510 cut-in response off")
    self.on = target is not None
    if target is not None:
      if self.cap is None:
        self.cap = max(c.a_prev, target)
      if self.cap > target:
        self.cap = max(target, self.cap - CR_JERK_DOWN * c.dt)
      else:
        self.cap = min(target, self.cap + CR_JERK_UP * c.dt)
      a = min(a, self.cap)
    elif self.cap is not None:
      self.cap += CR_JERK_UP * c.dt   # release smoothly: no throttle blip
      if self.cap >= a or c.reset_state:
        self.cap = None
      else:
        a = min(a, self.cap)
    return a


class HazardCoast(Rule):
  """7: hazard coast (see HAZARD_*)."""
  name = "hazard"

  def __init__(self):
    self.off = os.path.exists(HAZARD_OFF_FILE)
    self.cnt, self.until, self.cap, self.on = 0, 0., None, False

  def poll(self):
    self.off = os.path.exists(HAZARD_OFF_FILE)

  def apply(self, c, a):
    sm, now = c.sm, c.now
    hz = None
    if c.long and not self.off and 'radarTracks' in sm.data and sm.updated['radarTracks']:
      pos = sm['modelV2'].position
      for p in sm['radarTracks'].points:
        if int(p.trackId) != HAZARD_TRACK_ID:
          continue
        path_y = float(np.interp(p.dRel, pos.x, pos.y)) if len(pos.x) > 1 else 0.
        off = -float(p.yRel) - path_y   # right-positive offset to the model path
        if abs(off) < HAZARD_CROSS_M:   # the interface already required in-lane or moving toward my lane
          hz = p
        break
      self.cnt = self.cnt + 1 if hz is not None else max(self.cnt - 2, 0)
      if hz is not None and hz.vRel < 0 and hz.dRel / -hz.vRel < HAZARD_FAST_TTC_S:
        self.cnt = max(self.cnt, HAZARD_HOLD)
      if self.cnt >= HAZARD_HOLD:
        self.until = now + HAZARD_KEEP_S
        if not self.on:
          cloudlog.info(f"ARS510 hazard coast on d={hz.dRel:.1f} vRel={hz.vRel:.1f} y={hz.yRel:.1f} v={c.v_ego*3.6:.0f}")
          self.on = True
    if now < self.until and not c.reset_state:
      cap = max(c.a_prev, HAZARD_ACCEL) if self.cap is None else max(HAZARD_ACCEL, self.cap - HAZARD_JERK * c.dt)
      self.cap = cap
      a = min(a, cap)
    else:
      self.cap = None
      if self.on:
        cloudlog.info("ARS510 hazard coast off")
        self.on = False
    return a


class TurnHold(Rule):
  """8: hold speed (coast) instead of accelerating while signalling at low speed or turning at a junction."""
  name = "turn_hold"

  def __init__(self):
    self.turning = False

  def apply(self, c, a):
    CS = c.sm['carState']
    steer_angle = abs(CS.steeringAngleDeg)
    if steer_angle > TURN_NO_ACCEL_ANGLE:
      self.turning = True
    elif steer_angle < TURN_NO_ACCEL_EXIT_ANGLE:
      self.turning = False
    if ((CS.leftBlinker or CS.rightBlinker) and c.v_ego < BLINKER_NO_ACCEL_V) or \
       (self.turning and c.v_ego > TURN_NO_ACCEL_MIN_V):
      a = min(a, 0.0)
    return a


class PosAccelCeiling(Rule):
  """9: an unhurried driver's ceiling on positive acceleration, whatever the source (MPC lead pull-away, model, cruise,
  fill): A_POS_MAX(v) x the scale in /data/ars510_accel_scale (0.3-1.0, read once a second, no restart), rising at
  A_POS_JERK. Braking is never touched. 10/30 evening: with a lead pulling away the MPC / model asked +0.8..+1.0 at
  90-100 km/h, which felt harsh; the hybrid also keeps mostly electric below about 0.25 m/s^2 at highway speed."""
  name = "a_pos"

  def __init__(self):
    self.scale = read_accel_scale()
    self.capped = False

  def poll(self):
    self.scale = read_accel_scale()

  def apply(self, c, a):
    if c.long and not c.reset_state and self.scale > 0.:
      v_ego = c.v_ego
      cap = float(np.interp(v_ego, A_POS_MAX_BP, A_POS_MAX_V)) * self.scale
      # first-order approach to the set speed: a <= (v_set - v) / tau, so the speed settles exponentially with no overshoot
      cap = min(cap, max(c.v_cruise - v_ego, 0.) / A_POS_TAU)
      lead = c.sm['radarState'].leadOne
      if lead.present and lead.dRel < LEAD_MAX_D:
        desired = c.t_follow * max(lead.vLead, 0.) + STOP_DISTANCE
        lead_cap = (max(lead.vLead - v_ego, 0.) / float(np.interp(v_ego, LEAD_TAU_BP, LEAD_TAU_V)) +
                    LEAD_A_GAIN * max(lead.aLeadK, 0.) + min(LEAD_GAP_K * max(lead.dRel - desired, 0.), LEAD_GAP_MAX))
        cap = min(cap, lead_cap)
      cap = min(cap, max(c.a_prev, 0.) + float(np.interp(v_ego, A_POS_JERK_BP, A_POS_JERK_V)) * c.dt)
      if a > cap:
        a = cap
        self.capped = True
      elif self.capped and a < cap - 0.05:
        self.capped = False
    return a


class Rules:
  """The arbiter: e2e_fill on the model candidate, then RULES in order; tracks the decider."""

  def __init__(self, dt):
    self.e2e_fill = E2EFill()
    self.rules: list[Rule] = [FarLeadSoft(), FarLeadEarly(), NearStop(), LeadStop(dt), CutinConfirm(), CutinResponse(),
                              HazardCoast(), TurnHold(), PosAccelCeiling()]
    self.by_name = {r.name: r for r in [self.e2e_fill] + self.rules}
    self.cycle = 0
    self.decider = "none"
    self._pending, self._pending_n, self._logged = "none", 0, "none"
    self._rel: dict[str, float] = {}   # rule name -> value its cap is being released from

  def __getitem__(self, name):
    return self.by_name[name]

  def tick(self):
    """Once per planner cycle before anything else: polls the off files once a second."""
    self.cycle += 1
    if self.cycle >= POLL_CYCLES:
      self.cycle = 0
      for r in self.by_name.values():
        r.poll()

  def apply(self, c: Ctx, a: float, base: str) -> float:
    decider = base
    for r in self.rules:
      a2 = r.apply(c, a)
      if not r.softens and a2 > a:   # invariant: only far_soft may lighten braking
        a2 = a
      if r.name in RELEASE_RULES:
        if a2 < a:
          self._rel[r.name] = a2
        elif r.name in self._rel:
          rel = self._rel[r.name] + RELEASE_JERK * c.dt
          if rel >= a or c.reset_state:
            del self._rel[r.name]
          else:
            self._rel[r.name] = a2 = rel
      if a2 != a:
        decider = r.name
      a = a2
    self.decider = decider
    if decider != self._pending:
      self._pending, self._pending_n = decider, 0
    self._pending_n += 1
    if self._pending_n == DECIDER_HOLD and self._pending != self._logged:
      self._logged = self._pending
      cloudlog.info(f"ARS510 decider {decider} a={a:.2f} v={c.v_ego*3.6:.0f}")
    return a
