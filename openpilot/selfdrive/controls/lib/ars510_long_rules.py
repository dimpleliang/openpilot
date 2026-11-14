"""ars510 longitudinal rules: everything the ARS510 fork adds on top of the stock planner's acceleration, in one place.

The planner (longitudinal_planner.py) computes the stock candidates (MPC, cruise, model e2e), takes the lowest, then runs
RULES below in this fixed order; each rule gets the acceleration so far and returns a new one. The last rule that changed
the value is the "decider" (LongitudinalPlanner.decider, cloudlog "ARS510 decider ..." when it changes and holds 0.5 s).

  #  name        kind           what it may do                                             off / on file (/data/...)
  0  e2e_fill    candidate      raise the model's candidate while it is indifferent         ars510_no_e2e_fill
                                (still goes through the min with MPC / cruise)
  1  far_soft    SOFTENS        less braking for a far, newly seen radar lead (lead0 only)  ars510_no_far_lead_soft
  2  far_early   cap            ease off early for a far slower lead / radar-only object    (same as far_soft)
  2b far_vision  brake floor    brake in time for a far, slow / standing camera-only lead   ars510_no_far_vision
  3  near_stop   brake floor    firm enough to stop 4.5 m behind a stopped radar lead       ars510_no_near_stop
  4  lead_stop   brake floor    even braking to 4.5 m behind a lead slowing to a stop       ars510_no_lead_stop
  4b landing     SOFTENS        one brake application fading with the speed over the last    ars510_no_landing
                                metres to a stopped radar lead (the driver's own stop)
  5  ci_confirm  cap >= -0.3    lift-off for a confirmed cut-in                             ars510_no_cutin_coast
  6  cutin_resp  cap >= -1.0    lift-off / light braking for a merging car                  ON only with ars510_cutin_predict
  7  hazard      cap = -0.5     lift-off for the radar's fast-closing hazard candidate      ars510_no_hazard_coast
  8  turn_hold   cap >= 0       gentle start, no acceleration from 20 km/h, while signalling < 50 km/h or turning
  9  a_pos       cap >= 0       unhurried positive acceleration (speed, set speed, lead)    ars510_accel_scale (0 = off)

Invariants (checked by the arbiter):
  * only far_soft and landing may make braking lighter. far_soft runs before the floors and caps, so a floor / cap
    always wins over it; landing shapes the last metres within LAND_URGENT of the plan and gives way to anything firmer.
  * every other rule only lowers the acceleration (min); two rules never add up, the firmest one decides.
  * each rule ramps its own onset (its *_JERK constants); the release of the lift-off caps is ramped by the arbiter
    (RELEASE_RULES, RELEASE_JERK).
Off files are polled once a second (every POLL_CYCLES).
Rain mode (ars510_rain.py, from the wipers) is not a rule: it lengthens t_follow (MPC, far_soft, far_early, cutin_resp),
scales the a_pos ceiling and the e2e fill, and lets far_early ease off harder for a camera-only lead.

Shared distances: STOP_DISTANCE (6 m, long_mpc) is the MPC's following stop distance; the low-speed stop gap behind a
stopped / stopping lead is STOP_GAP_LOW (4.5 m, = long_mpc get_stop_distance at <= 10 m/s), used by near_stop and
lead_stop (ars510_lead_stop.GAP).
"""
import math
import os
import types
import numpy as np

from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalPlanSource, STOP_DISTANCE
from openpilot.selfdrive.controls.lib.ars510_ego_lc import EgoLaneChange
from openpilot.selfdrive.controls.lib import ars510_cutin
from openpilot.selfdrive.controls.lib import ars510_lead_stop
from openpilot.selfdrive.controls.lib import ars510_rain

# 11/10: the lift-off caps of far_early / ci_confirm / hazard used to vanish in one step when their condition ended (plan
# from e.g. -0.5 straight back to the MPC's value). The arbiter now lets the value rise from the last capped one at
# RELEASE_JERK until it meets the plan again (cutin_resp already did this itself). Only ever slows a rise.
RELEASE_RULES = ("far_early", "ci_confirm", "hazard")
RELEASE_JERK = 1.0        # m/s^3
POLL_CYCLES = 20          # off / on files and the accel scale are re-read once a second
DECIDER_HOLD = 10         # cycles a new decider must hold before it is logged
STOP_GAP_LOW = 4.5        # m, see above


# --- 0 e2e_fill ---------------------------------------------------------------------------------------------
# see the E2EFill docstring
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


# --- 1 far_soft ---------------------------------------------------------------------------------------------
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
# 11/11 15:48 (jam tail 105 m ahead at 62 km/h, 106 km/h, lead slowing at -1.1): once the lead braked harder than
# FAR_LEAD_MAX_LEAD_DECEL there was no limit and the MPC went from -1.5 to -2.74 in 1 s; holding a steady -1.4 from the
# moment the lead was taken would have settled behind it (driver: without the car that then cut in, no hard braking was
# needed). A braking far lead is now limited too, but never to less than FAR_LEAD_BRAKING_FLOOR, and never to less than
# FAR_LEAD_BRAKING_K x the constant deceleration that stops STOP_DISTANCE behind the point where the lead would stand if
# it kept its deceleration to a stop (the worst case for a lead that is slowing). Same fade with distance and time to
# contact as above, so a close or fast-closing lead is left to the MPC.
# Between the two limits there is no step: from FAR_LEAD_MAX_LEAD_DECEL + FAR_LEAD_BRAKING_BLEND (-0.5) down to
# FAR_LEAD_MAX_LEAD_DECEL the limit moves linearly from the one above to this one (first version: -2.00 -> -2.39 within
# 0.2 s when the lead's deceleration came back above -1.0).
FAR_LEAD_BRAKING_FLOOR = -2.0  # m/s^2
FAR_LEAD_BRAKING_K = 1.0
FAR_LEAD_BRAKING_BLEND = 0.5   # m/s^2 of lead deceleration
FAR_LEAD_OFF_FILE = "/data/ars510_no_far_lead_soft"


def far_lead_accel_floor(lead, t_follow):
  """Lowest acceleration allowed for a far radar lead, and how much of it applies (0..1). None when not applicable."""
  if not (lead.present and lead.radar):
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
  floor = min(min(lead.aLeadK, 0.) - FAR_LEAD_K * a_req, FAR_LEAD_MIN_DECEL)
  k = (FAR_LEAD_MAX_LEAD_DECEL + FAR_LEAD_BRAKING_BLEND - lead.aLeadK) / FAR_LEAD_BRAKING_BLEND
  if k > 0.:
    v_lead = max(lead.vLead, 0.)
    room = lead.dRel + v_lead ** 2 / (2. * -lead.aLeadK) - STOP_DISTANCE
    if room <= 1.:
      return None
    braking = min(-FAR_LEAD_BRAKING_K * (v_lead + closing) ** 2 / (2. * room), FAR_LEAD_BRAKING_FLOOR)
    floor += min(k, 1.) * (braking - floor)
  return floor, w


# --- 2 far_early (+ the radar-only far object and the logging-only cut-in coast candidate) ------------------
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
FAR_RADAR_ONLY_MAX_CLOSING = 12.    # m/s, the radar's ACC target as the radar-only source
# 11/11 15:48 (jam tail after a double lane change, 106 km/h): the object list had the tail from 15:48:05 (120 m, 62 km/h,
# in path), radard took it as the lead 2.4 s later and the MPC then needed -2.5 within 1.5 s; the far object was published
# from 15:48:06.0 but closed at 12-16 m/s, just over the 12 m/s limit, so there was no coast. The object-list far object
# now qualifies up to FAR_OBJ_MAX_CLOSING. Above FAR_RADAR_ONLY_MAX_CLOSING it stays a plain coast
# (FAR_RADAR_ONLY_MAX_DECEL) and is not fused into a camera lead (fuse_far_object: that would allow FAR_EARLY_MAX_DECEL).
# far_closing_eval.py, 2040 segments / 214 km above 70 km/h: 7 more episodes (0.6-2.4 s each), 3 became the lead,
# the other 4 cost 0.6-1.1 s of coasting each. A stopped object at speed still closes faster than this and stays out.
FAR_OBJ_MAX_CLOSING = 20.           # m/s, the object-list far object
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


def far_lead_accel_ceil(lead, t_follow, vision_max_decel=FAR_EARLY_MAX_DECEL_VISION):
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
  return max(ceil, FAR_EARLY_MAX_DECEL if lead.radar else vision_max_decel), w


def fuse_far_object(lead, fo):
  """A camera-only lead with a radar far object at about its range (FAR_FUSE_D_M or 15 % of range, the camera's
  distance error grows with range): the lead with the radar's closing speed if faster, and treated as radar-confirmed."""
  if fo is None or getattr(fo, "fast", False) or not lead.present or lead.radar or abs(fo.dRel - lead.dRel) > max(FAR_FUSE_D_M, 0.15 * lead.dRel) or fo.vRel >= lead.vRel:
    return lead
  return types.SimpleNamespace(present=True, radar=True, dRel=lead.dRel, vRel=fo.vRel, vLead=fo.vLead, aLeadK=min(lead.aLeadK, 0.), fused=True)


# cut-in coast (10/30): a radar object-list car (radarTracks CUTIN_TRACK_ID, see ars510_radar_interface.py) 25-90 m ahead,
# CUTIN_PATH_Y_M off the model's path (partly in the lane, not centred: that is the lead), slower by 2 m/s+, held CUTIN_HOLD
# cycles, above CUTIN_MIN_V: lift off to at most CUTIN_MAX_DECEL. 35 highway minutes: 12 of 16 such cars did cut in, 0-8 s
# later (median ~3 s); the 4 others cost a few seconds of coasting. The radar's own lane-crossing flag never fired.
CUTIN_TRACK_ID = 1 << 27
CUTIN_PATH_Y_M = (1.0, 2.6)
CUTIN_HOLD = 20                     # planner cycles (1 s)
CUTIN_MIN_V = 60. / 3.6
CUTIN_MAX_DECEL = -0.3
# 10/30 19:23-20:16 (54 min): 13 cut-in coasts, 1 real cut-in; the rest were adjacent-lane cars hugging the line (up to
# 6 s / -8 km/h). Logging only until a discriminator (lateral motion, duration) is found in the reports' clips.
CUTIN_COAST_ENABLED = False


def cutin_accel_ceil(obj, t_follow):
  """Coast cap for a cut-in candidate: the constant deceleration that would settle behind it, never below CUTIN_MAX_DECEL."""
  closing = -obj.vRel
  if closing < 2.0:
    return None
  gap = obj.dRel - (t_follow * max(obj.vLead, 0.) + STOP_DISTANCE + FAR_LEAD_MARGIN)
  a_req = closing ** 2 / (2. * gap) if gap > 1. else -CUTIN_MAX_DECEL
  return max(-a_req, CUTIN_MAX_DECEL), 1.0


# --- 2b far_vision --------------------------------------------------------------------------------------------
# ars510: brake in time for a far, slow or standing lead that only the camera has. 11/14 14:43 (rain, 56 km/h, three
# scooters standing at a red light, first seen by the camera 90 m ahead; the radar's object list had them only from 46 m
# and radard took one as the radar lead at 21 m): for 2.8 s the camera's lead speed jumped between 2 and 9 m/s (true: 0),
# the MPC asked -0.6..-0.9 m/s^2 where a standing lead needed about -1.4, and the driver braked at 21 m. The lead's speed
# is taken as the lowest of the last FAR_VISION_WIN (a standing object whose speed estimate jumps), and once the constant
# deceleration that then stops the comfortable gap short of it (same gap as far_soft) has been at least
# FAR_VISION_MIN_REQ for FAR_VISION_HOLD, the plan brakes at least FAR_VISION_K x that, never more than
# FAR_VISION_MAX_DECEL, reached at FAR_VISION_JERK. Only below FAR_VISION_MAX_VLEAD: a faster camera lead at highway
# speed gave 3 short false starts in 760 km of logs (11/07, 11/11, 11/12), the 6 left all needed -1.5..-3.8 or the
# driver's brake (tools/ars510/mac/analysis/far_vision_eval.py, 2826 segments). Onset from FAR_VISION_MIN_D: nearer the radar
# usually has the object and the MPC / near_stop / lead_stop decide; with 30 m instead of 50 m the logs give 7 onsets, all real. Off: touch /data/ars510_no_far_vision.
FAR_VISION_MIN_D = 30.         # m, dRel at the onset (50 -> 30 on 11/14: one more onset in 767 km, 11/01 17:44, a real one; none false)
FAR_VISION_MIN_V = 8.          # m/s, ego
FAR_VISION_MIN_PROB = 0.5      # modelProb
FAR_VISION_WIN = 1.0           # s
FAR_VISION_MAX_VLEAD = 8.      # m/s, lowest lead speed of the window
FAR_VISION_MIN_REQ = 0.8       # m/s^2
FAR_VISION_HOLD = 1.0          # s
FAR_VISION_GAP = 0.3           # s, the conditions may drop out this long (the camera lead flickers)
FAR_VISION_K = 0.9
FAR_VISION_MAX_DECEL = -2.0    # m/s^2 (= FAR_LEAD_BRAKING_FLOOR; sim: -1.5 left a -2.7 peak, -2.0 none)
FAR_VISION_JERK = 1.0          # m/s^3
# Once it brakes it keeps the object: the distance is never taken as larger than the last one minus the distance closed
# since (14:43:37-40 the camera's distance stayed 11-25 m too long), a camera lead that drops out for up to
# FAR_VISION_LOST is dead-reckoned (14:43:38.3: 0.4 s without a lead and the MPC planned +1.4; 71 % of 129 camera-lead
# dropouts in the logs came back within 1 s), and it goes on down to FAR_VISION_KEEP_D / FAR_VISION_KEEP_V.
FAR_VISION_LOST = 1.0          # s
FAR_VISION_KEEP_D = 15.        # m
FAR_VISION_KEEP_V = 3.         # m/s
FAR_VISION_OFF_FILE = "/data/ars510_no_far_vision"


def far_vision_req(d_rel, v_ego, v_lead_low, t_follow):
  """Constant deceleration (> 0) that stops the comfortable gap short of a far camera-only lead; 0. when not applicable."""
  v_lead = max(v_lead_low, 0.)
  if v_lead >= FAR_VISION_MAX_VLEAD or v_ego - v_lead <= FAR_LEAD_MIN_CLOSING:
    return 0.
  gap = d_rel - (t_follow * v_lead + STOP_DISTANCE + FAR_LEAD_MARGIN)
  if gap <= 1.:
    return 0.
  return (v_ego - v_lead) ** 2 / (2. * gap)


# --- 3 near_stop --------------------------------------------------------------------------------------------
# ars510: near-stop floor (10/31 11:51, red light behind a stopping car from 51 km/h: the MPC / model eased the request
# from -1.3 to -1.0 at 7 m and the PCM delivered less, the car stopped 2.3 m behind the lead and then crept to 1.8 m at a
# -0.5 hold; the low-speed stop distance is 4.5 m). While a radar lead within NEAR_STOP_D_MAX is stopped or nearly so,
# braking is floored at NEAR_STOP_K x the constant deceleration that stops the car NEAR_STOP_GAP behind it (never more
# than NEAR_STOP_MAX_DECEL). Closer than NEAR_STOP_TOO_CLOSE a firm NEAR_STOP_HOLD_ACCEL is requested even when the wheel
# speed reads 0 (Toyota wheel speed reads 0 while still creeping), which longcontrol follows as an urgent plan.
# 11/10 17:05 (parking lot, engaged at standstill 8-10 m behind a parked car): at 1 km/h the floor is about -0.01, and as
# a plain "min" it turned every positive request into that, so the car could only creep and stopped 5.2-8.5 m back.
# A floor weaker than NEAR_STOP_FREE therefore leaves room to close up: near_stop_cap() rises from the floor itself at
# NEAR_STOP_FREE to +NEAR_STOP_FREE_ACCEL at 0, which also keeps the closing speed proportional to sqrt(room).
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
NEAR_STOP_FREE = -0.3          # m/s^2, a floor weaker than this is no reason to hold the car back
NEAR_STOP_FREE_ACCEL = 0.5     # m/s^2, acceleration allowed when no braking is needed at all
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


def near_stop_cap(floor):
  """What the plan is limited to for a near-stop floor: the floor itself when it is a real braking need, more (up to
  +NEAR_STOP_FREE_ACCEL) the weaker it is, so the car can close up to a stopped lead from further back."""
  if floor <= NEAR_STOP_FREE:
    return floor
  return floor + (floor - NEAR_STOP_FREE) / -NEAR_STOP_FREE * NEAR_STOP_FREE_ACCEL


# --- 4b landing ---------------------------------------------------------------------------------------------
# 11/10 parking lot (driver: "your stop still has a jolt, mine has none"). Closing up to a parked car at 4-5 km/h the
# plan wobbled -0.30 -> -0.17 -> -0.36 -> -0.20 -> -0.47 in 4 s (the MPC re-solves from the measured speed, the PCM
# answers a request with ~0.4 s lag and overshoot: 0.55 for -0.30, nothing for -0.17), so the brake went on, off and on
# again and the car stopped on the second grab at -0.40..-0.46 (raw IMU). The driver's own stops over the same metres:
# friction force flat (0x0A6: 0.27-0.33 the whole way), deceleration fading 0.4 -> 0.15, -0.14 / -0.22 at the stop.
# 0x0A6 vs wheel speed, 935 half-second windows below 6 km/h: the same friction gives the same deceleration for the
# driver and for the PCM, and the PCM follows a steady request well (median: -0.22 -> -0.14, -0.27 -> -0.28, -0.33 ->
# -0.23); a request above about -0.2 does not slow the car against the creep torque at all.
# So for the last metres to a stopped radar lead the request follows one fixed shape, the driver's: firm first and
# fading with the speed, a = LAND_END + k x v, where k is what brings the car to rest NEAR_STOP_GAP behind the lead
# (closed form, landing_k). k is filtered (LAND_K_TAU) and frozen over the last LAND_ROOM_FREEZE metres, where the radar
# distance and the wheel speed are too coarse to steer by; from there the request simply runs out to LAND_END at the
# stop. It starts when a constant deceleration of LAND_ON would be needed and is approached at LAND_JERK.
# 11/10 18:18 (first version: a latched constant that handed back 0.3 m before the target): the car arrived early,
# rolled on at 1.9 km/h against -0.30 and longcontrol's creep-stall logic added brake -> -0.35 at the stop. The landing
# now keeps the request until the car stands. A plan firmer than the landing by LAND_URGENT, a lead that moves off, or
# more than LAND_EXIT needed ends it at once (near_stop / the MPC take over).
# Off: /data/ars510_no_landing. cloudlog "ARS510 landing on/off".
LAND_V_MAX = 2.0               # m/s (7 km/h): only the last, slow stretch
LAND_VLEAD_MAX = 0.5           # m/s, |vLead|: the lead stands
LAND_ON = 0.22                 # m/s^2, constant deceleration needed at which the landing starts ...
LAND_MAX = 0.6                 # m/s^2 ... and above which it does not start (not a gentle landing any more)
LAND_EXIT = 0.9                # m/s^2 needed: give up
LAND_END = 0.28                # m/s^2, request left at the stop (11/10 17:43:58: a steady -0.30 gave 0.2 fading to 0.14 at
                               # the stop, like the driver's own -0.14 / -0.22)
LAND_ROOM_FREEZE = 0.4         # m
LAND_K_TAU = 0.4               # s
LAND_K_MAX = 2.0               # 1/s
LAND_LEAD = 0.35               # s
LAND_JERK = 0.8                # m/s^3
LAND_URGENT = 0.3              # m/s^2
LAND_OFF_FILE = "/data/ars510_no_landing"


# --- 4 lead_stop --------------------------------------------------------------------------------------------
# constants and logic in ars510_lead_stop.py


# --- 5 ci_confirm -------------------------------------------------------------------------------------------
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
# 11/01: no confirmation while I change lanes myself (ars510_ego_lc.py): the cars of the lane I move into are not cutting in


# --- 6 cutin_resp -------------------------------------------------------------------------------------------
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


# --- 7 hazard -----------------------------------------------------------------------------------------------
# ars510: hazard coast (11/01, driver: "at least lift off"). The radar interface's hazard candidate (radarTracks
# HAZARD_TRACK_ID, see ars510_radar_interface.py HAZARD_*: a tracked car closing fast on a short time-to-collision that
# the camera has not made the lead) in my lane or moving toward
# it (those checks, HAZARD_PATH_M / HAZARD_CROSS_VY, live in the interface: RadarPoint carries no lateral speed here) and
# within HAZARD_CROSS_M of the model path, for HAZARD_HOLD cycles -> acceleration capped at HAZARD_ACCEL (lift-off, no
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


# --- 8 turn_hold --------------------------------------------------------------------------------------------
# ars510: no surge while turning. Signalling below BLINKER_NO_ACCEL_V (10/27: raised from 30 km/h; at 21:41 the car was
# at 31-32 km/h in a left turn when the lead left the view and it accelerated toward the set speed), or the wheel turned
# past TURN_NO_ACCEL_ANGLE (leaves below TURN_NO_ACCEL_EXIT_ANGLE; losing the lead mid-turn must not speed the car up).
# 11/10 (driver: the rule is against surging in a turn, the car should still pull away by itself on green): the cap is
# now TURN_ACCEL(v) instead of 0 -- a gentle start from rest, fading to no acceleration at 20 km/h and above (as before
# there). Before, a signal held the car at a green light, and the turned-wheel branch had no limit at all below 3 m/s.
# The cap comes down at TURN_JERK_DOWN (it used to cut acceleration to 0 in one step); the rise after the turn is
# a_pos's jerk limit.
BLINKER_NO_ACCEL_V = 50. * CV.KPH_TO_MS
TURN_NO_ACCEL_ANGLE = 45.        # deg, steering wheel
TURN_NO_ACCEL_EXIT_ANGLE = 30.   # deg
TURN_ACCEL_BP = [0., 10. / 3.6, 20. / 3.6]   # m/s
TURN_ACCEL_V = [0.8, 0.5, 0.]                # m/s^2
TURN_JERK_DOWN = 1.5             # m/s^3


# --- 9 a_pos ------------------------------------------------------------------------------------------------
# an unhurried ceiling on positive acceleration, see the PosAccelCeiling docstring
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
# 11/10 17:05: behind a stopped lead the law above is ~0 (0.003 x 4 m = +0.01), the car only crept and stopped 5-8 m
# back. A stopped / crawling lead (below LEAD_CLOSE_VLEAD) may be closed up to: the speed that a LEAD_CLOSE_A stop still
# brings to rest STOP_GAP_LOW behind it, reached with the same LEAD_TAU.
LEAD_CLOSE_A = 0.3                         # m/s^2
LEAD_CLOSE_VLEAD = [1., 3.]                # m/s, full effect below / none above


def read_accel_scale() -> float:
  try:
    with open(ACCEL_SCALE_FILE) as f:
      v = float(f.read().strip())
    return 0. if v <= 0. else float(np.clip(v, 0.3, 1.0))
  except Exception:
    return 1.0


# ---------------------------------------------------------------------------------------------------------------------
# rules. Each: name, apply(c, a) -> new a (c = Ctx). State lives in the rule. Order and kinds: see the module docstring.

class Ctx:
  """Per-cycle inputs shared by the rules."""
  __slots__ = ("sm", "long", "v_ego", "v_cruise", "reset_state", "a_prev", "t_follow", "now", "fcw", "mpc", "dt", "rain")

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
      fill = min(E2E_FILL_K * (c.v_cruise - c.v_ego),
                 float(np.interp(c.v_ego, E2E_FILL_A_MAX_BP, E2E_FILL_A_MAX_V)) * (c.rain.accel_scale if c.rain is not None else 1.))
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
      # rain mode: a camera-only lead may ease off harder (the radar sees less in rain, 11/09 18:12)
      res = far_lead_accel_ceil(fused, t_follow, c.rain.vision_max_decel if c.rain is not None else FAR_EARLY_MAX_DECEL_VISION)
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
      # closing faster than FAR_OBJ_MAX_CLOSING / FAR_RADAR_ONLY_MAX_CLOSING is not a slower car (oncoming / roadside / stopped: the far-stopped-car
      # path and the camera handle those) and is left alone (10/30 20:xx: 124 m, -16.9 m/s, gone 2 s later)
      cands = [p for p in pts if p.trackId == FAR_OBJ_TRACK_ID and -p.vRel <= FAR_OBJ_MAX_CLOSING] or \
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
                                               src="obj" if p.trackId == FAR_OBJ_TRACK_ID else "acc",
                                               fast=-float(p.vRel) > FAR_RADAR_ONLY_MAX_CLOSING)
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


class FarVision(Rule):
  """2b: brake floor for a far, slow / standing camera-only lead (see FAR_VISION_*)."""
  name = "far_vision"

  def __init__(self):
    self.off = os.path.exists(FAR_VISION_OFF_FILE)
    self.active = False
    self._reset()

  def _reset(self):
    self.v_leads: list[float] = []
    self.miss = 0
    self.held = 0
    self.low = 0
    self.floor: float | None = None
    self.d: float | None = None
    self.v_low = 0.

  def poll(self):
    self.off = os.path.exists(FAR_VISION_OFF_FILE)

  def apply(self, c, a):
    lead = c.sm['radarState'].leadOne
    gap_n = int(round(FAR_VISION_GAP / c.dt))
    on = self.floor is not None   # already braking for this object
    req = 0.                      # constant deceleration needed this cycle (0. = no claim)
    if not (c.long and not self.off and not c.reset_state and c.v_ego > (FAR_VISION_KEEP_V if on else FAR_VISION_MIN_V)):
      self._reset()   # not engaged / driver override / too slow: forget everything
    elif (lead.present and not lead.radar and lead.modelProb >= FAR_VISION_MIN_PROB
          and lead.dRel >= (FAR_VISION_KEEP_D if on else FAR_VISION_MIN_D)):
      # a camera-only lead: lowest speed of the last FAR_VISION_WIN (a standing object whose speed estimate jumps)
      self.miss = 0
      self.v_leads = (self.v_leads + [lead.vLead])[-int(round(FAR_VISION_WIN / c.dt)):]
      self.v_low = max(min(self.v_leads), 0.)
      d = lead.dRel
      if on and self.d is not None:
        d = min(d, self.d - (c.v_ego - self.v_low) * c.dt)   # a standing object cannot stay at the same distance
      self.d = d
      req = far_vision_req(d, c.v_ego, self.v_low, c.t_follow)
    elif on and self.d is not None and self.miss * c.dt < FAR_VISION_LOST:
      # the camera lead dropped out (or one radar frame): keep the object by dead reckoning
      self.miss += 1
      self.d -= (c.v_ego - self.v_low) * c.dt
      if self.d >= FAR_VISION_KEEP_D:
        req = far_vision_req(self.d, c.v_ego, self.v_low, c.t_follow)
    else:
      self.miss += 1
      if on or self.miss > gap_n:
        self._reset()
        on = False
    # onset only after the need has lasted FAR_VISION_HOLD (dropouts up to FAR_VISION_GAP do not restart the count)
    if req >= FAR_VISION_MIN_REQ:
      self.held += 1
      self.low = 0
    else:
      self.low += 1
      if self.low > gap_n:
        self.held = 0
        self.floor = None
    active = False
    if req >= FAR_VISION_MIN_REQ and (on or self.held * c.dt >= FAR_VISION_HOLD):
      # the floor: K x the need, limited, ramped from the previous output; it only ever lowers the plan (min)
      target = max(-FAR_VISION_K * req, FAR_VISION_MAX_DECEL)
      self.floor = max(target, (c.a_prev if self.floor is None else self.floor) - FAR_VISION_JERK * c.dt)
      if a > self.floor:
        a = self.floor
        active = True
    if active != self.active:
      self.active = active
      cloudlog.info(f"ARS510 far vision {'on' if active else 'off'} d={lead.dRel:.1f} vLead={lead.vLead:.1f} v={c.v_ego*3.6:.1f} a={a:.2f}")
    return a


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
      if ns is not None:
        ns = near_stop_cap(ns)
        if a > ns:
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


def landing_k(v, room):
  """k of the fading stop a = LAND_END + k v that comes to rest in `room` from speed v (0 if LAND_END alone stops
  earlier). Distance of that stop: v / k - LAND_END / k^2 x ln(1 + k v / LAND_END), falling with k."""
  if v <= 0. or v * v / (2. * LAND_END) <= room:
    return 0.
  lo, hi = 0., LAND_K_MAX
  for _ in range(24):
    k = 0.5 * (lo + hi)
    if v / k - LAND_END / (k * k) * math.log(1. + k * v / LAND_END) > room:
      lo = k
    else:
      hi = k
  return hi


class Landing(Rule):
  """4b: the driver's stop over the last metres to a stopped radar lead: one brake application fading with the speed
  (see LAND_*), kept until the car stands. May brake lighter than the plan by at most LAND_URGENT; anything firmer
  than that wins and ends the landing."""
  name = "landing"
  softens = True

  def __init__(self):
    self.off = os.path.exists(LAND_OFF_FILE)
    self.k = None         # 1/s, None = not landing
    self.out = 0.         # the ramped request

  def poll(self):
    self.off = os.path.exists(LAND_OFF_FILE)

  def _end(self, lead, c, why):
    if self.k is not None:
      cloudlog.info(f"ARS510 landing off ({why}) d={lead.dRel:.1f} v={c.v_ego*3.6:.1f} k={self.k:.2f} a={self.out:.2f}")
    self.k = None

  def apply(self, c, a):
    lead = c.sm['radarState'].leadOne
    if not c.long or self.off or c.reset_state or c.fcw:
      self._end(lead, c, "off")
      return a
    if not (lead.present and lead.radar) or abs(lead.vLead) > LAND_VLEAD_MAX:
      self._end(lead, c, "lead")
      return a
    v = c.v_ego
    room = lead.dRel - NEAR_STOP_GAP
    need = v * v / (2. * max(room, 0.1))
    if self.k is None:
      if not (NEAR_STOP_MIN_V <= v <= LAND_V_MAX and room >= LAND_ROOM_FREEZE and LAND_ON <= need <= LAND_MAX):
        return a
      self.k = landing_k(v, room)
      self.out = min(c.a_prev, 0.)
      cloudlog.info(f"ARS510 landing on d={lead.dRel:.1f} v={v*3.6:.1f} need={need:.2f} k={self.k:.2f} plan={a:.2f}")
    elif v < 0.05:
      self._end(lead, c, "stopped")
      return a
    elif v > LAND_V_MAX + 0.3 or (room >= LAND_ROOM_FREEZE and need > LAND_EXIT):
      self._end(lead, c, "speed / need")
      return a
    elif room >= LAND_ROOM_FREEZE and v >= NEAR_STOP_MIN_V:
      self.k += (landing_k(v, room) - self.k) * c.dt / (LAND_K_TAU + c.dt)
    # the brake answers about LAND_LEAD late: shape the request on the speed the car will have by then
    target = -(LAND_END + self.k * max(v + self.out * LAND_LEAD, 0.))
    if a < target - LAND_URGENT:
      self._end(lead, c, "plan firmer")
      return a
    step = LAND_JERK * c.dt
    self.out += min(max(target - self.out, -step), step)
    return self.out


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
  """8: no surge while signalling at low speed or turning at a junction: acceleration capped at TURN_ACCEL(v)."""
  name = "turn_hold"

  def __init__(self):
    self.turning = False
    self.cap = None

  def apply(self, c, a):
    CS = c.sm['carState']
    steer_angle = abs(CS.steeringAngleDeg)
    if steer_angle > TURN_NO_ACCEL_ANGLE:
      self.turning = True
    elif steer_angle < TURN_NO_ACCEL_EXIT_ANGLE:
      self.turning = False
    if ((CS.leftBlinker or CS.rightBlinker) and c.v_ego < BLINKER_NO_ACCEL_V) or self.turning:
      target = float(np.interp(c.v_ego, TURN_ACCEL_BP, TURN_ACCEL_V))
      start = max(c.a_prev if c.a_prev is not None else 0., target) if self.cap is None else self.cap
      self.cap = target if c.reset_state else max(target, start - TURN_JERK_DOWN * c.dt)
      a = min(a, self.cap)
    else:
      self.cap = None
    return a


class PosAccelCeiling(Rule):
  """9: an unhurried driver's ceiling on positive acceleration, whatever the source (MPC lead pull-away, model, cruise,
  fill): A_POS_MAX(v) x the scale in /data/ars510_accel_scale (0.3-1.0, read once a second, no restart), rising at
  A_POS_JERK. Braking is never touched. 10/30 evening: with a lead pulling away the MPC / model asked +0.8..+1.0 at
  90-100 km/h, which felt harsh; the hybrid also keeps mostly electric below about 0.25 m/s^2 at highway speed."""
  name = "a_pos"

  def __init__(self):
    self.scale = read_accel_scale()

  def poll(self):
    self.scale = read_accel_scale()

  def apply(self, c, a):
    if c.long and not c.reset_state and self.scale > 0.:
      v_ego = c.v_ego
      cap = float(np.interp(v_ego, A_POS_MAX_BP, A_POS_MAX_V)) * self.scale * (c.rain.accel_scale if c.rain is not None else 1.)
      # first-order approach to the set speed: a <= (v_set - v) / tau, so the speed settles exponentially with no overshoot
      cap = min(cap, max(c.v_cruise - v_ego, 0.) / A_POS_TAU)
      lead = c.sm['radarState'].leadOne
      if lead.present and lead.dRel < LEAD_MAX_D:
        desired = c.t_follow * max(lead.vLead, 0.) + STOP_DISTANCE
        lead_cap = (max(lead.vLead - v_ego, 0.) / float(np.interp(v_ego, LEAD_TAU_BP, LEAD_TAU_V)) +
                    LEAD_A_GAIN * max(lead.aLeadK, 0.) + min(LEAD_GAP_K * max(lead.dRel - desired, 0.), LEAD_GAP_MAX))
        w_close = float(np.interp(lead.vLead, LEAD_CLOSE_VLEAD, [1., 0.]))
        if w_close > 0.:
          v_close = math.sqrt(2. * LEAD_CLOSE_A * max(lead.dRel - STOP_GAP_LOW, 0.))
          lead_cap += w_close * max(v_close - v_ego, 0.) / float(np.interp(v_ego, LEAD_TAU_BP, LEAD_TAU_V))
        cap = min(cap, lead_cap)
      cap = min(cap, max(c.a_prev, 0.) + float(np.interp(v_ego, A_POS_JERK_BP, A_POS_JERK_V)) * c.dt)
      a = min(a, cap)
    return a


class Rules:
  """The arbiter: e2e_fill on the model candidate, then RULES in order; tracks the decider."""

  def __init__(self, dt):
    self.e2e_fill = E2EFill()
    self.rules: list[Rule] = [FarLeadSoft(), FarLeadEarly(), FarVision(), NearStop(), LeadStop(dt), Landing(), CutinConfirm(), CutinResponse(),
                              HazardCoast(), TurnHold(), PosAccelCeiling()]
    self.by_name = {r.name: r for r in [self.e2e_fill] + self.rules}
    self.rain = ars510_rain.RainMode(dt)   # not a rule: scales t_follow / acceleration for the rules and the MPC
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
      self.rain.poll()

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
