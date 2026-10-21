#!/usr/bin/env python3
import math
import struct
import time
import numpy as np
from collections import deque
from typing import Any

import capnp
from openpilot.cereal import messaging, log, custom
from opendbc.car.structs import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "present": True,
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


_UNDERFLOW = {"n": 0, "last_log": 0.0}  # dimpleliang/ars510: camera/radar match score underflow counter (see match_vision_to_track)


# dimpleliang/ars510: keep last cycle's radar lead when the camera's range drifts past the 25 % check (at speed the camera
# often puts a car the radar holds at 33 m at 45 m; the lead then flips radar <-> camera and its distance jumps 10+ m).
# Only a track no farther than the camera's lead (+ KEEP_FARTHER_M) is kept, so a closer car the camera sees first (a cut-in)
# still takes over at once; the track must stay laterally on the camera's lead and pass the same speed check.
# Replay of 106 min (20 Oct): radar<->camera flips 7.1 -> 5.0 per lead-minute, lead jumps > 5 m 22.8 -> 18.1 per minute.
KEEP_DIST_FRAC = 0.5
KEEP_DIST_MIN_M = 8.0
KEEP_FARTHER_M = 2.0
KEEP_LAT_M = 1.5


# dimpleliang/ars510: a two-wheeler in the ego path. The camera's lead is often the car beyond a scooter riding in our lane
# (Taiwan: 6/6 such episodes on 21 Oct verified on video, 33 episodes / 91 s in 206 min). The ARS510 classifies objects;
# its display message (customReservedRawData0, see opendbc ars510_radar_interface.py) carries the class per published
# track. A settled, moving two-wheeler whose track sits within VRU_PATH_M of the model's path and is VRU_CLOSER_M closer
# than the lead we would otherwise pick becomes leadOne after VRU_MIN_CYCLES consecutive cycles; it is kept while it
# stays within VRU_PATH_KEEP_M of the path and closer than the camera's lead.
VRU_CLASSES = (5, 6)       # radar object class 163|3: two-wheelers (validated on route 0000000e)
VRU_PATH_M = 1.0           # lateral overlap with the car (half car 0.95 + half scooter 0.4 = 1.35; 1.0 is conservative)
VRU_PATH_KEEP_M = 1.3
VRU_CLOSER_M = 5.0
VRU_MIN_CYCLES = 10        # 0.5 s
VRU_MAX_D = 60.0
VRU_MIN_V_LEAD = 1.0       # m/s over ground: a parked scooter at the kerb is not a lead
DISPLAY_FMT = "<fffB"
DISPLAY_FMT_SIZE = struct.calcsize(DISPLAY_FMT)


def parse_display_classes(payload: bytes) -> dict[tuple[float, float], tuple[int, bool]]:
  """(dRel, yRel) -> (class, settling) from the ARS510 display message. Same float32 values as the radarTracks points."""
  out: dict[tuple[float, float], tuple[int, bool]] = {}
  if len(payload) < 4 or payload[:2] != b"A5" or payload[2] != 1:
    return out
  for i in range(payload[3]):
    off = 4 + i * DISPLAY_FMT_SIZE
    if off + DISPLAY_FMT_SIZE > len(payload):
      break
    d, y, _v, flags = struct.unpack_from(DISPLAY_FMT, payload, off)
    out[(d, y)] = ((flags >> 2) & 7, bool(flags & 2))
  return out


def path_lateral_error(track: Track, path_x, path_y) -> float:
  """Lateral distance from the model path at the track's range. Model y is right-positive, radar yRel left-positive."""
  if len(path_x) < 2:
    return abs(track.yRel)
  return abs(-track.yRel - float(np.interp(track.dRel, path_x, path_y)))


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track], keep_id: int | None = None):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def log_prob(c):
    # dimpleliang/ars510: same ranking as the product of the three Laplace pdfs (laplacian_pdf), without the exponential
    # underflow that made every candidate score 0.0 and left the pick to dict order.
    return -(abs(c.dRel - offset_vision_dist) / max(lead.xStd[0], 1e-4)
             + abs(c.yRel + lead.y[0]) / max(lead.yStd[0], 1e-4)
             + abs(c.vRel + v_ego - lead.v[0]) / max(lead.vStd[0], 1e-4))

  track = max(tracks.values(), key=log_prob)
  best = log_prob(track)
  if math.exp(best) == 0.0:
    # The original product would have underflowed to 0.0 for every candidate. Count it so rlogs can answer how often
    # this actually happens; at most one log line per 5 s.
    _UNDERFLOW["n"] += 1
    now = time.monotonic()
    if now - _UNDERFLOW["last_log"] > 5.0:
      _UNDERFLOW["last_log"] = now
      cloudlog.info(f"ARS510 match underflow #{_UNDERFLOW['n']}: {len(tracks)} tracks, best log score {best:.0f}, "
                    f"picked {track.dRel:.1f} m vs camera {offset_vision_dist:.1f} m")

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track

  kept = tracks.get(keep_id) if keep_id is not None else None
  if kept is not None:
    keep_dist = abs(kept.dRel - offset_vision_dist) < max(offset_vision_dist * KEEP_DIST_FRAC, KEEP_DIST_MIN_M)
    not_farther = kept.dRel < offset_vision_dist + KEEP_FARTHER_M
    on_lead = abs(kept.yRel + lead.y[0]) < KEEP_LAT_M
    keep_vel = (abs(kept.vRel + v_ego - lead.v[0]) < 10) or (v_ego + kept.vRel > 3)
    if keep_dist and not_farther and on_lead and keep_vel:
      return kept
  return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "modelProb": float(lead_prob),
    "present": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, lead_prob: float, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
             low_speed_override: bool = True, keep_id: int | None = None) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks, keep_id)
  else:
    track = None

  lead_dict = {'present': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_prob)
    lead_dict = get_custom_yrel(CP, CP_SP, lead_dict, lead_msg)
  elif (track is None) and ready and (lead_prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


def get_custom_yrel(CP: structs.CarParams, CP_SP: structs.CarParamsSP, lead_dict: dict[str, Any],
                    lead_msg: capnp._DynamicStructReader) -> dict[str, Any]:
  if CP.brand == "hyundai" and (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC or
                                CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
    lead_dict['yRel'] = float(-lead_msg.y[0])

  return lead_dict


class RadarD:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParams, delay: float = 0.0):
    self.CP = CP
    self.CP_SP = CP_SP

    self.current_time = 0.0
    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False
    self.keep_ids: list[int | None] = [None, None]
    self.vru_classes: dict[int, int] = {}     # track id -> radar class, settled two-wheelers only
    self.vru_cnt: dict[int, int] = {}         # consecutive cycles a two-wheeler qualified as an in-path override
    self.vru_active: int | None = None        # track id currently overriding leadOne
    self.vru_overrides = 0

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # dimpleliang/ars510: radar classes for the current points (display message is published in the same decoder cycle).
    # The display service is optional: it must never gate radarState.valid, and a stale copy (publisher stopped or
    # delayed) must not keep feeding old classes, so only a fresh (<= 150 ms) valid message is used.
    classes = {}
    display_service = 'customReservedRawData0'
    if display_service in sm.data:
      now_ns = max(sm.logMonoTime[s] for s in ('modelV2', 'carState', 'radarTracks', display_service))
      display_age_ns = now_ns - sm.logMonoTime[display_service]
      if sm.seen[display_service] and sm.valid[display_service] and display_age_ns <= 150_000_000:
        classes = parse_display_classes(bytes(sm[display_service]))
    self.vru_classes = {}
    for pt in rr.points:
      cls = classes.get((pt.dRel, pt.yRel))
      if cls is not None and cls[0] in VRU_CLASSES and not cls[1]:
        self.vru_classes[pt.trackId] = cls[0]

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        # Asymmetric filter on lead prob to keep lead when uncertain
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, self.lead_prob_filters[0].x,
                                          self.CP, self.CP_SP, low_speed_override=True, keep_id=self.keep_ids[0])
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, self.lead_prob_filters[1].x,
                                          self.CP, self.CP_SP, low_speed_override=False, keep_id=self.keep_ids[1])
      self.keep_ids = [lead.radarTrackId if (lead.present and lead.radar) else None
                       for lead in (self.radar_state.leadOne, self.radar_state.leadTwo)]
      self.vru_override(sm['modelV2'].position)
    else:
      self.keep_ids = [None, None]
      self.vru_cnt.clear()
      self.vru_active = None

  def vru_override(self, position) -> None:
    """leadOne := a settled, moving two-wheeler in the ego path that is clearly closer than the lead chosen so far."""
    lead = self.radar_state.leadOne
    lead_d = lead.dRel if lead.present else float('inf')
    path_x, path_y = list(position.x), list(position.y)
    cands = {}
    for tid, cls in self.vru_classes.items():
      t = self.tracks.get(tid)
      if t is None or not (0.75 < t.dRel < VRU_MAX_D) or t.vLead < VRU_MIN_V_LEAD:
        continue
      lat = path_lateral_error(t, path_x, path_y)
      keep = tid == self.vru_active
      if lat < (VRU_PATH_KEEP_M if keep else VRU_PATH_M) and t.dRel < lead_d - (0.0 if keep else VRU_CLOSER_M):
        cands[tid] = t
    self.vru_cnt = {tid: self.vru_cnt.get(tid, 0) + 1 for tid in cands}
    pick = None
    if self.vru_active in cands:
      pick = cands[self.vru_active]
    else:
      ready = [t for tid, t in cands.items() if self.vru_cnt[tid] >= VRU_MIN_CYCLES]
      if ready and self.v_ego > V_EGO_STATIONARY:
        pick = min(ready, key=lambda t: t.dRel)
        self.vru_overrides += 1
        cloudlog.info(f"ARS510 two-wheeler lead override #{self.vru_overrides}: {pick.dRel:.1f} m, camera lead {lead_d:.1f} m, v_ego {self.v_ego:.1f}")
    if pick is None:
      self.vru_active = None
      return
    self.vru_active = pick.identifier
    # This override has no camera association to the selected object, so it does not inherit the camera lead's modelProb.
    state = pick.get_RadarState()
    for k, v in state.items():
      setattr(self.radar_state.leadOne, k, v)
    # keep_ids belongs to the ordinary camera/radar match; the override keeps its own identity in vru_active.

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  cloudlog.info("radard is waiting for CarParamsSP")
  CP_SP = messaging.log_from_bytes(Params().get("CarParamsSP", block=True), custom.CarParamsSP)
  cloudlog.info("radard got CarParamsSP")

  # *** setup messaging
  optional_services = ['customReservedRawData0']  # dimpleliang/ars510: radar classes for the two-wheeler override only
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks', *optional_services], poll='modelV2',
                           ignore_alive=optional_services, ignore_avg_freq=optional_services, ignore_valid=optional_services)
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP, CP_SP, CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['radarTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
