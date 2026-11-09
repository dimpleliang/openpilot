"""ARS510 cut-in rule, shared by the UI marks (ui/sunnypilot/onroad/ars510_tracks.py) and the planner's response
(longitudinal_planner.py, 11/04): the driver sees an amber circle exactly when the planner reacts.

Input: the radar's settled object list for display (customReservedRawData0, written by ars510_radar_interface.py), my
predicted path and lane lines (modelV2), my lead (radarState). Output per object: predicted / confirmed cut-in, the side it
comes from, how far it is past the lane line, its sideways speed toward me, whether it qualifies for braking (strict).
The constants' history is in ARS510_RADAR.md (cut-in marks) and in the comments below.
"""
import struct

import numpy as np

from openpilot.selfdrive.controls.lib.ars510_ego_lc import EgoLaneChange

SERVICE = "customReservedRawData0"
FMT_V1 = "<fffB"
FMT = "<fffBH"  # v2: + native track id
FMT_V3 = "<fffBHhB"  # v3: + radar lateral speed (cm/s, left-positive), lane-crossing code (2 / 5 cut-in from left / right)
LEAD_MATCH_D_M = 2.0
LEAD_MATCH_Y_M = 1.2
SELF_LEAD_D_M = 3.0   # an object this near my lead's range (and its y, if radar) is the lead itself
MAX_D_M = 120.0
FRAME_HZ = 20.0  # update() is called once per modelV2 (UI frame / planner cycle)

# Cut-in prediction (10/31): a next-lane car whose lateral distance to my path has been shrinking faster than CUTIN_RATE
# over the last CUTIN_WIN seconds, still CUTIN_OFF_MIN..CUTIN_OFF_MAX m from the path, slower than me, 25-90 m ahead.
CUTIN_WIN = 2.0
CUTIN_RATE = -0.15  # m/s of |offset|
CUTIN_OFF_MIN, CUTIN_OFF_MAX = 0.8, 2.6
CUTIN_D = (25.0, 90.0)
CUTIN_VREL = -1.5
CUTIN_HOLD_FRAMES = 10  # 0.5 s at 20 Hz
CUTIN_MIN_V_GROUND = 2.0  # m/s over the ground, forward: excludes oncoming cars
CUTIN_MIN_V_EGO = 1.0     # m/s: nothing while I am (nearly) stopped (11/02 driver)
CUTIN_RELEASE_FRAMES = 20
# 11/04 driver: with a lead, a cut-in is an object entering the GAP between me and my lead. The radar range is to the
# object's rear: it must sit at least its own length (less ~1.5 m of overlap) nearer than the lead's rear. By radar class
# (163|3): 1 unclassified, 2 car, 3 large vehicle, 4 pedestrian, 5 provisional two-wheeler, 6 two-wheeler.
GAP_MARGIN_M = {1: 3.0, 2: 3.0, 3: 10.5, 4: 0.5, 5: 0.5, 6: 0.5}
GAP_MARGIN_DEFAULT_M = 3.0
# 11/04 slimming (replay ablation, 128 km, merge_replay.py): removed the wide (2.6-5 m) trend, the fast (> 1 m/s sideways)
# trigger, the close fast trigger and the curve skip: with them 32 true / 23 line-huggers / 25 noise marks, without them
# 35 / 21 / 22. Kept: near trend, close moving-in (most real cut-ins), the radar's lane-crossing code, confirmed.
# 11/04 motion check (borrowed from CarrotPilot radar_motion/trajectory_cutin.py: direction consistency + jitter filter, and a
# distance-dependent noise floor for our front radar): the near trend and the close moving-in triggers also need, over the
# last MOTION_WIN_S, the offset steps to point inward (inward travel / all travel >= MOTION_MIN_CONSISTENCY), no jitter (if it
# moved >= JITTER_MIN_TRAVEL_M in all, the net inward part >= JITTER_MIN_NET_FRACTION of it) and a net inward progress above
# the radar's lateral noise at that range (NOISE_FLOOR_*: ARS510 lateral noise grows with range, 17:05 +-0.6 m at 80 m).
MOTION_GATE = True
MOTION_WIN_S = 1.0
MOTION_MIN_CONSISTENCY = 0.65
JITTER_MIN_TRAVEL_M = 0.45
JITTER_MIN_NET_FRACTION = 0.5
NOISE_FLOOR_BP = [20.0, 80.0]
NOISE_FLOOR_V = [0.2, 0.6]
LC_OFF = (0.9, 5.0)          # m from my path (radar lane-crossing code)
LC_D = (15.0, 120.0)         # m
CUTIN_MAX_VREL = 1.5         # m/s, lane-crossing code: slower or about my speed
LC_CUTIN = (2, 5)
SLOW_TOWARD = 0.4
SLOW_HOLD_RECORDS = 17
# 11/05 route 53 (75 km/h, a car from the next lane in at 17 m, 1.2 m/s sideways): the slow counter needs ~0.9 s, the car
# was the lead before it was marked. A fast sideways move toward me needs fewer records.
FAST_TOWARD = 0.8
FAST_HOLD_RECORDS = 6
CLOSE_D = (5.0, 40.0)
CLOSE_OFF = (0.9, 4.0)
# Confirmed cut-in: within CONFIRM_OFF of my path, CONFIRM_D, slower by CONFIRM_VREL, CONFIRM_FROM m or more out within CONFIRM_WIN s.
CONFIRM_OFF = 1.3
CONFIRM_D = (10.0, 60.0)
CONFIRM_VREL = -0.5
CONFIRM_FROM = 1.8
CONFIRM_WIN = 3.0
CONFIRM_SHOW_S = 3.0
# Lane-line gate (11/01) and strict check (11/01 false-mark study), see ARS510_RADAR.md
LINE_GATE_M = -1.5
LINE_STRICT_M = -1.0
STRICT_TOWARD = 1.2
SHOW_AFTER_S = 0.0
LINE_PATH_EXTRA_M = 0.3
LINE_MIN_PROB = 0.5
# 11/08 23:51 (route 63, 45-50 km/h, straight road): my lead moved out to the right where the lane lines faded (prob 0.2-0.5);
# my predicted path, which had followed it ~0.4 m, swung back, and the departing car (steady 1.3 m right of me) was marked as
# a predicted cut-in 2.8 s after it stopped being my lead. A car that was my radar lead within LEAD_DEPART_S is not marked
# as a predicted cut-in (a confirmed one - back out past CONFIRM_FROM and in again - still is). Without both lane lines the
# offsets come from the swinging path: a predicted mark there also needs the radar's own sideways speed toward me
# >= NO_LINE_MIN_TOWARD (None: off).
LEAD_DEPART_S = 5.0
NO_LINE_MIN_TOWARD = 0.4
TREND_REF_LANE = False   # 11/04 replay: offsets from the lane centre instead of my path, no gain (55 vs 54 false / 100 km)
# 11/04 response: braking (beyond a lift-off) only for an object this close to / past a reliable lane line (or the radar's
# own cut-in code, or confirmed)
BRAKE_LINE_M = -0.5
CURVE_TRUST_K = 10.0   # m: offsets trusted up to sqrt(K x curve radius), see update()


def parse(payload: bytes) -> list[tuple]:
  """(d, y, vrel, sent, settling, class, track id, v_lat left-positive m/s, lane-crossing code)"""
  if len(payload) < 4 or payload[:2] != b"A5" or payload[2] not in (1, 2, 3):
    return []
  fmt = {1: FMT_V1, 2: FMT, 3: FMT_V3}[payload[2]]
  size = struct.calcsize(fmt)
  out = []
  for i in range(payload[3]):
    off = 4 + i * size
    if off + size > len(payload):
      break
    f = struct.unpack_from(fmt, payload, off)
    d, y, v, flags = f[:4]
    out.append((d, y, v, bool(flags & 1), bool(flags & 2), (flags >> 2) & 7, f[4] if len(f) > 4 else -1,
                f[5] / 100. if len(f) > 5 else float("nan"), f[6] if len(f) > 6 else 0))
  return out


class CutObj:
  __slots__ = ("tid", "d", "y_right", "vrel", "cls", "settling", "kind", "arrow", "intr", "toward", "strict", "in_lane")

  def __init__(self, **kw):
    for k in self.__slots__:
      setattr(self, k, kw.get(k))


class CutinRule:
  def __init__(self):
    # tid -> last time it was my radar lead; aged out after LEAD_DEPART_S, NOT cleared by reset() (11/08: one empty radar
    # frame made the UI reset the rule, the departing lead was forgotten and marked the same frame)
    self._lead_t: dict[int, float] = {}
    self.reset()
    self._ego_lc = EgoLaneChange()
    self._lane_w = 3.5

  def reset(self):
    self._hist: dict[int, list[tuple[int, float]]] = {}
    self._hold: dict[int, int] = {}
    self._out_t: dict[int, float] = {}
    self._confirm_t: dict[int, tuple[float, int]] = {}
    self._side: dict[int, int] = {}
    self._slow: dict[int, int] = {}
    self._fast: dict[int, int] = {}
    self._release: dict[int, int] = {}
    self._qual_t: dict[int, float] = {}

  def update(self, car_state, md, points, radar_state, now: float, frame: int, fresh: bool, model_updated: bool,
             path_y, camera_offset: float) -> list[CutObj]:
    """One frame. path_y(d): my predicted path's y (right-positive, camera offset applied)."""
    leads = []
    gap_leads = []   # (d, y or None if vision-only) of leadOne, leadTwo: for the gap rule
    if radar_state is not None:
      for lead in (radar_state.leadOne, radar_state.leadTwo):
        if lead.present and lead.radar:
          leads.append((lead.dRel, lead.yRel))
        if lead.present:
          gap_leads.append((lead.dRel, lead.yRel if lead.radar else None))
    v_ego = car_state.vEgo
    ego_lc = self._ego_lc.update(car_state, md, now)
    lanes = None
    if len(md.laneLines) >= 3 and len(md.laneLineProbs) >= 3:
      lanes = [(np.asarray(md.laneLines[i].x, dtype=float), np.asarray(md.laneLines[i].y, dtype=float) + camera_offset,
                float(md.laneLineProbs[i])) for i in (1, 2)]
      if model_updated and lanes[0][2] >= LINE_MIN_PROB and lanes[1][2] >= LINE_MIN_PROB and len(lanes[0][0]) > 1:
        w = float(np.interp(8., lanes[1][0], lanes[1][1]) - np.interp(8., lanes[0][0], lanes[0][1]))
        if 2.5 < w < 4.5:
          self._lane_w += 0.05 * (float(np.clip(w, 3.0, 4.0)) - self._lane_w)
    ref_y = path_y
    # 11/05 route 53 ramp (55 km/h, 3.0 m/s^2): a next-lane car 54 m ahead was "confirmed" (-0.56 braking) though it never
    # crossed its line: on a curve my predicted path's error grows ~ d^2 / 2R, so the offsets are only trusted up to
    # sqrt(CURVE_TRUST_K x R) (R 78 m: 28 m; straight: no limit in range)
    k = abs(2. * (path_y(30.) - path_y(0.)) / 900.)
    d_trust = (CURVE_TRUST_K / k) ** 0.5 if k > 1e-5 else MAX_D_M
    if TREND_REF_LANE and lanes is not None and lanes[0][2] >= LINE_MIN_PROB and lanes[1][2] >= LINE_MIN_PROB \
       and len(lanes[0][0]) > 1 and len(lanes[1][0]) > 1:
      (lx, ly, _lp), (rx, ry, _rp) = lanes
      ref_max = min(float(lx[-1]), float(rx[-1]))

      def ref_y(dd, lx=lx, ly=ly, rx=rx, ry=ry, ref_max=ref_max):
        if dd > ref_max:
          return path_y(dd)
        return 0.5 * (float(np.interp(dd, lx, ly)) + float(np.interp(dd, rx, ry)))

    out: list[CutObj] = []
    seen = set()
    for d, y, v, _sent, settling, cls, tid, vlat, lc in points:
      if not (0.5 < d < MAX_D_M):
        continue
      y_right = -y + camera_offset  # radar yRel is left-positive; the model path is right-positive (calibrated frame)
      off_signed = y_right - ref_y(d)
      off = abs(off_signed)
      # the gap rule's lead is my lead other than this object: 11/05 route 53 (75 km/h) the model's (vision) lead had already
      # moved to the car cutting in, 1 s before it crossed the line, so it was "beyond the lead" (itself) and never marked.
      # Only while it is still outside my lane line: inside, it is my lead (replay: else leads off-centre got marked, braking)
      self_lead = [abs(d - ld) < SELF_LEAD_D_M and (ly is None or abs(y - ly) < LEAD_MATCH_Y_M) for ld, ly in gap_leads]
      lead_d = next((ld for (ld, _ly), sl in zip(gap_leads, self_lead) if not sl), None)
      intr = self._intrusion(lanes, d, y_right, off_signed)
      is_self_lead = bool(self_lead) and self_lead[0]
      beyond = (lead_d is not None and d > lead_d - GAP_MARGIN_M.get(cls, GAP_MARGIN_DEFAULT_M)) or (is_self_lead and intr >= 0.)
      toward = vlat * (1. if off_signed > 0 else -1.) if vlat == vlat else float("nan")  # left-positive vlat
      o = CutObj(tid=tid, d=d, y_right=y_right, vrel=v, cls=cls, settling=settling, kind=None, arrow=0, intr=intr,
                 toward=toward, strict=False, in_lane=off < 1.8)
      out.append(o)
      if tid >= 0:
        if off >= CONFIRM_FROM:
          self._out_t[tid] = now
        # (11/05: + over the ground forward: oncoming / crossing cars at junctions were "confirmed", -1.0 braking)
        if not ego_lc and off < CONFIRM_OFF and CONFIRM_D[0] < d < CONFIRM_D[1] and v < CONFIRM_VREL and \
           v_ego > CUTIN_MIN_V_GROUND and v + v_ego > CUTIN_MIN_V_GROUND and now - self._out_t.get(tid, -99.) < CONFIRM_WIN and tid not in self._confirm_t:
          came_right = self._side.get(tid, 0) > 0
          self._confirm_t[tid] = (now, -1 if came_right else 1)
        if off >= CONFIRM_FROM:
          self._side[tid] = 1 if off_signed > 0 else -1
        ct = self._confirm_t.get(tid)
        if ct is not None and now - ct[0] < CONFIRM_SHOW_S and not beyond and d <= d_trust:
          o.kind, o.arrow, o.strict = "confirmed", ct[1], True
          seen.add(tid)
          continue
      if any(abs(d - ld) < LEAD_MATCH_D_M and abs(y - ly) < LEAD_MATCH_Y_M for ld, ly in leads):
        if tid >= 0:
          self._lead_t[tid] = now
        continue   # (not "seen": a lead's rule state is cleared, as in the UI since 10/31)
      if tid >= 0:
        seen.add(tid)
      cutin = self._cutin(tid, off, d, v, frame, fresh, settling, v_ego, toward, lc) \
        if (tid >= 0 and v_ego > CUTIN_MIN_V_EGO and v + v_ego > CUTIN_MIN_V_GROUND) else False
      qual = False
      if cutin:
        lines_ok = lanes is not None and lanes[0][2] >= LINE_MIN_PROB and lanes[1][2] >= LINE_MIN_PROB
        if ego_lc or intr < LINE_GATE_M or beyond or d > d_trust:
          cutin = False   # the rule's own state above keeps running
        elif now - self._lead_t.get(tid, -99.) < LEAD_DEPART_S:
          cutin = False   # my lead leaving (11/08)
        elif not lines_ok and NO_LINE_MIN_TOWARD is not None and not (toward == toward and toward >= NO_LINE_MIN_TOWARD):
          cutin = False   # no lane lines: the offset trend may be my path swinging (11/08)
        elif intr >= LINE_STRICT_M or (toward == toward and toward >= STRICT_TOWARD):
          qual = True
          cutin = now - self._qual_t.setdefault(tid, now) >= SHOW_AFTER_S
        else:
          cutin = False
      if not qual:
        self._qual_t.pop(tid, None)
      if cutin:
        o.kind = "predicted"
        o.arrow = -1 if off_signed > 0 else 1  # car to my right drifts left into my lane, and vice versa
        # braking only on a real lane line (11/04 replay: on the path-width fallback 13 of 31 noise marks asked for braking)
        line = self._line_intr(lanes, d, y_right, off_signed)
        o.strict = (line == line and line >= BRAKE_LINE_M) or lc in LC_CUTIN
    if fresh:
      for tid in [k for k, t in self._lead_t.items() if now - t > LEAD_DEPART_S]:
        self._lead_t.pop(tid, None)
      for tid in [k for k in self._hist if k not in seen]:
        for store in (self._hist, self._hold, self._release, self._slow, self._fast, self._out_t,
                      self._confirm_t, self._side, self._qual_t):
          store.pop(tid, None)
    return out

  def _intrusion(self, lanes, d: float, y_right: float, off_signed: float) -> float:
    """How far the object is past the lane line on its side (+ = inside my lane). Without a reliable line: my path
    +- half the filtered lane width, less LINE_PATH_EXTRA_M (my own place in the lane is unknown)."""
    if lanes is not None:
      x, y, p = lanes[1] if off_signed > 0 else lanes[0]
      if p >= LINE_MIN_PROB and len(x) > 1:
        ly = float(np.interp(d, x, y))
        return (ly - y_right) if off_signed > 0 else (y_right - ly)
    return self._lane_w / 2. - abs(off_signed) - LINE_PATH_EXTRA_M

  @staticmethod
  def _motion_ok(h, frame: int, d: float) -> bool:
    """MOTION_* check on the offset history h [(frame, |offset|)]: consistent, not jitter, above the noise floor."""
    if not MOTION_GATE:
      return True
    w = [o for f, o in h if frame - f <= MOTION_WIN_S * FRAME_HZ]
    if len(w) < 4:
      return False
    steps = np.diff(np.asarray(w, float))
    travel = float(np.abs(steps).sum())
    inward = float(-steps[steps < 0].sum())
    net = float(w[0] - w[-1])
    if travel <= 1e-6 or inward / travel < MOTION_MIN_CONSISTENCY:
      return False
    if travel >= JITTER_MIN_TRAVEL_M and net < JITTER_MIN_NET_FRACTION * travel:
      return False
    return net >= float(np.interp(d, NOISE_FLOOR_BP, NOISE_FLOOR_V))

  def _line_intr(self, lanes, d: float, y_right: float, off_signed: float) -> float:
    """As _intrusion, from a reliable model lane line only (nan without one, or beyond its range)."""
    if lanes is not None:
      x, y, p = lanes[1] if off_signed > 0 else lanes[0]
      if p >= LINE_MIN_PROB and len(x) > 1 and d <= float(x[-1]):
        ly = float(np.interp(d, x, y))
        return (ly - y_right) if off_signed > 0 else (y_right - ly)
    return float("nan")

  def _cutin(self, tid: int, off: float, d: float, vrel: float, frame: int, fresh: bool, settling: bool, v_ego: float = 99.,
             toward: float = float("nan"), lc: int = 0) -> bool:
    """Lateral-offset trend over CUTIN_WIN: True while the car has been drifting toward my path for CUTIN_HOLD_FRAMES."""
    h = self._hist.setdefault(tid, [])
    if fresh:
      h.append((frame, off))
      while h and frame - h[0][0] > CUTIN_WIN * FRAME_HZ:
        h.pop(0)
    ok = False
    # near trend: a slower car 0.8-2.6 m off my path, 25-90 m ahead, its offset shrinking over CUTIN_WIN
    near = CUTIN_OFF_MIN < off < CUTIN_OFF_MAX and vrel < CUTIN_VREL and CUTIN_D[0] <= d <= CUTIN_D[1]
    if len(h) >= 6 and not settling and near:
      t = np.array([x[0] for x in h], float) / FRAME_HZ; o = np.array([x[1] for x in h], float)
      slope = float(np.polyfit(t, o, 1)[0]) if t[-1] - t[0] > 0.5 else 0.
      ok = slope < CUTIN_RATE and self._motion_ok(h, frame, d)
    # close, moving in: within CLOSE_D / CLOSE_OFF, the radar's sideways speed toward me > SLOW_TOWARD for SLOW_HOLD_RECORDS
    close = CLOSE_OFF[0] < off < CLOSE_OFF[1] and CLOSE_D[0] <= d <= CLOSE_D[1]
    if fresh:
      self._slow[tid] = self._slow.get(tid, 0) + 1 if (close and toward == toward and toward > SLOW_TOWARD) else 0
      self._fast[tid] = self._fast.get(tid, 0) + 1 if (close and toward == toward and toward > FAST_TOWARD) else 0
    # the radar's own lane-crossing code (2 / 5)
    lc_ok = lc in LC_CUTIN and LC_OFF[0] < off < LC_OFF[1] and LC_D[0] <= d <= LC_D[1] and vrel < CUTIN_MAX_VREL
    moving_in = self._slow.get(tid, 0) >= SLOW_HOLD_RECORDS or self._fast.get(tid, 0) >= FAST_HOLD_RECORDS
    if lc_ok or (close and moving_in and self._motion_ok(h, frame, d)):
      ok = True
      if fresh:
        self._hold[tid] = max(self._hold.get(tid, 0), CUTIN_HOLD_FRAMES - 1)
    if fresh:
      held = self._hold.get(tid, 0)
      if ok:
        self._hold[tid] = held + 1
        if self._hold[tid] >= CUTIN_HOLD_FRAMES:
          self._release[tid] = CUTIN_RELEASE_FRAMES
      else:
        self._hold[tid] = 0
        if self._release.get(tid, 0) > 0:
          self._release[tid] -= 1
    return self._hold.get(tid, 0) >= CUTIN_HOLD_FRAMES or self._release.get(tid, 0) > 0


# ---- 11/04 planner response (driver: by my speed, the distance and its speed; sometimes a lift-off, sometimes light braking)
RESP_STOP_DISTANCE = 6.0   # m, = long_mpc STOP_DISTANCE
# m/s^2: the most a prediction alone may ask (a lift-off), by my speed. 11/04 coast_scan.py (the driver lifting off, no
# brake, openpilot off, 846 segments, 8 min): -0.22 at 10-20 km/h, -0.36..-0.41 at 20-80 km/h (no data above 80 yet).
RESP_LIFT_BP = [15 / 3.6, 25 / 3.6]
RESP_LIFT_V = [0.2, 0.35]
RESP_LIFT_MAX = 0.35       # the highest of RESP_LIFT_V
RESP_BRAKE_MAX = 1.0       # m/s^2: light braking, only for strict / confirmed objects; harder is the MPC's job once it is the lead
RESP_MIN = 0.05            # m/s^2: below this, nothing
RESP_ACC_WIN_S = 1.0       # s: relative-speed history for the object's relative acceleration
# 11/05 route 53 (driver: "several merges felt close"): at 80 km/h a car cut in at 19 m (0.9 s) but slightly faster than me,
# so nothing was "closing" and nothing happened; the MPC then kept the 20 m gap for 5 s. Gap term: an object inside the
# desired gap (t_follow x its speed + stop distance) asks for the relative speed that opens the gap in GAP_OPEN_S, reached
# in GAP_RESP_S. Not closing, it only gets a lift-off (lift_max). It also keeps acting for POST_HOLD_S after the marked object
# became my lead (the rule stops marking leads; the MPC alone was soft with an opening lead).
GAP_TERM = True
GAP_OPEN_S = 8.0
GAP_RESP_S = 2.0
POST_HOLD_S = 3.0
# 11/09 route 62 19:38:40 (driver: the merge rules also slow me early): a car merged 14.6 m ahead at 25 km/h, closing only
# 0.7 m/s, already at my desired gap (room -1.3 m). room <= 1 m took an emergency branch (need 2.0 whatever the closing
# speed) and braked -1.0 for a car I was hardly closing on. The closing term now uses room >= ROOM_FLOOR_M instead
# (closing^2 / 2 x 3 m: 0.7 m/s -> 0.08, the gap term then asks a lift; 2.8 m/s -> 1.3, still -1.0), continuous with the
# outside-gap formula. A car far inside the gap still gets the gap term. 0.0 = the old emergency branch.
ROOM_FLOOR_M = 3.0


def lift_max(v_ego: float) -> float:
  return float(np.interp(v_ego, RESP_LIFT_BP, RESP_LIFT_V))


class CutinResponse:
  """Deceleration a cut-in object needs: closing^2 / (2 x room), room = range - (t_follow x its speed + stop distance),
  less what its own relative acceleration (it speeding up) gives back; capped by confidence."""
  def __init__(self):
    self._vrel: dict[int, list[tuple[float, float]]] = {}
    self._marked: dict[int, tuple[float, bool]] = {}   # tid -> (last time marked, strict)

  def update(self, objs: list[CutObj], v_ego: float, t_follow: float, now: float):
    best = None  # (a_req capped, obj, a_req raw)
    live = set()
    for o in objs:
      if o.tid is None or o.tid < 0:
        continue
      live.add(o.tid)
      h = self._vrel.setdefault(o.tid, [])
      h.append((now, o.vrel))
      while h and now - h[0][0] > RESP_ACC_WIN_S:
        h.pop(0)
      if o.kind is not None:
        self._marked[o.tid] = (now, bool(o.strict))
      elif not (GAP_TERM and o.tid in self._marked and now - self._marked[o.tid][0] < POST_HOLD_S and o.in_lane):
        continue
      strict = bool(o.strict) or (o.kind is None and self._marked[o.tid][1])
      closing = -o.vrel
      if v_ego + o.vrel < CUTIN_MIN_V_GROUND:   # not moving forward (oncoming / crossing)
        continue
      a_rel = 0.
      if len(h) >= 5 and h[-1][0] - h[0][0] > 0.5:
        a_rel = float(np.polyfit([x[0] - h[0][0] for x in h], [x[1] for x in h], 1)[0])  # + = it is pulling away
      v_obj = max(v_ego + o.vrel, 0.)
      d_des = t_follow * v_obj + RESP_STOP_DISTANCE
      room = o.d - d_des
      a_close = 0.
      if closing > 0.:
        if ROOM_FLOOR_M > 0.:
          a_close = closing ** 2 / (2. * max(room, ROOM_FLOOR_M))
        else:
          a_close = closing ** 2 / (2. * room) if room > 1. else RESP_BRAKE_MAX * 2
        a_close = max(a_close - max(a_rel, 0.), 0.)
      a_gap = 0.
      if GAP_TERM and room < 0.:
        a_gap = max((-room / GAP_OPEN_S - o.vrel - max(a_rel, 0.) * GAP_RESP_S) / GAP_RESP_S, 0.)
      a_req = max(a_close, a_gap)
      if a_req < RESP_MIN:
        continue
      lim = RESP_BRAKE_MAX if (strict and closing > 0.) else lift_max(v_ego)
      cap = min(a_req, lim)
      if best is None or cap > best[0]:
        best = (cap, o, a_req)
    for tid in [k for k in self._vrel if k not in live]:
      self._vrel.pop(tid, None)
      self._marked.pop(tid, None)
    return best
