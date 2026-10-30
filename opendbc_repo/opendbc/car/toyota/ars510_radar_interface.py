"""openpilot radar tracks from the Toyota / Continental ARS510: the ACC target (`acc`, default) or the native object list (`fused`, `raw`).

Installed by `openpilot/install.py` as `opendbc/car/toyota/ars510_radar_interface.py`, together with the `ars510`
decoder package as `opendbc/car/toyota/ars510/`. The installer appends a marked block to the end of
`opendbc/car/toyota/interface.py` that calls `hook_car_interface(CarInterface)`; every other file of the fork stays as shipped,
so the same install works on openpilot and its forks (sunnypilot, StarPilot, CarrotPilot, FrogPilot-style trees):
  - detection: the radar-ACC platforms (ARS510_PLATFORMS: RAV4 2022 and 2023-25, C-HR, Yaris) whose radar frames
    reach bus 1 during fingerprinting get radarUnavailable = False (stock keeps radar-ACC Toyotas radar-unavailable);
  - tracks: `CarInterface.RadarInterface` becomes a thin dispatcher that builds `Ars510RadarInterface` for such a car
    and the fork's own RadarInterface otherwise. ToyotaFlags stays as shipped, so fork flag bits stay free.
  - PCS alarm: the radar's alarm bit on bus 1 (0x100) sets carState.stockFcw, as 0x411 FCW does with the radar enabled.
    Upstream this is part of commaai/opendbc#3860, in Toyota's carstate (upstream/opendbc_ars510.patch); the hook does the same after the
    fork's update, since the installer leaves the fork's carstate as shipped.
Worth knowing (docs/07, docs/08, docs/12):
  - the object list's vRel has occasional 1-10 s excursions (mostly false closings beyond 40 m, docs/07): `fused` filters them with the ACC target's speed, and the default `acc` publishes the ACC target alone;
  - the radar deletes most new stationary objects at age 5, more often the faster ego drives (7 % kept above 10 m/s),
    so a car that was already stopped when it came into view comes mostly from vision;
  - radard weights lateral position only through the camera's uncertainty when it picks the lead; `fused` adds a
    yaw-rate path gate.

The object-list profiles (`fused`, `raw`) parse from card's raw `can_packets`: the object list is ONE 742-byte record sent as 106 ISO-TP-style frames on 0x80 every
~60 ms. CANParser keeps the latest value per address (or a per-call list in vl_all), and a DBC maps address and bits, while the
part of the record a frame carries follows from its sequence nibble (which wraps every 16 frames). The frames are reassembled from the
raw `can_packets` card.py passes in, CRC-checked, and decoded in `ars510`. The DBCs in `dbc/` are for cabana; the `acc`
profile parses its three 50 Hz ACC target messages with CANParser and `toyota_tss2_ars510.dbc` (installed as
`ars510_acc_target.dbc`, so opendbc's own copy of that DBC stays as shipped).

Output cadence: one RadarData per completed record (~16.6 Hz, inside radard's 8-24 Hz radarTracks window).
- After power-up the radar sends its first object record ~6 s after first CAN. Until the first valid record, this
  behaves like openpilot's radarless default: empty RadarData at 20 Hz, error-free status, vision-only leads. A warning is
  logged when the record count is still 0 at NO_RECORD_WARN_S after start (e.g. wiring, or firmware that also silences bus 1 on
  the alpha-longitudinal radar disable; on 8821F0R03100 bus 1 keeps running).
- Once records have arrived and STALE_S passes between two of them, a radarUnavailableTemporary RadarData is sent at 20 Hz
  until records resume, so radard sees that report in place of the last tracks.
- A CRC-failed record is dropped; the staleness check covers sustained loss.
- Every profile: while the radar's own fault flag (0x196 byte 1 bit 0) is set on two or more consecutive cycles, the
  only report is radarUnavailableTemporary with no points, at 20 Hz. The radar holds it
  through minutes in which it declares itself blocked (car-bus 0x411 dust), sometimes with no ACC target for a lead
  that the object list still tracks (docs/05).
"""
import importlib
from collections import deque
from dataclasses import replace

from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.toyota.ars510 import BASE_CONFIG, FUSED_CONFIG, Ars510NativeRadarInterface
from opendbc.car.toyota.ars510.constants import (ACC_TARGET_POS_ADDR, ACC_TARGET_VREL_ADDR, CAR_BUS, ID80_ADDR, PCS_STATE_ADDR, RADAR_BUS,
                                                 RADAR_FAULT_ADDR, SUMMARY_ADDRS, TOYOTA_KINEMATICS_ADDR, TOYOTA_SPEED_ADDR)
from opendbc.car.toyota.ars510.support import RadarFaultFlag, pcs_alarm

# Decoder profile (docs/08): "acc" (default: the radar's ACC target alone as one point, ars510_acc_target.py, docs/17),
# "fused" (FUSED_CONFIG: one Kalman speed filter fusing the object list and the ACC target by the radar's own
# uncertainty, plus a path gate, docs/12), "raw" (BASE_CONFIG: the unfiltered radar decode, research only) or
# "openpilot" (fused in one file, ars510_upstream.py); "acc_summaries" adds the radar's summary cars to acc (docs/11);
# "shared_speed" / "shared_range" are research files with one gate-free Kalman model
# over every object (docs/17). `install.py --profile` rewrites this one line.
STANDALONE = {"acc": "ars510_acc_target", "acc_summaries": "ars510_acc_summaries", "shared_speed": "ars510_shared_speed", "shared_range": "ars510_shared_range"}
PROFILES = {"fused": FUSED_CONFIG, "openpilot": None, "raw": BASE_CONFIG, **{k: k for k in STANDALONE}}
PROFILE = PROFILES["acc_summaries"]
# dimpleliang/ars510 (10/29): `acc_summaries` drives radard (upstream withdrew `fused`: on a second RAV4's 38 drives it added
# ~2 s/h of unnecessary braking over `acc`, incl. one hard brake, from object-list speed excursions, mostly beyond 40 m).
# The fused object-list decoder still runs beside it, for the display (every object, as before) and for one thing only on
# radarTracks: two-wheelers it publishes (settled, path-gated) within VRU_MAX_D_M, as extra points with trackId
# VRU_TRACK_ID_BASE + id, for radard's in-lane two-wheeler override (the camera lead is often the car beyond a scooter;
# the ACC target and the summaries usually hold the car). A VRU point within SAME_D_M / SAME_Y_M of an ACC / summary point
# is that object and is not added. Set STANDALONE_DISPLAY = False to run the standalone profile alone.
STANDALONE_DISPLAY = True
VRU_CLASSES = (5, 6)  # radar object class 163|3: provisional two-wheeler, two-wheeler
VRU_MAX_D_M = 40.0
VRU_TRACK_ID_BASE = 1 << 30  # must match radard.VRU_TRACK_ID_BASE; above every standalone (0-127) and fused trackId, below Int32 radarTrackId
VRU_SAME_D_M, VRU_SAME_Y_M = 1.0, 0.5
# Far stopped cars (10/29): the object list's stopped / crawling tracks it publishes beyond FAR_MIN_D_M (path-gated; beyond
# 60 m only these, far_slow_keep_mps) go to radarTracks too, with trackId FAR_TRACK_ID_BASE + id, but only when the range
# itself confirms the speed: over the last FAR_CHECK_S the least-squares range rate must agree with vRel within
# FAR_RATE_ENTER_MPS to start publishing a track, and within FAR_RATE_KEEP_MPS to keep it. 10/29 recorded data (stationary
# tracks): |range rate - vRel| median 0.5-1.2 m/s, p90 2.2-3.8 m/s at 20-80 m, from range noise (~1 m at 40-70 m); a single
# 1 s / 2 m/s gate published 73 % of the driving candidate cycles and dropped tracks 3 times, 1.5 s / 3 / 4 published 91 %
# with 1 drop (6 tracks, thin data). Typical object-list excursions are -3 to -5 m/s and ramp up over ~1 s. The object list's speed excursions are what upstream withdrew `fused` for, and the worst one (closing
# 23.7 m/s at a constant range, with ego at about that speed) looks exactly like a stopped car by speed alone; its range
# rate (~0) disagrees with its vRel by ~24 m/s. radard still only uses a track that matches the camera's lead.
FAR_MIN_D_M = 40.0
FAR_STOPPED_MPS = 3.0       # |over-ground speed| below this
FAR_CHECK_S = 1.5
FAR_CHECK_MIN_SPAN_S = 1.35
FAR_CHECK_MIN_N = 15
FAR_RATE_ENTER_MPS = 3.0
FAR_RATE_KEEP_MPS = 4.0
FAR_TRACK_ID_BASE = 1 << 29  # below VRU_TRACK_ID_BASE, so radard does not take these for two-wheelers

# Radar-ACC platforms: every radar firmware they list (RAV4 8821F0R01100 / 8821F0R03100, C-HR 8821FF41xx00, Yaris
# 8821F0D0xx00) is the Continental ARS510 sending the same bus-1 messages (docs/01), so the car model selects the parser.
# The RAV4s carry the replay and road evidence; C-HR and Yaris are enabled for testing.
ARS510_PLATFORMS = {"TOYOTA_RAV4_TSS2_2022", "TOYOTA_RAV4_TSS2_2023", "TOYOTA_CHR_TSS2", "TOYOTA_YARIS"}
# The object-list profiles need the 0xB4 wheel speed, which the Yaris does not send: it runs `acc` whatever the profile.
ACC_ONLY_PLATFORMS = {"TOYOTA_YARIS"}

# far in-path objects, logging only (10/30): object-list points at >= FAR_LOG_MIN_D_M within |y| FAR_LOG_MAX_Y_M, closing
# faster than FAR_LOG_MAX_VREL, ego above FAR_LOG_MIN_V, for FAR_LOG_N records in a row -> carlog "ARS510 far object on/off".
# Offline, these are matched to the lead openpilot later takes, to see whether a gentle early approach would pay off.
FAR_LOG_MIN_D_M = 120.0
FAR_LOG_KEEP_D_M = 80.0  # once on, kept down to this range (bridges to the camera's lead)
FAR_LOG_MAX_Y_M = 4.0  # loose here (curves); the planner gates against the model path (FAR_OBJ_PATH_Y_M)
FAR_LOG_MAX_VREL = -1.5
FAR_LOG_MIN_V = 70.0 / 3.6
FAR_LOG_N = 5
# the far object (coast only) does not wait for the publish age (20 records, 1.2 s): a radar track of FAR_LOG_MIN_AGE
# records (0.5 s) that the radar already calls a car / large vehicle with existence >= FAR_LOG_MIN_EXIST qualifies.
# 15:04: the track started at 09.68, was a settled car from 10.03, the age-20 gate fired at 11.08.
FAR_LOG_MIN_AGE = 8
FAR_LOG_MIN_EXIST = 60
FAR_LOG_CLASSES = (2, 3)
# the logged far object (while "on") also goes to radarTracks as one point with this trackId. radard leaves it out of
# its tracks (no lead from it); the planner only uses it to ease off early (FAR_RADAR_ONLY_MAX_DECEL there).
FAR_OBJ_TRACK_ID = 1 << 28
# cut-in candidate (coast only, 10/30): the closest object-list car at CUTIN_D_M that is partly in the ego lane
# (|y| < CUTIN_MAX_Y_M here; the planner gates 1.0-2.6 m from the model path), closing faster than CUTIN_MAX_VREL, and not
# one of the published (ACC / summary) cars. radarTracks point with trackId CUTIN_TRACK_ID; radard ignores it.
# 46:00 on 10/30: a car merged from the right over 9 s (y -2.1 -> -0.7) before the radar's ACC took it at 65 m.
CUTIN_TRACK_ID = 1 << 27
CUTIN_D_M = (25.0, 90.0)
CUTIN_MAX_Y_M = 3.5
CUTIN_MAX_VREL = -2.0
STALE_S = 0.5  # radard leaves track-content staleness to this interface
NO_RECORD_WARN_S = 15.0
EMPTY_PERIOD = 5  # during record gaps, report every 5th update (card calls update at 100 Hz -> 20 Hz)
# 0x24 (yaw rate) feeds yvRel, which forks with the legacy RadarPoint fields (sunnypilot) publish
WANTED = {(RADAR_BUS, ID80_ADDR), (CAR_BUS, TOYOTA_SPEED_ADDR), (CAR_BUS, TOYOTA_KINEMATICS_ADDR), (RADAR_BUS, ACC_TARGET_VREL_ADDR),
          (RADAR_BUS, ACC_TARGET_POS_ADDR)} | {(RADAR_BUS, a) for a in SUMMARY_ADDRS}


# ARS510 display: every settled radar object (before the path gates) goes to the UI on customReservedRawData0, a
# service openpilot reserves for forks. Display only: radarTracks, which radard and the controls use, is unchanged.
# Payload: b"A5" + version byte + count byte, then per object <fffB: dRel, yRel, vRel, flags (bit 0 = sent to radard, bit 1 = still settling,
# bits 2-4 = radar object class: 1 unclassified, 2 car, 3 large vehicle, 4 pedestrian, 5 provisional, 6 two-wheeler).
DISPLAY_SERVICE = "customReservedRawData0"
DISPLAY_FMT = "<fffB"


class _DisplayPublisher:
  def __init__(self):
    self.pm = None
    self.failed = False

  def send(self, points) -> None:
    if self.failed:
      return
    try:
      if self.pm is None:
        from openpilot.cereal import messaging
        self.pm = messaging.PubMaster([DISPLAY_SERVICE])
        self._messaging = messaging
      import struct
      pts = points[:64]
      payload = b"A5" + bytes([1, len(pts)]) + b"".join(
        struct.pack(DISPLAY_FMT, d, y, v if v == v else 0.0, (1 if sent else 0) | (2 if settling else 0) | ((cls & 7) << 2))
        for d, y, v, sent, settling, cls in pts)
      msg = self._messaging.new_message(None, valid=True)  # Data field: set, not init()
      setattr(msg, DISPLAY_SERVICE, payload)
      self.pm.send(DISPLAY_SERVICE, msg)
    except Exception:
      self.failed = True
      carlog.exception("ARS510: display publisher disabled")


class Ars510RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, *args, **kwargs):  # forks add arguments (sunnypilot: CP_SP)
    super().__init__(CP, *args, **kwargs)
    self.standalone = None
    profile = "acc" if str(CP.carFingerprint) in ACC_ONLY_PLATFORMS else PROFILE
    if isinstance(profile, str):  # a standalone file: a complete RadarInterface of its own (frames, cadence, staleness)
      module = importlib.import_module(f"opendbc.car.toyota.{STANDALONE[profile]}")
      self.standalone = module.RadarInterface(CP, *args, **kwargs)
      self.standalone_fills_legacy = hasattr(module, "LEGACY")  # acc_summaries sets aRel / yvRel / measured itself
      self.ars = None
      if STANDALONE_DISPLAY and str(CP.carFingerprint) not in ACC_ONLY_PLATFORMS:
        self.ars = Ars510NativeRadarInterface(replace(FUSED_CONFIG, include_metadata=False))
      self.vru_points: list = []
      self.far_points: list = []
      self.range_hist: dict = {}  # fused trackId -> deque of (record time, dRel)
      self.far_active: set = set()  # fused trackIds currently published as far stopped cars
      self.vru_time_s: float | None = None
      self.last_out: list[tuple[float, float]] = []  # (dRel, yRel) of the last radarTracks points from the standalone profile
    else:
      self.ars = UpstreamCandidate() if profile is None else Ars510NativeRadarInterface(replace(profile, include_metadata=False))
    self.display = _DisplayPublisher()
    self.start_s: float | None = None
    self.last_record_s: float | None = None
    self.warned = False
    self.fault = RadarFaultFlag()
    self.calls = 0

  def update(self, can_packets):
    self.calls += 1
    for _, frames in can_packets:
      for address, dat, src in frames:
        if address == RADAR_FAULT_ADDR and src == RADAR_BUS:
          self.fault.update(dat)
    return self._with_fault(self._update(can_packets))

  def _with_fault(self, ret):
    if not self.fault.active:
      return ret
    if self.calls % EMPTY_PERIOD != 0:  # one report per 5 updates (20 Hz), so radard's 8-24 Hz radarTracks check holds
      return None
    ret = structs.RadarData()
    ret.errors.radarUnavailableTemporary = True
    return ret

  def _update(self, can_packets):
    if self.standalone is not None:
      now_s = self._feed_display(can_packets) if self.ars is not None else None
      ret = self.standalone.update(can_packets)
      if ret is not None and ret.points and LEGACY_FIELDS and not self.standalone_fills_legacy:  # as for the upstream version
        ret.points = [_point({"trackId": p.trackId, "dRel": p.dRel, "yRel": p.yRel, "vRel": p.vRel, **ACC_LEGACY}) for p in ret.points]
      if ret is not None and self.ars is not None:
        ret = self._add_vru(ret, now_s)
      return ret
    self.frame += 1
    latest = None
    now_s = None
    for nanos, frames in can_packets:
      now_s = nanos * 1e-9
      # card.py passes plain (address, data, src) tuples; CanData is a NamedTuple in the same order
      for address, dat, src in frames:
        if (src, address) in WANTED:
          out = self.ars.update_frame(now_s, src, address, dat)
          if out is not None:
            latest = out
            display = getattr(self.ars, "display_points", None)
            if display is not None:
              self.display.send(display)
    if now_s is None:
      return None
    if self.start_s is None:
      self.start_s = now_s

    if latest is None:
      if self.frame % EMPTY_PERIOD != 0:
        return None
      if self.last_record_s is None:  # radar still starting (or silenced): radarless behaviour, error-free status
        if not self.warned and now_s - self.start_s > NO_RECORD_WARN_S:
          carlog.warning("ARS510: waiting for the first valid 0x80 object record on bus 1; radar tracks unavailable until then")
          self.warned = True
        return structs.RadarData()
      if now_s - self.last_record_s > STALE_S:
        ret = structs.RadarData()
        ret.errors.radarUnavailableTemporary = True
        return ret
      return None

    self.last_record_s = latest["time_s"]
    ret = structs.RadarData()
    # Every profile withholds points with a NaN vRel, which would poison radard's per-track Kalman filter.
    ret.points = [_point(p) for p in latest["radarData"]["points"]]
    return ret

  def _feed_display(self, can_packets):
    """Object-list decoder beside a standalone profile: display every object, keep the in-range two-wheelers it publishes."""
    now_s = None
    for nanos, frames in can_packets:
      now_s = nanos * 1e-9
      for address, dat, src in frames:
        if (src, address) not in WANTED:
          continue
        out = self.ars.update_frame(now_s, src, address, dat)
        if out is None:
          continue
        display = getattr(self.ars, "display_points", None) or []
        cls = {(row[0], row[1]): row[5] for row in display}
        self.vru_points = [p for p in out["radarData"]["points"]
                           if cls.get((p["dRel"], p["yRel"])) in VRU_CLASSES and p["dRel"] <= VRU_MAX_D_M]
        self.vru_time_s = out["time_s"]
        self.far_points = self._far_stopped(out)
        self._log_far(out, display)
        vru_xy = {(p["dRel"], p["yRel"]) for p in self.vru_points}
        for row in display:  # "sent to radard": a VRU point, or the object the ACC target / a summary reports
          row[3] = (row[0], row[1]) in vru_xy or any(abs(row[0] - d) < VRU_SAME_D_M and abs(row[1] - y) < VRU_SAME_Y_M
                                                       for d, y in self.last_out)
        self.display.send(display)
    return now_s


  def _log_far(self, out, display) -> None:
    """Far object for the planner's coast (and its log). Beyond 60 m the fused decoder publishes only the ACC target, so
    this reads every decoded object: the closest car past FAR_LOG_MIN_D_M within |y| FAR_LOG_MAX_Y_M, closing faster than
    FAR_LOG_MAX_VREL, for FAR_LOG_N records in a row -> on; FAR_LOG_N records without one -> off."""
    try:
      t = out["time_s"]
      v_ego = self.ars._fresh_ego_speed(t)
      st = self.__dict__.setdefault("far_log", {"n": 0, "miss": 0, "on": False, "t0": 0.0, "d0": 0.0, "last": None})
      min_d = FAR_LOG_KEEP_D_M if st["on"] else FAR_LOG_MIN_D_M
      rows = getattr(self.ars, "far_rows", None) or []
      cut = [r for r in rows if v_ego is not None and v_ego > FAR_LOG_MIN_V and r[3] >= FAR_LOG_MIN_AGE and r[4] >= FAR_LOG_MIN_EXIST
             and r[5] in FAR_LOG_CLASSES and CUTIN_D_M[0] <= r[0] < CUTIN_D_M[1] and abs(r[1]) < CUTIN_MAX_Y_M and r[2] == r[2]
             and r[2] < CUTIN_MAX_VREL and not any(abs(r[0] - d) < 6.0 and abs(r[1] - y) < 1.5 for d, y in self.last_out)]
      self.cutin_points = [{"trackId": 0, "dRel": float(c[0]), "yRel": float(c[1]), "vRel": float(c[2])} for c in sorted(cut)[:1]]
      cands = [r for r in rows if v_ego is not None and v_ego > FAR_LOG_MIN_V and r[3] >= FAR_LOG_MIN_AGE and r[4] >= FAR_LOG_MIN_EXIST
               and r[5] in FAR_LOG_CLASSES and r[0] >= min_d and abs(r[1]) < FAR_LOG_MAX_Y_M and r[2] == r[2] and r[2] < FAR_LOG_MAX_VREL]
      if st["on"] and st["last"] is not None:  # stay on the same car: within 10 m of its last range
        cands = [r for r in cands if abs(r[0] - st["last"][0]) < 10.0]
      if cands:
        r = min(cands, key=lambda r: r[0])
        if st["n"] == 0:
          st["t0"], st["d0"] = t, r[0]
        st["n"] += 1; st["miss"] = 0; st["last"] = r
        if not st["on"] and st["n"] >= FAR_LOG_N:
          st["on"] = True
          carlog.info(f"ARS510 far object on: d {r[0]:.1f} y {r[1]:.1f} vRel {r[2]:.1f} class {r[5]} v_ego {v_ego:.1f}")
        self.far_obj_points = [{"trackId": 0, "dRel": float(r[0]), "yRel": float(r[1]), "vRel": float(r[2])}] if st["on"] else []
      else:
        st["miss"] += 1
        self.far_obj_points = []
        if st["miss"] >= FAR_LOG_N:
          if st["on"]:
            r = st["last"]
            carlog.info(f"ARS510 far object off: after {t - st['t0']:.1f} s, d {st['d0']:.1f} -> {r[0]:.1f} y {r[1]:.1f}")
          st["n"] = 0; st["on"] = False
    except Exception:
      self.far_obj_points = []
      self.cutin_points = []
      carlog.exception("ARS510 far object log")

  def _far_stopped(self, out) -> list:
    t = out["time_s"]
    v_ego = self.ars._fresh_ego_speed(t)
    seen, far = set(), []
    for p in out["radarData"]["points"]:
      tid = p["trackId"]
      seen.add(tid)
      hist = self.range_hist.setdefault(tid, deque())
      hist.append((t, p["dRel"]))
      while hist and t - hist[0][0] > FAR_CHECK_S:
        hist.popleft()
      active = tid in self.far_active
      if v_ego is None or p["dRel"] <= FAR_MIN_D_M or abs(p["vRel"] + v_ego) >= FAR_STOPPED_MPS:
        self.far_active.discard(tid)
        continue
      if len(hist) < FAR_CHECK_MIN_N or hist[-1][0] - hist[0][0] < FAR_CHECK_MIN_SPAN_S:
        continue
      tm = sum(h[0] for h in hist) / len(hist)
      dm = sum(h[1] for h in hist) / len(hist)
      rate = sum((h[0] - tm) * (h[1] - dm) for h in hist) / sum((h[0] - tm) ** 2 for h in hist)
      err = abs(rate - p["vRel"])
      if err < (FAR_RATE_KEEP_MPS if active else FAR_RATE_ENTER_MPS):
        if not active:
          self.far_active.add(tid)
          carlog.info(f"ARS510 far stopped car on: {p['dRel']:.1f} m, vRel {p['vRel']:.1f}, range rate {rate:.1f}, v_ego {v_ego:.1f}")
        far.append(p)
      elif active:
        self.far_active.discard(tid)
        carlog.info(f"ARS510 far stopped car off: {p['dRel']:.1f} m, vRel {p['vRel']:.1f}, range rate {rate:.1f}")
    for tid in list(self.range_hist):
      if tid not in seen:
        del self.range_hist[tid]
        self.far_active.discard(tid)
    return far

  def _add_vru(self, ret, now_s):
    # Snapshot the published points as plain values: with capnp-backed structs (sunnypilot) re-assigning the list
    # re-initializes it and the old element builders read back as zeros (10/30: the ACC point became 0 m / 0 m/s
    # whenever a far object or VRU point was appended, so radard lost the radar lead exactly then).
    base = [_point({**ACC_LEGACY, **{k: getattr(q, k) for k in ("trackId", "dRel", "yRel", "vRel", *LEGACY_FIELDS)}}) for q in ret.points]
    self.last_out = [(p.dRel, p.yRel) for p in base]
    if ret.errors.radarUnavailableTemporary or self.vru_time_s is None or now_s is None or now_s - self.vru_time_s > STALE_S:
      return ret
    extra = []
    for points, id_base in ((self.vru_points, VRU_TRACK_ID_BASE), (self.far_points, FAR_TRACK_ID_BASE),
                            (getattr(self, "far_obj_points", []), FAR_OBJ_TRACK_ID),
                            (getattr(self, "cutin_points", []), CUTIN_TRACK_ID)):
      for p in points:
        if any(abs(p["dRel"] - q.dRel) < VRU_SAME_D_M and abs(p["yRel"] - q.yRel) < VRU_SAME_Y_M for q in base):
          continue
        extra.append(_point({**ACC_LEGACY, **p, "trackId": id_base + int(p["trackId"])}))
    if extra:
      ret.points = base + extra
    return ret


def _point(p: dict) -> structs.RadarData.RadarPoint:
  """RadarPoint is trackId / dRel / yRel / vRel; forks that still carry the legacy aRel / yvRel / measured fields get them too."""
  pt = structs.RadarData.RadarPoint()
  pt.trackId, pt.dRel, pt.yRel, pt.vRel = p["trackId"], p["dRel"], p["yRel"], p["vRel"]
  for legacy in LEGACY_FIELDS:
    setattr(pt, legacy, p[legacy])
  return pt


class UpstreamCandidate:
  """The single-file fused version behind the decoder's update_frame call (aRel / yvRel at their legacy defaults)."""
  def __init__(self):
    from opendbc.car.toyota.ars510_upstream import Ars510Radar
    self.radar = Ars510Radar()

  def update_frame(self, time_s, bus, address, dat):
    points = self.radar.update(time_s, bus, address, dat)
    if points is None:
      return None
    return {"time_s": self.radar.record_t, "radarData": {"points": [
      {"trackId": i, "dRel": d, "yRel": y, "vRel": v, "aRel": float("nan"), "yvRel": float("nan"), "measured": True}
      for i, d, y, v in points]}}


def _legacy_fields() -> tuple[str, ...]:
  pt = structs.RadarData.RadarPoint()
  return tuple(f for f in ("aRel", "yvRel", "measured") if hasattr(pt, f))


LEGACY_FIELDS = _legacy_fields()
ACC_LEGACY = {"aRel": float("nan"), "yvRel": float("nan"), "measured": True}


# ---- generic hook (called from the block install.py appends to opendbc/car/toyota/interface.py) ------------------
def _radar_acc(flags) -> bool:
  from opendbc.car.toyota.values import ToyotaFlags
  return bool(int(flags) & ToyotaFlags.RADAR_ACC)


# The radar's cycle frames on bus 1 (cycle header, target summaries, ACC target) start 0.14-1.0 s after the first CAN frame,
# inside openpilot's fingerprint window on all 74 recorded cold starts; the object list (0x80) starts only after ~6 s.
RADAR_BUS1_EARLY = {0x190, 0x191, 0x193, ACC_TARGET_VREL_ADDR, ACC_TARGET_POS_ADDR, 0x239}


def ars510_platform(CP) -> bool:
  return _radar_acc(CP.flags) and str(CP.carFingerprint) in ARS510_PLATFORMS


def detect(ret, fingerprint, car_fw) -> bool:
  """ARS510: a radar-ACC RAV4 2022 / 2023-25, C-HR or Yaris (the car model) whose radar frames reach bus 1. Without them
  (wiring, harness) the car gets a radarless setup with vision leads: a radar that is declared but never reports
  leaves radarState invalid, and openpilot then refuses to engage (commIssue). A fork that passes no fingerprint gets the
  car model alone."""
  if not ars510_platform(ret):
    return False
  return fingerprint is None or bool(RADAR_BUS1_EARLY & set(fingerprint.get(RADAR_BUS, {})))


def is_ars510(CP) -> bool:
  """An ARS510 platform the hook left with a radar. Radar-ACC cars outside ARS510_PLATFORMS keep the fork's interface,
  whatever radar opendbc declares for them."""
  return ars510_platform(CP) and not CP.radarUnavailable


def hook_car_interface(car_interface):
  """Wrap a fork's Toyota CarInterface in place: detection in _get_params, dispatch in RadarInterface, PCS alarm in update."""
  if getattr(car_interface, "_ars510_hooked", False):
    return car_interface
  import inspect
  get_params = inspect.getattr_static(car_interface, "_get_params")  # the fork's class, or a base class it inherits from
  get_params = getattr(get_params, "__func__", get_params)
  sig = inspect.signature(get_params)

  def _get_params(*args, **kwargs):
    ret = get_params(*args, **kwargs)
    bound = sig.bind_partial(*args, **kwargs).arguments
    # Set both ways: stock keeps radar-ACC Toyotas radarless, and opendbc with its own ARS510 radar
    # (commaai/opendbc#3860) declares the radar without checking that its frames arrive.
    if ars510_platform(ret):
      ret.radarUnavailable = not detect(ret, bound.get("fingerprint"), bound.get("car_fw"))
    return ret

  fork_radar_interface = car_interface.RadarInterface

  class RadarInterface(fork_radar_interface):
    def __new__(cls, CP, *args, **kwargs):
      if is_ars510(CP):
        return Ars510RadarInterface(CP, *args, **kwargs)
      return fork_radar_interface(CP, *args, **kwargs)

  fork_update = car_interface.update

  def update(self, can_packets, *args, **kwargs):
    ret = fork_update(self, can_packets, *args, **kwargs)
    if is_ars510(self.CP):
      # as the carstate in upstream/opendbc_ars510.patch, read from the raw frames so a quiet 0x100 can't mark CAN invalid
      for _, frames in can_packets:
        for address, dat, src in frames:
          if address == PCS_STATE_ADDR and src == RADAR_BUS:
            self.ars510_pcs_alarm = pcs_alarm(dat)
      cs = ret[0] if isinstance(ret, tuple) else ret  # sunnypilot and StarPilot return (CarState, extra)
      cs.stockFcw = cs.stockFcw or getattr(self, "ars510_pcs_alarm", False)
    return ret

  car_interface._get_params = staticmethod(_get_params)
  car_interface.RadarInterface = RadarInterface
  car_interface.update = update
  car_interface._ars510_hooked = True
  return car_interface
