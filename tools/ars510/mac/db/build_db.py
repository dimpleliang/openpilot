#!/usr/bin/env python3
"""Build a parquet database from openpilot rlog segments for ARS510 radar analysis.

Usage: python3 build_db.py [--workers N] [--limit N] [--only SEGMENT ...] [--force]
Incremental: a segment directory with a `_DONE` marker is skipped. A lock file stops two
concurrent runs (e.g. a 15-minute scheduler) from stepping on each other.
Env: RLOG_RAW (default <dir of this script>/../raw), RLOG_DB (default directory of this script),
ARS510_OP_ROOT (sparse clone of dimpleliang/openpilot ars510-dev; default <dir of this script>/../op, falling back to
$HOME/op). Runs natively on the Mac (venv, launchd via ts_pull.sh -> post_pull.sh) and in the Linux VM (mount).
"""
from __future__ import annotations

import argparse, fcntl, json, math, os, sys, time, traceback
from pathlib import Path

HOME = Path(os.environ.get("HOME", "/root"))
HERE = Path(__file__).resolve().parent      # .../ars510_rlogs/db
ROOT = HERE.parent                           # .../ars510_rlogs (Mac: ~/ars510_rlogs, VM: ~/mnt/ars510_rlogs)
RAW = Path(os.environ.get("RLOG_RAW", ROOT / "raw"))
DB = Path(os.environ.get("RLOG_DB", HERE))


def _default_op_root() -> Path:
    for cand in (ROOT / "op", HOME / "op"):
        if (cand / "openpilot/cereal/log.capnp").exists():
            return cand
    return ROOT / "op"


OP = Path(os.environ.get("ARS510_OP_ROOT") or os.environ.get("OP_ROOT") or _default_op_root())
ARS510_SRC = OP / "opendbc_repo/opendbc/car/toyota/ars510"

RADAR_BUS = 1
CAR_BUS = 0
CAR_ADDRS = {0x24, 0xB4, 0x3B7, 0x1C4, 0x411}
CAR_DECIMATE = 5  # carState 100 Hz -> 20 Hz


def setup_imports() -> None:
    """openpilot.cereal / opendbc on sys.path; the four ARS510 decode modules as a synthetic package `ars510` whose
    __path__ is the source directory (python3.10 cannot import opendbc.car: it needs enum.ReprEnum; the modules only
    import each other, so a package pointing at the directory is enough and no symlinks are needed)."""
    import types
    if not (OP / "openpilot/cereal/log.capnp").exists():
        sys.exit(f"ARS510_OP_ROOT={OP}: openpilot/cereal/log.capnp not found (clone dimpleliang/openpilot ars510-dev there)")
    for p in (str(OP), str(OP / "opendbc_repo")):
        if p not in sys.path:
            sys.path.insert(0, p)
    if "ars510" not in sys.modules:
        pkg = types.ModuleType("ars510")
        pkg.__path__ = [str(ARS510_SRC)]
        pkg.__package__ = "ars510"
        sys.modules["ars510"] = pkg


setup_imports()

import numpy as np  # noqa: E402
import pyarrow as pa  # noqa: E402
import pyarrow.parquet as pq  # noqa: E402
import zstandard  # noqa: E402
from ars510 import constants as C, objects as O, record as R, transport as T  # noqa: E402

PQ_OPTS = dict(compression="zstd", compression_level=6)

# objects.parquet columns: every NativeObject field (tuple expanded) + raw codes of every NAMED_FIELD.
OBJ_SCALARS = [f for f in O.NativeObject.__dataclass_fields__ if f not in ("slot", "raw_weights148")]
CODE_NAMES = [f.name for f in O.NAMED_FIELDS]


def read_rlog(path: Path) -> tuple[bytes, bool]:
    raw = path.read_bytes()
    try:
        return zstandard.ZstdDecompressor().stream_reader(raw).read(), False
    except Exception:
        try:
            dobj = zstandard.ZstdDecompressor().decompressobj()
            return dobj.decompress(raw), True
        except Exception:
            out = bytearray()
            dobj = zstandard.ZstdDecompressor().decompressobj()
            for i in range(0, len(raw), 1 << 20):
                try:
                    out += dobj.decompress(raw[i:i + (1 << 20)])
                except Exception:
                    break
            return bytes(out), True


def write(path: Path, cols: dict, n: int | None = None) -> int:
    tbl = pa.table(cols)
    pq.write_table(tbl, path, **PQ_OPTS)
    return tbl.num_rows


def f32(x):
    return np.asarray(x, dtype=np.float32)


def f64(x):
    return np.asarray(x, dtype=np.float64)


def first(lst, default=float("nan")):
    try:
        return lst[0]
    except Exception:
        return default


def process_segment(segment: str) -> dict:
    t0 = time.time()
    from openpilot.cereal import log  # noqa: delayed; capnp load per process
    seg_dir = DB / segment
    seg_dir.mkdir(parents=True, exist_ok=True)
    rlog = RAW / segment / "rlog.zst"
    data, truncated = read_rlog(rlog)
    route, _, seg_idx = segment.rpartition("--")

    # --- collectors
    rr_t, rr_a, rr_d = [], [], []                 # radar_raw
    cc_t, cc_a, cc_d = [], [], []                 # can_car
    rec = {k: [] for k in ("t", "t_first", "crc_ok", "live_count", "header", "tail", "n_occupied", "n_frames")}
    rec_lens = {}
    obj = {k: [] for k in ["t", "record_idx", "slot", "raw"] + OBJ_SCALARS + ["raw_weight148", "raw_weight152", "raw_weight156"] + [f"code_{n}" for n in CODE_NAMES]}
    car = {k: [] for k in ("t", "v_ego", "v_ego_raw", "a_ego", "yaw_rate", "steering_angle_deg", "steering_rate", "gas_pressed",
                           "brake_pressed", "left_blinker", "right_blinker", "standstill", "cruise_enabled", "cruise_speed", "gear")}
    car_all_t, car_all_v = [], []
    mdl = {k: [] for k in ["t", "frame_id", "frame_drop_perc", "lead0_x", "lead0_y", "lead0_v", "lead0_a", "lead0_prob", "lead1_x", "lead1_y", "lead1_v", "lead1_prob"]
           + [f"lane_prob_{i}" for i in range(4)] + [f"lane_y_{i}" for i in range(4)] + ["edge_y_0", "edge_y_1", "lane_change_state", "desired_curvature"]}
    rs = {k: [] for k in ["t"] + [f"{l}_{f}" for l in ("lead_one", "lead_two") for f in ("status", "radar", "d_rel", "y_rel", "v_rel", "v_lead_k", "a_lead_k", "radar_track_id", "model_prob")]}
    pose = {k: [] for k in ("t", "acc_x", "acc_y", "acc_z", "pitch", "roll", "yaw", "gyro_x", "gyro_y", "gyro_z", "lat", "lon", "gps_valid", "vel_ned")}
    plan = {k: [] for k in ("t", "a_target", "v_target", "should_stop", "source", "has_lead")}
    rt = {k: [] for k in ("t", "track_id", "d_rel", "y_rel", "v_rel", "yv_rel", "measured")}

    asm = T.Id80RecordAssembler()
    n_events = 0
    parse_error = None
    t_start = None
    t_end = None
    car_i = 0
    wall_off = None          # unix wall time - logMonoTime (s), from the first `clocks` event
    wall_off_init = None     # fallback: initData.wallTimeNanos
    try:
        for ev in log.Event.read_multiple_bytes(data, traversal_limit_in_words=2**61):
            n_events += 1
            t = ev.logMonoTime / 1e9
            w = ev.which()
            if w == "can":
                if t_start is None:
                    t_start = t
                t_end = t
                for m in ev.can:
                    src = m.src
                    if src == RADAR_BUS:
                        dat = bytes(m.dat)
                        addr = m.address
                        rr_t.append(t); rr_a.append(addr); rr_d.append(dat)
                        if addr == C.ID80_ADDR:
                            done = asm.push(t, dat)
                            if done is not None:
                                r = done.payload
                                rec_lens[len(r)] = rec_lens.get(len(r), 0) + 1
                                ridx = len(rec["t"])
                                occ = list(R.occupied_slots(r))
                                rec["t"].append(t); rec["t_first"].append(done.time_s)
                                rec["crc_ok"].append(R.id80_crc_ok(r)); rec["live_count"].append(R.live_object_count(r))
                                rec["header"].append(r[:C.ID80_OBJECT_START]); rec["tail"].append(r[C.ID80_CRC_FIELD[1]:])
                                rec["n_occupied"].append(len(occ)); rec["n_frames"].append(done.frame_count)
                                for slot, sb in occ:
                                    o = O.decode_native_slot(slot, sb)
                                    obj["t"].append(t); obj["record_idx"].append(ridx); obj["slot"].append(slot); obj["raw"].append(sb)
                                    for k in OBJ_SCALARS:
                                        obj[k].append(getattr(o, k))
                                    w148 = o.raw_weights148
                                    obj["raw_weight148"].append(w148[0]); obj["raw_weight152"].append(w148[1]); obj["raw_weight156"].append(w148[2])
                                    for f in O.NAMED_FIELDS:
                                        obj[f"code_{f.name}"].append(O.field_code(sb, f))
                    elif src == CAR_BUS and m.address in CAR_ADDRS:
                        cc_t.append(t); cc_a.append(m.address); cc_d.append(bytes(m.dat))
            elif w == "carState":
                cs = ev.carState
                car_all_t.append(t); car_all_v.append(cs.vEgo)
                if car_i % CAR_DECIMATE == 0:
                    car["t"].append(t); car["v_ego"].append(cs.vEgo); car["v_ego_raw"].append(cs.vEgoRaw); car["a_ego"].append(cs.aEgo)
                    car["yaw_rate"].append(cs.yawRate); car["steering_angle_deg"].append(cs.steeringAngleDeg); car["steering_rate"].append(cs.steeringRateDeg)
                    car["gas_pressed"].append(cs.gasPressed); car["brake_pressed"].append(cs.brakePressed)
                    car["left_blinker"].append(cs.leftBlinker); car["right_blinker"].append(cs.rightBlinker); car["standstill"].append(cs.standstill)
                    car["cruise_enabled"].append(cs.cruiseState.enabled); car["cruise_speed"].append(cs.cruiseState.speed); car["gear"].append(str(cs.gearShifter))
                car_i += 1
            elif w == "clocks":
                if wall_off is None:
                    wall_off = ev.clocks.wallTimeNanos / 1e9 - t
            elif w == "initData":
                if wall_off_init is None and ev.initData.wallTimeNanos:
                    wall_off_init = ev.initData.wallTimeNanos / 1e9 - t
            elif w == "modelV2":
                mv = ev.modelV2
                leads = mv.leadsV3
                mdl["t"].append(t); mdl["frame_id"].append(mv.frameId); mdl["frame_drop_perc"].append(mv.frameDropPerc)
                if len(leads) > 0:
                    l0 = leads[0]
                    mdl["lead0_x"].append(first(l0.x)); mdl["lead0_y"].append(first(l0.y)); mdl["lead0_v"].append(first(l0.v)); mdl["lead0_a"].append(first(l0.a)); mdl["lead0_prob"].append(l0.prob)
                else:
                    for k in ("lead0_x", "lead0_y", "lead0_v", "lead0_a", "lead0_prob"):
                        mdl[k].append(float("nan"))
                if len(leads) > 1:
                    l1 = leads[1]
                    mdl["lead1_x"].append(first(l1.x)); mdl["lead1_y"].append(first(l1.y)); mdl["lead1_v"].append(first(l1.v)); mdl["lead1_prob"].append(l1.prob)
                else:
                    for k in ("lead1_x", "lead1_y", "lead1_v", "lead1_prob"):
                        mdl[k].append(float("nan"))
                probs = mv.laneLineProbs; lls = mv.laneLines; res = mv.roadEdges
                for i in range(4):
                    mdl[f"lane_prob_{i}"].append(probs[i] if i < len(probs) else float("nan"))
                    mdl[f"lane_y_{i}"].append(first(lls[i].y) if i < len(lls) else float("nan"))
                for i in range(2):
                    mdl[f"edge_y_{i}"].append(first(res[i].y) if i < len(res) else float("nan"))
                mdl["lane_change_state"].append(str(mv.meta.laneChangeState))
                mdl["desired_curvature"].append(mv.action.desiredCurvature)
            elif w == "radarState":
                r_ = ev.radarState
                rs["t"].append(t)
                for name, lead in (("lead_one", r_.leadOne), ("lead_two", r_.leadTwo)):
                    rs[f"{name}_status"].append(lead.present); rs[f"{name}_radar"].append(lead.radar)
                    rs[f"{name}_d_rel"].append(lead.dRel); rs[f"{name}_y_rel"].append(lead.yRel); rs[f"{name}_v_rel"].append(lead.vRel)
                    rs[f"{name}_a_lead_k"].append(lead.aLeadK); rs[f"{name}_v_lead_k"].append(lead.vLeadK); rs[f"{name}_radar_track_id"].append(lead.radarTrackId); rs[f"{name}_model_prob"].append(lead.modelProb)
            elif w == "liveLocationKalman":
                lk = ev.liveLocationKalman
                acc = lk.accelerationCalibrated.value; ori = lk.orientationNED.value; gyr = lk.angularVelocityCalibrated.value
                geo = lk.positionGeodetic; vel = lk.velocityNED.value
                pose["t"].append(t)
                pose["acc_x"].append(first(acc)); pose["acc_y"].append(acc[1] if len(acc) > 1 else float("nan")); pose["acc_z"].append(acc[2] if len(acc) > 2 else float("nan"))
                pose["roll"].append(first(ori)); pose["pitch"].append(ori[1] if len(ori) > 1 else float("nan")); pose["yaw"].append(ori[2] if len(ori) > 2 else float("nan"))
                pose["gyro_x"].append(first(gyr)); pose["gyro_y"].append(gyr[1] if len(gyr) > 1 else float("nan")); pose["gyro_z"].append(gyr[2] if len(gyr) > 2 else float("nan"))
                pose["lat"].append(first(geo.value)); pose["lon"].append(geo.value[1] if len(geo.value) > 1 else float("nan")); pose["gps_valid"].append(geo.valid)
                pose["vel_ned"].append(math.sqrt(sum(v * v for v in vel)) if len(vel) else float("nan"))
            elif w == "longitudinalPlan":
                lp = ev.longitudinalPlan
                plan["t"].append(t); plan["a_target"].append(lp.aTarget); plan["v_target"].append(first(lp.speeds)); plan["should_stop"].append(lp.shouldStop)
                plan["source"].append(str(lp.longitudinalPlanSource)); plan["has_lead"].append(lp.hasLead)
            elif w == "radarTracks":
                for p in ev.radarTracks.points:
                    rt["t"].append(t); rt["track_id"].append(p.trackId); rt["d_rel"].append(p.dRel); rt["y_rel"].append(p.yRel); rt["v_rel"].append(p.vRel); rt["measured"].append(p.deprecated.measured); rt["yv_rel"].append(p.deprecated.yvRel)
    except Exception as e:  # keep what was read
        parse_error = f"{type(e).__name__}: {e} (event {n_events} {w if n_events else '?'})"
        if n_events < 100:
            raise

    # --- write tables
    counts = {}
    counts["radar_raw"] = write(seg_dir / "radar_raw.parquet", {"t": f64(rr_t), "address": pa.array(rr_a, pa.uint32()), "dat": pa.array(rr_d, pa.binary())})
    counts["records"] = write(seg_dir / "records.parquet", {
        "t": f64(rec["t"]), "t_first": f64(rec["t_first"]), "crc_ok": pa.array(rec["crc_ok"], pa.bool_()), "live_count": pa.array(rec["live_count"], pa.uint8()),
        "header": pa.array(rec["header"], pa.binary()), "tail": pa.array(rec["tail"], pa.binary()), "n_occupied": pa.array(rec["n_occupied"], pa.uint8()),
        "n_frames": pa.array(rec["n_frames"], pa.uint8())})
    ocols = {"t": f64(obj["t"]), "record_idx": pa.array(obj["record_idx"], pa.uint32()), "slot": pa.array(obj["slot"], pa.uint8()), "raw": pa.array(obj["raw"], pa.binary())}
    for k in OBJ_SCALARS:
        v = obj[k]
        ft = O.NativeObject.__dataclass_fields__[k].type
        if ft.startswith("bool"):
            ocols[k] = pa.array(v, pa.bool_())
        elif ft.startswith("float"):
            ocols[k] = f32(v)
        else:
            ocols[k] = pa.array(v, pa.int32())
    for k in ("raw_weight148", "raw_weight152", "raw_weight156"):
        ocols[k] = pa.array(obj[k], pa.uint8())
    for n in CODE_NAMES:
        ocols[f"code_{n}"] = pa.array(obj[f"code_{n}"], pa.uint16())
    counts["objects"] = write(seg_dir / "objects.parquet", ocols)
    counts["car"] = write(seg_dir / "car.parquet", {
        "t": f64(car["t"]), **{k: f32(car[k]) for k in ("v_ego", "v_ego_raw", "a_ego", "yaw_rate", "steering_angle_deg", "steering_rate", "cruise_speed")},
        **{k: pa.array(car[k], pa.bool_()) for k in ("gas_pressed", "brake_pressed", "left_blinker", "right_blinker", "standstill", "cruise_enabled")},
        "gear": pa.array(car["gear"], pa.string())})
    counts["model"] = write(seg_dir / "model.parquet", {"t": f64(mdl["t"]), "frame_id": pa.array(mdl["frame_id"], pa.uint32()),
                                                       **{k: f32(mdl[k]) for k in mdl if k not in ("t", "frame_id", "lane_change_state")},
                                                       "lane_change_state": pa.array(mdl["lane_change_state"], pa.string())})
    rscols = {"t": f64(rs["t"])}
    for l in ("lead_one", "lead_two"):
        rscols[f"{l}_status"] = pa.array(rs[f"{l}_status"], pa.bool_()); rscols[f"{l}_radar"] = pa.array(rs[f"{l}_radar"], pa.bool_())
        for f in ("d_rel", "y_rel", "v_rel", "v_lead_k", "a_lead_k", "model_prob"):
            rscols[f"{l}_{f}"] = f32(rs[f"{l}_{f}"])
        rscols[f"{l}_radar_track_id"] = pa.array(rs[f"{l}_radar_track_id"], pa.int32())
    counts["radar_state"] = write(seg_dir / "radar_state.parquet", rscols)
    counts["pose"] = write(seg_dir / "pose.parquet", {"t": f64(pose["t"]), **{k: f64(pose[k]) for k in pose if k not in ("t", "gps_valid")}, "gps_valid": pa.array(pose["gps_valid"], pa.bool_())})
    counts["plan"] = write(seg_dir / "plan.parquet", {"t": f64(plan["t"]), "a_target": f32(plan["a_target"]), "v_target": f32(plan["v_target"]),
                                                     "should_stop": pa.array(plan["should_stop"], pa.bool_()), "source": pa.array(plan["source"], pa.string()),
                                                     "has_lead": pa.array(plan["has_lead"], pa.bool_())})
    counts["can_car"] = write(seg_dir / "can_car.parquet", {"t": f64(cc_t), "address": pa.array(cc_a, pa.uint32()), "dat": pa.array(cc_d, pa.binary())})
    counts["radar_tracks"] = write(seg_dir / "radar_tracks.parquet", {"t": f64(rt["t"]), "track_id": pa.array(rt["track_id"], pa.uint64()), "d_rel": f32(rt["d_rel"]),
                                                                     "y_rel": f32(rt["y_rel"]), "v_rel": f32(rt["v_rel"]), "yv_rel": f32(rt["yv_rel"]), "measured": pa.array(rt["measured"], pa.bool_())})

    # --- meta / index row
    vt = np.asarray(car_all_t, dtype=np.float64); vv = np.asarray(car_all_v, dtype=np.float64)
    distance = float(np.sum(0.5 * (vv[1:] + vv[:-1]) * np.diff(vt))) if len(vt) > 1 else 0.0
    n_rec = counts["records"]
    off = wall_off if wall_off is not None else wall_off_init
    wall_t_start = (t_start + off) if (t_start is not None and off is not None) else float("nan")
    meta = {
        "segment": segment, "route": route, "seg_idx": int(seg_idx) if seg_idx.isdigit() else -1,
        "t_start": t_start if t_start is not None else float("nan"), "t_end": t_end if t_end is not None else float("nan"),
        "wall_t_start": wall_t_start,
        "duration_s": (t_end - t_start) if t_start is not None else 0.0, "distance_m": distance,
        "n_records": n_rec, "n_crc_fail": int(n_rec - sum(rec["crc_ok"])), "n_objects": counts["objects"],
        "v_max": float(vv.max()) if len(vv) else 0.0, "has_pose": counts["pose"] > 0, "has_model": counts["model"] > 0,
        "n_events": n_events, "n_radar_frames": counts["radar_raw"], "zstd_truncated": truncated, "parse_error": parse_error,
        "record_lens": {str(k): v for k, v in rec_lens.items()}, "build_s": round(time.time() - t0, 1), "counts": counts,
    }
    (seg_dir / "_meta.json").write_text(json.dumps(meta))
    (seg_dir / "_DONE").write_text(time.strftime("%Y-%m-%dT%H:%M:%S"))
    return meta


def build_index() -> int:
    rows = []
    for d in sorted(DB.iterdir()):
        if (d / "_DONE").exists() and (d / "_meta.json").exists():
            m = json.loads((d / "_meta.json").read_text())
            rows.append({k: m[k] for k in ("segment", "route", "seg_idx", "t_start", "t_end", "duration_s", "distance_m", "n_records", "n_crc_fail",
                                           "n_objects", "v_max", "has_pose", "has_model", "n_events", "zstd_truncated")}
                        | {"parse_error": m.get("parse_error") or "", "wall_t_start": m.get("wall_t_start", float("nan"))})
    rows.sort(key=lambda r: (r["route"], r["seg_idx"]))
    cols = {k: [r[k] for r in rows] for k in rows[0]} if rows else {}
    if rows:
        tbl = pa.table({
            "segment": pa.array(cols["segment"], pa.string()), "route": pa.array(cols["route"], pa.string()), "seg_idx": pa.array(cols["seg_idx"], pa.int32()),
            "t_start": f64(cols["t_start"]), "t_end": f64(cols["t_end"]), "duration_s": f64(cols["duration_s"]), "distance_m": f64(cols["distance_m"]),
            "n_records": pa.array(cols["n_records"], pa.int32()), "n_crc_fail": pa.array(cols["n_crc_fail"], pa.int32()), "n_objects": pa.array(cols["n_objects"], pa.int32()),
            "v_max": f32(cols["v_max"]), "has_pose": pa.array(cols["has_pose"], pa.bool_()), "has_model": pa.array(cols["has_model"], pa.bool_()),
            "n_events": pa.array(cols["n_events"], pa.int32()), "zstd_truncated": pa.array(cols["zstd_truncated"], pa.bool_()), "parse_error": pa.array(cols["parse_error"], pa.string()),
            "wall_t_start": f64(cols["wall_t_start"])})
        tmp = DB / "index.parquet.tmp"
        pq.write_table(tbl, tmp, **PQ_OPTS)
        os.replace(tmp, DB / "index.parquet")
    return len(rows)


def _worker(segment: str) -> tuple[str, dict | None, str | None]:
    try:
        return segment, process_segment(segment), None
    except Exception:
        return segment, None, traceback.format_exc()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    DB.mkdir(parents=True, exist_ok=True)
    lock = open(DB / ".build.lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("another build is running; exit", flush=True)
        return 0
    segs = sorted(p.name for p in RAW.iterdir() if (p / "rlog.zst").exists())
    if args.only:
        segs = [s for s in segs if s in set(args.only)]
    todo = [s for s in segs if args.force or not (DB / s / "_DONE").exists()]
    if args.limit:
        todo = todo[:args.limit]
    print(f"{time.strftime('%F %T')} segments={len(segs)} todo={len(todo)} workers={args.workers} host={os.uname().nodename} "
          f"py={sys.version.split()[0]} op={OP}", flush=True)
    failed = []
    t0 = time.time()
    if todo:
        import multiprocessing as mp
        ctx = mp.get_context(os.environ.get("ARS510_MP_START", "fork"))
        with ctx.Pool(min(args.workers, len(todo)), maxtasksperchild=1) as pool:
            for i, (seg, meta, err) in enumerate(pool.imap_unordered(_worker, todo), 1):
                if err:
                    failed.append(seg)
                    print(f"[{i}/{len(todo)}] FAIL {seg}\n{err}", flush=True)
                else:
                    print(f"[{i}/{len(todo)}] ok {seg} rec={meta['n_records']} crc_fail={meta['n_crc_fail']} obj={meta['n_objects']} "
                          f"frames={meta['n_radar_frames']} dur={meta['duration_s']:.1f}s trunc={meta['zstd_truncated']} err={meta['parse_error']} {meta['build_s']}s", flush=True)
    n = build_index()
    print(f"{time.strftime('%F %T')} done: indexed={n} failed={len(failed)} {failed} elapsed={time.time() - t0:.0f}s", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
