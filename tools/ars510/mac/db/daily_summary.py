#!/usr/bin/env python3
"""Daily driving / radar summary from the ARS510 parquet database -> ~/ars510_rlogs/summary/YYYY-MM-DD.md (+ latest.md).

Usage: daily_summary.py [--date YYYY-MM-DD ...] [--all] [--force]
Default: (re)write every day whose segments were built after its summary (first run: every day); latest.md is a copy
of the most recent day with data. Runs after build_db.py from post_pull.sh (Mac venv); also fine in the Linux VM.
Days are the local time zone of the machine running this (the Mac: Asia/Taipei). The segment's wall-clock start comes
from index.wall_t_start (rlog `clocks` messages); for segments built before that column existed it falls back to the
rlog.zst mtime minus the segment duration (marked in the summary).
Env: RLOG_DB (default <dir of this script>), RLOG_RAW (../raw), ARS510_SUMMARY (../summary), ARS510_PULL_LOG (../ts_pull.log).
"""
from __future__ import annotations

import argparse, fcntl, os, re, shutil, sys, time
from datetime import date, datetime
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow.parquet as pq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DB = Path(os.environ.get("RLOG_DB", HERE))
RAW = Path(os.environ.get("RLOG_RAW", ROOT / "raw"))
OUT = Path(os.environ.get("ARS510_SUMMARY", ROOT / "summary"))
PULL_LOG = Path(os.environ.get("ARS510_PULL_LOG", ROOT / "ts_pull.log"))

# thresholds (mirroring opendbc/car/toyota/ars510/interface.py where one exists there)
STOP_HARD_DECEL = 0.35      # pose acc_x still below -0.35 m/s^2 when the wheels stop (locationd zeroes acc_x at standstill,
STOP_PRE_S = 0.25           #   so the minimum over the last 0.25 s before the standstill edge is the decel at the instant of stopping)
STOP_REBOUND = 0.20         # pose acc_x > +0.2 m/s^2 within STOP_POST_S after the stop (suspension rebound)
STOP_POST_S = 0.75
ROLL_V = 0.1                # wheel speed 0 -> > ROLL_V m/s after a stop
ROLL_RESTOP_S = 1.5         # ... and longitudinalPlan.shouldStop within this many seconds
GHOST_CLASSES = (1, 4)      # unclassified, pedestrian
GHOST_MIN_D = 25.0
MM_S, MM_N, MM_MPS, MM_MAX_RANGE_UNC, MM_MAX_Y, MM_MIN_AGE, MM_MIN_SAMPLES = 1.5, 10, 5.0, 12, 2.5, 8, 15


def read(seg: str, name: str, cols: list[str] | None = None) -> pd.DataFrame:
    p = DB / seg / f"{name}.parquet"
    if not p.exists():
        return pd.DataFrame()
    try:
        have = pq.read_schema(p).names
        use = [c for c in cols if c in have] if cols else None
        return pq.read_table(p, columns=use).to_pandas()
    except Exception as e:  # a segment being rewritten by build_db
        print(f"warn: {p}: {e}", file=sys.stderr)
        return pd.DataFrame()


def concat(segs: list[str], name: str, cols: list[str] | None = None) -> pd.DataFrame:
    parts = []
    for s in segs:
        d = read(s, name, cols)
        if len(d):
            d["segment"] = s
            parts.append(d)
    if not parts:
        return pd.DataFrame()
    return pd.concat(parts, ignore_index=True).sort_values("t", kind="stable").reset_index(drop=True)


def load_index() -> pd.DataFrame:
    idx = pq.read_table(DB / "index.parquet").to_pandas()
    if "wall_t_start" not in idx.columns:
        idx["wall_t_start"] = np.nan
    # logMonoTime is one clock per boot (= per route): use the route's largest wall - mono offset for every segment, since the
    # device clock is only behind (not yet synced) in the first segment(s) of a drive
    off = idx.wall_t_start - idx.t_start
    off_route = off.groupby(idx.route).transform("max")
    idx["wall_t_start"] = idx.t_start + off_route
    wall = idx.wall_t_start.to_numpy(dtype=float)
    fb = ~np.isfinite(wall)
    for i in range(len(idx)):   # no clock at all (nan), or still unsynced (> 7 days before the file arrived): rlog mtime - duration
        p = RAW / idx.segment.iat[i] / "rlog.zst"
        if not p.exists():
            continue
        est = p.stat().st_mtime - float(idx.duration_s.iat[i])
        if fb[i] or wall[i] < est - 7 * 86400:
            fb[i] = True
            idx.loc[idx.index[i], "wall_t_start"] = est
    idx["wall_fallback"] = fb
    idx["wall_t_end"] = idx.wall_t_start + idx.duration_s
    idx["day"] = [date.fromtimestamp(x).isoformat() if np.isfinite(x) else None for x in idx.wall_t_start]
    return idx


def hms(ts: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(ts))


def fmt_dur(s: float) -> str:
    s = int(round(s))
    return f"{s // 3600}h{(s % 3600) // 60:02d}m" if s >= 3600 else f"{s // 60}m{s % 60:02d}s"


# ---------------------------------------------------------------- stops
def stop_metrics(car: pd.DataFrame, plan: pd.DataFrame, pose: pd.DataFrame) -> dict:
    r = dict(stops=0, stops_engaged=0, hard=0, rebound=0, pose_na=0, releases=0, roll_restop=0, roll_under_stop=0, gas_releases=0)
    if not len(car):
        return r
    t = car.t.to_numpy(); v = car.v_ego.to_numpy(); vraw = car.v_ego_raw.to_numpy()
    ss = car.standstill.to_numpy(bool); eng = car.cruise_enabled.to_numpy(bool)
    gas = car.gas_pressed.to_numpy(bool) if "gas_pressed" in car.columns else np.zeros(len(car), bool)
    pt = pose.t.to_numpy() if len(pose) else np.empty(0); pax = pose.acc_x.to_numpy() if len(pose) else np.empty(0)
    plt_ = plan.t.to_numpy() if len(plan) else np.empty(0); pss = plan.should_stop.to_numpy(bool) if len(plan) else np.empty(0, bool)
    # standstill rising edges after real motion (> 1 m/s in the previous 5 s)
    for i in np.flatnonzero(ss[1:] & ~ss[:-1]) + 1:
        j0 = np.searchsorted(t, t[i] - 5.0)
        if j0 >= i or v[j0:i].max() < 1.0:
            continue
        r["stops"] += 1
        if not eng[i]:
            continue
        r["stops_engaged"] += 1
        a, m_, b = np.searchsorted(pt, t[i] - STOP_PRE_S), np.searchsorted(pt, t[i] + 0.05), np.searchsorted(pt, t[i] + STOP_POST_S)
        pre, post = pax[a:m_], pax[m_:b]
        if len(pre) and np.any(pre != 0):
            if pre.min() < -STOP_HARD_DECEL:
                r["hard"] += 1
            if len(post) and post.max() > STOP_REBOUND:
                r["rebound"] += 1
        else:
            r["pose_na"] += 1
    # releases: wheel speed 0 -> > ROLL_V after >= 0.5 s standstill, openpilot engaged
    for i in np.flatnonzero((vraw[1:] > ROLL_V) & (vraw[:-1] <= 0.02) & ss[:-1]) + 1:
        j0 = np.searchsorted(t, t[i] - 0.5)
        if j0 >= i or not ss[j0:i].all() or not eng[i]:
            continue
        if gas[max(0, i - 10):i + 1].any():       # driver gas override: not a creep
            r["gas_releases"] += 1
            continue
        r["releases"] += 1
        a, b = np.searchsorted(plt_, t[i] - 0.1), np.searchsorted(plt_, t[i] + ROLL_RESTOP_S)
        w = pss[a:b]
        if not len(w):
            continue
        if w[0]:
            r["roll_under_stop"] += 1          # rolled while the plan still said stop
        elif w.any():
            r["roll_restop"] += 1              # plan re-entered stopping within 1.5 s of the roll
    return r


# ---------------------------------------------------------------- model
def model_metrics(segs: list[str]) -> dict:
    r = dict(has_frame_id=False, n_msgs=0, gaps=0, dropped=0, drop_perc_max=0.0, per_seg=[])
    for s in segs:
        m = read(s, "model", ["t", "frame_id", "frame_drop_perc"])
        if not len(m):
            continue
        r["n_msgs"] += len(m)
        if "frame_id" not in m.columns:
            continue
        r["has_frame_id"] = True
        d = np.diff(m.frame_id.to_numpy(dtype=np.int64))
        g = d[d > 1]
        r["gaps"] += int(len(g)); r["dropped"] += int((g - 1).sum())
        if "frame_drop_perc" in m.columns and len(m):
            r["drop_perc_max"] = max(r["drop_perc_max"], float(np.nanmax(m.frame_drop_perc.to_numpy())))
        if len(g):
            r["per_seg"].append((s, int(len(g)), int((g - 1).sum())))
    return r


# ---------------------------------------------------------------- radar
def lead_switches(rs: pd.DataFrame) -> dict:
    r = dict(switches=0, acquired=0, radar_frac=float("nan"))
    if not len(rs):
        return r
    ids = rs.lead_one_radar_track_id.to_numpy(dtype=np.int64)
    valid = ids >= 0
    r["switches"] = int(np.sum(valid[1:] & valid[:-1] & (ids[1:] != ids[:-1])))
    r["acquired"] = int(np.sum(valid[1:] & ~valid[:-1]))
    r["radar_frac"] = float(rs.lead_one_radar.mean())
    return r


def ghost_metrics(obj: pd.DataFrame) -> dict:
    n = len(obj)
    if not n:
        return dict(n=0, far=0, gated=0)
    far = obj.d_rel.to_numpy() > GHOST_MIN_D
    gated = far & np.isin(obj.object_class.to_numpy(), GHOST_CLASSES) & (obj.camera_assoc.to_numpy() == 0)
    return dict(n=n, far=int(far.sum()), gated=int(gated.sum()))


def track_ids(obj: pd.DataFrame) -> np.ndarray:
    """Per-slot tracks: a new track when the age counter does not advance or the slot is silent > 0.5 s."""
    tid = np.zeros(len(obj), dtype=np.int64)
    nxt = 0
    for slot, g in obj.groupby("slot", sort=False):
        i = g.index.to_numpy(); t = g.t.to_numpy(); age = g.age.to_numpy(dtype=np.int64)
        new = np.ones(len(i), bool)
        if len(i) > 1:   # age counts up by one per record and saturates at 126, so only a decrease starts a new track
            new[1:] = (age[1:] < age[:-1]) | (np.diff(t) > 0.5)
        ids = np.cumsum(new) - 1 + nxt
        nxt = ids[-1] + 1
        tid[i] = ids
    return tid


def mismatch_metrics(obj: pd.DataFrame, car: pd.DataFrame) -> dict:
    r = dict(tracks=0, hits=0, candidates=0)
    if not len(obj) or not len(car):
        return r
    obj = obj.copy()
    obj["tid"] = track_ids(obj)
    v_ego = np.interp(obj.t.to_numpy(), car.t.to_numpy(), car.v_ego.to_numpy())
    obj["vrel"] = obj.v_long_ground.to_numpy() - v_ego
    r["tracks"] = int(obj.tid.nunique())
    for _, g in obj.groupby("tid", sort=False):
        if len(g) < MM_MIN_SAMPLES:
            continue
        t = g.t.to_numpy(); d = g.d_rel.to_numpy(dtype=float); n = len(t)
        j = np.searchsorted(t, t - MM_S)                       # window start per sample
        cnt = np.arange(n) - j + 1
        ok = (cnt >= MM_MIN_SAMPLES) & (t - t[j] >= MM_S - 0.2)
        if not ok.any():
            continue
        c1 = np.concatenate([[0.0], np.cumsum(t)]); c2 = np.concatenate([[0.0], np.cumsum(t * t)])
        c3 = np.concatenate([[0.0], np.cumsum(d)]); c4 = np.concatenate([[0.0], np.cumsum(t * d)])
        i1 = np.arange(n) + 1
        st = c1[i1] - c1[j]; st2 = c2[i1] - c2[j]; sd = c3[i1] - c3[j]; std = c4[i1] - c4[j]
        with np.errstate(invalid="ignore", divide="ignore"):
            den = cnt * st2 - st * st
            rate = np.where(den > 0, (cnt * std - st * sd) / den, 0.0)
        vrel = g.vrel.to_numpy(); ru = g.range_unc_code.to_numpy(dtype=float); y = g.y_rel.to_numpy(dtype=float); age = g.age.to_numpy()
        hit = ok & (age >= MM_MIN_AGE) & (ru < MM_MAX_RANGE_UNC) & (np.abs(y) <= MM_MAX_Y) & (np.abs(rate - vrel) > MM_MPS) & (np.abs(vrel) > 3.0)
        if not hit.any():
            continue
        r["hits"] += int(hit.sum())
        # longest run of consecutive hits (the interface logs a track after MM_N consecutive hits)
        run = best = 0
        for h in hit:
            run = run + 1 if h else 0
            best = max(best, run)
        if best >= MM_N:
            r["candidates"] += 1
    return r


# ---------------------------------------------------------------- ts_pull.log
def pull_log_for_day(day: str) -> dict:
    r = dict(runs=0, onroad=[], offroad_segments=0, offline=0, errors=0, mode_known=False, no_mode=0)
    if not PULL_LOG.exists():
        return r
    cur = None
    for line in PULL_LOG.read_text(errors="replace").splitlines():
        m = re.match(r"^== (\d{4}-\d{2}-\d{2}) (\d{2}:\d{2}:\d{2})", line)
        if m:
            cur = dict(day=m.group(1), hm=m.group(2)[:5], mode=None, n=None)
            if cur["day"] == day:
                r["runs"] += 1
            continue
        if cur is None or cur["day"] != day:
            continue
        if line.startswith("mode "):
            cur["mode"] = line.split()[1]; r["mode_known"] = True
        elif line.startswith("comma offline"):
            r["offline"] += 1
        elif line.startswith(("rsync failed", "partial", "size mismatch")) or "error" in line.lower():
            r["errors"] += 1
        else:
            m = re.match(r"^done(?: \((\w+)\))?: (\d+) segments pulled", line)
            if m:
                mode = cur["mode"] or m.group(1); n = int(m.group(2))
                if mode is None:
                    r["no_mode"] += 1
                if mode == "onroad":
                    r["onroad"].append((cur["hm"], n))
                else:
                    r["offroad_segments"] += n
    return r


# ---------------------------------------------------------------- report
def summarize_day(idx: pd.DataFrame, day: str) -> str:
    d = idx[idx.day == day].sort_values(["route", "seg_idx"])
    routes = []
    tot = dict(stops=0, stops_engaged=0, hard=0, rebound=0, pose_na=0, releases=0, roll_restop=0, roll_under_stop=0, gas_releases=0,
               switches=0, acquired=0, n=0, far=0, gated=0, tracks=0, hits=0, candidates=0, gaps=0, dropped=0, n_msgs=0)
    has_fid = False; drop_segs = []; radar_fracs = []; drop_perc_max = 0.0
    for route, g in d.groupby("route", sort=True):
        segs = g.segment.tolist()
        car = concat(segs, "car", ["t", "v_ego", "v_ego_raw", "standstill", "cruise_enabled", "gas_pressed"])
        plan = concat(segs, "plan", ["t", "should_stop"])
        pose = concat(segs, "pose", ["t", "acc_x"])
        rs = concat(segs, "radar_state", ["t", "lead_one_radar", "lead_one_radar_track_id"])
        obj = concat(segs, "objects", ["t", "slot", "age", "d_rel", "y_rel", "v_long_ground", "camera_assoc", "object_class", "range_unc_code"])
        sm = stop_metrics(car, plan, pose); ls = lead_switches(rs); gm = ghost_metrics(obj); mm = mismatch_metrics(obj, car); md = model_metrics(segs)
        has_fid |= md["has_frame_id"]; drop_segs += md["per_seg"]; drop_perc_max = max(drop_perc_max, md["drop_perc_max"])
        if np.isfinite(ls["radar_frac"]):
            radar_fracs.append((ls["radar_frac"], len(rs)))
        for k in tot:
            for src in (sm, ls, gm, mm, md):
                if k in src and isinstance(src[k], (int, np.integer)):
                    tot[k] += int(src[k])
        eng = float(car.cruise_enabled.mean()) if len(car) else float("nan")
        routes.append(dict(route=route, n=len(g), t0=g.wall_t_start.min(), t1=g.wall_t_end.max(), km=g.distance_m.sum() / 1000,
                           dur=g.duration_s.sum(), eng=eng, vmax=g.v_max.max() * 3.6, rec=int(g.n_records.sum()), crc=int(g.n_crc_fail.sum()),
                           fb=bool(g.wall_fallback.any()), sm=sm, ls=ls, gm=gm, mm=mm, md=md, trunc=int(g.zstd_truncated.sum()),
                           perr=int((g.parse_error.astype(str) != "").sum())))
    km = sum(r["km"] for r in routes); dur = sum(r["dur"] for r in routes)
    per100 = (lambda x: f"{x / km * 100:.1f} / 100 km" if km > 0.5 else "–")
    pl = pull_log_for_day(day)
    L = []
    L.append(f"# ARS510 每日摘要 {day}")
    L.append(f"產生：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}（{os.uname().nodename}，資料庫 {DB}）  ")
    L.append(f"當日 **{len(routes)} 趟**、**{km:.1f} km**、**{fmt_dur(dur)}**、{int(d.segment.count())} 段 "
             f"（openpilot 接管比例依時間 {np.nansum([r['eng'] * r['dur'] for r in routes]) / dur * 100 if dur else 0:.0f} %）")
    if any(r["fb"] for r in routes):
        L.append("> 註：標 * 的 route 為舊建庫資料，日期以 rlog.zst 檔案時間推估。")
    L.append("")
    L.append("## 路程")
    L.append("| route | 起 | 訖 | 段 | km | 時間 | 接管% | vmax km/h | records | CRC失敗 |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    for r in routes:
        L.append(f"| {r['route']}{'*' if r['fb'] else ''} | {hms(r['t0'])} | {hms(r['t1'])} | {r['n']} | {r['km']:.1f} | {fmt_dur(r['dur'])} | "
                 f"{r['eng'] * 100:.0f} | {r['vmax']:.0f} | {r['rec']} | {r['crc']} |")
    L.append("")
    L.append("## 煞停（openpilot 接管中）")
    L.append(f"- 停車次數：{tot['stops']}（其中接管中 **{tot['stops_engaged']}**）")
    L.append(f"- 停住瞬間 pose 減速 < −{STOP_HARD_DECEL} m/s²（輪停前 {STOP_PRE_S} s 內最小值）：**{tot['hard']}**；"
             f"停後 {STOP_POST_S} s 內回彈 > +{STOP_REBOUND} m/s²：**{tot['rebound']}**"
             + (f"（{tot['pose_na']} 次無 pose 資料）" if tot["pose_na"] else ""))
    L.append(f"- 停住後前滾（接管中、無油門，輪速 0 → >{ROLL_V} m/s）：{tot['releases']} 次起步，其中 **{tot['roll_restop']}** 次在 {ROLL_RESTOP_S} s 內 plan 再進 stopping、"
             f"**{tot['roll_under_stop']}** 次 plan 仍為 stopping 時就滾動；另 {tot['gas_releases']} 次為駕駛油門起步（不計）")
    if routes and any(r["sm"]["stops_engaged"] for r in routes):
        L.append("")
        L.append("| route | 接管停車 | 急停 | 回彈 | 起步 | 再進stopping | 停車中滾動 |")
        L.append("|---|---|---|---|---|---|---|")
        for r in routes:
            s = r["sm"]
            if s["stops_engaged"] or s["releases"]:
                L.append(f"| {r['route']} | {s['stops_engaged']} | {s['hard']} | {s['rebound']} | {s['releases']} | {s['roll_restop']} | {s['roll_under_stop']} |")
    L.append("")
    L.append("## 模型／定位")
    if has_fid:
        L.append(f"- modelV2 {tot['n_msgs']} 則；frameId 間隔 > 1：**{tot['gaps']}** 處，共掉 **{tot['dropped']}** 幀；frameDropPerc 最大 {drop_perc_max * 100:.1f} %")
        if drop_segs:
            drop_segs.sort(key=lambda x: -x[2])
            L.append("  - 掉幀最多的段：" + "、".join(f"{s}（{g} 處/{n} 幀）" for s, g, n in drop_segs[:8]))
    else:
        L.append("- 模型掉幀：model.parquet 無 frame_id 欄位（舊建庫），略過")
    L.append("- locationd 警報：建庫未存 selfdrive 警報表，略過")
    L.append("")
    L.append("## 雷達")
    L.append(f"- records：{sum(r['rec'] for r in routes)}，CRC 失敗：**{sum(r['crc'] for r in routes)}**"
             + (f"；zstd 截斷段 {sum(r['trunc'] for r in routes)}、rlog 中斷段 {sum(r['perr'] for r in routes)}（多為路程末段）" if any(r["trunc"] or r["perr"] for r in routes) else ""))
    rf = (sum(f * n for f, n in radar_fracs) / sum(n for _, n in radar_fracs) * 100) if radar_fracs else float("nan")
    L.append(f"- radard 前車切換（lead_one radar_track_id 變換）：**{tot['switches']}** 次 = {per100(tot['switches'])}；取得雷達前車 {tot['acquired']} 次；"
             f"leadOne 來自雷達的時間比例 {rf:.0f} %")
    gfrac = tot["gated"] / tot["n"] * 100 if tot["n"] else 0.0
    L.append(f"- 鬼影門（class 1/4、無相機關聯、d > {GHOST_MIN_D:.0f} m）：物件 {tot['n']}（d > 25 m：{tot['far']}）→ 門後 {tot['n'] - tot['gated']}，"
             f"擋下 **{tot['gated']}**（{gfrac:.1f} %）")
    L.append(f"- 速度／距離不一致（{MM_S} s 視窗 |d(d)/dt − v_rel| > {MM_MPS:.0f} m/s、range_unc < {MM_MAX_RANGE_UNC}、|y| ≤ {MM_MAX_Y}）："
             f"命中 {tot['hits']} 筆，候選軌跡（連續 ≥ {MM_N} 筆）**{tot['candidates']}** = {per100(tot['candidates'])}，軌跡總數 {tot['tracks']}")
    L.append("")
    L.append("| route | 切換 | 取得 | 物件 | 擋下 | 不一致候選/命中 |")
    L.append("|---|---|---|---|---|---|")
    for r in routes:
        L.append(f"| {r['route']} | {r['ls']['switches']} | {r['ls']['acquired']} | {r['gm']['n']} | {r['gm']['gated']} | {r['mm']['candidates']}/{r['mm']['hits']} |")
    L.append("")
    L.append("## 同步健康（ts_pull.log）")
    if pl["runs"]:
        on = sum(n for _, n in pl["onroad"])
        L.append(f"- 當日 {pl['runs']} 次執行；行車中（mode onroad）{len(pl['onroad'])} 次、處理 {on} 段"
                 + ("：" + "、".join(f"{hm} {n} 段" for hm, n in pl["onroad"]) if pl["onroad"] else "")
                 + f"；停車時 rsync 處理 {pl['offroad_segments']} 段次（含每輪重檢的未釋放段）；comma 離線 {pl['offline']} 次；錯誤／部分傳輸 {pl['errors']} 行")
        if pl["no_mode"]:
            L.append(f"- （{pl['no_mode']} 次執行無 mode 標記＝11/02 前的舊版 ts_pull.sh，無法區分行車中，計入停車時）")
    else:
        L.append("- 當日 ts_pull.log 無紀錄")
    L.append("- kswapd／CPU 統計：需 procLog，建庫未存，略過")
    L.append("")
    return "\n".join(L) + "\n"


def done_mtime(idx: pd.DataFrame, day: str) -> float:
    m = 0.0
    for s in idx.segment[idx.day == day]:
        p = DB / s / "_DONE"
        if p.exists():
            m = max(m, p.stat().st_mtime)
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", nargs="*", default=None)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    if not (DB / "index.parquet").exists():
        print("no index.parquet"); return 0
    OUT.mkdir(parents=True, exist_ok=True)
    lock = open(OUT / ".summary.lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("another summary is running; exit"); return 0
    idx = load_index()
    days = sorted(set(x for x in idx.day if x))
    if not days:
        print("no dated segments"); return 0
    OUT.mkdir(parents=True, exist_ok=True)
    if args.date:
        todo = [x for x in args.date]
    elif args.all:
        todo = days
    else:
        todo = []
        for day in days:
            out = OUT / f"{day}.md"
            if args.force or not out.exists() or done_mtime(idx, day) > out.stat().st_mtime:
                todo.append(day)
    t0 = time.time()
    for day in todo:
        if day not in days:
            print(f"{day}: no data"); continue
        out = OUT / f"{day}.md"
        tmp = out.with_suffix(".md.tmp")
        tmp.write_text(summarize_day(idx, day))
        os.replace(tmp, out)
        print(f"wrote {out}")
    latest = OUT / f"{days[-1]}.md"
    if latest.exists():
        shutil.copyfile(latest, OUT / "latest.md")
    print(f"{time.strftime('%F %T')} summary: days={len(days)} written={len(todo)} latest={days[-1]} {time.time() - t0:.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
