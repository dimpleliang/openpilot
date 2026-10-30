#!/usr/bin/env python3
"""ars510 drive report: one pass over a route's rlogs (qlog fallback) -> summary.txt, events.txt, signals.csv (5 Hz),
clips/*.csv (20 Hz, +-15 s around braking and ARS510 events). Written for the device; also runs on a PC with the route
directory given. Usage: report.py ROUTE [--realdata DIR] [--out DIR]"""
import bisect
import csv
import datetime
import glob
import json
import os
import sys
import time

import numpy as np
import zstandard

from openpilot.cereal import log

REALDATA = "/data/media/0/realdata"
OUT_ROOT = "/data/ars510_reports"
KEEP_ROUTES = 30
TZ = datetime.timezone(datetime.timedelta(hours=8))
CSV_EVERY = 4            # longitudinalPlan is 20 Hz -> signals.csv at 5 Hz
BRAKE_EVENT_A = -1.0     # m/s^2
CLIP_S = 15.0
EVENT_GAP_S = 8.0

# early: 0 off, 1 lead (camera/radar lead), 2 cut-in candidate, 3 radar-only far object / ACC target
SIG_FIELDS = ["t", "clock", "v_kmh", "a_ego", "gas", "brake", "enabled", "v_cruise_kmh", "a_target", "source", "should_stop",
              "lead", "lead_radar", "lead_d", "lead_vrel", "lead_a", "lead_y", "cam_prob", "cam_x", "cam_vrel", "e2e_a",
              "pitch_deg", "steer_deg", "blinker", "early", "far_obj", "cutin", "alerts"]


def read_segment(path):
  with open(path, "rb") as f:
    data = zstandard.ZstdDecompressor().stream_reader(f).read()
  return log.Event.read_multiple_bytes(data, traversal_limit_in_words=2**61)


def route_segments(route, realdata=REALDATA):
  segs = glob.glob(os.path.join(realdata, f"{route}--*"))
  return sorted(segs, key=lambda p: int(p.rsplit("--", 1)[1]))


class RouteScan:
  def __init__(self):
    self.commit = None
    self.rows = []          # 20 Hz: dict per longitudinalPlan
    self.events = []        # (t, msg) ARS510 cloudlog
    self.segstats = []      # per segment
    self.clock = None       # (wall_s, mono_s)
    self.cur = dict(v=0., a=0., gas=0, brake=0, enabled=0, vc=0., lp=0, rad=0, d=0., vr=0., al=0., y=0., mp=0., mx=0., mv=0.,
                    e2e=0., pitch=float("nan"), sa=0., bl=0, alerts="")
    self.flags = dict(early=0, far_obj=0, cutin=0)
    self.alert_until = 0.

  def wall(self, t):
    if self.clock is None:
      return ""
    return datetime.datetime.fromtimestamp(self.clock[0] + t - self.clock[1], TZ).strftime("%H:%M:%S.%f")[:-5]

  def scan_segment(self, segdir, seg_idx):
    f = os.path.join(segdir, "rlog.zst")
    kind = "rlog"
    if not os.path.exists(f):
      f = os.path.join(segdir, "qlog.zst"); kind = "qlog"
    if not os.path.exists(f):
      return
    c = self.cur; n_plan = 0; amin = 0.; vmax = 0.; brk = 0; eng = 0; ncs = 0; nl = 0; nr = 0; nrs = 0; early = 0; t_first = None; t_last = None
    for m in read_segment(f):
      w = m.which(); t = m.logMonoTime / 1e9
      if w == "initData":
        if self.commit is None:
          self.commit = m.initData.gitCommit[:7]
        continue
      if t_first is None:
        t_first = t
      t_last = t
      if w == "clocks":
        self.clock = (m.clocks.wallTimeNanos / 1e9, t)
      elif w == "carState":
        s = m.carState; ncs += 1
        c.update(v=s.vEgo, a=s.aEgo, gas=int(s.gasPressed), brake=int(s.brakePressed), enabled=int(s.cruiseState.enabled), vc=s.vCruise,
                 sa=s.steeringAngleDeg, bl=int(s.leftBlinker) - int(s.rightBlinker))
        vmax = max(vmax, s.vEgo); brk += s.brakePressed; eng += s.cruiseState.enabled
      elif w == "radarState":
        l = m.radarState.leadOne; nrs += 1; nl += l.present; nr += (l.present and l.radar)
        c.update(lp=int(l.present), rad=int(l.radar), d=l.dRel, vr=l.vRel, al=l.aLeadK, y=l.yRel)
      elif w == "modelV2":
        L = m.modelV2.leadsV3
        c.update(mp=L[0].prob if len(L) else 0., mx=L[0].x[0] if len(L) else 0., mv=(L[0].v[0] - c["v"]) if len(L) else 0.,
                 e2e=m.modelV2.action.desiredAcceleration)
      elif w == "liveLocationKalman":
        o = m.liveLocationKalman.calibratedOrientationNED.value
        if len(o) == 3:
          c["pitch"] = float(np.degrees(o[1]))
      elif w == "onroadEvents":
        names = [str(e.name) for e in m.onroadEvents if str(e.name) not in ("pedalPressed", "gasPressedOverride", "steerOverride")]
        if names:
          c["alerts"] = ",".join(names)[:60]; self.alert_until = t + 0.5
        elif t > self.alert_until:
          c["alerts"] = ""
      elif w == "logMessage" and "ARS510" in m.logMessage:
        msg = m.logMessage
        try:
          msg = json.loads(msg).get("msg", msg)
        except Exception:
          pass
        self.events.append((t, msg))
        if "far lead early" in msg:
          on = " off" not in msg
          self.flags["early"] = (2 if "cut-in" in msg else (3 if (" obj" in msg or " acc" in msg) else 1)) if on else 0
          early += on
        elif "far object on" in msg: self.flags["far_obj"] = 1
        elif "far object off" in msg: self.flags["far_obj"] = 0
      elif w == "longitudinalPlan":
        p = m.longitudinalPlan; n_plan += 1; amin = min(amin, p.aTarget)
        self.rows.append(dict(t=t, seg=seg_idx, v=c["v"], a=c["a"], gas=c["gas"], brake=c["brake"], enabled=c["enabled"], vc=c["vc"],
                              aT=p.aTarget, src=str(p.longitudinalPlanSource)[:6], stop=int(p.shouldStop), lp=c["lp"], rad=c["rad"],
                              d=c["d"], vr=c["vr"], al=c["al"], y=c["y"], mp=c["mp"], mx=c["mx"], mv=c["mv"], e2e=c["e2e"],
                              pitch=c["pitch"], sa=c["sa"], bl=c["bl"], early=self.flags["early"], far_obj=self.flags["far_obj"],
                              alerts=c["alerts"]))
    self.segstats.append(dict(seg=seg_idx, kind=kind, t0=t_first, t1=t_last, vmax=vmax, amin=amin, brake_s=brk / 100., eng=eng / max(ncs, 1),
                              lead=nl / max(nrs, 1), radar=nr / max(nrs, 1), early=early, start=self.wall(t_first) if t_first else ""))


def classify_brake_events(rows, events):
  """Local minima of aTarget below BRAKE_EVENT_A, with context and an automatic class."""
  ts = [r["t"] for r in rows]
  def at(t):
    i = bisect.bisect_right(ts, t) - 1
    return rows[max(i, 0)]
  out = []; i = 0
  while i < len(rows):
    if rows[i]["aT"] <= BRAKE_EVENT_A:
      j = i
      while j < len(rows) and (rows[j]["aT"] <= BRAKE_EVENT_A or rows[j]["t"] - rows[i]["t"] < EVENT_GAP_S):
        j += 1
      k = min(range(i, j), key=lambda n: rows[n]["aT"]); out.append(k); i = j
    else:
      i += 1
  res = []
  for k in out:
    r0 = rows[k]; t0 = r0["t"]
    ctx = [at(t0 + d) for d in (-6, -4, -2, -1, 0)]
    brk = any(at(t0 + d)["brake"] for d in (-3, -2, -1, 0, 1)); gas = any(at(t0 + d)["gas"] for d in (-3, -2, -1, 0))
    al_min = min((x["al"] for x in ctx if x["lp"]), default=0.)
    ds = [x["d"] for x in ctx if x["lp"]]
    jump = any(ds[n] - ds[n + 1] > 10 for n in range(len(ds) - 1))
    new_lead = not ctx[0]["lp"] and ctx[-1]["lp"]
    d6 = ctx[0]["d"] if ctx[0]["lp"] else 0.
    if not r0["enabled"]: cls = "未啟用"
    elif brk: cls = "駕駛踩煞車"
    elif al_min < -1.0: cls = "前車減速"
    elif jump or new_lead: cls = "切入／新前車"
    elif d6 > 60: cls = "遠處慢車"
    elif r0["v"] < 8: cls = "停車"
    else: cls = "近距跟車"
    early = sum(1 for t, m in events if t0 - 12 <= t <= t0 and "far lead early" in m and " off" not in m)
    res.append(dict(t=t0, aT=r0["aT"], src=r0["src"], v6=ctx[0]["v"], v0=r0["v"], ctx=ctx, gas=gas, cls=cls, early=early))
  return res


def write_outputs(route, scan, out_dir):
  os.makedirs(os.path.join(out_dir, "clips"), exist_ok=True)
  rows = scan.rows; ev = scan.events
  W = scan.wall
  brakes = classify_brake_events(rows, ev)
  # summary
  with open(os.path.join(out_dir, "summary.txt"), "w") as f:
    v = [r["v"] for r in rows]; eng = [r["enabled"] for r in rows]
    f.write(f"路線 {route}  版本 {scan.commit}  {len(scan.segstats)} 段  {(rows[-1]['t'] - rows[0]['t']) / 60:.0f} 分鐘" if rows else f"路線 {route}: 無資料")
    if rows:
      f.write(f"  最高 {max(v) * 3.6:.0f} km/h  啟用 {100 * np.mean(eng):.0f}%  > 70 km/h {sum(x > 70 / 3.6 for x in v) / 20 / 60:.0f} 分\n\n")
      kinds = {}
      for t, m in ev:
        if " off" in m or "off:" in m: continue
        k = ("收油·切入" if "cut-in" in m else "收油·雷達單獨" if (" obj" in m or " acc" in m) else "收油·雷達前車" if "radar=True" in m else "收油·鏡頭前車") if "far lead early" in m \
            else "煞車上限" if "far lead soft" in m else "雷達遠距物件" if "far object" in m else "遠距停車" if "far stopped" in m else "e2e 放開" if "e2e free" in m else "其他"
        kinds[k] = kinds.get(k, 0) + 1
      f.write("ARS510 事件：" + "、".join(f"{k} {n}" for k, n in sorted(kinds.items())) + "\n\n")
      f.write("系統煞車 ≤ -1.0 m/s²（啟用中）：\n")
      cls_cnt = {}
      for b in brakes:
        cls_cnt[b["cls"]] = cls_cnt.get(b["cls"], 0) + 1
      f.write("  " + "、".join(f"{k} {n}" for k, n in cls_cnt.items()) + "\n\n")
      f.write(f"{'段':>3} {'時間':>8} {'最高':>4} {'aT最小':>6} {'踩煞車s':>6} {'啟用':>4} {'前車':>4} {'雷達':>4} {'收油':>3}\n")
      for s in scan.segstats:
        f.write(f"{s['seg']:>3} {s['start']:>8} {s['vmax'] * 3.6:>4.0f} {s['amin']:>6.2f} {s['brake_s']:>6.0f} {s['eng']:>4.2f} {s['lead']:>4.2f} {s['radar']:>4.2f} {s['early']:>3}\n")
  # events
  with open(os.path.join(out_dir, "events.txt"), "w") as f:
    f.write("== 系統煞車事件（aTarget ≤ -1.0）\n")
    for b in brakes:
      c = b["ctx"]
      def L(x):
        return f"{x['d']:.0f}m {x['vr']:+.1f} a{x['al']:.1f}{'R' if x['rad'] else 'C'}" if x["lp"] else "無"
      f.write(f"{W(b['t'])} aT={b['aT']:5.2f} {b['src']:6} v {b['v6'] * 3.6:3.0f}->{b['v0'] * 3.6:3.0f} | 前車 -6s:{L(c[0])} -2s:{L(c[2])} 0s:{L(c[4])} | 油門{int(b['gas'])} | {b['cls']} | 之前收油 {b['early']}\n")
    f.write("\n== ARS510 紀錄\n")
    for t, m in ev:
      f.write(f"{W(t)} {m[:160]}\n")
  # signals.csv 5 Hz
  with open(os.path.join(out_dir, "signals.csv"), "w", newline="") as f:
    wr = csv.writer(f); wr.writerow(SIG_FIELDS)
    for r in rows[::CSV_EVERY]:
      wr.writerow(sig_row(r, W))
  # clips
  ts = [r["t"] for r in rows]
  marks = [(b["t"], f"brake_{b['aT']:.1f}") for b in brakes] + [(t, "early") for t, m in ev if "far lead early" in m and " off" not in m and ("cut-in" in m or " obj" in m or " acc" in m)]
  last = -1e9
  for t0, tag in sorted(marks):
    if t0 - last < 5:
      continue
    last = t0
    i0 = bisect.bisect_left(ts, t0 - CLIP_S); i1 = bisect.bisect_left(ts, t0 + CLIP_S)
    with open(os.path.join(out_dir, "clips", f"{W(t0).replace(':', '')}_{tag}.csv"), "w", newline="") as f:
      wr = csv.writer(f); wr.writerow(SIG_FIELDS)
      for r in rows[i0:i1]:
        wr.writerow(sig_row(r, W))
  with open(os.path.join(out_dir, "status.json"), "w") as f:
    json.dump(dict(route=route, segments=len(scan.segstats), commit=scan.commit, generated=time.time(), brake_events=len(brakes)), f)


def sig_row(r, W):
  return [f"{r['t']:.2f}", W(r["t"]), f"{r['v'] * 3.6:.1f}", f"{r['a']:.2f}", r["gas"], r["brake"], r["enabled"], f"{r['vc']:.0f}", f"{r['aT']:.2f}", r["src"],
          r["stop"], r["lp"], r["rad"], f"{r['d']:.1f}", f"{r['vr']:.2f}", f"{r['al']:.2f}", f"{r['y']:.1f}", f"{r['mp']:.2f}", f"{r['mx']:.1f}", f"{r['mv']:.1f}",
          f"{r['e2e']:.2f}", f"{r['pitch']:.1f}" if r["pitch"] == r["pitch"] else "", f"{r['sa']:.0f}", r["bl"], r["early"], r["far_obj"], int(r["early"] == 2), r["alerts"]]


def generate(route, realdata=REALDATA, out_root=OUT_ROOT):
  segs = route_segments(route, realdata)
  if not segs:
    return None
  scan = RouteScan()
  for i, sd in enumerate(segs):
    try:
      scan.scan_segment(sd, int(sd.rsplit("--", 1)[1]))
    except Exception as e:
      scan.events.append((scan.rows[-1]["t"] if scan.rows else 0., f"report: segment {sd} failed: {e}"))
  out_dir = os.path.join(out_root, route); tmp = out_dir + ".tmp"
  if os.path.exists(tmp):
    import shutil; shutil.rmtree(tmp)
  os.makedirs(tmp, exist_ok=True)
  write_outputs(route, scan, tmp)
  if os.path.exists(out_dir):
    import shutil; shutil.rmtree(out_dir)
  os.rename(tmp, out_dir)
  prune(out_root)
  return out_dir


def prune(out_root=OUT_ROOT, keep=KEEP_ROUTES):
  dirs = sorted(d for d in glob.glob(os.path.join(out_root, "*")) if os.path.isdir(d) and not d.endswith(".tmp"))
  for d in dirs[:-keep]:
    import shutil; shutil.rmtree(d, ignore_errors=True)


if __name__ == "__main__":
  import argparse
  ap = argparse.ArgumentParser(); ap.add_argument("route"); ap.add_argument("--realdata", default=REALDATA); ap.add_argument("--out", default=OUT_ROOT)
  a = ap.parse_args()
  t0 = time.monotonic()
  d = generate(a.route, a.realdata, a.out)
  print(d, f"{time.monotonic() - t0:.0f} s")
