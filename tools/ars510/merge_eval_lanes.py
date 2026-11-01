#!/usr/bin/env python3
"""ars510: score cut-in / merge display rules on slim rlogs + model lane lines (slim_rlog.py, slim_lanes.py), from the raw 0x80 object list.

  python3 merge_eval_slim.py SLIM_DIR [--list]

Every object-list record is placed against the model path (offset = right-positive y - path y, as in ars510_tracks.py).
A track is (slot, birth). A REAL cut-in is a track that was >= 1.8 m from the path and then within 0.9 m of it (or the
radar lead), 15-110 m ahead, within 6 s, at > 20 km/h. Each rule marks a track when its condition holds for HOLD records;
an episode is TRUE when the track then reaches my lane (or becomes the lead) within 8 s. Results by ego-speed band.
"""
import glob
import os
import pickle
import sys
from collections import defaultdict

import numpy as np
import zstandard

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "opendbc_repo"))
import opendbc.car.toyota.ars510.interface as I  # noqa: E402

REC = []
_orig = I.decode_native_slot
def _hook(i, s):
  o = _orig(i, s); REC.append(o); return o
I.decode_native_slot = _hook

RATE_HZ = 16.7          # object-list records per second
BANDS = [(0, 60), (60, 90), (90, 200)]


def load(path):
  return pickle.loads(zstandard.ZstdDecompressor().decompress(open(path, "rb").read(), max_output_size=1 << 31))


LANE_DIR = os.environ.get("LANE_DIR", os.path.expanduser("~/mnt/ars510_rlogs/lanes"))
LANE_MIN_PROB = 0.5


def lanes_for(name):
  p = os.path.join(LANE_DIR, name + ".lanes.pkl.zst")
  return load(p) if os.path.exists(p) else None


def tracks_of(seg, lanes=None):
  """[(track key, [sample...])] with sample = (t, d, off, vr, vlat_toward, v_ego, steer, curv50, is_lead)."""
  ev = [(t, 0, x) for t, *x in seg["can"]] + [(t, 1, x) for t, *x in seg["car"]] + [(t, 2, x) for t, *x in seg["model"]] + \
       [(t, 3, x) for t, *x in seg["lead"]]
  if lanes is not None and lanes["x"] is not None:
    ev += [(t, 4, (p, yl, yrr)) for t, p, yl, yrr in lanes["lanes"]]
    lx = np.asarray(lanes["x"], dtype=float)
  ev.sort(key=lambda e: e[0])
  lane = None; lane_w = 3.5
  ri = I.Ars510NativeRadarInterface(I.NativeInterfaceConfig(radar_bus=1))
  v = 0.; steer = 0.; path = None; leads = []; life = {}; tr = defaultdict(list)
  for t, k, x in ev:
    if k == 4:
      p, yl, yrr = x
      lane = (np.asarray(p, dtype=float), np.asarray(yl, dtype=float), np.asarray(yrr, dtype=float))
      if p[1] >= LANE_MIN_PROB and p[2] >= LANE_MIN_PROB:
        w = float(np.interp(8., lx, lane[2]) - np.interp(8., lx, lane[1]))
        if 2.5 < w < 4.5:
          lane_w += 0.05 * (float(np.clip(w, 3.0, 4.0)) - lane_w)
      continue
    if k == 1:
      v, steer = x[0], x[1]
    elif k == 2:
      path = (np.array(x[0]), np.array(x[1]))
    elif k == 3:
      leads = [(l[2], -l[3]) for l in x[0] if l[0] and l[1]]  # (dRel, y right-positive)
    elif k == 0 and path is not None and len(path[0]):
      bus, addr, dat = x
      if addr not in (0x80, 0xB4, 0x24):
        continue
      REC.clear(); ri.update_frame(t, bus, addr, dat)
      MODE = os.environ.get("VLAT_MODE", "raw_median")
      def vlat_of(o):
        if MODE.startswith("yaw"):
          w = ri._fresh_yaw_rate(t)
          return o.v_lat_ground - (w or 0.) * o.d_rel
        return o.v_lat_ground
      vl = [vlat_of(o) for o in REC if o.geometry_valid and o.lateral_valid and 15 <= o.d_rel <= 120]
      bias = float(np.median(vl)) if len(vl) >= 3 else 0.   # left-positive, common to all objects (my yaw, road bend)
      for o in REC:
        prev = life.get(o.slot)
        if prev is None or o.age < prev[0]:
          life[o.slot] = [o.age, (o.slot, round(t, 2))]
        else:
          life[o.slot][0] = o.age
        if not (o.geometry_valid and o.lateral_valid) or o.age < 4 or not (5 < o.d_rel < 120):
          continue
        if (o.object_class or 0) not in (1, 2, 3, 4):   # vehicles / two-wheelers
          pass
        yr = -o.y_rel
        py = float(np.interp(o.d_rel, path[0], path[1]))
        off = yr - py
        vr = o.v_long_ground - v
        if o.v_long_ground < 2.0:   # oncoming / standing: never a cut-in
          continue
        vlat = -(vlat_of(o) - (bias if MODE.endswith("median") else 0.))  # right-positive
        toward = -vlat if off > 0 else vlat        # + = moving toward my path
        curv50 = abs(float(np.interp(50., path[0], path[1])))
        is_lead = any(abs(o.d_rel - ld) < 4 and abs(yr - ly) < 1.5 for ld, ly in leads)
        # lane-line intrusion (herizon1054 radard_ext): + = past the line on the object's side, inside my lane
        intr_line = None
        if lane is not None:
          side = 2 if off > 0 else 1
          if lane[0][side] >= LANE_MIN_PROB:
            ly = float(np.interp(o.d_rel, lx, lane[side]))
            intr_line = (ly - yr) if off > 0 else (yr - ly)
        intr_path = lane_w / 2. - abs(off)
        tr[life[o.slot][1]].append((t, o.d_rel, off, vr, toward, v, steer, curv50, is_lead, int(o.lane_crossing_code or 0),
                                    intr_line, intr_path))
  return tr


class Rule:
  def __init__(self, name, wide=False, gate=None, lat=False, curve=False, wide_hold=8, near=True, fast=False, vmax=None, fast_thr=0.5, fast_hold=3, fast_vr=1.5, close=False, close_thr=1.0, close_hold=3, close_d=40.):
    self.name, self.wide, self.gate, self.lat, self.curve, self.wide_hold, self.near = name, wide, gate, lat, curve, wide_hold, near
    self.fast, self.vmax, self.fast_thr, self.fast_hold, self.fast_vr = fast, vmax, fast_thr, fast_hold, fast_vr
    self.close, self.close_thr, self.close_hold, self.close_d = close, close_thr, close_hold, close_d

  def marks(self, s):
    """indices where the track is marked"""
    out = []; hold = 0; rel = 0; hist = []; fastn = 0; closen = 0
    for i, (t, d, off, vr, tow, v, steer, c50, lead, lc) in enumerate(x[:10] for x in s):
      a = abs(off); hist.append((t, a))
      while hist and t - hist[0][0] > 2.0:
        hist.pop(0)
      slope = 0.
      if len(hist) >= 6 and hist[-1][0] - hist[0][0] > 0.5:
        tt = np.array([h[0] for h in hist]); oo = np.array([h[1] for h in hist]); slope = float(np.polyfit(tt - tt[0], oo, 1)[0])
      vlim = self.vmax if self.vmax is not None else -1.5
      near = self.near and 0.8 < a < 2.6 and vr < vlim and 25 <= d <= 90 and slope < -0.15
      wide = False; need = 8
      if self.wide and 2.6 <= a < 5.0 and vr < (self.vmax if self.vmax is not None else -2.5) and 25 <= d <= 110:
        ok = (tow > 0.4) if self.lat else (slope < -0.3)
        if self.gate is not None and v < self.gate / 3.6:
          ok = False
        if self.curve and (abs(steer) > 20 or c50 > 3.0):
          ok = False
        wide = ok; need = self.wide_hold
      fastok = False
      if self.fast:
        inr = 0.9 < a < 5.0 and 15 <= d <= 120 and vr < self.fast_vr
        curvy = self.curve and (abs(steer) > 15 or c50 > 2.5)
        fastn = fastn + 1 if (inr and tow > self.fast_thr and not curvy) else 0
        fastok = inr and not curvy and (lc in (2, 5) or fastn >= self.fast_hold)
      if self.curve and (abs(steer) > 15 or c50 > 2.5):
        near = wide = False
      if self.close:
        cin = 0.9 < a < 4.0 and 5 <= d <= self.close_d and not (self.curve and (abs(steer) > 15 or c50 > 2.5))
        closen = closen + 1 if (cin and tow > self.close_thr) else 0
        if cin and closen >= self.close_hold:
          fastok = True
      if fastok:
        near = True
      if lead:
        near = wide = False
      if near or wide:
        hold += 1
        if hold >= (8 if near else need):
          rel = 17
      else:
        hold = 0
        rel = max(rel - 1, 0)
      if (hold >= (8 if near else need)) or rel > 0:
        out.append(i)
    return out


class LineGate:
  """another rule, kept only while the object is within thr m of crossing the lane line on its side"""
  def __init__(self, name, base, thr, path_extra=0.3):
    self.name, self.base, self.thr, self.path_extra = name, base, thr, path_extra

  def marks(self, s):
    out = []
    for i in self.base.marks(s):
      il, ip = s[i][10], s[i][11]
      intr, thr = (il, self.thr) if il is not None else (ip, self.thr + self.path_extra)
      if intr >= thr:
        out.append(i)
    return out


class DPRule:
  """herizon1054/openpilot radard_ext v9.6.7 cut-in prediction (TSS2 radar points), on ARS510 objects.
  line_enter: -0.1 as in dp (reflection points); an ARS510 object y is closer to the car centre, so -1.0 ~ the car edge."""
  def __init__(self, name, line_enter=-0.1, line_release=-0.2, path_enter=0.2, path_release=-0.1, max_d=None):
    self.name, self.le, self.lr, self.pe, self.pr, self.max_d = name, line_enter, line_release, path_enter, path_release, max_d

  def marks(self, s):
    out = []; foff = None; fvy = 0.; tp = None; hist = []; hits = 0; active = False; act_t = 0.; entered = False
    for i, (t, d, off, vr, tow, v, steer, c50, lead, lc, il, ip) in enumerate(s):
      dt = 0.06 if tp is None else max(t - tp, 0.02); tp = t
      if foff is None:
        foff, fvy = off, 0.
      else:
        pred = foff + fvy * dt; res = off - pred
        foff = pred + 0.3 * res; fvy = fvy + (0.05 / 0.05) * res   # dp: alpha 0.3, beta/DT_MDL (per record)
      hist.append((t, off))
      while hist and t - hist[0][0] > 1.0:
        hist.pop(0)
      gated = abs(steer) >= 15.
      intr, mode = (il, "line") if il is not None else (ip, "path")
      vabs = vr + v
      max_d = self.max_d if self.max_d is not None else float(np.interp(v, [10., 30.], [20., 50.]))
      base_ok = (not gated and v > ((20 if active else 30) / 3.6) and 3. < d < max_d and vabs > max(3., 0.2 * v) and
                 (vr < -0.5 or d < 1.0 * v))
      a = abs(foff)
      if active:
        within = intr >= (self.lr if mode == "line" else self.pr)
        away = foff * fvy > 0 and abs(fvy) > 0.3 and a > 1.0
        act_t += dt
        entered = entered or a <= 1.0
        if (not entered and act_t > 2.5) or not (base_ok and within and not away):
          active = False; hits = 0; act_t = 0.; entered = False
      else:
        lat_ok = a > 1.0 and intr >= (self.le if mode == "line" else self.pe)
        hit = base_ok and lat_ok and foff * fvy < 0 and 0.3 < abs(fvy) < 2.5 and (a - 1.0) / abs(fvy) < 1.5
        hits = hits + 1 if hit else 0
        need = 6 if v < 36 / 3.6 else 4
        if hits >= need:
          now = hist[-1][1]; direction = -1. if now > 0 else 1.
          shift_ok = len(hist) >= 11 and (now - hist[0][1]) * direction >= 0.5
          if shift_ok and t - hist[0][0] < 0.95:   # tracked < 1 s: no outward bounce > 0.15 m
            best = hist[0][1] * direction
            for _, x in hist[1:]:
              if best - x * direction > 0.15:
                shift_ok = False; break
              best = max(best, x * direction)
          if shift_ok:
            active = True; act_t = 0.; entered = a <= 1.0
      if active and not lead:
        out.append(i)
    return out


_NOW = Rule("ours (deployed)", wide=True, gate=60, curve=False, fast=True, fast_thr=1.0, fast_hold=8, close=True)
RULES = [_NOW,
         LineGate("ours + line >= -1.5", _NOW, -1.5), LineGate("ours + line >= -1.0", _NOW, -1.0),
         LineGate("ours + line >= -0.5", _NOW, -0.5), LineGate("ours + line >= -0.1", _NOW, -0.1),
         DPRule("dp v9.6.7 as-is (-0.1)"), DPRule("dp, object centre (-1.0)", line_enter=-1.0, line_release=-1.1, path_enter=-0.7, path_release=-1.0),
         DPRule("dp centre, to 110 m", line_enter=-1.0, line_release=-1.1, path_enter=-0.7, path_release=-1.0, max_d=110.)]

CACHE = os.environ.get("MERGE_CACHE", os.path.expanduser("~/.cache/ars510_merge_lanes")) + "_" + os.environ.get("VLAT_MODE", "raw_median")


def prepared(f):
  """(km by band, tracks) for one slim file, cached (decoding the object list is the slow part)."""
  name = os.path.basename(f).replace(".pkl.zst", "")
  cp = os.path.join(CACHE, name + ".pkl")
  if os.path.exists(cp):
    return pickle.load(open(cp, "rb"))
  seg = load(f)
  km = defaultdict(float)
  car = seg["car"]
  for (t0, v0, *_), (t1, *_) in zip(car, car[1:]):
    for lo, hi in BANDS:
      if lo <= v0 * 3.6 < hi:
        km[(lo, hi)] += v0 * (t1 - t0) / 1000
  out = (dict(km), dict(tracks_of(seg, lanes_for(name))))
  os.makedirs(CACHE, exist_ok=True)
  tmp = cp + ".part"
  pickle.dump(out, open(tmp, "wb"), protocol=4); os.replace(tmp, cp)
  return out


def prep(files, budget_s):
  """decode up to budget_s seconds of not-yet-cached files on all cores (for short shells), then return"""
  import time
  from multiprocessing import Pool
  todo = [f for f in files if not os.path.exists(os.path.join(CACHE, os.path.basename(f).replace(".pkl.zst", "") + ".pkl"))]
  t0 = time.monotonic(); done = 0
  with Pool(os.cpu_count()) as pool:
    for _ in pool.imap_unordered(prepared, todo):
      done += 1
      if time.monotonic() - t0 > budget_s:
        pool.terminate(); break
  print(f"prepared {done}, remaining {len(todo) - done}", flush=True)


def main():
  d = sys.argv[1]
  files = sorted(glob.glob(os.path.join(d, "*.pkl.zst")))
  if os.environ.get("ONLY_LANES", "1") == "1":
    files = [f for f in files if os.path.exists(os.path.join(LANE_DIR, os.path.basename(f).replace(".pkl.zst", ".lanes.pkl.zst")))]
  real = {}; km = defaultdict(float); stats = {r.name: defaultdict(lambda: [0, 0, [], set()]) for r in RULES}
  episodes = {r.name: [] for r in RULES}
  if "--prep" in sys.argv:
    prep(files, float(sys.argv[sys.argv.index("--prep") + 1])); return
  for f in files:
    name = os.path.basename(f).replace(".pkl.zst", "")
    fkm, ftracks = prepared(f)
    for b, x in fkm.items():
      km[b] += x
    for key, s in ftracks.items():
      if len(s) < 10:
        continue
      k = (name, key)
      # real cut-in
      out_t = None; tin = None
      for t, dd, off, vr, tow, v, steer, c50, lead, lc in (x[:10] for x in s):
        if abs(off) >= 1.8 and not lead:
          out_t = t
        elif out_t is not None and (abs(off) < 0.9 or lead) and v > 20 / 3.6 and 15 < dd < 110 and t - out_t < 6:
          tin = t; break
      if tin is not None:
        real[k] = (tin, next(x[5] for x in s if x[0] >= tin))
      for r in RULES:
        idx = r.marks(s)
        if not idx:
          continue
        # split into episodes (gap > 1 s)
        eps = []; cur = [idx[0]]
        for i in idx[1:]:
          if s[i][0] - s[cur[-1]][0] > 1.0:
            eps.append(cur); cur = [i]
          else:
            cur.append(i)
        eps.append(cur)
        for e in eps:
          ta, tb = s[e[0]][0], s[e[-1]][0]; v = s[e[0]][5]
          band = next(b for b in BANDS if b[0] <= v * 3.6 < b[1])
          ok = tin is not None and ta - 1 <= tin <= tb + 8
          if not ok:
            ok = any(ta <= x[0] <= tb + 8 and (abs(x[2]) < 0.9 or x[8]) for x in s)
          st = stats[r.name][band]
          st[0 if ok else 1] += 1
          if ok:
            st[3].add(k)
            if tin is not None:
              st[2].append(tin - ta)
          episodes[r.name].append((name, key, ok, round(ta - s[0][0], 1), round(s[e[0]][1]), round(s[e[0]][2], 1), round(v * 3.6), round(tb - ta, 1)))
  rb = defaultdict(int)
  for k, (tin, v) in real.items():
    rb[next(b for b in BANDS if b[0] <= v * 3.6 < b[1])] += 1
  print(f"files {len(files)}; km by band: " + ", ".join(f"{lo}-{hi}: {km[(lo, hi)]:.0f}" for lo, hi in BANDS))
  print("real cut-ins by band: " + ", ".join(f"{lo}-{hi}: {rb[(lo, hi)]}" for lo, hi in BANDS))
  for r in RULES:
    print(f"\n[{r.name}]")
    for b in BANDS:
      tp, fp, lt, hit = stats[r.name][b]
      rec = len([k for k in hit if k in real and next(bb for bb in BANDS if bb[0] <= real[k][1] * 3.6 < bb[1]) == b]) / max(rb[b], 1)
      lt = sorted(lt); med = lt[len(lt) // 2] if lt else float("nan")
      per100 = fp / max(km[b], 1e-6) * 100
      print(f"  {b[0]:3d}-{b[1]:3d} km/h: true {tp:4d} false {fp:4d} ({fp / max(tp + fp, 1) * 100:3.0f}%)  false/100km {per100:5.1f}  "
            f"recall {rec * 100:3.0f}%  warning median {med:4.1f} s")
  if "--list" in sys.argv:
    for r in RULES[-1:]:
      for e in episodes[r.name]:
        print(r.name, *e)


if __name__ == "__main__":
  main()
