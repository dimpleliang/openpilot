#!/usr/bin/env python3
"""ars510: score cut-in / merge display rules on slim rlogs (tools/ars510/slim_rlog.py), from the raw 0x80 object list.

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


def tracks_of(seg):
  """[(track key, [sample...])] with sample = (t, d, off, vr, vlat_toward, v_ego, steer, curv50, is_lead)."""
  ev = [(t, 0, x) for t, *x in seg["can"]] + [(t, 1, x) for t, *x in seg["car"]] + [(t, 2, x) for t, *x in seg["model"]] + \
       [(t, 3, x) for t, *x in seg["lead"]]
  ev.sort(key=lambda e: e[0])
  ri = I.Ars510NativeRadarInterface(I.NativeInterfaceConfig(radar_bus=1))
  v = 0.; steer = 0.; path = None; leads = []; life = {}; tr = defaultdict(list)
  for t, k, x in ev:
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
        vlat = -o.v_lat_ground                     # right-positive
        toward = -vlat if off > 0 else vlat        # + = moving toward my path
        curv50 = abs(float(np.interp(50., path[0], path[1])))
        is_lead = any(abs(o.d_rel - ld) < 4 and abs(yr - ly) < 1.5 for ld, ly in leads)
        tr[life[o.slot][1]].append((t, o.d_rel, off, vr, toward, v, steer, curv50, is_lead))
  return tr


class Rule:
  def __init__(self, name, wide=False, gate=None, lat=False, curve=False, wide_hold=8, near=True):
    self.name, self.wide, self.gate, self.lat, self.curve, self.wide_hold, self.near = name, wide, gate, lat, curve, wide_hold, near

  def marks(self, s):
    """indices where the track is marked"""
    out = []; hold = 0; rel = 0; hist = []
    for i, (t, d, off, vr, tow, v, steer, c50, lead) in enumerate(s):
      a = abs(off); hist.append((t, a))
      while hist and t - hist[0][0] > 2.0:
        hist.pop(0)
      slope = 0.
      if len(hist) >= 6 and hist[-1][0] - hist[0][0] > 0.5:
        tt = np.array([h[0] for h in hist]); oo = np.array([h[1] for h in hist]); slope = float(np.polyfit(tt - tt[0], oo, 1)[0])
      near = self.near and 0.8 < a < 2.6 and vr < -1.5 and 25 <= d <= 90 and slope < -0.15
      wide = False; need = 8
      if self.wide and 2.6 <= a < 5.0 and vr < -2.5 and 25 <= d <= 110:
        ok = (tow > 0.4) if self.lat else (slope < -0.3)
        if self.gate is not None and v < self.gate / 3.6:
          ok = False
        if self.curve and (abs(steer) > 20 or c50 > 3.0):
          ok = False
        wide = ok; need = self.wide_hold
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


RULES = [Rule("old (near only)"), Rule("near+wide, no gate", wide=True), Rule("near+wide, gate 60", wide=True, gate=60),
         Rule("wide: ground lateral", wide=True, lat=True), Rule("wide: no curve/steer", wide=True, curve=True),
         Rule("wide: hold 1.5 s", wide=True, wide_hold=25),
         Rule("wide: lateral+curve+1.5 s", wide=True, lat=True, curve=True, wide_hold=25)]


def main():
  d = sys.argv[1]
  files = sorted(glob.glob(os.path.join(d, "*.pkl.zst")))
  real = {}; km = defaultdict(float); stats = {r.name: defaultdict(lambda: [0, 0, [], set()]) for r in RULES}
  episodes = {r.name: [] for r in RULES}
  for f in files:
    seg = load(f); name = os.path.basename(f).replace(".pkl.zst", "")
    car = seg["car"]
    for (t0, v0, *_), (t1, *_) in zip(car, car[1:]):
      for lo, hi in BANDS:
        if lo <= v0 * 3.6 < hi:
          km[(lo, hi)] += v0 * (t1 - t0) / 1000
    for key, s in tracks_of(seg).items():
      if len(s) < 10:
        continue
      k = (name, key)
      # real cut-in
      out_t = None; tin = None
      for t, dd, off, vr, tow, v, steer, c50, lead in s:
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
