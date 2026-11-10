#!/usr/bin/env python3
"""ars510: closed-loop comparison of two longcontrol.py versions on the crawl plant of longcontrol_equiv.py (PCM lag,
brake band at crawl, creep, grade, wheel-speed drop-out). Unlike the equivalence check, each version drives its OWN
plant, so the effect of a control change shows: deceleration over the last second before the stop, the largest
deceleration step (grab), stop position vs the plan and creep after the stop.

  python3 tools/ars510/brake_hold_sim.py [--old FILE | --old-rev REV] [--new FILE] [--n-rand 20]
"""
import argparse
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import longcontrol_equiv as E  # noqa: E402

STOP_SCENARIOS = ["normal_stop", "normal_stop_fast", "normal_stop_short", "stop_no_creep", "stop_strong_creep",
                  "stop_small_band", "stop_no_band", "stop_long_lag", "stop_early_dropout", "stop_with_pid_ki",
                  "downhill_mild", "uphill", "pre_release_fail_creep", "brake_hold_in_out", "creep_stall"]


def run(mod, over, rng):
  p = dict(E.BASE); p.update(over)
  CP, CP_SP = E.make_cp((0., 5.) if len(p["ki"]) == 2 else (0.,), p["ki"])
  lc = mod.LongControl(CP, CP_SP)
  if p["pre_release"] is not None:
    lc.pre_release_enabled = p["pre_release"]
  plant = E.Plant(p, rng)
  cmd = 0.; stopped_for = 0.; hist = []
  for k in range(int(p["T"] / E.DT)):
    t = k * E.DT
    vraw, pose = plant.step(t, cmd, False, False)
    stopped_for = stopped_for + E.DT if vraw == 0. else 0.
    a_t, ss = E.planner(p, t, plant.v_f, plant.x, stopped_for)
    CS = E.make_cs(plant.v_f, vraw, plant.a_f, vraw < 1e-3, False, False, False)
    cmd = float(lc.update(True, CS, a_t, ss, p["a_lim"], pose_accel=pose))
    hist.append((t, plant.v, plant.a_true, plant.x, cmd))
  h = np.array(hist)
  moving = np.where(h[:, 1] > 0.)[0]
  i_stop = moving[-1] + 1 if len(moving) and moving[-1] + 1 < len(h) else len(h) - 1
  t_stop = h[i_stop, 0]
  last = (h[:, 0] > t_stop - 1.0) & (h[:, 0] <= t_stop)
  # smooth the plant acceleration like the IMU EMA (~80 ms) for "felt" deceleration
  a_f = np.zeros(len(h))
  for i in range(1, len(h)):
    a_f[i] = a_f[i - 1] + (h[i, 2] - a_f[i - 1]) * E.DT / 0.08
  steps = np.abs(np.diff(a_f[(h[:, 0] > t_stop - 2.0) & (h[:, 0] <= t_stop)]) / E.DT) if last.any() else np.array([0.])
  creep = float(h[-1, 3] - h[i_stop, 3])
  # a_stop: felt decel at the instant the wheels stop (the jolt: it drops to 0 right after), like stop_report's a_stop
  return dict(peak=float(a_f[last].min()) if last.any() else 0., jerk=float(steps.max()) if len(steps) else 0.,
              x=float(h[i_stop, 3] - p["stop_x"]), creep=creep, a_stop=float(a_f[max(i_stop - 1, 0)]),
              rolled=float(creep > 0.02))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--old"); ap.add_argument("--old-rev", default="HEAD"); ap.add_argument("--new")
  ap.add_argument("--n-rand", type=int, default=20); ap.add_argument("--seed", type=int, default=1)
  ap.add_argument("--plant", default="band", help="band = the 10/28 crawl plant; track = the 11/10 measured plant "
                  "(the PCM brakes for light requests, see longcontrol_equiv.Plant) with creep 0.28 / comp 0.25 +- spread")
  a = ap.parse_args()
  E.setup_deps(True)
  old_src = open(a.old).read() if a.old else subprocess.check_output(["git", "-C", E.REPO, "show", f"{a.old_rev}:{E.LC_PATH}"], text=True)
  new_src = open(a.new or os.path.join(E.REPO, E.LC_PATH)).read()
  old, new = E.load_module(old_src, "lc_old"), E.load_module(new_src, "lc_new")
  print(f"{'scenario':24s} {'peak decel last 1 s':>22s} {'max jerk (felt)':>18s} {'stop pos vs plan':>18s} {'creep after':>13s}")
  allr = {"old": [], "new": []}
  for si, name in enumerate(STOP_SCENARIOS):
    over = dict(E.SCENARIOS[name])
    if a.plant == "track":
      # creep in the scenarios was tuned for the band plant (0.3 base, 0.6 strong): rescale to the measured 0.28 at rest
      over = dict(over, track=True, creep=over.get("creep", 0.3) * 0.28 / 0.3)
    prng = np.random.default_rng(a.seed * 1000 + si + 500)
    runs = [over] + [E.perturb(over, prng) for _ in range(a.n_rand)]
    if a.plant == "track":
      for r in runs[1:]:
        r["comp"] = float(np.clip(prng.normal(0.25, 0.05), 0.12, 0.38))   # PCM creep compensation spread
    res = {"old": [], "new": []}
    for r, o in enumerate(runs):
      for k, m in (("old", old), ("new", new)):
        res[k].append(run(m, o, np.random.default_rng(a.seed * 100000 + si * 100 + r)))
    for k in res:
      allr[k] += res[k]
    f = lambda k, q: np.median([x[q] for x in res[k]])
    print(f"{name:24s} {f('old','peak'):+6.2f} -> {f('new','peak'):+6.2f}       {f('old','jerk'):5.2f} -> {f('new','jerk'):5.2f}"
          f"      {f('old','x'):+5.2f} -> {f('new','x'):+5.2f} m   {f('old','creep'):4.2f} -> {f('new','creep'):4.2f}")
  g = lambda k, q, fn: fn([x[q] for x in allr[k]])
  print(f"a_stop (jolt) median {g('old','a_stop',np.median):+.2f} -> {g('new','a_stop',np.median):+.2f}, strongest "
        f"{g('old','a_stop',np.min):+.2f} -> {g('new','a_stop',np.min):+.2f}; runs that rolled > 2 cm after the stop: "
        f"{int(g('old','rolled',np.sum))} -> {int(g('new','rolled',np.sum))}")
  print(f"\nALL ({len(allr['new'])} runs) median peak {g('old','peak',np.median):+.2f} -> {g('new','peak',np.median):+.2f}, "
        f"worst {g('old','peak',np.min):+.2f} -> {g('new','peak',np.min):+.2f}; median felt jerk {g('old','jerk',np.median):.2f} -> "
        f"{g('new','jerk',np.median):.2f}; stop pos median {g('old','x',np.median):+.2f} -> {g('new','x',np.median):+.2f} m, "
        f"latest {g('old','x',np.max):+.2f} -> {g('new','x',np.max):+.2f} m; creep max {g('old','creep',np.max):.2f} -> {g('new','creep',np.max):.2f} m")


if __name__ == "__main__":
  main()
