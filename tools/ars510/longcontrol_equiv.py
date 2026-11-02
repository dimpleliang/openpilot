#!/usr/bin/env python3
"""Frame-by-frame equivalence check of two longcontrol.py versions (G1 refactor: behaviour must not change).

Both versions are loaded as standalone modules against the SAME dependency modules (the real openpilot / opendbc ones
when importable, otherwise minimal stubs: LongCtrlState enum, PIDController copy, DT_CTRL, CONTROL_N, T_IDXS), fed the
same inputs every frame, and compared on output_accel (|diff| <= 1e-9) and long_control_state; with --strict (default)
also on the internal latches both versions share. The first differing frame is reported with context.

  # synthetic closed-loop scenarios (plant driven by the OLD output; both get identical inputs)
  python3 tools/ars510/longcontrol_equiv.py synth [--n-rand 50] [--seed 1]
  # extract per-frame inputs from rlogs (one LoC.update per carControl message, as controlsd does)
  python3 tools/ars510/longcontrol_equiv.py extract --out route.npz SEG_DIR [SEG_DIR ...]
  # replay extracted inputs
  python3 tools/ars510/longcontrol_equiv.py replay route.npz [...]
Common options: --old FILE (default: `git show 462845c:...longcontrol.py`), --new FILE (default: the repo file),
--stub (force the stub dependencies).
"""
import argparse
import math
import os
import subprocess
import sys
import types
from collections import Counter, deque
from types import SimpleNamespace

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LC_PATH = "openpilot/selfdrive/controls/lib/longcontrol.py"
BASE_REV = "462845c"
DT = 0.01
TOL = 1e-9
SHARED_ATTRS = ("v_est", "coast_frames", "pose_a_f", "stall_stop", "brake_hold", "was_rolling", "settle_frames",
                "settle_used", "coast_active", "true_stopped", "no_decel_frames", "brake_added", "pre_release",
                "pre_release_failed", "pre_release_frames", "pre_release_slow", "pre_release_start", "still_frames",
                "still_set", "prev_v_ego")


# ------------------------------------------------------------------------------------------------- dependency stubs
class _StubPID:
  """Copy of openpilot.common.pid.PIDController (used only when the real one cannot be imported)."""
  def __init__(self, k_p, k_i, k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
    self._k_p = ([0], [k_p]) if isinstance(k_p, (int, float)) else k_p
    self._k_i = ([0], [k_i]) if isinstance(k_i, (int, float)) else k_i
    self._k_d = ([0], [k_d]) if isinstance(k_d, (int, float)) else k_d
    self.pos_limit, self.neg_limit = pos_limit, neg_limit
    self.i_dt = 1.0 / rate
    self.speed = 0.0
    self.reset()

  k_p = property(lambda s: np.interp(s.speed, s._k_p[0], s._k_p[1]))
  k_i = property(lambda s: np.interp(s.speed, s._k_i[0], s._k_i[1]))
  k_d = property(lambda s: np.interp(s.speed, s._k_d[0], s._k_d[1]))

  def reset(self):
    self.p = self.i = self.d = self.f = 0.0
    self.control = 0

  def update(self, error, error_rate=0.0, speed=0.0, feedforward=0., freeze_integrator=False):
    self.speed = speed
    self.p = self.k_p * float(error)
    self.d = self.k_d * error_rate
    self.f = feedforward
    if not freeze_integrator:
      i = self.i + self.k_i * self.i_dt * error
      test_control = self.p + i + self.d + self.f
      i_upperbound = self.i if test_control > self.pos_limit else self.pos_limit
      i_lowerbound = self.i if test_control < self.neg_limit else self.neg_limit
      self.i = np.clip(i, i_lowerbound, i_upperbound)
    control = self.p + self.i + self.d + self.f
    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    return self.control


def _install_stubs():
  def mod(name, **attrs):
    m = sys.modules.get(name) or types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m
  lcs = SimpleNamespace(off=0, pid=1, stopping=2, starting=3)
  car = SimpleNamespace(CarControl=SimpleNamespace(Actuators=SimpleNamespace(LongControlState=lcs)))
  for n in ("opendbc", "opendbc.car", "openpilot", "openpilot.common", "openpilot.selfdrive",
            "openpilot.selfdrive.controls", "openpilot.selfdrive.controls.lib", "openpilot.selfdrive.modeld"):
    if n not in sys.modules:
      try:
        __import__(n)
      except Exception:
        mod(n).__path__ = []
  mod("opendbc.car.structs", car=car)
  mod("openpilot.common.realtime", DT_CTRL=0.01)
  mod("openpilot.common.pid", PIDController=_StubPID)
  mod("openpilot.selfdrive.controls.lib.drive_helpers", CONTROL_N=17)
  t_idxs = [10.0 * (i / 32) ** 2 for i in range(33)]
  mod("openpilot.selfdrive.modeld.constants", ModelConstants=SimpleNamespace(T_IDXS=t_idxs))
  return "stub"


def setup_deps(force_stub=False):
  if not force_stub:
    try:
      sys.path[:0] = [REPO, os.path.join(REPO, "opendbc_repo")]
      import opendbc.car.structs  # noqa: F401
      import openpilot.common.pid
      import openpilot.common.realtime
      import openpilot.selfdrive.controls.lib.drive_helpers
      import openpilot.selfdrive.modeld.constants  # noqa: F401
      return "real"
    except Exception:
      pass
  return _install_stubs()


def load_module(src, name):
  m = types.ModuleType(name)
  m.__file__ = name + ".py"
  exec(compile(src, name + ".py", "exec"), m.__dict__)
  return m


def load_versions(args):
  if args.old:
    old_src = open(args.old).read()
  else:
    old_src = subprocess.check_output(["git", "-C", REPO, "show", f"{BASE_REV}:{LC_PATH}"], text=True)
  new_src = open(args.new or os.path.join(REPO, LC_PATH)).read()
  return load_module(old_src, "lc_old"), load_module(new_src, "lc_new")


def make_cp(ki_bp=(0.,), ki_v=(0.,), stop_accel=-2.0):
  CP = SimpleNamespace(stopAccel=stop_accel, longitudinalTuning=SimpleNamespace(kiBP=list(ki_bp), kiV=list(ki_v)))
  return CP, SimpleNamespace(enableGasInterceptor=False)


def make_cs(v, vraw, a, standstill, brake, gas, cruise_ss):
  return SimpleNamespace(vEgo=v, vEgoRaw=vraw, aEgo=a, standstill=standstill, brakePressed=brake, gasPressed=gas,
                         cruiseState=SimpleNamespace(standstill=cruise_ss))


# ------------------------------------------------------------------------------------------------------- comparison
class Comparator:
  def __init__(self, old_mod, new_mod, CP, CP_SP, strict=True, pre_release_enabled=None):
    self.old = old_mod.LongControl(CP, CP_SP)
    self.new = new_mod.LongControl(CP, CP_SP)
    if pre_release_enabled is not None:
      self.old.pre_release_enabled = self.new.pre_release_enabled = pre_release_enabled
    self.strict = strict
    self.n = 0
    self.diff = None
    self.hist = deque(maxlen=8)
    self.phases = Counter()
    self.transitions = Counter()

  def step(self, active, CS, a_target, should_stop, limits, pose):
    if not active:  # controlsd: LoC.reset() when not CC.longActive
      self.old.reset()
      self.new.reset()
    o1 = float(self.old.update(active, CS, a_target, should_stop, limits, pose_accel=pose))
    lt_before = self.new.last_transition
    o2 = float(self.new.update(active, CS, a_target, should_stop, limits, pose_accel=pose))
    if self.new.last_transition is not lt_before:
      self.transitions[(self.new.last_transition[0].name, self.new.last_transition[1].name)] += 1
    self.phases[self.new.stop_phase.name] += 1
    rec = dict(i=self.n, active=active, v=CS.vEgo, vraw=CS.vEgoRaw, a=CS.aEgo, ss=CS.standstill, brk=CS.brakePressed,
               gas=CS.gasPressed, a_t=a_target, stop=should_stop, pose=pose, old=o1, new=o2,
               s_old=self.old.long_control_state, s_new=self.new.long_control_state, phase=self.new.stop_phase.name,
               trans=self.new.last_transition)
    self.hist.append(rec)
    bad = []
    if not (abs(o1 - o2) <= TOL):
      bad.append(f"output {o1!r} != {o2!r}")
    if self.old.long_control_state != self.new.long_control_state:
      bad.append(f"state {self.old.long_control_state} != {self.new.long_control_state}")
    if self.strict:
      for k in SHARED_ATTRS:
        x, y = getattr(self.old, k), getattr(self.new, k)
        if (isinstance(x, float) or isinstance(y, float)) and not isinstance(x, bool):
          if not abs(float(x) - float(y)) <= TOL:
            bad.append(f"{k} {x!r} != {y!r}")
        elif x != y:
          bad.append(f"{k} {x!r} != {y!r}")
    self.n += 1
    if bad and self.diff is None:
      self.diff = (self.n - 1, bad, list(self.hist))
    return o1

  def report_diff(self, label):
    i, bad, hist = self.diff
    print(f"  !! {label}: first difference at frame {i}: {'; '.join(bad)}")
    for r in hist:
      print("     ", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()})


# ------------------------------------------------------------------------------------------------ synthetic scenarios
def win(t, w):
  return any(a <= t < b for a, b in (w or []))


class Plant:
  """Crude Toyota-at-crawl plant: PCM lag, no brake for small requests at crawl (then a grab), idle creep, grade, wheel
  speed that drops to 0 below ~0.22 m/s, filtered vEgo / aEgo, noisy pose acceleration."""
  def __init__(self, p, rng):
    self.p, self.rng = p, rng
    self.v = p["v0"]
    self.x = 0.0
    self.cmd = deque([0.0] * max(1, int(p["lag"] / DT)), maxlen=max(1, int(p["lag"] / DT)))
    self.v_f = self.v
    self.a_f = 0.0
    self.a_true = 0.0

  def step(self, t, cmd, gas, brake):
    p = self.p
    self.cmd.append(cmd)
    a_cmd = self.cmd[0]
    if brake:
      a_cmd = min(a_cmd, -2.0)
    if gas:
      a_cmd = max(a_cmd, 1.0)
    band = self.v < 0.6 and p["band"] < a_cmd < 0.0
    a_brake = 0.0 if band else a_cmd
    creep = p["creep"] * max(0.0, 1.0 - self.v / 2.0) if a_cmd > -0.6 else 0.0
    drive = a_brake + creep + p["grade"]
    self.a_true = drive if (self.v > 0.0 or drive > 0.0) else 0.0   # at standstill: held unless the push wins
    self.v = max(0.0, self.v + self.a_true * DT)
    self.x += self.v * DT
    vraw = self.v + self.rng.normal(0, p["wnoise"]) if self.v >= p["drop_v"] else 0.0
    vraw = max(0.0, vraw)
    for a, b, vb in p.get("blip", []):
      if a <= t < b:
        vraw = vb
    v_prev = self.v_f
    self.v_f += (vraw - self.v_f) * 0.3
    if vraw == 0.0 and self.v_f < 0.03:
      self.v_f = 0.0
    self.a_f += ((self.v_f - v_prev) / DT - self.a_f) * 0.1
    pose = None if win(t, p.get("pose_off")) else self.a_true + self.rng.normal(0, p["pnoise"])
    return vraw, pose


BASE = dict(T=20.0, v0=5.0, stop_x=20.0, lag=0.4, band=-0.33, creep=0.3, grade=0.0, drop_v=0.22, wnoise=0.005,
            pnoise=0.02, restart_t=None, urgent=None, pose_off=None, gas=None, brake=None, inactive=None, blip=[],
            cruise_ss_t=None, plan="stop", plan_a=-0.1, ki=(0.,), pre_release=None, a_lim=(-3.5, 2.0))

SCENARIOS = {
  "normal_stop": {},
  "normal_stop_fast": dict(v0=12.0, stop_x=60.0),
  "normal_stop_short": dict(v0=3.0, stop_x=6.0),
  "stop_no_creep": dict(creep=0.0),
  "stop_strong_creep": dict(creep=0.6),
  "stop_small_band": dict(band=-0.15),
  "stop_no_band": dict(band=0.0),
  "stop_long_lag": dict(lag=0.7),
  "stop_early_dropout": dict(drop_v=0.4),
  "stop_with_pid_ki": dict(ki=(0.3, 0.2)),
  "creep_stall": dict(plan="stall", v0=0.45, plan_a=-0.1, creep=0.35, T=12.0),
  "creep_stall_then_go": dict(plan="stall", v0=0.45, plan_a=-0.1, creep=0.35, restart_t=6.0, T=12.0),
  "creep_stall_slow_drop": dict(plan="stall", v0=0.55, plan_a=-0.1, creep=0.0, grade=-0.07, T=10.0),
  "creep_stall_gas": dict(plan="stall", v0=0.45, plan_a=-0.1, creep=0.35, gas=[(3.0, 3.5)], T=12.0),
  "wheel_zero_pose_decel": dict(drop_v=0.35, creep=0.0),
  "pose_off_all": dict(pose_off=[(0.0, 99.0)]),
  "pose_off_approach": dict(pose_off=[(3.0, 6.5)]),
  "pose_off_late": dict(pose_off=[(6.5, 8.0)]),
  "pose_off_short_blips": dict(pose_off=[(5.0, 5.05), (6.0, 6.03), (7.0, 7.1), (7.5, 7.52)]),
  "pose_off_after_stop": dict(pose_off=[(9.0, 12.0)]),
  "pre_release_disabled": dict(pre_release=False),
  "pre_release_fail_creep": dict(creep=0.5, band=-0.4),
  "pre_release_fail_downhill": dict(grade=0.25),
  "urgent_during_release": dict(urgent=[(None, 0.8, -2.0)]),
  "urgent_mid_stop": dict(urgent=[(6.0, 6.5, -2.5)]),
  "urgent_after_stop": dict(urgent=[(10.0, 10.5, -3.0)]),
  "undo_wheel_blip": dict(blip=[(10.0, 10.3, 0.3)]),
  "undo_wheel_blip_small": dict(blip=[(10.0, 10.3, 0.12)]),
  "downhill_accelerating": dict(grade=0.6),
  "downhill_mild": dict(grade=0.2),
  "uphill": dict(grade=-0.3),
  "brake_hold_in_out": dict(plan="hover", v0=1.2, T=15.0),
  "brake_hold_gas": dict(plan="hover", v0=1.2, gas=[(6.0, 6.6)], T=15.0),
  "brake_hold_gas_crawl": dict(plan="stall", plan_a=-0.2, v0=0.5, creep=0.6, gas=[(0.5, 0.6), (1.5, 1.8), (3.0, 3.05)], T=8.0),
  "start_after_stop": dict(restart_t=12.0, T=20.0),
  "start_cruise_standstill": dict(restart_t=12.0, cruise_ss_t=10.0, T=20.0),
  "disengage_mid_stop": dict(inactive=[(6.0, 7.0)]),
  "disengage_at_stop": dict(inactive=[(10.0, 10.5)]),
  "driver_brake_mid_stop": dict(brake=[(6.5, 7.5)]),
  "driver_gas_at_stop": dict(gas=[(11.0, 12.0)]),
  "stop_from_standstill_engage": dict(v0=0.0, stop_x=0.0, T=8.0),
  "fast_should_stop": dict(plan="model_stop", v0=2.0, T=10.0),
  "lead_creeping": dict(plan="follow_creep", v0=1.0, T=15.0),
}


def planner(p, t, v, x, stopped_for):
  """Returns (a_target, should_stop)."""
  if p["restart_t"] is not None and t >= p["restart_t"]:
    return 0.8, False
  kind = p["plan"]
  if kind == "stop":
    d = p["stop_x"] - x
    a = -v * v / (2.0 * max(d, 0.3)) - 0.05 if v > 0.01 or d < 1.0 else -0.05
    a = float(np.clip(a, -3.0, 0.5))
    return a, (v < 0.3 and d < 2.0) or (d < 0.5)
  if kind == "stall":
    return p["plan_a"], False
  if kind == "hover":   # around the brake-hold thresholds, closing slowly
    return -0.15 + 0.12 * math.sin(t * 1.3) + (0.4 if 9.0 < t < 10.0 else 0.0), False
  if kind == "model_stop":
    return -0.6, t > 1.0
  if kind == "follow_creep":
    return 0.15 * math.sin(t * 0.7) - 0.05, v < 0.2
  raise ValueError(kind)


def run_scenario(old_mod, new_mod, name, over, rng, strict=True):
  p = dict(BASE)
  p.update(over)
  CP, CP_SP = make_cp((0., 5.) if len(p["ki"]) == 2 else (0.,), p["ki"])
  cmp = Comparator(old_mod, new_mod, CP, CP_SP, strict, p["pre_release"])
  plant = Plant(p, rng)
  cmd = 0.0
  stopped_for = 0.0
  for k in range(int(p["T"] / DT)):
    t = k * DT
    gas, brake = win(t, p["gas"]), win(t, p["brake"])
    active = not win(t, p["inactive"])
    vraw, pose = plant.step(t, cmd, gas, brake)
    stopped_for = stopped_for + DT if vraw == 0.0 else 0.0
    a_t, should_stop = planner(p, t, plant.v_f, plant.x, stopped_for)
    for a, b, val in (p["urgent"] or []):
      if a is None:   # relative to the first pre-release (old version's flag)
        if cmp.old.pre_release and not getattr(cmp, "_urg_t", None):
          cmp._urg_t = t
        a = getattr(cmp, "_urg_t", None)
        if a is not None and a <= t < a + b:
          a_t = val
      elif a <= t < b:
        a_t = val
    cruise_ss = p["cruise_ss_t"] is not None and t >= p["cruise_ss_t"] and (p["restart_t"] is None or t < p["restart_t"] + 0.5)
    CS = make_cs(plant.v_f, vraw, plant.a_f, vraw < 1e-3, brake, gas, cruise_ss)
    cmd = cmp.step(active, CS, a_t, should_stop, p["a_lim"], pose)
    if cmp.diff:
      break
  return cmp


def perturb(over, rng):
  o = dict(over)
  def j(k, rel):
    o[k] = BASE[k] if k not in o else o[k]
    o[k] = o[k] * (1.0 + rng.uniform(-rel, rel)) if isinstance(o[k], float) else o[k]
  for k, rel in (("v0", 0.3), ("stop_x", 0.3), ("lag", 0.4), ("creep", 0.5), ("drop_v", 0.25)):
    j(k, rel)
  o["band"] = o.get("band", BASE["band"]) + rng.uniform(-0.1, 0.1)
  o["grade"] = o.get("grade", BASE["grade"]) + rng.uniform(-0.15, 0.15)
  o["pnoise"] = rng.uniform(0.0, 0.08)
  o["wnoise"] = rng.uniform(0.0, 0.02)
  shift = rng.uniform(-1.5, 1.5)
  for key in ("pose_off", "gas", "brake", "inactive"):
    if o.get(key):
      o[key] = [(a + shift, b + shift) for a, b in o[key]]
  if rng.random() < 0.3:   # random pose drop-outs anywhere
    t0 = rng.uniform(0, o.get("T", BASE["T"]))
    o["pose_off"] = list(o.get("pose_off") or []) + [(t0, t0 + rng.uniform(0.02, 1.5))]
  if o.get("urgent"):
    o["urgent"] = [(a if a is None else a + shift, b if a is None else b + shift, v) for a, b, v in o["urgent"]]
  return o


def cmd_synth(args):
  old_mod, new_mod = load_versions(args)
  tot_frames = n_runs = n_bad = 0
  phases, trans = Counter(), Counter()
  for si, (name, over) in enumerate(SCENARIOS.items()):
    runs = [(name, over, np.random.default_rng(args.seed * 1000 + si))]
    prng = np.random.default_rng(args.seed * 1000 + si + 500)
    runs += [(f"{name}#{r}", perturb(over, prng), np.random.default_rng(args.seed * 100000 + si * 100 + r))
             for r in range(args.n_rand)]
    frames = bad = 0
    for label, o, rng in runs:
      cmp = run_scenario(old_mod, new_mod, label, o, rng, args.strict)
      frames += cmp.n
      phases.update(cmp.phases)
      trans.update(cmp.transitions)
      if cmp.diff:
        bad += 1
        cmp.report_diff(label)
    tot_frames += frames
    n_runs += len(runs)
    n_bad += bad
    print(f"{name:28s} runs {len(runs):3d}  frames {frames:7d}  differing runs {bad}")
  print(f"\nSYNTH: {len(SCENARIOS)} scenarios, {n_runs} runs, {tot_frames} frames, differing runs: {n_bad}")
  print("phase frames:", dict(sorted(phases.items())))
  print("transitions seen:", len(trans), dict(sorted(trans.items())))
  return n_bad


# --------------------------------------------------------------------------------------------------- rlog extraction
def _rot_from_euler(rpy):
  r, p, y = rpy
  Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
  Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
  Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
  return Rz @ Ry @ Rx


def extract_segment(seg_dir, log):
  import zstandard
  data = zstandard.ZstdDecompressor().stream_reader(open(os.path.join(seg_dir, "rlog.zst"), "rb")).read()
  cs = lp = None
  pose = None
  pose_t = -1e18
  calib = np.eye(3)
  calib_valid = False
  cal_ok = log.ExtrinsicsCalibration.Status.calibrated
  rows = []
  for m in log.Event.read_multiple_bytes(data):
    w = m.which()
    if w == "carState":
      c = m.carState
      cs = (c.vEgo, c.vEgoRaw, c.aEgo, c.standstill, c.brakePressed, c.gasPressed, c.cruiseState.standstill)
    elif w == "longitudinalPlan":
      lp = (m.longitudinalPlan.aTarget, m.longitudinalPlan.shouldStop)
    elif w == "extrinsicsCalibration":
      e = m.extrinsicsCalibration
      calib = _rot_from_euler(list(e.rpyCalib)).T if len(e.rpyCalib) == 3 else np.eye(3)
      calib_valid = e.calStatus == cal_ok
    elif w == "deviceMotion":
      d = m.deviceMotion.accelerationDevice
      pose = float((calib @ np.array([d.x, d.y, d.z]))[0]) if (m.valid and calib_valid) else None
      pose_t = m.logMonoTime
    elif w == "carControl" and cs is not None and lp is not None:
      cc = m.carControl
      pa = pose if (pose is not None and m.logMonoTime - pose_t < 150e6) else None
      rows.append((m.logMonoTime * 1e-9, cc.longActive) + cs + lp + (np.nan if pa is None else pa,
                  cc.actuators.accel, cc.actuators.longControlState.raw))
  return rows


COLS = ("t", "active", "v", "vraw", "a", "standstill", "brake", "gas", "cruise_ss", "a_target", "should_stop", "pose",
        "log_accel", "log_state")


def cmd_extract(args):
  sys.path[:0] = [os.path.expanduser("~/op"), os.path.expanduser("~/op/opendbc_repo")]
  from openpilot.cereal import log
  rows = []
  segs = []
  for s in args.segs:
    r = extract_segment(s, log)
    segs += [len(rows)] if r else []
    rows += r
    print(f"{os.path.basename(s.rstrip('/'))}: {len(r)} frames", flush=True)
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(COLS))
  np.savez_compressed(args.out, data=arr, cols=np.array(COLS), seg_start=np.array(segs))
  print(f"wrote {args.out}: {len(arr)} frames")


def cmd_replay(args):
  old_mod, new_mod = load_versions(args)
  tot = stops = bad = 0
  phases, trans = Counter(), Counter()
  for f in args.files:
    z = np.load(f)
    d = {c: z["data"][:, i] for i, c in enumerate(z["cols"])}
    CP, CP_SP = make_cp()
    cmp = Comparator(old_mod, new_mod, CP, CP_SP, args.strict, False if args.no_pre_release else None)
    n = len(d["t"])
    if args.no_pose:
      d["pose"] = np.full(n, np.nan)
    elif args.pose_gaps:   # variant: knock out random 0.05-1.5 s pose blocks (~args.pose_gaps of the frames)
      rng = np.random.default_rng(7)
      d["pose"] = d["pose"].copy()
      for _ in range(int(n * args.pose_gaps / 75)):
        k = int(rng.integers(0, n))
        d["pose"][k:k + int(rng.integers(5, 150))] = np.nan
    prev_ss = True
    route_stops = 0
    for i in range(n):
      active = bool(d["active"][i])
      ss = bool(d["standstill"][i])
      CS = make_cs(float(d["v"][i]), float(d["vraw"][i]), float(d["a"][i]), ss, bool(d["brake"][i]), bool(d["gas"][i]),
                   bool(d["cruise_ss"][i]))
      pose = None if math.isnan(d["pose"][i]) else float(d["pose"][i])
      cmp.step(active, CS, float(d["a_target"][i]), bool(d["should_stop"][i]), (-3.5, 2.0), pose)
      if active and ss and not prev_ss:
        route_stops += 1
      prev_ss = ss
      if cmp.diff:
        break
    pose_valid = float(np.mean(~np.isnan(d["pose"])))
    print(f"{os.path.basename(f)}: frames {cmp.n}/{n}  active {int(d['active'].sum())}  stops {route_stops}  "
          f"pose valid {pose_valid:.1%}  differences {'NONE' if not cmp.diff else 'YES'}")
    if cmp.diff:
      cmp.report_diff(os.path.basename(f))
      bad += 1
    tot += cmp.n
    stops += route_stops
    phases.update(cmp.phases)
    trans.update(cmp.transitions)
  print(f"\nREPLAY: {len(args.files)} files, {tot} frames, {stops} stops (active, standstill rising), differing files: {bad}")
  print("phase frames:", dict(sorted(phases.items())))
  print("transitions seen:", dict(sorted(trans.items())))
  return bad


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--old")
  ap.add_argument("--new")
  ap.add_argument("--stub", action="store_true")
  ap.add_argument("--no-strict", dest="strict", action="store_false")
  sub = ap.add_subparsers(dest="cmd", required=True)
  s = sub.add_parser("synth")
  s.add_argument("--n-rand", type=int, default=50)
  s.add_argument("--seed", type=int, default=1)
  e = sub.add_parser("extract")
  e.add_argument("--out", required=True)
  e.add_argument("segs", nargs="+")
  r = sub.add_parser("replay")
  r.add_argument("files", nargs="+")
  r.add_argument("--no-pose", action="store_true", help="variant: pose_accel None throughout (timer settle path)")
  r.add_argument("--pose-gaps", type=float, default=0.0, help="variant: fraction of frames with the pose knocked out")
  r.add_argument("--no-pre-release", action="store_true", help="variant: pre-release disabled in both versions")
  args = ap.parse_args()
  if args.cmd == "extract":
    return cmd_extract(args)
  print("dependencies:", setup_deps(args.stub))
  rc = cmd_synth(args) if args.cmd == "synth" else cmd_replay(args)
  sys.exit(1 if rc else 0)


if __name__ == "__main__":
  main()
