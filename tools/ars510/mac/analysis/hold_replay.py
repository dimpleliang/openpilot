"""Open-loop replay of LongControl on rlogs: old hold (pre 11/19) vs new soft hold.
Validates the replay (old output vs logged actuators.accel) and reports the post-stop request per stop.
usage: venv/bin/python hold_replay.py <repo root> <out.json> rlog...
"""
import sys, os, types, json, glob
import numpy as np, zstandard

root = sys.argv[1]
sys.path[:0] = [root, os.path.join(root, 'opendbc_repo')]
sys.modules['setproctitle'] = types.SimpleNamespace(getproctitle=lambda: '', setproctitle=lambda *a: None)
class _Log:
  def __getattr__(self, k): return lambda *a, **kw: None
sys.modules['openpilot.common.swaglog'] = types.SimpleNamespace(cloudlog=_Log())

from openpilot.cereal import log  # noqa: E402
from openpilot.selfdrive.controls.lib import longcontrol as L  # noqa: E402
from openpilot.selfdrive.locationd.helpers import Pose, PoseCalibrator  # noqa: E402

NS = types.SimpleNamespace


def run(path):
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(path, 'rb').read())
  msgs = list(log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61))
  CP = None
  for m in msgs:
    if m.which() == 'carParams':
      c = m.carParams
      CP = NS(stopAccel=c.stopAccel, longitudinalTuning=NS(kiBP=list(c.longitudinalTuning.kiBP), kiV=list(c.longitudinalTuning.kiV)))
      break
  if CP is None:
    CP = NS(stopAccel=-2.0, longitudinalTuning=NS(kiBP=[0.], kiV=[0.5]))
  CP_SP = NS(enableGasInterceptor=False)
  old, new = L.LongControl(CP, CP_SP), L.LongControl(CP, CP_SP)
  for lc in (old, new):
    lc.stop_floor = -0.30; lc.hold_target = L.HOLD_TARGET; lc.stop_floor_poll = -10**9
  old.hold_old, new.hold_old = True, False
  cal = PoseCalibrator(); pose = None
  plan = None; cs = None; cc_long = False; logged = 0.; lead = None
  rows = []; err = []
  for m in msgs:
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'extrinsicsCalibration':
      cal.feed_extrinsics_calibration(m.extrinsicsCalibration)
    elif w == 'deviceMotion':
      pose = cal.build_calibrated_pose(Pose.from_device_motion(m.deviceMotion))
    elif w == 'longitudinalPlan':
      plan = m.longitudinalPlan
    elif w == 'radarState':
      l = m.radarState.leadOne; lead = (l.dRel, l.vLead, l.radar) if l.present else None
    elif w == 'carControl':
      cc_long = m.carControl.longActive; logged = m.carControl.actuators.accel
      if cs is None or plan is None:
        continue
      pa = pit = None
      if pose is not None and cal.calib_valid:
        pa = float(pose.acceleration.x); pit = float(pose.orientation.pitch)
      CS = NS(vEgo=cs.vEgo, vEgoRaw=cs.vEgoRaw, aEgo=cs.aEgo, brakePressed=cs.brakePressed, gasPressed=cs.gasPressed,
              standstill=cs.standstill, cruiseState=NS(standstill=cs.cruiseState.standstill))
      lim = (-3.5, 2.0)
      if not cc_long:
        old.reset(); new.reset()
      a_t = plan.aTarget
      # near_stop change (rules): standing behind a stopped radar lead closer than TOO_CLOSE -> plan held at -0.3 only
      a_new = a_t
      if lead and lead[2] and lead[0] < 4.0 and lead[1] < 0.3 and cs.vEgo < 0.05 and a_t <= -1.49:
        a_new = -0.3
      o = float(old.update(cc_long, CS, a_t, plan.shouldStop, lim, pose_accel=pa, pitch=pit))
      n = float(new.update(cc_long, CS, a_new, plan.shouldStop, lim, pose_accel=pa, pitch=pit))
      if cc_long:
        err.append(abs(o - logged))
      rows.append((t, cs.vEgo, logged, o, n, int(new.stop_phase) + (10 if new.hold_rolled else 0), pit if pit is not None else np.nan,
                   pa if pa is not None else np.nan, cc_long))
    elif w == 'carState':
      cs = m.carState
  return rows, err


def stops(rows):
  """Each entry to HOLD (phase 5) in the new controller: request traces for 4 s from there."""
  out = []
  R = np.array([r[:8] for r in rows], dtype=float)
  if len(R) == 0:
    return out
  rolled = R[:, 5] >= 10; ph = R[:, 5] % 10
  idx = np.where((ph[1:] == int(L.StopPhase.HOLD)) & (ph[:-1] != int(L.StopPhase.HOLD)))[0] + 1
  for i in idx:
    j = min(len(R), i + 400)
    seg = R[i:j]
    old_r, new_r = seg[:, 3], seg[:, 4]
    out.append(dict(t=float(R[i, 0]), pitch=float(np.nanmedian(seg[:, 6])) if np.isfinite(seg[:, 6]).any() else None,
                    old_end=float(old_r[-1]), new_end=float(new_r[-1]),
                    old_rate=float(np.max(np.abs(np.diff(old_r))) / 0.01) if len(seg) > 1 else 0.,
                    new_rate=float(np.max(np.abs(np.diff(new_r))) / 0.01) if len(seg) > 1 else 0.,
                    new_min_while_old_firmer=float(np.max(new_r - old_r)),
                    pose_max=float(np.nanmax(seg[:, 7])) if np.isfinite(seg[:, 7]).any() else None,
                    rolled=bool(rolled[i:j].any()), n=len(seg)))
  return out


res = dict(stops=[], err=[])
for p in sys.argv[3:]:
  try:
    rows, err = run(p)
  except Exception as e:
    print('fail', p, e, file=sys.stderr); continue
  s = stops(rows)
  for x in s:
    x['seg'] = os.path.basename(os.path.dirname(p))
  res['stops'] += s
  res['err'] += [float(np.percentile(err, 50)), float(np.percentile(err, 99)), float(np.max(err))] if err else []
  print(os.path.basename(os.path.dirname(p)), len(s), 'stops', 'err p99 %.3f max %.3f' % (np.percentile(err, 99), np.max(err)) if err else '', flush=True)
json.dump(res, open(sys.argv[2], 'w'))
