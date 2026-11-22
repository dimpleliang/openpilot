"""Open-loop replay: brake-hold onset 0.6 (before 11/22) vs BRAKE_HOLD_V_EARLY, per engaged stop.
Reports how much earlier the floor is requested and the open-loop stopping-distance change (double integral of the
request difference over the 6 s before the wheel stop, i.e. as if the car followed the request change exactly; an upper
bound, the PCM gives less than asked at crawl).
usage: venv/bin/python bh_replay.py <repo root> <out.json> rlog...
"""
import sys, os, types, json
import numpy as np, zstandard
from multiprocessing import Pool
root = sys.argv[1]
sys.path[:0] = [root, os.path.join(root, 'opendbc_repo')]
sys.modules['setproctitle'] = types.SimpleNamespace(getproctitle=lambda: '', setproctitle=lambda *a: None)
class _Log:
  def __getattr__(self, k): return lambda *a, **kw: None
sys.modules['openpilot.common.swaglog'] = types.SimpleNamespace(cloudlog=_Log())
NS = types.SimpleNamespace


def run(path):
  try:
    from openpilot.cereal import log
    from openpilot.selfdrive.controls.lib import longcontrol as L
    from openpilot.selfdrive.locationd.helpers import Pose, PoseCalibrator
    d = zstandard.ZstdDecompressor().decompressobj().decompress(open(path, 'rb').read())
    msgs = list(log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61))
    CP = NS(stopAccel=-2.0, longitudinalTuning=NS(kiBP=[0.], kiV=[0.5]))
    for m in msgs:
      if m.which() == 'carParams':
        c = m.carParams
        CP = NS(stopAccel=c.stopAccel, longitudinalTuning=NS(kiBP=list(c.longitudinalTuning.kiBP), kiV=list(c.longitudinalTuning.kiV)))
        break
    CP_SP = NS(enableGasInterceptor=False)
    old, new = L.LongControl(CP, CP_SP), L.LongControl(CP, CP_SP)
    for lc in (old, new):
      lc.stop_floor = -0.30; lc.stop_floor_poll = -10**9; lc.hold_target = L.HOLD_TARGET; lc.hold_old = False
    old.brake_hold_v = 0.6; new.brake_hold_v = L.BRAKE_HOLD_V_EARLY
    cal = PoseCalibrator(); pose = None; plan = None; cs = None
    rows = []
    for m in msgs:
      w = m.which(); t = m.logMonoTime / 1e9
      if w == 'extrinsicsCalibration': cal.feed_extrinsics_calibration(m.extrinsicsCalibration)
      elif w == 'deviceMotion': pose = cal.build_calibrated_pose(Pose.from_device_motion(m.deviceMotion))
      elif w == 'longitudinalPlan': plan = m.longitudinalPlan
      elif w == 'carState': cs = m.carState
      elif w == 'carControl':
        if cs is None or plan is None: continue
        cl = m.carControl.longActive
        pa = pit = None
        if pose is not None and cal.calib_valid:
          pa = float(pose.acceleration.x); pit = float(pose.orientation.pitch)
        CS = NS(vEgo=cs.vEgo, vEgoRaw=cs.vEgoRaw, aEgo=cs.aEgo, brakePressed=cs.brakePressed, gasPressed=cs.gasPressed,
                standstill=cs.standstill, cruiseState=NS(standstill=cs.cruiseState.standstill))
        if not cl: old.reset(); new.reset()
        o = float(old.update(cl, CS, plan.aTarget, plan.shouldStop, (-3.5, 2.0), pose_accel=pa, pitch=pit))
        n = float(new.update(cl, CS, plan.aTarget, plan.shouldStop, (-3.5, 2.0), pose_accel=pa, pitch=pit))
        rows.append((t, cs.vEgo, o, n, float(cl), float(cs.brakePressed or cs.gasPressed), plan.aTarget))
    R = np.array(rows)
    out = []
    if len(R) < 200: return out
    for i in range(600, len(R)):
      if R[i, 1] < 0.05 and R[i - 1, 1] >= 0.05 and R[i - 600:i, 4].min() > 0 and R[i - 600:i, 5].max() == 0 and R[i - 600:i, 1].max() > 3:
        seg = R[i - 600:i + 1]
        da = seg[:, 3] - seg[:, 2]
        dv = np.cumsum(da) * 0.01
        dd = float(np.sum(dv) * 0.01)
        diff = np.where(np.abs(da) > 0.02)[0]
        t_first = float(seg[diff[0], 0] - R[i, 0]) if len(diff) else 0.
        v_first = float(seg[diff[0], 1]) if len(diff) else 0.
        out.append(dict(seg=os.path.basename(os.path.dirname(path)), t=float(R[i, 0]), dd=dd, t_first=t_first, v_first=v_first,
                        max_da=float(da.min()), plan_at_first=float(seg[diff[0], 6]) if len(diff) else 0.))
    return out
  except Exception as e:
    return [dict(err=str(e), seg=path)]


if __name__ == '__main__':
  with Pool(8) as p:
    res = p.map(run, sys.argv[3:])
  S = [s for r in res for s in r]
  json.dump(S, open(sys.argv[2], 'w'))
  ok = [s for s in S if 'err' not in s]
  print('stops', len(ok), 'errors', len(S) - len(ok))
  ch = [s for s in ok if s['t_first'] != 0.]
  print('changed', len(ch))
  if ch:
    dd = np.array([s['dd'] for s in ch])
    print('stopping distance change (m, open loop, - = shorter): median %.2f p10 %.2f min %.2f' % (np.median(dd), np.percentile(dd, 10), dd.min()))
    print('first difference: v median %.2f, %.1f s before the wheel stop; largest extra brake %.2f' % (
      np.median([s['v_first'] for s in ch]), np.median([s['t_first'] for s in ch]), min(s['max_da'] for s in ch)))
    for s in sorted(ch, key=lambda s: s['dd'])[:8]:
      print(' ', s['seg'][-12:], round(s['t'], 1), 'dd %.2f v_first %.2f t_first %.1f plan %.2f max_da %.2f' % (s['dd'], s['v_first'], s['t_first'], s['plan_at_first'], s['max_da']))
