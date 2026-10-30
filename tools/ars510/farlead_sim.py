#!/usr/bin/env python3
"""ars510 closed-loop sim of the far-lead floor + early ease-off with the real long MPC (run on the device, offroad).
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 farlead_sim.py [planner.py]"""
import sys, types, importlib.util
sys.path.insert(0, "/data/openpilot")
import numpy as np
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS as T_IDXS_MPC, get_T_FOLLOW
path = sys.argv[1] if len(sys.argv) > 1 else "/data/openpilot/openpilot/selfdrive/controls/lib/longitudinal_planner.py"
spec = importlib.util.spec_from_file_location("lp_sim", path); LP = importlib.util.module_from_spec(spec); spec.loader.exec_module(LP)
CT = ModelConstants.T_IDXS[:CONTROL_N]
STD = log.LongitudinalPersonality.standard
TF = get_T_FOLLOW(STD)


def run(d0, v0, vl0, mode, radar=True, t_end=30., lead_brake_at=None, lead_brake=0., seen_vrel=None, cam_delay=0.):
  mpc = LongitudinalMpc(dt=DT_MDL)
  x, v, a = 0., v0, 0.; xl, vl = d0, vl0
  out_a = 0.; vf = v0; cap = None; res = []
  for i in range(int(t_end / DT_MDL)):
    t = i * DT_MDL
    alc = lead_brake if (lead_brake_at is not None and t >= lead_brake_at and vl > 0) else 0.
    rs = log.RadarState.new_message()
    l = rs.leadOne; l.present = True; l.radar = radar; l.dRel = xl - x
    l.vRel = (vl - v) if seen_vrel is None else max(vl - v, seen_vrel)  # camera-only: underestimated closing
    l.vLead = v + l.vRel; l.vLeadK = l.vLead; l.aLeadK = alc; l.aLeadTau = 1.5; l.modelProb = 1.0
    fo = None
    if t < cam_delay:  # openpilot has no lead yet; only the radar-only far object
      fo = types.SimpleNamespace(present=True, radar=True, dRel=l.dRel, vRel=l.vRel, vLead=l.vLead, aLeadK=0.)
      l.present = False
    mpc.set_weights(True, personality=STD); mpc.set_cur_state(vf, out_a); mpc.update(rs, personality=STD)
    vt = np.interp(CT, T_IDXS_MPC, mpc.v_solution); at = np.interp(CT, T_IDXS_MPC, mpc.a_solution)
    a_cmd = min(get_accel_from_plan(vt, at, CT, action_t=0.15 + DT_MDL), 0.3)  # cruise at set speed: no more than +0.3
    if mode != "off" and mpc.crash_cnt == 0 and radar and fo is None:
      r = LP.far_lead_accel_floor(l, TF)
      if r and a_cmd < r[0]: a_cmd = a_cmd + r[1] * (r[0] - a_cmd)
    if mode == "early" and mpc.crash_cnt == 0:
      r = LP.far_lead_accel_ceil(l, TF) if fo is None else LP.far_lead_accel_ceil(fo, TF)
      if r and fo is not None: r = (max(r[0], LP.FAR_RADAR_ONLY_MAX_DECEL), r[1])
      if r:
        tgt = a_cmd + r[1] * (r[0] - a_cmd) if a_cmd > r[0] else a_cmd
        cap = max(out_a, tgt) if cap is None else cap
        cap = max(tgt, cap - LP.FAR_EARLY_JERK * DT_MDL)
        a_cmd = min(a_cmd, cap)
      else:
        cap = None
    a_prev = out_a; out_a = float(np.clip(a_cmd, -3.5, 2.0))
    vf = vf + DT_MDL * (out_a + a_prev) / 2.
    a += (out_a - a) * DT_MDL / 0.4
    v = max(v + a * DT_MDL, 0.); x += v * DT_MDL
    vl = max(vl + alc * DT_MDL, 0.); xl += vl * DT_MDL
    vf = 0.9 * vf + 0.1 * v
    res.append((t, v, a, out_a, xl - x, vl - v, mpc.crash_cnt))
  r = np.array(res)
  brake = r[:, 3] < -0.5
  first = np.flatnonzero(r[:, 3] < -0.1)
  return dict(a_min=round(float(r[:, 3].min()), 2), t_start=round(float(r[first[0], 0]), 1) if len(first) else None,
              t_below_m05=round(float(brake.sum() * DT_MDL), 1), d_min=round(float(r[:, 4].min()), 1),
              timegap_min=round(float((r[:, 4] / np.maximum(r[:, 1], 1.)).min()), 2), crash=int(r[:, 6].max()),
              d_end=round(float(r[-1, 4]), 1), vrel_end=round(float(r[-1, 5]), 2), v_loss_kmh=round(float((v0 - r[-1, 1]) * 3.6), 1))


if __name__ == "__main__":
  cases = [("15:04 126m 107->80, camera 1.5s after radar", dict(d0=126, v0=29.8, vl0=22.3, cam_delay=1.5)),
           ("15:04 120m 107->80", dict(d0=120, v0=29.8, vl0=22.4)),
           ("radar-only object that is never a lead (5 s)", dict(d0=140, v0=30.6, vl0=25.0, cam_delay=5., t_end=5.)),
           ("13:27 118m 118->75", dict(d0=118, v0=32.9, vl0=20.8)),
           ("100m 110->90", dict(d0=100, v0=30.6, vl0=25.0)),
           ("150m 120->50", dict(d0=150, v0=33.3, vl0=13.9)),
           ("120m 110->80, lead brakes -3 at 3s", dict(d0=120, v0=30.6, vl0=22.2, lead_brake_at=3., lead_brake=-3.)),
           ("110m 110->70 camera only, vRel seen -5", dict(d0=110, v0=30.6, vl0=19.4, radar=False, seen_vrel=-5.)),
           ("130m 110->0 stopped car", dict(d0=130, v0=30.6, vl0=0.0)),
           ("60m 100->95 close", dict(d0=60, v0=27.8, vl0=26.4))]
  for name, c in cases:
    print(name)
    for mode in ("off", "floor", "early"):
      print(f"   {mode:6} {run(mode=mode, **c)}")
