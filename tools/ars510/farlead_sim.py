#!/usr/bin/env python3
"""ars510 closed-loop sim of the far-lead braking limit with the real long MPC (run on the device, offroad).
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 /data/farlead_sim.py"""
import sys, types
sys.path.insert(0, "/data/openpilot")
import numpy as np
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS as T_IDXS_MPC, get_T_FOLLOW
import openpilot.selfdrive.controls.lib.longitudinal_planner as LP
CT = ModelConstants.T_IDXS[:CONTROL_N]
STD = log.LongitudinalPersonality.standard


def run(d0, v0, vl0, al=0.0, k=None, margin=None, on=True, t_end=25., lead_brake_at=None, lead_brake=0.):
  if k is not None: LP.FAR_LEAD_K = k
  if margin is not None: LP.FAR_LEAD_MARGIN = margin
  mpc = LongitudinalMpc(dt=DT_MDL)
  x, v, a = 0., v0, 0.; xl, vl = d0, vl0
  out_a = 0.; vf = v0; res = []
  for i in range(int(t_end / DT_MDL)):
    t = i * DT_MDL
    alc = lead_brake if (lead_brake_at is not None and t >= lead_brake_at and vl > 0) else al
    rs = log.RadarState.new_message()
    l = rs.leadOne; l.present = True; l.radar = True; l.dRel = xl - x; l.vRel = vl - v; l.vLead = vl; l.vLeadK = vl
    l.aLeadK = alc; l.aLeadTau = 1.5; l.modelProb = 1.0
    mpc.set_weights(True, personality=STD)
    mpc.set_cur_state(vf, out_a)
    mpc.update(rs, personality=STD)
    vt = np.interp(CT, T_IDXS_MPC, mpc.v_solution); at = np.interp(CT, T_IDXS_MPC, mpc.a_solution)
    a_cmd = get_accel_from_plan(vt, at, CT, action_t=0.15 + DT_MDL)
    if on and mpc.crash_cnt == 0:
      r = LP.far_lead_accel_floor(l, get_T_FOLLOW(STD))
      if r and a_cmd < r[0]: a_cmd = a_cmd + r[1] * (r[0] - a_cmd)
    a_prev = out_a; out_a = float(np.clip(a_cmd, -3.5, 2.0))
    vf = vf + DT_MDL * (out_a + a_prev) / 2.
    # car: accel follows the command with 0.4 s lag, then 0.2 s delay-ish
    a += (out_a - a) * DT_MDL / 0.4
    v = max(v + a * DT_MDL, 0.); x += v * DT_MDL
    vl = max(vl + alc * DT_MDL, 0.); xl += vl * DT_MDL
    vf = 0.9 * vf + 0.1 * v  # planner filter tracks v_ego
    res.append((t, v, a, out_a, xl - x, vl - v, mpc.crash_cnt))
  r = np.array(res)
  gap_t = r[:, 4] / np.maximum(r[:, 1], 1.)
  return dict(a_min=round(float(r[:, 3].min()), 2), a_car_min=round(float(r[:, 2].min()), 2),
              d_min=round(float(r[:, 4].min()), 1), timegap_min=round(float(gap_t.min()), 2),
              t_below_m1=round(float((r[:, 3] < -1.0).sum() * DT_MDL), 1), crash=int(r[:, 6].max()),
              d_end=round(float(r[-1, 4]), 1), vrel_end=round(float(r[-1, 5]), 2))


if __name__ == "__main__":
  cases = [("13:27 118m 118->75", dict(d0=118, v0=32.9, vl0=20.8)),
           ("100m 110->60", dict(d0=100, v0=30.6, vl0=16.7)),
           ("150m 120->50", dict(d0=150, v0=33.3, vl0=13.9)),
           ("120m 110->80, lead brakes -3 at 3s", dict(d0=120, v0=30.6, vl0=22.2, lead_brake_at=3., lead_brake=-3.)),
           ("90m 100->70", dict(d0=90, v0=27.8, vl0=19.4)),
           ("130m 110->0 stopped car", dict(d0=130, v0=30.6, vl0=0.0))]
  for name, c in cases:
    base = run(on=False, **c)
    print(f"{name:38} OFF {base}")
    for k, m in ((1.5, 10.), (1.25, 5.)):
      print(f"{'':38} K={k} M={m:.0f} {run(k=k, margin=m, **c)}")
