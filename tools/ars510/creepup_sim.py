#!/usr/bin/env python3
"""ars510: closed-loop sim of closing up to a stopped lead from standstill and of a slow approach (the last metres).
Real long MPC + the real near_stop / landing / a_pos rules + the real longcontrol.py, on a crawl plant identified on
11/10 (parking lot, routes 77-79: 0x0A6 friction force vs wheel speed, 935 windows below 6 km/h).

  python creepup_sim.py <openpilot root> [--trace N]      (in the Mac replay container, see mac/replay.sh)

What it cannot show: the model (e2e) candidate and the Toyota carcontroller's own PID are not in the loop. It is for
comparing two trees on the same plant: stop distance, deceleration at the stop, number of brake applications.
Plant: steady response = PCM_SS (median real deceleration per request), 2nd-order lag (PCM_WN, PCM_ZETA, PCM_DELAY),
wheel speed drops out below 0.22 m/s, the car is held at standstill until the request is positive.
"""
import os
import sys
from types import SimpleNamespace as NS

import numpy as np

root = sys.argv[1]
sys.path[:0] = [root, os.path.join(root, 'opendbc_repo')]
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, get_T_FOLLOW  # noqa: E402
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC  # noqa: E402
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop  # noqa: E402
from openpilot.selfdrive.modeld.constants import ModelConstants  # noqa: E402
from openpilot.selfdrive.controls.lib import ars510_long_rules as R  # noqa: E402
from openpilot.selfdrive.controls.lib import longcontrol as LC  # noqa: E402

CT = ModelConstants.T_IDXS[:CONTROL_N]
DT = 0.01
PCM_SS_BP = [-2.0, -0.6, -0.45, -0.38, -0.33, -0.27, -0.22, -0.15, -0.05, 0.0, 0.3, 2.0]
PCM_SS_V = [-2.0, -0.75, -0.40, -0.35, -0.27, -0.26, -0.14, -0.06, 0.06, 0.10, 0.3, 2.0]
PCM_WN, PCM_ZETA, PCM_DELAY = float(os.environ.get('PCM_WN', 3.5)), float(os.environ.get('PCM_ZETA', 0.6)), 0.1
NOLEAD = NS(present=False, radar=False, dRel=0., vLead=0., vRel=0., aLeadK=0., aLeadTau=1.5, modelProb=0., radarTrackId=-1, yRel=0.)


def run(d0, v0=0., T=30., seed=0, trace=False):
  rng = np.random.default_rng(seed)
  mpc = LongitudinalMpc()
  rules = [R.NearStop()] + ([R.Landing()] if hasattr(R, 'Landing') else []) + [R.PosAccelCeiling()]
  CP = NS(stopAccel=-2.0, longitudinalTuning=NS(kiBP=[0.], kiV=[0.]))
  lc = LC.LongControl(CP, NS(enableGasInterceptor=False))
  x = 0.; v = v0; a = 0.; da = 0.; v_f = v0; a_f = 0.; vf_plan = v0
  a_out = 0.; ss = False; cmd = 0.
  q = [0.] * int(PCM_DELAY / DT)
  t_follow = get_T_FOLLOW()
  hist = []
  for k in range(int(T / DT)):
    t = k * DT
    d = d0 - x
    vraw = max(v + rng.normal(0, 0.005), 0.) if v >= 0.22 else 0.
    v_prev = v_f
    v_f += (vraw - v_f) * 0.3
    if vraw == 0. and v_f < 0.03:
      v_f = 0.
    a_f += ((v_f - v_prev) / DT - a_f) * 0.1
    standstill = vraw < 1e-3
    if k % 5 == 0:
      lead = NS(present=True, radar=True, dRel=d, vLead=0., vRel=-v, aLeadK=0., aLeadTau=1.5, modelProb=1., radarTrackId=1, yRel=0.)
      rs = NS(leadOne=lead, leadTwo=NOLEAD)
      vf_plan = max(0., vf_plan + 0.05 / (2.0 + 0.05) * (v_f - vf_plan))
      mpc.set_weights(not standstill)
      mpc.set_cur_state(vf_plan, a_out)
      mpc.update(rs)
      vt = np.interp(CT, T_IDXS_MPC, mpc.v_solution); at = np.interp(CT, T_IDXS_MPC, mpc.a_solution)
      a_mpc = float(get_accel_from_plan(vt, at, CT, action_t=0.2))
      ss = should_stop(v_f, a_mpc)
      a_prev = a_out
      c = R.Ctx(sm={'radarState': rs}, long=True, v_ego=v_f, v_cruise=30 / 3.6, reset_state=False, a_prev=a_prev,
                t_follow=t_follow, now=t, fcw=False, mpc=mpc, dt=0.05, rain=None)
      a_new = a_mpc
      for r in rules:
        a2 = r.apply(c, a_new)
        a_new = a2 if (r.softens or a2 <= a_new) else a_new
      a_out = float(np.clip(a_new, -3.5, 2.0))
      vf_plan = vf_plan + 0.05 * (a_out + a_prev) / 2.
    CS = NS(vEgo=v_f, vEgoRaw=vraw, aEgo=a_f, standstill=standstill, brakePressed=False, gasPressed=False,
            cruiseState=NS(standstill=False))
    cmd = float(lc.update(True, CS, a_out, ss, (-3.5, 2.0), pose_accel=a + rng.normal(0, 0.02)))
    q.append(cmd); u = float(np.interp(q.pop(0), PCM_SS_BP, PCM_SS_V))
    if v <= 0. and u <= 0.05:
      a = 0.; da = 0.
    else:
      dda = PCM_WN ** 2 * (u - a) - 2 * PCM_ZETA * PCM_WN * da
      da += dda * DT; a += da * DT
      v = max(v + a * DT, 0.)
      if v == 0.:
        a = min(a, 0.)
    x += v * DT
    hist.append((t, v, a, d, a_out, cmd, int(lc.stop_phase) if hasattr(lc, 'stop_phase') else 0))
  H = np.array(hist)
  mv = np.where(H[:, 1] > 0.)[0]
  if not len(mv):
    return dict(d_stop=d0, a_stop=0., t_stop=0., pulses=0, vmax=0., amin=0., restarts=0)
  i = mv[-1]
  # felt deceleration: 80 ms EMA of the true acceleration
  af = np.zeros(len(H))
  for j in range(1, len(H)):
    af[j] = af[j - 1] + (H[j, 2] - af[j - 1]) * DT / 0.09
  mov = H[:, 1] > 0.05
  on = (af < -0.25) & mov; off = (af > -0.08) & mov
  pulses = 0; state = False
  for j in range(len(H)):
    if not state and on[j]:
      state = True; pulses += 1
    elif state and off[j]:
      state = False
  starts = int(((H[1:, 1] > 0.) & (H[:-1, 1] == 0.)).sum())
  if trace:
    for j in range(0, i + 60, 20):
      print(f"   t={H[j,0]:5.1f} v={H[j,1]*3.6:4.1f} d={H[j,3]:5.2f} plan={H[j,4]:+.2f} cmd={H[j,5]:+.2f} a={af[j]:+.2f} ph={int(H[j,6])}")
  return dict(d_stop=float(H[i, 3]), a_stop=float(af[max(i - 1, 0)]), t_stop=float(H[i, 0]), pulses=pulses,
              vmax=float(H[:, 1].max() * 3.6), amin=float(af[mov].min()) if mov.any() else 0., restarts=starts - 1)


if __name__ == '__main__':
  trace = int(sys.argv[sys.argv.index('--trace') + 1]) if '--trace' in sys.argv else -1
  print('scenario                d_stop  a_stop  amin  vmax  t_stop  brake applications  restarts')
  rows = []
  cases = [('standstill %4.1f m' % d, d, 0.) for d in (6.5, 7.5, 8.5, 9.5, 11., 13., 16.)] + \
          [('rolling %d km/h %4.1f m' % (v, d), d, v / 3.6) for v, d in ((5, 9.), (5, 12.), (8, 12.), (8, 16.), (12, 18.))]
  for n, (name, d, v0) in enumerate(cases):
    for seed in range(3):
      r = run(d, v0, seed=seed, trace=(n == trace and seed == 0))
      rows.append(r)
      print(f"{name:22s} {r['d_stop']:6.2f} {r['a_stop']:+6.2f} {r['amin']:+6.2f} {r['vmax']:5.1f} {r['t_stop']:6.1f} {r['pulses']:6d} {r['restarts']:12d}")
  A = lambda k: np.array([r[k] for r in rows])
  print(f"summary: d_stop median {np.median(A('d_stop')):.2f} (min {A('d_stop').min():.2f} max {A('d_stop').max():.2f})  "
        f"a_stop median {np.median(A('a_stop')):+.2f} worst {A('a_stop').min():+.2f}  applications mean {A('pulses').mean():.1f}  restarts {int(A('restarts').sum())}")
