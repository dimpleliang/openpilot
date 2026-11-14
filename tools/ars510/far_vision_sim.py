import sys, json, types, importlib.util
sys.path.insert(0, '/data/openpilot')
import numpy as np
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS as T_IDXS_MPC, get_T_FOLLOW
spec = importlib.util.spec_from_file_location('lp_sim', sys.argv[1]); LP = importlib.util.module_from_spec(spec); spec.loader.exec_module(LP)
S = json.load(open(sys.argv[2])); R = S['rows']; OBJ = S['obj']
PERS = getattr(log.LongitudinalPersonality, S['pers'])
TF = get_T_FOLLOW(PERS); CT = ModelConstants.T_IDXS[:CONTROL_N]
def run(rule, truth=False, cruise=True):
  mpc = LongitudinalMpc(dt=DT_MDL); fv = LP.FarVision(); fv.off = not rule
  x, v, a = 0., R[0]['v'], R[0]['a']; out_a = a; res = []; on_t = None
  for i in range(int(12. / DT_MDL)):
    t = i * DT_MDL; k = min(i, len(R) - 1); r = R[k]; true_d = OBJ - x
    rs = log.RadarState.new_message(); l = rs.leadOne
    if i >= len(R) - 1 or r['r'] or truth:   # radar lead (as logged from 14:43:40.5) or the true lead
      l.present = True; l.radar = bool(r['r']) or i >= len(R) - 1; l.dRel = true_d; l.vLead = 0.; l.vLeadK = 0.; l.aLeadK = 0.; l.aLeadTau = 1.5; l.modelProb = 0.9
    elif r['p']:
      l.present = True; l.radar = False; l.dRel = max(true_d + (r['d'] - (OBJ - r['x'])), 2.); l.vLead = r['vl']; l.vLeadK = r['vl']; l.aLeadK = r['al']; l.aLeadTau = r['tau']; l.modelProb = r['pr']
    l.vRel = l.vLead - v
    mpc.set_weights(True, personality=PERS); mpc.set_cur_state(v, out_a); mpc.update(rs, personality=PERS)
    vt = np.interp(CT, T_IDXS_MPC, mpc.v_solution); at = np.interp(CT, T_IDXS_MPC, mpc.a_solution)
    a_cmd = get_accel_from_plan(vt, at, CT, action_t=0.15 + DT_MDL)
    a_cmd = a_cmd[0] if isinstance(a_cmd, tuple) else a_cmd
    if cruise: a_cmd = min(a_cmd, float(np.clip(0.5 * (50. / 3.6 - v), -0.75, 0.3)))
    c = LP.Ctx(sm={'radarState': rs}, long=True, v_ego=v, reset_state=False, a_prev=out_a, t_follow=TF, dt=DT_MDL)
    a2 = fv.apply(c, a_cmd)
    if a2 < a_cmd - 1e-6 and on_t is None: on_t = t
    out_a = float(a2)
    a += (out_a - a) * DT_MDL / 0.3
    v = max(v + a * DT_MDL, 0.); x += v * DT_MDL
    res.append((t, v, out_a, true_d, l.dRel if l.present else -1, l.vLead if l.present else -1, a2 < a_cmd - 1e-6))
    if v < 0.1: break
  return res, on_t
for name, kw in (('base', dict(rule=False)), ('far_vision', dict(rule=True)), ('truth(standing lead from start)', dict(rule=False, truth=True))):
  res, on_t = run(**kw); A = np.array(res, dtype=float)
  pk = A[:, 2].min(); tp = A[A[:, 2].argmin(), 0]
  print('==', name, 'rule on at', on_t, '| peak a %.2f at t=%.1f | stop gap %.1f m | min gap %.1f' % (pk, tp, A[-1, 3], A[:, 3].min()), '| v@2.8s %.1f km/h v@4s %.1f' % (np.interp(2.8, A[:, 0], A[:, 1]) * 3.6, np.interp(4., A[:, 0], A[:, 1]) * 3.6))
  print('  ', ' '.join('t%.1f v%.0f a%.2f d%.0f%s' % (q[0], q[1] * 3.6, q[2], q[3], '*' if q[6] else '') for q in res[::10]))
