"""What SCC-V (sunnypilot Smart Cruise Control - Vision) would have done on a logged curve where it was off (Mac, offline).
  python3 sccv_sim.py <dir> <seg> [seg ...] --start HH:MM:SS [--from HH:MM:SS --to HH:MM:SS]
Re-implements vision_controller.py's state machine (same thresholds) on the logged modelV2 / controlsState, and integrates a
simulated speed: a = min(logged aEgo, SCC-V pull toward its target). The model's predicted lateral accelerations and the current
one are rescaled to the simulated speed ((v_sim / v_log)^2: same path curvature). Assumptions: planner tracks the SCC-V speed
target with a = clip(0.6 (v_target - v), -1.5, +1.0); the logged decelerations (lead, driver) still happen.
"""
import os, sys, zstandard, numpy as np

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log

ENTER, ABORT, TURN, LEAVE, FINISH = 1.3, 1.1, 1.6, 1.3, 1.1
A_LAT_MAX, HORIZON = 2.0, 4.0
ENTER_BP, ENTER_V = [1.3, 3.], [-0.2, -1.]
TURN_BP, TURN_V = [1.5, 2.3, 3.], [0.5, 0., -0.4]
LEAVE_A, MIN_V = 0.5, 20 / 3.6


def hms(s):
  h, m, x = (int(v) for v in s.split(':')); return h * 3600 + m * 60 + x


def main():
  a = sys.argv[1:]
  w0 = hms(a[a.index('--start') + 1]); f = hms(a[a.index('--from') + 1]) if '--from' in a else 0; to = hms(a[a.index('--to') + 1]) if '--to' in a else 1e9
  d0, segs = a[0], [x for x in a[1:] if x.isdigit()]
  rows = []; st = dict(v=0., a=0., curv=0., rate=None, vel=None, vc=0.); t0 = None
  for sg in segs:
    dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(os.path.join(d0, sg, 'rlog.zst'), 'rb').read())
    for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
      w = e.which(); t = e.logMonoTime / 1e9
      if w == 'carState': st.update(v=e.carState.vEgo, a=e.carState.aEgo, vc=e.carState.vCruise / 3.6)
      elif w == 'controlsState':
        st['curv'] = e.controlsState.curvature
        if t0 is None: t0 = t
      elif w == 'modelV2' and t0 is not None:
        m = e.modelV2
        rows.append((w0 + t - t0, st['v'], st['a'], st['curv'], np.abs(np.array(m.orientationRate.z)), np.array(m.velocity.x), st['vc']))
  state = 'enabled'; v_sim = None; last_print = -1
  print('clock     v_log v_sim | pred_lat(log→sim) cur_lat(log→sim) | state     vTgt  a_scc | a_log a_used')
  summary = []
  for i, (tw, v, al, curv, rate, vel, vc) in enumerate(rows):
    if tw < f or tw > to: continue
    dt = 0.05 if i == 0 else rows[i][0] - rows[i - 1][0]
    if v_sim is None: v_sim = v
    k = (v_sim / max(v, 0.1)) ** 2
    pred_log = float(np.percentile(rate * vel, 97)); pred = pred_log * k
    cur_log = v * v * abs(curv); cur = cur_log * k
    if state == 'enabled':
      if v_sim > MIN_V and pred >= ENTER: state = 'entering'
    elif state == 'entering':
      if cur >= TURN: state = 'turning'
      elif pred < ABORT: state = 'enabled'
    elif state == 'turning':
      if cur <= LEAVE: state = 'leaving'
    elif state == 'leaving':
      if cur >= TURN: state = 'turning'
      elif cur < FINISH: state = 'enabled'
    max_curve = max(pred, 1e-3) / max(v_sim, 0.1) ** 2
    v_t = (A_LAT_MAX / max_curve) ** 0.5
    if state == 'entering': a_t = float(np.interp(pred, ENTER_BP, ENTER_V))
    elif state == 'turning': a_t = float(np.interp(cur, TURN_BP, TURN_V))
    elif state == 'leaving': a_t = LEAVE_A
    else: a_t = al
    active = state in ('entering', 'turning', 'leaving')
    v_out = max(v_t, MIN_V) + a_t * HORIZON if active else 1e9
    a_scc = float(np.clip(0.6 * (min(v_out, vc) - v_sim), -1.5, 1.0)) if active else 9.
    a_used = min(al, a_scc)
    v_sim = max(0., v_sim + a_used * dt)
    summary.append((tw, v, v_sim, cur_log, cur))
    if tw - last_print >= 0.5:
      last_print = tw
      print(f'{int(tw//3600)%24:02d}:{int(tw//60)%60:02d}:{int(tw%60):02d} {v*3.6:5.0f} {v_sim*3.6:5.0f} | {pred_log:4.2f}→{pred:4.2f}      {cur_log:4.2f}→{cur:4.2f}      | '
            f'{state:9s} {min(v_out, 99)*3.6:4.0f} {a_scc if active else float("nan"):5.2f} | {al:5.2f} {a_used:5.2f}')
  if summary:
    s = np.array(summary); j = int(np.argmax(s[:, 3]))
    print(f'\npeak curve: logged {s[j,1]*3.6:.0f} km/h lat {s[j,3]:.2f} m/s^2  ->  with SCC-V {s[j,2]*3.6:.0f} km/h lat {s[j,4]:.2f} m/s^2')


if __name__ == '__main__':
  main()
