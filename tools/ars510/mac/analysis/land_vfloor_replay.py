"""Open-loop replay of the landing rule (ars510_long_rules.Landing) on rlogs: LAND_V_FLOOR 0 (before 11/22) vs the new
value. Input: logged radarState lead, carState vEgo, and the logged plan aTarget as the plan seen by the rule (it
already contains the old landing where that was active, a lighter plan than the new one, so the "plan firmer" exit is
not triggered spuriously). Per engaged stop with a landing: how long the request sat at the floor before the wheel
speed dropped out, the peak request, and the open-loop stopping-distance change (double integral of the difference).
usage: venv/bin/python land_replay.py <repo root> <out.json> rlog..."""
import sys, os, types, json, importlib.util
import numpy as np, zstandard
from multiprocessing import Pool
ROOT = sys.argv[1]


def load_rules():
  sys.path[:0] = [ROOT, os.path.join(ROOT, 'opendbc_repo')]
  for name in ['openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc', 'openpilot.selfdrive.controls.lib.ars510_ego_lc',
               'openpilot.selfdrive.controls.lib.ars510_cutin', 'openpilot.selfdrive.controls.lib.ars510_lead_stop',
               'openpilot.selfdrive.controls.lib.ars510_rain', 'openpilot.common.swaglog']:
    sys.modules[name] = types.ModuleType(name)
  mpc = sys.modules['openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc']
  mpc.LongitudinalPlanSource = types.SimpleNamespace(cruise=0, lead0=1, lead1=2, lead2=3, e2e=4); mpc.STOP_DISTANCE = 6.0
  sys.modules['openpilot.selfdrive.controls.lib.ars510_ego_lc'].EgoLaneChange = object
  sys.modules['openpilot.selfdrive.controls.lib.ars510_lead_stop'].GAP = 4.5
  sys.modules['openpilot.common.swaglog'].cloudlog = types.SimpleNamespace(info=lambda *a, **k: None, warning=lambda *a, **k: None)
  pkg = sys.modules.setdefault('openpilot.selfdrive.controls.lib', types.ModuleType('openpilot.selfdrive.controls.lib'))
  for n in ('ars510_cutin', 'ars510_lead_stop', 'ars510_rain'):
    setattr(pkg, n, sys.modules['openpilot.selfdrive.controls.lib.' + n])
  spec = importlib.util.spec_from_file_location('LR', os.path.join(ROOT, 'openpilot/selfdrive/controls/lib/ars510_long_rules.py'))
  LR = importlib.util.module_from_spec(spec); spec.loader.exec_module(LR)
  return LR


def run(path):
  try:
    LR = load_rules()
    from openpilot.cereal import log
    d = zstandard.ZstdDecompressor().decompressobj().decompress(open(path, 'rb').read())
    old, new = LR.Landing(), LR.Landing()
    old.v_floor, new.v_floor = 0.0, LR.LAND_V_FLOOR
    old.k_min = new.k_min = 0.0; old.off = new.off = False
    rs = None; v = 0.; en = False; prev = {id(old): 0., id(new): 0.}; rows = []; last_t = None
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      w = m.which(); t = m.logMonoTime / 1e9
      if w == 'radarState': rs = m.radarState
      elif w == 'carState': v = m.carState.vEgo; brk = m.carState.brakePressed or m.carState.gasPressed
      elif w == 'carControl': en = m.carControl.longActive
      elif w == 'longitudinalPlan' and rs is not None:
        a = float(m.longitudinalPlan.aTarget)
        dt = 0.05 if last_t is None else min(max(t - last_t, 0.01), 0.2); last_t = t
        outs = []
        for lr in (old, new):
          c = types.SimpleNamespace(sm={'radarState': rs}, long=en, v_ego=v, reset_state=False, a_prev=prev[id(lr)], fcw=False, dt=dt)
          o = float(lr.apply(c, a)); prev[id(lr)] = o; outs.append(o)
        rows.append((t, v, float(en), a, outs[0], outs[1], float(old.k is not None), float(new.k is not None)))
    R = np.array(rows); out = []
    for i in range(100, len(R)):
      if R[i, 1] < 0.05 and R[i - 1, 1] >= 0.05 and R[i - 100:i, 2].min() > 0:
        s = R[max(i - 160, 0):i + 1]
        if not (s[:, 6].any() or s[:, 7].any()):
          continue
        res = dict(seg=os.path.basename(os.path.dirname(path)), t=float(R[i, 0]))
        for k, nm in ((4, 'old'), (5, 'new')):
          firm = np.where(s[:, k] < -0.30)[0]
          res['held_' + nm] = float(s[-1, 0] - s[firm[-1], 0]) if len(firm) else 8.0
          res['peak_' + nm] = float(s[:, k].min())
        dtt = np.diff(s[:, 0], prepend=s[0, 0])
        dv = np.cumsum((s[:, 5] - s[:, 4]) * dtt)
        res['dd'] = float(np.sum(dv * dtt))
        out.append(res)
    return out
  except Exception as e:
    return [dict(err=repr(e), seg=path)]


if __name__ == '__main__':
  with Pool(8) as p:
    res = p.map(run, sys.argv[3:])
  S = [s for r in res for s in r]
  E = [s for s in S if 'err' in s]; S = [s for s in S if 'err' not in s]
  json.dump(S, open(sys.argv[2], 'w'))
  print('landing stops', len(S), 'errors', len(E), E[:1])
  if S:
    ho = np.array([s['held_old'] for s in S]); hn = np.array([s['held_new'] for s in S])
    for nm, h in (('old', ho), ('new', hn)):
      print('%s: floor held before wheel stop median %.2f s (p25 %.2f p75 %.2f); < 0.4 s %d, 0.4-1.0 s %d, > 1.0 s %d' % (
        nm, np.median(h), np.percentile(h, 25), np.percentile(h, 75), (h < 0.4).sum(), ((h >= 0.4) & (h <= 1.0)).sum(), (h > 1.0).sum()))
    pk = np.array([[s['peak_old'], s['peak_new']] for s in S]); dd = np.array([s['dd'] for s in S])
    print('peak request old median %.2f new median %.2f, largest firmer by %.2f' % (np.median(pk[:, 0]), np.median(pk[:, 1]), (pk[:, 0] - pk[:, 1]).max()))
    print('open-loop stopping distance change (m, - = stops earlier): median %+.2f, min %+.2f, max %+.2f' % (np.median(dd), dd.min(), dd.max()))
