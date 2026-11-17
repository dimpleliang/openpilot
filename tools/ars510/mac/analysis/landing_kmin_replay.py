import sys, os, types, json, zstandard, numpy as np, datetime, importlib.util
ROOT = os.path.expanduser('~/ars510_rlogs'); sys.path[:0] = [ROOT + '/op', ROOT + '/op/opendbc_repo']
from openpilot.cereal import log
for name in ['openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc', 'openpilot.selfdrive.controls.lib.ars510_ego_lc', 'openpilot.selfdrive.controls.lib.ars510_cutin',
             'openpilot.selfdrive.controls.lib.ars510_lead_stop', 'openpilot.selfdrive.controls.lib.ars510_rain', 'openpilot.common.swaglog']:
  m = types.ModuleType(name); sys.modules[name] = m
sys.modules['openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc'].LongitudinalPlanSource = types.SimpleNamespace(cruise=0, lead0=1, lead1=2, lead2=3, e2e=4)
sys.modules['openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc'].STOP_DISTANCE = 6.0
sys.modules['openpilot.selfdrive.controls.lib.ars510_ego_lc'].EgoLaneChange = object
sys.modules['openpilot.selfdrive.controls.lib.ars510_lead_stop'].GAP = 4.5
sys.modules['openpilot.common.swaglog'].cloudlog = types.SimpleNamespace(info=lambda *a, **k: None, warning=lambda *a, **k: None)
pkg = sys.modules.setdefault('openpilot.selfdrive.controls.lib', types.ModuleType('openpilot.selfdrive.controls.lib'))
for n in ('ars510_cutin', 'ars510_lead_stop', 'ars510_rain'): setattr(pkg, n, sys.modules['openpilot.selfdrive.controls.lib.' + n])
spec = importlib.util.spec_from_file_location('LR', os.path.expanduser('~/ars510_rlogs/review/openpilot/openpilot/selfdrive/controls/lib/ars510_long_rules.py'))
LR = importlib.util.module_from_spec(spec); spec.loader.exec_module(LR)
tz = datetime.timezone(datetime.timedelta(hours=8))
S = [s for s in json.load(open('stops_wk.json')) if s['kind'] == 'sys']
for s in sorted(S, key=lambda s: s['t']):
  r, sg = s['seg'].rsplit('--', 1); ev = []
  for k in (int(sg) - 1, int(sg)):
    p = ROOT + '/raw/%s--%d/rlog.zst' % (r, k)
    if os.path.exists(p): ev += list(log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read()), traversal_limit_in_words=2**61))
  ev.sort(key=lambda m: m.logMonoTime)
  off = [m.clocks.wallTimeNanos / 1e9 - m.logMonoTime / 1e9 for m in ev if m.which() == 'clocks' and m.clocks.wallTimeNanos > 1.7e18][0]
  res = {}
  for kmin in (0.0, LR.LAND_K_MIN):
    L = LR.Landing(); L.k_min = kmin; L.off = False; v = 0.; rs = None; plan = 0.; aprev = 0.; first = None; outs = []
    for m in ev:
      w = m.which(); t = m.logMonoTime / 1e9 + off - s['t']
      if t < -8 or t > 0.5: continue
      if w == 'carState': v = m.carState.vEgo
      elif w == 'radarState': rs = m.radarState
      elif w == 'carControl': aprev = m.carControl.actuators.accel
      elif w == 'longitudinalPlan' and rs is not None:
        plan = m.longitudinalPlan.aTarget
        c = LR.Ctx(sm={'radarState': rs}, long=True, v_ego=v, reset_state=False, a_prev=aprev, fcw=False, dt=0.05)
        a2 = L.apply(c, plan)
        if L.k is not None:
          if first is None: first = (t, L.k, v * 3.6, rs.leadOne.dRel)
          outs.append((t, a2))
    res[kmin] = (first, outs)
  F = np.array(s['imu']['ftrace']); tt = F[:, 0]; f = F[:, 1]; w_ = (tt >= 0) & (tt <= 1.3); k_ = np.argmax(np.where(w_, f, -9)); jolt = f[k_] - f[(tt >= tt[k_] - 0.5) & (tt < tt[k_] - 0.15)].mean()
  def fmt(x):
    fst, o = x
    if fst is None: return 'no landing'
    return 'on %.1fs before, k %.2f, %.1f km/h, d %.1f m, request %.2f -> %.2f' % (-fst[0], fst[1], fst[2], fst[3], o[0][1], o[-1][1])
  print(datetime.datetime.fromtimestamp(s['t'], tz).strftime('%m/%d %H:%M'), 'jolt %.2f' % jolt, '| old:', fmt(res[0.0]), '| new:', fmt(res[LR.LAND_K_MIN]))
