#!/usr/bin/env python3
"""Open-loop replay of radard on every rlog: stock track filter vs vision lead-speed fusion (radard VISION_FUSION, from
commaai/openpilot#39021). Mac native (~/ars510_rlogs, sparse checkout incl. openpilot/selfdrive/controls, openpilot/common,
opendbc_repo/opendbc/sunnypilot); messaging / params / realtime / swaglog are stubbed.

Per route (segments in order, radard state carried across segments) both variants see the same carState / modelV2 /
radarTracks / customReservedRawData0 stream; leadOne is recorded at every modelV2. Yardstick ("hindsight"): for each
continuous radar lead (same trackId, >= 4 s) the lead speed from the 4 s range slope, v_ego + d(dRel)/dt, and its 2 s slope as
the lead's acceleration. Metrics (radar leads only, both variants on the same track):
  speed error vs hindsight (median / p95 |vLeadK - v_hs|)
  false closing: aLeadK < -1 m/s^2 for >= 0.5 s while the hindsight lead accel > -0.3 (episodes / h)
  real braking: hindsight accel < -1.5 for >= 1 s; delay of aLeadK < -1 after its onset (s)
  ~/ars510_rlogs/venv/bin/python tools/ars510/mac/analysis/radard_fusion_eval.py [route-prefix ...]
"""
import glob, os, sys, types, json
import numpy as np
from collections import defaultdict
from multiprocessing import Pool

ROOT = os.path.expanduser('~/ars510_rlogs')
OP = os.path.join(ROOT, 'op')
sys.path[:0] = [OP, os.path.join(OP, 'opendbc_repo')]

def _stubs():
  m = types.ModuleType("openpilot.cereal.messaging"); m.SubMaster = m.PubMaster = object
  sys.modules["openpilot.cereal.messaging"] = m
  import openpilot.cereal as oc; oc.messaging = m
  p = types.ModuleType("openpilot.common.params"); p.Params = object; sys.modules["openpilot.common.params"] = p
  r = types.ModuleType("openpilot.common.realtime"); r.DT_MDL = 0.05; r.Priority = types.SimpleNamespace(CTRL_LOW=0)
  r.config_realtime_process = lambda *a, **k: None; sys.modules["openpilot.common.realtime"] = r
  s = types.ModuleType("openpilot.common.swaglog"); s.cloudlog = types.SimpleNamespace(info=lambda *a: None, warning=lambda *a: None)
  sys.modules["openpilot.common.swaglog"] = s
_stubs()
import zstandard
from openpilot.cereal import log
from openpilot.selfdrive.controls import radard as R

OUT = os.path.join(ROOT, 'analysis/fusion'); os.makedirs(OUT, exist_ok=True)
SERVICES = ('carState', 'modelV2', 'radarTracks', 'customReservedRawData0')


class FakeSM:
  def __init__(self):
    self.data, self.seen, self.recv_frame, self.logMonoTime, self.valid = {}, defaultdict(bool), defaultdict(int), defaultdict(int), defaultdict(bool)
    self.frame = 0
  def put(self, w, msg, t, valid):
    self.frame += 1
    self.data[w] = getattr(msg, w); self.seen[w] = True; self.recv_frame[w] = self.frame; self.logMonoTime[w] = t; self.valid[w] = valid
  def __getitem__(self, k): return self.data[k]
  def all_checks(self): return True


def events(seg):
  p = seg + '/rlog.zst'
  if not os.path.exists(p): return
  try: raw = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  except Exception: return
  try:
    for m in log.Event.read_multiple_bytes(raw, traversal_limit_in_words=2**61):
      yield m
  except Exception:
    return


def run_route(segs):
  sm = FakeSM(); rds = None; CP = None
  rows = []     # t, vEgo, [present, radar, trackId, dRel, vLeadK, aLeadK] x 2, logged vLeadK
  logged = {}
  for seg in segs:
    for m in events(seg):
      w = m.which()
      if w == 'carParams' and CP is None:
        CP = m.carParams
        rds = []
        for fuse in (False, True):
          rd = R.RadarD(CP, None, CP.radarDelay); rd.fuse_vision = fuse; rds.append(rd)
      elif w == 'radarState':
        lo = m.radarState.leadOne
        logged[m.radarState.mdMonoTime] = lo.vLeadK if lo.present else np.nan
      elif w in SERVICES:
        sm.put(w, m, m.logMonoTime, m.valid)
        if w == 'modelV2' and rds and sm.seen['carState'] and sm.seen['radarTracks']:
          row = [m.logMonoTime / 1e9, sm['carState'].vEgo]
          for rd in rds:
            rd.update(sm, sm['radarTracks'])
            lo = rd.radar_state.leadOne
            row += [lo.present, lo.radar, lo.radarTrackId, lo.dRel, lo.vLeadK, lo.aLeadK]
          row.append(m.logMonoTime)
          rows.append(row)
  if not rows: return None
  a = np.array(rows, dtype=float)
  lv = np.array([logged.get(int(x), np.nan) for x in a[:, -1]])
  return a[:, :-1], lv


def slope(t, y, i0, i1):
  tt = t[i0:i1] - t[i0:i1].mean()
  return float((tt * (y[i0:i1] - y[i0:i1].mean())).sum() / max((tt * tt).sum(), 1e-9))


def metrics(a):
  t, v = a[:, 0], a[:, 1]
  A = dict(p=a[:, 2], r=a[:, 3], id=a[:, 4], d=a[:, 5], vl=a[:, 6], al=a[:, 7])
  B = dict(p=a[:, 8], r=a[:, 9], id=a[:, 10], d=a[:, 11], vl=a[:, 12], al=a[:, 13])
  ok = (A['p'] > 0) & (A['r'] > 0) & (B['p'] > 0) & (B['r'] > 0) & (A['id'] == B['id'])
  vhs = np.full(len(t), np.nan); ahs = np.full(len(t), np.nan)
  # continuous runs of the same radar lead
  i = 0; n = len(t)
  while i < n:
    if not ok[i]: i += 1; continue
    j = i
    while j + 1 < n and ok[j + 1] and A['id'][j + 1] == A['id'][i] and t[j + 1] - t[j] < 0.2: j += 1
    if t[j] - t[i] >= 4.0:
      for k in range(i, j + 1):
        lo = np.searchsorted(t, t[k] - 2.0); hi = np.searchsorted(t, t[k] + 2.0)
        if lo >= i and hi <= j + 1 and t[hi - 1] - t[lo] > 3.5:
          vhs[k] = v[k] + slope(t, A['d'], lo, hi)
      for k in range(i, j + 1):
        lo = np.searchsorted(t, t[k] - 1.0); hi = np.searchsorted(t, t[k] + 1.0)
        seg = vhs[lo:hi]
        if lo >= i and hi <= j + 1 and np.isfinite(seg).sum() > 25:
          g = np.isfinite(seg); ahs[k] = slope(t[lo:hi][g], seg[g], 0, g.sum())
    i = j + 1
  hs = np.isfinite(vhs) & np.isfinite(ahs)
  res = dict(hours=float(np.sum(np.diff(t) < 0.2) * 0.05 / 3600), radar_lead_h=float(ok.sum() * 0.05 / 3600),
             scored_h=float(hs.sum() * 0.05 / 3600))
  for name, X in (('stock', A), ('fusion', B)):
    e = np.abs(X['vl'][hs] - vhs[hs])
    res[f'{name}_err_med'] = float(np.median(e)) if e.size else np.nan
    res[f'{name}_err_p95'] = float(np.percentile(e, 95)) if e.size else np.nan
    fc = hs & (X['al'] < -1.0) & (ahs > -0.3)
    res[f'{name}_false_close_eps'] = int(episodes(t, fc, 0.5))
    res[f'{name}_false_close_s'] = float(fc.sum() * 0.05)
  # real braking onsets: hindsight accel < -1.5 for >= 1 s
  rb = hs & (ahs < -1.5)
  delays = []
  for s0, s1 in runs(t, rb, 1.0):
    w = (t >= t[s0] - 1.0) & (t <= t[s1])
    da = [t[np.argmax(w & (X['al'] < -1.0))] - t[s0] if np.any(w & (X['al'] < -1.0)) else np.nan for X in (A, B)]
    delays.append(da)
  res['brake_events'] = len(delays)
  d = np.array(delays) if delays else np.zeros((0, 2))
  res['brake_delay_stock_med'] = float(np.nanmedian(d[:, 0])) if len(d) else np.nan
  res['brake_delay_fusion_med'] = float(np.nanmedian(d[:, 1])) if len(d) else np.nan
  res['brake_delay_diff_med'] = float(np.nanmedian(d[:, 1] - d[:, 0])) if len(d) else np.nan
  res['brake_missed_stock'] = int(np.isnan(d[:, 0]).sum()) if len(d) else 0
  res['brake_missed_fusion'] = int(np.isnan(d[:, 1]).sum()) if len(d) else 0
  return res, d


def runs(t, mask, min_s):
  out = []; i = 0; n = len(mask)
  while i < n:
    if not mask[i]: i += 1; continue
    j = i
    while j + 1 < n and mask[j + 1]: j += 1
    if t[j] - t[i] >= min_s: out.append((i, j))
    i = j + 1
  return out


def episodes(t, mask, min_s):
  return len(runs(t, mask, min_s))


def route_job(prefix):
  segs = sorted(glob.glob(os.path.join(ROOT, 'raw', prefix + '--*')), key=lambda s: int(s.rsplit('--', 1)[1]))
  r = run_route(segs)
  if r is None: return prefix, None
  a, lv = r
  m, d = metrics(a)
  g = np.isfinite(lv) & (a[:, 2] > 0)
  m['replay_vs_logged_max'] = float(np.nanmax(np.abs(a[g, 6] - lv[g]))) if g.any() else np.nan
  m['replay_vs_logged_p99'] = float(np.percentile(np.abs(a[g, 6] - lv[g]), 99)) if g.any() else np.nan
  return prefix, m


if __name__ == '__main__':
  prefixes = sys.argv[1:] or sorted({os.path.basename(s).rsplit('--', 1)[0] for s in glob.glob(os.path.join(ROOT, 'raw', '*--*'))})
  out = {}
  with Pool(6) as pool:
    for p, m in pool.imap_unordered(route_job, prefixes):
      if m: out[p] = m; print(p, json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()}), flush=True)
  json.dump(out, open(os.path.join(OUT, 'fusion_routes.json'), 'w'), indent=1)
  keys = ['hours', 'radar_lead_h', 'scored_h', 'stock_false_close_eps', 'fusion_false_close_eps', 'stock_false_close_s',
          'fusion_false_close_s', 'brake_events', 'brake_missed_stock', 'brake_missed_fusion']
  tot = {k: sum(m[k] for m in out.values()) for k in keys}
  w = np.array([m['scored_h'] for m in out.values()])
  for k in ('stock_err_med', 'fusion_err_med', 'stock_err_p95', 'fusion_err_p95', 'brake_delay_stock_med', 'brake_delay_fusion_med',
            'brake_delay_diff_med', 'replay_vs_logged_p99', 'replay_vs_logged_max'):
    x = np.array([m[k] for m in out.values()], dtype=float); g = np.isfinite(x)
    tot[k + ('(max)' if k.endswith('max') else '(scored-h weighted)')] = float(np.max(x[g]) if k.endswith('max') else np.average(x[g], weights=np.maximum(w[g], 1e-6))) if g.any() else None
  sh = max(tot['scored_h'], 1e-9)
  tot['stock_false_close_per_h'] = tot['stock_false_close_eps'] / sh
  tot['fusion_false_close_per_h'] = tot['fusion_false_close_eps'] / sh
  txt = json.dumps(tot, indent=1, ensure_ascii=False)
  print(txt); open(os.path.join(OUT, 'fusion_summary.json'), 'w').write(txt)
