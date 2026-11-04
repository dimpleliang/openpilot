"""Replay the UI's merge / cut-in marks (ui/sunnypilot/onroad/ars510_tracks.py, the real class, pyray stubbed) on rlogs and score
them (Mac, offline).

  python3 merge_replay.py [--routes 33-4d] [--max-segs N] [--set NAME=VALUE ...] [--list] [--jobs 6]

--set overrides a module constant before the run (e.g. --set TREND_REF_LANE=True --set CUTIN_WIDE_D="(25.,110.)").
Ground truth per track (same path / camera offset as the UI): a REAL cut-in is a track that was >= CUT_FROM_M from my path and
then within IN_LANE_M of it (or became the radar lead) while I drove > MIN_KMH. A mark episode (contiguous marked frames of one
track, gaps < 1 s merged) is TRUE when that track gets in my lane (same test) within HORIZON_S after the episode starts.
Reports episodes, true / false, false per 100 km, recall of real cut-ins (marked within HORIZON_S before entering), median warning.
"""
import glob, importlib.util, os, sys, types, zstandard, numpy as np
from collections import defaultdict
from multiprocessing import Pool

ROOT = os.path.expanduser('~/ars510_rlogs')
OP = os.path.join(ROOT, 'op')
sys.path[:0] = [OP, os.path.join(OP, 'opendbc_repo')]

HORIZON_S, IN_LANE_M, CUT_FROM_M, MIN_KMH = 8.0, 0.9, 1.8, 60.0
HUG_LINE_M = 0.5   # a false episode whose object came within this of the lane line = a line-hugger (driver 11/04: a hazard too)
SIM = {'t': 0.0}


def _stubs():
  rl = types.ModuleType('pyray')

  class V2:
    def __init__(self, x=0., y=0.): self.x, self.y = x, y

  class Rect:
    def __init__(self, x=0., y=0., w=0., h=0.): self.x, self.y, self.width, self.height = x, y, w, h
  rl.Vector2 = V2; rl.Vector3 = V2; rl.Rectangle = Rect
  rl.Color = lambda *a: a
  rl.get_time = lambda: SIM['t']
  for n in ('draw_circle', 'draw_line_ex', 'draw_rectangle_rounded', 'draw_text_ex', 'draw_triangle', 'draw_circle_v',
            'draw_ring', 'draw_poly'):
    setattr(rl, n, lambda *a, **k: None)
  sys.modules['pyray'] = rl
  ui = types.ModuleType('openpilot.selfdrive.ui.ui_state'); ui.ui_state = types.SimpleNamespace(is_metric=True)
  sys.modules['openpilot.selfdrive.ui.ui_state'] = ui
  app = types.ModuleType('openpilot.system.ui.lib.application')
  app.gui_app = types.SimpleNamespace(font=lambda *a: None); app.FontWeight = types.SimpleNamespace(SEMI_BOLD=0, BOLD=0, NORMAL=0)
  sys.modules['openpilot.system.ui.lib.application'] = app
  tm = types.ModuleType('openpilot.system.ui.lib.text_measure'); tm.measure_text_cached = lambda *a, **k: V2(10, 10)
  sys.modules['openpilot.system.ui.lib.text_measure'] = tm
  return rl


RL = _stubs()
from openpilot.cereal import log  # noqa: E402


def load_tracks(sets):
  path = os.path.join(OP, 'openpilot', 'selfdrive', 'ui', 'sunnypilot', 'onroad', 'ars510_tracks.py')
  spec = importlib.util.spec_from_file_location('ars510_tracks_replay', path)
  m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
  for k, v in sets.items():
    setattr(m, k, eval(v))
  m.MARKS_OFF_FILE = '/nonexistent'
  return m


class SM:
  def __init__(self):
    self.data, self.updated, self.recv_frame, self.frame = {}, defaultdict(bool), defaultdict(int), 0

  def __getitem__(self, k): return self.data[k]


def run_seg(args):
  seg, sets = args
  p = os.path.join(seg, 'rlog.zst')
  try:
    dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
    evs = list(log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61))
  except Exception:
    return None
  m = load_tracks(sets)
  tr = m.Ars510Tracks()
  marked_now = []
  orig = tr._draw_confirmed
  tr._draw_confirmed = lambda pt, d, arrow, flash=True: marked_now.append(d)
  sm = SM(); pend = set(); t0 = None
  rect = RL.Rectangle(0, 0, 2160, 1080)
  frames = []  # (t, v_ego, engaged, {tid: (off_signed, d)}, lead (d, y) or None, marked tids)
  km = 0.; last_t = None
  for e in evs:
    w = e.which()
    if w in ('carState', 'modelV2', 'radarState', 'selfdriveState', 'customReservedRawData0', 'controlsState', 'liveCalibration'):
      sm.data[w] = getattr(e, w); pend.add(w)
    if w != 'modelV2' or 'carState' not in sm.data:
      continue
    t = e.logMonoTime / 1e9; SIM['t'] = t
    sm.frame += 1
    for k in list(sm.updated): sm.updated[k] = False
    for k in pend: sm.updated[k] = True; sm.recv_frame[k] = sm.frame
    pend = set()
    md = sm.data['modelV2']
    xs = np.asarray(md.position.x, float); ys = np.asarray(md.position.y, float); zs = np.asarray(md.position.z, float)
    if len(xs) < 2: continue
    path_y = lambda d: float(np.interp(d, xs, ys))
    path_z = lambda d: float(np.interp(d, xs, zs))
    mts = lambda d, y, z: RL.Vector2(100, 100)
    rs = sm.data.get('radarState')
    marked_now.clear()
    tr.draw(sm, mts, path_z, path_y, 0.0, 1.2, rs, rect)
    v = sm.data['carState'].vEgo
    if last_t is not None: km += v * (t - last_t) / 1000.
    last_t = t
    pts = {}
    ll = [(np.asarray(md.laneLines[i].x, float), np.asarray(md.laneLines[i].y, float), float(md.laneLineProbs[i])) for i in (1, 2)] \
      if len(md.laneLines) >= 3 and len(md.laneLineProbs) >= 3 else None
    for d, y, vr, _s, settling, cls, tid, vlat, lc in tr._points:
      if tid >= 0 and 0.5 < d < 120:
        yr = -y; off_s = yr - path_y(d)
        line = float('nan')   # distance from the object to the lane line on its side (negative: inside my lane)
        if ll is not None:
          x, ly, pr = ll[1] if off_s > 0 else ll[0]
          if pr >= 0.5 and len(x) > 1 and d <= x[-1]:
            lyd = float(np.interp(d, x, ly)); line = (yr - lyd) if off_s > 0 else (lyd - yr)
        pts[tid] = (off_s, d, line)
    lead = (rs.leadOne.dRel, rs.leadOne.yRel) if rs is not None and rs.leadOne.present and rs.leadOne.radar else None
    mk = set()
    for dm in marked_now:
      for d, y, vr, _s, settling, cls, tid, vlat, lc in tr._points:
        if d == dm: mk.add(tid)
    eng = 'selfdriveState' in sm.data and sm.data['selfdriveState'].enabled
    frames.append((t, v, eng, pts, lead, mk, {tid: (d, y) for d, y, *_r, tid, _vl, _lc in
                                             [(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8]) for q in tr._points]}))
  return os.path.basename(seg), frames, km


def score(segres):
  eps, real, km = [], [], 0.
  for name, frames, k in segres:
    km += k
    hist = defaultdict(list)  # tid -> [(t, |off|, is_lead, v)]
    for t, v, eng, pts, lead, mk, raw in frames:
      for tid, (off, d, line) in pts.items():
        dy = raw.get(tid)
        is_lead = lead is not None and dy is not None and abs(dy[0] - lead[0]) < 2.0 and abs(dy[1] - lead[1]) < 1.2
        hist[tid].append((t, abs(off), is_lead, v, eng, line))
    entry = {}
    for tid, h in hist.items():
      out_seen = False
      for t, off, il, v, eng, _line in h:
        if off >= CUT_FROM_M: out_seen = True
        elif out_seen and (off < IN_LANE_M or il) and v * 3.6 > MIN_KMH:
          entry[tid] = t; break
    for tid, te in entry.items():
      if any(x[4] for x in hist[tid] if abs(x[0] - te) < 0.5):
        real.append((name, tid, te))
    # mark episodes
    cur = {}
    for t, v, eng, pts, lead, mk, raw in frames:
      for tid in mk:
        if tid in cur and t - cur[tid][1] < 1.0: cur[tid][1] = t
        else:
          if tid in cur: eps.append((name, tid, cur[tid][0], cur[tid][1], cur[tid][2]))
          cur[tid] = [t, t, v]
    for tid, (a, b, v) in cur.items(): eps.append((name, tid, a, b, v))
    for i, ep in enumerate(eps):
      if ep[0] == name and len(ep) == 5:
        te = entry.get(ep[1])
        near = [x[5] for x in hist[ep[1]] if ep[2] - 0.5 <= x[0] <= ep[3] + 0.5 and x[5] == x[5]]
        eps[i] = ep + (te is not None and ep[2] - 1.0 <= te <= ep[2] + HORIZON_S, te, min(near) if near else float('nan'))
  tp = sum(1 for e in eps if e[5]); fp = len(eps) - tp
  caught, warn = 0, []
  for name, tid, te in real:
    ms = [e for e in eps if e[0] == name and e[1] == tid and te - HORIZON_S <= e[2] <= te]
    if ms: caught += 1; warn.append(te - min(e[2] for e in ms))
  hug = [e for e in eps if not e[5] and e[7] == e[7] and e[7] <= HUG_LINE_M]
  noise = [e for e in eps if not e[5] and not (e[7] == e[7] and e[7] <= HUG_LINE_M)]
  sec = lambda es: sum(min(e[3] - e[2] + 0.5, 3.0) for e in es)   # coast seconds (marked time + 0.5 s release, max 3 s)
  extra = dict(hug=len(hug), noise=len(noise), coast_s_true=sec([e for e in eps if e[5]]), coast_s_hug=sec(hug), coast_s_noise=sec(noise))
  return dict(**extra, km=km, episodes=len(eps), true=tp, false=fp, false_per_100km=fp / max(km, 1e-3) * 100,
              real=len(real), recall=caught / max(len(real), 1), warn_med=float(np.median(warn)) if warn else float('nan')), eps


def main():
  a = sys.argv[1:]
  sets = {}
  for i, x in enumerate(a):
    if x == '--set': k, v = a[i + 1].split('=', 1); sets[k] = v
  lo, hi = (a[a.index('--routes') + 1].split('-') if '--routes' in a else ('33', 'ff'))
  maxs = int(a[a.index('--max-segs') + 1]) if '--max-segs' in a else 100000
  jobs = int(a[a.index('--jobs') + 1]) if '--jobs' in a else 6
  segs = []
  for s in sorted(glob.glob(os.path.join(ROOT, 'raw', '*--*--*'))):
    r = int(os.path.basename(s).split('--')[0], 16)
    if int(lo, 16) <= r <= int(hi, 16) and os.path.exists(s + '/rlog.zst'): segs.append(s)
  segs = segs[:maxs]
  with Pool(jobs) as pool:
    res = [r for r in pool.imap_unordered(run_seg, [(s, sets) for s in segs], chunksize=1) if r]
  sc, eps = score(res)
  print(f"sets {sets or 'current'}: segments {len(res)}, {sc['km']:.0f} km | episodes {sc['episodes']} true {sc['true']} false {sc['false']} "
        f"({sc['false_per_100km']:.0f}/100 km: hugger {sc['hug']} noise {sc['noise']}) | coast s true/hug/noise "
        f"{sc['coast_s_true']:.0f}/{sc['coast_s_hug']:.0f}/{sc['coast_s_noise']:.0f} | real cut-ins {sc['real']} recall {sc['recall']*100:.0f}% warn median {sc['warn_med']:.1f} s")
  if '--list' in a:
    for e in sorted(eps, key=lambda e: (e[0], e[2])):
      print(f"  {e[0]} tid {e[1]} {e[3]-e[2]:4.1f}s v {e[4]*3.6:3.0f} {'TRUE' if e[5] else ('hug' if e[7] == e[7] and e[7] <= HUG_LINE_M else 'noise')} line {e[7]:.1f}")


if __name__ == '__main__':
  main()
