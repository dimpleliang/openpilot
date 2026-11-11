#!/usr/bin/env python3
"""ars510: far radar object (FAR_OBJ_TRACK_ID) episodes that a higher FAR_RADAR_ONLY_MAX_CLOSING would newly admit.
Replays the planner's far_object() gate on logged radarTracks + modelV2 path (Mac, offline).
  python3 far_closing_eval.py OUT.jsonl [route_prefix ...]      (all of ~/ars510_rlogs/raw when no prefix)"""
import glob, json, os, sys, zstandard, numpy as np
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
FAR_ID = 1 << 28
OLD, NEW = 12., 20.
MIN_V = 70. / 3.6
PATH_BP, PATH_V = [80., 150.], [2.2, 3.0]
T_FOLLOW, STOP_D, MARGIN = 1.45, 6., 5.
def ceil_w(d, vrel, vlead):
  closing = -vrel
  if closing < 1.0: return None
  ttc = d / closing
  w = float(np.interp(d, [50., 70.], [0., 1.]) * np.interp(ttc, [5., 7.], [0., 1.]))
  gap = d - (T_FOLLOW * max(vlead, 0.) + STOP_D + MARGIN)
  if w <= 0 or gap <= 1.: return None
  return w
def seg(p):
  from openpilot.cereal import log
  out = dict(seg=os.path.basename(os.path.dirname(p)), km=0., km_eng=0., n_far=0, frames=[], leads=[], wall=None)
  try:
    d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
    v = 0.; lt = None; px = py = None; lead = None; en = False; gas = False; brk = False
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      w = m.which()
      if w == 'carState':
        c = m.carState; t = m.logMonoTime / 1e9; v = c.vEgo; gas = c.gasPressed; brk = c.brakePressed
        if lt is not None and v > MIN_V and 0 < t - lt < 0.1:
          out['km'] += v * (t - lt) / 1000.
          if en: out['km_eng'] += v * (t - lt) / 1000.
        lt = t
      elif w == 'clocks' and out['wall'] is None:
        if m.clocks.wallTimeNanos > 1.7e18: out['wall'] = (m.clocks.wallTimeNanos / 1e9, m.logMonoTime / 1e9)
      elif w == 'selfdriveState': en = bool(m.selfdriveState.enabled)
      elif w == 'modelV2':
        pos = m.modelV2.position; px = np.array(pos.x); py = np.array(pos.y)
      elif w == 'radarState':
        l = m.radarState.leadOne; t = m.logMonoTime / 1e9
        lead = (bool(l.radar), float(l.dRel), float(l.vRel), float(l.vLead)) if l.present else None
        if lead is not None and v > 40 / 3.6: out['leads'].append((round(t, 2), int(lead[0]), round(lead[1], 1), round(lead[3], 1)))
      elif w == 'radarTracks':
        t = m.logMonoTime / 1e9
        for q in m.radarTracks.points:
          if q.trackId != FAR_ID: continue
          out['n_far'] += 1
          if v <= MIN_V or px is None: continue
          path_y = float(np.interp(q.dRel, px, py)) if len(px) > 1 and q.dRel <= px[-1] else 0.
          off = -float(q.yRel) - path_y
          if abs(off) >= float(np.interp(q.dRel, PATH_BP, PATH_V)): continue
          closing = -q.vRel
          if closing > NEW + 15.: continue
          if ceil_w(q.dRel, q.vRel, v + q.vRel) is None: continue
          out['frames'].append((round(t, 2), round(float(q.dRel), 1), round(float(q.vRel), 2), round(v, 2), round(off, 2), int(en), int(gas), int(brk),
                                None if lead is None else (int(lead[0]), round(lead[1], 1), round(lead[2], 2))))
  except Exception as ex:
    out['err'] = str(ex)[:120]
  return out
if __name__ == '__main__':
  outp = sys.argv[1]; pref = sys.argv[2:]
  files = sorted(glob.glob(ROOT + '/raw/*/rlog.zst'))
  if pref: files = [f for f in files if any(os.path.basename(os.path.dirname(f)).startswith(x) for x in pref)]
  done = set()
  if os.path.exists(outp):
    for line in open(outp): done.add(json.loads(line)['seg'])
  files = [f for f in files if os.path.basename(os.path.dirname(f)) not in done]
  print(len(files), 'segments to do', flush=True)
  with Pool(7) as pool, open(outp, 'a') as fo:
    for i, r in enumerate(pool.imap_unordered(seg, files, chunksize=4)):
      fo.write(json.dumps(r) + '\n'); fo.flush()
      if i % 100 == 0: print(i, flush=True)
  print('EVAL_DONE', flush=True)
