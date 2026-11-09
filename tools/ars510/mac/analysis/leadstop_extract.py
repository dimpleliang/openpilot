#!/usr/bin/env python3
# ars510 lead-stop extract: one row per radarState frame (20 Hz) -> ~/ars510_rlogs/analysis/leadstop/<route>.npz
# usage: leadstop_extract.py ROUTE [ROUTE ...]   (input for tools/ars510/sim_lead_stop.py)
import sys, os, glob, numpy as np, zstandard
sys.path[:0] = [os.path.expanduser('~/ars510_rlogs/op'), os.path.expanduser('~/ars510_rlogs/op/opendbc_repo')]
from openpilot.cereal import log

COLS = ['t', 'v', 'a', 'long', 'gas', 'brk', 'at', 'src_lead', 'present', 'radar', 'dRel', 'vLead', 'vRel', 'aLeadK',
        'aLeadTau', 'tid', 'yRel', 'standstill', 'cmd', 'seg']

def safe(g):
  out = []
  try:
    for e in g: out.append(e)
  except Exception: pass
  return out

def route(r, out):
  rows = []
  segs = sorted(glob.glob(os.path.expanduser(f'~/ars510_rlogs/raw/{r}--*')), key=lambda s: int(s.rsplit('--', 1)[1]))
  for s in segs:
    f = s + '/rlog.zst'
    if not os.path.exists(f):
      continue
    try:
      evs = log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().decompressobj().decompress(open(f, 'rb').read()),
                                          traversal_limit_in_words=2**61)
      evs = safe(evs)
    except Exception as e:
      print('ERR', s, e); continue
    sn = int(s.rsplit('--', 1)[1])
    st = dict(v=0., a=0., long=0, gas=0, brk=0, at=0., src=0, ss=0, cmd=0.)
    for e in evs:
      w = e.which()
      if w == 'carState':
        cs = e.carState; st.update(v=cs.vEgo, a=cs.aEgo, gas=int(cs.gasPressed), brk=int(cs.brakePressed), ss=int(cs.standstill))
      elif w == 'carControl':
        st['long'] = int(e.carControl.longActive); st['cmd'] = e.carControl.actuators.accel
      elif w == 'longitudinalPlan':
        lp = e.longitudinalPlan; st['at'] = lp.aTarget
        try: st['src'] = 1 if str(lp.longitudinalPlanSource).startswith('lead') else 0
        except Exception: st['src'] = 0
      elif w == 'radarState':
        l = e.radarState.leadOne
        rows.append([e.logMonoTime / 1e9, st['v'], st['a'], st['long'], st['gas'], st['brk'], st['at'], st['src'],
                     int(l.present), int(l.radar), l.dRel, l.vLead, l.vRel, l.aLeadK, l.aLeadTau,
                     getattr(l, 'radarTrackId', -1), l.yRel, st['ss'], st['cmd'], sn])
  if rows:
    np.savez_compressed(out, d=np.array(rows, dtype=np.float64), cols=np.array(COLS))
  print(r, len(rows), flush=True)

if __name__ == '__main__':
  od = os.path.expanduser('~/ars510_rlogs/analysis/leadstop'); os.makedirs(od, exist_ok=True)
  for r in sys.argv[1:]:
    o = f'{od}/{r}.npz'
    if not os.path.exists(o):
      route(r, o)
  print('DONE', flush=True)
