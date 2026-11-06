"""Engaged stops with no lead (red light / stop line: the model stops the car, no radar or vision lead), from rlogs: where the
model planned to stop vs where the car stopped, at 30 / 20 / 10 / 5 km/h on the way down (Mac, offline).
  python3 model_stops.py <route prefix> [...]
Planned stop = distance along the model's plan where its speed first falls below 0.5 m/s. Overshoot = travelled - planned
(+: the car went past the model's stop point). Also the planner target vs the actual accel in the last 10 km/h.
"""
import glob, os, sys, zstandard
import numpy as np
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log

MARKS = (30, 20, 10, 5)
ALL = '--all' in sys.argv


def load(seg):
  p = seg + '/rlog.zst'
  try:
    return list(log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read()),
                                              traversal_limit_in_words=2**61))
  except Exception:
    return []


def scan(route):
  segs = sorted(glob.glob(os.path.join(ROOT, 'raw', route + '--*')), key=lambda s: int(s.rsplit('--', 1)[1]))
  with Pool(6) as pool:
    for seg, evs in zip(segs, pool.imap(load, segs)):
      yield int(seg.rsplit('--', 1)[1]), evs


def main():
  for route in [a for a in sys.argv[1:] if not a.startswith('--')]:
    st = dict(en=False, lead=False, plan=None, at=0., src='', gas=0, brk=0)
    run = None
    for seg, evs in scan(route):
      for e in evs:
        w = e.which(); t = e.logMonoTime / 1e9
        if w == 'selfdriveState':
          st['en'] = e.selfdriveState.enabled
        elif w == 'radarState':
          a, b = e.radarState.leadOne, e.radarState.leadTwo
          st['lead'] = (a.present and a.dRel < 60) or (b.present and b.dRel < 60)
        elif w == 'modelV2':
          m = e.modelV2
          vx = np.asarray(m.velocity.x, float); px = np.asarray(m.position.x, float)
          i = np.argmax(vx < 0.5) if (vx < 0.5).any() else None
          st['plan'] = float(px[i]) if i is not None else None
          vl = any(l.prob > 0.5 and l.x[0] < 60 for l in m.leadsV3) if len(m.leadsV3) else False
          st['vlead'] = vl
        elif w == 'longitudinalPlan':
          st['at'] = e.longitudinalPlan.aTarget; st['src'] = str(e.longitudinalPlan.longitudinalPlanSource)
        elif w == 'carState':
          c = e.carState; v = c.vEgo
          if run is None:
            if st['en'] and v * 3.6 > 32:
              run = dict(t=t, last=t, dist=0., marks={}, lead=False, gas=0, brk=0, src=set(), err=[])
            continue
          dt = t - run['last']; run['last'] = t
          if not st['en']:
            run = None; continue
          if v * 3.6 > 32:
            run.update(t=t, dist=0., marks={}, lead=False, gas=0, brk=0, src=set(), err=[]); continue
          run['dist'] += v * dt
          run['lead'] |= st['lead'] or st.get('vlead', False)
          run['gas'] |= int(c.gasPressed); run['brk'] |= int(c.brakePressed); run['src'].add(st['src'])
          for mk in MARKS:
            if mk not in run['marks'] and v * 3.6 <= mk:
              run['marks'][mk] = (run['dist'], st['plan'])
          if v * 3.6 <= 10:
            run['err'].append(c.aEgo - st['at'])
          if v < 0.1:
            if (ALL or not run['lead']) and not run['gas']:
              total = run['dist']
              parts = []
              for mk in MARKS:
                if mk in run['marks']:
                  d0, plan = run['marks'][mk]
                  parts.append(f"{mk}km/h plan {plan:5.1f} went {total - d0:5.1f} over {total - d0 - plan:+5.1f}" if plan is not None
                               else f"{mk}km/h plan -")
              err = np.mean(run['err']) if run['err'] else float('nan')
              print(f"{route} seg {seg:3d} | lead {int(run['lead'])} | driver brake {run['brk']} | src {sorted(run['src'])} | a - target (<10 km/h) {err:+.2f}")
              print('    ' + ' | '.join(parts))
            run = None


if __name__ == '__main__':
  main()
