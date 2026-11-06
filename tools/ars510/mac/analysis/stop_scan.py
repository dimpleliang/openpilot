"""Stops while engaged (from > 15 km/h to standstill) of routes, from qlogs: who braked (driver brake pedal / openpilot), the
lead at the stop (radar / vision-only, distance), experimental mode, planner source and the lowest planned accel (Mac, offline).
  python3 stop_scan.py <route prefix> [...]
"""
import glob, os, sys, zstandard
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log


def main():
  for route in sys.argv[1:]:
    segs = sorted(glob.glob(os.path.join(ROOT, 'raw', route + '--*')), key=lambda s: int(s.rsplit('--', 1)[1]))
    st = dict(v=0., en=False, exp=None, brk=0, lead=None, src='', at=0.); run = None
    for s in segs:
      p = s + '/qlog.zst'
      if not os.path.exists(p): continue
      try: evs = log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read()), traversal_limit_in_words=2**61)
      except Exception: continue
      seg = int(s.rsplit('--', 1)[1])
      try:
        for e in evs:
          w = e.which(); t = e.logMonoTime / 1e9
          if w == 'selfdriveState':
            st['en'] = e.selfdriveState.enabled; st['exp'] = e.selfdriveState.experimentalMode
          elif w == 'radarState':
            l = e.radarState.leadOne
            st['lead'] = (round(l.dRel, 1), round(l.vLead, 1), 'R' if l.radar else 'V') if l.present else None
          elif w == 'longitudinalPlan':
            lp = e.longitudinalPlan; st['at'] = lp.aTarget
            try: st['src'] = str(lp.longitudinalPlanSource)
            except Exception: pass
          elif w == 'carState':
            c = e.carState; v = c.vEgo
            if run is None and st['en'] and v * 3.6 > 15:
              run = dict(t=t, seg=seg, vmax=v, brk=0, amin=0., leads=[], exp=st['exp'])
            if run is not None:
              if not st['en'] or c.gasPressed: run = None; continue
              run['vmax'] = max(run['vmax'], v); run['brk'] |= int(c.brakePressed); run['amin'] = min(run['amin'], st['at'])
              if v * 3.6 > 15: run['t'] = t; run['brk'] = 0; run['amin'] = 0.; run['leads'] = []
              else: run['leads'].append(st['lead'])
              if v < 0.3:
                ld = [x for x in run['leads'] if x is not None]
                last = run['leads'][-1] if run['leads'] else None
                print(f"{route} seg {seg:3d} +{t - run['t']:4.1f}s from 15 km/h | driver brake {run['brk']} | exp {st['exp']} | "
                      f"lead at stop {last} | lead seen {len(ld)}/{len(run['leads'])} | src {st['src']} | aTarget min {run['amin']:+.2f}")
                run = None
      except Exception:
        pass


if __name__ == '__main__':
  main()
