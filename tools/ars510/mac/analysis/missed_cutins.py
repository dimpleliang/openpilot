"""Every real cut-in of a route (track >= CUT_FROM_M off my path, then < IN_LANE_M or became the lead) with what the UI rule /
planner response did, and the track's history before entering, to see why a merge got no mark or response (Mac, offline).
  python3 missed_cutins.py <route prefix e.g. 00000053> [--min-kmh 15] [--hist 4]
"""
import datetime, glob, os, sys
from collections import defaultdict
from multiprocessing import Pool
import merge_replay as mr

TZ = datetime.timezone(datetime.timedelta(hours=8))


def main():
  a = sys.argv[1:]
  route = a[0]
  mr.MIN_KMH = float(a[a.index('--min-kmh') + 1]) if '--min-kmh' in a else 15.
  hist_s = float(a[a.index('--hist') + 1]) if '--hist' in a else 4.
  segs = sorted(glob.glob(os.path.join(mr.ROOT, 'raw', route + '--*')), key=lambda s: int(s.rsplit('--', 1)[1]))
  segs = [s for s in segs if os.path.exists(s + '/rlog.zst')]
  with Pool(6) as pool:
    res = [r for r in pool.imap(mr.run_seg, [(s, {}) for s in segs], chunksize=1) if r]
  for name, frames, km, gps in res:
    wall = (lambda t: datetime.datetime.fromtimestamp(gps[1] + t - gps[0], TZ).strftime('%H:%M:%S')) if gps else (lambda t: f'+{t:.0f}')
    hist = defaultdict(list)
    for t, v, eng, pts, lead, mk, tg, raw in frames:
      for tid, (off, d, line) in pts.items():
        r = raw.get(tid)
        il = lead is not None and r is not None and abs(r[0] - lead[0]) < 2.0 and abs(r[1] - lead[1]) < 1.2
        hist[tid].append(dict(t=t, off=off, d=d, line=line, lead=il, v=v, eng=eng, vr=r[2] if r else 0., cls=r[3] if r else -1,
                              vl=r[4] if r else 0., mk=tid in mk, tg=(tg[0] if tg is not None and tg[1] == tid else None),
                              ld=lead[0] if lead else None))
    for tid, h in hist.items():
      out = False
      for i, x in enumerate(h):
        if abs(x['off']) >= mr.CUT_FROM_M: out = True
        elif out and (abs(x['off']) < mr.IN_LANE_M or x['lead']) and x['v'] * 3.6 > mr.MIN_KMH:
          te = x['t']
          pre = [y for y in h if te - hist_s <= y['t'] <= te + 1.0]
          marked = any(y['mk'] for y in pre); resp = min([y['tg'] for y in pre if y['tg'] is not None], default=None)
          # was it a real lane entry from the side (moved >= 1.2 m laterally within hist_s)?
          moved = max(abs(y['off']) for y in pre) - abs(x['off'])
          if moved < 1.0: break
          ttc = x['d'] / -x['vr'] if x['vr'] < -0.3 else float('inf')
          print(f"\n{name} {wall(te)} tid {tid} cls {x['cls']} v {x['v']*3.6:.0f} km/h eng {int(x['eng'])} | at entry d {x['d']:.1f} "
                f"vrel {x['vr']:+.1f} ttc {ttc:.1f}s lead_d {x['ld'] if x['ld'] is None else round(x['ld'], 1)} | "
                f"mark {'Y' if marked else 'N'} resp {resp if resp is None else round(-resp, 2)}")
          last = -9.
          for y in pre:
            if y['t'] - last >= 0.5:
              last = y['t']
              ld = '-' if y['ld'] is None else f"{y['ld']:.0f}"
              rs = '' if y['tg'] is None else f" RESP {-y['tg']:.2f}"
              print(f"   {y['t']-te:+5.1f}s d {y['d']:5.1f} off {y['off']:+5.2f} line {y['line']:+5.2f} vr {y['vr']:+5.1f} "
                    f"vlat {y['vl']:+4.1f} lead_d {ld}{' MARK' if y['mk'] else ''}{rs}")
          break


if __name__ == '__main__':
  main()
