#!/usr/bin/env python3
"""ars510: episodes from far_closing_eval.py output that a FAR_OBJ_MAX_CLOSING above 12 m/s newly admits, and whether
openpilot took that car as the radar lead afterwards.   python3 far_closing_agg.py OUT.jsonl [new limit, default 20]"""
import json, sys, datetime, collections
TZ = datetime.timezone(datetime.timedelta(hours=8))
OLD = 12.; NEW = float(sys.argv[2]) if len(sys.argv) > 2 else 20.
rows = [json.loads(l) for l in open(sys.argv[1])]
km = sum(r['km'] for r in rows); kme = sum(r['km_eng'] for r in rows); nfar = sum(1 for r in rows if r['n_far'])
print(f"segments {len(rows)}  with far-object points {nfar}  km above 70 km/h {km:.0f}  of which engaged {kme:.0f}")
kmfar = sum(r['km'] for r in rows if r['n_far']); 
def episodes(fr, pred):
  eps = []; cur = None
  for f in fr:
    if not pred(f): continue
    if cur is not None and f[0] - cur[-1][0] <= 0.6: cur.append(f)
    else:
      cur = [f]; eps.append(cur)
  return eps
tot = collections.Counter(); out = []
for r in sorted(rows, key=lambda r: (r['seg'].rsplit('--', 1)[0], int(r['seg'].rsplit('--', 1)[1]))):
  fr = r['frames']
  for e in episodes(fr, lambda f: -f[2] <= OLD): 
    if e[-1][0] - e[0][0] >= 0.3: tot['old_eps'] += 1
  for e in episodes(fr, lambda f: -f[2] <= NEW):
    newf = [f for f in e if -f[2] > OLD]
    if len(newf) < 6: continue   # < 0.3 s of newly admitted frames
    t0 = newf[0][0]; t1 = newf[-1][0]; f0 = newf[0]; fl = e[-1]
    vobj = fl[3] + fl[2]
    conf = None
    for (t, rad, d, vl) in r['leads']:
      if t < t0 or t > fl[0] + 6.: continue
      if rad and abs(d - (fl[1] + fl[2] * (t - fl[0]))) < 25. and abs(vl - vobj) < 6.:
        conf = t - t0; break
    lead0 = f0[8]
    act = sum(1 for f in newf if f[5] and not f[6] and not f[7]) * 0.05
    kind = 'confirmed' if conf is not None else 'unconfirmed'
    if lead0 and lead0[0] and abs(lead0[1] - f0[1]) < 25.: kind = 'already_lead'
    tot[kind] += 1; tot[kind + '_act_s'] += act
    w = r['wall']; ts = datetime.datetime.fromtimestamp(w[0] + (t0 - w[1]), TZ).strftime('%m/%d %H:%M:%S') if w else '?'
    out.append((kind, f"{kind:12s} {ts} {r['seg'][:8]}--{r['seg'].rsplit('--',1)[1]:>3s} t{t0:8.1f} new {len(newf)*0.05:4.1f}s (acting {act:4.1f}s) d {f0[1]:5.1f}->{fl[1]:5.1f} closing {-f0[2]:4.1f}->{-fl[2]:4.1f} v_ego {f0[3]*3.6:5.1f} v_obj {vobj*3.6:5.1f} off {f0[4]:+.1f} lead0 {lead0} conf {'-' if conf is None else f'{conf:.1f}s'}"))
for k, s in sorted(out): print(s)
print(dict(tot))
print(f"km (>70 km/h) in segments that have far-object points: {kmfar:.0f}")
