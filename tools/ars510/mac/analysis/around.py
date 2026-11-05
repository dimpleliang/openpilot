"""Print my speed / accel / pedals / planner target / lead every 0.5 s around given mono times of a route (Mac, offline).
  python3 around.py <route> <mono_s>[,<mono_s>...] [--pre 3 --post 5]
"""
import glob, os, sys, zstandard
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log

a = sys.argv[1:]
route = a[0]; ts = [float(x) for x in a[1].split(',')]
pre = float(a[a.index('--pre') + 1]) if '--pre' in a else 3.; post = float(a[a.index('--post') + 1]) if '--post' in a else 5.
segs = sorted(glob.glob(os.path.join(ROOT, 'raw', route + '--*')), key=lambda s: int(s.rsplit('--', 1)[1]))
st = {}; last = {t: -99. for t in ts}
for s in segs:
  p = s + '/rlog.zst'
  if not os.path.exists(p): continue
  dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
    t = e.logMonoTime / 1e9; w = e.which()
    if w == 'carState':
      c = e.carState; st.update(v=c.vEgo * 3.6, a=c.aEgo, gas=int(c.gasPressed), brk=int(c.brakePressed))
    elif w == 'longitudinalPlan':
      lp = e.longitudinalPlan; st.update(at=lp.aTarget, src=str(lp.longitudinalPlanSource) if hasattr(lp, 'longitudinalPlanSource') else '')
    elif w == 'radarState':
      l = e.radarState.leadOne; st.update(ld=l.dRel if l.present else None, lv=l.vRel if l.present else None)
    elif w == 'selfdriveState':
      st['en'] = int(e.selfdriveState.enabled)
    if w != 'carState' or 'at' not in st: continue
    for t0 in ts:
      if t0 - pre <= t <= t0 + post and t - last[t0] >= 0.5:
        last[t0] = t
        ld = '-' if st.get('ld') is None else f"{st['ld']:.0f}/{st['lv']:+.1f}"
        print(f"{t0:.0f} {t - t0:+5.1f}s v {st['v']:5.1f} a {st['a']:+5.2f} aTgt {st['at']:+5.2f} gas {st['gas']} brk {st['brk']} en {st.get('en')} lead {ld}")
