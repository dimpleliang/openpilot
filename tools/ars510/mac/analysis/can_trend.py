"""Per route (first segment with an rlog): the set of values of selected CAN bytes, to see when a constant changed (Mac).
  python3 can_trend.py 0x380:2,3 0x42F:3 0x614:6 0x610:7 ...   [--routes N, default last 25]
"""
import glob, os, sys, zstandard, collections
from multiprocessing import Pool

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log

WANT = {}
for a in sys.argv[1:]:
  if ':' in a:
    addr, bs = a.split(':'); WANT[int(addr, 16)] = [int(b) for b in bs.split(',')]
N = int(sys.argv[sys.argv.index('--routes') + 1]) if '--routes' in sys.argv else 25


def scan(seg):
  vals = collections.defaultdict(set)
  try:
    d = zstandard.ZstdDecompressor().decompressobj().decompress(open(seg + '/rlog.zst', 'rb').read())
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      if m.which() != 'can': continue
      for c in m.can:
        if c.src == 0 and c.address in WANT:
          for b in WANT[c.address]:
            if b < len(c.dat): vals[(c.address, b)].add(c.dat[b])
  except Exception as e:
    return seg, str(e)
  return seg, vals


if __name__ == '__main__':
  routes = collections.defaultdict(list)
  for s in glob.glob(os.path.join(ROOT, 'raw', '*--*--*')):
    if os.path.exists(s + '/rlog.zst'): routes[s.rsplit('--', 1)[0]].append(s)
  pick = []
  for r, ss in routes.items():
    ss.sort(key=lambda s: int(s.rsplit('--', 1)[1])); pick.append(ss[min(1, len(ss) - 1)])
  pick.sort(key=lambda s: os.path.getmtime(s + '/rlog.zst'))
  pick = pick[-N:]
  with Pool(6) as p: res = dict(p.map(scan, pick))
  import time
  for s in pick:
    v = res[s]
    if isinstance(v, str): print(os.path.basename(s), 'error', v); continue
    when = time.strftime('%m/%d %H:%M', time.localtime(os.path.getmtime(s + '/rlog.zst')))
    print(f'{when} {os.path.basename(s):28s} ' + '  '.join(f'{a:X}.b{b}={sorted(v.get((a, b), []))[:5]}' for a in WANT for b in WANT[a]))
