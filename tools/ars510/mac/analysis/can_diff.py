"""Compare which CAN messages exist (and how often, and which bytes vary) between two routes (Mac, read-only, offline).
  python3 can_diff.py <routeA> <routeB> [segs, default 5-9]
Prints addresses only in A / only in B, rate changes > 20 %, and for messages in both, bytes that never vary in one route but
do in the other, or that hold a different constant value.
"""
import os, sys, zstandard, collections, numpy as np

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log


def load(route, segs):
  cnt = collections.Counter(); vals = collections.defaultdict(lambda: collections.defaultdict(set)); dur = 0.
  for s in segs:
    p = os.path.join(ROOT, 'raw', f'{route}--{s}', 'rlog.zst')
    if not os.path.exists(p): continue
    d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read()); t = []
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      if m.which() != 'can': continue
      t.append(m.logMonoTime)
      for c in m.can:
        if c.src >= 128: continue
        k = (c.src, c.address); cnt[k] += 1
        for i, b in enumerate(c.dat[:8]):
          v = vals[k][i]
          if len(v) < 300: v.add(b)
    if t: dur += (t[-1] - t[0]) / 1e9
  return cnt, vals, dur


def main():
  A, B = sys.argv[1], sys.argv[2]
  segs = range(*(int(x) for x in sys.argv[3].split('-'))) if len(sys.argv) > 3 else range(5, 10)
  (ca, va, da), (cb, vb, db) = load(A, segs), load(B, segs)
  print(f'A {A}: {da:.0f} s, {len(ca)} ids   B {B}: {db:.0f} s, {len(cb)} ids')
  ra = {k: v / da for k, v in ca.items()}; rb = {k: v / db for k, v in cb.items()}
  print('only in A (gone in B):', [f'bus{b} 0x{a:X} {ra[(b, a)]:.1f}Hz' for b, a in sorted(set(ra) - set(rb))])
  print('only in B (new):      ', [f'bus{b} 0x{a:X} {rb[(b, a)]:.1f}Hz' for b, a in sorted(set(rb) - set(ra))])
  print('rate change > 20 %:')
  for k in sorted(set(ra) & set(rb)):
    if ra[k] > 0.2 and abs(rb[k] / ra[k] - 1) > 0.2: print(f'  bus{k[0]} 0x{k[1]:X} {ra[k]:.1f} -> {rb[k]:.1f} Hz')
  print('byte differences (A values | B values, constant or few-valued bytes):')
  for k in sorted(set(ra) & set(rb)):
    out = []
    for i in range(8):
      a, b = va[k].get(i, set()), vb[k].get(i, set())
      if a == b or len(a) > 6 or len(b) > 6: continue
      if (len(a) == 1 or len(b) == 1) and not (a & b and len(a) > 1 and len(b) > 1):
        out.append(f'b{i} {sorted(a)[:6]}|{sorted(b)[:6]}')
    if out: print(f'  bus{k[0]} 0x{k[1]:X}: ' + '  '.join(out))


if __name__ == '__main__':
  main()
