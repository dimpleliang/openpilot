"""Dump, around a moment, my lead (radarState leadOne: radar or vision, model lead distance) and every ARS510 track in the next
lanes (customReservedRawData0, v3 records) so a reported cut-in mark can be checked by distance (Mac, offline).
  python3 merge_dump.py <route> <seg> [seg ...] --start HH:MM:SS --from HH:MM:SS --to HH:MM:SS
Radar distance d is the radar's reported object position (longitudinal, from the radar), y left-positive.
"""
import os, sys, struct, zstandard, numpy as np

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log

FMTS = {1: "<fffB", 2: "<fffBH", 3: "<fffBHhB"}


def parse(p):
  if len(p) < 4 or p[:2] != b"A5" or p[2] not in FMTS: return []
  fmt = FMTS[p[2]]; sz = struct.calcsize(fmt); out = []
  for i in range(p[3]):
    off = 4 + i * sz
    if off + sz > len(p): break
    f = struct.unpack_from(fmt, p, off)
    out.append(dict(d=f[0], y=f[1], v=f[2], settling=bool(f[3] & 2), tid=f[4] if len(f) > 4 else -1,
                    vlat=f[5] / 100 if len(f) > 5 else float('nan'), code=f[6] if len(f) > 6 else 0))
  return out


def hms(s):
  h, m, x = (int(v) for v in s.split(':')); return h * 3600 + m * 60 + x


def main():
  a = sys.argv[1:]
  route = a[0]; segs = [x for x in a[1:] if x.isdigit()]
  w0, f, to = hms(a[a.index('--start') + 1]), hms(a[a.index('--from') + 1]), hms(a[a.index('--to') + 1])
  t0 = None; last = -1; st = dict(v=0., lead=None, mlead=float('nan'), tracks=[])
  print('clock     v   | my lead: d  y  vLead src | model lead d | next-lane tracks (tid d y vrel vlat code)')
  for sg in segs:
    dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(os.path.join(ROOT, 'raw', f'{route}--{sg}', 'rlog.zst'), 'rb').read())
    for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
      w = e.which(); t = e.logMonoTime / 1e9
      if w == 'controlsState' and t0 is None: t0 = t
      if t0 is None: continue
      if w == 'carState': st['v'] = e.carState.vEgo
      elif w == 'radarState':
        l = e.radarState.leadOne
        st['lead'] = (l.dRel, l.yRel, l.vLead * 3.6, 'R' if l.radar else 'V') if l.present else None
      elif w == 'modelV2':
        try:
          ld = e.modelV2.leadsV3[0]; st['mlead'] = ld.x[0] if ld.prob > 0.5 else float('nan')
        except Exception: pass
      elif w == 'customReservedRawData0':
        st['tracks'] = parse(bytes(e.customReservedRawData0))
      tw = w0 + t - t0
      if w == 'radarState' and f <= tw <= to and tw - last >= 0.5:
        last = tw
        ld = st['lead']; ls = f"{ld[0]:5.1f} {ld[1]:+4.1f} {ld[2]:4.0f} {ld[3]}" if ld else '   -               '
        side = [x for x in st['tracks'] if 1.8 <= abs(x['y']) <= 6.5 and 5 < x['d'] < 110 and not x['settling']]
        side.sort(key=lambda x: x['d'])
        ts = '  '.join(f"{x['tid']}:{x['d']:.1f}/{x['y']:+.1f}/{x['v']:+.1f}/{x['vlat']:+.2f}{'/C' + str(x['code']) if x['code'] else ''}" for x in side[:5])
        print(f"{int(tw//3600)%24:02d}:{int(tw//60)%60:02d}:{int(tw%60):02d} {st['v']*3.6:4.0f} | {ls} | {st['mlead']:5.1f} | {ts}")


if __name__ == '__main__':
  main()
