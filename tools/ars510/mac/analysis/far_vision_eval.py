import glob, json, os, sys, struct, zstandard, numpy as np
from collections import deque
from multiprocessing import Pool
ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
FMTS = {1: '<fffB', 2: '<fffBH', 3: '<fffBHhB', 4: '<fffBHhBBB'}
TF = 2.0; MIND = float(os.environ.get("MIND", "30"))
def seg(p):
  try: return seg0(p)
  except Exception as e: return dict(seg=os.path.basename(os.path.dirname(p)), km=0., A=[], B=[], err=str(e)[:80])
def seg0(p):
  from openpilot.cereal import log
  name = os.path.basename(os.path.dirname(p))
  out = dict(seg=name, km=0., A=[], B=[])
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  v = a = at = 0.; en = gas = brk = False; lt = None; wall = None; px = py = None; objs = []; ot = -9
  F = []; TA = []; TB = []
  dq = deque(); a_start = None; a_last = -9; b_id = None; b_start = None
  for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == 'carState':
      c = m.carState; v = c.vEgo; a = c.aEgo; brk = c.brakePressed; gas = c.gasPressed
      if lt is not None and 0 < t - lt < 0.1 and en and v > 5.: out['km'] += v * (t - lt) / 1000.
      lt = t
    elif w == 'clocks' and wall is None and m.clocks.wallTimeNanos > 1.7e18: wall = m.clocks.wallTimeNanos / 1e9 - t
    elif w == 'selfdriveState': en = bool(m.selfdriveState.enabled)
    elif w == 'longitudinalPlan': at = m.longitudinalPlan.aTarget
    elif w == 'modelV2': q = m.modelV2.position; px, py = list(q.x), list(q.y)
    elif w == 'customReservedRawData0':
      b = bytes(m.customReservedRawData0); objs = []; ot = t
      if len(b) >= 4 and b[:2] == b'A5' and b[2] in FMTS:
        fmt = FMTS[b[2]]; sz = struct.calcsize(fmt)
        for i in range(b[3]):
          o = 4 + i * sz
          if o + sz > len(b): break
          f = struct.unpack_from(fmt, b, o); objs.append((f[0], f[1], f[2], bool(f[3] & 2), (f[3] >> 2) & 7, f[4] if len(f) > 4 else -1))
    elif w == 'radarState':
      l = m.radarState.leadOne
      pr, rad, dr, vl, mp = bool(l.present), bool(l.radar), l.dRel, l.vLead, float(l.modelProb)
      F.append((t, v, a, brk, pr, rad, dr, vl))
      act = en and not gas
      # A
      camok = act and pr and not rad and mp >= 0.5 and dr >= MIND and v > 8.
      if camok:
        dq.append((t, vl))
        while dq and t - dq[0][0] > 1.0: dq.popleft()
        vlf = max(min(x[1] for x in dq), 0.)
        gap = dr - (TF * vlf + 11.)
        areq = (v - vlf) ** 2 / (2. * gap) if (gap > 1. and v - vlf > 1. and vlf < 8.) else 0.
      else:
        areq = 0.
        if t - a_last > 0.3: dq.clear()
      if areq >= 0.8:
        if a_start is None: a_start = t
        a_last = t
        if t - a_start >= 1.0:
          ex = at + min(0.9 * areq, 1.5)
          if ex > 0.2: TA.append((t, ex, v, dr, vl, vlf, areq, at))
      elif t - a_last > 0.3: a_start = None
      # B
      best = None
      if act and v > 5. and pr and not rad and mp >= 0.5 and px is not None and t - ot < 0.3:
        for (od, oy, ovr, sett, cls, oid) in objs:
          if sett or cls not in (2, 3, 5, 6) or not (10. < od < 80.) or -ovr < 2.: continue
          off = -oy - (float(np.interp(od, px, py)) if od <= px[-1] else py[-1])
          if abs(off) > 0.9 or dr - od < 8.: continue
          if best is None or od < best[0]: best = (od, ovr, cls, oid, off)
      if best is not None:
        if best[3] != b_id: b_id = best[3]; b_start = t
        if t - b_start >= 0.5:
          od, ovr = best[0], best[1]; vg = max(v + ovr, 0.); g = od - 6. - TF * vg
          ar = min(ovr ** 2 / (2. * g), 3.5) if g > 1. else 3.5
          if ar >= 1.0 and at + ar > 0.3: TB.append((t, at + ar, v, od, vg, best[2], dr, vl, ar, at, best[4]))
      else: b_id = None
  if not F: return out
  A = np.array(F, dtype=float); ft = A[:, 0]
  def outcome(t0, hor):
    i = np.searchsorted(ft, t0); j = max(np.searchsorted(ft, t0 + hor), i + 1); S = A[i:j]
    pres = S[:, 4] > 0
    return dict(mina=float(S[:, 2].min()), brk=bool((S[:, 3] > 0).any()), radlead_d=float(S[(S[:, 5] > 0), 6].min()) if (S[:, 5] > 0).any() else -1.,
                stopped=bool((pres & (S[:, 7] < 3.) & (S[:, 6] < 45.)).any()), mind=float(S[pres, 6].min()) if pres.any() else -1.)
  def eps(T, hor, key):
    i = 0
    while i < len(T):
      j = i
      while j + 1 < len(T) and T[j + 1][0] - T[j][0] < 1.0: j += 1
      r = T[i:j + 1]; e = dict(seg=name, t=float(r[0][0] + (wall or 0)), dur=float(r[-1][0] - r[0][0]), ex_max=float(max(x[1] for x in r)), ex_mean=float(np.mean([x[1] for x in r])), first=[float(x) for x in r[0][2:]])
      e.update(outcome(r[0][0], hor)); out[key].append(e); i = j + 1
  eps(TA, 8., 'A'); eps(TB, 4., 'B')
  return out
if __name__ == '__main__':
  files = sorted(glob.glob(ROOT + '/raw/*/rlog.zst'))
  if len(sys.argv) > 2: files = [f for f in files if any(('/raw/' + q) in f for q in sys.argv[2:])]
  with Pool(4) as pool: res = pool.map(seg, files, chunksize=8)
  with open(sys.argv[1], 'w') as f:
    for r in res: f.write(json.dumps(r) + chr(10))
  print('DONE', len(files), round(sum(r['km'] for r in res), 1), sum(len(r['A']) for r in res), sum(len(r['B']) for r in res), sum(1 for r in res if r.get('err')))
