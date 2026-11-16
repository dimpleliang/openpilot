import json, os, sys, zstandard, datetime, numpy as np
ROOT = os.path.expanduser('~/ars510_rlogs'); sys.path[:0] = [ROOT + '/op', ROOT + '/op/opendbc_repo']
from openpilot.cereal import log
tz = datetime.timezone(datetime.timedelta(hours=8))
def load(name):
  p = ROOT + '/raw/' + name + '/rlog.zst'
  if not os.path.exists(p): return []
  d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
  return list(log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61))
E = sorted([e for l in open('osc_out.jsonl') for e in json.loads(l)['e']], key=lambda e: e['t'])
real = ['11/05 21:47', '11/07 07:35', '11/07 10:39', '11/07 11:41', '11/10 13:31', '11/11 19:33', '11/11 23:39', '11/13 08:44', '11/16 08:52']
for e in E:
  s = datetime.datetime.fromtimestamp(e['t'], tz).strftime('%m/%d %H:%M')
  if s not in real: continue
  r, sg = e['seg'].rsplit('--', 1); ev = load('%s--%d' % (r, int(sg) - 1)) + load(e['seg']) + load('%s--%d' % (r, int(sg) + 1))
  ev.sort(key=lambda m: m.logMonoTime)
  off = [m.clocks.wallTimeNanos / 1e9 - m.logMonoTime / 1e9 for m in ev if m.which() == 'clocks' and m.clocks.wallTimeNanos > 1.7e18][0]
  t0 = e['t'] - 8.; t1 = e['t'] + e['dur'] + 1.
  cmd = out = None; R = []; k = 0
  act = False; ent = 0; ex = 0; rel = 999
  for m in ev:
    w = m.which(); t = m.logMonoTime / 1e9 + off
    if w == 'carControl': cmd = m.carControl.actuators.steeringAngleDeg
    elif w == 'carOutput': out = m.carOutput.actuatorsOutput.steeringAngleDeg
    elif w == 'carState' and cmd is not None and out is not None and t0 - 5 <= t <= t1:
      k += 1
      if k % 2: continue
      c = m.carState; tq = abs(c.steeringTorque)
      if not act:
        ent = ent + 1 if tq > 150 else 0
        if ent >= 5: act = True; ex = 0
        else: rel = min(rel + 1, 999)
      elif tq < 100:
        ex += 1
        if ex >= 10: act = False; ent = 0; rel = 0
      else: ex = 0
      if t >= t0: R.append((t, float(act), float((not act) and rel < 50), cmd, out, c.steeringAngleDeg, tq))
  A = np.array(R); osc = A[:, 0] >= e['t']
  O = A[osc]; pre = A[~osc]
  lim = np.abs(O[:, 3] - O[:, 4]) > 1.5
  def ivals(col):
    x = A[:, col] > 0; out_ = []; st = None
    for i in range(len(x)):
      if x[i] and st is None: st = A[i, 0]
      if not x[i] and st is not None: out_.append('%+.1f..%+.1f' % (st - e['t'], A[i, 0] - e['t'])); st = None
    if st is not None: out_.append('%+.1f..end' % (st - e['t']))
    return out_
  print(s, 'dur%.0fs | during osc: override %.0f%% ramp %.0f%% | cmd limited(>1.5deg) %.0f%% mean|cmd-out| %.1f max %.1f | max|out-ang| %.1f | tq>150 %.1f%% tq>100 %.1f%% | override intervals (s rel. to start) %s ramp %s' % (
    e['dur'], 100 * O[:, 1].mean(), 100 * O[:, 2].mean(), 100 * lim.mean(), np.abs(O[:, 3] - O[:, 4]).mean(), np.abs(O[:, 3] - O[:, 4]).max(), np.abs(O[:, 4] - O[:, 5]).max(), 100 * (O[:, 6] > 150).mean(), 100 * (O[:, 6] > 100).mean(), ivals(1)[:6], ivals(2)[:6]))
