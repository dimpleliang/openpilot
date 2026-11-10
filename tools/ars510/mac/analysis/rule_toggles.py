#!/usr/bin/env python3
"""ars510: how often the near_stop floor and the e2e fill switch on / off, why, and whether a switch moves the plan
(Mac, offline; reads the 'ARS510 near stop' / 'ARS510 e2e fill' log lines and longitudinalPlan.aTarget from rlogs).

  python3 rule_toggles.py <route prefix> [...]      e.g. 0000006c--0f7526a1c2

near_stop   an episode = switches less than GAP_S apart. Off reasons from the logged values:
              vLead  lead faster than NEAR_STOP_VLEAD_MAX (1.0 m/s)      -> the boundary this report is about
              far    lead beyond NEAR_STOP_D_MAX (20 m)
              slow   ego below NEAR_STOP_MIN_V (0.3 m/s)
              plan   none of these: the plan itself asked for at least the floor (harmless)
e2e fill    off reasons: slow_req (model below E2E_FILL_INDIFFERENT -0.15: the fill is dropped in one step),
              cross (model rose above the fill: continuous by construction), set (at / above the set speed).
step        largest change of aTarget between two plan frames (50 ms) around the switch, m/s^2.
"""
import glob
import json
import os
import re
import sys
from collections import Counter

import numpy as np
import zstandard

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log  # noqa: E402

GAP_S = 10.
NUM = r'(-?\d+\.?\d*)'
RE_NS = re.compile(rf'ARS510 near stop (on|off) d={NUM} vLead={NUM} v={NUM} a={NUM}')
RE_FILL = re.compile(rf'ARS510 e2e fill (on|off) v={NUM} set={NUM} e2e={NUM} fill={NUM}')


def load(route):
  ns, fill, plan, span = [], [], [], 0.
  for seg in sorted(glob.glob(os.path.join(ROOT, 'raw', route + '--*')), key=lambda s: int(s.rsplit('--', 1)[1])):
    p = seg + '/rlog.zst'
    if not os.path.exists(p):
      continue
    try:
      dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
      for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
        w = e.which()
        t = e.logMonoTime * 1e-9
        if w == 'longitudinalPlan':
          plan.append((t, e.longitudinalPlan.aTarget))
        elif w == 'logMessage':
          s = e.logMessage
          if 'ARS510 near stop' in s or 'ARS510 e2e fill' in s:
            try:
              s = json.loads(s).get('msg', s)
            except Exception:
              pass
            m = RE_NS.search(s)
            if m:
              ns.append((t, m.group(1) == 'on') + tuple(float(x) for x in m.groups()[1:]))
            m = RE_FILL.search(s)
            if m:
              fill.append((t, m.group(1) == 'on') + tuple(float(x) for x in m.groups()[1:]))
    except Exception as ex:
      print(f'  {os.path.basename(seg)}: {ex}', file=sys.stderr)
  plan = np.array(plan) if plan else np.zeros((0, 2))
  if len(plan):
    span = len(plan) * 0.05
  return ns, fill, plan, span


def step(plan, t):
  w = (plan[:, 0] > t - 0.11) & (plan[:, 0] < t + 0.16)
  a = plan[w, 1]
  if len(a) < 2:
    return 0.
  d = np.diff(a)
  return float(d[np.argmax(np.abs(d))])


def main():
  ns, fill, span = [], [], 0.
  for k, route in enumerate(sys.argv[1:]):
    n, f, p, s = load(route)
    # the step is taken from this route's own plan (monotonic time restarts at boot), then times are made unique
    ns += [(e[0] + k * 1e6,) + e[1:] + (step(p, e[0]),) for e in n]
    fill += [(e[0] + k * 1e6,) + e[1:] + (step(p, e[0]),) for e in f]
    span += s
  print(f'{span / 60:.0f} min of plan')

  # near_stop
  eps, cur = [], []
  for ev in ns:
    if cur and ev[0] - cur[-1][0] > GAP_S:
      eps.append(cur); cur = []
    cur.append(ev)
  if cur:
    eps.append(cur)
  print(f'\nnear_stop: {len(ns)} switches in {len(eps)} episodes')
  why = Counter(); steps = {}
  multi = 0
  for ep in eps:
    ons = sum(1 for e in ep if e[1])
    multi += ons > 1
    for t, on, d, vl, v, a, st in ep:
      if on:
        steps.setdefault('on', []).append(st)
        continue
      r = 'vLead' if vl > 1.0 else 'far' if d > 20. else 'slow' if v / 3.6 < 0.3 else 'plan'
      why[r] += 1
      steps.setdefault(r, []).append(st)
  print(f'  episodes with more than one on: {multi}')
  for k in ('on', 'vLead', 'plan', 'slow', 'far'):
    if k in steps:
      s = np.array(steps[k])
      print(f"  {'on ' if k == 'on' else 'off:' + k:10s} n={len(s):3d}  step median {np.median(s):+.2f}  largest {s[np.argmax(np.abs(s))]:+.2f}  |step|>0.3: {int((np.abs(s) > 0.3).sum())}")
  worst = sorted((e for ep in eps for e in ep), key=lambda e: -abs(e[6]))[:6]
  for t, on, d, vl, v, a, st in worst:
    print(f"    route {int(t // 1e6)} t={t % 1e6:8.1f} {'on ' if on else 'off'} d={d:4.1f} vLead={vl:3.1f} v={v:4.1f} km/h a={a:+.2f} step={st:+.2f}")
  # vLead dither: off by vLead then on again within 3 s
  dither = 0
  for ep in eps:
    for a, b in zip(ep, ep[1:]):
      if not a[1] and a[3] > 1.0 and b[1] and b[0] - a[0] < 3.:
        dither += 1
  print(f'  off-by-vLead then on again within 3 s: {dither}')

  # e2e fill
  print(f'\ne2e fill: {len(fill)} switches ({len(fill) / max(span / 60, 1e-6):.1f} per min)')
  on_t = None; dur = []; why = Counter(); steps = {}
  for t, on, v, vset, e2e, f, st in fill:
    if on:
      on_t = t
      steps.setdefault('on', []).append(st)
      continue
    if on_t is not None:
      dur.append(t - on_t); on_t = None
    r = 'slow_req' if e2e <= -0.15 else 'set' if vset <= v + 0.5 else 'cross'
    why[r] += 1
    steps.setdefault(r, []).append(st)
  if dur:
    d = np.array(dur)
    print(f'  on time: median {np.median(d):.1f} s, < 1 s: {int((d < 1).sum())}, < 3 s: {int((d < 3).sum())}, total {d.sum() / 60:.1f} min')
  for k in ('on', 'cross', 'slow_req', 'set'):
    if k in steps:
      s = np.array(steps[k])
      print(f"  {'on ' if k == 'on' else 'off:' + k:12s} n={len(s):4d}  step median {np.median(s):+.2f}  largest {s[np.argmax(np.abs(s))]:+.2f}  "
            f"|step|>0.1: {int((np.abs(s) > 0.1).sum())}  >0.2: {int((np.abs(s) > 0.2).sum())}")


if __name__ == '__main__':
  main()
