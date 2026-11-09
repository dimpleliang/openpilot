#!/usr/bin/env python3
"""ars510: frame-by-frame equivalence of two LongitudinalPlanner versions on rlogs (offroad, on the device: needs the
compiled MPC). Every ars510 rule is on (no off files) and the cut-in response is forced on, so all rules are exercised.

  dump: python3 planner_equiv.py dump <root> <out.npz> <rlog.zst> [...]
        <root> is a tree with the planner version to test at <root>/openpilot/... (e.g. a `cp -as` copy of /data/openpilot
        with the changed .py files replaced); it goes first on sys.path, /data/openpilot supplies everything else.
  cmp:  python3 planner_equiv.py cmp <a.npz> <b.npz>
"""
import os
import sys
from collections import defaultdict

import numpy as np


def dump(root, out, rlogs):
  sys.path[:0] = [root, '/data/openpilot', '/data/openpilot/opendbc_repo']
  real_exists = os.path.exists

  def exists(p):
    if isinstance(p, str) and p.startswith('/data/ars510_') and p != '/data/ars510_accel_scale':
      return p == '/data/ars510_cutin_predict'
    return real_exists(p)
  os.path.exists = exists

  import zstandard
  import openpilot.cereal.messaging as messaging
  from opendbc.car.structs import car
  from openpilot.cereal import custom, log
  from openpilot.common.params import Params
  from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
  print('planner from', lp.__file__, flush=True)

  services = ['carControl', 'carState', 'controlsState', 'radarState', 'modelV2', 'selfdriveState', 'liveMapDataSP',
              'carStateSP', 'selfdriveStateSP', 'gpsLocation', 'gpsLocationExternal', 'radarTracks', 'customReservedRawData0']

  class SM:
    def __init__(self):
      self.data, self.updated, self.logMonoTime, self.recv_frame, self.frame = {}, defaultdict(bool), {}, defaultdict(int), 0
      self.valid, self.alive = defaultdict(lambda: True), defaultdict(lambda: True)

    def __getitem__(self, k):
      if k not in self.data:
        self.data[k] = getattr(messaging.new_message(k), k)
      return self.data[k]
    def all_checks(self, *a): return True
    def all_alive(self, *a): return True
    def all_valid(self, *a): return True

  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParamsPersistent") or params.get("CarParams"), car.CarParams)
  CP_SP = messaging.log_from_bytes(params.get("CarParamsSPPersistent") or params.get("CarParamsSP"), custom.CarParamsSP)
  rows, deciders = [], []
  for f in rlogs:
    pl = lp.LongitudinalPlanner(CP, CP_SP)
    try:
      dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(f, 'rb').read())
      evs = log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61)
      sm = SM(); pend = set()
      for e in evs:
        w = e.which()
        if w not in services:
          continue
        sm.data[w] = getattr(e, w); sm.logMonoTime[w] = e.logMonoTime; pend.add(w)
        if w != 'modelV2' or not all(k in sm.data for k in ('carState', 'controlsState', 'radarState', 'selfdriveState', 'carControl')):
          continue
        sm.frame += 1
        for k in list(sm.updated): sm.updated[k] = False
        for k in pend: sm.updated[k] = True; sm.recv_frame[k] = sm.frame
        pend = set()
        pl.update(sm)
        rows.append((e.logMonoTime / 1e9, float(pl.output_a_target), float(pl.output_should_stop), int(pl.mpc.source),
                     float(pl.v_desired_filter.x)))
        deciders.append(getattr(pl, 'decider', ''))
    except Exception as ex:
      print('segment error', f, repr(ex), flush=True)
    print(f, len(rows), flush=True)
  np.savez_compressed(out, d=np.array(rows), dec=np.array(deciders))


def cmp(a, b):
  A, B = np.load(a), np.load(b)
  da, db = A['d'], B['d']
  print(f"frames {len(da)} vs {len(db)}")
  n = min(len(da), len(db))
  diff = np.abs(da[:n] - db[:n])
  names = ['t', 'aTarget', 'shouldStop', 'source', 'v_desired']
  bad = False
  for i, nm in enumerate(names):
    m = float(diff[:, i].max()) if n else 0.
    k = int((diff[:, i] > 0).sum())
    print(f"  {nm:10s} max diff {m:.3e}  frames differing {k}")
    bad |= k > 0
  if 'dec' in B.files:
    u, c = np.unique(B['dec'], return_counts=True)
    print('  deciders (b):', ', '.join(f"{x} {y / 20:.0f}s" for x, y in sorted(zip(u, c), key=lambda z: -z[1])))
  print('IDENTICAL' if not bad and len(da) == len(db) else 'DIFFERENT')


if __name__ == '__main__':
  if sys.argv[1] == 'dump':
    dump(sys.argv[2], sys.argv[3], sys.argv[4:])
  else:
    cmp(sys.argv[2], sys.argv[3])
