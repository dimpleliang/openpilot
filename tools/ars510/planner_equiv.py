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
  cp_b = params.get("CarParamsPersistent") or params.get("CarParams")
  cpsp_b = params.get("CarParamsSPPersistent") or params.get("CarParamsSP")
  if cp_b is None or cpsp_b is None:
    # off the device (Mac container, tools/ars510/mac/replay.sh): take them from a segment-0 rlog (ARS510_CP_RLOG)
    dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(os.environ['ARS510_CP_RLOG'], 'rb').read())
    for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
      if e.which() == 'carParams' and cp_b is None:
        cp_b = e.carParams.as_builder().to_bytes()
      elif e.which() == 'carParamsSP' and cpsp_b is None:
        cpsp_b = e.carParamsSP.as_builder().to_bytes()
      if cp_b is not None and cpsp_b is not None:
        break
    print('CarParams from', os.environ['ARS510_CP_RLOG'], flush=True)
  CP = messaging.log_from_bytes(cp_b, car.CarParams)
  CP_SP = messaging.log_from_bytes(cpsp_b, custom.CarParamsSP)
  rows, deciders, rain_lv = [], [], []
  # ARS510_WIPER=can: the wiper state for the rain mode from raw CAN 0x2F9 (logs from before carStateSP.wiperLevel);
  # ARS510_WIPER=<n>: forced level. Unset: whatever carStateSP carries. The state carries over between segments.
  wiper_mode = os.environ.get('ARS510_WIPER', '')
  wiper = [int(wiper_mode) if wiper_mode.isdigit() else 0, False]
  rain_keep = None
  for f in rlogs:
    pl = lp.LongitudinalPlanner(CP, CP_SP)
    if wiper_mode and hasattr(pl, 'wiper_src'):
      pl.wiper_src = lambda sm: (wiper[0], wiper[1])
      if rain_keep is not None:
        pl.ars510.rain = rain_keep   # the rain state is minutes long: keep it across the per-segment planner
      rain_keep = pl.ars510.rain
    try:
      dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(f, 'rb').read())
      evs = log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61)
      sm = SM(); pend = set()
      for e in evs:
        w = e.which()
        if w == 'can' and wiper_mode == 'can':
          for c in e.can:
            if c.address == 0x2F9 and c.src == 0:
              wiper[0], wiper[1] = (c.dat[0] >> 5) & 3, bool(c.dat[0] & 0x10)
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
        rain_lv.append(pl.ars510.rain.level if hasattr(getattr(pl, 'ars510', None), 'rain') else 0)
    except Exception as ex:
      print('segment error', f, repr(ex), flush=True)
    print(f, len(rows), flush=True)
  np.savez_compressed(out, d=np.array(rows), dec=np.array(deciders), rain=np.array(rain_lv, dtype=np.int8))


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
  if bad and n:
    # what an intended change did: signed aTarget difference (b - a), its stretches, and frame-to-frame plan steps
    d = db[:n, 1] - da[:n, 1]
    dif = np.abs(d) > 1e-9
    edges = np.flatnonzero(np.diff(np.concatenate(([0], dif.astype(int), [0]))))
    runs = (edges[1::2] - edges[::2]) / 20.
    print(f"  aTarget b-a: min {d.min():+.3f} max {d.max():+.3f}; b softer in {int((d > 1e-9).sum())} frames, firmer in {int((d < -1e-9).sum())}"
          f" (firmer by > 0.02: {int((d < -0.02).sum())})")
    if len(runs):
      print(f"  {len(runs)} stretches, median {np.median(runs):.2f} s, longest {runs.max():.2f} s")
    print(f"  differing frames where a asked <= -0.5: {int((dif & (da[:n, 1] <= -0.5)).sum())}; shouldStop differing: {int((diff[:, 2] > 0).sum())}")
    same_seg = np.diff(da[:n, 0]) < 0.2
    for nm, x in (('a', da), ('b', db)):
      st = np.diff(x[:n, 1])[same_seg]
      print(f"  plan steps per frame ({nm}): down > 0.2: {int((st < -0.2).sum())}, down > 0.3: {int((st < -0.3).sum())}, up > 0.2: {int((st > 0.2).sum())}, largest down {st.min():+.2f}")


if __name__ == '__main__':
  if sys.argv[1] == 'dump':
    dump(sys.argv[2], sys.argv[3], sys.argv[4:])
  else:
    cmp(sys.argv[2], sys.argv[3])
