#!/usr/bin/env python3
"""ars510: run the real LongitudinalPlanner over an rlog (offroad, on the device) to check that the cut-in response
(CR_*, /data/ars510_cutin_predict) runs without errors and to print when it acts.
  PYTHONPATH=/data/openpilot:/data/openpilot/openpilot:/data/openpilot/opendbc_repo python3 planner_replay_test.py <rlog.zst>
"""
import sys
from collections import defaultdict

import zstandard
import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.cereal import custom, log
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp

SERVICES = ['carControl', 'carState', 'controlsState', 'radarState', 'modelV2', 'selfdriveState', 'liveMapDataSP',
            'carStateSP', 'selfdriveStateSP', 'gpsLocation', 'radarTracks', 'customReservedRawData0']


class SM:
  def __init__(self):
    self.data, self.updated, self.logMonoTime, self.recv_frame, self.frame = {}, defaultdict(bool), {}, defaultdict(int), 0
    self.valid, self.alive = defaultdict(lambda: True), defaultdict(lambda: True)

  def __getitem__(self, k): return self.data[k]
  def all_checks(self, *a): return True
  def all_alive(self, *a): return True
  def all_valid(self, *a): return True


def main():
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParamsPersistent") or params.get("CarParams"), car.CarParams)
  CP_SP = messaging.log_from_bytes(params.get("CarParamsSPPersistent") or params.get("CarParamsSP"), custom.CarParamsSP)
  pl = lp.LongitudinalPlanner(CP, CP_SP)
  pl.cr_off = False
  dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(sys.argv[1], 'rb').read())
  sm = SM(); pend = set(); n = 0; acted = 0; mn = 0.
  for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
    w = e.which()
    if w not in SERVICES:
      continue
    sm.data[w] = getattr(e, w); sm.logMonoTime[w] = e.logMonoTime; pend.add(w)
    if w != 'modelV2' or len(sm.data) < 8:
      continue
    sm.frame += 1
    for k in list(sm.updated): sm.updated[k] = False
    for k in pend: sm.updated[k] = True; sm.recv_frame[k] = sm.frame
    pend = set()
    pl.update(sm)
    n += 1
    if pl.cr_cap is not None:
      acted += 1; mn = min(mn, pl.cr_cap)
  print(f"planner cycles {n}, cut-in response active {acted} cycles ({acted / 20:.1f} s), deepest cap {mn:.2f} m/s^2, no errors")


if __name__ == '__main__':
  main()
