"""Trace the cut-in rule's state for one track of one segment (Mac, offline).
  python3 dbg_tid.py <seg dir> <tid> [--set NAME=VALUE ...]
"""
import os, sys
import merge_replay as mr
from openpilot.selfdrive.controls.lib import ars510_cutin as rule

seg, tid0 = sys.argv[1], int(sys.argv[2])
sets = {}
for i, x in enumerate(sys.argv):
  if x == '--set': k, v = sys.argv[i + 1].split('=', 1); sets[k] = v
orig_cutin = rule.CutinRule._cutin
orig_update = rule.CutinRule.update
trace = []


def cutin(self, tid, off, d, vrel, frame, fresh, settling, v_ego=99., toward=float('nan'), lc=0):
  r = orig_cutin(self, tid, off, d, vrel, frame, fresh, settling, v_ego, toward, lc)
  if tid == tid0:
    h = self._hist.get(tid, [])
    trace.append(f"  cutin f{frame} fresh {int(fresh)} off {off:.2f} d {d:.1f} vr {vrel:+.1f} tw {toward:+.2f} slow {self._slow.get(tid,0)} "
                 f"fast {self._fast.get(tid,0)} mot {int(self._motion_ok(h, frame, d))} hold {self._hold.get(tid,0)} rel {self._release.get(tid,0)} -> {int(r)}")
  return r


def update(self, car_state, md, points, radar_state, now, frame, fresh, model_updated, path_y, camera_offset):
  out = orig_update(self, car_state, md, points, radar_state, now, frame, fresh, model_updated, path_y, camera_offset)
  for o in out:
    if o.tid == tid0:
      ld = radar_state.leadOne.dRel if radar_state is not None and radar_state.leadOne.present else None
      trace.append(f"{now:.2f} d {o.d:.1f} intr {o.intr:+.2f} tw {o.toward:+.2f} kind {o.kind} strict {o.strict} lead {ld}")
  return out


rule.CutinRule._cutin = cutin
rule.CutinRule.update = update
mr.run_seg((seg, sets))
print('\n'.join(trace))
