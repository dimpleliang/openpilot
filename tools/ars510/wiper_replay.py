#!/usr/bin/env python3
"""ars510: replay the Toyota CarState on rlog CAN (offroad, on the device) and check the wiper decode (0x2F9):
CarStateSP.wiperLevel / wiperMist against the raw byte 0 of 0x2F9, canValid against the logged carState.canValid,
and a few carState fields against the logged ones (the parser change must not touch them).

  python3 wiper_replay.py <root> <rlog.zst> [...]
  <root>: tree with the opendbc version to test at <root>/opendbc_repo/opendbc/... (first on sys.path)
"""
import sys


def main(root, rlogs):
  sys.path[:0] = [root, root + '/opendbc_repo']  # root only: car.capnp from two trees -> duplicate ID
  import zstandard
  from opendbc.car.structs import car
  from openpilot.cereal import custom, log
  import os
  from opendbc.car.toyota.carstate import CarState
  import opendbc.car.toyota.carstate as cs_mod
  print('carstate from', cs_mod.__file__, flush=True)

  pdir = os.environ.get('PARAMS_DIR', '/data/params/d')  # CarParams(SP)Persistent copied from the device
  CP = car.CarParams.from_bytes(open(pdir + '/CarParamsPersistent', 'rb').read()).__enter__()
  CP_SP = custom.CarParamsSP.from_bytes(open(pdir + '/CarParamsSPPersistent', 'rb').read()).__enter__()
  tot = dict(frames=0, valid_mismatch=0, field_mismatch=0, wiper_mismatch=0)
  levels = {}
  dump = []  # every carState field per frame, for an A/B against the unchanged tree (DUMP=<file.json>)
  for f in rlogs:
    cs = CarState(CP, CP_SP)
    cps = cs.get_can_parsers(CP, CP_SP)
    try:
      dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(f, 'rb').read())
      evs = log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61)
    except Exception as ex:
      print('read error', f, repr(ex)); continue
    raw_b0, logged = None, None
    try:
      for e in evs:
        w = e.which()
        if w == 'carState':
          logged = e.carState
        elif w == 'can':
          pk = [(c.address, bytes(c.dat), c.src) for c in e.can]
          for a, d, s in pk:
            if a == 761 and s == 0:
              raw_b0 = d[0]
          for p in cps.values():
            if p is not None:
              p.update([(e.logMonoTime, pk)])
          ret, ret_sp = cs.update(cps)
          tot['frames'] += 1
          ok = all(p is None or p.can_valid for p in cps.values())
          if os.environ.get('DUMP'):
            dump.append(repr((ret, ok)))
          if logged is not None:
            tot['valid_mismatch'] += int(ok != logged.canValid)
            tot['field_mismatch'] += int(abs(ret.vEgoRaw - logged.vEgoRaw) > 1e-3 or ret.gearShifter != logged.gearShifter
                                         or ret.brakePressed != logged.brakePressed or ret.leftBlinker != logged.leftBlinker)
          if raw_b0 is not None:
            exp = ((raw_b0 >> 5) & 3, bool(raw_b0 & 0x10))
            tot['wiper_mismatch'] += int((ret_sp.wiperLevel, ret_sp.wiperMist) != exp)
            levels[exp] = levels.get(exp, 0) + 1
    except Exception as ex:
      print('segment error', f, repr(ex))
    print(f, tot, flush=True)
  if os.environ.get('DUMP'):
    import json; json.dump(dump, open(os.environ['DUMP'], 'w'))
  print('wiper (level, mist) frames:', {k: v for k, v in sorted(levels.items())})
  print('PASS' if tot['frames'] and not tot['wiper_mismatch'] and not tot['field_mismatch'] else 'CHECK', tot)


if __name__ == '__main__':
  main(sys.argv[1], sys.argv[2:])
