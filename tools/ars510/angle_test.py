#!/usr/bin/env python3
"""TEST BRANCH ars510-test-angle: standstill steering-angle test beyond the stock 94.95 deg LTA limit.

Run on the comma while parked: car in D, foot on the brake, lateral (MADS) active, wheel untouched.
  python3 tools/ars510/angle_test.py            # both sides, 90 -> 180 deg
  python3 tools/ars510/angle_test.py --max 130  # stop earlier

controlsd reads the command from /dev/shm/ars510_angle_target while /data/ars510_angle_test exists and
vEgo < 0.5 m/s (see controlsd.py). The script ramps the command in steps, holds each for HOLD_S, logs the
measured angle / EPS torque / faults, and always returns the command to 0 and removes both files on exit.

Abort: driver steering or torque, speed > 3 km/h, lateral inactive, a steer fault, or the wheel not reaching
the step within LAG_S.
"""
import argparse
import os
import time

import openpilot.cereal.messaging as messaging

FLAG = '/data/ars510_angle_test'
TARGET = '/dev/shm/ars510_angle_target'
LOG = '/data/ars510_angle_test.csv'
STEPS = [60, 90, 110, 130, 150, 180]  # deg
HOLD_S = 3.0      # hold each step this long once reached
LAG_S = 4.0       # max time to get within REACH_DEG of a step (50 deg/s rate limit at standstill)
REACH_DEG = 3.0
RAMP_DPS = 30.0   # command ramp, deg/s (below the 50 deg/s LTA rate limit at standstill)
DRIVER_TQ = 60    # driver torque considered as the driver steering


def write_target(a):
  tmp = TARGET + '.tmp'
  with open(tmp, 'w') as f:
    f.write(f'{a:.2f}')
  os.replace(tmp, TARGET)


class Abort(Exception):
  pass


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--max', type=float, default=180.)
  ap.add_argument('--side', choices=['left', 'right', 'both'], default='both')
  args = ap.parse_args()

  sm = messaging.SubMaster(['carState', 'carControl'])
  log = open(LOG, 'w')
  log.write('t,cmd,angle,eps_tq,driver_tq,v,lat_active,steer_fault_tmp,steer_fault_perm\n')
  t0 = time.monotonic()
  cmd = 0.

  def tick():
    sm.update(20)
    cs, cc = sm['carState'], sm['carControl']
    log.write(f'{time.monotonic()-t0:.2f},{cmd:.1f},{cs.steeringAngleDeg:.1f},{cs.steeringTorqueEps:.0f},'
              f'{cs.steeringTorque:.0f},{cs.vEgo:.2f},{int(cc.latActive)},{int(cs.steerFaultTemporary)},'
              f'{int(cs.steerFaultPermanent)}\n')
    if cs.vEgo > 3 / 3.6:
      raise Abort(f'speed {cs.vEgo*3.6:.1f} km/h')
    if cs.steeringPressed or abs(cs.steeringTorque) > DRIVER_TQ:
      raise Abort(f'driver torque {cs.steeringTorque:.0f}')
    if cs.steerFaultTemporary or cs.steerFaultPermanent:
      raise Abort('steer fault (temporary=%d permanent=%d)' % (cs.steerFaultTemporary, cs.steerFaultPermanent))
    if not cc.latActive:
      raise Abort('lateral not active')
    return cs

  def go(target):
    nonlocal cmd
    while abs(cmd - target) > 0.01:  # ramp the command
      cmd += max(-RAMP_DPS * 0.02, min(RAMP_DPS * 0.02, target - cmd))
      write_target(cmd)
      tick()
    t = time.monotonic()
    while True:  # wait for the wheel
      cs = tick()
      if abs(cs.steeringAngleDeg - target) < REACH_DEG:
        break
      if time.monotonic() - t > LAG_S:
        return False, cs.steeringAngleDeg
    t = time.monotonic()
    peak = cs.steeringAngleDeg
    while time.monotonic() - t < HOLD_S:
      cs = tick()
      peak = cs.steeringAngleDeg
    return True, peak

  # start from the current angle so there is no jump
  sm.update(100)
  cmd = sm['carState'].steeringAngleDeg
  write_target(cmd)
  open(FLAG, 'w').close()
  results = []
  try:
    sides = {'left': [1], 'right': [-1], 'both': [1, -1]}[args.side]
    for s in sides:
      for a in [x for x in STEPS if x <= args.max]:
        ok, ang = go(s * a)
        results.append((s * a, ok, ang))
        print(f'cmd {s*a:+5.0f}  measured {ang:+6.1f}  {"OK" if ok else "NOT REACHED"}', flush=True)
        if not ok:
          break
      go(0.)
  except Abort as e:
    print('ABORT:', e, flush=True)
  except KeyboardInterrupt:
    print('ABORT: Ctrl-C', flush=True)
  finally:
    # hand the wheel back: ramp toward 0 if still possible, then remove the override
    try:
      for _ in range(int(abs(cmd) / (RAMP_DPS * 0.02)) + 1):
        cmd -= max(-RAMP_DPS * 0.02, min(RAMP_DPS * 0.02, cmd))
        write_target(cmd)
        sm.update(20)
    except Exception:
      pass
    for p in (FLAG, TARGET):
      try:
        os.remove(p)
      except OSError:
        pass
    log.close()
    print('log:', LOG)
    print('max reached:', max([abs(a) for a, ok, _ in results if ok], default=0))


if __name__ == '__main__':
  main()
