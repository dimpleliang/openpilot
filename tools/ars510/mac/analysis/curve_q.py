"""Curve time series from qlogs (Mac; 10 Hz carState/controlsState, 4 Hz radarState, drivingModelData for the model):
  python3 curve_q.py <dir> <seg> [seg ...] --start HH:MM:SS     (start = wall clock at the first seg's start)
dir holds <seg>/qlog.zst. Prints every 0.5 s: speed, accel, longitudinalPlan aTarget, SCC-V state, lateral acceleration
(model / desired / actual), steering angle cmd / measured, driver torque, EPS torque, lead distance.
"""
import os, sys, zstandard, numpy as np

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log


def main():
  a = sys.argv[1:]
  start = a[a.index('--start') + 1] if '--start' in a else '00:00:00'
  h, m, s = (int(x) for x in start.split(':')); t_wall0 = h * 3600 + m * 60 + s
  d0, segs = a[0], [x for x in a[1:] if x.isdigit()]
  st = dict(v=0., aego=0., model=np.nan, lat=0, cmd=np.nan, ang=0., drv=0., eps=0., pressed=0, atgt=np.nan, sccv='', lead=np.nan, vlead=np.nan,
            vcruise=np.nan, gas=0, brake=0)
  t0 = None; last = -1
  print('clock     km/h  aEgo aTgt | sccv | latacc model desired actual | cmd° meas° | drv  eps P | lead m (vlead) | cruise')
  for sg in segs:
    dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(os.path.join(d0, sg, 'qlog.zst'), 'rb').read())
    for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
      w = e.which(); t = e.logMonoTime / 1e9
      if w == 'carState':
        c = e.carState; st.update(v=c.vEgo, aego=c.aEgo, ang=c.steeringAngleDeg, drv=c.steeringTorque, eps=c.steeringTorqueEps,
                                  pressed=int(c.steeringPressed), vcruise=c.vCruise, gas=int(c.gasPressed), brake=int(c.brakePressed))
      elif w == 'carControl':
        st['lat'] = int(e.carControl.latActive); st['cmd'] = e.carControl.actuators.steeringAngleDeg
      elif w == 'drivingModelData':
        try: st['model'] = e.drivingModelData.action.desiredCurvature
        except Exception: pass
      elif w == 'longitudinalPlan':
        try: st['atgt'] = e.longitudinalPlan.aTarget
        except Exception: pass
      elif w == 'longitudinalPlanSP':
        try:
          sc = e.longitudinalPlanSP.smartCruiseControl.vision
          st['sccv'] = f'{str(sc.state)[:8]} {sc.vTarget*3.6:.0f}'
        except Exception: st['sccv'] = '?'
      elif w == 'radarState':
        l = e.radarState.leadOne; st['lead'] = l.dRel if l.present else np.nan; st['vlead'] = l.vLead * 3.6 if l.present else np.nan
      elif w == 'controlsState':
        if t0 is None: t0 = t
        if t - last < 0.5: continue
        last = t; c = e.controlsState; v = st['v']; clock = t_wall0 + (t - t0)
        print(f"{int(clock//3600)%24:02d}:{int(clock//60)%60:02d}:{int(clock%60):02d} {v*3.6:5.0f} {st['aego']:5.2f} {st['atgt']:5.2f} | {st['sccv']:12s} | "
              f"{abs(st['model'])*v*v:5.2f} {abs(c.desiredCurvature)*v*v:5.2f} {abs(c.curvature)*v*v:5.2f} | {st['cmd']:6.1f} {st['ang']:6.1f} | "
              f"{st['drv']:5.0f} {st['eps']:5.0f} {'P' if st['pressed'] else '-'}{'L' if st['lat'] else '-'}{'G' if st['gas'] else ''}{'B' if st['brake'] else ''} | "
              f"{st['lead']:5.1f} ({st['vlead']:4.0f}) | {st['vcruise']:.0f}")


if __name__ == '__main__':
  main()
