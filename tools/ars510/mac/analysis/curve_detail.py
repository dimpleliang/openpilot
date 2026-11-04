"""Time series of one curve (Mac): model / desired / measured lateral acceleration, steering angle command vs measured,
driver and EPS torque, liveParameters (steer ratio, stiffness, offset), yaw-rate lateral acceleration (livePose, independent
of the vehicle model).   python3 curve_detail.py <route> <seg> [seg2 ...] [--from S --to S]  (seconds inside the first seg)
"""
import os, sys, zstandard, numpy as np

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log


def main():
  args = [a for a in sys.argv[1:]]
  t_from = float(args[args.index('--from') + 1]) if '--from' in args else 0.
  t_to = float(args[args.index('--to') + 1]) if '--to' in args else 1e9
  pos = [a for i, a in enumerate(args) if not a.startswith('--') and (i == 0 or not args[i - 1].startswith('--'))]
  route, segs = pos[0], [int(s) for s in pos[1:]]
  st = dict(v=0., model=np.nan, lat=0, cmd=np.nan, ang=0., drv=0., eps=0., pressed=0, sr=np.nan, stiff=np.nan, off=np.nan, roll=0.,
            yaw=np.nan, des_ang=np.nan, sat=0)
  rows, t0 = [], None
  for seg in segs:
    p = os.path.join(ROOT, 'raw', f'{route}--{seg}', 'rlog.zst')
    d = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
    for m in log.Event.read_multiple_bytes(d, traversal_limit_in_words=2**61):
      w = m.which(); t = m.logMonoTime / 1e9
      if t0 is None: t0 = t
      if w == 'carControl':
        st['lat'] = int(m.carControl.latActive); st['cmd'] = m.carControl.actuators.steeringAngleDeg
      elif w == 'carState':
        c = m.carState; st.update(v=c.vEgo, ang=c.steeringAngleDeg, drv=c.steeringTorque, eps=c.steeringTorqueEps, pressed=int(c.steeringPressed))
      elif w == 'modelV2': st['model'] = m.modelV2.action.desiredCurvature
      elif w == 'liveParameters':
        q = m.liveParameters; st.update(sr=q.steerRatio, stiff=q.stiffnessFactor, off=q.angleOffsetDeg, roll=q.roll)
      elif w == 'livePose': st['yaw'] = m.livePose.angularVelocityDevice.z
      elif w == 'controlsState':
        c = m.controlsState
        if c.lateralControlState.which() == 'angleState':
          a = c.lateralControlState.angleState; st['des_ang'] = a.steeringAngleDesiredDeg; st['sat'] = int(a.saturated)
        rows.append((t - t0, st['v'], st['model'], c.desiredCurvature, c.curvature, st['yaw'], st['lat'], st['cmd'], st['des_ang'], st['ang'],
                     st['drv'], st['eps'], st['pressed'], st['sat'], st['sr'], st['stiff'], st['off'], st['roll']))
  a = np.array(rows, dtype=float)
  a = a[(a[:, 0] >= t_from) & (a[:, 0] <= t_to)]
  print('t(s)  km/h | latacc model desired actual yaw*v | lat cmd° desired° meas° | drv eps press sat | steerRatio stiff offset° roll°')
  last = -1
  for r in a:
    if r[0] - last < 0.5: continue
    last = r[0]; v = r[1]
    print(f'{r[0]:6.1f} {v*3.6:4.0f} | {r[2]*v*v:5.2f} {r[3]*v*v:5.2f} {r[4]*v*v:5.2f} {r[5]*v:5.2f} | {int(r[6])} {r[7]:6.1f} {r[8]:6.1f} {r[9]:6.1f} | '
          f'{r[10]:5.0f} {r[11]:5.0f} {int(r[12])} {int(r[13])} | {r[14]:5.2f} {r[15]:4.2f} {r[16]:5.2f} {np.degrees(r[17]):4.1f}')


if __name__ == '__main__':
  main()
