"""List the planner's cut-in response events (cloudlog "ARS510 cut-in response on/off") of routes, with the car's speed and
acceleration while it acted and whether the driver touched gas / brake / steering (Mac, offline).
  python3 cr_events.py <route> [<route> ...]
"""
import glob, json, os, sys, zstandard

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log


def main():
  for route in sys.argv[1:]:
    segs = sorted(glob.glob(os.path.join(ROOT, 'raw', route + '--*')), key=lambda s: int(s.rsplit('--', 1)[1]))
    t0 = None; cur = None; n = 0
    for s in segs:
      p = s + '/rlog.zst'
      if not os.path.exists(p): continue
      try: dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
      except Exception: continue
      seg = int(s.rsplit('--', 1)[1])
      try:
        for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
          w = e.which(); t = e.logMonoTime / 1e9
          if t0 is None: t0 = t
          if w == 'logMessage':
            try: msg = json.loads(e.logMessage).get('msg', '')
            except Exception: msg = str(e.logMessage)
            if 'ARS510 cut-in response on' in str(msg):
              cur = dict(seg=seg, t=t, msg=str(msg).replace('ARS510 cut-in response on: ', ''), v=[], a=[], gas=0, brake=0, steer=0)
            elif 'ARS510 cut-in response off' in str(msg) and cur:
              n += 1
              dur = t - cur['t']
              print(f"{route} seg {cur['seg']:3d} +{cur['t'] - t0:7.1f}s {dur:4.1f}s | {cur['msg']} | v {min(cur['v'] or [0]):.0f}-{max(cur['v'] or [0]):.0f} "
                    f"a_min {min(cur['a'] or [0]):+.2f} | gas {cur['gas']} brake {cur['brake']} steer {cur['steer']}")
              cur = None
          elif w == 'carState' and cur:
            c = e.carState; cur['v'].append(c.vEgo * 3.6); cur['a'].append(c.aEgo)
            cur['gas'] |= int(c.gasPressed); cur['brake'] |= int(c.brakePressed); cur['steer'] |= int(c.steeringPressed)
      except Exception:
        pass
    print(f'{route}: {n} response events')


if __name__ == '__main__':
  main()
