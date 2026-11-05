"""Quick review of one drive from qlogs (Mac): cut-in response / confirmed lift-off events (cloudlog), SCC-V state use,
curves (lateral accel >= CURVE_MIN) with speed, SCC-V state, gas / steering by the driver, and disengagements.
  python3 q_review.py <dir with <seg>/qlog.zst> --start HH:MM:SS   (wall clock at seg 0's start)
"""
import collections, glob, json, os, sys, zstandard

ROOT = os.path.expanduser('~/ars510_rlogs')
sys.path[:0] = [os.path.join(ROOT, 'op'), os.path.join(ROOT, 'op', 'opendbc_repo')]
from openpilot.cereal import log

CURVE_MIN = 2.0


def hms(s):
  h, m, x = (int(v) for v in s.split(':')); return h * 3600 + m * 60 + x


def clk(t):
  return f'{int(t // 3600) % 24:02d}:{int(t // 60) % 60:02d}:{int(t % 60):02d}'


def main():
  d0 = sys.argv[1]; w0 = hms(sys.argv[sys.argv.index('--start') + 1])
  segs = sorted((int(os.path.basename(p)) for p in glob.glob(os.path.join(d0, '*')) if os.path.basename(p).isdigit()))
  t0 = None; st = dict(v=0., gas=0, press=0, lat=0, en=False, sccv='?')
  sccv = collections.Counter(); events = []; curves = []; cur = None; dis = []; last_en = None
  for sg in segs:
    p = os.path.join(d0, str(sg), 'qlog.zst')
    if not os.path.exists(p): continue
    try: dat = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read())
    except Exception: continue
    try:
      for e in log.Event.read_multiple_bytes(dat, traversal_limit_in_words=2**61):
        w = e.which(); t = e.logMonoTime / 1e9
        if t0 is None and w == 'controlsState': t0 = t - 60 * sg
        if t0 is None: continue
        tw = w0 + t - t0
        if w == 'carState':
          c = e.carState; st.update(v=c.vEgo, gas=int(c.gasPressed), press=int(c.steeringPressed))
        elif w == 'selfdriveState':
          en = e.selfdriveState.enabled
          if last_en is True and not en: dis.append((clk(tw), round(st['v'] * 3.6)))
          last_en = en; st['en'] = en
        elif w == 'longitudinalPlanSP':
          try: st['sccv'] = str(e.longitudinalPlanSP.smartCruiseControl.vision.state)
          except Exception: pass
          if st['en']: sccv[st['sccv']] += 1
        elif w == 'logMessage':
          s = str(e.logMessage)
          if 'ARS510 cut-in' in s:
            try: msg = json.loads(s).get('msg', s)
            except Exception: msg = s
            events.append((clk(tw), round(st['v'] * 3.6), str(msg)[:170]))
        elif w == 'controlsState':
          v = st['v']; a = abs(e.controlsState.curvature) * v * v
          if a >= CURVE_MIN and st['en']:
            if cur is None: cur = dict(t=tw, amax=0., v=[], gas=0, press=0, sccv=collections.Counter())
            cur['amax'] = max(cur['amax'], a); cur['v'].append(v * 3.6); cur['gas'] |= st['gas']; cur['press'] |= st['press']
            cur['sccv'][st['sccv']] += 1; cur['end'] = tw
          elif cur is not None and tw - cur['end'] > 1.0:
            curves.append(cur); cur = None
    except Exception:
      pass
  print(f'segments {len(segs)}; SCC-V states while engaged: {dict(sccv)}')
  print(f'disengagements: {dis}')
  print('cut-in events:')
  for x in events: print('  ', *x)
  print(f'curves >= {CURVE_MIN} m/s^2 (engaged): {len(curves)}')
  for c in sorted(curves, key=lambda c: -c['amax'])[:20]:
    print(f"  {clk(c['t'])} {c['end'] - c['t']:4.1f}s  max {c['amax']:.2f}  v {min(c['v']):.0f}-{max(c['v']):.0f}  "
          f"gas {c['gas']} press {c['press']}  SCC-V {dict(c['sccv'])}")


if __name__ == '__main__':
  main()
