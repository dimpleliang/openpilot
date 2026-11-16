import numpy as np, sys
DT = 0.01; K = 0.000321   # curvature per degree of steering (from the log: cmd -18 deg <-> 0.00578 1/m)
def run(eps='lag', tau=0.45, d=0.10, rmax=25., ky=0.0021, kpsi=0.20, gain=1.0, rate_in=22., rate_out=28., cap=None, dm=0.10, v=12.2, T=30., y0=-0.31, s0=0.049, a0=-32., damp=None, pred=0., pred_err=None, t_mit=1e9, t_rec=1e9, tau_n=0.12, kick=0., t_kick=12., rearm=False):
  n = int(T / DT); y, s, ang, out = y0, s0, a0, a0
  hist_out = [a0] * (int(d / DT) + 1); hist_obs = [(y0, s0)] * (int(dm / DT) + 1); A = []; O = []; Y = []
  flips = []; last_sign = 0; scale = 1.0
  for i in range(n):
    yo, so = hist_obs[0]
    tt = i * DT
    f = 1. if tt < t_mit else max(0., 1. - (tt - t_mit) / 1.0)
    if rearm and tt >= t_kick and tt < t_kick + t_mit: f = 1.
    pd = pred * f
    g_eff = 1. - (1. - gain) * f
    if tt >= t_rec: tau = tau_n; rmax = 60.
    if kick and abs(tt - t_kick) < DT / 2: y += kick
    pdf = pd if i == 0 else pdf + (pd - pdf) * DT / 0.3
    kp = -K * ang
    mdc = g_eff * (ky * (yo + v * so * pdf) + kpsi * (so - v * kp * pdf))
    cmd = -mdc / K
    if damp is not None:   # oscillation damper: count sign flips of the command within damp[0] s, then scale changes
      sg = np.sign(cmd) if abs(cmd) > 3. else 0
      if sg != 0 and sg != last_sign:
        if last_sign != 0: flips.append(i * DT)
        last_sign = sg
      flips = [f for f in flips if i * DT - f < damp[0]]
      scale = damp[2] if len(flips) >= damp[1] else 1.0
      cmd *= scale
    lim = (rate_out if abs(cmd) < abs(out) and np.sign(cmd) == np.sign(out) else rate_in) * DT
    out = float(np.clip(cmd, out - lim, out + lim))
    if cap is not None: out = float(np.clip(out, ang - cap, ang + cap))
    hist_out.append(out); od = hist_out.pop(0)
    if eps == 'lag': ang += (od - ang) / tau * DT
    else: ang += float(np.clip((od - ang) / 0.08, -rmax, rmax)) * DT
    kap = -K * ang                     # right-positive curvature
    s += -v * kap * DT; y += v * s * DT
    hist_obs.append((y, s)); hist_obs.pop(0)
    A.append(ang); O.append(out); Y.append(y)
  A = np.array(A); O = np.array(O); Y = np.array(Y)
  def amp(a, b): q = A[int(a / DT):int(b / DT)]; return (q.max() - q.min()) / 2.
  zc = np.where(np.diff(np.sign(A[int(2 / DT):])) != 0)[0]
  per = 2 * np.mean(np.diff(zc)) * DT if len(zc) > 3 else float('nan')
  x = O[200:1500] - O[200:1500].mean(); z = A[200:1500] - A[200:1500].mean(); best = (0, -9)
  for k in range(0, 120, 2):
    c = float((x[:len(x) - k] * z[k:]).sum() / max(np.sqrt((x[:len(x) - k] ** 2).sum() * (z[k:] ** 2).sum()), 1e-9))
    if c > best[1]: best = (k * DT, c)
  settle = next((i * DT for i in range(n) if np.abs(A[i:]).max() < 3. and np.abs(Y[i:]).max() < 0.15), None)
  return dict(amp2_8=amp(2, 8), amp8_14=amp(8, 14), amp20_30=amp(20, 30), period=per, lag=best[0], ypp=float(np.ptp(Y[int(2 / DT):int(14 / DT)])), settle=settle, first=float(np.abs(A[int(1.5 / DT):int(6 / DT)]).max()), a6_12=amp(6, 12), late=amp(13, 30), ymax=float(np.abs(Y[int(1 / DT):]).max()))
def show(nm, **kw):
  r = run(**kw); print('%-46s amp 2-8s %5.1f  8-14s %5.1f  20-30s %5.1f deg | period %4.1f s | out>ang lag %.2f | lane offset p-p %.2f m | settled %s' % (nm, r['amp2_8'], r['amp8_14'], r['amp20_30'], r['period'], r['lag'], r['ypp'], ('%.1f s' % r['settle']) if r['settle'] is not None else 'never'))

def show2(nm, **kw):
  r = run(**kw); print('%-44s first swing %5.1f deg | 6-12s %5.1f | after 13s %5.1f | max lane offset %.2f m' % (nm, r['first'], r['a6_12'], r['late'], r['ymax']))
plants = [('EPS normal throughout', dict(tau=0.12, d=0.06)), ('EPS slow (lag .50) for 8 s, then normal', dict(tau=0.45, d=0.10, t_rec=8.)), ('EPS slow (lag .60) for 8 s, then normal', dict(tau=0.55, d=0.10, t_rec=8.)),
          ('EPS slow (lag .50) all the time', dict(tau=0.45, d=0.10)), ('wheel max 25 deg/s + .40 delay for 8 s, then normal', dict(eps='rl', d=0.40, rmax=25., t_rec=8.))]
mits = [('no change (today)', {}), ('look-ahead +0.4 s for 5 s', dict(pred=0.4, t_mit=5.)), ('look-ahead +0.6 s for 5 s', dict(pred=0.6, t_mit=5.)), ('correction x0.5 for 5 s', dict(gain=0.5, t_mit=5.)), ('correction x0.7 for 5 s', dict(gain=0.7, t_mit=5.)),
        ('+0.4 s and x0.7 for 5 s', dict(pred=0.4, gain=0.7, t_mit=5.)), ('+0.4 s and x0.5 for 5 s', dict(pred=0.4, gain=0.5, t_mit=5.)), ('+0.4 s and x0.7 for 8 s', dict(pred=0.4, gain=0.7, t_mit=8.)), ('+0.4 s and x0.7 for 3 s', dict(pred=0.4, gain=0.7, t_mit=3.))]
print('start: the state at the driver release on 11/16 (wheel -32 deg, 0.31 m off, heading 0.049 rad)')
for pn, pk in plants:
  print('--', pn)
  for mn, mk in mits: show2('  ' + mn, **pk, **mk)
