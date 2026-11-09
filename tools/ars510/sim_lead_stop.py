#!/usr/bin/env python3
# ars510 lead-stop floor replay (selfdrive/controls/lib/ars510_lead_stop.py): open-loop activation scan of logged
# drives + closed-loop sim of engaged stops behind a radar lead with the real long MPC and a PCM lag model.
# Input: npz files from tools/ars510/mac/analysis/leadstop_extract.py. Run on the comma (compiled MPC):
#   PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo python sim_lead_stop.py *.npz
# Parameters can be overridden: LSF_LIFT_CAP=-0.3 ...   QUIET=1 prints only the summaries.
import sys, os, glob, importlib.util
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0, '/data/openpilot')
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib import ars510_lead_stop as lsf
for _k, _v in os.environ.items():
  if _k.startswith('LSF_'):
    setattr(lsf, _k[4:], float(_v)); print('set', _k[4:], _v)
QUIET = os.environ.get('QUIET') == '1'
CT = ModelConstants.T_IDXS[:CONTROL_N]
DT = 0.05
NOLEAD = NS(present=False, radar=False, dRel=0., vLead=0., aLeadK=0., aLeadTau=1.5, modelProb=0., radarTrackId=-1)
PCM_DELAY, PCM_TAU = 0.15, 0.35


def near_stop(lead, v):  # copy of ars510_long_rules.near_stop_accel_floor
  if not (lead.present and lead.radar) or lead.dRel > 20. or lead.vLead > 1.0:
    return None
  if lead.dRel < 3.5 and lead.vLead < 0.3:
    return -1.5
  if v < 0.3:
    return None
  return max(-1.15 * v ** 2 / (2. * max(lead.dRel - 4.5, 0.5)), -2.0)


def load(f):
  z = np.load(f)
  d = z['d']; c = list(z['cols'])
  return {k: d[:, i] for i, k in enumerate(c)}


def lead_at(D, i, drel=None):
  if not (D['present'][i] and D['radar'][i]):
    return NOLEAD
  return NS(present=True, radar=True, dRel=float(D['dRel'][i] if drel is None else drel), vLead=float(D['vLead'][i]),
            aLeadK=float(D['aLeadK'][i]), aLeadTau=float(D['aLeadTau'][i]), modelProb=1.0, radarTrackId=int(D['tid'][i]))


def scan(D, name):
  """open loop: floor episodes on logged data while engaged"""
  F = lsf.LeadStopFloor(DT); eps = []; cur = None
  n = len(D['t'])
  for i in range(n):
    eng = D['long'][i] > 0 and not D['gas'][i] and not D['brk'][i]
    ld = lead_at(D, i)
    f = F.update(ld, D['v'][i], D['t'][i], D['at'][i]) if eng else (F.reset() or None)
    if f is not None and f < D['at'][i] - 0.05:
      if cur is None:
        cur = dict(i0=i, fmin=f, dmin=f - D['at'][i], f0=f)
      cur['fmin'] = min(cur['fmin'], f); cur['dmin'] = min(cur['dmin'], f - D['at'][i]); cur['i1'] = i
    elif cur is not None and i - cur['i1'] > 10:
      eps.append(cur); cur = None
  if cur: eps.append(cur)
  out = []
  for e in eps:
    i0 = e['i0']; t0 = D['t'][i0]
    w = (D['t'] >= t0) & (D['t'] <= t0 + 10.)
    stopped = bool(np.any(D['v'][w] < 0.1))
    took = bool(np.any(D['brk'][w] > 0) or np.any(D['gas'][w] > 0))
    lead_min = float(np.min(np.where(D['present'][w] > 0, D['vLead'][w], 99.)))
    out.append(dict(route=name, seg=int(D['seg'][i0]), t=t0, v=D['v'][i0] * 3.6, d=D['dRel'][i0], vl=D['vLead'][i0] * 3.6,
                    plan=D['at'][i0], fmin=e['fmin'], extra=e['dmin'], dur=D['t'][e['i1']] - t0, stopped=stopped,
                    lead_min=lead_min * 3.6, took=took, logmin=float(np.min(D['at'][w]))))
  return out


def stops(D):
  """engaged stops behind a radar lead: index of the stop, start index 10 s before"""
  res = []; n = len(D['t']); i = 1
  while i < n:
    if D['v'][i] < 0.05 and D['v'][i - 1] >= 0.05:
      j0 = np.searchsorted(D['t'], D['t'][i] - 12.)
      seg = slice(j0, i + 1)
      if (np.all(D['long'][seg] > 0) and not np.any(D['gas'][seg]) and not np.any(D['brk'][seg]) and np.max(D['v'][seg]) > 2.5
          and D['present'][i] and D['radar'][i] and D['dRel'][i] < 15.):
        res.append((j0, i))
      i += 40
    i += 1
  return res


def closed(D, j0, i_stop, use_floor):
  mpc = LongitudinalMpc()
  F = lsf.LeadStopFloor(DT)
  # lead absolute position: x_ego_log + dRel
  t = D['t']; v_log = D['v']
  x_ego_log = np.concatenate([[0.], np.cumsum(np.diff(t) * v_log[1:])])
  x_lead = x_ego_log + D['dRel']
  v = float(v_log[j0]); a = float(D['a'][j0]); x = x_ego_log[j0]; a_prev = float(D['at'][j0])
  cmd_q = [a] * int(round(PCM_DELAY / DT))
  amin = 0.; jmin = 0.; a_hist = []; fl_on = 0
  i_end = min(i_stop + 60, len(t) - 1)
  for i in range(j0, i_end):
    ld = lead_at(D, i, drel=x_lead[i] - x)
    mpc.set_weights(prev_accel_constraint=True)
    mpc.set_cur_state(v, a_prev)
    mpc.update(NS(leadOne=ld, leadTwo=NOLEAD))
    vt = np.interp(CT, T_IDXS_MPC, mpc.v_solution); at = np.interp(CT, T_IDXS_MPC, mpc.a_solution)
    plan = float(get_accel_from_plan(vt, at, CT, action_t=0.1))
    if use_floor:
      f = F.update(ld, v, t[i], plan)
      if f is not None and f < plan:
        plan = f; fl_on += 1
    ns = near_stop(ld, v)
    if ns is not None:
      plan = min(plan, ns)
    a_prev = plan
    cmd_q.append(plan); c = cmd_q.pop(0)
    a_new = a + (c - a) * DT / PCM_TAU
    jmin = min(jmin, (a_new - a) / DT)
    a = a_new
    if v <= 0.05 and a < 0:
      a = 0.; v = 0.
    else:
      v = max(v + a * DT, 0.)
    x += v * DT
    if v > 0.3:
      amin = min(amin, a)
    a_hist.append(a)
  return dict(amin=amin, jmin=jmin, gap=float(x_lead[i_end] - x), fl=fl_on * DT)


if __name__ == '__main__':
  files = sys.argv[1:]
  allep = []; rows = []
  for f in files:
    name = os.path.basename(f)[:10]
    D = load(f)
    allep += scan(D, name)
    for j0, i in stops(D):
      log_amin = float(np.min(np.where(D['v'][j0:i] > 0.3, D['a'][j0:i], 0.)))
      b = closed(D, j0, i, False); n = closed(D, j0, i, True)
      rows.append((name, int(D['seg'][i]), D['t'][i], log_amin, float(D['dRel'][i]), b, n))
  print('== closed loop stops: route seg t | log amin gap | sim base amin jerk gap | floor amin jerk gap on_s')
  for r in ([] if QUIET else rows):
    name, seg, ts, la, lg, b, n = r
    mark = ' <<' if n['amin'] < b['amin'] - 0.05 else (' ++' if n['amin'] > b['amin'] + 0.05 else '')
    print(f"{name} {seg:3d} {ts:9.1f} | {la:+5.2f} {lg:5.1f} | {b['amin']:+5.2f} {b['jmin']:+5.2f} {b['gap']:5.1f} | "
          f"{n['amin']:+5.2f} {n['jmin']:+5.2f} {n['gap']:5.1f} {n['fl']:4.1f}{mark}")
  if rows:
    print('improved', sum(1 for r in rows if r[6]['amin'] > r[5]['amin'] + 0.05), 'worse', sum(1 for r in rows if r[6]['amin'] < r[5]['amin'] - 0.05))
    B = np.array([r[5]['amin'] for r in rows]); N = np.array([r[6]['amin'] for r in rows])
    print(f"stops {len(rows)}  mean amin base {B.mean():+.2f} floor {N.mean():+.2f}  worst base {B.min():+.2f} floor {N.min():+.2f}"
          f"  base<-0.9: {int((B < -0.9).sum())} floor<-0.9: {int((N < -0.9).sum())}")
  print('== open-loop floor episodes (floor firmer than logged plan by >0.05)')
  for e in ([] if QUIET else allep):
    print(f"{e['route']} {e['seg']:3d} {e['t']:9.1f} v{e['v']:5.1f} d{e['d']:5.1f} vl{e['vl']:5.1f} plan{e['plan']:+5.2f} "
          f"fmin{e['fmin']:+5.2f} extra{e['extra']:+5.2f} dur{e['dur']:4.1f} stop{int(e['stopped'])} leadmin{e['lead_min']:5.1f} "
          f"took{int(e['took'])} logmin{e['logmin']:+5.2f}")
  ns = [e for e in allep if not e['stopped'] and e['lead_min'] > 3.]
  bs = sum(-e['extra'] * max(e['dur'], 0.2) for e in ns)
  print(f"nuisance brake-sum {bs:.1f}  worst extra {min([e['extra'] for e in ns] or [0]):+.2f}")
  print(f"episodes {len(allep)}  no-stop & lead never <3 km/h (nuisance?) {len(ns)}  extra<-0.2 among them {sum(1 for e in ns if e['extra'] < -0.2)}")
