"""ars510: lead-stop floor -- brake early and evenly behind a radar lead that is slowing to a stop.

11/09 08:50 (route 64): behind a car slowing at a junction, 11 m at 12 km/h. The radar Kalman's aLeadK showed the
lead's first slowing (-0.47) but decayed to ~0 for 3 s while the lead kept slowing; when the lead then braked to a
stop, aLeadK grew only slowly and its short aLeadTau told the MPC the deceleration would fade, so the plan stayed ~0
until 1 s before, then ramped -0.12 -> -1.00 in 2 s and the car peaked at -1.16. A constant deceleration to the
lead's predicted stopping point needed only about -0.6 from the moment the lead started braking.

A driver watches the lead's speed drop and brakes evenly to stop a gap behind where the lead will stop. So:
  a_lead  = min(aLeadK, slope of vLead over the last WIN s of the same radar track)   (no fade assumed)
  x_stop  = dRel + vLead^2 / (2 |a_lead|)                                            (where the lead stops)
  floor   = K * -v_ego^2 / (2 (x_stop - GAP))                                        (even braking to GAP behind it)
Only for a radar lead within D_MAX, slower than VLEAD_MAX, clearly slowing (a_lead <= A_LEAD_MIN), predicted to stop
within X_STOP_MAX, and while I close on it (or it brakes hard, A_HARD). While the lead is still above LIFT_VLEAD the
floor is capped at LIFT_CAP (a lift / light brake: in stop-and-go the lead often slows and goes again); the full floor
once the lead is almost stopped. The floor engages below ON and releases above OFF; its onset is jerk-limited (JERK)
from the current plan so it starts as a smooth lift. Braking only gets firmer (the planner takes the min).
Off: /data/ars510_no_lead_stop. cloudlog "ARS510 lead stop on/off".

Replay (tools/ars510/sim_lead_stop.py, routes 57-64, closed loop with the real MPC and a PCM lag model): 34 engaged
stops behind a radar lead, 9 softer (08:50: peak -1.76 -> -0.83 in the sim), none firmer; open loop 60 episodes
without a following stop, worst extra braking -0.52 for about 1 s (lifts while the lead slows in stop-and-go).
"""
from collections import deque

D_MAX = 30.          # m
VLEAD_MAX = 12.      # m/s
WIN = 1.0            # s, vLead slope window
WIN_MIN = 0.7        # s of the same track needed before the slope is trusted
A_LEAD_MIN = -0.25   # m/s^2, lead clearly slowing
X_STOP_MAX = 45.     # m, the lead is predicted to stop within this distance of me
GAP = 4.5            # m, = STOP_DISTANCE_LOW / NEAR_STOP_GAP
K = 1.0
ON = -0.20           # m/s^2, engage when the floor is firmer than this
OFF = -0.10          # m/s^2, release when weaker than this
MAX_DECEL = -1.5     # m/s^2
LIFT_CAP = -0.40     # m/s^2, while the lead is still above LIFT_VLEAD only a lift / light brake (stop-and-go traffic
LIFT_VLEAD = 2.0     # m/s    often slows and goes again); the full floor once the lead is almost stopped
CLOSE_MIN = 0.3      # m/s, only while I am closing on the lead (v_ego - vLead) ...
A_HARD = -0.6        # m/s^2, ... or the lead brakes at least this hard
JERK = 1.2           # m/s^3, onset ramp
OFF_FILE = "/data/ars510_no_lead_stop"


class LeadStopFloor:
  def __init__(self, dt=0.05):
    self.dt = dt
    self.hist = deque()   # (t, vLead)
    self.tid = None
    self.active = False
    self.out = 0.0        # ramped floor while active
    self.dbg = None

  def reset(self):
    self.hist.clear()
    self.tid = None
    self.active = False

  def _slope(self, t):
    while self.hist and t - self.hist[0][0] > WIN:
      self.hist.popleft()
    if len(self.hist) < 3 or self.hist[-1][0] - self.hist[0][0] < WIN_MIN:
      return None
    n = len(self.hist)
    mt = sum(h[0] for h in self.hist) / n
    mv = sum(h[1] for h in self.hist) / n
    den = sum((h[0] - mt) ** 2 for h in self.hist)
    if den <= 0.:
      return None
    return sum((h[0] - mt) * (h[1] - mv) for h in self.hist) / den

  def raw(self, lead, v_ego, t):
    """Unramped floor or None; also updates the vLead history."""
    if not (lead.present and lead.radar):
      self.reset()
      return None
    tid = int(getattr(lead, 'radarTrackId', -1))
    if tid != self.tid:
      self.hist.clear()
      self.tid = tid
    self.hist.append((t, float(lead.vLead)))
    slope = self._slope(t)
    if slope is None or lead.dRel > D_MAX or lead.vLead > VLEAD_MAX:
      return None
    a_lead = min(float(lead.aLeadK), slope)
    if a_lead > A_LEAD_MIN or (v_ego - lead.vLead < CLOSE_MIN and a_lead > A_HARD):
      return None
    x_stop = float(lead.dRel) + max(float(lead.vLead), 0.) ** 2 / (2. * -a_lead)
    if x_stop > X_STOP_MAX:
      return None
    room = max(x_stop - GAP, 0.5)
    self.dbg = (a_lead, x_stop)
    f = max(-K * v_ego ** 2 / (2. * room), MAX_DECEL)
    if lead.vLead > LIFT_VLEAD:
      f = max(f, LIFT_CAP)
    return f

  def update(self, lead, v_ego, t, a_plan):
    """Floor to apply (min with the plan) or None."""
    f = self.raw(lead, v_ego, t)
    if f is None or (self.active and f > OFF) or (not self.active and f > ON):
      self.active = False
      return None
    if not self.active:
      self.active = True
      self.out = min(a_plan, 0.)
    self.out = max(f, self.out - JERK * self.dt) if f < self.out else f
    return self.out
