"""
ARS510: draw every object the radar tracks, including cars in the next lanes.

opendbc/car/toyota/ars510_radar_interface.py publishes the radar's settled objects (before its path gates) on
customReservedRawData0. Each one gets a small marker with "R <distance>" and its speed. Green: within LANE_HALF_M of the
model's predicted path at that distance (my lane); red: outside it (next lanes). The colour is the lane only; radard's
lead choice does not depend on it. A track still settling (younger than the decoder's publish age, ~1.2 s in fused; its
range and speed have not converged) is drawn faint with its distance only. Pedestrians and two-wheelers (radar class) get a label. Display only: nothing here feeds the controls. A track that already carries a radar lead chevron is skipped, so the lead keeps one label.
"""
import os

import numpy as np

from openpilot.selfdrive.controls.lib import ars510_cutin as rule
from openpilot.selfdrive.controls.lib.ars510_cutin import SERVICE, parse, parse_sizes  # noqa: F401  (parse: tools import it from here)
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

STALE_FRAMES = 10  # ui frames (20 Hz) without a radar record before the markers go away
LANE_HALF_M = 1.8  # half a lane: an object's lateral distance from the predicted path below this is in my lane

COLOR_PATH = (60, 220, 120)
COLOR_SIDE = (255, 60, 60)  # next lanes: red (cyan was hard to see)
SHOW_ALL_TRACKS = False  # False: draw only the cut-in / merge markers (10/31 driver)
COLOR_CUTIN = (255, 190, 0)  # amber
MARKS_OFF_FILE = "/data/ars510_no_cutin_marks"  # touch to hide the merge marks even while engaged
# The cut-in rule itself (what is marked, and when) lives in controls/lib/ars510_cutin.py since 11/04, shared with the planner's
# response: an amber circle (steady = predicted, flashing = confirmed) is shown exactly while the planner may react to it.
CONFIRM_PULSE_HZ = 3.0
CUTIN_PULSE_HZ = 2.5
SHOW_BANNER = False  # 10/31 driver: no edge banner ("右側匯入／左側匯入"), the arrows on the car are enough
CUTIN_SCALE = 1.5
CUTIN_MIN_SIZE = 24.0
BANNER_FONT = 56
# 11/04 driver: a lamp at the bottom left (right of the driver-monitoring icon) for LAMP_HOLD_S whenever the cut-in response's
# conditions hold (ars510_cutin.CutinResponse: a marked object that needs me to slow down), shown whether or not the planner's
# response is switched on (/data/ars510_cutin_predict). Amber = a lift-off is enough, red = light braking is needed.
LAMP_HOLD_S = 3.0
LAMP_R = 46
LAMP_X = 30 + 192 + 40 + LAMP_R      # UI_BORDER_SIZE + driver-monitoring icon (BTN_SIZE) + gap
LAMP_Y_FROM_BOTTOM = 30 + 192 // 2   # same height as the driver-monitoring icon's centre
COLOR_LAMP_LIFT = (255, 190, 0)
COLOR_LAMP_BRAKE = (255, 60, 40)
# 11/11 driver: a box on the lead only while the radar has it (radarState.leadOne.radar), so no box = camera only (11/11
# 15:48: 8 s without a radar lead after a double lane change, then the jam tail). The lead chevron stays as it is.
# A thin dashed amber box marks the radar-only far object the planner may coast for (radarTracks FAR_OBJ_TRACK_ID; the
# gate of ars510_long_rules.FarLeadEarly.far_object, copied here because that module pulls in the MPC).
# Lead box colour: white; amber while the plan asks for BOX_LIFT or less; red for BOX_BRAKE or less (a colour is kept
# BOX_COLOR_HOLD_S so it does not flicker). Only while engaged, like the merge marks.
# The box is a car-sized rectangle (BOX_W_M x BOX_H_M) placed at the radar's range and lateral position, not a camera
# detection: far away it can sit a little off the car, and it is never drawn narrower than BOX_MIN_W.
# Display only. Off: /data/ars510_no_lead_box.
BOX_OFF_FILE = "/data/ars510_no_lead_box"
BOX_W_M, BOX_H_M = 1.9, 1.5    # when the radar object is not found / has no size yet
# 11/11 driver: a more exact box. The object list carries the radar's own width and length for each object and its
# class; the box takes the width from the radar and the height from the class (the radar does not measure height):
# car 1.5 m, large vehicle 2.4 m (shorter than BOX_LONG_M: van, small truck) or 3.3 m (bus, truck), two-wheeler 1.6 m,
# pedestrian 1.7 m. 11/11 route, 11 highway minutes, settled objects: cars 1.7-1.9 m wide (median 1.8), large vehicles
# 1.8-2.4 m (median 2.1), the same at 20 m and at 120 m; within one track the width moves 0.2-0.3 m, so it is smoothed.
# The lead / far object is paired with the object-list entry within BOX_MATCH_D_M / BOX_MATCH_Y_M of it.
BOX_MATCH_D_M, BOX_MATCH_Y_M = 4.0, 1.5
BOX_CLASS_SIZE = {2: (1.85, 1.5), 3: (2.3, 2.4), 4: (0.6, 1.7), 6: (0.8, 1.6)}   # class -> (width without a radar size, height) m
BOX_LONG_M, BOX_LONG_H_M = 6.5, 3.3
BOX_W_RANGE = {2: (1.5, 2.2), 3: (1.7, 2.6), 4: (0.4, 1.0), 6: (0.5, 1.2)}        # radar width accepted per class
BOX_SIZE_TAU_S = 0.8
BOX_MIN_W = 44.0               # px
BOX_THICK = 5.0                # px
BOX_LIFT, BOX_BRAKE = -0.25, -1.5   # m/s^2, longitudinalPlan.aTarget
BOX_COLOR_HOLD_S = 0.6
COLOR_BOX = (255, 255, 255)
FAR_OBJ_TRACK_ID = 1 << 28     # = ars510_radar_interface.FAR_OBJ_TRACK_ID
FAR_OBJ_MAX_CLOSING = 20.      # m/s, = ars510_long_rules.FAR_OBJ_MAX_CLOSING
FAR_OBJ_MIN_V = 70. / 3.6      # = ars510_long_rules.FAR_RADAR_ONLY_MIN_V
FAR_OBJ_PATH_Y_BP = [80., 150.]   # = ars510_long_rules.FAR_OBJ_PATH_Y_*
FAR_OBJ_PATH_Y_V = [2.2, 3.0]
FAR_OBJ_HOLD_FRAMES = 10       # ui frames (0.5 s), = ars510_long_rules.FAR_OBJ_HOLD
FAR_OBJ_SAME_LEAD_M = 15.      # the far object within this range of the boxed radar lead is that car: no second box
FAR_BOX_THICK = 3.0
FAR_BOX_DASH, FAR_BOX_GAP = 12.0, 9.0

# 11/11 driver: a pedestrian the radar reports while I drive PED_MIN_V or faster gets a solid pink marker, wherever it
# is (roadside included) and whether or not openpilot is engaged: a person next to a fast road is worth a look.
# The radar's class 4, settled tracks only, kept PED_HOLD_FRAMES after the last report. A person-sized filled box
# (PED_W_M x PED_H_M at the radar's position, at least PED_MIN_W px wide). 35 h of logs: no such object at 75 km/h or
# more, 8 at 60-75 km/h (all 4-11 m to the side, standing still, under 1.5 s; not checked against the video, some
# may be posts). Display only. Off: /data/ars510_no_ped_mark.
PED_OFF_FILE = "/data/ars510_no_ped_mark"
PED_CLASS = 4
PED_MIN_V = 70. / 3.6
PED_W_M, PED_H_M = 0.6, 1.7
PED_MIN_W = 40.0               # px
PED_HOLD_FRAMES = 10           # ui frames (0.5 s)
COLOR_PED = (255, 80, 170)     # pink

T_FOLLOW_DEFAULT = 1.45
T_FOLLOW_BY_PERSONALITY = {"relaxed": 2.0, "standard": 1.45, "aggressive": 1.25}   # = long_mpc.get_T_FOLLOW (not imported here)

# Radar object class (163|3) labels for vulnerable road users; validated on route 0000000e (7/9 pedestrians, 5/6
# two-wheelers on sampled frames). Shown once the track has settled. Chinese only where the UI font carries the glyphs.
CLASS_LABELS = {4: ("行人", "PED"), 6: ("機車", "MOTO")}


def _label(cls: int, settling: bool) -> str:
  if settling or cls not in CLASS_LABELS:
    return ""
  try:
    from openpilot.system.ui.lib.multilang import multilang
    zh = multilang.language == "zh-CHT"
  except Exception:
    zh = False
  return CLASS_LABELS[cls][0 if zh else 1] + " "


def _cutin_label() -> str:
  try:
    from openpilot.system.ui.lib.multilang import multilang
    return "併入 " if multilang.language == "zh-CHT" else "CUT-IN "
  except Exception:
    return "CUT-IN "


class Ars510Tracks:
  def __init__(self):
    self._marks_off = os.path.exists(MARKS_OFF_FILE)
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._points: list[tuple] = []
    self._rule = rule.CutinRule()
    self._resp = rule.CutinResponse()
    self._lamp_until = 0.
    self._lamp_brake = False
    self._box_off = os.path.exists(BOX_OFF_FILE)
    self._box_lvl = 0
    self._box_lvl_until = 0.
    self._far: tuple[float, float] | None = None   # (dRel, y right of the camera) of the radar-only far object
    self._far_frame = 0
    self._sizes: list[tuple] = []
    self._ped_off = os.path.exists(PED_OFF_FILE)
    self._peds: list[tuple] = []                   # (dRel, y left) of the radar's pedestrians
    self._peds_frame = 0
    self._box_size: dict[str, list] = {}           # "lead" / "far" -> [d, width, height, time]

  def draw(self, sm, map_to_screen, path_z, path_y, camera_offset: float, path_offset_z: float, radar_state, rect: rl.Rectangle):
    try:
      if SERVICE in sm.data and sm.updated[SERVICE]:
        self._sizes = parse_sizes(bytes(sm[SERVICE]))
        peds = [(o[0], o[1]) for o in self._sizes if o[3] == PED_CLASS and not o[2]]
        if peds:
          self._peds, self._peds_frame = peds, sm.frame
      if self._peds and sm.frame - self._peds_frame > PED_HOLD_FRAMES:
        self._peds = []
      if self._peds and not self._ped_off and sm['carState'].vEgo >= PED_MIN_V:
        for d, y_left in self._peds:
          r = self._box_rect(map_to_screen, d, -y_left + camera_offset, path_z(d) + path_offset_z, PED_W_M, PED_H_M, PED_MIN_W)
          if r is not None:
            rl.draw_rectangle_rec(rl.Rectangle(r.x - 3, r.y - 3, r.width + 6, r.height + 6), rl.Color(0, 0, 0, 170))
            rl.draw_rectangle_rec(r, rl.Color(*COLOR_PED, 245))
    except Exception:
      self._peds = []
    try:
      if not self._box_off and "selfdriveState" in sm.data and sm["selfdriveState"].enabled:
        self._draw_boxes(sm, map_to_screen, path_z, path_y, camera_offset, path_offset_z, radar_state)
      else:
        self._far = None
    except Exception:
      self._far = None
    try:
      if SERVICE not in sm.data:
        return
      if sm.updated[SERVICE]:
        self._points = parse(bytes(sm[SERVICE]))
      # 11/02 driver: while I drive myself the merge marks are a distraction with no effect; draw them only while
      # openpilot is engaged. MARKS_OFF_FILE hides them altogether. Not drawing (or no fresh data): forget the rule state.
      if self._marks_off or not ("selfdriveState" in sm.data and sm["selfdriveState"].enabled) or not self._points \
         or sm.frame - sm.recv_frame[SERVICE] > STALE_FRAMES:
        self._rule.reset()
        return
      objs = self._rule.update(sm['carState'], sm['modelV2'], self._points, radar_state, rl.get_time(), sm.frame,
                               sm.updated[SERVICE], sm.updated['modelV2'], path_y, camera_offset)
      v_ego = sm['carState'].vEgo
      merging = []
      for o in objs:
        pt = map_to_screen(o.d, o.y_right, path_z(o.d) + path_offset_z)
        if pt is None:
          continue
        if o.kind is not None:
          # predicted: steady solid circle with the arrow inside; confirmed (really cut in, slower): flashing (10/31 driver)
          self._draw_confirmed(pt, o.d, o.arrow, flash=o.kind == "confirmed")
          if o.kind == "predicted":
            merging.append((o.d, -o.arrow))
        elif SHOW_ALL_TRACKS:
          self._draw_marker(pt, o.d, o.vrel + v_ego, COLOR_PATH if o.in_lane else COLOR_SIDE, rect, o.settling, o.cls, False, 0)
      if merging and SHOW_BANNER:
        self._draw_banner(min(merging), rect)
      now = rl.get_time()
      t_follow = T_FOLLOW_BY_PERSONALITY.get(str(sm['selfdriveState'].personality), T_FOLLOW_DEFAULT)
      best = self._resp.update(objs, v_ego, t_follow, now)
      if best is not None:
        brake = best[0] > rule.lift_max(v_ego) + 1e-3
        self._lamp_brake = brake or (self._lamp_brake and now < self._lamp_until)
        self._lamp_until = now + LAMP_HOLD_S
      if now < self._lamp_until:
        self._draw_lamp(rect, self._lamp_brake)
    except Exception:
      self._points = []

  def _box_size_m(self, key: str, d: float, y_left: float, now: float) -> tuple[float, float]:
    """(width, height) in metres for the box at (d, y_left): the paired radar object's width and its class height."""
    w, h = BOX_W_M, BOX_H_M
    best = None
    for (od, oy, settling, cls, ow, ol) in self._sizes:
      if settling or abs(od - d) > BOX_MATCH_D_M or abs(oy - y_left) > BOX_MATCH_Y_M:
        continue
      cost = abs(od - d) / BOX_MATCH_D_M + abs(oy - y_left) / BOX_MATCH_Y_M
      if best is None or cost < best[0]:
        best = (cost, cls, ow, ol)
    if best is not None and best[1] in BOX_CLASS_SIZE:
      _, cls, ow, ol = best
      w, h = BOX_CLASS_SIZE[cls]
      lo, hi = BOX_W_RANGE[cls]
      if lo <= ow <= hi:
        w = ow
      if cls == 3 and ol >= BOX_LONG_M:
        h = BOX_LONG_H_M
    prev = self._box_size.get(key)
    if prev is not None and abs(prev[0] - d) < 2 * BOX_MATCH_D_M and 0 <= now - prev[3] < 1.0:   # the same car: smooth
      k = min((now - prev[3]) / BOX_SIZE_TAU_S, 1.0)
      w, h = prev[1] + k * (w - prev[1]), prev[2] + k * (h - prev[2])
    self._box_size[key] = [d, w, h, now]
    return w, h

  @staticmethod
  def _box_rect(map_to_screen, d: float, y_right: float, z: float, w_m: float = BOX_W_M, h_m: float = BOX_H_M, min_w: float | None = None):
    """Screen rectangle of a w_m x h_m box standing on the road at (d, y_right), or None when it is off screen."""
    p = map_to_screen(d, y_right, z)
    if p is None:
      return None
    q = map_to_screen(d, y_right + 1.0, z) or map_to_screen(d, y_right - 1.0, z)
    if q is None:
      return None
    w = max(w_m * abs(q[0] - p[0]), BOX_MIN_W * min(w_m / BOX_W_M, 1.0) if min_w is None else min_w)
    h = w * h_m / w_m
    return rl.Rectangle(p[0] - w / 2, p[1] - h, w, h)

  @staticmethod
  def _draw_box(r, rgb, thick: float, dashed: bool):
    color = rl.Color(*rgb, 235)
    shadow = rl.Color(0, 0, 0, 150)
    if not dashed:
      rl.draw_rectangle_lines_ex(rl.Rectangle(r.x - 2, r.y - 2, r.width + 4, r.height + 4), thick + 4, shadow)
      rl.draw_rectangle_lines_ex(r, thick, color)
      return
    corners = [(r.x, r.y), (r.x + r.width, r.y), (r.x + r.width, r.y + r.height), (r.x, r.y + r.height)]
    for i in range(4):
      (x0, y0), (x1, y1) = corners[i], corners[(i + 1) % 4]
      length = abs(x1 - x0) + abs(y1 - y0)
      ux, uy = (x1 - x0) / length, (y1 - y0) / length
      s = 0.
      while s < length:
        e = min(s + FAR_BOX_DASH, length)
        a, b = rl.Vector2(x0 + ux * s, y0 + uy * s), rl.Vector2(x0 + ux * e, y0 + uy * e)
        rl.draw_line_ex(a, b, thick + 3, shadow)
        rl.draw_line_ex(a, b, thick, color)
        s += FAR_BOX_DASH + FAR_BOX_GAP

  def _draw_boxes(self, sm, map_to_screen, path_z, path_y, camera_offset: float, path_offset_z: float, radar_state):
    now = rl.get_time()
    a_target = sm['longitudinalPlan'].aTarget if 'longitudinalPlan' in sm.data else 0.
    lvl = 2 if a_target <= BOX_BRAKE else 1 if a_target <= BOX_LIFT else 0
    if lvl >= self._box_lvl:
      self._box_lvl, self._box_lvl_until = lvl, now + BOX_COLOR_HOLD_S
    elif now >= self._box_lvl_until:
      self._box_lvl = lvl
    lead = radar_state.leadOne if radar_state is not None else None
    lead_d = None
    if lead is not None and lead.present and lead.radar:
      lead_d = lead.dRel
      w_m, h_m = self._box_size_m("lead", lead.dRel, lead.yRel, now)
      r = self._box_rect(map_to_screen, lead.dRel, -lead.yRel + camera_offset, path_z(lead.dRel) + path_offset_z, w_m, h_m)
      if r is not None:
        self._draw_box(r, (COLOR_BOX, COLOR_CUTIN, COLOR_LAMP_BRAKE)[self._box_lvl], BOX_THICK, False)
    if 'radarTracks' not in sm.data:
      return
    if sm.updated['radarTracks']:
      v_ego = sm['carState'].vEgo
      for p in sm['radarTracks'].points:
        if p.trackId != FAR_OBJ_TRACK_ID:
          continue
        y_right = -p.yRel + camera_offset   # path_y() is in the same frame (model y + camera offset)
        if v_ego > FAR_OBJ_MIN_V and -p.vRel <= FAR_OBJ_MAX_CLOSING and \
           abs(y_right - path_y(p.dRel)) < float(np.interp(p.dRel, FAR_OBJ_PATH_Y_BP, FAR_OBJ_PATH_Y_V)):
          self._far, self._far_frame = (float(p.dRel), y_right), sm.frame
        break
    if self._far is not None and sm.frame - self._far_frame > FAR_OBJ_HOLD_FRAMES:
      self._far = None
    if self._far is not None and (lead_d is None or abs(lead_d - self._far[0]) > FAR_OBJ_SAME_LEAD_M):
      w_m, h_m = self._box_size_m("far", self._far[0], -(self._far[1] - camera_offset), now)
      r = self._box_rect(map_to_screen, self._far[0], self._far[1], path_z(self._far[0]) + path_offset_z, w_m, h_m)
      if r is not None:
        self._draw_box(r, COLOR_CUTIN, FAR_BOX_THICK, True)

  def _draw_lamp(self, rect: rl.Rectangle, brake: bool):
    """Bottom-left lamp: a filled circle with a bold down chevron (slow down); red when light braking is needed."""
    x = rect.x + LAMP_X
    y = rect.y + rect.height - LAMP_Y_FROM_BOTTOM
    rgb = COLOR_LAMP_BRAKE if brake else COLOR_LAMP_LIFT
    rl.draw_circle(int(x), int(y + 3), LAMP_R + 4, rl.Color(0, 0, 0, 150))
    rl.draw_circle(int(x), int(y), LAMP_R, rl.Color(*rgb, 240))
    dark = rl.Color(20, 20, 20, 255)
    th = 11.0
    for dy in (-14, 10):
      rl.draw_line_ex(rl.Vector2(x - 24, y + dy - 10), rl.Vector2(x, y + dy + 10), th, dark)
      rl.draw_line_ex(rl.Vector2(x, y + dy + 10), rl.Vector2(x + 24, y + dy - 10), th, dark)

  def _draw_confirmed(self, pt, d: float, direction: int, flash: bool = True):
    """Cut-in marker: a solid amber circle on the car with a dark arrow inside pointing into my lane (the way it moves).
    Steady while predicted; flashing once the car has really cut in and is slower (10/31 driver)."""
    x, y = pt
    r = float(np.clip((30 * 30) / (d / 3 + 30), 18.0, 34.0))
    on = (rl.get_time() * CONFIRM_PULSE_HZ) % 1.0 < 0.6 if flash else True
    cy = y - r * 1.1
    rl.draw_circle(int(x), int(cy + 3), r + 3, rl.Color(0, 0, 0, 150))
    rl.draw_circle(int(x), int(cy), r, rl.Color(*COLOR_CUTIN, 255 if on else 120))
    dark = rl.Color(20, 20, 20, 255 if on else 150)
    L, hw, th = r * 1.3, r * 0.45, max(4.0, r * 0.22)
    tip = x + direction * L / 2; tail = x - direction * L / 2; base = tip - direction * hw * 1.1
    rl.draw_line_ex(rl.Vector2(tail, cy), rl.Vector2(base, cy), th, dark)
    p1, p2, p3 = rl.Vector2(tip, cy), rl.Vector2(base, cy - hw), rl.Vector2(base, cy + hw)
    rl.draw_triangle(p1, p2, p3, dark)
    rl.draw_triangle(p1, p3, p2, dark)

  def _draw_banner(self, nearest, rect: rl.Rectangle):
    """Big amber banner at the screen edge on the side the merging car comes from, arrow pointing into my lane."""
    d, side = nearest
    try:
      from openpilot.system.ui.lib.multilang import multilang
      zh = multilang.language == "zh-CHT"
    except Exception:
      zh = False
    who = ("右側匯入" if side > 0 else "左側匯入") if zh else ("MERGE R" if side > 0 else "MERGE L")
    dist = f"{max(0.0, d):.0f} m" if ui_state.is_metric else f"{max(0.0, d) * 3.28084:.0f} ft"
    text = f"{who}  {dist}"
    pulse = 0.5 + 0.5 * float(np.sin(rl.get_time() * 2 * np.pi * CUTIN_PULSE_HZ))
    tw = measure_text_cached(self._font, text, BANNER_FONT, 0).x
    head = BANNER_FONT * 0.9
    w, h = tw + head + 70, BANNER_FONT + 34
    y = rect.y + rect.height * 0.42
    x = rect.x + rect.width - w - 30 if side > 0 else rect.x + 30
    box = rl.Rectangle(x, y, w, h)
    rl.draw_rectangle_rounded(rl.Rectangle(x + 4, y + 5, w, h), 0.3, 10, rl.Color(0, 0, 0, 150))
    rl.draw_rectangle_rounded(box, 0.3, 10, rl.Color(*COLOR_CUTIN, int(200 + 55 * pulse)))
    dark = rl.Color(20, 20, 20, 255)
    # arrow pointing toward my lane (left when the car comes from the right)
    ay = y + h / 2
    if side > 0:
      tip, base = x + 18, x + 18 + head
      tx = base + 22
    else:
      tip, base = x + w - 18, x + w - 18 - head
      tx = x + 26
    p1, p2, p3 = rl.Vector2(tip, ay), rl.Vector2(base, ay - head * 0.6), rl.Vector2(base, ay + head * 0.6)
    rl.draw_triangle(p1, p2, p3, dark)
    rl.draw_triangle(p1, p3, p2, dark)
    rl.draw_text_ex(self._font, text, rl.Vector2(tx, y + 17), BANNER_FONT, 0, dark)

  def _draw_arrow(self, x: float, y: float, direction: int, sz: float, color):
    """A bold horizontal arrow beside the marker pointing the way the car is drifting (into my lane)."""
    L = sz * 3.0; head = sz * 1.1; th = max(5.0, sz * 0.35)
    x0 = x - direction * (sz * 1.3)          # tail starts just outside the diamond on the car's side
    x1 = x0 - direction * L                  # head toward my lane
    yc = y - sz * 0.7
    shadow = rl.Color(0, 0, 0, 160)
    for col, w in ((shadow, th + 4), (color, th)):
      rl.draw_line_ex(rl.Vector2(x0, yc), rl.Vector2(x1 + direction * head * 0.6, yc), w, col)
    tip = rl.Vector2(x1, yc); b1 = rl.Vector2(x1 + direction * head, yc - head * 0.8); b2 = rl.Vector2(x1 + direction * head, yc + head * 0.8)
    # raylib draws a 2D triangle only with counter-clockwise vertices; the winding depends on the direction, so draw both
    rl.draw_triangle(rl.Vector2(tip.x, tip.y + 3), rl.Vector2(b1.x, b1.y + 3), rl.Vector2(b2.x, b2.y + 3), shadow)
    rl.draw_triangle(rl.Vector2(tip.x, tip.y + 3), rl.Vector2(b2.x, b2.y + 3), rl.Vector2(b1.x, b1.y + 3), shadow)
    rl.draw_triangle(tip, b1, b2, color)
    rl.draw_triangle(tip, b2, b1, color)

  def _draw_marker(self, pt, d: float, v_ground: float, rgb, rect: rl.Rectangle, settling: bool = False, cls: int = 0, cutin: bool = False, arrow: int = 0):
    x, y = pt
    sz = float(np.clip((25 * 30) / (d / 3 + 30), 12.0, 30.0)) * 0.9
    color = rl.Color(*rgb, 110 if settling else 230)  # settling: range / speed not converged yet
    pulse = 0.6 if cutin else 0.  # 10/31: the predicted cut-in marker is steady; only a confirmed cut-in flashes
    if cutin:
      sz = max(sz * CUTIN_SCALE, CUTIN_MIN_SIZE) * (1.0 + 0.12 * pulse)
    if arrow:
      self._draw_arrow(x, y, arrow, max(sz, 16.0) * (1.3 if cutin else 1.0), rl.Color(*rgb, 255 if cutin else 240))
    # A diamond sitting on the road at the object (filled and pulsing when the car is cutting in)
    top, right, bottom, left = rl.Vector2(x, y - sz * 1.4), rl.Vector2(x + sz, y - sz * 0.7), rl.Vector2(x, y), rl.Vector2(x - sz, y - sz * 0.7)
    if cutin:
      fill = rl.Color(*rgb, int(90 + 110 * pulse))
      for a, b, c in ((top, left, bottom), (top, bottom, left), (top, right, bottom), (top, bottom, right)):
        rl.draw_triangle(a, b, c, fill)  # both windings: raylib draws counter-clockwise only
    shadow_line = rl.Color(0, 0, 0, 180)
    for a, b in ((top, right), (right, bottom), (bottom, left), (left, top)):
      if cutin:
        rl.draw_line_ex(a, b, 10.0, shadow_line)
      rl.draw_line_ex(a, b, 7.0 if cutin else 4.0, color)

    tag = _cutin_label() if cutin else _label(cls, settling)
    if ui_state.is_metric:
      dist, speed = f"{tag}R {max(0.0, d):.0f} m", f"{max(0.0, v_ground * CV.MS_TO_KPH):.0f} km/h"
    else:
      dist, speed = f"{tag}R {max(0.0, d) * 3.28084:.0f} ft", f"{max(0.0, v_ground * CV.MS_TO_MPH):.0f} mph"

    font_size, line_h, margin = (40, 46, 20) if cutin else (32, 38, 20)
    text_y = y + 8
    shadow = rl.Color(0, 0, 0, 100 if settling else 200)
    for i, line in enumerate((dist,) if settling else (dist, speed)):
      w = measure_text_cached(self._font, line, font_size, 0).x
      tx = int(np.clip(x - w / 2, margin, rect.width - w - margin))
      ty = int(text_y + i * line_h)
      if ty + line_h > rect.height - margin:
        break
      rl.draw_text_ex(self._font, line, rl.Vector2(tx + 2, ty + 2), font_size, 0, shadow)
      rl.draw_text_ex(self._font, line, rl.Vector2(tx, ty), font_size, 0, color)
