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
from openpilot.selfdrive.controls.lib.ars510_cutin import SERVICE, parse  # noqa: F401  (parse: tools import it from here)
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

  def draw(self, sm, map_to_screen, path_z, path_y, camera_offset: float, path_offset_z: float, radar_state, rect: rl.Rectangle):
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
    except Exception:
      self._points = []

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
