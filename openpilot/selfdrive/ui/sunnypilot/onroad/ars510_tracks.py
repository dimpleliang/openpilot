"""
ARS510: draw every object the radar tracks, including cars in the next lanes.

opendbc/car/toyota/ars510_radar_interface.py publishes the radar's settled objects (before its path gates) on
customReservedRawData0. Each one gets a small marker with "R <distance>" and its speed. Green: within LANE_HALF_M of the
model's predicted path at that distance (my lane); red: outside it (next lanes). The colour is the lane only; radard's
lead choice does not depend on it. A track still settling (younger than the decoder's publish age, ~1.2 s in fused; its
range and speed have not converged) is drawn faint with its distance only. Pedestrians and two-wheelers (radar class) get a label. Display only: nothing here feeds the controls. A track that already carries a radar lead chevron is skipped, so the lead keeps one label.
"""
import struct

import numpy as np
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

SERVICE = "customReservedRawData0"
FMT_V1 = "<fffB"
FMT = "<fffBH"  # v2: + native track id
STALE_FRAMES = 10  # ui frames (20 Hz) without a radar record before the markers go away
MAX_DRAW_M = 120.0
LEAD_MATCH_D_M = 2.0
LEAD_MATCH_Y_M = 1.2
LANE_HALF_M = 1.8  # half a lane: an object's lateral distance from the predicted path below this is in my lane

COLOR_PATH = (60, 220, 120)
COLOR_SIDE = (255, 60, 60)  # next lanes: red (cyan was hard to see)
COLOR_CUTIN = (255, 190, 0)  # amber: a next-lane car drifting toward my lane (see _cutin)

# Cut-in prediction (display only, 10/31): a next-lane car whose lateral distance to my path has been shrinking faster than
# CUTIN_RATE over the last CUTIN_WIN seconds, still CUTIN_OFF_MIN..CUTIN_OFF_MAX m from the path, slower than me, 25-90 m
# ahead. On 220 km of 10/30 logs this marks about a third of real cut-ins ~2 s early and one in five line-hugging cars for
# about a second. Needs the v2 display rows (track id) to follow a car between frames.
CUTIN_WIN = 2.0
CUTIN_RATE = -0.15  # m/s of |offset|
CUTIN_OFF_MIN, CUTIN_OFF_MAX = 0.8, 2.6
CUTIN_D = (25.0, 90.0)
CUTIN_VREL = -1.5
CUTIN_HOLD_FRAMES = 10  # 0.5 s at 20 Hz
CUTIN_MIN_V_GROUND = 2.0  # m/s over the ground, forward: excludes oncoming cars


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


def parse(payload: bytes) -> list[tuple[float, float, float, bool, bool, int, int]]:
  if len(payload) < 4 or payload[:2] != b"A5" or payload[2] not in (1, 2):
    return []
  fmt = FMT_V1 if payload[2] == 1 else FMT
  size = struct.calcsize(fmt)
  n = payload[3]
  out = []
  for i in range(n):
    off = 4 + i * size
    if off + size > len(payload):
      break
    f = struct.unpack_from(fmt, payload, off)
    d, y, v, flags = f[:4]
    out.append((d, y, v, bool(flags & 1), bool(flags & 2), (flags >> 2) & 7, f[4] if len(f) > 4 else -1))
  return out


class Ars510Tracks:
  def __init__(self):
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._points: list[tuple[float, float, float, bool, bool, int, int]] = []
    self._hist: dict[int, list[tuple[int, float]]] = {}  # track id -> [(ui frame, |offset to path|)]
    self._hold: dict[int, int] = {}                      # track id -> consecutive frames the cut-in condition held

  def draw(self, sm, map_to_screen, path_z, path_y, camera_offset: float, path_offset_z: float, radar_state, rect: rl.Rectangle):
    try:
      if SERVICE not in sm.data:
        return
      if sm.updated[SERVICE]:
        self._points = parse(bytes(sm[SERVICE]))
      if not self._points or sm.frame - sm.recv_frame[SERVICE] > STALE_FRAMES:
        return

      leads = []
      if radar_state is not None:
        for lead in (radar_state.leadOne, radar_state.leadTwo):
          if lead.present and lead.radar:
            leads.append((lead.dRel, lead.yRel))

      v_ego = sm['carState'].vEgo
      fresh = sm.updated[SERVICE]
      seen = set()
      for d, y, v, _sent, settling, cls, tid in self._points:
        if not (0.5 < d < MAX_DRAW_M):
          continue
        if any(abs(d - ld) < LEAD_MATCH_D_M and abs(y - ly) < LEAD_MATCH_Y_M for ld, ly in leads):
          continue
        y_right = -y + camera_offset  # radar yRel is left-positive; the model path and map_to_screen use right-positive (calibrated frame)
        pt = map_to_screen(d, y_right, path_z(d) + path_offset_z)
        if pt is None:
          continue
        off = abs(y_right - path_y(d))
        in_lane = off < LANE_HALF_M
        # oncoming traffic (moving backward over the ground) never counts as a cut-in
        cutin = self._cutin(tid, off, d, v, sm.frame, fresh, settling) if (tid >= 0 and v + v_ego > CUTIN_MIN_V_GROUND) else False
        if tid >= 0:
          seen.add(tid)
        arrow = 0
        if cutin:
          arrow = -1 if (y_right - path_y(d)) > 0 else 1  # car to my right drifts left into my lane, and vice versa
        self._draw_marker(pt, d, v + v_ego, COLOR_CUTIN if cutin else (COLOR_PATH if in_lane else COLOR_SIDE), rect, settling, cls, cutin, arrow)
      if fresh:
        for tid in [k for k in self._hist if k not in seen]:
          self._hist.pop(tid, None); self._hold.pop(tid, None)
    except Exception:
      self._points = []

  def _cutin(self, tid: int, off: float, d: float, vrel: float, frame: int, fresh: bool, settling: bool) -> bool:
    """Lateral-offset trend over CUTIN_WIN: True while the car has been drifting toward my path for CUTIN_HOLD_FRAMES."""
    h = self._hist.setdefault(tid, [])
    if fresh:
      h.append((frame, off))
      while h and frame - h[0][0] > CUTIN_WIN * 20:
        h.pop(0)
    ok = False
    if len(h) >= 6 and not settling and CUTIN_OFF_MIN < off < CUTIN_OFF_MAX and vrel < CUTIN_VREL and CUTIN_D[0] <= d <= CUTIN_D[1]:
      t = np.array([x[0] for x in h], float) / 20.0; o = np.array([x[1] for x in h], float)
      slope = float(np.polyfit(t, o, 1)[0]) if t[-1] - t[0] > 0.5 else 0.
      ok = slope < CUTIN_RATE
    if fresh:
      self._hold[tid] = self._hold.get(tid, 0) + 1 if ok else 0
    return self._hold.get(tid, 0) >= CUTIN_HOLD_FRAMES

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
    if arrow:
      self._draw_arrow(x, y, arrow, max(sz, 16.0), rl.Color(*rgb, 240))
    # A diamond outline sitting on the road at the object
    top, right, bottom, left = rl.Vector2(x, y - sz * 1.4), rl.Vector2(x + sz, y - sz * 0.7), rl.Vector2(x, y), rl.Vector2(x - sz, y - sz * 0.7)
    for a, b in ((top, right), (right, bottom), (bottom, left), (left, top)):
      rl.draw_line_ex(a, b, 4.0, color)

    tag = _cutin_label() if cutin else _label(cls, settling)
    if ui_state.is_metric:
      dist, speed = f"{tag}R {max(0.0, d):.0f} m", f"{max(0.0, v_ground * CV.MS_TO_KPH):.0f} km/h"
    else:
      dist, speed = f"{tag}R {max(0.0, d) * 3.28084:.0f} ft", f"{max(0.0, v_ground * CV.MS_TO_MPH):.0f} mph"

    font_size, line_h, margin = 32, 38, 20
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
