"""
ARS510: draw every object the radar tracks, including cars in the next lanes.

opendbc/car/toyota/ars510_radar_interface.py publishes the radar's settled objects (before its path gates) on
customReservedRawData0. Each one gets a small marker with "R <distance>" and its speed. Green: within LANE_HALF_M of the
model's predicted path at that distance (my lane); cyan: outside it (next lanes). The colour is the lane only; radard's
lead choice does not depend on it. A track still settling (younger than the decoder's publish age, ~3.6 s; its
range and speed have not converged) is drawn faint with its distance only. Display only: nothing here feeds the controls. A track that already carries a radar lead chevron is skipped, so the lead keeps one label.
"""
import struct

import numpy as np
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

SERVICE = "customReservedRawData0"
FMT = "<fffB"
FMT_SIZE = struct.calcsize(FMT)
STALE_FRAMES = 10  # ui frames (20 Hz) without a radar record before the markers go away
MAX_DRAW_M = 120.0
LEAD_MATCH_D_M = 2.0
LEAD_MATCH_Y_M = 1.2
LANE_HALF_M = 1.8  # half a lane: an object's lateral distance from the predicted path below this is in my lane

COLOR_PATH = (60, 220, 120)
COLOR_SIDE = (70, 200, 255)


def parse(payload: bytes) -> list[tuple[float, float, float, bool, bool]]:
  if len(payload) < 4 or payload[:2] != b"A5" or payload[2] != 1:
    return []
  n = payload[3]
  out = []
  for i in range(n):
    off = 4 + i * FMT_SIZE
    if off + FMT_SIZE > len(payload):
      break
    d, y, v, flags = struct.unpack_from(FMT, payload, off)
    out.append((d, y, v, bool(flags & 1), bool(flags & 2)))
  return out


class Ars510Tracks:
  def __init__(self):
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._points: list[tuple[float, float, float, bool, bool]] = []

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
      for d, y, v, _sent, settling in self._points:
        if not (0.5 < d < MAX_DRAW_M):
          continue
        if any(abs(d - ld) < LEAD_MATCH_D_M and abs(y - ly) < LEAD_MATCH_Y_M for ld, ly in leads):
          continue
        y_left = -y + camera_offset  # radar yRel is right-positive; model / path y is left-positive
        pt = map_to_screen(d, y_left, path_z(d) + path_offset_z)
        if pt is None:
          continue
        in_lane = abs(y_left - path_y(d)) < LANE_HALF_M
        self._draw_marker(pt, d, v + v_ego, COLOR_PATH if in_lane else COLOR_SIDE, rect, settling)
    except Exception:
      self._points = []

  def _draw_marker(self, pt, d: float, v_ground: float, rgb, rect: rl.Rectangle, settling: bool = False):
    x, y = pt
    sz = float(np.clip((25 * 30) / (d / 3 + 30), 12.0, 30.0)) * 0.9
    color = rl.Color(*rgb, 110 if settling else 230)  # settling: range / speed not converged yet
    # A diamond outline sitting on the road at the object
    top, right, bottom, left = rl.Vector2(x, y - sz * 1.4), rl.Vector2(x + sz, y - sz * 0.7), rl.Vector2(x, y), rl.Vector2(x - sz, y - sz * 0.7)
    for a, b in ((top, right), (right, bottom), (bottom, left), (left, top)):
      rl.draw_line_ex(a, b, 4.0, color)

    if ui_state.is_metric:
      dist, speed = f"R {max(0.0, d):.0f} m", f"{max(0.0, v_ground * CV.MS_TO_KPH):.0f} km/h"
    else:
      dist, speed = f"R {max(0.0, d) * 3.28084:.0f} ft", f"{max(0.0, v_ground * CV.MS_TO_MPH):.0f} mph"

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
