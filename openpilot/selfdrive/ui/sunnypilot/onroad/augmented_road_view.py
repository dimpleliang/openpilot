"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import UIStatus, ui_state
from openpilot.system.ui.lib.application import gui_app

BORDER_COLORS_SP = {
  UIStatus.LAT_ONLY: rl.Color(0x00, 0xC8, 0xC8, 0xFF),  # Cyan for lateral-only state
  UIStatus.LONG_ONLY: rl.Color(0x96, 0x1C, 0xA8, 0xFF),  # Purple for longitudinal-only state
}


BLIND_SPOT_BORDER_COLOR = rl.Color(0xB4, 0x5F, 0xFF, 0xFF)  # 盲點有車時，該側邊框改成紫色


class AugmentedRoadViewSP:
  def __init__(self):
    self._bsm_left_alpha = FirstOrderFilter(0, 0.08, 1 / gui_app.target_fps)
    self._bsm_right_alpha = FirstOrderFilter(0, 0.08, 1 / gui_app.target_fps)
    self._fade_texture = gui_app.texture("icons_mici/onroad/onroad_fade.png")
    self._fade_alpha_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

  def update_fade_out_bottom_overlay(self, _content_rect):
    # Fade out bottom of overlays for looks (only when engaged)
    fade_alpha = self._fade_alpha_filter.update(ui_state.status != UIStatus.DISENGAGED)
    if ui_state.torque_bar and fade_alpha > 1e-2:
      # Scale the fade texture to the content rect
      rl.draw_texture_pro(self._fade_texture,
                          rl.Rectangle(0, 0, self._fade_texture.width, self._fade_texture.height),
                          _content_rect, rl.Vector2(0, 0), 0.0,
                          rl.Color(255, 255, 255, int(255 * fade_alpha)))

  def draw_blind_spot_border(self, rect: rl.Rectangle, border_rect: rl.Rectangle, thickness: float, roundness: float):
    # 盲點有車時，把同一側（左半／右半）的邊框畫成紫色，一眼就能看出哪一側有車
    CS = ui_state.sm['carState']
    left = bool(CS.leftBlindspot) and ui_state.blindspot
    right = bool(CS.rightBlindspot) and ui_state.blindspot
    la = self._bsm_left_alpha.update(1.0 if left else 0.0)
    ra = self._bsm_right_alpha.update(1.0 if right else 0.0)

    half = rect.width / 2
    for alpha, x in ((la, rect.x), (ra, rect.x + half)):
      if alpha < 0.02:
        continue
      color = rl.Color(BLIND_SPOT_BORDER_COLOR.r, BLIND_SPOT_BORDER_COLOR.g, BLIND_SPOT_BORDER_COLOR.b, int(255 * alpha))
      rl.begin_scissor_mode(int(x), int(rect.y), int(half), int(rect.height))
      rl.draw_rectangle_rounded_lines_ex(border_rect, roundness, 10, thickness, color)
      rl.end_scissor_mode()
