"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from importlib.resources import as_file

import pyray as rl

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight, FONT_DIR
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


class RoadNameRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.road_name = ""
    self.is_metric = False
    self.font_demi = gui_app.font(FontWeight.SEMI_BOLD)
    # Inter has no CJK glyphs: Taiwanese map road names (中山高速公路) came out as boxes. For a non-ASCII name, load the
    # bundled unifont with exactly that name's characters (one small atlas per name, replaced when the name changes).
    self._cjk_font = None
    self._cjk_name = None

  def _font_for(self, text: str):
    if text.isascii():
      return self.font_demi
    if text != self._cjk_name:
      try:
        cps = sorted(set(map(ord, text + "...")) | set(range(32, 127)))
        arr = rl.ffi.new("int[]", cps)
        with as_file(FONT_DIR) as fspath:
          font = rl.load_font_ex((fspath / "unifont.otf").as_posix(), 46, rl.ffi.cast("int *", arr), len(cps))
        rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        if self._cjk_font is not None:
          rl.unload_font(self._cjk_font)
        self._cjk_font, self._cjk_name = font, text
      except Exception:
        cloudlog.exception("road name font failed")
        self._cjk_name = text
        return self.font_demi if self._cjk_font is None else self._cjk_font
    return self._cjk_font

  def update(self):
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      return

    self.is_metric = ui_state.is_metric

    if sm.updated["liveMapDataSP"]:
      lmd = sm["liveMapDataSP"]
      self.road_name = lmd.roadName

  def _render(self, rect: rl.Rectangle):
    if not self.road_name or not ui_state.road_name_toggle:
      return

    text = self.road_name
    font = self._font_for(text)
    text_size = measure_text_cached(font, text, 46)

    padding = 40
    rect_width = max(200, min(text_size.x + padding, rect.width - 40))

    road_rect = rl.Rectangle(rect.x + rect.width / 2 - rect_width / 2, rect.y - 4, rect_width, 60)

    rl.draw_rectangle_rounded(road_rect, 0.2, 10, rl.Color(0, 0, 0, 120))

    max_text_width = road_rect.width - 20
    if text_size.x > max_text_width:
      while text_size.x > max_text_width and len(text) > 3:
        text = text[:-1]
        text_size = measure_text_cached(font, text + "...", 46)
      text = text + "..."

    sz = measure_text_cached(font, text, 46)
    origin = rl.Vector2(road_rect.x + road_rect.width / 2 - sz.x / 2, road_rect.y + road_rect.height / 2 - sz.y / 2)
    rl.draw_text_ex(font, text, origin, 46, 0, rl.Color(255, 255, 255, 200))
