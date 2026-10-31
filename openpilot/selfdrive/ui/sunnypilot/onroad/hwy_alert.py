"""Freeway incident banner (高公局 LiveEvents) — reads /dev/shm/hwyalert.json written by selfdrive/hwyalert/hwyalertd.py."""
import json
import time
from importlib.resources import as_file

import pyray as rl

from openpilot.selfdrive.hwyalert.hwyalertd import STATE_FILE, TARGET, SHORT
from openpilot.system.ui.lib.application import FONT_DIR
from openpilot.system.ui.lib.text_measure import measure_text_cached

FONT_SIZE = 50
READ_S = 1.0
STALE_S = 5.0
BG = rl.Color(0xC6, 0x28, 0x28, 0xE6)     # red: accident / fire / wrong-way / people or animals on the road
BG_WORK = rl.Color(0xEF, 0x8F, 0x00, 0xE6)  # amber: construction, debris, breakdowns, road surface
RED = ("事故", "逆行車輛", "人員停留", "機車誤闖", "動物誤闖", "自行車誤闖", "行人誤闖", "道路封閉")
GLYPHS = "前方 km（）[]、.0123456789第車道內外路肩全部線封閉匝平面高架火" + "".join(TARGET) + "".join(SHORT.values())


class HwyAlertRenderer:
  def __init__(self):
    self._alerts: list[dict] = []
    self._t_read = 0.
    self._font = None

  def _load_font(self):
    chars = sorted(set(map(chr, range(32, 127))) | set(GLYPHS))
    cps = rl.ffi.new("int[]", [ord(c) for c in chars])
    with as_file(FONT_DIR) as fspath:
      self._font = rl.load_font_ex((fspath / "NotoSansCJKtc-Regular.otf").as_posix(), FONT_SIZE, rl.ffi.cast("int *", cps), len(chars))
    rl.set_texture_filter(self._font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)

  def update(self):
    now = time.monotonic()
    if now - self._t_read < READ_S:
      return
    self._t_read = now
    try:
      with open(STATE_FILE) as f:
        st = json.load(f)
      self._alerts = st.get("alerts", []) if time.time() - st.get("t", 0) < STALE_S else []
    except Exception:
      self._alerts = []

  def render(self, rect: rl.Rectangle):
    if not self._alerts:
      return
    if self._font is None:
      self._load_font()
    y = rect.y + 310  # below the 300 px header (set / current speed)
    for a in self._alerts[:2]:
      text = a["text"]
      sz = measure_text_cached(self._font, text, FONT_SIZE, 0)
      w, h = sz.x + 48, FONT_SIZE + 24
      r = rl.Rectangle(rect.x + rect.width / 2 - w / 2, y, w, h)
      red = a.get("name") in RED or "火" in a.get("name", "")
      rl.draw_rectangle_rounded(r, 0.3, 10, BG if red else BG_WORK)
      rl.draw_text_ex(self._font, text, rl.Vector2(r.x + 24, r.y + 10), FONT_SIZE, 0, rl.WHITE)
      y += h + 12
