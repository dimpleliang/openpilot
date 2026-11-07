"""Freeway incident banner (高公局 LiveEvents) — reads /dev/shm/hwyalert.json written by selfdrive/hwyalert/hwyalertd.py.

Everything sits at the bottom of the screen (driver's request 11/02): while the car is on a freeway a grey status strip
always shows where we are (國1高架 北向 24.8K) and the nearest event up to LOOKAHEAD_KM ahead; events within ALERT_KM
are drawn above it as red / amber banners and stay until passed."""
import json
import time
from importlib.resources import as_file

import pyray as rl

from openpilot.common.swaglog import cloudlog

from openpilot.selfdrive.hwyalert.hwyalertd import STATE_FILE, DISPLAY_CHARS, MAX_ALERTS
from openpilot.system.ui.lib.application import FONT_DIR
from openpilot.selfdrive.ui.sunnypilot.onroad import raw_text

FONT_SIZE = 50
STRIP_FONT_SIZE = 40
READ_S = 1.0
STALE_S = 5.0
BOTTOM_MARGIN = 30                        # px above the screen edge
BG = rl.Color(0xC6, 0x28, 0x28, 0xE6)     # red: accident / fire / wrong-way / people or animals on the road
BG_WORK = rl.Color(0xEF, 0x8F, 0x00, 0xE6)  # amber: construction, debris, breakdowns, road surface
BG_STRIP = rl.Color(0x20, 0x20, 0x20, 0xB4)  # grey: status strip
FG_STRIP = rl.Color(0xE0, 0xE0, 0xE0, 0xFF)
RED = ("事故", "逆行車輛", "人員停留", "機車誤闖", "動物誤闖", "自行車誤闖", "行人誤闖", "道路封閉")
GLYPHS = DISPLAY_CHARS


class HwyAlertRenderer:
  def __init__(self):
    self._alerts: list[dict] = []
    self._where = ""
    self._next = ""
    self._t_read = 0.
    self._font = None
    self._chars: set[str] = set()
    self._failed = False

  def _load_font(self, need: set[str]):
    # Atlas of ASCII + the characters on screen, rebuilt when a new one appears (11/07; the "?" in the photo that day were the
    # app swapping this font for its Noto subset under the zh-CHT UI, see raw_text).
    chars = sorted(set(map(chr, range(32, 127))) | need)
    cps = rl.ffi.new("int[]", [ord(c) for c in chars])
    with as_file(FONT_DIR) as fspath:
      font = rl.load_font_ex((fspath / "unifont.otf").as_posix(), FONT_SIZE, rl.ffi.cast("int *", cps), len(chars))
    rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    if self._font is not None:
      rl.unload_font(self._font)
    self._font, self._chars = font, set(chars)

  def update(self):
    now = time.monotonic()
    if now - self._t_read < READ_S:
      return
    self._t_read = now
    try:
      with open(STATE_FILE) as f:
        st = json.load(f)
      fresh = time.time() - st.get("t", 0) < STALE_S
      self._alerts = st.get("alerts", []) if fresh else []
      self._where = st.get("where", "") if fresh else ""
      self._next = st.get("next", "") if fresh else ""
    except Exception:
      self._alerts, self._where, self._next = [], "", ""

  def render(self, rect: rl.Rectangle):
    if (not self._alerts and not self._where) or self._failed:
      return
    try:
      self._render(rect)
    except Exception:
      self._failed = True   # never take the onroad UI down for the banner
      cloudlog.exception("hwyalert banner failed")

  def _pill(self, rect: rl.Rectangle, text: str, size: int, y_bottom: float, bg: rl.Color, fg: rl.Color) -> float:
    """Draw a centred pill whose bottom edge is at y_bottom; return its top edge."""
    sz = raw_text.measure_text(self._font, text, size, 0)   # not the app's Noto-fallback swap (zh-CHT UI)
    w, h = sz.x + 48, size + 24
    r = rl.Rectangle(rect.x + rect.width / 2 - w / 2, y_bottom - h, w, h)
    rl.draw_rectangle_rounded(r, 0.3, 10, bg)
    raw_text.draw_text(self._font, text, rl.Vector2(r.x + 24, r.y + 12), size, 0, fg)
    return r.y

  def _render(self, rect: rl.Rectangle):
    strip = (self._where + (f"　｜　{self._next}" if self._next else "")) if self._where else ""
    missing = set(strip + "".join(a.get("text", "") for a in self._alerts[:MAX_ALERTS])) - self._chars
    if self._font is None or missing:
      self._load_font(set(strip + "".join(a.get("text", "") for a in self._alerts[:MAX_ALERTS])))
    y = rect.y + rect.height - BOTTOM_MARGIN
    if self._where:
      y = self._pill(rect, strip, STRIP_FONT_SIZE, y, BG_STRIP, FG_STRIP) - 10
    for a in reversed(self._alerts[:MAX_ALERTS]):   # nearest event ends up lowest, right above the strip
      red = a.get("name") in RED or "火" in a.get("name", "")
      y = self._pill(rect, a["text"], FONT_SIZE, y, BG if red else BG_WORK, rl.WHITE) - 12
