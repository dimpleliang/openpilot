"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import pyray as rl
from openpilot.selfdrive.ui.layouts.home import HomeLayout, HomeLayoutState, HEAD_BUTTON_FONT_SIZE, SPACING
from openpilot.system.ui.lib.application import gui_app, FontWeight, TextAlignment
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.multilang import tr, trn
from openpilot.system.ui.widgets.label import gui_label

BRAND_FONT_SIZE = 48
BRAND_DESC_SPACING = 12
# ars510 (11/05 driver): network upload / download speed on the offroad home header (all links but loopback / VPN tunnels,
# the VPN's traffic is counted on the link it rides), sampled from /proc/net/dev once a second
NET_RATE_FONT_SIZE = 40
NET_RATE_SKIP = ("lo", "tailscale", "dummy", "docker", "bond", "ifb", "sit", "ip6tnl", "tun")


def _net_bytes() -> tuple[int, int]:
  rx = tx = 0
  try:
    with open("/proc/net/dev") as f:
      for line in f.readlines()[2:]:
        name, data = line.split(":", 1)
        if name.strip().startswith(NET_RATE_SKIP):
          continue
        v = data.split()
        rx += int(v[0])
        tx += int(v[8])
  except (OSError, ValueError, IndexError):
    pass
  return rx, tx


def _fmt_rate(bps: float) -> str:
  return f"{bps / 1e6:.2f} MB/s" if bps >= 1e6 else f"{bps / 1e3:.0f} KB/s"


class HomeLayoutSP(HomeLayout):
  _net_last: tuple[float, int, int] | None = None
  _net_rate: tuple[float, float] | None = None   # (up, down) bytes/s

  def _update_net_rate(self):
    now = time.monotonic()
    if self._net_last is not None and now - self._net_last[0] < 1.0:
      return
    rx, tx = _net_bytes()
    if self._net_last is not None:
      dt = now - self._net_last[0]
      self._net_rate = (max(tx - self._net_last[2], 0) / dt, max(rx - self._net_last[1], 0) / dt)
    self._net_last = (now, rx, tx)

  def _render_net_rate(self, x: float, avail: float):
    self._update_net_rate()
    if self._net_rate is None:
      return
    font = gui_app.font(FontWeight.MEDIUM)
    texts = [_fmt_rate(v) for v in self._net_rate]
    for fs in (NET_RATE_FONT_SIZE, 34, 28):   # largest size that fits left of the brand / version text
      s, gap = fs * 0.35, fs * 0.6
      sizes = [measure_text_cached(font, t, fs) for t in texts]
      if sum(2 * s + 10 + z.x for z in sizes) + gap <= avail:
        break
    else:
      return
    cy = self.header_rect.y + self.header_rect.height / 2
    colors = (rl.Color(255, 170, 60, 255), rl.Color(90, 200, 255, 255))
    for i, (text, size) in enumerate(zip(texts, sizes)):
      if i == 0:   # upload: triangle pointing up
        rl.draw_triangle(rl.Vector2(x + s, cy - s), rl.Vector2(x, cy + s * 0.8), rl.Vector2(x + 2 * s, cy + s * 0.8), colors[i])
      else:        # download: pointing down
        rl.draw_triangle(rl.Vector2(x, cy - s * 0.8), rl.Vector2(x + s, cy + s), rl.Vector2(x + 2 * s, cy - s * 0.8), colors[i])
      rl.draw_text_ex(font, text, rl.Vector2(int(x + 2 * s + 10), int(cy - size.y / 2)), fs, 0, rl.WHITE)
      x += 2 * s + 10 + size.x + gap

  def _render_header(self):
    left = self.header_rect.x
    if self.update_available:
      left += self.update_notif_rect.width + SPACING
    if self.alert_count > 0:
      left += self.alert_notif_rect.width + SPACING
    desc = self.params.get("UpdaterCurrentDescription") or ""
    right_w = measure_text_cached(gui_app.font(FontWeight.AUDIOWIDE), "sunnypilot", BRAND_FONT_SIZE).x
    if desc:
      right_w += BRAND_DESC_SPACING + measure_text_cached(gui_app.font(FontWeight.NORMAL), desc, BRAND_FONT_SIZE).x
    self._render_net_rate(left, self.header_rect.x + self.header_rect.width - right_w - 30 - left)
    font = gui_app.font(FontWeight.MEDIUM)

    version_text_width = self.header_rect.width

    if self.update_available:
      version_text_width -= self.update_notif_rect.width

      highlight_color = rl.Color(75, 95, 255, 255) if self.current_state == HomeLayoutState.UPDATE else rl.Color(54, 77, 239, 255)
      rl.draw_rectangle_rounded(self.update_notif_rect, 0.3, 10, highlight_color)

      text = tr("UPDATE")
      text_size = measure_text_cached(font, text, HEAD_BUTTON_FONT_SIZE)
      text_x = self.update_notif_rect.x + (self.update_notif_rect.width - text_size.x) // 2
      text_y = self.update_notif_rect.y + (self.update_notif_rect.height - text_size.y) // 2
      rl.draw_text_ex(font, text, rl.Vector2(int(text_x), int(text_y)), HEAD_BUTTON_FONT_SIZE, 0, rl.WHITE)

    if self.alert_count > 0:
      version_text_width -= self.alert_notif_rect.width

      highlight_color = rl.Color(255, 70, 70, 255) if self.current_state == HomeLayoutState.ALERTS else rl.Color(226, 44, 44, 255)
      rl.draw_rectangle_rounded(self.alert_notif_rect, 0.3, 10, highlight_color)

      alert_text = trn("{} ALERT", "{} ALERTS", self.alert_count).format(self.alert_count)
      text_size = measure_text_cached(font, alert_text, HEAD_BUTTON_FONT_SIZE)
      text_x = self.alert_notif_rect.x + (self.alert_notif_rect.width - text_size.x) // 2
      text_y = self.alert_notif_rect.y + (self.alert_notif_rect.height - text_size.y) // 2
      rl.draw_text_ex(font, alert_text, rl.Vector2(int(text_x), int(text_y)), HEAD_BUTTON_FONT_SIZE, 0, rl.WHITE)

    if self.update_available or self.alert_count > 0:
      version_text_width -= SPACING * 1.5

    version_right = self.header_rect.x + self.header_rect.width
    version_left = version_right - version_text_width

    brand = "sunnypilot"
    description = self.params.get("UpdaterCurrentDescription") or ""

    desc_width = 0
    if description:
      desc_size = measure_text_cached(gui_app.font(FontWeight.NORMAL), description, BRAND_FONT_SIZE)
      desc_width = desc_size.x
      desc_rect = rl.Rectangle(version_right - desc_width, self.header_rect.y, desc_width, self.header_rect.height)
      gui_label(desc_rect, description, BRAND_FONT_SIZE, rl.WHITE, alignment=TextAlignment.RIGHT)

    brand_size = measure_text_cached(gui_app.font(FontWeight.AUDIOWIDE), brand, BRAND_FONT_SIZE)
    spacing = BRAND_DESC_SPACING if description else 0
    brand_x = version_right - desc_width - spacing - brand_size.x
    brand_rect = rl.Rectangle(max(version_left, brand_x), self.header_rect.y, brand_size.x, self.header_rect.height)
    gui_label(brand_rect, brand, BRAND_FONT_SIZE, rl.WHITE, font_weight=FontWeight.AUDIOWIDE)
