"""Draw / measure text with exactly the given font. With a UI language that needs the Noto fallback (zh-CHT, th), the app
patches rl.draw_text_ex and measure_text_cached to swap ANY font for that fallback, a subset with only the translation's
characters: our own unifont atlases (road names, freeway banners) then came out as "?" (11/07 photo: 前? 1.0 km ???).
These two keep the app's global text scale but not the font swap."""
import pyray as rl

from openpilot.system.ui.lib.application import FONT_SCALE

_cache: dict[int, rl.Vector2] = {}


def draw_text(font: rl.Font, text: str, pos: rl.Vector2, size: float, spacing: float, tint: rl.Color):
  draw = getattr(rl, "_orig_draw_text_ex", rl.draw_text_ex)
  draw(font, text, pos, size * FONT_SCALE, spacing, tint)


def measure_text(font: rl.Font, text: str, size: float, spacing: float = 0) -> rl.Vector2:
  key = hash((font.texture.id, text, size, round(spacing, 4)))
  if key not in _cache:
    _cache[key] = rl.measure_text_ex(font, text, size * FONT_SCALE, spacing)  # noqa: TID251
  return _cache[key]
