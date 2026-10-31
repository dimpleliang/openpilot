#!/usr/bin/env python3
"""Freeway incident alert (高公局 MOTC LiveEvents) for the onroad UI.

Every 60 s fetch the national freeway live events (updated every minute by the 1968 centre), match the car to a
freeway section from GPS position + heading (direction-specific section polylines, freeway_sections.json.gz built
from MOTC Section.xml + SectionShape.xml), and keep the events on the same road and direction within ALERT_KM ahead.
The result goes to STATE_FILE (json, tmpfs) for the UI banner, and every change is logged with cloudlog.

Off switch: touch /data/hwyalert_off.   Offline check: tools/ars510/hwyalert_replay.py.
"""
import calendar
import gzip
import json
import math
import os
import re
import threading
import time
import urllib.request
import xml.etree.ElementTree as ET
from collections import defaultdict

URL = "https://tisvcloud.freeway.gov.tw/history/motc20/LiveEvents.xml"
STATE_FILE = "/dev/shm/hwyalert.json"
OFF_FILE = "/data/hwyalert_off"
SECTIONS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "freeway_sections.json.gz")

FETCH_S = 60.
FEED_MAX_AGE_S = 300.      # drop the feed when it could not be refreshed for 5 min
ALERT_KM = 3.0             # show events up to 3 km ahead (straight line to the event point)
BEHIND_KM = 0.15           # ... and just passed (GPS / km-post noise)
MATCH_M = 40.              # GPS to section polyline
MATCH_HDG = 45.            # heading vs polyline direction (deg)
MATCH_MIN_V = 5.           # heading is only trusted above this speed (m/s)
STACK_M = 8.               # a second road this close to the best match is ambiguous: show its events too, tagged
DR_MAX_KM = 15.            # no GPS (tunnels, 雪山隧道 12.9 km): carry the last match forward by speed this far
MAX_ALERTS = 2             # banners shown (and the only events the voice announces)
SAY_GAP_S = 4.             # one announcement at a time ("前方三公里" + "故障車" is ~2 s)
RANGE_KM = 1.0             # inside an event range at least this long (移動性施工, closures) -> "本路段"
FETCH_RETRY_S = 15.
MATCH_FRAMES = 3           # consecutive matches before the car counts as on the freeway
LONG_TERM_H = 24.          # 施工維護 older than this is long-term construction: not shown

# 1968 event names that matter for the driver ("火" in the name also counts: 車輛火災, 其他火災, 邊坡起火, ...)
TARGET = ("散落物", "事故", "故障車", "施工維護", "移動性施工", "機車誤闖", "動物誤闖", "自行車誤闖", "行人誤闖", "人員停留", "坑洞",
          "路樹", "坍方", "路面積水", "道路封閉", "逆行車輛", "濃霧連動", "濃霧")
SHORT = {"機車誤闖": "機車誤闖", "動物誤闖": "動物", "自行車誤闖": "自行車誤闖", "行人誤闖": "行人", "施工維護": "施工",
         "移動性施工": "移動施工", "路面積水": "積水", "道路封閉": "封閉", "逆行車輛": "逆向車", "濃霧連動": "濃霧"}
# voice clips (assets/sounds/hwy_<clip>.wav, macOS "say -v Meijia"): distance + event, played once per event
VOICE = {"散落物": "debris", "事故": "accident", "故障車": "breakdown", "施工維護": "work", "移動性施工": "mobilework",
         "機車誤闖": "moto", "動物誤闖": "animal", "自行車誤闖": "bike", "行人誤闖": "ped", "人員停留": "person", "坑洞": "pothole",
         "路樹": "tree", "坍方": "landslide", "路面積水": "water", "道路封閉": "closed", "逆行車輛": "wrongway", "濃霧連動": "fog",
         "濃霧": "fog", "車輛火災": "carfire", "邊坡起火": "slopefire"}
SAY_FILE = "/dev/shm/hwyalert_say.json"
SAY_OFF_FILE = "/data/hwyalert_voice_off"


# every character an alert text can contain (the UI loads only these glyphs)
DISPLAY_CHARS = ("前方本路段 km（）[]、.–K0123456789第車道內外路肩全線匝封閉平面高架其他火災" + "".join(TARGET) + "".join(SHORT.values()) +
                 "".join(VOICE))


def voice_clips(d_km: float, name: str) -> list[str]:
  dist = "d3" if d_km >= 2.5 else "d2" if d_km >= 1.5 else "d1" if d_km >= 0.5 else "d0"
  return [dist, VOICE.get(name, "fire" if "火" in name else "accident")]


DIRS = {"北向": "N", "南向": "S", "東向": "E", "西向": "W"}
CN = {"一": 1, "二": 2, "三": 3, "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}


def is_target(name: str) -> bool:
  return name in TARGET or "火" in name


def norm_road(r: str) -> str:
  """國道一號 / 國道1號 -> 1, 國道一號高架 / 國道1號汐止五股高架道路 -> 1高架, 國道三甲 -> 3甲, 臺76線 -> 台76"""
  r = r.replace("國道", "").replace("號", "").replace("汐止五股高架道路", "高架").replace("臺", "台").replace("線", "")
  m = re.match(r"([一二三四五六七八九十]+)(.*)", r)
  if m:
    s = m.group(1)
    n = 10 + CN.get(s[1:], 0) if s.startswith("十") else CN[s[0]] * (10 if s.endswith("十") and len(s) == 2 else 1)
    if len(s) == 3:
      n = CN[s[0]] * 10 + CN[s[2]]
    r = str(n) + m.group(2)
  return r


def parse_km(s: str):
  m = re.match(r"\s*(\d+)K\+(\d+)", s or "")
  return int(m.group(1)) + int(m.group(2)) / 1000. if m else None


def lanes_text(s: str) -> str:
  """BlockedLanes (高公局欄位說明): lanes numbered 0,1,2,... from the left of the travel direction; LS / RS = left /
  right shoulder; -1 = no lane blocked; 111111 = mainline fully closed; 222222 = all ramps closed; 255 / -99 = unknown."""
  s = (s or "").strip()
  if s == "111111":
    return "全線封閉"
  if s == "222222":
    return "匝道封閉"
  out = []
  for p in s.split(","):
    p = p.strip()
    if p == "LS":
      out.append("內路肩")
    elif p == "RS":
      out.append("外路肩")
    elif p.isdigit() and int(p) < 10:
      out.append(f"第{int(p) + 1}車道")
  return "、".join(out)


LAT0 = 23.7
KX = 111320. * math.cos(math.radians(LAT0))
KY = 110540.


def xy(lon, lat):
  return (lon - 121.) * KX, (lat - LAT0) * KY


class Sections:
  """Direction-specific freeway polylines with km posts, on a 1 km grid."""
  def __init__(self, path=SECTIONS):
    self.segs = []   # (road, dir, km0, km1, x0, y0, x1, y1, hdg, frac0, frac1)
    self.grid = defaultdict(list)
    for sid, road, d, ks, ke, pts in json.loads(gzip.open(path).read()):
      p = [xy(*q) for q in pts]
      ln = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:])]
      tot = sum(ln) or 1.
      acc = 0.
      for (a, b), l in zip(zip(p, p[1:]), ln):
        if l < 1e-3:
          continue
        f0, acc = acc / tot, acc + l
        i = len(self.segs)
        hdg = math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360
        self.segs.append((road, d, ks, ke, a[0], a[1], b[0], b[1], hdg, f0, acc / tot))
        for gx in range(int(min(a[0], b[0]) // 1000) - 1, int(max(a[0], b[0]) // 1000) + 2):
          for gy in range(int(min(a[1], b[1]) // 1000) - 1, int(max(a[1], b[1]) // 1000) + 2):
            self.grid[(gx, gy)].append(i)

  def match(self, lon, lat, bearing=None, only=None):
    """[(dist m, road, dir, km, km increases along travel)] best per road/dir, nearest first.
    bearing None = no heading check (slow traffic); only = set of (road, dir) to consider."""
    x, y = xy(lon, lat)
    best = {}
    for i in self.grid.get((int(x // 1000), int(y // 1000)), ()):
      road, d, ks, ke, x0, y0, x1, y1, hdg, f0, f1 = self.segs[i]
      if only is not None and (road, d) not in only:
        continue
      if bearing is not None and abs((bearing - hdg + 180) % 360 - 180) > MATCH_HDG:
        continue
      vx, vy = x1 - x0, y1 - y0
      t = max(0., min(1., ((x - x0) * vx + (y - y0) * vy) / (vx * vx + vy * vy)))
      dist = math.hypot(x - x0 - t * vx, y - y0 - t * vy)
      if dist > MATCH_M:
        continue
      km = ks + (ke - ks) * (f0 + t * (f1 - f0))
      k = (road, d)
      if k not in best or dist < best[k][0]:
        best[k] = (dist, road, d, km, ke >= ks)
    return sorted(best.values())


def parse_feed(xml_bytes: bytes, now_s: float):
  """Target events: dict(id, road, dir, km, km2, lon, lat, name, lanes, title)."""
  out = []
  root = ET.fromstring(xml_bytes)
  for e in root.iter():
    if not e.tag.endswith("LiveEvent") or e.tag.endswith("LiveEvents"):
      continue
    g = {c.tag.split("}")[-1]: c for c in e.iter()}
    txt = lambda k: (g[k].text or "").strip() if k in g and g[k].text else ""
    desc = txt("Description")
    # the first <Description> is the event text ("國道一號 南向 24K+100 事故-故障車"), Impact has its own
    for c in e:
      if c.tag.split("}")[-1] == "Description":
        desc = (c.text or "").strip()
    name = desc.rsplit("-", 1)[-1] if "-" in desc else txt("EventTitle")
    if not is_target(name) or txt("LocationType") in ("1", "2"):   # 1 / 2 = interchange ramp / ramp entry-exit
      continue
    if name == "施工維護":
      try:
        eff = _iso_to_epoch(txt("EffectiveTime"))
        if now_s - eff > LONG_TERM_H * 3600:
          continue
      except Exception:
        pass
    m = re.search(r"POINT\s*\(\s*([\d.]+)\s+([\d.]+)", txt("Positions"))
    out.append(dict(id=txt("EventID"), road=norm_road(txt("Road")), dir=DIRS.get(txt("Direction"), ""),
                    km=parse_km(txt("StartKM")), km2=parse_km(txt("EndKM")),
                    lon=float(m.group(1)) if m else None, lat=float(m.group(2)) if m else None,
                    name=name, lanes=lanes_text(txt("BlockedLanes")), title=txt("EventTitle")))
  return out


def _iso_to_epoch(s: str) -> float:
  # 2026-11-01T03:46:00+08:00
  m = re.match(r"(\d+)-(\d+)-(\d+)T(\d+):(\d+):(\d+)([+-])(\d+):(\d+)", s)
  y, mo, d, h, mi, se = (int(m.group(i)) for i in range(1, 7))
  off = (int(m.group(8)) * 60 + int(m.group(9))) * 60 * (1 if m.group(7) == "+" else -1)
  return calendar.timegm((y, mo, d, h, mi, se, 0, 0, 0)) - off


def ahead(matches, events, pos=None, bearing=0.):
  """Events on the matched road/dir, ALERT_KM ahead, nearest first: [(dist km, event)].
  pos = (lon, lat) for the straight-line check to the event point; None (no GPS) = km posts only."""
  if not matches:
    return []
  # roads within STACK_M of the best match (國1 and 汐五高架 run stacked / side by side and share km posts)
  keys = {(r, d): (km, inc) for dist, r, d, km, inc in matches if dist <= matches[0][0] + STACK_M}
  both = len({r for r, _ in keys}) > 1
  if pos is not None:
    x, y = xy(*pos)
    hx, hy = math.sin(math.radians(bearing)), math.cos(math.radians(bearing))
  out = []
  for ev in events:
    k = (ev["road"], ev["dir"])
    if k not in keys or ev["km"] is None:
      continue
    km, inc = keys[k]
    sgn = 1 if inc else -1
    a, b = ev["km"], ev["km2"] if ev["km2"] is not None else ev["km"]
    lo, hi = (a, b) if (b - a) * sgn >= 0 else (b, a)   # lo = first reached along travel
    d_km = (lo - km) * sgn
    inside = (hi - km) * sgn >= -BEHIND_KM and d_km <= 0  # inside an event range (moving construction, closures)
    if inside:
      d_km = 0.
    if not (-BEHIND_KM <= d_km <= ALERT_KM + 1.0):
      continue
    if pos is not None and ev["lon"] is not None and d_km > 0:
      ex, ey = xy(ev["lon"], ev["lat"])
      dx, dy = ex - x, ey - y
      dist = math.hypot(dx, dy) / 1000.
      if (dx * hx + dy * hy) < -BEHIND_KM * 1000 or dist > ALERT_KM:
        continue
      d_km = dist if (dx * hx + dy * hy) > 0 else 0.
    elif d_km > ALERT_KM:
      continue
    e = dict(ev)
    if both:
      e["tag"] = "高架" if ev["road"].endswith("高架") else "平面"
    if inside and abs(hi - lo) >= RANGE_KM:
      e["range"] = (lo, hi)
    out.append((max(d_km, 0.), e))
  return sorted(out, key=lambda x: x[0])


def _k(v: float) -> str:
  return f"{v:.0f}K" if abs(v - round(v)) < 0.05 else f"{v:.1f}K"


def alert_text(d_km: float, ev: dict) -> str:
  if ev.get("range"):
    lo, hi = ev["range"]
    return f"本路段 {SHORT.get(ev['name'], ev['name'])}（{_k(lo)}–{_k(hi)}）" + (f" [{ev['tag']}]" if ev.get("tag") else "")
  where = "前方" if d_km < 0.1 else f"前方 {d_km:.1f} km"
  s = f"{where} {SHORT.get(ev['name'], ev['name'])}"
  if ev.get("lanes"):
    s += f"（{ev['lanes']}）"
  if ev.get("tag"):
    s += f" [{ev['tag']}]"
  return s


class Feed:
  def __init__(self):
    self.events, self.t = [], 0.
    self.lock = threading.Lock()

  def fetch_loop(self, stop):
    from openpilot.common.swaglog import cloudlog
    while not stop.is_set():
      try:
        req = urllib.request.Request(URL, headers={"User-Agent": "openpilot-hwyalert"})
        data = urllib.request.urlopen(req, timeout=15).read()
        evs = parse_feed(data, time.time())
        with self.lock:
          self.events, self.t = evs, time.monotonic()
        stop.wait(FETCH_S)
      except Exception as e:
        cloudlog.warning(f"hwyalert fetch failed: {e}")
        stop.wait(FETCH_RETRY_S)

  def get(self):
    with self.lock:
      return self.events if time.monotonic() - self.t < FEED_MAX_AGE_S else []


def write_json(path: str, obj: dict):
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    json.dump(obj, f, ensure_ascii=False)
  os.replace(tmp, path)


def write_state(state: dict):
  write_json(STATE_FILE, state)


class Tracker:
  """GPS fix -> freeway road / direction / km -> alerts and voice. Shared by the daemon and the offline replay."""
  def __init__(self, sections: Sections):
    self.S = sections
    self.streak, self.last_key = 0, None
    self.held, self.held_dist, self.last_t = [], 0., None   # last good match, metres driven since (no GPS)
    self.spoken, self.last_say_t = set(), -1e9

  def locate(self, t, fix, v):
    """fix = (lon, lat, bearing) or None -> matches"""
    dt = 0. if self.last_t is None else min(max(t - self.last_t, 0.), 5.)
    self.last_t = t
    if fix is not None:
      lon, lat, brg = fix
      if v > MATCH_MIN_V:
        m = self.S.match(lon, lat, brg)
      elif self.held:   # slow traffic: heading is noise, stay on the last road while within MATCH_M of it
        m = self.S.match(lon, lat, None, only={(r, d) for _, r, d, _, _ in self.held})
      else:
        m = []
      self.held, self.held_dist = m, 0.
      return m
    if self.held and self.held_dist < DR_MAX_KM * 1000:   # no GPS: carry the last match forward by speed
      self.held_dist += v * dt
      return [(dist, r, d, km + (1 if inc else -1) * self.held_dist / 1000., inc) for dist, r, d, km, inc in self.held]
    return []

  def step(self, t, fix, v, events):
    """-> (state dict, voice clips or None)"""
    m = self.locate(t, fix, v)
    key = (m[0][1], m[0][2]) if m else None
    self.streak = self.streak + 1 if key is not None and key == self.last_key else (1 if key else 0)
    self.last_key = key
    state = dict(road="", alerts=[])
    if not m or self.streak < MATCH_FRAMES:
      return state, None
    found = ahead(m, events, fix[:2] if fix else None, fix[2] if fix else 0.)[:MAX_ALERTS]
    state.update(road=f"{key[0]}{key[1]}", km=round(m[0][3], 2), dr=fix is None,
                 alerts=[dict(text=alert_text(d, e), dist=round(d, 2), name=e["name"], id=e["id"]) for d, e in found])
    say = None
    new = [(d, e) for d, e in found if e["id"] not in self.spoken]
    if new and t - self.last_say_t >= SAY_GAP_S:
      d, e = new[0]
      self.spoken.add(e["id"])
      self.last_say_t = t
      say = voice_clips(d, e["name"])
    return state, say


def main():
  from openpilot.cereal import messaging
  from openpilot.common.swaglog import cloudlog
  from openpilot.common.realtime import Ratekeeper

  tracker = Tracker(Sections())
  feed = Feed()
  stop = threading.Event()
  threading.Thread(target=feed.fetch_loop, args=(stop,), daemon=True).start()
  sm = messaging.SubMaster(["gpsLocation", "gpsLocationExternal", "carState"])
  rk = Ratekeeper(1.0, print_delay_threshold=None)
  last_ids, say_seq = (), 0
  try:
    while True:
      sm.update(0)
      state = dict(road="", alerts=[])
      if not os.path.exists(OFF_FILE):
        fix = None
        for s in ("gpsLocationExternal", "gpsLocation"):
          g = sm[s]
          if sm.recv_frame[s] > 0 and sm.alive[s] and g.hasFix and abs(g.latitude) > 1:
            fix = (g.longitude, g.latitude, g.bearingDeg)
            break
        state, say = tracker.step(time.monotonic(), fix, sm["carState"].vEgo, feed.get())
        if say and not os.path.exists(SAY_OFF_FILE):
          say_seq += 1
          write_json(SAY_FILE, dict(seq=say_seq, t=time.time(), clips=say))
        ids = tuple(a["id"] for a in state["alerts"])
        if ids != last_ids:
          cloudlog.info(f"hwyalert {state['road']} {state.get('km', 0):.2f} km{' (no GPS)' if state.get('dr') else ''}: " +
                        " | ".join(a["text"] for a in state["alerts"]))
          last_ids = ids
      state["t"] = time.time()
      write_state(state)
      rk.keep_time()
  finally:
    stop.set()
    try:
      os.remove(STATE_FILE)
    except OSError:
      pass


if __name__ == "__main__":
  main()
