"""Expressway incidents (公路局 THB, 台61-台88) from TDX, for hwyalertd.

TDX 道路事件 v1 `LiveEvent/Highway` is one nationwide call (every province road and county), made at most once every
FETCH_S (30 min, owner's limit: TDX bills per call; the last result is kept in EVENTS_CACHE so a restart or the next drive
does not call again early); only the expressways
(快速公路, RoadClass 1 in the 公路局 section list) are kept. Section polylines with km posts come from `Section/Highway` +
`SectionShape/Highway`, cached in SECTIONS_CACHE and refreshed every SECTIONS_MAX_AGE_D days.

TDX needs a member key: the owner writes CRED_FILE himself ({"client_id": "...", "client_secret": "..."}); without it this
feed stays empty and the freeway alerts work as before. Off: OFF_FILE.
"""
import gzip
import json
import os
import re
import threading
import time
import urllib.parse
import urllib.request
from collections import Counter

CRED_FILE = "/data/tdx_credentials.json"
OFF_FILE = "/data/hwyalert_thb_off"
SECTIONS_CACHE = "/data/hwyalert_thb_sections.json.gz"
SECTIONS_MAX_AGE_D = 7.
EVENTS_CACHE = "/data/hwyalert_thb_events.json"
TOKEN_URL = "https://tdx.transportdata.tw/auth/realms/TDXConnect/protocol/openid-connect/token"
API = "https://tdx.transportdata.tw/api/basic"
EVENTS_PATH = "/v1/Traffic/RoadEvent/LiveEvent/Highway"
SECTION_PATH = "/v2/Road/Traffic/Section/Highway"
SHAPE_PATH = "/v2/Road/Traffic/SectionShape/Highway"
FETCH_S = 1800.              # at most one events call per 30 min, failed ones included (owner's limit, 11/02)
FEED_MAX_AGE_S = 2400.       # events older than this (two calls missed) are dropped
EXPRESSWAY_CLASS = 1         # Section.RoadClass: 0 國道, 1 快速道路, 3 省道, ...

OPP = {"N": "S", "S": "N", "E": "W", "W": "E"}
DIRS = {"北向": "N", "南向": "S", "東向": "E", "西向": "W"}
AXIS = {"N": "NS", "S": "NS", "E": "EW", "W": "EW"}

# TDX EventType (即時路況資料標準): 1 事故, 2 施工, 3 壅塞, 4 管制, 5 天候, 6 災變, 7 活動, 8 其他. Names are mapped onto the
# 1968 names hwyalertd already shows and voices; NAMES are the extra ones (banner only, no voice clip).
NAMES = ("壅塞", "交通障礙")
_KEYWORDS = (("火", "車輛火災"), ("散落", "散落物"), ("掉落", "散落物"), ("拋錨", "故障車"), ("故障", "故障車"), ("動物", "動物誤闖"),
             ("行人", "行人誤闖"), ("自行車", "自行車誤闖"), ("機車", "機車誤闖"), ("逆向", "逆行車輛"), ("逆行", "逆行車輛"),
             ("坑洞", "坑洞"), ("路樹", "路樹"), ("倒樹", "路樹"), ("落石", "坍方"), ("坍", "坍方"), ("積水", "路面積水"),
             ("淹水", "路面積水"), ("霧", "濃霧"), ("封閉", "道路封閉"))


def event_name(ev: dict) -> str | None:
  t = ev.get("EventType")
  text = (ev.get("EventTitle") or "") + " " + (ev.get("Description") or "")
  imp = ev.get("Impact") or {}
  closed = imp.get("Severity") == 2 or str(imp.get("BlockedLanes", "")).strip() == "111111"
  for k, name in _KEYWORDS:
    if k in text:
      if name == "道路封閉" and not closed and t != 4:
        continue
      return name
  if t == 1:
    return "事故"
  if t == 2:
    return "施工維護"
  if t == 3:
    return "壅塞"
  if t == 4:
    return "道路封閉" if closed else None
  if t in (5, 6, 8) and "障礙" in text:
    return "交通障礙"
  return None


def norm_road(r: str) -> str:
  """台61線 / 臺61 -> 台61, 台62甲線 -> 台62甲"""
  return (r or "").replace("臺", "台").replace("線", "").strip()


def parse_km(s):
  m = re.match(r"\s*(\d+)K\+(\d+)", s or "")
  return int(m.group(1)) + int(m.group(2)) / 1000. if m else None


def wkt_points(s: str):
  return [[float(a), float(b)] for a, b in re.findall(r"(-?\d+\.\d+)\s+(-?\d+\.\d+)", s or "")]


def build_sections(sections: list, shapes: list):
  """TDX Section + SectionShape -> (rows [sid, road, dir, km_start, km_end, pts] as in freeway_sections.json.gz,
  road -> compass letter of the carriageway whose km posts increase)."""
  shape = {s.get("SectionID"): wkt_points(s.get("Geometry", "")) for s in shapes}
  raw = []
  for s in sections:
    if s.get("RoadClass") != EXPRESSWAY_CLASS:
      continue
    mile = s.get("SectionMile") or {}
    ks, ke = parse_km(mile.get("StartKM")), parse_km(mile.get("EndKM"))
    pts = shape.get(s.get("SectionID"))
    if ks is None or ke is None or not pts or len(pts) < 2 or ks == ke:
      continue
    raw.append((s["SectionID"], norm_road(s.get("RoadName")), (s.get("RoadDirection") or "").upper(), ks, ke, pts))
  # the section's RoadDirection is its local compass bearing (N, NE, SW ...), which wanders along a road: name each
  # carriageway by the letter that goes with increasing km posts on most of the road's sections
  votes: dict[str, Counter] = {}
  for _, road, d, ks, ke, _ in raw:
    c = votes.setdefault(road, Counter())
    for ch in d:
      if ch in OPP:
        c[ch if ke > ks else OPP[ch]] += 1
  inc = {road: c.most_common(1)[0][0] for road, c in votes.items() if c}
  rows = [[sid, road, inc[road] if ke > ks else OPP[inc[road]], ks, ke, pts] for sid, road, d, ks, ke, pts in raw if road in inc]
  return rows, inc


def parse_events(data: dict, now_s: float, inc: dict, iso_to_epoch, long_term_h: float, locate=None):
  """TDX LiveEvents -> hwyalertd event dicts on the expressways in `inc`.
  locate(road, dir, lon, lat) -> km, for events that give a position but no km post."""
  out = []
  for ev in data.get("LiveEvents") or []:
    loc = (ev.get("Location") or {}).get("FreeExpressHighway") or {}
    road = norm_road(loc.get("Road"))
    if road not in inc or ev.get("LocationType") in (1, 2):   # 1 / 2 = interchange ramp / ramp entry-exit
      continue
    name = event_name(ev)
    if name is None:
      continue
    if name == "施工維護":
      try:
        if now_s - iso_to_epoch(ev.get("EffectiveTime") or "") > long_term_h * 3600:
          continue
      except Exception:
        pass
    m = re.search(r"POINT\s*\(\s*([\d.]+)\s+([\d.]+)", ev.get("Positions") or "")
    lon, lat = (float(m.group(1)), float(m.group(2))) if m else (None, None)
    d = DIRS.get((loc.get("Direction") or "").strip())
    axis = AXIS[inc[road]]
    dirs = [d] if d in OPP and d in axis else [inc[road], OPP[inc[road]]] if (loc.get("Direction") or "").strip() == "雙向" else []
    imp = ev.get("Impact") or {}
    for dd in dirs:
      km, km2 = parse_km(loc.get("StartKM")), parse_km(loc.get("EndKM"))
      if (km is None or (km == 0 and km2 in (0, None))) and lon is not None and locate is not None:
        km = locate(road, dd, lon, lat)
        km2 = None
      if km is None:
        continue
      out.append(dict(id=f"thb:{ev.get('EventID')}:{dd}", road=road, dir=dd, km=km, km2=km2, lon=lon, lat=lat, name=name,
                      lanes=lanes_text(str(imp.get("BlockedLanes", ""))), title=ev.get("EventTitle") or "", src="thb"))
  return out


def lanes_text(s: str) -> str:
  from openpilot.selfdrive.hwyalert.hwyalertd import lanes_text as lt
  return lt(s)


class ThbFeed:
  """Background fetcher; get() / age() like hwyalertd.Feed. on_sections(rows) is called once the sections are loaded."""
  def __init__(self, iso_to_epoch, long_term_h, on_sections, locate):
    self.events, self.t = [], -1e9   # t: wall time (time.time()) of the call that produced the events
    self.inc: dict = {}
    self.lock = threading.Lock()
    self.token, self.token_exp = None, 0.
    self._iso, self._lt, self._on_sections, self._locate = iso_to_epoch, long_term_h, on_sections, locate

  def _token(self):
    if self.token and time.time() < self.token_exp - 300:
      return self.token
    cred = json.load(open(CRED_FILE))
    body = urllib.parse.urlencode(dict(grant_type="client_credentials", client_id=cred["client_id"],
                                       client_secret=cred["client_secret"])).encode()
    req = urllib.request.Request(TOKEN_URL, data=body, headers={"Content-Type": "application/x-www-form-urlencoded"})
    r = json.loads(urllib.request.urlopen(req, timeout=15).read())
    self.token, self.token_exp = r["access_token"], time.time() + float(r.get("expires_in", 3600))
    return self.token

  def _get(self, path):
    req = urllib.request.Request(f"{API}{path}?%24format=JSON",
                                 headers={"Authorization": f"Bearer {self._token()}", "Accept-Encoding": "gzip",
                                          "User-Agent": "openpilot-hwyalert"})
    r = urllib.request.urlopen(req, timeout=30)
    data = r.read()
    if r.headers.get("Content-Encoding") == "gzip":
      data = gzip.decompress(data)
    return json.loads(data)

  @staticmethod
  def _items(d: dict) -> list:
    return next((v for v in d.values() if isinstance(v, list)), [])

  def _load_sections(self):
    if os.path.exists(SECTIONS_CACHE) and time.time() - os.path.getmtime(SECTIONS_CACHE) < SECTIONS_MAX_AGE_D * 86400:
      c = json.loads(gzip.open(SECTIONS_CACHE).read())
      return c["rows"], c["inc"]
    try:
      rows, inc = build_sections(self._items(self._get(SECTION_PATH)), self._items(self._get(SHAPE_PATH)))
    except Exception:
      if os.path.exists(SECTIONS_CACHE):   # stale cache beats nothing
        c = json.loads(gzip.open(SECTIONS_CACHE).read())
        return c["rows"], c["inc"]
      raise
    if rows:
      tmp = SECTIONS_CACHE + ".tmp"
      with gzip.open(tmp, "wt") as f:
        json.dump(dict(rows=rows, inc=inc), f, ensure_ascii=False)
      os.replace(tmp, SECTIONS_CACHE)
    return rows, inc

  def _last_call(self):
    """(wall time of the last events call, its raw response or None) from EVENTS_CACHE."""
    try:
      c = json.load(open(EVENTS_CACHE))
      return float(c["t"]), c.get("data")
    except Exception:
      return -1e9, None

  def _save_call(self, t, data):
    try:
      tmp = EVENTS_CACHE + ".tmp"
      with open(tmp, "w") as f:
        json.dump(dict(t=t, data=data), f, ensure_ascii=False)
      os.replace(tmp, EVENTS_CACHE)
    except Exception:
      pass

  def fetch_loop(self, stop):
    from openpilot.common.swaglog import cloudlog
    warned = False
    while not stop.is_set():
      if os.path.exists(OFF_FILE) or not os.path.exists(CRED_FILE):
        if not warned:
          cloudlog.info(f"hwyalert thb: off ({OFF_FILE} or no {CRED_FILE})")
          warned = True
        stop.wait(FETCH_S)
        continue
      try:
        if not self.inc:
          rows, inc = self._load_sections()
          self._on_sections(rows)
          self.inc = inc
          cloudlog.info(f"hwyalert thb: {len(rows)} expressway sections, roads {sorted(inc)}")
        t_last, data = self._last_call()
        if data is not None and self.t < t_last:   # the previous call (this or an earlier drive) is still recent enough
          self._publish(data, t_last)
        wait = t_last + FETCH_S - time.time()
        if wait > 0:
          stop.wait(min(wait, FETCH_S))
          continue
        t_call = time.time()
        self._save_call(t_call, None)   # counts as a call even if it fails
        data = self._get(EVENTS_PATH)
        self._save_call(t_call, data)
        self._publish(data, t_call)
        cloudlog.info(f"hwyalert thb: {len(self.events)} expressway events")
      except Exception as e:
        cloudlog.warning(f"hwyalert thb fetch failed: {e}")
        stop.wait(60.)   # the next events call still waits for FETCH_S since the last one (EVENTS_CACHE)

  def _publish(self, data, t):
    evs = parse_events(data, time.time(), self.inc, self._iso, self._lt, self._locate)
    with self.lock:
      self.events, self.t = evs, t

  def get(self):
    with self.lock:
      return self.events if time.time() - self.t < FEED_MAX_AGE_S else []

  def age(self):
    with self.lock:
      return time.time() - self.t
