// ars510 TDX relay (Cloudflare Worker, free plan), on demand with a monthly point budget.
// TDX is called only when a car asks and the stored copy is older than the current interval; every other request gets the
// stored copy. No cron: nobody driving = no TDX calls. Data: MOTC TDX (政府資料開放授權條款 1.0).
//
// Interval for events: MIN_INTERVAL_S while this month's TDX points are on or below a straight-line pace to BUDGET_POINTS,
// longer the further ahead of pace (up to MAX_INTERVAL_S), MAX_INTERVAL_S from 90 % of the budget, and no TDX
// calls at all once it is spent (the stored copy is served until the month ends). TDX basic
// services cost 1 point per 1,500 calls plus 1 point per 150 MB; the free member gets 3 points a month (service stops at 105 %).
// Sections / shapes: every 180 days (owner).
//
//   GET /v1/events|section|shape   header X-Access-Token: <ACCESS_TOKEN>   optional ?since=<epoch s> -> 304 when not newer
//   GET /health                     (no token) ages, errors, this month's calls / MB / points, current events interval
// Secrets (wrangler secret put): TDX_CLIENT_ID, TDX_CLIENT_SECRET, ACCESS_TOKEN.  KV binding: TDX.
// CPU: the free plan allows 10 ms per invocation; TDX responses are streamed (tee) into KV and to the car, never parsed.
// KV writes (free 1,000 / day): 3 per TDX call (data, its time, usage) + 2 per events call for the change log.
// Change log (owner, 11/03): every events call compares the event list with the previous call (EventID + LastUpdateTime +
// EventStep) and counts per Taiwan day: calls, calls that changed anything, events added / removed / updated. That is roughly
// what an MQTT push would have sent (TDX has no MQTT for road events). Measured only while cars ask (no extra TDX calls).

const API = "https://tdx.transportdata.tw/api/basic";
const TOKEN_URL = "https://tdx.transportdata.tw/auth/realms/TDXConnect/protocol/openid-connect/token";
const SOURCES = {
  events: { path: "/v1/Traffic/RoadEvent/LiveEvent/Highway", estMB: 0.1 },
  section: { path: "/v2/Road/Traffic/Section/Highway", maxAgeS: 180 * 86400, estMB: 2.4 },   // owner: twice a year at most
  shape: { path: "/v2/Road/Traffic/SectionShape/Highway", maxAgeS: 180 * 86400, estMB: 5.8 },
  taipei: { path: "/v1/Traffic/RoadEvent/LiveEvent/City/Taipei", maxAgeS: 1800, estMB: 0.1 },   // city roads (市民／建國高架), trial
};
const BUDGET_POINTS = 2.8;          // of the free 3, leaving room for the token calls and rounding
const MIN_INTERVAL_S = 120;         // fastest events refresh (owner wants it dense)
const MAX_INTERVAL_S = 3600;
const CALLS_PER_POINT = 1500, MB_PER_POINT = 150;

const month = () => new Date(Date.now() + 8 * 3600e3).toISOString().slice(0, 7);   // Taiwan month, as TDX bills

async function usage(env) {
  return JSON.parse((await env.TDX.get(`usage_${month()}`)) || '{"calls":0,"mb":0}');
}
const points = u => u.calls / CALLS_PER_POINT + u.mb / MB_PER_POINT;

function eventsInterval(u) {
  const now = new Date(Date.now() + 8 * 3600e3);
  const start = Date.UTC(now.getUTCFullYear(), now.getUTCMonth(), 1);
  const end = Date.UTC(now.getUTCFullYear(), now.getUTCMonth() + 1, 1);
  const timeFrac = Math.max((now - start) / (end - start), 1 / 30);   // first day: as if a day had passed
  const used = points(u) / BUDGET_POINTS;
  if (used >= 1) return Infinity;                 // budget spent: serve the stored copy until next month
  if (used >= 0.9) return MAX_INTERVAL_S;
  const ahead = used / timeFrac;                                      // 1 = exactly on pace
  return ahead <= 1 ? MIN_INTERVAL_S : Math.min(MAX_INTERVAL_S, MIN_INTERVAL_S * ahead * ahead);
}

async function tdxToken(env) {
  const cached = await env.TDX.getWithMetadata("token");
  if (cached.value && cached.metadata && cached.metadata.exp > Date.now() / 1000 + 300) return cached.value;
  const body = new URLSearchParams({ grant_type: "client_credentials", client_id: env.TDX_CLIENT_ID, client_secret: env.TDX_CLIENT_SECRET });
  const r = await fetch(TOKEN_URL, { method: "POST", body, headers: { "Content-Type": "application/x-www-form-urlencoded" } });
  if (!r.ok) throw new Error(`token ${r.status}`);
  const j = await r.json();
  await env.TDX.put("token", j.access_token, { metadata: { exp: Date.now() / 1000 + Number(j.expires_in || 3600) } });
  return j.access_token;
}

// -> { body: ReadableStream, t } fresh from TDX (also being written to KV), or null on failure
async function callTdx(env, ctx, name, u) {
  const t = Date.now() / 1000;
  try {
    const r = await fetch(`${API}${SOURCES[name].path}?%24format=JSON`, {
      headers: { Authorization: `Bearer ${await tdxToken(env)}`, "Accept-Encoding": "gzip" },
    });
    const mb = Number(r.headers.get("Content-Length")) / 1e6 || SOURCES[name].estMB;
    ctx.waitUntil(env.TDX.put(`usage_${month()}`, JSON.stringify({ calls: u.calls + 1, mb: u.mb + mb }),
                              { expirationTtl: 70 * 86400 }));
    if (!r.ok) throw new Error(`TDX ${r.status}`);
    const [toKv, toRest] = r.body.tee();
    let toCar = toRest;
    if (name === "events") {
      const [a, b] = toRest.tee();
      toCar = a;
      ctx.waitUntil(logChanges(env, b, t).catch(e => env.TDX.put("changes_error", `${e}`)));
    }
    ctx.waitUntil((async () => {
      await env.TDX.put(name, toKv, { metadata: { t } });
      await env.TDX.put(`${name}_t`, String(t));
      if (await env.TDX.get(`${name}_error`)) await env.TDX.delete(`${name}_error`);
    })());
    return { body: toCar, t };
  } catch (e) {
    ctx.waitUntil(env.TDX.put(`${name}_error`, `${e} ${new Date().toISOString()}`));
    return null;
  }
}

const day = t => new Date(t * 1000 + 8 * 3600e3).toISOString().slice(0, 10);

async function logChanges(env, stream, t) {
  const d = JSON.parse(await new Response(stream).text());
  const now = {};
  for (const e of d.LiveEvents || []) now[e.EventID] = `${e.LastUpdateTime || ""}|${e.EventStep ?? ""}`;
  const prev = JSON.parse((await env.TDX.get("events_sig")) || "null");
  const key = `changes_${day(t)}`;
  const c = JSON.parse((await env.TDX.get(key)) || '{"calls":0,"changed":0,"added":0,"removed":0,"updated":0,"gaps_s":[]}');
  c.calls += 1;
  if (prev) {
    let added = 0, removed = 0, updated = 0;
    for (const id in now) { if (!(id in prev.ev)) added++; else if (prev.ev[id] !== now[id]) updated++; }
    for (const id in prev.ev) if (!(id in now)) removed++;
    c.added += added; c.removed += removed; c.updated += updated;
    if (added + removed + updated) c.changed += 1;
    if (c.gaps_s.length < 200) c.gaps_s.push(Math.round(t - prev.t));   // seconds since the previous call (context)
  }
  c.events = Object.keys(now).length;
  await env.TDX.put("events_sig", JSON.stringify({ t, ev: now }));
  await env.TDX.put(key, JSON.stringify(c), { expirationTtl: 60 * 86400 });
}

export default {
  async fetch(req, env, ctx) {
    const url = new URL(req.url);
    const now = Date.now() / 1000;
    if (url.pathname === "/health") {
      const u = await usage(env);
      const out = { month: month(), tdx_calls: u.calls, tdx_mb: Math.round(u.mb * 10) / 10,
                    points: Math.round(points(u) * 1000) / 1000, budget: BUDGET_POINTS,
                    events_interval_s: Number.isFinite(eventsInterval(u)) ? Math.round(eventsInterval(u)) : "budget spent" };
      out.changes = {};
      for (let i = 0; i < 7; i++) {
        const k = day(now - i * 86400), v = await env.TDX.get(`changes_${k}`);
        if (v) { const c = JSON.parse(v); delete c.gaps_s; out.changes[k] = c; }
      }
      for (const name of Object.keys(SOURCES)) {
        const last = Number(await env.TDX.get(`${name}_t`));
        out[name] = last ? Math.round(now - last) + " s ago" : "never";
        const err = await env.TDX.get(`${name}_error`);
        if (err) out[`${name}_error`] = err;
      }
      return Response.json(out);
    }
    const name = url.pathname.replace(/^\/v1\//, "");
    if (!(name in SOURCES)) return new Response("not found", { status: 404 });
    if (!env.ACCESS_TOKEN || req.headers.get("X-Access-Token") !== env.ACCESS_TOKEN) return new Response("forbidden", { status: 403 });

    const since = Number(url.searchParams.get("since"));
    const last = Number(await env.TDX.get(`${name}_t`)) || 0;
    const u = await usage(env);
    const maxAge = SOURCES[name].maxAgeS || eventsInterval(u);
    const headers = (t, fresh) => ({ "X-Fetched-At": String(t), "X-From-Tdx-Now": fresh ? "1" : "0", "X-Interval-S": String(Number.isFinite(maxAge) ? Math.round(maxAge) : -1),
                                     "Content-Type": "application/json", "X-Data-Source": "MOTC TDX" });
    if (now - last >= maxAge) {
      const got = await callTdx(env, ctx, name, u);
      if (got) return new Response(got.body, { headers: headers(got.t, true) });
    }
    if (since && last && last <= since) return new Response(null, { status: 304, headers: headers(last, false) });
    const { value, metadata } = await env.TDX.getWithMetadata(name, { type: "stream" });
    if (!value) return new Response("no data yet", { status: 503 });
    return new Response(value, { headers: headers(metadata.t, false) });
  },
};
