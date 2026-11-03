// ars510 TDX relay (Cloudflare Worker, free plan). Holds the TDX key, calls TDX on its own schedule and serves the stored
// responses to the cars, so TDX sees 48 event calls a day however many cars poll. Data: MOTC TDX (政府資料開放授權條款 1.0).
//
//   cron */30   -> LiveEvent/Highway (公路局 events, one nationwide call) into KV "events"
//                  Section/Highway + SectionShape/Highway into KV "section" / "shape" when older than 7 days
//   GET /v1/events|section|shape   header X-Access-Token: <ACCESS_TOKEN>   optional ?since=<epoch s> -> 304 when not newer
//   GET /health                     (no token) last call times
// Secrets (wrangler secret put): TDX_CLIENT_ID, TDX_CLIENT_SECRET, ACCESS_TOKEN.  KV binding: TDX.
// CPU: the free plan allows 10 ms per invocation; responses are stored as raw bytes (no JSON parsing), network waits are free.

const API = "https://tdx.transportdata.tw/api/basic";
const TOKEN_URL = "https://tdx.transportdata.tw/auth/realms/TDXConnect/protocol/openid-connect/token";
const SOURCES = {
  events: { path: "/v1/Traffic/RoadEvent/LiveEvent/Highway", maxAgeS: 0 },
  section: { path: "/v2/Road/Traffic/Section/Highway", maxAgeS: 7 * 86400 },
  shape: { path: "/v2/Road/Traffic/SectionShape/Highway", maxAgeS: 7 * 86400 },
};

async function tdxToken(env) {
  const cached = await env.TDX.getWithMetadata("token");
  if (cached.value && cached.metadata && cached.metadata.exp > Date.now() / 1000 + 300) return cached.value;
  const body = new URLSearchParams({ grant_type: "client_credentials", client_id: env.TDX_CLIENT_ID, client_secret: env.TDX_CLIENT_SECRET });
  const r = await fetch(TOKEN_URL, { method: "POST", body, headers: { "Content-Type": "application/x-www-form-urlencoded" } });
  if (!r.ok) throw new Error(`token ${r.status}`);
  const j = await r.json();
  const exp = Date.now() / 1000 + Number(j.expires_in || 3600);
  await env.TDX.put("token", j.access_token, { metadata: { exp } });   // one KV write a day
  return j.access_token;
}

async function refresh(env, name) {
  const src = SOURCES[name];
  if (src.maxAgeS) {
    const last = Number(await env.TDX.get(`${name}_t`));
    if (last && Date.now() / 1000 - last < src.maxAgeS) return "fresh";
  }
  const t = Date.now() / 1000;
  const r = await fetch(`${API}${src.path}?%24format=JSON`, {
    headers: { Authorization: `Bearer ${await tdxToken(env)}`, "Accept-Encoding": "gzip" },
  });
  if (!r.ok) {
    await env.TDX.put(`${name}_error`, `${r.status} ${new Date().toISOString()}`);
    return `error ${r.status}`;
  }
  // stream straight into KV: no buffering or parsing in the Worker (10 ms CPU on the free plan)
  await env.TDX.put(name, r.body, { metadata: { t } });
  await env.TDX.put(`${name}_t`, String(t));   // small key for /health and the weekly check (2 writes per refresh)
  await env.TDX.delete(`${name}_error`);
  return "ok";
}

export default {
  async scheduled(event, env, ctx) {
    // one call per source per run; events every run, the static sections weekly
    ctx.waitUntil((async () => {
      for (const name of ["events", "section", "shape"]) {
        try { await refresh(env, name); } catch (e) { await env.TDX.put(`${name}_error`, `${e} ${new Date().toISOString()}`); }
      }
    })());
  },

  async fetch(req, env) {
    const url = new URL(req.url);
    if (url.pathname === "/health") {
      const out = {};
      for (const name of Object.keys(SOURCES)) {
        const last = Number(await env.TDX.get(`${name}_t`));
        out[name] = last ? Math.round(Date.now() / 1000 - last) + " s ago" : "never";
        out[`${name}_error`] = await env.TDX.get(`${name}_error`);
      }
      return Response.json(out);
    }
    const name = url.pathname.replace(/^\/v1\//, "");
    if (!(name in SOURCES)) return new Response("not found", { status: 404 });
    if (!env.ACCESS_TOKEN || req.headers.get("X-Access-Token") !== env.ACCESS_TOKEN) return new Response("forbidden", { status: 403 });
    const { value, metadata } = await env.TDX.getWithMetadata(name, { type: "stream" });
    if (!value) return new Response("no data yet", { status: 503 });
    const headers = { "X-Fetched-At": String(metadata.t), "Content-Type": "application/json", "X-Data-Source": "MOTC TDX" };
    const since = Number(url.searchParams.get("since"));
    if (since && metadata.t <= since) {
      value.cancel();
      return new Response(null, { status: 304, headers });
    }
    return new Response(value, { headers });
  },
};
