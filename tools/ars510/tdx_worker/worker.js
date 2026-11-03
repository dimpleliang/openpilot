// ars510 TDX relay (Cloudflare Worker, free plan), on demand: TDX is called only when a car asks and the stored copy is
// older than the source's max age (events 30 min, sections 180 days); every other request within that window gets the stored
// copy. No cron: nobody driving = no TDX calls. Data: MOTC TDX (政府資料開放授權條款 1.0).
//
//   GET /v1/events|section|shape   header X-Access-Token: <ACCESS_TOKEN>   optional ?since=<epoch s> -> 304 when not newer
//   GET /health                     (no token) age of each stored copy and the last error
// Secrets (wrangler secret put): TDX_CLIENT_ID, TDX_CLIENT_SECRET, ACCESS_TOKEN.  KV binding: TDX.
// CPU: the free plan allows 10 ms per invocation; TDX responses are streamed (tee) into KV and to the car, never parsed.

const API = "https://tdx.transportdata.tw/api/basic";
const TOKEN_URL = "https://tdx.transportdata.tw/auth/realms/TDXConnect/protocol/openid-connect/token";
const SOURCES = {
  events: { path: "/v1/Traffic/RoadEvent/LiveEvent/Highway", maxAgeS: 30 * 60 },
  section: { path: "/v2/Road/Traffic/Section/Highway", maxAgeS: 180 * 86400 },   // owner: road sections at most twice a year
  shape: { path: "/v2/Road/Traffic/SectionShape/Highway", maxAgeS: 180 * 86400 },
};
const LOCK_S = 60;   // one TDX call at a time per source: a second car arriving during a call gets the old copy

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
async function callTdx(env, ctx, name) {
  const t = Date.now() / 1000;
  await env.TDX.put(`${name}_lock`, String(t), { expirationTtl: LOCK_S });
  try {
    const r = await fetch(`${API}${SOURCES[name].path}?%24format=JSON`, {
      headers: { Authorization: `Bearer ${await tdxToken(env)}`, "Accept-Encoding": "gzip" },
    });
    if (!r.ok) throw new Error(`TDX ${r.status}`);
    const [toKv, toCar] = r.body.tee();
    ctx.waitUntil((async () => {
      await env.TDX.put(name, toKv, { metadata: { t } });
      await env.TDX.put(`${name}_t`, String(t));
      await env.TDX.delete(`${name}_error`);
      await env.TDX.delete(`${name}_lock`);
    })());
    return { body: toCar, t };
  } catch (e) {
    ctx.waitUntil(env.TDX.put(`${name}_error`, `${e} ${new Date().toISOString()}`));
    return null;
  }
}

export default {
  async fetch(req, env, ctx) {
    const url = new URL(req.url);
    const now = Date.now() / 1000;
    if (url.pathname === "/health") {
      const out = {};
      for (const name of Object.keys(SOURCES)) {
        const last = Number(await env.TDX.get(`${name}_t`));
        out[name] = last ? Math.round(now - last) + " s ago" : "never";
        out[`${name}_error`] = await env.TDX.get(`${name}_error`);
      }
      return Response.json(out);
    }
    const name = url.pathname.replace(/^\/v1\//, "");
    if (!(name in SOURCES)) return new Response("not found", { status: 404 });
    if (!env.ACCESS_TOKEN || req.headers.get("X-Access-Token") !== env.ACCESS_TOKEN) return new Response("forbidden", { status: 403 });

    const since = Number(url.searchParams.get("since"));
    const last = Number(await env.TDX.get(`${name}_t`)) || 0;
    const stale = now - last >= SOURCES[name].maxAgeS;
    const headers = (t, fresh) => ({ "X-Fetched-At": String(t), "X-From-Tdx-Now": fresh ? "1" : "0",
                                     "Content-Type": "application/json", "X-Data-Source": "MOTC TDX" });
    if (stale && !(await env.TDX.get(`${name}_lock`))) {
      const got = await callTdx(env, ctx, name);
      if (got) return new Response(got.body, { headers: headers(got.t, true) });
    }
    // within the window (or TDX failed / another call in flight): the stored copy
    if (since && last && last <= since) return new Response(null, { status: 304, headers: headers(last, false) });
    const { value, metadata } = await env.TDX.getWithMetadata(name, { type: "stream" });
    if (!value) return new Response("no data yet", { status: 503 });
    return new Response(value, { headers: headers(metadata.t, false) });
  },
};
