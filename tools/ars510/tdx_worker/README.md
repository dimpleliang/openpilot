# ars510 TDX relay (Cloudflare Worker)

Calls TDX (公路局 expressway events) only when a car asks and the stored copy is older than the current interval; every
other request gets the stored copy. No schedule: nobody driving means no TDX calls. The car never holds the TDX key.

The interval follows a monthly point budget (2.8 of the free member's 3 points; TDX: 1 point per 1,500 calls + 1 point per
150 MB): 2 min while the month's usage is on or below a straight-line pace, longer when ahead of pace, 60 min from 90 %,
and no TDX calls once the budget is spent (the stored copy is served until the month ends). Simulated month, car asking
every minute while driving: 1 h/day -> every 2 min (1.2 points); 2 h/day -> 2 min (2.4); 4 h/day -> 3.8 min (2.55);
24 h/day -> 21 min (2.8). Sections / shapes: every 180 days. `/health` shows this month's calls, MB, points and interval, and per day how often the event list actually changed
(calls / changed / added / removed / updated) — roughly what an MQTT push would send; measured only while cars ask.
Cloudflare free plan: 100,000 requests a day for the whole account; going over only returns errors (no charge).

## Deploy (on your Mac, once)

    cd tools/ars510/tdx_worker
    npx wrangler login                       # opens the browser, your Cloudflare account
    npx wrangler kv namespace create TDX     # copy the printed id into wrangler.toml (REPLACE_WITH_KV_NAMESPACE_ID)
    npx wrangler secret put TDX_CLIENT_ID    # paste the TDX Client Id
    npx wrangler secret put TDX_CLIENT_SECRET
    npx wrangler secret put ACCESS_TOKEN     # any long random string, e.g. from: openssl rand -hex 24
    npx wrangler deploy                      # prints https://ars510-tdx.<your-subdomain>.workers.dev

After the car's first request, check `https://ars510-tdx.<sub>.workers.dev/health` (events "… s ago", no error).

## Car

Write `/data/tdx_proxy.json` on the comma (the token is the ACCESS_TOKEN above):

    {"url": "https://ars510-tdx.<sub>.workers.dev", "token": "<ACCESS_TOKEN>"}

hwyalertd then polls the Worker every 2 min (304 when nothing is new) and loads the expressway sections from it at most every 180 days.
With `/data/tdx_proxy.json` absent it falls back to calling TDX directly with `/data/tdx_credentials.json` (one call per 30 min).
Other cars using this fork: give them the URL and the ACCESS_TOKEN; rotate the token with `wrangler secret put ACCESS_TOKEN`.

Data source: MOTC TDX, Open Government Data License 1.0 (attribution required when shown or redistributed).
