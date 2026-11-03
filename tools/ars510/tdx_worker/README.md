# ars510 TDX relay (Cloudflare Worker)

Calls TDX (公路局 expressway events) every 30 min with your key and serves the result to the car(s). The car never holds the
TDX key, and TDX sees 48 event calls a day however many cars or drives there are. Free plan: 100,000 requests a day for
the whole Cloudflare account; going over only returns errors (no charge) unless the account is upgraded.

## Deploy (on your Mac, once)

    cd tools/ars510/tdx_worker
    npx wrangler login                       # opens the browser, your Cloudflare account
    npx wrangler kv namespace create TDX     # copy the printed id into wrangler.toml (REPLACE_WITH_KV_NAMESPACE_ID)
    npx wrangler secret put TDX_CLIENT_ID    # paste the TDX Client Id
    npx wrangler secret put TDX_CLIENT_SECRET
    npx wrangler secret put ACCESS_TOKEN     # any long random string, e.g. from: openssl rand -hex 24
    npx wrangler deploy                      # prints https://ars510-tdx.<your-subdomain>.workers.dev

Wait for the first :00 / :30, then check `https://ars510-tdx.<sub>.workers.dev/health` (events "… s ago", no error).

## Car

Write `/data/tdx_proxy.json` on the comma (the token is the ACCESS_TOKEN above):

    {"url": "https://ars510-tdx.<sub>.workers.dev", "token": "<ACCESS_TOKEN>"}

hwyalertd then polls the Worker every 2 min (304 when nothing is new) and loads the expressway sections from it once a week.
With `/data/tdx_proxy.json` absent it falls back to calling TDX directly with `/data/tdx_credentials.json` (one call per 30 min).
Other cars using this fork: give them the URL and the ACCESS_TOKEN; rotate the token with `wrangler secret put ACCESS_TOKEN`.

Data source: MOTC TDX, Open Government Data License 1.0 (attribution required when shown or redistributed).
