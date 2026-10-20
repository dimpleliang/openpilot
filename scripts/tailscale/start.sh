#!/usr/bin/env bash
# Start tailscaled if it is installed in /data/tailscale (install.sh). Called from launch_chffrplus.sh on boot; a no-op
# without the install, and it never blocks or fails the openpilot launch. State and the control socket stay in /data.
TS=/data/tailscale
[ -x "$TS/tailscaled" ] || exit 0
pgrep -x tailscaled > /dev/null && exit 0
sudo mkdir -p /dev/net
[ -c /dev/net/tun ] || sudo mknod /dev/net/tun c 10 200 2> /dev/null || true
sudo nohup "$TS/tailscaled" --state="$TS/tailscaled.state" --socket="$TS/tailscaled.sock" --statedir="$TS" \
  --port=41641 > "$TS/tailscaled.log" 2>&1 < /dev/null &
disown || true
exit 0
