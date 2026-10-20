#!/usr/bin/env bash
# Install Tailscale on the comma (dimpleliang/ars510). Everything lives in /data/tailscale, which survives AGNOS updates;
# the launch script starts it on every boot (see start.sh). Run once on the device:
#   bash /data/openpilot/scripts/tailscale/install.sh
# then authenticate once:
#   sudo /data/tailscale/tailscale --socket=/data/tailscale/tailscaled.sock up --ssh=false --hostname=comma
set -e
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
VER="${1:-1.104.1}"
TS=/data/tailscale
mkdir -p "$TS"
cd "$TS"
if [ ! -x "$TS/tailscaled" ] || [ "$(cat "$TS/VERSION" 2>/dev/null)" != "$VER" ]; then
  echo "downloading tailscale $VER (arm64)"
  curl -fsSL "https://pkgs.tailscale.com/stable/tailscale_${VER}_arm64.tgz" -o ts.tgz
  tar xzf ts.tgz
  mv "tailscale_${VER}_arm64/tailscaled" "tailscale_${VER}_arm64/tailscale" "$TS/"
  rm -rf ts.tgz "tailscale_${VER}_arm64"
  echo "$VER" > "$TS/VERSION"
fi
chmod +x "$TS/tailscaled" "$TS/tailscale"
bash "$HERE/start.sh"
sleep 2
sudo "$TS/tailscale" --socket="$TS/tailscaled.sock" version
echo "installed. authenticate with:"
echo "  sudo $TS/tailscale --socket=$TS/tailscaled.sock up --hostname=comma"
