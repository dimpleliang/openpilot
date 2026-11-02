#!/bin/bash
# ars510: pull every finished segment's rlog/qlog/qcamera (low-res road video) from the comma over Tailscale, then free
# the rlog's space on the device. Full-resolution video on demand: pull_video.sh.
# Runs from launchd every 15 min (see ts_pull.plist). Only while the device is offroad (pulling while driving starved
# modeld of memory, see below); the newest segment is skipped anyway. The rlog is deleted on the device only after it arrived
# here with the same size; the video stays there for the device's own cleanup / comma connect, so the camera can
# still be reviewed for a few days.
#
#   ~/ars510_rlogs/ts_pull.sh            normal run
#   KEEP=1 ~/ars510_rlogs/ts_pull.sh     pull only, delete nothing on the device
set -u
DEST=~/ars510_rlogs/raw
LOG=~/ars510_rlogs/ts_pull.log
HOST=comma-ts                       # ssh alias over Tailscale (see the ~/.ssh/config block in the setup notes)
O="-n -o ConnectTimeout=20 -o BatchMode=yes"   # -n: never read the loop stdin
mkdir -p "$DEST"
exec >> "$LOG" 2>&1
echo "== $(date '+%F %T')"
if ! ssh $O $HOST true 2>/dev/null; then echo "comma offline"; exit 0; fi
off=$(ssh $O $HOST 'cat /data/params/d/IsOffroad 2>/dev/null')
# 11/02: pulling while driving caused page-cache reclaim storms on the device (kswapd0 up to 32 %, tailscaled 27 %), two
# modeld frame drops and two "locationd Temporary Error" alerts (route 38, 11:17 / 11:28). Offroad only.
if [ "$off" != "1" ] && [ "${ONROAD_OK:-0}" != "1" ]; then echo "onroad: skipped"; exit 0; fi
# one listing: segment and rlog size (0 = rlog already removed), oldest first
list=$(ssh $O $HOST 'cd /data/media/0/realdata && for d in $(ls -d */ | tr -d / | grep -v boot | sort); do echo "$d $(stat -c %s $d/rlog.zst 2>/dev/null || echo 0)"; done')
[ "$off" = "1" ] || list=$(echo "$list" | sed '$d')   # newest segment is still being written while onroad
n=0; freed=0
while read -r s remote; do
  [ -n "$s" ] || continue
  [ "$remote" = "0" ] && [ -s "$DEST/$s/rlog.zst" ] && [ -s "$DEST/$s/qcamera.ts" ] && continue   # already pulled and freed
  mkdir -p "$DEST/$s"
  rsync -a --partial --timeout=120 --rsync-path="nice -n 10 rsync" \
        --include='rlog*' --include='qlog*' --include='qcamera.ts' --exclude='*' \
        "$HOST:/data/media/0/realdata/$s/" "$DEST/$s/" < /dev/null || { echo "rsync failed $s"; continue; }
  n=$((n + 1))
  if [ "${KEEP:-0}" != "1" ] && [ "$remote" -gt 0 ]; then
    local_=$(stat -f %z "$DEST/$s/rlog.zst" 2>/dev/null || echo -1)
    if [ "$remote" = "$local_" ]; then
      ssh $O $HOST "rm -f /data/media/0/realdata/$s/rlog.zst" && freed=$((freed + 1))
    else
      echo "size mismatch $s remote=$remote local=$local_ (kept on device)"
    fi
  fi
done <<< "$list"
echo "done: $n segments pulled, $freed rlogs freed on the device, local $(du -sh "$DEST" | cut -f1)"
