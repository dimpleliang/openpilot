#!/bin/bash
# ars510: pull the full-resolution road video (fcamera.hevc) of given segments from the comma over Tailscale.
#   ~/ars510_rlogs/pull_video.sh 00000036--d9e05ff4d1--3 [more segments]
#   ALL=1 ~/ars510_rlogs/pull_video.sh <segment>     also the wide (ecamera) and driver (dcamera) videos
#   ~/ars510_rlogs/pull_video.sh 00000036--d9e05ff4d1     a whole route (every segment still on the device)
# Videos stay on the device only until its own cleanup removes them (a few days at most).
set -u
DEST=~/ars510_rlogs/raw
HOST=comma-ts
inc="--include=fcamera.hevc"; [ "${ALL:-0}" = "1" ] && inc="--include=fcamera.hevc --include=ecamera.hevc --include=dcamera.hevc"
for arg in "$@"; do
  segs=$(ssh -n -o ConnectTimeout=20 -o BatchMode=yes $HOST "cd /data/media/0/realdata && ls -d ${arg}*/ 2>/dev/null | tr -d /")
  [ -n "$segs" ] || { echo "no such segment on the device: $arg"; continue; }
  for s in $segs; do
    mkdir -p "$DEST/$s"
    rsync -a --partial --progress --rsync-path="nice -n 10 rsync" $inc --exclude='*' "$HOST:/data/media/0/realdata/$s/" "$DEST/$s/" < /dev/null \
      && echo "ok $s -> $DEST/$s/" || echo "failed $s"
  done
done
