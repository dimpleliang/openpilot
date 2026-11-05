#!/bin/bash
# ars510: pull every finished segment's rlog/qlog/qcamera (low-res road video) from the comma over Tailscale, then free
# the rlog's space on the device. Full-resolution video on demand: pull_video.sh.
# Runs from launchd every 15 min (see ts_pull.plist).
#   offroad: rsync, full speed.
#   onroad:  the device streams each file with cat_nocache.py (reads dropped from the page cache as they go, nice 19,
#            ionice idle, RATE_ONROAD bytes/s). 11/02: a plain rsync while driving filled the page cache and the kernel
#            reclaimed in bursts (kswapd0 15-30 %, tailscaled 27 %), modeld dropped frames and openpilot raised
#            "locationd Temporary Error" twice. The newest segment (still being written) is skipped while onroad.
# The rlog is deleted on the device only after it arrived here with the same size; the video stays there for the
# device's own cleanup / comma connect, so the camera can still be reviewed for a few days.
#
#   ~/ars510_rlogs/ts_pull.sh            normal run
#   KEEP=1 ~/ars510_rlogs/ts_pull.sh     pull only, delete nothing on the device
set -u
DEST=~/ars510_rlogs/raw
LOG=~/ars510_rlogs/ts_pull.log
HOST=comma-ts                       # ssh alias over Tailscale (see the ~/.ssh/config block in the setup notes)
O="-n -o ConnectTimeout=20 -o BatchMode=yes"   # -n: never read the loop stdin
# 11/05: one shared ssh connection for the whole run (every burst + offroad check used to be a new ssh handshake on the comma)
O="$O -o ControlMaster=auto -o ControlPath=$HOME/.ssh/cm-ars510-%C -o ControlPersist=60"
RATE_ONROAD=400000                  # bytes/s within a burst while driving
CHUNK_ONROAD=3000000                # bytes per burst while driving (11/05), then PAUSE_ONROAD s of nothing
PAUSE_ONROAD=5
# Throughput while driving (11/05 driver: keep up even on an 8 h day without changing the device): ~2.8 bursts in the ~35 s per
# minute outside the uploader's window = ~8 MB/min. The comma keeps ~5.4 h of driving (243 MB/min, ~79 GB) before its deleter
# removes the oldest segments, rlog included; rlog + qlog are ~12.4 MB/min, so 8 MB/min keeps every rlog for drives up to ~15 h.
CAT=/data/openpilot/tools/ars510/cat_nocache.py
FILES="rlog.zst qlog.zst qcamera.ts"
FILES_ONROAD="rlog.zst qlog.zst"    # 11/05: the low-res video waits until the car is off (it is on comma connect anyway)
# 11/05: the comma's own uploader sends the previous segment's qlog + qcamera in the first ~20 s after each new segment;
# a burst waits until UPLOADER_QUIET_S after the newest segment started, and while the CPU is hotter than HOT_C
UPLOADER_QUIET_S=25
HOT_C=80
# 11/05 driver: in P (car on, parked) pull at full speed too: 5 MB bursts, no rate limit, 1 s apart, low-res video included,
# P re-checked before every burst (a one-shot carState read, ~0.75 s CPU at nice 19). While driving P is checked once per segment.
CHUNK_PARK=5000000
PAUSE_PARK=1
mkdir -p "$DEST"
exec >> "$LOG" 2>&1
echo "== $(date '+%F %T')"
# single instance (11/02 review): two overlapping runs would append to the same .part files
LOCK=~/ars510_rlogs/.ts_pull.lock
if ! mkdir "$LOCK" 2>/dev/null; then
  if [ -n "$(find "$LOCK" -mmin +120 2>/dev/null)" ]; then rmdir "$LOCK" && mkdir "$LOCK" || exit 0; else echo "another run is active"; exit 0; fi
fi
trap 'rmdir "$LOCK" 2>/dev/null' EXIT
if ! ssh $O $HOST true 2>/dev/null; then echo "comma offline"; exit 0; fi
offroad() { [ "$(ssh $O $HOST 'cat /data/params/d/IsOffroad 2>/dev/null')" = "1" ]; }
parked() { [ "$(ssh $O $HOST 'cd /data/openpilot && PYTHONPATH=/data/openpilot:/data/openpilot/openpilot timeout 5 nice -n 19 /usr/local/venv/bin/python3 -c "
from cereal import messaging
m = messaging.recv_one(messaging.sub_sock(\"carState\", conflate=True, timeout=1000))
print(\"P\" if m is not None and str(m.carState.gearShifter) == \"park\" and m.carState.vEgo < 0.1 else \"-\")
" 2>/dev/null')" = "P" ]; }
# the streamer must exist on the device (it ships with the fork; until the device is updated, copy it over)
if ! ssh $O $HOST "test -f $CAT"; then
  CAT=/data/ars510_cat_nocache.py
  ssh $O $HOST "test -f $CAT" || scp -q -o BatchMode=yes ~/ars510_rlogs/cat_nocache.py "$HOST:$CAT"
fi
# one listing: segment and the size of each file (0 = missing), oldest first
# (11/02 review round 2: segment numbers are not zero-padded, so a plain sort put --10 before --9 and "drop the last line" did
#  not skip the segment loggerd is writing. Sort route, then segment number; and skip any segment whose files changed in the
#  last SETTLE_S seconds, whatever its position.)
SETTLE_S=90
list=$(ssh $O $HOST "cd /data/media/0/realdata && now=\$(date +%s) && for d in \$(ls -d */ | tr -d / | grep -v boot | sort -t- -k1,1 -k5,5n); do
  m=\$(stat -c %Y \$d/rlog.zst \$d/qlog.zst \$d/qcamera.ts 2>/dev/null | sort -n | tail -1); [ -n \"\$m\" ] && [ \$((now - m)) -lt $SETTLE_S ] && continue
  echo \"\$d \$(stat -c %s \$d/rlog.zst 2>/dev/null || echo 0) \$(stat -c %s \$d/qlog.zst 2>/dev/null || echo 0) \$(stat -c %s \$d/qcamera.ts 2>/dev/null || echo 0)\"; done")
if offroad; then mode=offroad; else mode=onroad; fi
echo "mode $mode"
IONICE=$(ssh $O $HOST 'command -v ionice >/dev/null && echo "ionice -c3" || echo ""')
# 11/05 offroad: one rsync over the shared ssh connection per BATCH segments (was one rsync = new ssh + file list per segment:
# over the phone hotspot, ~220 ms latency, the speed fell to a few kB/s between segments). The car may be started during a
# long run: offroad is re-checked between batches. The per-segment loop below then only verifies, frees, and retries misses.
BATCH=10
if [ "$mode" = offroad ]; then
  todo=$(while read -r s r_sz q_sz c_sz; do
    [ -n "$s" ] || continue
    [ "$r_sz" = "0" ] && [ -s "$DEST/$s/rlog.zst" ] && [ -s "$DEST/$s/qcamera.ts" ] && continue
    for x in "rlog.zst $r_sz" "qlog.zst $q_sz" "qcamera.ts $c_sz"; do
      set -- $x; [ "$2" = "0" ] && continue
      [ "$(stat -f %z "$DEST/$s/$1" 2>/dev/null || echo 0)" = "$2" ] || echo "$s/$1"
    done
  done <<< "$list")
  if [ -n "$todo" ]; then
    nseg=$(echo "$todo" | cut -d/ -f1 | uniq | wc -l | tr -d ' ')
    echo "batch rsync: $(echo "$todo" | wc -l | tr -d ' ') files in $nseg segments"
    for segs in $(echo "$todo" | cut -d/ -f1 | uniq | awk -v b=$BATCH '{printf "%s%s", $0, (NR % b ? "," : "\n")} END {print ""}'); do
      offroad || { echo "car went onroad: stopping batch rsync"; break; }
      echo "$todo" | grep -E "^($(echo "$segs" | sed 's/,$//; s/,/|/g'))/" > "$DEST/.batch"
      rsync -a --partial --timeout=120 --rsync-path="nice -n 10 rsync" -e "ssh ${O#-n }" --files-from="$DEST/.batch" \
            "$HOST:/data/media/0/realdata/" "$DEST/" < /dev/null || echo "batch rsync failed (per-segment retry below)"
    done
    rm -f "$DEST/.batch"
  fi
fi
n=0; freed=0
while read -r s r_sz q_sz c_sz; do
  [ -n "$s" ] || continue
  [ "$r_sz" = "0" ] && [ -s "$DEST/$s/rlog.zst" ] && [ -s "$DEST/$s/qcamera.ts" ] && continue   # already pulled and freed
  mkdir -p "$DEST/$s"
  if [ "$mode" = offroad ]; then
    # the car may be driven off during a long offroad run: stop the full-speed rsync, the next run streams gently
    have_all=1
    for x in "rlog.zst $r_sz" "qlog.zst $q_sz" "qcamera.ts $c_sz"; do
      set -- $x; [ "$2" = "0" ] || [ "$(stat -f %z "$DEST/$s/$1" 2>/dev/null || echo 0)" = "$2" ] || have_all=0
    done
    if [ "$have_all" = 0 ]; then
    if ! offroad; then echo "car went onroad after $n segments: stopping rsync"; break; fi
    rsync -a --partial --timeout=120 --rsync-path="nice -n 10 rsync" \
          --include='rlog*' --include='qlog*' --include='qcamera.ts' --exclude='*' \
          "$HOST:/data/media/0/realdata/$s/" "$DEST/$s/" < /dev/null || { echo "rsync failed $s"; continue; }
    fi
  else
    ok=1
    if parked; then pk=1; files=$FILES; else pk=0; files=$FILES_ONROAD; fi
    for f in $files; do
      case $f in rlog.zst) want=$r_sz;; qlog.zst) want=$q_sz;; qcamera.ts) want=$c_sz;; esac
      [ "$want" = "0" ] && continue
      have=$(stat -f %z "$DEST/$s/$f" 2>/dev/null || echo 0)
      [ "$have" = "$want" ] && continue
      part="$DEST/$s/$f.part"; off=$(stat -f %z "$part" 2>/dev/null || echo 0)
      [ "$off" -gt "$want" ] && { rm -f "$part"; off=0; }
      # one streamer only (11/02 review: a retry after a partial first attempt would append from a stale offset).
      # 11/05: in short bursts (CHUNK_ONROAD, then PAUSE_ONROAD s quiet), resumed from the .part size each time: a continuous
      # stream at 22:02 was followed by "locationd Temporary Error" (driver: send 2-3 MB at a time, it resumes anyway)
      while :; do
        if [ "$pk" = 1 ]; then ch=$CHUNK_PARK; rt=0; pz=$PAUSE_PARK; q=0; else ch=$CHUNK_ONROAD; rt=$RATE_ONROAD; pz=$PAUSE_ONROAD; q=$UPLOADER_QUIET_S; fi
        ssh $O $HOST "cd /data/media/0/realdata && a=\$(( \$(date +%s) - \$(stat -c %Y \$(ls -td */ | head -1)) )); \
          [ \$a -ge 0 ] && [ \$a -lt $q ] && sleep \$(( $q - a )); \
          for i in 1 2 3; do t=\$(for z in /sys/class/thermal/thermal_zone*; do case \$(cat \$z/type) in cpu*|gpu*) cat \$z/temp;; esac; done 2>/dev/null | sort -n | tail -1); \
            [ \$(( \${t:-0} / 1000 )) -lt $HOT_C ] && break; sleep 20; done; \
          $IONICE python3 $CAT /data/media/0/realdata/$s/$f $off $rt 2>/dev/null | head -c $ch" >> "$part"
        got=$(stat -f %z "$part" 2>/dev/null || echo 0)
        [ "$got" -ge "$want" ] || [ "$got" = "$off" ] && break   # done, or no progress (offline): resume next run
        off=$got
        offroad && break
        sleep $pz
        if [ "$pk" = 1 ] && ! parked; then pk=0; fi   # shifted out of P: back to the gentle bursts
      done
      if [ "$got" = "$want" ]; then mv -f "$part" "$DEST/$s/$f"; else echo "partial $s/$f $got/$want (resumes)"; ok=0; break; fi
      offroad && { echo "car stopped: switching to rsync next run"; break 2; }
    done
    [ "$ok" = "1" ] || break
  fi
  n=$((n + 1))
  if [ "${KEEP:-0}" != "1" ] && [ "$r_sz" -gt 0 ]; then
    local_=$(stat -f %z "$DEST/$s/rlog.zst" 2>/dev/null || echo -1)
    now_sz=$(ssh $O $HOST "stat -c %s /data/media/0/realdata/$s/rlog.zst 2>/dev/null || echo 0")
    if [ "$r_sz" = "$local_" ] && [ "$now_sz" = "$r_sz" ]; then   # unchanged since the listing and fully here
      ssh $O $HOST "rm -f /data/media/0/realdata/$s/rlog.zst" && freed=$((freed + 1))
    else
      echo "size mismatch $s remote=$r_sz local=$local_ (kept on device)"
    fi
  fi
done <<< "$list"
echo "done ($mode): $n segments pulled, $freed rlogs freed on the device, local $(du -sh "$DEST" | cut -f1)"
~/ars510_rlogs/post_pull.sh   # Mac native: git pull decode sources, incremental parquet build, daily summary (db/build.log, summary/summary.log)
