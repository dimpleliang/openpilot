#!/bin/bash
# ars510: runs at the end of every ts_pull.sh (launchd com.ars510.tspull, every 15 min, Mac native):
#   1. git pull the decode sources (sparse clone of dimpleliang/openpilot ars510-dev in ~/ars510_rlogs/op)
#   2. db/build_db.py  - incremental rlog -> parquet (segments with a _DONE marker are skipped; < 1 s when nothing is new)
#   3. db/daily_summary.py - summary/YYYY-MM-DD.md for every day whose segments changed + summary/latest.md
# Logs: db/build.log (1+2), summary/summary.log (3). Python: ~/ars510_rlogs/venv (python3.11, numpy pandas pyarrow zstandard pycapnp).
#   ~/ars510_rlogs/post_pull.sh        run by hand
set -u
ROOT=~/ars510_rlogs
cd "$ROOT" || exit 1
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:/opt/homebrew/bin
export GIT_TERMINAL_PROMPT=0
mkdir -p summary
{
  echo "== $(date '+%F %T') post_pull"
  if [ -d op/.git ]; then
    out=$(git -C op -c http.lowSpeedLimit=1000 -c http.lowSpeedTime=60 pull -q --ff-only 2>&1) || echo "git pull failed (building with the existing checkout): $out"
  fi
  if [ ! -x venv/bin/python3 ]; then
    echo "venv missing: /opt/homebrew/bin/python3.11 -m venv venv && venv/bin/pip install numpy pandas pyarrow zstandard pycapnp"; exit 1
  fi
  venv/bin/python3 db/build_db.py
} >> db/build.log 2>&1
venv/bin/python3 db/daily_summary.py >> summary/summary.log 2>&1
