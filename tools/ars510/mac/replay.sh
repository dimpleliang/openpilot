#!/bin/bash
# ars510: planner replay on the Mac (Docker, linux/arm64 like the comma 3X) with the device's prebuilt MPC.
# Tests run here, not on the car (11/10). One segment takes ~0.6 s (the device: ~30 s).
#
#   replay.sh dump <label> <version> <route prefix> [...]   -> ~/ars510_rlogs/replay/out/<label>.npz
#   replay.sh cmp <label a> <label b>
#
# <version>: '-' = the device copy as it is; wt = this repo's working tree; ref:<git ref> = a commit / branch;
# a path = only that ars510_long_rules.py over the device copy. wt / ref lay every *.py and *.capnp of the repo over
# the device copy (the prebuilt .so stay), so changes to the planner, the MPC wrapper or cereal are covered.
# Environment: ARS510_WIPER=can | <level> is passed on (rain mode input, see planner_equiv.py). Setup (once): ~/ars510_rlogs/replay/dev/data/openpilot = a copy of
# /data/openpilot from the device (prebuilt .so included); image ars510-replay = ubuntu:24.04 + python3.12
# (replay/img/Dockerfile); replay/venv = pip numpy pycapnp==2.1.0 pyzmq setproctitle zstandard cffi tqdm requests sympy
# casadi comma-deps-acados. CarParams come from segment 0 of the first route (planner_equiv.py, ARS510_CP_RLOG).
# Note: openpilot params are defaults here, not the car's (copy /data/params/d from the device into replay/dev/data/params/d).
set -e
export PATH=$PATH:/usr/local/bin
RP=~/ars510_rlogs/replay
HERE=$(cd $(dirname $0)/.. && pwd)
mkdir -p $RP/out $RP/trees
cp $HERE/planner_equiv.py $RP/out/planner_equiv.py
RUN=(docker run --rm -v $RP/dev/data:/data -v $RP/venv:/usr/local/venv -v $HOME/ars510_rlogs/raw:/raw:ro -v $RP/out:/out -v $RP/trees:/trees)
PY=/usr/local/venv/bin/python
if [ $1 = cmp ]; then
  ${RUN[@]} ars510-replay $PY /out/planner_equiv.py cmp /out/$2.npz /out/$3.npz
  exit
fi
label=$2; rules=$3; shift 3
L=''
for r in $@; do
  for s in $(ls -d ~/ars510_rlogs/raw/$r--* | sed 's/.*--//' | sort -n); do
    [ -f ~/ars510_rlogs/raw/$r--$s/rlog.zst ] && L=$L' '/raw/$r--$s/rlog.zst
  done
done
root=/data/openpilot
REPO=$(cd $HERE/../.. && pwd)
if [ $rules = wt ] || [ ${rules#ref:} != $rules ]; then
  root=/trees/$label
  if [ $rules = wt ]; then
    (cd $REPO && git ls-files -co --exclude-standard '*.py' '*.capnp' | COPYFILE_DISABLE=1 tar --no-xattrs -cf $RP/out/src_$label.tar -T -)
  else
    (cd $REPO && git archive -o $RP/out/src_$label.tar ${rules#ref:})
  fi
  PRE='rm -rf '$root' && cp -as /data/openpilot '$root' && tar -xf /out/src_'$label'.tar -C '$root' --unlink-first --wildcards "*.py" "*.capnp" && rm -f /out/src_'$label'.tar && '
elif [ $rules != - ]; then
  cp $rules $RP/out/rules_$label.py
  root=/trees/$label
  PRE='rm -rf '$root' && cp -as /data/openpilot '$root' && rm -f '$root'/openpilot/selfdrive/controls/lib/ars510_long_rules.py && cp /out/rules_'$label'.py '$root'/openpilot/selfdrive/controls/lib/ars510_long_rules.py && '
fi
${RUN[@]} -e ARS510_WIPER=$ARS510_WIPER -e ARS510_CP_RLOG=/raw/$1--0/rlog.zst ars510-replay sh -c "$PRE $PY /out/planner_equiv.py dump $root /out/$label.npz $L" | tail -2
