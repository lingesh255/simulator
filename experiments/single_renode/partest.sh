#!/bin/bash
# partest.sh <tag> [n=2] [seconds=25]: does an N-machine script survive?
# Runs the generated script with no MAVLink client, no physics and no GPS
# hub (the Task 16 fault shows without them), samples virtual time twice,
# and prints ABORT / FROZEN / RUNNING. Generator knobs come from the
# environment (see make_fleet_resc.py; SINGLE_RENODE_SHARED_TIME=1 reproduces the
# Task 16 fault); RENODE_BIN picks another Renode.
cd "$(dirname "$0")/../.."
tag=$1; n=${2:-2}; secs=${3:-25}
out=$PWD/experiments/single_renode/out
bin=${RENODE_BIN:-$PWD/pixhawk6c_renode_standalone/renode-bin/renode}
SINGLE_RENODE_DROP="${SINGLE_RENODE_DROP:+$SINGLE_RENODE_DROP|}physics Connect|gpsHub" \
    .venv/bin/python experiments/single_renode/make_fleet_resc.py "$n" > $out/$tag.resc || exit 1
if [ -n "$RAW_BEFORE_START" ]; then sed -i "s/^start\$/$RAW_BEFORE_START\nstart/" $out/$tag.resc; fi
cd pixhawk6c_renode_standalone
DOTNET_GCConserveMemory=7 nohup setsid "$bin" --disable-xwt -P 4455 -e "include @$out/$tag.resc" > "$out/${tag}_renode.log" 2>&1 < /dev/null &
cd ..
vt() { .venv/bin/python experiments/single_renode/mon.py 'machine ElapsedVirtualTime' 2>/dev/null | grep -o "Virtual Time: [0-9:.]*" | head -1 | awk '{print $3}'; }
sleep "$secs"; t1=$(vt); sleep 4; t2=$(vt)
abort=$(grep -a -c "CPU abort" "$out/${tag}_renode.log")
errs=$(grep -a "\[ERROR\]" "$out/${tag}_renode.log" | grep -a -v "CPU abort" | cut -c1-150 | sort | uniq -c | head -3)
if [ "$abort" != 0 ]; then verdict=ABORT; elif [ -z "$t2" ]; then verdict=NO-MONITOR; elif [ "$t1" = "$t2" ]; then verdict=FROZEN; else verdict=RUNNING; fi
echo "$tag: $verdict  virtual time $t1 -> $t2 after ${secs}s+4s  aborts=$abort"
[ -n "$errs" ] && echo "$errs"
pkill -9 -x renode; sleep 1
