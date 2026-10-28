#!/bin/bash
# nightly_try.sh <n> <secs>: run the generated script on the downloaded nightly
# with --console and print what it complains about (diagnostics, Task 17 lead 5).
cd "$(dirname "$0")/../.."
N=$PWD/$(ls -d experiments/renode-new/nightly/*/ | head -1)renode
out=$PWD/experiments/single_renode/out
SINGLE_RENODE_DROP="${SINGLE_RENODE_DROP:+$SINGLE_RENODE_DROP|}physics Connect|gpsHub" .venv/bin/python experiments/single_renode/make_fleet_resc.py "$1" > $out/nt.resc
cd pixhawk6c_renode_standalone
nohup setsid "$N" --disable-xwt --console -e "include @$out/nt.resc" > $out/nt_renode.log 2>&1 < /dev/null &
sleep "$2"
sed 's/\x1b\[[0-9;]*m//g' $out/nt_renode.log | grep -a -v "Loading block\|ungzip\|Including" | grep -a -i -A4 "error\|abort\|Could not" | cut -c1-260 | head -14
pkill -9 -x renode
