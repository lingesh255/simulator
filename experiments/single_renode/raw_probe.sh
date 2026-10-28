#!/bin/bash
# raw_probe.sh <tag> <n> <seconds>: run the generated script with NO MAVLink
# client and no physics sidecar (set SINGLE_RENODE_DROP to leave lines out);
# diagnostics for the two-machine race only. Monitor on telnet 4455.
cd "$(dirname "$0")/../.."
tag=$1; n=$2; secs=$3
out=experiments/single_renode/out
.venv/bin/python experiments/single_renode/make_fleet_resc.py "$n" > $out/$tag.resc
# RAW_BEFORE_START: extra monitor lines (\n-separated) inserted before the final start
if [ -n "$RAW_BEFORE_START" ]; then sed -i "s/^start\$/$RAW_BEFORE_START\nstart/" $out/$tag.resc; fi
cd pixhawk6c_renode_standalone
DOTNET_GCConserveMemory=7 nohup setsid ./renode-bin/renode --disable-xwt -P 4455 -e "include @$OLDPWD/$out/$tag.resc" > "$OLDPWD/$out/${tag}_renode.log" 2>&1 < /dev/null &
sleep "$secs"
grep -a "abort" "$OLDPWD/$out/${tag}_renode.log" | cut -c1-160
