#!/bin/bash
# sentinel.sh <tag> [n=1] [secs=20]: count Renode's "An interrupt from the
# unsynchronized thread" warnings per machine (cpu ThreadSentinelEnabled).
cd "$(dirname "$0")/../.."
tag=$1; n=${2:-1}; secs=${3:-20}
cmds=""
for i in $(seq 1 "$n"); do cmds="${cmds}mach set \"drone$i\"\ncpu ThreadSentinelEnabled true\nlogLevel 1 sysbus.cpu\n"; done
out=$(RAW_BEFORE_START="${cmds%\\n}" experiments/single_renode/partest.sh "$tag" "$n" "$secs" 2>&1 | grep -v Killed)
log=experiments/single_renode/out/${tag}_renode.log
echo "$out"
for i in $(seq 1 "$n"); do
  c=$(grep -a "unsynchronized" $log | grep -a -E "(drone$i/)?cpu:" | { [ "$n" = 1 ] && cat || grep -a "drone$i/"; } | sed -E 's/.*thread\.( \(([0-9]+)\))?.*/\2/' | awk '{s+=($1==""?1:$1)} END{print s+0}')
  echo "   drone$i unsynchronized-thread interrupts: $c"
done
