#!/bin/bash
# probe.sh <tag> <seconds> [run_fleet.py args...]: start a run detached, wait, show the Renode errors.
cd "$(dirname "$0")/../.."
tag=$1; secs=$2; shift 2
nohup setsid .venv/bin/python experiments/single_renode/run_fleet.py --tag "$tag" "$@" > experiments/single_renode/out/$tag.out 2>&1 < /dev/null &
sleep "$secs"
grep -a "abort\|ERROR" experiments/single_renode/out/${tag}_renode.log | cut -c1-200 | head -5
grep -a -v "refused" experiments/single_renode/out/$tag.out | grep -a "drone\|RESULT\|MEMORY\|CPU" | tail -8
