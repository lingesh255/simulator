#!/bin/bash
# run.sh <tag> [speed_run.py options...]: clean up, run one experiment, keep its log (capped).
cd "$(dirname "$0")/../.."
tag=$1; shift
tests/harness/rclean.sh > /dev/null
.venv/bin/python experiments/shared_speed/speed_run.py --tag "$tag" "$@" 2>&1 | grep --line-buffered -v "Connection refused sleeping" | head -c 400000 > experiments/shared_speed/out/$tag.log
grep "RESULT\|SKIPPED\|ABORT\|BOOT ERROR" experiments/shared_speed/out/$tag.log | cut -c1-600
tests/harness/rclean.sh > /dev/null
