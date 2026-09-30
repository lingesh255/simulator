#!/bin/bash
# Run a list of gui_drive.py scenarios one after another, clearing Renode
# before each and showing pgrep right after each app exits. Before
# fleet_grid4 it prints `free -m` for information. Progress goes to stdout; each
# scenario's own log to tests/harness/out/<prefix>_<scenario>.out.
#
#   tests/harness/regression.sh <prefix> <scenario> [<scenario> ...]
cd "$(dirname "$0")/../.." || exit 1
H=tests/harness
prefix=$1; shift
mkdir -p $H/out
for sc in "$@"; do
    echo "##### $sc  ($(date +%H:%M:%S))"
    $H/rclean.sh
    if [ "$sc" = fleet_grid4 ]; then
        free -m  # information only - never skips anything
    fi
    timeout 3000 .venv/bin/python $H/gui_drive.py "$sc" > "$H/out/${prefix}_$sc.out" 2>&1
    echo "exit=$?"
    echo "pgrep right after app exit:"
    pgrep -af '[r]enode-bin/[r]enode|[r]enode-physics' || echo "(nothing)"
done
echo "profiles: $(ls data/profiles | tr '\n' ' ')  git status: $(git status --short data/profiles | tr '\n' ' ')"
$H/rclean.sh
echo "ALL DONE ($(date +%H:%M:%S))"
