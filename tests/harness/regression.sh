#!/bin/bash
# Run a list of gui_drive.py scenarios one after another, clearing Renode
# before each and showing pgrep right after each app exits. Before
# fleet_grid4 it prints `free -m` for information. Progress goes to stdout; each
# scenario's own log to tests/harness/out/<prefix>_<scenario>.out; the shared
# Renode's console log (if that run used one) and the table screenshots are
# copied next to it under the same prefix.
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
    started=$(date +%s)
    timeout 3000 .venv/bin/python $H/gui_drive.py "$sc" > "$H/out/${prefix}_$sc.out" 2>&1
    echo "exit=$?"
    echo "pgrep right after app exit:"
    pgrep -af '[r]enode-bin/[r]enode|[r]enode-physics' || echo "(nothing)"
    # keep this run's shared-Renode console log and table screenshots under the prefix
    shared_log=renode_instances/shared/renode-console.log
    if [ -e "$shared_log" ] && [ "$(stat -c %Y "$shared_log")" -ge "$started" ]; then
        cp "$shared_log" "$H/out/${prefix}_${sc}_shared_renode.log"
    fi
    for shot in "$H"/out/${sc}_*.png; do
        [ -e "$shot" ] && cp "$shot" "$H/out/${prefix}_$(basename "$shot")"
    done
done
echo "profiles: $(ls data/profiles | tr '\n' ' ')  git status: $(git status --short data/profiles | tr '\n' ' ')"
$H/rclean.sh
echo "ALL DONE ($(date +%H:%M:%S))"
