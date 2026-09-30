#!/bin/bash
# Kill every Renode / physics process, wait, and show pgrep (the [r] keeps
# the patterns from matching this script's or its caller's own command line).
pkill -9 -f '[r]enode-bin/[r]enode|[r]enode-physics'; sleep 2
out=$(pgrep -af '[r]enode-bin/[r]enode|[r]enode-physics')
if [ -z "$out" ]; then echo "pgrep: (nothing)"; else echo "pgrep:"; echo "$out"; fi
