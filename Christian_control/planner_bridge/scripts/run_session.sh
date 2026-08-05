#!/usr/bin/env bash
# One-terminal supervised session for the Stage 1.5 bridge workflow.
# Sequences what the operator previously did by hand; bypasses nothing.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONTROLLER="$REPO/Christian_control/basic_control/build/controller"
BRIDGE="$REPO/Christian_control/planner_bridge/build/planner_bridge"
PIPE="/tmp/humansl_bridge_targets"
ALLOW_STALE=0; DRY_RUN=0
for arg in "$@"; do case "$arg" in
    --allow-stale) ALLOW_STALE=1 ;;
    --dry-run) DRY_RUN=1 ;;
    *) echo "usage: run_session.sh [--allow-stale] [--dry-run]"; exit 1 ;;
esac; done

fresh_or_die() { # $1 binary, $2 source dir
    [[ -x "$1" ]] || { echo "missing binary: $1 — build it first"; exit 1; }
    local stale
    stale=$(find "$2" -name '*.cpp' -o -name '*.h' | xargs -r ls -t | head -1)
    if [[ "$stale" -nt "$1" ]]; then
        echo "STALE: $1 is older than $stale"
        [[ $ALLOW_STALE = 1 ]] || { echo "rebuild, or pass --allow-stale"; exit 1; }
    fi
}
fresh_or_die "$CONTROLLER" "$REPO/Christian_control/basic_control/src"
fresh_or_die "$BRIDGE"     "$REPO/Christian_control/planner_bridge/src"

echo "== Supervised session checklist (project CLAUDE.md) =="
echo "  - Christian present, workspace clear, e-stop in reach"
echo "  - Kinova web dashboard CLOSED (it blocks SetServoingMode)"
echo "  - This run is explicitly authorized"
read -r -p "Type GO to start the controller: " confirm
[[ "$confirm" == "GO" ]] || { echo "aborted"; exit 1; }
[[ $DRY_RUN = 1 ]] && { echo "dry-run: would start $CONTROLLER now"; exit 0; }

SESSION_MARK=$(mktemp)   # anything newer than this was created by THIS session
"$CONTROLLER" & CONTROLLER_PID=$!   # no --log: timestamped default under runs/
trap 'kill -INT $CONTROLLER_PID 2>/dev/null || true; wait $CONTROLLER_PID 2>/dev/null || true; rm -f "$SESSION_MARK"' EXIT

echo "waiting for the controller's run log..."
for _ in $(seq 1 60); do
    LATEST=$(find "$REPO/runs" -name 'loop_log*.csv' -newer "$SESSION_MARK" 2>/dev/null | head -1)
    [[ -n "${LATEST:-}" ]] && break
    kill -0 $CONTROLLER_PID 2>/dev/null || { echo "controller exited during startup"; exit 1; }
    sleep 1
done
[[ -n "${LATEST:-}" ]] || { echo "no run log appeared after 60 s"; exit 1; }
echo "state source: $LATEST"

while read -r -p "bridge> " cmd args; do case "$cmd" in
    goal) # goal X Y Z [box CX CY CZ HX HY HZ]
        if [[ ! -p "$PIPE" ]]; then
            echo "target pipe not ready: $PIPE (controller creates it at startup — is it still starting up?)"
            continue
        fi
        read -r gx gy gz maybe_box rest <<<"$args"
        extra=()
        [[ "${maybe_box:-}" == "box" ]] && extra=(--box $rest)
        "$BRIDGE" --goal "$gx" "$gy" "$gz" "${extra[@]}" > "$PIPE" \
            || echo "bridge exited $? — nothing was sent"
        ;;
    quit|q) break ;;
    *) echo "commands: goal X Y Z [box CX CY CZ HX HY HZ] | quit" ;;
esac; done
echo "stopping controller..."
