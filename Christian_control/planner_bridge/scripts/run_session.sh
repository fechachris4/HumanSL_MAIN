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

# The bridge's DH YAML is generated from the URDF at build time. If the URDF
# was edited and the build not rerun (or it failed at the generator), the
# stale generated file must not reach a session.
URDF="$REPO/Christian_control/basic_control/config/GEN3_dual_mounted.urdf"
DH_YAML="$REPO/Christian_control/planner_bridge/build/config/dh_params_tool.yaml"
[[ -f "$DH_YAML" ]] || { echo "missing generated $DH_YAML — build planner_bridge first"; exit 1; }
if [[ "$URDF" -nt "$DH_YAML" ]]; then
    echo "STALE: $DH_YAML is older than the URDF — rebuild planner_bridge"
    [[ $ALLOW_STALE = 1 ]] || { echo "rebuild, or pass --allow-stale"; exit 1; }
fi

echo "== Supervised session checklist (project CLAUDE.md) =="
echo "  - Christian present, workspace clear, e-stop in reach"
echo "  - Kinova web dashboard CLOSED (it blocks SetServoingMode)"
echo "  - This run is explicitly authorized"
read -r -p "Type GO to start the controller: " confirm
[[ "$confirm" == "GO" ]] || { echo "aborted"; exit 1; }
[[ $DRY_RUN = 1 ]] && { echo "dry-run: would start $CONTROLLER now"; exit 0; }

SESSION_MARK=$(mktemp)   # anything newer than this was created by THIS session
CONTROLLER_PID=""        # set right after the fork; trap is a no-op until then
trap 'kill -INT "$CONTROLLER_PID" 2>/dev/null || true; wait "$CONTROLLER_PID" 2>/dev/null || true; rm -f "$SESSION_MARK"' EXIT
"$CONTROLLER" & CONTROLLER_PID=$!   # no --log: timestamped default under runs/

mkdir -p "$REPO/runs"   # first run on a fresh checkout has no runs/ dir yet
echo "waiting for the controller's run log..."
for _ in $(seq 1 60); do
    LATEST=$(find "$REPO/runs" -name 'loop_log*.csv' -newer "$SESSION_MARK" 2>/dev/null | head -1) || true
    [[ -n "${LATEST:-}" ]] && break
    kill -0 "$CONTROLLER_PID" 2>/dev/null || { echo "controller exited during startup"; exit 1; }
    sleep 1
done
[[ -n "${LATEST:-}" ]] || { echo "no run log appeared after 60 s"; exit 1; }
echo "state source: $LATEST"

# The file existing is not enough: the controller's CSV header and first
# data rows may still be buffered, and the bridge errors on a header-less
# file. Wait until a meas_j1 header line AND at least one row after it are
# actually on disk.
echo "waiting for telemetry data in the run log..."
TELEMETRY_READY=0
for _ in $(seq 1 30); do
    if awk '/meas_j1/{h=NR} END{exit !(h && NR>h)}' "$LATEST"; then
        TELEMETRY_READY=1; break
    fi
    kill -0 "$CONTROLLER_PID" 2>/dev/null || { echo "controller exited before telemetry started"; exit 1; }
    sleep 1
done
[[ $TELEMETRY_READY = 1 ]] || { echo "no telemetry rows appeared in $LATEST after 30 s"; exit 1; }

# The goal comes from config/goal.yaml (edit it before starting a session;
# the bridge reads it because no --goal is passed here). One bridge run per
# session — no prompt, no typed coordinates.
for _ in $(seq 1 10); do [[ -p "$PIPE" ]] && break; sleep 1; done
[[ -p "$PIPE" ]] || { echo "target pipe never appeared: $PIPE"; exit 1; }
if "$BRIDGE" > "$PIPE"; then
    echo "goal sent — arm is following the plan. Press Enter to stop the controller."
    read -r
else
    echo "bridge exited $? — nothing was sent; stopping controller."
fi
echo "stopping controller..."
