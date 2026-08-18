#!/usr/bin/env bash
# One-terminal supervised session for the single-process planner/controller.
# Sequences what the operator previously did by hand; bypasses nothing.
#
# Which arm(s): --arm right|left|both on the command line, or (if --arm is
# omitted) the `session_arms:` key in config/goal.yaml — the one file this
# workflow already asks you to edit before every run, so the session's arm
# selection lives next to the targets it's about to send. --arm both starts
# ONE controller process (two arm threads, in-process planner workers, one
# stop flag — a fault on either arm stops both).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONTROLLER="$REPO/Christian_control/runtime/build/controller"
CONTROLLER_BUILD="$REPO/Christian_control/runtime/build"
PLANNER_SRC="$REPO/Christian_control/planning/src"
PLANNER_TG="$REPO/Christian_control/planning/optimisation"
GOAL_YAML="$REPO/Christian_control/planning/config/goal.yaml"
PLANNER_YAML="$REPO/Christian_control/planning/config/planner.yaml"
ARM=""
ALLOW_STALE=0; DRY_RUN=0
while [[ $# -gt 0 ]]; do case "$1" in
    --arm)
        [[ $# -ge 2 ]] || { echo "--arm needs a value (right, left, or both)"; exit 1; }
        ARM="$2"; shift 2 ;;
    --allow-stale) ALLOW_STALE=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    *) echo "usage: run_session.sh [--arm right|left|both] [--allow-stale] [--dry-run]"; exit 1 ;;
esac; done

if [[ -z "$ARM" ]]; then
    ARM=$(awk -F': *' '/^session_arms:/{print $2; exit}' "$GOAL_YAML" 2>/dev/null || true)
    [[ -n "$ARM" ]] || {
        echo "error: no --arm given and no 'session_arms:' key in $GOAL_YAML"
        echo "either pass --arm right|left|both, or add session_arms: <value> to the file"
        exit 1
    }
    echo "arm not given on the command line — using session_arms: $ARM from $GOAL_YAML"
fi
if [[ "$ARM" != "right" && "$ARM" != "left" && "$ARM" != "both" ]]; then
    echo "error: --arm (or session_arms:) must be 'right', 'left', or 'both' (got '$ARM')"
    exit 1
fi
# Which arms this run actually drives, as a bash array — one element for
# single-arm, two for both. Every per-arm loop below iterates this instead
# of duplicating itself for the right/left/both cases.
if [[ "$ARM" == "both" ]]; then ARMS=(right left); else ARMS=("$ARM"); fi

fresh_or_die() { # $1 binary, remaining args are source directories
    [[ -x "$1" ]] || { echo "missing binary: $1 — build it first"; exit 1; }
    local binary="$1" source_dir stale
    shift
    for source_dir in "$@"; do
        stale=$(find "$source_dir" -type f \( -name '*.cpp' -o -name '*.h' \) -print0 |
            xargs -0 -r ls -t | head -1)
        if [[ -n "$stale" && "$stale" -nt "$binary" ]]; then
            echo "STALE: $binary is older than $stale"
            [[ $ALLOW_STALE = 1 ]] || { echo "rebuild, or pass --allow-stale"; exit 1; }
        fi
    done
}
fresh_or_die "$CONTROLLER" \
    "$REPO/Christian_control/control" "$REPO/Christian_control/runtime" "$PLANNER_SRC" "$PLANNER_TG"

# The controller build's nested planner library generates these DH YAML files
# from the URDF. If the URDF was edited and the build not rerun, stale model
# geometry must not reach a session.
URDF="$REPO/Christian_control/model/GEN3_dual_mounted.urdf"
dh_yaml_for() { # $1 arm
    if [[ "$1" == "left" ]]; then
        echo "$CONTROLLER_BUILD/planning/config/dh_params_flange.yaml"
    else
        echo "$CONTROLLER_BUILD/planning/config/dh_params_tool.yaml"
    fi
}
for a in "${ARMS[@]}"; do
    dh_yaml="$(dh_yaml_for "$a")"
    [[ -f "$dh_yaml" ]] || { echo "missing generated $dh_yaml — build controller first"; exit 1; }
    if [[ "$URDF" -nt "$dh_yaml" ]]; then
        echo "STALE: $dh_yaml is older than the URDF — rebuild controller"
        [[ $ALLOW_STALE = 1 ]] || { echo "rebuild, or pass --allow-stale"; exit 1; }
    fi
done

echo "== Supervised session checklist (project CLAUDE.md) =="
echo "  - arm(s): ${ARMS[*]}"
echo "  - Christian present, workspace clear, e-stop in reach"
echo "  - Kinova web dashboard CLOSED (it blocks SetServoingMode)"
echo "  - This run is explicitly authorized"
read -r -p "Type GO to start the controller: " confirm
[[ "$confirm" == "GO" ]] || { echo "aborted"; exit 1; }
[[ $DRY_RUN = 1 ]] && { echo "dry-run: would start $CONTROLLER --arm $ARM now"; exit 0; }

mkdir -p "$REPO/runs"   # first run on a fresh checkout has no runs/ dir yet

# One directory per session, holding everything needed to explain the run
# afterwards. Without it a run CSV is linked to the goal and plan that
# produced it only by timestamp adjacency, and the planner diagnostics —
# including the line that says whether the goal orientation was inherited —
# go to the terminal and die with the scrollback.
#
# Created after the GO confirmation, so an aborted or dry-run session leaves
# nothing behind.
SESSION_DIR="$REPO/runs/$(date +%F)/session_$(date +%H%M%S)"
mkdir -p "$SESSION_DIR"
echo "session artifacts: $SESSION_DIR"
cp "$GOAL_YAML" "$SESSION_DIR/goal.yaml"
[[ -f "$PLANNER_YAML" ]] && cp "$PLANNER_YAML" "$SESSION_DIR/planner.yaml"

sha_of() { [[ -f "$1" ]] && sha256sum "$1" 2>/dev/null | cut -d' ' -f1 || echo "unavailable"; }
SESSION_START="$(date -Is)"

SESSION_MARK=$(mktemp)   # anything newer than this was created by THIS session
CONTROLLER_PID=""        # set right after the fork; trap is a no-op until then
CONTROLLER_TAIL_PID=""
declare -A LATEST

# Written on EVERY exit path, so a session that fails early still leaves a
# record of what it was. `wait` before finalising, so the controller has
# flushed its CSV by the time it is linked in.
finalize_session() {
    {
        echo "{"
        echo "  \"git_revision\": \"$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)\","
        echo "  \"git_dirty\": $(if [[ -n "$(git -C "$REPO" status --porcelain 2>/dev/null)" ]]; then echo true; else echo false; fi),"
        echo "  \"arm\": \"$ARM\","
        echo "  \"controller_sha256\": \"$(sha_of "$CONTROLLER")\","
        echo "  \"planner_handoff\": \"in_process_typed_world_cartesian\","
        echo "  \"urdf_sha256\": \"$(sha_of "$URDF")\","
        echo "  \"started\": \"$SESSION_START\","
        echo "  \"ended\": \"$(date -Is)\","
        echo "  \"run_logs\": {"
        local first=1
        for a in "${ARMS[@]}"; do
            [[ $first = 1 ]] || echo ","
            first=0
            printf '    "%s": "%s"' "$a" "${LATEST[$a]:-}"
        done
        echo
        echo "  }"
        echo "}"
    } > "$SESSION_DIR/session.json" 2>/dev/null || true

    # Hard-link the run CSVs in (copy across filesystems). The controller owns
    # the originals under runs/<date>/; this makes the session directory
    # self-contained without duplicating data on the same volume.
    for a in "${ARMS[@]}"; do
        local src="${LATEST[$a]:-}"
        [[ -n "$src" && -f "$src" ]] || continue
        ln "$src" "$SESSION_DIR/$(basename "$src")" 2>/dev/null \
            || cp "$src" "$SESSION_DIR/" 2>/dev/null || true
    done
}

cleanup_session() {
    local rc=$?
    [[ -n "$CONTROLLER_PID" ]] && kill -INT "$CONTROLLER_PID" 2>/dev/null || true
    [[ -n "$CONTROLLER_PID" ]] && wait "$CONTROLLER_PID" 2>/dev/null || true
    [[ -n "$CONTROLLER_TAIL_PID" ]] && kill "$CONTROLLER_TAIL_PID" 2>/dev/null || true
    finalize_session
    rm -f "$SESSION_MARK"
    return "$rc"
}
trap cleanup_session EXIT

# A PLAIN redirect to a file, then `tail -f` for the live view. This looks
# clumsier than `> >(tee log)` and it is deliberate.
#
# Measured on this machine: with `cmd > >(tee log) &`, $! is the wrapper
# subshell, NOT cmd — the child reported $$ = 2445580 while $! was 2445577,
# and `kill -INT "$!"` never reached it (the child ran to completion). The
# trap above is how Ctrl-C stops a MOVING ARM, so that failure is silent and
# dangerous. A plain `> file` forks nothing extra: $! is the controller, and
# the trap works. tests/test_run_session.sh pins this.
"$CONTROLLER" --arm "$ARM" > "$SESSION_DIR/controller.log" 2>&1 & CONTROLLER_PID=$!
tail -n +1 -f "$SESSION_DIR/controller.log" 2>/dev/null & CONTROLLER_TAIL_PID=$!


for a in "${ARMS[@]}"; do
    echo "waiting for the $a controller thread's run log..."
    found=""
    for _ in $(seq 1 60); do
        found=$(find "$REPO/runs" -name "loop_log_${a}*.csv" -newer "$SESSION_MARK" 2>/dev/null | head -1) || true
        [[ -n "$found" ]] && break
        kill -0 "$CONTROLLER_PID" 2>/dev/null || { echo "controller exited during startup"; exit 1; }
        sleep 1
    done
    [[ -n "$found" ]] || { echo "no $a run log appeared after 60 s"; exit 1; }
    LATEST[$a]="$found"
    echo "  $a state source: $found"

    # The file existing is not enough: the controller's CSV header and first
    # data rows may still be buffered. Wait until a meas_j1 header line AND at least one
    # row after it are actually on disk.
    echo "waiting for telemetry data in the $a run log..."
    ready=0
    for _ in $(seq 1 30); do
        if awk '/meas_j1/{h=NR} END{exit !(h && NR>h)}' "$found"; then
            ready=1; break
        fi
        kill -0 "$CONTROLLER_PID" 2>/dev/null || { echo "controller exited before $a telemetry started"; exit 1; }
        sleep 1
    done
    [[ $ready = 1 ]] || { echo "no telemetry rows appeared in $found after 30 s"; exit 1; }
done

# Wait for each selected controller thread to activate its first typed plan.
for a in "${ARMS[@]}"; do
    echo "waiting for the $a controller thread to activate its first plan..."
    activated=0
    for _ in $(seq 1 180); do
        if awk -F, '
            /^#/ { next }
            !header {
                for (i = 1; i <= NF; ++i) if ($i == "cart_traj_activated") column = i
                header = 1
                next
            }
            column && $column == "1" { found = 1; exit }
            END { exit found ? 0 : 1 }
        ' "$found"; then
            activated=1
            break
        fi
        kill -0 "$CONTROLLER_PID" 2>/dev/null || {
            echo "controller exited before $a activated its initial plan"
            exit 1
        }
        sleep 1
    done
    [[ $activated = 1 ]] || {
        echo "no complete $a plan was activated after 180 s"
        exit 1
    }
    echo "  $a: first typed world-Cartesian plan activated; worker remains in controller."
done

echo "In-process planner worker(s) remain active. Press Enter to stop the controller."
read -r
echo "stopping controller..."
