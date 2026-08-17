#!/usr/bin/env bash
# Hardware-free rehearsal of the single-process run_session.sh workflow.
# The controller below is a throwaway stub; no Kortex or planner process runs.

set -uo pipefail

REAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCRIPT_UNDER_TEST="$REAL_REPO/Christian_control/planner_bridge/scripts/run_session.sh"
FAILURES=0

check() {
    if [[ "$2" == "ok" ]]; then
        echo "  ok   $1"
    else
        echo "  FAIL $1"
        FAILURES=$((FAILURES + 1))
    fi
}

make_fake_repo() { # $1: 0 = activate a plan, 1 = exit before activation
    local fail="$1"
    local root
    root="$(mktemp -d)"
    mkdir -p "$root/Christian_control/basic_control/build/planner_bridge/config" \
             "$root/Christian_control/basic_control/src" \
             "$root/Christian_control/basic_control/config" \
             "$root/Christian_control/planner_bridge/src" \
             "$root/Christian_control/planner_bridge/trajectory_generation" \
             "$root/Christian_control/planner_bridge/config" \
             "$root/Christian_control/planner_bridge/scripts" \
             "$root/runs"

    cp "$SCRIPT_UNDER_TEST" \
       "$root/Christian_control/planner_bridge/scripts/run_session.sh"
    chmod +x "$root/Christian_control/planner_bridge/scripts/run_session.sh"
    printf 'int main(){}\n' > "$root/Christian_control/basic_control/src/x.cpp"
    printf 'int main(){}\n' > "$root/Christian_control/planner_bridge/src/x.cpp"
    printf 'int main(){}\n' > "$root/Christian_control/planner_bridge/trajectory_generation/x.cpp"
    printf 'frame: mount\n' > \
        "$root/Christian_control/basic_control/config/GEN3_dual_mounted.urdf"
    printf 'session_arms: right\nright:\n  goal: [0,0,0]\nleft:\n  goal: [0,0,0]\n' > \
        "$root/Christian_control/planner_bridge/config/goal.yaml"
    printf 'motion:\n  nominal_speed_mps: 0.05\n' > \
        "$root/Christian_control/planner_bridge/config/planner.yaml"
    printf 'a: 1\n' > \
        "$root/Christian_control/basic_control/build/planner_bridge/config/dh_params_tool.yaml"
    printf 'a: 1\n' > \
        "$root/Christian_control/basic_control/build/planner_bridge/config/dh_params_flange.yaml"

    cat > "$root/Christian_control/basic_control/build/controller" <<'CTRL'
#!/usr/bin/env python3
import os
import signal
import sys
import time
from datetime import date, datetime

selected = "right"
for index, value in enumerate(sys.argv):
    if value == "--arm" and index + 1 < len(sys.argv):
        selected = sys.argv[index + 1]
arms = ["right", "left"] if selected == "both" else [selected]
root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
fail = os.environ.get("STUB_PLAN_FAIL") == "1"

def on_sigint(signum, frame):
    with open(os.path.join(root, "controller_sigint.txt"), "w") as output:
        output.write("SIGINT\n")
    raise SystemExit(0)

signal.signal(signal.SIGINT, on_sigint)
day = os.path.join(root, "runs", date.today().isoformat())
os.makedirs(day, exist_ok=True)
for arm in arms:
    csv_path = os.path.join(day, f"loop_log_{arm}_{datetime.now():%H%M%S%f}.csv")
    with open(csv_path, "w") as output:
        output.write("# log_format = 13\n")
        output.write("time_s,meas_j1,cart_traj_activated\n")
        output.write(f"0.0,1,{'0' if fail else '1'}\n")
print(f"stub controller: ready for {selected}", flush=True)
if fail:
    raise SystemExit(3)
while True:
    time.sleep(0.1)
CTRL
    chmod +x "$root/Christian_control/basic_control/build/controller"
    echo "$root"
}

run_session() { # $1 root, $2 arm, optional STUB_PLAN_FAIL=1
    STUB_PLAN_FAIL="${3:-0}" timeout 15 \
        bash -c "printf 'GO\\n\\n' | '$1/Christian_control/planner_bridge/scripts/run_session.sh' --arm '$2'" \
        > "$1/session_stdout.txt" 2>&1
}

echo "== single controller process succeeds for both arms =="
ROOT_OK="$(make_fake_repo 0)"
run_session "$ROOT_OK" both 0
SESSION_DIR="$(find "$ROOT_OK/runs" -maxdepth 2 -type d -name 'session_*' | head -1)"
[[ -n "$SESSION_DIR" ]] && check "session directory created" ok || check "session directory created" no
[[ -f "$ROOT_OK/controller_sigint.txt" ]] \
    && check "EXIT trap SIGINT reaches the controller process" ok \
    || check "EXIT trap SIGINT reaches the controller process" no
[[ ! -e /tmp/humansl_bridge_targets_right &&
   ! -e /tmp/humansl_bridge_targets_left &&
   ! -e /tmp/humansl_planning_requests_right &&
   ! -e /tmp/humansl_planning_requests_left ]] \
    && check "no production FIFO paths are created" ok \
    || check "no production FIFO paths are created" no
for arm in right left; do
    if find "$ROOT_OK/runs" -maxdepth 2 -name "loop_log_${arm}_*.csv" -print -quit | grep -q .; then
        check "$arm controller log exists" ok
    else
        check "$arm controller log exists" no
    fi
done
for file in goal.yaml planner.yaml controller.log session.json; do
    [[ -s "$SESSION_DIR/$file" ]] && check "captured $file" ok \
        || check "captured $file" no
done
grep -q 'planner_handoff.*in_process_typed_world_cartesian' "$SESSION_DIR/session.json" \
    && check "session provenance records typed in-process handoff" ok \
    || check "session provenance records typed in-process handoff" no

echo "== failed controller startup is reported without planner artifacts =="
ROOT_BAD="$(make_fake_repo 1)"
run_session "$ROOT_BAD" right 1
grep -q "controller exited before right activated its initial plan" \
    "$ROOT_BAD/session_stdout.txt" \
    && check "failed activation is reported to the operator" ok \
    || check "failed activation is reported to the operator" no
BAD_SESSION="$(find "$ROOT_BAD/runs" -maxdepth 3 -name session.json | head -1)"
[[ -f "$BAD_SESSION" ]] && check "failed session still leaves session.json" ok \
    || check "failed session still leaves session.json" no
! find "$ROOT_BAD/runs" -maxdepth 2 -name 'published_*.ok' | grep -q . \
    && check "failed session leaves no publication marker" ok \
    || check "failed session leaves no publication marker" no

rm -rf "$ROOT_OK" "$ROOT_BAD"
rm -f /tmp/humansl_bridge_targets_right /tmp/humansl_bridge_targets_left \
      /tmp/humansl_planning_requests_right /tmp/humansl_planning_requests_left
echo
if [[ $FAILURES -eq 0 ]]; then
    echo "test_run_session: PASSED"
else
    echo "test_run_session: FAILED ($FAILURES)"
fi
exit $((FAILURES > 0))
