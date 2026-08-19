#!/usr/bin/env bash
# Hardware-free rehearsal of the single-process run_session.sh workflow.
# The controller below is a throwaway stub; no Kortex or planner process runs.

set -uo pipefail

REAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCRIPT_UNDER_TEST="$REAL_REPO/Christian_control/planning/scripts/run_session.sh"
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
    mkdir -p "$root/Christian_control/runtime/build/planning/config" \
             "$root/Christian_control/control" \
             "$root/Christian_control/runtime" \
             "$root/Christian_control/model" \
             "$root/Christian_control/planning/src" \
             "$root/Christian_control/planning/optimisation" \
             "$root/Christian_control/planning/config" \
             "$root/Christian_control/planning/scripts" \
             "$root/runs"

    cp "$SCRIPT_UNDER_TEST" \
       "$root/Christian_control/planning/scripts/run_session.sh"
    chmod +x "$root/Christian_control/planning/scripts/run_session.sh"
    printf 'int main(){}\n' > "$root/Christian_control/control/x.cpp"
    printf 'int main(){}\n' > "$root/Christian_control/runtime/x.cpp"
    printf 'int main(){}\n' > "$root/Christian_control/planning/src/x.cpp"
    printf 'int main(){}\n' > "$root/Christian_control/planning/optimisation/x.cpp"
    printf 'frame: mount\n' > \
        "$root/Christian_control/model/GEN3_dual_mounted.urdf"
    printf 'session_arms: right\nright:\n  goal: [0,0,0]\nleft:\n  goal: [0,0,0]\n' > \
        "$root/Christian_control/planning/config/goal.yaml"
    printf 'motion:\n  nominal_speed_mps: 0.05\n' > \
        "$root/Christian_control/planning/config/planner.yaml"
    printf 'a: 1\n' > \
        "$root/Christian_control/runtime/build/planning/config/dh_params_tool.yaml"
    printf 'a: 1\n' > \
        "$root/Christian_control/runtime/build/planning/config/dh_params_flange.yaml"

    cat > "$root/Christian_control/runtime/build/controller" <<'CTRL'
#!/usr/bin/env python3
import os
import signal
import sys
import time
from datetime import date, datetime

selected = "right"
mount = ""
record = "on"
for index, value in enumerate(sys.argv):
    if value == "--arm" and index + 1 < len(sys.argv):
        selected = sys.argv[index + 1]
    if value == "--mount" and index + 1 < len(sys.argv):
        mount = sys.argv[index + 1]
    if value == "--record" and index + 1 < len(sys.argv):
        record = sys.argv[index + 1]
if mount not in ("fixed", "vicon"):
    print("error: --mount is required (fixed or vicon)", flush=True)
    raise SystemExit(2)
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
for arm in arms if record == "on" else []:
    csv_path = os.path.join(day, f"loop_log_{arm}_{datetime.now():%H%M%S%f}.csv")
    with open(csv_path, "w") as output:
        output.write("# log_format = 13\n")
        output.write("time_s,meas_j1,cart_traj_activated\n")
        output.write(f"0.0,1,{'0' if fail else '1'}\n")
with open(os.path.join(root, "controller_argv.txt"), "w") as output:
    output.write(" ".join(sys.argv[1:]) + "\n")
print(f"stub controller: ready for {selected}", flush=True)
if fail:
    raise SystemExit(3)
while True:
    time.sleep(0.1)
CTRL
    chmod +x "$root/Christian_control/runtime/build/controller"
    echo "$root"
}

run_session() { # $1 root, $2 arm, $3 STUB_PLAN_FAIL, remaining args pass through
    local root="$1" arm="$2" fail="${3:-0}"
    shift 3 || shift $#
    STUB_PLAN_FAIL="$fail" timeout 15 \
        bash -c "printf 'GO\\n\\n' | '$root/Christian_control/planning/scripts/run_session.sh' --arm '$arm' $*" \
        > "$root/session_stdout.txt" 2>&1
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

echo "== plan off: no DH gate, no activation wait, stub never activates =="
ROOT_NOPLAN="$(make_fake_repo 1)"   # fail=1: the stub NEVER activates a plan
rm -f "$ROOT_NOPLAN"/Christian_control/runtime/build/planning/config/*.yaml
run_session "$ROOT_NOPLAN" right 0 --mount fixed --plan off
grep -q "session artifacts:" "$ROOT_NOPLAN/session_stdout.txt" \
    && check "plan-off session starts with DH tables deleted" ok \
    || check "plan-off session starts with DH tables deleted" no
! grep -q "activate its first plan" "$ROOT_NOPLAN/session_stdout.txt" \
    && check "plan-off session never waits for plan activation" ok \
    || check "plan-off session never waits for plan activation" no
grep -q "Controller holding (planning off)" "$ROOT_NOPLAN/session_stdout.txt" \
    && check "plan-off session reaches the running prompt" ok \
    || check "plan-off session reaches the running prompt" no
grep -q -- "--mount fixed --plan off --record on" "$ROOT_NOPLAN/controller_argv.txt" 2>/dev/null \
    && check "controller received the session's choices" ok \
    || check "controller received the session's choices" no

echo "== record off: no CSV waits, session still supervises the process =="
ROOT_NOREC="$(make_fake_repo 0)"
run_session "$ROOT_NOREC" right 0 --mount fixed --plan off --record off
grep -q "recording off: no run CSVs" "$ROOT_NOREC/session_stdout.txt" \
    && check "record-off session skips the log waits" ok \
    || check "record-off session skips the log waits" no
! find "$ROOT_NOREC/runs" -maxdepth 2 -name 'loop_log_*.csv' | grep -q . \
    && check "record-off session produced no run CSV" ok \
    || check "record-off session produced no run CSV" no
[[ -f "$ROOT_NOREC/controller_sigint.txt" ]] \
    && check "record-off session still stops the controller cleanly" ok \
    || check "record-off session still stops the controller cleanly" no

echo "== vicon + plan on stays gated: stale DH still refuses =="
ROOT_STALE="$(make_fake_repo 0)"
touch "$ROOT_STALE/Christian_control/model/GEN3_dual_mounted.urdf"
run_session "$ROOT_STALE" right 0 --mount vicon --plan on
grep -q "STALE.*older than the URDF" "$ROOT_STALE/session_stdout.txt" \
    && check "plan-on session still refuses on stale DH tables" ok \
    || check "plan-on session still refuses on stale DH tables" no

rm -rf "$ROOT_OK" "$ROOT_BAD" "$ROOT_NOPLAN" "$ROOT_NOREC" "$ROOT_STALE"
rm -f /tmp/humansl_bridge_targets_right /tmp/humansl_bridge_targets_left \
      /tmp/humansl_planning_requests_right /tmp/humansl_planning_requests_left
echo
if [[ $FAILURES -eq 0 ]]; then
    echo "test_run_session: PASSED"
else
    echo "test_run_session: FAILED ($FAILURES)"
fi
exit $((FAILURES > 0))
