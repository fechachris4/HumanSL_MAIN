#!/usr/bin/env bash
# Hardware-free rehearsal of scripts/run_session.sh against stub binaries.
#
# Three properties, all of which the session-artifact change could plausibly
# have broken:
#
#   A. The EXIT trap's `kill -INT` still reaches the CONTROLLER. $! must be
#      the controller itself. Measured here: with `cmd > >(tee log) &` it is
#      NOT — $! is a wrapper subshell and the signal never arrives, which
#      would silently stop Ctrl-C reaching a MOVING ARM. A plain `> file`
#      redirect forks nothing extra and keeps $! correct. This is the reason
#      this file exists.
#   B. What reaches the target pipe is byte-identical to the saved plan.
#   C. A failing bridge sends nothing at all to the pipe.
#
# No robot, no Kortex, no real binaries: a fake repo tree with shell stubs.

set -uo pipefail

REAL_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCRIPT_UNDER_TEST="$REAL_REPO/Christian_control/planner_bridge/scripts/run_session.sh"
FAILURES=0

check() { # $1 condition-description, $2 = "ok" or anything else
    if [[ "$2" == "ok" ]]; then
        echo "  ok   $1"
    else
        echo "  FAIL $1"
        FAILURES=$((FAILURES + 1))
    fi
}

# Builds a throwaway repo tree with stub binaries. $1 = bridge exit code.
make_fake_repo() {
    local bridge_rc="$1"
    local root; root="$(mktemp -d)"
    mkdir -p "$root/Christian_control/basic_control/build" \
             "$root/Christian_control/basic_control/src" \
             "$root/Christian_control/basic_control/config" \
             "$root/Christian_control/planner_bridge/build/config" \
             "$root/Christian_control/planner_bridge/src" \
             "$root/Christian_control/planner_bridge/trajectory_generation" \
             "$root/Christian_control/planner_bridge/config" \
             "$root/Christian_control/planner_bridge/scripts" \
             "$root/runs"

    cp "$SCRIPT_UNDER_TEST" "$root/Christian_control/planner_bridge/scripts/run_session.sh"
    chmod +x "$root/Christian_control/planner_bridge/scripts/run_session.sh"

    # Sources must be OLDER than the binaries or fresh_or_die refuses to run.
    # Every directory fresh_or_die is pointed at must EXIST, too: it runs
    # `find` under `set -euo pipefail`, so a missing directory aborts the whole
    # script before it does anything — which is how this file silently stopped
    # testing what it is for when trajectory_generation moved here.
    echo "int main(){}" > "$root/Christian_control/basic_control/src/x.cpp"
    echo "int main(){}" > "$root/Christian_control/planner_bridge/src/x.cpp"
    echo "int main(){}" > "$root/Christian_control/planner_bridge/trajectory_generation/x.cpp"
    printf 'frame: mount\n' > "$root/Christian_control/basic_control/config/GEN3_dual_mounted.urdf"
    printf 'session_arms: right\nright:\n  frame: mount\n  goal: [0,0,0]\n' \
        > "$root/Christian_control/planner_bridge/config/goal.yaml"
    printf 'motion:\n  nominal_speed_mps: 0.05\n' \
        > "$root/Christian_control/planner_bridge/config/planner.yaml"
    printf 'a: 1\n' > "$root/Christian_control/planner_bridge/build/config/dh_params_tool.yaml"
    printf 'a: 1\n' > "$root/Christian_control/planner_bridge/build/config/dh_params_flange.yaml"
    sleep 0.05

    # Stub controller, in PYTHON deliberately — not bash.
    #
    # A background job started from a non-interactive shell inherits SIGINT as
    # SIG_IGN, and bash REFUSES to install a trap for a signal it inherited as
    # ignored. A bash stub therefore cannot catch the trap's kill -INT, and a
    # test built on one measures the stub rather than the system. The real
    # controller is C++ and calls std::signal(SIGINT, ...) at Main.cpp:562,
    # which DOES reset an inherited SIG_IGN. Python's signal.signal() resets it
    # the same way, so this stub is faithful where a shell one is not.
    #
    # It prints the disposition it inherited, so the log records the fact
    # rather than leaving it as folklore.
    cat > "$root/Christian_control/basic_control/build/controller" <<'CTRL'
#!/usr/bin/env python3
import os, signal, sys, threading, time
from datetime import date, datetime

arm = "right"
a = sys.argv[1:]
for i, v in enumerate(a):
    if v == "--arm" and i + 1 < len(a):
        arm = a[i + 1]
root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))

print(f"stub controller: starting for arm {arm}", flush=True)
print(f"stub controller: inherited SIGINT disposition = "
      f"{'SIG_IGN' if signal.getsignal(signal.SIGINT) is signal.SIG_IGN else 'default/handler'}",
      flush=True)

def on_sigint(signum, frame):
    with open(os.path.join(root, f"sigint_{arm}.txt"), "w") as f:
        f.write("SIGINT\n")
    sys.exit(0)

signal.signal(signal.SIGINT, on_sigint)   # resets an inherited SIG_IGN

day = os.path.join(root, "runs", date.today().isoformat())
os.makedirs(day, exist_ok=True)
csv = os.path.join(day, f"loop_log_{arm}_{datetime.now():%H%M%S}.csv")
with open(csv, "w") as f:
    f.write("# log_format = 9\ntime_s,meas_j1,meas_j2\n0.0,1.0,2.0\n")

pipe = f"/tmp/humansl_bridge_targets_{arm}"
if os.path.exists(pipe):
    os.remove(pipe)
os.mkfifo(pipe)
received = os.path.join(root, f"received_{arm}.txt")
open(received, "w").close()

def drain():
    with open(pipe, "rb") as p, open(received, "wb") as out:
        out.write(p.read())
threading.Thread(target=drain, daemon=True).start()

for _ in range(300):
    time.sleep(0.1)
CTRL
    chmod +x "$root/Christian_control/basic_control/build/controller"

    # Stub bridge: a recognisable plan on stdout, diagnostics on stderr.
    cat > "$root/Christian_control/planner_bridge/build/planner_bridge" <<BRIDGE
#!/usr/bin/env bash
echo "goal orientation: INHERITED from the start pose" >&2
echo "TRAJ_BEGIN 2"
echo "0.000000 1 2 3 4 5 6 7 0 0 0 0 0 0 0"
echo "1.000000 1 2 3 4 5 6 7 0 0 0 0 0 0 0"
echo "TRAJ_END"
exit $bridge_rc
BRIDGE
    chmod +x "$root/Christian_control/planner_bridge/build/planner_bridge"
    echo "$root"
}

run_session() { # $1 repo root -> exits when the script does
    printf 'GO\n\n' | timeout 60 \
        "$1/Christian_control/planner_bridge/scripts/run_session.sh" --arm right \
        > "$1/session_stdout.txt" 2>&1
}

echo "== A/B: bridge succeeds =="
ROOT_OK="$(make_fake_repo 0)"
run_session "$ROOT_OK"
SESSION_DIR="$(find "$ROOT_OK/runs" -maxdepth 2 -type d -name 'session_*' | head -1)"

[[ -n "$SESSION_DIR" ]] && check "a session directory is created" ok \
    || check "a session directory is created" no

# A. THE ONE THAT MATTERS.
[[ -f "$ROOT_OK/sigint_right.txt" ]] \
    && check "trap's kill -INT reaches the controller (\$! is the controller)" ok \
    || check "trap's kill -INT reaches the controller (\$! is the controller)" no

# B.
if [[ -f "$SESSION_DIR/plan_right.traj" && -f "$ROOT_OK/received_right.txt" ]] \
   && cmp -s "$SESSION_DIR/plan_right.traj" "$ROOT_OK/received_right.txt"; then
    check "what reached the pipe is byte-identical to the saved plan" ok
else
    check "what reached the pipe is byte-identical to the saved plan" no
fi

for f in goal.yaml planner.yaml controller.log bridge_right.log session.json; do
    [[ -s "$SESSION_DIR/$f" ]] && check "captured $f" ok || check "captured $f" no
done
grep -q "INHERITED" "$SESSION_DIR/bridge_right.log" 2>/dev/null \
    && check "bridge stderr diagnostics are captured, not lost to the terminal" ok \
    || check "bridge stderr diagnostics are captured, not lost to the terminal" no
ls "$SESSION_DIR"/loop_log_right_*.csv >/dev/null 2>&1 \
    && check "the run CSV is linked into the session directory" ok \
    || check "the run CSV is linked into the session directory" no

echo "== C: bridge fails =="
ROOT_BAD="$(make_fake_repo 3)"
run_session "$ROOT_BAD"
SESSION_BAD="$(find "$ROOT_BAD/runs" -maxdepth 2 -type d -name 'session_*' | head -1)"

[[ ! -s "$ROOT_BAD/received_right.txt" ]] \
    && check "a failing bridge sends nothing to the pipe" ok \
    || check "a failing bridge sends nothing to the pipe" no
grep -q "bridge exited 3" "$ROOT_BAD/session_stdout.txt" 2>/dev/null \
    && check "the bridge's exit code is reported to the operator" ok \
    || check "the bridge's exit code is reported to the operator" no
[[ -f "$SESSION_BAD/session.json" ]] \
    && check "a failed session still leaves a session.json" ok \
    || check "a failed session still leaves a session.json" no

rm -rf "$ROOT_OK" "$ROOT_BAD"
rm -f /tmp/humansl_bridge_targets_right
echo
if [[ $FAILURES -eq 0 ]]; then echo "test_run_session: PASSED"; else echo "test_run_session: FAILED ($FAILURES)"; fi
exit $((FAILURES > 0))
