#!/usr/bin/env python3
"""Control panel for basic_control — a local web front-end for the knobs.

What it does:
  * shows the tunable constants from basic_control/src/Config.h with their
    meaning, lets you edit them in the browser, and writes them back
    (a one-time backup Config.h.panel.bak is kept per session);
  * rebuilds the controller on demand and shows the build output;
  * lists run CSVs and summarizes each run's health signals
    (posture error, base compensation, replan advisories, trajectory edges);
  * prints the exact terminal commands for a hardware session.

What it deliberately does NOT do:
  * it never starts the controller or the planner — hardware sessions stay
    in your terminal, behind run_session.sh and its typed GO gate;
  * it never touches the guard overrides (kStopOnFault and friends) —
    safety policy must not be one click away.

Run:  python3 Christian_control/tools/control_panel.py
Then open  http://127.0.0.1:8765  (localhost only, nothing is exposed).
"""

import json
import re
import shutil
import subprocess
import sys
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
CONFIG_H = REPO / "Christian_control/basic_control/src/Config.h"
BUILD_DIR = REPO / "Christian_control/basic_control/build"
RUNS_DIR = REPO / "runs"
BACKUP = CONFIG_H.with_suffix(".h.panel.bak")

# name -> (type, one-line meaning). The whitelist IS the permission model:
# only these constants can be read or written through the panel.
KNOBS = {
    "kTrajectoryPosePrimary": ("bool", "Trajectory mode: true = pose-primary (Cartesian law owns the command, plan guides the posture), false = joint mode (today's joint tracking)"),
    "kPostureEnabled": ("bool", "Null-space posture guidance toward the planner's q_nom"),
    "kPostureGain": ("double", "1/s pull toward q_nom; keep well below kKpCartesian (leak!)"),
    "kKpCartesian": ("double", "1/s on the Cartesian position error"),
    "kKpRotation": ("double", "1/s on the rotation-log error"),
    "kKdPosition": ("double", "Gain on the linear-velocity error (damping/feedforward)"),
    "kKdRotation": ("double", "Gain on the angular-velocity error"),
    "kDlsLambda": ("double", "DLS damping; larger = slower but safer near singularities"),
    "kOrientationEnabled": ("bool", "Track orientation as well as position"),
    "kVelocityTermEnabled": ("bool", "Kd term (twist error) on/off"),
    "kNullSpaceEnabled": ("bool", "Null-space objectives (limit avoidance + posture) on/off"),
    "kLimitAvoidZoneDeg": ("double", "Zone before a software limit where avoidance wakes up"),
    "kLimitAvoidGain": ("double", "1/s push per degree of zone intrusion"),
    "kKpJointTracking": ("double", "1/s on joint error (joint mode only)"),
    "kTrajStartToleranceDeg": ("double", "Splice guard: reject a plan starting farther than this"),
    "kTrajFollowingErrorStopDeg": ("double", "Joint-mode plan-deviation stop (deg)"),
    "kArrivalToleranceM": ("double", "Arrival position tolerance (m)"),
    "kArrivalOrientationToleranceRad": ("double", "Arrival orientation tolerance (rad)"),
    "kReplanSigmaMin": ("double", "Advise replan when sigma_min(J) falls below this"),
    "kReplanJointMarginDeg": ("double", "Advise replan when a joint's limit margin falls below this"),
    "kReplanPostureErrorDeg": ("double", "Advise replan when plan deviation exceeds this"),
    "kReplanPositionErrorM": ("double", "Advise replan when tracking error exceeds this"),
    "kReplanAdviseCycles": ("int", "Consecutive degraded cycles before the advisory raises"),
}

KNOB_RE = r"(inline\s+constexpr\s+(?:double|bool|int)\s+{name}\s*=\s*)([^;]+)(;)"


def read_knobs():
    text = CONFIG_H.read_text()
    out = {}
    for name, (ktype, doc) in KNOBS.items():
        m = re.search(KNOB_RE.format(name=name), text)
        out[name] = {
            "type": ktype,
            "doc": doc,
            "value": m.group(2).strip() if m else None,
        }
    return out


def write_knob(name, value):
    if name not in KNOBS:
        return False, "unknown knob (not on the whitelist)"
    ktype = KNOBS[name][0]
    if ktype == "bool":
        if value not in ("true", "false"):
            return False, "bool knobs take true or false"
        rendered = value
    elif ktype == "int":
        try:
            rendered = str(int(value))
        except ValueError:
            return False, "not an integer"
    else:
        try:
            rendered = repr(float(value))
        except ValueError:
            return False, "not a number"
    text = CONFIG_H.read_text()
    pattern = KNOB_RE.format(name=name)
    if not re.search(pattern, text):
        return False, f"{name} not found in Config.h"
    if not BACKUP.exists():
        shutil.copy2(CONFIG_H, BACKUP)
    CONFIG_H.write_text(re.sub(pattern, rf"\g<1>{rendered}\g<3>", text, count=1))
    return True, rendered


def build_controller():
    result = subprocess.run(
        ["cmake", "--build", str(BUILD_DIR), "-j8"],
        capture_output=True, text=True, timeout=600,
    )
    tail = "\n".join((result.stdout + result.stderr).splitlines()[-40:])
    return result.returncode == 0, tail


def list_runs(limit=30):
    if not RUNS_DIR.exists():
        return []
    files = sorted(RUNS_DIR.glob("*/loop_log_*.csv"),
                   key=lambda p: p.stat().st_mtime, reverse=True)[:limit]
    return [{"file": str(p.relative_to(RUNS_DIR)),
             "mb": round(p.stat().st_size / 1e6, 1)} for p in files]


def summarize_run(rel):
    path = (RUNS_DIR / rel).resolve()
    if not str(path).startswith(str(RUNS_DIR.resolve())) or not path.is_file():
        return {"error": "no such run"}
    preamble, header = [], None
    tail = deque(maxlen=5000)
    with open(path, errors="replace") as f:
        for line in f:
            if line.startswith("#"):
                preamble.append(line.rstrip())
            elif header is None:
                header = line.rstrip().split(",")
            else:
                tail.append(line.rstrip())
    if header is None:
        return {"error": "no data rows", "preamble": preamble[:40]}
    idx = {n: i for i, n in enumerate(header)}

    def col_stats(name, flag=False):
        i = idx.get(name)
        if i is None:
            return None
        best, fired = None, 0
        for row in tail:
            parts = row.split(",")
            if i >= len(parts):
                continue
            v = parts[i]
            if flag:
                fired += v == "1"
            else:
                try:
                    x = float(v)
                except ValueError:
                    continue
                if x == x and (best is None or x > best):
                    best = x
        return fired if flag else best

    return {
        "preamble": preamble[:45],
        "rows_sampled": len(tail),
        "max_posture_error_deg": col_stats("posture_error_deg"),
        "max_base_comp_m": col_stats("base_comp_m"),
        "min_joint_margin_deg": None,  # min needs its own pass below
        "replan_advised_rows": col_stats("replan_advised", flag=True),
        "traj_activated_rows": col_stats("traj_activated", flag=True),
        "traj_rejected_rows": col_stats("traj_rejected", flag=True),
        "traj_complete_rows": col_stats("traj_complete", flag=True),
        "max_joint_follow_error_deg": col_stats("joint_follow_error_deg"),
    }


PAGE = """<!doctype html><meta charset="utf-8">
<title>basic_control panel</title>
<style>
 body{font:14px/1.45 system-ui;margin:1.2rem;max-width:70rem}
 h1{font-size:1.2rem} h2{font-size:1rem;margin-top:1.6rem}
 table{border-collapse:collapse;width:100%} td,th{padding:.3rem .5rem;border-bottom:1px solid #ddd;text-align:left;vertical-align:top}
 input[type=text]{width:7rem} .doc{color:#555;font-size:.85rem}
 button{margin:.2rem .2rem .2rem 0;padding:.3rem .8rem}
 pre{background:#f6f6f6;padding:.6rem;overflow-x:auto;font-size:.8rem}
 .warn{background:#fff3cd;padding:.5rem;border:1px solid #e0c068}
 .ok{color:#0a7a0a}.bad{color:#b00020}
</style>
<h1>basic_control panel</h1>
<p class="warn">This panel edits <code>Config.h</code> and rebuilds. It never
starts the robot — hardware sessions stay in your terminal:
<code>Christian_control/planner_bridge/scripts/run_session.sh</code>
(it launches the controller AND pipes the planner into it, then waits for
your typed <b>GO</b>).</p>
<h2>Knobs (compile-time — press Build after changing)</h2>
<table id="knobs"></table>
<button onclick="build()">Build controller</button>
<span id="buildstate"></span>
<pre id="buildlog" hidden></pre>
<h2>Runs</h2>
<div id="runs"></div>
<pre id="runsummary" hidden></pre>
<script>
async function j(u,opt){const r=await fetch(u,opt);return r.json()}
async function loadKnobs(){
 const k=await j('/api/config');const t=document.getElementById('knobs');
 t.innerHTML='<tr><th>knob</th><th>value</th><th></th><th class="doc">meaning</th></tr>';
 for(const [n,v] of Object.entries(k)){
  const row=t.insertRow();row.insertCell().textContent=n;
  const c=row.insertCell();
  if(v.type==='bool'){const b=document.createElement('button');
   b.textContent=v.value;b.onclick=()=>setKnob(n,v.value==='true'?'false':'true');
   c.appendChild(b);}
  else{const i=document.createElement('input');i.type='text';i.value=v.value;
   i.id='in_'+n;c.appendChild(i);}
  const a=row.insertCell();
  if(v.type!=='bool'){const s=document.createElement('button');s.textContent='set';
   s.onclick=()=>setKnob(n,document.getElementById('in_'+n).value);a.appendChild(s);}
  row.insertCell().innerHTML='<span class="doc">'+v.doc+'</span>';
 }}
async function setKnob(n,val){
 const r=await j('/api/set',{method:'POST',body:JSON.stringify({name:n,value:val})});
 if(r.error)alert(r.error);loadKnobs();}
async function build(){
 const s=document.getElementById('buildstate');s.textContent=' building…';
 const log=document.getElementById('buildlog');log.hidden=false;log.textContent='';
 const r=await j('/api/build',{method:'POST'});
 s.innerHTML=r.ok?'<span class="ok"> build OK</span>':'<span class="bad"> build FAILED</span>';
 log.textContent=r.log;}
async function loadRuns(){
 const runs=await j('/api/runs');const d=document.getElementById('runs');
 d.innerHTML='';
 for(const r of runs){const b=document.createElement('button');
  b.textContent=r.file+' ('+r.mb+' MB)';
  b.onclick=async()=>{const s=document.getElementById('runsummary');
   s.hidden=false;s.textContent=JSON.stringify(await j('/api/run?f='+encodeURIComponent(r.file)),null,2);};
  d.appendChild(b);d.appendChild(document.createElement('br'));}}
loadKnobs();loadRuns();
</script>"""


class Handler(BaseHTTPRequestHandler):
    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/config":
            self._json(read_knobs())
        elif self.path == "/api/runs":
            self._json(list_runs())
        elif self.path.startswith("/api/run?f="):
            from urllib.parse import unquote
            self._json(summarize_run(unquote(self.path.split("=", 1)[1])))
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        if self.path == "/api/set":
            length = int(self.headers.get("Content-Length", 0))
            try:
                req = json.loads(self.rfile.read(length))
                ok, msg = write_knob(str(req.get("name")), str(req.get("value")))
            except (json.JSONDecodeError, TypeError):
                ok, msg = False, "bad request"
            self._json({"ok": ok} if ok else {"error": msg})
        elif self.path == "/api/build":
            try:
                ok, log = build_controller()
            except subprocess.TimeoutExpired:
                ok, log = False, "build timed out"
            self._json({"ok": ok, "log": log})
        else:
            self._json({"error": "not found"}, 404)

    def log_message(self, *args):  # keep the terminal quiet
        pass


def main():
    if not CONFIG_H.is_file():
        sys.exit(f"cannot find {CONFIG_H}")
    server = ThreadingHTTPServer(("127.0.0.1", 8765), Handler)
    print("control panel: http://127.0.0.1:8765  (Ctrl+C to stop)")
    print(f"editing: {CONFIG_H}")
    print(f"backup : {BACKUP} (created on first change)")
    server.serve_forever()


if __name__ == "__main__":
    main()
