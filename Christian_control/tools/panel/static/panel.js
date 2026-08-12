/*
  The panel's state and wiring: it fetches, it listens, it renders, and it
  sends the three write actions the design allows (set a knob, build, start or
  stop a session). It contains no judgement about whether motion is safe —
  that lives in the controller, and this file only triggers paths that already
  exist there.

  Two things it is deliberately careful about.

  The stop control is never gated on the state of this file. It is a plain
  POST that runs whether telemetry is dead, the page is stale, or the view
  modules failed to load. Every call into scene.js and readouts.js therefore
  goes through call() below, so a missing or broken method degrades to nothing
  drawn rather than a thrown exception that takes the page — and the stop
  button's click handler — down with it.

  NaN is never turned into a number. The controller logs 'nan' for "this cycle
  did not compute it", and 0.0 there would read as perfect tracking. Anything
  unknown stays null and prints as an em dash.

  The two sibling view modules own the drawing; this file owns what they are
  told:

    scene.js     createScene(canvas, {frameLabel}) -> setDh, setTelemetry,
                 setPlan, setProgress, setObstacle, setClearance,
                 setSelectedArm, resize, draw. It reads its palette from
                 panel.css, so colour is defined in one place.
    readouts.js  createSlot -> {el, set, setLevel}
                 createErrorRow/createJointBar/createBanner/createStaleness
                 -> {el, update(...)}

  The server shapes this file reads are the ones the Python modules actually
  produce:

    /api/status    {freshness:{controller,bridge,dh,stale,reasons}, session,
                    goal_arms, pipes, dh, replay, hardware, wall_ms}
    /api/config    {knobs:{name:{type,doc,value}},
                    thresholds:{name:{value,unit,consequence}},
                    joints:{bounded_mask, lower_deg, upper_deg, warn_deg,
                            error_deg, software_limit_deg, velocity_limit_deg_s}}
    SSE hello      {arm, wall_ms, replay, local}   — where "may I start" comes from
    SSE telemetry  {arm, wall_ms, row, worst, server_age_s, growing, rows_seen}

  telemetry.py has already turned every 'nan' into null before it reaches here,
  so a missing value arrives as null and stays null.
*/

import { createScene } from './scene.js';
import {
  createSlot,
  createErrorRow,
  createJointBar,
  createBanner,
  createStaleness,
} from './readouts.js';

/* ----------------------------------------------------------- small tools */

const $ = (id) => document.getElementById(id);

// Calls into the view modules. See the note at the top of the file: a method
// this file expects but that module does not provide must not be able to
// break the stop button.
function call(obj, method, ...args) {
  const fn = obj && obj[method];
  if (typeof fn !== 'function') return undefined;
  try {
    return fn.apply(obj, args);
  } catch (err) {
    console.error(`${method} failed`, err);
    return undefined;
  }
}

function mount(parent, made) {
  const node = made && made.el ? made.el : (made instanceof Node ? made : null);
  if (node) parent.appendChild(node);
  return made;
}

async function getJSON(url) {
  const response = await fetch(url, { cache: 'no-store' });
  if (!response.ok) throw new Error(`${url} returned ${response.status}`);
  return response.json();
}

async function postJSON(url, body) {
  const response = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body ?? {}),
  });
  // The API reports its own failures in the body, so a non-2xx still carries
  // the reason the operator needs to read.
  let payload = null;
  try { payload = await response.json(); } catch { payload = null; }
  if (payload === null) throw new Error(`${url} returned ${response.status}`);
  return payload;
}

// A number or null. 'nan', null, undefined and anything non-finite are all
// "not computed"; none of them become zero.
function numOrNull(value) {
  if (value === null || value === undefined) return null;
  if (typeof value === 'number') return Number.isFinite(value) ? value : null;
  if (typeof value === 'boolean') return value ? 1 : 0;
  if (typeof value === 'string') {
    const parsed = Number.parseFloat(value);
    return Number.isFinite(parsed) ? parsed : null;
  }
  return null;
}

// The first spelling of a name that is actually present. Server modules are
// written alongside this one; accepting camelCase and snake_case costs a line
// and removes a whole class of "the field is there but named differently".
function pick(obj, ...names) {
  if (!obj) return undefined;
  for (const name of names) {
    if (obj[name] !== undefined) return obj[name];
    const snake = name.replace(/^k/, '').replace(/([a-z0-9])([A-Z])/g, '$1_$2').toLowerCase();
    if (obj[snake] !== undefined) return obj[snake];
  }
  return undefined;
}

// Thresholds may arrive as a bare number or as {value, unit, meaning}.
function scalarOf(entry) {
  if (entry === null || entry === undefined) return null;
  if (typeof entry === 'object') return numOrNull(pick(entry, 'value'));
  return numOrNull(entry);
}

function fmt(value, digits = 2) {
  return value === null || value === undefined ? '—' : value.toFixed(digits);
}

// Kortex reports joint angles on [0, 360) while limits and errors are signed,
// so the same physical angle can arrive a full turn from where it is expected.
// Everything angular in this file goes through the short way round first.
function wrap180(deg) {
  if (deg === null) return null;
  let x = (deg + 180) % 360;
  if (x < 0) x += 360;
  return x - 180;
}

/* ------------------------------------------------------------------ state */

const state = {
  arm: 'right',
  view: 'run',
  viewChosen: false,      // once the operator picks a tab, stop moving it
  local: null,            // may the server accept a start from this browser
  replay: false,          // a recorded run is being played back, not a robot
  stopping: false,        // a stop was requested and the loop has not exited
  status: null,
  config: null,
  readoutsFrom: null,     // the config the error rows and joint bars were built from
  frame: null,            // newest telemetry payload for the selected arm
  frameAt: 0,             // performance.now() when it arrived
  frameServerAge: 0,      // the age the server reported at that moment
  connected: false,
  growing: false,
  heartbeatReason: '',
  otherWorst: null,
  streams: { primary: null, other: null },
  scene: null,
  banner: null,
  staleness: null,
  slots: {},
  errorRows: [],
  jointBars: [],
  activation: null,       // {row_time_s} of the last trajectory activation edge
  plan: null,
  goal: null,             // goal.yaml as parsed by the server, for the scene
  runs: [],
  selectedRun: null,
  buildPoll: null,
  logLines: [],
  lastNotable: '',      // the newest line that looks like an explanation
  startStates: [],      // runs a solve could plan from, newest first
  driveChosen: false,   // once set by hand, stop following goal.yaml
};

const STALE_AFTER_S = 2.0;   // the hard visual break, from the design document
const STATUS_POLL_MS = 2000; // how often the panel re-checks what is actually true

/* ------------------------------------------------------------------- boot */

function init() {
  wireSafetyBar();
  wireTabs();
  wireConfigPane();
  wireTargetsPane();
  wireSessionPane();

  buildRunView();

  setInterval(tick, 100);
  // Re-ask the server what is true, whether or not the event stream is alive.
  // Without this the panel only learns about a session through the stream, so
  // a dropped stream freezes its idea of the world — and it went on saying
  // DRIVING BOTH ARMS over a screen that said nothing was running. The stream
  // is for speed; this is for being right.
  setInterval(refreshStatus, STATUS_POLL_MS);

  refreshStatus();
  refreshConfig();
  refreshGoal();
  refreshRuns();
  refreshPlan();
  refreshStartStates();
  refreshControllerLogOnce();
  openStreams();

  window.addEventListener('resize', () => call(state.scene, 'resize'));
}

/* -------------------------------------------------------------- safety bar */

// Switch which arm's numbers are shown. The scene keeps drawing both; only the
// bars, the error rows and the plan follow the selection, so everything tied
// to the previous arm is dropped rather than left to look current.
function setArm(arm) {
  state.arm = arm;
  $('arm-select').value = arm;
  state.frame = null;
  state.otherWorst = null;
  state.activation = null;
  $('other-arm-name').textContent = otherArm();
  openStreams();
  refreshPlan();
  refreshStartStates();
  drawObstacle();
  renderRun();
}

function wireSafetyBar() {
  const select = $('arm-select');
  select.value = state.arm;
  select.addEventListener('change', () => setArm(select.value));

  // Not gated on anything this page believes. If the request reaches the
  // server the session stops; if it does not, say which stop still works.
  $('stop-button').addEventListener('click', async () => {
    state.stopping = true;
    renderBanner();
    setNotice('start-status', 'stopping…', null);
    try {
      await postJSON('/api/session/stop', {});
      setNotice('start-status', 'stop sent: commanding has ended and the arm is back on high-level servoing.', null);
    } catch {
      const message = state.local === false
        ? 'the stop did not reach the workstation — you cannot stop from here, use the workstation'
        : 'the stop did not reach the server — paste the kill command under SESSION into a terminal';
      setNotice('start-status', message, 'is-stop');
      showView('session');
    }
    refreshStatus();
  });
}

function otherArm() {
  return state.arm === 'right' ? 'left' : 'right';
}

/* -------------------------------------------------------------------- tabs */

function wireTabs() {
  const tabs = [...document.querySelectorAll('.tab')];
  tabs.forEach((tab, index) => {
    tab.addEventListener('click', () => {
      state.viewChosen = true;
      showView(tab.dataset.view);
    });
    tab.addEventListener('keydown', (event) => {
      const step = event.key === 'ArrowRight' ? 1 : event.key === 'ArrowLeft' ? -1 : 0;
      if (!step) return;
      event.preventDefault();
      const next = tabs[(index + step + tabs.length) % tabs.length];
      next.focus();
      state.viewChosen = true;
      showView(next.dataset.view);
    });
  });

  document.querySelectorAll('[data-goto]').forEach((button) => {
    button.addEventListener('click', () => {
      state.viewChosen = true;
      showView(button.dataset.goto);
    });
  });
}

function showView(name) {
  state.view = name;
  document.querySelectorAll('.tab').forEach((tab) => {
    tab.setAttribute('aria-selected', String(tab.dataset.view === name));
  });
  document.querySelectorAll('.view').forEach((view) => {
    view.hidden = view.id !== `view-${name}`;
  });
  if (name === 'run') call(state.scene, 'resize');
  if (name === 'runs') refreshRuns();
}

/* ----------------------------------------------------------------- streams */

function openStreams() {
  closeStream('primary');
  closeStream('other');
  state.streams.primary = openStream(state.arm, true);
  state.streams.other = openStream(otherArm(), false);
}

function closeStream(slot) {
  const source = state.streams[slot];
  if (source) source.close();
  state.streams[slot] = null;
}

// The selected arm's stream drives the whole screen. The other arm's stream is
// read for one worst-case line only: fourteen joint bars are more than anyone
// reads under pressure, but an alarm on the arm you are not watching still has
// to be able to reach you.
function openStream(arm, primary) {
  const source = new EventSource(`/api/stream?arm=${encodeURIComponent(arm)}`);

  source.addEventListener('open', () => {
    if (primary) {
      state.connected = true;
      renderConnection();
    }
  });

  // The server decides whether this browser may start a session, from the
  // address the connection came from. The page asks rather than deciding for
  // itself, so the control that is offered and the rule that is enforced are
  // the same rule.
  source.addEventListener('hello', (event) => {
    const data = parseEvent(event);
    if (!data || !primary) return;
    state.local = Boolean(data.local);
    state.replay = Boolean(data.replay);
    renderSession();
    renderConnection();
  });

  source.addEventListener('error', () => {
    if (primary) {
      state.connected = false;
      renderConnection();
    }
    // EventSource retries on its own while the connection merely drops, but
    // once it reaches CLOSED — which is what a restarted server leaves behind
    // — it never tries again, and the page sits on "stream down" until it is
    // reloaded by hand. Reopen it, unhurriedly.
    if (source.readyState === EventSource.CLOSED) {
      const slot = primary ? 'primary' : 'other';
      if (state.streams[slot] !== source) return;   // already replaced
      state.streams[slot] = null;
      setTimeout(() => {
        if (state.streams[slot]) return;            // openStreams got there first
        state.streams[slot] = openStream(arm, primary);
      }, 2000);
    }
  });

  source.addEventListener('telemetry', (event) => {
    const data = parseEvent(event);
    if (!data) return;
    if (primary) acceptFrame(data);
    else acceptOtherArm(data);
  });

  source.addEventListener('heartbeat', (event) => {
    const data = parseEvent(event);
    if (!data || !primary) return;
    // The file stopped growing but the server is alive and saying so: the
    // panel lost the controller, not the other way round.
    state.growing = false;
    state.heartbeatReason = data.reason || 'the log stopped growing';
    state.frameServerAge = numOrNull(data.server_age_s) ?? state.frameServerAge;
    state.frameAt = performance.now();
    renderConnection();
  });

  source.addEventListener('log', (event) => {
    const data = parseEvent(event);
    if (!data || !primary) return;
    const line = String(data.line ?? '');
    state.logLines.push(line);
    if (state.logLines.length > 400) state.logLines.shift();
    if (isNotable(line)) state.lastNotable = line;
    renderControllerLog();
  });

  source.addEventListener('session', (event) => {
    const data = parseEvent(event);
    if (!data) return;
    state.status = { ...(state.status || {}), session: data };
    renderSession();
    renderBanner();
  });

  source.addEventListener('build', (event) => {
    const data = parseEvent(event);
    if (!data) return;
    // Build lines are not appended here: /api/build/status returns the whole
    // list, and taking both would print every line twice. The event is used
    // only to poll again immediately.
    pollBuild();
  });

  return source;
}

function parseEvent(event) {
  try {
    return JSON.parse(event.data);
  } catch {
    return null;
  }
}

function acceptFrame(data) {
  state.frame = data;
  state.frameAt = performance.now();
  state.frameServerAge = numOrNull(data.server_age_s) ?? 0;
  state.growing = data.growing !== false;
  state.heartbeatReason = '';
  state.connected = true;

  const row = data.row || {};
  // The activation edge is the only honest zero for progress: the controller
  // does not echo a trajectory identity back, so the panel times from the
  // cycle it saw the plan accepted.
  if (numOrNull(row.traj_activated)) state.activation = { time_s: numOrNull(row.time_s) };
  if (numOrNull(row.traj_complete)) state.activation = null;

  renderRun();
}

// The other arm gets one worst-case line of its own, but it is drawn in the
// same scene: inter-arm proximity is only visible when both arms are there.
function acceptOtherArm(data) {
  state.otherWorst = data;
  call(state.scene, 'setTelemetry', otherArm(), data.row || null);
  renderOtherArm();
}

/* --------------------------------------------------------------- staleness */

// Age by the browser's own clock: the server's age when it sent the frame,
// plus everything that has passed here since. A frozen server cannot hold
// this still, and a dead SSE connection makes it climb.
function ageSeconds() {
  if (!state.frameAt) return null;
  return state.frameServerAge + (performance.now() - state.frameAt) / 1000;
}

function tick() {
  const age = ageSeconds();
  const stale = age === null || age >= STALE_AFTER_S;

  const link = $('link');
  link.classList.toggle('is-live', !stale && state.connected);
  link.classList.toggle('is-stale', stale);
  $('link-age').textContent = age === null ? '—' : `${age.toFixed(1)} s`;
  $('link-text').textContent = age === null ? 'no telemetry'
    : stale ? 'not current' : 'link';

  // The component runs its own clock from the server's wall_ms stamp; it is
  // handed the stamp and the sentence, and counts up by itself.
  call(state.staleness, 'update',
       numOrNull(state.frame?.wall_ms), stalenessReason(stale));

  document.querySelectorAll('.readout-column, .banner-host, .scalar-strip')
    .forEach((node) => node.classList.toggle('is-stale', stale));
}

// The two failures look identical on a bar and are not the same problem, so
// they are named differently wherever they are said.
function stalenessReason(stale) {
  if (!state.connected) {
    return state.local === false
      ? 'I lost the panel — you cannot stop from here, use the workstation'
      : 'I lost the panel: the event stream to this page is down';
  }
  if (!state.growing) {
    return `the panel lost the controller: ${state.heartbeatReason || 'the log stopped growing'}`;
  }
  if (stale) return 'the newest row is older than 2 s';
  return '';
}

/* ------------------------------------------------- what the controller says */

// The event stream only carries lines written after this page connected, so a
// panel opened after a session already failed would show nothing while the
// file holds the answer. Load the tail once at boot.
async function refreshControllerLogOnce() {
  try {
    const data = await getJSON('/api/controller-log?n=60');
    if (!data.lines || !data.lines.length) return;
    state.logLines = data.lines.map(String);
    for (const line of state.logLines) {
      if (isNotable(line)) state.lastNotable = line;
    }
    const source = $('session-log-source');
    if (source && data.path) source.textContent = data.path;
    renderControllerLog();
    renderBanner();
  } catch {
    // A missing log is normal before the first session; say nothing.
  }
}

// Lines worth pulling out of the stream. Every PrintStopReport sentence opens
// with "loop stopped" (Safety.cpp), and the Kortex transport failures and the
// startup refusals announce themselves with one of the others. Keeping this
// list next to its use means a new message shows up in the log pane even
// before anyone teaches this function to notice it.
const NOTABLE = ['loop stopped', 'Error:', 'error:', 'internal error',
                 'WARNING', 'takeover hold failed', 'input rejected',
                 'rejected', 'STALE:'];

function isNotable(line) {
  return NOTABLE.some((marker) => line.includes(marker));
}

// The controller's own words, on the run screen and on the session pane. This
// exists because the panel once held a SetServoingMode timeout in memory and
// showed nothing, and the only way to find it was to read the log by hand.
function renderControllerLog() {
  const text = state.logLines.length
    ? state.logLines.join('\n')
    : 'Nothing from the controller yet.';
  for (const id of ['run-log', 'session-log']) {
    const node = $(id);
    if (!node) continue;
    const atBottom = node.scrollHeight - node.scrollTop - node.clientHeight < 24;
    node.textContent = text;
    // Follow the tail, unless the reader has scrolled up to look at something.
    if (atBottom) node.scrollTop = node.scrollHeight;
  }
  const notable = $('run-log-card');
  if (notable) notable.classList.toggle('has-notice', Boolean(state.lastNotable));
}

function renderConnection() {
  const node = $('connection');
  const lost = !state.connected;
  node.classList.toggle('is-lost', lost || !state.growing);
  node.textContent = lost
    ? (state.local === false ? 'no link to the workstation' : 'stream down')
    : state.growing ? 'streaming' : 'controller quiet';
  $('replay-mark').hidden = !state.replay;
}

/* -------------------------------------------------------------- the run view */

// The error rows, each named by the consequence of its threshold rather than
// by a round number. Where nothing enforces a threshold the row says so:
// no threshold stops the arm on orientation error, and a bar that implied one
// would be a lie the operator later relies on.
function errorRowSpecs() {
  const t = (name) => scalarOf(pick(state.config?.thresholds || {}, name));
  const arrivalM = t('kArrivalToleranceM');
  const arrivalRad = t('kArrivalOrientationToleranceRad');
  const followDeg = t('kFollowingErrorLimitDeg');
  const trajStopDeg = t('kTrajFollowingErrorStopDeg');

  return [
    {
      key: 'position',
      label: 'Position, world frame',
      unit: 'mm',
      decimals: 1,
      chars: 7,
      read: (row) => scale(positionErrorM(row), 1000),
      worst: (worst) => scale(numOrNull(pick(worst, 'pos_err_m')), 1000),
      bands: band([[arrivalM, 'arrival', 1000]]),
    },
    {
      key: 'orientation',
      label: 'Orientation',
      unit: 'deg',
      decimals: 2,
      chars: 6,
      // The end of scale is set here rather than derived from the one band,
      // because arrival is a thousandth of a radian and a bar scaled to it
      // would peg on every normal cycle.
      full: 10,
      read: (row) => scale(numOrNull(row.rot_error_rad), 180 / Math.PI),
      worst: (worst) => scale(numOrNull(pick(worst, 'rot_error_rad')), 180 / Math.PI),
      bands: band([[arrivalRad, 'arrival', 180 / Math.PI]]),
      note: 'nothing stops the arm on orientation error',
    },
    {
      key: 'following',
      label: 'Following, command vs measured',
      unit: 'deg',
      decimals: 2,
      chars: 6,
      read: (row) => followingErrorDeg(row),
      worst: (worst) => numOrNull(pick(worst, 'follow_err_deg')),
      bands: band([[followDeg, 'stop', 1]]),
    },
    {
      key: 'joint_reference',
      label: 'Plan deviation',
      unit: 'deg',
      decimals: 2,
      chars: 6,
      read: (row) => numOrNull(row.joint_follow_error_deg),
      worst: (worst) => numOrNull(pick(worst, 'joint_follow_error_deg')),
      bands: band([[trajStopDeg, 'stop', 1]]),
    },
    {
      key: 'sigma_min',
      label: 'Sigma min',
      unit: '',
      decimals: 3,
      chars: 6,
      // Small is bad here: the severity test flips, the drawing does not.
      worseWhen: 'below',
      read: (row) => numOrNull(row.sigma_min),
      worst: (worst) => numOrNull(pick(worst, 'sigma_min')),
      bands: [],
      note: 'how close the arm is to a direction it cannot move in',
    },
  ];
}

function band(entries) {
  return entries
    .filter(([value]) => value !== null && value !== undefined)
    .map(([value, name, factor]) => ({ value: value * factor, name }));
}

function scale(value, factor) {
  return value === null ? null : value * factor;
}

function positionErrorM(row) {
  const parts = ['x', 'y', 'z'].map((axis) => {
    const desired = numOrNull(row[`pd_${axis}`]);
    const measured = numOrNull(row[`p_${axis}`]);
    return desired === null || measured === null ? null : desired - measured;
  });
  if (parts.some((part) => part === null)) return null;
  return Math.hypot(...parts);
}

// What the following-error guard actually tests: command against measurement,
// worst joint, taken the short way round.
function followingErrorDeg(row) {
  let worst = null;
  for (let j = 1; j <= 7; j += 1) {
    const commanded = numOrNull(row[`cmd_j${j}`]);
    const measured = numOrNull(row[`meas_j${j}`]);
    if (commanded === null || measured === null) continue;
    const error = Math.abs(wrap180(commanded - measured));
    if (worst === null || error > worst) worst = error;
  }
  return worst;
}

// Every band comes from the compiled config, so the bars move when a knob
// moves and the panel can never disagree with the binary. config_file.py
// already reports the continuous joints as unbounded with null everywhere, so
// a bar is only drawn where a real range exists — J1, J3, J5 and J7 get angle
// and velocity and no range at all.
function jointLimits() {
  const joints = state.config?.joints || {};
  const array = (name) => (Array.isArray(joints[name]) ? joints[name].map(numOrNull) : []);
  const bounded = Array.isArray(joints.bounded_mask) ? joints.bounded_mask : [];
  const lower = array('lower_deg');
  const upper = array('upper_deg');
  const warn = array('warn_deg');
  const error = array('error_deg');
  const software = array('software_limit_deg');
  const velocity = array('velocity_limit_deg_s');

  const threshold = (name) => scalarOf(pick(state.config?.thresholds || {}, name));

  return Array.from({ length: 7 }, (unused, index) => ({
    index: index + 1,
    limits: {
      bounded: Boolean(bounded[index]),
      lower_deg: lower[index] ?? null,
      upper_deg: upper[index] ?? null,
      warn_deg: warn[index] ?? null,
      error_deg: error[index] ?? null,
      software_limit_deg: software[index] ?? null,
      velocity_limit_deg_s: velocity[index] ?? null,
      // Passed so the bar colours from the compiled threshold rather than
      // inventing its own.
      follow_error_stop_deg: threshold('kFollowingErrorLimitDeg'),
    },
  }));
}

function buildRunView() {
  // The scene reads its colours from the stylesheet itself, so the palette is
  // defined once, in panel.css, and cannot drift between the drawing and the
  // page around it.
  state.scene = createScene($('scene'), {
    frameLabel: 'mount frame · world = mount · no Vicon',
  });
  call(state.scene, 'setSelectedArm', state.arm);

  state.banner = mount($('banner-host'), createBanner());
  state.staleness = mount($('staleness-host'),
    createStaleness({ staleAfterMs: STALE_AFTER_S * 1000 }));

  // Widths are reserved for the widest value each quantity can reach, not for
  // the value expected, or the strip twitches on the day it matters.
  const strip = $('scalar-strip');
  for (const spec of [
    { key: 'time_s', label: 'run time', unit: 's', chars: 8, decimals: 1 },
    { key: 'cycle', label: 'cycle', unit: '', chars: 9, decimals: 0 },
    { key: 'dt_s', label: 'loop dt', unit: 'ms', chars: 6, decimals: 2 },
    { key: 'null_leak_mps', label: 'null leak', unit: 'mm/s', chars: 7, decimals: 1 },
  ]) {
    state.slots[spec.key] = mount(strip, createSlot(spec));
  }

  $('other-arm-name').textContent = otherArm();
  loadDhTables();
}

// Forward kinematics runs in the browser from the build-generated DH tables,
// which is why the panel can draw an arm at all. If a table is stale against
// the URDF the server says so and the scene is labelled, never silently drawn
// from the wrong geometry.
async function loadDhTables() {
  for (const arm of ['right', 'left']) {
    try {
      const dh = await getJSON(`/api/dh?arm=${arm}`);
      call(state.scene, 'setDh', arm, dh);
      if (dh && dh.stale) {
        setNotice('clearance-note', dh.reason
          || `the ${arm} DH table is older than the URDF — rebuild planner_bridge before trusting the drawn arm`, 'is-warn');
      }
    } catch {
      /* No table: the scene draws what it can and says nothing it cannot. */
    }
  }
}

function renderRun() {
  const empty = !state.frame && !state.status?.session?.commanding;
  $('run-empty').hidden = !empty;
  document.querySelector('.run-grid').hidden = empty;

  const row = state.frame?.row || {};
  const worst = state.frame?.worst || {};

  renderBanner();
  renderErrorRows(row, worst);
  renderJointBars(row);
  renderScalars(row);
  renderOtherArm();

  // The scene keeps a setter per thing it draws rather than one update(),
  // because the plan and the DH table change once while the row changes
  // twenty times a second.
  call(state.scene, 'setSelectedArm', state.arm);
  call(state.scene, 'setTelemetry', state.arm, row);
  call(state.scene, 'setProgress', planProgress(row));
}

function renderErrorRows(row, worst) {
  const host = $('error-rows');
  const specs = errorRowSpecs();

  // The bands are compiled constants, so the rows are rebuilt when the
  // configuration arrives or changes and not on every frame. Without this the
  // rows built before /api/config answered would keep their empty bands for
  // the life of the page.
  if (state.readoutsFrom !== state.config || state.errorRows.length !== specs.length) {
    host.textContent = '';
    state.errorRows = specs.map((spec) => mount(host, createErrorRow(spec)));
    $('joint-bars').textContent = '';
    state.jointBars = [];
    state.readoutsFrom = state.config;
  }

  specs.forEach((spec, index) => {
    call(state.errorRows[index], 'update', spec.read(row), spec.worst(worst));
  });
}

function renderJointBars(row) {
  const host = $('joint-bars');
  const joints = jointLimits();

  if (state.jointBars.length !== joints.length) {
    host.textContent = '';
    state.jointBars = joints.map((joint) => mount(host, createJointBar(joint)));
  }

  joints.forEach((joint, index) => {
    const j = index + 1;
    const measured = numOrNull(row[`meas_j${j}`]);
    const commanded = numOrNull(row[`cmd_j${j}`]);
    call(state.jointBars[index], 'update', {
      // Raw, not wrapped: a bounded joint's bar wraps it against limits that
      // are symmetric about zero, and a continuous joint has no such
      // reference, so wrapping there would invent one.
      angle_deg: measured,
      velocity_deg_s: numOrNull(row[`vel_j${j}`]),
      follow_err_deg: measured === null || commanded === null
        ? null : wrap180(commanded - measured),
      margin_deg: jointMarginDeg(joint.limits, measured),
      fault: numOrNull(row[`fault_j${j}`]),
    });
  });
}

// How far this joint is from its own software limit. The log carries only the
// worst joint's margin, so a per-joint number has to be computed here — from
// the same symmetric magnitude Config.h uses, which is why it is |angle| that
// is compared.
function jointMarginDeg(limits, measuredDeg) {
  const limit = limits.software_limit_deg;
  if (!limits.bounded || limit === null || measuredDeg === null) return null;
  return limit - Math.abs(wrap180(measuredDeg));
}

function renderScalars(row) {
  const values = {
    time_s: numOrNull(row.time_s),
    cycle: numOrNull(row.cycle),
    dt_s: scale(numOrNull(row.dt_s), 1000),
    null_leak_mps: scale(numOrNull(row.null_leak_mps), 1000),
  };
  for (const [key, value] of Object.entries(values)) {
    call(state.slots[key], 'set', value);
  }
}

function renderOtherArm() {
  const node = $('other-arm-worst');
  const frame = state.otherWorst;
  if (!frame) {
    node.textContent = 'no telemetry from the other arm';
    node.className = 'worst-line';
    return;
  }
  const row = frame.row || {};
  const following = followingErrorDeg(row);
  const fault = numOrNull(row.base_fault) || [1, 2, 3, 4, 5, 6, 7]
    .some((j) => numOrNull(row[`fault_j${j}`]));

  node.textContent = fault
    ? `${otherArm()} arm reports a fault`
    : `${otherArm()} arm · worst following ${fmt(following, 2)} deg`;
  node.className = `worst-line${fault ? ' is-stop' : (following !== null && following > 2 ? ' is-warn' : '')}`;
}

/* ------------------------------------------------------------------ banner */

// Derived, and marked as such on screen: no state machine in the controller
// publishes these names. The panel infers them from the session it knows about
// and from the trajectory edges in the log.
function derivedState(row) {
  const session = state.status?.session || {};
  const faulted = numOrNull(row.base_fault)
    || [1, 2, 3, 4, 5, 6, 7].some((j) => numOrNull(row[`fault_j${j}`]));

  if (faulted) return 'FAULT';
  // STOPPING is brief by construction: there is no ramp, so it covers only the
  // interval between the request and the loop exiting.
  if (state.stopping && session.commanding) return 'STOPPING';
  // Nothing is being commanded, so nothing downstream of this is true — even
  // if a frame from the run that just ended is still in hand, and even if the
  // session script is still sitting there. A replay has no session by design
  // and is read from its frames instead.
  if (!session.commanding && !state.replay) return 'IDLE';
  if (!session.commanding && !state.frame) return 'IDLE';
  if (numOrNull(row.traj_rejected)) return 'READY';
  if (state.activation) return 'MOVING';
  if (numOrNull(row.traj_complete)) return 'HOLDING TARGET';
  if (session.commanding && !state.frame) return 'PLANNING';
  return 'READY';
}

function planProgress(row) {
  const duration = numOrNull(pick(state.plan || {}, 'duration_s'));
  const now = numOrNull(row.time_s);
  if (!state.activation || duration === null || now === null || duration <= 0) return null;
  const elapsed = now - (state.activation.time_s ?? 0);
  return Math.max(0, Math.min(1, elapsed / duration));
}

function renderBanner() {
  const row = state.frame?.row || {};
  const progress = planProgress(row);
  const duration = numOrNull(pick(state.plan || {}, 'duration_s'));

  // Rejection is one of the few things worth spelling out in full: the number
  // and the guard it failed are both on the wire, so the message can be exact.
  let detail = '';
  if (numOrNull(row.traj_rejected)) {
    const started = numOrNull(row.traj_start_error_deg);
    const allowed = scalarOf(pick(state.config?.thresholds || {}, 'kTrajStartToleranceDeg'));
    detail = `the plan started ${fmt(started, 1)} deg from the measured pose on its worst joint; the guard allows ${fmt(allowed, 1)}`;
  } else if (progress !== null && duration !== null) {
    detail = `${(progress * 100).toFixed(0)}% · ${fmt(duration * (1 - progress), 1)} s remaining`;
  } else if (state.status?.session?.commanding) {
    detail = 'no trajectory is running; the arm holds where it was left';
  }

  // A stop reason or a startup failure outranks anything above it. The
  // controller states these in sentences, so quote it rather than paraphrase:
  // "Error: timeout detected: BaseClient::SetServoingMode" is the whole answer
  // to why a session produced no motion, and it belongs where it is read.
  if (state.lastNotable && !numOrNull(row.traj_rejected)) {
    detail = state.lastNotable.replace(/^\[\w+\]\s*/, '');
  }

  // The task line names the plan, not the compiled reference source. The
  // preamble is the only authority for which controller is running, and it is
  // not on the telemetry stream — so the RUNS pane says it, where it is read
  // from the run's own preamble, and the live banner does not guess.
  call(state.banner, 'update', {
    state: derivedState(row),
    arm: state.arm,
    task: planLabel(),
    progress,
    remaining_s: progress !== null && duration !== null
      ? duration * (1 - progress) : null,
  });
  setNotice('run-message', detail,
            (numOrNull(row.traj_rejected) || state.lastNotable) ? 'is-warn' : null);
}

function planLabel() {
  const plan = state.plan;
  if (!plan || plan.error) return '';
  const points = numOrNull(plan.points);
  const duration = numOrNull(plan.duration_s);
  const parts = [];
  if (points !== null) parts.push(`${points} point plan`);
  if (duration !== null) parts.push(`${duration.toFixed(1)} s`);
  return parts.join(' · ');
}

/* ------------------------------------------------------------- config pane */

function wireConfigPane() {
  $('build-controller').addEventListener('click', () => startBuild('controller'));
  $('build-bridge').addEventListener('click', () => startBuild('bridge'));
}

async function refreshConfig() {
  try {
    state.config = await getJSON('/api/config');
  } catch (err) {
    setNotice('config-error', `could not read the configuration: ${err.message}`, 'is-stop');
    return;
  }
  renderKnobs();
  renderReadOnlyTable('threshold-table', state.config.thresholds, 'These are the numbers the controller enforces. Change one through its knob above, then rebuild.');
  renderReadOnlyTable('joint-limit-table', state.config.joints, 'Firmware and software boundaries. Only J2, J4 and J6 are bounded; the others turn continuously.');
  renderRun();
}

function renderKnobs() {
  const host = $('knob-table');
  host.textContent = '';
  const knobs = state.config?.knobs || {};

  for (const [name, entry] of Object.entries(knobs)) {
    const value = typeof entry === 'object' ? pick(entry, 'value') : entry;
    const doc = typeof entry === 'object' ? (pick(entry, 'doc', 'meaning') || '') : '';

    const label = document.createElement('label');
    label.className = 'knob-name';
    label.textContent = name;
    label.htmlFor = `knob-${name}`;

    const input = document.createElement('input');
    input.className = 'knob-input num';
    input.id = `knob-${name}`;
    input.type = 'text';
    input.value = value === null || value === undefined ? '' : String(value);
    input.dataset.committed = input.value;

    input.addEventListener('input', () => {
      input.classList.toggle('is-dirty', input.value !== input.dataset.committed);
      input.classList.remove('is-rejected');
    });
    input.addEventListener('change', () => commitKnob(name, input));
    input.addEventListener('keydown', (event) => {
      if (event.key === 'Enter') input.blur();
    });

    const note = document.createElement('span');
    note.className = 'knob-doc';
    note.textContent = doc;

    host.append(label, input, note);
  }
}

async function commitKnob(name, input) {
  try {
    const result = await postJSON('/api/config/set', { name, value: input.value.trim() });
    if (result.error) {
      input.classList.add('is-rejected');
      setNotice('config-error', `${name} was not written: ${result.error}. Config.h is unchanged.`, 'is-stop');
      return;
    }
    input.value = String(result.value ?? input.value);
    input.dataset.committed = input.value;
    input.classList.remove('is-dirty', 'is-rejected');
    setNotice('config-error', `${name} written to Config.h. The running binary still holds the old value until you build.`, 'is-warn');
    refreshStatus();
  } catch (err) {
    input.classList.add('is-rejected');
    setNotice('config-error', `${name} was not written: ${err.message}`, 'is-stop');
  }
}

function renderReadOnlyTable(hostId, data, note) {
  const host = $(hostId);
  host.textContent = '';
  for (const [key, entry] of Object.entries(data || {})) {
    const keyNode = document.createElement('span');
    keyNode.className = 'kv-key';
    keyNode.textContent = key;
    const valueNode = document.createElement('span');
    valueNode.className = 'kv-value';
    valueNode.textContent = renderValue(entry);
    host.append(keyNode, valueNode);
  }
  if (note) {
    const noteNode = document.createElement('span');
    noteNode.className = 'kv-note';
    noteNode.textContent = note;
    host.append(noteNode);
  }
}

// A threshold arrives as {value, unit, consequence}: the consequence is the
// part that matters, because a band is named by what it does — arrival,
// replan advised, stop — and never by a round number.
function renderValue(entry) {
  if (entry === null || entry === undefined) return '—';
  if (Array.isArray(entry)) return entry.map(renderValue).join('  ');
  if (typeof entry === 'object') {
    if (entry.value === undefined) {
      return Object.entries(entry).map(([k, v]) => `${k} ${renderValue(v)}`).join(' · ');
    }
    const unit = entry.unit ? ` ${entry.unit}` : '';
    const meaning = entry.consequence || entry.doc || entry.meaning || '';
    const head = `${renderValue(entry.value)}${unit}`;
    return meaning ? `${head} — ${meaning}` : head;
  }
  return String(entry);
}

/* ------------------------------------------------------------------- build */

async function startBuild(target) {
  $('build-log').textContent = `building ${target}…`;
  setBuildButtons(true);
  try {
    const started = await postJSON('/api/build', { target });
    if (started.error) {
      $('build-log').textContent = started.error;
      setBuildButtons(false);
      return;
    }
  } catch (err) {
    $('build-log').textContent = `the build did not start: ${err.message}`;
    setBuildButtons(false);
    return;
  }
  if (state.buildPoll) clearInterval(state.buildPoll);
  state.buildPoll = setInterval(pollBuild, 500);
  pollBuild();
}

function setBuildButtons(disabled) {
  $('build-controller').disabled = disabled;
  $('build-bridge').disabled = disabled;
}

async function pollBuild() {
  let status;
  try {
    status = await getJSON('/api/build/status');
  } catch {
    return;
  }
  const pane = $('build-log');
  pane.textContent = (status.lines || []).join('\n');
  pane.scrollTop = pane.scrollHeight;

  if (!status.running) {
    if (state.buildPoll) clearInterval(state.buildPoll);
    state.buildPoll = null;
    setBuildButtons(false);
    if (status.ok === true) {
      pane.textContent += `\n\n${status.target || 'the target'} built. The binary now matches the source.`;
    } else if (status.ok === false) {
      pane.textContent += `\n\nthe build failed; the old binary is still in place.`;
    }
    refreshStatus();
  }
}

/* ------------------------------------------------------------ targets pane */

function wireTargetsPane() {
  $('goal-save').addEventListener('click', saveGoal);
  $('plan-solve').addEventListener('click', solvePlan);
  $('start-state-select').addEventListener('change', onStartStateChange);
}

// Which measured configuration a solve plans from. "newest" is what the bridge
// does on its own; the rest exist because the newest run is not always usable.
function onStartStateChange() {
  const choice = $('start-state-select').value;
  const typed = choice === 'typed';
  $('start-deg-row').hidden = !typed;

  const option = (state.startStates || []).find((o) => o.file === choice);
  if (option && option.final_deg && !$('start-deg-input').value) {
    $('start-deg-input').value = option.final_deg.map((v) => v.toFixed(2)).join(' ');
  }
  $('start-state-note').textContent = startStateNote(choice, option);
}

function startStateNote(choice, option) {
  if (choice === 'typed') return 'seven angles in degrees, in Kortex actuator order';
  if (choice === 'newest') {
    const newest = (state.startStates || [])[0];
    if (!newest) return 'no recorded run to plan from';
    return newest.usable
      ? `the bridge will read ${newest.name} (${newest.size_mb} MB)`
      : `${newest.name} has no data rows — pick another run or type the angles`;
  }
  if (!option) return '';
  return `${option.size_mb} MB, ending at the configuration shown when you switch to typed`;
}

// The list is rebuilt whenever the arm changes, because a right-arm plan can
// only start from a right-arm run.
async function refreshStartStates() {
  const select = $('start-state-select');
  try {
    const data = await getJSON(`/api/plan/start-states?arm=${state.arm}`);
    state.startStates = data.options || [];
  } catch {
    state.startStates = [];
  }
  select.textContent = '';
  const add = (value, label) => {
    const option = document.createElement('option');
    option.value = value;
    option.textContent = label;
    select.appendChild(option);
  };
  const newest = state.startStates[0];
  add('newest', newest
    ? `newest run — ${newest.name}${newest.usable ? '' : '  (no data rows)'}`
    : 'newest run — none recorded');
  for (const option of state.startStates.slice(1)) {
    if (!option.usable) continue;
    add(option.file, `${option.name}  (${option.size_mb} MB)`);
  }
  add('typed', 'type the joint angles');

  // If the newest run cannot seed a plan, do not leave it selected and let the
  // solve fail: choose the newest usable run and say why it moved.
  if (newest && !newest.usable) {
    const usable = state.startStates.find((o) => o.usable);
    select.value = usable ? usable.file : 'typed';
  }
  onStartStateChange();
}

async function refreshGoal() {
  try {
    const goal = await getJSON('/api/goal');
    $('goal-text').value = goal.text ?? '';
    state.goal = goal.parsed || null;
    drawObstacle();
  } catch (err) {
    setNotice('goal-status', `could not read goal.yaml: ${err.message}`, 'is-stop');
  }
}

async function saveGoal() {
  try {
    const result = await postJSON('/api/goal', { text: $('goal-text').value });
    if (result.error) {
      setNotice('goal-status', `not saved: ${result.error}. The file on disk is unchanged.`, 'is-stop');
      return;
    }
    setNotice('goal-status', 'goal.yaml saved. Solve to see what it produces.', null);
    refreshGoal();
  } catch (err) {
    setNotice('goal-status', `not saved: ${err.message}`, 'is-stop');
  }
}

// Turns the selector into the request body. Returning an error string rather
// than throwing keeps the refusal in the same place as every other one.
function startStateRequest() {
  const choice = $('start-state-select').value;
  if (choice === 'newest' || !choice) return {};
  if (choice === 'typed') {
    const parts = $('start-deg-input').value.trim().split(/[\s,]+/).filter(Boolean);
    if (parts.length !== 7) {
      return { error: `type seven joint angles in degrees — got ${parts.length}` };
    }
    const angles = parts.map(Number);
    if (angles.some((v) => !Number.isFinite(v))) {
      return { error: 'the joint angles must all be numbers' };
    }
    return { start_deg: angles };
  }
  const option = (state.startStates || []).find((o) => o.file === choice);
  if (!option || !option.final_deg) {
    return { error: 'that run has no usable final configuration' };
  }
  return { start_deg: option.final_deg };
}

// Why a solve produced nothing, in the most useful words available.
//
// Three different things can fail and they are not equally informative. If the
// panel refused before running the bridge, its own message names the file and
// the way out. If the bridge RAN and rejected the plan, its own "error:" line
// is the answer — "joint 2 at 224.3 deg exceeds ±126.9" — and the parse result
// only reports the symptom, that no TRAJ_BEGIN came back. Preferring the parse
// error there would tell you the bridge emitted nothing while hiding why.
function solveFailureReason(result) {
  const plan = result.plan || {};
  if (plan.returncode === null || plan.returncode === undefined) {
    return plan.error || 'the solve did not run.';
  }
  const stderrError = lastErrorLine(result.stderr || '');
  if (stderrError) return stderrError;
  return plan.error || 'the solve did not produce a plan; the solver output below says why.';
}

function lastErrorLine(text) {
  const lines = text.split('\n').filter((line) => /^\s*error:/i.test(line));
  return lines.length ? lines[lines.length - 1].trim() : '';
}

async function solvePlan() {
  const chosen = startStateRequest();
  if (chosen.error) {
    setNotice('goal-status', chosen.error, 'is-warn');
    return;
  }
  setNotice('goal-status', 'solving…', null);
  $('plan-arm-note').textContent = `${state.arm} arm`;
  try {
    const result = await postJSON('/api/plan/solve', { arm: state.arm, ...chosen });
    $('plan-stderr').textContent = result.stderr || '(the solver printed nothing)';
    if (!result.ok) {
      setNotice('goal-status', solveFailureReason(result), 'is-warn');
      refreshStartStates();
      return;
    }
    state.plan = result.plan || null;
    renderPlanSummary();
    drawPlan();
    setNotice('goal-status', 'solved. The plan is drawn in the scene; nothing was sent to an arm.', null);
  } catch (err) {
    setNotice('goal-status', `the solve failed: ${err.message}`, 'is-stop');
  }
}

// The last solved plan, so the run screen can draw where the arm was going
// even after the panel has been restarted mid-session.
async function refreshPlan() {
  try {
    const plan = await getJSON(`/api/plan?arm=${state.arm}`);
    state.plan = plan && !plan.error ? plan : null;
  } catch {
    state.plan = null;
  }
  renderPlanSummary();
  drawPlan();
}

// The scene draws the plan as a polyline through the solved joint states, so
// it is handed the rows themselves rather than the summary around them.
function drawPlan() {
  call(state.scene, 'setPlan', state.arm,
       state.plan && Array.isArray(state.plan.rows) ? state.plan.rows : null);
}

function renderPlanSummary() {
  const empty = !state.plan;
  $('plan-empty').hidden = !empty;
  renderReadOnlyTable('plan-summary', empty ? {} : summaryOf(state.plan), empty ? '' :
    'Clearance is a property of this plan, measured when it was solved. Nothing measures clearance while the arm moves.');
}

// Plans carry their whole trajectory; the summary shows the scalars and says
// how large the rest is rather than printing thousands of joint angles.
function summaryOf(plan) {
  const out = {};
  for (const [key, value] of Object.entries(plan)) {
    if (key === 'rows') { out.rows = `${value.length} sampled points`; continue; }
    out[key] = Array.isArray(value) && value.length > 8
      ? `${value.length} values` : value;
  }
  return out;
}

// goal.yaml supports one optional axis-aligned box per arm, and it is
// commented out today. Only a box the planner actually has is drawn: an
// exclusion zone nothing avoids would be worse than drawing nothing.
function drawObstacle() {
  const block = state.goal?.arms?.[state.arm];
  const box = block && block.box;
  const usable = box && (box.centre || box.center) && (box.halfExtent || box.half_extent);
  call(state.scene, 'setObstacle',
       usable ? { ...box, frame: block.frame || 'base', arm: state.arm } : null);
}

/* --------------------------------------------------------------- runs pane */

async function refreshRuns() {
  try {
    state.runs = await getJSON('/api/runs');
  } catch {
    return;
  }
  const host = $('runs-list');
  host.textContent = '';
  $('runs-empty').hidden = state.runs.length > 0;
  $('runs-count').textContent = state.runs.length ? `${state.runs.length} recorded` : '';

  for (const run of state.runs) {
    const item = document.createElement('button');
    item.type = 'button';
    item.className = 'run-item';
    item.setAttribute('aria-current', String(run.file === state.selectedRun));

    const file = document.createElement('span');
    file.className = 'run-file';
    file.textContent = run.file;
    const size = document.createElement('span');
    size.className = 'num';
    size.textContent = `${Number(run.mb ?? 0).toFixed(1)} MB`;
    const when = document.createElement('span');
    when.className = 'num';
    when.textContent = run.mtime_iso ? run.mtime_iso.replace('T', ' ').slice(5, 16)
                                     : formatTime(run.mtime);

    item.append(file, size, when);
    item.addEventListener('click', () => openRun(run.file));
    host.append(item);
  }
}

function formatTime(mtime) {
  const seconds = numOrNull(mtime);
  if (seconds === null) return '—';
  const date = new Date(seconds * 1000);
  return date.toLocaleString(undefined, { month: 'short', day: '2-digit', hour: '2-digit', minute: '2-digit' });
}

async function openRun(file) {
  state.selectedRun = file;
  $('run-name').textContent = file;
  $('run-hint').textContent = 'reading…';
  try {
    const summary = await getJSON(`/api/run?f=${encodeURIComponent(file)}`);
    renderRunSummary(summary);
    $('run-hint').textContent = summary.error ? summary.error : '';
  } catch (err) {
    $('run-hint').textContent = `could not read that run: ${err.message}`;
  }
  refreshRuns();
}

// The preamble is the run's own record of what it was compiled with, so it is
// shown verbatim: it is the only place that says which controller actually
// ran, and Config.h may have moved on since.
function renderRunSummary(summary) {
  renderReadOnlyTable('run-summary', {
    arm: summary.arm ?? '—',
    'reference source': summary.reference_source ?? '—',
    'log format': summary.log_format ?? '—',
    rows: summary.rows ?? '—',
    columns: summary.columns ?? '—',
    'duration (s)': summary.duration_s === null || summary.duration_s === undefined
      ? '—' : Number(summary.duration_s).toFixed(1),
    session: summary.session ?? '—',
  }, '');

  renderReadOnlyTable('run-worst', summary.worst || {},
    'Extremes over the whole run. Judge them against the thresholds in this run\'s own preamble, not against today\'s Config.h.');
  renderReadOnlyTable('run-stop', summary.stop || {}, '');

  const edges = summary.trajectory_edges || [];
  const edgeRows = {};
  edges.forEach((edge, index) => {
    const at = edge.time_s === null || edge.time_s === undefined
      ? '—' : `${Number(edge.time_s).toFixed(2)} s`;
    const extra = edge.start_error_deg === undefined || edge.start_error_deg === null
      ? '' : ` · started ${Number(edge.start_error_deg).toFixed(1)} deg away on its worst joint`;
    edgeRows[`${index + 1}. ${edge.name ?? 'edge'}`] = `${at}${extra}`;
  });
  renderReadOnlyTable('run-edges', edgeRows,
    edges.length ? (summary.trajectory_edges_truncated ? 'more edges than are listed here' : '')
                 : 'no trajectory was activated, rejected or completed in this run');

  $('run-preamble').textContent = (summary.preamble_lines || []).join('\n')
    || 'this run wrote no preamble';
}

/* ------------------------------------------------------------ session pane */

function wireSessionPane() {
  $('start-button').addEventListener('click', startSession);
  $('copy-kill').addEventListener('click', async () => {
    const text = $('kill-command').textContent;
    try {
      await navigator.clipboard.writeText(text);
      $('copy-kill').textContent = 'COPIED';
      setTimeout(() => { $('copy-kill').textContent = 'COPY'; }, 1500);
    } catch {
      // Clipboard access can be refused; the text is selectable either way.
      window.getSelection()?.selectAllChildren($('kill-command'));
    }
  });
  $('drive-select').addEventListener('change', () => {
    state.driveChosen = true;   // stop following goal.yaml once it is set by hand
    renderDriveNote();
  });
  $('copy-diagnostics-session').addEventListener('click',
    () => copyDiagnostics('copy-diagnostics-session'));
  $('copy-diagnostics').addEventListener('click',
    () => copyDiagnostics('copy-diagnostics'));
  $('show-diagnostics').addEventListener('click', async () => {
    const area = $('diagnostics-text');
    if (!area.hidden) { area.hidden = true; return; }
    area.value = await diagnosticsText();
    area.hidden = false;
  });
}

/* -------------------------------------------------------------- diagnostics */

// The server's report plus what only the browser knows. The two halves matter:
// the server can say the binary is stale, but only the page can say the event
// stream died or that the numbers on screen are eleven seconds old.
async function diagnosticsText() {
  let serverPart;
  try {
    const response = await fetch('/api/diagnose', { cache: 'no-store' });
    serverPart = await response.text();
  } catch (err) {
    serverPart = `(could not reach /api/diagnose from this page: ${err.message})\n`;
  }
  const age = ageSeconds();
  const browserPart = [
    '',
    'BROWSER',
    '-------',
    `  page          ${location.href}`,
    `  may start     ${state.local === false ? 'no — this page is remote' : 'yes'}`,
    `  event stream  ${state.connected ? 'connected' : 'DOWN'}`,
    `  newest frame  ${age === null ? 'none received' : age.toFixed(1) + ' s old'}`,
    `  growing       ${state.growing ? 'yes' : 'no — the log stopped growing'}`,
    `  view          ${state.view}, arm ${state.arm}${state.replay ? ', replay' : ''}`,
    `  last notable  ${state.lastNotable || '(none)'}`,
    '',
  ].join('\n');
  return serverPart + browserPart;
}

// navigator.clipboard needs a secure context, and this panel is served over
// plain http to another machine, so it is simply absent there. Falling back to
// selecting the text means the button still does something useful rather than
// failing silently.
async function copyDiagnostics(buttonId) {
  const button = $(buttonId);
  const original = button.textContent;
  button.textContent = 'GATHERING…';
  const text = await diagnosticsText();
  try {
    if (!navigator.clipboard) throw new Error('no clipboard in this context');
    await navigator.clipboard.writeText(text);
    button.textContent = 'COPIED';
    setNotice('diagnostics-status', 'copied — paste it wherever you are asking', null);
  } catch {
    const area = $('diagnostics-text');
    area.value = text;
    area.hidden = false;
    area.focus();
    area.select();
    button.textContent = 'SELECTED';
    setNotice('diagnostics-status',
      'this page is not a secure context, so the clipboard is unavailable — '
      + 'the text is selected below, press Cmd-C or Ctrl-C', 'is-warn');
  }
  setTimeout(() => { button.textContent = original; }, 2000);
}

async function refreshStatus() {
  try {
    state.status = await getJSON('/api/status');
    state.statusAt = Date.now();
    // The server answered, so any "it did not answer" left on screen is now
    // false. A stale failure message is its own small lie.
    if (state.serverSilent) {
      state.serverSilent = false;
      setNotice('start-status', '', null);
    }
  } catch (err) {
    state.serverSilent = true;
    setNotice('start-status', `the panel server did not answer: ${err.message}`, 'is-stop');
    // Do not clear the last known status: "I could not ask" is not the same
    // as "nothing is running", and guessing the second would hide a live arm.
    // renderSession marks the claim unverified from statusAt instead.
    renderSession();
    return;
  }

  // Until the stream's hello event answers, fall back to the address bar. The
  // server refuses a remote start regardless, so being wrong here only ever
  // shows a control that would be refused, never one that would be honoured.
  if (state.local === null) state.local = isLoopbackHost();
  if (state.status?.replay !== undefined) state.replay = Boolean(state.status.replay);
  if (!state.status?.session?.running) state.stopping = false;

  // The drive selector starts where goal.yaml's session_arms points, because
  // that is the key run_session.sh reads when no --arm is given, and having
  // the panel disagree with the file you just edited would be its own trap.
  // Only before the operator has touched it, and only once.
  if (!state.driveChosen) {
    const running = state.status?.session?.arm;
    const preferred = running || state.status?.goal_arms;
    if (preferred && ['right', 'left', 'both'].includes(preferred)) {
      $('drive-select').value = preferred;
    }
    renderDriveNote();
  }

  renderSession();
  renderFreshness();
  renderConnection();
  // renderRun, not renderBanner: the banner, the empty state and the driving
  // mark all read this same status, and rendering only some of them is how
  // they came to contradict each other on screen.
  renderRun();

  // The first view follows the machine: watch the run when one is live,
  // otherwise start where a session is started.
  if (!state.viewChosen) {
    showView(state.status?.session?.running ? 'run' : 'session');
  }
}

function isLoopbackHost() {
  return ['127.0.0.1', 'localhost', '::1', '[::1]'].includes(location.hostname);
}

// build.freshness() reports one flag for the whole system and a sentence for
// each thing that is out of date. The panel shows the sentences: refusing to
// start without saying why is what made this pane necessary.
function renderFreshness() {
  const freshness = state.status?.freshness;
  const stateNode = $('freshness-state');
  const detail = $('freshness-detail');

  if (!freshness) {
    stateNode.textContent = 'UNKNOWN';
    stateNode.classList.remove('is-stale');
    detail.textContent = 'the panel could not compare the binaries against their sources';
    return;
  }

  const stale = Boolean(freshness.stale);
  stateNode.textContent = stale ? 'STALE' : 'UP TO DATE';
  stateNode.classList.toggle('is-stale', stale);
  detail.textContent = stale
    ? `${(freshness.reasons || []).join('; ')} — build before starting, or the session refuses at the freshness gate`
    : 'every binary was built after the newest source change, and the DH tables match the URDF';
}

function renderSession() {
  const session = state.status?.session || {};
  const running = Boolean(session.running);
  const pipes = state.status?.pipes || {};

  // Which arms are actually being driven, in the safety bar, where it belongs:
  // reading the right arm's bars while both are moving is normal, and the bar
  // must not let that be mistaken for only one arm running.
  // DRIVING must mean an arm is actually being commanded. run_session.sh
  // outlives its controller — it waits at `read` for the Enter that ends the
  // session — so keying this on the session existing showed DRIVING BOTH ARMS
  // over a workstation whose controller had already stopped.
  const commanding = Boolean(session.commanding);
  const mark = $('driving-mark');
  mark.hidden = !commanding;
  if (commanding) {
    const both = session.arm === 'both';
    mark.textContent = both
      ? 'DRIVING BOTH ARMS' : `DRIVING ${String(session.arm || '').toUpperCase()}`;
    mark.classList.toggle('is-both', both);
    // This claim is only as good as the last answer from the server. Say so
    // rather than assert it, because "both arms are being driven" is exactly
    // the sentence that must not be stale.
    const stale = Date.now() - (state.statusAt || 0) > 3 * STATUS_POLL_MS;
    mark.classList.toggle('is-unverified', stale);
    mark.title = stale
      ? 'The panel has not been able to confirm this with the server recently.'
      : 'Which arms this session is commanding.';
  }

  renderReadOnlyTable('session-state', {
    running: running ? 'yes' : 'no',
    commanding: running
      ? (commanding ? 'yes — a controller is driving the arm'
                    : 'no — the controller has exited')
      : 'no',
    arm: session.arm ?? '—',
    pid: session.pid ?? '—',
    started: session.started ?? '—',
    'session directory': session.session_dir ?? '—',
    'controller log': session.log_path ?? '—',
    'target pipe': Object.entries(pipes)
      .map(([arm, exists]) => `${arm} ${exists ? 'open' : 'absent'}`).join(' · ') || '—',
    'telemetry source': state.replay ? 'replaying a recorded run' : 'live',
  }, '');

  // The lingering-script case gets said first, because it is the one that
  // looks like a running session and is not one.
  $('session-attach').textContent = running && !commanding
    ? (session.note || 'The controller has exited; the session script is still waiting.')
      + ' Nothing is commanding an arm. Press STOP AND RELEASE to clear it.'
    : running && session.attached
      ? 'This session was already running when the panel started, so the panel re-attached to it through its state file. Stopping from here still reaches it.'
      : running
        ? 'This panel started the session. Closing the browser does not stop it — the arm keeps doing what it was told.'
        : '';
  $('session-attach').classList.toggle('is-warn', running && !commanding);

  // stop_command signals the process GROUP, which is what the stop button
  // does and what Ctrl-C in the session's terminal would have done.
  $('kill-command').textContent = session.stop_command
    || (session.pgid ? `kill -INT -${session.pgid}` : 'no session is running');

  // Start is offered only from the workstation. Remote stop is always safe;
  // remote start never is, so the control is absent rather than disabled with
  // an excuse.
  const startBlock = $('start-block');
  if (state.local === false) {
    startBlock.hidden = true;
    setNotice('start-status', 'You are viewing this panel from another machine. Starting a session requires being at the workstation, beside the physical e-stop. Stopping works from anywhere.', 'is-warn');
  } else {
    startBlock.hidden = false;
    $('start-button').disabled = running;
    $('start-prose').textContent = running
      ? 'A session is already running. Stop it before starting another.'
      : 'Starting commands the arm. Type GO to confirm you are at the workstation, the workspace is clear, and the physical e-stop is in reach.';
  }
}

// Which arms a session drives is a different question from whose numbers are
// on screen, so it has its own control. Selecting one does not change the
// other: with both arms running you still read one arm's bars at a time, and
// the scene draws both either way.
function driveArms() {
  return $('drive-select').value || 'right';
}

function renderDriveNote() {
  const choice = driveArms();
  const note = choice === 'both'
    // A real property of run_session.sh, not a warning for its own sake: one
    // controller process drives both arms with a single stop flag.
    ? 'one controller process, two threads, one stop: a fault on either arm '
      + 'stops both. Each arm reads its own block of goal.yaml.'
    : `only the ${choice} arm moves; the other is not commanded.`;
  setNotice('drive-note', note, null);
}

async function startSession() {
  const confirm = $('go-input').value.trim().toUpperCase();
  if (confirm !== 'GO') {
    setNotice('start-status', 'type GO to confirm. The gate is typed on purpose.', 'is-warn');
    return;
  }
  const arms = driveArms();
  setNotice('start-status', 'starting…', null);
  try {
    const result = await postJSON('/api/session/start', { arm: arms, confirm: 'GO' });
    if (result.error) {
      // A refusal carries its reasons — a stale binary lists which file is
      // newer than which. Showing them is the difference between "it refused"
      // and knowing what to build.
      const reasons = (result.reasons || []).join('; ');
      setNotice('start-status', reasons ? `${result.error}: ${reasons}` : result.error, 'is-stop');
      if (reasons) showView('config');
    } else {
      $('go-input').value = '';
      setNotice('start-status', arms === 'both'
        ? 'session started on both arms. The scene draws both; the bars follow '
          + 'the arm selected at the top.'
        : `session started on the ${arms} arm.`, null);
      // Looking at an arm this session is not driving would show a hold that
      // never changes, so follow the run when only one arm is moving.
      if (arms !== 'both' && state.arm !== arms) setArm(arms);
      state.viewChosen = true;
      showView('run');
    }
  } catch (err) {
    setNotice('start-status', `the session did not start: ${err.message}`, 'is-stop');
  }
  refreshStatus();
}

/* ---------------------------------------------------------------- notices */

function setNotice(id, message, tone) {
  const node = $(id);
  if (!node) return;
  node.textContent = message;
  node.classList.remove('is-warn', 'is-stop');
  if (tone) node.classList.add(tone);
}

init();
