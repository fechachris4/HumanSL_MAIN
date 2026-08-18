// readouts.js — the panel's numbers and bars.
//
// Five plain factory functions. Each builds a piece of DOM once and returns
// { el, update(...) }; nothing here is a class, and nothing here fetches,
// polls or knows about the server. The page hands each component the values
// it already has, so a component can be exercised entirely from a static
// test page with no controller running.
//
// Two rules from the design decide most of what follows.
//
// Fixed-width value slots. Every number lives in a slot whose width was
// reserved in advance, in tabular figures, so a digit changing from 9 to 10
// cannot push its neighbours sideways. The panel is then physically
// motionless while values change, and any movement on screen means something
// actually happened.
//
// Colour only for abnormal states. A healthy screen carries no red, no amber
// and no green: those three are reserved for a crossed threshold. This is
// why nothing below paints a permanent "danger zone" on a bar — a shaded
// region that is always on screen is colour spent on the normal case, and it
// is exactly what buries the one alarm that matters.
//
// panel.js composes these components and, having been written from the same
// design at the same time, names several options and one method differently:
// it says digits for decimals, value for a band's at, continuous for the
// negation of bounded, and calls set() where this file says update(). Both
// spellings are accepted at the boundary of each factory and nowhere else, so
// no component carries two shapes internally. That compatibility is worth
// converging on one contract and deleting; it is here so the panel works in
// the meantime rather than rendering nothing.
//
// The palette itself is not defined here. Every colour and face is read as
// var(--token, fallback) using the names panel.css publishes — --ground,
// --panel, --ink, --ink-dim, --rule, --ask, --warn, --stop, --label-font,
// --num-font — so the panel owns the palette and this file only names the
// roles it needs. --ask is the one blue, and it means "what was asked for"
// here exactly as it does in the 3D scene. The fallbacks exist so
// readouts.test.html looks right on its own.

const STYLE_ID = "readouts-styles";

// Band names arrive from the server named by consequence ("arrival",
// "replan advised", "stop", "plan rejected"). Severity is read from the
// name rather than from a separate field, because the name is the thing the
// operator reads and the two must never disagree. Arrival is deliberately
// not a severity: crossing the arrival tolerance on the way out is what
// normal motion looks like, and colouring it would make the healthy screen
// shout.
const STOP_NAME = /stop|reject|fault|abort/i;
const WARN_NAME = /replan|warn|advis|margin/i;

const CSS = `
.ro-slot {
  display: inline-block;
  text-align: right;
  white-space: pre;
  font-family: var(--num-font, ui-monospace, "SF Mono", Menlo, Consolas, monospace);
  font-variant-numeric: tabular-nums;
  font-feature-settings: "tnum" 1;
  letter-spacing: 0;
  color: var(--ink, #24221f);
}
.ro-slot-unit {
  padding-left: 0.35em;
  font-family: var(--label-font, system-ui, sans-serif);
  font-size: 0.82em;
  color: var(--ink-dim, #6f6a63);
}
.ro-slot--void .ro-slot-num { color: var(--ink-dim, #6f6a63); opacity: 0.55; }
.ro-slot--ghost .ro-slot-num { color: var(--ink-dim, #6f6a63); }
.ro-slot--warn .ro-slot-num { color: var(--warn, #a86a00); }
.ro-slot--stop .ro-slot-num { color: var(--stop, #9d2118); }

.ro-label, .ro-tag, .ro-note, .ro-cap {
  font-family: var(--label-font, system-ui, sans-serif);
  color: var(--ink-dim, #6f6a63);
}
.ro-cap {
  font-size: 0.7rem;
  text-transform: uppercase;
  letter-spacing: 0.08em;
}

.ro-row {
  /* The right padding is room for the over-range arrow, which sits just off
     the end of the track and would otherwise widen the page. */
  padding: 0.45rem 1rem 0.6rem 0;
  border-bottom: 1px solid var(--rule, #cdc8bf);
}
.ro-row-head {
  display: flex;
  align-items: baseline;
  gap: 0.75rem;
  /* A long label must push the numbers onto their own line rather than off
     the edge of the column: a value half out of view is worse than a value
     one line lower. The slots keep their widths either way. */
  flex-wrap: wrap;
}
.ro-row .ro-label {
  flex: 1 1 8rem;
  min-width: 0;
  font-size: 0.86rem;
  color: var(--ink, #24221f);
}
.ro-row-worst { flex: 0 0 auto; }
.ro-row-worst {
  display: inline-flex;
  align-items: baseline;
  gap: 0.3rem;
}

.ro-track {
  position: relative;
  height: 10px;
  margin: 0.35rem 0 0.15rem;
  background: var(--track, var(--rule, #cdc8bf));
  border: 1px solid var(--rule, #cdc8bf);
  overflow: visible;
}
.ro-fill {
  position: absolute;
  top: 0; bottom: 0; left: 0;
  width: 0;
  background: var(--ink, #24221f);
}
.ro-track--warn .ro-fill { background: var(--warn, #a86a00); }
.ro-track--stop .ro-fill { background: var(--stop, #9d2118); }
.ro-mark {
  position: absolute;
  top: -3px; bottom: -3px;
  width: 1px;
  background: var(--ink, #24221f);
}
.ro-ghost {
  position: absolute;
  top: -4px; bottom: -4px;
  width: 2px;
  background: var(--ink-dim, #6f6a63);
  opacity: 0.55;
}
.ro-over {
  position: absolute;
  right: -0.9rem; top: -0.25rem;
  font-size: 0.7rem;
  color: var(--ink, #24221f);
  visibility: hidden;
}
.ro-track--over .ro-over { visibility: visible; }

.ro-scale {
  position: relative;
  height: 1.1rem;
}
.ro-mark-label {
  position: absolute;
  top: 0;
  white-space: nowrap;
  font-family: var(--label-font, system-ui, sans-serif);
  font-size: 0.68rem;
  color: var(--ink-dim, #6f6a63);
}
.ro-note {
  font-size: 0.7rem;
  font-style: italic;
  white-space: nowrap;
}

.ro-jb {
  /* Seven of these are read at a glance, so the columns are fixed and the
     row is allowed to be wider than a narrow window: the container scrolls,
     rather than the numbers reflowing into each other. */
  min-width: 54rem;
  display: grid;
  grid-template-columns: 2.4rem 10rem 10.5rem 11rem 9.5rem minmax(150px, 1fr);
  align-items: center;
  gap: 0.6rem;
  padding: 0.25rem 0;
  border-bottom: 1px solid var(--rule, #cdc8bf);
}
.ro-jb-name {
  font-family: var(--num-font, ui-monospace, monospace);
  font-size: 0.85rem;
  color: var(--ink, #24221f);
}
.ro-jb-cell {
  display: flex;
  align-items: baseline;
  gap: 0.35rem;
  white-space: nowrap;
  min-width: 0;
}
.ro-jb-cell .ro-cap { flex: 0 0 auto; }
.ro-jb-right {
  display: flex;
  align-items: center;
  gap: 0.6rem;
  min-width: 0;
}
.ro-jb-bar {
  position: relative;
  flex: 1 1 auto;
  height: 14px;
  min-width: 120px;
  background: var(--track, var(--rule, #cdc8bf));
  border: 1px solid var(--rule, #cdc8bf);
}
.ro-jb-travel {
  position: absolute;
  top: 0; bottom: 0;
  background: var(--ground, #f2f0ec);
}
.ro-jb-tick {
  position: absolute;
  top: 0; bottom: 0;
  width: 1px;
  background: var(--rule, #cdc8bf);
}
.ro-jb-tick--soft { background: var(--ink, #24221f); top: -3px; bottom: -3px; }
.ro-jb-zero { position: absolute; top: 3px; bottom: 3px; width: 1px; background: var(--rule, #cdc8bf); }
.ro-jb-needle {
  position: absolute;
  top: -3px; bottom: -3px;
  width: 2px;
  margin-left: -1px;
  background: var(--ink, #24221f);
}
.ro-jb--warn .ro-jb-needle { background: var(--warn, #a86a00); }
.ro-jb--stop .ro-jb-needle { background: var(--stop, #9d2118); }
.ro-jb-cmd {
  position: absolute;
  top: -1px; bottom: -1px;
  width: 1px;
  background: var(--ask, #5b7f9c);
}
.ro-jb-continuous {
  font-family: var(--label-font, system-ui, sans-serif);
  font-size: 0.7rem;
  letter-spacing: 0.06em;
  text-transform: uppercase;
  color: var(--ink-dim, #6f6a63);
  border-bottom: 1px dashed var(--rule, #cdc8bf);
  padding-bottom: 2px;
}
.ro-fault {
  flex: 0 0 auto;
  font-family: var(--label-font, system-ui, sans-serif);
  font-size: 0.7rem;
  letter-spacing: 0.06em;
  color: var(--stop, #9d2118);
  border: 1px solid var(--stop, #9d2118);
  padding: 0 0.25rem;
  visibility: hidden;
}
.ro-fault--on { visibility: visible; }

.ro-banner {
  border: 1px solid var(--rule, #cdc8bf);
  padding: 0.75rem 1rem;
  background: var(--panel, #fbfaf8);
}
.ro-banner-top {
  display: flex;
  align-items: baseline;
  gap: 0.9rem;
  flex-wrap: wrap;
}
.ro-banner-state {
  font-family: var(--label-font, system-ui, sans-serif);
  font-size: clamp(1.9rem, 4.5vw, 3.1rem);
  font-weight: 600;
  letter-spacing: 0.02em;
  line-height: 1;
  color: var(--ink, #24221f);
}
.ro-banner--warn .ro-banner-state { color: var(--warn, #a86a00); }
.ro-banner--stop .ro-banner-state { color: var(--stop, #9d2118); }
.ro-tag {
  font-size: 0.62rem;
  text-transform: uppercase;
  letter-spacing: 0.12em;
  border: 1px solid var(--rule, #cdc8bf);
  padding: 0.05rem 0.3rem;
}
.ro-banner-meta {
  display: flex;
  gap: 1.4rem;
  flex-wrap: wrap;
  margin-top: 0.6rem;
  font-family: var(--label-font, system-ui, sans-serif);
  font-size: 0.8rem;
  color: var(--ink, #24221f);
}
.ro-banner-meta .ro-cap { display: block; }
.ro-banner-note {
  margin-top: 0.5rem;
  font-family: var(--label-font, system-ui, sans-serif);
  font-size: 0.72rem;
  font-style: italic;
  color: var(--ink-dim, #6f6a63);
}
/* On a laptop the banner competes with the scene for height, and the scene is
   where a deviation is actually seen. The state word stays large enough to
   read from a step back; everything around it tightens. */
@media (max-height: 900px) {
  .ro-banner { padding: 0.45rem 1rem 0.5rem; }
  .ro-banner-state { font-size: clamp(1.5rem, 3vw, 2.1rem); }
  .ro-banner-top { gap: 0.7rem; }
  .ro-banner-meta { margin-top: 0.3rem; font-size: 0.75rem; }
  .ro-progress { margin-top: 0.35rem; }
}

.ro-progress {
  position: relative;
  height: 6px;
  margin-top: 0.6rem;
  background: var(--track, var(--rule, #cdc8bf));
  border: 1px solid var(--rule, #cdc8bf);
}
.ro-progress-done {
  position: absolute;
  top: 0; bottom: 0; left: 0;
  width: 0;
  background: var(--ink, #24221f);
}

.ro-stale {
  display: inline-flex;
  align-items: baseline;
  gap: 0.5rem;
  padding: 0.25rem 0.5rem;
  border: 1px solid var(--rule, #cdc8bf);
}
.ro-stale-word {
  font-family: var(--label-font, system-ui, sans-serif);
  font-size: 0.72rem;
  text-transform: uppercase;
  letter-spacing: 0.1em;
  color: var(--ink-dim, #6f6a63);
}
.ro-stale--stale {
  border-color: var(--stop, #9d2118);
  border-width: 2px;
  background: var(--stop-wash, #f7e7e5);
}
.ro-stale--stale .ro-stale-word,
.ro-stale--stale .ro-slot-num,
.ro-stale--stale .ro-slot-unit { color: var(--stop, #9d2118); }
.ro-stale-reason {
  font-family: var(--label-font, system-ui, sans-serif);
  font-size: 0.7rem;
  font-style: italic;
  color: var(--ink-dim, #6f6a63);
}
.ro-stale--stale .ro-stale-reason { color: var(--stop, #9d2118); font-style: normal; }
`;

// The stylesheet is injected once, on first use, and appends to <head>.
// Appending late means panel.css cannot override these rules by cascade
// order, which is why no rule here sets a --ro-* variable: the variables are
// panel.css's to define, and this file only ever reads them with a fallback.
function ensureStyles() {
    if (document.getElementById(STYLE_ID)) return;
    const style = document.createElement("style");
    style.id = STYLE_ID;
    style.textContent = CSS;
    document.head.appendChild(style);
}

function node(tag, className, text) {
    const n = document.createElement(tag);
    if (className) n.className = className;
    if (text !== undefined) n.textContent = text;
    return n;
}

// Anything that is not a real finite number becomes null, and null renders as
// a placeholder. The one thing that must never happen is a missing value
// arriving on screen as 0.0, because 0.0 in an error column reads as perfect
// tracking. The CSV writes 'nan' in many columns, so the string is handled
// here too rather than trusting every caller to have parsed it.
function toNumber(value) {
    if (value === null || value === undefined) return null;
    if (typeof value === "number") return Number.isFinite(value) ? value : null;
    if (typeof value === "string") {
        const trimmed = value.trim();
        if (trimmed === "" || /^-?nan$/i.test(trimmed)) return null;
        const parsed = Number(trimmed);
        return Number.isFinite(parsed) ? parsed : null;
    }
    return null;
}

function clamp01(x) {
    return x < 0 ? 0 : x > 1 ? 1 : x;
}

// Wrap to (-180, 180]. Bounded joints are limited symmetrically about zero,
// but the arm reports positions in 0..360, so a joint sitting at -109 deg
// arrives as 251 and would otherwise be drawn off the end of its own range
// bar. Wrapping is idempotent, so a caller that already wrapped loses
// nothing.
function wrapDeg(deg) {
    let w = deg % 360;
    if (w > 180) w -= 360;
    if (w <= -180) w += 360;
    return w;
}

function bandLevel(name) {
    const text = String(name || "");
    if (STOP_NAME.test(text)) return "stop";
    if (WARN_NAME.test(text)) return "warn";
    return "none";
}

// Position a label on a track by fraction, pulling the two ends inward so a
// label at 0 or 1 does not hang off the component.
function placeLabel(element, fraction) {
    element.style.left = (fraction * 100).toFixed(3) + "%";
    if (fraction < 0.06) element.style.transform = "translateX(0)";
    else if (fraction > 0.9) element.style.transform = "translateX(-100%)";
    else element.style.transform = "translateX(-50%)";
}

/**
 * A fixed-width numeric slot: the primitive every other component builds on.
 *
 * @param {{chars:number, decimals?:number, unit?:string, ghost?:boolean}} opts
 *   chars is the reserved width in characters, including sign and decimal
 *   point. Reserve for the widest value the quantity can reach, not for the
 *   value you expect, or the panel will twitch on the day it matters.
 * @returns {{el:HTMLElement, set:(value:number|string|null)=>void,
 *            setLevel:(level:string)=>void}}
 */
export function createSlot({
    chars = null,
    decimals = null,
    digits = null,
    unit = "",
    label = "",
    ghost = false,
}) {
    ensureStyles();
    const places = decimals !== null ? decimals : digits !== null ? digits : 3;
    // Where no width was reserved, reserve one wide enough for a signed
    // four-digit value. Callers that know their quantity should still pass
    // chars: the guess is a floor, not a judgement.
    const width = chars !== null ? chars : 5 + places + (places > 0 ? 1 : 0);

    const el = node("span", "ro-slot" + (ghost ? " ro-slot--ghost" : ""));
    if (label) el.appendChild(node("span", "ro-cap", label));
    const num = node("span", "ro-slot-num");
    el.appendChild(num);
    if (unit) el.appendChild(node("span", "ro-slot-unit", unit));

    // The width is reserved on the number alone, so the unit never moves and
    // the number never reflows its neighbours.
    num.style.display = "inline-block";
    num.style.minWidth = width + "ch";
    num.style.textAlign = "right";

    const placeholder = "–".repeat(Math.max(1, width));

    function set(value) {
        const v = toNumber(value);
        if (v === null) {
            num.textContent = placeholder;
            el.classList.add("ro-slot--void");
            return;
        }
        el.classList.remove("ro-slot--void");
        num.textContent = v.toFixed(places);
    }

    // Severity colouring is separate from the value, because the component
    // that knows the thresholds is not always the one that owns the slot.
    function setLevel(level) {
        el.classList.remove("ro-slot--warn", "ro-slot--stop");
        if (level === "warn") el.classList.add("ro-slot--warn");
        if (level === "stop") el.classList.add("ro-slot--stop");
    }

    set(null);
    return { el, set, setLevel };
}

/**
 * One error quantity: current value, worst value so far, and a bar whose
 * marks are the compiled thresholds, labelled by what they cause.
 *
 * @param {{label:string, unit?:string, decimals?:number,
 *          bands?:Array<{at:number,name:string}>,
 *          worseWhen?:"above"|"below", full?:number, chars?:number}} opts
 *   bands come from the server, which reads them from Config.h, so the panel
 *   can never disagree with the binary and changing a knob moves the marks.
 *   worseWhen is "below" for quantities where small is bad — sigma_min and
 *   the joint limit margin — so the severity test flips while the drawing
 *   stays the same.
 *   full is the bar's end of scale. It is fixed for the life of the row: a
 *   scale that grew to fit the value would move the threshold marks, and a
 *   moving mark is worse than a pegged bar. Values past the end peg and show
 *   an over-range arrow; the number itself is always readable, so nothing is
 *   lost.
 * @returns {{el:HTMLElement, update:(value:any, worst?:any)=>void}}
 */
export function createErrorRow({
    label,
    unit = "",
    decimals = null,
    digits = null,
    bands = [],
    note = null,
    worseWhen = "above",
    full = null,
    chars = null,
}) {
    ensureStyles();

    const places = decimals !== null ? decimals : digits !== null ? digits : 3;
    const sorted = bands
        .map((b) => ({
            at: Number(b.at !== undefined ? b.at : b.value),
            name: String(b.name),
        }))
        .filter((b) => Number.isFinite(b.at))
        .sort((a, b) => a.at - b.at);

    const largest = sorted.length ? sorted[sorted.length - 1].at : 1;
    const endOfScale =
        full !== null && Number.isFinite(full)
            ? full
            : worseWhen === "below"
              ? largest * 4
              : largest * 1.3;

    const width =
        chars !== null
            ? chars
            : String(Math.trunc(Math.abs(endOfScale))).length + places + 2;

    const el = node("div", "ro-row");

    const head = node("div", "ro-row-head");
    const labelWrap = node("span", "ro-label");
    labelWrap.appendChild(document.createTextNode(label));
    head.appendChild(labelWrap);

    const value = createSlot({ chars: width, decimals: places, unit });
    head.appendChild(value.el);

    const worstWrap = node("span", "ro-row-worst");
    worstWrap.appendChild(node("span", "ro-cap", "worst"));
    const worstSlot = createSlot({
        chars: width,
        decimals: places,
        unit,
        ghost: true,
    });
    worstWrap.appendChild(worstSlot.el);
    head.appendChild(worstWrap);
    el.appendChild(head);

    const track = node("div", "ro-track");
    const fill = node("div", "ro-fill");
    track.appendChild(fill);
    const ghostMark = node("div", "ro-ghost");
    ghostMark.style.visibility = "hidden";
    const scale = node("div", "ro-scale");

    for (const band of sorted) {
        const fraction = clamp01(band.at / endOfScale);
        const mark = node("div", "ro-mark");
        mark.style.left = (fraction * 100).toFixed(3) + "%";
        mark.title = band.name + " at " + band.at + " " + unit;
        track.appendChild(mark);

        const text = node("span", "ro-mark-label", band.name);
        placeLabel(text, fraction);
        scale.appendChild(text);
    }

    track.appendChild(ghostMark);
    track.appendChild(node("span", "ro-over", "▸"));
    el.appendChild(track);

    // Where nothing stops the arm, the row says so. Orientation error is the
    // case that forced this: it has an arrival tolerance and no stop, and an
    // invented round number on that bar is exactly the kind of thing someone
    // later believes.
    // It rides with the label rather than with the bar, because a band label
    // sitting near the right-hand end of the scale would collide with it,
    // and the one sentence that must never be half-covered is the one saying
    // that nothing here stops the arm.
    const hasStop = sorted.some((b) => bandLevel(b.name) === "stop");
    const noteText = note !== null ? note : hasStop ? "" : "no stop threshold";
    if (noteText) {
        labelWrap.appendChild(document.createTextNode(" "));
        labelWrap.appendChild(node("span", "ro-note", "— " + noteText));
    }
    el.appendChild(scale);

    function levelFor(v) {
        let level = "none";
        for (const band of sorted) {
            const crossed = worseWhen === "below" ? v <= band.at : v >= band.at;
            if (!crossed) continue;
            const bandsLevel = bandLevel(band.name);
            if (bandsLevel === "stop") level = "stop";
            else if (bandsLevel === "warn" && level !== "stop") level = "warn";
        }
        return level;
    }

    function update(current, worst) {
        const v = toNumber(current);
        value.set(v);
        const w = toNumber(worst);
        worstSlot.set(w);

        track.classList.remove("ro-track--warn", "ro-track--stop", "ro-track--over");
        if (v === null) {
            fill.style.width = "0%";
            value.setLevel("none");
        } else {
            const fraction = clamp01(v / endOfScale);
            fill.style.width = (fraction * 100).toFixed(3) + "%";
            if (v > endOfScale) track.classList.add("ro-track--over");
            const level = levelFor(v);
            value.setLevel(level);
            if (level !== "none") track.classList.add("ro-track--" + level);
        }

        if (w === null) {
            ghostMark.style.visibility = "hidden";
            worstSlot.setLevel("none");
        } else {
            ghostMark.style.visibility = "visible";
            ghostMark.style.left = (clamp01(w / endOfScale) * 100).toFixed(3) + "%";
            worstSlot.setLevel(levelFor(w));
        }
    }

    update(null, null);
    // set() is panel.js's spelling, and it passes one object rather than two
    // arguments.
    const set = (a, b) =>
        a !== null && typeof a === "object"
            ? update(a.value, a.worst)
            : update(a, b);
    return { el, update, set };
}

/**
 * One joint: angle, velocity, following error, and — only where the joint
 * actually has limits — distance to the software limit and a bar drawn to
 * the joint's true travel.
 *
 * @param {{index:number, limits:{bounded:boolean, lower_deg?:number,
 *          upper_deg?:number, warn_deg?:number, error_deg?:number,
 *          software_limit_deg?:number, follow_error_stop_deg?:number,
 *          replan_margin_deg?:number}}} opts
 *   Only J2, J4 and J6 are bounded on this arm. J1, J3, J5 and J7 turn
 *   continuously, so they get no range bar and no distance-to-limit at all:
 *   drawing a limit a joint does not have is the kind of thing one later
 *   believes, and a normalised 0-100 bar would do exactly that.
 *   follow_error_stop_deg and replan_margin_deg are optional and come from
 *   the compiled config. Without them the corresponding number is shown but
 *   never coloured, because this component will not invent the threshold
 *   that stops the arm or the one that asks for a replan.
 * @returns {{el:HTMLElement, update:(sample:{angle_deg?:any,
 *            velocity_deg_s?:any, follow_err_deg?:any, margin_deg?:any,
 *            fault?:any})=>void}}
 */
export function createJointBar(options) {
    ensureStyles();

    // panel.js passes one flat object per joint, carrying index and
    // continuous rather than a nested limits and bounded.
    const opts = options || {};
    const limits = opts.limits || opts;
    const index = opts.index !== undefined ? opts.index : (limits.index || 0);
    const bounded =
        limits.bounded !== undefined
            ? Boolean(limits.bounded)
            : limits.continuous !== undefined
              ? !limits.continuous
              : false;

    const lower = toNumber(limits && limits.lower_deg);
    const upper = toNumber(limits && limits.upper_deg);
    const softLimit = toNumber(limits && limits.software_limit_deg);
    const warn = toNumber(limits && limits.warn_deg);
    const error = toNumber(limits && limits.error_deg);
    const followStop = toNumber(limits && limits.follow_error_stop_deg);
    const replanMargin = toNumber(limits && limits.replan_margin_deg);

    const el = node("div", "ro-jb");
    el.appendChild(node("span", "ro-jb-name", "J" + index));

    function cell(caption, slot) {
        const wrap = node("span", "ro-jb-cell");
        wrap.appendChild(node("span", "ro-cap", caption));
        wrap.appendChild(slot.el);
        el.appendChild(wrap);
        return wrap;
    }

    const angle = createSlot({ chars: 8, decimals: 2, unit: "deg" });
    cell("pos", angle);
    const velocity = createSlot({ chars: 7, decimals: 2, unit: "deg/s" });
    cell("vel", velocity);
    const follow = createSlot({ chars: 6, decimals: 2, unit: "deg" });
    cell("cmd−meas", follow);

    const margin = createSlot({ chars: 6, decimals: 2, unit: "deg" });
    const marginCell = cell("to limit", margin);
    if (!bounded) {
        // The cell is emptied rather than dashed. A dash would read as "not
        // measured yet", when the truth is that the quantity does not exist
        // for this joint; the words at the end of the row say which.
        marginCell.replaceChildren();
    }

    const right = node("span", "ro-jb-right");

    let bar = null;
    let needle = null;
    let cmdTick = null;

    // True travel: the bar spans the widest thing that matters — the
    // mechanical range or the firmware error threshold, whichever reaches
    // further — so the mechanical range is drawn as a region inside it and the
    // firmware thresholds sit where they really are relative to it.
    const scaleMax = Math.max(
        Math.abs(lower === null ? 0 : lower),
        Math.abs(upper === null ? 0 : upper),
        softLimit === null ? 0 : softLimit,
        warn === null ? 0 : warn,
        error === null ? 0 : error,
    );
    // A joint said to be bounded but carrying no numbers gets no bar either.
    // The alternative is a bar with invented ends, which is the failure this
    // component exists to avoid.
    const hasRange = bounded && scaleMax > 0;

    if (hasRange) {
        bar = node("div", "ro-jb-bar");
        const toFraction = (deg) => clamp01((deg + scaleMax) / (2 * scaleMax));

        if (lower !== null && upper !== null) {
            const travel = node("div", "ro-jb-travel");
            travel.style.left = (toFraction(lower) * 100).toFixed(3) + "%";
            travel.style.right = ((1 - toFraction(upper)) * 100).toFixed(3) + "%";
            travel.title =
                "mechanical travel " + lower + " to " + upper + " deg";
            bar.appendChild(travel);
        }

        const zero = node("div", "ro-jb-zero");
        zero.style.left = (toFraction(0) * 100).toFixed(3) + "%";
        bar.appendChild(zero);

        const tick = (deg, className, title) => {
            if (deg === null) return;
            for (const signed of [-deg, deg]) {
                const t = node("div", "ro-jb-tick " + className);
                t.style.left = (toFraction(signed) * 100).toFixed(3) + "%";
                t.title = title + " at " + signed.toFixed(1) + " deg";
                bar.appendChild(t);
            }
        };
        tick(error, "", "firmware error threshold");
        tick(warn, "", "firmware warning threshold");
        tick(softLimit, "ro-jb-tick--soft", "software limit");

        // The commanded tick is drawn in the one colour reserved across the
        // whole panel for "what was asked for", so cmd means the same thing
        // here as it does in the 3D scene.
        cmdTick = node("div", "ro-jb-cmd");
        cmdTick.style.visibility = "hidden";
        bar.appendChild(cmdTick);

        needle = node("div", "ro-jb-needle");
        needle.style.visibility = "hidden";
        bar.appendChild(needle);

        bar.title =
            "travel " +
            (lower === null ? "?" : lower) +
            " to " +
            (upper === null ? "?" : upper) +
            " deg; software limit ±" +
            (softLimit === null ? "?" : softLimit) +
            " deg";
        right.appendChild(bar);
    } else {
        right.appendChild(
            node(
                "span",
                "ro-jb-continuous",
                bounded ? "limits not reported" : "continuous — no limit",
            ),
        );
    }

    const fault = node("span", "ro-fault", "FAULT");
    right.appendChild(fault);
    el.appendChild(right);

    function update(sample) {
        const s = sample || {};
        const rawAngle = toNumber(s.angle_deg);
        // A bounded joint's limits are symmetric about zero, so its angle is
        // shown wrapped; a continuous joint has no such reference, so its
        // angle is shown exactly as it was reported.
        const shownAngle =
            rawAngle === null ? null : bounded ? wrapDeg(rawAngle) : rawAngle;
        angle.set(shownAngle);
        velocity.set(s.velocity_deg_s);

        // The commanded angle is either given outright or reconstructed from
        // the following error; panel.js sends the first, this file's own
        // caller the second. Its magnitude alone (following_deg) is enough to
        // print but not to place a tick, so the tick is only drawn when the
        // signed value is known.
        const commanded = toNumber(s.commanded_deg);
        let followErr = toNumber(s.follow_err_deg);
        if (followErr === null && commanded !== null && shownAngle !== null) {
            followErr = wrapDeg(commanded - shownAngle);
        }
        const followMagnitude =
            followErr !== null ? followErr : toNumber(s.following_deg);
        follow.set(followMagnitude);
        follow.setLevel(
            followMagnitude !== null &&
                followStop !== null &&
                Math.abs(followMagnitude) >= followStop
                ? "stop"
                : "none",
        );

        // The fault column is a code in the CSV and a boolean by the time
        // panel.js has finished with it; either way the chip is on or off.
        const faultCode = toNumber(s.fault);
        const faulted = s.fault === true || (faultCode !== null && faultCode !== 0);
        fault.classList.toggle("ro-fault--on", faulted);
        fault.textContent = faultCode ? "FAULT " + faultCode : "FAULT";

        el.classList.remove("ro-jb--warn", "ro-jb--stop");
        if (!bounded) return;

        // The margin is the distance from the wrapped angle to the software
        // limit, which is what the controller's own margin means. It is
        // computed here only when the server did not send it, so the two can
        // never quietly differ in a run where both exist.
        let marginDeg = toNumber(s.margin_deg);
        if (marginDeg === null && softLimit !== null && shownAngle !== null) {
            marginDeg = softLimit - Math.abs(shownAngle);
        }
        margin.set(marginDeg);

        // A margin of zero or less means the joint is at or past its
        // software limit, which is a definition rather than a tunable
        // number, so it is the one severity this component decides for
        // itself. The replan margin above it is the compiled one, and is
        // only used when the server actually supplied it.
        let level = "none";
        if (marginDeg !== null) {
            if (marginDeg <= 0) level = "stop";
            else if (replanMargin !== null && marginDeg <= replanMargin)
                level = "warn";
        }
        if (faulted) level = "stop";
        margin.setLevel(level);
        if (level !== "none") el.classList.add("ro-jb--" + level);

        if (!hasRange) return;

        const toFraction = (deg) => clamp01((deg + scaleMax) / (2 * scaleMax));
        if (shownAngle === null) {
            needle.style.visibility = "hidden";
        } else {
            needle.style.visibility = "visible";
            needle.style.left = (toFraction(shownAngle) * 100).toFixed(3) + "%";
        }
        if (shownAngle === null || followErr === null) {
            cmdTick.style.visibility = "hidden";
        } else {
            cmdTick.style.visibility = "visible";
            cmdTick.style.left =
                (toFraction(wrapDeg(shownAngle + followErr)) * 100).toFixed(3) + "%";
        }
    }

    update(null);
    return { el, update, set: update };
}

/**
 * The state banner: what the panel believes is happening, in text at a size
 * that reads across the room.
 *
 * The state word is marked DERIVED because no state machine in the
 * controller publishes these names — the panel infers them from the session
 * it launched and from traj_activated / traj_complete. The controller line
 * is marked REPORTED instead, because the CSV preamble records the compiled
 * reference source, so the panel can say what it says because the binary
 * said so.
 *
 * @returns {{el:HTMLElement, update:(status:{state?:string, arm?:string,
 *            task?:string, controller?:string, trajectory_id?:string,
 *            progress?:number|null, remaining_s?:number|null,
 *            derived?:boolean})=>void}}
 */
export function createBanner({ derived: derivedDefault = true } = {}) {
    ensureStyles();

    const el = node("div", "ro-banner");

    const top = node("div", "ro-banner-top");
    const state = node("span", "ro-banner-state", "IDLE");
    const derivedTag = node("span", "ro-tag", "derived");
    top.appendChild(state);
    top.appendChild(derivedTag);
    el.appendChild(top);

    const meta = node("div", "ro-banner-meta");
    function metaField(caption) {
        const wrap = node("span");
        wrap.appendChild(node("span", "ro-cap", caption));
        const v = node("span", "", "–");
        wrap.appendChild(v);
        meta.appendChild(wrap);
        return v;
    }
    const armValue = metaField("arm");
    const taskValue = metaField("task");

    const controllerWrap = node("span");
    controllerWrap.appendChild(node("span", "ro-cap", "controller"));
    const controllerValue = node("span", "", "–");
    controllerWrap.appendChild(controllerValue);
    controllerWrap.appendChild(document.createTextNode(" "));
    controllerWrap.appendChild(node("span", "ro-tag", "reported"));
    meta.appendChild(controllerWrap);

    const trajectoryValue = metaField("session id");
    el.appendChild(meta);

    const progressBar = node("div", "ro-progress");
    const progressDone = node("div", "ro-progress-done");
    progressBar.appendChild(progressDone);
    el.appendChild(progressBar);

    const progressLine = node("div", "ro-banner-meta");
    const progressWrap = node("span");
    progressWrap.appendChild(node("span", "ro-cap", "progress"));
    const progressSlot = createSlot({ chars: 5, decimals: 1, unit: "%" });
    progressWrap.appendChild(progressSlot.el);
    progressLine.appendChild(progressWrap);
    const remainingWrap = node("span");
    remainingWrap.appendChild(node("span", "ro-cap", "remaining"));
    const remainingSlot = createSlot({ chars: 6, decimals: 1, unit: "s" });
    remainingWrap.appendChild(remainingSlot.el);
    progressLine.appendChild(remainingWrap);
    el.appendChild(progressLine);

    const note = node("div", "ro-banner-note", "");
    el.appendChild(note);

    const NOTES = {
        // Worth saying every time it is on screen: there is no ramp in the
        // controller, so STOPPING covers only the gap between the stop
        // request and the loop exiting.
        STOPPING: "no ramp: commanding ceases, then the arm is released",
        FAULT: "the controller exited; the stop reason is in controller.log",
    };

    function update(status) {
        const s = status || {};
        const name = String(s.state || "IDLE").toUpperCase();
        state.textContent = name;

        el.classList.remove("ro-banner--warn", "ro-banner--stop");
        if (name === "FAULT") el.classList.add("ro-banner--stop");
        else if (name === "STOPPING") el.classList.add("ro-banner--warn");

        // The tag is a claim about where the state came from, so it is
        // allowed to be switched off — but only by a caller that really does
        // have a published state, which today nothing does.
        const derived = s.derived !== undefined ? s.derived : derivedDefault;
        derivedTag.style.visibility = derived === false ? "hidden" : "visible";

        armValue.textContent = s.arm ? String(s.arm) : "–";
        taskValue.textContent = s.task ? String(s.task) : "–";
        controllerValue.textContent = s.controller ? String(s.controller) : "–";
        trajectoryValue.textContent = s.trajectory_id
            ? String(s.trajectory_id)
            : "–";
        // The identity is the panel's own: the controller never echoes an ID
        // back, so the panel must not claim a confirmation it does not have.
        trajectoryValue.title =
            "panel-side session id; the controller does not echo it back";

        const progress = toNumber(s.progress);
        if (progress === null) {
            progressDone.style.width = "0%";
            progressSlot.set(null);
        } else {
            const fraction = clamp01(progress);
            progressDone.style.width = (fraction * 100).toFixed(3) + "%";
            progressSlot.set(fraction * 100);
        }
        remainingSlot.set(s.remaining_s);

        // detail is whatever the page has to add about this state — the stop
        // reason, the rejection distance — and it never replaces the standing
        // note about the state itself.
        const standing = NOTES[name] || "";
        const detail = s.detail ? String(s.detail) : "";
        note.textContent =
            standing && detail ? standing + " · " + detail : standing || detail;
    }

    update(null);
    return { el, update, set: update };
}

/**
 * The age of the newest telemetry frame, always on screen.
 *
 * Age is measured by the browser's own clock against the wall_ms the server
 * stamped on the frame, so a server that has frozen cannot hide behind
 * frames it is no longer producing. That does assume the viewing machine's
 * clock agrees with the workstation's, which on loopback it does by
 * construction and over the LAN is worth knowing about.
 *
 * At 2000 ms the component takes a hard visual break, and isStale() lets the
 * page grey every live region at the same moment, so nothing on screen goes
 * on claiming to be current. The stop control is deliberately not one of
 * those regions: it must work when everything else has stopped meaning
 * anything.
 *
 * @param {{staleAfterMs?:number, breakAfterS?:number,
 *          onChange?:(stale:boolean)=>void}} [opts]
 * @returns {{el:HTMLElement, update:(lastFrameWallMs:number|null|
 *            {age_s?:number, stale?:boolean, reason?:string},
 *            reason?:string)=>void, set:Function, isStale:()=>boolean,
 *            ageMs:()=>number|null, stop:()=>void}}
 */
export function createStaleness({
    staleAfterMs = null,
    breakAfterS = null,
    onChange = null,
} = {}) {
    ensureStyles();
    const staleAt =
        staleAfterMs !== null
            ? staleAfterMs
            : breakAfterS !== null
              ? breakAfterS * 1000
              : 2000;

    const el = node("div", "ro-stale");
    el.appendChild(node("span", "ro-stale-word", "last frame"));
    const age = createSlot({ chars: 6, decimals: 2, unit: "s ago" });
    el.appendChild(age.el);
    const reasonText = node("span", "ro-stale-reason", "no frames yet");
    el.appendChild(reasonText);

    let lastWallMs = null;
    // panel.js keeps its own age and staleness — it has to, because it greys
    // the rest of the page from the same decision — and hands them over
    // already made. Whichever side computes it, it is computed from the
    // browser's clock, which is the property that matters: a frozen server
    // cannot hide behind frames it is no longer producing.
    let given = null;
    let reason = "";
    let stale = true;

    function render() {
        const wasStale = stale;
        if (given !== null) {
            age.set(given.ageMs === null ? null : given.ageMs / 1000);
            stale =
                given.stale !== null
                    ? given.stale
                    : given.ageMs !== null && given.ageMs >= staleAt;
            reasonText.textContent = stale
                ? reason || "values are not current"
                : reason;
        } else if (lastWallMs === null) {
            age.set(null);
            stale = true;
            reasonText.textContent = reason || "no frames yet";
        } else {
            const ms = Date.now() - lastWallMs;
            age.set(ms / 1000);
            stale = ms >= staleAt;
            reasonText.textContent = stale
                ? reason || "values are not current"
                : reason;
        }
        el.classList.toggle("ro-stale--stale", stale);
        if (stale !== wasStale && onChange) onChange(stale);
    }

    // The age has to keep counting up when frames stop arriving, so the
    // component runs its own clock rather than waiting to be told.
    const timer = setInterval(render, 100);

    function update(lastFrameWallMs, note) {
        if (lastFrameWallMs !== null && typeof lastFrameWallMs === "object") {
            const s = lastFrameWallMs;
            const seconds = toNumber(s.age_s);
            given = {
                ageMs: seconds === null ? null : seconds * 1000,
                stale: typeof s.stale === "boolean" ? s.stale : null,
            };
            lastWallMs = null;
            reason = s.reason ? String(s.reason) : "";
        } else {
            given = null;
            lastWallMs = toNumber(lastFrameWallMs);
            reason = note ? String(note) : "";
        }
        render();
    }

    render();
    return {
        el,
        update,
        set: update,
        isStale: () => stale,
        ageMs: () =>
            given !== null
                ? given.ageMs
                : lastWallMs === null
                  ? null
                  : Date.now() - lastWallMs,
        stop: () => clearInterval(timer),
    };
}
