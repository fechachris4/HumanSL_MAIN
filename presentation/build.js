// Generator for World_Stable_Supernumerary_Effectors_Akabueze.pptx (10-minute talk, 9 slides:
// title, 7 content, closing; no backups — Q&A material lives in a separate doc).
//
//   python3 make_assets.py            # once: copies media, crops a figure, makes QR codes
//   node build.js                     # writes the deck next to this file
//
// Content source of truth: STORYBOARD.md (only fact-sheet numbers appear on slides).
// pptxgenjs is resolved normally; if it is not installed, PPTXGENJS_MODULES (or the
// session scratchpad install) is appended to NODE_PATH.
"use strict";
const path = require("path");
const Module = require("module");
try {
	require.resolve("pptxgenjs");
} catch {
	const extra = process.env.PPTXGENJS_MODULES || "/tmp/claude-0/-home-user-HumanSL-MAIN/0c476332-a23b-581f-a648-6a214f9fc77e/scratchpad/node_modules";
	process.env.NODE_PATH = [process.env.NODE_PATH, extra].filter(Boolean).join(path.delimiter);
	Module._initPaths();
}
const pptxgen = require("pptxgenjs");
const sizeOf = require("image-size");
const SKILL = "/root/.claude/skills/synced/6f33a6e6-2fbf-4d9d-b1ab-6e494ecda283_0d43473b-fa93-4340-a5ee-dd4429b79439/pptx";
const { applyTheme } = require(path.join(SKILL, "scripts/apply_theme.js"));

const HERE = __dirname;
const A = (f) => path.join(HERE, "assets", f);
const OUT = path.join(HERE, "World_Stable_Supernumerary_Effectors_Akabueze.pptx");

// ---------------------------------------------------------------- theme
const THEME = {
	name: "Imperial Lookalike",
	headFontFace: "Arial",
	bodyFontFace: "Arial",
	colors: {
		dk1: "1B2633", // body text
		lt1: "FFFFFF",
		dk2: "002147", // Imperial navy: titles, dark slides
		lt2: "EBEEEE", // light grey: cards
		accent1: "003E74", // Imperial blue
		accent2: "0091D4", // process blue: data
		accent3: "D24000", // orange: the delay story only
		accent4: "5B6770", // slate: captions, secondary labels
		accent5: "DCEEF8", // light blue tint: chips
		accent6: "9AA3A8", // mid grey: "arm locked" / inactive
		hlink: "0091D4",
		folHlink: "003E74",
	},
};
const HEX = THEME.colors; // hex-only options (shadows)

const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE"; // 13.333 x 7.5 in
pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
pres.author = "Christian Akabueze";
pres.company = "Imperial College London";
pres.title = "World-Stable Supernumerary Effectors for Human Augmentation Under Locomotion-Scale Base Motion";
pres.subject = "MSc project talk";
const C = pres.SchemeColor;
const NAVY = C.text2, BLUE = C.accent1, DATA = C.accent2, ORANGE = C.accent3, SLATE = C.accent4, TINT = C.accent5, MIDGREY = C.accent6, GREY = C.background2, WHITE = C.background1, INK = C.text1;

// ---------------------------------------------------------------- grid
const W = 13.333, M = 0.6, R = W - M; // left / right content edges
const CW = R - M; // 12.133 content width
const TOP = 1.95; // content top on content slides
const BOTTOM = 6.85; // content bottom on content slides

// ---------------------------------------------------------------- layouts
const wordmark = (color) => ({
	text: {
		text: "Imperial College London",
		options: { x: R - 3.4, y: 0.42, w: 3.4, h: 0.4, fontSize: 16, bold: true, color, align: "right", valign: "middle", margin: 0, fontFace: "Arial" },
	},
});

pres.defineSlideMaster({
	title: "TITLE_DARK",
	background: { color: HEX.dk2 },
	objects: [
		wordmark(WHITE),
		{ placeholder: { options: { name: "title", type: "title", x: M, y: 1.0, w: CW, h: 1.1, fontSize: 32, bold: true, color: WHITE, align: "left", valign: "top", margin: 0, fontFace: "Arial" }, text: "" } },
	],
});

pres.defineSlideMaster({
	title: "CONTENT",
	background: { color: HEX.lt1 },
	objects: [
		wordmark(NAVY),
		{ placeholder: { options: { name: "title", type: "title", x: M, y: 0.95, w: CW, h: 0.8, fontSize: 32, bold: true, color: NAVY, align: "left", valign: "middle", margin: 0, fontFace: "Arial" }, text: "" } },
	],
	slideNumber: { x: R - 0.6, y: 6.92, w: 0.6, h: 0.3, fontSize: 14, color: HEX.accent4, align: "right", fontFace: "Arial" },
});

pres.defineSlideMaster({
	title: "CLOSING_DARK",
	background: { color: HEX.dk2 },
	objects: [
		wordmark(WHITE),
		{ placeholder: { options: { name: "title", type: "title", x: M, y: 0.95, w: CW, h: 1.0, fontSize: 44, bold: true, color: WHITE, align: "center", valign: "middle", margin: 0, fontFace: "Arial" }, text: "" } },
	],
});

// ---------------------------------------------------------------- helpers
const shadow = () => ({ type: "outer", color: "000000", opacity: 0.18, blur: 10, offset: 3, angle: 90 });

function text(slide, str, opts) {
	slide.addText(str, Object.assign({ isTextBox: true, margin: 0, fontSize: 18, color: INK, valign: "top" }, opts));
}

// One text run: r("K", ORANGE, { subscript: true })
const r = (t, color, o = {}) => ({ text: t, options: { ...(color ? { color } : {}), ...o } });

// Section tag: a small rounded pill, same place on every content slide.
function tag(slide, label) {
	const w = 0.4 + label.length * 0.152; // ~Arial bold 14 pt + 2 pt char spacing
	slide.addText(label, {
		isTextBox: true, objectName: "Section tag", shape: pres.shapes.ROUNDED_RECTANGLE, rectRadius: 0.17,
		x: M, y: 0.44, w, h: 0.36, fill: { color: BLUE }, color: WHITE, fontSize: 14, bold: true,
		charSpacing: 2, align: "center", valign: "middle", margin: 0,
	});
}

function contentSlide(section, tagLabel, title) {
	const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: section });
	tag(s, tagLabel);
	s.addText(title, { placeholder: "title" });
	return s;
}

// Fit an image inside box (x, y, w, h) at its exact aspect ratio. align: "l" | "c" | "r", valign "t" | "m".
function fit(file, box, align = "l", valign = "t") {
	const d = sizeOf(A(file));
	const ar = d.width / d.height;
	let w = box.w, h = box.w / ar;
	if (h > box.h) { h = box.h; w = h * ar; }
	const x = align === "c" ? box.x + (box.w - w) / 2 : align === "r" ? box.x + box.w - w : box.x;
	const y = valign === "m" ? box.y + (box.h - h) / 2 : box.y;
	return { x, y, w, h, px: d };
}

// Image inside a rounded card (white frame + soft shadow). Returns the card rectangle.
function framedImage(slide, file, box, opts = {}) {
	const pad = opts.pad ?? 0.08;
	const inner = fit(file, { x: box.x + pad, y: box.y + pad, w: box.w - 2 * pad, h: box.h - 2 * pad }, opts.align, opts.valign);
	const card = { x: inner.x - pad, y: inner.y - pad, w: inner.w + 2 * pad, h: inner.h + 2 * pad };
	slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
		objectName: `Frame ${file}`, ...card, rectRadius: 0.12, fill: { color: opts.fill || WHITE },
		line: { color: opts.line || HEX.lt2, width: 0.75 }, shadow: shadow(),
	});
	slide.addImage({ path: A(file), x: inner.x, y: inner.y, w: inner.w, h: inner.h, objectName: file, altText: opts.alt || file });
	return { ...card, img: inner };
}

function caption(slide, str, x, y, w, opts = {}) {
	text(slide, str, { x, y, w, h: opts.h || 0.35, fontSize: 14, color: opts.color || SLATE, objectName: "Caption", ...opts });
}

function roundCard(slide, box, fill, name, extra = {}) {
	slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { objectName: name, ...box, rectRadius: extra.r ?? 0.14, fill: { color: fill }, line: extra.line || { type: "none" }, ...(extra.shadow ? { shadow: shadow() } : {}) });
}

function arrow(slide, x1, y1, x2, y2, color = NAVY, width = 2) {
	// Lines are drawn from (x, y) with w/h; flips encode direction.
	const o = { x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.abs(x2 - x1), h: Math.abs(y2 - y1), line: { color, width, endArrowType: "triangle" } };
	if (x2 < x1) o.flipH = true;
	if (y2 < y1) o.flipV = true;
	slide.addShape(pres.shapes.LINE, o);
}

// Box with centred text (diagram node).
function node(slide, str, box, fill, color, opts = {}) {
	slide.addText(str, {
		isTextBox: true, objectName: opts.name || `Node ${String(str).slice(0, 20)}`, shape: pres.shapes.ROUNDED_RECTANGLE, rectRadius: opts.r ?? 0.1,
		...box, fill: { color: fill }, line: opts.line || { type: "none" }, color, fontSize: opts.fontSize || 18, bold: opts.bold ?? true,
		align: "center", valign: "middle", margin: 0.04, ...(opts.shadow ? { shadow: shadow() } : {}),
	});
}

// Big number with a label underneath (stat column rows).
function stat(slide, big, label, x, y, w, o) {
	text(slide, big, { x, y, w, h: o.bigH, fontSize: o.bigPt, bold: true, color: o.color, valign: "middle", align: o.align || "left", objectName: `Stat ${big}` });
	text(slide, label, { x, y: y + o.bigH + (o.gap ?? 0.04), w, h: o.labelH, fontSize: o.labelPt, color: o.labelColor || INK, align: o.align || "left", objectName: `Stat ${big} label` });
}

// ---------------------------------------------------------------- notes (spoken script)
// First person, ~130 words/min. Each note starts with a timing cue computed from its word
// count: [slide time | running total], both rounded to the nearest 5 s.
const SCRIPT = {
	title: `Imagine wearing two extra robot arms on your back and asking them to hold something still while you walk. That was my MSc project, in the MUVE Lab at Imperial; I'm Christian Akabueze. Here the wearer twists about sixty degrees: the elbows swing, and the hands stay close to where they were. To be clear up front: the simulation, my controllers, the trial analysis and the diagnosis are mine; the hardware trials ran a 400 hertz controller on the lab machine.`,
	problem: `The arms are two seven-joint Kinova Gen3 robots on a backpack, which I'll call the mount, and Vicon motion capture tracks it. A robot arm you wear is an extra pair of hands, until you take a step: anything it holds moves with you. If the arms were locked, walking would move the hands 63 to 68 millimetres RMS, at every speed we tried. Locked means the arm held rigid on the mount, so the hand just rides along with the backpack. And steps come about 1.8 times a second, so this is fast, repeating motion, not slow drift. So the goal is set in the world frame: hold each hand fixed in the room, not on the body.`,
	frames: `The approach is a chain of 3D frames. Vicon gives the mount's pose in the room, live. The arm base sits at a fixed, known offset on the mount, and forward kinematics places the hand relative to the base. Compose the three and I have the hand's pose and its Jacobian in the room frame: the same composition as camera extrinsics. The control law is a PD term on that room-frame error, how far the hand is from its spot and how fast it is drifting, turned into joint speeds by a damped pseudo-inverse. The null-space term keeps the spare seventh joint near mid-range, and a safety filter can override a command that comes too close to the wearer; it was off in the simulation runs. I wrote this law in Python for simulation and as a 500 hertz real-time C++ controller; that's the second link at the end. Keep an eye on the orange part, K-d times e-dot, the velocity term. It comes back.`,
	sim: `Before hardware, I ran my Python controller in MuJoCo, a physics engine, with the safety filter off. The mount follows a walking-like motion at 1.8 hertz, smaller than real walking. Same motion, three controllers. Over eight seconds, locked arms leave 31.0 millimetres RMS, and the reactive controller brings that to 6.7. Adding feedforward, which tells the controller how the mount is moving instead of waiting for the error, brings it to 2.6. The counters in the clip are a running RMS over a shorter window. The gains here are stiffer than on hardware, so compare trends, not millimetres. And remember that 2.6.`,
	hardware: `Then real wearers. Seven people walked on an instrumented treadmill at 0.5, 1 and 1.5 metres per second. The trials ran a 400 hertz controller on the lab machine: Kalman-filtered Vicon, no feedforward, and a velocity term that is a gain, K-d, on the room-frame velocity error. My C++ controller did not run these trials; the analysis is mine. In the replay, grey is where a locked arm would have put the hand, and blue is where it went. I checked that locked-arm model on arms that weren't being controlled: it is right to within half a percent. Scored that way, the arm removed 71, 63 and 56 percent of the motion. Those are means over six of the seven people, four at the fastest speed; the first session is reported separately. That's 128 trials, mostly one arm at a time. Faster walking moves the mount faster, not further, and speed is what hurts: at 1 metre per second, slow drift is 85 percent cancelled at a quarter of a hertz, but the bounce of each step only 29 percent at 2 hertz.`,
	gain: `The obvious fix for that bounce is more velocity gain. In a pilot session with one wearer, K-d was stepped from 0.8 to 2.6, more than three times higher. The model predicted that the response near 1 hertz would fall by 38 percent. It fell by 9, about a quarter of the predicted effect. And across six people, the measured amplitude sits on a model with no velocity term at all. So the velocity term was barely acting, and more tuning was not going to fix that. Before changing anything else, I needed to know why.`,
	delay: `So I traced the velocity signal end to end. The Kalman estimate of the mount's velocity lags the real motion by 36 to 46 milliseconds. The Vicon measurement is about 10 milliseconds old, and the joints take about 15 to respond. In total, the velocity arrives 61 to 71 milliseconds late. That's the trade-off: filtering makes the velocity smooth, but late, and a late velocity has the right size but the wrong timing, so subtracting it does not cancel the real one. In plain terms, the arm reacts to how the backpack was moving, not how it is moving now. For feedforward, what's left over is exactly two times the sine of omega d over two, where omega is two pi times the step frequency and d is the delay. At 1.8 hertz and 60 milliseconds that is two-thirds of the velocity, a little more across 61 to 71. Inside a feedback loop it also depends on the gains, so I checked the full model: with the measured delay and nothing fitted, it matches the hardware better than the no-velocity model in six of six people. It's the explanation that fits the data, not yet proof; confirming it means changing the estimator on hardware, which is the next experiment.`,
	impact: `So, the impact. On real arms worn by seven people, the arm removed about two-thirds of walking motion: 56 to 71 percent, means over six. The part I'm proudest of is the diagnosis. When tuning stopped helping, the data pointed to a 61 to 71 millisecond delay, and that sets the next step: a faster velocity estimate, then feedforward on the real arms, which in simulation, with no delay, took the error from 6.7 to 2.6 millimetres. And my own C++ controller isn't that fix yet: its velocity estimate uses a 50 millisecond low-pass, itself about 45 milliseconds of lag at step frequency. What transfers beyond robotics: frames and calibration are camera extrinsics, and the constant 18 to 21 millimetre offset I found is a calibration problem; a latency budget is sensor time-sync; and I validated my metric against ground truth, to half a percent, before trusting any number.`,
	close: `Thank you. The results, simulation and figures are in msc_project, and my C++ real-time controller is in HumanSL_MAIN. Happy to take questions.`,
};
const WPM = 130;
const wordCount = (str) => str.split(/\s+/).filter(Boolean).length;
const mmss = (sec) => { const t = Math.round(sec); return `${Math.floor(t / 60)}:${String(t % 60).padStart(2, "0")}`; };
const NOTES = {};
{
	let run = 0; // exact cumulative seconds; both cues are rounded to the nearest 5 s
	const r5 = (x) => Math.round(x / 5) * 5;
	for (const [k, str] of Object.entries(SCRIPT)) {
		const sec = (wordCount(str) / WPM) * 60;
		run += sec;
		NOTES[k] = `[${mmss(r5(sec))} | ${mmss(r5(run))}] ${str}`;
	}
}

// ================================================================= slides
const TAG = { P: "PROBLEM", AS: "APPROACH & SOLUTION", TC: "TRADE-OFFS & CHALLENGES", I: "IMPACT" };

// ---- 1. Title (dark)
// The thesis title needs >= 8.0 in per line to fit 3 lines at 32 pt, so it runs full width
// (2 lines, explicit break) and the hook clip gets the large right-hand block below it.
{
	pres.addSection({ title: "Opening" });
	const s = pres.addSlide({ masterName: "TITLE_DARK", sectionTitle: "Opening" });
	s.addText([
		{ text: "World-Stable Supernumerary Effectors for Human", options: { breakLine: true } },
		{ text: "Augmentation Under Locomotion-Scale Base Motion", options: {} },
	], { placeholder: "title" });
	text(s, "Keeping a wearable robot's hands still while the wearer walks", { x: M, y: 2.27, w: CW, h: 0.45, fontSize: 24, color: WHITE, objectName: "Subtitle" });
	const capH = 0.3, capGap = 0.08, top = 2.85, bottom = 7.0;
	const pad = 0.08, cardW = 5.45;
	const d = fit("hw_turn.gif", { x: 0, y: 0, w: cardW - 2 * pad, h: 10 });
	const cardH = d.h + 2 * pad;
	const cy = top + Math.max(0, (bottom - top - (cardH + capGap + capH)) / 2);
	const card = framedImage(s, "hw_turn.gif", { x: R - cardW, y: cy, w: cardW, h: cardH }, { pad, align: "r", line: HEX.lt1, alt: "The wearer twists his torso; the robot hands stay close to where they were" });
	caption(s, "Watch the hands, not the elbows.", card.x, card.y + card.h + capGap, card.w, { h: capH, fontSize: 16, color: WHITE });
	const lx = M, lw = card.x - M - 0.4, blockH = 1.3, ly = card.y + (card.h - blockH) / 2;
	text(s, [
		{ text: "Christian Akabueze", options: { fontSize: 24, bold: true, color: WHITE, breakLine: true } },
		{ text: "MSc Human and Biological Robotics", options: { fontSize: 20, color: GREY, breakLine: true } },
		{ text: "Imperial College London · MUVE Lab · 2026", options: { fontSize: 20, color: GREY } },
	], { x: lx, y: ly, w: lw, h: blockH, paraSpaceAfter: 4, objectName: "Presenter" });
	s.addNotes(NOTES.title);
}

// ---- 2. Problem: every step moves your extra hands (+ the goal, in one line)
{
	pres.addSection({ title: "Problem" });
	const s = contentSlide("Problem", TAG.P, "Every step moves your extra hands");
	const card = framedImage(s, "hw_walk.gif", { x: M, y: 1.92, w: 4, h: 7.0 - 1.92 }, { alt: "A participant walking on the treadmill with both arms on" });
	const sx = card.x + card.w + 0.9, sw = R - sx;
	const goalH = 0.5, goalY = card.y + card.h - goalH;
	const o = { bigPt: 80, bigH: 1.15, labelPt: 24, labelH: 0.48, gap: 0.06, color: BLUE };
	const pitch = 1.8, y0 = 2.35;
	stat(s, "63–68 mm", "hand motion, arm locked (RMS)", sx, y0, sw, o);
	stat(s, "~1.8 Hz", "step rate", sx, y0 + pitch, sw, o);
	text(s, "Goal: hands fixed in the room, not the body", { x: sx, y: goalY, w: sw, h: goalH, fontSize: 26, bold: true, color: NAVY, valign: "bottom", objectName: "Goal" });
	s.addNotes(NOTES.problem);
}

// ---- 3. Approach & solution: frame chain + control law
{
	pres.addSection({ title: "Approach & solution" });
	const s = contentSlide("Approach & solution", TAG.AS, "Measure the backpack, cancel it through the arm");
	const nodes = ["World", "Backpack", "Arm base", "Hand"];
	const nw = 2.3, ny = 2.1, nh = 1.05;
	const gap = (CW - 4 * nw) / 3;
	const xs = nodes.map((_, i) => M + i * (nw + gap));
	nodes.forEach((a, i) => node(s, a, { x: xs[i], y: ny, w: nw, h: nh }, NAVY, WHITE, { fontSize: 24, name: `Frame ${a}` }));
	const edges = [
		["Vicon, live", DATA, WHITE],
		["fixed offset", GREY, NAVY],
		["forward kinematics", TINT, NAVY],
	];
	const chipW = 3.0, chipH = 0.65, chipY = ny + nh + 0.27;
	edges.forEach(([a, fill, col], i) => {
		const x1 = xs[i] + nw + 0.08, x2 = xs[i + 1] - 0.08;
		arrow(s, x1, ny + nh / 2, x2, ny + nh / 2, NAVY, 2.25);
		const cx = (x1 + x2) / 2;
		node(s, a, { x: cx - chipW / 2, y: chipY, w: chipW, h: chipH }, fill, col, { fontSize: 20, r: 0.12, name: `Edge ${a}` });
	});

	// Control law. The velocity term Kd·ė is orange: it is the term that fails on slides 6-7.
	const ey = 4.45, eh = 1.2;
	roundCard(s, { x: M, y: ey, w: CW, h: eh }, GREY, "Equation card");
	const P1 = DATA, P2 = BLUE, P3 = SLATE;
	text(s, [
		r("q̇ = ", NAVY),
		r("J", P2), r("+", P2, { superscript: true }), r("λ", P2, { subscript: true }),
		r(" (K", P1), r("p", P1, { subscript: true }), r("·e + ", P1),
		r("K", ORANGE), r("d", ORANGE, { subscript: true }), r("·ė", ORANGE),
		r(")", P1),
		r(" + ", NAVY),
		r("(I − J", P3), r("+", P3, { superscript: true }), r("J)·q̇", P3), r("0", P3, { subscript: true }),
	], { x: M, y: ey, w: CW, h: eh, fontSize: 40, bold: true, align: "center", valign: "middle", objectName: "Control law" });

	// One gloss line under the band. The orange marker sits under Kd·ė; its x was measured on the
	// rendered equation (Arial / Liberation Sans metrics), the gloss text is anchored to it.
	const KD_X = 6.65; // centre of "Kd·ė" in the 40 pt centred equation, inches from the left edge
	const gy = ey + eh + 0.2, gh = 0.4, tri = 0.2;
	s.addShape(pres.shapes.ISOSCELES_TRIANGLE, { objectName: "Velocity-term marker", x: KD_X - tri / 2, y: gy + 0.08, w: tri, h: 0.17, fill: { color: ORANGE }, line: { type: "none" } });
	text(s, "velocity term", { x: KD_X + tri / 2 + 0.1, y: gy, w: 2.4, h: gh, fontSize: 20, color: ORANGE, valign: "middle", objectName: "Velocity-term label" });
	text(s, "room-frame error → joint speeds", { x: M, y: gy, w: KD_X - tri / 2 - 0.55 - M, h: gh, fontSize: 20, color: SLATE, align: "right", valign: "middle", objectName: "Control-law gloss" });
	s.addNotes(NOTES.frames);
}

// ---- 4. Approach & solution: simulation
{
	const s = contentSlide("Approach & solution", TAG.AS, "Simulation first: my controller in a physics engine");
	const card = framedImage(s, "hold_pose.gif", { x: M, y: TOP, w: 8.4, h: BOTTOM - TOP }, { alt: "Simulation: arms locked, reactive controller, reactive controller with feedforward" });
	const sx = card.x + card.w + 0.45, sw = R - sx;
	text(s, "Hand error, RMS over 8 s", { x: sx, y: TOP, w: sw, h: 0.4, fontSize: 18, bold: true, color: SLATE, objectName: "Sim header" });
	const o = { bigPt: 54, bigH: 0.85, labelPt: 20, labelH: 0.4, gap: 0.02 };
	[["31.0 mm", "arms locked", SLATE], ["6.7 mm", "reactive", DATA], ["2.6 mm", "reactive + feedforward", ORANGE]].forEach(([big, label, color], i) => {
		stat(s, big, label, sx, TOP + 0.6 + i * 1.5, sw, { ...o, color });
	});
	s.addNotes(NOTES.sim);
}

// ---- 5. Approach & solution: hardware result
{
	const s = contentSlide("Approach & solution", TAG.AS, "On real wearers: about 2/3 of walking motion removed");
	text(s, "Trials: a 400 Hz controller on the lab machine. Analysis: mine.", { x: M, y: 1.85, w: CW, h: 0.4, fontSize: 20, color: NAVY, objectName: "Provenance" });
	const rep = framedImage(s, "hw_replay.gif", { x: M, y: 2.6, w: 8.7, h: 4.2 }, { alt: "One trial replayed: locked-arm path in grey, measured hand in blue" });
	const sx = rep.x + rep.w + 0.4, sw = R - sx;
	const o = { bigPt: 60, bigH: 0.85, labelPt: 20, labelH: 0.38, gap: 0.0, color: DATA };
	const y0 = 2.4, pitch = 1.3;
	[["71%", "0.5 m/s"], ["63%", "1.0 m/s"], ["56%", "1.5 m/s"]].forEach(([big, label], i) => stat(s, big, label, sx, y0 + i * pitch, sw, o));
	text(s, "Mean of 6 wearers", { x: sx, y: y0 + 3 * pitch + 0.02, w: sw, h: 0.35, fontSize: 18, color: SLATE, objectName: "Results note" });
	s.addNotes(NOTES.hardware);
}

// ---- 6. Trade-offs & challenges: the gain barely helped
{
	pres.addSection({ title: "Trade-offs & challenges" });
	const s = contentSlide("Trade-offs & challenges", TAG.TC, "Turning up the gain barely helped");
	text(s, [r("One-wearer pilot: K"), r("d", null, { subscript: true }), r(" 0.8 → 2.6. Response near 1 Hz fell:")],
		{ x: M, y: 2.0, w: CW, h: 0.5, fontSize: 22, color: SLATE, align: "center", valign: "middle", objectName: "Pilot set-up" });
	const colW = 4.4, cxs = [W / 2 - 2.75, W / 2 + 2.75];
	[["38%", "predicted", SLATE], ["9%", "measured", ORANGE]].forEach(([big, label, color], i) => {
		stat(s, big, label, cxs[i] - colW / 2, 2.65, colW, { bigPt: 130, bigH: 1.9, labelPt: 24, labelH: 0.5, gap: 0.02, color, align: "center" });
	});
	text(s, "The velocity term was barely acting. Why?", { x: M, y: 5.7, w: CW, h: 0.65, fontSize: 30, bold: true, color: NAVY, align: "center", valign: "middle", objectName: "Takeaway" });
	s.addNotes(NOTES.gain);
}

// ---- 7. Trade-offs & challenges: the delay
{
	const s = contentSlide("Trade-offs & challenges", TAG.TC, "Best explanation: the velocity arrives 61–71 ms late");
	// Delay budget, proportional boxes in orange shades. Kalman lag: 36 ms solid + a fading tail to 46 ms.
	const bx = M, by = 2.3, bh = 1.3, barW = 8.75, k = barW / 71;
	const kw = 46 * k;
	s.addShape(pres.shapes.RECTANGLE, { objectName: "Kalman lag (36 ms)", x: bx, y: by, w: 36 * k + 0.02, h: bh, fill: { color: ORANGE }, line: { type: "none" } });
	// 36 -> 46 ms: a gradient image (no gradient fills in pptxgenjs), so the bar fades instead of ending in a box
	s.addImage({ path: A("kalman_fade.png"), x: bx + 36 * k, y: by, w: 10 * k, h: bh, objectName: "Kalman lag fade (36-46 ms)", altText: "Kalman lag range, 36 to 46 ms" });
	text(s, "Kalman filter 36–46 ms", { x: bx, y: by, w: kw, h: bh, fontSize: 20, bold: true, color: WHITE, align: "center", valign: "middle", objectName: "Kalman label" });
	const segs = [
		{ ms: 10, transp: 25, runs: [r("sensing", NAVY, { breakLine: true }), r("~10 ms", NAVY)] },
		{ ms: 15, transp: 45, runs: [r("joints", NAVY, { breakLine: true }), r("~15 ms", NAVY)] },
	];
	let x = bx + kw;
	segs.forEach((sg, i) => {
		const w = sg.ms * k;
		s.addShape(pres.shapes.RECTANGLE, { objectName: `Delay segment ${sg.runs[0].text}`, x, y: by, w, h: bh, fill: { color: ORANGE, transparency: sg.transp }, line: { type: "none" } });
		s.addShape(pres.shapes.LINE, { objectName: `Delay divider ${i + 1}`, x, y: by, w: 0, h: bh, line: { color: WHITE, width: 2.5 } });
		text(s, sg.runs, { x, y: by, w, h: bh, fontSize: 20, bold: true, align: "center", valign: "middle" });
		x += w;
	});
	text(s, "= 61–71 ms", { x: x + 0.22, y: by, w: R - x - 0.22, h: bh, fontSize: 40, bold: true, color: ORANGE, valign: "middle", objectName: "Delay total" });

	text(s, [
		r("Uncancelled: 2·sin("), r("ωd", null, { italic: true }), r("/2) ≈ "), r("2/3", ORANGE), r(" at 1.8 Hz"),
	], { x: M, y: 4.4, w: CW, h: 0.75, fontSize: 32, bold: true, color: NAVY, valign: "middle", objectName: "Delay formula" });
	text(s, "Nothing fitted: beats no-velocity model, 6/6 wearers", { x: M, y: 5.5, w: CW, h: 0.45, fontSize: 20, color: SLATE, valign: "middle", objectName: "Evidence" });
	s.addNotes(NOTES.delay);
}

// ---- 8. Impact, and what transfers
{
	pres.addSection({ title: "Impact" });
	const s = contentSlide("Impact", TAG.I, "Impact, and what transfers");
	const lw = 5.4, h = BOTTOM - TOP, ix = M + 0.4, iw = lw - 0.8;
	roundCard(s, { x: M, y: TOP, w: lw, h }, NAVY, "Impact card", { shadow: true });
	text(s, "2/3", { x: ix, y: TOP + 0.25, w: iw, h: 1.65, fontSize: 120, bold: true, color: WHITE, valign: "middle", objectName: "Impact stat" });
	text(s, [r("walking motion removed", null, { breakLine: true }), r("on hardware")], { x: ix, y: TOP + 2.0, w: iw, h: 0.95, fontSize: 24, color: WHITE, objectName: "Impact label" });
	// Next step as an inset panel at the foot of the card
	const fh = 0.95, fy = TOP + h - 0.3 - fh;
	roundCard(s, { x: M + 0.25, y: fy, w: lw - 0.5, h: fh }, BLUE, "Next-step panel", { r: 0.1 });
	text(s, [r("Next: ", null, { bold: true }), r("faster estimator")], { x: M + 0.5, y: fy, w: lw - 1.0, h: fh, fontSize: 22, color: WHITE, valign: "middle", objectName: "Next step" });

	// What transfers: deliberately quieter than the card (no heading, slate, small grey dots)
	const tx = M + lw + 0.6, items = ["Frames, calibration → camera extrinsics", "Latency budget → sensor time-sync", "Ground-truth the metric"];
	const rowH = 0.45, pitch = 1.15, y0 = TOP + (h - (2 * pitch + rowH)) / 2, dot = 0.16;
	items.forEach((t, i) => {
		const y = y0 + i * pitch;
		s.addShape(pres.shapes.OVAL, { objectName: `Transfer dot ${i + 1}`, x: tx, y: y + (rowH - dot) / 2, w: dot, h: dot, fill: { color: MIDGREY }, line: { type: "none" } });
		text(s, t, { x: tx + 0.4, y, w: R - tx - 0.4, h: rowH, fontSize: 22, color: SLATE, valign: "middle", objectName: `Transfer ${i + 1}` });
	});
	s.addNotes(NOTES.impact);
}

// ---- 9. Close (dark)
{
	pres.addSection({ title: "Close" });
	const s = pres.addSlide({ masterName: "CLOSING_DARK", sectionTitle: "Close" });
	s.addText("Thank you. Questions?", { placeholder: "title" });
	const repos = [
		["qr_msc_project.png", "Results, simulation, figures", "github.com/fechachris4/msc_project", "https://github.com/fechachris4/msc_project"],
		["qr_humansl_main.png", "C++ real-time controller", "github.com/fechachris4/HumanSL_MAIN", "https://github.com/fechachris4/HumanSL_MAIN"],
	];
	const qs = 2.5, blockW = 5.2;
	repos.forEach(([png, label, short, url], i) => {
		const cxm = i === 0 ? W / 2 - 3.0 : W / 2 + 3.0;
		roundCard(s, { x: cxm - qs / 2, y: 2.35, w: qs, h: qs }, WHITE, `QR card ${i + 1}`, { r: 0.14 });
		s.addImage({ path: A(png), x: cxm - qs / 2 + 0.12, y: 2.47, w: qs - 0.24, h: qs - 0.24, objectName: `QR ${short}`, altText: `QR code for ${url}`, hyperlink: { url, tooltip: short } });
		text(s, label, { x: cxm - blockW / 2, y: 5.0, w: blockW, h: 0.42, fontSize: 20, bold: true, color: WHITE, align: "center" });
		text(s, [{ text: short, options: { hyperlink: { url, tooltip: label }, color: DATA } }], { x: cxm - blockW / 2, y: 5.45, w: blockW, h: 0.38, fontSize: 18, align: "center" });
	});
	text(s, [{ text: "fecha412@gmail.com", options: { hyperlink: { url: "mailto:fecha412@gmail.com" }, color: WHITE } }], { x: M, y: 6.2, w: CW, h: 0.45, fontSize: 22, bold: true, align: "center", objectName: "Email" });
	s.addNotes(NOTES.close);
}

// ================================================================= write
(async () => {
	await pres.writeFile({ fileName: OUT });
	await applyTheme(OUT, THEME);
	const words = Object.values(SCRIPT).map(wordCount);
	const total = words.reduce((a, b) => a + b, 0);
	console.log(`wrote ${OUT}`);
	console.log(`notes words per slide: ${words.join(", ")} | total ${total} | ${mmss(total / WPM * 60)} at ${WPM} wpm`);
	console.log(`cues: ${Object.values(NOTES).map((n) => n.slice(0, n.indexOf("]") + 1)).join(" ")}`);
})().catch((e) => { console.error(e); process.exit(1); });
