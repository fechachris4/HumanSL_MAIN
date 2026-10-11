// Generator for MSc_Project_Christian_Akabueze.pptx (10-minute talk).
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
const HEX = THEME.colors; // hex-only options (shadows, chart colours)

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

function numberDot(slide, n, x, y, d = 0.5, fill = BLUE) {
	slide.addText(String(n), { isTextBox: true, objectName: `Step ${n}`, shape: pres.shapes.OVAL, x, y, w: d, h: d, fill: { color: fill }, color: WHITE, fontSize: 18, bold: true, align: "center", valign: "middle", margin: 0 });
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

// ---------------------------------------------------------------- notes (spoken script)
// First person, ~130 words/min. Each main-slide note starts with a timing cue computed from
// its word count: [slide time | running total]. Backup notes are untimed.
const SCRIPT = {
	title: `Imagine wearing two extra robot arms on your back and asking them to hold something still while you walk. Every step moves them six to seven centimetres. That was my MSc project at Imperial, in the MUVE Lab. I'm Christian Akabueze. In this clip the wearer twists about sixty degrees: the elbows swing, the hands stay close to where they were. To be clear up front: the simulation, the controller, the trial analysis and the diagnosis are mine; the hardware trials ran a controller on the lab machine.`,
	problem: `The two arms are Kinova Gen3 robots, seven joints each, mounted on a backpack, which I'll call the mount. Vicon motion-capture cameras track the backpack in the room. The problem is walking. A robot arm you wear is an extra pair of hands, until you take a step: anything it holds moves with you. If the arms were simply locked, the hands would move 63 to 68 millimetres RMS, at every walking speed we tried. And steps come at roughly 1.8 per second, so this is fast, repeating motion, not slow drift.`,
	goal: `The goal: hold each hand fixed in the room, not on the body. This is my simulation with the controller on: four positions of the backpack in one cycle, with the motion enlarged three times. Panel e averages the whole cycle. The backpack and upper arms blur, because they move. The hands stay sharp, because the controller cancels that motion.`,
	frames: `The approach is a chain of coordinate frames. Vicon measures the backpack's pose in the room. The arm base sits at a fixed, known offset on the backpack. The joint angles, through forward kinematics, place the hand relative to the base. Compose the three and I know where the hand is in the room, and how each joint moves it: the Jacobian. The part in brackets is a PD controller, proportional plus derivative, on the error in the room frame: how far the hand is from its spot, and how fast it is drifting. A damped pseudo-inverse of the Jacobian turns that into joint speeds, and the damping keeps it calm near awkward poses. The spare seventh joint drifts toward mid-range without moving the hand. A safety filter can also override a command that would bring the arm too close to the wearer. It was switched off in the simulation runs I'll show. This is the law in my own controllers, in simulation and in C++.`,
	sim: `Before hardware, I tested my Python controller in MuJoCo, a physics engine. The backpack follows a walking-like motion at 1.8 hertz. It's a controlled stand-in for walking, smaller than the real thing. Same motion three times: arms locked, my controller, and the controller plus feedforward. Over eight seconds, locked arms leave 31 millimetres of error. The controller brings it to 6.7. Feedforward, which tells the controller how the backpack is moving, gets it to 2.6. The simulation's gains are stiffer than on hardware, so compare trends, not millimetres.`,
	hardware: `Seven participants wore the arms on an instrumented treadmill at 0.5, 1 and 1.5 metres per second. These trials ran a 400 hertz controller on the lab machine. It estimates the backpack's pose and velocity from Vicon with a Kalman filter. It has no feedforward. My own C++ controller wasn't used in these trials. The analysis is mine. On the left, one trial replayed: grey is where the hand would have gone with the arm locked; blue is where it went. To score a trial, I compare the two. I checked the locked model on arms that weren't being controlled: it's right to within half a percent. On the right: 71 percent of the motion removed at slow walking, 63 at normal speed, 56 at fast, means over six of the seven; the first session is reported separately. That's over 128 walking trials, mostly one arm at a time. And on a platform that pitched and swayed, it removed 78 percent.`,
	cpp: `Alongside the analysis, I wrote a real-time controller in C++ for the same rig, in the HumanSL_MAIN repository. The trade-off I designed around: planning is slow and unpredictable, but the control loop has to run every 2 milliseconds without fail. So Vicon arrives at 100 hertz on its own thread and is handed over through a wait-free slot; the loop never waits on a lock. The 500 hertz core runs the kinematics and the law and commands both arms. Planning, with GPMP2, runs on a separate worker thread, and every plan passes one validator before it can run. Planning moves a hand to a new target around the wearer's body; the fast loop holds it there. One honest caveat: its backpack-velocity estimate is low-pass filtered with a 50 millisecond time constant, which at step frequency is itself roughly 45 milliseconds of lag, so a better estimator is the next piece. The cost: a new plan arrives later, after the validator.`,
	gain: `Back to the trials, and the main challenge. Slow sway was mostly cancelled, 85 percent at a quarter of a hertz, but the bounce of each step was harder: only 29 percent at 2 hertz. The obvious fix: turn up the velocity gain. In a pilot with one wearer I raised it from 0.8 to 2.6. The model predicted the response near 1 hertz, about the stride rate, would fall by 38 percent. It fell by 9. And across six people, the measured amplitude matches a model with no velocity term at all. So the velocity term was barely acting. More tuning would not fix that; I needed to know why.`,
	delay: `I traced the velocity signal end to end. The Kalman filter's estimate of the backpack's velocity lags the real motion by 36 to 46 milliseconds. The measurement is about 10 milliseconds old, and the joints take about 15 to respond. In total, the velocity is 61 to 71 milliseconds late. A late estimate has the right size but the wrong timing, so subtracting it does not cancel the true velocity. What's left over is two times the sine of half the phase lag, and the phase lag is omega times d. Steps come at 1.8 hertz. At 60 milliseconds late, the velocity is off by two-thirds of its own size. For a feedforward term that's the whole story; inside a feedback loop it also depends on the gains, so I checked it with the full model. With the measured delay and nothing fitted, the model matches the hardware better than the no-velocity model in six of six people. It's the explanation that fits the data; confirming it directly means changing the estimator on hardware, which is the next experiment.`,
	impact: `On real arms worn by seven people, the arm removes about two-thirds of walking motion, 56 to 71 percent depending on speed. The part I'm proudest of is the diagnosis: when tuning stopped helping, I found the cause in the data, a 61 to 71 millisecond delay. That sets the next step: first a faster velocity estimate, then feedforward on the real arms; in simulation, with no delay, feedforward cut the error from 6.7 to 2.6 millimetres. What transfers: frames are camera extrinsics; the 18 to 21 millimetre offset I found is an extrinsic-calibration problem; a latency budget is sensor time-sync; and I check my metric against ground truth before I trust a result. The lesson that transfers: measure the latency budget before you tune the gains.`,
	close: `Thank you. Everything is on GitHub: results, simulation and figures in msc_project, and the C++ controller in HumanSL_MAIN. Happy to take questions.`,
};
const BACKUP_SCRIPT = {
	loop: `The per-cycle loop as implemented in simulation, 500 times a second. Dashed orange is feedforward, simulation only. The safety-filter step keeps 13 spheres per arm outside a cylinder around the wearer; it was inactive in the reported runs.`,
	freq: `Frequency response at 1 metre per second, median of six participants. Blue is measured. The dotted model with no velocity term sits on it; the dash-dot model with the logged gains predicts more rejection than the arm reached; green adds a late velocity signal with the delay fitted per person, 65 to 90 milliseconds; plugging in the measured 61 to 71 instead gives almost the same curve.`,
	simdelay: `Simulation, right arm at 1.8 hertz, with the backpack-velocity signal delayed on purpose. Each 10 milliseconds of delay costs the feedforward about 1 millimetre. Past about 30 milliseconds the velocity-gain term alone is worse than no velocity term; at 60 milliseconds both sit at 8.2 to 8.6 millimetres. Grey band: the 61 to 71 milliseconds measured on hardware. The simulation's gains are stiffer than the hardware's, so compare the trend, not the millimetres.`,
	limits: `Three limits from the trial data. In 12 of 128 walking trials the motion planner moved the reference more than 100 millimetres from the target, and those trials had the most hand motion, so that is a planner setting. At 1.5 metres per second a commanded joint speed was at its limit 32 percent of the time, mostly in the wrist, against about 10 percent at slower speeds. And the hand sat a constant 18 to 21 millimetres from its goal; that offset follows the disagreement between the arm's kinematic model and Vicon, so it is calibration.`,
	platform: `Five participants also stood on the treadmill platform while it pitched and swayed. The arm removed 78 percent of the motion. Pitch steps moved the locked point 94 millimetres RMS and the hand 14; sway steps 81 and 17.`,
	rig: `The rig: two Kinova Gen3 seven-joint arms on a backpack, with Vicon cameras around the instrumented treadmill.`,
};
const WPM = 130;
const wordCount = (str) => str.split(/\s+/).filter(Boolean).length;
const mmss = (sec) => `${Math.floor(sec / 60)}:${String(Math.round(sec % 60)).padStart(2, "0")}`;
const NOTES = {};
{
	let run = 0; // exact cumulative seconds; both cues are rounded to the nearest 5 s
	const r5 = (x) => Math.round(x / 5) * 5;
	for (const [k, str] of Object.entries(SCRIPT)) {
		const sec = (wordCount(str) / WPM) * 60;
		run += sec;
		NOTES[k] = `[${mmss(r5(sec))} | ${mmss(r5(run))}] ${str}`;
	}
	for (const [k, str] of Object.entries(BACKUP_SCRIPT)) NOTES[k] = `[backup, only if asked] ${str}`;
}

// ================================================================= main slides
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
	const capH = 0.32, top = 2.95, bottom = 7.0;
	const pad = 0.08, cardW = 5.45;
	const d = fit("hw_turn.gif", { x: 0, y: 0, w: cardW - 2 * pad, h: 10 });
	const cardH = d.h + 2 * pad;
	const cy = top + (bottom - top - (cardH + 0.1 + capH)) / 2;
	const card = framedImage(s, "hw_turn.gif", { x: R - cardW, y: cy, w: cardW, h: cardH }, { pad, align: "r", line: HEX.lt1, alt: "The wearer twists his torso; the robot hands stay close to where they were" });
	caption(s, "Watch the hands, not the elbows.", card.x, card.y + card.h + 0.1, card.w, { color: GREY });
	const lx = M, lw = card.x - M - 0.4, blockH = 2.2, ly = card.y + (card.h - blockH) / 2;
	text(s, [
		{ text: "Christian Akabueze", options: { fontSize: 24, bold: true, color: WHITE, breakLine: true } },
		{ text: "MSc Human and Biological Robotics", options: { fontSize: 20, color: GREY, breakLine: true } },
		{ text: "Imperial College London · MUVE Lab · 2026", options: { fontSize: 20, color: GREY } },
	], { x: lx, y: ly, w: lw, h: 1.3, paraSpaceAfter: 4, objectName: "Presenter" });
	text(s, [{ text: "github.com/fechachris4/msc_project", options: { hyperlink: { url: "https://github.com/fechachris4/msc_project", tooltip: "Project repository" }, color: DATA } }],
		{ x: lx, y: ly + blockH - 0.35, w: lw, h: 0.35, fontSize: 18, objectName: "GitHub link" });
	s.addNotes(NOTES.title);
}

// ---- 2. Problem: every step moves your extra hands
{
	pres.addSection({ title: "Problem" });
	const s = contentSlide("Problem", TAG.P, "Every step moves your extra hands");
	const card = framedImage(s, "hw_walk.gif", { x: M, y: 1.92, w: 4, h: 7.0 - 1.92 }, { alt: "A participant walking on the treadmill with both arms on" });
	const sx = card.x + card.w + 0.9, sw = R - sx, mid = card.y + card.h / 2;
	caption(s, "Walking on the treadmill, both arms on.", sx, card.y + card.h - 0.32, sw);
	const stats = [
		["63–68 mm", "hand motion if the arm were locked (RMS)"],
		["~1.8 Hz", "how often steps come: fast, repeating motion"],
	];
	const rowH = 1.75, y0 = mid - rowH;
	stats.forEach(([big, label], i) => {
		const y = y0 + i * rowH;
		text(s, big, { x: sx, y, w: sw, h: 1.15, fontSize: 80, bold: true, color: BLUE, valign: "middle", objectName: `Stat ${i + 1}` });
		text(s, label, { x: sx, y: y + 1.22, w: sw, h: 0.45, fontSize: 22, color: INK, objectName: `Stat ${i + 1} label` });
	});
	s.addNotes(NOTES.problem);
}

// ---- 3. Problem: the goal
{
	const s = contentSlide("Problem", TAG.P, "Goal: fix the hand in the room, not on the body");
	const capH = 0.75, gapC = 0.15;
	const d = fit("task_illustration.png", { x: M, y: 0, w: CW, h: 4 });
	const capText = 0.6, visTop = 0.1, y = (1.6 + 6.9 - visTop - (d.h + gapC + capText)) / 2 + 0; // equal visual gap above and below
	s.addImage({ path: A("task_illustration.png"), x: M, y, w: d.w, h: d.h, objectName: "Task illustration", altText: "Four backpack positions in one cycle and the average, where the hands stay sharp" });
	text(s, [{ text: "(e) one cycle averaged: the backpack blurs, the hands stay sharp.", options: { breakLine: true } }, { text: "Simulation, controller on. Motion enlarged 3×.", options: {} }], { x: M, y: y + d.h + gapC, w: CW, h: capH, fontSize: 18, color: SLATE, objectName: "Caption" });
	s.addNotes(NOTES.goal);
}

// ---- 4. Approach: frame chain + control law
{
	pres.addSection({ title: "Approach & solution" });
	const s = contentSlide("Approach & solution", TAG.AS, "Measure the backpack, cancel it through the arm");
	const nodes = [["World", "the room"], ["Backpack", "mount"], ["Arm base", ""], ["Hand", "end-effector"]];
	const nw = 2.1, ny = TOP + 0.05, nh = 1.0;
	const gap = (CW - 4 * nw) / 3;
	const xs = nodes.map((_, i) => M + i * (nw + gap));
	nodes.forEach(([a, b], i) => {
		const runs = [{ text: a, options: { fontSize: 20, bold: true, color: WHITE, breakLine: !!b } }];
		if (b) runs.push({ text: `(${b})`, options: { fontSize: 16, bold: false, color: WHITE } });
		node(s, runs, { x: xs[i], y: ny, w: nw, h: nh }, NAVY, WHITE, { name: `Frame ${a}` });
	});
	const edges = [
		["Vicon motion capture", "measured live", DATA, WHITE],
		["Fixed mount offset", "known", GREY, NAVY],
		["Joint angles → kinematics", "computed", TINT, NAVY],
	];
	edges.forEach(([a, b, fill, col], i) => {
		const x1 = xs[i] + nw + 0.08, x2 = xs[i + 1] - 0.08;
		arrow(s, x1, ny + nh / 2, x2, ny + nh / 2, NAVY, 2.25);
		const cx = (x1 + x2) / 2, cwid = 3.1;
		node(s, [
			{ text: a, options: { fontSize: 16, bold: true, color: col, breakLine: true } },
			{ text: b, options: { fontSize: 14, bold: false, color: col } },
		], { x: cx - cwid / 2, y: ny + nh + 0.2, w: cwid, h: 0.78 }, fill, col, { r: 0.12, name: `Edge ${a}` });
	});

	// Control law + one-line meaning per term
	const ey = 4.2;
	roundCard(s, { x: M, y: ey, w: CW, h: 0.9 }, GREY, "Equation card");
	const P1 = DATA, P2 = BLUE, P3 = SLATE;
	const r = (t, color, o = {}) => ({ text: t, options: { color, ...o } });
	text(s, [
		r("q̇ = ", NAVY),
		r("J", P2), r("+", P2, { superscript: true }), r("λ", P2, { subscript: true }),
		r(" (K", P1), r("p", P1, { subscript: true }), r("·e + K", P1), r("d", P1, { subscript: true }), r("·ė)", P1),
		r("  +  ", NAVY),
		r("(I − J", P3), r("+", P3, { superscript: true }), r("J)·q̇", P3), r("0", P3, { subscript: true }),
	], { x: M, y: ey, w: CW, h: 0.9, fontSize: 32, bold: true, align: "center", valign: "middle", objectName: "Control law" });
	const terms = [
		[[r("K", P1), r("p", P1, { subscript: true }), r("·e + K", P1), r("d", P1, { subscript: true }), r("·ė", P1)], "push the hand back to its spot in the room"],
		[[r("J", P2), r("+", P2, { superscript: true }), r("λ", P2, { subscript: true })], "→ joint speeds, damped near awkward poses"],
		[[r("(I − J", P3), r("+", P3, { superscript: true }), r("J)·q̇", P3), r("0", P3, { subscript: true })], "spare 7th joint drifts to mid-range"],
	];
	terms.forEach(([runs, gloss], i) => {
		const y = 5.3 + i * 0.52;
		text(s, runs.map((x) => ({ text: x.text, options: { ...x.options, bold: true } })), { x: M + 0.3, y, w: 2.9, h: 0.46, fontSize: 20, valign: "middle", objectName: `Term ${i + 1}` });
		text(s, gloss, { x: M + 3.3, y, w: CW - 3.3, h: 0.46, fontSize: 20, valign: "middle", objectName: `Gloss ${i + 1}` });
	});
	s.addNotes(NOTES.frames);
}

// ---- 5. Approach & solution: simulation
{
	const s = contentSlide("Approach & solution", TAG.AS, "Simulation first: my controller in a physics engine");
	const card = framedImage(s, "hold_pose.gif", { x: M, y: TOP, w: 7.55, h: 4.4 }, { alt: "Simulation: arms locked, reactive controller, controller with feedforward" });
	caption(s, "Python controller in MuJoCo. Same motion (1.8 Hz), three controllers. Watch the red target, middle row.", card.x, card.y + card.h + 0.12, card.w, { h: 0.5 });
	const sx = 8.75, sw = R - sx;
	text(s, "Hand error over 8 s (RMS)", { x: sx, y: TOP, w: sw, h: 0.4, fontSize: 18, bold: true, color: SLATE });
	const rows = [["31.0 mm", "Arms locked", SLATE], ["6.7 mm", "Controller", DATA], ["2.6 mm", "+ feedforward", ORANGE]];
	rows.forEach(([big, label, color], i) => {
		const y = TOP + 0.5 + i * 1.33;
		text(s, big, { x: sx, y, w: sw, h: 0.8, fontSize: 54, bold: true, color, valign: "middle", objectName: `Sim stat ${i + 1}` });
		text(s, label, { x: sx, y: y + 0.8, w: sw, h: 0.38, fontSize: 18, objectName: `Sim stat ${i + 1} label` });
	});
	s.addNotes(NOTES.sim);
}

// ---- 6. Approach & solution: hardware result (trials + chart)
{
	const s = contentSlide("Approach & solution", TAG.AS, "On real wearers: about 2/3 of walking motion removed");
	text(s, "Trials ran a 400 Hz controller on the lab machine (Kalman-filtered Vicon). The analysis is mine.", { x: M, y: 1.78, w: CW, h: 0.4, fontSize: 18, color: NAVY, objectName: "Provenance" });
	const top = 2.3, chartW = 4.7, chartX = R - chartW, noteH = 0.62, chartH = BOTTOM - top - noteH;
	// Clip + caption, vertically centred against the chart. Kept >= 6.5 in wide (baked-in tick labels).
	const pad = 0.08, boxW = chartX - M - 0.18;
	const d = fit("hw_replay.gif", { x: 0, y: 0, w: boxW - 2 * pad, h: 10 });
	const blockH = d.h + 2 * pad + 0.12 + 0.35;
	const rep = framedImage(s, "hw_replay.gif", { x: M, y: top + 0.2, w: boxW, h: d.h + 2 * pad }, { alt: "One trial replayed: locked-arm path in grey, measured hand in blue" });
	caption(s, "Grey: where the hand would go if the arm were locked. Blue: where it went.", rep.x, rep.y + rep.h + 0.12, rep.w);
	s.addChart(pres.charts.BAR, [{ name: "Motion removed", labels: ["0.5 m/s", "1.0 m/s", "1.5 m/s"], values: [71, 63, 56] }], {
		x: chartX, y: top, w: chartW, h: chartH, objectName: "Results chart", altText: "Bar chart: 71, 63 and 56 percent of backpack motion removed at 0.5, 1.0 and 1.5 m/s",
		barDir: "col", barGapWidthPct: 50, chartColors: [HEX.accent2],
		showTitle: true, title: "Motion removed, by walking speed", titleFontSize: 16, titleColor: HEX.accent4, titleFontFace: "+mn-lt", titleBold: true,
		showValue: true, dataLabelPosition: "outEnd", dataLabelFormatCode: '0"%"', dataLabelFontSize: 28, dataLabelFontBold: true, dataLabelColor: HEX.dk2, dataLabelFontFace: "+mn-lt",
		catAxisLabelFontSize: 18, catAxisLabelColor: HEX.dk1, catAxisLabelFontFace: "+mn-lt", catAxisLineShow: true, catAxisLineColor: HEX.accent6,
		valAxisHidden: true, valAxisMinVal: 0, valAxisMaxVal: 100, valGridLine: { style: "none" }, catGridLine: { style: "none" },
		showLegend: false,
	});
	text(s, "Mean of 6 people (4 at 1.5 m/s) · 128 trials, mostly one arm", { x: chartX + 0.15, y: top + chartH + 0.02, w: chartW - 0.15, h: noteH - 0.02, fontSize: 16, color: SLATE, objectName: "Chart note" });
	s.addNotes(NOTES.hardware);
}

// ---- 7. Trade-offs & challenges: real-time C++ architecture
{
	pres.addSection({ title: "Trade-offs & challenges" });
	const s = contentSlide("Trade-offs & challenges", TAG.TC, "Trade-off: planning can never block the 2 ms loop");
	const by = TOP, bh = 2.15;
	roundCard(s, { x: M, y: by, w: CW, h: bh }, GREY, "Real-time band");
	text(s, "REAL-TIME PATH · EVERY 2 ms", { x: M + 0.25, y: by + 0.1, w: 5, h: 0.32, fontSize: 14, bold: true, color: SLATE, charSpacing: 1 });
	const ry = by + 0.5, rh = 1.45;
	const vic = { x: 0.85, y: ry, w: 1.6, h: rh };
	const slot = { x: 2.75, y: ry, w: 1.35, h: rh };
	const core = { x: 4.4, y: ry, w: 4.85, h: rh };
	const kor = { x: 9.55, y: ry, w: 1.75, h: rh };
	const arms = { x: 11.6, y: ry, w: 0.95, h: rh };
	node(s, [{ text: "Vicon thread", options: { breakLine: true } }, { text: "100 Hz", options: { bold: false } }], vic, WHITE, NAVY, { fontSize: 18, shadow: true, name: "Vicon thread" });
	node(s, [{ text: "Wait-free", options: { breakLine: true } }, { text: "slot", options: {} }], slot, WHITE, NAVY, { fontSize: 18, shadow: true, name: "Wait-free slot" });
	roundCard(s, core, NAVY, "Control core", { r: 0.1, shadow: true });
	text(s, "500 Hz control core", { x: core.x, y: core.y + 0.1, w: core.w, h: 0.42, fontSize: 20, bold: true, color: WHITE, align: "center" });
	const chips = ["Kinematics", "Control law", "Clamp + integrate"];
	const cpw = 1.4, cgap = (core.w - 0.24 - 3 * cpw) / 2, chy = core.y + 0.62, chh = 0.72;
	chips.forEach((c, i) => {
		const x = core.x + 0.12 + i * (cpw + cgap);
		node(s, c, { x, y: chy, w: cpw, h: chh }, TINT, NAVY, { fontSize: 16, name: `Core ${c}` });
		if (i < 2) arrow(s, x + cpw + 0.03, chy + chh / 2, x + cpw + cgap - 0.03, chy + chh / 2, WHITE, 1.75);
	});
	node(s, [{ text: "Kortex cyclic", options: { breakLine: true } }, { text: "exchange", options: {} }], kor, WHITE, NAVY, { fontSize: 18, shadow: true, name: "Kortex" });
	node(s, [{ text: "Both", options: { breakLine: true } }, { text: "arms", options: {} }], arms, WHITE, NAVY, { fontSize: 18, shadow: true, name: "Arms" });
	const my = ry + rh / 2;
	[[vic, slot], [slot, core], [core, kor], [kor, arms]].forEach(([a, b]) => arrow(s, a.x + a.w + 0.04, my, b.x - 0.04, my, NAVY, 2.25));

	// Planner worker thread: label sits outside the card (left) so the card stays compact
	const py = by + bh + 0.45, ph = 1.15;
	const pl = { x: 2.9, y: py, w: 7.65, h: ph };
	roundCard(s, pl, WHITE, "Planner thread", { r: 0.12, line: { color: HEX.accent6, width: 1.25, dashType: "dash" } });
	text(s, [{ text: "PLANNER THREAD", options: { breakLine: true } }, { text: "NOT REAL-TIME", options: {} }], { x: M, y: py, w: pl.x - M - 0.2, h: ph, fontSize: 14, bold: true, color: SLATE, charSpacing: 1, align: "right", valign: "middle", objectName: "Planner label" });
	const pcw = 2.25, pch = 0.8, pcy = py + (ph - pch) / 2, pgap = (pl.w - 0.4 - 3 * pcw) / 2;
	const pcs = ["GPMP2 planner", "Validator", "Trajectory mailbox"].map((c, i) => [c, pl.x + 0.2 + i * (pcw + pgap)]);
	pcs.forEach(([c, x], i) => {
		node(s, c, { x, y: pcy, w: pcw, h: pch }, TINT, NAVY, { fontSize: 18, name: `Planner ${c}` });
		if (i < 2) arrow(s, x + pcw + 0.04, pcy + pch / 2, pcs[i + 1][1] - 0.04, pcy + pch / 2, NAVY, 2);
	});
	const coreBottom = core.y + core.h, gapMid = (by + bh + py) / 2;
	const reqX = 4.95, retX = 8.85; // inside the core, over the planner and mailbox chips
	arrow(s, reqX, coreBottom + 0.04, reqX, pcy - 0.04, NAVY, 2);
	text(s, "plan request", { x: reqX - 1.85, y: gapMid - 0.17, w: 1.7, h: 0.34, fontSize: 16, color: SLATE, align: "right" });
	arrow(s, retX, pcy - 0.04, retX, coreBottom + 0.04, NAVY, 2);
	text(s, "checked trajectory", { x: retX + 0.15, y: gapMid - 0.17, w: 2.4, h: 0.34, fontSize: 16, color: SLATE });

	const ty = py + ph + 0.33;
	text(s, [
		{ text: "Gain: ", options: { bold: true, color: NAVY } }, { text: "the 2 ms loop never waits on Vicon or the planner.", options: { breakLine: true } },
		{ text: "Cost: ", options: { bold: true, color: NAVY } }, { text: "a new plan starts later, only after it passes the validator.", options: {} },
	], { x: M, y: ty, w: CW, h: 0.62, fontSize: 18, objectName: "Gain and cost" });
	caption(s, "My C++ controller (HumanSL_MAIN); a MuJoCo twin runs the same core.", M, ty + 0.66, CW, { h: 0.28 });
	s.addNotes(NOTES.cpp);
}

// ---- 8. Trade-offs & challenges: the gain barely helped
{
	const s = contentSlide("Trade-offs & challenges", TAG.TC, "Turning up the gain barely helped");
	const colW = 4.2, cxs = [W / 2 - 2.75, W / 2 + 2.75];
	const groupH = 4.0, g0 = TOP + (BOTTOM - TOP - groupH) / 2; // whole group centred vertically
	[["38%", "predicted drop near 1 Hz", SLATE], ["9%", "measured drop near 1 Hz", ORANGE]].forEach(([big, label, color], i) => {
		text(s, big, { x: cxs[i] - colW / 2, y: g0, w: colW, h: 1.55, fontSize: 96, bold: true, color, align: "center", valign: "middle", objectName: `Compare ${i + 1}` });
		text(s, label, { x: cxs[i] - colW / 2, y: g0 + 1.6, w: colW, h: 0.45, fontSize: 22, color: INK, align: "center", objectName: `Compare ${i + 1} label` });
	});
	text(s, "vs", { x: W / 2 - 0.6, y: g0, w: 1.2, h: 1.55, fontSize: 28, color: SLATE, align: "center", valign: "middle" });
	text(s, "Pilot, one wearer: velocity gain Kd raised 0.8 → 2.6", { x: M, y: g0 + 2.55, w: CW, h: 0.45, fontSize: 22, color: SLATE, align: "center", objectName: "Pilot" });
	text(s, "The velocity term was barely acting. Why?", { x: M, y: g0 + 3.3, w: CW, h: 0.7, fontSize: 32, bold: true, color: NAVY, align: "center", valign: "middle", objectName: "Takeaway" });
	s.addNotes(NOTES.gain);
}

// ---- 9. Trade-offs & challenges: the delay
{
	const s = contentSlide("Trade-offs & challenges", TAG.TC, "Best explanation: the velocity arrives 61–71 ms late");
	// Delay budget, proportional boxes in orange shades. Kalman lag: 36 ms solid + lighter tail to 46 ms.
	const totalW = 3.0, bx = M, by = TOP + 0.35, bh = 1.1, k = (CW - totalW - 0.25) / 71;
	// Kalman lag: one bar, solid to 36 ms then fading to 46 ms; one label centred over the whole 46 ms.
	const kw = 46 * k;
	s.addShape(pres.shapes.RECTANGLE, { objectName: "Kalman lag (36 ms)", x: bx, y: by, w: 36 * k + 0.02, h: bh, fill: { color: ORANGE }, line: { type: "none" } });
	// 36 -> 46 ms: a gradient image (no gradient fills in pptxgenjs), so the bar fades instead of ending in a box
	s.addImage({ path: A("kalman_fade.png"), x: bx + 36 * k, y: by, w: 10 * k, h: bh, objectName: "Kalman lag fade (36-46 ms)", altText: "Kalman lag range, 36 to 46 ms" });
	text(s, "Kalman filter 36–46 ms", { x: bx, y: by, w: kw, h: bh, fontSize: 20, bold: true, color: WHITE, align: "center", valign: "middle", objectName: "Kalman label" });
	const segs = [
		{ ms: 10, transp: 25, runs: [{ text: "sensing", options: { color: NAVY, breakLine: true } }, { text: "~10 ms", options: { color: NAVY } }] },
		{ ms: 15, transp: 45, runs: [{ text: "joints", options: { color: NAVY, breakLine: true } }, { text: "~15 ms", options: { color: NAVY } }] },
	];
	let x = bx + kw;
	segs.forEach((sg, i) => {
		const w = sg.ms * k;
		s.addShape(pres.shapes.RECTANGLE, { objectName: `Delay segment ${sg.runs[0].text}`, x, y: by, w, h: bh, fill: { color: ORANGE, transparency: sg.transp }, line: { type: "none" } });
		s.addShape(pres.shapes.LINE, { objectName: `Delay divider ${i + 1}`, x, y: by, w: 0, h: bh, line: { color: WHITE, width: 2.5 } });
		text(s, sg.runs, { x, y: by, w, h: bh, fontSize: 20, bold: true, align: "center", valign: "middle" });
		x += w;
	});
	text(s, "= 61–71\u00A0ms", { x: x + 0.25, y: by, w: R - x - 0.25, h: bh, fontSize: 36, bold: true, color: ORANGE, valign: "middle", objectName: "Delay total" });

	text(s, [
		{ text: "share of velocity left uncancelled = 2·sin(", options: {} }, { text: "ωd", options: { italic: true } }, { text: "/2)", options: {} },
	], { x: M, y: 4.0, w: CW, h: 0.75, fontSize: 32, bold: true, color: NAVY, valign: "middle", objectName: "Delay formula" });
	text(s, "ω = 2π × step frequency · d: delay", { x: M, y: 4.75, w: CW, h: 0.32, fontSize: 16, color: SLATE, objectName: "Formula key" });
	text(s, [
		{ text: "Steps at 1.8 Hz, 60 ms late: the velocity is off by ", options: {} },
		{ text: "two-thirds", options: { bold: true, color: ORANGE } },
		{ text: " of its own size.", options: {} },
	], { x: M, y: 5.3, w: CW, h: 0.5, fontSize: 24, objectName: "Formula gloss" });
	text(s, "Measured delay, nothing fitted: it beats a no-velocity model in 6 of 6 people.", { x: M, y: 6.05, w: CW, h: 0.45, fontSize: 20, color: SLATE, objectName: "Evidence" });
	s.addNotes(NOTES.delay);
}

// ---- 10. Impact, and what transfers
{
	pres.addSection({ title: "Impact" });
	const s = contentSlide("Impact", TAG.I, "Impact, and what transfers");
	const lw = 5.4, h = BOTTOM - TOP, ix = M + 0.4, iw = lw - 0.8;
	roundCard(s, { x: M, y: TOP, w: lw, h }, NAVY, "Impact card", { shadow: true });
	text(s, "2/3", { x: ix, y: TOP + 0.25, w: iw, h: 1.65, fontSize: 120, bold: true, color: WHITE, valign: "middle", objectName: "Impact stat" });
	text(s, [{ text: "of walking motion removed on real arms, worn by 7\u00A0people", options: { breakLine: true } }, { text: "(56–71%)", options: {} }], { x: ix, y: TOP + 2.0, w: iw, h: 1.15, fontSize: 22, color: WHITE, objectName: "Impact label" });
	// "Found why" as an inset panel inside the card (no divider line)
	const fy = TOP + 3.3, fh = h - 3.3 - 0.3;
	roundCard(s, { x: M + 0.25, y: fy, w: lw - 0.5, h: fh }, BLUE, "Found-why panel", { r: 0.1 });
	text(s, [
		{ text: "Diagnosed from data: ", options: { bold: true } },
		{ text: "a 61–71\u00A0ms velocity delay. ", options: {} },
		{ text: "Next test: ", options: { bold: true } },
		{ text: "change the estimator.", options: {} },
	], { x: M + 0.5, y: fy, w: lw - 1.0, h: fh, fontSize: 20, color: WHITE, valign: "middle", objectName: "Found why" });

	const tx = M + lw + 0.6, tw = R - tx;
	text(s, "What transfers", { x: tx, y: TOP + 0.05, w: tw, h: 0.5, fontSize: 26, bold: true, color: NAVY, objectName: "Transfers heading" });
	const items = [
		["3D frames and calibration: ", "the same maths as camera extrinsics"],
		["Latency budgets: ", "time-align sensors before tuning anything"],
		["Validate the metric first: ", "0.5% check against ground truth"],
	];
	const rowH = 1.3, y0 = TOP + 0.85;
	items.forEach(([lead, rest], i) => {
		const y = y0 + i * rowH;
		s.addShape(pres.shapes.OVAL, { objectName: `Transfer dot ${i + 1}`, x: tx, y: y + 0.07, w: 0.3, h: 0.3, fill: { color: i === 0 ? DATA : BLUE }, line: { type: "none" } });
		text(s, [{ text: lead, options: { bold: true, color: NAVY } }, { text: rest, options: {} }], { x: tx + 0.55, y, w: tw - 0.55, h: 1.05, fontSize: 22, objectName: `Transfer ${i + 1}` });
	});
	s.addNotes(NOTES.impact);
}

// ---- 11. Close (dark)
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

// ================================================================= backups (after the close, no tag)
function backupSlide(title) {
	const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: "Backup" });
	s.addText(`Backup: ${title}`, { placeholder: "title" });
	return s;
}
function backupFigure(s, file, cap, alt, figH = 4.45, capH = 0.35) {
	const fig = framedImage(s, file, { x: M, y: TOP, w: CW, h: figH }, { align: "c", alt });
	caption(s, cap, M, fig.y + fig.h + 0.14, CW, { align: "center", h: capH });
}
pres.addSection({ title: "Backup" });
{
	const s = backupSlide("the control loop (simulation)");
	backupFigure(s, "control_loop.png", "Per cycle, every 2 ms in simulation. Orange dashes: feedforward (simulation only).", "Block diagram of the control loop");
	s.addNotes(NOTES.loop);
}
{
	const s = backupSlide("frequency response, 1.0 m/s");
	backupFigure(s, "hw_rejection_bc.png", "Median of 6 people. In amplitude (top), measured follows the no-velocity model (dotted). Green: late-velocity model, delay fitted per person (65–90 ms). Lower = more removed.", "Frequency response at 1.0 m/s: measured and three models", 4.2, 0.55);
	s.addNotes(NOTES.freq);
}
{
	const s = backupSlide("simulation, error vs delay");
	backupFigure(s, "velocity_delay.png", "Velocity signal delayed on purpose, 1.8 Hz. Grey band: the delay measured on hardware.", "Simulation: hand error against delay on the velocity signal");
	s.addNotes(NOTES.simdelay);
}
{
	const s = backupSlide("limits the data showed");
	const rows = [
		["Re-planning", "12 of 128 trials: the planner moved the target > 100 mm; those had the most hand motion → a planner setting"],
		["Joint speed", "at 1.5 m/s a joint at its limit 32% of the time, mostly wrist (~10% at slower walking speeds)"],
		["Fixed offset", "constant 18–21 mm from the goal follows model-vs-Vicon disagreement → calibration"],
	];
	const lw = 2.8, rh = 1.05, gap = 0.4, y0 = TOP + 0.3;
	rows.forEach(([a, b], i) => {
		const y = y0 + i * (rh + gap);
		node(s, a, { x: M, y, w: lw, h: rh }, GREY, NAVY, { fontSize: 22, r: 0.14, name: `Limit ${a}` });
		text(s, b, { x: M + lw + 0.4, y, w: CW - lw - 0.4, h: rh, fontSize: 22, valign: "middle", objectName: `Limit ${i + 1} text` });
	});
	s.addNotes(NOTES.limits);
}
{
	const s = backupSlide("pitching and swaying platform");
	text(s, "78%", { x: M, y: TOP + 0.4, w: 5, h: 1.6, fontSize: 110, bold: true, color: BLUE, valign: "middle", objectName: "Platform stat" });
	text(s, "of platform motion removed (5 people)", { x: M, y: TOP + 2.05, w: 6.2, h: 0.5, fontSize: 24 });
	text(s, "Locked arm → hand, RMS", { x: 7.0, y: TOP + 0.55, w: R - 7.0, h: 0.4, fontSize: 18, bold: true, color: SLATE });
	text(s, [
		{ text: "Pitch: 94 mm → 14 mm", options: { breakLine: true } },
		{ text: "Sway: 81 mm → 17 mm", options: {} },
	], { x: 7.0, y: TOP + 1.05, w: R - 7.0, h: 1.4, fontSize: 32, bold: true, color: NAVY, paraSpaceAfter: 10 });
	s.addNotes(NOTES.platform);
}
{
	const s = backupSlide("the rig");
	backupFigure(s, "rig.jpg", "Two Kinova Gen3 arms on a backpack (mount); Vicon cameras around the treadmill.", "Two robot arms on a backpack worn by a person on a treadmill");
	s.addNotes(NOTES.rig);
}

// ================================================================= write
(async () => {
	await pres.writeFile({ fileName: OUT });
	await applyTheme(OUT, THEME);
	const words = Object.values(SCRIPT).map(wordCount);
	const secs = words.reduce((a, b) => a + b, 0) / WPM * 60;
	console.log(`wrote ${OUT}`);
	console.log(`notes words per slide: ${words.join(", ")} | total ${words.reduce((a, b) => a + b, 0)} | ${mmss(secs)} at ${WPM} wpm`);
})().catch((e) => { console.error(e); process.exit(1); });
