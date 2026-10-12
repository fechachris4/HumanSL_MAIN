// Generator for World_Stable_Supernumerary_Effectors_Akabueze.pptx (10-minute talk, 10 slides:
// title, 8 content, closing; no backups). Speaker notes follow Christian's own script
// (12 Oct 2026), merged to 10 slides and de-jargoned.
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
// Christian's script (12 Oct 2026), merged to ten slides and de-jargoned; his "Check:" and
// "Optional" lines are excluded. Paced at 140 words/min, his rate. Each note starts with a cue
// computed from its word count: [slide time | running total].
const SCRIPT = {
	title: `Imagine wearing two extra robot arms on your back and asking them to hold something still while you walk. Every step shakes them. I'm Christian Akabueze, and this was my MSc project at Imperial, in the MUVE Lab. In this clip the wearer twists about sixty degrees. The elbows swing; the hands barely move. It works, mostly. This talk is about the part that didn't, and why the obvious fix did almost nothing.`,
	problem: `The arms are two Kinova Gen3 robots, seven joints each, on a backpack. Motion-capture cameras around the room, a Vicon system, track the backpack. An arm you wear is an extra pair of hands, until you take a step: anything it holds moves with you. With the arms locked, the hands move 63 to 68 millimetres on average, at every walking speed we tried. Steps come about 1.8 times a second, so this is fast, repeating motion, not slow drift. Remember that number, 1.8 steps a second. It comes back. So the goal is to hold each hand fixed in the room, not on the body.`,
	frames: `The idea is a chain of coordinate frames. The cameras give me where the backpack is in the room. The arm's base sits at a fixed, known spot on the backpack. The joint angles then place the hand relative to that base. Chain the three and I know where the hand is in the room, and how each joint moves the hand. On top sits a feedback controller on the error in the room: how far the hand is from its spot, and how fast it's drifting. That gets turned into joint speeds, staying smooth near awkward poses. The same law runs in my simulation, my C++ controller, and the lab controller used in the trials.`,
	sim: `Before hardware, I tested my Python controller in MuJoCo, a physics engine, with the backpack following a walking-like motion at 1.8 steps a second. Same motion three times. Arms locked: 31 millimetres of error. My controller: 6.7. Add feedforward, which tells the controller how the backpack is moving: 2.6. So knowing the backpack's velocity is worth a lot. Hold on to that, because on real hardware, getting that velocity on time turns out to be the hard part. The sim is stiffer than the rig, so read trends, not millimetres.`,
	filter: `The first problem on real hardware wasn't accuracy. It was shaking. The camera signal is noisy, and fed straight in, the arm vibrated. Unusable. So I added a Kalman filter, a smoothing filter, to estimate the backpack's position and speed cleanly, and the shaking stopped. But a filter buys smoothness with time. Its estimate comes out late, so the position built from it is late too. So I fight the delay with the speed itself: if I know how fast the backpack is moving, I can predict its position a short step into the future and cancel part of the lag. The filter pushes me into the past; the prediction pulls me back toward now.`,
	hardware: `Seven participants wore the arms on a treadmill at 0.5, 1 and 1.5 metres per second. The controller ran 400 times a second on the lab machine, with no feedforward; the trial analysis is mine. To score a trial, I replay what a locked arm would have done and compare it with the real hand: the percent removed is how much less the hand moved than the locked arm would have. On arms that weren't being controlled, that locked model is right to within half a percent. Averaged over six of the seven people: 71 percent removed at slow walking, 63 at normal, 56 at fast, across 128 walking trials. On a pitching, swaying platform, 78 percent. But notice the trend. The faster you walk, the less it removes.`,
	gain: `So where does the missing third go? I split the motion by how fast it repeats. Slow sway, about once every four seconds: 85 percent cancelled. The bounce of each step, twice a second: only 29. That's a speed problem, and the textbook answer is to react harder to velocity. Turn up the velocity gain. In a pilot with one wearer, I raised it from 0.8 to 2.6, more than three times. My model said the response near the stride rate would drop by 38 percent. It dropped by 9. And across six people, the arm behaves like a model with no velocity term at all, as if the velocity wasn't there. That's when I stopped tuning and started asking why.`,
	delay: `I traced the velocity signal end to end. Remember the prediction: it uses the velocity to pull the position back toward now. But nothing pulls the velocity itself forward. The smoothing filter's velocity estimate trails the real motion by 36 to 46 milliseconds. The camera measurement is about 10 milliseconds old, and the joints take about 15 to respond. In total, the velocity is 61 to 71 milliseconds late. A late velocity has the right size but the wrong timing, so subtracting it doesn't cancel the real one. The leftover is two times the sine of half the timing error, measured as an angle of the step cycle. At 1.8 steps a second and 60 milliseconds, that's two-thirds of the velocity, wrong. And it explains the trend: walk faster, steps come quicker, and the same delay costs more. With the measured delay and nothing fitted, the full model matches the hardware better than the no-velocity model in six of six people. Confirming it directly means changing the speed estimate on hardware. That's the next experiment.`,
	impact: `On real arms worn by seven people, the controller removes about two-thirds of walking motion: 56 to 71 percent, depending on speed. The part I'm proudest of is the diagnosis. When tuning stopped helping, I traced it in the data to a 61 to 71 millisecond delay. That sets the order of what comes next. First, a faster velocity estimate. Only then feedforward: in simulation with no delay it cut the error from 6.7 to 2.6 millimetres, but at 60 milliseconds it loses that advantage, to around 8. I also wrote my own real-time controller in C++ for this rig, built so the path planner can never block the 2 millisecond control loop. With what I know now, I can see the same weak spot there: its own smoothing filter adds about 45 milliseconds of lag at step frequency, so the speed estimate is the next piece there too. The lesson I'd carry to any robot that sees through sensors: measure every delay in the chain before you tune the gains.`,
	close: `Thank you. The code and results are on GitHub; the links are on the slide. Happy to take questions.`,
};
const WPM = 140;
const wordCount = (str) => str.split(/\s+/).filter(Boolean).length;
const mmss = (sec) => { const t = Math.round(sec); return `${Math.floor(t / 60)}:${String(t % 60).padStart(2, "0")}`; };
const NOTES = {};
{
	// The running total is rounded to the nearest 5 s, and each slide's cue is the difference of
	// consecutive rounded totals, so the per-slide cues always add up to the running total.
	let run = 0, prev = 0;
	const r5 = (x) => Math.round(x / 5) * 5;
	for (const [k, str] of Object.entries(SCRIPT)) {
		run += (wordCount(str) / WPM) * 60;
		const cum = r5(run);
		NOTES[k] = `[${mmss(cum - prev)} | ${mmss(cum)}] ${str}`;
		prev = cum;
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
	stat(s, "63–68\u00A0mm", "hand motion, arm locked (average)", sx, y0, sw, o);
	stat(s, "~1.8\u00A0Hz", "steps per second", sx, y0 + pitch, sw, o);
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
		["joint angles", TINT, NAVY],
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
	// Only the PD term is shown; the null-space term (joints toward mid-range) is in the notes.
	const P1 = DATA, P2 = BLUE;
	text(s, [
		r("q̇ = ", NAVY),
		r("J", P2), r("+", P2, { superscript: true }), r("λ", P2, { subscript: true }),
		r(" (K", P1), r("p", P1, { subscript: true }), r("·e + ", P1),
		r("K", ORANGE), r("d", ORANGE, { subscript: true }), r("·ė", ORANGE),
		r(")", P1),
	], { x: M, y: ey, w: CW, h: eh, fontSize: 44, bold: true, align: "center", valign: "middle", objectName: "Control law" });

	// One gloss line under the band. The orange marker sits under Kd·ė; its x was measured on the
	// rendered equation (Arial / Liberation Sans metrics), the gloss text is anchored to it.
	const KD_X = 8.61; // centre of "Kd·ė" in the 44 pt centred equation, inches from the left edge
	const gy = ey + eh + 0.2, gh = 0.4, tri = 0.2;
	s.addShape(pres.shapes.ISOSCELES_TRIANGLE, { objectName: "Velocity-term marker", x: KD_X - tri / 2, y: gy + 0.08, w: tri, h: 0.17, fill: { color: ORANGE }, line: { type: "none" } });
	text(s, "velocity term", { x: KD_X + tri / 2 + 0.1, y: gy, w: 2.4, h: gh, fontSize: 20, color: ORANGE, valign: "middle", objectName: "Velocity-term label" });
	text(s, "error in the room → joint speeds", { x: M, y: gy, w: KD_X - tri / 2 - 0.55 - M, h: gh, fontSize: 20, color: SLATE, align: "right", valign: "middle", objectName: "Control-law gloss" });
	s.addNotes(NOTES.frames);
}

// ---- 4. Approach & solution: simulation
{
	const s = contentSlide("Approach & solution", TAG.AS, "Simulation: knowing the velocity is worth a lot");
	const card = framedImage(s, "hold_pose.gif", { x: M, y: TOP, w: 8.4, h: BOTTOM - TOP }, { alt: "Simulation: arms locked, reactive controller, reactive controller with feedforward" });
	const sx = card.x + card.w + 0.45, sw = R - sx;
	text(s, "Hand error, 8 s average", { x: sx, y: TOP, w: sw, h: 0.4, fontSize: 18, bold: true, color: SLATE, objectName: "Sim header" });
	const o = { bigPt: 54, bigH: 0.85, labelPt: 20, labelH: 0.4, gap: 0.02 };
	[["31.0\u00A0mm", "arms locked", SLATE], ["6.7\u00A0mm", "my controller", DATA], ["2.6\u00A0mm", "+ speed fed forward", ORANGE]].forEach(([big, label, color], i) => {
		stat(s, big, label, sx, TOP + 0.6 + i * 1.5, sw, { ...o, color });
	});
	s.addNotes(NOTES.sim);
}

// ---- 5. Approach & solution: the filter that costs time (Christian's script, slide 6)
{
	const s = contentSlide("Approach & solution", TAG.AS, "Making it usable: a filter that costs time");
	const steps = [
		["Raw camera tracking", "noisy, arm shakes", GREY, NAVY],
		["Smoothing filter (Kalman)", "steady, but late", ORANGE, WHITE],
		["Predict ahead from speed", "pulls back toward now", BLUE, WHITE],
	];
	const nw = 3.5, nh = 2.0, ny = 2.7, gap = (CW - 3 * nw) / 2;
	steps.forEach(([a, b, fill, col], i) => {
		const x = M + i * (nw + gap);
		node(s, [r(a, col, { breakLine: true, bold: true, fontSize: 22 }), r(b, col, { fontSize: 20 })], { x, y: ny, w: nw, h: nh }, fill, col, { r: 0.14, bold: false, shadow: true, name: `Filter step ${i + 1}` });
		if (i < 2) arrow(s, x + nw + 0.08, ny + nh / 2, x + nw + gap - 0.08, ny + nh / 2, NAVY, 2.5);
	});
	text(s, "Smoothness costs time.", { x: M, y: 5.35, w: CW, h: 0.8, fontSize: 36, bold: true, color: NAVY, align: "center", valign: "middle", objectName: "Takeaway" });
	s.addNotes(NOTES.filter);
}

// ---- 6. Approach & solution: hardware result
{
	const s = contentSlide("Approach & solution", TAG.AS, "On real wearers: about 2/3 of walking motion removed");
	text(s, "Controller: 400 Hz on the lab machine, no feedforward. Analysis: mine.", { x: M, y: 1.85, w: CW, h: 0.4, fontSize: 20, color: NAVY, objectName: "Provenance" });
	// The stat column (three rows + the n note) and the replay card share one vertical span.
	const rep = framedImage(s, "hw_replay.gif", { x: M, y: 2.75, w: 8.7, h: 4.0 }, { alt: "One trial replayed: locked-arm path in grey, measured hand in blue" });
	const sx = rep.x + rep.w + 0.4, sw = R - sx;
	const o = { bigPt: 60, bigH: 0.85, labelPt: 20, labelH: 0.38, gap: 0.0, color: DATA };
	const y0 = 2.5, pitch = 1.22;
	[["71%", "0.5 m/s"], ["63%", "1.0 m/s"], ["56%", "1.5 m/s"]].forEach(([big, label], i) => stat(s, big, label, sx, y0 + i * pitch, sw, o));
	text(s, "Mean of 6 of 7 wearers · 128 trials", { x: sx, y: y0 + 2 * pitch + o.bigH + o.labelH + 0.02, w: sw, h: 0.62, fontSize: 18, color: SLATE, objectName: "Results note" });
	s.addNotes(NOTES.hardware);
}

// ---- 7. Trade-offs & challenges: the obvious fix
{
	pres.addSection({ title: "Trade-offs & challenges" });
	// The title carries the question; the set-up line and the number pair sit centred below it.
	const s = contentSlide("Trade-offs & challenges", TAG.TC, "The obvious fix did almost nothing");
	text(s, "One-wearer pilot: velocity gain 0.8 → 2.6. Hand motion near 1 Hz fell:",
		{ x: M, y: 2.45, w: CW, h: 0.5, fontSize: 22, color: SLATE, align: "center", valign: "middle", objectName: "Pilot set-up" });
	const colW = 4.4, cxs = [W / 2 - 2.75, W / 2 + 2.75];
	[["38%", "predicted", SLATE], ["9%", "measured", ORANGE]].forEach(([big, label, color], i) => {
		stat(s, big, label, cxs[i] - colW / 2, 3.05, colW, { bigPt: 130, bigH: 1.9, labelPt: 24, labelH: 0.5, gap: 0.02, color, align: "center" });
	});
	text(s, "Slow sway: 85% removed · Step bounce: only 29%", { x: M, y: 6.05, w: CW, h: 0.5, fontSize: 22, color: SLATE, align: "center", valign: "middle", objectName: "Frequency split" });
	s.addNotes(NOTES.gain);
}

// ---- 8. Trade-offs & challenges: the delay
{
	const s = contentSlide("Trade-offs & challenges", TAG.TC, "The velocity arrives 61–71 ms late");
	// Delay budget, proportional boxes in orange shades. Kalman lag: 36 ms solid + a fading tail to 46 ms.
	const bx = M, by = 2.3, bh = 1.3, barW = 8.75, k = barW / 71;
	const kw = 46 * k;
	s.addShape(pres.shapes.RECTANGLE, { objectName: "Kalman lag (36 ms)", x: bx, y: by, w: 36 * k + 0.02, h: bh, fill: { color: ORANGE }, line: { type: "none" } });
	// 36 -> 46 ms: pptxgenjs has no gradient fills, so the tail is native opaque slices that step from
	// full orange to 35 % orange over white. Shapes (not an image) share the bar's exact top and bottom
	// edges; each slice overlaps the next slightly so no seams show.
	const FADE_N = 36, fadeX = bx + 36 * k, sliceW = (10 * k) / FADE_N;
	const mix = (t) => { // t = 0: orange D24000; t = 1: white
		const c = [0xd2, 0x40, 0x00].map((o) => Math.round(o + (255 - o) * t));
		return c.map((v) => v.toString(16).padStart(2, "0")).join("").toUpperCase();
	};
	for (let i = 0; i < FADE_N; i++) {
		const t = ((i + 0.5) / FADE_N) * 0.65;
		s.addShape(pres.shapes.RECTANGLE, { objectName: `Kalman lag fade ${i + 1}`, x: fadeX + i * sliceW, y: by, w: sliceW + (i < FADE_N - 1 ? 0.012 : 0), h: bh, fill: { color: mix(t) }, line: { type: "none" } });
	}
	text(s, "Smoothing filter 36–46 ms", { x: bx, y: by, w: kw, h: bh, fontSize: 20, bold: true, color: WHITE, align: "center", valign: "middle", objectName: "Kalman label" });
	const segs = [
		{ ms: 10, transp: 25, runs: [r("camera", NAVY, { breakLine: true }), r("~10 ms", NAVY)] },
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
	text(s, "= 61–71\u00A0ms", { x: x + 0.22, y: by, w: R - x - 0.22, h: bh, fontSize: 40, bold: true, color: ORANGE, valign: "middle", objectName: "Delay total" });

	text(s, [
		// A d-late velocity is off by 2·sin(ωd/2) of its own size (exact for any loop); 0.68-0.78 at 61-71 ms.
		r("Velocity error: 2·sin("), r("ωd", null, { italic: true }), r("/2)"),
	], { x: M, y: 4.2, w: CW, h: 0.75, fontSize: 32, bold: true, color: NAVY, valign: "middle", objectName: "Delay formula" });
	text(s, [r("Late by 60 ms at 1.8 steps/s → "), r("two-thirds", ORANGE, { bold: true }), r(" of the speed signal is wrong")], { x: M, y: 5.0, w: CW, h: 0.5, fontSize: 24, color: INK, valign: "middle", objectName: "Formula gloss" });
	text(s, "Nothing fitted: explains the hardware better than a no-speed model, 6 of 6 wearers", { x: M, y: 5.75, w: CW, h: 0.45, fontSize: 20, color: SLATE, valign: "middle", objectName: "Evidence" });
	s.addNotes(NOTES.delay);
}

// ---- 9. Impact: what it achieved, what comes next
{
	pres.addSection({ title: "Impact" });
	const s = contentSlide("Impact", TAG.I, "What it achieved, and what comes next");
	const lw = 5.4, h = BOTTOM - TOP, ix = M + 0.4, iw = lw - 0.8;
	roundCard(s, { x: M, y: TOP, w: lw, h }, NAVY, "Impact card", { shadow: true });
	text(s, "~2/3", { x: ix, y: TOP + 0.25, w: iw, h: 1.65, fontSize: 120, bold: true, color: WHITE, valign: "middle", objectName: "Impact stat" });
	text(s, [r("walking motion removed", null, { breakLine: true }), r("on hardware")], { x: ix, y: TOP + 2.0, w: iw, h: 0.95, fontSize: 24, color: WHITE, objectName: "Impact label" });
	const fh = 1.05, fy = TOP + h - 0.3 - fh;
	roundCard(s, { x: M + 0.25, y: fy, w: lw - 0.5, h: fh }, BLUE, "Found-limit panel", { r: 0.1 });
	text(s, [r("Found the limit: ", null, { bold: true }), r("a 61–71\u00A0ms delay")], { x: M + 0.5, y: fy, w: lw - 1.0, h: fh, fontSize: 22, color: WHITE, valign: "middle", objectName: "Found the limit" });

	const tx = M + lw + 0.6, tw = R - tx;
	text(s, "Next", { x: tx, y: TOP + 0.1, w: tw, h: 0.55, fontSize: 28, bold: true, color: NAVY, objectName: "Next heading" });
	const steps = [["1", "Faster speed estimate"], ["2", [r("Then feedforward", null, { breakLine: true }), r("(sim: 6.7\u00A0→\u00A02.6\u00A0mm)", SLATE, { fontSize: 20 })]]];
	steps.forEach(([n, t], i) => {
		const y = TOP + 0.85 + i * 1.1;
		node(s, n, { x: tx, y: y + 0.05, w: 0.55, h: 0.55 }, i === 0 ? ORANGE : DATA, WHITE, { fontSize: 22, r: 0.27, name: `Next ${n}` });
		text(s, t, { x: tx + 0.8, y, w: tw - 0.8, h: i ? 0.85 : 0.65, fontSize: 24, color: INK, valign: "middle", objectName: `Next step ${n}` });
	});
	roundCard(s, { x: tx, y: TOP + h - 1.35, w: tw, h: 1.35 }, GREY, "Lesson card");
	text(s, [r("Lesson: ", NAVY, { bold: true }), r("measure delays before tuning")], { x: tx + 0.35, y: TOP + h - 1.35, w: tw - 0.7, h: 1.35, fontSize: 26, color: INK, valign: "middle", objectName: "Lesson" });
	s.addNotes(NOTES.impact);
}

// ---- 10. Close (dark)
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
		text(s, [{ text: short, options: { hyperlink: { url, tooltip: label }, color: TINT } }], { x: cxm - blockW / 2, y: 5.45, w: blockW, h: 0.38, fontSize: 18, align: "center" });
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
