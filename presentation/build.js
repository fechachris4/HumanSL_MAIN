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
const OUT = path.join(HERE, "MSc_Project_Christian_Akabueze.pptx");

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
pres.title = "Keeping a robot's hands still while its wearer moves";
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
		{ placeholder: { options: { name: "title", type: "title", x: M, y: 1.6, w: 6.3, h: 2.55, fontSize: 40, bold: true, color: WHITE, align: "left", valign: "bottom", margin: 0, fontFace: "Arial" }, text: "" } },
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
	const w = 0.42 + label.length * 0.135;
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
// First person, ~130 words/min. Each note starts with a timing cue computed from its word
// count: [slide time | running total].
const SCRIPT = {
	1: `Hi, I'm Christian Akabueze. My MSc project at Imperial, in the MUVE Lab, was about wearable robot arms: two arms on a backpack that act as an extra pair of hands. In this clip the wearer twists his torso about sixty degrees. The elbows swing, but the hands stay close to where they were. I'll show you how it works, how well it works on real people, and what limits it.`,
	2: `Here's the rig. Two Kinova Gen3 arms, each with seven joints, are mounted on a backpack, which I'll call the mount. Vicon motion-capture cameras track the backpack in the room. The problem is walking. Every step moves the backpack, and everything attached to it. If the arms were simply locked, the hands would move 63 to 68 millimetres RMS, at every walking speed we tried. And steps come at roughly 1.8 per second, so this is fast, repeating motion, not slow drift.`,
	3: `The goal: hold each hand fixed in the room, not on the body. This is the simulated version: four positions of the backpack in one cycle, with the motion enlarged three times so it shows in a still. Panel e averages the whole cycle. The backpack and upper arms blur, because they move. The hands stay sharp, because the controller cancels that motion. If you work in 3D, think of a camera gimbal, except the gimbal is a seven-joint arm.`,
	4: `The approach is a chain of coordinate frames. Vicon measures the backpack's pose in the room. The arm base sits at a fixed, known offset on the backpack. And the joint angles, through forward kinematics, place the hand relative to the base. Compose the three and I know where the hand is in the room, and how each joint moves it: the Jacobian. Then the law. The bracket is a PD controller on the error in the room frame: how far the hand is from its target, and how fast it is drifting. J-plus-lambda turns that into joint speeds, damped near awkward poses. The arm has a spare seventh joint, and the last term uses that spare freedom to keep the joints mid-range without moving the hand.`,
	5: `Here is one cycle, as it runs in my simulation: every 2 milliseconds, 500 times a second. One: read where the backpack is. Two: work out where the hand is in the room. Three: compute joint speeds that close the gap. Four: a safety filter keeps 13 spheres on each arm outside a cylinder around the wearer; if a command would break that, a small quadratic program swaps it for the closest safe one. Five: integrate, and send the position command to the arm.`,
	6: `A controller on real arms needs more than a control law, so I wrote the real-time software in C++. Vicon arrives at 100 hertz on its own thread and is handed over through a wait-free slot, so the loop never waits on a lock. The 500 hertz core runs the kinematics and the law, clamps and integrates, and sends commands to both arms over Kinova's cyclic interface. Planning, with GPMP2, is slow and unpredictable, so it runs on a separate thread, and every plan passes one validator first. Nothing it does can block the 2 millisecond cycle.`,
	7: `I tested the same law first in MuJoCo, a physics engine. The backpack follows a walking-like motion at 1.8 hertz. Three columns, same motion: arms locked, my controller, and the controller plus feedforward. Watch the red target sphere in the middle row. Over eight seconds, locked arms leave 31 millimetres of error. The controller brings it to 6.7. Feedforward, which tells the controller how the backpack is moving, gets it to 2.6. The numbers on the clip are running values, so they differ a little.`,
	8: `Seven participants wore the arms on an instrumented treadmill at 0.5, 1 and 1.5 metres per second: 128 walking trials, mostly one arm at a time. To be precise about who did what: these trials ran the lab's existing 400 hertz controller on the MUVE Lab machine, which estimates the backpack's pose and velocity from Vicon with a Kalman filter. It had no feedforward. The C++ code on slide 6 is my own 500 hertz controller in HumanSL_MAIN, not what ran here. The analysis is mine. On the right, one trial replayed: grey is where the hand would have gone with the arm locked; blue is where it went.`,
	9: `To score a trial, I compare where Vicon saw the hand go with where it would have gone if the arm were locked to the backpack. That locked model is accurate to within half a percent on arms that were not being controlled. The result: 71 percent of the motion removed at slow walking, 63 at normal speed, 56 at fast. About two-thirds, on real arms worn by real people. These are means over six participants. On a platform that pitched and swayed, with five people, it removed 78 percent.`,
	10: `Now the challenge. Slow sway was mostly cancelled; the bounce of each step was harder. The obvious fix: turn up the velocity gain, K-d. In a pilot with one wearer I raised it from 0.8 to 2.6. The model predicted the response near 1 hertz would fall by 38 percent. It fell by 9. And across six people, the measured response, in blue, sits on the model with no velocity term, the dotted line. So the velocity term was barely acting. More tuning would not fix that; I needed to know why.`,
	11: `I traced the velocity signal end to end. The Kalman filter's estimate lags the backpack's real motion by 36 to 46 milliseconds. The measurement is about 10 milliseconds old, and the joints take about 15 to respond. In total, the controller acts on a velocity 61 to 71 milliseconds late. A late estimate cancels the true velocity only up to a phase error; what is left is two sine of omega d over two. At 1.8 hertz and 60 milliseconds, two-thirds of the velocity is left uncancelled. With that measured delay and nothing fitted, the model matches the measured response better in six of six participants. And in simulation, past about 30 milliseconds of delay, the velocity term does worse than none.`,
	12: `I put safety before stillness: if a command would bring the arm too close to the wearer, the filter swaps it for the closest safe one, even if that tracks worse. I kept planning off the fast loop: the cycle never waits, but plans arrive later and must pass the validator. The data also showed two limits. In 12 of 128 trials the planner moved the target more than 100 millimetres, and those had the most hand motion, so that is a planner setting. And at fast walking a joint, mostly the wrist, sat at its speed limit 32 percent of the time.`,
	13: `So, the impact. On real arms worn by seven people, the controller removes about two-thirds of walking motion, with one control law carried from simulation to hardware. When tuning stopped helping, I found the cause in the data, a 61 to 71 millisecond delay, and that sets the next step: faster sensing and a better velocity estimate, then feedforward on the real arms, which in simulation, with no delay, cut the error from 6.7 to 2.6 millimetres. The lesson that transfers: measure the latency budget before touching the gains.`,
	14: `Thank you. Everything is on GitHub: results, simulation and figures in msc_project, and the C++ controller in HumanSL_MAIN. Happy to take questions.`,
};
const WPM = 130;
const wordCount = (str) => str.split(/\s+/).filter(Boolean).length;
const mmss = (sec) => `${Math.floor(sec / 60)}:${String(Math.round(sec % 60)).padStart(2, "0")}`;
const NOTES = {};
{
	let run = 0;
	for (const [k, str] of Object.entries(SCRIPT)) {
		const sec = Math.round((wordCount(str) / WPM) * 60 / 5) * 5; // nearest 5 s
		run += sec;
		NOTES[k] = `[${mmss(sec)} | ${mmss(run)}] ${str}`;
	}
}

// ================================================================= slides
// ---- 1. Title (dark)
{
	pres.addSection({ title: "Opening" });
	const s = pres.addSlide({ masterName: "TITLE_DARK", sectionTitle: "Opening" });
	s.addText("Keeping a robot's hands still while its wearer moves", { placeholder: "title" });
	text(s, [
		{ text: "Christian Akabueze", options: { fontSize: 24, bold: true, color: WHITE, breakLine: true } },
		{ text: "MSc Human and Biological Robotics", options: { fontSize: 20, color: GREY, breakLine: true } },
		{ text: "Imperial College London · MUVE Lab · 2026", options: { fontSize: 20, color: GREY } },
	], { x: M, y: 4.45, w: 6.3, h: 1.3, paraSpaceAfter: 4, objectName: "Presenter" });
	text(s, [{ text: "github.com/fechachris4/msc_project", options: { hyperlink: { url: "https://github.com/fechachris4/msc_project", tooltip: "Project repository" }, color: DATA } }],
		{ x: M, y: 6.25, w: 6.3, h: 0.35, fontSize: 16, objectName: "GitHub link" });
	const card = framedImage(s, "hw_turn.gif", { x: 7.45, y: 1.6, w: 5.28, h: 3.8 }, { pad: 0.1, align: "r", line: HEX.lt1, alt: "The wearer twists his torso; the robot hands stay close to where they were" });
	caption(s, "Wearer twists about 60°. Watch the hands, not the elbows.", card.x, card.y + card.h + 0.15, card.w, { color: GREY });
	s.addNotes(NOTES[1]);
}

// ---- 2. Problem: every step moves your extra hands
{
	pres.addSection({ title: "Problem" });
	const s = contentSlide("Problem", "PROBLEM", "Every step moves your extra hands");
	const card = framedImage(s, "rig.jpg", { x: M, y: TOP, w: 6.2, h: 4.5 }, { alt: "Two robot arms on a backpack worn by a person on a treadmill" });
	caption(s, "Two arms on a backpack (mount), tracked by Vicon cameras.", card.x, card.y + card.h + 0.14, card.w);
	const sx = 7.45, sw = R - sx;
	const stats = [
		["63–68 mm", "hand motion if the arm were locked (RMS)"],
		["~1.8 Hz", "how often steps come: fast, repeating motion"],
		["2 × 7-DoF", "Kinova Gen3 arms, seven joints each"],
	];
	stats.forEach(([big, label], i) => {
		const y = TOP + 0.05 + i * 1.6;
		text(s, big, { x: sx, y, w: sw, h: 0.85, fontSize: 54, bold: true, color: BLUE, valign: "middle", objectName: `Stat ${i + 1}` });
		text(s, label, { x: sx, y: y + 0.88, w: sw, h: 0.62, fontSize: 18, color: INK, objectName: `Stat ${i + 1} label` });
	});
	s.addNotes(NOTES[2]);
}

// ---- 3. Problem: the goal
{
	const s = contentSlide("Problem", "PROBLEM", "Goal: fix the hand in the room, not on the body");
	const img = fit("task_illustration.png", { x: M, y: TOP + 0.1, w: CW, h: 2.8 }, "c");
	s.addImage({ path: A("task_illustration.png"), ...img, objectName: "Task illustration", altText: "Four backpack positions in one cycle and the average, where the hands stay sharp" });
	caption(s, "Watch (e), the average over one cycle: the backpack blurs, the hands stay sharp. Motion enlarged 3×.", M, img.y + img.h + 0.12, CW);
	const cy = 5.3, ch = 1.5, cw = (CW - 0.3) / 2;
	roundCard(s, { x: M, y: cy, w: cw, h: ch }, GREY, "Body-fixed card");
	text(s, [
		{ text: "Fixed on the body", options: { bold: true, fontSize: 22, color: SLATE, breakLine: true } },
		{ text: "Arm locked to the backpack: the hand rides along with every step.", options: { fontSize: 18 } },
	], { x: M + 0.3, y: cy + 0.2, w: cw - 0.6, h: ch - 0.4, valign: "middle", paraSpaceAfter: 6 });
	roundCard(s, { x: M + cw + 0.3, y: cy, w: cw, h: ch }, NAVY, "World-fixed card");
	text(s, [
		{ text: "Fixed in the room: the goal", options: { bold: true, fontSize: 22, color: WHITE, breakLine: true } },
		{ text: "Controller on: each hand holds its point while the backpack moves.", options: { fontSize: 18, color: WHITE } },
	], { x: M + cw + 0.6, y: cy + 0.2, w: cw - 0.6, h: ch - 0.4, valign: "middle", paraSpaceAfter: 6 });
	s.addNotes(NOTES[3]);
}

// ---- 4. Approach: frame chain + control law
{
	pres.addSection({ title: "Approach" });
	const s = contentSlide("Approach", "APPROACH", "Measure the backpack, cancel it through the arm");
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
		["Joint angles", "forward kinematics: computed", TINT, NAVY],
	];
	edges.forEach(([a, b, fill, col], i) => {
		const x1 = xs[i] + nw + 0.08, x2 = xs[i + 1] - 0.08;
		arrow(s, x1, ny + nh / 2, x2, ny + nh / 2, NAVY, 2.25);
		const cx = (x1 + x2) / 2, cwid = 2.9;
		node(s, [
			{ text: a, options: { fontSize: 16, bold: true, color: col, breakLine: true } },
			{ text: b, options: { fontSize: 14, bold: false, color: col } },
		], { x: cx - cwid / 2, y: ny + nh + 0.2, w: cwid, h: 0.78 }, fill, col, { r: 0.12, name: `Edge ${a}` });
	});

	// Control law
	const ey = 4.18;
	roundCard(s, { x: M, y: ey, w: CW, h: 0.9 }, GREY, "Equation card");
	const t = (str, color = NAVY) => ({ text: str, options: { color } });
	const P1 = DATA, P2 = BLUE, P3 = SLATE;
	text(s, [
		t("q̇ = "),
		{ text: "J", options: { color: P2 } }, { text: "+", options: { superscript: true, color: P2 } }, { text: "λ", options: { subscript: true, color: P2 } },
		{ text: " (K", options: { color: P1 } }, { text: "p", options: { subscript: true, color: P1 } }, { text: "·e + K", options: { color: P1 } }, { text: "d", options: { subscript: true, color: P1 } }, { text: "·ė)", options: { color: P1 } },
		t("  +  "),
		{ text: "(I − J", options: { color: P3 } }, { text: "+", options: { superscript: true, color: P3 } }, { text: "J)·q̇", options: { color: P3 } }, { text: "0", options: { subscript: true, color: P3 } },
	], { x: M, y: ey, w: CW, h: 0.9, fontSize: 32, bold: true, align: "center", valign: "middle", objectName: "Control law" });
	const gl = [
		[P1, "Push the hand back: e is how far it is from its target in the room, ė how fast it drifts."],
		[P2, "Turn that into joint speeds q̇, damped (λ) so the arm stays calm near awkward poses."],
		[P3, "Use the 7th joint's spare freedom to drift toward mid-range, without moving the hand."],
	];
	const gw = (CW - 2 * 0.35) / 3;
	const heads = [
		[{ text: "K", options: {} }, { text: "p", options: { subscript: true } }, { text: "·e + K", options: {} }, { text: "d", options: { subscript: true } }, { text: "·ė", options: {} }],
		[{ text: "J", options: {} }, { text: "+", options: { superscript: true } }, { text: "λ", options: { subscript: true } }],
		[{ text: "(I − J", options: {} }, { text: "+", options: { superscript: true } }, { text: "J)·q̇", options: {} }, { text: "0", options: { subscript: true } }],
	];
	gl.forEach(([color, body], i) => {
		const x = M + i * (gw + 0.35);
		text(s, heads[i].map((r) => ({ text: r.text, options: { ...r.options, bold: true, color } })), { x, y: 5.3, w: gw, h: 0.4, fontSize: 20, objectName: `Term ${i + 1}` });
		text(s, body, { x, y: 5.72, w: gw, h: 1.1, fontSize: 18, objectName: `Gloss ${i + 1}` });
	});
	s.addNotes(NOTES[4]);
}

// ---- 5. Approach: one control cycle
{
	const s = contentSlide("Approach", "APPROACH", "One control cycle, every 2 ms");
	const card = framedImage(s, "control_loop.png", { x: M, y: TOP, w: 6.9, h: 4.4 }, { alt: "Block diagram of the control loop" });
	caption(s, "Orange dashes: feedforward, tested in simulation only.", card.x, card.y + card.h + 0.14, card.w);
	const steps = [
		"Read where the backpack is (Vicon)",
		"Work out where the hand is in the room",
		"Compute joint speeds that close the gap",
		"Safety filter: keep the arm clear of the wearer",
		"Integrate, and send the command to the arm",
	];
	const sx = 8.0, rowH = 0.92;
	steps.forEach((str, i) => {
		const y = TOP + 0.1 + i * rowH;
		numberDot(s, i + 1, sx, y + 0.06, 0.52);
		text(s, str, { x: sx + 0.75, y, w: R - sx - 0.75, h: 0.66, fontSize: 18, valign: "middle", objectName: `Step ${i + 1} text` });
	});
	s.addNotes(NOTES[5]);
}

// ---- 6. Approach: C++ real-time architecture
{
	const s = contentSlide("Approach", "APPROACH", "A fast loop that nothing can block");
	// Real-time band
	const by = TOP, bh = 2.1;
	roundCard(s, { x: M, y: by, w: CW, h: bh }, GREY, "Real-time band");
	text(s, "REAL-TIME PATH · EVERY 2 ms", { x: M + 0.25, y: by + 0.1, w: 5, h: 0.32, fontSize: 14, bold: true, color: SLATE, charSpacing: 1 });
	const ry = by + 0.5, rh = 1.4;
	const vic = { x: 0.85, y: ry, w: 1.7, h: rh };
	const slot = { x: 2.9, y: ry, w: 1.45, h: rh };
	const core = { x: 4.7, y: ry, w: 4.4, h: rh };
	const kor = { x: 9.45, y: ry, w: 1.65, h: rh };
	const arms = { x: 11.45, y: ry, w: 1.05, h: rh };
	node(s, [{ text: "Vicon thread", options: { breakLine: true } }, { text: "100 Hz", options: { bold: false } }], vic, WHITE, NAVY, { fontSize: 16, shadow: true, name: "Vicon thread" });
	node(s, [{ text: "Wait-free", options: { breakLine: true } }, { text: "slot", options: {} }], slot, WHITE, NAVY, { fontSize: 16, shadow: true, name: "Wait-free slot" });
	roundCard(s, core, NAVY, "Control core", { r: 0.1, shadow: true });
	text(s, "500 Hz control core", { x: core.x, y: core.y + 0.1, w: core.w, h: 0.4, fontSize: 18, bold: true, color: WHITE, align: "center" });
	const chips = ["Kinematics", "Control law", "Clamp + integrate"];
	const cpw = 1.22, cgap = (core.w - 0.3 - 3 * cpw) / 2;
	chips.forEach((c, i) => {
		const x = core.x + 0.15 + i * (cpw + cgap);
		node(s, c, { x, y: core.y + 0.6, w: cpw, h: 0.62 }, TINT, NAVY, { fontSize: 15, name: `Core ${c}` });
		if (i < 2) arrow(s, x + cpw + 0.03, core.y + 0.91, x + cpw + cgap - 0.03, core.y + 0.91, WHITE, 1.75);
	});
	node(s, [{ text: "Kortex cyclic", options: { breakLine: true } }, { text: "exchange", options: {} }], kor, WHITE, NAVY, { fontSize: 16, shadow: true, name: "Kortex" });
	node(s, [{ text: "Both", options: { breakLine: true } }, { text: "arms", options: {} }], arms, WHITE, NAVY, { fontSize: 16, shadow: true, name: "Arms" });
	const my = ry + rh / 2;
	[[vic, slot], [slot, core], [core, kor], [kor, arms]].forEach(([a, b]) => arrow(s, a.x + a.w + 0.04, my, b.x - 0.04, my, NAVY, 2.25));

	// Planner worker
	const py = 4.6, ph = 1.6;
	const pl = { x: 3.0, y: py, w: 7.45, h: ph };
	roundCard(s, pl, WHITE, "Planner thread", { r: 0.12, line: { color: HEX.accent6, width: 1.25, dashType: "dash" } });
	text(s, "PLANNER WORKER THREAD · NOT REAL-TIME", { x: pl.x + 0.25, y: py + ph - 0.44, w: 6, h: 0.32, fontSize: 14, bold: true, color: SLATE, charSpacing: 1 });
	const pcs = [["GPMP2 planner", 3.2], ["Validator", 5.65], ["Trajectory mailbox", 8.1]];
	const pcw = 2.15, pcy = py + 0.22, pch = 0.78;
	pcs.forEach(([c, x], i) => {
		node(s, c, { x, y: pcy, w: pcw, h: pch }, TINT, NAVY, { fontSize: 16, name: `Planner ${c}` });
		if (i < 2) arrow(s, x + pcw + 0.04, pcy + pch / 2, pcs[i + 1][1] - 0.04, pcy + pch / 2, NAVY, 2);
	});
	// Request down, trajectory up
	const coreBottom = core.y + core.h;
	arrow(s, 4.95, coreBottom + 0.04, 4.95, pcy - 0.04, NAVY, 2);
	text(s, "plan request", { x: 3.2, y: 4.13, w: 1.62, h: 0.32, fontSize: 14, color: SLATE, align: "right" });
	arrow(s, 8.85, pcy - 0.04, 8.85, coreBottom + 0.04, NAVY, 2);
	text(s, "checked trajectory", { x: 9.0, y: 4.13, w: 2.2, h: 0.32, fontSize: 14, color: SLATE });
	node(s, [{ text: "Never blocks the", options: { breakLine: true } }, { text: "2 ms cycle", options: {} }], { x: 10.75, y: pcy - 0.11, w: R - 10.75, h: 1.0 }, BLUE, WHITE, { fontSize: 16, r: 0.14, name: "Never blocks callout" });
	caption(s, "My C++ controller (HumanSL_MAIN). Only the runtime can reach the arms; a MuJoCo twin runs the same core.", M, 6.4, CW);
	s.addNotes(NOTES[6]);
}

// ---- 7. Solution: simulation
{
	pres.addSection({ title: "Solution" });
	const s = contentSlide("Solution", "SOLUTION", "Simulation first: same law, physics engine");
	const card = framedImage(s, "hold_pose.gif", { x: M, y: TOP, w: 7.55, h: 4.4 }, { alt: "Simulation: arms locked, reactive controller, controller with feedforward" });
	caption(s, "Same motion (1.8 Hz), three controllers. Watch the red target, middle row.", card.x, card.y + card.h + 0.14, card.w);
	const sx = 8.75, sw = R - sx;
	text(s, "Hand error over 8 s (RMS)", { x: sx, y: TOP, w: sw, h: 0.4, fontSize: 18, bold: true, color: SLATE });
	const rows = [["31.0 mm", "Arms locked", SLATE], ["6.7 mm", "Controller", DATA], ["2.6 mm", "+ feedforward", ORANGE]];
	rows.forEach(([big, label, color], i) => {
		const y = TOP + 0.5 + i * 1.33;
		text(s, big, { x: sx, y, w: sw, h: 0.8, fontSize: 54, bold: true, color, valign: "middle", objectName: `Sim stat ${i + 1}` });
		text(s, label, { x: sx, y: y + 0.8, w: sw, h: 0.38, fontSize: 18, objectName: `Sim stat ${i + 1} label` });
	});
	s.addNotes(NOTES[7]);
}

// ---- 8. Solution: hardware trials
{
	const s = contentSlide("Solution", "SOLUTION", "Seven people wore the arms on a treadmill");
	const walk = framedImage(s, "hw_walk.gif", { x: M, y: TOP, w: 2.6, h: 4.3 }, { alt: "A participant walking on the treadmill with both arms on" });
	caption(s, "A participant walking, both arms on.", walk.x, walk.y + walk.h + 0.12, walk.w, { h: 0.6 });
	const rx = 3.55;
	const rep = framedImage(s, "hw_replay.gif", { x: rx, y: TOP, w: R - rx, h: 3.95 }, { alt: "One trial replayed: locked-arm path in grey, measured hand in blue" });
	caption(s, "Grey: where the hand would go if the arm were locked. Blue: where it went.", rep.x, rep.y + rep.h + 0.12, rep.w);
	const chips = [["7 people", 1.45], ["0.5, 1.0, 1.5 m/s", 2.35], ["128 walking trials", 2.35], ["mostly one arm at a time", 2.85]];
	const total = chips.reduce((a, c) => a + c[1], 0);
	const cg = (rep.w - total) / (chips.length - 1);
	let cx = rep.x;
	chips.forEach(([c, w]) => {
		node(s, c, { x: cx, y: 6.4, w, h: 0.45 }, GREY, NAVY, { fontSize: 16, r: 0.22, name: `Chip ${c}` });
		cx += w + cg;
	});
	s.addNotes(NOTES[8]);
}

// ---- 9. Impact: results chart
{
	pres.addSection({ title: "Results" });
	const s = contentSlide("Results", "IMPACT", "About two-thirds of walking motion removed");
	s.addChart(pres.charts.BAR, [{ name: "Motion removed", labels: ["Slow, 0.5 m/s", "Normal, 1.0 m/s", "Fast, 1.5 m/s"], values: [71, 63, 56] }], {
		x: M, y: TOP, w: 7.4, h: 4.35, objectName: "Results chart", altText: "Bar chart: 71, 63 and 56 percent of backpack motion removed at 0.5, 1.0 and 1.5 m/s",
		barDir: "col", barGapWidthPct: 55, chartColors: [HEX.accent2],
		showTitle: true, title: "Backpack motion removed while walking", titleFontSize: 18, titleColor: HEX.accent4, titleFontFace: "+mn-lt", titleBold: true,
		showValue: true, dataLabelPosition: "outEnd", dataLabelFormatCode: '0"%"', dataLabelFontSize: 28, dataLabelFontBold: true, dataLabelColor: HEX.dk2, dataLabelFontFace: "+mn-lt",
		catAxisLabelFontSize: 18, catAxisLabelColor: HEX.dk1, catAxisLabelFontFace: "+mn-lt", catAxisLineShow: true, catAxisLineColor: HEX.accent6,
		valAxisHidden: true, valAxisMinVal: 0, valAxisMaxVal: 100, valGridLine: { style: "none" }, catGridLine: { style: "none" },
		showLegend: false,
	});
	caption(s, "Removed = 1 − (hand motion ÷ motion if the arm were locked), from Vicon. Mean of 6 people; 4 at 1.5 m/s.", M, 6.45, CW);
	const cx = 8.45, cwid = R - cx;
	roundCard(s, { x: cx, y: TOP, w: cwid, h: 4.35 }, NAVY, "Platform card", { shadow: true });
	text(s, "78%", { x: cx + 0.35, y: TOP + 0.3, w: cwid - 0.7, h: 1.05, fontSize: 66, bold: true, color: WHITE, valign: "middle", objectName: "Platform stat" });
	text(s, "removed on a platform that pitched and swayed (5 people)", { x: cx + 0.35, y: TOP + 1.4, w: cwid - 0.7, h: 0.95, fontSize: 18, color: WHITE });
	text(s, "Locked arm → hand, RMS", { x: cx + 0.35, y: TOP + 2.55, w: cwid - 0.7, h: 0.35, fontSize: 16, bold: true, color: GREY, charSpacing: 0.5 });
	text(s, [
		{ text: "Pitch: 94 mm → 14 mm", options: { breakLine: true } },
		{ text: "Sway: 81 mm → 17 mm", options: {} },
	], { x: cx + 0.35, y: TOP + 2.95, w: cwid - 0.7, h: 1.0, fontSize: 20, color: WHITE, paraSpaceAfter: 6 });
	s.addNotes(NOTES[9]);
}

// ---- 10. Challenge: gain barely helped
{
	pres.addSection({ title: "Challenge" });
	const s = contentSlide("Challenge", "CHALLENGE", "Turning up the gain barely helped");
	const bw = 2.95, bh = 1.95;
	[["38%", "predicted drop near 1 Hz", SLATE], ["9%", "measured drop near 1 Hz", ORANGE]].forEach(([big, label, color], i) => {
		const x = M + i * (bw + 0.3);
		roundCard(s, { x, y: TOP, w: bw, h: bh }, GREY, `Compare ${i + 1}`);
		text(s, big, { x: x + 0.3, y: TOP + 0.15, w: bw - 0.6, h: 1.05, fontSize: 60, bold: true, color, valign: "middle" });
		text(s, label, { x: x + 0.3, y: TOP + 1.22, w: bw - 0.6, h: 0.6, fontSize: 18 });
	});
	text(s, [
		{ text: "Pilot, one wearer: velocity gain K", options: {} }, { text: "d", options: { subscript: true } }, { text: " raised from 0.8 to 2.6.", options: { breakLine: true } },
		{ text: "Measured response (blue) sits on the model with no velocity term (dotted).", options: { breakLine: true } },
		{ text: "The velocity term was barely acting. Why?", options: { bold: true, color: NAVY } },
	], { x: M, y: 4.2, w: 6.25, h: 2.6, fontSize: 20, paraSpaceAfter: 14, objectName: "Gain finding" });
	const fig = framedImage(s, "hw_rejection_bc.png", { x: 7.3, y: TOP, w: R - 7.3, h: 4.45 }, { align: "r", alt: "Frequency response at 1.0 m/s: measured, model with logged gains, model with no velocity term, model with late velocity" });
	caption(s, "At 1.0 m/s, 6 people. Lower = more motion removed.", fig.x, fig.y + fig.h + 0.14, fig.w);
	s.addNotes(NOTES[10]);
}

// ---- 11. Challenge: the delay
{
	const s = contentSlide("Challenge", "CHALLENGE", "The cause: the velocity arrives 61–71 ms late");
	// Delay budget, proportional boxes (1 ms = k inches). Kalman lag drawn as 36 ms solid + 10 ms range.
	const bx = M, by = TOP + 0.15, bh = 0.75, k = 9.0 / 71;
	const segs = [
		{ ms: 36, fill: ORANGE, label: "Kalman filter lag 36–46 ms" },
		{ ms: 10, fill: ORANGE, transp: 55, label: "range", labelColor: INK },
		{ ms: 10, fill: BLUE, label: "~10 ms" },
		{ ms: 15, fill: DATA, label: "~15 ms" },
	];
	let x = bx;
	segs.forEach((sg, i) => {
		const w = sg.ms * k;
		s.addShape(pres.shapes.RECTANGLE, { objectName: `Delay segment ${i + 1}`, x, y: by, w, h: bh, fill: { color: sg.fill, ...(sg.transp ? { transparency: sg.transp } : {}) }, line: { color: WHITE, width: 1.5 } });
		if (sg.label) text(s, sg.label, { x, y: by, w, h: bh, fontSize: 18, bold: true, color: sg.labelColor || WHITE, align: "center", valign: "middle" });
		x += w;
	});
	text(s, "the velocity estimate trails the backpack's motion", { x: bx, y: by + bh + 0.1, w: 46 * k, h: 0.35, fontSize: 16, color: SLATE });
	text(s, "sensing age", { x: bx + 46 * k, y: by + bh + 0.1, w: 10 * k + 0.2, h: 0.35, fontSize: 16, color: SLATE, align: "center" });
	text(s, "joint response", { x: bx + 56 * k, y: by + bh + 0.1, w: 15 * k, h: 0.35, fontSize: 16, color: SLATE, align: "center" });
	text(s, "= 61–71 ms", { x: bx + 71 * k + 0.25, y: by - 0.08, w: R - (bx + 71 * k + 0.25), h: 0.9, fontSize: 36, bold: true, color: ORANGE, valign: "middle", objectName: "Delay total" });
	text(s, "late, end to end", { x: bx + 71 * k + 0.25, y: by + bh + 0.1, w: R - (bx + 71 * k + 0.25), h: 0.35, fontSize: 16, color: SLATE });

	// Formula card
	const fy = 3.75, fw = 5.55, fh = 3.1;
	roundCard(s, { x: M, y: fy, w: fw, h: fh }, GREY, "Formula card");
	text(s, [
		{ text: "residual = 2·sin(", options: {} }, { text: "ω", options: { italic: true } }, { text: "d", options: { italic: true } }, { text: "/2)", options: {} },
	], { x: M + 0.3, y: fy + 0.22, w: fw - 0.6, h: 0.6, fontSize: 32, bold: true, color: NAVY, objectName: "Delay formula" });
	text(s, "ω: step frequency · d: delay", { x: M + 0.3, y: fy + 0.85, w: fw - 0.6, h: 0.32, fontSize: 16, color: SLATE });
	text(s, [
		{ text: "At 1.8 Hz and 60 ms, ", options: {} },
		{ text: "two-thirds", options: { bold: true, color: ORANGE } },
		{ text: " of the velocity is left uncancelled.", options: {} },
	], { x: M + 0.3, y: fy + 1.3, w: fw - 0.6, h: 0.75, fontSize: 20, objectName: "Formula gloss" });
	text(s, "With this delay and nothing fitted, the model fits the hardware better in 6 of 6 people.", { x: M + 0.3, y: fy + 2.1, w: fw - 0.6, h: 0.85, fontSize: 18, color: INK });

	const ix = M + fw + 0.35;
	const fig = framedImage(s, "velocity_delay.png", { x: ix, y: fy, w: R - ix, h: 2.75 }, { align: "r", alt: "Simulation: hand error against delay on the velocity signal" });
	caption(s, "Simulation, delay added on purpose. Grey band: the hardware's delay.", fig.x, fig.y + fig.h + 0.14, fig.w);
	s.addNotes(NOTES[11]);
}

// ---- 12. Trade-offs
{
	pres.addSection({ title: "Trade-offs" });
	const s = contentSlide("Trade-offs", "TRADE-OFFS", "Trade-offs I made, and limits I found");
	const cw = (CW - 0.3) / 2, top = TOP + 0.4, gapY = 0.25, chh = (BOTTOM - top - gapY) / 2;
	text(s, "CHOICES I MADE", { x: M, y: TOP, w: cw, h: 0.32, fontSize: 14, bold: true, color: SLATE, charSpacing: 1 });
	text(s, "LIMITS THE DATA SHOWED", { x: M + cw + 0.3, y: TOP, w: cw, h: 0.32, fontSize: 14, bold: true, color: SLATE, charSpacing: 1 });
	const cards = [
		["Safety before stillness", "Gain:", "the arm stays outside a cylinder around the wearer.", "Cost:", "tracking gives way near the wearer."],
		["Re-planning moved the target", "Seen:", "12 of 128 trials moved the reference >\u00A0100\u00A0mm; those had the most hand motion.", "Means:", "a planner setting, not the control law."],
		["Planning kept off the fast loop", "Gain:", "GPMP2 runs on its own thread; the 2 ms cycle never waits.", "Cost:", "plans arrive later, once validated."],
		["Wrist at its speed limit", "Seen:", "at 1.5 m/s a joint was at its limit 32% of the time (~10% slower).", "Means:", "fast walking outruns the wrist."],
	];
	cards.forEach(([title, l1, b1, l2, b2], i) => {
		const col = i % 2, row = Math.floor(i / 2);
		const x = M + col * (cw + 0.3), y = top + row * (chh + gapY);
		roundCard(s, { x, y, w: cw, h: chh }, GREY, `Card ${title}`);
		numberDot(s, i + 1, x + 0.3, y + 0.22, 0.46, col === 0 ? BLUE : DATA);
		text(s, title, { x: x + 0.95, y: y + 0.2, w: cw - 1.2, h: 0.5, fontSize: 22, bold: true, color: NAVY, valign: "middle" });
		text(s, [
			{ text: `${l1} `, options: { bold: true } }, { text: b1, options: { breakLine: true } },
			{ text: `${l2} `, options: { bold: true } }, { text: b2, options: {} },
		], { x: x + 0.3, y: y + 0.82, w: cw - 0.6, h: chh - 0.95, fontSize: 18, paraSpaceAfter: 6 });
	});
	s.addNotes(NOTES[12]);
}

// ---- 13. Impact and next
{
	pres.addSection({ title: "Impact" });
	const s = contentSlide("Impact", "IMPACT", "Impact, and what comes next");
	const lw = 4.4, h = BOTTOM - TOP;
	roundCard(s, { x: M, y: TOP, w: lw, h: 4.55 }, NAVY, "Impact card", { shadow: true });
	text(s, "56–71%", { x: M + 0.35, y: TOP + 0.3, w: lw - 0.7, h: 0.95, fontSize: 54, bold: true, color: WHITE, valign: "middle", objectName: "Impact stat" });
	text(s, "of walking motion removed, on real arms worn by 7 people", { x: M + 0.35, y: TOP + 1.3, w: lw - 0.7, h: 0.95, fontSize: 18, color: WHITE });
	text(s, [
		{ text: "One control law, simulation to hardware", options: { bullet: true, breakLine: true } },
		{ text: "Limit traced to a 61–71 ms delay, from data, not guesswork", options: { bullet: true } },
	], { x: M + 0.35, y: TOP + 2.45, w: lw - 0.7, h: 2.2, fontSize: 18, color: WHITE, paraSpaceAfter: 10 });

	const colX = [M + lw + 0.45, 0], colW = (R - (M + lw + 0.45) - 0.4) / 2;
	colX[1] = colX[0] + colW + 0.4;
	const cols = [
		["What comes next", [
			"Faster motion sensing and velocity estimate",
			"Feedforward on the real arms (sim: 6.7 → 2.6 mm)",
			"Both arms at once",
			"Predictive control (MPC) that compensates the delay",
			"Replay recorded walking in simulation",
		]],
		["What transfers", [
			"Measure the latency budget before tuning gains",
			"Frame chains: the same rigid-body maths as camera tracking",
			"Keep slow, unpredictable work off the real-time loop",
		]],
	];
	cols.forEach(([head, items], i) => {
		text(s, head, { x: colX[i], y: TOP + 0.05, w: colW, h: 0.5, fontSize: 22, bold: true, color: NAVY, objectName: `Column ${head}` });
		text(s, items.map((t, j) => ({ text: t, options: { bullet: true, breakLine: j < items.length - 1 } })), { x: colX[i], y: TOP + 0.7, w: colW, h: h - 0.75, fontSize: 18, paraSpaceAfter: 10 });
	});
	s.addNotes(NOTES[13]);
}

// ---- 14. Close (dark)
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
	s.addNotes(NOTES[14]);
}

// ================================================================= write
(async () => {
	await pres.writeFile({ fileName: OUT });
	await applyTheme(OUT, THEME);
	const words = Object.values(SCRIPT).map(wordCount);
	console.log(`wrote ${OUT}`);
	console.log(`notes words per slide: ${words.join(", ")} | total ${words.reduce((a, b) => a + b, 0)}`);
})().catch((e) => { console.error(e); process.exit(1); });
