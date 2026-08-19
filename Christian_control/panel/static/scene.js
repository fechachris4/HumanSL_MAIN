//
// scene.js — the panel's 3D view: both arms, the plan, and the gap between
// where the tool is and where it was asked to be.
//
// Everything here is drawing. Nothing in this file decides anything about
// motion, and nothing it draws is a measurement the controller does not
// already publish: the arms come from meas_j*/cmd_j* through the same DH
// tables the planner uses, the desired point from pd_*, the plan from a
// solved trajectory, the clearance from a plan-time validation report.
// Where a quantity is a property of the plan rather than of this instant,
// the label says so.
//
// Frame: everything is drawn in the `mount` frame, because that is the one
// frame both arms and the SDF grid share. The CSV's pd_*/p_* columns are in
// the CONTROLLED arm's own base_link (State.h, PoseReference), so they are
// carried into mount here through the mounting transform below.
//
// Rendering is a plain 2D canvas with its own perspective projection and a
// painter's-algorithm depth sort. No WebGL and no three.js: the scene is
// line art of a few hundred segments, it has to run in Safari on a MacBook
// with no build step, and a vendored 3D library is listed in the design as
// deferred. The palette is read from CSS custom properties so panel.css
// stays the one place colour is decided; the fallbacks below only matter
// when this module is opened standalone (scene.test.html).
//

const DEG = Math.PI / 180;

// Mount-frame placement of each arm's base_link.
//
// The source of truth for these two numbers is
// model/dual_arm_mounting.yaml (separation 0.075 m, tilt
// 1.2085 rad), and tests/test_dual_arm_mounting.cpp fails the build if the
// URDF ever disagrees with it. They are copied here because /api/dh carries
// only the per-joint DH table; pass options.mountFromBase to override once
// the server serves them. Read that YAML before trusting them for anything
// but drawing: it records that the two numbers are inherited from an early
// commit, not surveyed off the physical rig.
//
// A URDF <origin> composes as Trans(xyz) then Rot(rpy), and both mounting
// joints are a pure roll about mount X, mirrored between the arms.
// print_dual_arm_fk prints the same pair on its first two lines, which is
// how these were checked.
const DEFAULT_MOUNT_FROM_BASE = {
    right: { xyz: [0, -0.0375, 0], rpy: [1.2085, 0, 0] },
    left: { xyz: [0, 0.0375, 0], rpy: [-1.2085, 0, 0] },
};

// The fixed rotation between the DH chain's root and base_link. It is a
// convention, not data: PinocchioKinematicsAdapter::DhRootInBaseLink() is
// Rx(pi) with no translation, and every hand-rolled DH chain in this
// repository composes with it. Getting it wrong turns the arm upside down,
// which is at least obvious.
const DH_ROOT_ROLL_RAD = Math.PI;

// Only four colours, because the scene draws nothing abnormal: the warning
// amber and the stop red are reserved for states the panel's readouts judge,
// and a scene that reached for them would be judging too.
const PALETTE_FALLBACK = {
    ground: '#DCDFD8',
    ink: '#23282A',
    hairline: '#A9AFA6',
    ask: '#2E6E8E',
};

// ---------------------------------------------------------------
// Small linear algebra. 4x4 row-major, plain arrays of sixteen.
// ---------------------------------------------------------------

function identity() {
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function multiply(a, b) {
    const out = new Array(16);
    for (let row = 0; row < 4; row++) {
        for (let col = 0; col < 4; col++) {
            let sum = 0;
            for (let k = 0; k < 4; k++) sum += a[row * 4 + k] * b[k * 4 + col];
            out[row * 4 + col] = sum;
        }
    }
    return out;
}

function rotationX(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1];
}

function rotationY(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1];
}

function rotationZ(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function translation(x, y, z) {
    return [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
}

function transformPoint(m, p) {
    return [
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
        m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
        m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
    ];
}

// Column `index` of the rotation part: the frame's own x, y or z axis
// expressed in the parent frame. Used for the tool triad.
function axisOf(m, index) {
    return [m[index], m[4 + index], m[8 + index]];
}

function originOf(m) {
    return [m[3], m[7], m[11]];
}

function subtract(a, b) {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function add(a, b) {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scale(v, k) {
    return [v[0] * k, v[1] * k, v[2] * k];
}

function dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
}

function norm(v) {
    return Math.sqrt(dot(v, v));
}

function normalise(v) {
    const length = norm(v);
    return length > 1e-12 ? scale(v, 1 / length) : [0, 0, 0];
}

// ---------------------------------------------------------------
// Forward kinematics
// ---------------------------------------------------------------

// One joint's Denavit-Hartenberg transform:
//
//     Rz(theta + theta_offset) * Tz(d) * Tx(a) * Rx(alpha)
//
// This is the convention generate_dh_params.cpp derives the tables under
// and the one gpmp2's Arm chains: same order, same base roll. Checked
// against tools/print_dual_arm_fk on 2026-08-11 — both arms, in base_link
// and in mount, agree to within 6 micrometres, which is the floor the URDF's
// five-significant-figure rotations impose. It also reproduces the
// startup_position_m recorded in runs/2026-08-11/loop_log_right_20260811_153920.
function jointTransform(joint, thetaRad) {
    const offset = Number(joint.theta_offset) || 0;
    const d = Number(joint.d) || 0;
    const a = Number(joint.a) || 0;
    const alpha = Number(joint.alpha) || 0;
    let m = rotationZ(thetaRad + offset);
    m = multiply(m, translation(0, 0, d));
    m = multiply(m, translation(a, 0, 0));
    return multiply(m, rotationX(alpha));
}

// Joint-frame origins along the chain and the tool frame at its end.
//
// Angles arrive in DEGREES because that is what the CSV carries. They also
// arrive unwrapped — Kortex reports [0, 360) so a joint at -20 deg reads
// 340 — which does not matter here, since only cos and sin of the angle are
// used and both are periodic. Wrapping matters where a DIFFERENCE is taken
// (State.h WrappedJointError), not here.
//
// Returns null rather than a half-drawn arm when the inputs are unusable, so
// a row of NaN produces no arm instead of an arm at the origin.
//
// Exported without the canvas so the forward kinematics can be cross-checked
// against tools/print_dual_arm_fk from a test runner.
export function forwardKinematics(joints, anglesDeg, baseMatrix) {
    if (!Array.isArray(joints) || joints.length === 0) return null;
    if (!Array.isArray(anglesDeg) || anglesDeg.length < joints.length) return null;
    let frame = multiply(baseMatrix || identity(), rotationX(DH_ROOT_ROLL_RAD));
    const points = [originOf(frame)];
    for (let i = 0; i < joints.length; i++) {
        // A column the controller did not compute arrives as null, and
        // Number(null) is 0 — finite, and indistinguishable from a joint
        // genuinely at zero. Reject the empty values before converting, or a
        // missing measurement is drawn as a real one.
        const raw = anglesDeg[i];
        if (raw === null || raw === undefined || raw === "") return null;
        const angle = Number(raw);
        if (!isFinite(angle)) return null;
        frame = multiply(frame, jointTransform(joints[i], angle * DEG));
        points.push(originOf(frame));
    }
    return { points: points, tool: frame };
}

// Trans(xyz) then Rot(rpy), fixed-axis roll-pitch-yaw, the way a URDF
// <origin> composes.
function mountTransform(placement) {
    if (!placement) return identity();
    const xyz = placement.xyz || [0, 0, 0];
    const rpy = placement.rpy || [0, 0, 0];
    let m = translation(xyz[0] || 0, xyz[1] || 0, xyz[2] || 0);
    m = multiply(m, rotationZ(rpy[2] || 0));
    m = multiply(m, rotationY(rpy[1] || 0));
    return multiply(m, rotationX(rpy[0] || 0));
}

// ---------------------------------------------------------------
// Reading what the server sends
// ---------------------------------------------------------------

// A CSV cell that is 'nan', empty, absent or a non-number becomes null. It
// must never become 0, because zero reads as perfect tracking: the panel
// would draw a desired point sitting exactly on the tool and a 0.0 mm
// dimension that nothing measured.
function finite(value) {
    if (value === null || value === undefined || value === '') return null;
    const number = typeof value === 'number' ? value : Number(value);
    return isFinite(number) ? number : null;
}

function jointAngles(row, prefix) {
    if (!row) return null;
    const angles = [];
    for (let i = 1; i <= 7; i++) {
        const value = finite(row[prefix + i]);
        if (value === null) return null;
        angles.push(value);
    }
    return angles;
}

function pointFromRow(row, prefix) {
    if (!row) return null;
    const x = finite(row[prefix + 'x']);
    const y = finite(row[prefix + 'y']);
    const z = finite(row[prefix + 'z']);
    if (x === null || y === null || z === null) return null;
    return [x, y, z];
}

// A plan row can reasonably arrive in several shapes, so accept the ones
// that exist rather than force the server into one: the controller's wire
// row (t_s then seven joint degrees then seven velocities), a joint vector
// in degrees or radians, or a tool point already in Cartesian metres. Joint
// shapes are run through the same forward kinematics as the arms, so the
// plan polyline and the arm can never come from different models.
function planRowToPoint(row, joints, baseMatrix) {
    if (!row) return null;
    if (Array.isArray(row)) {
        if (row.length === 3) return row.map(Number);
        if (row.length === 7) return toolPoint(joints, row, baseMatrix);
        if (row.length >= 15) return toolPoint(joints, row.slice(1, 8), baseMatrix);
        return null;
    }
    if (Array.isArray(row.q_deg)) return toolPoint(joints, row.q_deg, baseMatrix);
    if (Array.isArray(row.q_rad)) {
        return toolPoint(joints, row.q_rad.map(function (v) { return v / DEG; }), baseMatrix);
    }
    if (Array.isArray(row.p)) return row.p.map(Number);
    const x = finite(row.x);
    const y = finite(row.y);
    const z = finite(row.z);
    if (x !== null && y !== null && z !== null) return [x, y, z];
    return null;
}

function toolPoint(joints, anglesDeg, baseMatrix) {
    const chain = forwardKinematics(joints, anglesDeg, baseMatrix);
    return chain ? chain.points[chain.points.length - 1] : null;
}

// ---------------------------------------------------------------
// Camera and projection
// ---------------------------------------------------------------

// The camera orbits a target with Z up, which is the mount frame's own up.
// A view is (azimuth, elevation, distance, target); the projection is a
// single focal length, so a segment's screen position depends only on its
// depth and nothing has to be tracked between frames.
function cameraBasis(view) {
    const cosElevation = Math.cos(view.elevation);
    const eye = add(view.target, [
        view.distance * cosElevation * Math.cos(view.azimuth),
        view.distance * cosElevation * Math.sin(view.azimuth),
        view.distance * Math.sin(view.elevation),
    ]);
    const forward = normalise(subtract(view.target, eye));
    let right = cross(forward, [0, 0, 1]);
    if (norm(right) < 1e-6) right = [1, 0, 0]; // looking straight down
    right = normalise(right);
    const up = cross(right, forward);
    return { eye: eye, forward: forward, right: right, up: up };
}

// Camera-space coordinates: x right, y up, z forward (positive in front of
// the camera). Points behind the near plane are clipped before projection,
// so a segment that crosses it is shortened rather than flipped.
function toCamera(basis, point) {
    const relative = subtract(point, basis.eye);
    return {
        x: dot(relative, basis.right),
        y: dot(relative, basis.up),
        z: dot(relative, basis.forward),
    };
}

const NEAR_M = 0.05;

function clipNear(a, b) {
    if (a.z >= NEAR_M && b.z >= NEAR_M) return [a, b];
    if (a.z < NEAR_M && b.z < NEAR_M) return null;
    const near = a.z < NEAR_M ? a : b;
    const far = a.z < NEAR_M ? b : a;
    const t = (NEAR_M - near.z) / (far.z - near.z);
    const cut = {
        x: near.x + (far.x - near.x) * t,
        y: near.y + (far.y - near.y) * t,
        z: NEAR_M,
    };
    return a.z < NEAR_M ? [cut, far] : [far, cut];
}

function project(camera, viewport) {
    return {
        x: viewport.cx + (viewport.focal * camera.x) / camera.z,
        y: viewport.cy - (viewport.focal * camera.y) / camera.z,
        depth: camera.z,
    };
}

// ---------------------------------------------------------------
// The scene
// ---------------------------------------------------------------

export function createScene(canvas, options) {
    const opts = options || {};
    const ctx = canvas.getContext('2d');

    const mounts = opts.mountFromBase || DEFAULT_MOUNT_FROM_BASE;
    const frameLabel = opts.frameLabel || 'mount frame - world = mount, no Vicon';
    // index.html carries a legend under the canvas, so the scene draws its own
    // only when it is standing alone (scene.test.html): two legends saying the
    // same thing would just cost canvas.
    const drawLegend = opts.legend === true;

    const state = {
        dh: { right: null, left: null },
        row: { right: null, left: null },
        plan: { right: null, left: null },
        progress: null,
        obstacle: null,
        clearance: null,
        selected: 'right',
    };

    // Starting view: from the front-right and slightly above, far enough out
    // that a fully extended arm and the SDF grid both fit.
    const view = {
        azimuth: opts.azimuth !== undefined ? opts.azimuth : 0.85,
        elevation: opts.elevation !== undefined ? opts.elevation : 0.30,
        distance: opts.distance !== undefined ? opts.distance : 3.0,
        target: opts.target ? opts.target.slice() : [0, 0, 0],
    };

    // The arms hang below the mount in some configurations and reach above it
    // in others, so a fixed default view can leave them half off screen. The
    // first draw that has geometry therefore frames it once and then stops:
    // refitting on every telemetry frame would make the whole scene creep
    // while the arm moved, which is exactly the movement the design wants to
    // mean "something happened". A drag or a wheel hands framing to the user
    // for good.
    let framingOwned = opts.target !== undefined || opts.distance !== undefined;

    let palette = readPalette();
    let fonts = readFonts();
    let viewport = { width: 0, height: 0, cx: 0, cy: 0, focal: 0, dpr: 1 };
    let pending = 0;

    function readPalette() {
        const style = window.getComputedStyle(canvas);
        const pick = function (name, fallback) {
            const value = style.getPropertyValue(name);
            return value && value.trim() ? value.trim() : fallback;
        };
        // The names are panel.css's own custom properties, so the drawing and
        // the page around it cannot drift apart.
        return {
            ground: pick('--ground', PALETTE_FALLBACK.ground),
            ink: pick('--ink', PALETTE_FALLBACK.ink),
            hairline: pick('--rule', PALETTE_FALLBACK.hairline),
            ask: pick('--ask', PALETTE_FALLBACK.ask),
        };
    }

    function readFonts() {
        const style = window.getComputedStyle(canvas);
        const mono = style.getPropertyValue('--num-font');
        const label = style.getPropertyValue('--label-font');
        return {
            mono: mono && mono.trim() ? mono.trim() : 'ui-monospace, Menlo, Consolas, monospace',
            label: label && label.trim() ? label.trim() : 'system-ui, -apple-system, sans-serif',
        };
    }

    function baseMatrix(arm) {
        return mountTransform(mounts[arm]);
    }

    // ---- redraw scheduling -------------------------------------
    //
    // Setters mark the scene dirty and let one animation frame do the work,
    // so twenty telemetry frames a second cost twenty draws at most and a
    // burst costs one. There is no idle animation at all, which is also the
    // whole of this module's answer to prefers-reduced-motion: nothing moves
    // on screen unless the data moved or the user dragged the view.
    function schedule() {
        if (pending) return;
        pending = window.requestAnimationFrame(function () {
            pending = 0;
            draw();
        });
    }

    // ---- sizing ------------------------------------------------

    // The CSS size of the canvas belongs to the page; only its backing store
    // is set here. Note for whoever writes that CSS: a canvas whose `width`
    // attribute grows also grows its intrinsic size, so inside a flex row it
    // must carry `min-width: 0` or it will push its neighbours out and this
    // function will keep enlarging it as it does.
    function resize() {
        const dpr = window.devicePixelRatio || 1;
        const width = Math.max(1, canvas.clientWidth || canvas.width || 1);
        const height = Math.max(1, canvas.clientHeight || canvas.height || 1);
        const pixelWidth = Math.round(width * dpr);
        const pixelHeight = Math.round(height * dpr);
        if (canvas.width !== pixelWidth) canvas.width = pixelWidth;
        if (canvas.height !== pixelHeight) canvas.height = pixelHeight;
        // Draw in CSS pixels and let the transform scale to device pixels,
        // so a hairline stays one hairline on a Retina display instead of
        // becoming a blurred two.
        viewport = {
            width: width,
            height: height,
            cx: width / 2,
            cy: height / 2,
            // Focal length in pixels from a ~40 degree vertical field.
            focal: height / (2 * Math.tan(0.35)),
            dpr: dpr,
        };
        palette = readPalette();
        fonts = readFonts();
        schedule();
    }

    // ---- collecting geometry -----------------------------------
    //
    // Every 3D primitive becomes an item with a depth and a paint closure.
    // They are sorted far-to-near and painted in that order — the painter's
    // algorithm. For line art this is exact enough: the only visible error
    // would be two segments crossing in depth, and a stick figure has none.

    function addSegment(items, basis, a, b, style) {
        const clipped = clipNear(toCamera(basis, a), toCamera(basis, b));
        if (!clipped) return;
        const pa = project(clipped[0], viewport);
        const pb = project(clipped[1], viewport);
        items.push({
            depth: (clipped[0].z + clipped[1].z) / 2,
            paint: function () {
                ctx.save();
                ctx.globalAlpha = style.alpha === undefined ? 1 : style.alpha;
                ctx.strokeStyle = style.colour;
                ctx.lineWidth = style.width || 1;
                ctx.lineCap = 'round';
                if (style.dash) ctx.setLineDash(style.dash);
                ctx.beginPath();
                ctx.moveTo(pa.x, pa.y);
                ctx.lineTo(pb.x, pb.y);
                ctx.stroke();
                ctx.restore();
            },
        });
    }

    function addDot(items, basis, point, style) {
        const radius = style.radius === undefined ? 3 : style.radius;
        if (radius <= 0) return; // a caller asking for no dot means no dot
        const camera = toCamera(basis, point);
        if (camera.z < NEAR_M) return;
        const p = project(camera, viewport);
        items.push({
            depth: camera.z,
            paint: function () {
                ctx.save();
                ctx.globalAlpha = style.alpha === undefined ? 1 : style.alpha;
                ctx.fillStyle = style.colour;
                ctx.beginPath();
                ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
                ctx.fill();
                ctx.restore();
            },
        });
    }

    // The desired point is drawn as an open cross rather than a filled dot,
    // so it reads as a target and never as a joint.
    function addCross(items, basis, point, style) {
        const camera = toCamera(basis, point);
        if (camera.z < NEAR_M) return;
        const p = project(camera, viewport);
        const r = style.radius || 6;
        items.push({
            depth: camera.z,
            paint: function () {
                ctx.save();
                ctx.strokeStyle = style.colour;
                ctx.lineWidth = style.width || 1.25;
                ctx.beginPath();
                ctx.moveTo(p.x - r, p.y);
                ctx.lineTo(p.x + r, p.y);
                ctx.moveTo(p.x, p.y - r);
                ctx.lineTo(p.x, p.y + r);
                ctx.stroke();
                ctx.beginPath();
                ctx.arc(p.x, p.y, r * 0.55, 0, Math.PI * 2);
                ctx.stroke();
                ctx.restore();
            },
        });
    }

    function addPolyline(items, basis, points, style) {
        for (let i = 1; i < points.length; i++) {
            addSegment(items, basis, points[i - 1], points[i], style);
        }
    }

    function addBox(items, basis, minimum, maximum, style) {
        const corner = function (i) {
            return [
                i & 1 ? maximum[0] : minimum[0],
                i & 2 ? maximum[1] : minimum[1],
                i & 4 ? maximum[2] : minimum[2],
            ];
        };
        for (let i = 0; i < 8; i++) {
            for (let bit = 1; bit <= 4; bit <<= 1) {
                if (i & bit) continue; // draw each edge once
                addSegment(items, basis, corner(i), corner(i | bit), style);
            }
        }
    }

    // The mount XY plane. It is a reference plane through the rig, NOT the
    // floor: nothing in this system knows where the floor is, so the grid is
    // kept faint and unlabelled rather than pretending to be ground.
    function addGrid(items, basis) {
        const half = 1.2;
        const step = 0.2;
        const style = { colour: palette.hairline, width: 0.6, alpha: 0.45 };
        for (let i = -half; i <= half + 1e-9; i += step) {
            const t = Math.round(i * 1000) / 1000;
            addSegment(items, basis, [t, -half, 0], [t, half, 0], style);
            addSegment(items, basis, [-half, t, 0], [half, t, 0], style);
        }
        // The mount origin's own axes, lettered rather than coloured —
        // colour on this screen is reserved for abnormal states.
        const axes = [
            { v: [0.25, 0, 0], label: 'x' },
            { v: [0, 0.25, 0], label: 'y' },
            { v: [0, 0, 0.25], label: 'z' },
        ];
        for (let i = 0; i < axes.length; i++) {
            addSegment(items, basis, [0, 0, 0], axes[i].v, {
                colour: palette.ink,
                width: 1,
                alpha: 0.55,
            });
        }
        return axes;
    }

    function addArm(items, basis, arm, prefix, style) {
        const joints = state.dh[arm];
        if (!joints) return null;
        const angles = jointAngles(state.row[arm], prefix);
        if (!angles) return null;
        const chain = forwardKinematics(joints, angles, baseMatrix(arm));
        if (!chain) return null;
        addPolyline(items, basis, chain.points, style);
        for (let i = 0; i < chain.points.length; i++) {
            addDot(items, basis, chain.points[i], {
                colour: style.colour,
                radius: style.jointRadius === undefined ? 2 : style.jointRadius,
                alpha: style.alpha,
            });
        }
        return chain;
    }

    // A short triad at the tool, so the tool's orientation is visible even
    // when no orientation error is being shown. Its axes are NOT lettered:
    // the tool is exactly where the dimension callout and the desired marker
    // already are, and three more characters there cost more than they say.
    // The lettered triad at the mount origin names the axes for the scene.
    function addTriad(items, basis, frame, style) {
        const origin = originOf(frame);
        for (let i = 0; i < 3; i++) {
            const tip = add(origin, scale(axisOf(frame, i), style.length || 0.08));
            addSegment(items, basis, origin, tip, {
                colour: style.colour,
                width: style.width || 1,
                alpha: style.alpha,
            });
        }
    }

    // ---- the dimension callout ---------------------------------
    //
    // The one borrowed flourish in the design: the gap between the measured
    // tool point and the desired point is annotated the way a draughtsman
    // annotates a print — extension ticks at both ends, a dimension line
    // with arrowheads, and the number sitting in a gap in the line.
    //
    // The number is the true 3D distance in millimetres, not the distance on
    // screen, so rotating the view never changes it. `cmd - meas` is what the
    // controller's following-error guard tests, and this pair (desired point
    // against measured tool) is the Cartesian reading of the same question.
    // `awayFrom` is a point the callout should lean away from — the last
    // wrist joint. Without it the dimension line lands on the forearm about
    // half the time, since the tool is at the end of a link and the offset
    // has two equally valid sides.
    function paintDimension(basis, fromPoint, toPoint, awayFrom) {
        const a = toCamera(basis, fromPoint);
        const b = toCamera(basis, toPoint);
        if (a.z < NEAR_M || b.z < NEAR_M) return;
        const pa = project(a, viewport);
        const pb = project(b, viewport);
        const metres = norm(subtract(toPoint, fromPoint));
        const millimetres = metres * 1000;
        const text = (millimetres >= 100 ? millimetres.toFixed(0) : millimetres.toFixed(1)) + ' mm';

        const dx = pb.x - pa.x;
        const dy = pb.y - pa.y;
        const screenLength = Math.sqrt(dx * dx + dy * dy);
        ctx.save();
        ctx.strokeStyle = palette.ink;
        ctx.fillStyle = palette.ink;
        ctx.lineWidth = 1;
        ctx.font = '11px ' + fonts.mono;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';

        if (screenLength < 1) {
            // The two points coincide on screen. A dimension line has no
            // direction to be drawn along, so state the number on a short
            // leader instead of inventing one.
            const tip = { x: pa.x + 34, y: pa.y - 26 };
            ctx.beginPath();
            ctx.moveTo(pa.x, pa.y);
            ctx.lineTo(tip.x, tip.y);
            ctx.lineTo(tip.x + 26, tip.y);
            ctx.stroke();
            labelWithHalo(text, tip.x + 30, tip.y, 'left');
            ctx.restore();
            return;
        }

        const ux = dx / screenLength;
        const uy = dy / screenLength;
        let nx = -uy;
        let ny = ux;
        if (awayFrom) {
            const camera = toCamera(basis, awayFrom);
            if (camera.z >= NEAR_M) {
                const pv = project(camera, viewport);
                if (nx * (pa.x - pv.x) + ny * (pa.y - pv.y) < 0) {
                    nx = -nx;
                    ny = -ny;
                }
            }
        }
        // Offset the dimension line clear of the geometry it measures, the
        // way an offset dimension keeps a drawing readable.
        const offset = 22;
        const a1 = { x: pa.x + nx * offset, y: pa.y + ny * offset };
        const b1 = { x: pb.x + nx * offset, y: pb.y + ny * offset };

        // Extension lines: from each measured point out past the dimension
        // line, with a small gap at the point itself.
        ctx.beginPath();
        ctx.moveTo(pa.x + nx * 3, pa.y + ny * 3);
        ctx.lineTo(pa.x + nx * (offset + 8), pa.y + ny * (offset + 8));
        ctx.moveTo(pb.x + nx * 3, pb.y + ny * 3);
        ctx.lineTo(pb.x + nx * (offset + 8), pb.y + ny * (offset + 8));
        ctx.stroke();

        const midX = (a1.x + b1.x) / 2;
        const midY = (a1.y + b1.y) / 2;
        const textWidth = ctx.measureText(text).width;
        const gap = textWidth / 2 + 5;

        if (screenLength > 2 * gap + 16) {
            // Room for the number inside the line: break the line for it.
            ctx.beginPath();
            ctx.moveTo(a1.x, a1.y);
            ctx.lineTo(midX - ux * gap, midY - uy * gap);
            ctx.moveTo(midX + ux * gap, midY + uy * gap);
            ctx.lineTo(b1.x, b1.y);
            ctx.stroke();
            arrowhead(a1, { x: ux, y: uy });
            arrowhead(b1, { x: -ux, y: -uy });
            labelWithHalo(text, midX, midY, 'center');
        } else {
            // Too short: arrows point inwards from outside and the number
            // sits off the end, which is what a draughtsman does with a
            // small dimension.
            ctx.beginPath();
            ctx.moveTo(a1.x - ux * 14, a1.y - uy * 14);
            ctx.lineTo(b1.x + ux * 14, b1.y + uy * 14);
            ctx.stroke();
            arrowhead(a1, { x: -ux, y: -uy });
            arrowhead(b1, { x: ux, y: uy });
            labelWithHalo(text, b1.x + ux * 20 + nx * 8, b1.y + uy * 20 + ny * 8, 'left');
        }

        // Name the far end on the side the callout is not using, so the word
        // and the number never sit on top of each other.
        ctx.font = '10px ' + fonts.label;
        ctx.fillStyle = palette.ask;
        labelWithHalo('desired', pb.x - nx * 15, pb.y - ny * 15, 'center');
        ctx.restore();
    }

    function arrowhead(tip, direction) {
        const length = 8;
        const width = 3;
        const nx = -direction.y;
        const ny = direction.x;
        ctx.beginPath();
        ctx.moveTo(tip.x, tip.y);
        ctx.lineTo(tip.x + direction.x * length + nx * width, tip.y + direction.y * length + ny * width);
        ctx.lineTo(tip.x + direction.x * length - nx * width, tip.y + direction.y * length - ny * width);
        ctx.closePath();
        ctx.fill();
    }

    // Text over line art needs the line cleared behind it or the number
    // becomes unreadable exactly when it matters. The halo is the ground
    // colour, so it works on the pale instrument ground and nowhere needs a
    // box drawn around it.
    function labelWithHalo(text, x, y, align) {
        ctx.save();
        ctx.textAlign = align || 'center';
        ctx.lineJoin = 'round';
        ctx.lineWidth = 3.5;
        ctx.strokeStyle = palette.ground;
        ctx.strokeText(text, x, y);
        ctx.fillText(text, x, y);
        ctx.restore();
    }

    // ---- the plan ----------------------------------------------

    function planPolyline(arm) {
        const rows = state.plan[arm];
        if (!rows || rows.length < 2) return null;
        const joints = state.dh[arm];
        const base = baseMatrix(arm);
        const points = [];
        for (let i = 0; i < rows.length; i++) {
            const point = planRowToPoint(rows[i], joints, base);
            if (point && isFinite(point[0]) && isFinite(point[1]) && isFinite(point[2])) {
                points.push(point);
            }
        }
        return points.length >= 2 ? points : null;
    }

    // Split the polyline at the progress point by ARC LENGTH, not by row
    // index: the solved states are evenly spaced in time, so an index split
    // would move the marker at whatever speed the plan happens to run at
    // while the drawn line stayed still.
    function splitAtProgress(points, fraction) {
        if (fraction === null || fraction === undefined || !isFinite(fraction)) return null;
        const f = Math.min(1, Math.max(0, fraction));
        let total = 0;
        const lengths = [];
        for (let i = 1; i < points.length; i++) {
            const segment = norm(subtract(points[i], points[i - 1]));
            lengths.push(segment);
            total += segment;
        }
        if (total <= 0) return null;
        const wanted = total * f;
        let travelled = 0;
        for (let i = 0; i < lengths.length; i++) {
            if (travelled + lengths[i] >= wanted) {
                const t = lengths[i] > 0 ? (wanted - travelled) / lengths[i] : 0;
                const cut = add(points[i], scale(subtract(points[i + 1], points[i]), t));
                return {
                    travelled: points.slice(0, i + 1).concat([cut]),
                    remaining: [cut].concat(points.slice(i + 1)),
                    point: cut,
                };
            }
            travelled += lengths[i];
        }
        const last = points[points.length - 1];
        return { travelled: points.slice(), remaining: [last], point: last };
    }

    // ---- overlay text ------------------------------------------

    function paintOverlay(basis, notes) {
        ctx.save();
        ctx.font = '11px ' + fonts.label;
        ctx.fillStyle = palette.ink;
        ctx.textBaseline = 'alphabetic';
        ctx.textAlign = 'left';
        ctx.globalAlpha = 0.75;
        ctx.fillText(frameLabel, 10, viewport.height - 10);
        ctx.globalAlpha = 1;

        // A legend, because "solid" and "ghosted" only mean something if the
        // screen says which is which. The desired point shares the commanded
        // colour on purpose: in this palette that one blue means "what was
        // asked for", and both of them are that.
        if (drawLegend) {
            const legendY = 16;
            ctx.textAlign = 'right';
            ctx.fillStyle = palette.ink;
            ctx.fillText('measured', viewport.width - 10, legendY);
            ctx.fillStyle = palette.ask;
            ctx.fillText('commanded', viewport.width - 10, legendY + 14);
            ctx.fillText('desired point', viewport.width - 10, legendY + 28);
        }
        ctx.restore();

        for (let i = 0; i < notes.length; i++) {
            const note = notes[i];
            const camera = toCamera(basis, note.point);
            if (camera.z < NEAR_M) continue;
            const p = project(camera, viewport);
            ctx.save();
            ctx.font = (note.size || 10) + 'px ' + (note.mono ? fonts.mono : fonts.label);
            ctx.fillStyle = note.colour || palette.ink;
            ctx.textBaseline = 'middle';
            ctx.globalAlpha = note.alpha === undefined ? 0.85 : note.alpha;
            labelWithHalo(note.text, p.x + (note.dx || 6), p.y + (note.dy || 0), note.align || 'left');
            ctx.restore();
        }
    }

    function paintEmptyState() {
        ctx.save();
        ctx.font = '12px ' + fonts.label;
        ctx.fillStyle = palette.ink;
        ctx.globalAlpha = 0.6;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('no telemetry yet', viewport.cx, viewport.cy);
        ctx.restore();
    }

    // ---- draw --------------------------------------------------

    function draw() {
        if (!viewport.width) resize();
        ctx.setTransform(viewport.dpr, 0, 0, viewport.dpr, 0, 0);
        ctx.clearRect(0, 0, viewport.width, viewport.height);
        ctx.fillStyle = palette.ground;
        ctx.fillRect(0, 0, viewport.width, viewport.height);

        const basis = cameraBasis(view);
        const items = [];
        const notes = [];
        // Points worth having on screen, collected for the one-time framing
        // below: the SELECTED arm, its plan and its desired point. The grid
        // and the SDF bounds are deliberately not in it — they are fixed
        // volumes far larger than an arm, and framing them would shrink the
        // arm to nothing — and neither is the unselected arm, which can be
        // anywhere and would do the same.
        const framed = [];

        const axes = addGrid(items, basis);
        for (let i = 0; i < axes.length; i++) {
            notes.push({ point: axes[i].v, text: axes[i].label, alpha: 0.5, size: 9 });
        }

        // Both arms, the selected one at full weight. The unselected arm is
        // still drawn — inter-arm proximity is only visible when both are on
        // screen — but quieter, so the numbers panel and the scene agree
        // about which arm is being read.
        const arms = ['right', 'left'];
        const measured = {};
        for (let i = 0; i < arms.length; i++) {
            const arm = arms[i];
            const selected = arm === state.selected;
            const commanded = addArm(items, basis, arm, 'cmd_j', {
                colour: palette.ask,
                width: selected ? 3.5 : 2.5,
                alpha: selected ? 0.55 : 0.3,
                jointRadius: 0,
            });
            const chain = addArm(items, basis, arm, 'meas_j', {
                colour: palette.ink,
                width: selected ? 1.8 : 1.2,
                alpha: selected ? 1 : 0.45,
                jointRadius: selected ? 2.4 : 1.6,
            });
            if (chain) {
                measured[arm] = chain;
                if (selected) {
                    for (let p = 0; p < chain.points.length; p++) framed.push(chain.points[p]);
                }
                addTriad(items, basis, chain.tool, {
                    colour: palette.ink,
                    alpha: selected ? 0.8 : 0.35,
                    length: 0.08,
                });
            }
            if (commanded && !chain) {
                // Commanded without measured is worth seeing rather than
                // hiding: it means the row carried no measured angles.
                notes.push({
                    point: originOf(commanded.tool),
                    text: arm + ': commanded only',
                    colour: palette.ask,
                });
            }

            const plan = planPolyline(arm);
            if (plan) {
                if (selected) for (let p = 0; p < plan.length; p++) framed.push(plan[p]);
                const split = arm === state.selected ? splitAtProgress(plan, state.progress) : null;
                if (split) {
                    addPolyline(items, basis, split.travelled, {
                        colour: palette.ink,
                        width: 1,
                        alpha: 0.25,
                        dash: [3, 3],
                    });
                    addPolyline(items, basis, split.remaining, {
                        colour: palette.ink,
                        width: 1.4,
                        alpha: 0.7,
                    });
                    addDot(items, basis, split.point, { colour: palette.ink, radius: 3, alpha: 0.8 });
                } else {
                    addPolyline(items, basis, plan, {
                        colour: palette.ink,
                        width: arm === state.selected ? 1.4 : 1,
                        alpha: arm === state.selected ? 0.7 : 0.3,
                    });
                }
            }
        }

        // The desired tool point and the dimension between it and where the
        // tool actually is. Both come from the selected arm's row, and the
        // point is in that arm's base_link, so it is carried into mount here.
        let dimension = null;
        let desiredPoint = null;
        const row = state.row[state.selected];
        const desiredBase = pointFromRow(row, 'pd_');
        if (desiredBase) {
            const desired = transformPoint(baseMatrix(state.selected), desiredBase);
            framed.push(desired);
            addCross(items, basis, desired, { colour: palette.ask, radius: 6 });
            desiredPoint = desired;
            const chain = measured[state.selected];
            if (chain) {
                dimension = {
                    from: chain.points[chain.points.length - 1],
                    to: desired,
                    awayFrom: chain.points[chain.points.length - 2],
                };
            }
        }

        // With a dimension drawn, the marker is named by paintDimension on
        // the side the callout is not using; without one it is named here.
        if (desiredPoint && !dimension) {
            notes.push({
                point: desiredPoint, text: 'desired', colour: palette.ask,
                dx: 0, dy: 15, align: 'center',
            });
        }

        if (state.obstacle) drawObstacle(items, basis);
        if (state.clearance) {
            const point = state.clearance.point;
            addDot(items, basis, point, { colour: palette.ink, radius: 3.5 });
            addCross(items, basis, point, { colour: palette.ink, radius: 8, width: 1 });
            notes.push({
                point: point,
                // Named as a property of the plan, because nothing measures
                // clearance while the arm is moving.
                text: 'plan min clearance ' + formatMetres(state.clearance.metres),
                dx: 11,
                dy: 10,
                mono: true,
            });
        }

        items.sort(function (a, b) { return b.depth - a.depth; });
        for (let i = 0; i < items.length; i++) items[i].paint();

        if (dimension) paintDimension(basis, dimension.from, dimension.to, dimension.awayFrom);
        paintOverlay(basis, notes);

        const nothing = !state.row.right && !state.row.left &&
            !state.plan.right && !state.plan.left;
        if (nothing) paintEmptyState();

        if (!framingOwned && framed.length) {
            framingOwned = true;
            frameOn(framed);
            schedule();
        }
    }

    // Put the given points comfortably inside the vertical field of view, and
    // leave the viewing direction alone: what the user is looking at may be
    // wrong, but which way they are looking at it is their choice.
    function frameOn(points) {
        const minimum = points[0].slice();
        const maximum = points[0].slice();
        for (let i = 1; i < points.length; i++) {
            for (let axis = 0; axis < 3; axis++) {
                minimum[axis] = Math.min(minimum[axis], points[i][axis]);
                maximum[axis] = Math.max(maximum[axis], points[i][axis]);
            }
        }
        view.target = [
            (minimum[0] + maximum[0]) / 2,
            (minimum[1] + maximum[1]) / 2,
            (minimum[2] + maximum[2]) / 2,
        ];
        const radius = Math.max(0.25, norm(subtract(maximum, minimum)) / 2);
        // 0.35 rad is the half-field the focal length is built from; the 1.35
        // leaves room for the labels that hang off the geometry.
        view.distance = Math.min(12, (radius / Math.tan(0.35)) * 1.35);
    }

    function drawObstacle(items, basis) {
        const box = state.obstacle;
        if (box.centre && box.halfExtent) {
            const base = box.frame === 'mount' ? identity() : baseMatrix(box.arm || state.selected);
            const style = { colour: palette.ink, width: 1, alpha: 0.6 };
            // The box is axis-aligned in ITS OWN frame (goal.yaml is explicit
            // that it must be), so its corners are transformed individually
            // rather than the min/max pair: in mount it is no longer
            // axis-aligned, and drawing it as if it were would move its faces.
            const c = box.centre;
            const h = box.halfExtent;
            const corner = function (i) {
                return transformPoint(base, [
                    c[0] + (i & 1 ? h[0] : -h[0]),
                    c[1] + (i & 2 ? h[1] : -h[1]),
                    c[2] + (i & 4 ? h[2] : -h[2]),
                ]);
            };
            for (let i = 0; i < 8; i++) {
                for (let bit = 1; bit <= 4; bit <<= 1) {
                    if (i & bit) continue;
                    addSegment(items, basis, corner(i), corner(i | bit), style);
                }
            }
        }
        if (box.bounds && box.bounds.min && box.bounds.max) {
            // The SDF grid is defined in mount and only ever queried there.
            addBox(items, basis, box.bounds.min, box.bounds.max, {
                colour: palette.hairline,
                width: 0.8,
                alpha: 0.7,
                dash: [5, 5],
            });
        }
    }

    function formatMetres(metres) {
        if (metres === null || metres === undefined || !isFinite(metres)) return 'unknown';
        const mm = metres * 1000;
        return (mm >= 100 ? mm.toFixed(0) : mm.toFixed(1)) + ' mm';
    }

    // ---- orbit control -----------------------------------------

    let dragging = null;

    function onPointerDown(event) {
        dragging = { x: event.clientX, y: event.clientY, id: event.pointerId };
        if (canvas.setPointerCapture) canvas.setPointerCapture(event.pointerId);
    }

    function onPointerMove(event) {
        if (!dragging || event.pointerId !== dragging.id) return;
        const dx = event.clientX - dragging.x;
        const dy = event.clientY - dragging.y;
        dragging.x = event.clientX;
        dragging.y = event.clientY;
        framingOwned = true;
        view.azimuth -= dx * 0.008;
        // Stop just short of the poles: at the pole the up vector and the
        // view direction are parallel and the camera basis is undefined.
        const limit = Math.PI / 2 - 0.05;
        view.elevation = Math.min(limit, Math.max(-limit, view.elevation + dy * 0.006));
        schedule();
    }

    function onPointerUp(event) {
        if (dragging && canvas.releasePointerCapture) {
            try { canvas.releasePointerCapture(dragging.id); } catch (ignored) { /* already gone */ }
        }
        dragging = null;
    }

    function onWheel(event) {
        event.preventDefault();
        framingOwned = true;
        const factor = Math.exp(event.deltaY * 0.0012);
        view.distance = Math.min(12, Math.max(0.4, view.distance * factor));
        schedule();
    }

    canvas.addEventListener('pointerdown', onPointerDown);
    canvas.addEventListener('pointermove', onPointerMove);
    canvas.addEventListener('pointerup', onPointerUp);
    canvas.addEventListener('pointercancel', onPointerUp);
    canvas.addEventListener('wheel', onWheel, { passive: false });

    if (typeof ResizeObserver !== 'undefined') {
        new ResizeObserver(function () { resize(); }).observe(canvas);
    } else {
        window.addEventListener('resize', resize);
    }

    resize();

    // ---- the interface panel.js uses ---------------------------

    // An arm name that is neither of the two is dropped rather than added as
    // a third: the scene draws the rig it has.
    function known(arm) {
        return arm === 'right' || arm === 'left';
    }

    return {
        // joints is the `joints` array of /api/dh, or that whole response
        // object — both are accepted so the caller need not unwrap it.
        setDh: function (arm, joints) {
            if (!known(arm)) return;
            const list = Array.isArray(joints) ? joints : joints && joints.joints;
            state.dh[arm] = Array.isArray(list) && list.length ? list : null;
            schedule();
        },
        // row is one parsed telemetry row, keys by CSV column NAME.
        setTelemetry: function (arm, row) {
            if (!known(arm)) return;
            state.row[arm] = row || null;
            schedule();
        },
        setPlan: function (arm, planRows) {
            if (!known(arm)) return;
            state.plan[arm] = Array.isArray(planRows) && planRows.length ? planRows : null;
            schedule();
        },
        setProgress: function (fraction) {
            const value = finite(fraction);
            state.progress = value === null ? null : Math.min(1, Math.max(0, value));
            schedule();
        },
        // box: {centre:[x,y,z], halfExtent:[x,y,z], frame:'base'|'mount',
        // arm:'right'|'left', bounds:{min:[..], max:[..]}}. `centre`/
        // `center` and `halfExtent`/`half_extent` are both accepted because
        // goal.yaml spells them the second way.
        setObstacle: function (box) {
            if (!box) {
                state.obstacle = null;
            } else {
                state.obstacle = {
                    centre: box.centre || box.center || null,
                    halfExtent: box.halfExtent || box.half_extent || null,
                    frame: box.frame || 'base',
                    arm: box.arm || null,
                    bounds: box.bounds || null,
                };
            }
            schedule();
        },
        // point is in mount unless it carries {frame:'base', arm:...}.
        setClearance: function (point, metres) {
            if (!point) {
                state.clearance = null;
            } else {
                const xyz = Array.isArray(point)
                    ? point.map(Number)
                    : [Number(point.x), Number(point.y), Number(point.z)];
                if (!xyz.every(isFinite)) {
                    state.clearance = null;
                } else {
                    const frame = !Array.isArray(point) && point.frame === 'base'
                        ? baseMatrix(point.arm || state.selected)
                        : identity();
                    state.clearance = {
                        point: transformPoint(frame, xyz),
                        metres: finite(metres),
                    };
                }
            }
            schedule();
        },
        setSelectedArm: function (arm) {
            if (known(arm)) state.selected = arm;
            schedule();
        },
        resize: resize,
        draw: draw,
    };
}
