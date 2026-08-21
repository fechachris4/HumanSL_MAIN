//
// scene.js — mount-frame TCP telemetry, requested goal previews, and
// planner-owned scene geometry.
//
// The runtime owns robot kinematics. This file projects supplied points and
// draws configured primitives; it contains no DH chain or robot FK.
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
// Goal preview — pure functions, exported so a test runner can pin
// them without a canvas. Nothing in here talks to a server: a preview
// is arithmetic on the numbers the operator typed, and that is the
// property that makes it safe to offer at all.
// ---------------------------------------------------------------

// The frames a goal block may declare, and what the viewer can do with
// each. Anything else is refused BY NAME, saying which transform is
// missing, rather than drawn somewhere misleading.
const PREVIEW_FRAMES = ['mount'];

// Two orthonormal vectors spanning the plane whose normal is `normal`.
// The seed-picking is copied line for line from PlaneBasis in
// planning/src/CartesianPath.cpp, because the circle's START point is
// centre + r*u and the preview must mark the same rim point the bridge
// will actually plan from. Returns null for a degenerate normal.
export function planeBasis(normal) {
    const n = normalise(normal);
    if (norm(n) < 0.5) return null; // normalise() zeroed a near-zero vector
    const ax = Math.abs(n[0]);
    const ay = Math.abs(n[1]);
    const az = Math.abs(n[2]);
    let seed;
    if (ax > ay) {
        seed = ay <= az ? [0, 1, 0] : [0, 0, 1];
    } else {
        seed = ax <= az ? [1, 0, 0] : [0, 0, 1];
    }
    const u = normalise(cross(n, seed));
    const v = cross(n, u); // already unit: n and u are orthonormal
    return { u: u, v: v };
}

function previewNumber(value) {
    if (value === null || value === undefined) return null;
    const text = String(value).trim();
    if (text === '') return null;
    const number = Number(text);
    return isFinite(number) ? number : null;
}

function previewVector(values) {
    if (!Array.isArray(values) || values.length !== 3) return null;
    const parsed = values.map(previewNumber);
    return parsed.every(function (v) { return v !== null; }) ? parsed : null;
}

// Validate the typed goal fields into a drawable preview, or say what is
// wrong in words the operator can act on. fields:
//   { mode: 'point'|'circle', frame, goal, rpy_deg|null,
//     centre, radius_m, normal, path_rpy_deg|null }
// rpy fields are null when the orientation is inherited/radial, in which
// case no triad is drawn. Returns {ok:true, preview} or {ok:false, error}.
export function parseGoalPreview(fields) {
    const f = fields || {};
    const frame = String(f.frame || 'mount');
    if (PREVIEW_FRAMES.indexOf(frame) < 0) {
        const missing = frame === 'world'
            ? 'the mount-in-world transform comes from Vicon and the panel does not have it'
            : 'no transform from that frame to the mount frame is known to the viewer';
        return { ok: false, error: 'cannot preview in frame \'' + frame + '\' — '
            + missing + ' (previewable frames: ' + PREVIEW_FRAMES.join(', ') + ')' };
    }
    if (f.mode === 'point') {
        const goal = previewVector(f.goal);
        if (!goal) {
            return { ok: false, error: 'goal xyz must be three finite numbers in metres' };
        }
        const rpy = f.rpy_deg === null || f.rpy_deg === undefined
            ? null : previewVector(f.rpy_deg);
        if (f.rpy_deg !== null && f.rpy_deg !== undefined && !rpy) {
            return { ok: false, error: 'orientation rpy must be three finite numbers in degrees' };
        }
        return { ok: true, preview: { kind: 'point', frame: frame, point: goal, rpyDeg: rpy } };
    }
    if (f.mode === 'circle') {
        const centre = previewVector(f.centre);
        if (!centre) {
            return { ok: false, error: 'circle centre must be three finite numbers in metres' };
        }
        const radius = previewNumber(f.radius_m);
        if (radius === null || radius <= 0) {
            return { ok: false, error: 'circle radius must be a positive number in metres' };
        }
        const normal = previewVector(f.normal);
        if (!normal || !planeBasis(normal)) {
            return { ok: false, error: 'circle normal must be three finite numbers and not all zero' };
        }
        const rpy = f.rpy_deg === null || f.rpy_deg === undefined
            ? null : previewVector(f.rpy_deg);
        if (f.rpy_deg !== null && f.rpy_deg !== undefined && !rpy) {
            return { ok: false, error: 'orientation rpy must be three finite numbers in degrees' };
        }
        return { ok: true, preview: { kind: 'circle', frame: frame, centre: centre,
                                      radius: radius, normal: normal, rpyDeg: rpy } };
    }
    return { ok: false, error: 'preview mode must be point or circle' };
}

// R = Rz*Ry*Rx from degrees at a position — the composition goal.yaml,
// the controller and print_dual_arm_fk share (and the same order
// poseFromRpyDeg composes below).
function poseFromRpyDeg(position, rpyDeg) {
    let m = translation(position[0], position[1], position[2]);
    m = multiply(m, rotationZ((rpyDeg[2] || 0) * DEG));
    m = multiply(m, rotationY((rpyDeg[1] || 0) * DEG));
    return multiply(m, rotationX((rpyDeg[0] || 0) * DEG));
}

const CIRCLE_PREVIEW_SEGMENTS = 64;

// A mount-frame preview ready to draw:
//   { target, points|null, centre|null, triad|null }
// `target` is the point goal, or the circle's start point (the rim at
// start angle zero, exactly where GenerateCircle begins). Non-mount requests
// are refused by parseGoalPreview before reaching this function.
export function goalPreviewGeometry(preview) {
    if (preview.kind === 'point') {
        return {
            target: preview.point.slice(),
            points: null,
            centre: null,
            triad: preview.rpyDeg
                ? poseFromRpyDeg(preview.point, preview.rpyDeg)
                : null,
        };
    }
    const basis = planeBasis(preview.normal);
    const points = [];
    for (let i = 0; i <= CIRCLE_PREVIEW_SEGMENTS; i++) {
        const angle = (2 * Math.PI * i) / CIRCLE_PREVIEW_SEGMENTS;
        const rim = add(preview.centre,
            add(scale(basis.u, preview.radius * Math.cos(angle)),
                scale(basis.v, preview.radius * Math.sin(angle))));
        points.push(rim);
    }
    const start = add(preview.centre, scale(basis.u, preview.radius));
    return {
        target: start,
        points: points,
        centre: preview.centre.slice(),
        triad: preview.rpyDeg
            ? poseFromRpyDeg(start, preview.rpyDeg)
            : null,
    };
}

// ---------------------------------------------------------------
// Reading what the server sends
// ---------------------------------------------------------------

// A CSV cell that is 'nan', empty, absent or a non-number becomes null. It
// must never become 0, because zero would fabricate a TCP at the mount origin.
function finite(value) {
    if (value === null || value === undefined || value === '') return null;
    const number = typeof value === 'number' ? value : Number(value);
    return isFinite(number) ? number : null;
}

function tcpMountPoint(row, kind) {
    if (!row) return null;
    const fields = kind === 'measured'
        ? ['measured_tcp_x_mount_m', 'measured_tcp_y_mount_m',
           'measured_tcp_z_mount_m']
        : ['commanded_tcp_x_mount_m', 'commanded_tcp_y_mount_m',
           'commanded_tcp_z_mount_m'];
    const x = finite(row[fields[0]]);
    const y = finite(row[fields[1]]);
    const z = finite(row[fields[2]]);
    if (x === null || y === null || z === null) return null;
    return [x, y, z];
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

    const frameLabel = opts.frameLabel || 'mount frame · runtime TCP telemetry';
    // index.html carries a legend under the canvas, so the scene draws its own
    // only when it is standing alone (scene.test.html): two legends saying the
    // same thing would just cost canvas.
    const drawLegend = opts.legend === true;

    const state = {
        row: { right: null, left: null },
        goalPreview: { right: null, left: null },
        obstacles: [],
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

    // Open crosses distinguish supplied points from filled scene geometry.
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

    // Display geometry only. The planner owns the finite-cylinder SDF and
    // every collision decision; this draws the same persisted centre, radius
    // and full height without adding epsilon or a clearance boundary.
    function addCylinder(items, basis, cylinder, style) {
        const centre = cylinder.center_mount_m;
        const radius = cylinder.radius_m;
        const halfHeight = cylinder.height_m / 2;
        const segments = 24;
        for (let i = 0; i < segments; i++) {
            const a = (2 * Math.PI * i) / segments;
            const b = (2 * Math.PI * (i + 1)) / segments;
            const lowerA = [centre[0] + radius * Math.cos(a),
                            centre[1] + radius * Math.sin(a),
                            centre[2] - halfHeight];
            const lowerB = [centre[0] + radius * Math.cos(b),
                            centre[1] + radius * Math.sin(b),
                            centre[2] - halfHeight];
            const upperA = [lowerA[0], lowerA[1], centre[2] + halfHeight];
            const upperB = [lowerB[0], lowerB[1], centre[2] + halfHeight];
            addSegment(items, basis, lowerA, lowerB, style);
            addSegment(items, basis, upperA, upperB, style);
            if (i % 4 === 0) addSegment(items, basis, lowerA, upperA, style);
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

    // A short unlettered triad for a requested fixed-orientation preview.
    // The lettered triad at the mount origin names the scene axes.
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

        // Standalone test pages opt into this compact marker legend.
        if (drawLegend) {
            const legendY = 16;
            ctx.textAlign = 'right';
            ctx.fillStyle = palette.ink;
            ctx.fillText('measured', viewport.width - 10, legendY);
            ctx.fillStyle = palette.ask;
            ctx.fillText('commanded', viewport.width - 10, legendY + 14);
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
        // Selected-arm TCP and preview points drive one-time camera framing.
        // Fixed scene volumes and the unselected arm do not, so they cannot
        // shrink or move the operator's view unexpectedly.
        const framed = [];

        const axes = addGrid(items, basis);
        for (let i = 0; i < axes.length; i++) {
            notes.push({ point: axes[i].v, text: axes[i].label, alpha: 0.5, size: 9 });
        }

        // Runtime-owned canonical mount-frame TCP positions. The browser
        // projects markers only; it performs no robot forward kinematics.
        const arms = ['right', 'left'];
        for (let i = 0; i < arms.length; i++) {
            const arm = arms[i];
            const selected = arm === state.selected;
            const row = state.row[arm];
            const measured = tcpMountPoint(row, 'measured');
            const commanded = tcpMountPoint(row, 'commanded');

            if (measured) {
                addDot(items, basis, measured, {
                    colour: palette.ink,
                    radius: selected ? 4 : 3,
                    alpha: selected ? 1 : 0.45,
                });
                addCross(items, basis, measured, {
                    colour: palette.ink,
                    radius: selected ? 8 : 6,
                    width: selected ? 1.6 : 1,
                });
                if (selected) framed.push(measured);
                notes.push({
                    point: measured, text: arm + ' measured TCP',
                    colour: palette.ink, dx: 10, dy: 12,
                });
            }
            if (commanded) {
                addCross(items, basis, commanded, {
                    colour: palette.ask,
                    radius: selected ? 8 : 6,
                    width: selected ? 1.6 : 1,
                });
                if (selected) framed.push(commanded);
                notes.push({
                    point: commanded, text: arm + ' commanded TCP',
                    colour: palette.ask, dx: 10, dy: -10,
                });
            }

            const preview = state.goalPreview[arm];
            if (preview) {
                const geometry = goalPreviewGeometry(preview);
                if (selected) framed.push(geometry.target);
                if (geometry.points) {
                    addPolyline(items, basis, geometry.points, {
                        colour: palette.ask,
                        width: selected ? 1.6 : 1.1,
                        alpha: selected ? 0.85 : 0.4,
                        dash: [6, 4],
                    });
                    addDot(items, basis, geometry.centre, {
                        colour: palette.ask,
                        radius: 2.5,
                        alpha: selected ? 0.8 : 0.4,
                    });
                }
                addCross(items, basis, geometry.target, {
                    colour: palette.ask,
                    radius: 7,
                    width: selected ? 1.5 : 1,
                });
                if (geometry.triad) {
                    addTriad(items, basis, geometry.triad, {
                        colour: palette.ask,
                        alpha: selected ? 0.85 : 0.4,
                        length: 0.06,
                        width: 1.4,
                    });
                }
                notes.push({
                    point: geometry.target,
                    text: arm + ' requested goal preview',
                    colour: palette.ask, dx: 10, dy: -24, mono: true,
                });
            }
        }

        if (state.obstacles.length) drawObstacles(items, basis);
        items.sort(function (a, b) { return b.depth - a.depth; });
        for (let i = 0; i < items.length; i++) items[i].paint();

        paintOverlay(basis, notes);

        const nothing = !state.row.right && !state.row.left &&
            !state.goalPreview.right && !state.goalPreview.left &&
            !state.obstacles.length;
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

    function drawObstacles(items, basis) {
        const style = { colour: palette.ink, width: 1.2, alpha: 0.75 };
        for (const obstacle of state.obstacles) {
            if (!obstacle || obstacle.enabled === false) continue;
            if (obstacle.shape === 'cylinder') {
                addCylinder(items, basis, obstacle, style);
            } else if (obstacle.shape === 'box') {
                const c = obstacle.center_mount_m;
                const h = obstacle.half_extent_m;
                addBox(items, basis,
                    [c[0] - h[0], c[1] - h[1], c[2] - h[2]],
                    [c[0] + h[0], c[1] + h[1], c[2] + h[2]], style);
            }
        }
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
        // row is one parsed telemetry row, keys by CSV column NAME.
        setTelemetry: function (arm, row) {
            if (!known(arm)) return;
            state.row[arm] = row || null;
            schedule();
        },
        // preview is parseGoalPreview's output (or null to clear). Drawing
        // only: setting a preview saves nothing, solves nothing, and sends
        // nothing — there is no code path from here to a request.
        setGoalPreview: function (arm, preview) {
            if (!known(arm)) return;
            state.goalPreview[arm] = preview || null;
            schedule();
        },
        // Configured mount-frame primitives. Drawing only: no SDF, epsilon,
        // collision or save behavior lives in this module.
        setObstacles: function (obstacles) {
            state.obstacles = Array.isArray(obstacles)
                ? obstacles.filter((obstacle) => {
                    if (!obstacle || !Array.isArray(obstacle.center_mount_m) ||
                        obstacle.center_mount_m.length !== 3 ||
                        !obstacle.center_mount_m.every(Number.isFinite)) return false;
                    if (obstacle.shape === 'cylinder')
                        return Number.isFinite(obstacle.radius_m) && obstacle.radius_m > 0 &&
                               Number.isFinite(obstacle.height_m) && obstacle.height_m > 0;
                    if (obstacle.shape === 'box')
                        return Array.isArray(obstacle.half_extent_m) &&
                               obstacle.half_extent_m.length === 3 &&
                               obstacle.half_extent_m.every((value) => Number.isFinite(value) && value > 0);
                    return false;
                })
                : [];
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
