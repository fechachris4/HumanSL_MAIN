# World-hold slice — design (slice 2 of the world-frame work)

Date: 2026-08-13. Status: awaiting Christian's approval; no code yet.
Parent derivation: `Christian_control/docs/thesis/world-frame-hold-derivation.md`
(equations (A1)–(A5), (B5), §5 sample contract, §6 slices).
Decisions baked in (Christian, this date, interactive): **auto-engage when
fresh**, **50 ms freeze + re-anchor on recovery**, **no Nexus-reads gate —
the first tethered run verifies signs itself**.

## 1. The behaviour, in equations first

Anchor, at the engage instant `t0` (first cycle where the Vicon sample is
fresh and the needed segments valid):

```
world_T_B(t)  = world_T_MS(t) · MS_T_B                    (A1, ZOH sample)
X_des^W       = world_T_B(t0) · FK(q(t0))                  (anchor once)
```

`MS_T_B` in this slice: `MSeg_T_mount` is ASSUMED IDENTITY (the Nexus
template origin taken as the URDF `mount` frame — unverified until the
GUI reads); `mount_T_base_link` comes from the Pinocchio model, computed
once at startup. The §8 argument of the derivation doc applies: a hold
mostly cancels a constant error here; the residual scales with base
rotation, and the first run is the sign check.

Per cycle while engaged (500 Hz):

```
ref^B(t)      = T_B_W(t) · X_des^W                         (the seam image)
e_p^W         = X_des^W.p − [world_T_B(t)·FK(q)].p         (logged, watched)
```

The existing reactive law then runs UNCHANGED on `ref^B` through the
existing `PoseReference` seam — pose error, DLS, null space, velocity
clip, integration, every guard. **Algebraic identity with Architecture
A:** `e^B = R_BW · e^W` and `DLS(J^B)(R_BW V^W) = DLS(J^W) V^W` row-for-
row, so the commanded `q̇` is bit-for-bit the Architecture-A command; the
world-frame quantities are computed and logged in world (mirroring
`frames.py`), and the seam carries their base-frame image. When
`V_base,E` arrives, the subtraction enters the same way
(`twist^B = R_BW(V_des^W − V_base,E)`) — no rework, which was the point
of choosing A.

Ramp-in (new, risk-shaping for auto-engage): for `kWorldHoldRampS` after
every (re-)engage, the reference is interpolated from the engage-time
pose toward `ref^B(t)`:

```
ref_used^B = FK(q(t0'))·(1−r) ⊕ ref^B(t)·r,   r = UnitRamp(t−t0', kWorldHoldRampS)
```

(⊕: position lerp + rotation slerp; `t0'` = this engage's instant.) A
wrong sign therefore appears as a slow, visible drift the operator kills,
not a step at full gain. `UnitRamp` is the pattern the law already uses
for limit-avoidance.

## 2. State machine

```
        takeover hold (unchanged, today's path)
              │ auto: sample fresh (age < kWorldHoldFreshMaxAgeS)
              │       AND Mount segment valid          [checked each cycle]
              ▼
   ┌─── WORLD_HOLD (anchored, ramping in) ──────────────┐
   │  fresh: ref^B = T_B_W·X_des^W                      │
   │  stale/invalid > 50 ms: FREEZE ref^B at last good  │
   │      (= today's base-frame hold; logged, no stop)  │
   │  recovered: RE-ANCHOR X_des^W at current pose,     │
   │      ramp in again (hold_reanchor_count++)         │
   └── divergence: ‖e_p^W‖ > kWorldHoldMaxErrorM ───────┘
              │
              ▼
   joint hold + WORLD-HOLD LATCHED OFF for this run
   (printed + logged; prevents an engage/diverge/engage
    oscillation when the sign is wrong — degradation to
    today's behaviour, never a stop)
```

A trajectory arriving on the stdin pipe disengages the world hold in
favour of the trajectory (existing behaviour wins); after the trajectory
completes, re-engage happens through the same fresh+valid check and ramp.

## 3. Files and why

- `src/WorldHold.h` (new) — pure logic, no SDK/Kortex: anchor state,
  freshness/freeze/re-anchor/divergence-latch state machine, the ramped
  reference computation. Everything unit-testable with synthetic samples.
- `src/Targets.h/.cpp` — a `WorldHoldSource : ReferenceSource` wrapping
  WorldHold; produces the `PoseReference`; composes with the existing
  joint-trajectory source (trajectory wins while active).
- `src/Config.h` — `kWorldHoldAutoEngage=true`, `kWorldHoldFreshMaxAgeS
  =0.05`, `kWorldHoldRampS=2.0`, `kWorldHoldMaxErrorM=0.08`. (Config.h
  currently carries the uncommitted velocity-limit edit; these constants
  are appended without touching it.)
- `src/Runner.h/.cpp` — passes the slot sample into the source (it
  already reads it for logging); no other loop change.
- `src/Hardware.h/.cpp` — log_format 10 → 11: `hold_state` (0 off / 1
  engaged / 2 frozen / 3 latched-off), `world_err_m`, `world_err_rot_rad`,
  `hold_ramp`, `hold_reanchor_count`. Schema test updated with it.
- `src/Kinematics.*` or startup code — extract `mount_T_base_link` once
  from the model.
- tests: `tests/test_world_hold.cpp` (new) — engage/freeze/re-anchor/
  divergence-latch/ramp, ZOH reuse, NaN handling; plus the identity check
  `DLS(J^B)(R_BW V^W) ≡ DLS(J^W) V^W` on random poses, tying the
  implementation to the derivation doc's claim.

## 4. Offline acceptance (before any hardware)

- All existing tests unchanged and green (no behaviour change while the
  hold is disengaged or latched).
- WorldHold unit tests cover: engage gating, freeze at exactly the
  threshold, re-anchor producing zero instantaneous error, ramp
  continuity (no reference step at engage/freeze/recover boundaries),
  divergence latch one-way, trajectory precedence.
- The A↔seam identity test passes at 1e-12 on randomized configurations.

## 5. Supervised bring-up (Christian present, e-stop in hand)

Per his decision the first run IS the sign check: tethered, low Cartesian
gains (panel: drop `kp`/`kp_rot` to ~25% first), workspace clear, watch
`world_err_m` on engage — shrinking ⇒ signs right, growing ⇒ ramp +
divergence latch demote it to joint hold and the run continues. Then
nominal gains; then the sway test (wearer/rig moved gently while the tip
is watched against a fixed room reference). Deliberate Mount occlusion
mid-hold exercises freeze/re-anchor live. Every claim afterwards cites
`world_err_m` from the log, not impressions.

## 6. Deliberately out of scope

`V_base,E` and any velocity estimation (next slice, per the feedback-only
decision); torso targets; calibration beyond the identity assumption;
panel display changes beyond the columns appearing (the name-based reader
already surfaces them).
