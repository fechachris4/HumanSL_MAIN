# Artifact review ledger — 2026-08-14

Every issue raised by the 2026-08-14 red-team review of the two published
documentation artifacts ("Gen3 Command Path" and "HumanSL_MAIN — Architecture
Map"), with its outcome. The review ran as a 14-agent workflow: seven finder
lenses (math, controller, planner, vicon, pedagogy, visual, consistency),
each paired with an adversarial verifier that re-checked every quote, source
line, computation and geometry claim against the repository before a finding
could count as confirmed. A second 5-agent workflow then filled the content
gaps, ran a final compliance sweep (which caught 14 residuals, including one
defect the fix pass itself had introduced), and generated the artifacts'
"Open issues" section (Command Path §12).

Status meanings: **fixed** — the artifacts were corrected the same day;
**open** — the fix needs a code change, a hardware/lab observation, or
Christian's decision (tracked in Command Path §12); **refuted** — the paired
verifier showed the finding was wrong or already handled, so it was
deliberately not applied.


## Lens: math — 8 confirmed, 0 refuted

### math-0 [high] — fixed 2026-08-14
- **Where:** File A §0 Fact 1, §0 'two live paths' diagram, §3.6 heading + intro, §5 completion note, §7.3 heading, §8.1; File B header, §2 intro, Fig. 2 caption, Cartesian-law note, LIVE card
- **Issue:** Both artifacts state unconditionally that every idle-hold cycle runs the Cartesian law, with the re-seated base-frame Cartesian hold as the no-Vicon fallback. The code disagrees: before the world hold has ever engaged, an idle hold with a joint reference and no fresh Vicon runs the JOINT tracking law (Controller.cpp:207-226: `if (!world_hold_ever_engaged_ && reference.joint)` returns SolveJointTracking, with the comment 'Before the world hold has EVER engaged ... the no-Vicon behaviour is bit-for-bit today's'). The Cartesian law runs on idle holds only when the world hold provides a target (fresh Vicon) or after a first engage (frozen/latched fallback). File A contradicts itself: §3.6b's source note correctly says 'with no Vicon at all, fresh is never true and the pre-Vicon joint hold runs unchanged.'
- **Why it matters:** The most common bench scenario — controller run without Vicon, or before the first fresh sample — behaves opposite to the headline claim. A student would expect taskvel_j*/nullvel_j*/null_leak_mps telemetry and, critically, the null-space joint-limit avoidance to be active during every hold. File B's safety claim that the graded null-space push-back near the j2/4/6 limits 'is finally on a live path during holds' is true only with fresh Vicon or after a first engage; on a Vicon-less session the only limit protection during holds remains the hard boundary stop.
- **Affected:** Any student debugging a Vicon-less or pre-engage run; anyone reasoning about which soft-safety layers are active during holds
- **Resolution applied:** Qualify every 'every idle-hold cycle' statement in both artifacts: the Cartesian law runs on idle holds once the world hold has engaged at least once (fresh Vicon), and on all subsequent idle holds via the frozen/latched fallback (re-seated Cartesian base-frame hold); before the first engage, an idle hold runs the joint tracking law exactly as pre-Vicon. Fix File B's null-space push-back claim and Fig. 2 caption to state those conditions.
- **Verifier's correction:** Verified exactly as reported. One extension: File A §5's completion note ('what happens after step 10') is wrong in the same way — after a trajectory completes on a Vicon-less run, the endpoint is held by the joint law (the idle joint reference IS the endpoint), not by a re-seated Cartesian base-frame hold, so that note needs the same qualification.

### math-1 [medium] — fixed 2026-08-14
- **Where:** §4, note 'the timestamp columns lose their resolution within seconds' — the four-row resolution table
- **Issue:** Every row of the resolution table is one decade too coarse. With six-significant-digit default stream formatting (confirmed: Hardware.h:493 'rely on the stream's default formatting (six significant digits)'), the correct resolutions are: t in [1,10) s -> 0.01 ms; [10,100) -> 0.1 ms; [100,1000) -> 1 ms; [1000,10000) -> 10 ms. The note's own examples prove the table wrong: '2800.12' is six significant digits at 0.01 s = 10 ms resolution (the table claims 100 ms for t < 10000 s), and 'the p99 of 10 ms and max of 20 ms ... are one and two quanta' is only arithmetically possible with 10 ms quanta.
- **Why it matters:** This is a reference document the student is told to recompute from. The table cannot be reproduced from the stated 6-significant-digit rule and contradicts the worked example two sentences later. It also understates the usable window: a 0.34 ms round trip remains resolvable (about 3 quanta at 0.1 ms) until t of roughly 100 s, not just 'the first minute or so' — a student following the table would discard recoverable data and lose trust when their own arithmetic disagrees.
- **Affected:** Anyone doing latency analysis on run CSVs; any student who checks the arithmetic (the intended audience)
- **Resolution applied:** Shift the table one decade (t<10 s -> 0.01 ms, t<100 s -> 0.1 ms, t<1000 s -> 1 ms, t<10000 s -> 10 ms) and restate the practical cutoff: round-trip measurement is quantisation-limited beyond roughly 100 s. The 2800.12 example and the one/two-quanta sentence then become consistent.

### math-2 [medium] — fixed 2026-08-14
- **Where:** §0 Fact 2 ('at K_p = 10 s^-1, a 0.1 m/s walk leaves the tool ~1 cm behind its anchor'), §8.4 ('steady base motion leaves e_ss ~ v_B/K_p'), §9 structural note (the e_ss line as the feedback-only signature)
- **Issue:** The e_ss ~ v_B/K_p prediction neglects the Kd damping term, which is compiled on (Config.h:218 kKdPosition = 0.5, Config.h:221 kVelocityTermEnabled = true) and subtracts the measured twist. During steady base motion at v_B the world-hold equilibrium has the tool's base-relative twist equal to -v_B, so the damping term opposes the compensating motion: solving p_dot = v_B + Kp*e/(1+Kd) = 0 gives e_ss ~ (1+Kd)*v_B/Kp = 1.5*v_B/Kp, roughly 1.5 cm at 0.1 m/s — 50% above the stated line (plus a small further increase from the DLS lambda-squared attenuation). The §3.5 joint-law version e_ss = d/Kp is correct because that law has no Kd term.
- **Why it matters:** The document explicitly designates this number as the quantitative signature to watch on hardware (world_err_m vs base speed). A student comparing measured hold error against v_B/Kp would see a systematic ~50% excess and could misattribute it to the missing mountseg_T_mount calibration or a frame error — the exact wrong-diagnosis failure this page exists to prevent.
- **Affected:** Anyone validating the world hold on hardware or writing the feedback-only steady-state analysis into the thesis
- **Resolution applied:** State the prediction with the damping term: e_ss ~ (1+K_d)*v_B/K_p for the compiled gains (K_d = 0.5 on measured twist, reference twist zero during holds), i.e. about 1.5 cm at 0.1 m/s; keep v_B/K_p only as the K_d = 0 lower bound. Update all three occurrences.

### math-3 [medium] — fixed 2026-08-14
- **Where:** Source citations across §1 (boundary table), §3.1, §3.4, §3.5, §3.6, §4, §5 — e.g. §3.6 'applied at Controller.cpp:121-132' and 'ramp applied Controller.cpp:177-179'
- **Issue:** Several file:line citations no longer match the working tree the page claims to document ('every claim cites file:line'). Verified wrong: §3.6 eq 1-2 'applied at Controller.cpp:121-132' (the pose/twist errors are at Controller.cpp:236-249; 121-132 is the pose-sequence re-arm and FK block); §3.6 'ramp applied Controller.cpp:177-179' (the null ramp is at 298-300 via `ramped_gains.limit_avoid_gain_s_inv *= UnitRamp(...)`; 177-179 is reference-twist/WorldHoldInput code); §3.5 step 2 'WrappedJointError, State.h:135-144' (actual definition State.h:166-175); §3.4 'Runner.cpp:352-357' for the per-cycle measurement (the in-loop deg-to-rad conversion is at 399-403; 352-354 is the one-time T5 seeding) and delta-t 'Runner.cpp:333-339' (actual 380-386); §3.6 gains 'Config.h:186, 208-213' (kKpCartesian is at 195, kKpRotation 217, Kd gains 218-219); §5 'Config.h:153 compiles kTakeoverHoldS' (actual 162); §1 boundary table 'State.h:158-161' for Reference (actual 189-199), 'State.h:114' for PoseReference (actual 136-154), 'State.h:149' for JointReference (actual 180-183). Citations checked around them (Controller.cpp:78, 112-116, 151-177, 180-206; WorldHold.h ranges) are correct, so the drift is localized, not systemic.
- **Why it matters:** The document's central trust device is jump-to-line verification. A student following §3.6's citation lands in the dead pose-channel code instead of the live error computation — actively misleading, not merely stale. At the ramp citation there is also a small formula/code mismatch: the code scales the objective's gain before projection rather than multiplying N*objective by ramp(t) afterwards — algebraically identical (both linear), but a reader comparing the doc's q_dot_null = N*objective*ramp(t) against the cited lines finds neither the formula nor the ramp.
- **Affected:** Every reader who follows a citation into the source — the document's stated primary workflow
- **Resolution applied:** Re-resolve the Controller.cpp, State.h, Runner.cpp, Config.h and Hardware.h line citations against the current working tree, and at the ramp citation note that the code applies the ramp as a gain scale inside the objective (Controller.cpp:298-300), equivalent to scaling the projected result.
- **Verifier's correction:** Reviewer's substance verified against the working tree with three line-number refinements: WrappedJointError is at State.h:166-175 (reviewer said 163-174 — the doc comment starts at 156, the function at 166); Reference is State.h:189-199; JointReference State.h:180-183. One additional instance of the same drift, not in the reviewer's list: §4's citation 'Hardware.h:403-405' for the six-significant-digit formatting note — that comment is at Hardware.h:493-494 (403-405 is the vicon_age_s field).

### math-4 [low] — fixed 2026-08-14
- **Where:** §7.3 equation block ('projector N = ... diag(0.00990 x6, 1); q_dot_null,2 = 0.00990 x (-0.45728) = -0.004528 rad/s')
- **Issue:** The worked example silently drops the ramp factor that §3.6 defines as part of the law ('q_dot_null = N * objective * ramp(t), ramp(t) = clamp(t / 1.0 s, 0, 1)') and never states the assumption t >= 1 s. All arithmetic that is shown is correct — independently verified: q_dot_task = 0.5/1.01 = 0.4950 rad/s = 28.36 deg/s; N = diag(1 - 1/1.01 = 0.009901 x6, 1); excess = 120 - 106.9 = 13.1 deg = 0.22864 rad; objective_2 = -0.45728 rad/s; q_dot_null,2 = -0.004528 rad/s = -0.259 deg/s; and joint 2's column of J = [I6|0] is the linear-y task row, so the 4.53 mm/s leak direction claim is right.
- **Why it matters:** A student reproducing the example from §3.6's equations during the first second after takeover (or in a fresh unit test with t < 1 s) gets a number up to 100% smaller with no stated assumption to reconcile against — the 'example contradicts stated rules' trap in a page built for hand-verification.
- **Affected:** Students hand-verifying §7.3 or writing tests against it
- **Resolution applied:** Add one line to the givens: 't >= 1 s since takeover, so ramp(t) = 1 and the null ramp drops out.'

### math-5 [low] — fixed 2026-08-14
- **Where:** §3.6b deck ('All five knobs are compiled and echoed into every CSV preamble') and §6 parsing-trap note ('now including the five world_hold_* knobs and vicon_source')
- **Issue:** There are six world_hold_* preamble keys, not five. Main.cpp:136-146 writes world_hold_auto_engage, world_hold_fresh_max_age_s, world_hold_ramp_s, world_hold_max_error_m, world_hold_max_rot_error_rad, and world_hold_reanchor_after_s; Config.h defines the matching six kWorldHold* constants, and §3.6b's own equations use all six values.
- **Why it matters:** A student inventorying a CSV preamble against the doc finds six keys where five were promised and must guess which one the doc considers extra — needless friction in an otherwise exact section.
- **Affected:** Anyone parsing or auditing the CSV preamble
- **Resolution applied:** Change both occurrences of 'five' to 'six', or enumerate the keys.

### math-6 [low] — fixed 2026-08-14
- **Where:** §3.6 eq 1 ('e_pos = p_des - p_cur [m, base frame]') versus §3.6b latch block ('e_pos = ||p_hat - p_cur|| ... r*e_pos > 0.08 m'); §3.5 step 2 ('in [-pi, pi]') versus §3.6 eq 5 ('q_tilde_i = remainder(q_i, 2pi) [signed, (-pi, pi]]')
- **Issue:** Two notation inconsistencies in the maths reference. (1) e_pos is a 3-vector in §3.6 but a scalar norm in §3.6b's latch inequality, with no notational distinction. (2) The same std::remainder(., 2pi) operation is given two different ranges — [-pi, pi] in §3.5 and (-pi, pi] (with a stray double bracket ']]') in §3.6; std::remainder's actual range is [-pi, pi] (remainder(-pi, 2pi) returns -pi under round-half-to-even).
- **Why it matters:** In a document students are told to recompute by hand, a symbol that changes type between adjacent sections and an interval convention that flips invite exactly the branch-cut confusion the vocabulary table works to prevent. Small, but the page's whole value is precision.
- **Affected:** Students hand-deriving the latch condition or the wrap behaviour at +/-180 deg
- **Resolution applied:** Write the latch quantities as norms with distinct names (e.g. eps_pos = ||p_hat - p_cur||, eps_rot = ||log3(R_hat R_cur^T)||), use one wrap-range convention ([-pi, pi], matching std::remainder) in both places, and fix the ']]' typo.
- **Verifier's correction:** The (-pi, pi] range in §3.6 was evidently transcribed from the source comment in ReactiveLaw.h ('wrapped to (-pi, pi] because Kortex reports positions in [0, 360)'), which is itself imprecise about std::remainder. The doc's own §3 policy — 'where the code and a comment disagree, the code is what is written here' — says the fix is [-pi, pi] in both places.

### math-7 [low] — fixed 2026-08-14
- **Where:** §3 safety table, row 'Vicon freshness gate': 'trips on: sample age > 50 ms, invalid segment, or repeated frame'
- **Issue:** 'Repeated frame' is not a trip condition of the freshness gate. Runner.cpp:425-429 computes fresh = mount.valid AND sequence > 0 AND isfinite(age) AND age <= 0.05 s — there is no repeated-frame test. A repeated or stalled Vicon stream trips the gate only indirectly, once the zero-order-held sample's age grows past 50 ms; a single repeated frame at 100 Hz stays fresh.
- **Why it matters:** A student auditing the gate against the table would search for repeated-frame detection logic that does not exist, or wrongly expect a one-frame hiccup to freeze the hold. File A states the actual condition correctly in both §3.6b and the §6 debugging table, so the map is the odd one out.
- **Affected:** Readers using the map's safety table as the gate inventory
- **Resolution applied:** Replace 'or repeated frame' with the actual mechanism, e.g. 'sample age > 50 ms (a stalled/repeated stream trips this via ZOH age), invalid segment, or no sample yet (seq = 0)'.


## Lens: controller — 15 confirmed, 0 refuted

### controller-0 [critical] — fixed 2026-08-14
- **Where:** Section 1, Fig. 1 SVG, Kinova Gen3 box (architecture-map.html line 276): "right .9 / left .10"
- **Issue:** The two arms' IP addresses are swapped. Config.h defines kRightRobotIp = "192.168.1.10" and kLeftRobotIp = "192.168.1.9" (Config.h:33-34) — the right arm is .10 and the left arm is .9, the inverse of what the diagram says.
- **Why it matters:** This is the page a newcomer uses to identify hardware. File A's recovery table sends the student to "the Kinova web dashboard at the arm's IP" to clear faults; with this diagram they act on the wrong physical arm. Worse, a student preparing a supervised run could clear the workspace around the arm labeled .9 believing it is the one `--arm right` will move, while the physical .10 arm actually moves. The map's own ORPHANED card mocks test_kinova.cpp for "labels the .9 arm wrongly" — and then repeats the same class of error.
- **Affected:** Any student doing hardware-adjacent work: dashboard fault clearing, network diagnosis, physically identifying which arm a run will command.
- **Resolution applied:** Change the label to "right 192.168.1.10 / left 192.168.1.9" — print the full IPs, since the per-arm ProcessLock paths and log prefixes also key on them.

### controller-1 [high] — fixed 2026-08-14
- **Where:** Command Path §0 Fact 1 (line 423, "an idle hold falls through to the Cartesian resolved-velocity law ... toward the re-seated base-frame hold pose otherwise"), §3.6 heading (line 1040), §7.3 heading (line 1774); Architecture Map §2 (line 336), Fig 2 caption (lines 398-404), the LIVE note (line 411), and the "Cartesian law + world hold" card (line 490)
- **Issue:** The idle-hold dispatch is three-way, not two-way. Controller.cpp:213-226 shows that until the world hold has engaged at least once in the run (!world_hold_ever_engaged_ && reference.joint), an idle-hold cycle runs the JOINT-space law on the idle joint reference — the pre-Vicon behaviour, not the Cartesian law. The Cartesian base-frame fallback described as "otherwise" exists only after a first engage. A run with no Vicon at all never enters the Cartesian law on any cycle. File A states this correctly exactly once, buried in §3.6b's source note (line 1151: "with no Vicon at all, fresh is never true and the pre-Vicon joint hold runs unchanged") — directly contradicting its own §0, §3.6 and §7.3.
- **Why it matters:** A student running or replaying a Vicon-less session (common — the Vicon PC is in the lab) will expect Cartesian telemetry (taskvel_j*, nullvel_j*, sigma_min, pd_*/p_*) and the null-space limit avoidance to be active during holds; instead those columns are NaN and the joint law ran. They would misdiagnose that as a bug, or believe the Map's LIVE-note claim that the graded null-space push-back "is finally on a live path during holds" when on their no-Vicon run it is not.
- **Affected:** Anyone debugging idle-hold behaviour or telemetry on runs without fresh Vicon; anyone reasoning about which soft-safety terms are active during holds.
- **Resolution applied:** State the three-way dispatch everywhere the two-way version appears: active trajectory → joint law; idle hold after first world-hold engage → Cartesian law (world anchor if fresh, re-seated base-frame hold if stale/latched); idle hold before any engage (including every Vicon-less run) → joint-space hold, bit-for-bit the pre-Vicon behaviour. Fix File B's Fig 2 fork and the LIVE note to match.

### controller-2 [high] — fixed 2026-08-14
- **Where:** Command Path §4 warn note (line 1240: "the CSV preamble of a run, not today's Config.h, is what tells you which limits that run obeyed"), §6 (line 1686: "#-prefixed lines carry every compiled setting"), §11 src note (lines 1999-2002: "a run's CSV preamble records the compiled values it ACTUALLY had"); Architecture Map §5 (line 548: "trust a run's CSV preamble — not today's source — for the limits it obeyed")
- **Issue:** The CSV preamble does not record the velocity limits. WriteConfigLines (Main.cpp:81-147) echoes gains, tolerances, guard overrides, stop counters, timing and the six world-hold knobs — but no line for kQdotLimitDegS / kModelVelocityLimitsDegS, and none for the software joint boundaries either; WriteCsvPreamble (Main.cpp:149-159) adds only log_format, vicon_source and the arm/IP on top. The one setting both pages tell the reader to look up in the preamble — which velocity clip a binary was built with, the whole point of the 45 vs 76/66.5 warning — is precisely the setting the preamble omits. "carry every compiled setting" is false as stated.
- **Why it matters:** A student follows the recommended procedure — grep '^#' run.csv to learn which clip an old run obeyed — finds no velocity line, and either concludes the run predates limit logging or mistakes another line for it. The advice fails exactly in the scenario it was written for (binaries predating the uncommitted 2026-08-13 raise). The only current evidence is indirect: session.json's binary SHA-256 or the run date versus the decision date.
- **Affected:** Anyone auditing an old run's velocity limits — the exact task §4's warning box sets up; anyone trusting "self-describing without the console".
- **Resolution applied:** Correct both pages to say the preamble records gains, guards, hold knobs and timing but NOT the velocity clip or the software joint boundaries, and point to session.json's binary hash / build time as the evidence for limits; note that one line("qdot_limit_deg_s", ...) in WriteConfigLines would close the gap. Do not leave advice that cannot be executed.
- **Verifier's correction:** The reviewer's claim is exactly right; note additionally that File A's §11 build-section source note repeats the same false claim a third time, so the fix must touch three places in File A plus one in File B.

### controller-3 [medium] — fixed 2026-08-14
- **Where:** §4 Group D, "Goal pose prior" row (line 1346): "σ_rpy 0.01; σ_xyz (0.01, 0.1, 0.01)" and the consequence note "y is 10× looser by inheritance — plans drift in y and land accurately in x, z"; echoed in §6's GPMP2 row and §9's tracking-accuracy cell
- **Issue:** These are HEAD's values, not the working tree the page claims to document. The same uncommitted diff that raised the pacing (which the page DID pick up: "was 0.05 / 4.0") also tightened the goal position sigmas 10× to [0.001, 0.01, 0.001], because at the new 1.0 s pacing the smoothness prior outweighed the old anchor — the canonical 0.2 m test move missed by 82 mm at the old values and lands within 3.2 mm at the new ones.
- **Why it matters:** The page's provenance promises "HEAD 02348ecc + uncommitted working tree". A student tuning the planner or explaining a plan's goal error in the thesis would quote sigmas that are 10× off, and would miss the documented 82 mm → 3.2 mm behaviour change — evidence directly relevant to §6's "drift in y" debugging row.
- **Affected:** Anyone reading Group D as the factor-graph weight reference; anyone debugging goal accuracy via §6's GPMP2 row.
- **Resolution applied:** Update the row to σ_xyz (0.001, 0.01, 0.001) (working tree; HEAD: 0.01/0.1/0.01), keep the "y 10× looser" ratio note (the ratio was preserved), and mention the 82 mm miss that forced the tightening.

### controller-4 [medium] — fixed 2026-08-14
- **Where:** §4 warn note (lines 1399-1408), the resolution ladder: "t < 10 s → 0.1 ms resolution ... t < 100 s → 1 ms ... t < 1000 s → 10 ms ... t < 10000 s → 100 ms"
- **Issue:** Every bracket in the ladder is one decade too coarse. Six significant digits (Hardware.h:493 confirms the stream default) give: t∈[1,10) → 10 µs steps, t∈[10,100) → 0.1 ms, t∈[100,1000) → 1 ms, t∈[1000,10000) → 10 ms. The page's own worked sample proves it: "2800.12" is a 10 ms quantum (the ladder says 100 ms at that t), and the closing sentence "the p99 of 10 ms and max of 20 ms ... are one and two quanta" is only true with the correct 10 ms quantum — the note contradicts itself four lines apart.
- **Why it matters:** This box exists to teach precision discipline; a student recomputing the resolution from a real file (trivial) finds the table off by 10× and rightly starts doubting the rest of the page. Anyone sizing when latency measurements become quantisation artefacts gets the crossover time wrong by an order of magnitude.
- **Affected:** Anyone doing latency analysis from run CSVs; anyone spot-checking the page's arithmetic.
- **Resolution applied:** Shift each row one decade: t<10 s → 10 µs, t<100 s → 0.1 ms, t<1000 s → 1 ms, t<10000 s → 10 ms; the 2800.12 example and the one-and-two-quanta conclusion then agree with the table. The "0.34 ms round trip is 3 quanta" aside must also be redone (it becomes 34 quanta at 10 µs).
- **Verifier's correction:** One extra fix beyond the reviewer's: the first row's own example ("a 0.34 ms round trip is 3 quanta") was built on the wrong quantum and must be updated alongside the ladder, or it reintroduces the inconsistency.

### controller-5 [medium] — fixed 2026-08-14
- **Where:** Throughout — provenance claims "every claim cites file:line"; e.g. §5 src "Config.h:153 compiles kTakeoverHoldS", §3.6 "gains Config.h:186, 208–213", §3.3 "kArrivalDwellS (Config.h:352–358)", §3.4 "Runner.cpp:333–339" / "Runner.cpp:352–357", §5 "Runner.cpp:194–195", §8.5 "Runner.cpp:505–538, 577", §4 "Hardware.h:403–405", §5 note "Hardware.h:286–291", §3.6 "applied at Controller.cpp:121–132" and "ramp applied Controller.cpp:177–179", §2 "Main.cpp:337" / "Main.cpp:562–563", §3.1/§3.5 State.h:114/135–144/158–161
- **Issue:** Roughly fifteen file:line citations are stale by 9–90 lines against the working tree. Independently verified: kTakeoverHoldS is Config.h:162 (153 is kControlDtS); the Cartesian gains are Config.h:195/213/217-218; kArrivalDwellS is Config.h:390 (352–358 is unrelated); the Δt clamp (ClampedCycleDt call) is Runner.cpp:384; the stale "exactly 0.5 s" comment is Runner.cpp:239; loop-thread cout spans Runner.cpp:581–651; the six-digit formatting comment is Hardware.h:493; the cross-exchange rule is Hardware.h:306–311; the null-avoidance ramp is applied at Controller.cpp:298–300; ClearFaults is Main.cpp:354 and the signal handlers Main.cpp:593–594; PoseReference is State.h:136, WrappedJointError State.h:167, Reference State.h:189. Several other citations are exact (Controller.cpp:78, Config.h:241, StopPriority.h:43–58, Runner.cpp:425–429, Safety.cpp:178).
- **Why it matters:** The page's stated contract — jump from any claim straight to the code — silently fails about one time in four. A student landing on Config.h:153 for the takeover hold reads kControlDtS instead; worse, some misses land on lookalike code (Runner.cpp:352–357 lands near line 348's takeover-hold copy of the identical feedback conversion), so the reader may not even notice they are in the wrong place. The systematic drift erodes exactly the trust the provenance block asks for.
- **Affected:** Every reader who uses the citations as entry points — the page's declared purpose.
- **Resolution applied:** Re-resolve every citation against the current working tree (the drift pattern suggests the page was written against a pre-panel-edit Config.h and an earlier Runner.cpp). Prefer symbol name plus line ('Config.h kTakeoverHoldS, :162') so future drift degrades to a search instead of a wrong landing.
- **Verifier's correction:** Minor adjustments to the reviewer's resolved lines: the gains sit at Config.h:195 (kKpCartesian), 213 (kDlsLambda), 217–218 (kKpRotation/kKdPosition); the Δt clamp call is Runner.cpp:384 and the in-loop feedback conversion Runner.cpp:401. The overall count and pattern stand.

### controller-6 [medium] — fixed 2026-08-14
- **Where:** §3 safety table, "During idle holds" row (line 436): "Vicon freshness gate | trips on: sample age > 50 ms, invalid segment, or repeated frame"
- **Issue:** "Repeated frame" is not a trip condition. The freshness test (Runner.cpp:425–429) is: mount segment valid AND sequence > 0 (a sample has ever arrived) AND finite age AND age ≤ 50 ms. Repeated frames are the design: at 100 Hz Vicon under a 500 Hz loop every sample is reused ~5 cycles by zero-order hold — which this same page's §1 input table and File A's vocabulary both explain. If repetition tripped the gate, the hold would freeze four cycles out of five.
- **Why it matters:** A student cross-reading the two pages gets contradictory rules for the single most safety-relevant gate on the world-hold path, and could mis-classify normal ZOH rows (unchanged vicon_seq, growing vicon_age_s) as gate trips when reading a CSV.
- **Affected:** Anyone debugging hold_state transitions or vicon_age_s traces against the Map's table.
- **Resolution applied:** Change the trip list to: "sample age > 50 ms, invalid Mount segment, or no sample has ever arrived (seq 0)". If "repeated frame" was meant to gesture at seq 0, say that; repetition of a fresh frame is normal ZOH.

### controller-7 [low] — fixed 2026-08-14
- **Where:** §4 Group C intro (line 1310): "Resolved every cycle by one pure function, ResolveStopPriority, in this exact precedence" over a 7-row table including "#6 Non-finite command" and "#7 Cycle overrun"
- **Issue:** ResolveStopPriority (StopPriority.h:43–58) takes five stop facts plus the stop_on_fault policy and resolves only rows 1–5. The non-finite and overrun stops are consecutive-cycle counters checked after the priority switch in the Runner (Runner.cpp:556–574, commented '// Decision-12 counters, checked after the independent live-state ...'). The table's effective ordering is correct; the attribution is not.
- **Why it matters:** The page praises StopPriority.h as the testable pure seam; a student opening it to verify the table finds only five reasons and no counters, and cannot tell whether the page or the code is wrong. Rows 6–7 have no coverage in the pure function's tests because they are not in it.
- **Affected:** Anyone verifying or extending stop precedence, or writing the hardware-free tests §8.5 calls for.
- **Resolution applied:** Say: "rows 1–5 are resolved by the pure ResolveStopPriority; rows 6–7 are consecutive-cycle counters checked immediately after it in the Runner, so the table's order is the effective precedence."
- **Verifier's correction:** Downgraded from medium to low: the finding is accurate, but the same artifact's §8.5 already places "the overrun and non-finite counters, the stop dispatch" in Runner.cpp with no coverage and recommends extracting them "as StopPriority.h already does for precedence" — so a careful reader is pointed at the truth elsewhere on the page. The §4 sentence still needs the fix because it is the reader's first contact.

### controller-8 [low] — fixed 2026-08-14
- **Where:** §3.6 eq 5–6 source note (line 1104: "`null_leak_m_s` ... became first-class telemetry") and §7.3 worked example (line 1794: "null_leak_m_s = 4.53 mm/s") vs §6 plots list (line 1680) and §7.3 closing paragraph (line 1799), which use "null_leak_mps"
- **Issue:** The CSV column is named null_leak_mps (Hardware.h:231 column list); null_leak_m_s is the C++ struct field (LoopLogSample, Hardware.h:376). The page uses both spellings without saying they are the same quantity at two layers.
- **Why it matters:** A student grepping a run CSV header for null_leak_m_s finds nothing and may conclude their log predates format 8. Small, but this is a page that teaches by exact column names.
- **Affected:** Anyone locating the leak column in a CSV from the §3.6 or §7.3 text.
- **Resolution applied:** Use the CSV name null_leak_mps everywhere prose refers to telemetry, or note once: "struct field null_leak_m_s, CSV column null_leak_mps".

### controller-9 [low] — fixed 2026-08-14
- **Where:** §3.6b intro (line 1109): "All five knobs are compiled and echoed into every CSV preamble" and §6 (line 1690): "now including the five world_hold_* knobs"
- **Issue:** Six world_hold_* lines are echoed (Main.cpp:136–145): auto_engage, fresh_max_age_s, ramp_s, max_error_m, max_rot_error_rad, reanchor_after_s — matching six Config constants (kWorldHoldAutoEngage plus five numeric thresholds, Config.h:330–346).
- **Why it matters:** A parser or checklist built to expect exactly five preamble lines mis-counts; trivial, but the page invites exact matching against preambles.
- **Affected:** Anyone validating a format-11 preamble against the page.
- **Resolution applied:** Say "all six world_hold_* knobs (the auto-engage switch plus five thresholds)".

### controller-10 [low] — fixed 2026-08-14
- **Where:** §4 warn note (line 1239): "Decision record: docs/motion-limits-map.md"; §11 recovery table (line 2079): "docs/thesis/far-target-joint-limit-stop.md"
- **Issue:** Both paths drop the Christian_control/ prefix. The files live at Christian_control/docs/motion-limits-map.md and Christian_control/docs/thesis/far-target-joint-limit-stop.md; repository-root docs/ contains neither (it holds the audits and intent docs, which the page cites correctly elsewhere).
- **Why it matters:** The page's other doc references ARE repo-root docs/, so a reader has no way to know these two need a different prefix; `ls docs/motion-limits-map.md` fails and the decision record looks missing — exactly when defending the velocity raise in the thesis.
- **Affected:** Anyone following the decision-record links.
- **Resolution applied:** Write both as Christian_control/docs/... and sweep File A for other Christian_control-relative doc paths.

### controller-11 [low] — fixed 2026-08-14
- **Where:** §11 recovery table (line 2071 onward, "When it stops — what each exit_reason means")
- **Issue:** The table omits internal_error, a real exit_reason (Safety.cpp:178, LoopStop::kInternalError — the catch path for exceptions that are neither Kortex nor communication errors, printed at Runner.cpp:699/706 as "internal error").
- **Why it matters:** A student whose run ends with exit_reason=internal_error has no row to consult on the page that promises to cover every exit_reason — on precisely the exit that most needs guidance.
- **Affected:** Anyone triaging an exception-terminated run from the CSV trailer.
- **Resolution applied:** Add a row: internal_error — an unexpected C++ exception escaped the loop (servoing restore still ran); read the console/controller.log for the exception text before anything else.

### controller-12 [low] — fixed 2026-08-14
- **Where:** §3 safety table, during-motion row (line 441): "j2/4/6 software boundary | outward command past Table-39 − 2°"
- **Issue:** The boundary is min(Table-39 upper − 2°, firmware warning) (Config.h:257–276). For j2 that is 126.9° (128.9 − 2), but for j4 and j6 the firmware warning wins: 145.0° (not 147.8 − 2 = 145.8) and 118.0° (not 120.3 − 2 = 118.3). File A's Group A states the min() correctly; the Map's shorthand is wrong for two of the three joints.
- **Why it matters:** A student computing the j4/j6 stop angle from the Map's formula is 0.8°/0.3° outside the real boundary — small, but these are the numbers compared against a stopped run's bounded_q, and the two pages disagree.
- **Affected:** Anyone reconstructing a joint_limit_warning stop from the Map alone.
- **Resolution applied:** Write "outward past the software boundary (min(Table-39 − 2°, firmware warning): 126.9 / 145.0 / 118.0°)".

### controller-13 [low] — fixed 2026-08-14
- **Where:** §1 "Every output" bullet (line 324): "the world-hold evidence quartet hold_state / world_err_m / world_err_rot_rad / hold_ramp"
- **Issue:** Format 11 appends five hold columns, not four — hold_reanchor_count is omitted from the "quartet", though the Map's own §3 freshness row says re-anchors are "counted".
- **Why it matters:** hold_reanchor_count is the one column that reveals silent anchor movement across blackouts — the failure mode the freshness-gate row itself warns about; leaving it out of the evidence list makes the key blackout diagnostic invisible.
- **Affected:** Anyone listing hold evidence columns from the Map when analysing a world-hold run.
- **Resolution applied:** Name all five: hold_state / world_err_m / world_err_rot_rad / hold_ramp / hold_reanchor_count.

### controller-14 [low] — fixed 2026-08-14
- **Where:** §6 "Sampling" row, telemetry column (line 1611): "reconstruct it by evaluating §3.5's Hermite equations on the saved plan file at each row's time_s"
- **Issue:** The trajectory clock is not time_s. It is elapsed_s_ in JointTrajectorySource: it starts at 0 on the activation cycle and advances by the measured, 4 ms-clamped dt each cycle (Targets.cpp:290–292). Following the recipe literally is wrong twice over: time_s counts from loop start, not activation (so any trajectory activated mid-run is evaluated at wildly wrong times), and after clamped stalls (166 clamped cycles in the page's own reference run) the two clocks diverge permanently even after subtracting the activation time.
- **Why it matters:** The same page documents the Δt clamp and the 24.87 ms stall tail; a student following this recipe on that very run computes a reference shifted from what the controller actually sampled and chases phantom tracking error — the precise mis-reading the row exists to prevent.
- **Affected:** Anyone reconstructing q_ref(t) offline for tracking-error analysis.
- **Resolution applied:** Say: "evaluate the Hermite equations at the trajectory clock: the cumulative sum of dt_s from the traj_activated row (equals time_s − t_activate only while no cycle hit the 4 ms integration ceiling)."
- **Verifier's correction:** Sharpened: the reviewer led with the clamp divergence, but the dominant error in the recipe as written is the activation offset — time_s is seconds since loop start, while the trajectory clock starts at zero on activation. The clamp divergence is the secondary, subtler error that survives even after correcting the offset.


## Lens: planner — 8 confirmed, 0 refuted

### planner-0 [high] — fixed 2026-08-14
- **Where:** Section 11 'The two input formats', src note under the goal.yaml example: "An optional obstacle box goes in the same block — axis-aligned, in the arm's own base frame, inside the SDF grid." (file line 2028); repeated in the Section 1 diagram box "--box obstacle / axis-aligned, arm base frame" (file lines 547-548)
- **Issue:** The obstacle box is not read in the arm's own base frame. BridgeMain.cpp parses the box with no frame key of its own (~line 400-404) and converts it at the single frame boundary using the block's declared frame: `parsed.box->center = ToMount(parsed.box->center, declared_frame)` where `declared_frame = parsed.frame` is the block's `frame:` key. The parser comment says it outright: "One `frame:` per arm block governs that block whole — goal and box alike" (BridgeMain.cpp ~320-322). Both example blocks in section 11 use `frame: mount`, so a box added there is mount-frame coordinates. Section 8.5 of the same page (file line 1864) correctly describes centre conversion from the declared frame, contradicting section 11.
- **Why it matters:** A student who follows section 11 and writes base-frame box coordinates into a `frame: mount` block places the modelled obstacle in the wrong location — the repo's own goal.yaml comment records that reading base-frame numbers as mount shifts a point by 0.709 m, and the arm bases are rolled ~69 deg from mount. The planner then avoids empty space while the real obstacle is unmodelled, and on the point-goal route no clearance is ever measured afterwards, so nothing downstream catches it.
- **Affected:** Any student adding an obstacle box for a hardware run; safety-relevant because collision avoidance targets the wrong volume with no downstream check on the point-goal route.
- **Resolution applied:** State that the box, like the goal, is read in the block's declared `frame:` and converted to mount internally; note that a non-mount frame (with a point goal) additionally triggers enclosing-AABB half-extent inflation, and per section 8.5 that inflation is skipped on the traced-path route. Fix the Section 1 diagram label too. Also flag that the repo's own goal.yaml comment ('in THIS block's arm's own base frame only') and the CLI usage text for --box are stale on the same point — the code's block-frame rule wins.
- **Verifier's correction:** Verified and slightly extended: the same wrong claim also appears in the Section 1 architecture diagram ("axis-aligned, arm base frame"), and the repo's --box usage text in BridgeMain.cpp (~line 50 and ~98-100) carries the same stale base-frame wording, so the artifact copied a repo documentation bug that the code itself contradicts.

### planner-1 [medium] — fixed 2026-08-14
- **Where:** Section 1, Fig. 1 planner_bridge box: "validate: limits, clearance, FK" (file line 221, drawn as one unconditional pipeline stage), and 'Every output' bullet: "Planner stdout — the TRAJ block, plus a validation report that states measured quantities (clearance, error percentiles, velocity headroom) before its verdict." (file line 327)
- **Issue:** The map presents clearance/FK validation and the measured validation report as happening on every plan. In source, ValidatePlannedPath has exactly one caller — PlanSolver.cpp:263, inside SolveAlongPath (traced path) — while a point goal gets only ValidateJointPath (BridgeMain.cpp:810), a joint-position sweep with no clearance, no FK fidelity, no dynamics check, and no report. The map never mentions this asymmetry; the companion command-path page calls it "the single most important asymmetry in the planner" (its file line 1495).
- **Why it matters:** The map's own reading order says to read it first, and Fig. 1 is the mental model it installs. A reader who internalises the figure will believe a `--goal` plan with a `--box` was clearance-checked before emission, when the obstacle entered only as a soft cost that can be outvoted and no measurement of the result exists on that route.
- **Affected:** New readers using the map as their mental model of the planner; anyone planning point goals around obstacles.
- **Resolution applied:** Add one caption sentence to Fig. 1: 'The full validate stage (clearance, FK fidelity, dynamics, report) runs on traced paths only; a point goal gets only the joint-position sweep — see the command-path page §4/§5.' Qualify the 'Planner stdout' bullet the same way, since on the common point-goal route no clearance/percentile/headroom report is printed.
- **Verifier's correction:** Confirmed but downgraded from high to medium: the map's step-2 companion page — which the map explicitly directs every new reader to next — covers the asymmetry prominently (§4 warn note, §5, and again in §11's example note), so the doc set as a whole does teach it; the residual defect is that the map alone asserts an unconditional validate stage and an always-present measured report, both false on the point-goal route.

### planner-2 [medium] — fixed 2026-08-14
- **Where:** Section 4, Group D 'Goal pose prior' row (file line 1346): "σ_rpy 0.01; σ_xyz (0.01, 0.1, 0.01)" with "y is 10× looser by inheritance"; echoed in §9 Trade-offs (file line 1891): "The goal prior's y-sigma being 10× looser is inherited, not chosen."
- **Issue:** The quoted position sigmas are the superseded HEAD values. The working tree the page claims to document sets position_sigma_xyz: [0.001, 0.01, 0.001] — tightened 10× on 2026-08-13 with the ratio preserved — in both config/planner.yaml (line 77, with the dated comment) and the OptimizerTuning default (goal_position_sigma_xyz in TrajectoryOptimization.h, ~line 69). The 10×-looser-y ratio survives, but the position magnitudes in the row are one order off, and §9's 'inherited, not chosen' is now false for the magnitude: the current values were deliberately chosen on 2026-08-13 with a measured effect (0.2 m test move: 82 mm miss → 3.2 mm, per the yaml comment and test_plan_solver).
- **Why it matters:** A student verifying the page against planner.yaml finds numbers 10× different and loses trust in the rest of the table; anyone tuning goal accuracy or writing up the planner's weighting uses the wrong baseline, and the page misses the causal link between the new 1.0 s pacing and the sigma tightening — two Group D rows that changed together.
- **Affected:** Anyone tuning goal accuracy, interpreting final_goal_error_m, or writing up the planner's weighting for the thesis.
- **Resolution applied:** Update the row to σ_xyz (0.001, 0.01, 0.001) with a '(working tree; HEAD: 0.01/0.1/0.01)' marker matching the velocity rows' convention, keep the 10×-looser-y note, and add the 2026-08-13 tightening with its measured justification (82 mm → 3.2 mm at 1.0 s pacing). Reword §9: the ratio is inherited; the magnitude was chosen.
- **Verifier's correction:** Two adjustments to the reviewer's claim. First, not every absolute number in the row is off: σ_rpy 0.01 is still current (rotation_sigma_rpy is unchanged at [0.01, 0.01, 0.01]); only the three position sigmas are 10× stale. Second, downgraded from high to medium: the error's direction is conservative (the real planner anchors the goal tighter than documented), it cannot cause an unsafe action, and planner.yaml's own comment at the value corrects a student on first contact.

### planner-3 [medium] — fixed 2026-08-14
- **Where:** Command path §4 warn note (file lines 1238-1240): "Commit the controller table, the planner table and the gate fix together — they are one decision" (naming only kModelVelocityLimitsDegS, joint_limits.yaml, and the PathValidationReport.h gate fix), plus the masthead provenance line naming only "velocity limits, planner gate fix"; Architecture Map §5 bullet 1 (file lines 544-548) with the same three-item enumeration.
- **Issue:** Both pages under-describe the uncommitted planner delta. `git diff HEAD` on planner_bridge also contains: pacing nominal_speed_mps 0.05 → 0.25 and min_duration_s 4.0 → 1.0, the goal position-sigma ×10 tightening, approach pacing approach_velocity_fraction 0.3 → 0.9 / approach_min_duration_s 2.0 → 0.1, and the config rail raise (nominal_speed cap 0.25 → 2.0 m/s in PlannerConfig.cpp). Group D's pacing row presents 0.25/1.0 as current with only a 'was 0.05 / 4.0' aside — no '(working tree; HEAD: …)' marker, unlike the velocity-limit rows which carry exactly that marker, so a reader cannot tell these values are part of the same uncommitted decision.
- **Why it matters:** Followed literally, 'commit them together' either leaves the pacing/sigma/approach changes uncommitted or sweeps a 4-16× motion-speed change and a 10× goal-weighting change into a commit the student believes covers only velocity limits. Anyone checking out clean HEAD will also find the page's pacing and sigma numbers wrong without warning.
- **Affected:** Christian when committing the 2026-08-13 decision; anyone checking out clean HEAD or auditing which values a given commit carried.
- **Resolution applied:** Enumerate the full uncommitted planner delta in the §4 warn note and the masthead provenance (pacing, goal sigmas, approach pacing, config rail, gate fix, both yaml tables), and add the '(working tree; HEAD: 0.05 / 4.0)' marker to the Group D pacing row for consistency with the velocity rows. Mirror one sentence into the Architecture Map bullet.
- **Verifier's correction:** One claim weakened: reconstructing an older run's pacing does not depend on these pages alone — the session evidence bundle copies the exact planner.yaml used into each run's directory (both pages document this), and the bridge echoes effective values per run. The confirmed defect is the incomplete 'commit them together' enumeration and the inconsistent working-tree markers, not an absence of any per-run record.

### planner-4 [medium] — fixed 2026-08-14
- **Where:** Section 4, Group D 'Joint velocity limit factor' row (file line 1343): "gpmp2 hinge, threshold 0.05 rad/s" with no route qualifier; §0 vocabulary 'support states' row (file line 488): "the handful of discrete configurations the optimiser actually solves for (11 here)"; Group D 'Support states' row (file line 1341): "n = waypoints + 1 … 10 → 11 states, Δ = T/10"; §1 diagram "11 support states" (file line 572).
- **Issue:** Point-goal-route constants are stated as if they applied to the whole planner. The velocity-hinge threshold is 0.05 rad/s only in optimizeJointTrajectory (TrajectoryOptimization.cpp ~89); the traced-path graph in optimizeTaskTrajectory uses 0.1 rad/s (~285). Likewise '11 support states' and 'n = waypoints + 1' hold only for point goals; a traced path's support-state count is waypoints.size() from chord-error circle sampling plus approach waypoints (`total_time_step = waypoints.size() - 1`, ~213), typically far more than 11.
- **Why it matters:** Group D otherwise tags rows '(traced path only)' carefully, so an unqualified row reads as route-independent. A student computing when the velocity soft cost engages on a circle trace, or estimating a traced solve's size, gets wrong numbers while believing the table's per-route tagging protected them.
- **Affected:** Students analysing factor costs (start_costs/final_costs) or tuning traced-path plans.
- **Resolution applied:** Tag the velocity-hinge row 'point route 0.05 / traced route 0.1 rad/s' with both file cites, and qualify the support-state claims: '(point goals; a traced path has one state per sampled path point plus 5 approach waypoints)'. Fix the vocabulary entry's '(11 here)' and the §1 diagram label the same way.

### planner-5 [low] — fixed 2026-08-14
- **Where:** Section 4, Group D (file lines 1349-1353): 'Fidelity gate', 'Modelled clearance' and 'Start-state match' rows say on breach "reported invalid; not cleared for hardware", while only the 'Joint-position sweep (both routes)' row says "exit 4, nothing written to the pipe".
- **Issue:** The asymmetric phrasing implies different consequences, but in source every failed verdict on the traced route also refuses emission: `if (!plan.report.hardware_execution_allowed) { … "Nothing was emitted." return 4; }` (BridgeMain.cpp ~712-719, before `targets << plan.emitted_block`). Two smaller inaccuracies in the sweep row: its '(both routes)' tag is wrong — ValidateJointPath's only caller is the point route (BridgeMain.cpp:810); the traced route enforces joint limits via the dense-reconstruction margin inside ValidatePlannedPath (`report.joint_limits_valid = report.minimum_joint_limit_margin_rad > 0.0`, ValidatePath.cpp ~269). And on the point route ValidateJointPath sweeps the full densified trajectory (~1 kHz points, capped at 1000 rows on emit), not just the 11 support states — despite its 'support state %zu' error message.
- **Why it matters:** A student may conclude a fidelity- or clearance-failing traced plan is still emitted with a warning and must be manually screened, or hunt in ValidateJointPath for why a circle was rejected on joint limits when the responsible code is ValidatePath.cpp. Both misdirect debugging; neither causes unsafe behaviour — the real behaviour is stricter than documented.
- **Affected:** Students debugging a rejected traced-path plan or auditing what a 'passed' plan implies.
- **Resolution applied:** Unify the traced-path rows' breach column to 'reported invalid → exit 4, nothing emitted (BridgeMain.cpp)'. In the sweep row, drop '(both routes)', note the traced route enforces joint limits via the report's dense-reconstruction margin, and note the point route sweeps every densified point despite the 'support state' wording (which is itself a mislabel in the source message).

### planner-6 [low] — fixed 2026-08-14
- **Where:** Section 6 debugging table, 'Wire transport' row, Typical failures column (file line 1594): "6-significant-digit stream formatting; a decimated block whose Hermite path differs from the GP path".
- **Issue:** The wire emitter does not use default 6-significant-digit stream formatting: FormatTrajectoryBlock sets `block << std::fixed << std::setprecision(6)` (TrajectoryEmit.cpp, verified at ~line 35) — six decimal places, micro-degree resolution independent of magnitude, effectively lossless against the 5 mm fidelity gate. The 6-significant-digit failure mode is real but belongs to the run-CSV writer, which the page itself covers correctly in §8.5's 'Timestamp columns quantise to uselessness' row.
- **Why it matters:** A student chasing reconstruction error is pointed at a precision mechanism that does not exist on this path and may 'fix' emit formatting instead of looking at decimation, the actual lossy step the same row also names.
- **Affected:** Anyone debugging a fidelity-gate failure attributed to transport.
- **Resolution applied:** Replace with: 'decimation to at most 1000 points is the lossy step; angle text is fixed 6-decimal (micro-degree), so formatting is effectively lossless here — the 6-significant-digit trap lives in the run CSV timestamps (§8.5), not the wire'.

### planner-7 [low] — fixed 2026-08-14
- **Where:** Section 11 goal.yaml example (file line 2020: "centre: [ 0.31, 0.386, 0.5213 ]") together with the src note (file line 2028) mentioning the optional obstacle box — whose keys are never shown.
- **Issue:** The config grammar mixes spellings: the circle path key is British `centre` (BridgeMain.cpp:349, `ReadVector3(path_node["centre"], "path.centre")`) while the box keys are American `center` and `half_extent` (BridgeMain.cpp ~400-404, `arm_node["box"]["center"]`). The example teaches only the `centre` spelling and omits the box keys entirely, so a student adding a box by analogy will write `centre:` and be refused.
- **Why it matters:** The failure is loud (ReadVector3 throws naming 'box center'), so this is friction rather than danger — but the box block is exactly the part section 11 leaves undocumented, on a page whose stated job is that a student can work from these pages alone.
- **Affected:** Any student adding an obstacle box from the page without opening the repo's goal.yaml.
- **Resolution applied:** Show the box sub-block in the example with its real keys and flag the spelling split explicitly (and, per the box-frame finding above, its actual frame semantics).


## Lens: vicon — 11 confirmed, 0 refuted

### vicon-0 [high] — fixed 2026-08-14
- **Where:** Command Path sec 00 Fact 1; sec 3.6 intro ("every idle-hold cycle enters it"); sec 7.3 heading; sec 8.1 first bullet; Architecture Map sec 2 prose + Fig 2 caption; Cartesian-law LIVE note and card
- **Issue:** Both pages describe a two-way dispatch (active trajectory -> joint law; idle hold -> Cartesian law with world anchor or re-seated base-frame fallback), omitting the third branch: while the world hold has never engaged (no Vicon, stub build, or Vicon not yet fresh), an idle-hold cycle runs SolveJointTracking on the idle joint reference — the pre-Vicon joint law, not the Cartesian law. The Cartesian base-frame fallback only exists after a first engage.
- **Why it matters:** A student on a no-Vicon bench run — the most common configuration — will expect Cartesian telemetry (taskvel_j*, nullvel_j*, pd_*/p_*, sigma_min) on idle-hold rows and find NaN. Worse, the Architecture Map claims the graded null-space push-back near the j2/4/6 limits "is finally on a live path during holds" — it is not live during any hold before the first engage, so a safety property is asserted for a regime where it does not exist. File A contradicts itself: its own sec 3.6b source note correctly says "with no Vicon at all, fresh is never true and the pre-Vicon joint hold runs unchanged".
- **Affected:** Any student reasoning about no-Vicon or pre-engage runs; anyone relying on null-space limit avoidance being active during idle holds
- **Resolution applied:** State the three-way dispatch everywhere the headline claim appears: active trajectory -> joint law; idle hold after the world hold has ever engaged -> Cartesian law (world anchor when fresh, re-seated base-frame hold when frozen/latched); idle hold before any engage -> the pre-Vicon joint hold, bit-for-bit the old behaviour. Fix the Architecture Map's null-space sentence to "on a live path during engaged/post-engage holds".
- **Verifier's correction:** The reviewer's claim is accurate as stated. One precision: every idle-hold cycle does enter the dispatch function's pose section, but pre-engage it exits through SolveJointTracking without computing any Cartesian quantity — so "never enters the Cartesian law" is the right description of a no-Vicon run.

### vicon-1 [high] — fixed 2026-08-14
- **Where:** Sec 3.2b answer 1: "logged as p_x, p_y, p_z (m) and quat_x..w, every cycle"; sec 6 plots bullet "‖p_desired − p_current‖ against time. Both are logged"; sec 11 CSV table row "pd_x/y/z, p_x/y/z | m | desired / FK tool position" (only the cycle==0 caveat given)
- **Issue:** p_x/y/z and pd_x/y/z are NaN on every joint-law cycle, which includes every active trajectory and every pre-engage idle hold. The columns are written every row, but the FK values exist only on cycles that ran the Cartesian law; the page's sole stated caveat is the cycle==0 struct-zeros case.
- **Why it matters:** Answer 1 is the page's flagship "where is the tool?" answer, and it promises a per-cycle FK record that is absent for exactly the rows a student most wants — the planned motion. Anyone verifying a trajectory's tool path or comparing FK against the Vicon EE segment during a move finds NaN and concludes the log is broken. plot_world_hold.py itself documents the trap: "The joint-tracking hold ... skips FK entirely and leaves p_x/y/z blank -> NaN".
- **Affected:** Anyone doing offline analysis of tool position from the run CSV, especially during trajectory execution
- **Resolution applied:** Correct answer 1 and the CSV table: p_*/pd_* carry FK only on cycles that ran the Cartesian law; they are NaN on joint-law cycles (active trajectories, pre-engage idle holds) and uncomputed zeros on cycle==0 rows. Add the workaround: run FK offline on meas_j* to get the tool path of a planned move.
- **Verifier's correction:** Sharpened: the columns ARE written every cycle — the false claim is that they carry FK every cycle. The finding's substance stands unchanged.

### vicon-2 [high] — fixed 2026-08-14
- **Where:** Sec 3.2b answer 3 source note: "Comparing answer 3 against answer 2 for the SAME instant exposes the total chain error — that comparison is precisely what scripts/plot_world_hold.py's EE-in-world figure sets up, and it is the experiment that turns 'the hold works' from a claim into evidence"
- **Issue:** plot_world_hold.py's EE-in-world figure never reads the vicon_leftee_*/vicon_rightee_* columns (answer 2). It recomputes answer 3 offline (logged Mount pose composed with the mounting YAML and the controller's logged p_* FK) and plots its flatness against Mount motion. The answer-3-vs-answer-2 chain-error comparison exists in no script in the repository.
- **Why it matters:** The page presents this comparison as THE experiment that validates the hold and points at a script as if it already performs it. A student running the script gets a flatness figure that inherits every chain error (URDF error, mountseg-vs-mount offset) and cannot expose them. Presenting self-consistent, controller-derived evidence as the independent check is exactly the validation anti-pattern the project's own rules forbid — for MSc evidence it is the difference between a self-consistency check and an independent measurement.
- **Affected:** The student preparing hold-validation evidence for the thesis; anyone assessing whether "the hold works"
- **Resolution applied:** Either fix the text — "that comparison is the experiment to build; plot_world_hold.py does NOT yet draw it: its EE-in-world figure recomputes answer 3 offline and shows flatness only" — or add the vicon_rightee/leftee overlay to fig_ee_world and keep the claim.

### vicon-3 [medium] — fixed 2026-08-14
- **Where:** Command Path sec 6 plots bullet: "an independent EE-in-world figure that recomputes world_T_base from the logged Mount pose and the mounting YAML — evidence the controller's own numbers did not produce"; Architecture Map sec 5: "whose EE-in-world figure is deliberately an independent reconstruction"
- **Issue:** The independence is overstated. The reconstruction is independent of the hold logic (it does not trust world_err_m or hold_state), but the EE world position is built from the controller's own logged FK p_x/y/z — controller-produced numbers — composed with equivalent mounting geometry. Only the world_T_base composition is redone.
- **Why it matters:** "Evidence the controller's own numbers did not produce" invites presenting the figure as an independent oracle in the thesis; an examiner would correctly object that p_* is the controller's model-derived FK. The genuinely independent measurement (the tracked EE segment) is logged but unused.
- **Affected:** Thesis evidence quality; anyone auditing the anti-gaming claim
- **Resolution applied:** Reword to "independent of the hold logic's own error computation, but still built on the controller's logged FK — the fully independent check is the tracked EE segment (vicon_rightee_*/leftee_*), not yet drawn".

### vicon-4 [medium] — fixed 2026-08-14
- **Where:** Sec 3 safety table, "Vicon freshness gate" row: "Trips on: sample age > 50 ms, invalid segment, or repeated frame"
- **Issue:** "Repeated frame" is not a trip condition of the freshness gate. Freshness is `mount.valid && sequence > 0 && isfinite(age) && age <= 0.05 s` — no repeated-frame term. A repeated sample is the normal zero-order-hold state: at 100 Hz Vicon vs the 500 Hz loop, roughly 4 of 5 cycles reuse the previous sample with the hold engaged. Duplicate Vicon frames are suppressed upstream in the acquisition thread precisely so they never inflate the sequence — a stalled server surfaces only as growing age.
- **Why it matters:** As written, a student would conclude the hold freezes on ~80% of cycles, or would misdiagnose a frozen hold as "a frame repeated" when the actual cause is age or occlusion. In the safety table — the page students trust for what trips what — this is a wrong mechanism.
- **Affected:** Anyone reading the safety table to predict or diagnose hold freezes
- **Resolution applied:** Change the trip column to "sample age > 50 ms or invalid Mount segment", and if duplicate frames are worth mentioning, place it correctly: "the acquisition thread never re-publishes a repeated Vicon frame, so a stalled server shows up as age growth, never as fake freshness".

### vicon-5 [low] — fixed 2026-08-14
- **Where:** Sec 3.2b answer 3 equation annotation: "world_T_base = world_T_mountseg ∘ mount_T_base ... ↑ fixed geometry (dual_arm_mounting.yaml)"
- **Issue:** The runtime chain does not read dual_arm_mounting.yaml. The controller's mount_T_base comes from the URDF through Pinocchio (Kinematics.cpp caches mount_from_right_base_/mount_from_left_base_ from data_.oMf), exactly as the page's own sec 3.2 states ("all constant, all read from the URDF via Pinocchio"). The YAML is the human-readable source-of-truth that a ctest pins against the URDF, and it is what plot_world_hold.py reads offline — which is what makes the plot's composition semi-independent.
- **Why it matters:** The annotation contradicts the page's own sec 3.2 and misattributes the plot script's one independent input. The practical edit-the-YAML failure is largely fenced: the YAML header instructs "edit this file first, then the URDF, then run the test", and test_dual_arm_mounting fails naming the offending number on any disagreement.
- **Affected:** Anyone modifying mounting geometry or reasoning about what plot_world_hold.py trusts
- **Resolution applied:** Annotate the chain as "fixed geometry from the URDF via Pinocchio (dual_arm_mounting.yaml documents it, a ctest pins the two together, and the plot script reads the YAML — which is why its offline composition is not the controller's arithmetic)".
- **Verifier's correction:** Downgraded from medium to low: the ctest pin and the YAML's own edit-workflow header prevent the silent-divergence failure the reviewer projected; what remains is a provenance misattribution that contradicts the page's own sec 3.2.

### vicon-6 [medium] — fixed 2026-08-14
- **Where:** Sec 1, Fig 1: Vicon box (x20,y216,w300) with arrow path M90,258 V378 H460 V412 terminating atop the tracking-law box (x372-548, y416); RT lane box list; sec 1 deck: "Four execution contexts"
- **Issue:** Fig 1's RT-thread lane contains only the joint-law chain (splice guard, Hermite, tracking law, clip, integrate, lead bound, software limit hold, Send) — no Cartesian-law or world-hold box — so the Vicon arrow visually feeds the joint tracking law, which never reads it; the caption text "→ world hold (idle only)" points at a box that is not the world hold. The Vicon box is drawn in the planner-lane band (y 216-258 lies between the y=120 and y=294 lane rules) though the acquisition thread lives in the controller process. The deck's "four execution contexts" omits the Vicon acquisition thread and the log-writer thread that the page's own Fact 2 and the diagram itself make central.
- **Why it matters:** Fig 1 is the page's master map and the first thing the reading order sends a new reader to. The single biggest architectural change this version documents — a second live control law fed by Vicon — is absent from the diagram, and the arrow added for it points at the wrong law. A student who internalises the picture gets the pre-02348ecc architecture with a decorative Vicon input.
- **Affected:** Every first-time reader; the sec 00 reading order sends them to sec 1 first
- **Resolution applied:** Add a Cartesian-law/world-hold box to the RT lane with the joint_is_idle_hold fork (the Architecture Map's Fig 2 already draws this correctly — mirror it), route the Vicon arrow into it, move the Vicon box into a controller-process band, and update the deck to name the additional threads (or scope it: "four contexts on the command path, plus the Vicon acquisition and log-writer threads").

### vicon-7 [low] — fixed 2026-08-14
- **Where:** Sec 3.6b intro: "All five knobs are compiled and echoed into every CSV preamble"; sec 6 parsing-traps note: "now including the five world_hold_* knobs"
- **Issue:** There are six world_hold_* knobs, not five: kWorldHoldAutoEngage, kWorldHoldFreshMaxAgeS, kWorldHoldRampS, kWorldHoldMaxErrorM, kWorldHoldMaxRotErrorRad, kWorldHoldReanchorAfterS — and Main.cpp echoes six matching preamble lines.
- **Why it matters:** A parser or checklist written against "five" misses one line, and the knob the count omits is kWorldHoldAutoEngage — the master switch that turns the whole behaviour off, which a student should know exists (sec 3.6b's equations name the other five values but never the auto-engage flag).
- **Affected:** Anyone parsing the preamble or enumerating the hold's configuration
- **Resolution applied:** Say "all six knobs" and list them once, flagging kWorldHoldAutoEngage as the master switch.

### vicon-8 [low] — fixed 2026-08-14
- **Where:** Sec 00 vocabulary, "frames W / T / B / E" row: "W = Vicon world, T = torso, B = arm base, E = end-effector"
- **Issue:** T is defined without the caveat that no Torso segment is tracked or logged: the five logged segments are Mount, LeftBase, RightBase, LeftEE, RightEE, and the moving tracked body in every hardware equation is the Mount segment (Frames.h maps sim-torso to the hardware Mount segment). The Torso segment is an open lab checklist item.
- **Why it matters:** A student cross-referencing the vocabulary against the CSV will hunt for a torso column that does not exist, and may conflate T with the Mount segment — exactly the confusion the 2026-08-13 "torso is a separate 6th segment, NOT the mount" decision exists to prevent.
- **Affected:** New readers mapping notation to log columns
- **Resolution applied:** Add to the row: "T is notation for the planned torso frame — no Torso segment is tracked or logged today; on hardware the moving tracked body in every current equation is the Mount segment".
- **Verifier's correction:** The page never falsely claims a Torso segment is logged — the defect is a missing caveat in a vocabulary table that otherwise carefully flags hardware caveats, not a false statement. Low severity is right.

### vicon-9 [low] — fixed 2026-08-14
- **Where:** Sec 3.6 equations 5-6 source note: "LimitAvoidanceVelocity, ReactiveLaw.h:140–169; ramp applied Controller.cpp:177–179"
- **Issue:** The null-space ramp is applied at Controller.cpp:298-300 (`ramped_gains.limit_avoid_gain_s_inv *= UnitRamp(state.t_s, config::kNullRampDurationS)`). Line 177 is `resolved_reference_twist_ = resolved.twist_world;` inside the reference.pose branch the page itself calls dead. The citation predates the world-hold rewrite of Controller.cpp.
- **Why it matters:** The page's stated contract is "every claim cites file:line". Following this cite lands in the dead pose-channel code, where a student may conclude the ramp lives on an unreachable path; one confirmed stale cite also lowers trust in the rest.
- **Affected:** Anyone jumping from the page into Controller.cpp
- **Resolution applied:** Update to Controller.cpp:298-300 and re-sweep the page's Controller.cpp/WorldHold.h line references against the post-02348ecc files (the WorldHold.h:150-177 latch cite is also a few lines adrift — the latch logic spans ~146-174).

### vicon-10 [low] — fixed 2026-08-14
- **Where:** Sec 1 "Every output" list: "...the world-hold evidence quartet hold_state / world_err_m / world_err_rot_rad / hold_ramp"
- **Issue:** The world-hold evidence is five columns, not a quartet: hold_reanchor_count is appended after hold_ramp and is the only place blackout re-anchors are counted.
- **Why it matters:** Re-anchoring silently moves the absolute anchor point in the room; hold_reanchor_count is the column that reveals it (the command-path page's sec 6 and plot_world_hold.py both rely on it). Omitting it from the map's inventory makes the one "the anchor moved" indicator easy to miss — though the map's own sec 3 safety table does say re-anchors are "counted", softening the harm.
- **Affected:** Anyone using the map's output inventory as a column checklist
- **Resolution applied:** "...the world-hold evidence columns hold_state / world_err_m / world_err_rot_rad / hold_ramp / hold_reanchor_count".


## Lens: pedagogy — 18 confirmed, 2 refuted

### pedagogy-0 [high] — fixed 2026-08-14
- **Where:** Command Path §00 'Fact 1' and §05 'what happens after step 10' note; Architecture Map §2 intro, §4 card 'Cartesian law + world hold', Fig. 2 caption
- **Issue:** Both pages state a two-way dispatch (active trajectory -> joint law; idle hold -> Cartesian law), but Controller.cpp implements a tri-state dispatch: on an idle-hold cycle before the world hold has EVER engaged (world_hold_ever_engaged_ false, e.g. no Vicon source at all), the joint tracking law runs on the idle joint reference. File A contradicts itself: §00 says an idle hold always 'falls through to the Cartesian resolved-velocity law', while §3.6b's source note correctly says 'with no Vicon at all, fresh is never true and the pre-Vicon joint hold runs unchanged'. File B says the Cartesian law is 'now running on every idle-hold cycle'.
- **Why it matters:** This dispatch is the headline fact both pages open with. A student on a Vicon-less bench run (the common hardware-free-adjacent case) would look for Cartesian/hold telemetry that is driving nothing, or conclude the joint idle hold is dead code. The only correct statement is buried in a source note that contradicts the page's own headline.
- **Affected:** Any student reasoning about idle-hold behaviour, especially runs without Vicon
- **Resolution applied:** State the dispatch as three explicit cases in §00 Fact 1, the §05 completion note, and the Architecture Map card and Fig. 2 caption: (1) active trajectory -> joint law; (2) idle hold after the world hold has engaged at least once this run -> Cartesian law (world anchor when fresh, re-seated base-frame hold otherwise); (3) idle hold before any first engage (no fresh Vicon ever, or auto-engage disabled) -> the pre-Vicon joint hold, unchanged. Cite the Controller.cpp comment directly.
- **Verifier's correction:** The reviewer's claim is fully confirmed; one refinement: the third case also applies when config::kWorldHoldAutoEngage is compiled off, not only when Vicon is absent (hold_in.sample_fresh = kWorldHoldAutoEngage && state.world_fresh).

### pedagogy-1 [high] — fixed 2026-08-14
- **Where:** Architecture Map Fig. 1 robot box ('right .9 / left .10'); Command Path §11 robot_fault row ('the Kinova web dashboard at the arm's IP')
- **Issue:** Worse than reported: the Map's shorthand is not only incomplete, it is INVERTED. Config.h (both at HEAD 02348ecc and in the working tree) defines kRightRobotIp = 192.168.1.10 and kLeftRobotIp = 192.168.1.9, with matching lock paths /tmp/basic_control-192.168.1.10.lock (right) and …1.9.lock (left). Fig. 1's 'right .9 / left .10' assigns each arm the other arm's address. Additionally, as the reviewer said, no full arm IP appears on either page, so §11's 'dashboard at the arm's IP' instruction is unexecutable.
- **Why it matters:** A student following Fig. 1 during fault recovery would open the WRONG arm's web dashboard or misread which arm a ProcessLock refusal names — on a two-arm rig this is an operational hazard, not just a documentation gap. Ironically the Map's own ORPHANED card criticises test_kinova for 'labelling the .9 arm wrongly' while committing the same swap.
- **Affected:** Anyone doing fault recovery, dashboard access, or interpreting per-arm identity in ProcessLock messages and logs
- **Resolution applied:** Fix Fig. 1 to 'right .10 / left .9' and state the two full addresses once on each page (Map input table or Fig. 1; Command Path §11 'Where things live'): 'right arm 192.168.1.10, left arm 192.168.1.9 (Config.h:33-34) — the .10/.9 shorthand elsewhere means these'.
- **Verifier's correction:** Upgraded from medium to high and reframed: the reviewer reported only the missing full addresses; verification against Config.h shows the shorthand itself swaps the two arms, which is a factual error with direct operational consequences.

### pedagogy-2 [high] — fixed 2026-08-14
- **Where:** §06 Debugging by stage, 'Sampling' row, Telemetry column
- **Issue:** The reference-reconstruction instruction uses the wrong time base: it says to evaluate §3.5's Hermite equations 'at each row's time_s', but time_s is seconds since loop start while the trajectory clock is reset to zero at activation (Targets.cpp: elapsed_s_ = 0.0 on the traj_activated edge) and §05 step 5 states this. The needed offset is never given.
- **Why it matters:** A student following the cell literally gets a reference shifted by the activation time, computes a large phantom tracking error, and either distrusts the log or diagnoses a nonexistent bug. The procedure completes silently with plausible-looking curves.
- **Affected:** Anyone reconstructing q_ref(t) offline to analyse tracking — the exact task the row exists for
- **Resolution applied:** Amend the cell: reconstruct at the trajectory clock, not time_s. The exact trajectory time is the cumulative sum of dt_s from the row where traj_activated = 1 (Targets.cpp advances elapsed_s_ by the clamped dt each cycle); time_s minus time_s(activation row) is the first-order approximation, exact only when no cycle's dt was clamped.
- **Verifier's correction:** The finding is confirmed and slightly strengthened: because the trajectory clock advances by the CLAMPED dt (4 ms ceiling), even the reviewer's subtract-the-activation-edge fix drifts after overrun cycles (166 clamped cycles in the page's own sample run); the exact reconstruction is the running sum of dt_s from the activation row.

### pedagogy-3 [high] — fixed 2026-08-14
- **Where:** §11 warn note 'the hardware rule, first' vs the exit_reason table row 'robot_fault'
- **Issue:** The warn note's blanket claim 'Everything else on this page — building, all 35 ctest suites, the panel, every plot script, replay — touches no hardware' is contradicted three paragraphs later, where the 'safe first steps' column tells the student to 'clear with tools/clear_faults'. clear_faults is a Kortex-linked binary (CMakeLists.txt:163-172, Base::ClearFaults) that writes to the physical arm over TCP and falls under the project's explicit-authorization hardware rule.
- **Why it matters:** A student who internalised the blanket safety statement can reasonably run clear_faults unauthorised — the column heading literally labels it a safe first step. Clearing a fault also destroys the diagnostic state before the cause is understood, which the row's own 'understand the cause before re-running' cannot then help with.
- **Affected:** Any student recovering from a robot_fault exit; lab safety protocol
- **Resolution applied:** Tag clear_faults in the robot_fault row as hardware-touching (TCP config write, same explicit authorization as any Kortex-linked run; read the decoded fault name and CSV evidence first), and amend the warn note: '…touches no hardware; the recovery tools named later (clear_faults, set_joint_limits) DO touch the arm and fall under the hardware rule.'
- **Verifier's correction:** The row's own parenthetical '(needs the arm to run)' does disclose arm interaction, so the defect is the contradictory blanket claim plus the 'safe first steps' framing and missing authorization language, not total silence about hardware contact.

### pedagogy-4 [high] — fixed 2026-08-14
- **Where:** §11 'The two input formats', goal.yaml block (orientation_rpy_deg); §4 Group D goal pose prior (sigma_rpy); §00 vocabulary table
- **Issue:** The page insists an orientation be stated ('state it!') but never defines the RPY convention — rotation order, intrinsic vs extrinsic, or frame. The actual convention exists in source: CartesianPath.h:76-77 states 'Roll/pitch/yaw, R = Rz*Ry*Rx — the convention BridgeMain's RotationFromRpy and FramePrint.h use'. [90, 0, 90] produces different rotations under other conventions, and 'rpy' has no vocabulary entry.
- **Why it matters:** A student can follow the instruction exactly and command a wrong orientation. On the point-goal route this reaches hardware unvalidated: §4 documents that point goals receive only the joint-position sweep, so a misinterpreted orientation becomes a legal but unintended pose.
- **Affected:** Anyone writing a goal.yaml — the primary student-facing input
- **Resolution applied:** Where the field is introduced, state: 'orientation_rpy_deg = [roll, pitch, yaw] applied as R = Rz(yaw)·Ry(pitch)·Rx(roll), in the goal's declared frame's axes (CartesianPath.h:76-77, BridgeMain's RotationFromRpy)', and add an RPY row to the §00 vocabulary table.
- **Verifier's correction:** Verification supplies the concrete convention and citation the reviewer's recommendation left as a placeholder: R = Rz(yaw)·Ry(pitch)·Rx(roll) per CartesianPath.h:76-77.

### pedagogy-5 [medium] — fixed 2026-08-14
- **Where:** §04 warn note 'the timestamp columns lose their resolution within seconds' — the resolution table
- **Issue:** Every row of the resolution table is one decade too coarse and contradicts the note's own worked sample. Six significant digits give 0.01 ms steps for t in [1,10) s, 0.1 ms in [10,100), 1 ms in [100,1000), 10 ms in [1000,10000). The note's own sample '2800.12' is a 10 ms step, yet the table's bucket for 2800 s claims 100 ms; and the same note's 'p99 of 10 ms … one quantum' is only consistent with the correct 10 ms step. '0.34 ms round trip is 3 quanta' at the claimed 0.1 ms is 34 quanta at the true 0.01 ms.
- **Why it matters:** The qualitative conclusion survives, but a student who checks the arithmetic — the audience this page cultivates — finds the table inconsistent with its own worked example, and anyone sizing the usable-latency window of a run is off by 10x.
- **Affected:** Anyone doing latency analysis on run CSVs
- **Resolution applied:** Rewrite the buckets as half-open ranges matching 6-significant-digit formatting — [1,10) s -> 0.01 ms, [10,100) -> 0.1 ms, [100,1000) -> 1 ms, [1000,10000) -> 10 ms — and recompute the quanta statements ('0.34 ms is ~34 quanta early in a run; at 2800 s the step is 10 ms, so the naive p99 of 10 ms is one quantum').
- **Verifier's correction:** Note the surrounding prose (the 2800.12 sample and the one/two-quanta statement) is already consistent with the CORRECT values — only the boxed table is wrong, which localises the fix.

### pedagogy-6 [medium] — fixed 2026-08-14
- **Where:** Command Path §05 step 3 and §06 Sampling row; Architecture Map Fig. 1 box 'plan file (saved)'
- **Issue:** Two workflows depend on the saved plan file — inspecting a failed plan and reconstructing the reference — but neither page states where it is written or what it is named. Verified in run_session.sh: SESSION_DIR="$REPO/runs/$(date +%F)/session_$(date +%H%M%S)" and plan_file="$SESSION_DIR/plan_$a.traj", with bridge stderr in bridge_$a.log alongside.
- **Why it matters:** The instructions are unexecutable as written: a student told to inspect the 'file it left behind' or evaluate Hermite equations 'on the saved plan file' cannot find it without reading run_session.sh, defeating the pages' debug-from-these-pages-alone goal.
- **Affected:** Anyone debugging a rejected plan or reconstructing q_ref offline
- **Resolution applied:** State the pattern once in §05 step 3 and §11 'Where things live': 'run_session.sh saves the bridge's stdout to runs/YYYY-MM-DD/session_HHMMSS/plan_<arm>.traj (stderr to bridge_<arm>.log beside it) before copying it into the FIFO — that file is the artefact to inspect and the input to any Hermite reconstruction.'

### pedagogy-7 [low] — fixed 2026-08-14
- **Where:** Architecture Map orchestrator bar ('refuses DH YAML older than URDF') and Fig. 2 labels; Command Path §3.2 source note and §3.7 stage D
- **Issue:** The Map's six-term box promises to pre-define foreign vocabulary but the pages use undefined terms anyway: DH (Denavit-Hartenberg) is never expanded in either file despite gating the freshness-gate and dh_params build contract; Pinocchio appears in File A's frame-transform source note ('all read from the URDF via Pinocchio') with no introduction beyond a vendored-deps listing in the Map; protobuf appears once in §3.7 stage D undefined.
- **Why it matters:** The Map explicitly commits to defining its foreign vocabulary before use, and DH in particular is needed to understand why the orchestrator refuses a run ('DH YAML older than URDF').
- **Affected:** First-pass readers of the Map; anyone touching the DH/URDF build contract
- **Resolution applied:** Add one-line definitions: DH ('Denavit-Hartenberg parameters — the compact joint-geometry table generated from the URDF at build time; the freshness gate refuses a stale one') to the Map's term box, and Pinocchio ('the rigid-body kinematics library used for FK and Jacobians') plus protobuf to File A's vocabulary.
- **Verifier's correction:** Downgraded from medium to low: DLS and splice ARE defined in File A's vocabulary, which the recommended reading order (Map first, then Command Path) reaches within minutes, so the residual gap is the three genuinely never-defined terms, none of which blocks a debugging task.

### pedagogy-8 [low] — fixed 2026-08-14
- **Where:** §04 Group C row 1 ('Following error — Cartesian rule') and §7.2
- **Issue:** The 3 degree guard is displayed as the 'Cartesian rule' although its formula is a per-joint angle comparison, and neither page explains the name; File B uses a different phrasing ('3° command-vs-measured gap (any joint)'), so the two pages diverge in terminology.
- **Why it matters:** A student maps 'Cartesian' to task space and may briefly believe a tool-pose error check exists. The confusion is bounded because the per-joint formula is printed directly beside the name in both places it appears.
- **Affected:** Anyone reasoning about which quantities the stop rules watch
- **Resolution applied:** Keep the name (it is grep-able against the source) but gloss it on first use: 'the 3° command-vs-measured joint rule — Config.h calls it the Cartesian following-error rule because it is the rule the Cartesian mode relies on; it watches joint angles, not tool pose', and use the same gloss in File B's safety table.
- **Verifier's correction:** Downgraded from medium to low: the artifact transcribes Config.h's own terminology rather than inventing it, and the per-joint formula sits immediately beside the name, so the failure mode is a transient confusion, not a wrong model of what is supervised. The residual defect is the missing one-line why and the cross-page inconsistency.

### pedagogy-9 [low] — fixed 2026-08-14
- **Where:** §05 step 1 vs §11 'Build and test (hardware-free)'
- **Issue:** The hands-on section's typed commands are only cmake/ctest/panel; the safe offline planner run — the one end-to-end exercise consolidating §§3-5 — is never presented there with a binary path or expected output to compare against.
- **Why it matters:** A student moving from reading to safe practice must assemble the exercise from three places (§05's invocation, §11's build block, §11's wire-format sample) and infer the binary path.
- **Affected:** Students moving from reading to safe practice
- **Resolution applied:** Add a short 'safe planner run' block to §11: the binary at Christian_control/planner_bridge/build/planner_bridge, one point-goal invocation, two or three lines of expected stdout (TRAJ_BEGIN header, verdict, exit 0), and one check to make against the start-state CSV.
- **Verifier's correction:** Downgraded from medium to low and the claim 'no expected output' softened: the invocation appears in §05, the TRAJ block's exact shape is shown in §11's wire-format sample, and the binary path is inferable from the standalone-build pattern; the genuine gap is that no single hands-on block assembles these into a runnable exercise.

### pedagogy-10 [low] — fixed 2026-08-14
- **Where:** Header legend vs Fig. 1 dashed-blue line
- **Issue:** The header promises 'Colors mean status everywhere on this page' and the legend defines two families, but Fig. 1 uses dashed blue (--bring) for a data-flow loop, defined only in the figure caption — a colour outside the stated decoding contract.
- **Why it matters:** A reader decoding Fig. 1 by the stated rule reads the blue dashed arrow as an undefined status category until reaching the caption. Minor, and the caption does rescue it.
- **Affected:** First-time readers of Fig. 1
- **Resolution applied:** Either add the dashed-blue line to the legend ('dashed blue — data fed back between runs, not a status') or soften the header to 'card and chip colors mean status; diagram line colors are explained in each caption'.
- **Verifier's correction:** Downgraded from medium to low, and the amber half of the claim is dropped: grep shows the .note.warn and .chip.dorm classes are defined in the CSS but used by NO element in the body, so amber never actually appears on the page; only the dashed-blue line sits outside the legend, and its caption definition limits the damage to one paragraph of confusion.

### pedagogy-11 [low] — fixed 2026-08-14
- **Where:** §3.6 equations 5-6 source note vs §06 plots list and §7.3
- **Issue:** The null-leak telemetry appears under two names without the page ever equating them: 'null_leak_m_s' (§3.6 source note and §7.3's worked number) and 'null_leak_mps' (§06 and §7.3's closing sentence). Verified in Hardware.h: the CSV column is null_leak_mps (line 231) and the struct member is null_leak_m_s (line 376) — both real, never connected.
- **Why it matters:** A student grepping a CSV header for null_leak_m_s finds nothing and concludes the §3.6 telemetry is not logged.
- **Affected:** Anyone locating the null-leak evidence in a run CSV
- **Resolution applied:** Use the CSV column name null_leak_mps wherever prose refers to logged telemetry, noting once: 'struct field null_leak_m_s, CSV column null_leak_mps (Hardware.h:376 / :231)'.

### pedagogy-12 [low] — fixed 2026-08-14
- **Where:** §00 Vocabulary, row 'frames W / T / B / E, world_T_base'
- **Issue:** The vocabulary teaches W/T/B/E with T = torso, but the T frame is never used in any equation or table on either page, while the frames the pages use constantly — mount and mountseg — are absent from the entry and only defined later (§3.2 and by usage).
- **Why it matters:** A student holding the W/T/B/E scheme naturally slots 'mount' into the unused T, equating the mount with the torso — which the project's own design records treat as wrong (the torso is a separate tracked segment, not the mount).
- **Affected:** Anyone building the frame model from the vocabulary table
- **Resolution applied:** Extend the entry to the frames the pages actually use: 'W = Vicon world, mount = rig root (NOT the torso), mountseg = the Vicon-tracked marker cluster on the mount (uncalibrated, mountseg_T_mount pending), B = arm base, E = end-effector; T = torso exists in the target notation but is unused on this page.'
- **Verifier's correction:** The reviewer's 'the torso frame never appears again anywhere' is slightly overstated — torso appears three more times in prose (SDF caveat, §9 future work) — but never as the frame T, so the substantive point stands.

### pedagogy-13 [low] — fixed 2026-08-14
- **Where:** Architecture Map §5 last bullet; Command Path §11 paragraph after the CSV table
- **Issue:** Both pages instruct keeping controller.log beside the CSV but neither says what writes it or where it lives. Verified: run_session.sh line 162 redirects the controller's stdout/stderr to $SESSION_DIR/controller.log (runs/YYYY-MM-DD/session_HHMMSS/); a controller started manually produces no such file at all.
- **Why it matters:** The one piece of evidence needed to reconstruct a joint-limit stop (the joint number) is in a file the student cannot locate from the pages — and would not exist on a manual run, which the pages also never say.
- **Affected:** Anyone reconstructing a joint_limit_warning stop
- **Resolution applied:** State once in §11 'Where things live': 'controller.log is run_session.sh's redirect of the controller's console (run_session.sh:162), at runs/<date>/session_<HHMMSS>/controller.log; a controller launched outside the script prints to the terminal instead and leaves no log file.'
- **Verifier's correction:** Sharpened with the verified writer and path, plus the manual-run caveat the reviewer did not note.

### pedagogy-14 [low] — fixed 2026-08-14
- **Where:** Architecture Map §5 bullet 'The planner's acceleration bound is invented'; Command Path §4 Group D dynamic-limits row
- **Issue:** The invented bound is written as 'q̈max = 2·q̇max' / '2×velocity (~2.65/2.32 rad/s²)', which does not balance dimensionally — twice a velocity is a velocity; the implicit division by 1 s is never stated. Verified: PlanSolver.cpp:238 literally computes joint_acceleration_limits_rad_s2(j) = vel_limits.upper(j) * 2.0, reading a rad/s quantity as rad/s².
- **Why it matters:** A units-checking student — exactly the discipline §3 trains — stalls on an unbalanced equation in the very row explaining the bound is suspect.
- **Affected:** Anyone auditing the planner's dynamic-limits gate
- **Resolution applied:** Write it honestly: 'q̈max = 2·q̇max / (1 s) — the code multiplies the velocity limit by a bare 2.0 and stores it in a rad/s² field (PlanSolver.cpp:238); the implicit one-second timescale is part of what makes the bound invented.'

### pedagogy-15 [low] — fixed 2026-08-14
- **Where:** §04 Group A, 'Joint speed clip' row, Config column
- **Issue:** The cell 'compile-time; panel-editable' can be skimmed as 'editable live from the panel'; in fact the panel edits Config.h source and the change is inert until a rebuild, a fact stated only in the §4 warn note and §11's source note, not in the cell.
- **Why it matters:** The limits table is where someone checks 'can I change this?', and §2 states 'There is deliberately no runtime tuning path' — the cell reads as the exception. The page's own stale-binary warning shows the misreading has real consequences.
- **Affected:** Anyone changing limits via the panel before a run
- **Resolution applied:** Change the cell to 'compile-time (panel edits the source; rebuild required)'.
- **Verifier's correction:** Confirmed at the reviewer's low severity; note the rebuild semantics ARE stated twice elsewhere in the same section, so this is a one-phrase clarity fix, not a missing fact.

### pedagogy-16 [low] — fixed 2026-08-14
- **Where:** §00 Vocabulary 'named pipe (FIFO)' and §01 diagram FIFO box
- **Issue:** Only the right arm's FIFO path is given; the left path (/tmp/humansl_bridge_targets_left, Config.h:126) appears on neither page, and neither page says which process creates the pipes (the controller does, Main.cpp:516 mkfifo on its arm's target_pipe_path).
- **Why it matters:** A student debugging left-arm ingest cannot name the file to inspect, and cannot tell whether a missing FIFO before controller start is an error or expected (it is expected — the controller creates it).
- **Affected:** Anyone inspecting the planner-to-controller handoff, especially on the left arm
- **Resolution applied:** Give both paths and ownership once: '/tmp/humansl_bridge_targets_right and _left (Config.h:120/126), created by the controller at startup (Main.cpp:516) — absent before the controller runs, present after.'

### pedagogy-17 [low] — fixed 2026-08-14
- **Where:** Command Path §00 deck; Architecture Map header paragraph and §2 LIVE note
- **Issue:** Both pages frame their opening content as a delta against prior versions the declared first-time audience has never seen ('both are the inverse of what the 2026-08-12 version of this page said'; 'Since the previous version of this map…'; 'the old audit's S1 finding … resolved the hard way'), forcing history-parsing to extract current state.
- **Why it matters:** For a zero-context student this is filtering load, and the negation framing briefly plants the wrong model before correcting it. The current-state statements do all follow, so the cost is friction, not misinformation.
- **Affected:** First-time readers — the pages' declared primary audience
- **Resolution applied:** Lead with the current-state statements plainly and move version-delta remarks into a one-line 'changed since 2026-08-12' footnote or the provenance block.

### Refuted (pedagogy)
- §01 boundary table row 'source → controller' misleads about the live idle-hold path's frame (base_link) and no boundary row covers the world-hold target
  - *Refuted because:* Misreading of the table's scope plus already-handled-elsewhere: the table documents inter-thread/process crossings and the world-hold target never crosses one (WorldHold is controller-internal, and §02's own 'World hold' row states exactly that it 'emits a pose target into the same Cartesian law'), while the pose channel's dormancy is stated prominently in §00 Fact 1 and §8.1 and the world-frame path is fully documented in §3.6/§3.6b and the boundary table's own Vicon row ('Vicon world' frame).
- §04 firmware table's Relationship column ('stops first, in software' / 'arm-side, second') contradicts the equal j4/j6 values (145.0=145.0, 118.0=118.0)
  - *Refuted because:* The equal thresholds are real (verified: Config.h:262-276 computes min(Table-39 − 2°, firmware warn), collapsing j4/j6 to the warn values), but the two rows test different signals — the software boundary blocks the commanded setpoint before transmission while the firmware band watches measured position, which the 1° lead bound keeps at or behind the command — so the software stop still fires first causally even at equal values, and simultaneous assertion would require physical overshoot, which the column's ordering claim does not preclude.


## Lens: visual — 16 confirmed, 1 refuted

### visual-0 [high] — fixed 2026-08-14
- **Where:** §01 Architecture, main SVG (viewBox 0 0 1000 690): 'integrate' rect at line 637 (x=740 w=150, spans 740-890) and 'log writer thread' rect at line 697 (x=820 w=164), both y=416 h=56
- **Issue:** The two rectangles share the y-band 416-472 and overlap in x from 820 to 890 (70x56 px). The log-writer rect is painted later in document order with the opaque box-fill (--surface-2), so it covers the right 70 px of the integrate box, its border, and the tails of both integrate sub-labels: 'q_cmd += q̇·Δt' (14 glyphs at ~5.7 px each from x=750, ends ~830) and 'persistent state' (ends ~841) both cross x=820 and are occluded.
- **Why it matters:** This is the page's primary architecture figure, and the integrate stage is the one the page repeatedly calls safety-critical (persistent state, first-cycle jump). The two boxes render as one malformed shape with amputated labels, at exactly the stage a reader most needs, undermining trust in the rest of the figure.
- **Affected:** Every reader of the main §1 figure; especially a student using it as the mental map for §3.7 and §5.
- **Resolution applied:** Move the log-writer box out of the RT command row entirely — e.g. x=820 y=372 w=164 h=40, above the RT row — and re-route its dashed meas arrow to the new bottom edge. Any layout where the two rects no longer share x∈[820,890] at y∈[416,472] fixes it.
- **Verifier's correction:** The reviewer's recommendation text was self-confused ('x=830 is impossible'); the fix itself (relocate log-writer above the RT row) is sound and kept.

### visual-1 [high] — fixed 2026-08-14
- **Where:** §01 Architecture SVG, RT-thread lane, versus §00 'Fact 1 — both control laws are now reachable' (line 421-423)
- **Issue:** The RT lane draws only the joint-space path (splice guard, Hermite sample, tracking law, clip q̇, integrate, lead bound, software limit hold, Send) — no box for the Cartesian law, the world hold, or the idle-hold dispatch. Worse, the Vicon arrow (d="M90,258 V378 H460 V412", line 693) terminates at (460,412), 4 px above the top edge of the 'tracking law' box (x 372-548, y 416) whose label is the joint law — so the figure visually routes Vicon into the one law Vicon never feeds. Only the 9.5px side note 'one slot read per cycle → world hold (idle only)' (line 694) and the figcaption say otherwise.
- **Why it matters:** The figure teaches the pre-02348ecc mental model (one law, Vicon as bystander) that §00 explicitly says is inverted, and adds a false data-flow (Vicon → joint tracking law). The most consequential architectural change in the repo is absent from the architecture drawing.
- **Affected:** Anyone using the §1 figure as the system map; anyone debugging idle-hold behaviour who looks for the world hold in the RT lane.
- **Resolution applied:** Add a meas-tinted 'world hold → Cartesian PD + DLS' box parallel to 'tracking law', both converging on 'clip q̇', with an 'idle?' dispatch fork after the reference stage — mirroring what the Architecture Map's Fig. 2 already draws correctly — and route the Vicon arrow into that box, ending on its edge rather than 4 px short.

### visual-2 [high] — fixed 2026-08-14
- **Where:** §1 Fig. 1 planner_bridge internals, line 221: <text x="294" y="255" class="svg-s">validate: limits, clearance, FK</text>; also the 'Every output' bullet (line 327) promising clearance in every validation report; no mention of the point-goal/traced-path split anywhere on the page
- **Issue:** Fig. 1 draws 'validate: limits, clearance, FK' as an unconditional stage of every plan, and the page nowhere mentions that clearance/fidelity/dynamics/start-state validation runs only on the traced-path route. Repository check confirms the asymmetry: ValidatePlannedPath has exactly one caller, PlanSolver.cpp:263 inside the path route; a point goal (--goal X Y Z) gets only ValidateJointPath at BridgeMain.cpp:~809, a joint-position sweep with no clearance check. The companion page carries a red warning about exactly this; the overview page — the one the reading order says to read first — visually asserts the opposite.
- **Why it matters:** A new reader will believe every emitted plan was clearance-checked. On the default --goal route it was not — a hardware-safety-relevant false belief: an obstacle box that was never verified against the emitted trajectory may be trusted.
- **Affected:** New readers who start with the Architecture Map per its own 'Read in this order' box; anyone planning point goals near obstacles.
- **Resolution applied:** Change the box label to 'validate (depth depends on route)' or split it visually, and add one sentence to §1 or §5: point goals get only the joint-position sweep; the full clearance/fidelity/dynamics report runs only for traced paths — with a pointer to the command-path page's §4 warning. The 'Every output' bullet's unconditional 'clearance' claim needs the same qualifier.

### visual-3 [medium] — fixed 2026-08-14
- **Where:** §04 'the timestamp columns lose their resolution within seconds' note, lines 1398-1408: the four-row resolution table
- **Issue:** Every band is one decade too pessimistic given the note's own stated premise of six significant digits (default C++ ostream precision, confirmed by the comment at Hardware.cpp:522, 'at the stream's default precision', and no setprecision call anywhere in Hardware.h/cpp). Six significant digits give: t∈[1,10) → 0.01 ms, [10,100) → 0.1 ms, [100,1000) → 1 ms, [1000,10000) → 10 ms. The note's own sample contradicts its table: '2800.12' has two decimals = 10 ms resolution at t≈2800 s, while the table's applicable row (t < 10000 s) claims 100 ms. The dependent sentence 'a 0.34 ms round trip is 3 quanta' holds only in the [10,100) band; in the first ten seconds it is ~34 quanta.
- **Why it matters:** The table is presented as the authoritative caveat for all latency analysis of run CSVs and is wrong by 10x in every band, self-contradicted by its own sampled evidence. A student citing the quantisation limits in the thesis, or deciding which portion of a log is usable, gets the wrong numbers.
- **Affected:** Anyone doing timing analysis on run CSVs; the thesis write-up.
- **Resolution applied:** Shift every row one decade ([1,10)→0.01 ms, [10,100)→0.1 ms, [100,1000)→1 ms, [1000,10000)→10 ms) and move the '0.34 ms is ~3 quanta' example to the [10,100) band; note that t≥1000 s at 10 ms resolution matches the sampled 2800.12 exactly. One fix also covers the §6 'Timestamps quantise' trap that references this table.
- **Verifier's correction:** Downgraded from high to medium: the error direction is conservative (readers discard usable data rather than trust bad data) and the note's qualitative conclusion — long-run latency figures are quantisation artefacts — survives intact.

### visual-4 [medium] — fixed 2026-08-14
- **Where:** §01 SVG input-thread lane, line 610: <path class="flow flow-ref" d="M470,258 V306"/>
- **Issue:** The arrow from the named pipe lands at (470,306), the top edge of the 'ValidateJointTrajectory' box (x 366-566), not the 'parse grammar' box (x 150-330). The drawn pipeline is FIFO → validate, while the separate parse→validate arrow (M330,332 H366, line 611) still exists — so parse grammar has no input arrow and the figure shows two contradictory entry points into validation.
- **Why it matters:** The stated order (§5: text arrives on the FIFO, degrees become radians at parse, then the checks run) is parse-then-validate. The figure says the FIFO bypasses parsing; a student tracing an ingest rejection starts at the wrong stage.
- **Affected:** Readers tracing the ingest path in the figure; anyone debugging ingest rejections.
- **Resolution applied:** Route the FIFO output to the parse box, e.g. d="M470,258 V282 H240 V306", so the chain reads FIFO → parse grammar → ValidateJointTrajectory → mailbox.

### visual-5 [low] — fixed 2026-08-14
- **Where:** §01 legend, lines 502-507, versus the SVG's red (--stop) elements
- **Issue:** The legend's four entries (planning, reference, measurement, and a grey 'control & data') omit the --stop red used on six semantically loaded elements (ValidateJointTrajectory, clip q̇, lead bound, software limit hold, the stop-priority box, the 'reject → stderr' text), while the grey 'control & data' entry matches no drawn arrow — every path in the SVG carries flow-ref, flow-meas or flow-plan; the bare #ah grey marker is defined but never used.
- **Why it matters:** The legend is the figure's decoding contract (the CSS comment promises signal colours 'used identically in prose, diagram and tables') and it is incomplete in one direction and phantom in the other.
- **Affected:** First-time readers decoding the figure's colours.
- **Resolution applied:** Add a '<i style="background:var(--stop)"></i> limits / guards / stops' entry and drop the unused 'control & data' entry.
- **Verifier's correction:** Downgraded from medium to low: red's meaning is recoverable from the page's pervasive stop-tinting in prose, equations and tables, so this is a completeness defect in the key, not misinformation.

### visual-6 [medium] — fixed 2026-08-14
- **Where:** CSS tokens: --ink-3 (#6b7684 light / #7d8794 dark) used for .eyebrow, .provenance, nav.toc, figcaption, sec-num, thead th, .eq-label, svg .lbl-sm/.lbl-lane
- **Issue:** Recomputed WCAG contrast confirms the reviewer's numbers exactly: #6b7684 on --ground #f6f7f9 = 4.31:1; on --surface-2 #eef0f4 = 4.05:1; dark #7d8794 on --surface-2 #1e232b = 4.33:1. All three fail AA's 4.5:1 for normal text, and the affected text is uniformly small (9.5px SVG sub-labels on surface-2 box fills, 10.5px uppercase table headers on surface-2, 10.5-13.5px chrome) — none qualifies for the 3:1 large-text allowance.
- **Why it matters:** The failing pairs carry real information: every table's column headers, every SVG box's second line (units, thresholds, file names), and the figure captions — precisely the details (units, limits, frames) the page tells the reader to check.
- **Affected:** All readers, most acutely light mode and the 9.5px SVG labels; low-vision readers fail outright.
- **Resolution applied:** Darken light --ink-3 toward #5d6773 and lighten dark --ink-3 toward #8b95a2, or switch .lbl-sm and thead th to --ink-2 (verified 8.52:1 on light ground), keeping --ink-3 for decoration only.

### visual-7 [low] — fixed 2026-08-14
- **Where:** Fig. 2 SVG (viewBox 0 0 960 320): the two law boxes (x 390-560) and PositionIntegration (x 798-938)
- **Issue:** Several 10.5px .svg-s labels overrun their boxes at ~6.3 px/glyph: 'qdot_ref + Kp·wrap(q_ref−q)' (27 glyphs from x=402 ends ≈572 vs box edge 560), 'world hold anchor / base hold' (29 glyphs, ends ≈585, ~25px overflow), 'PD + DLS + null-space avoid' (ends ≈572), '+ j2/4/6 boundary hold' (22 glyphs from x=810 ends ≈949 vs edge 938), 'log push + edge prints' (ends ≈755 vs 754, grazing). Wider fallback fonts (DejaVu Sans Mono on Linux) worsen it.
- **Why it matters:** The overflowing strings are the two control-law formulas and the boundary-hold note — the load-bearing labels — and text protruding past its box weakens the box=stage visual grammar.
- **Affected:** Fig. 2 readers, worst where the font stack falls through to DejaVu Sans Mono.
- **Resolution applied:** Widen the law boxes to ~195px and shift the clamp column right, or wrap each formula onto two lines; widen the PositionIntegration box or shorten its sub-label.
- **Verifier's correction:** Downgraded from medium to low, and the Fig. 1 half of the claim is dropped: 'world pose → hold' (ends ≈671), 'plan file (saved)' (≈673) and 'validate: limits, clearance, FK' (≈489 vs inner edge 492) all fit inside their boxes at nominal metrics — tight padding, not overflow.

### visual-8 [low] — fixed 2026-08-14
- **Where:** §01 SVG lane label, line 532: 'CONTROLLER PROCESS — RT thread, 500 Hz, no blocking I/O' versus §8.5 row at line 1867
- **Issue:** The lane label asserts 'no blocking I/O' as an achieved property of the RT thread while the same page's §8.5 documents the opposite: 'std::cout runs on the loop thread… it is a blocking call.' Repository check confirms the §8.5 side: std::cout calls sit inside the control loop in Runner.cpp (trajectory accept/reject/complete prints and the 1 Hz status line, currently at lines ~581-651).
- **Why it matters:** A student citing the architecture invariant from the figure would be wrong, and the invariant is exactly the CLAUDE.md constraint ('blocking terminal operations outside the 500 Hz control thread') that is report-relevant.
- **Affected:** Readers who take the lane label as a verified invariant; the thesis's architecture description.
- **Resolution applied:** Soften the label to 'RT thread, 500 Hz — no blocking file I/O (console prints in post-Send slack, §8.5)' or simply 'RT thread, 500 Hz'.
- **Verifier's correction:** Downgraded from medium to low: the same artifact documents the exception prominently in §8.5, so the exposure is one over-strong label rather than undocumented misinformation. Note the §8.5 line citation (Runner.cpp:505-538, 577) has drifted from the current working tree (~581-651).

### visual-9 [medium] — fixed 2026-08-14
- **Where:** §01 SVG operator-row dashed arrows, lines 588-589: d="M120,90 V132" and d="M336,90 V132"
- **Issue:** Both dashed operator inputs land on the wrong planner stage. The goal arrow (x=120) lands on the 'StartState' box (x 20-172) — but StartState reads the previous run's CSV (StartState.h, and the box's own sub-label says 'last run CSV'), not the goal; the goal feeds IK initialisation/GPMP2. The '--box obstacle' arrow (x=336) lands on the 'IK initialiser' box (x 204-368) — but the repository's PathIk.cpp contains zero SDF/obstacle references; obstacles feed the SDF/factor-graph stage. Config.h and Ctrl+C have no arrows at all.
- **Why it matters:** The figure teaches two false data-flow facts — that the operator's goal is the plan's start state, and that the obstacle box conditions the IK seed — both contradicted by the page's own text and by the source.
- **Affected:** Readers learning the planner pipeline from the figure; anyone reasoning about why an obstacle didn't change the IK seed.
- **Resolution applied:** Route the goal arrow to the IK-initialiser box, the --box arrow to the 'GPMP2 factor graph' box (x 400-580), and label or draw StartState's actual input (the newest run CSV).

### visual-10 [low] — fixed 2026-08-14
- **Where:** All tables in both files; architecture-map heading sequence (h3 'New to this repository?' at line 156 before the first h2 at line 177; §4 cards jump h2 → h4) and the rowspan'd td 'Before any motion' at line 429
- **Issue:** grep confirms zero scope attributes on any th in either file; the Architecture Map's safety table uses rowspan'd td cells as de-facto row-group headers, invisible to screen-reader table navigation; and the heading outline is broken twice (h3 before the first h2, and h4 cards directly under an h2 with no h3 level).
- **Why it matters:** Screen-reader users lose column/row associations in the two most information-dense tables (who-controls-what; the safety gates), and the heading outline misrepresents document structure. Mechanical fixes.
- **Affected:** Assistive-technology users; anyone navigating by heading outline.
- **Resolution applied:** Add scope="col" to header cells, convert the rowspan'd phase cells to th scope="rowgroup", retag the intro note's h3 as a styled p (or promote it), and change the card h4 elements to h3 (they are styled by class anyway).

### visual-11 [low] — fixed 2026-08-14
- **Where:** §01 SVG: mailbox→splice path (line 657, d="M692,358 V397 H95 V416"), Vicon arrow terminus (line 693, V412), feedback arrow (line 685, d="M240,652 H60 V480")
- **Issue:** Three small geometry misses: (a) the mailbox→splice-guard horizontal segment runs at y=397, only ~2-3 px below the RT lane label's descenders (baseline y=392, label spans x≈6-400) and 3 px above the y=400 divider — visually crowding the label though not overlapping at nominal metrics; (b) the Vicon arrowhead stops at y=412, 4 px shy of the tracking-law box top at y=416, so it floats; (c) the feedback arrow ends at (60,480), 8 px below the splice-guard box bottom (y=472), pointing at empty inter-lane space.
- **Why it matters:** Individually cosmetic, but the two floating arrowheads leave genuinely ambiguous targets (what does feedback enter? what does Vicon enter?), and (b) compounds the mis-routed Vicon arrow finding.
- **Affected:** All figure readers.
- **Resolution applied:** Route the y=397 segment at y≈406 (below the divider), extend the Vicon path to V416 and the feedback path to V472 so both heads touch their target boxes.
- **Verifier's correction:** Part (a) softened: at nominal font metrics the path clears the label descenders by ~2-3 px — crowding, not the claimed 'grazes or touches'; (b) and (c) confirmed as stated.

### visual-12 [low] — fixed 2026-08-14
- **Where:** Fig. 1 start-state bezier (line 298, M 130 482 C 130 420 160 350 268 306) and its two-line label at (126,400)/(126,414); header claim 'Colors mean status everywhere on this page' (line 139) versus the legend (lines 142-145)
- **Issue:** (a) The dashed bezier crosses its own label: solving the cubic, the curve sits at x≈150.6 at y=400 and x≈143.7 at y=414 — inside both label lines' text spans (starting x=126), cutting through roughly characters 3-5 of each line. (b) The curve is coloured --bring (blue) and the header promises colours mean status everywhere, yet the page legend defines only LIVE-green and FROZEN-red; blue is explained only in a passing figcaption sentence.
- **Why it matters:** A reader applying the page's own colour contract reads the blue dashed loop as an undefined status; the stroke-through-text collision makes the loop's label harder to read at the seam the page itself highlights.
- **Affected:** Fig. 1 readers decoding colour; the label collision affects everyone slightly.
- **Resolution applied:** Move the labels right (x≈165) or reshape the bezier's first control points; add blue to the legend as 'data loop (not a status)' or soften 'everywhere' to 'on cards and chips'.
- **Verifier's correction:** The reviewer's curve position (x≈132-138) was slightly off — the actual crossing is at x≈144-155, a few characters deeper into the labels; the collision itself is confirmed.

### visual-13 [low] — fixed 2026-08-14
- **Where:** Telemetry-name wildcards across §4/§6/§7: 'reqvel_j*' (lines 1214, 1257, 1322), 'reqvel_j×'/'meas_j×'/'cmd_j×'/'fault_j×' (lines 1560, 1619, 1627, 1635, 1667, 1749), 'reqvel_j1..7' (lines 2054-2059)
- **Issue:** Three wildcard conventions coexist for the same CSV column families — j*, j× (multiplication sign, visually near-identical to x), and j1..7 — and none of the first two is a literal column name. The real headers are reqvel_j1..reqvel_j7 etc. (Hardware.h:223-229). No sentence in §6's intro defines the convention.
- **Why it matters:** A student who copies 'reqvel_j×' into grep or pandas gets zero matches and may conclude the column is absent from their log format; consistency with the actual header row is what makes the debugging tables usable verbatim.
- **Affected:** Anyone scripting against the CSV from the §6 telemetry column.
- **Resolution applied:** Standardise on j1..7 (matching the real headers) or state once in §6's intro that the wildcard stands for the joint index 1-7, and eliminate the × variant entirely.

### visual-14 [medium] — fixed 2026-08-14
- **Where:** Fig. 1 robot box subtitle, line 276: 'right .9 / left .10'; §3 safety table, line 441: 'outward command past Table-39 − 2°'
- **Issue:** Two problems, one worse than reported. (a) The '.9 / .10' shorthand is not merely undefined — it is INVERTED: the repository consistently assigns right = 192.168.1.10 and left = 192.168.1.9 (Config.h:33-34 kRightRobotIp="192.168.1.10", kLeftRobotIp="192.168.1.9"; confirmed by the shared-robot-io design doc, panel/server.py, and every '[left] == left arm (192.168.1.9)' line in the raw prompt log). The map's own ORPHANED card even warns that test_kinova 'labels the .9 arm wrongly' — and then Fig. 1 repeats an arm/IP swap itself. (b) 'Table-39' is Kinova user-guide vocabulary defined only on the companion page; the map's six-term primer does not include it, and no full IP appears on the page to decode the octets from.
- **Why it matters:** Arm identity vs IP is hardware-relevant information (which dashboard to open, which process lock, which arm a log belongs to), and the figure states it backwards on the page positioned as document #1 in the reading order.
- **Affected:** New readers following the map's recommended reading order; anyone identifying an arm by IP during a session.
- **Resolution applied:** Correct and expand the subtitle to 'right 192.168.1.10 / left 192.168.1.9 (arm IPs)', and gloss Table-39 inline: 'past the documented joint range (Kinova user-guide Table 39) minus 2°'.
- **Verifier's correction:** Upgraded from low to medium: the reviewer treated this as undefined shorthand and their suggested expansion ('right 192.168.1.9 / left 192.168.1.10') would have baked the inversion in. The mapping in the figure is factually backwards, not just cryptic.

### visual-15 [low] — fixed 2026-08-14
- **Where:** §8.5 row, line 1861: 'nothing checks whether LM converged'; §11 goal.yaml example, line 2013: 'orientation_rpy_deg: [ 90, 0, 90 ]'; §4 goal-prior row, line 1346: 'σ_rpy'
- **Issue:** Two abbreviations escape the page's vocabulary discipline: 'LM' appears bare twice in the §8.5 row while the vocabulary (line 479) and solver table (line 1348) always write 'Levenberg–Marquardt' in full without ever introducing the abbreviation; and 'rpy' is never expanded anywhere ('roll' does not occur in the file) despite sitting in the load-bearing config key orientation_rpy_deg and the σ_rpy prior row.
- **Why it matters:** The page's own standard ('every recurring term, defined once') is what makes it usable standalone; rpy lives in the one file a student edits before every run, and roll-pitch-yaw ordering/axis convention is a classic orientation-bug source.
- **Affected:** Students editing goal.yaml or reading §4 Group D; non-robotics readers hitting 'LM' cold.
- **Resolution applied:** Add a vocabulary row 'rpy (roll-pitch-yaw)' stating the parser's axis/ordering convention, and write 'LM (Levenberg–Marquardt)' at the §8.5 use or spell it out.

### Refuted (visual)
- Fig. 2 clamp box 'ClampJointVelocity / 76/66.5 °/s per joint' states working-tree values as the running system's with no qualifier (architecture-map)
  - *Refuted because:* Already handled in the same artifact: the page's header scopes everything to 'HEAD 02348ecc + uncommitted working tree', and §5's first, bolded tension bullet says exactly this — the values are uncommitted, every on-disk binary predates them, and 'trust a run's CSV preamble — not today's source' — so the figure accurately documents the page's declared scope and the discrepancy the reviewer fears is explicitly flagged three paragraphs later; a figure annotation would be polish, not a correction.


## Lens: consistency — 13 confirmed, 3 refuted

### consistency-0 [high] — fixed 2026-08-14
- **Where:** Section 1, Fig. 1, Kinova Gen3 robot box: "right .9 / left .10" (file line 276)
- **Issue:** The arm-to-IP mapping is swapped. The working tree's Config.h:33-34 defines kRightRobotIp = "192.168.1.10" and kLeftRobotIp = "192.168.1.9" — the opposite of the diagram. This is the only arm-to-IP mapping in either artifact.
- **Why it matters:** File A's recovery table (line 2077) sends the student to "the Kinova web dashboard at the arm's IP" to clear faults, and the only mapping the pair of pages supplies is this reversed one — so a student would operate the configuration surface (clear_faults, set_joint_limits, dashboard) of the wrong physical arm. B's own test_kinova card even criticises the dead file for labelling "the .9 arm wrongly" while its figure makes the same mistake.
- **Affected:** Anyone doing fault recovery, joint-limit configuration, or dashboard inspection on a specific arm; anyone matching per-arm run logs to a machine.
- **Resolution applied:** Correct the figure to right .10 / left .9, preferably with full addresses and the citation Config.h:33-34, since ".9/.10" also assumes the student knows the 192.168.1.x subnet.

### consistency-1 [high] — fixed 2026-08-14
- **Where:** A §3.6 heading (line 1040) "live on every idle-hold cycle", §0 Fact 1 (line 423), §5 completion note (line 1539), §8.1 (line 1809); B §2 (line 336) "the Cartesian law on every idle hold", Fig. 2 fork, LIVE card (line 411) "now runs on every idle-hold cycle"
- **Issue:** The headline claim — the Cartesian law runs on EVERY idle-hold cycle — is false before the world hold has ever engaged. Controller.cpp:213 (verified): if (!world_hold_ever_engaged_ && reference.joint) an idle hold runs SolveJointTracking, the joint law, with the comment "the no-Vicon behaviour is bit-for-bit today's". So on any run without fresh Vicon, and on every idle cycle before the first engage, the joint hold runs — exactly what A's own §3.6b source note admits ("with no Vicon at all, fresh is never true and the pre-Vicon joint hold runs unchanged") in direct contradiction of its §3.6 heading. B's LIVE card's claim that the null-space push-back "is finally on a live path during holds" is likewise untrue for no-Vicon and pre-engage holds.
- **Why it matters:** A student debugging a no-Vicon idle hold will predict Cartesian telemetry (pd_*, taskvel, null-space activity) and instead see the joint hold with p_desired NaN — and, worse, could carry a wrong thesis safety claim that null-space limit avoidance is active during all holds. The claim IS correct once the hold has engaged at least once (the fallback is then the re-seated Cartesian base-frame hold), which is exactly the qualification both pages omit.
- **Affected:** Anyone debugging a no-Vicon or early-run idle hold; anyone reasoning about which soft-safety layers are active during holds, including the thesis safety argument.
- **Resolution applied:** Qualify every occurrence: the Cartesian law runs on idle-hold cycles once the world hold has engaged at least once; before any engage — including all no-Vicon runs — an idle hold runs the pre-Vicon joint hold unchanged (Controller.cpp:213-226). Fix A §3.6 heading, §0 Fact 1, §5 completion note, §8.1 bullet, and B's §2 text, Fig. 2 fork/caption and Cartesian-law card in one pass.
- **Verifier's correction:** A's §5 completion note is wrong only for never-engaged runs: after a completed trajectory ON A RUN WHERE THE HOLD HAD ENGAGED, the base-frame-hold fallback described is correct. The defect is the missing pre-first-engage/no-Vicon carve-out, not the post-engage behaviour.

### consistency-2 [medium] — fixed 2026-08-14
- **Where:** Section 3 safety table, Vicon freshness gate row (line 436): "Trips on: sample age > 50 ms, invalid segment, or repeated frame"
- **Issue:** "or repeated frame" is not a trip condition. The real predicate (Runner.cpp:425-429, verified) is fresh = valid ∧ sequence > 0 ∧ isfinite(age) ∧ age ≤ 0.05 s. Repeated frames are the normal ZOH case — each 100 Hz sample is reused for ~5 of the 500 Hz cycles, as B's own six-terms note defines two sections earlier — and seq > 0 means "a sample has ever arrived", not "the frame is new".
- **Why it matters:** Taken literally, B says the world hold freezes on ~4 of every 5 cycles, contradicting the ZOH design the same page defines. A student writing up the freshness gate from the overview page would state a safety-relevant predicate wrongly. File A carries the correct predicate (§3.6b), so the two pages also contradict each other.
- **Affected:** Students learning the world-hold freshness mechanism; anyone reproducing the predicate in analysis scripts or the thesis.
- **Resolution applied:** Replace with the real predicate: "sample age > 50 ms, segment invalid/occluded, or no sample ever received (seq = 0)", and note that ZOH reuse inside the 50 ms window is normal operation.
- **Verifier's correction:** Downgraded from high to medium: the misstatement is purely documentary — File A gives the correct predicate, and the gate degrades (freeze) rather than stops, so no hazardous action follows from the misreading.

### consistency-3 [medium] — fixed 2026-08-14
- **Where:** Section 2, Fig. 2 clamp box (line 366): "ClampJointVelocity / 76/66.5 °/s per joint"
- **Issue:** The diagram states the uncommitted working-tree limits with no working-tree/HEAD marker, and B nowhere states the HEAD value of 45 deg/s uniform (verified: no such figure appears anywhere in B). B's own §5 tension bullet says "every on-disk controller binary was built before it" — so a binary run today clips at 45, and Fig. 2 silently contradicts B's own caveat. File A handles the same fact correctly in its §4 warn note and Group A row ("working tree; HEAD: 45").
- **Why it matters:** Fig. 2 is the natural first stop for "what clips my command"; a student reading it alone will predict 76/66.5 from an existing binary and misread a run whose CSV preamble says 45.
- **Affected:** Anyone interpreting velocity saturation in existing run CSVs or predicting arm speed before the pending rebuild.
- **Resolution applied:** Annotate the Fig. 2 clamp box "(working tree; built binaries: 45)" or move the number into text carrying the caveat; state the HEAD value 45 somewhere in B.
- **Verifier's correction:** The reviewer's secondary claim about A's Group B rows is weak: those rows (lines 1301-1302) sit inside §4, which opens with a prominent warn note carrying the full working-tree/HEAD caveat, so A is adequately covered; the actionable defect is B's Fig. 2 and B's missing HEAD value.

### consistency-4 [medium] — fixed 2026-08-14
- **Where:** A §5 takeover src note (line 1475): "Config.h:153 compiles kTakeoverHoldS = 0.05 s"; A §3.6 eq 5-6 src (line 1097): "ramp applied Controller.cpp:177–179"; A lines 423/1539 and B Fig. 2 caption (line 400): "Targets.cpp:280–284" / "303–305"
- **Issue:** Verified-wrong file:line citations on a page whose provenance promises "every claim cites file:line". (a) kTakeoverHoldS is at Config.h:162 in the working tree (156 at HEAD); line 153 is kControlDtS — the cite matches neither version. (b) The limit-avoidance ramp is applied at Controller.cpp:298-300 (ramped_gains.limit_avoid_gain_s_inv *= UnitRamp(...)); line 179 is "WorldHoldInput hold_in;", adjacent to the reference.pose branch A's §0 declares "dead at runtime" — the cite lands a reader in dead code while reading about a live mechanism. (c) Both pages cite Targets.cpp:280-284 and 303-305 for the joint_is_idle_hold flags; the assignments are at lines 285 and 306 (verified at HEAD and working tree), just outside both cited ranges.
- **Why it matters:** Students are explicitly told to jump from these pages into the code by line. The ramp cite is the worst landing possible (inside the dead pose branch); the takeover-hold cite lands on kControlDtS while checking a duration the page itself says other comments got wrong by 10×.
- **Affected:** Anyone verifying claims against source, which the page's own method demands.
- **Resolution applied:** Sweep the citations against the working tree: Config.h:162 for kTakeoverHoldS, Controller.cpp:298-300 for the ramp, Targets.cpp:285 and :306 for the idle-hold flags (fix B's Fig. 2 caption to match).
- **Verifier's correction:** The Targets.cpp cites are near-misses (the ranges cover the immediately preceding context, off by one line), so they are cleanup, not confusion; the Config.h and Controller.cpp cites are the substantively wrong ones. Also note Config.h:153 was correct for neither HEAD nor working tree, so this is not a version-skew artifact.

### consistency-5 [low] — fixed 2026-08-14
- **Where:** A §3.6b intro (line 1109): "All five knobs are compiled and echoed into every CSV preamble"; §6 parsing note (line 1690): "now including the five world_hold_* knobs"
- **Issue:** There are six world_hold knobs, not five. Config.h:330-346 (verified) defines kWorldHoldAutoEngage, FreshMaxAgeS, RampS, MaxErrorM, MaxRotErrorRad, ReanchorAfterS, and Main.cpp:136-146 echoes six world_hold_* preamble lines.
- **Why it matters:** A student auditing a CSV preamble against the promised five will find six lines and wonder which is undocumented — or, counting from the doc, will miss that auto_engage is the knob that can disable the whole hold.
- **Affected:** Anyone auditing a run's preamble against the documentation.
- **Resolution applied:** Say six, and list them once so the preamble is checkable line for line.

### consistency-6 [low] — fixed 2026-08-14
- **Where:** Section 1 "Every output", Run CSV bullet (line 324): "the world-hold evidence quartet hold_state / world_err_m / world_err_rot_rad / hold_ramp"
- **Issue:** The "quartet" omits hold_reanchor_count, the fifth hold-evidence column. A lists all five in §3.6b, §6 and §11, and Controller.cpp:186-193 (verified) populates all five; hold_reanchor_count appears nowhere in B.
- **Why it matters:** hold_reanchor_count is the only evidence that a blackout silently moved the absolute anchor — the failure A's §6 explicitly warns about ("silently moves the absolute point (counted)"). A student taking B's quartet as the complete set will not look for it.
- **Affected:** Anyone analysing world-hold runs from B's column summary.
- **Resolution applied:** Say quintet and add hold_reanchor_count, or drop the count word.

### consistency-7 [low] — fixed 2026-08-14
- **Where:** B Fig. 1 controller box (line 263) "safety: 5 pre-motion gates" + §3 rowspan-5 table (lines 429-433); A §5 takeover sequence (lines 1437-1476)
- **Issue:** The pre-motion gate inventories do not close across the two pages. B names five gates by symbol (ProcessLock, RobotReadyForTakeover, VerifyKinematicHardLimits, EnsureJointLimits, takeover hold); A describes four in prose names only (readiness gate, hard-speed gate, joint-limit gate, takeover hold) with zero occurrences of any of B's four symbol names (grep verified: 0 hits each), and never mentions ProcessLock or the process lock in any form.
- **Why it matters:** A student mapping B's five onto A's four finds no name overlap and cannot tell whether A omitted a gate or B invented one. ProcessLock — real code, ProcessLock.h in basic_control/src — ends up documented only as a table row.
- **Affected:** Students building the safety-gate picture from the two pages together.
- **Resolution applied:** Add ProcessLock to A's takeover sequence (it is acquired before connection, Main.cpp:325) and tag A's prose gate names with B's symbols — "readiness gate (RobotReadyForTakeover)", "hard-speed gate (VerifyKinematicHardLimits)", "joint-limit gate (EnsureJointLimits)" — so the count closes on both pages.
- **Verifier's correction:** Downgraded from medium: the "complete gate inventory" both pages promise is explicitly attributed to docs/architecture_and_debugging_audit.md, not to A, so A makes no completeness claim; the residual defect is the unmappable naming and A's total silence on ProcessLock.

### consistency-8 [low] — fixed 2026-08-14
- **Where:** B statrow (line 150) "4 frozen root trees" and §4 cards; A §1 warn note (line 709) and §8.1 (line 1815) frozen list
- **Issue:** The two pages' frozen inventories have different membership, and A never mentions TrajectoryRealTime at all (grep verified: 0 occurrences). A's bold list is "TrajectoryGeneration/, TrajectoryExecution/, ViconDataStream/ and the root main.cpp"; B's four dead directory trees are TrajectoryGeneration/, TrajectoryExecution/, ViconDataStream/ and TrajectoryRealTime/ (GHOST), with root main.cpp carried on a separate card. A reader of A alone never learns TrajectoryRealTime/joint_mpc exists and is dead.
- **Why it matters:** "What can I safely ignore" is the map's core promise; the two four-item lists differ in membership, and A's omission leaves one dead tree invisible to readers of the detailed page.
- **Affected:** New readers building the repo map; anyone tempted to build or edit under TrajectoryRealTime/.
- **Resolution applied:** Use one canonical frozen list verbatim on both pages, and add TrajectoryRealTime to A's warn note.
- **Verifier's correction:** The reviewer's internal-inconsistency claim against B is refuted in part: "4 frozen root trees" is self-consistent — B's dead directory trees are exactly four (TrajectoryGeneration, TrajectoryExecution, ViconDataStream, TrajectoryRealTime); root main.cpp, the orphaned test files and root config/ are root files/config, not trees. The surviving defect is the cross-page membership mismatch and A's TrajectoryRealTime omission. Severity reduced from medium to low accordingly.

### consistency-9 [low] — fixed 2026-08-14
- **Where:** B footer (line 570-571) "panel 286 pytest (one dirty-tree failure in test_diagnose)" vs B statrow (line 148) and A §11 (line 1998) "# 286 tests"
- **Issue:** The known failing panel test in the current working tree is disclosed only in B's small-print footer. A §11 — the hands-on page that actually gives the pytest command — presents 286 tests with no caveat.
- **Why it matters:** A student following A §11 hits a failure the instructions said nothing about and reasonably suspects their own environment or change — a wasted debugging session on the page meant to prevent exactly that.
- **Affected:** Anyone running the panel test suite from A's hands-on instructions.
- **Resolution applied:** Add the caveat where the command is given in A §11: "286 tests; one known dirty-tree failure in test_diagnose until the velocity decision is committed".

### consistency-10 [low] — fixed 2026-08-14
- **Where:** A §4 Sampling and latency (lines 1384, 1392): "936 bytes/row → 457 KB/s" from runs/2026-08-12/loop_log_left_20260812_151449.csv; §9 (line 1906): "The log costs 457 KB/s of disk"
- **Issue:** The disk-cost figure was measured on a pre-world-hold run whose preamble reads log_format 9 with a 141-column header (verified), yet the page presents it as the cost of the current 190-column format-11 log. A format-11 row is necessarily wider (49 added columns, roughly a third more bytes/row), so 457 KB/s materially understates current runs — while the same page flags Config.h's "~175 KB/s" comment as stale by 2.6×.
- **Why it matters:** A pre-HEAD measurement silently generalised to the current format is the class of leftover the page's provenance discipline exists to catch; a student budgeting disk or log-queue headroom for long world-hold runs will under-provision.
- **Affected:** Anyone budgeting disk or analysing log-writer throughput for format-11 runs.
- **Resolution applied:** Tag both figures with the format they were measured on ("format-9, 141-column run of 2026-08-12; format-11 rows are ~35% wider — remeasure on the first format-11 run").
- **Verifier's correction:** The reviewer called the source run "format-10"; it is actually format 9 with 141 columns, which strengthens the finding — the width gap to format 11 is even larger than claimed.

### consistency-11 [low] — fixed 2026-08-14
- **Where:** B orchestrator bar (line 198) "refuses DH YAML older than URDF" and input table (line 210) "→ dh_params (build-time)"; A §9 (line 1910) "gpmp2's DH kinematics"; A §00 vocabulary table (lines 461-493)
- **Issue:** The acronym DH (Denavit–Hartenberg) is used on both pages and defined on neither — grep of both files finds zero occurrences of "Denavit" — despite A's vocabulary table styling itself "every recurring term, defined once". URDF is glossed ("the robot's geometric model") but never expanded.
- **Why it matters:** The stated audience must understand the system from these pages alone, and "DH YAML older than URDF" is a condition that refuses the whole session script — a student needs to know what a DH parameter file is to reason about why a session refuses to start.
- **Affected:** Readers new to robot-kinematics conventions — the vocabulary table's stated audience.
- **Resolution applied:** Add a vocabulary row: "DH parameters (Denavit–Hartenberg) — the standard four-number-per-joint description of link geometry; dh_params_tool/flange.yaml is generated from the URDF at build time and must never be older than it." Expand URDF once on first use.
- **Verifier's correction:** The reviewer's lesser escapees are weaker than claimed and are dropped from the core finding: EE is defined in A's frames row ("E = end-effector"), "joint_mpc" in B is a literal directory name, and C3D appears once in a passage about a deleted pipeline. The confirmed defect is DH (and the unexpanded URDF).

### consistency-12 [low] — fixed 2026-08-14
- **Where:** A §00 vocabulary row (line 483): "frames W / T / B / E, world_T_base — W = Vicon world, T = torso, B = arm base, E = end-effector. world_T_base reads as ..."
- **Issue:** The symbol T is given two meanings inside one row — T the torso frame, and the infix _T_ meaning "transform" in world_T_base / world_T_mountseg — and the collision is never flagged. The transform notation then appears throughout both pages while the torso frame barely recurs.
- **Why it matters:** A student who internalises "T = torso" from the frame list can parse world_T_mountseg as "world, torso, mountseg" — a real misreading of the notation the entire world-hold chain (§3.2b answer 3, §9) is written in. The collision is live because Christian's own W/T/B/E notation is the project standard.
- **Affected:** Readers new to transform notation — the vocabulary table's audience.
- **Resolution applied:** Split into two rows — one for the frame letters W/T/B/E, one for the a_T_b transform notation — with an explicit sentence that the infix T means "transform", not the torso frame.

### Refuted (consistency)
- The per-cycle CSV carries three names ("run CSV", "loop CSV", "run log") across the two pages, and B Fig. 2 abbreviates "base-frame hold" to "base hold".
  - *Refuted because:* Style-only nitpick: each usage is anchored in context — B's Fig. 1 label is literally "runs/…/loop CSV — 500 Hz rows" (the path names the file), A §11 ties "run log" to the exact filename loop_log_<arm>_*.csv, and B Fig. 2's caption expands "base hold" to "the re-seated base-frame hold" in the same figure — so no reader can mistake the names for different files; harmonising them is polish, not an error.
- A §8.2's last design-choice row ("world hold ... deferred to the Vicon slices ... leaving world-frame hold unimplemented") survives without a supersession marker.
  - *Refuted because:* Already handled prominently in the same artifact: the row is dated ("Recorded: 2026-08-12 rollback record") and the supersession is stated three times — §0's deck ("The world hold went live at HEAD 02348ecc"), §5's completion note, and §9's choice note ("The 2026-08-12 rollback's debt was repaid") — so the wrong reading requires skipping the page's own headline; an appended cross-reference would be polish, not a correction.
- B's header claim "Colors mean status everywhere on this page" is contradicted by the blue dashed CSV-loop arrow, and the legend covers only two of four CSS palette families.
  - *Refuted because:* Style-only overreach, resolved where it occurs: the single non-status use of colour (the blue arrow) is explained in the same figure's caption ("Dashed blue: the measured-state feedback loop between runs"), the unused --dorm family appears on no element a reader sees, and every card, chip and coloured box on the page does encode status — no factual misreading of the system can result.


## Compliance-sweep residuals (second workflow)

The strict final sweep found 14 items where a fix was partial, inconsistent
across locations, or (in one case) newly broken — most notably the
scope-attribute regex that had corrupted every `<thead>` tag in the Command
Path into invalid HTML. All 14 were applied on 2026-08-14:

1. **NEW DEFECT introduced by the fix for visual#11 (table th scope attributes)** — File A gen3-command-path.html — all 20 tables (lines 465, 723, 747, 840, 862, 1258, 1305, 1324, 1348, 1382, 1397, 1428, 1565, 1713, 1834, 1868, 1894, 1954, 2076, 2101)
   - Every <thead> opening tag was corrupted into '<th scope="col"ead>' by the scope-attribute search-replace (a th-with-junk-attribute before the <tr>, and 20 closing </thead> tags now have no opener). This is invalid HTML in every table of the page; browser error-recovery may inject a stray empty header cell or misplace the header row. File B is unaffected.
   - Applied: Replace all 20 occurrences: '<th scope="col"ead>' -> '<thead>' (keep the scope="col" attributes already present on the real <th> cells inside the <tr>).
2. **pedagogy#2 / visual#16 / consistency#1 (arm IPs) — A-side half not done** — File A section 11, robot_fault recovery row (line 2104); no IP string exists anywhere in File A (grep '192.168' = 0 hits)
   - File B now decodes .10/.9, but File A still instructs 'the Kinova web dashboard at the arm's IP' without ever stating either IP, so the instruction is unexecutable from File A alone — the exact defect the finding named.
   - Applied: In the robot_fault row: 'or the Kinova web dashboard at the arm's IP;' -> 'or the Kinova web dashboard at the arm's IP (right arm 192.168.1.10, left arm 192.168.1.9 — Config.h:33–34);'
3. **planner#2 / visual#3 (map presents full validation as unconditional) — bullet not qualified** — File B section 1, 'Every output' list, 'Planner stdout' bullet (line 330)
   - Fig. 1's box and caption were fixed ('validate — route-dependent'), but the bullet still promises 'a validation report that states measured quantities (clearance, error percentiles, velocity headroom)' unconditionally — on the common point-goal route no such report is printed. The finding explicitly required qualifying this bullet.
   - Applied: 'plus a validation report that states measured quantities (clearance, error percentiles, velocity headroom) before its verdict.' -> 'plus, on traced paths, a validation report that states measured quantities (clearance, error percentiles, velocity headroom) before its verdict; a point goal prints on
4. **vicon#4 (EE-in-world figure's independence overstated) — B-side location untouched** — File B section 5, panel/telemetry bullet (lines 562–563)
   - File A's two locations were softened, but B still says the figure 'is deliberately an independent reconstruction' with no caveat that it is built on the controller's logged FK p_* — the thesis-evidence overstatement the finding targeted.
   - Applied: 'whose EE-in-world figure is deliberately an independent reconstruction.' -> 'whose EE-in-world figure recomputes world_T_base independently of the hold's own arithmetic — though it still builds on the controller's logged FK (p_*); the fully independent check against the tracked EE segments is not y
5. **planner#6 (validation-row asymmetries) — sweep-row corrections not applied** — File A section 4, Group D, 'Joint-position sweep' row (line 1359)
   - The breach columns of the three traced-path rows were unified (exit 4, nothing emitted — done), but the sweep row still carries the wrong '(both routes)' tag (ValidateJointPath's only caller is the point route) and still says 'every support state' although the full densified ~1 kHz trajectory is swept.
   - Applied: 'Joint-position sweep <span class="k">(both routes)</span>' -> 'Joint-position sweep <span class="k">(point route; the traced route enforces joint limits via the report's dense-reconstruction margin)</span>', and 'every support state within &plusmn;S<sub>i</sub>' -> 'every densified point (the sourc
6. **consistency#9 (frozen-tree inventories differ; TrajectoryRealTime missing from A) — fixed in §1 but not §8.1** — File A section 8.1, last bullet (line 1828)
   - The §1 warn note now lists five frozen items including TrajectoryRealTime/, but §8.1's frozen list still names only four — the two lists inside File A now disagree with each other (the exact cross-location inconsistency this sweep is meant to catch).
   - Applied: '<code>TrajectoryGeneration/</code>, <code>TrajectoryExecution/</code>, <code>ViconDataStream/</code> and the root <code>main.cpp</code> are <strong>frozen and unbuildable</strong>' -> '<code>TrajectoryGeneration/</code>, <code>TrajectoryExecution/</code>, <code>ViconDataStream/</code>, <code>Trajec
7. **consistency#11 (457 KB/s measured on a format-9 run) — §9 occurrence not tagged** — File A section 9, Computational cost row (line 1919)
   - The §4 occurrence now carries '(format-9 run, 141 columns; format-11 rows carry 190 — scale ≈ ×1.35)', but §9 still states 'The log costs 457 KB/s of disk' with no qualifier — the number was updated in one table but not the other.
   - Applied: 'The log costs 457&nbsp;KB/s of disk.' -> 'The log costs ~457&nbsp;KB/s of disk (format-9 measurement; format-11 rows are ~1.35&times; wider &mdash; &sect;4).'
8. **math#2 / visual#4 residual (corrected resolution ladder vs stale 'first minute' prose)** — File A section 6 parsing-traps note (line 1701) and section 8.5 timestamp row (line 1876)
   - The §4 ladder now correctly says round-trip measurement survives to 'roughly the first 100 s', but §6 still says latency 'past the first minute is a rounding step' and §8.5 says 'no sub-quantum information after the first minute' — at t = 60–100 s the resolution is 0.1 ms (~3 quanta for a 0.34 ms round trip), so both phrases contradict the corrected table and discard usable data.
   - Applied: Line 1701: 'past the first minute is a rounding step' -> 'beyond roughly the first 100&nbsp;s is a rounding step (&sect;4's table)'. Line 1876: 'carry no sub-quantum information after the first minute' -> 'carry no sub-quantum information beyond roughly 100&nbsp;s of run time'
9. **math#4 / consistency#5 residual (two stale file:line citations survived the sweep)** — File A §3.1 variables table, Δt row (line 848); File A §5 completion note (line 1549)
   - (a) The Δt row still cites 'Runner.cpp:334–339' — that region is takeover-hold code; the ClampedCycleDt call is at Runner.cpp:380–386 (verified; §3.4's own citation was fixed but this row was not). (b) The completion note still cites 'Targets.cpp:303–305' for the completed-trajectory idle flag; the assignment is at line 306 (verified), and Fact 1 and B's Fig. 2 caption now both say :306 — an internal inconsistency.
   - Applied: Line 848: 'Runner.cpp:334&ndash;339' -> 'Runner.cpp:380&ndash;386'. Line 1549: 'Targets.cpp:303&ndash;305' -> 'Targets.cpp:303&ndash;306'
10. **vicon#7 residual (Fig. 1 execution-context count and Vicon box lane placement)** — File A section 1 deck (line 502) and Fig. 1 Vicon box (lines 693–695, drawn at y=216–258 in the planner band)
   - The arrow mis-route and missing Cartesian law were fixed (dispatch box + V416 terminus), but the deck still says 'Four execution contexts' naming neither the Vicon-acquisition nor the log-writer thread, and the Vicon box still sits in the planner-process band although the SDK thread runs inside the controller process.
   - Applied: Deck: '&mdash; three of them outside the real-time loop.' -> '&mdash; three of them outside the real-time loop &mdash; plus two controller-side helper threads the figure also shows: Vicon acquisition and the log writer.'. Vicon box sub-label: 'SDK thread &rarr; slot' -> 'SDK thread (controller proce
11. **visual#6 residual (legend phantom entry)** — File A section 1 legend (line 509)
   - The missing red 'limits, guards, stops' entry was added, but the grey 'control & data' entry remains and still matches no drawn arrow (every SVG path carries flow-ref/flow-meas/flow-plan; the bare grey marker is unused) — the phantom half of the finding.
   - Applied: Delete the line '<span><i style="background:var(--ink-3)"></i> control &amp; data</span>'
12. **visual#16(b) (Table-39 vocabulary undefined on the map)** — File B section 3 safety table, j2/4/6 software boundary row (line 446; also 'Table-43' at line 559)
   - File A now defines Table 39/41/43 in its vocabulary, but B — the page the reading order puts first — still uses 'Table-39' (and 'Table-43') with no gloss and its seven-term primer omits it.
   - Applied: 'outward command past min(Table-39 &minus; 2&deg;, firmware warn)' -> 'outward command past min(Kinova user-guide Table&nbsp;39 range &minus; 2&deg;, firmware warn)'
13. **pedagogy#8 / consistency#12 residual (never-defined terms: Pinocchio, protobuf, URDF expansion)** — File A: first Pinocchio use (line 874 eq-label), protobuf (line 1228), vocabulary FK row (line 468); DH is now defined in both files — done
   - Of the confirmed never-defined terms, only DH was fixed. Pinocchio is used 7+ times in A with no introduction, protobuf appears once bare, and URDF is glossed but never expanded in either file.
   - Applied: Line 874: 'all read from the URDF via Pinocchio' -> 'all read from the URDF via Pinocchio (the rigid-body kinematics library used here for FK and Jacobians)'. Line 1228: 'left at protobuf default 0' -> 'left at protobuf (the Kortex message-serialisation format) default 0'. Vocab FK row: '(the URDF f
14. **visual#11 residual (B accessibility sub-parts)** — File B safety table rowspan cells (lines 434, 439, 441, 443) and section 4 cards (h4 elements under an h2, lines 463+)
   - B's column scopes and the h3-before-h2 fix were applied, but the rowspan'd phase cells are still <td> (invisible as row-group headers to screen readers) and the repo-map cards still jump h2 -> h4 in the heading outline.
   - Applied: Change each '<td rowspan="N">' phase cell to '<th rowspan="N" scope="rowgroup">' (add a 'th[scope=rowgroup]' CSS rule mirroring the current td styling so the look is unchanged), and retag the card '<h4>'/'</h4>' elements as '<h3 class="cardtitle">' (move the .card h4 CSS to the new selector).

## Open items (not closable by editing documentation)

Tracked live in the Command Path artifact's §12 "Open issues" table with
evidence and owners. Summary: commit the 2026-08-13 one-decision working
tree and rebuild; add the velocity-clip echo to WriteConfigLines; script the
vicon-EE-vs-composed-chain comparison (the genuinely independent hold
evidence); add the torso segment to the Nexus subject; fix the stale repo
comments the review exposed (goal.yaml box-frame text, the "observe/log
only" family, the --box usage text); and the hardware observations that
would upgrade [code]-verified claims to [hw] (pre-engage joint-hold
behaviour, the (1+Kd)·v_B/Kp steady-state line).

Totals: 89 confirmed findings (all fixed), 6 refuted, 14
compliance residuals (all fixed), plus the open items above.
