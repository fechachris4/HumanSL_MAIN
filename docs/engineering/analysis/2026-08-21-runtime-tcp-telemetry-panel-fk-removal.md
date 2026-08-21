# Runtime TCP Telemetry and Browser-FK Removal Analysis

Status: accepted for implementation on 2026-08-21.

## Physical objective

The panel must display end-effector positions produced by the same canonical
Pinocchio model as the controller. It must not independently reconstruct robot
geometry from browser-side DH or mounting constants. This is diagnostic only:
it changes no reference, controller law, limit, command, stop, or Kortex path.

## Frames, units, and state ownership

Use `A_T_B` for the pose of `B` expressed in `A`. `M` is the model `mount`
root; `W` is Vicon world; `E` is the controlled TCP. Joint order is Kortex
actuator order 1..7. Joint input is degrees at the Kortex/log boundary and
radians inside control. Positions are metres; rotations are unit quaternions
on the CSV wire and matrices internally.

Let cycle `k` start at `t_cycle[k]`. `q_meas_input[k]` is copied from the
feedback returned by the previous successful exchange at `t_recv[k-1]` (for
the first normal cycle, the reply to the T5 seed send). `W_T_M[k]` is the
single wait-free world-slot sample read at `t_cycle[k]`. Therefore:

```text
M_T_E_meas[k] = canonical_FK(q_meas_input[k])
W_T_E_meas[k] = W_T_M[k] * M_T_E_meas[k]
```

For the integrated position frame offered to the exchange in cycle `k`:

```text
M_T_E_cmd_tx[k] = canonical_FK(q_cmd_tx[k])
```

`q_cmd_tx[k]` is the integrated, limited frame passed to
`CyclicSession::Send`. A CSV row is queued only after `Send` returns normally
at `t_recv[k]`, so the row proves that the exchange completed with that
transmitted frame. It does **not** prove actuator acceptance or physical
motion. `command_frame_id`, `feedback_frame_id`, and per-actuator
`command_ack_j*` remain separate downstream evidence. The same row's raw
`meas_j*` fields are from the new reply at `t_recv[k]`, while
`M_T_E_meas[k]` is from `q_meas_input[k]`; this existing one-exchange offset
is preserved and documented.

`TrackingController::Measure` already runs canonical FK and shall expose its
mount-frame position alongside the existing world pose; measured FK is not
run again. `ArmExecutionCore` shall expose a small telemetry-only method that
uses its existing `DualArmKinematics::ToolPoseInMount` on the already-stored
integrated command. Runner calls it only **after** `CyclicSession::Send`
returns and **after** `ResolveStop` returns `kNone` with no counter stop. Stop
cycles skip UI FK and proceed directly to their existing log/break/teardown
path. Therefore UI FK cannot delay command transmission, stop classification,
or a selected stop. JavaScript performs no joint-chain FK.

## System and motion decomposition

The physical chain remains:

```text
W_T_E = W_T_MSeg * MSeg_T_M * M_T_B * B_T_E(q)
```

The current implementation has no separate `MSeg_T_M` calibration and treats
it as identity. This task does not change that assumption. Mount-frame TCP
telemetry stops before the world attachment:

```text
M_T_E = M_T_B * B_T_E(q)
```

so measured and commanded markers can be drawn with the planner-owned static
scene, which is also expressed in `M`, without assuming `W = M`.

Joint motion changes `B_T_E(q)`. Mount motion changes `W_T_MSeg` but must not
move the mount-frame markers. Existing world measurement/reference and error
telemetry are unchanged and remain the controller-error authority.

## Error decomposition

- Model/TCP error: URDF frame and tool configuration affect both markers.
- Mount calibration error: does not affect mount-frame marker consistency,
  but affects existing world control through the current `MSeg_T_M = I`
  assumption.
- Joint measurement error: affects `M_T_E_meas`.
- Integrated-command versus physical response: separates
  `M_T_E_cmd` from `M_T_E_meas`; it is not itself a controller error metric.
- Timing: the two markers describe different pipeline states in the same log
  row; neither is the post-send reply pose.
- Panel rendering/projection error remains visual only.

The panel shall not compute a new Cartesian error from these markers. The
existing controller world error `||p_desired_world - p_current_world||`
remains authoritative.

## Time decomposition and real-time impact

Vicon remains asynchronous (~100 Hz), control remains ~500 Hz, and logging
remains asynchronous. The new commanded position is computed after the
successful exchange and stop classification, before the asynchronous log-row
publish. It adds one pose-only call to the remaining cycle headroom but is not
on the command-send or safety-decision critical path. It adds no I/O, lock,
sleep, dynamic configuration, or planner work. Allocation freedom must be
checked hardware-free. Existing `time_s`, `t_send_s`, and `t_recv_s` semantics
remain unchanged.

The CSV schema advances from format 13 to format 14 and adds only six
mount-frame XYZ columns: measured TCP and transmitted-command TCP. Takeover
rows use NaN for these
uncomputed quantities. Readers continue selecting columns by name; older logs
lack the new fields and therefore show no TCP markers rather than fabricated
zeros or browser-FK fallbacks.

## Limiting cases

1. Identity `W_T_M`: mount and world measured poses coincide.
2. Moving/rotating Mount with fixed joints: mount poses remain fixed while
   existing world poses move.
3. `q_cmd_tx == q_meas_input`: measured and commanded markers coincide.
4. Following lag: markers separate, with no new severity/error judgement.
5. Missing/non-finite columns or old replay: omit the affected marker.
6. Takeover before canonical Cartesian computation: both new poses are NaN.
7. Right/left controller processes: each publishes only its controlled arm;
   the panel combines the two streams without assuming the other arm pose.
8. Typed goal preview: only mount-frame requests are drawable in this slice;
   base/world requests are refused with the missing-transform reason rather
   than reviving browser model transforms.

## Falsifiable predictions

1. A source scan finds no DH table, `jointTransform`, `forwardKinematics`,
   `DH_ROOT_ROLL_RAD`, or `setDh` in production browser code.
2. A frozen pre-change fixture for right-arm `[10,20,30,40,50,60,70]` degrees
   gives mount TCP position `[0.396290,-0.976259,-0.149543]` m from the
   existing offline `print_dual_arm_fk`, with RPY
   `[0.307514,-3.122600,0.525426]` rad and hemisphere-fixed quaternion xyzw
   `[-0.258031,0.953876,-0.150327,0.030710]`. The new measured and command
   paths reproduce both pose halves when given those joints. This fixture
   predates the implementation and catches transform direction, TCP-frame,
   rotation, and quaternion-order mistakes.
3. The format-14 header and row have equal width; new pose fields round-trip;
   takeover defaults are NaN.
4. Existing world measurement/reference/error columns are byte-semantically
   unchanged apart from their later numeric column positions.
5. Panel tests show markers only from `measured_tcp_*_mount_m` and
   `commanded_tcp_*_mount_m`; missing fields draw no marker.
6. The frozen pre-change hardware-free benchmark of 10,000 complete
   `Step + ResolveStop` calls is mean 2.289 us and p99 2.315 us on this
   workstation. The post-change benchmark must run the actual no-stop
   production tail `Step + ResolveStop + CommandedTcpMount`, keep total p99
   below 1.0 ms (half the 2 ms cycle), and increase p99 by less than 100 us,
   with zero heap allocations in the added pose path. A separate stop-path
   test must prove the telemetry FK method is not called after any selected
   stop. The full runtime/control build must pass. This is an offline
   compute-headroom gate, not hardware timing evidence. The control binary is
   compiled but never executed.

## Safety and behaviour classification

Classification: telemetry/visualisation ownership change with a Level-2
real-time timing implication. It does not alter requested velocity, clamp,
integration, command values, send order, safety facts, stop priority, fault
handling, takeover, or teardown. No new runtime guard is added. Hardware
timing and physical correctness remain unverified until a separately
authorised supervised run.

## Code-shape and file-touch proposal

- `control/State.h`, `Controller.cpp`: expose measured mount pose already
  produced by canonical FK.
- `control/ExecutionCore.h/.cpp`: expose one post-send telemetry query through
  the existing canonical kinematics object; it does not enter `Step`.
- `runtime/Runner.cpp`, `Hardware.h/.cpp`, `runtime/README.md`: copy and
  serialize explicit format-14 mount poses.
- One focused hardware-free core test and one log-schema test.
- `panel/static/scene.js`, `panel.js`, `index.html`, `scene.test.html` and
  panel tests: replace articulated-arm DH rendering with TCP markers.
- `panel/server.py`: remove the `/api/dh` browser endpoint. Planner DH
  freshness diagnostics remain server side. Mount-frame obstacles and TCP
  markers require no browser model transform; non-mount typed previews are
  explicitly unavailable in this slice.

Concepts removed: browser DH parsing input, browser joint-chain FK, browser
mount-plus-DH composition, articulated-arm scene claims. Concepts added:
explicit measured/commanded mount-frame TCP telemetry. No manager, registry,
factory, service, or plugin is introduced.

Production-line count must decrease overall. If implementation adds more
production lines than it removes, stop and explain before proceeding.

## Adversarial review

ACCEPTED after correction rounds and the user's post-send simplification. The
reviewers required and then accepted:
exact cross-exchange timestamps; transmitted-versus-acknowledged command
semantics; frozen position and orientation fixtures; a measured before/after
full-tail timing gate; stop-path exclusion; format-14 reader compatibility;
left/right frame evidence; zero-allocation evidence; and complete removal of
browser/server DH parsing used only by the retired renderer.
