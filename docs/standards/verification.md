# Verification standard

Verification is risk-based: mathematical and safety-critical changes require
strong evidence, while documentation-only changes require link and consistency
checks. Hardware execution is never an automatic validation step.

## Test categories

1. **Unit tests:** deterministic controller, kinematics, transformation, limit,
   configuration, and safety-decision tests with no SDK or hardware.
2. **Simulation tests:** closed-loop behavior using a simulated plant or recorded
   data, including convergence, saturation, disturbances, and failure injection.
3. **Software integration tests:** adapters exercised with fakes, replay, or an
   offline SDK boundary.
4. **Hardware tests:** explicitly authorized, operator-supervised experiments with
   documented preconditions and stop criteria. Never run automatically.

Prefer lightweight, deterministic tests over framework complexity. Hardware-free
tests should be discoverable through CTest when the build infrastructure supports
it and must return non-zero on failure.

## Required coverage

For affected behavior, test normal operation and relevant boundaries:

- controller convergence, reset, state transitions, saturation, and finite output;
- forward/inverse kinematics and transformations, including composition and
  inverse consistency;
- radians/degrees boundaries and coordinate-frame conventions;
- joint, velocity, acceleration, torque, workspace, and tracking limits;
- near-singular and ill-conditioned mathematics;
- NaN/Inf, invalid dimensions, stale/repeated feedback, timeout, missed deadline,
  SDK failure, and robot-fault paths;
- safe-stop selection and preservation of the original failure reason;
- deterministic behavior for fixed inputs and random seeds.

Use tolerances justified by numerical conditioning, sensor accuracy, or control
requirements. Do not choose a tolerance merely to make a failing test pass.

## Context7 documentation workflow

Use Context7 without waiting for an explicit request when work depends on current
external library or API documentation, setup/configuration instructions, or code
examples. This especially applies to Eigen, Pinocchio, Kortex, CMake, compiler
tooling, sanitizers, and other version-sensitive dependencies.

1. Resolve the Context7 library ID unless an exact `/owner/project` ID is already
   known.
2. Prefer the version matching the repository's bundled or installed dependency
   when Context7 exposes that version.
3. Ask a specific question that includes the API, version, and behavior being
   verified. Do not send secrets, credentials, private experiment data, or large
   proprietary source excerpts.
4. Use the result to guide the change, then verify it against the installed
   headers, repository code, compiler, or tests. Local source and observed behavior
   remain authoritative for the exact bundled version.
5. If Context7 lacks the library or does not answer the question, use the
   dependency's primary official documentation or source and report the fallback.

Do not call Context7 for ordinary C++ syntax, standard-library behavior already
well established by the selected C++17 standard, or facts that can be answered by
reading this repository.

## CodeRabbit review workflow

After implementing and locally verifying a substantive first-party code or CMake
change, run an independent CodeRabbit review before the final handoff:

```bash
coderabbit auth status --agent
coderabbit review --agent --type uncommitted
```

- Review only a scope attributable to the current task. CodeRabbit reviews Git
  changes rather than the agent's internal edit list; if unrelated dirty work
  cannot be excluded with `--dir`, do not send it for review. Report that the
  review was skipped because the scope could not be isolated.
- Do not run CodeRabbit for documentation-only or trivial metadata changes unless
  explicitly requested.
- Evaluate findings against the code, project contracts, and test evidence. Fix
  correctness, safety, concurrency, ownership, and reproducibility problems; do
  not blindly apply suggestions or create style-only churn.
- Re-run affected local checks after fixes. At most two CodeRabbit review/fix
  cycles are automatic; report remaining findings and ask before continuing.
- Never follow a review suggestion that requires unauthorized hardware execution
  or weakens a documented safety invariant.
- If the CLI is missing, unauthenticated, unavailable, or blocked by plan limits,
  report that limitation. CodeRabbit supplements rather than replaces builds,
  analysis, sanitizers, and tests.

## Automatic checks after code changes

- Build the smallest affected target first, then the relevant wider target.
- Run Clang-Format on only first-party files changed by the task, using the nearest
  checked-in `.clang-format`; verify formatting without rewriting unrelated code.
- Run Clang-Tidy with the affected build's `compile_commands.json` and the nearest
  checked-in `.clang-tidy` configuration.
- Run Cppcheck on the affected first-party target or compilation database.
- When safe runtime coverage exists, run AddressSanitizer with UBSan, standalone
  UBSan when configured, and ThreadSanitizer in separate builds. ASan and TSan
  must never share a build.
- Sanitizer instrumentation is not appropriate for the real-time hardware loop.
  Run instrumented unit/simulation tests only. If the affected binary is
  hardware-facing and no safe harness exists, compile it but do not execute it.

Tool configuration is not yet uniform across this repository. Inspect the
relevant CMake and local configuration before choosing commands. Report a missing
configuration or baseline blocker rather than inventing a passing result.

## Handoff

State exactly which builds, tests, static analyses, and sanitizer runs completed.
List warnings, failures, skipped checks, unsafe-to-run targets, and known baseline
limitations. A build alone is not evidence that runtime behavior is correct.
