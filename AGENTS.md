# HumanSL repository instructions

These are the canonical repository-wide instructions for coding agents. Claude
Code imports this file through `CLAUDE.md`; other agents read it directly.

## Engineering priorities

This is C++17 research robotics software for Kinova Gen3 systems using Eigen,
Pinocchio, the Kortex SDK, and CMake. In descending order, prioritize:

1. human and hardware safety;
2. experimental and mathematical correctness;
3. reproducibility and observable failure;
4. mathematical readability and maintainability;
5. development speed and convenience.

Prefer direct research code over enterprise architecture. Add an abstraction
only when it clarifies mathematics, owns state, or isolates a real external
boundary.

## Audit before acting

- Inspect the working tree, relevant documentation, build files, and current
  implementation before editing, building, or installing anything.
- Read every file before modifying it. Preserve unrelated and in-progress work.
- Prefer installed tools and bundled dependencies. Install only after confirming
  that a required tool is missing.
- Treat `third_party/` as vendored code. Do not edit or reformat it unless the
  task explicitly requires a vendor change.

## Mandatory guidance by task

Before making the corresponding change, read and follow:

| Work | Required guidance |
|---|---|
| First-party C or C++ | `docs/standards/cpp.md` |
| Robot control, Kortex, Vicon, timing, or hardware I/O | `docs/standards/control-safety.md` and `docs/robotics-contracts.md` |
| C++, CMake, or tests | `docs/standards/verification.md` |
| Module boundaries or dependencies | `docs/architecture.md` |
| External library or API behavior | Context7 workflow in `docs/standards/verification.md` |
| Completion of substantive code changes | CodeRabbit workflow in `docs/standards/verification.md` |

Nested `AGENTS.md` files add subsystem facts and constraints. They supplement
these rules and must not duplicate or weaken them.

Use Context7 automatically for version-sensitive external library/API guidance,
setup, configuration, and code examples. Use CodeRabbit as an independent review
gate for substantive code changes when its review can be limited to the task's
changes. Neither tool replaces repository inspection, primary project evidence,
builds, static analysis, tests, or hardware-safety judgment.

## Hardware execution

- Never connect to, command, or run against physical robot or Vicon hardware
  without explicit authorization for that run.
- Hardware-moving tests require Christian present with the workspace clear and
  the emergency stop immediately available.
- Treat root `main`, `Christian_control/basic_control/controller`,
  `test_kinova`, `test_task_impedance`, and any unaudited Kortex-linked binary as
  hardware-facing. Building is allowed; execution is not an incidental test.
- Do not assume a binary is safe because its intended operation is read-only.
  Audit its startup and teardown paths first.

## Change discipline

- Keep changes proportional to the task. Do not combine a feature or bug fix
  with unrelated cleanup, renaming, or architectural migration.
- Apply these standards to new and modified code. Legacy violations are not
  precedent, but they also do not authorize a mass rewrite.
- Make state, units, coordinate frames, ownership, and failure handling explicit.
- Do not hide controller behavior in `main.cpp`, logging, adapters, or callbacks.
- Update stable documentation when architecture, safety behavior, interfaces, or
  experiment semantics change. Put decisions in `docs/decisions/` or the
  relevant subsystem decision directory, not in instruction files.
- Report what was tested, what was not safe or possible to test, and every known
  warning or failure. Never claim a check passed unless it ran successfully.

