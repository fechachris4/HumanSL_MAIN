# C++ standard

This standard applies to new and modified first-party C++ code. Improve legacy
code locally when it supports the task; do not start unrelated mass migrations.

## Language and naming

- Use standard C++17. Do not introduce C++20 features or compiler-specific
  extensions without an explicit, documented need.
- Use the repository's `.cpp` and `.h` extensions. Headers must be self-contained.
- Use this Google-style naming subset:
  - types and type aliases: `PascalCase`;
  - functions and methods: `PascalCase`;
  - variables and struct fields: `snake_case`;
  - constants and enumerators: `kPascalCase`;
  - class data members: `snake_case_`;
  - namespaces and filenames: lowercase with underscores;
  - macros, only when unavoidable: `HUMANSL_UPPER_SNAKE_CASE`.
- Prefer names that expose physical meaning, such as `velocity_rad_s`, over short
  generic names outside compact mathematical derivations.
- Do not rename an unrelated legacy API merely to satisfy naming style.

## Values, ownership, and dependencies

- Prefer values. Use `const T&` for required borrowed inputs and `T&` only when a
  function intentionally mutates the object.
- Use a pointer for a borrowed dependency only when null is meaningful or an
  external SDK requires pointer syntax. A raw pointer is always non-owning and
  must never be deleted by the borrower.
- Use `std::unique_ptr` for dynamic ownership. Use `std::shared_ptr` only when the
  design genuinely has multiple independent owners; document why shared lifetime
  is required. Never use a raw owning pointer.
- Use constructor injection for major external dependencies such as robot,
  recorder, or clock adapters. Pass ordinary mathematical inputs directly.
- Prefer concrete controller, kinematics, and dynamics classes. Introduce an
  interface only at an external boundary, for multiple real implementations, or
  for a test seam with demonstrated value.
- Do not add dependency-injection containers, service locators, generic manager
  hierarchies, or factories for a single implementation.

## State and failure handling

- Give each mutable state one clear owner. Keep it private and expose explicit
  update, reset, or snapshot operations.
- Avoid global mutable state. A process-wide atomic stop signal is an acceptable
  narrow exception.
- Make controller initialization and reset behavior explicit and test it. Do not
  hide persistent state in function-local statics or unrelated adapters.
- Exceptions are acceptable during startup and configuration. Controller and
  real-time code returns explicit status/results; exceptions must not propagate
  through a critical control loop.
- Validate inputs at external boundaries and validate safety-critical invariants
  before use. Avoid repetitive defensive checks for internal states already
  guaranteed by construction.

## Eigen and robotics mathematics

- Use fixed-size Eigen types when the dimension is a Kinova invariant: seven
  physical joints, six-dimensional twists/wrenches, 3D vectors and rotations,
  and homogeneous transforms.
- Centralize aliases such as `JointVector`, `JointMatrix`, `Vector6d`, and
  `Matrix6d`; do not repeat literal dimensions throughout controller code.
- Dynamic Eigen types are appropriate when dimensions genuinely come from a
  Pinocchio model (`nq` and `nv` need not equal seven), an optimization horizon,
  a planner problem, or variable external data.
- Do not use explicit matrix inversion when a decomposition or linear solve
  expresses the mathematics more accurately.
- Check dimensions and finite values at system boundaries. Do not silently resize
  a fixed physical quantity to accommodate malformed data.
- Follow the unit and coordinate-frame contracts in
  `docs/robotics-contracts.md`. Convert degrees only at a named boundary; internal
  angular calculations use radians.

## Readability

- Keep equations close to the notation used in the associated paper, derivation,
  or design document. Use intermediate variables when they expose mathematical
  meaning or coordinate frames.
- Comments explain assumptions, frames, units, invariants, numerical method, and
  non-obvious safety reasoning. Do not narrate straightforward syntax.
- Replace unexplained constants with named parameters and cite the physical,
  experimental, SDK, or literature source when it matters.
- Keep `main.cpp` at the wiring and orchestration level. Controller mathematics,
  Kortex mechanics, transformation math, and recording formats belong in their
  technical modules.
