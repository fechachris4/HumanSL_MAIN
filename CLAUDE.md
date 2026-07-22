# Hardware safety (the only standing rule)

Binaries in this repository command a physical Kinova Gen3 arm.

- Never execute `Christian_control/basic_control/controller`, root `main`,
  `test_kinova`, `test_task_impedance`, or any Kortex-linked binary without
  Christian's explicit authorization for that specific run. Building is
  fine; running is never a test step.
- Hardware runs require Christian present, workspace clear, and the
  emergency stop immediately available.
