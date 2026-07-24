# Runtime configuration: CLI flags + TOML file (2026-07-23)

Status: accepted (approved as migration decision 11 / flag F2, 2026-07-22).
Amended 2026-07-24: default config file + controller/target_file TOML keys
(below). Supersedes the "no command-line flags" rule (old `Config.h` header,
`README.md`) and revisits `motion-txt-removal.md`'s removal of runtime
configuration files.

## Decision

`./controller` accepts runtime configuration with precedence
**CLI > TOML > compiled defaults** (`src/app/Options.{h,cpp}`):

- CLI: `--controller <name>`, `--kp <v>`, `--log <file>`, `--config <path>`.
- TOML (toml++ v3.4.0, vendored single header,
  `basic_control/third_party/tomlplusplus/`): gains, thresholds, controller
  selection and input sources —
  `controller`, `kp`, `dls_lambda`, `following_error_limit_deg`,
  `arrival_tolerance_m`, `nonfinite_stop_cycles`, `saturation_stop_cycles`,
  `overrun_stop_cycles`, `overrun_factor`; reactive-pose only: `kp_rot`,
  `kd_pos`, `kd_rot`, `null_gain`, `orientation_enabled`,
  `velocity_term_enabled`, `null_space_enabled`, `target_file`.

## Default config file (amendment, 2026-07-24)

Without `--config`, the compiled default file
`basic_control/config/control.toml` is loaded when it exists — an absolute
path baked in at build time (`DEFAULT_CONFIG_PATH`, CMake, same mechanism
as `GEN3_URDF_PATH`). The workflow this buys: edit one checked-in file, run
the bare binary. The no-auto-discovery hazard (below) was behavior
depending on files *lying in the working directory*; a single fixed
absolute path does not reintroduce it — the file's identity never depends
on where the program is started, `--config` still overrides it, and the
startup echo + CSV preamble name which file was loaded (`config_file`
line). `controller` became a TOML key with the same amendment, so the file
can select the law; `--controller` still wins.

## Safety boundaries (the conditions under which the old rule was lifted)

- **No working-directory discovery.** A TOML file is read only from an
  explicit `--config <path>` or the single compiled default path above.
  This is the guard against `motion-txt-removal.md`'s original hazard:
  program behavior must never depend on a file that happens to be lying in
  the working directory.
- **No safety keys.** `stop_on_fault` is compile-time only
  (`config::kStopOnFault`, F2) and is an explicit ERROR as a TOML key.
  Connection parameters and the speed-clip derivation stay compiled.
- **Typos are hard errors.** Unknown CLI flags and unknown TOML keys exit
  with a message; nothing silently falls back to a default.
- **Every run is self-describing.** The full effective config, each value
  tagged with its source (compiled/toml/cli), is echoed at startup and
  written as `#`-prefixed preamble lines above the CSV header (F3). The
  plot scripts refuse old-format files without the preamble unless
  `--allow-old` is passed; `replay_controller` accepts them with a note
  (the step-4 baseline predates the preamble).
