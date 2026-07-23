# Runtime configuration: CLI flags + TOML file (2026-07-23)

Status: accepted (approved as migration decision 11 / flag F2, 2026-07-22).
Supersedes the "no command-line flags" rule (old `Config.h` header,
`README.md`) and revisits `motion-txt-removal.md`'s removal of runtime
configuration files.

## Decision

`./controller` accepts runtime configuration with precedence
**CLI > TOML > compiled defaults** (`src/app/Options.{h,cpp}`):

- CLI: `--controller <name>`, `--kp <v>`, `--log <file>`, `--config <path>`.
- TOML (toml++ v3.4.0, vendored single header,
  `basic_control/third_party/tomlplusplus/`): gains and thresholds only —
  `kp`, `dls_lambda`, `following_error_limit_deg`, `arrival_tolerance_m`,
  `nonfinite_stop_cycles`, `saturation_stop_cycles`, `overrun_stop_cycles`,
  `overrun_factor`.

## Safety boundaries (the conditions under which the old rule was lifted)

- **No auto-discovery.** A TOML file is read only from an explicit
  `--config <path>`. This is the guard against `motion-txt-removal.md`'s
  original hazard: program behavior must never depend on a file that
  happens to be lying in the working directory.
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
