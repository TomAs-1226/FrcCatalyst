# Changelog

All notable changes to FrcCatalyst are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [0.3.0-beta] — 2026-05-14

### Added
- **Multi-follower support** on `CatalystMotor` and `LinearMechanism`. Each `withFollower(canId, oppose)` call now appends a follower instead of overwriting the previous one. New `FollowerSpec` record + `withFollowers(FollowerSpec...)` varargs convenience.
- **In-house logging core** under `frc.lib.catalyst.logging`:
  - `CatalystLog` — static facade routing all mechanism telemetry through a single pluggable sink.
  - `LogSink` — interface with typed `log(...)` overloads plus `processInputs(...)`.
  - `NetworkTablesSink` — default sink. Preserves the v0.2 `/Catalyst/<name>/...` NetworkTables layout.
  - `CompoundSink` — fan-out to multiple sinks (e.g., NT + AK during a competition).
  - `CatalystInputs` — symmetric `toLog`/`fromLog` contract for Inputs POJOs.
  - `LogTable` — typed key/value table used for serialization.
- **AdvantageKit bridge documentation** at `docs/advanced/logging.md`. Catalyst itself takes no AK dependency; teams write a ~10-line `LogSink` to bridge.
- **IO + Inputs contract** under `frc.lib.catalyst.io`:
  - `LinearMechanismInputs` / `LinearMechanismIO`
  - `RotationalMechanismInputs` / `RotationalMechanismIO`
  - `RollerMechanismInputs` / `RollerMechanismIO`
  - `FlywheelMechanismInputs` / `FlywheelMechanismIO`
  - `WinchMechanismInputs` / `WinchMechanismIO`
  - `ClawMechanismInputs` / `ClawMechanismIO`
  - `DifferentialWristMechanismInputs` / `DifferentialWristMechanismIO`
  - `PneumaticMechanismInputs` / `PneumaticMechanismIO`
  - All five built-in mechanisms (plus the three new ones) populate their Inputs POJO each loop and ship it via `CatalystMechanism#processInputs`.
- **`ClawMechanism`** — motor-driven gripper with stall-current grip detection and a low passive hold voltage. Supports optional beam-break and follower motor.
- **`DifferentialWristMechanism`** — two-motor diffy wrist. Resolves `(pitch, roll) ↔ (left, right)` and commands both axes via Motion Magic with software pitch/roll limits and named presets.
- **`PneumaticMechanism`** — single/double solenoid wrapper with `extend()` / `retract()` / `toggle()` / `pulse(duration)` commands, optional pressure-aware actuation guard (`requirePressureAbove(psi)`), and cycle counting.
- **Forward-limit auto-zero** on `LinearMechanism` (mirrors the existing reverse-limit support).
- **Configurable tolerances** on `RotationalMechanism` via `tolerance(degrees)`; new tolerance support on `DifferentialWristMechanism`.

### Changed
- `CatalystMechanism` now routes all telemetry through `CatalystLog` instead of writing directly to NetworkTables. The `telemetryTable` field is preserved for backwards compatibility with v0.2 user code.
- `VisionSubsystem` now raises an `AlertManager` warning when constructed without a drive subsystem, instead of silently no-op'ing.

### Fixed
- `RotationalMechanism.atPosition(String)` no longer ignores the configured angular tolerance (used a hardcoded `2.0` degrees regardless of config).
- `LinearMechanism` reverse-limit auto-zero now seeds the encoder to `config.minPosition` instead of zero (was incorrect when `minPosition != 0`).
- `LinearMechanism` simulation motor count is now derived from the live follower count, so it can no longer drift out of sync when followers are added.

### Migration notes
- The default behavior is unchanged. Existing v0.2 robot code keeps working without modification.
- Teams wanting AdvantageKit telemetry can install a `LogSink` at robot init — no mechanism code changes required.

## [0.2.0-beta] — 2026 season

Initial public beta. See git history.
