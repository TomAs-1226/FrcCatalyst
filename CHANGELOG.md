# Changelog

All notable changes to FrcCatalyst are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [0.3.5-beta] — 2026-05-18

### Fixed
- **`ClawMechanism` followers are no longer capped at one.** The Config builder now appends rather than overwriting, so calling `.follower(canId, oppose)` repeatedly attaches multiple followers as advertised. Per-follower OverCurrent / HighTemp health checks register automatically.
- **`FlywheelMechanism` gained follower paths.** Use `.primaryFollower(canId, oppose)` for single-wheel shooters with two or more motors ganged on one shaft, and `.secondaryFollower(...)` when running independent top/bottom wheels each with their own followers. The independent `.secondMotor(...)` API is unchanged.
- Both fixes credited to **avrahamavraham** for opening the issue on Chief Delphi.

### Changed
- **`DifferentialWristMechanism` now uses Phoenix-6 native differential control.** Internally the left motor is the master and runs `DifferentialMotionMagicVoltage`; the right motor is in `DifferentialFollower` mode and is wired via `DifferentialSensors.RemoteTalonFX_HalfDiff`. Both targets ship in one CAN frame and the firmware keeps them coordinated — replaces the previous "two independent Motion Magic loops" pattern. Thanks to **tcrvo** for the suggestion.
  - Pitch axis tunes through the existing `.pid(...) / .feedforward(...)` builder methods (Slot 0).
  - Roll axis can now be tuned independently via `.differentialPid(...) / .differentialFeedforward(...)` (Slot 1) — defaults to the same gains as pitch when not specified.
  - Slot 1 gains are live-tunable under `/Catalyst/Tuning/<Name>/Diff/...` alongside the existing Slot 0 tunables.

### Added
- **`RobotSafety` watchdog** in `frc.lib.catalyst.util` — opt-in cross-mechanism trip. Configure with `RobotSafety.configure(...)`, supplies `isTripped()`, `reason()`, manual `reset()`, optional auto-reset, and `onTrip` / `onReset` callbacks. Driven each loop by `HealthMonitor.tick(errorCount, warnCount)`; when no config is installed it's a zero-cost no-op. Publishes to `/Catalyst/Safety/{Tripped,Reason,ErrorCount,WarnCount}`.
- **Catalyst Builder UI** at `docs/tools/builder/index.html` — single-file dark-themed form that generates ready-to-paste mechanism config code (`LinearMechanism.Config.builder()...build()` snippets) for every Catalyst mechanism. Multi-follower fields, motor-type dropdown, copy-to-clipboard. Credits tcrvo / yteam3211's original [FRC Catalyst Subsystem Generator](https://yteam3211.github.io/frc-catalyst-subsystem-generator) for the design idea.
- **More `MotorType` presets**:
  - `KRAKEN_X44`, `KRAKEN_X44_FOC` (shipped earlier in 0.3.3 but worth restating)
  - `NEO`, `NEO_VORTEX`, `NEO_550` — REV motors for teams running a mixed stack (sim + physics only; not Phoenix-controllable)
  - `MINION` — the WCP Minion
- **Hot-reload for Slot 1** — `CatalystMotor.updateSlot1(kP, kI, kD, kS, kV, kA)` for any mechanism using a differential control mode.

### Docs
- Added an **Acknowledgements** section to the README crediting outside contributors (currently tcrvo / yteam3211 and avrahamavraham). Removed scattered team-name name-drops elsewhere in the docs and source comments in favour of generic "successful teams" / "in-house" language.
- Bumped install snippets and the AdvantageScope tab bundle versions to 0.3.5.

## [0.3.3-beta] — 2026-05-14

### Fixed — IMPORTANT, READ THIS
- **`MotorType` FOC variants had the same stall torque as their non-FOC counterparts.** Phoenix 6 FOC delivers about 30% more stall torque, so any team using `MotorType.KRAKEN_X60_FOC` or `MotorType.FALCON_500_FOC` was getting wrong numbers out of `holdingVoltage(...)`, `maxMechanismTorque(...)`, `getDCMotor(...)`, and `MotionConstraintCalculator`. Concrete effect: gravity feedforward voltages were over-stated by ~30%, and sim models under-reported torque. Re-check any hand-tuned `kG` values after upgrading.
  - `KRAKEN_X60_FOC`: stall torque 7.09 → **9.37 Nm**, free speed 6000 → **5800 RPM**, stall current 366 → **483 A**
  - `FALCON_500_FOC`: stall torque 4.69 → **5.84 Nm**, stall current 257 → **304 A**, free speed → **6080 RPM**
  - Values now match CTRE's published specs and WPILib 2026's `DCMotor.getKrakenX60Foc()` / `DCMotor.getFalcon500Foc()`.

### Changed
- **`MotorType` is no longer an `enum`** — it's a regular `final class` with the same `public static final` constants (`MotorType.KRAKEN_X60` etc.) so existing code keeps compiling unchanged. The change unlocks user-declared motor specs, which teams running NEO, NEO Vortex, Minion, or anything else Catalyst doesn't ship a preset for can now use directly: `new MotorType("NEO 550", 0.97, 11000, 100, 1.4)`.

### Added
- **`MotorType.KRAKEN_X44`** and **`MotorType.KRAKEN_X44_FOC`** presets — previously missing. Specs sourced from CTRE: 4.05 / 5.45 Nm stall torque, 7530 / 7200 RPM free, 275 / 366 A stall current.
- **`CatalystMath`** gained FOC and X44 constants (`KRAKEN_X60_FOC_STALL_TORQUE`, `KRAKEN_X44_STALL_TORQUE`, `FALCON_FOC_STALL_TORQUE`, etc.). The existing non-FOC constants are unchanged.
- **Health Kit** under `frc.lib.catalyst.util`:
  - `HealthCheck` — a single debounced fault condition with `Severity` (INFO/WARN/ERROR), a `BooleanSupplier` predicate, optional live `detail` string, `debounce(seconds)`, `clearAfter(seconds)`, and `onFire`/`onClear` hooks. Built via a fluent `HealthCheck.builder(subsystem, id)` and registered with one `.register()` call.
  - `HealthMonitor` — singleton registry that ticks every check once per loop (throttled to 5 ms, so all eight built-in mechanisms calling it cost one evaluation per scheduler tick). Publishes per-check state to `Catalyst/Health/<subsystem>/<id>/{firing,severity,description,detail,firedAt}` and rollup counts to `Catalyst/Health/{ErrorCount,WarnCount,InfoCount,Healthy}`. Every fire/clear edge is relayed to the existing `AlertManager` so dashboards already wired against it keep working.
  - `HealthMonitor.standardMotorChecks(...)` — one call registers OverCurrent (WARN at 90% of stator limit, debounce 0.5 s), HighTemp (WARN, debounce 1.0 s, clearAfter 5.0 s), and OverTemp (ERROR at warn+10 °C, immediate fire, auto-calls `motor.stop()`).
- **Health checks wired into every built-in mechanism** — Linear/Rotational add Stall + NotZeroed, Flywheel adds NotSpinningUp, Pneumatic adds LowPressure (with `requirePressureAbove(psi)` gating actuation). Multi-motor mechanisms (Flywheel, Winch, Claw follower, DifferentialWrist) register per-motor checks with collision-free id suffixes.
- **Health Dashboard** at `docs/tools/health/index.html` — single-file dark-themed web viewer that connects to NT4 read-only, shows per-subsystem cards with severity-colored firing checks, filter buttons (All / Firing only / Errors only), and a search box.

### Notes
- HealthCheck predicates and detail suppliers are wrapped in try/catch — a buggy lambda from team code won't take down the whole monitor.
- All thresholds use honest seconds-based debounce/clearAfter semantics rather than loop-cycle counts, so behavior is unaffected by scheduler period.
- Backward-compatible: existing `AlertManager.error/warning/info` calls in team code keep working; the built-in mechanisms just go through HealthCheck now.

## [0.3.2-beta] — 2026-05-14

### Added
- **Live-tunable PID + Motion Magic gains by default** on every closed-loop mechanism (`LinearMechanism`, `RotationalMechanism`, `FlywheelMechanism`, `DifferentialWristMechanism`). All Slot-0 gains (`kP`, `kI`, `kD`, `kS`, `kV`, `kA`, `kG`) and Motion Magic profile constants (`CruiseVelocity`, `Acceleration`, `Jerk`) are published under `Catalyst/Tuning/<MechanismName>/...` on NetworkTables. Edit from any dashboard, change applies on the next loop. Zero user code required.
- `CatalystMotor.updateSlot0(p, i, d, s, v, a, g)` and `CatalystMotor.updateMotionMagic(cruise, accel, jerk)` — public hot-reload methods used by the new tunable wiring. Gravity model from initial builder config is preserved.
- `frc.lib.catalyst.util.TunableGains` — bundles every gain into one helper and only re-applies when a value has actually changed (cheap to call every periodic).
- `docs/advanced/tuning.md` — full guide including the recommended competition lock-down pattern.

### Notes
- Default behavior is unchanged at robot init: gains start at whatever you put in the Config builder.
- Call `TunableNumber.disableTuning()` once in `robotInit()` for competition builds. After that, `hasChanged()` always returns false, no NT reads happen, and the motor configurator is never touched. Tuning is essentially free at runtime once disabled.
- No new dependencies. No API breaks.

## [0.3.1-beta] — 2026-05-14

### Fixed
- `ClawMechanism.hasPiece()` now OR-combines beam-break and stall detection. Previously, configuring a beam-break sensor would short-circuit the method and make the stall-current latch unreachable, even though the builder docstring describes beam-break as an "alternative / additional" signal.

### Added
- `PneumaticMechanism.timeInState()` — seconds since the last forward/reverse/off transition. Lets teams sequence actions with `Commands.waitUntil(() -> piston.timeInState() > 0.25)` instead of hand-rolling timers.
- `PneumaticMechanism.getTransitionCount()` — public getter for the transition counter already tracked internally.

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
