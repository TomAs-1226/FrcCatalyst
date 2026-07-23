<p align="center">
  <img src="docs/assets/banner.svg" alt="FrcCatalyst Banner" width="100%"/>
</p>

<p align="center">
  <a href="https://github.com/TomAs-1226/FrcCatalyst/actions"><img src="https://img.shields.io/github/actions/workflow/status/TomAs-1226/FrcCatalyst/build.yml?style=for-the-badge&logo=github&label=Build" alt="Build Status"/></a>
  <a href="https://github.com/TomAs-1226/FrcCatalyst/releases"><img src="https://img.shields.io/github/v/release/TomAs-1226/FrcCatalyst?style=for-the-badge&logo=semanticrelease&color=e94560" alt="Release"/></a>
  <a href="https://github.com/TomAs-1226/FrcCatalyst/blob/main/LICENSE"><img src="https://img.shields.io/github/license/TomAs-1226/FrcCatalyst?style=for-the-badge&color=0f3460" alt="License"/></a>
  <a href="https://tomas-1226.github.io/FrcCatalyst/"><img src="https://img.shields.io/badge/Docs-GitHub%20Pages-blue?style=for-the-badge&logo=github" alt="Docs"/></a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/WPILib-2026.2.1-green?style=flat-square" alt="WPILib"/>
  <img src="https://img.shields.io/badge/Phoenix%206-26.1.1-orange?style=flat-square" alt="Phoenix 6"/>
  <img src="https://img.shields.io/badge/Java-17-blue?style=flat-square&logo=openjdk" alt="Java 17"/>
  <img src="https://img.shields.io/badge/PathPlanner-2026.1.2-purple?style=flat-square" alt="PathPlanner"/>
  <img src="https://img.shields.io/badge/PhotonVision-v2026.3.1-yellow?style=flat-square" alt="PhotonVision"/>
</p>

---

## What is FrcCatalyst?

**FrcCatalyst** is a plug-and-play Java library for FRC teams using **CTRE Phoenix 6 hardware**. It provides production-ready mechanism building blocks, hardware wrappers, and utilities so your team can focus on strategy and game-specific logic instead of writing boilerplate.

> **One import. One builder call. A fully functional mechanism with telemetry, simulation, safety limits, and command factories.**

### Why FrcCatalyst?

| Feature | Raw WPILib/Phoenix | FrcCatalyst |
|---------|-------------------|-------------|
| Elevator with gravity FF | ~150 lines | **8 lines** |
| Swerve + PathPlanner + Vision | ~400 lines | **15 lines** |
| Mechanism with sim + telemetry | Build it yourself | **Built-in** |
| Browser sim cockpit for any mechanism | Hand-write per robot | **Generic `SimDashboard`** |
| Safe temperature cutoffs | Manual | **Automatic** |
| Limit switch auto-zeroing | Manual wiring | **One builder call** |
| Whole-robot state machine | Hand-rolled per season | **Declarative graph + full logging** |

---

## Quick Start

### 1. Add the dependency

**Easiest:** in WPILib VS Code, run **Manage Vendor Libraries → Install new libraries (online)** and paste:

```
https://tomas-1226.github.io/FrcCatalyst/vendordep/FrcCatalyst.json
```

Make sure the **Phoenix 6**, **PathPlanner**, and **PhotonVision** vendordeps are installed too (Catalyst builds on them).

<details><summary>Or add it by hand in <code>build.gradle</code></summary>

```gradle
repositories {
    maven { url "https://jitpack.io" }
}

dependencies {
    implementation "com.github.TomAs-1226:FrcCatalyst:v1.2.1"
}
```
</details>

### 2. Build a mechanism in seconds

```java
// A full-featured elevator in ~10 lines
LinearMechanism elevator = new LinearMechanism(
    LinearMechanism.Config.builder()
        .name("Elevator")
        .motor(13)
        .follower(14, true)
        .motorType(MotorType.KRAKEN_X60)
        .gearRatio(10.0)
        .drumRadius(0.0254) // 1 inch
        .stages(2) // 2-stage cascade
        .range(0.0, 1.2) // meters
        .mass(5.0) // kg
        .pid(50, 0, 0.5)
        .gravityGain(0.35)
        .motionMagic(2.0, 4.0, 20.0)
        .currentLimit(40)
        .reverseLimitSwitch(0, true) // DIO 0, auto-zero
        .maxTemperature(70) // safety cutoff
        .position("STOW", 0.0)
        .position("INTAKE", 0.3)
        .position("AMP", 0.8)
        .position("HIGH", 1.1)
        .build()
);

// Use it
elevator.setDefaultCommand(elevator.holdPosition());
operatorController.a().onTrue(elevator.goTo("HIGH"));
operatorController.b().onTrue(elevator.goTo("STOW"));
```

---

## What's New in v1.2.1: shooter, sim, and logging plumbing

A maintenance release that clears the issue backlog and the ready PRs. All additive.

- **Torque-current FOC velocity control** for shooters ([#20](https://github.com/TomAs-1226/FrcCatalyst/pull/20)) —
  `FlywheelMechanism` torque-current mode with a per-loop amps-feedforward `track(...)`, closing the
  last [#17](https://github.com/TomAs-1226/FrcCatalyst/issues/17) gap.
- **Swerve sim opt-out** ([#19](https://github.com/TomAs-1226/FrcCatalyst/pull/19)) — the internal
  sim thread stands down when maple-sim takes over, plus `getModuleTargets()` as the clean
  mechanism-level bridge seam.
- **`AimingSolverVector`** ([#27](https://github.com/TomAs-1226/FrcCatalyst/pull/27)) — a second,
  vector-adding shoot-on-the-fly solver alongside the time-of-flight `AimingSolver`.
- **All telemetry routes through `CatalystLog`** ([#24](https://github.com/TomAs-1226/FrcCatalyst/issues/24)) —
  swap the sink once and every stream follows; existing NT keys unchanged.

Full notes: [CHANGELOG.md](CHANGELOG.md).

## What's New in v1.2.0: a real state machine for the whole robot

Resolves [#22](https://github.com/TomAs-1226/FrcCatalyst/issues/22). The new
`frc.lib.catalyst.statemachine` package replaces `SuperstructureCoordinator`, which only ever
understood `LinearMechanism` and `RotationalMechanism` positions and so could not describe a whole
robot. Backward compatible — the coordinator is deprecated but frozen, not removed, and existing
robot code keeps working unchanged.

- **Every mechanism type, plus yours.** All nine Catalyst mechanisms (elevator, arm, differential
  wrist, turret, flywheel, roller, claw, winch, pneumatic) have a typed binding. Any `SubsystemBase`
  you wrote yourself reaches the same fidelity through four escape-hatch tiers in `Mechanisms`:
  `instant` (fire and forget), `custom` (a method call plus a real arrival test), `commands` (you
  already expose `Command` factories), and `build` (units, tolerance, ranges, notes, re-assertion).
  The engine knows nothing about the typed bindings — it knows only `Actuator` — so a team's own
  swerve or climber is not second-class.
- **A transition graph, not a preset applier.** You declare which state changes are legal; an edge
  you did not declare is a request the robot refuses, with a reason, in the log. On top of that:
  guards and entry guards, global interlocks, per-edge staged actuation (raise the elevator *then*
  swing the arm), optional multi-hop routing, per-state and per-edge deadlines, and fault policies.
- **Arrival is proven, never assumed.** `current()` is only ever a state whose every gating
  mechanism was *measured* to be at its goal. This is the bug that made the old coordinator
  dangerous: it set the current state to the target even when a transition was interrupted or timed
  out, so the next transition planned its route from a state the robot was not in. Now a timeout
  leaves the machine where it actually is.
- **Logging you do not have to build.** A structured schema under `/Catalyst/<prefix>/` covering the
  graph, the live phase, progress and deadlines; a `Blocker` string that names *which* mechanism is
  holding things up; a 50-entry transition history with outcomes and per-mechanism arrival times;
  and one-shot Driver Station warnings, so you can diagnose a stuck superstructure with no dashboard
  at all.

Full guide: [docs/advanced/statemachine.md](docs/advanced/statemachine.md).

## What's New in v1.1.0: audit fixes from a full robot port

Resolves the 15-finding v1.0.0 audit ([#17](https://github.com/TomAs-1226/FrcCatalyst/issues/17)) from porting Team 5805's clone of team581's 2026 comp-bot onto Catalyst. Backward compatible.

- **Swerve simulates now.** `SwerveSubsystem` starts a 200 Hz sim thread, so the drivetrain actually moves in the simulator instead of freezing.
- **`CatalystGyro` stopped wiping the Pigeon 2 mount pose** (the default config apply is gone; opt in with the new config-taking constructor).
- **Correctness fixes**: `LinearMechanism.zero()` sets `hasBeenZeroed`; `pathfindToPose()` is lazy; `xBrake()`/`idle()` hold their requirement; PathPlanner path following is closed-loop and keeps its feedforwards; `configurePathPlanner()` fails loudly.
- **New API**: runtime current limits (`CatalystMotor.setSupplyCurrentLimit(...)`), current-spike homing (`mechanism.homeOnCurrent(...)`), `RotationalMechanism.hasBeenZeroed()`, `SwerveSubsystem.setMaxAngularRate(...)`.
- **Packaging**: the vendordep now lists the PhotonVision, PathPlanner, and Phoenix repos so transitives resolve; corrected the install coordinate and test-coverage wording.

## 🎉 v1.0.0 is here: out of beta

**FrcCatalyst is 1.0.0.** After a long beta run (v0.3 through v0.10) and four
release candidates, the library is stable, documented, and ready for the 2026
season. Every mechanism, the swerve stack, Shoot-On-The-Fly aiming, the behavior
framework, logging, and the generic `SimDashboard` are covered by a real unit
test suite and proven in the simulator and on a real robot.

Thank you to everyone who filed issues and PRs through the beta, especially
**@avrahamavraham** and the teams following along on Chief Delphi. This is your
release too.

1.0.0 is the same code as rc4, promoted after the release-candidate series held
up. Semantic versioning starts here: the public API is stable, and breaking
changes wait for 2.0.0. Go build something awesome. 🚀

## What's New in v1.0.0-rc4: community PRs, telemetry, swerve, sim, vision

- **Merged 7 community PRs** (thanks @avrahamavraham): red-alliance field-centric fix, wheel-radius-based max angular rate, an `idle()` command + ChassisSpeeds publisher, struct logging, a logging enable/disable toggle, and custom `WpilogSink` destinations.
- **Struct + struct-array logging** end to end: `CatalystLog.log(key, struct, value)` and `(key, struct, array)` publish any WPILib struct type (poses, module states) as real objects in AdvantageScope, through both the NetworkTables and WPILOG sinks.
- **Swerve module telemetry**: measured and target `SwerveModuleState[]` publish to `/Catalyst/Swerve/ModuleStates` and `/ModuleTargets` for the AdvantageScope swerve view.
- **SimDashboard v2**: per-mechanism sparkline history, a pause/resume toggle, and CSV export of the live snapshot.
- **`SimCameraSource`**: a vision source that emits noisy, latency-delayed pose estimates from a simulated pose, so the multi-camera fusion pipeline runs in the simulator with no hardware.
- **WPILib 2027**: a `wpilib-2027` branch with a grounded migration plan (`MIGRATION_2027.md`). Main stays on WPILib 2026.

## What's New in v1.0.0-rc3: Configurable simulation

- **`SimDashboard`** (`frc.lib.catalyst.sim.SimDashboard`): a generic, dependency-free simulation cockpit. Register any mechanism with `dash.add(...)` and it renders a fitting live widget from that mechanism's `describe()`, plus opt-in `.button()` / `.command()` / `.slider()` / `.toggle()` controls. Browser input is queued and run on the main thread, and `start()` / `update()` no-op off simulation, so the same calls are safe to leave in real robot code. Default port 5805.
- **Self-describing mechanisms**: every mechanism now implements `describe()`, returning a uniform `MechanismView` snapshot, so the dashboard (or your own tooling) can read its state without knowing its concrete type.
- **Real sim for four more mechanisms**: Roller, Claw, Winch and Differential Wrist now run physics in simulation (`FlywheelSim` / `DCMotorSim` / `ElevatorSim`), joining the rest. New sim-only helpers: `setSimHasPiece(boolean)` on Roller and Claw, and the `loadMass` / `momentOfInertia` builder fields.
- **`MechanismShowcase` example**: drives one of every mechanism kind through a `SimDashboard` on port 5806, side by side with the game cockpit on 5805.

## What's New in v1.0.0-rc2

- **`AllianceFlipUtil`** — write your field coordinates once in blue-origin, call `AllianceFlipUtil.apply(...)` for the current alliance. Configurable field size + symmetry (rotational / mirrored), unit-tested.
- **[Shoot-On-The-Fly Visualizer](https://tomas-1226.github.io/FrcCatalyst/tools/aiming/)** — a new browser tool: drag the robot, set a velocity, see the virtual goal / lead / turret bearing / feedforward rate update live, running the exact `AimingSolver` math. Copies the `track(...)` wiring.

## What's New in v1.0.0-rc1 — Preseason Release Candidate 1

The Shoot-On-The-Fly stack, hardened and proven. Backward compatible.

- **Fixed a real SOTF bug:** the aiming solver's virtual-goal iteration was off-by-one, so even a *perfect* shot landed slightly off when moving — an error that scaled with speed and could never reach zero. The solve now converges to a true fixed point; a closed-loop unit test over **4,459** pose/velocity cases confirms the ideal shot lands within **1.3 × 10⁻¹⁴ m** of the target.
- **`flywheel.track(...)` and `hood.track(...)`** — continuous setpoint tracking so RPM and hood follow the live distance during SOTF (closes the radial-motion gap; the turret already had `track`).
- **`turret.track(solution, heading, yawRate)`** — exact analytic-rate velocity feedforward, so the turret leads a moving goal even while the chassis spins. Plus `turret.aimErrorDeg(...)` / `isOnTarget(...)` for "ready to shoot" gating.
- **TurretMechanism now self-simulates** (`DCMotorSim`) — turret/SOTF code moves in the WPILib simulator with no hardware.
- **First JUnit test suite ships with the library** (`./gradlew test`): SOTF proof, solver self-consistency, lead geometry, NaN/edge cases, and turret continuous-wrap.
- Velocity-aware **NaN guard** in the solver so a pose glitch can't poison the aim.

## What's New in v0.10.1-beta

- **Fixed: `slowModeWhileHeld` (and the reset commands) no longer require the drivetrain**, so holding slow mode keeps you driving instead of interrupting the default command. No more `.proxy()` workaround, and it behaves in sim. Thanks to tcrvo (3211) for the report.

## What's New in v0.10.0-beta

- **Reactive autonomous architecture** — blend a planned PathPlanner route with reactive behavior (chasing pieces, auto-aligning) the way it's actually done in 2026:
  - **`PathCorrection`** — bend a path toward a live target *without leaving it* via PathPlanner's feedback-override (face a goal while following the path; shoot-on-the-move). Overrides are always cleared on command end — no leaks.
  - **`SwerveSubsystem.pathfindThenFollowPath(name)`** — rejoin the plan from wherever a deviation left you (the primitive most teams miss), plus `followPath(name)` for exact segments.
  - **Auto Builder** command templates map onto these; full [Autonomous Architecture](https://tomas-1226.github.io/FrcCatalyst/advanced/autonomous.html) guide explains plan-as-feedforward vs reaction-as-feedback.

## What's New in v0.9.0-beta

- **Auto Builder tool** — generate a behavior-framework auto (resilient `BehaviorEngine` sequence or utility `Strategist`) in the browser. Path-following stays with PathPlanner / Choreo; the tool builds the reactive strategy layer. [Open it](https://tomas-1226.github.io/FrcCatalyst/tools/auto/).
- **`BrownoutMonitor` is now passive by default** — no output throttling or tripping unless you explicitly `.enableThrottling()` / `.tripsRobotSafety(true)`, with warnings that those behaviours are aggressive.
- **maple-sim support** — `SimGamePieces` (game-piece viz) + `SwerveSubsystem.setSimPose(...)` seam + a [wiring guide](https://tomas-1226.github.io/FrcCatalyst/advanced/simulation.html). Dependency-free — you add maple-sim to your own project; Catalyst provides the integration.

## What's New in v0.8.0-beta

- **Choreo paths** — `swerve.followChoreoPath(name)` (via PathPlanner, no extra dep).
- **`driveToPiece`** — vision-pursuit primitive for the Autopilot acquire step.
- **`WheelRadiusCalibration`** — spin the robot, get the corrected wheel radius (fixes CAD-vs-real odometry drift).
- **`BrownoutMonitor`** — predict sag voltage, ease off output, preemptively trip `RobotSafety` before a brownout.
- **`CatalystMotor.optimizeCanBus()`** — cut CAN-bus load on high-device-count robots (opt-in).
- **`WpilogSink`** — record everything to a standard `.wpilog` that opens in AdvantageScope (no extra dep).
- Roadmap updated: Tier 2 complete; QuestNav + maple-sim saved for later with integration notes; Auto Builder / log-scrubber tools declined (PathPlanner + AdvantageScope already do those).

## What's New in v0.7.0-beta

- **`SystemCheck`** — pre-match self-test. Run every subsystem through a verification routine from one button; get a per-test pass/fail board, a `Ready` go/no-go, and a copy-paste report. Catches loose connectors, inverted motors, dead followers, stuck encoders — the failures that actually lose matches. No mainstream FRC library packages this. [Guide](https://tomas-1226.github.io/FrcCatalyst/advanced/system-check.html).
- **[Roadmap + competitive analysis](https://tomas-1226.github.io/FrcCatalyst/ROADMAP.html)** — where Catalyst stands vs YAGSL / AdvantageKit / maple-sim / QuestNav / Choreo, and what's next (QuestNav pose source, maple-sim physics sim, deterministic replay).

## What's New in v0.6.1-beta

- **Cleanup pass.** Audited every doc example against the source; fixed methods that were documented but never built and two browser tools generating non-compiling code.
- Added `SwerveSubsystem.getFieldRelativeSpeeds()` (the correct input for SOTF — robot-relative speeds would mis-aim), `getMaxSpeedMPS()`/`getMaxAngularRate()` getters, and `RobotSafety.trip(...)`/`tripCommand(...)`.
- Confirmed + documented PathPlanner `AutoBuilder` support.

## What's New in v0.6.0-beta

- **Behavior framework** (`frc.lib.catalyst.behavior`) — game-agnostic autonomous + assisted-driving orchestration:
  - **`BehaviorEngine`** — resilient autos that fall back (or bail to a sure-thing score) when a piece isn't where you expected.
  - **`Strategist`** — utility selector: "chase scattered pieces until the goal's met or time runs short, then go align and shoot," expressed as crossing score curves. No state machine.
  - **`Autopilot`** — teleop cycle co-pilot: hold a button, the robot runs acquire → score → repeat, releases to you instantly.
  - **`Action`** — the atomic building block (command + precondition + success + cost), composed from `pathfindToPose`, `AimingSolver`, mechanisms, vision. Carries forward to future games unchanged. See [the guide](https://tomas-1226.github.io/FrcCatalyst/advanced/behavior.html).
- **Multi-camera vision hardening** — `VisionSubsystem` now fuses 4+ cameras deterministically (snapshot, NaN-guard, timestamp-ordered adds, quality/index tiebreaks) so vision-driven decisions are reproducible.

## What's New in v0.5.1-beta

- **`ShotCompensation`** — live aim-bias module for when you're defended or the tables drift. Turret/distance/RPM/hood nudges (D-pad-bindable), a SOTF-aggressiveness scale, and a velocity deadband + clamp so a defender's hit can't fling the turret.
- **`AimingSolver` hardened** — consumes `ShotCompensation`, adds `.maxRange(...)` feasibility, and exposes the analytic field-bearing rate for turret feedforward.
- **Turret velocity feedforward** — `track(...)` now leads a moving goal via `kV` feedforward instead of lagging a loop behind.

## What's New in v0.5.0-beta

- **`TurretMechanism`** — single-axis turret with the continuous-angle wrap / soft-limit unwrap logic done right (picks the shortest reachable move, only unwraps when blocked). Field-relative aim, vision lock, optional CANcoder homing.
- **`AimingSolver`** — Shoot-On-The-Fly math (virtual-goal method, iterated). Pure + unit-testable; feed it robot pose + field velocity, get back a field aim bearing, distance, flight time, RPM and hood from your lookup tables. Pairs with `TurretMechanism.track(...)` for shoot-while-moving. Full writeup: [Turret & Shoot-On-The-Fly](https://tomas-1226.github.io/FrcCatalyst/advanced/aiming.html).

## What's New in v0.4.1-beta

- **`GhostReplay`** — record a teleop drive, replay it later as a ghost pose for driver practice. CSV files under the deploy directory, ghost pose published to NT for AdvantageScope field overlay.
- **Per-mechanism `bindRumble(events, pattern, channel)`** shortcut — picks the mechanism's natural event (Claw/Roller → has-piece, Flywheel → at-speed, DifferentialWrist → at-setpoint). One-liner instead of having to remember which trigger to bind.

## What's New in v0.4.0-beta

- **Driver feel** — `RumbleEvents` (bind any Trigger to controller rumble, five built-in patterns) and `DriverProfile` (deadband, response curve, max-speed cap, slow mode — swap drivers by swapping profiles).
- **`RobotState`** — one singleton wrapping `DriverStation` and `RobotController`. Cached, has ready-to-bind triggers (`lateMatch(20)`, `lowBattery(11.0)`, `autonomous()`).
- **SysId on every motor** — `mechanism.sysIdQuasistatic(Direction)` / `.sysIdDynamic(Direction)` work out of the box. Phoenix-6 SignalLogger captures the data; just call `SignalLogger.start()` in `robotInit()`.
- **`LimelightTriggers`** — `hasTarget()`, `tagInView(int)`, `detectorClass("note")`, `targetWithinArea(2.0)`, etc.
- **`SwerveSetpointGenerator`** — accel-clamped chassis-aware setpoint wrapper for the common skid case.
- **Health Dashboard timeline** — recent `HealthHistory` events as severity-colored swim-lanes with hover tooltips.

## What's New in v0.3.6.1-beta

- **Silent follower loss in Linear / Rotational, fixed.** `.follower(canId, oppose)` is now additive on every mechanism (same fix that landed for Claw and Flywheel in v0.3.5). The pre-existing `additionalFollower(...)` workaround is `@Deprecated` and now forwards to `.follower(...)`.
- **`RollerMechanism` now supports followers** — two-motor intakes with one builder call per follower.
- **Builder UI generated code that didn't compile** for elevator / arm / roller. The schema was using `gravity()` (no such method, it's `gravityGain()`), `toleranceDegrees()` (it's `tolerance()`), and `intakeVoltage / outtakeVoltage / holdVoltage` on Roller (it's `intakeSpeed / ejectSpeed / stallDetection`). All four fixed. The Intake preset now also wires a mirrored follower so 2-motor intakes work out of the box.

## What's New in v0.3.6-beta

- **`CANRegistry`** — every `CatalystMotor` (and any CANcoder it touches) now auto-claims its `(bus, id)` at construction. Duplicates throw with both sides named. Plan is published to `/Catalyst/CAN/Devices`.
- **CAN ID Planner ships a "Generate Catalyst Java" button** — emit a `CANIds.java` with `SCREAMING_SNAKE_CASE` constants and a static block that pre-registers every planned device. Call `CANIds.init()` from `Robot.robotInit()` and a missing or mis-named device crashes the program at boot instead of failing silently mid-match.

## What's New in v0.3.5.1-beta

- **Five hosted tools, no install required.** Open [tomas-1226.github.io/FrcCatalyst/tools/](https://tomas-1226.github.io/FrcCatalyst/tools/) and you get [🛠️ Catalyst Builder](https://tomas-1226.github.io/FrcCatalyst/tools/builder/), [🎚 Catalyst Tuner](https://tomas-1226.github.io/FrcCatalyst/tools/tuner/), [🩺 Health Dashboard](https://tomas-1226.github.io/FrcCatalyst/tools/health/), [📈 Motion Profile Visualizer](https://tomas-1226.github.io/FrcCatalyst/tools/motion/), and [⚡ MotorType Browser](https://tomas-1226.github.io/FrcCatalyst/tools/motors/) — all in one click.
- **Builder upgrades**: localStorage persistence, **download as `.java`**, **full subsystem class** mode (wraps the config in a `SubsystemBase` skeleton), and **import** to populate the form from an existing `Foo.Config.builder()...build()` snippet.
- **Tuner**: **Download gains JSON** — archive a working tune between events with one click.
- **Health Dashboard**: **Download report** — text snapshot of every check, ready to paste into team chat when triaging.
- **`RobotSafety.trippedTrigger()`** — returns a WPILib `Trigger` so you can bind safety responses in one line in `configureBindings()`.
- **`HealthHistory`** — fixed-capacity ring buffer of recent fire/clear events; auto-fed by `HealthMonitor`, published at `/Catalyst/Health/History`, queryable from robot code.

## What's New in v0.3.5-beta

- **`DifferentialWristMechanism` now uses Phoenix-6's native differential control** — `DifferentialMotionMagicVoltage` on the master with `DifferentialFollower` on the slave. Both targets are sent in one CAN frame and stay coordinated at firmware level. Slot 0 holds the pitch (average) gains; Slot 1 holds the roll (differential) gains, tunable separately via `.differentialPid(...)`. Thanks to **tcrvo** for flagging this on Chief Delphi.
- **Multi-follower support for Claw and Flywheel** — `ClawMechanism.Config.follower(canId, oppose)` is now additive (call it once per follower), and `FlywheelMechanism.Config.primaryFollower(...) / secondaryFollower(...)` add follower motors on each shaft. Thanks to **avrahamavraham** for catching that the single-follower limit was broken.
- **`RobotSafety` watchdog** — an opt-in cross-mechanism trip that fires when too many ERROR-severity health checks fire simultaneously. Hooks into `HealthMonitor`, publishes to `Catalyst/Safety/`, and runs your `onTrip` callback so team code decides what "all-stop" means.
- **Catalyst Builder UI** at `docs/tools/builder/index.html` — single-file form-driven Java code generator for every mechanism. Inspired by tcrvo / yteam3211's [FRC Catalyst Subsystem Generator](https://yteam3211.github.io/frc-catalyst-subsystem-generator).
- **More motor presets** — `KRAKEN_X44` / `KRAKEN_X44_FOC` (added in v0.3.3) joined by `NEO`, `NEO_VORTEX`, `NEO_550`, and `MINION`. `MotorType` also exposes a public constructor for anything we don't ship.

## What's New in v0.3.3-beta

> ⚠️ **Important fix — read if you used `MotorType.KRAKEN_X60_FOC` or `MotorType.FALCON_500_FOC` in earlier 0.3.x versions.** Those two presets shipped with the same stall torque as their non-FOC counterparts; Phoenix-6 FOC actually delivers ~30% more. `holdingVoltage(...)` was over-stated and sim torque under-stated by the same factor, so re-check hand-tuned `kG` values after upgrading. Details in [CHANGELOG.md](CHANGELOG.md).

- **Health Kit** — `frc.lib.catalyst.util.HealthCheck` + `HealthMonitor` give every mechanism a debounced fault layer with INFO/WARN/ERROR severities, live detail strings, and per-edge fire/clear hooks. Every built-in mechanism wires standard motor checks (over-current, high-temp, over-temp cutoff that auto-stops the motor) plus a few mechanism-specific ones (stall, not-zeroed, not-spinning-up, low-pressure). State publishes to `Catalyst/Health/...` on NT and forwards through the existing `AlertManager`.
- **Health Dashboard** — `docs/tools/health/index.html`, a single-file web viewer that connects to NT4 read-only, shows per-subsystem cards with severity-colored firing checks, filter buttons, and search. Pairs with the existing Tuner UI.
- **Kraken X44 presets** + a public `MotorType` constructor so teams using NEO, Minion, or any other motor can declare their own: `new MotorType("NEO 550", 0.97, 11000, 100, 1.4)`.
- **Teams can add their own health checks in one fluent call**: `HealthCheck.builder("Climber", "RopeSlack").severity(WARN).when(() -> ...).debounce(0.5).register();`

## What's New in v0.3.2-beta

- **Tunable PID + Motion Magic by default** — every closed-loop mechanism now publishes its gains under `Catalyst/Tuning/<MechanismName>/...` and applies dashboard edits live on the next robot loop. No extra robot code, no new dependencies, just deploy and tune. See [docs/advanced/tuning.md](docs/advanced/tuning.md). Lock everything for competition with one call: `TunableNumber.disableTuning()`.

## What's New in v0.3.1-beta

- **Fixed**: `ClawMechanism.hasPiece()` now OR-combines beam-break and stall detection (previously beam-break short-circuited the stall latch).
- **Added**: `PneumaticMechanism.timeInState()` for sequencing with `Commands.waitUntil`.

## What's New in v0.3.0-beta

- **Multi-follower support on `LinearMechanism`** — chain as many follower TalonFXs as you need. Each `withFollower(...)` call is now additive, fixing the v0.2 limitation that capped you at one.
- **In-house logging core** (`frc.lib.catalyst.logging`) — every mechanism routes telemetry through a pluggable `LogSink`. Default sink keeps the v0.2 `/Catalyst/...` NetworkTables layout so dashboards work unchanged.
- **AdvantageKit interop without bundling** — install a ~10-line `LogSink` to forward everything into AK, DataLog, or anything else. Catalyst itself has zero AK dependency. See [docs/advanced/logging.md](docs/advanced/logging.md).
- **IO + Inputs contract** (`frc.lib.catalyst.io`) — every mechanism now publishes a replay-shaped `*MechanismInputs` POJO each loop. Default Phoenix 6 / sim IO swaps land in v0.4.
- **Three new mechanisms** — `ClawMechanism`, `DifferentialWristMechanism`, `PneumaticMechanism`.
- **Configurable tolerances** — `RotationalMechanism` and `DifferentialWristMechanism` accept `tolerance(...)` in their builder; `LinearMechanism` already supports it.
- **Forward-limit auto-zero** on `LinearMechanism` (mirrors the existing reverse-limit support).
- **Bug fixes** — `RotationalMechanism.atPosition(name)` no longer ignores the configured tolerance; `LinearMechanism` auto-zero now seeds to `config.minPosition` instead of 0; sim motor count tracks the real follower count.

---

## Architecture

```
frc.lib.catalyst
|
+-- hardware/           Motor, encoder, gyro wrappers
|   +-- CatalystMotor       TalonFX with builder config + telemetry + fault detection
|   +-- CatalystEncoder     CANcoder wrapper
|   +-- CatalystGyro        Pigeon2 IMU wrapper
|   +-- MotorType           Motor specs enum (Kraken, Falcon)
|
+-- mechanisms/          Generic reusable mechanisms
|   +-- LinearMechanism             Elevator, slide, telescoping arm
|   +-- RotationalMechanism         Arm, wrist, turret, hood
|   +-- FlywheelMechanism           Shooter, accelerator wheel
|   +-- RollerMechanism             Intake, conveyor, indexer (with ramp, pulse, voltage feed)
|   +-- WinchMechanism              Climber, deployment
|   +-- ClawMechanism               Motor-driven gripper with stall-based grip detection
|   +-- DifferentialWristMechanism  Two-motor diffy wrist (pitch + roll)
|   +-- PneumaticMechanism          Single/double solenoid with pressure-aware safety
|   +-- SuperstructureCoordinator   Deprecated in v1.2.0 — use statemachine/ below
|
+-- statemachine/        Whole-robot state machine (v1.2.0)
|   +-- StateMachineCore         The engine: graph, guards, staging, deadlines, logging
|   +-- StateGraph               Declared legal transitions + route planning
|   +-- StateSpec, EdgeSpec      Per-state goals; per-edge guards, staging, timeouts
|   +-- Binding, Handle          What a mechanism must answer; typed reference to one
|   +-- Phase, RejectReason      Where a transition is; why one was refused
|   +-- Routing, FaultPolicy     Direct-only vs shortest-path; what a blown deadline does
|   +-- TransitionRecord         One history entry: outcome + per-mechanism arrival times
|   +-- Snapshot                 Whole-machine state as one immutable object
|   +-- goals/                   LinearGoal, RotationalGoal, WristGoal, FlywheelGoal,
|   |                            TurretGoal, ClawGoal, RollerGoal, WinchGoal, PneumaticGoal
|   +-- mech/Mechanisms          Factory facade: 9 typed bindings + 4 escape-hatch tiers
|   +-- robot/Superstructure     Public entry point — builder, commands, triggers
|
+-- io/                  Hardware-abstraction layer (v0.3)
|   +-- *MechanismInputs            Per-loop input snapshots (replay-friendly)
|   +-- *MechanismIO                Hardware contracts; default Phoenix 6 impls land in v0.4
|
+-- logging/             In-house logging core (v0.3)
|   +-- CatalystLog                 Static facade — swap sinks at robot init
|   +-- LogSink                     Pluggable sink interface (NT default, AK-bridgeable)
|   +-- NetworkTablesSink           Default sink — same /Catalyst/... layout as v0.2
|   +-- CompoundSink                Fan-out to multiple sinks simultaneously
|   +-- CatalystInputs              Symmetric toLog/fromLog contract for Inputs POJOs
|
+-- subsystems/          Complex subsystems
|   +-- SwerveSubsystem      Swerve drive with skew correction, snap-to, advanced drive
|   +-- VisionSubsystem      Multi-camera with innovation tracking + speed rejection
|   +-- LEDSubsystem         14 addressable LED patterns
|
+-- util/                Utilities
    +-- FeedforwardGains         kS/kV/kA/kG storage + calculators
    +-- TrapezoidProfileHelper   Motion profile factories
    +-- AlertManager             Centralized fault system
    +-- MechanismVisualizer      Mechanism2d dashboard helper
    +-- CharacterizationHelper   SysId routine wrapper
    +-- CatalystMath             Joystick curves, geometry, physics
    +-- InterpolatingTable       Shooter lookup tables
    +-- SlewRateLimiter          Asymmetric rate limiting
    +-- MovingAverage            Sliding window filter
    +-- TimedBoolean             Debounced boolean
    +-- StateSpaceController     LQR + Kalman filter (optimal control)
    +-- SignalProcessor          EMA, median, low-pass, composite filters
    +-- MotionConstraintCalculator Physics-based motion constraints
    +-- PoseHistory              Temporal pose tracking + interpolation
    +-- DynamicAutoBuilder       Runtime PathPlanner path generation
    +-- TunableNumber            Dashboard-editable constants (live PID tuning)
    +-- AutoSelector             PathPlanner auto chooser with safe defaults
    +-- GamePieceTracker         Multi-stage piece tracking with Triggers
```

---

## Mechanisms

Every mechanism provides:
- **Builder-pattern config** with sensible defaults
- **Motion Magic** position control (on TalonFX)
- **WPILib ProfiledPID** alternative (on roboRIO)
- **Named position presets** (`goTo("STOW")`)
- **Gravity compensation** (elevator static, arm cosine)
- **Built-in simulation** (proper DCMotor models)
- **Automatic telemetry** to NetworkTables
- **Safety features** (temperature cutoff, limit switches, soft limits)
- **Pre-built commands** (goTo, hold, jog, zero)

### LinearMechanism

For elevators, linear slides, and telescoping arms.

```java
LinearMechanism elevator = new LinearMechanism(
    LinearMechanism.Config.builder()
        .name("Elevator")
        .motor(13).follower(14, true)
        .motorType(MotorType.KRAKEN_X60)
        .gearRatio(10.0)
        .drumRadius(0.0254)
        .stages(2) // cascading stages
        .range(0.0, 1.2).mass(5.0)
        .pid(50, 0, 0.5).gravityGain(0.35)
        .motionMagic(2.0, 4.0, 20.0)
        .reverseLimitSwitch(0, true) // auto-zero
        .position("STOW", 0.0)
        .position("HIGH", 1.1)
        .build()
);
```

### RotationalMechanism

For arms, wrists, turrets, and hoods.

```java
RotationalMechanism arm = new RotationalMechanism(
    RotationalMechanism.Config.builder()
        .name("Arm")
        .motor(15)
        .motorType(MotorType.KRAKEN_X60)
        .gearRatio(50.0)
        .length(0.5).mass(3.0) // for simulation + FF estimation
        .range(-10, 120)
        .pid(80, 0, 1.0).gravityGain(0.4)
        .motionMagic(200, 400, 2000)
        .hardStop(1, true, 0.0) // DIO 1, auto-zero at 0 degrees
        .position("STOW", 0).position("SCORE", 100)
        .build()
);
```

### FlywheelMechanism

For shooters with optional dual-motor differential spin.

```java
FlywheelMechanism shooter = new FlywheelMechanism(
    FlywheelMechanism.Config.builder()
        .name("Shooter")
        .motor(20).secondMotor(21) // dual flywheels
        .gearRatio(1.5)
        .pid(0.3, 0, 0).feedforward(0.12, 0.11)
        .velocityTolerance(3.0) // RPS
        .build()
);

// Spin with backspin for shot arc control
shooter.spinUp(70, 50); // top 70 RPS, bottom 50 RPS
```

### RollerMechanism

For intakes with game piece detection.

```java
RollerMechanism intake = new RollerMechanism(
    RollerMechanism.Config.builder()
        .name("Intake")
        .motor(16)
        .intakeSpeed(0.8).ejectSpeed(-0.6)
        .stallDetection(25, 0.2) // 25A for 0.2s = game piece
        .beamBreak(2) // DIO 2
        .build()
);

// Auto-stops when game piece detected
intake.intake();
```

### ClawMechanism

Motor-driven gripper. Closes onto a piece, then drops to a low passive hold
voltage once stall-current detection trips — so the motor doesn't cook trying
to grip harder.

```java
ClawMechanism claw = new ClawMechanism(
    ClawMechanism.Config.builder()
        .name("Claw")
        .motor(30)
        .closeVoltage(6.0).openVoltage(-4.0).holdVoltage(1.5)
        .stallDetection(25.0, 0.2) // 25A for 0.2s => piece gripped
        .currentLimit(40)
        .build()
);

operator.a().onTrue(claw.closeUntilGripped());
operator.b().onTrue(claw.open());
```

### DifferentialWristMechanism

Two motors coupled through a bevel differential to give pitch + roll control
with a single mechanism. Resolves `(pitch, roll) ↔ (leftRotations, rightRotations)`
for you and drives both axes via Motion Magic.

```java
DifferentialWristMechanism wrist = new DifferentialWristMechanism(
    DifferentialWristMechanism.Config.builder()
        .name("Wrist")
        .leftMotor(40).rightMotor(41)
        .gearRatio(20.0)
        .pitchRange(-90, 90).rollRange(-180, 180)
        .pid(40, 0, 0.5)
        .motionMagic(50, 100, 500)
        .position("STOW", 0, 0)
        .position("SCORE", 60, 90)
        .build()
);

wrist.goTo("SCORE");
```

### PneumaticMechanism

Single or double solenoid actuator with command factories, pressure-aware
safety, and cycle counting. Covers climbers, hatch ejectors, shifters, kickers.

```java
PneumaticMechanism climbHook = new PneumaticMechanism(
    PneumaticMechanism.Config.builder()
        .name("ClimbHook")
        .doubleSolenoid(PneumaticsModuleType.REVPH, 0, 1)
        .compressor(PneumaticsModuleType.REVPH)
        .requirePressureAbove(40.0) // refuse to fire below 40 psi
        .build()
);

driver.x().onTrue(climbHook.extend());
driver.y().onTrue(climbHook.retract());
```

---

## State machine

`Superstructure` coordinates every mechanism on the robot from one enum of states. You declare what
each state means — the goal every mechanism holds — and which transitions between states are legal.
A transition you did not declare is one the robot refuses to make, and it says why in the log.

The invariant worth understanding before anything else: **`current()` is only ever a state whose
arrival was proven**, meaning every gating mechanism was measured to be at its goal. If a transition
times out, the machine stays where it actually is rather than pretending it arrived, so the next
transition plans its route from the truth. This is exactly what `SuperstructureCoordinator` got
wrong, and it is why that class is now deprecated.

```java
public enum SuperState { STOW, INTAKE, CARRY, AIM }

var b = Superstructure.builder(SuperState.class, "Superstructure");

var elevator = b.bind("elevator", Mechanisms.linear(elevatorMech));
var arm      = b.bind("arm",      Mechanisms.rotational(armMech));
var intake   = b.bind("intake",   Mechanisms.roller(intakeMech));
var claw     = b.bind("claw",     Mechanisms.claw(clawMech));
var shooter  = b.bind("shooter",  Mechanisms.flywheel(shooterMech));

superstructure = b
    // The safe posture, stated once. Every state inherits it unless it says otherwise, so a
    // mechanism you forget in one state does not stay where the last state parked it.
    .defaults(s -> s
        .set(claw,    ClawGoal.hold())
        .set(intake,  RollerGoal.idle())
        .set(shooter, FlywheelGoal.idle()))

    .state(SuperState.STOW, s -> s
        .set(elevator, LinearGoal.meters(0.0))
        .set(arm,      RotationalGoal.degrees(0)))

    .state(SuperState.INTAKE, s -> s
        .set(elevator, LinearGoal.meters(0.05))
        .set(arm,      RotationalGoal.degrees(-20))
        .set(intake,   RollerGoal.intakeUntilPiece(3.0))
        .set(claw,     ClawGoal.open()))

    .state(SuperState.CARRY, s -> s
        .set(elevator, LinearGoal.meters(0.15))
        .set(arm,      RotationalGoal.degrees(10))
        .set(claw,     ClawGoal.grip(1.5)))

    .state(SuperState.AIM, s -> s
        .set(elevator, LinearGoal.meters(0.30))
        .set(arm,      RotationalGoal.degrees(35))
        .set(shooter,  FlywheelGoal.rpm(4200))
        .entryGuard(clawMech::hasPiece, "no piece"))   // refuse to aim with an empty claw

    .hub(SuperState.STOW)                              // STOW connects to everything
    .allow(SuperState.INTAKE, SuperState.CARRY)
    .allowBoth(SuperState.CARRY, SuperState.AIM)

    // Raise the elevator BEFORE the arm swings out, or the arm hits the chassis.
    .edge(SuperState.STOW, SuperState.AIM, e -> e.stage(elevator).stage(arm))

    .build();

// In robotInit, tell it where the robot is physically built. Nothing moves; this only
// establishes the origin the first transition plans from.
superstructure.engine().seed(SuperState.STOW);

// Bindings. The String is recorded in the transition history, so the log says who asked.
operator.a().onTrue(superstructure.goTo(SuperState.INTAKE, "op.a"));
operator.y().onTrue(superstructure.goTo(SuperState.AIM,    "op.y"));
superstructure.arrivedAt(SuperState.CARRY).onTrue(leds.flash(Color.kGreen));
```

Any subsystem you wrote yourself works here too. `Mechanisms.instant`, `.custom`, `.commands` and
`.build` cover everything from an LED strip with no arrival concept up to a vendor-SDK wrapper that
needs its own tolerance, range checking and blocker notes.

Full guide — every goal type, the four escape-hatch tiers, the logging schema, fault policies, and
migrating off `SuperstructureCoordinator`: [docs/advanced/statemachine.md](docs/advanced/statemachine.md).

---

## Logging & AdvantageKit Bridge

Every mechanism routes telemetry through `CatalystLog`, a static facade backed
by a pluggable `LogSink`. By default a `NetworkTablesSink` keeps the v0.2
`/Catalyst/<name>/...` layout. To send everything to AdvantageKit instead,
install a thin sink at robot init:

```java
CatalystLog.setSink(new LogSink() {
    @Override public void log(String key, double v)  { Logger.recordOutput(key, v); }
    @Override public void log(String key, boolean v) { Logger.recordOutput(key, v); }
    // ... wire the remaining typed overloads similarly
});
```

Catalyst itself has **no AdvantageKit dependency** — the bridge lives in your
code. Full details and a fan-out example in [docs/advanced/logging.md](docs/advanced/logging.md).

---

## Swerve Drive

Wraps CTRE Tuner X generated swerve code with PathPlanner, vision, heading lock, point-at-target, skew correction, and advanced drive features.

```java
SwerveSubsystem drive = new SwerveSubsystem(
    TunerConstants.createDrivetrain(),
    4.5, // max speed m/s
    SwerveSubsystem.PathPlannerConfig.builder()
        .translationPID(5.0, 0, 0)
        .rotationPID(5.0, 0, 0)
        .build()
);

// Enable advanced features
drive.setSkewCorrectionEnabled(true);
drive.enableSlewRateLimiting(2.0, 5.0);
drive.setSnapToAngles(new double[]{0.0, 90.0, 180.0, 270.0}, 5.0);

// Advanced drive: deadband + slew + heading lock + snap + skew correction
drive.setDefaultCommand(drive.advancedDrive(
    () -> -controller.getLeftY(),
    () -> -controller.getLeftX(),
    () -> -controller.getRightX(),
    0.05
));

// Slow mode for precision
driver.leftBumper().whileTrue(drive.slowModeWhileHeld(0.3));
```

---

## Vision

Multi-camera pose estimation with advanced Kalman filter tuning, innovation tracking, and multi-layer filtering.

```java
VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
    .addLimelight("limelight-front",
        new Transform3d(0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-15), 0)))
    .addPhotonCamera("cam-back",
        new Transform3d(-0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-20), Math.PI)),
        fieldLayout)
    .driveSubsystem(drive)
    .baseXYStdDev(0.3)
    .baseRotStdDev(0.7)
    .xyDistanceScaling(1.0)
    .rejectDuringSpin(2.0)
    .rejectDuringHighSpeed(3.0)        // reject when > 3 m/s
    .maxHeadingDivergence(15.0)        // reject if heading disagrees > 15 deg
    .fieldDimensions(16.54, 8.21)      // custom field bounds
    .maxLatency(0.5)
    .build());
```

---

## Utilities

| Utility | Description |
|---------|-------------|
| `FeedforwardGains` | Store and calculate kS/kV/kA/kG for any mechanism type |
| `TrapezoidProfileHelper` | Factory methods for WPILib ProfiledPIDController |
| `AlertManager` | Centralized fault/warning system with NetworkTables publishing |
| `MechanismVisualizer` | Mechanism2d dashboard builder (elevator + arm visualization) |
| `CharacterizationHelper` | SysId routine wrapper for one-line characterization setup |
| `Superstructure` | Whole-robot state machine: every mechanism type, legal-transition graph, proven arrival, full logging |
| `SuperstructureCoordinator` | **Deprecated in v1.2.0** (frozen, not removed) — use `Superstructure` |
| `InterpolatingTable` | TreeMap-based linear interpolation (shooter distance tables) |
| `CatalystMath` | Joystick curves, angle math, geometry helpers, physics |
| `SlewRateLimiter` | Asymmetric rate limiter (different accel/decel profiles) |
| `MovingAverage` | Sliding window average filter |
| `TimedBoolean` | Debounced boolean with rising/falling edge detection |
| `StateSpaceController` | LQR + Kalman filter for optimal mechanism control |
| `SignalProcessor` | EMA, median, low-pass, composite sensor filters |
| `MotionConstraintCalculator` | Physics-based max velocity/acceleration from motor specs |
| `PoseHistory` | Temporal pose tracking with interpolation for latency compensation |
| `DynamicAutoBuilder` | Runtime path generation with PathPlanner |
| `TunableNumber` | Dashboard-editable constants for live PID tuning |
| `AutoSelector` | PathPlanner auto chooser with safe fallbacks |
| `GamePieceTracker` | Multi-stage game piece state machine with Triggers |

---

## Encoder Architecture

By default, FrcCatalyst uses the **TalonFX internal encoder** — no extra hardware needed. For mechanisms requiring absolute positioning (e.g., an arm that must know its angle on startup), you can optionally fuse a CANcoder:

```java
// Default: internal encoder only (simplest, no extra hardware)
CatalystMotor.builder(1)
    .sensorToMechanismRatio(10.0)   // 10 motor rotations = 1 mechanism rotation
    .build();

// FusedCANcoder (requires Phoenix Pro license)
CatalystMotor.builder(1)
    .fusedCANcoder(20, 1.0)         // CANcoder ID 20, 1:1 rotor-to-sensor
    .sensorToMechanismRatio(10.0)
    .build();

// SyncCANcoder (no Pro license needed)
CatalystMotor.builder(1)
    .syncCANcoder(20, 1.0)
    .sensorToMechanismRatio(10.0)
    .build();
```

| Mode | Pro Required | Accuracy | Use Case |
|------|-------------|----------|----------|
| Internal (default) | No | Good | Elevators, flywheels, most mechanisms |
| FusedCANcoder | Yes | Best | Swerve azimuth, precision arms |
| SyncCANcoder | No | Good+ | Arms that need boot-up absolute position |
| RemoteCANcoder | No | Moderate | Legacy setups |

---

## Physics Estimation

FrcCatalyst can estimate feedforward gains and max speeds from your mechanism's physical specs:

```java
var config = LinearMechanism.Config.builder()
    .motorType(MotorType.KRAKEN_X60)
    .gearRatio(10.0)
    .drumRadius(0.0254)
    .mass(5.0)
    .stages(2)
    .build();

double gravityFF = config.estimateGravityFF();  // ~0.35V
double maxSpeed = config.estimateMaxSpeed();      // ~1.9 m/s
```

---

## Testing

FrcCatalyst ships **70 JUnit tests** in `src/test`, run with `./gradlew test`:

- **46 tests covering the state machine engine** — graph validation and refused transitions
  (`StateMachineGraphTest`), transition sequencing, staging and deadlines
  (`StateMachineTransitionTest`), the proven-arrival invariant (`StateMachineTruthTest`), the
  logging schema (`StateMachineTelemetryTest`), and robustness against throwing guards and
  post-timeout recovery (`StateMachineRobustnessTest`).
- **Aiming and field math** — the Shoot-On-The-Fly solver closed-loop proof (`AimingSolverTest`),
  the vector-adding SOTF solver (`AimingSolverVectorTest`),
  turret continuous-wrap (`TurretMathTest`), and alliance flipping (`AllianceFlipUtilTest`).

```bash
./gradlew test
```

A separate end-to-end project at [FrcCatalystTest](https://github.com/TomAs-1226/FrcCatalystTest)
exercises the library in simulation across every mechanism type.

---

## Requirements

| Dependency | Version |
|-----------|---------|
| WPILib | 2026.2.1 |
| CTRE Phoenix 6 | 26.1.1 |
| PhotonVision | v2026.3.1 |
| PathPlanner | 2026.1.2 |
| Java | 17+ |

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## Acknowledgements

- **tcrvo / yteam3211** — feedback on the differential wrist (now uses Phoenix-6's native differential control) and inspiration for the Catalyst Builder UI. Their original [FRC Catalyst Subsystem Generator](https://yteam3211.github.io/frc-catalyst-subsystem-generator) was the seed for `docs/tools/builder/`.
- **avrahamavraham** — caught that `ClawMechanism` only supported one follower and `FlywheelMechanism` had no follower path. Both fixed in v0.3.5-beta.

The rest of the design is in-house work informed by general FRC engineering literature (whitepapers, ChiefDelphi threads, WPILib + CTRE docs) — not copied from any specific team's codebase.

---

## License

This project is available under the [MIT License](LICENSE).

---

<p align="center">
  <sub>Made by an FRC team for FRC teams.</sub>
</p>
