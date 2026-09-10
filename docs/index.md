---
layout: default
title: Home
nav_order: 1
permalink: /
---

# FrcCatalyst
{: .fs-9 }

Pre-built mechanisms, swerve, vision and whole-robot state machines for FRC — on Phoenix 6, WPILib 2027 and Limelight Systemcore.
{: .fs-6 .fw-300 }

{: .warning }
> **This is the beta site.** It documents Catalyst 2.x for WPILib 2027 on Limelight Systemcore,
> which is an alpha platform: the WPILib release is an alpha, the Systemcore OS is a beta, and this
> library has run on that hardware only on a bench — a Pigeon on `can_s0`, the onboard IMU, and the
> board's own status topics. Swerve, mechanisms, autos and vision have not been driven by a real
> robot. Things here will move.
>
> For the version running on robots today - Catalyst 1.x, WPILib 2026, roboRIO - use the
> [stable documentation](https://tomas-1226.github.io/FrcCatalyst/).

[Get Started](getting-started/installation){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 }
[What changed for Systemcore](advanced/systemcore){: .btn .fs-5 .mb-4 .mb-md-0 .mr-2 }
[Tools](tools/){: .btn .fs-5 .mb-4 .mb-md-0 .mr-2 }
[GitHub](https://github.com/TomAs-1226/FrcCatalyst){: .btn .fs-5 .mb-4 .mb-md-0 }

{% include hero3d.html %}

<p style="margin-top: -8px">
  <img src="https://img.shields.io/badge/WPILib-2027.0.0--alpha--7-1f6feb?style=flat-square" alt="WPILib"/>
  <img src="https://img.shields.io/badge/Phoenix%206-26.50.0--alpha--1-e94560?style=flat-square" alt="Phoenix 6"/>
  <img src="https://img.shields.io/badge/Java-25-orange?style=flat-square&logo=openjdk" alt="Java 25"/>
  <img src="https://img.shields.io/badge/PathPlanner-2027.0.0--alpha--3-7c3aed?style=flat-square" alt="PathPlanner"/>
  <img src="https://img.shields.io/badge/LimelightLib-2.0.0--beta8--alpha7-22c55e?style=flat-square" alt="LimelightLib"/>
</p>

---

## 2.0.0-beta.1 — the pre-season release

Tagged 9 September 2026. 867 tests, 0 failures. On JitPack.
{: .fs-5 .fw-300 }

WPILib 2027 is not a version bump. The package root moved from `edu.wpi.first` to `org.wpilib`, the
hardware classes were reorganised, commands v2 was replaced by commands v3 on JDK continuations, and
the roboRIO is replaced by Systemcore. Every team has that port ahead of them.

This release exists so you can do it **now, on an offseason robot, months before kickoff** — rather
than in January with a build season already running. Catalyst absorbed most of the reorganisation
behind its own API: what it forces you to change is the five-row table further down this page. The
rest of the port — your own `edu.wpi.first` imports, any subsystem you wrote yourself — is work
Catalyst cannot do for you, and the [port guide](advanced/systemcore) walks it.

```groovy
repositories {
    maven { url "https://jitpack.io" }
}

dependencies {
    implementation 'com.github.TomAs-1226:FrcCatalyst:v2.0.0-beta.1'
}
```

The vendordep route, and the Phoenix 6 / PathPlanner / LimelightLib vendordeps you need alongside
it, are on [Installation](getting-started/installation).

{: .warning }
> **Flash the Systemcore to OS beta 14 first.** The pairing with WPILib `2027.0.0-alpha-7` is hard,
> not advisory — that OS release is titled "(REQUIRES WPILIB ALPHA 7)", and a mismatch does not
> degrade, it aborts before any robot code runs. On the mismatch we actually measured, alpha-6
> against beta 13, the program died with `MRC API version mismatch` and systemd restarted it 22
> times in two minutes. If Catalyst appears to deploy and then do nothing, check the image before
> you read a line of your own code.

{: .note }
> **Competing this season? Stay on 1.x.** Catalyst `v1.12.0` is the line for robots that have to
> work: WPILib 2026, roboRIO, driven at events. Nothing on this page is a reason to move a
> competition robot. Use 2.x on the practice bot, the offseason chassis, or a laptop.

### Where this build has actually run

| | |
|---|---|
| **On a Systemcore, on a bench** | A Pigeon 2 on `can_s0`, the onboard IMU, and the board's own status topics. That is the whole hardware story. |
| **On a desktop** | 867 tests, 0 failures — most with no HAL at all, the rest booting the robot in WPILib simulation. And the eleven browser tools below. |
| **Not yet driven** | Swerve, mechanisms, autos and vision. None of it has moved a robot. |

The API is what 2.0.0 will be. The version number says what it is standing on, and it is standing on
an alpha that has changed `Mechanism` from an interface to a class and back across three builds.
2.0.0 follows when WPILib 2027 does.

---

## What Catalyst does

Five things it does, and one list of what 2027 changed. Every type and method in the samples below
was checked against the 2.0.0-beta.1 source — they are fragments to read, not files to paste.
{: .fs-5 .fw-300 }

### A mechanism is a description, not a subsystem

```java
import frc.lib.catalyst.mechanisms.LinearMechanism;
import frc.lib.catalyst.hardware.MotorType;

LinearMechanism elevator = new LinearMechanism(
    LinearMechanism.Config.builder()
        .name("Elevator")
        .motor(13).follower(14, true)        // TalonFX CAN ids, follower opposed
        .motorType(MotorType.KRAKEN_X60)
        .gearRatio(10.0).drumRadius(0.0254)  // 10:1 into a 1 in spool
        .range(0.0, 1.2).mass(5.0)           // metres of travel, kg of carriage
        .pid(50, 0, 0.5).gravityGain(0.35)
        .motionMagic(2.0, 4.0, 20.0)         // cruise, accel, jerk
        .currentLimit(40).maxTemperature(80)
        .position("STOW", 0.0)
        .position("HIGH", 1.1)
        .build());
```

There is no second file. Those numbers configure the Motion Magic gains on the TalonFX, build the
WPILib `ElevatorSim` model used in simulation, register the NetworkTables publisher, register the
current, temperature and stall health checks, and produce the command factories below.

Nine more mechanism types take the same shape: arm or wrist, differential wrist, turret, flywheel,
roller, claw, winch, pneumatic and PWM servo. `ServoMechanism` is the exception on this platform:
Systemcore's IO pins output 3.3 V at nowhere near the current a servo draws, so no PWM servo runs
off the board — WPILib removed its `Servo` class for the same reason. Use a CAN servo hub.

### The commands are already written

```java
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.XboxController;

XboxController operator = new XboxController(1);

elevator.setDefaultCommand(elevator.holdPosition());

new Trigger(operator::getAButton).onTrue(elevator.goTo("STOW"));
new Trigger(operator::getYButton).onTrue(elevator.goTo("HIGH"));
new Trigger(operator::getMenuButton).onTrue(elevator.zero());
new Trigger(operator::getLeftBumperButton)
        .whileTrue(elevator.jog(() -> -operator.getLeftY() * 4.0));

// True when the encoder says the elevator is there — not when a wait expired.
Trigger raised = elevator.atPositionTrigger("HIGH");
```

`goTo`, `goToAndWait`, `holdPosition`, `jog`, `zero`, `homeOnCurrent` and `stopCommand` come with
every linear mechanism; the other types have their own. SysId quasistatic and dynamic routines are
on every mechanism without wiring anything.

{: .note }
> **Five decorator names changed in 2.x.** Commands v3 took those names for methods that return
> group *builders*, and Java will not let Catalyst narrow a builder back to a finished command — so
> Catalyst's are renamed rather than shadowing v3's: `until` → `untilTrue`, `andThen` → `then`,
> `alongWith` → `together`, `raceWith` → `racing`, `withTimeout` → `timeoutAfter`. `withName`,
> `finallyDo` and `beforeStarting` are unchanged.

### The whole robot is one state machine

Not a pile of command groups that each assume the last one finished. A declared graph: an edge you
did not declare is a transition the robot refuses to make, and it says why in the log.

```java
import frc.lib.catalyst.statemachine.robot.Superstructure;
import frc.lib.catalyst.statemachine.mech.Mechanisms;
import frc.lib.catalyst.statemachine.goals.LinearGoal;
import frc.lib.catalyst.statemachine.goals.RotationalGoal;
import frc.lib.catalyst.statemachine.goals.RollerGoal;

public enum SuperState { STOW, INTAKE, CARRY, SCORE }

var b      = Superstructure.builder(SuperState.class, "Superstructure");
var lift   = b.bind("elevator", Mechanisms.linear(elevator));
var arm    = b.bind("arm",      Mechanisms.rotational(armMech));
var intake = b.bind("intake",   Mechanisms.roller(intakeMech));

Superstructure<SuperState> superstructure = b
    // Stated once. Every state inherits it, so a mechanism you forget in one
    // state does not stay where the previous state parked it.
    .defaults(s -> s.set(intake, RollerGoal.idle()))

    .state(SuperState.STOW,   s -> s.set(lift, LinearGoal.meters(0.00))
                                    .set(arm,  RotationalGoal.degrees(0)))
    .state(SuperState.INTAKE, s -> s.set(lift, LinearGoal.meters(0.05))
                                    .set(arm,  RotationalGoal.degrees(-20))
                                    .set(intake, RollerGoal.intakeUntilPiece(3.0)))
    .state(SuperState.CARRY,  s -> s.set(lift, LinearGoal.meters(0.15))
                                    .set(arm,  RotationalGoal.degrees(10)))
    .state(SuperState.SCORE,  s -> s.set(lift, LinearGoal.meters(0.30))
                                    .set(arm,  RotationalGoal.degrees(35))
                                    .settleFor(0.2))

    .hub(SuperState.STOW)                              // STOW connects to everything
    .allow(SuperState.INTAKE, SuperState.CARRY)
    .allowBoth(SuperState.CARRY, SuperState.SCORE)

    // Raise the elevator BEFORE the arm swings out, or the arm hits the chassis.
    .edge(SuperState.STOW, SuperState.SCORE, e -> e.stage(lift).stage(arm))
    .build();

superstructure.engine().seed(SuperState.STOW);         // where the robot is physically built
new Trigger(operator::getBButton).onTrue(superstructure.goTo(SuperState.INTAKE));
```

{: .note }
> **Drop `setDefaultCommand` on anything you bind.** The state machine installs the default command
> for every bound mechanism itself, so the `elevator.setDefaultCommand(elevator.holdPosition())`
> line further up comes out once the elevator joins a `Superstructure`. It is not silently
> overwritten: `build()` throws `StateMachineConfigException` naming the binding, and
> `.manageDefaults(false)` is there if you want to schedule the goal runners yourself.

`current()` is only ever a state whose every gating mechanism was *measured* at its goal, so a
timeout leaves the machine where the robot actually is and the next transition plans from the truth.
Guards, entry guards and interlocks each carry a reason string that appears in the log when they
block something. `engine().explain()` prints the graph and where it is stuck in plain language.

[State machine guide](advanced/statemachine){: .btn .fs-3 .mr-2 }
[Draw your graph in the browser](tools/statemachine/){: .btn .fs-3 }

### Telemetry, health and identity you did not write

Every mechanism publishes itself under `/Catalyst/<name>/` from its first scheduler loop. For the
elevator above, with no logging code anywhere in the project:

| Key | |
|---|---|
| `PositionMeters` / `VelocityMPS` | measured position and velocity |
| `SetpointMeters` / `AtSetpoint` | where it was told to go, and whether it got there |
| `CurrentAmps` | stator current |
| `HasBeenZeroed` | whether the zero is trustworthy |
| `State` | what it is doing, as a string, e.g. `GoTo 1.10m` |

Health checks are registered with it. Every motor gets `OverCurrent` (WARN at 90% of
`statorCurrentLimit` — 80 A by default, not the 40 A supply `currentLimit` set above) and
`HighTemp` (WARN at `maxTemperature`), both debounced so one noisy frame is not a fault, plus
`OverTemp` — ERROR at `maxTemperature` + 10 °C, undebounced, and it stops the motor. Two more lines
cover the robot and the board:

```java
import frc.lib.catalyst.identity.RobotIdentity;
import frc.lib.catalyst.util.HealthMonitor;

RobotIdentity.declare("Ratchet");    // spec sheet on NT: team, season, geometry, CAN inventory
HealthMonitor.systemCoreChecks();    // Systemcore CPU, memory, storage, power — no-ops elsewhere
```

Team number, season, controller serial, CAN inventory and — once a `SwerveSubsystem` exists — track
width, wheelbase and top speed are read rather than typed in, so they cannot drift from the robot
after a regear. A builder covers the handful of facts no API can answer, like the frame perimeter
and your build's git SHA. Anything the library cannot read is absent from NetworkTables rather than
published as a zero, because a dashboard can tell an absent key from a published one and cannot tell
a placeholder from a measurement. `systemCoreChecks()` is new for this platform and reads the
board's own CPU, memory, storage and power: a robot that browns out because logs filled the disk
says so in the pit rather than on the field.

[Robot identity](advanced/robot-identity.html){: .btn .fs-3 .mr-2 }
[Health kit](advanced/health.html){: .btn .fs-3 .mr-2 }
[Live health dashboard](tools/health/){: .btn .fs-3 }

### It runs before the robot does

The state machine validates without touching hardware — no HAL, no scheduler, no robot — so the
graph you just declared is a unit test rather than a discovery on the field:

```java
var report = Superstructure.builder(SuperState.class, "Superstructure")
        // ... the same states and edges ...
        .validate();

assertTrue(report.ok(), report.toString());
```

The same is true of the `autonomy` decision cores added in this release: every decision in that
package is a pure function of its inputs returning a record, so a season of scoring rules can be
exercised at a desk with no HAL, no scheduler and no robot. That property is what makes 867 tests
possible on a library pinned to an alpha. On a build that has never been driven, those tests are the
only evidence there is.

Mechanisms also carry a WPILib sim model and a `describe()` snapshot, so `SimDashboard` gives you a
browser cockpit for a robot that does not exist yet.

[Testing guide](testing/){: .btn .fs-3 .mr-2 }
[Simulation](advanced/simulation.html){: .btn .fs-3 }

### What 2027 forced to change, and what it did not

Coming from 1.x, this is what changes in Catalyst's own API. Your own WPILib imports and any
subsystem you wrote yourself are a separate job; the port guide below covers those.

| | |
|---|---|
| `SubsystemBase` is gone | Catalyst subsystems implement `CatalystSubsystem`, which restores `run(Runnable)`, `runOnce(Runnable)` and `startEnd(...)`. If you implement `CatalystSubsystem` directly and write your own `periodic()`, call `registerPeriodic()` in your constructor — v3's `Mechanism` is an interface with no constructor to hook, and an unregistered `periodic()` compiles and silently never ticks. Subclasses of `CatalystMechanism` already do this. |
| Five decorators renamed | `untilTrue`, `then`, `together`, `racing`, `timeoutAfter`. |
| `AutoSelector.getChooser()` | Returns `Selectable<String>`; alpha-7 deleted `SendableChooser` outright, so there is no type left to return. Rename `setDefaultOption` → `addDefault` and `addOption` → `add`. **The one API break 2027 forced on this library** — every other member of `AutoSelector`, `getSelected()` included, is unchanged. The widget's NetworkTables key also moves from `/SmartDashboard/<key>` to `/<key>`, so a dashboard layout hardcoding the old path needs repointing. |
| `ServoMechanism.getServo()` | Now `getPwm()`. Not a rename for its own sake: servos are a hardware limitation on Systemcore, and the port guide below says what actually works. |
| Commands need two JVM flags | `--add-opens java.base/jdk.internal.vm=ALL-UNNAMED` and `--add-opens java.base/java.lang=ALL-UNNAMED`. Nothing fails at build time and nothing fails at startup — the first command the scheduler runs throws `ExceptionInInitializerError` from inside WPILib, naming no class of yours. Catalyst checks when it builds a command and throws something legible instead; `CommandRuntime.isAvailable()` asks directly. |

[The full port guide](advanced/systemcore){: .btn .btn-primary .fs-4 }

---

## What's in the box

Everything below is in the 2.x source and covered by the test suite. Almost none of it has been
driven: the bench list further up is the whole hardware story, and `SwerveSubsystem` and
`VisionSubsystem` in particular have never run on a robot.
{: .fw-300 }

### Mechanisms

| Mechanism | Use Case | Key Features |
|-----------|----------|--------------|
| **LinearMechanism** | Elevators, slides | Position control, gravity FF, limit switches, multi-follower |
| **RotationalMechanism** | Arms, wrists, hoods | Cosine gravity, hard stops, Motion Magic, multi-follower |
| **TurretMechanism** | Aiming turrets | Continuous angle, wrap-safe, field-relative tracking, Motion Magic |
| **FlywheelMechanism** | Shooters | Dual motor + per-shaft followers, velocity PID, at-speed trigger |
| **RollerMechanism** | Intakes, conveyors | Stall detection, beam break, auto-stop |
| **WinchMechanism** | Climbers | Extend/retract limits, position tracking, dual-arm support |
| **ClawMechanism** | Motor-driven grippers | Stall detection, beam break, multi-follower, passive-hold |
| **DifferentialWristMechanism** | Diffy wrists (pitch + roll) | **Phoenix-6 native differential control**, separate Slot 0 / Slot 1 tuning |
| **PneumaticMechanism** | Solenoids / pistons | Double or single solenoid, optional pressure-gating, pulse / toggle commands |
| **ServoMechanism** | PWM servos: hoods, ratchet releases, funnel flappers | Open-loop PWM, named positions, angle clamped to the real travel. **Systemcore cannot drive a PWM servo** — 3.3 V pins, nowhere near the current. Use a CAN servo hub |
| **Superstructure** | Whole-robot coordination | Real state machine over **all ten mechanism types plus your own subsystems** — legal-transition graph, guards, interlocks, staged actuation, measured arrival, full logging. Replaces the deprecated `SuperstructureCoordinator` (linear + rotational only), which still works and is not being removed |

### Subsystems

- **SwerveSubsystem** — wraps the TunerX-generated drivetrain. Heading lock, point-at-target, skew correction, PathPlanner and Choreo path following, `driveToPose`, `pathfindToPose`, X-brake.
- **VisionSubsystem** — multi-camera with Kalman innovation tracking, high-speed rejection, heading-divergence filtering. Limelight-first; MegaTag1 and MegaTag2 are de-duplicated per source.
- **LEDSubsystem** — 14 patterns (fire, gradients, Larson scanner, alignment indicator).

### Advanced

| | |
|---|---|
| `StateSpaceController` | LQR + Kalman for optimal mechanism control |
| `MotionConstraintCalculator` | Physics-derived velocity / accel from motor specs |
| `SignalProcessor` | EMA, median, low-pass, composite filters |
| `PoseHistory` | Timestamped pose ring buffer with interpolation |
| `DynamicAutoBuilder` | Runtime path generation via PathPlanner |
| `TunableNumber` | Dashboard-editable constants for live PID tuning |
| `AutoSelector` | PathPlanner auto chooser with safe fallbacks |
| `GamePieceTracker` | Multi-stage piece state machine with Triggers |
| `Superstructure` / `StateMachineCore` | Enum-typed whole-robot state machine: undeclared edges are refused, arrival is measured rather than assumed, and every decision is logged |
| [`RobotIdentity`](advanced/robot-identity.html) | One line publishes the robot's spec sheet on NetworkTables — read from the robot rather than typed in, and silent about anything the library cannot read |
| [`CatalystFeatures`](advanced/robot-identity.html) | Which parts of Catalyst the robot runs, recorded by each part as it is built |
| [Physics Core](advanced/physics.html) | Optional, strictly advisory: fused velocity with a confidence, live centre of mass and tipping margin, closed-form ballistics, online gain identification (reported, never applied) |
| `frc.lib.catalyst.autonomy` | New in 2.x. Pure-function decision cores — cycle phase, task arbitration, target chase, authority limiting, current shedding, driver-intent inference — each testable with no HAL |
| Skew correction | Pose-exponential discretization for swerve |
| Collision zones | A named condition that should never be true. It **detects, it does not prevent** — the deprecated `SuperstructureCoordinator` logs a warning when one goes active. Staged actuation and interlocks on `Superstructure` are the current answer |
| `SimDashboard` | Generic browser sim cockpit that adapts to any mechanism via `describe()` / `MechanismView`; sim-only, dependency-free |

### Health and safety

| | |
|---|---|
| `HealthCheck` / `HealthMonitor` | Debounced INFO / WARN / ERROR layer; every motor gets OverCurrent / HighTemp / OverTemp by default |
| `HealthMonitor.systemCoreChecks()` | Systemcore's own CPU, memory, storage and power; no-ops off Systemcore |
| `HealthHistory` | Ring buffer of recent transitions; published for the dashboard timeline |
| `RobotSafety` | Optional watchdog with `trippedTrigger()` for one-line bindings |
| `MotorType` | NEO / Vortex / 550 / Minion / Kraken (including X44 and corrected FOC torques), plus a public constructor for custom motors |
| `CANBusPlanner` | Systemcore's five CAN buses are three controllers — `validate()` finds the plans that look fine per-bus and overload a shared controller |

### Driver & robot state

| | |
|---|---|
| `DriverProfile` | Per-driver deadband + curve + speed cap + slow mode |
| `RumbleEvents` | Bind any Trigger to a controller rumble pattern |
| `RobotState` | One singleton wrapping alliance / match time / mode / battery + ready-to-bind triggers |
| `LimelightTriggers` | `Trigger.tagInView(7)`, `.detectorClass("note")`, `.horizontalErrorBelow(2.0)` |
| SysId on every motor | `mechanism.sysIdQuasistatic(Direction)` / `.sysIdDynamic(Direction)` work out of the box |
| `SwerveSetpointGenerator` | Chassis-aware accel + skid clamp for the common driver-skid case |

### Every mechanism gets

Builder config, Motion Magic or ProfiledPID, named position presets (`goTo("STOW")`),
WPILib sim, NetworkTables telemetry, temperature cutoff, limit-switch auto-zero,
HealthCheck-based fault monitoring, multi-follower support, pre-built commands,
SysId quasistatic / dynamic routines, and a self-describing `describe()` / `MechanismView`
snapshot driveable in the built-in `SimDashboard`.

---

## Browser tools

Eleven single-file tools served from this site. Click and use — no clone, no install.

<style>
.hero-tools {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(220px, 1fr));
  gap: 14px;
  margin: 18px 0 28px;
}
.hero-tool {
  display: flex; flex-direction: column;
  padding: 18px 18px 16px;
  border: 1px solid rgba(255,255,255,0.08);
  border-radius: 12px;
  text-decoration: none !important;
  color: inherit;
  background: linear-gradient(180deg, rgba(255,255,255,0.04), rgba(255,255,255,0.01));
  position: relative; overflow: hidden;
  transition: transform 0.18s ease, border-color 0.18s ease, box-shadow 0.18s ease;
}
.hero-tool::before {
  content: ""; position: absolute; inset: 0;
  background: radial-gradient(300px 160px at 0% 0%, rgba(233,69,96,0.10), transparent 60%);
  opacity: 0; transition: opacity 0.2s ease;
  pointer-events: none;
}
.hero-tool:hover {
  transform: translateY(-3px);
  border-color: rgba(233,69,96,0.55);
  box-shadow: 0 14px 30px rgba(0,0,0,0.45);
}
.hero-tool:hover::before { opacity: 1; }
.hero-tool .icon {
  font-size: 22px;
  width: 40px; height: 40px;
  display: flex; align-items: center; justify-content: center;
  border-radius: 10px;
  background: rgba(233,69,96,0.12);
  border: 1px solid rgba(233,69,96,0.20);
  margin-bottom: 10px;
}
.hero-tool .name {
  font-weight: 700; font-size: 15px;
  display: block; margin-bottom: 4px; color: #e94560;
  letter-spacing: -0.01em;
}
.hero-tool .desc {
  font-size: 12px; line-height: 1.5;
  color: var(--body-text-color, #aaa);
}
</style>

<div class="hero-tools">
  <a class="hero-tool" href="tools/builder/"><span class="icon">🛠️</span><span class="name">Builder</span><span class="desc">Form generates ready-to-paste Java for any mechanism.</span></a>
  <a class="hero-tool" href="tools/tuner/"><span class="icon">🎚</span><span class="name">Tuner</span><span class="desc">Live PID + Motion Magic over NT4.</span></a>
  <a class="hero-tool" href="tools/health/"><span class="icon">🩺</span><span class="name">Health Dashboard</span><span class="desc">Every <code>HealthCheck</code> on the robot, live.</span></a>
  <a class="hero-tool" href="tools/motion/"><span class="icon">📈</span><span class="name">Motion Profile</span><span class="desc">Trapezoid + S-curve sketcher with copyable constants.</span></a>
  <a class="hero-tool" href="tools/pid/"><span class="icon">🎯</span><span class="name">PID Step Response</span><span class="desc">Dial gains, watch the simulated response.</span></a>
  <a class="hero-tool" href="tools/motors/"><span class="icon">⚡</span><span class="name">MotorType Browser</span><span class="desc">Every motor preset + gear-ratio calculator.</span></a>
  <a class="hero-tool" href="tools/canids/"><span class="icon">🔌</span><span class="name">CAN ID Planner</span><span class="desc">Catch CAN ID collisions before crimping.</span></a>
  <a class="hero-tool" href="tools/wiring/"><span class="icon">⚡</span><span class="name">Wiring Diagram</span><span class="desc">Power tree and CAN bus from your CAN ID plan.</span></a>
  <a class="hero-tool" href="tools/auto/"><span class="icon">🧭</span><span class="name">Auto Builder</span><span class="desc">Generate a behavior-framework auto.</span></a>
  <a class="hero-tool" href="tools/aiming/"><span class="icon">🎯</span><span class="name">Shoot-On-The-Fly</span><span class="desc">The <code>AimingSolver</code> virtual goal and lead, drawn.</span></a>
  <a class="hero-tool" href="tools/statemachine/"><span class="icon">🔀</span><span class="name">State Machine Visualizer</span><span class="desc">Draw the graph your <code>Superstructure</code> logs — states, guards, dead-ends.</span></a>
</div>

---

## Documentation

| | |
|---|---|
| [Installation](getting-started/installation) | Add Catalyst to your `build.gradle` |
| [Quick Start](getting-started/quickstart) | First mechanism in five minutes |
| [Systemcore & WPILib 2027](advanced/systemcore) | What changed, what Catalyst absorbed, what you change yourself |
| [Mechanisms](mechanisms/) | All ten mechanism types |
| [Subsystems](subsystems/) | Swerve, Vision, LEDs, LimelightTriggers, SwerveSetpointGenerator |
| [Driver](driver/) | DriverProfile, RumbleEvents, controller feel |
| [Utilities](utilities/) | Health Kit, RobotSafety, RobotState, MotorType, CANRegistry, feedforward, profiles |
| [Advanced](advanced/) | Behavior framework, turret + SOTF, state-space, live tuning, health, SysId |
| [Tools](tools/) | The eleven browser tools — incl. Auto Builder + State Machine Visualizer |
| [Examples](examples/) | Whole-robot examples |
| [Testing](testing/) | Unit-testing Catalyst-based code |

---

## Release history

**2.0.0-beta.1** is the 2027 port, on the WPILib and Systemcore OS pair that actually run together.
Alongside the port it fixes a defect worth knowing about even if you stay on 1.x: `CatalystMath`,
`AllianceFlipUtil` and `VisionConfig` all defaulted to a **field width of 8.21 m**, the REEFSCAPE
number, while the rest of the library used 8.07 m. A team that never called `configure(...)` had
every red-alliance Y flipped about an axis 7 cm off centre — 14 cm of error on every mirrored
waypoint, on one alliance only, with nothing reporting a fault. The 2026 REBUILT field is
**16.54 m x 8.07 m** everywhere now, pinned by a test against WPILib's own layout so the two cannot
drift together and still agree.

It also adds `frc.lib.catalyst.autonomy` — the decision cores — and moves `Autopilot` onto
`CycleCore` and `BehaviorEngine` onto `StepCore`. `Strategist` gains `yieldWhen(...)`, so a driver
who interrupts the selector's command no longer has it rescheduled on the next loop.

The 1.x line is where the mechanism library, the `Superstructure` state machine, Physics Core, the
robot spec sheet and the eleven browser tools were built. Those releases are documented on the
[stable site](https://tomas-1226.github.io/FrcCatalyst/); the full history for both lines is in the
[release notes](https://github.com/TomAs-1226/FrcCatalyst/releases) and the
[CHANGELOG](https://github.com/TomAs-1226/FrcCatalyst/blob/main/CHANGELOG.md).

---

## Compatibility

| Component | Version |
|-----------|---------|
| WPILib | 2027.0.0-alpha-7 |
| CTRE Phoenix 6 | 26.50.0-alpha-1 |
| LimelightLib | 2.0.0-beta8-alpha7 |
| PathPlanner | 2027.0.0-alpha-3 |
| Java | 25 |
| Systemcore OS | beta 14 (`limelightosr-2027.0.0-beta14-210`) |
