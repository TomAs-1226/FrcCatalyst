---
layout: default
title: Mechanisms
nav_order: 3
has_children: false
---

# Mechanisms
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

FrcCatalyst provides nine generic mechanism types that cover virtually every FRC subsystem. Each mechanism extends `CatalystMechanism` (which extends WPILib's `SubsystemBase`) and provides:

- **Builder-pattern configuration** with validation and sensible defaults
- **Two control modes**: CTRE Motion Magic (on TalonFX) or WPILib ProfiledPID (on roboRIO)
- **Named position presets** for quick `goTo("STOW")` commands
- **Gravity compensation** (constant for elevator, cosine for arm)
- **Built-in simulation** using accurate WPILib DCMotor models
- **Automatic telemetry** published to NetworkTables under `Catalyst/<name>/`
- **Built-in health monitoring** — every motor-driven mechanism auto-registers `OverCurrent`, `HighTemp`, and `OverTemp` checks with sensible debouncing. See [Health Monitoring](../advanced/health.html).
- **Live-tunable gains** — Slot 0 PID and Motion Magic constants are exposed under `Catalyst/Tuning/...` by default. See [Live Tuning](../advanced/tuning.html).
- **Multi-follower support** — every motor-driven mechanism accepts an arbitrary number of follower motors on the primary shaft (and on the secondary shaft, for Flywheel).
- **Pre-built command factories**: goTo, goToAndWait, holdPosition, jog, zero

## Mechanism Types

| Mechanism | Use Case | Position Unit | Control |
|-----------|----------|---------------|---------|
| **LinearMechanism** | Elevators, slides, telescoping arms | Meters | Motion Magic + Gravity FF |
| **RotationalMechanism** | Arms, wrists, hoods | Degrees | Motion Magic + Cosine Gravity |
| **TurretMechanism** | Aiming turrets (continuous angle, wrap-safe) | Degrees | Motion Magic + field-relative tracking |
| **FlywheelMechanism** | Shooters, accelerator wheels | RPS (velocity) | Velocity PID |
| **RollerMechanism** | Intakes, conveyors, indexers | N/A (duty cycle) | Open-loop + detection |
| **WinchMechanism** | Climbers, deployments | Meters | Duty cycle + limits |
| **ClawMechanism** | Motor-driven grippers | N/A (duty cycle) | Open-loop + stall / beam-break |
| **DifferentialWristMechanism** | Diffy wrists (2-motor pitch+roll) | Degrees (pitch, roll) | Phoenix-6 native differential Motion Magic |
| **PneumaticMechanism** | Solenoids / pistons | FORWARD / REVERSE / OFF | DoubleSolenoid + optional pressure gate |

## Self-describing & simulated

New in rc3, every mechanism describes itself and runs real physics in simulation.

### Every mechanism implements `describe()`

Each mechanism implements `describe()`, which returns a `MechanismView` snapshot
of its live state (name, kind, value, unit, setpoint, range, velocity, current,
and a map of kind-specific extras). That snapshot is what powers the generic
[SimDashboard](../advanced/simulation.html): the dashboard reads `describe()` and
renders a fitting widget for each mechanism without knowing its concrete type.
`Double.NaN` is the universal "not applicable" value, so a flywheel can leave
`min`/`max` unset and a claw can leave `value` unset.

The base `CatalystMechanism.describe()` returns a `"generic"` view, and every
built-in mechanism overrides it with the right kind and units. A team that
subclasses a mechanism still shows up on the dashboard for free, and can
override `describe()` to surface its own state:

```java
@Override
public MechanismView describe() {
    return MechanismView.of(getMechanismName(), "rotational")
        .value(getAngle(), "deg")
        .setpoint(getTargetAngle())
        .range(-90, 90)
        .velocity(getVelocity())
        .current(getCurrent())
        .extra("homed", isHomed())
        .build();
}
```

### Real physics in simulation

All built-in mechanisms run accurate WPILib physics models in simulation. In
rc3, `RollerMechanism`, `ClawMechanism`, `WinchMechanism`, and
`DifferentialWristMechanism` gained `simulationPeriodic()` models, joining the
mechanisms that already simulated (Linear, Rotational, Flywheel, Turret):

- **RollerMechanism** uses a `FlywheelSim`.
- **ClawMechanism** uses a `DCMotorSim`.
- **WinchMechanism** uses an `ElevatorSim` when a spool radius is configured,
  otherwise a `DCMotorSim`.
- **DifferentialWristMechanism** uses two `DCMotorSim` models (one per axis).
- **PneumaticMechanism** has no continuous position, so its `describe()` is
  cosmetic and there is no physics model.

### Sim-only helpers

A few inputs exist purely to drive the simulation models. They are no-ops or
ignored on a real robot, where state comes from real sensors:

- `RollerMechanism.setSimHasPiece(boolean)` and
  `ClawMechanism.setSimHasPiece(boolean)` force the simulated game-piece state.
  A flywheel or DC-motor model will not naturally stall against a virtual game
  piece, so this lets a dashboard toggle simulate intaking and scoring. Both
  only take effect under `RobotBase.isSimulation()`; on a real robot detection
  still comes from the beam break or stall logic.
- `WinchMechanism.Config.builder().loadMass(double kg)` sets the mass the winch
  lifts in the sim model (default `6.0` kg). Used only by the simulation, ignored
  on a real robot.
- `DifferentialWristMechanism.Config.builder().momentOfInertia(double kgMetersSquared)`
  sets the per-axis moment of inertia for the sim model (default `0.004` kg m^2).
  Used only by the simulation, ignored on a real robot.

## DifferentialWristMechanism

A two-motor differential wrist (a.k.a. "diffy wrist") where sum of motor rotations
controls pitch and difference controls roll. Catalyst drives this through
**Phoenix-6's native differential control**: the left motor is the differential
master running `DifferentialMotionMagicVoltage`; the right is configured as a
`DifferentialFollower`. Both targets ship in a single CAN frame and stay
coordinated at firmware level.

```java
DifferentialWristMechanism wrist = new DifferentialWristMechanism(
    DifferentialWristMechanism.Config.builder()
        .name("Wrist")
        .leftMotor(40)     // becomes differential master
        .rightMotor(41)    // becomes differential follower
        .gearRatio(20.0)
        .pitchRange(-90, 90)
        .rollRange(-180, 180)
        .pid(40, 0, 0.5)              // Slot 0 → pitch (average) axis
        .differentialPid(30, 0, 0.3)  // Slot 1 → roll  (differential) axis (optional)
        .motionMagic(50, 100, 500)
        .currentLimit(40)
        .position("STOW", 0, 0)
        .position("SCORE", 60, 90)
        .build());

controller.a().onTrue(wrist.goTo("SCORE"));
```

Slot 1 gains are live-tunable at `/Catalyst/Tuning/<Name>/Diff/...` alongside
the existing Slot 0 tunables. If `.differentialPid(...)` isn't called the
differential controller uses the same gains as the average controller —
fine for symmetric wrists, suboptimal for ones where roll has very different
inertia from pitch.

## ClawMechanism

Motor-driven gripper with stall-current grip detection and an optional
beam-break sensor. The "closing" voltage drops to a low passive "holding"
voltage automatically once a piece is detected — the motor isn't asked to
keep squeezing.

```java
ClawMechanism claw = new ClawMechanism(
    ClawMechanism.Config.builder()
        .name("Claw")
        .motor(30)
        .follower(31, true)     // mirrored follower
        .follower(32, true)     // multi-follower supported
        .closeVoltage(6.0)
        .openVoltage(-4.0)
        .holdVoltage(1.5)
        .stallDetection(25.0, 0.2)   // 25 A for 0.2 s → has piece
        .beamBreak(0)
        .currentLimit(40)
        .build());

controller.a().onTrue(claw.closeUntilGripped());
controller.b().onTrue(claw.open());
```

For pneumatic claws use `PneumaticMechanism`.

## PneumaticMechanism

Single or double solenoid wrapped as a mechanism with the same logging,
command factories, and Health Kit integration as the motor mechanisms.

```java
PneumaticMechanism climbHook = new PneumaticMechanism(
    PneumaticMechanism.Config.builder()
        .name("ClimbHook")
        .doubleSolenoid(PneumaticsModuleType.REVPH, 0, 1)
        .compressor(PneumaticsModuleType.REVPH)
        .requirePressureAbove(40.0)  // refuse to actuate below 40 psi
        .build());

controller.x().onTrue(climbHook.extend());
controller.y().onTrue(climbHook.retract());
operator.b().onTrue(climbHook.pulse(0.25));    // kicker pattern
```

When `requirePressureAbove(psi)` is set and a compressor with an analog
pressure sensor is wired, the mechanism refuses to drive forward below the
threshold (raising an alert rather than firing a piston dry).

## TurretMechanism

Single-axis turret with continuous-angle resolution and field-relative
target tracking. Handles the wrap / soft-limit "unwrap" problem and pairs
with `AimingSolver` for shoot-while-moving.

```java
TurretMechanism turret = new TurretMechanism(
    TurretMechanism.Config.builder()
        .name("Turret")
        .motor(15)
        .gearRatio(40.0)
        .range(-200, 200)        // mechanical travel; >±180 gives overlap
        .pid(40, 0, 0.5)
        .feedforward(0.15, 0.0)
        .motionMagic(8, 16, 80)
        .tolerance(1.0)
        .cancoder(16, 40.0)      // optional absolute homing
        .build());

// Track a fixed field point while driving:
turret.setDefaultCommand(turret.track(
    () -> solver.solve(drive.getPose(), drive.getFieldRelativeSpeeds()),
    () -> drive.getHeading().getDegrees()));
```

Full guide — including the Shoot-On-The-Fly math — is in
[Turret & Shoot-On-The-Fly](../advanced/aiming.html).

## Multi-follower configuration

Every motor-driven mechanism accepts repeated `.follower(canId, oppose)`
calls — pass `oppose = true` for mirrored followers (e.g. arms or
double-stacked motors). The Flywheel mechanism splits this into
`.primaryFollower(...)` / `.secondaryFollower(...)` so each independently-
controlled wheel can have its own follower set.

```java
// Three-motor climber: master + two followers
WinchMechanism climber = new WinchMechanism(
    WinchMechanism.Config.builder()
        .name("Climber")
        .motor(25)
        .secondMotor(26)    // independent second arm
        // (use .secondMotor for an independent second motor;
        //  use .follower(...) on Claw/Linear/Rotational for ganged motors)
        .build());

// Two-motor-per-side intake claw
ClawMechanism intake = new ClawMechanism(
    ClawMechanism.Config.builder()
        .motor(30).follower(31, true)
        .build());

// Dual flywheel with two motors per wheel
FlywheelMechanism shooter = new FlywheelMechanism(
    FlywheelMechanism.Config.builder()
        .motor(50)
        .primaryFollower(51, true)
        .secondMotor(52)
        .secondaryFollower(53, true)
        .build());
```

Each follower gets its own `OverCurrent` and `HighTemp` checks, so a
fault on a follower is just as visible on the Health Dashboard as one
on the primary.

## SuperstructureCoordinator

{: .note }
**Superseded in v1.2.0 by `frc.lib.catalyst.statemachine.robot.Superstructure`.** The coordinator
only ever understood `LinearMechanism` and `RotationalMechanism` positions, so a robot with a claw,
a shooter, a turret or a climber could not put those mechanisms into its states at all. The new
`Superstructure` takes all nine Catalyst mechanism types and any subsystem you wrote yourself, and
it is a real state machine: a legal-transition graph, guards and interlocks, staged actuation, and
a full log under `/Catalyst/<prefix>/`. See
[the state machine guide](../advanced/statemachine.html).
<br><br>
`SuperstructureCoordinator` is marked `@Deprecated(since = "1.2.0", forRemoval = false)`. It is
frozen, not removed — it still compiles, still runs, and existing robot code keeps working
unchanged. The one thing to know if you stay on it is the bug that motivated the replacement: when
a transition timed out or was interrupted, the coordinator still set the current state to the
target, so the *next* transition planned its route from a state the robot was not actually in. The
new engine never does that — `current()` is only ever a state whose arrival every gating mechanism
was measured to have reached.

The `SuperstructureCoordinator` orchestrates multiple mechanisms into a robust state machine with safe transitions, collision zones, timeouts, entry/exit actions, and telemetry:

```java
SuperstructureCoordinator superstructure = new SuperstructureCoordinator()
    .withLinear("elevator", elevator)
    .withRotational("arm", arm)
    .withTimeout(3.0); // safety timeout

// Define states with entry/exit actions
superstructure.defineState("STOW")
    .setLinear("elevator", 0.0)
    .setRotational("arm", 0.0)
    .onEntry(() -> leds.setSolidColor(Color.kBlue))
    .done();

superstructure.defineState("SCORE_HIGH")
    .setLinear("elevator", 1.1)
    .setRotational("arm", 95.0)
    .onEntry(() -> leds.setSolidColor(Color.kGreen))
    .onExit(() -> leds.setSolidColor(Color.kBlue))
    .done();

// Collision zone: prevent arm extension when elevator is low
superstructure.addCollisionZone("ElevatorArmConflict",
    () -> elevator.getPosition() < 0.3 && arm.getAngle() > 45.0
);

// Custom transition: retract arm before raising elevator
superstructure.addTransitionRule("STOW", "SCORE_HIGH",
    (fromState, toState) -> arm.goTo("STOW")
        .andThen(elevator.goTo("SCORE_HIGH"))
        .andThen(arm.goTo("SCORE"))
);

// Conditional transition (only if holding a game piece)
operatorController.y().onTrue(
    superstructure.transitionToIf("SCORE_HIGH", () -> intake.hasGamePiece())
);

// Monitor transition progress (0.0 to 1.0) on dashboard
double progress = superstructure.getTransitionProgress();
```

## RollerMechanism Extras

In addition to the standard `intake()` and `eject()` commands, `RollerMechanism` provides advanced commands:

```java
// Gradual speed ramp to prevent wheel slip on intake
intake.intakeWithRamp(1.5); // ramp over 1.5 seconds

// Pulsed operation for unjamming
intake.pulse(0.15, 0.1, 0.8); // onTime (s), offTime (s), speed

// Voltage-based feed for battery-independent consistency
intake.feedVoltage(6.0); // apply 6V
```

## Base Class: CatalystMechanism

All mechanisms inherit from this base class which provides:

```java
public abstract class CatalystMechanism extends SubsystemBase {
    // Automatic NetworkTables telemetry
    protected void log(String key, double value);
    protected void log(String key, boolean value);
    protected void setState(String state);

    // Every mechanism has a stop command
    public Command stopCommand();
    protected abstract void stop();

    // Telemetry runs every cycle
    protected void updateTelemetry();
}
```

## Encoder Architecture

By default, FrcCatalyst uses the **TalonFX internal encoder** as the feedback source - no external encoder needed. Use `sensorToMechanismRatio()` to convert motor rotations to mechanism units.

For mechanisms that need **absolute positioning** (e.g., a swerve azimuth or an arm that must know its angle on startup), you can optionally fuse a CANcoder:

```java
// Default: internal encoder only (simplest, no extra hardware)
CatalystMotor.builder(1)
    .sensorToMechanismRatio(10.0)   // 10 motor rotations = 1 mechanism rotation
    .build();

// FusedCANcoder (requires Phoenix Pro license)
// Fuses CANcoder absolute position with internal encoder for best accuracy
CatalystMotor.builder(1)
    .fusedCANcoder(20, 1.0)         // CANcoder ID 20, 1:1 rotor-to-sensor
    .sensorToMechanismRatio(10.0)
    .build();

// SyncCANcoder (no Pro license needed)
// Syncs internal encoder on boot using CANcoder absolute position
CatalystMotor.builder(1)
    .syncCANcoder(20, 1.0)
    .sensorToMechanismRatio(10.0)
    .build();

// RemoteCANcoder (legacy, uses CANcoder as primary feedback)
CatalystMotor.builder(1)
    .remoteCANcoder(20)
    .build();
```

| Mode | Pro Required | Accuracy | Use Case |
|------|-------------|----------|----------|
| Internal (default) | No | Good | Elevators, flywheels, most mechanisms |
| FusedCANcoder | Yes | Best | Swerve azimuth, precision arms |
| SyncCANcoder | No | Good+ | Arms that need boot-up absolute position |
| RemoteCANcoder | No | Moderate | Legacy setups |

## Motion Magic vs. WPILib ProfiledPID

FrcCatalyst supports two control strategies:

### Motion Magic (Default)
Runs on the TalonFX's internal processor. Lower latency, higher bandwidth, and the profile runs at 1kHz. Use the `goTo()` and `holdPosition()` commands.

### WPILib ProfiledPID (Alternative)
Runs on the roboRIO. Enable it in the config builder and use `goToProfiled()` and `holdPositionProfiled()` commands.

```java
LinearMechanism.Config.builder()
    // ... normal config ...
    .useWPILibProfile(12.0, 0, 0.5, 2.0, 4.0) // kP, kI, kD, maxVel, maxAccel
    .build();

// Then use profiled commands:
elevator.goToProfiled("HIGH");
elevator.holdPositionProfiled();
```
