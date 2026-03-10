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
| Safe temperature cutoffs | Manual | **Automatic** |
| Limit switch auto-zeroing | Manual wiring | **One builder call** |

---

## Quick Start

### 1. Add the dependency

Add FrcCatalyst to your robot project's `build.gradle`:

```gradle
repositories {
    maven { url "https://jitpack.io" }
}

dependencies {
    implementation "com.github.TomAs-1226:FrcCatalyst:v1.0.0"
}
```

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

## Architecture

```
frc.lib.catalyst
|
+-- hardware/           Motor, encoder, gyro wrappers
|   +-- CatalystMotor       TalonFX with builder config + telemetry
|   +-- CatalystEncoder     CANcoder wrapper
|   +-- CatalystGyro        Pigeon2 IMU wrapper
|   +-- MotorType           Motor specs enum (Kraken, Falcon)
|
+-- mechanisms/          Generic reusable mechanisms
|   +-- LinearMechanism      Elevator, slide, telescoping arm
|   +-- RotationalMechanism  Arm, wrist, turret, hood
|   +-- FlywheelMechanism    Shooter, accelerator wheel
|   +-- RollerMechanism      Intake, conveyor, indexer
|   +-- WinchMechanism       Climber, deployment
|   +-- SuperstructureCoordinator  Multi-mechanism state machine
|
+-- subsystems/          Complex subsystems
|   +-- SwerveSubsystem      Swerve drive (wraps CTRE generated code)
|   +-- VisionSubsystem      Multi-camera pose estimation
|   +-- LEDSubsystem         Addressable LED patterns
|
+-- util/                Utilities
    +-- FeedforwardGains     kS/kV/kA/kG storage + calculators
    +-- TrapezoidProfileHelper  Motion profile factories
    +-- AlertManager         Centralized fault system
    +-- MechanismVisualizer  Mechanism2d dashboard helper
    +-- CharacterizationHelper  SysId routine wrapper
    +-- CatalystMath         Joystick curves, geometry, physics
    +-- InterpolatingTable   Shooter lookup tables
    +-- SlewRateLimiter      Asymmetric rate limiting
    +-- MovingAverage        Sliding window filter
    +-- TimedBoolean         Debounced boolean
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

---

## Swerve Drive

Wraps CTRE Tuner X generated swerve code with PathPlanner, vision, heading lock, and point-at-target.

```java
SwerveSubsystem drive = new SwerveSubsystem(
    TunerConstants.createDrivetrain(),
    4.5, // max speed m/s
    SwerveSubsystem.PathPlannerConfig.builder()
        .translationPID(5.0, 0, 0)
        .rotationPID(5.0, 0, 0)
        .build()
);

// Heading lock: auto-holds heading when driver isn't rotating
drive.headingLockDrive(
    () -> -controller.getLeftY(),
    () -> -controller.getLeftX(),
    () -> -controller.getRightX(),
    0.05
);

// Point at scoring target while driving
drive.pointAtTarget(
    () -> -controller.getLeftY(),
    () -> -controller.getLeftX(),
    () -> new Translation2d(0.0, 5.55), // speaker position
    0.05
);
```

---

## Vision

Multi-camera pose estimation with Kalman filter tuning.

```java
VisionSubsystem vision = new VisionSubsystem(
    drive::addVisionMeasurement,
    drive::getHeading,
    drive::getChassisSpeeds,
    VisionConfig.builder()
        .addLimelight("limelight-front", new Pose3d(...))
        .addPhotonCamera("cam-rear", new Pose3d(...))
        .singleTagStdDevs(4, 8)
        .multiTagStdDevs(0.5, 1)
        .xyDistanceScaling(1.0)
        .rejectDuringSpin(2.0) // rad/s
        .maxLatency(0.5)
        .build()
);
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
| `SuperstructureCoordinator` | Multi-mechanism state machine with safe transitions |
| `InterpolatingTable` | TreeMap-based linear interpolation (shooter distance tables) |
| `CatalystMath` | Joystick curves, angle math, geometry helpers, physics |
| `SlewRateLimiter` | Asymmetric rate limiter (different accel/decel profiles) |
| `MovingAverage` | Sliding window average filter |
| `TimedBoolean` | Debounced boolean with rising/falling edge detection |

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

FrcCatalyst includes a comprehensive test project at [FrcCatalystTest](https://github.com/TomAs-1226/FrcCatalystTest) that validates all library components:

- **68 JUnit tests** covering utilities, math helpers, hardware types, and mechanism construction
- **Simulation tests** for all mechanism types (elevator, arm, flywheel, roller, winch)
- **SuperstructureCoordinator** state machine integration tests

```bash
# Run unit tests (utilities, math, hardware)
./gradlew test

# Run all tests including mechanism simulation
./gradlew testAll
```

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

## License

This project is available under the [MIT License](LICENSE).

---

<p align="center">
  <sub>Built for the FRC community. Go build something awesome.</sub>
</p>
