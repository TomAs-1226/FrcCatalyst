---
layout: default
title: Home
nav_order: 1
permalink: /
---

# FrcCatalyst
{: .fs-9 }

Pre-built, configurable mechanism building blocks for FRC robots using CTRE Phoenix 6 and WPILib 2026.
{: .fs-6 .fw-300 }

[Get Started](getting-started/installation){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 }
[View on GitHub](https://github.com/TomAs-1226/FrcCatalyst){: .btn .fs-5 .mb-4 .mb-md-0 }

---

<p>
  <img src="https://img.shields.io/badge/WPILib-2026.2.1-green?style=flat-square" alt="WPILib"/>
  <img src="https://img.shields.io/badge/Phoenix%206-26.1.1-orange?style=flat-square" alt="Phoenix 6"/>
  <img src="https://img.shields.io/badge/Java-17-blue?style=flat-square&logo=openjdk" alt="Java 17"/>
  <img src="https://img.shields.io/badge/PathPlanner-2026.1.2-purple?style=flat-square" alt="PathPlanner"/>
  <img src="https://img.shields.io/badge/PhotonVision-v2026.3.1-yellow?style=flat-square" alt="PhotonVision"/>
</p>

## Why FrcCatalyst?

Writing robot code from scratch every season means re-implementing the same elevator, arm, intake, and swerve patterns. FrcCatalyst gives you battle-tested implementations with one builder call:

```java
LinearMechanism elevator = new LinearMechanism(
    LinearMechanism.Config.builder()
        .name("Elevator").motor(13).follower(14, true)
        .motorType(MotorType.KRAKEN_X60)
        .gearRatio(10.0).drumRadius(0.0254).mass(5.0)
        .pid(50, 0, 0.5).gravityGain(0.35)
        .motionMagic(2.0, 4.0, 20.0)
        .position("STOW", 0.0).position("HIGH", 1.1)
        .build()
);
```

You get Motion Magic control, gravity compensation, simulation, telemetry, safety limits, and command factories — all for free.

| Feature | Raw WPILib/Phoenix | FrcCatalyst |
|---------|-------------------|-------------|
| Elevator with gravity FF | ~150 lines | **8 lines** |
| Swerve + PathPlanner + Vision | ~400 lines | **15 lines** |
| Mechanism with sim + telemetry | Build it yourself | **Built-in** |
| Safe temperature cutoffs | Manual | **Automatic** |
| Limit switch auto-zeroing | Manual wiring | **One builder call** |

---

## What's Included

### Mechanisms

Pre-built, configurable mechanism types that cover virtually every FRC subsystem.

| Mechanism | Use Case | Key Features |
|-----------|----------|--------------|
| **LinearMechanism** | Elevators, slides | Position control, gravity FF, limit switches |
| **RotationalMechanism** | Arms, wrists, turrets | Cosine gravity, hard stops, Motion Magic |
| **FlywheelMechanism** | Shooters | Dual motor, velocity PID, at-speed trigger |
| **RollerMechanism** | Intakes, conveyors | Stall detection, beam break, auto-stop |
| **WinchMechanism** | Climbers | Extend/retract limits, position tracking |
| **SuperstructureCoordinator** | Multi-mechanism | State machine with safe transitions |

### Subsystems

Complex subsystem wrappers that integrate multiple components.

- **SwerveSubsystem** — CTRE Tuner X wrapper with heading lock, point-at-target, skew correction, advanced drive, PathPlanner
- **VisionSubsystem** — Multi-camera Kalman filter with innovation tracking, high-speed rejection, heading divergence filtering
- **LEDSubsystem** — 14 addressable LED patterns including fire, gradient, larson scanner, alignment indicator

### Advanced Features (New)

Competition-proven algorithms used by top FRC teams:

| Feature | Inspired By | Description |
|---------|------------|-------------|
| **StateSpaceController** | 6328, 254 | LQR + Kalman filter for optimal mechanism control |
| **MotionConstraintCalculator** | Top teams | Physics-based max velocity/acceleration from motor specs |
| **SignalProcessor** | 254 | EMA, median, low-pass, composite sensor filters |
| **PoseHistory** | 6328 | Temporal pose tracking with interpolation |
| **DynamicAutoBuilder** | 254, 3061 | Runtime path generation with PathPlanner |
| **Skew Correction** | 1690 | Pose exponential discretization for swerve |
| **Collision Zones** | 254 | Prevent physical mechanism collisions |

### Every Mechanism Includes

- Builder-pattern configuration with sensible defaults
- Motion Magic position control (TalonFX) or ProfiledPID (roboRIO)
- Named position presets (`goTo("STOW")`)
- Built-in simulation with accurate motor models
- Automatic NetworkTables telemetry
- Temperature cutoff and limit switch safety
- Pre-built command factories

---

## Documentation

| Section | Description |
|---------|-------------|
| [Installation](getting-started/installation) | Add FrcCatalyst to your project |
| [Quick Start](getting-started/quickstart) | Build your first mechanism in 5 minutes |
| [Mechanisms](mechanisms/) | LinearMechanism, RotationalMechanism, Flywheel, Roller, Winch |
| [Subsystems](subsystems/) | Swerve Drive, Vision, LEDs |
| [Utilities](utilities/) | Math, feedforward, profiles, alerts |
| [Advanced](advanced/) | State-space control, signal processing, dynamic paths |
| [Examples](examples/) | Complete robot examples with elevator, intake, and more |
| [Testing](testing/) | How to test your FrcCatalyst-based code |

---

## Compatibility

| Component | Version |
|-----------|---------|
| WPILib | 2026.2.1 |
| CTRE Phoenix 6 | 26.1.1 |
| PhotonVision | v2026.3.1 |
| PathPlanner | 2026.1.2 |
| Java | 17+ |
