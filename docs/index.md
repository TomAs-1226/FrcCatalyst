---
layout: default
title: Home
nav_order: 1
---

# FrcCatalyst Documentation

<p>
  <img src="https://img.shields.io/badge/WPILib-2026.2.1-green?style=flat-square" alt="WPILib"/>
  <img src="https://img.shields.io/badge/Phoenix%206-26.1.1-orange?style=flat-square" alt="Phoenix 6"/>
  <img src="https://img.shields.io/badge/Java-17-blue?style=flat-square&logo=openjdk" alt="Java 17"/>
</p>

**FrcCatalyst** is a production-ready Java library that provides pre-built, configurable mechanism building blocks for FRC robots using CTRE Phoenix 6 hardware and WPILib 2026.

## Why FrcCatalyst?

Writing robot code from scratch every season means re-implementing the same elevator, arm, intake, and swerve patterns. FrcCatalyst gives you battle-tested implementations with one builder call:

```java
LinearMechanism elevator = new LinearMechanism(
    LinearMechanism.Config.builder()
        .name("Elevator").motor(13).follower(14, true)
        .gearRatio(10.0).drumRadius(0.0254).mass(5.0)
        .pid(50, 0, 0.5).gravityGain(0.35)
        .motionMagic(2.0, 4.0, 20.0)
        .position("STOW", 0.0).position("HIGH", 1.1)
        .build()
);
```

You get Motion Magic control, gravity compensation, simulation, telemetry, safety limits, and command factories for free.

## Documentation

| Section | Description |
|---------|-------------|
| [Installation](getting-started/installation) | Add FrcCatalyst to your project |
| [Quick Start](getting-started/quickstart) | Build your first mechanism in 5 minutes |
| [Mechanisms](mechanisms/) | LinearMechanism, RotationalMechanism, Flywheel, Roller, Winch |
| [Subsystems](subsystems/) | Swerve Drive, Vision, LEDs |
| [Utilities](utilities/) | Math, feedforward, profiles, alerts |

## Feature Overview

### Mechanisms
- **LinearMechanism** - Elevators, linear slides, telescoping arms
- **RotationalMechanism** - Arms, wrists, turrets, hoods
- **FlywheelMechanism** - Shooters with dual-motor differential spin
- **RollerMechanism** - Intakes with stall detection and beam break
- **WinchMechanism** - Climbers with position limits
- **SuperstructureCoordinator** - Multi-mechanism state machine

### Subsystems
- **SwerveSubsystem** - CTRE swerve wrapper with PathPlanner + heading lock
- **VisionSubsystem** - Multi-camera Kalman filter pose estimation
- **LEDSubsystem** - Addressable LED patterns

### Every Mechanism Includes
- Builder-pattern configuration with sensible defaults
- Motion Magic position control (on TalonFX)
- WPILib ProfiledPID alternative (on roboRIO)
- Named position presets
- Built-in simulation with accurate motor models
- Automatic NetworkTables telemetry
- Temperature cutoff safety
- Limit switch support with auto-zeroing
- Pre-built command factories

## Compatibility

| Component | Version |
|-----------|---------|
| WPILib | 2026.2.1 |
| CTRE Phoenix 6 | 26.1.1 |
| PhotonVision | v2026.3.1 |
| PathPlanner | 2026.1.2 |
| Java | 17+ |
