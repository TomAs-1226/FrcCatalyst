---
layout: default
title: Mechanisms
nav_order: 4
has_children: true
---

# Mechanisms

FrcCatalyst provides five generic mechanism types that cover virtually every FRC subsystem. Each mechanism extends `CatalystMechanism` (which extends WPILib's `SubsystemBase`) and provides:

- **Builder-pattern configuration** with validation and sensible defaults
- **Two control modes**: CTRE Motion Magic (on TalonFX) or WPILib ProfiledPID (on roboRIO)
- **Named position presets** for quick `goTo("STOW")` commands
- **Gravity compensation** (constant for elevator, cosine for arm)
- **Built-in simulation** using accurate WPILib DCMotor models
- **Automatic telemetry** published to NetworkTables under `Catalyst/<name>/`
- **Safety features**: temperature cutoff, limit switch auto-zeroing, soft limits
- **Pre-built command factories**: goTo, goToAndWait, holdPosition, jog, zero

## Mechanism Types

| Mechanism | Use Case | Position Unit | Control |
|-----------|----------|---------------|---------|
| [LinearMechanism](linear) | Elevators, slides, telescoping arms | Meters | Motion Magic + Gravity FF |
| [RotationalMechanism](rotational) | Arms, wrists, turrets, hoods | Degrees | Motion Magic + Cosine Gravity |
| [FlywheelMechanism](flywheel) | Shooters, accelerator wheels | RPS (velocity) | Velocity PID |
| [RollerMechanism](roller) | Intakes, conveyors, indexers | N/A (duty cycle) | Open-loop + detection |
| [WinchMechanism](winch) | Climbers, deployments | Meters | Duty cycle + limits |

## SuperstructureCoordinator

The `SuperstructureCoordinator` orchestrates multiple mechanisms into a state machine with safe transitions. Define named states with positions for each mechanism, and optional custom transition rules:

```java
SuperstructureCoordinator superstructure = new SuperstructureCoordinator()
    .withLinear("elevator", elevator)
    .withRotational("arm", arm);

superstructure.defineState("STOW")
    .setLinear("elevator", 0.0)
    .setRotational("arm", 0.0)
    .done();

superstructure.defineState("SCORE_HIGH")
    .setLinear("elevator", 1.1)
    .setRotational("arm", 95.0)
    .done();

// Custom transition: retract arm before raising elevator
superstructure.addTransitionRule("STOW", "SCORE_HIGH",
    (fromState, toState) -> elevator.goTo("HIGH")
        .alongWith(arm.goTo("STOW"))
        .andThen(arm.goTo("SCORE"))
);

// Use in commands:
operatorController.y().onTrue(superstructure.transitionTo("SCORE_HIGH"));
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
