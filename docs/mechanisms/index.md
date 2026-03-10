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
    .linearPosition("elevator", 0.0)
    .rotationalPosition("arm", 0.0)
    .build();

superstructure.defineState("SCORE_HIGH")
    .linearPosition("elevator", 1.1)
    .rotationalPosition("arm", 95.0)
    .build();

// Custom transition: retract arm before raising elevator
superstructure.addTransitionRule("STOW", "SCORE_HIGH",
    (coordinator) -> elevator.goTo("HIGH")
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
