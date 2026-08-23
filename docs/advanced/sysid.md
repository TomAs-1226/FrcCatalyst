---
layout: default
title: SysId
parent: Advanced
nav_order: 4
---

# SysId
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Overview

Every Catalyst mechanism gets WPILib SysId for free. v0.4.0 wires the
WPILib `SysIdRoutine` into `CatalystMotor` and exposes zero-argument
helpers on `CatalystMechanism` so each mechanism's primary motor is
characterizable with a single command bind.

The data capture goes through Phoenix-6's `SignalLogger` — there's no
per-signal logging boilerplate to write. Teams just turn the logger on
once at robot init and bind the routine to a button.

---

## Setup

One line in `Robot.robotInit()` is everything the library needs:

```java
import com.ctre.phoenix6.SignalLogger;

@Override
public void robotInit() {
    SignalLogger.start();          // start capturing signals to .hoot
    container = new RobotContainer();
}
```

By default `SignalLogger` writes a `.hoot` file to the USB stick mounted
on Systemcore (or to `/home/systemcore/logs/` if no USB is present).

---

## Binding the routines

Each mechanism exposes both routines as Commands. Bind them to buttons
that the driver only uses in characterization sessions:

```java
import frc.lib.catalyst.sysid.SysIdRoutine.Direction;

// Elevator
driver.povUp()    .whileTrue(elevator.sysIdQuasistatic(Direction.FORWARD));
driver.povDown()  .whileTrue(elevator.sysIdQuasistatic(Direction.REVERSE));
driver.povRight() .whileTrue(elevator.sysIdDynamic(Direction.FORWARD));
driver.povLeft()  .whileTrue(elevator.sysIdDynamic(Direction.REVERSE));
```

Run each routine until the mechanism reaches its soft limit (or you
back off the button). Repeat for each mechanism you want to
characterize.

> ⚠️ **The SysId routines drive the motor with voltage and bypass every
> closed-loop controller.** Make sure you have room above and below
> the current position, and use the soft limits in your config — those
> still apply.

---

## Pulling the data into WPILib SysId

The Phoenix tools ship a converter that turns the `.hoot` file into
the WPILib SysId JSON format:

```
java -jar phoenix-tools.jar hoot2sysid <input>.hoot <output>.json
```

Then open WPILib SysId, load the JSON, and read off the gains. Drop
those into your Catalyst config:

```java
LinearMechanism.Config.builder()
    // ...
    .feedforward(kS, kV, kA)
    .gravityGain(kG)
    .pid(kP, 0, kD)
    // ...
    .build();
```

Re-deploy and you have a properly characterized mechanism.

---

## Custom routines

The defaults are 1 V/s ramp, 4 V dynamic step, 10 s timeout. Override
when you need to:

```java
import frc.lib.catalyst.sysid.SysIdRoutine;
import org.wpilib.units.Units;

SysIdRoutine.Config slowRoutine = new SysIdRoutine.Config(
    Units.Volts.of(0.5).per(Units.Seconds.of(1)),  // 0.5 V/s ramp
    Units.Volts.of(3.0),                            // 3 V dynamic step
    Units.Seconds.of(8.0),                          // 8 s timeout
    state -> SignalLogger.writeString("State", state.toString()));

SysIdRoutine routine = elevator.getMotor().sysIdRoutine(elevator, slowRoutine);

driver.povUp().whileTrue(routine.quasistatic(Direction.FORWARD));
```

Useful when the default 4 V step would slam the mechanism into a hard
stop too fast.

---

## Mechanisms that work out of the box

| Mechanism | Primary motor used for SysId |
|---|---|
| `LinearMechanism`        | Master + every follower |
| `RotationalMechanism`    | Master + every follower |
| `FlywheelMechanism`      | Primary wheel + its followers |
| `RollerMechanism`        | Master + every follower |
| `WinchMechanism`         | First motor only (use the underlying second motor's routine separately for dual-arm climbers) |
| `ClawMechanism`          | Master + every follower |
| `DifferentialWristMechanism` | Override required — diff control means each motor needs separate characterization |
| `PneumaticMechanism`     | N/A — no continuous actuator |

For mechanisms not in the table (your own subclass of
`CatalystMechanism`), override `primaryMotorForSysId()` to return the
motor you want characterized:

```java
@Override
protected CatalystMotor primaryMotorForSysId() {
    return master;
}
```

---

## Workflow tips

- **Characterize once per season.** Gains drift with belt wear and
  battery age but not enough to matter mid-season.
- **Disable health checks during the run.** The over-current and
  over-temp checks may trip during the dynamic step. Either temporarily
  raise the limits or expect a clean firing/clearing log.
- **Keep `TunableNumber.disableTuning()` off** during characterization
  — the live PID tuner can't hurt anything, the SysId routine doesn't
  use PID at all.
- **Use a marker button.** Have an operator hit a separate button to
  call `SignalLogger.writeString("Marker", "run-start")` so you can find
  each run easily when you split the data.
