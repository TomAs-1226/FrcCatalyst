---
layout: default
title: Live Tuning
parent: Advanced
nav_order: 5
---

# Live Tuning
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Overview

Catalyst v0.3.2 makes every mechanism's **PID and Motion Magic gains
live-tunable from the dashboard by default**. No extra code in your robot
project — declare the mechanism, deploy, and the gains show up under
`Catalyst/Tuning/<MechanismName>/...` on NetworkTables.

This is the same pattern most top-tier teams roll by hand each season: a
`TunableNumber` wrapper that reads from NT when tuning is enabled and falls
back to a cached default during competition. Catalyst just bakes it in so you
don't have to write the wiring yourself.

## Which mechanisms support it?

| Mechanism | PID + FF | Motion Magic |
|---|:---:|:---:|
| `LinearMechanism` | ✓ | ✓ |
| `RotationalMechanism` | ✓ | ✓ |
| `FlywheelMechanism` | ✓ | — |
| `DifferentialWristMechanism` | ✓ | ✓ |

`RollerMechanism`, `ClawMechanism`, `WinchMechanism`, and `PneumaticMechanism`
don't run closed-loop position/velocity control, so there's nothing to tune.

## What gets published

For every supported mechanism, the following NetworkTables entries appear under
`/Catalyst/Tuning/<MechanismName>/`:

```
kP, kI, kD                          (PID terms)
kS, kV, kA, kG                      (feedforward terms; kG only used if you set it)
MM/CruiseVelocity                   (Motion Magic only)
MM/Acceleration
MM/Jerk
```

Initial values come from your `Config` builder. Edit any of them from
AdvantageScope / Elastic / Glass / Shuffleboard and the change is reflected on
the next robot loop — no redeploy, no restart.

## Workflow

1. Declare the mechanism as you always would:

    ```java
    RotationalMechanism arm = new RotationalMechanism(
        RotationalMechanism.Config.builder()
            .name("Arm")
            .motor(15)
            .gearRatio(50.0)
            .pid(80, 0, 1.0)
            .feedforward(0.2, 0.12, 0.0)
            .gravityGain(0.4)
            .motionMagic(200, 400, 2000)
            .currentLimit(30)
            .build());
    ```

2. Deploy and enable the robot.

3. Open your dashboard and find the entries:

    ```
    /Catalyst/Tuning/Arm/kP            80.0
    /Catalyst/Tuning/Arm/kI             0.0
    /Catalyst/Tuning/Arm/kD             1.0
    /Catalyst/Tuning/Arm/MM/Acceleration  400.0
    ...
    ```

4. Edit a value. The mechanism re-applies the new gain via
   `motor.getConfigurator().apply(...)` on the next periodic.

## Disabling for competition

During practice/development, live tuning is great. During competition, every
`get()` call reads NT — that's still cheap, but you'd rather not have unrelated
people poking gains mid-match. Lock everything down with one call in
`robotInit()`:

```java
import frc.lib.catalyst.util.TunableNumber;

public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        if (DriverStation.getMatchType() != DriverStation.MatchType.None) {
            TunableNumber.disableTuning();
        }
        // ... rest of init
    }
}
```

After `disableTuning()`:

- Every `TunableNumber.get()` returns its cached default in constant time.
- Every `hasChanged()` returns false → no `apply()` calls fire on the motor.
- The NT entries still exist (created at construction) but are never read.

You can also flip the switch from a button binding for testing:

```java
controller.back().onTrue(Commands.runOnce(TunableNumber::disableTuning));
controller.start().onTrue(Commands.runOnce(TunableNumber::enableTuning));
```

## How it works under the hood

Each supported mechanism owns a `TunableGains` instance built from its
`Config`. On every periodic, the mechanism calls
`tunableGains.checkAndApply(motor)`, which:

1. Polls each gain's NT entry (skipped entirely when tuning is disabled).
2. If any Slot-0 value changed, calls `motor.updateSlot0(kP, kI, kD, kS, kV, kA, kG)`.
3. If any Motion Magic value changed, calls `motor.updateMotionMagic(cruise, accel, jerk)`.

The motor's gravity model (cosine for arms, static for elevators) is preserved
across re-applies — you set it once in the builder, it sticks.

Reads are cached and the change-detection is per-gain (uses
`TunableNumber.hasChanged()`), so the steady-state cost is ~10 floating-point
comparisons per mechanism per loop. Negligible.

## Tuning your own custom mechanism

If you write your own `CatalystMechanism` subclass (e.g., a custom turret), use
the same helper:

```java
import frc.lib.catalyst.util.TunableGains;

private final TunableGains gains = new TunableGains(
    "MyTurret",
    /* kP */ 50, /* kI */ 0, /* kD */ 0,
    /* kS */ 0,  /* kV */ 0, /* kA */ 0, /* kG */ 0,
    /* mmCruise */ 50, /* mmAccel */ 100, /* mmJerk */ 0);

@Override
protected void updateTelemetry() {
    gains.checkAndApply(motor);
    // ... your existing telemetry
}
```

That's it — one field, one line in `updateTelemetry`, and your turret's gains
are live-tunable.
