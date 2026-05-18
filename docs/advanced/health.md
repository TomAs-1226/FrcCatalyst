---
layout: default
title: Health Monitoring
parent: Advanced
nav_order: 6
---

# Health Monitoring
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Overview

Catalyst v0.3.3 ships a built-in fault layer. Every Catalyst mechanism
registers a small set of `HealthCheck`s in its constructor — over-current,
high-temperature, over-temperature cutoff, stall, not-zeroed, low-pressure
— and `HealthMonitor` evaluates them once per loop.

You don't need to write any wiring. Declare the mechanism, deploy, and
firing checks show up in three places:

1. **NetworkTables**, under `/Catalyst/Health/<subsystem>/<id>/...` — so any
   dashboard can subscribe.
2. **The Health Dashboard** at `docs/tools/health/index.html` — single-file
   web viewer with severity-colored cards and filters.
3. **The existing `AlertManager`** — so dashboards already wired against
   `/Catalyst/Alerts/...` keep working unchanged.

Teams add their own checks in one fluent call.

## The standard motor checks

Every motor-driven mechanism gets these for free:

| ID | Severity | Fires when | Debounce | Action |
|---|---|---|---|---|
| `OverCurrent` | WARN | stator current > 90% of configured limit | 0.5 s | log + NT |
| `HighTemp` | WARN | motor temp > configured warn threshold | 1.0 s | log + NT |
| `OverTemp` | ERROR | motor temp > warn + 10 °C | immediate | **calls `motor.stop()`** |

Multi-motor mechanisms (Flywheel secondary, Winch second, Claw follower,
DifferentialWrist left/right) register the same checks per motor with
collision-free id suffixes — `OverCurrentSec`, `HighTempLeft`, etc.

Mechanism-specific checks layered on top:

- `LinearMechanism` + `RotationalMechanism`: `Stall` (WARN) and `NotZeroed` (INFO).
- `FlywheelMechanism`: `NotSpinningUp` (WARN) — non-zero setpoint but no velocity.
- `PneumaticMechanism`: `LowPressure` (WARN) when a compressor is wired and
  `requirePressureAbove(psi)` is configured. Below the threshold the mechanism
  also **refuses to actuate forward** to avoid firing a piston dry.

## What gets published

For each registered check:

```
/Catalyst/Health/<subsystem>/<id>/firing       boolean
/Catalyst/Health/<subsystem>/<id>/severity     string (INFO|WARN|ERROR)
/Catalyst/Health/<subsystem>/<id>/description  string (static)
/Catalyst/Health/<subsystem>/<id>/detail       string (live — e.g. "92 C")
/Catalyst/Health/<subsystem>/<id>/firedAt      double (FPGA timestamp)
```

Plus a rollup at the top of `/Catalyst/Health/`:

```
ErrorCount, WarnCount, InfoCount   integer
Healthy                            boolean (true iff zero errors and zero warns)
```

## Adding your own check

A check has a name, severity, predicate, optional live detail string, and
debounce/clear timing. Register it once at construction time — typically
in the subsystem that owns the sensor:

```java
HealthCheck.builder("Climber", "RopeSlack")
    .severity(HealthCheck.Severity.WARN)
    .description("Climber rope slack detected")
    .when(() -> ropeTensionSensor.getNewtons() < 5)
    .detail(() -> String.format("%.0f N", ropeTensionSensor.getNewtons()))
    .debounce(0.5)      // must stay below 5N for 0.5s before firing
    .clearAfter(2.0)    // must stay above 5N for 2s to clear
    .onFire(() -> driver.rumble(0.5))
    .onClear(() -> driver.rumble(0))
    .register();
```

The predicate runs every loop. Buggy lambdas don't take down the monitor
— exceptions are caught and the check is treated as "not firing" for that
tick.

## Workflow

1. **At robot init**, Catalyst's built-in mechanisms register their checks.
   Anything you add fires off in your subsystem constructors.
2. **Each loop**, `HealthMonitor.update()` is called by every mechanism's
   `updateTelemetry()`. It's throttled to a 5 ms minimum so all eight
   mechanisms calling it cost one evaluation per scheduler tick.
3. **On a fire/clear edge**, NT entries update, `AlertManager` is poked,
   and `onFire` / `onClear` hooks run.
4. **Open the Health Dashboard** during practice/test to see what's hot.
   In a match, the same data is on whatever NT dashboard your team uses.

## Cross-mechanism safety: `RobotSafety` (v0.3.5+)

`HealthMonitor` reports per-check state; `RobotSafety` is the higher-level
watchdog that decides "the robot has too many simultaneous faults — stop
doing things." It's opt-in: when not configured, it adds zero overhead.

```java
public void robotInit() {
    RobotSafety.configure(
        RobotSafety.Config.builder()
            .maxConcurrentErrors(2)   // trip when 2+ ERROR checks fire at once
            .debounce(0.25)           // ...sustained for at least 0.25 s
            .onTrip(() -> {
                drive.stop();
                superstructure.stow();
                leds.fire();
            })
            .build());
}
```

Then read the signal from anywhere:

```java
new Trigger(RobotSafety::isTripped)
    .onTrue(Commands.runOnce(drive::stop, drive));
```

The watchdog publishes to `/Catalyst/Safety/{Tripped,Reason,ErrorCount,WarnCount}`
so the Health Dashboard and any other NT viewer light up when it fires.
Call `RobotSafety.reset()` to manually clear once you've intervened, or
configure `.autoReset(seconds)` to clear automatically once the underlying
checks have stayed clear.

The library never forcibly disables motors itself — the trip is advisory.
Each team decides what "all-stop" means for their robot.

## Severity rules of thumb

- **ERROR** → something is broken or about to be broken. The robot should
  stop the affected mechanism (use `onFire(motor::stop)` like the built-in
  `OverTemp` check does).
- **WARN** → degraded but operational. Driver should know, but the
  mechanism still runs.
- **INFO** → notable but expected. The default `NotZeroed` check is INFO
  because it's normal until the team zeroes the mechanism.

## Disabling specific checks

There's no global mute, by design — you shouldn't ship a robot to a
match with checks turned off. If you really need to suppress one in
sim, gate the registration:

```java
if (!Robot.isSimulation()) {
    HealthCheck.builder(...).register();
}
```

## Performance

Each `HealthCheck.evaluate()` runs the predicate once, optionally formats
a detail string, and updates two timestamps. No allocations on the steady
path. With eight built-in mechanisms × ~3 checks each = 24 evaluations per
loop, plus whatever teams add. At 50 Hz this is well under 1 ms total even
on a roboRIO 2.
