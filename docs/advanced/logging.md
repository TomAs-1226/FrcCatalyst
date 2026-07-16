---
layout: default
title: Logging & AdvantageKit Bridge
parent: Advanced
nav_order: 4
---

# Logging & AdvantageKit Bridge
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Overview

Catalyst v0.3 introduces an **in-house logging core** that routes all
mechanism telemetry through a single pluggable sink. By default everything
keeps publishing to NetworkTables exactly the way it did in v0.2, so v0.2
dashboards work unchanged. But the new architecture lets teams plug in
**AdvantageKit, DataLog, Elastic, or any custom logger** with about ten lines
of code — and without Catalyst itself depending on any of those projects.

The goal is **interoperability without entanglement**: Catalyst doesn't bundle
another team's library into your robot code, but it doesn't lock you out of
one either.

## Architecture

```
┌────────────────────┐   processInputs()   ┌─────────────┐    log calls    ┌──────────┐
│  Mechanism         │ ──────────────────► │ CatalystLog │ ──────────────► │   Sink   │
│  (Linear, Roller…) │                     │  (facade)   │                 │          │
└────────────────────┘                     └─────────────┘                 └────┬─────┘
                                                                                │
                                              ┌─────────────────────────────────┴───────────────────┐
                                              │                                                     │
                                       ┌──────▼──────────┐                                  ┌───────▼───────────┐
                                       │ NetworkTables   │  (default — v0.2 behavior)       │ Your AK bridge    │
                                       │ Sink            │                                  │ (10 lines, your   │
                                       └─────────────────┘                                  │ code)             │
                                                                                            └───────────────────┘
```

Three core pieces live in `frc.lib.catalyst.logging`:

| Class | Purpose |
|---|---|
| `CatalystInputs` | Interface implemented by every IO-layer Inputs POJO. Has `toLog(LogTable)` and `fromLog(LogTable)`. |
| `LogSink` | Interface implemented by anything that wants to receive telemetry. Has typed `log(...)` methods plus `processInputs(...)`. |
| `CatalystLog` | Static facade. Holds the active sink and forwards all calls to it. |

Mechanisms call `CatalystLog.log(key, value)` and `CatalystLog.processInputs(prefix, inputs)`
under the hood — they never know which sink is wired in.

## Default behavior

You don't have to do anything. On startup, `CatalystLog` is wired to a
`NetworkTablesSink` rooted at `/Catalyst/<MechanismName>/...`. Every Catalyst
mechanism publishes its per-loop state through that root. Dashboards built
against v0.2 keep working.

## Logging struct types (poses, module states)

Beyond primitives and arrays, `CatalystLog` logs any WPILib struct-serializable
type through its `Struct` descriptor, so AdvantageScope renders it as a real 2D
or 3D object instead of loose numbers:

```java
CatalystLog.log("Drive/Pose", Pose2d.struct, drive.getPose());
CatalystLog.log("Drive/ModuleStates", SwerveModuleState.struct, drive.getModuleStates());
```

Both a scalar (`log(key, struct, value)`) and an array
(`log(key, struct, value[])`) overload are available. They work through every
sink: the `NetworkTablesSink` publishes them with cached `StructPublisher` /
`StructArrayPublisher`, the `WpilogSink` records them as `StructLogEntry` /
`StructArrayLogEntry`, and `CompoundSink` forwards to both. `SwerveSubsystem`
uses this internally to publish `/Catalyst/Swerve/ModuleStates` and
`/ModuleTargets` for the AdvantageScope swerve view.

To cut NetworkTables traffic under loop overrun, gate the per-mechanism input
snapshots with `CatalystLog.enableLoggingInputs(false)` (outputs still log).

## Bridging to AdvantageKit

AdvantageKit's `Logger.recordOutput(...)` is the public API for getting custom
keys into AK's `.wpilog` recorder. To route all Catalyst telemetry through AK,
write a tiny sink and install it at robot init:

```java
// In your robot project — NOT in Catalyst.
import frc.lib.catalyst.logging.CatalystInputs;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.logging.LogSink;
import org.littletonrobotics.junction.Logger;

public class AdvantageKitSink implements LogSink {
    @Override public void log(String key, double v)     { Logger.recordOutput(key, v); }
    @Override public void log(String key, boolean v)    { Logger.recordOutput(key, v); }
    @Override public void log(String key, long v)       { Logger.recordOutput(key, v); }
    @Override public void log(String key, String v)     { Logger.recordOutput(key, v); }
    @Override public void log(String key, double[] v)   { Logger.recordOutput(key, v); }
    @Override public void log(String key, boolean[] v)  { Logger.recordOutput(key, v); }
    @Override public void log(String key, long[] v)     { Logger.recordOutput(key, v); }
    @Override public void log(String key, String[] v)   { Logger.recordOutput(key, v); }

    @Override
    public void processInputs(String prefix, CatalystInputs inputs) {
        // AdvantageKit has its own LoggableInputs contract; wrap and forward.
        Logger.processInputs(prefix, new org.littletonrobotics.junction.inputs.LoggableInputs() {
            @Override public void toLog(org.littletonrobotics.junction.LogTable t)   { /* call inputs.toLog */ }
            @Override public void fromLog(org.littletonrobotics.junction.LogTable t) { /* call inputs.fromLog */ }
        });
    }
}

// In Robot.robotInit():
CatalystLog.setSink(new AdvantageKitSink());
```

> **Why no built-in bridge?** Bundling AdvantageKit into Catalyst would force
> every team that uses Catalyst to also pull in AK's transitive dependencies.
> The library stays narrow on purpose; the bridge lives in your code where
> you can adapt it to your AK version.

## Recording to WPILOG (built-in)

`WpilogSink` (v0.8.0+) records everything Catalyst logs to a standard
`.wpilog` — opens directly in **AdvantageScope**, the DataLogTool, or any
WPILOG reader. No extra vendordep; DataLog ships with WPILib.

```java
public void robotInit() {
    // Records to /U/logs (USB) or ~/logs, also captures NT + DS data,
    // and keeps mirroring to NetworkTables so live dashboards still work.
    CatalystLog.setSink(new WpilogSink());
}
```

This is the lightweight **record** half of replay-style debugging — a
complete, timestamped, scrubable log of every mechanism's inputs and outputs.
For full deterministic **replay** (re-run modified code against the log),
forward into AdvantageKit's `Logger` instead — the `CatalystInputs`
`toLog`/`fromLog` layer is built for exactly that. See the bridge above.

## Fan-out to multiple sinks

To log to NT **and** AK simultaneously during the season (NT for live driver
station, AK `.wpilog` for replay analysis), use `CompoundSink`:

```java
import frc.lib.catalyst.logging.CompoundSink;
import frc.lib.catalyst.logging.NetworkTablesSink;

CatalystLog.setSink(new CompoundSink(
    new NetworkTablesSink(),
    new AdvantageKitSink()
));
```

## Replay-shaped data without AK

The Inputs POJOs (`LinearMechanismInputs`, `RotationalMechanismInputs`, …) are
already shaped like AK's `LoggableInputs` — they implement `CatalystInputs`
with symmetric `toLog`/`fromLog` methods. Even without AK, you can use them
for:

- **Unit testing**: drive a mechanism from a recorded Inputs sequence
- **Reproducing a competition issue**: dump Inputs to a JSON file, replay later
- **Bench validation**: capture good-state Inputs as a fixture

The v0.4 release will ship default Phoenix-6 IO implementations that fill
these Inputs from real hardware (and matching sim implementations for replay),
making the contract production-ready. v0.3 ships the contract and the
mechanism-side wiring; the swappable IO is the next milestone.

## What `processInputs` actually does

When a mechanism calls `processInputs(inputs)`:

1. `CatalystLog` builds a fresh `LogTable` and asks the POJO to serialize itself via `toLog`.
2. For each typed entry in the table, the sink's matching `log(prefix + "/" + key, value)` is called.

So every individual field still lands as a discrete NT/AK/DataLog key — no
opaque blob. Dashboards built against v0.2's per-key NT layout keep working
because the keys are unchanged.
