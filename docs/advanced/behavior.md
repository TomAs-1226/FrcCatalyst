---
layout: default
title: Behavior Framework
parent: Advanced
nav_order: 6
---

# Behavior Framework
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## What it is

A small, game-agnostic orchestration layer that turns your robot's
capabilities into autonomous and assisted behaviors. Three pieces, one
building block:

| Class | Role |
|---|---|
| `Action` | One atomic capability — a command plus a precondition, success test, and cost |
| `BehaviorEngine` | Reactive sequencer — runs actions in order, falls back when one can't start, bails on a deadline |
| `Strategist` | Utility selector — each loop, runs the highest-scoring eligible action ("what's best *now*") |
| `Autopilot` | Teleop cycle co-pilot — hold a button, the robot runs acquire → score → repeat |

Nothing here knows about a specific game. The framework sequences and
reacts; your `Action`s hold the game-specific work. The examples below use
the 2026 *Reefscape* fuel game for flavor, but swap them for any game and
the framework is unchanged.

> **Why not a search planner?** FRC has a tiny action space, a mostly-known
> field, no time to "think," and zero live debuggability. A planner that
> picks a surprising action loses you the match. This framework is reactive
> and inspectable by design — every decision is visible on NetworkTables.

---

## Action — the building block

```java
Action chaseNearestFuel = Action.named("ChaseNearestFuel")
    .when(() -> vision.hasFuelTarget())                          // precondition (team-side detection)
    .run(() -> drive.pathfindToPose(() -> vision.nearestFuelPose())  // real Catalyst command
                    .andThen(intake.intakeUntilPiece()))
    .until(intake::hasFuel)                                      // success
    .estimatedSeconds(2.5)
    .requires(drive, intake)                                    // for the orchestrators
    .build();
```

> `vision.hasFuelTarget()` / `vision.nearestFuelPose()` are team-side
> methods backed by your detection coprocessor — Catalyst doesn't ship
> object detection. `drive.pathfindToPose(...)` is the real Catalyst
> command that drives there.

- **`when`** — can it start right now? (default: always)
- **`run`** — a `Supplier<Command>` so the action is re-runnable (WPILib
  commands can't be rescheduled while running)
- **`until`** — success condition; the command is interrupted when met
- **`requires`** — every subsystem the command touches; the orchestrators
  need these to reserve subsystems correctly

---

## BehaviorEngine — resilient autonomous

The auto that doesn't faceplant when a piece isn't where you expected. Each
action's precondition is checked **at the moment it's reached**, and a
fallback fires if it can't start.

```java
Command auto = BehaviorEngine.sequence("ThreeFuel")
    .then(scorePreload)
    .attempt(chaseFuelA).orElse(chaseFuelB)   // substitute if A was taken
    .then(scoreIfHolding)
    .attempt(chaseFuelC).orElseSkip()          // just skip if C isn't there
    .then(scoreIfHolding)
    .deadline(14.0)                            // auto budget, from schedule time
    .onBail(alignAndShoot)                     // last-ditch points if time's up
    .build();
```

Fallback policies after `attempt(...)`:

| | If the action can't start when reached |
|---|---|
| `.orElse(other)` | run `other` instead (if *it* can start) |
| `.orElseSkip()` | skip this step, continue |
| `.orElseAbort()` | stop the whole sequence |

`bailWhen(BooleanSupplier)` is the general form of `deadline(...)` — pass
anything, e.g. `() -> RobotState.matchTimeRemaining() < 3`. When it fires
(or an attempt aborts), the sequence stops and `onBail(...)` runs.

Live introspection at `/Catalyst/Behavior/ThreeFuel/{Step, Action, FellBack, Bailed}`
— you can watch on the dashboard exactly which action is running and when a
fallback fired.

---

## Strategist — chase, then bail to a sure thing

The utility selector. Register behaviors with a **score function**; every
loop the highest scorer that can start runs, and it switches the moment a
different behavior starts winning. No explicit state machine — the scores
*are* the strategy.

This is exactly the "chase scattered fuel in auto, but go align-and-shoot
when time runs short" pattern:

```java
Command fuelAuto = Strategist.named("FuelAuto")
    .add("ChaseFuel", chaseNearestFuel,
         ctx -> (scored < goal && ctx.matchTimeRemaining() > 4.0
                 && vision.hasFuelTarget()) ? 10.0 : 0.0)
    .add("AlignAndShoot", alignAndShoot,
         ctx -> (scored >= goal || ctx.matchTimeRemaining() <= 4.0) ? 20.0 : 0.0)
    .build();
```

While there's time and fuel on the field, `ChaseFuel` scores 10 and wins.
The instant match time drops to 4 s (or the goal is met), `AlignAndShoot`
jumps to 20 and the Strategist switches to it mid-stride — drives back,
auto-aligns (your SOTF), and shoots whatever it's got. No fuel left to
chase, no time wasted.

Every behavior's score publishes to
`/Catalyst/Behavior/FuelAuto/Scores/<name>` and the running one to
`Active`, so you can see *why* it chose what it chose.

---

## Autopilot — teleop cycle co-pilot

Driver holds one button; the robot runs the acquire → score loop until they
release. Two actions and a "do we have a piece?" test:

```java
Autopilot copilot = Autopilot.builder()
    .name("Cycle")
    .acquire(grabNearestFuel)   // pathfind to detected fuel + intake
    .score(alignAndShoot)       // align (SOTF) + shoot
    .hasPiece(intake::hasFuel)
    .build();

driver.rightTrigger().whileTrue(copilot.run());   // hold to engage, release to take over
```

While engaged the co-pilot owns the subsystems its actions require, so the
driver's steering is suspended until they release. Pairs naturally with
[`RumbleEvents`](../driver/) — buzz the driver when a piece is acquired or a
score completes:

```java
events.onTrigger(intake.hasPieceTrigger(), Pattern.SHORT, Channel.DRIVER);
```

Phase publishes to `/Catalyst/Behavior/Cycle/Phase`
(`Acquire` / `Score` / `DriverControl`).

---

## How it composes with the rest of Catalyst

The framework is the capstone — its `Action`s are built from primitives you
already have:

| Need | Use |
|---|---|
| Drive to a spot | `SwerveSubsystem.pathfindToPose()` |
| Aim & shoot while moving | `AimingSolver` + `TurretMechanism` |
| Detect a piece for a precondition | `VisionSubsystem` / `LimelightTriggers` |
| Time / alliance awareness | `RobotState` (via `BehaviorContext`) |
| Multi-mechanism coordination | `SuperstructureCoordinator` |
| Abort on a fault | `RobotSafety` / `HealthMonitor` |
| Driver feedback | `RumbleEvents` |

---

## Multi-camera note

If your `Action` preconditions read vision (piece detection, tag-in-view),
the [VisionSubsystem](../subsystems/) now fuses 4+ cameras deterministically
— estimates are snapshotted, gated independently, and added to the pose
estimator in timestamp order with quality/index tiebreaks, so behavior
decisions built on vision are reproducible run-to-run rather than dependent
on which camera happened to report last.
