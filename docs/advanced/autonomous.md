---
layout: default
title: Autonomous Architecture
parent: Advanced
nav_order: 9
---

# Autonomous Architecture
{: .no_toc }

How to combine a pre-planned PathPlanner route with reactive behavior
(chasing pieces, auto-aligning) without the robot fighting itself.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## The core tension

A PathPlanner path is a **time-parameterized plan**: at time *t* the robot
should be at a specific (x, y, θ) moving at a specific velocity. That's
great for the gross route — but the moment you do something reactive
(chase a game piece that isn't exactly where the path assumed, run a
precision auto-align to score), you're no longer where the plan says you
should be at time *t*. Naively, the path controller then fights to "catch
back up" to a stale point. That's the problem.

The path controller's output is two parts:

- **feedforward** — the planned velocity. The *intent*, the momentum.
- **feedback** — a PID correction that nudges the robot back onto the
  planned point.

PathPlanner **removed automatic replanning** in 2025 — no single "robot
went off course, recover" behavior satisfies every team. So the modern
approach isn't "re-plan the path." It's three distinct tools, used
deliberately.

---

## Tool 1 — bend the path without leaving it (feedback override)

PathPlanner 2026 lets you **override the feedback while keeping the
feedforward**. The robot stays on the path's momentum and route, but the
correction bends toward what you actually see. Catalyst wraps this in
`PathCorrection` with safe lifecycle (the override is set when the command
starts and cleared when it ends — it can never leak into the next path).

### Face a goal while driving the path (shoot-on-the-move) — clean

This is the safe, common one. Keep following the path's XY, but override
the **rotation target** to face the goal. PathPlanner still does its own
rotation feedback, just toward your target.

```java
Command driveAndAim = PathCorrection.facingPoint(
    FieldConstants.GOAL,          // face this
    swerve::getPose,
    swerve.followPath("CrossField"));  // while following this
// turret/shooter can run SOTF on top — see the Aiming guide
```

### Nudge toward a target (chase while pathing) — sharp edge

`PathCorrection.nudgingXY(...)` overrides the **X/Y feedback** to bias the
robot toward a detected piece. ⚠️ This *replaces* PathPlanner's own XY
correction, so when the target is lost the robot follows pure feedforward
and can drift. Use it only while a target is actually visible — and for a
hard chase that fully leaves the route, prefer Tool 3 instead.

---

## Tool 2 — discrete reactive actions

Some deviations are too large to blend in. Driving onto a scattered game
piece, or a tight precision align at a scoring node, is its own action that
*intentionally* leaves the path. Don't fight that — make it a first-class
step:

```java
swerve.driveToPiece(() -> vision.nearestPiecePose());   // leaves the planned route, on purpose
```

After it runs, the robot is wherever the piece was — **not** at the next
path's start. Which is exactly what Tool 3 is for.

---

## Tool 3 — rejoin the plan from wherever you are (pathfind-then-follow)

This is the key primitive most teams miss. After a reactive action leaves
you off course, **pathfind from the current pose** to the start of the next
planned path, then follow it:

```java
swerve.pathfindThenFollowPath("ScoreFromMidfield");
```

Versus `followPath(...)`, which assumes you start at the path's beginning —
correct only between known waypoints, wrong right after a deviation. The
distinction is the whole game:

| Command | Use when |
|---|---|
| `followPath(name)` | you're at the path's start (segment between known points) |
| `pathfindThenFollowPath(name)` | you might be off course (right after a chase / align) |
| `pathfindToPose(pose)` | drive to an arbitrary pose (a scoring spot, not a named path) |

---

## Putting it together

The pattern advanced teams converge on — planned segments stitched by
pathfinding, with reactive bends where they help:

```java
Command auto = BehaviorEngine.sequence("Reactive3Piece")
    // segment 1: known start → known pickup, exact follow
    .then(Action.named("ToPickup1").run(() -> swerve.followPath("ToPickup1")).build())
    // reactive: the piece isn't exactly on the path — chase it (leaves the route)
    .then(Action.named("GrabPiece1").run(() -> swerve.driveToPiece(() -> vision.nearestPiecePose()))
              .until(intake::hasPiece).build())
    // rejoin the plan from wherever the chase ended, and face the goal on the way
    .then(Action.named("ToScore1").run(() ->
              PathCorrection.facingPoint(GOAL, swerve::getPose,
                  swerve.pathfindThenFollowPath("ToScore1"))).build())
    .then(Action.named("Score1").run(() -> superstructure.scoreCommand()).build())
    .deadline(14.5)
    .onBail(Action.named("Bail").run(() -> swerve.pathfindToPose(() -> SAFE_SHOT)
              .andThen(superstructure.scoreCommand())).build())
    .build();
```

Generate this skeleton in the [Auto Builder](../tools/auto/) — its command
templates map one-to-one onto the tools above.

### Rules of thumb

- **PathPlanner plans the gross route.** Don't try to encode chasing into a
  single monolithic path.
- **Bend, don't break, when you can** — `PathCorrection.facingPoint` for
  shoot-on-the-move keeps you on the route.
- **For real deviations, deviate on purpose** (a discrete reactive step) and
  **pathfind to rejoin** — never `followPath` right after a deviation.
- **The plan is feedforward, the reaction is feedback.** Keep the momentum,
  bend the correction.
