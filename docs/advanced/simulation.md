---
layout: default
title: Simulation (maple-sim)
parent: Advanced
nav_order: 8
---

# Simulation with maple-sim
{: .no_toc }

Test the whole Catalyst autonomy stack — behavior framework, SOTF,
pathfinding — against a physics-simulated, game-piece-aware field.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## How Catalyst integrates

[maple-sim](https://shenzhen-robotics-alliance.github.io/maple-sim/) is a
physics-engine simulation with collisions and game pieces. Catalyst doesn't
bundle it — it's an unstable, fast-moving, sim-only package, and Catalyst is
a library other teams depend on, so forcing maple-sim onto every user (and
risking a broken build when its API churns) would be the wrong trade.

Instead Catalyst gives you the **seam**: two dependency-free hooks that let
*your* maple-sim instance drive Catalyst's odometry and visualization. You
add maple-sim to your own robot project (it's a normal vendordep there) and
wire it through these.

| Hook | What it's for |
|---|---|
| `SwerveSubsystem.setSimPose(Pose2d)` | feed maple-sim's physics pose into Catalyst's estimator (sim only; no-op on a real robot) |
| `SimGamePieces` | stream simulated piece positions to NT for AdvantageScope |

---

## Setup

1. Add maple-sim to **your robot project** (not Catalyst) — follow
   [their install guide](https://shenzhen-robotics-alliance.github.io/maple-sim/).
2. Create a `SwerveDriveSimulation` from your drivetrain constants.
3. Wire it to Catalyst in `simulationPeriodic()`.

```java
private final SwerveDriveSimulation swerveSim = /* maple-sim setup */;
private final SimGamePieces fuel = new SimGamePieces("Fuel");

@Override
public void simulationPeriodic() {
    // 1. Step the physics world.
    SimulatedArena.getInstance().simulationPeriodic();

    // 2. Feed the simulated pose into Catalyst's odometry.
    drive.setSimPose(swerveSim.getSimulatedDriveTrainPose());

    // 3. Stream game pieces for AdvantageScope.
    fuel.clear();
    for (var piece : SimulatedArena.getInstance().getGamePiecesByType("Fuel")) {
        fuel.set(piece, piece.getPose3d());
    }
    fuel.publish();   // → /Catalyst/Sim/Fuel
}
```

> Method names follow maple-sim's API, which changes between releases —
> check their current docs. The Catalyst side (`setSimPose`,
> `SimGamePieces`) is stable.

---

## What you can now test in sim

Because Catalyst's odometry now tracks the physics world, the whole
autonomy stack runs against it:

- **Behavior framework** — drive your `Strategist` against simulated fuel;
  watch it chase pieces and bail to a shot as the (simulated) clock runs
  down, all on the AdvantageScope field.
- **SOTF** — the `AimingSolver` reads the simulated pose + velocity, so you
  can sanity-check the virtual-goal math before a turret exists.
- **Pathfinding / Choreo** — `pathfindToPose` and `followChoreoPath` drive
  the simulated robot through the simulated field.
- **Vision pursuit** — feed `driveToPiece` a supplier of the nearest
  simulated piece pose and watch it cycle.

---

## Visualization

Open AdvantageScope, connect to the simulator, and add:

- `/Catalyst/Swerve/Pose` — the robot (already published by `SwerveSubsystem`)
- `/Catalyst/Sim/<name>` — your game pieces (`Pose3d[]` from `SimGamePieces`)
- `/Catalyst/Ghost/Pose` — a recorded driver path, if you're using [GhostReplay](../driver/)

You get a full simulated match on the field view, driven by real physics.
