---
layout: default
title: Simulation
parent: Advanced
nav_order: 8
---

# Simulation
{: .no_toc }

Drive every mechanism on your robot from the browser with the built-in
dashboard, then add full-field physics with maple-sim when you want it.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Built-in mechanism dashboard (SimDashboard)

`SimDashboard` is the primary way to simulate and drive your mechanisms. You
register any `CatalystMechanism` and the dashboard renders a live, fitting
widget for it and lets you drive it from a web page. There is no per-robot
HTML to write and nothing to add to your build.

It works by reading each mechanism's `describe()` method, which returns a
`MechanismView` (name, kind, value, setpoint, range, velocity, current, and
kind-specific extras). The dashboard picks a widget from the `kind` field, so
a linear actuator gets a travel bar, a flywheel gets a speed readout, a claw
gets a grip-state chip, and a team's own subclass gets a widget too as long as
it overrides `describe()`. The base `CatalystMechanism.describe()` returns a
`generic` view, and every built-in mechanism overrides it.

Three properties make it safe to leave in shared robot code:

- **Dependency-free.** It serves a single page from the JDK's built-in
  `com.sun.net.httpserver.HttpServer`. No vendordep, no extra Gradle entry.
- **Sim-only.** `start()` and `update()` return immediately when
  `RobotBase.isSimulation()` is false, so the exact same calls do nothing on a
  real robot. You do not need to guard them.
- **Thread-safe.** The HTTP server runs on its own threads, but browser input
  is never executed there. Every button, slider, and toggle is queued and run
  on the main (scheduler) thread inside `update()`, and `describe()` is only
  ever called there too. So it is always safe to schedule a `Command` or mutate
  robot state from a control binding.

The default port is `5805`. Pass a port to the constructor to change it.

### Usage

Construct one `SimDashboard`, register each mechanism with `add(...)`, attach
optional controls, then `start()` it in `robotInit` and `update()` it once per
loop in `robotPeriodic`.

```java
private final SimDashboard dash = new SimDashboard();   // port 5805

@Override
public void robotInit() {
    dash.add(elevator)
        // slider that schedules a Command each time it moves
        .slider("Height (m)", 0.0, 0.6, v -> CommandScheduler.getInstance().schedule(elevator.goTo(v)))
        .command("Stow", () -> elevator.goTo("DOWN"))   // button that schedules a Command
        .command("Top",  () -> elevator.goTo("UP"));

    dash.add(intake)
        .command("Intake", intake::intake)
        .command("Stop",   () -> intake.runAtSpeed(0))
        .toggle("Game piece", intake::setSimHasPiece);   // flip the simulated piece state

    dash.add(shooter)
        .slider("Target (rps)", 0.0, 90.0, v ->
            CommandScheduler.getInstance().schedule(shooter.spinUp(v)));

    dash.start();
}

@Override
public void robotPeriodic() {
    dash.update();   // drains queued browser commands + snapshots state
}
```

A mechanism with no controls is still shown live, just read-only. The control
methods are fluent and chain: `button`, `command`, `slider`, `toggle` (with an
optional `BooleanSupplier` so a toggle tracks live robot state instead of just
the last click), and `add` to chain straight to the next mechanism. Call
`title(String)` to set the page title and `stop()` to shut the server down.

### Widget per mechanism kind

The dashboard maps each `MechanismView` kind to a fitting widget:

| Kind | Widget |
|---|---|
| `linear` | travel bar over the configured range |
| `rotational` / `turret` | angle gauge with setpoint tick |
| `flywheel` | speed readout (rps) with at-speed chip |
| `roller` | speed plus a game-piece chip |
| `claw` | grip-state chip |
| `diffwrist` | pitch value with roll in the extras |
| `winch` | extension value over its range |
| `pneumatic` | solenoid state chip |

Each widget also draws a live **sparkline** of its primary value with a setpoint
guide, so you can watch a mechanism settle. The header has a **Pause/Resume**
toggle (freeze the view mid-motion) and an **Export CSV** button that downloads
the current snapshot of every mechanism.

### Mechanisms now run real physics in sim

Four mechanisms that used to be inert in simulation now run real WPILib
physics models so their widgets move on their own when commanded:

- **Roller** runs a `FlywheelSim`.
- **Claw** runs a `DCMotorSim`.
- **Winch** runs an `ElevatorSim` when `spoolRadius > 0`, otherwise a
  `DCMotorSim`.
- **Differential Wrist** runs two `DCMotorSim` (one per axis).

Roller and Claw have no continuous sensor for a game piece, so call
`setSimHasPiece(boolean)` to flip the simulated piece state from a toggle (it
is sim-only and ignored on a real robot). Two new sim-only config fields tune
the physics: `WinchMechanism.Config.builder().loadMass(double kg)` (default
6.0) and `DifferentialWristMechanism.Config.builder().momentOfInertia(double
kgMetersSquared)` (default 0.004 per axis). Both affect simulation only.

### Try it

The example ships a full one-of-every-kind lab. `MechanismShowcase` builds one
of every mechanism kind (linear, rotational, roller, flywheel, turret, claw,
differential wrist, winch, pneumatic) on CAN IDs 30 to 39 and drives each from
its own `SimDashboard` on port `5806`. Run the example in simulation and open
**[localhost:5806](http://localhost:5806)** next to the game cockpit on
**[localhost:5805](http://localhost:5805)**. Nothing in the lab is specific to
this year's game; the same calls work against your real robot's mechanisms.

---

## Full-field physics (maple-sim)

`SimDashboard` drives mechanisms. When you also want a physics-simulated,
game-piece-aware *field* to test the whole autonomy stack against (behavior
framework, SOTF, pathfinding), add [maple-sim](https://shenzhen-robotics-alliance.github.io/maple-sim/).
This is the optional advanced layer.

### How Catalyst integrates

maple-sim is a physics-engine simulation with collisions and game pieces.
Catalyst does not bundle it. It is an unstable, fast-moving, sim-only package,
and Catalyst is a library other teams depend on, so forcing maple-sim onto
every user (and risking a broken build when its API churns) would be the wrong
trade.

Instead Catalyst gives you the **seam**: two dependency-free hooks that let
*your* maple-sim instance drive Catalyst's odometry and visualization. You add
maple-sim to your own robot project (it is a normal vendordep there) and wire
it through these.

| Hook | What it's for |
|---|---|
| `SwerveSubsystem.setSimPose(Pose2d)` | feed maple-sim's physics pose into Catalyst's estimator (sim only; no-op on a real robot) |
| `SimGamePieces` | stream simulated piece positions to NT for AdvantageScope |

### Setup

1. Add maple-sim to **your robot project** (not Catalyst) by following
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
    fuel.publish();   // -> /Catalyst/Sim/Fuel
}
```

> Method names follow maple-sim's API, which changes between releases, so
> check their current docs. The Catalyst side (`setSimPose`, `SimGamePieces`)
> is stable.

### What you can test in sim

Because Catalyst's odometry now tracks the physics world, the whole autonomy
stack runs against it:

- **Behavior framework** drives your `Strategist` against simulated fuel. Watch
  it chase pieces and bail to a shot as the simulated clock runs down, all on
  the AdvantageScope field.
- **SOTF** reads the simulated pose and velocity through the `AimingSolver`, so
  you can sanity-check the virtual-goal math before a turret exists.
- **Pathfinding / Choreo** drives the simulated robot through the simulated
  field with `pathfindToPose` and `followChoreoPath`.
- **Vision pursuit** feeds `driveToPiece` a supplier of the nearest simulated
  piece pose and cycles.
- **Vision fusion** runs with no camera: add a `SimCameraSource` to your
  `VisionConfig`, feeding it the true simulated pose. It emits noisy,
  latency-delayed pose estimates so the whole `VisionSubsystem` pipeline (accept
  / reject, std-dev weighting, feeding the swerve estimator) runs in the
  simulator, letting you tune `maxAmbiguity` and the std-dev model early.

```java
VisionConfig cfg = VisionConfig.builder()
    .driveSubsystem(drive)
    .addCamera(SimCameraSource.builder("SimFront")
        .truePose(() -> swerveSim.getSimulatedDriveTrainPose())
        .translationStdDevMeters(0.03)
        .latencySeconds(0.03)
        .build())
    .build();
```

### Visualization

Open AdvantageScope, connect to the simulator, and add:

- `/Catalyst/Swerve/Pose` for the robot (already published by `SwerveSubsystem`)
- `/Catalyst/Sim/<name>` for your game pieces (`Pose3d[]` from `SimGamePieces`)
- `/Catalyst/Ghost/Pose` for a recorded driver path, if you are using [GhostReplay](../driver/)

You get a full simulated match on the field view, driven by real physics.
