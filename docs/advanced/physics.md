---
layout: default
title: Physics Core
parent: Advanced
nav_order: 10
---

# Physics Core
{: .no_toc }

The optional layer that watches how your robot actually behaves — fused velocity with honest
confidence, per-module slip scoring, collision detection, and shot-release prediction.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## The problem

Every other part of Catalyst tells the robot what to do. Physics Core is the part that notices what
the robot *did*.

Three assumptions run through most FRC code, and all three are fine right up until they are not:

- **`Pose2d` is truth.** It is, until vision has not seen a tag for four seconds and you are
  dead-reckoning across the field.
- **Wheel velocity is the robot's velocity.** It is, until a tyre breaks loose leaving the wall and
  the encoders keep reporting motion the robot is not getting.
- **The robot only does what you commanded.** It does, until a defender lands a hit in the middle of
  auto and the path controller keeps driving as if nothing happened.

Physics Core fuses wheels and IMU into one velocity with a confidence attached, scores which wheel is
lying, notices when something hit the robot, and predicts where the robot will be when a shot
actually leaves it.

> **It is optional and it is advisory.** Every Catalyst API works identically without it. Physics
> Core writes no pose, schedules no command, blocks no transition, and changes no setpoint. It
> produces estimates and explanations; the state machine, `BehaviorEngine`, and `Autopilot` keep
> deciding what the robot does with them. Adding it can only give you more information.

It is also **not a rigid-body physics engine.** There is no collision solver and no contact model —
for simulation with game pieces, see the [maple-sim integration](simulation.md). Physics Core is
about the real robot's measured behaviour.

---

## Setting it up

Four measurements and five suppliers. The model values are things you can get in an afternoon with a
bathroom scale and a tape measure.

```java
PhysicsCore physics = PhysicsCore.builder()
    .robotModel(RobotModel.builder()
        .massKg(54.4)                     // robot + battery + bumpers, on a scale
        .footprintMeters(0.74, 0.74)      // between wheel contact patches
        .wheelRadiusInches(2.0)
        .centerOfMassHeightMeters(0.20)   // measure it, or estimate a quarter of robot height
        .build())
    .kinematics(drive.getDrivetrain().getKinematics())
    .poseSource(drive::getPose)
    .chassisSpeedsSource(drive::getChassisSpeeds)     // robot-relative
    .moduleStatesSource(drive::getModuleStates)
    .accelerationSource(() -> new Translation2d(imu.getAccelX(), imu.getAccelY()))
    .yawRateSource(() -> Units.degreesToRadians(imu.getRate()))
    .releaseDelaySeconds(0.12)                        // measure it once against a log
    .build();
```

One line in `robotPeriodic()`, after the drivetrain has updated:

```java
public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    physics.update();
}
```

And wherever you already call `addVisionMeasurement`, tell Physics Core too:

```java
drive.addVisionMeasurement(estimate.pose(), estimate.timestamp(), stdDevs);
physics.observe(PoseObservation.of(estimate.pose(), estimate.timestamp(), "limelight-front"));
```

Everything lands under `Catalyst/Physics/...` in NetworkTables, so a shadow-mode session is
reviewable in AdvantageScope with no dashboard work.

### Every source is optional

| Missing | What you lose | What still works |
|---|---|---|
| `accelerationSource` | IMU fusion, disturbance residuals, collision detection | Confidence, prediction, slip scoring |
| `moduleStatesSource` / `kinematics` | Per-module slip scoring | Everything else |
| `yawRateSource` | Angular acceleration | Everything else |
| Vision observations | Absolute-fix confidence (it decays and stays low) | Everything else |

Missing inputs are named in `physics.health()` rather than silently producing plausible nonsense.

---

## The five questions it answers

```java
physics.observe(observation);   // here is some evidence
physics.state();                // where is the robot, how fast, how sure
physics.predict(0.3);           // where will it be in 300 ms
physics.analyze();              // is anything going wrong physically
physics.health();               // is Physics Core itself working
```

### `state()` — the fused estimate

`PhysicalRobotState` is `Pose2d` plus the things a pose cannot tell you: which way the robot is
actually travelling, how hard it is accelerating, and how much to believe it.

```java
PhysicalRobotState state = physics.state();

state.speedMetersPerSecond();
state.fieldAcceleration();
state.quality().confidence();          // 0.0 to 1.0
state.quality().level();               // HIGH / MODERATE / LOW / LOST
state.quality().reason();              // "vision stale 4.1 s"
```

**How the fusion works.** The two velocity sources fail in opposite ways — wheel odometry is exact at
steady state and wrong the instant a tyre slips; integrated IMU acceleration does not care about
traction but drifts within a second or two. A complementary observer takes the part of each that
works:

```text
v_imu   = v_previous + a_measured * dt        (short-term, traction-blind)
v_fused = w * v_wheels + (1 - w) * v_imu      (w falls as slip rises)
```

`w` sits at 0.98 with everything rolling — on a healthy swerve the wheels really are that good — and
falls toward 0.15 as slip rises. It never reaches zero: an estimate running purely on integrated
acceleration walks away within seconds, so the wheels always keep a small anchoring vote.

**Where confidence comes from.** Three things subtract from a starting 1.0, each capped so no single
factor can zero it alone:

| Factor | Max cost | Saturates at |
|---|---|---|
| Time since an absolute fix | 0.40 | 3 seconds |
| Wheel slip | 0.35 | fully slipping |
| Wheels vs IMU disagreement | 0.25 | 1.0 m/s apart |

These are honest engineering judgement, not statistically validated probabilities. They are
documented so you can argue with them, and every one is tunable on
`PhysicalStateEstimator.builder()`.

### `analyze()` — what is going wrong

```java
PhysicsAnalysis analysis = physics.analyze();

analysis.slipFactor();        // mean across modules: how corrupted odometry is
analysis.peakSlip();          // worst single module: what to alert on
analysis.worstModule();       // which one
analysis.tractionUsage();     // 1.0 = at the traction limit
analysis.tippingUsage();      // 1.0 = inside wheels are unloaded
analysis.lastCollision();     // Optional<CollisionEvent>
analysis.describe();          // "module 2 slipping (87%)"
```

**Slip scoring** takes the chassis velocity you currently believe, runs it back through inverse
kinematics to get the speed each wheel *should* be turning, and compares. Only the component along
the module's predicted direction counts, so a module still rotating toward its setpoint is not
mistaken for one that is slipping. Below 0.25 m/s detection is suppressed — at a standstill every
residual is noise.

The two aggregates answer different questions. `slipFactor()` is the mean, because forward kinematics
averages the modules, so the mean is what describes how corrupted odometry is; that is the number the
estimator uses. `peakSlip()` is the max, because one wheel in the air is a real fault even though it
barely moves the mean; that is the number to alert on.

**Collision detection** compares the acceleration the wheels imply against the acceleration the IMU
measured. Two very different faults fall out of the same residual:

- Wheels accelerate, robot does not → the tyres are spinning.
- Robot accelerates, wheels did not command it → something hit it.

An event fires when the residual exceeds 70% of the traction limit for two consecutive loops, with a
0.5 s refractory period so one collision is reported once rather than every loop it rings for.

### `predict()` and `predictLaunchState()` — shot timing

A shot leaves the robot ~120 ms after the trigger. At 3 m/s that is 36 cm of error before the piece
has left. `predictLaunchState()` gives you the robot's state at release, and it drops straight into
the existing solver — no new overload, no new API:

```java
LaunchState launch = physics.predictLaunchState();

AimingSolverVector.TargetState shot =
    aimingSolver.calculate(launch.pose(), launch.fieldVelocity());

if (launch.fitsTarget(SHOT_FLIGHT_SECONDS, GOAL_RADIUS_METERS)) {
    shooter.fire();
}
```

`missRadiusMeters(timeOfFlight)` compounds the two robot-state errors: where the robot will be at
release, plus a velocity error that mis-cancels the motion compensation for the whole flight. It
accounts only for the *robot's* uncertainty — shooter repeatability and piece variation are your
shooter's own error budget — so treat it as a floor on the miss radius, not the whole of it.

Pass a variable delay for anything that changes shot to shot:

```java
LaunchState launch = physics.predictLaunchState(flywheel.secondsToRecover());
```

### `health()` — is Physics Core itself working

The most common way an advisory layer fails is quietly: a supplier was never wired, the accelerometer
returns zeros, `update()` stopped being called. The estimates keep coming out, they are just wrong.

```java
SystemCheck.builder()
    .test("Physics Core running", () -> physics.health().isHealthy())
    .build();
```

---

## Using it with the rest of Catalyst

Physics Core supplies suppliers. The existing hooks already take them — nothing in the state machine
or `BehaviorEngine` had to change.

**State machine guards.** The machine still owns which transitions are legal; Physics Core just
answers whether the physics agrees.

```java
.edge(STOWED, EXTENDED, e -> e
    .guard(() -> !physics.analyze().isNearTipping(), "tip-margin"))
```

**Behaviour preconditions.**

```java
Action scoreHigh = Action.named("Score high")
    .when(() -> physics.state().quality().isTrustworthy())
    .run(superstructure::scoreHigh)
    .build();

BehaviorEngine.sequence("Cycle")
    .attempt(scoreHigh)
    .orElse(scoreLow)
    .build();
```

**Confidence-aware speed.** Nothing here is automatic — you write the policy:

```java
drive.setSpeedMultiplier(switch (physics.state().quality().level()) {
    case HIGH     -> 1.0;
    case MODERATE -> 0.6;
    case LOW      -> 0.3;
    case LOST     -> 0.0;
});
```

**Approach speed from stopping distance.**

```java
double safe = physics.drivetrainModel().maxSpeedForStoppingDistance(distanceToWall);
```

### The `RobotStateSource` contract

`SwerveSubsystem` and `PhysicsCore` both implement `RobotStateSource`, so code that only needs "where
is the robot and how fast" can take the interface and work with either — and keeps working if you add
Physics Core later, or take it away again.

```java
void aimAt(RobotStateSource state, Translation2d target) { ... }

aimAt(drive, target);      // plain drivetrain
aimAt(physics, target);    // same call, better estimate
```

`UncertainRobotStateSource` adds `quality()`, and derives a diagonal `poseCovariance()` from it for
free.

---

## Compute profiles

Physics Core is told what it may spend rather than being hard-coded to a controller name.

| Profile | What runs | Notes |
|---|---|---|
| `MINIMAL` | Fused velocity, confidence, prediction | For a loop that is already tight |
| `BALANCED` | + slip scoring, disturbance residuals, collision detection | **Default.** What Phase 1 was designed against |
| `ADVANCED` | Same as `BALANCED` today | Reserved for richer estimation, a later phase |
| `SYSTEMCORE` | Same as `BALANCED` today | Reserved for coprocessor-class work, a later phase |

`ADVANCED` and `SYSTEMCORE` deliberately do not silently enable anything unvalidated. If you select
one today you get `BALANCED` behaviour, and the profile is recorded in `health()` so a log says what
was actually running.

Watch `Catalyst/Loop/Robot/AverageMs` before and after enabling it — see
[keeping the loop under 20 ms](logging.md).

---

## What ships today, and what does not

What is here is **Phase 1: shadow mode.** Every service is observation-only, every algorithm is
HAL-free and unit tested (94 tests), and nothing takes control authority.

Deliberately **not** in this release:

- **Pose fusion.** Physics Core reads your pose and never writes it. Your drivetrain estimator stays
  the single writer of `Pose2d`. A vision observation only resets the staleness term in confidence —
  and one that lands more than a metre from the current estimate is rejected outright and counted,
  because one bad frame should not convince the robot it teleported.
- **Dynamic constraints.** Nothing slows the robot down or clamps acceleration on its own. The limits
  are published; applying them is your policy, written where you can see it.
- **Online parameter identification.** Learning `kS`/`kV`/`kA`, effective mass, or battery resistance
  from real operation, and never applying learned values silently during a match.
- **Power modelling.** `BrownoutMonitor` already predicts sag from `Σ I × R` — see
  [health monitoring](health.md). Duplicating it would create a second, conflicting path.

Those are later, explicitly opt-in phases. See the [roadmap](../ROADMAP.md).

---

## Validating it on your robot

Physics Core is measurement, so measure it before you trust it.

1. **Run it in shadow mode for a practice session.** It has no control authority, so there is nothing
   to be careful about. Watch `Catalyst/Physics/...` in AdvantageScope.
2. **Check `SensorDisagreement` while driving normally.** It should sit near zero. A persistently
   large value with no slip reported points at a calibration problem — wrong wheel radius, wrong gear
   ratio, or an IMU mounted at an angle nobody told the code about. Fix that before reading anything
   else.
3. **Induce slip deliberately.** Full-throttle from a standstill on a slick patch. `Slip/Peak` should
   spike and `Slip/WorstModule` should name the right corner.
4. **Bump the robot.** `Collision/MpsSq` should record it once, not six times.
5. **Cover the cameras.** `Quality/Confidence` should fall over about three seconds and recover when
   they come back.
6. **Then, and only then**, start gating decisions on it.

If a Physics Core signal does not outperform the simple check you already had, keep using the simple
check. That rule applies to everything in this package.
