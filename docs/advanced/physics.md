---
layout: default
title: Physics Core
parent: Advanced
nav_order: 10
---

# Physics Core
{: .no_toc }

The optional layer that watches how your robot actually behaves — fused state with honest confidence,
slip and collision detection, a live centre of mass, online parameter identification, and predictive
capability evaluation.
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
| `BALANCED` | + slip scoring, disturbance residuals, collision detection | **Default.** What the shadow-mode benchmark targets |
| `ADVANCED` | Same as `BALANCED` today | Reserved for fixed-lag smoothing and a nonlinear estimator |
| `SYSTEMCORE` | Same as `BALANCED` today | Reserved for coprocessor-class work |

`ADVANCED` and `SYSTEMCORE` deliberately do not silently enable anything unvalidated. If you select
one today you get `BALANCED` behaviour, and the profile is recorded in `health()` so a log says what
was actually running.

The profile gates only what runs **inside** `PhysicsCore.update()`. The models, identifiers,
evaluators, and constraints described below are separate objects you construct and call yourself, so
their cost is yours to place — most are pure functions you can call once a second rather than once a
loop.

Watch `Catalyst/Loop/Robot/AverageMs` before and after enabling it — see
[keeping the loop under 20 ms](logging.md).

---

## The mass tree: where the centre of mass actually is

`RobotModel` carries one `centerOfMassHeightMeters`, and for a robot that stays low that is the end of
it. It stops being true the moment something heavy goes up. Raise a 10 kg elevator carriage 1.2 m on a
60 kg robot and the true centre of mass climbs 20 cm — which is the difference between a comfortable
tipping margin and a marginal one, and a static number cannot see it.

Describe the moving parts once and the model tracks them:

```java
ArticulatedRobotModel robot = ArticulatedRobotModel.builder()
    .chassis(chassisModel)                        // mass of everything that does NOT move
    .add(MechanismModel.linear("Elevator", 10.0)
        .mountedAt(new Translation3d(0, 0, 0.15))
        .position(elevator::getPosition)          // live, metres
        .build())
    .add(MechanismModel.rotational("Arm", 5.0)
        .childOf("Elevator")                      // rides on the carriage
        .comAt(new Translation3d(0.35, 0, 0))     // mass is out along the arm
        .angle(() -> Units.degreesToRadians(arm.getPosition()))
        .build())
    .build();

double h = robot.centerOfMassHeightMeters();                       // live
Pose3d gripper = robot.fieldPoseOf("Arm", drive.getPose()).get();  // field-relative end effector
```

Links form a transform tree, so `childOf` composes properly: raise the elevator and the arm goes with
it. That also gives you the **field-relative end-effector pose** placement games want, without
hand-rolling a transform chain every season.

> **Watch the sign on rotational links.** Right-handed rotation about `+Y` takes `+X` toward `−Z`, so
> a *positive* angle pitches the far end *down*. Most arms count up as positive, which is the
> opposite. Pass `.about(new Translation3d(0, -1, 0))` or negate in the supplier. Getting it backwards
> does not throw — it silently *lowers* the CoM when the arm goes up — so check it once with the arm
> raised.

### Stability, properly

`StabilityModel` computes the **zero-moment point**: gravity plus the robot's own acceleration produce
a resultant, and where that resultant meets the carpet is where the ground has to push back.

```text
zmp = (cx, cy) − (cz / g) · (ax, ay)
```

While the ZMP is inside the wheel rectangle every wheel carries load. At the edge, the inside wheels
have unloaded. Past it, the robot rotates about that edge.

```java
StabilityModel stability = new StabilityModel(robot);

double margin = stability.tipMarginMeters(state.fieldAcceleration(), state.pose().getRotation());
double limit  = stability.maxAccelerationMpsSq(new Translation2d(1, 0));   // hardest safe forward
double[] load = stability.wheelLoadsNewtons(robotRelativeAccel);           // FL, FR, BL, BR
```

`tipMarginMeters` is in metres, so it is a number you can show a driver and threshold a guard on.
`DrivetrainModel.maxTippingAccelerationMpsSq()` is this same calculation with the CoM assumed centred
and fixed — the two agree exactly in that case, which is asserted in the tests, and diverge as soon as
the robot extends or carries mass off-centre.

Wheel loads use the standard bilinear split. Four contacts on a rigid body are statically
indeterminate — the real split depends on frame flex Catalyst does not model — but the bilinear
distribution is the unique symmetric one that reproduces both the total weight and the moment about
each axis, and it goes to zero on exactly the wheel that lifts.

---

## Ballistics: time of flight without a magic constant

`missRadiusMeters(timeOfFlight)` needs a flight time, and a constant is wrong at every distance but
one. `ProjectileModel` solves it in closed form:

```java
ProjectileModel shot = ProjectileModel.noDrag();

double flight = shot.timeOfFlightSeconds(distance, exitSpeed, hoodAngle).orElse(1.0);
if (launch.fitsTarget(flight, GOAL_RADIUS)) { shooter.fire(); }

var angles = shot.launchAnglesFor(distance, goalHeight, exitSpeed).orElseThrow();
// angles.flat() is less sensitive to speed error; angles.lobbed() drops in more steeply
```

Two models, both exact rather than integrated: `noDrag()` (the parabola) and
`withLinearDrag(k)` (`v̇ = −k·v`, still closed-form).

> **Quadratic drag and spin are deliberately absent.** Real drag on a light game piece goes as `v²`,
> and backspin produces lift a ballistic model has no term for. Neither has a closed form, and fitting
> them needs coefficients you can only get by measuring — at which point the measurement is the better
> model. That is exactly what `AimingSolver`'s interpolating tables already are, and they remain the
> way to **aim**. Use this for timing and clearance, not to replace a characterisation sweep.

---

## Learning the constants the robot actually has

`kS`/`kV`/`kA` come from a SysId sweep in the pit, and then the robot gets driven for six weeks. Tread
wears, bearings tighten, someone swaps a motor. The identifiers watch normal operation and report what
the gains look like *now*.

```java
FeedforwardIdentifier elevatorFf = FeedforwardIdentifier.builder("Elevator")
    .withElevatorGravity()
    .seed(0.15, 2.4, 0.06, 0.35)     // your current gains, as a starting point
    .build();

elevatorFf.addSample(motor.getAppliedVoltage(), mech.getVelocity(), mech.getAcceleration());

// in the pit, when you want to know:
elevatorFf.recommendation().ifPresent(r -> System.out.println(r.describe()));
```

Same for the battery, whose internal resistance every brownout prediction rests on and which nobody
measures:

```java
battery.addSample(RobotController.getBatteryVoltage(), pdh.getTotalCurrent());
battery.recommendation().ifPresent(r -> System.out.println(r.describe()));
```

Both are recursive least squares — constant time, constant memory, mathematically identical to a batch
fit over the whole log.

> **Nothing applies these.** There is no method that writes a gain. You read the recommendation,
> review it, and type it into your constants file if you agree. A value learned mid-match and applied
> mid-match with nobody watching is how a robot develops a behaviour nobody can reproduce in the pit.

**When to believe it.** A fit needs *varied* data. A mechanism that only ever runs at one speed cannot
separate `kS` from `kV` — both explain the same observation — and no amount of collecting fixes that.
The covariance tracks exactly this, so `recommendation()` stays **empty** until the fit has genuinely
pinned the values down. A battery under a perfectly steady load reports nothing, forever, and that is
correct.

---

## Fault isolation: which cause fits the pattern

One biased residual says something is wrong. Which residuals are biased *together* says what.

`ModelResidualMonitor` separates noise from bias statistically rather than by threshold: a residual
that swings both ways and averages to zero is a noisy sensor and a correct model; one that sits
consistently to one side is a wrong model. It calls bias only when the offset beats both a tolerance
and its own standard error, so a 12 cm offset buried in 40 cm of scatter is still caught.

`FaultIsolator` then ranks causes by the pattern they predict:

```java
FaultIsolator isolator = FaultIsolator.builder()
    .monitor(frontLeftSpeedResidual)
    .monitor(wheelsVsImuResidual)
    .candidate("Front-left wheel slip", c -> c
        .expects("FL wheel speed", 1.0)
        .expects("wheels vs IMU", 0.8))
    .candidate("Front-left wheel radius calibration", c -> c
        .expects("FL wheel speed", 1.0)
        .expectsQuiet("wheels vs IMU", 1.0))     // this is what separates them
    .build();
```

Both causes disturb that module's speed residual. Only slip also disturbs the IMU comparison — a wheel
rolling correctly at the wrong assumed size still agrees about acceleration. `expectsQuiet` is what
turns "something is wrong with the front left" into "your wheel radius is off".

> **These are diagnostic scores, not probabilities.** 0.9 does not mean 90% likely; it means this
> cause explains the observed pattern about nine tenths as well as a textbook instance would. Nothing
> here has been validated against a population of real faults. They are for *ranking* — for pointing a
> student at the right corner of the robot first.

`JamDetector` handles the case motor signals alone genuinely cannot: a jam and a successful intake
produce identical current and velocity. It separates them the only honest way, by asking whether the
piece is there. With no piece sensor configured it reports `STALLED` and lets you decide, rather than
guessing.

---

## Deciding before you commit

`CapabilityEvaluator` answers "is this worth attempting, and what will it cost" *before* the action is
scheduled — a cycle abandoned at second 1.4 is a cycle wasted, and the information was available at
second 0.

```java
Action scoreHigh = Action.named("Score high")
    .when(() -> evaluator.evaluateDriveTo(physics.state(), scoringSpot, 55).isReliable())
    .run(superstructure::scoreHigh)
    .build();

BehaviorEngine.sequence("Cycle").attempt(scoreHigh).orElse(scoreLow).build();
```

```text
Feasible: yes
Predicted completion: 1.42 s
Predicted minimum voltage: 9.4 V
Predicted position error: 4.2 cm
Tip margin: 7.1 cm
Risk: low
```

Every figure has a derivation, not a heuristic: time from an exact trapezoidal profile under the
acceleration and speed limits in force; position error from the estimate's own uncertainty compounded
over that duration; tip margin from the live centre of mass at the acceleration the move requires;
voltage from `PowerPredictor`. Anything not configured is **absent rather than invented** — with no
power predictor the voltage line simply does not appear.

A robot already travelling too fast to stop in the distance available is reported infeasible with the
reason, which is a real and useful answer.

### Power: planning, not reflex

`BrownoutMonitor` and `PowerPredictor` are not duplicates. `BrownoutMonitor` watches the present and
backs off — a reflex. `PowerPredictor` is asked about the future — a plan.

```java
if (power.canSustain(60.0)) { elevator.raise().schedule(); }

var plan = power.plan(
    new PowerDemand("Elevator", 60), new PowerDemand("Shooter", 45), new PowerDemand("Intake", 25));
// "Elevator + Shooter then Intake"
```

Open-circuit voltage is inferred from the present reading (`V_now + I_now·R`), so the prediction
follows the battery down over a match instead of pretending it is always fresh. Demands are packed in
the order you give them — the sequencing respects the priority you already decided rather than
silently reordering the robot's behaviour.

---

## Bounded intervention — opt-in, and visibly so

This is the only part of Physics Core that could change how the robot drives, so it is built to make
that impossible by accident. There is no `install()`, no periodic hook, nothing that reaches into
`SwerveSubsystem`. It computes limits; you apply them, on a line you wrote:

```java
PhysicsConstraints limits = PhysicsConstraints.builder()
    .physics(physics).stability(stability).power(power).build();

// your code, your call:
drive.setSpeedMultiplier(limits.speedScale());
```

Four limits, each independent, tightest wins: traction and tipping (from the live CoM, so it tightens
on its own when the elevator goes up), confidence, slip, and electrical headroom.

`limits.explain()` names whichever is binding — every slowdown this recommends is attributable to a
sentence:

```text
speed scaled to 45%, accel capped at 4.2 m/s^2: confidence LOW (vision stale 4.1 s); wheel slip 34%
```

The speed scale never returns zero unless the estimate is genuinely lost; a robot frozen mid-match
because a camera blinked is worse than one moving cautiously.

---

## Replay: asking what a change would have done

Tuning an estimator on a real robot is a miserable loop — change a threshold, find a field, recreate
the situation, hope the log caught it. Recorded samples remove all of it.

```java
Result strict  = PhysicsReplay.of(samples).run(() -> buildCore(0.3));
Result relaxed = PhysicsReplay.of(samples).run(() -> buildCore(0.7));

System.out.println(strict.compareTo(relaxed));
// "peak slip 0.91 vs 0.44; 3 vs 1 collision(s); mean confidence 0.62 vs 0.78"
```

Replay is exact, not approximate: `update(PhysicsSample)` takes every measurement as an argument and
the clock is injectable, so the same samples in the same order give the same numbers every time. A
transform can also remove a sensor, to ask what it was worth:

```java
replay.run(this::buildCore, s -> new PhysicsSample(   // what if the IMU had been dead?
    s.timestampSeconds(), s.pose(), s.robotRelativeSpeeds(), s.moduleStates(), null, s.yawRateRadPerSec()));
```

And `DisturbanceInjector` produces faults on purpose, so a slip detector can be tested a hundred times
in a unit test rather than once on a slick patch of carpet.

---

## Known limits, stated plainly

Things Physics Core cannot do, so you are not surprised by them on a field:

- **Uniform slip is invisible to per-module scoring.** If all four wheels break loose together, every
  module reads fast, forward kinematics reads exactly as fast, and every residual is zero. The
  wheels-versus-IMU residual catches it instead — which means **a robot with no accelerometer cannot
  detect uniform slip at all.** Two checks, two halves of the problem.
- **Physics Core still never writes your pose.** Absolute observations reach it as evidence about
  confidence. One landing more than a metre from the current estimate is rejected and counted.
- **Range, bearing, and contact observations are accepted but not fused into the pose**, for the same
  reason. They reset the staleness clock and expose residuals you can watch; using them to *correct*
  position is pose fusion, which stays out.
- **Wheel-load distribution is an approximation.** Four contacts on a rigid body are statically
  indeterminate.
- **No quadratic drag, no spin, no contact model.** See the ballistics and scope notes above.
- **Diagnostic scores are not probabilities.** Ranking aid only.

---

## Adopting it, in order

Nothing here has to be adopted at once, and the order matters — each step is worth something on its
own and earns the next one.

**1. Shadow mode, one practice session.** Wire `PhysicsCore` and nothing else. It has no control
authority, so there is nothing to be careful about. Watch `Catalyst/Physics/...` in AdvantageScope.

**2. Check `SensorDisagreement` while driving normally.** It should sit near zero. A persistently
large value with no slip reported points at a calibration problem — wrong wheel radius, wrong gear
ratio, or an IMU mounted at an angle nobody told the code about. **Fix that before reading anything
else**, because every downstream number inherits it.

**3. Provoke each detector and confirm it both fires and clears.**

| Do this | Expect |
|---|---|
| Full throttle from a standstill on a slick patch | `Slip/Peak` spikes, `Slip/WorstModule` names the right corner |
| Bump the robot | `Collision/MpsSq` records it **once**, not six times |
| Cover the cameras | `Quality/Confidence` falls over ~3 s, recovers when they come back |

**4. Add the mass tree** if anything heavy moves. Check the sign on rotational links by raising the
arm and confirming `centerOfMassHeightMeters()` goes **up**.

**5. Start consuming it read-only** — a state-machine guard, a `BehaviorEngine` precondition, a shot
gate. Still no control authority; you are just letting decisions see the physics.

**6. Only then, `PhysicsConstraints`.** Log `speedScale()` alongside what you are actually commanding
for a session before you apply it, so you can see when it would have intervened and agree with it.

**7. Parameter identification is independent of all of the above** and safe to run from day one — it
cannot change anything. Let it collect over a few matches and compare its recommendations with your
constants file.

> **The rule that governs all of it:** if a Physics Core signal does not measurably outperform the
> simple check you already had, keep the simple check. That applies to everything in this package, and
> it is why the roadmap carries explicit kill criteria.
