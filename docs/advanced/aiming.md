---
layout: default
title: Turret & Shoot-On-The-Fly
parent: Advanced
nav_order: 5
---

# Turret & Shoot-On-The-Fly
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## The pieces

Catalyst splits turret aiming into two parts so the math is testable
without hardware:

| Class | Where | What it does |
|---|---|---|
| `AimingSolver` | `util/` | Pure geometry, time-of-flight approach. Robot pose + field velocity → field-relative aim bearing, distance, flight time, shooter RPM, hood angle. No motors. |
| `AimingSolverVector` | `util/` | Pure geometry, **3D vector-adding** approach (added in 1.2.1). Same inputs → hood pitch, yaw, and flywheel RPS, solved by subtracting the robot's velocity from the shot vector. No motors. |
| `TurretMechanism` | `mechanisms/` | The motor wrapper. Continuous-angle resolution (wrap / limit unwrap), field-relative tracking, vision lock. |

You can use either alone — `AimingSolver` to aim a swerve chassis at a
target, `TurretMechanism` for a turret that tracks a vision error
directly — but together they do shoot-while-moving.

> **Two solvers, two approaches.** `AimingSolver` leads a moving virtual goal by flight time and
> hands back a bearing (and bearing rate) — pair it with `TurretMechanism.track(...)`.
> `AimingSolverVector` (contributed in [#27](https://github.com/TomAs-1226/FrcCatalyst/pull/27),
> following the Huskie Robotics
> [shoot-on-the-move write-up](https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805))
> instead treats the shot as a velocity vector and also solves the hood angle and wheel speed. Pick
> whichever matches how your shooter is characterised — both are pure math and unit-tested.

---

## TurretMechanism

```java
TurretMechanism turret = new TurretMechanism(
    TurretMechanism.Config.builder()
        .name("Turret")
        .motor(15)
        .gearRatio(40.0)            // motor rotations per turret rotation
        .range(-200, 200)           // mechanical travel in degrees (wider than ±180 = overlap)
        .pid(40, 0, 0.5)
        .feedforward(0.15, 0.0)
        .motionMagic(8, 16, 80)     // turret rotations: rps, rps², rps³
        .tolerance(1.0)
        .cancoder(16, 40.0)         // optional: absolute homing, no boot-time zero needed
        .build());
```

### Continuous-angle resolution

The thing every turret team fights: the target sweeps across the robot,
the turret has finite travel, and a naive `target − heading` command will
either exceed a soft limit or spin the wiring off.

`TurretMechanism` resolves every aim request through
`resolveTurretAngle(desired, current, min, max)`:

- Considers every equivalent representation `desired + 360·k`.
- Picks the in-range one **closest to the current position** (shortest move).
- Only when the short way is blocked by a limit does it **unwrap** the
  long way around.
- If the target is genuinely unreachable (e.g. directly behind a ±150°
  turret), it clamps to the nearest limit rather than fighting itself.

`isUnwrapping()` and the `Unwrapping` NT key flag when it's taking the
long way, so you can hold fire during the swing.

The method is `static` and pure — unit-test it directly:

```java
assertEquals(185, TurretMechanism.resolveTurretAngle(-175, 195, -200, 200), 1e-6);
```

### Command factories

| Command | Use |
|---|---|
| `goToAngle(deg)` | Raw robot-relative angle, wrap-resolved |
| `lockForward()` | Point straight ahead (0°) |
| `holdAngle()` | Hold current setpoint — good default command |
| `aimAtFieldAngle(field, heading)` | Stay locked on a field bearing as the chassis spins |
| `aimAtTarget(poseSupplier, target)` | Aim at a field point (no motion comp) |
| `track(solutionSupplier, heading)` | Full SOTF — feed it `AimingSolver` |
| `aimWithVision(hasTarget, errorDeg)` | Close the loop on a camera error instead of odometry |
| `zero(deg)` | Seed the encoder for homing |

---

## AimingSolver — the SOTF math

When the robot is moving, the game piece inherits the chassis velocity.
To still hit a stationary goal you aim at a **virtual goal** shifted
opposite to your motion by one time-of-flight:

```
virtualGoal = target − v_field · timeOfFlight
```

Flight time depends on the distance to the *virtual* goal, not the real
one, so the solve iterates to converge (3 passes is plenty):

```java
AimingSolver solver = AimingSolver.builder()
    .target(FieldConstants.GOAL_CENTER)             // alliance-resolved Translation2d
    .shotTime(new InterpolatingTable()              // distance(m) → flight time(s)
        .add(1.5, 0.28).add(3.0, 0.42).add(5.0, 0.61))
    .shooterRpm(new InterpolatingTable()
        .add(1.5, 2600).add(3.0, 3400).add(5.0, 4200))
    .hoodAngle(new InterpolatingTable()
        .add(1.5, 12).add(3.0, 28).add(5.0, 40))
    .build();
```

You fill the tables from your own range testing — park at known
distances, tune RPM + hood until shots land, record the flight time off
a match video or the shooter's at-speed-to-impact delay.

### Wiring it to the turret

```java
// Recompute the SOTF solution each loop:
Supplier<AimingSolver.Solution> sol =
    () -> solver.solve(drive.getPose(), drive.getFieldRelativeSpeeds());

// Turret leads the moving goal with exact analytic-rate feedforward. Pass the
// chassis yaw rate too, so it stays locked while you ALSO rotate:
turret.setDefaultCommand(turret.track(
    sol,
    () -> drive.getHeading().getDegrees(),
    () -> Math.toDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond)));

// Flywheel RPM and hood angle follow the live distance every loop (v1.0):
shooter.setDefaultCommand(shooter.track(() -> sol.get().shooterRpm() / 60.0));  // RPM -> RPS
hood.setDefaultCommand(hood.track(() -> sol.get().hoodDegrees()));

// "Ready to shoot" — barrel on target AND wheel at speed AND in range:
boolean ready = turret.isOnTarget(sol.get(), drive.getHeading().getDegrees(), 2.0)
             && shooter.atSpeed();
```

> Field-relative speeds matter. If your drive only gives robot-relative
> `ChassisSpeeds`, rotate them first:
> `ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drive.getHeading())`.

### Reading the solution

```java
AimingSolver.Solution s = solver.solve(pose, fieldSpeeds);
s.turretFieldAngleDeg();  // where the bore should point (field frame)
s.distanceMeters();       // distance to the virtual goal — use for RPM/hood
s.shotTimeSeconds();      // estimated flight time
s.shooterRpm();           // looked up (0 if no table)
s.hoodDegrees();          // looked up (0 if no table)
s.virtualGoal();          // the motion-compensated aim point (for AdvantageScope)
s.feasible();             // false if the robot is essentially on the target
```

A clean "ready to fire" gate:

```java
boolean ready = turret.atSetpoint()
    && shooter.atSpeed()
    && solver.solve(pose, fieldSpeeds).feasible();
```

### Alliance flip

Resolve the target before handing it to the solver, and update it on
alliance change:

```java
RobotState.alliance() == DriverStation.Alliance.Red
    ? solver.setTarget(FieldConstants.RED_GOAL)
    : solver.setTarget(FieldConstants.BLUE_GOAL);
```

---

## Defense & live compensation

`ShotCompensation` is the module you reach for when shots are
consistently off, the field element has drifted, or a defender is
shoving you and the velocity estimate gets noisy. Attach it to the
solver and it's applied on top of every solve:

```java
ShotCompensation comp = ShotCompensation.builder()
    .name("Turret")
    .velocityDeadband(0.08)      // ignore chassis speed below 0.08 m/s
    .maxCompensatedSpeed(4.0)    // clamp collision spikes
    .build();

AimingSolver solver = AimingSolver.builder()
    .target(GOAL).shotTime(shotTimeTable).shooterRpm(rpmTable).hoodAngle(hoodTable)
    .maxRange(6.5)               // mark shots past 6.5 m infeasible
    .compensation(comp)
    .build();

// Operator dials — a D-pad is the classic binding:
operator.povUp()   .onTrue(Commands.runOnce(() -> comp.nudgeHoodBias(+1)));
operator.povDown() .onTrue(Commands.runOnce(() -> comp.nudgeHoodBias(-1)));
operator.povRight().onTrue(Commands.runOnce(() -> comp.nudgeTurretBias(+0.5)));
operator.povLeft() .onTrue(Commands.runOnce(() -> comp.nudgeTurretBias(-0.5)));
operator.start()   .onTrue(Commands.runOnce(comp::reset));

// Getting shoved? Trust SOTF less while the bumper is held:
operator.leftBumper().onTrue (Commands.runOnce(() -> comp.setVelocityScale(0.5)));
operator.leftBumper().onFalse(Commands.runOnce(() -> comp.setVelocityScale(1.0)));
```

| Knob | What it does |
|---|---|
| `turretBias` (deg)     | added to the final aim bearing |
| `distanceBias` (m)     | added to the lookup distance — shoot longer/shorter without re-aiming |
| `rpmBias` / `hoodBias` | added to the looked-up shooter values |
| `velocityScale` [0–2]  | SOTF aggressiveness. 1 = full comp, 0 = aim as if stationary |
| `velocityDeadband` (m/s)| chassis speed below this is treated as zero (noise rejection) |
| `maxCompensatedSpeed` (m/s)| the comp speed is clamped here so a collision spike can't fling the turret |

The deadband and clamp are the defense-robustness pieces: a hit that
makes the pose estimator briefly report a huge velocity won't throw the
aim, because the speed feeding the virtual-goal shift is bounded. Values
publish to `/Catalyst/Aiming/<name>/...` so the dashboard shows the
current dialed-in bias.

## Turret velocity feedforward

`TurretMechanism.track(...)` doesn't just chase the solution a loop
behind — it applies a `kV` voltage feedforward so the turret *leads* a
moving goal. As of **v1.0** the tracking path uses the solver's **exact
analytic field-bearing rate** (`Solution.turretFieldRateDps()`) rather
than a numeric derivative of the command, so the feedforward has no
finite-difference lag or noise. Set `kV` from [SysId](sysid.html); the
feedforward is skipped on an unwrap loop and clamped to ±2 V so it
assists rather than dominates the PID.

If the chassis **rotates while shooting**, use the 3-arg overload
`track(solution, headingDeg, yawRateDps)`. The turret's robot-relative
rate is `fieldRate − yawRate`, so feeding the yaw rate keeps the
feedforward exact while you spin — the turret counter-rotates and stays
locked on the goal.

The flywheel and hood get the same continuous treatment:
`FlywheelMechanism.track(DoubleSupplier rps)` and
`RotationalMechanism.track(DoubleSupplier deg)` re-read their setpoints
every loop, so RPM and hood angle follow the live distance during SOTF.

## Latency

`AimingSolver` uses whatever pose you hand it. For best accuracy feed a
latency-compensated pose — sample your fused estimator at the vision
timestamp using [`PoseHistory`](../utilities/) — so the aim reflects
where the robot actually was when the camera frame was captured, not one
loop later.

---

## Tuning checklist

1. **Mechanically zero the turret** (or wire a CANcoder) so `0°` really is
   robot-forward.
2. **Characterize** with [SysId](sysid.html) for `kS/kV/kA`, then tune `kP/kD`.
3. **Set `range(...)`** to the real mechanical limits. Use overlap past
   ±180 if your wiring allows — it reduces how often the turret has to
   unwrap.
4. **Build the shot-time table** before the RPM/hood tables — SOTF is
   only as good as the flight-time estimate.
5. **Validate static first** (`solveStatic`), then enable motion comp and
   confirm shots stay on while strafing.
