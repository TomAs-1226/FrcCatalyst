---
layout: default
title: Subsystems
nav_order: 4
has_children: false
---

# Subsystems
{: .no_toc }

FrcCatalyst provides three complex subsystem wrappers that integrate multiple components.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## SwerveSubsystem

Wraps CTRE Tuner X generated swerve code. Teams generate their drivetrain using Tuner X, then wrap it with `SwerveSubsystem` to get:

- **Field-centric and robot-centric drive** commands
- **Heading lock** — auto-holds heading when driver isn't rotating
- **Point-at-target** — always face a scoring target while translating
- **Drive-with-heading** — lock to a specific heading angle
- **PathPlanner integration** — one-line AutoBuilder configuration
- **Vision pose estimation** — `addVisionMeasurement()` bridge
- **Automatic telemetry** — pose, heading, speed to NetworkTables
- **Skew correction** — pose exponential discretization
- **Slew rate limiting** — smooth acceleration with asymmetric profiles
- **Snap-to-angle** — auto-snap heading to predefined angles
- **Advanced drive** — combined deadband, slew, heading lock, skew correction, snap-to-angle
- **Slow mode** — toggleable speed multiplier for precision
- **Auto-align** — drive while auto-rotating to a target heading

```java
SwerveSubsystem drive = new SwerveSubsystem(
    TunerConstants.createDrivetrain(),
    4.5,
    SwerveSubsystem.PathPlannerConfig.builder()
        .translationPID(5.0, 0, 0)
        .rotationPID(5.0, 0, 0)
        .build()
);

// Advanced drive (recommended default command)
// Combines deadband, slew limiting, heading lock, snap-to-angle, and skew correction
drive.setSkewCorrectionEnabled(true);
drive.enableSlewRateLimiting(2.0, 5.0); // accel, decel
drive.setSnapToAngles(List.of(0.0, 90.0, 180.0, 270.0), 5.0);
drive.setDefaultCommand(drive.advancedDrive(
    () -> -driver.getLeftY(),
    () -> -driver.getLeftX(),
    () -> -driver.getRightX(),
    0.05
));

// Slow mode for precision alignment
driver.leftBumper().whileTrue(drive.slowModeWhileHeld(0.3));

// Point at speaker while driving
driver.rightBumper().whileTrue(drive.pointAtTarget(
    () -> -driver.getLeftY(),
    () -> -driver.getLeftX(),
    () -> new Translation2d(0.0, 5.55),
    0.05
));
```

{: .tip }
See the [Advanced Features](../advanced/) section for detailed documentation on skew correction, slew rate limiting, snap-to-angle, and auto-align.

### PathPlanner support

`SwerveSubsystem` configures PathPlanner's `AutoBuilder` for you when you
pass a `PathPlannerConfig` (as above). It wires:

- `getPose` / `resetPose` — pose source + reset
- `getChassisSpeeds` — **robot-relative** speeds (what PathPlanner expects)
- a robot-relative `ChassisSpeeds` consumer for path output
- a `PPHolonomicDriveController` from your translation/rotation PID
- `RobotConfig.fromGUISettings()` — mass, MOI, module config from the
  PathPlanner GUI
- alliance flipping (mirrors paths for the red alliance automatically)

Once configured, named autos load with one call:

```java
autonomousCommand = AutoBuilder.buildAuto("MyAuto");
// or follow a single path:
new PathPlannerAuto("MyAuto").schedule();
```

Both `pathfindToPose(...)` (below) and the [behavior framework](../advanced/behavior.html)
autos build on this. If `AutoBuilder` isn't configured (you didn't pass a
`PathPlannerConfig`), pathfinding commands fall back to PID-only and print a
DS error rather than crashing.

> **Robot- vs field-relative:** `getChassisSpeeds()` is robot-relative for
> PathPlanner. For Shoot-On-The-Fly use `getFieldRelativeSpeeds()`, which
> rotates it into the field frame.

### pathfindToPose (v0.3.6.1+)

Pathfind to a pose with PathPlanner's `AutoBuilder.pathfindToPose`, then
hand off to the existing precision-align PID for the last metre.

```java
swerve.pathfindToPose(() -> ScoringPoses.RED_L2);                 // defaults: kP=4.0, tolerance=2 cm
swerve.pathfindToPose(() -> target, 5.0, 0.015);                  // tighter tolerance
swerve.pathfindToPose(() -> target, new PathConstraints(3, 2, …)); // your own constraints
```

If `AutoBuilder` isn't configured the command falls back to PID-only
align and prints a driver-station error explaining why — no crash.

### Choreo paths (v0.8.0+)

Follow [Choreo](https://sleipnirgroup.github.io/Choreo/)'s time-optimal
trajectories — loaded through PathPlanner, so **no extra vendordep** (Choreo
exports, PathPlanner follows). Put the `.traj` files in
`src/main/deploy/choreo/`:

```java
autonomousCommand = swerve.followChoreoPath("FourPieceFar");
```

Needs `AutoBuilder` configured (the `PathPlannerConfig` constructor). If the
trajectory can't be loaded it reports to the driver station and returns a
no-op instead of crashing.

### driveToPiece — vision pursuit (v0.8.0+)

The primitive the [Autopilot](../advanced/behavior.html) "acquire" action
wants. Give it a supplier of the detected piece's field position (empty when
your coprocessor sees nothing) and it drives onto it:

```java
swerve.driveToPiece(() -> vision.nearestFuelPose());          // defaults kP=3, tol=0.2m
swerve.driveToPiece(() -> vision.nearestFuelPose(), 4.0, 0.15);
```

Stops when it arrives or the piece disappears. `vision.nearestFuelPose()`
is your team-side detection method (`Supplier<Optional<Translation2d>>`).

### WheelRadiusCalibration (v0.8.0+)

Measures the **actual** wheel radius by spinning the robot — correcting the
CAD value, a documented source of autonomous inaccuracy (tread wears and
compresses). Compares the gyro arc to the distance odometry thinks each
wheel rolled:

```java
test.a().onTrue(WheelRadiusCalibration.builder(swerve)
    .currentWheelRadius(0.0508)   // your configured radius (m)
    .driveBaseRadius(0.42)        // center → module distance (m)
    .rotations(4)
    .build());
```

The corrected radius + a copy-paste constant publish to
`/Catalyst/Calibration/WheelRadius/...`.

### SwerveSetpointGenerator (v0.4.0+)

Light chassis-aware accel/skid clamp. Wraps a `ChassisSpeeds` and
returns one limited by max wheel speed, max angular rate, and a
per-second delta-v cap.

```java
SwerveSetpointGenerator gen = new SwerveSetpointGenerator(
    drive.getMaxSpeedMPS(), drive.getMaxAngularRate(), 8.0); // 8 m/s² accel cap

ChassisSpeeds limited = gen.generate(requestedSpeeds);
drivetrain.setControl(req.withVelocityX(limited.vxMetersPerSecond)
                        .withVelocityY(limited.vyMetersPerSecond)
                        .withRotationalRate(limited.omegaRadiansPerSecond));
```

Catches the most common driver-induced skid (jerking the stick from
full-forward to full-right) without doing a full per-wheel feasibility
solve. Cheap.

## VisionSubsystem

Multi-camera pose estimation with Kalman filter integration. Supports both Limelight (MegaTag2) and PhotonVision cameras simultaneously.

**Features:**
- **Distance-scaled standard deviations** — trusts close targets more
- **Ambiguity-scaled std devs** — higher ambiguity = less trust
- **Spin rejection** — ignores vision during fast rotation
- **High-speed rejection** — ignores vision while driving fast
- **Heading divergence filtering** — rejects single-tag poses that disagree with the gyro
- **Kalman innovation tracking** — logs innovation norms for tuning
- **Latency filtering** — rejects stale measurements
- **Configurable field bounds** — custom field dimensions for bounds checking
- **Per-cycle telemetry** — see which estimates are accepted/rejected with reasons

```java
VisionSubsystem vision = new VisionSubsystem(
    VisionConfig.builder()
        .driveSubsystem(drive)                                    // wires pose fusion
        .addLimelight("limelight-front", frontCameraPose)
        .addPhotonCamera("cam-rear", rearCameraPose, fieldLayout) // Photon needs the tag layout
        .singleTagStdDevs(4, 8)
        .multiTagStdDevs(0.5, 1)
        .xyDistanceScaling(1.0)
        .rotDistanceScaling(1.5)
        .rejectDuringSpin(2.0)
        .rejectDuringHighSpeed(3.0)       // reject when > 3 m/s
        .maxHeadingDivergence(15.0)       // reject if heading disagrees > 15 deg
        .fieldDimensions(16.54, 8.21)     // custom field bounds
        .maxLatency(0.5)
        .build()
);
```

> Pass 4+ cameras and they're fused deterministically — each is filtered
> independently and added to the pose estimator in timestamp order with
> quality/index tiebreaks, so the result is reproducible run-to-run.

### LimelightTriggers (v0.4.0+)

Wrap a Limelight's NetworkTables keys as WPILib `Trigger`s. Point at
the table name, bind, done.

```java
LimelightTriggers front = new LimelightTriggers("limelight-front");

front.hasTarget().onTrue(leds.solid(Color.kGreen));
front.tagInView(7).whileTrue(swerve.pathfindToPose(() -> SCORE_7));
front.detectorClass("note").onTrue(intake.intakeCommand());
front.horizontalErrorBelow(2.0).onTrue(rumble.fire(Pattern.DOUBLE_TAP, Channel.DRIVER));
front.targetWithinArea(2.0).onTrue(climber.armCommand());
```

| Trigger | Backed by |
|---|---|
| `hasTarget()`                   | `tv > 0.5` |
| `tagInView(int)`                | `tv > 0.5 && tid == id` |
| `detectorClass(String)`         | `tv > 0.5 && tclass == name` |
| `horizontalErrorBelow(double)`  | `\|tx\| <= degrees` |
| `targetWithinArea(double)`      | `ta >= percent` |

Plus diagnostic readers: `tx()`, `ty()`, `ta()`, `tid()`,
`latencyMs()` for direct polling when you don't want a trigger.

## LEDSubsystem

Addressable LED pattern controller with 14 pre-built effects.

**Basic patterns:** solid, blink, rainbow, chase, breathe, alternating

**Advanced patterns:** fire, gradient, scrolling gradient, strobe, larson scanner, dynamic progress, status indicator, alignment indicator

```java
LEDSubsystem leds = new LEDSubsystem(
    LEDConfig.builder()
        .port(0)
        .length(60)
        .build()
);

// Alliance color by default
leds.setDefaultCommand(leds.solid(Color.kBlue));

// Rainbow when scoring
scoring.whileTrue(leds.rainbow());

// Blink green when game piece acquired
intake.hasPieceTrigger().whileTrue(leds.blink(Color.kGreen, 0.1));

// Fire effect for celebration
scoring.whileTrue(leds.fire());

// Alignment indicator for driver (Color + 0..1 progress supplier)
aligning.whileTrue(leds.alignmentIndicator(Color.kGreen, () -> alignProgress));

// Progress bar for elevator height
leds.dynamicProgress(Color.kGreen, () -> elevator.getPosition() / 1.2);
```

{: .tip }
See the [Advanced Features](../advanced/) section for details on all new LED patterns.
