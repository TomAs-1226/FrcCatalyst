---
layout: default
title: Advanced
nav_order: 5.5
has_children: true
---

# Advanced Features
{: .no_toc }

Competition-proven algorithms and utilities used by top FRC teams (254, 6328, 1678, 1690).
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## StateSpaceController

Optimal control using LQR (Linear Quadratic Regulator) + Kalman filter for mechanism control. This is the approach used by teams 6328 and 254 for precise mechanism control — it automatically computes optimal gains and provides lag-free noise rejection.

**Advantages over PID:**
- LQR automatically computes optimal gains given error and effort constraints
- Kalman filter provides lag-free noise rejection (vs. moving average which adds lag)
- Model-based: changes in gear ratio or motor count auto-adjust gains

### Velocity Control (Flywheels)

```java
StateSpaceController.Velocity flywheelController = StateSpaceController.createFlywheel(
    DCMotor.getKrakenX60(1), 0.01, 1.5,  // motor, MOI, gearing
    3.0,    // model std dev (how much we trust the model)
    0.01,   // encoder std dev (how much we trust the encoder)
    8.0,    // max acceptable velocity error (rad/s)
    12.0    // max voltage
);

// In periodic:
flywheelController.setReference(targetVelocityRadPerSec);
flywheelController.correct(encoderVelocityRadPerSec);
flywheelController.predict(0.020);
motor.setVoltage(flywheelController.getVoltage());
```

### Position Control (Elevators, Arms)

```java
StateSpaceController.Position elevatorController = StateSpaceController.createElevator(
    DCMotor.getKrakenX60(2), 5.0, 0.0254, 10.0,  // motor, mass, drumRadius, gearing
    0.05,   // position model std dev
    3.0,    // velocity model std dev
    0.001,  // encoder position std dev
    0.01,   // encoder velocity std dev
    0.02,   // max acceptable position error (m)
    0.4,    // max acceptable velocity error (m/s)
    12.0    // max voltage
);

// In periodic:
elevatorController.setReference(targetPosition, targetVelocity);
elevatorController.correct(encoderPosition, encoderVelocity);
elevatorController.predict(0.020);
motor.setVoltage(elevatorController.getVoltage());
```

### Factory Methods

| Method | Use Case |
|--------|----------|
| `createFlywheel()` | Velocity control from motor model |
| `createFlywheelFromGains()` | Velocity control from SysId kV/kA |
| `createElevator()` | Position control for linear mechanisms |
| `createArm()` | Position control for single-jointed arms |
| `createPositionFromGains()` | Position control from SysId kV/kA |

---

## MotionConstraintCalculator

Physics-based motion constraint calculator that computes realistic max velocities, accelerations, and torques from motor specifications and mechanism geometry. Used by top teams to set Motion Magic constraints that respect actual motor capabilities.

### Elevator Constraints

```java
var constraints = MotionConstraintCalculator.elevator(
    MotorType.KRAKEN_X60, 2,      // motor type, motor count
    10.0,                          // gear ratio
    0.0254,                        // drum radius (1 inch)
    5.0,                           // mass (kg)
    40.0                           // current limit (amps)
);

// Use the computed values
double cruiseVelocity = constraints.maxVelocityRotations * 0.8; // 80% of max
double acceleration = constraints.maxAccelerationRotations * 0.6;
double gravityFF = constraints.gravityFF; // voltage to hold position
```

### Arm Constraints

```java
var armConstraints = MotionConstraintCalculator.arm(
    MotorType.KRAKEN_X60, 1,   // motor type, count
    50.0,                       // gear ratio
    0.5,                        // arm length (m)
    3.0,                        // mass (kg)
    40.0,                       // current limit (amps)
    130.0                       // range of motion (degrees)
);

double peakGravityFF = armConstraints.peakGravityFF; // at horizontal
double travelTime = armConstraints.fullTravelTime;    // estimated full-range time
```

### Flywheel Constraints

```java
var fwConstraints = MotionConstraintCalculator.flywheel(
    MotorType.KRAKEN_X60, 2, 1.5, 0.01, 40.0, 0.05
);

double maxRPS = fwConstraints.maxVelocityRPS;
double spinUpTime = fwConstraints.spinUpTime90Percent;
double surfaceSpeed = fwConstraints.surfaceSpeedMPS;
```

---

## SignalProcessor

Advanced signal processing filters for sensor data conditioning, providing better alternatives to simple moving averages.

### ExponentialMovingAverage

Responds faster than a simple moving average while still smoothing noise:

```java
// From alpha (0.1 = heavy smoothing, 0.9 = light)
var ema = new SignalProcessor.ExponentialMovingAverage(0.3);

// From time constant (more intuitive)
var ema2 = SignalProcessor.ExponentialMovingAverage.fromTimeConstant(0.1, 0.020);

double filtered = ema.calculate(rawSensorValue);
```

### MedianFilter

Removes impulse noise (spikes) while preserving edges — use for sensors with occasional outliers:

```java
var median = new SignalProcessor.MedianFilter(5); // 5-sample window
double cleaned = median.calculate(noisySensor);
```

### LowPassFilter

First-order IIR filter specified by cutoff frequency:

```java
var lowPass = new SignalProcessor.LowPassFilter(10.0, 0.020); // 10 Hz cutoff
double smooth = lowPass.calculate(rawValue);
```

### CompositeFilter

The recommended approach for noisy FRC sensors — chains median (removes spikes) then low-pass (smooths):

```java
var filter = new SignalProcessor.CompositeFilter(5, 10.0, 0.020);
double result = filter.calculate(noisySensor); // spike-free + smooth
```

### RateOfChange

Calculates the derivative of a signal with built-in smoothing:

```java
var rateCalc = new SignalProcessor.RateOfChange(0.3); // smoothing alpha
double velocity = rateCalc.calculate(position, Timer.getFPGATimestamp());
```

---

## PoseHistory

Temporal pose tracking with interpolation for latency compensation. Stores timestamped poses and provides interpolated lookups at arbitrary past timestamps.

```java
PoseHistory history = new PoseHistory(1.5); // 1.5 seconds of history

// In periodic:
history.addSample(drive.getPose());

// Get pose at a past timestamp (for vision latency compensation):
Pose2d pastPose = history.getPoseAtTime(visionTimestamp);

// Velocity estimation from pose differences:
double speed = history.getTranslationalVelocity();       // m/s
Translation2d velVector = history.getVelocityVector();     // (vx, vy)
double angularVel = history.getAngularVelocity();          // rad/s

// Distance and heading change since a past time:
double distTraveled = history.getDistanceSince(startTimestamp);
double headingChange = history.getHeadingChangeSince(startTimestamp);
```

---

## DynamicAutoBuilder

Runtime path generation using PathPlanner's on-the-fly capabilities. Generates paths at runtime from the robot's current position to target poses — the approach used by top teams for adaptive autonomous routines.

{: .note }
Requires PathPlanner's `AutoBuilder` to be configured first (done automatically by `SwerveSubsystem`).

### Pathfind to a Pose

```java
PathConstraints constraints = DynamicAutoBuilder.defaultConstraints(3.0, 2.0);
Command driveTo = DynamicAutoBuilder.pathfindToPose(
    new Pose2d(5.0, 2.0, Rotation2d.fromDegrees(0)), constraints);
```

### Pathfind Then Follow a Pre-made Path

```java
Command autoScore = DynamicAutoBuilder.pathfindThenFollowPath("ScorePath", constraints);
```

### Generate Path Through Waypoints

```java
Command complexPath = DynamicAutoBuilder.generatePath(
    drive::getPose,                  // current pose supplier
    List.of(
        new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45)),
        new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(90))
    ),
    constraints,
    Rotation2d.fromDegrees(90),      // end heading
    0.0                              // end velocity (0 = stop)
);
```

### Alliance Mirroring

```java
// Automatically mirrors for red alliance
Pose2d scorePose = DynamicAutoBuilder.alliancePose(
    new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(0))
);

// Manual mirror
Pose2d redPose = DynamicAutoBuilder.mirrorForRed(bluePose);
```

---

## Enhanced Swerve Drive

The `SwerveSubsystem` now includes advanced drive features used by top teams:

### Skew Correction

Team 1690's pose exponential discretization method — corrects translation skew during combined translation and rotation:

```java
drive.setSkewCorrectionEnabled(true);
```

### Slew Rate Limiting

Smooth acceleration/deceleration profiles to prevent wheel slip:

```java
// Symmetric limiting
drive.enableSlewRateLimiting(3.0); // 3 units/sec

// Asymmetric (accelerate slower, brake harder)
drive.enableSlewRateLimiting(2.0, 5.0); // accel rate, decel rate
```

### Snap-to-Angle

Auto-snap to predefined angles when the robot heading is within tolerance:

```java
drive.setSnapToAngles(
    List.of(0.0, 90.0, 180.0, 270.0), // snap angles
    5.0                                  // tolerance (degrees)
);
```

### Advanced Drive Command

Combines deadband, slew limiting, heading lock, snap-to-angle, skew correction, and speed multiplier into one optimized drive command:

```java
drive.setDefaultCommand(drive.advancedDrive(
    () -> -driver.getLeftY(),
    () -> -driver.getLeftX(),
    () -> -driver.getRightX(),
    0.05                          // deadband
));
```

### Slow Mode

Toggle slow mode for precise alignment:

```java
drive.setSpeedMultiplier(0.3); // 30% speed
driver.leftBumper().whileTrue(drive.slowModeWhileHeld(0.3));
```

### Auto-Align Drive

Drive normally while auto-rotating to face a target:

```java
driver.rightBumper().whileTrue(drive.autoAlignDrive(
    () -> -driver.getLeftY(),
    () -> -driver.getLeftX(),
    () -> targetHeadingDegrees,
    0.05
));
```

---

## Enhanced Vision

The `VisionSubsystem` now includes advanced filtering used by top teams:

- **Ambiguity-scaled standard deviations** — higher ambiguity = less trust
- **High-speed rejection** — ignores vision measurements when driving fast
- **Heading divergence filtering** — rejects single-tag poses that disagree with the gyro
- **Kalman innovation tracking** — logs the innovation norm for tuning

```java
VisionConfig.builder()
    .addLimelight("limelight-front", frontCameraPose)
    .singleTagStdDevs(4, 8)
    .multiTagStdDevs(0.5, 1)
    .rejectDuringHighSpeed(3.0)       // reject when > 3 m/s
    .maxHeadingDivergence(15.0)       // reject if heading disagrees > 15 deg
    .fieldDimensions(16.54, 8.21)     // custom field bounds
    .build();
```

---

## Enhanced SuperstructureCoordinator

The state machine coordinator now supports advanced patterns used by team 254:

### Collision Zones

Define unsafe mechanism configurations to prevent physical collisions:

```java
superstructure.addCollisionZone("ElevatorArmConflict",
    () -> elevator.getPosition() < 0.3 && arm.getAngle() > 45.0
);
```

### Timeouts

Safety timeout that automatically stops transitions:

```java
superstructure.withTimeout(3.0); // 3 second transition timeout
```

### Entry/Exit Actions

Run code when entering or leaving a state:

```java
superstructure.defineState("SCORE_HIGH")
    .setLinear("elevator", 1.1)
    .setRotational("arm", 95.0)
    .onEntry(() -> leds.setPattern(Color.kGreen))
    .onExit(() -> leds.setPattern(Color.kBlue))
    .done();
```

### Conditional Transitions

Only transition if a condition is met:

```java
superstructure.transitionToIf("SCORE_HIGH",
    () -> intake.hasGamePiece());
```

### Telemetry

Automatic NetworkTables publishing of current state, target state, transition progress, and collision zone status.

---

## Enhanced LED Patterns

Eight new pattern commands beyond the original set:

| Pattern | Description |
|---------|-------------|
| `fire()` | Realistic fire/flame animation |
| `gradient(color1, color2)` | Static two-color gradient |
| `scrollingGradient(color1, color2, speed)` | Moving gradient wave |
| `strobe(color, hz)` | High-frequency flashing |
| `larsonScanner(color, width)` | Cylon/KITT-style scanner |
| `dynamicProgress(color, progress)` | Dynamic progress bar (0-1) |
| `statusIndicator(good, warn, bad)` | Multi-zone status indicator |
| `alignmentIndicator(offset, tolerance)` | Auto-alignment visual feedback |

```java
// Fire effect for celebration
leds.fire();

// Show scoring progress
leds.dynamicProgress(Color.kGreen, () -> elevator.getPosition() / 1.2);

// Alignment feedback for driver
leds.alignmentIndicator(() -> visionOffset, 2.0);
```
