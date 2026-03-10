---
layout: default
title: Utilities
nav_order: 6
---

# Utilities

FrcCatalyst includes a comprehensive set of utilities commonly needed in FRC programming.

## FeedforwardGains

Stores and calculates feedforward voltages for different mechanism types.

```java
// From SysId results:
FeedforwardGains elevatorFF = FeedforwardGains.elevator(0.12, 2.5, 0.1, 0.35);
FeedforwardGains armFF = FeedforwardGains.arm(0.15, 1.8, 0.05, 0.5);
FeedforwardGains flywheelFF = FeedforwardGains.simple(0.12, 0.11);

// Calculate voltages:
double holdVoltage = elevatorFF.calculateElevator();             // static hold
double moveVoltage = elevatorFF.calculateElevator(1.5);          // at 1.5 m/s
double armHold = armFF.calculateArm(Math.toRadians(45));         // at 45 degrees
double shootVoltage = flywheelFF.calculateSimple(70.0);          // at 70 RPS
```

## TrapezoidProfileHelper

Factory methods for WPILib `ProfiledPIDController`:

```java
// Linear mechanism profile
ProfiledPIDController elevatorPID = TrapezoidProfileHelper.createLinear(
    50, 0, 0.5,   // PID gains
    2.0, 4.0      // max velocity (m/s), max accel (m/s^2)
);

// Rotational mechanism profile
ProfiledPIDController armPID = TrapezoidProfileHelper.createRotational(
    80, 0, 1.0,   // PID gains
    1.5, 3.0      // max velocity (rot/s), max accel (rot/s^2)
);

// Continuous rotation (turret, 360-degree)
ProfiledPIDController turretPID = TrapezoidProfileHelper.createContinuousRotational(
    40, 0, 0.5, 2.0, 4.0
);
```

## AlertManager

Centralized fault and warning system. Publishes to NetworkTables under `Catalyst/Alerts/`.

```java
AlertManager alerts = AlertManager.getInstance();

// Report faults from anywhere
alerts.error("Elevator", "Motor overtemp cutoff at 85C!");
alerts.warning("Intake", "Stall detected - game piece stuck?");
alerts.info("Vision", "No AprilTags visible");

// Clear faults
alerts.clearSubsystem("Elevator");
```

## CharacterizationHelper

One-line SysId characterization setup:

```java
CharacterizationHelper charHelper = new CharacterizationHelper(
    "Elevator", elevator,
    volts -> elevator.getMotor().setVoltage(volts),
    () -> elevator.getPosition(),
    () -> elevator.getVelocity()
);

// Four commands for the SysId routine:
charHelper.quasistaticForward();
charHelper.quasistaticReverse();
charHelper.dynamicForward();
charHelper.dynamicReverse();
```

## MechanismVisualizer

Dashboard visualization using WPILib's Mechanism2d:

```java
MechanismVisualizer viz = new MechanismVisualizer("Robot");
var elevatorViz = viz.addElevator("Elevator", 1.2, Color.kBlue);
var armViz = viz.addArm("Arm", elevatorViz, 0.5, Color.kRed);

// In periodic:
elevatorViz.setLength(elevator.getPosition());
armViz.setAngle(arm.getAngle());
```

## CatalystMath

Joystick processing, angle math, and geometry helpers:

```java
// Complete joystick processing pipeline
double output = CatalystMath.processJoystick(
    joystick.getY(), // raw input
    0.05,            // deadband
    2.0,             // square curve
    0.7              // 70% max speed (slow mode)
);

// Angle math
double diff = CatalystMath.angleDifference(90, 270);  // -180
boolean close = CatalystMath.angleWithinTolerance(89, 90, 2); // true

// Geometry
double dist = CatalystMath.distanceBetween(robotPose, targetPose);
Pose2d mirrored = CatalystMath.mirrorPose(bluePose); // for red alliance

// Physics estimation
double kG = CatalystMath.elevatorGravityFF(5.0, 0.0254, 10.0, 7.09);
```

## InterpolatingTable

TreeMap-based linear interpolation for shooter distance tables:

```java
InterpolatingTable shooterTable = new InterpolatingTable();
shooterTable.put(1.0, 3000);  // 1m -> 3000 RPM
shooterTable.put(2.0, 3500);  // 2m -> 3500 RPM
shooterTable.put(3.0, 4200);  // 3m -> 4200 RPM
shooterTable.put(5.0, 5000);  // 5m -> 5000 RPM

double rpm = shooterTable.get(2.5); // interpolates to ~3850 RPM
```

## SlewRateLimiter

Asymmetric rate limiter with different acceleration and deceleration profiles:

```java
SlewRateLimiter limiter = new SlewRateLimiter(
    3.0,  // max acceleration (units/sec)
    5.0   // max deceleration (units/sec) - brake harder than accelerate
);

// In periodic:
double smoothed = limiter.calculate(targetSpeed);
```

## MovingAverage

Sliding window average filter for noisy sensor data:

```java
MovingAverage filter = new MovingAverage(10); // 10-sample window

// In periodic:
filter.add(noisySensorValue);
double smooth = filter.getAverage();
```

## TimedBoolean

Debounced boolean with configurable duration threshold:

```java
TimedBoolean stallDetector = new TimedBoolean(0.2); // 200ms threshold

// In periodic:
boolean isStalled = stallDetector.update(current > 30.0);

// Edge detection:
boolean justStalled = stallDetector.risingEdge(current > 30.0);
boolean justCleared = stallDetector.fallingEdge(current > 30.0);
```
