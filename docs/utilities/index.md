---
layout: default
title: Utilities
nav_order: 5
---

# Utilities
{: .no_toc }

FrcCatalyst includes a comprehensive set of utilities commonly needed in FRC programming.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## FeedforwardGains

Stores and calculates feedforward voltages for different mechanism types. Create them from your SysId results or use manual estimates.

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

Factory methods for WPILib `ProfiledPIDController` — saves you from remembering the constraint constructors:

```java
// Linear mechanism profile (meters)
ProfiledPIDController elevatorPID = TrapezoidProfileHelper.createLinear(
    50, 0, 0.5,   // PID gains
    2.0, 4.0      // max velocity (m/s), max accel (m/s^2)
);

// Rotational mechanism profile (degrees)
ProfiledPIDController armPID = TrapezoidProfileHelper.createRotational(
    80, 0, 1.0,   // PID gains
    1.5, 3.0      // max velocity (rot/s), max accel (rot/s^2)
);

// Continuous rotation (turret, 360-degree wrapping)
ProfiledPIDController turretPID = TrapezoidProfileHelper.createContinuousRotational(
    40, 0, 0.5, 2.0, 4.0
);
```

## AlertManager

Centralized fault and warning system. Publishes to NetworkTables under `Catalyst/Alerts/` so you can monitor robot health on the dashboard.

```java
AlertManager alerts = AlertManager.getInstance();

// Report faults from anywhere in your code
alerts.error("Elevator", "Motor overtemp cutoff at 85C!");
alerts.warning("Intake", "Stall detected - game piece stuck?");
alerts.info("Vision", "No AprilTags visible");

// Check and clear faults
if (alerts.hasErrors()) {
    // handle errors
}
alerts.clearSubsystem("Elevator");
alerts.clearAll();
```

## CharacterizationHelper

One-line SysId characterization setup. Pass the mechanism and its motor — the helper creates all four SysId routines automatically:

```java
CharacterizationHelper charHelper = new CharacterizationHelper(
    "Elevator",          // name for SysId logging
    elevator,            // subsystem (extends SubsystemBase)
    elevator.getMotor()  // CatalystMotor for voltage control
);

// Four commands for the SysId routine:
SmartDashboard.putData("QS Fwd", charHelper.quasistaticForward());
SmartDashboard.putData("QS Rev", charHelper.quasistaticReverse());
SmartDashboard.putData("Dyn Fwd", charHelper.dynamicForward());
SmartDashboard.putData("Dyn Rev", charHelper.dynamicReverse());
```

## MechanismVisualizer

Dashboard visualization using WPILib's Mechanism2d. Creates a canvas with elevator and arm visualizations for real-time monitoring:

```java
// Create a canvas (name, width in meters, height in meters)
MechanismVisualizer viz = new MechanismVisualizer("Robot", 1.0, 2.0);

// Add an elevator visualization
// (name, rootX, rootY, maxHeight, color)
var elevatorViz = viz.addElevator("Elevator", 0.5, 0.0, 1.2, Color.kBlue);

// Add an arm on top of the elevator
// (name, rootX, rootY, length, color)
var armViz = viz.addArm("Arm", 0.5, 0.0, 0.5, Color.kRed);

// In periodic: update positions
elevatorViz.setLength(elevator.getPosition());
armViz.setAngle(arm.getAngle());
```

## CatalystMath

Joystick processing, angle math, and physics estimation helpers:

```java
// Complete joystick processing pipeline
double output = CatalystMath.processJoystick(
    joystick.getY(), // raw input
    0.05,            // deadband
    2.0,             // square curve exponent
    0.7              // 70% max speed (slow mode)
);

// Individual operations
double dead = CatalystMath.deadband(0.03, 0.05);   // 0.0 (inside deadband)
double sq = CatalystMath.squareInput(0.5);          // 0.25 (sign preserved)
double cb = CatalystMath.cubeInput(0.5);            // 0.125

// Angle math (all in degrees)
double normalized = CatalystMath.normalizeAngle(370);     // -170 ([-180, 180])
double diff = CatalystMath.angleDifference(90, 270);      // -180
boolean close = CatalystMath.angleWithinTolerance(89, 90, 2); // true

// Physics estimation
double kG = CatalystMath.elevatorGravityFF(5.0, 0.0254, 10.0, 7.09);
double armkG = CatalystMath.armGravityFF(3.0, 0.5, 50.0, 7.09, 45.0);
```

## InterpolatingTable

TreeMap-based linear interpolation for shooter distance tables. Uses method chaining with `add()`:

```java
InterpolatingTable shooterTable = new InterpolatingTable()
    .add(1.0, 3000)   // 1m -> 3000 RPM
    .add(2.0, 3500)   // 2m -> 3500 RPM
    .add(3.0, 4200)   // 3m -> 4200 RPM
    .add(5.0, 5000);  // 5m -> 5000 RPM

double rpm = shooterTable.get(2.5); // interpolates to ~3850 RPM
double clamped = shooterTable.get(0.5); // clamps to 3000 (below min key)
```

## SlewRateLimiter

Asymmetric rate limiter with different acceleration and deceleration profiles. Great for smooth joystick response:

```java
// Different accel vs brake rates
SlewRateLimiter limiter = new SlewRateLimiter(
    3.0,  // max acceleration (units/sec)
    5.0   // max deceleration (units/sec) - brake harder than accelerate
);

// Symmetric rate limit
SlewRateLimiter symmetric = new SlewRateLimiter(4.0);

// In periodic:
double smoothed = limiter.calculate(targetSpeed);
double current = limiter.get(); // read current output

// Reset when needed
limiter.reset(0.0);
```

## MovingAverage

Sliding window average filter for noisy sensor data. Use `calculate()` to add a sample and get the running average:

```java
MovingAverage filter = new MovingAverage(10); // 10-sample window

// In periodic:
double smooth = filter.calculate(noisySensorValue); // add + get average
double current = filter.get();                       // read current average

// Status checks
boolean full = filter.isFull();     // true when window is filled
int count = filter.getCount();      // number of samples added

// Clear and start over
filter.reset();
```

## TimedBoolean

Debounced boolean with configurable duration threshold. The condition must be sustained for the specified duration before the output goes true:

```java
TimedBoolean stallDetector = new TimedBoolean(0.2); // 200ms threshold

// In periodic:
boolean isStalled = stallDetector.update(current > 30.0);

// Edge detection (great for triggering one-shot actions):
boolean justStalled = stallDetector.risingEdge(current > 30.0);
boolean justCleared = stallDetector.fallingEdge(current > 30.0);

// Manual control
stallDetector.reset();
boolean state = stallDetector.get();
```

---

## Advanced Utilities

FrcCatalyst also includes advanced utilities used by top FRC teams. See the [Advanced](../advanced/) section for:

| Utility | Description |
|---------|-------------|
| **StateSpaceController** | LQR + Kalman filter for optimal mechanism control |
| **MotionConstraintCalculator** | Physics-based max velocity/acceleration from motor specs |
| **SignalProcessor** | EMA, median, low-pass, composite filters for sensor data |
| **PoseHistory** | Temporal pose tracking with interpolation |
| **DynamicAutoBuilder** | Runtime path generation with PathPlanner |
