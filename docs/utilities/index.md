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

## HealthCheck + HealthMonitor

The fault-monitoring kit added in v0.3.3 wires a debounced health check
layer into every Catalyst mechanism by default. Every motor-driven
mechanism automatically registers `OverCurrent`, `HighTemp`, and
`OverTemp` checks; the `OverTemp` check auto-calls `motor.stop()` when
it fires. Teams add their own with one fluent call:

```java
HealthCheck.builder("Climber", "RopeSlack")
    .severity(HealthCheck.Severity.WARN)
    .description("Climber rope slack detected")
    .when(() -> tensionSensor.getNewtons() < 5)
    .detail(() -> String.format("%.0f N", tensionSensor.getNewtons()))
    .debounce(0.5)
    .clearAfter(2.0)
    .onFire(() -> driver.rumble(0.5))
    .register();
```

State publishes to `/Catalyst/Health/<subsystem>/<id>/...` for any NT
dashboard, and every fire/clear edge is forwarded to the legacy
`AlertManager` so existing integrations keep working. Full guide:
[Health Monitoring](../advanced/health.html).

## HealthHistory

Ring buffer of recent fire/clear events (default 100). Automatically
fed by `HealthMonitor` and published as a string array at
`/Catalyst/Health/History` for the Health Dashboard's timeline view.
Queryable from team code for post-match triage:

```java
for (HealthHistory.Event e : HealthHistory.snapshot()) {
    System.out.println(e);   // newest first
}

HealthHistory.setCapacity(250);  // bigger buffer for long matches
```

Each `Event` carries `timestamp`, `subsystem`, `id`, `severity`, `kind`
(FIRED / CLEARED), and the live `detail` string at the moment of
transition.

## RobotSafety

Opt-in cross-mechanism watchdog driven by `HealthMonitor`. When too
many `ERROR`-severity (or, optionally, `WARN`) health checks fire
simultaneously the safety layer "trips" — a single boolean teams can
read to bail out of teleop / auto. Catalyst never forcibly disables
motors itself; the trip is advisory so each team decides what
"all-stop" means for their robot.

```java
public void robotInit() {
    RobotSafety.configure(
        RobotSafety.Config.builder()
            .maxConcurrentErrors(2)   // trip when 2+ ERROR checks fire at once
            .debounce(0.25)           // sustained for at least 0.25 s
            .onTrip(() -> {
                drive.stop();
                superstructure.stow();
                leds.fire();
            })
            .build());
}

// Read the signal anywhere:
RobotSafety.trippedTrigger().onTrue(drive.stopCommand());

// Or in any subsystem:
if (RobotSafety.isTripped()) { /* ... */ }
```

State publishes to `/Catalyst/Safety/{Tripped,Reason,ErrorCount,WarnCount}`
so the [Health Dashboard](../tools/) lights up when it fires. Manual
`RobotSafety.reset()` clears the trip; or pass `.autoReset(seconds)` to
the builder.

## MotorType

Spec sheet for an FRC motor — stall torque, free speed, current draw —
used by simulation models, `MotionConstraintCalculator`, and gravity
feedforward helpers. Pre-shipped presets:

| Preset | Notes |
|---|---|
| `KRAKEN_X60`, `KRAKEN_X60_FOC` | FOC variant gets the proper +30% stall torque |
| `KRAKEN_X44`, `KRAKEN_X44_FOC` | Compact Kraken (added in v0.3.3) |
| `FALCON_500`, `FALCON_500_FOC` | Phoenix-Pro FOC variant available |
| `NEO`, `NEO_VORTEX`, `NEO_550` | REV brushless line (sim/physics only — not Phoenix-controllable) |
| `MINION` | WCP Minion |

```java
LinearMechanism.Config.builder()
    .motorType(MotorType.KRAKEN_X60_FOC)
    .follower(14, true)        // second motor — count is leader + followers
    // ...
    .build();
```

`MotorType` is a regular `final class` (not an enum) with a public
constructor, so teams can ship their own preset for anything Catalyst
doesn't include:

```java
MotorType custom = new MotorType(
    "Custom Motor",
    3.2,    // stall torque (Nm)
    7200,   // free speed (RPM)
    230,    // stall current (A)
    1.6);   // free current (A)
```

> ⚠️ The pre-v0.3.3 FOC presets had the same stall torque as their
> non-FOC counterparts (Phoenix-6 FOC actually delivers ~30% more
> torque). If you used `KRAKEN_X60_FOC` or `FALCON_500_FOC` for
> gravity feedforward in 0.3.2 or earlier and hand-tuned `kG`, re-check
> the value after upgrading.

## RobotState

A singleton view of "what's the robot doing right now?" — alliance,
match time, mode, battery voltage, time since enable. Cached for 5 ms
so re-reading from multiple subsystems is free, and exposes ready-to-
bind triggers for the common end-game patterns.

```java
if (RobotState.isAutonomous()) { ... }
if (RobotState.isRed())        { mirrorPoseForRed(); }
if (RobotState.matchTimeRemaining() < 10) { climber.runDefault(); }

// As triggers — best for command bindings:
RobotState.lateMatch(20).onTrue(climber.deployCommand());
RobotState.lowBattery(11.0).onTrue(RobotSafety.tripCommand("low battery"));
RobotState.disabled().onTrue(LedSubsystem.idlePattern());
```

| Field | Method |
|---|---|
| Mode      | `isAutonomous()`, `isTeleop()`, `isTest()`, `isDisabled()`, `isEnabled()`, `isEStopped()` |
| Alliance  | `alliance()`, `allianceOpt()`, `isRed()`, `isBlue()`, `stationLocation()` |
| Match     | `matchTimeRemaining()`, `timeSinceEnable()` |
| Power     | `batteryVoltage()` |
| Triggers  | `autonomous()`, `teleop()`, `disabled()`, `enabled()`, `lateMatch(s)`, `lowBattery(v)` |

Refresh is lazy. To force a re-read in a test, call
`RobotState.refresh()`.

## CANRegistry

A process-wide registry of every CAN device the robot has claimed. Every
Catalyst motor (and any CANcoder it fuses / syncs / reads) calls
`CANRegistry.register(...)` automatically when its builder runs. If two
devices try to claim the same `(bus, canId)` with different names, the
registry throws a `CANConflictException` with both sides of the
collision named. Identical re-registrations are silently idempotent.

```java
// Manual registration — usually you let mechanisms do it, but useful
// when you control raw hardware outside the library.
CANRegistry.register("ShooterTop", 30, "rio", "Kraken X60");

// Look up what's on a given id:
CANRegistry.lookup(30, "rio").ifPresent(e ->
    System.out.println(e.name() + " (" + e.type() + ")"));

// Get the full plan (sorted by bus then id):
for (var e : CANRegistry.all()) System.out.println(e);
```

The full plan is also published at `/Catalyst/CAN/Devices` as a
pipe-delimited string array for the Health Dashboard.

The most ergonomic way to populate it is to use the [CAN ID
Planner](../tools/canids/) and click **Generate Catalyst Java**. The
exported `CANIds.java` looks like this:

```java
public final class CANIds {
    public static final String CANIVORE = "canivore";
    public static final String RIO      = "";

    public static final int FRONT_LEFT_DRIVE  = 1;
    public static final int FRONT_LEFT_STEER  = 2;
    // ...

    static {
        CANRegistry.register("FrontLeftDrive", FRONT_LEFT_DRIVE, CANIVORE, "Kraken X60");
        CANRegistry.register("FrontLeftSteer", FRONT_LEFT_STEER, CANIVORE, "Kraken X60");
        // ...
    }
    public static void init() {}
}
```

Call `CANIds.init()` once from `Robot.robotInit()` and any wiring
mistake — a missing device, a wrong name, a duplicate id — surfaces
immediately at boot instead of mid-match.

## TunableNumber + TunableGains

Live-tunable doubles backed by NetworkTables. `TunableNumber` is a
single `double` with a known default; `TunableGains` bundles every
Slot 0 gain + Motion Magic constant for a mechanism. Both are
internally cached so reading is cheap. See [Live Tuning](../advanced/tuning.html)
for the full story.

```java
TunableNumber shooterRPS = new TunableNumber("Catalyst/Tuning/Shooter/TargetRPS", 60.0);
shooter.spinUp(shooterRPS.get());

// For comp: lock everything in one call.
TunableNumber.disableTuning();
```

---

## Advanced Utilities

FrcCatalyst also includes advanced utilities that successful teams build
in-house every season. See the [Advanced](../advanced/) section for:

| Utility | Description |
|---------|-------------|
| **StateSpaceController** | LQR + Kalman filter for optimal mechanism control |
| **MotionConstraintCalculator** | Physics-based max velocity/acceleration from motor specs |
| **SignalProcessor** | EMA, median, low-pass, composite filters for sensor data |
| **PoseHistory** | Temporal pose tracking with interpolation |
| **DynamicAutoBuilder** | Runtime path generation with PathPlanner |
