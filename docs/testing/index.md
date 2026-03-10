---
layout: default
title: Testing
nav_order: 7
---

# Testing
{: .no_toc }

How to test, simulate, and characterize your FrcCatalyst-based robot code.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Simulation

FrcCatalyst mechanisms include built-in simulation models using WPILib's physics engine. Every mechanism simulates accurately with proper DCMotor models, gravity, and constraints.

### Running the Simulator

```bash
./gradlew simulateJava
```

This launches the WPILib Sim GUI with:
- **Driver Station** — enable/disable, switch modes
- **NetworkTables** — view all mechanism telemetry under `Catalyst/`
- **Mechanism2d** — visual representation of your elevator, arm, etc.

### What Gets Simulated

| Mechanism | Simulated Behavior |
|-----------|-------------------|
| LinearMechanism | Gravity, mass, gear ratio, drum radius, stages, soft limits |
| RotationalMechanism | Gravity (cosine), mass, arm length, MOI, range limits |
| FlywheelMechanism | Flywheel inertia, spin-up/spin-down dynamics |
| RollerMechanism | Motor response (no physics model needed for duty cycle) |
| WinchMechanism | Position tracking with range limits |

### Dashboard Visualization

Use `MechanismVisualizer` to see your mechanisms in real-time on the dashboard:

```java
MechanismVisualizer viz = new MechanismVisualizer("Robot", 1.0, 2.0);
var elevatorViz = viz.addElevator("Elevator", 0.5, 0.0, 1.2, Color.kBlue);
var armViz = viz.addArm("Arm", 0.5, 0.0, 0.5, Color.kRed);

// Update in periodic
elevatorViz.setLength(elevator.getPosition());
armViz.setAngle(arm.getAngle());
```

---

## Unit Testing

FrcCatalyst utilities can be tested with standard JUnit 5, no robot hardware required.

### Test Setup

Add test dependencies to your `build.gradle`:

```gradle
dependencies {
    testImplementation 'org.junit.jupiter:junit-jupiter:5.10.2'
    testRuntimeOnly 'org.junit.platform:junit-platform-launcher'
}

test {
    useJUnitPlatform()
}
```

### Testing Utilities (No HAL Required)

These classes can be tested in pure Java without any WPILib HAL initialization:

```java
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import frc.lib.catalyst.util.CatalystMath;
import frc.lib.catalyst.util.InterpolatingTable;
import frc.lib.catalyst.util.MovingAverage;
import frc.lib.catalyst.util.FeedforwardGains;
import frc.lib.catalyst.hardware.MotorType;

class UtilityTests {

    @Test
    void testDeadband() {
        assertEquals(0.0, CatalystMath.deadband(0.03, 0.05));
        assertTrue(CatalystMath.deadband(0.5, 0.05) > 0);
    }

    @Test
    void testInterpolation() {
        InterpolatingTable table = new InterpolatingTable()
            .add(1.0, 100)
            .add(3.0, 300);
        assertEquals(200.0, table.get(2.0), 0.01);
    }

    @Test
    void testMovingAverage() {
        MovingAverage avg = new MovingAverage(3);
        avg.calculate(10);
        avg.calculate(20);
        avg.calculate(30);
        assertEquals(20.0, avg.get(), 0.01);
    }

    @Test
    void testMotorType() {
        assertTrue(MotorType.KRAKEN_X60.freeSpeedRPS() > 0);
        assertNotNull(MotorType.KRAKEN_X60.getDCMotor(1));
    }

    @Test
    void testFeedforward() {
        FeedforwardGains ff = FeedforwardGains.elevator(0.1, 2.0, 0.05, 0.35);
        assertTrue(ff.calculateElevator() > 0); // gravity hold > 0
    }
}
```

### Testing with HAL (Timer-Dependent)

`SlewRateLimiter` and `TimedBoolean` use WPILib's `Timer` and require HAL initialization:

```java
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;

class HALDependentTests {

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @Test
    void testSlewRateLimiter() {
        SlewRateLimiter limiter = new SlewRateLimiter(10.0);
        double result = limiter.calculate(100.0);
        assertTrue(result < 100.0); // rate limited
    }

    @Test
    void testTimedBoolean() {
        TimedBoolean tb = new TimedBoolean(0.5);
        assertFalse(tb.update(true)); // not sustained long enough
    }
}
```

### Testing Mechanisms

Mechanism tests require CTRE Phoenix simulation runtime and must be run with `simulateJava`, not JUnit. Use the separate `testAll` Gradle task if configured:

```gradle
// In build.gradle
task testAll(type: Test) {
    useJUnitPlatform()
    // Include ALL tests (mechanisms need HAL + CTRE sim)
}

test {
    useJUnitPlatform()
    // Exclude mechanism tests from default test task
    exclude '**/mechanisms/**'
    exclude '**/SlewRateLimiterTest*'
    exclude '**/TimedBooleanTest*'
}
```

```bash
# Run only unit tests (fast, no native deps)
./gradlew test

# Run all tests including mechanisms (needs WPILib sim)
./gradlew testAll
```

---

## SysId Characterization

Use `CharacterizationHelper` to characterize your mechanisms and find accurate feedforward gains:

### Setup

```java
CharacterizationHelper charHelper = new CharacterizationHelper(
    "Elevator",
    elevator,
    elevator.getMotor()
);

// Bind to dashboard buttons
SmartDashboard.putData("QS Fwd", charHelper.quasistaticForward());
SmartDashboard.putData("QS Rev", charHelper.quasistaticReverse());
SmartDashboard.putData("Dyn Fwd", charHelper.dynamicForward());
SmartDashboard.putData("Dyn Rev", charHelper.dynamicReverse());
```

### Running Characterization

1. Deploy code to the robot
2. Open the **SysId** tool in WPILib
3. Run each test (quasistatic forward/reverse, dynamic forward/reverse)
4. Analyze results in SysId to get kS, kV, kA, and kG values
5. Plug the values into your mechanism config

### Physics Estimation (No Hardware Needed)

If you don't have the robot yet, estimate gains from your mechanism's specs:

```java
var config = LinearMechanism.Config.builder()
    .motorType(MotorType.KRAKEN_X60)
    .gearRatio(10.0)
    .drumRadius(0.0254)
    .mass(5.0)
    .stages(2)
    .build();

System.out.println("Estimated kG: " + config.estimateGravityFF());
System.out.println("Max speed: " + config.estimateMaxSpeed() + " m/s");
```

---

## Test Project

For a complete test project with 68+ JUnit tests covering every FrcCatalyst component, see the [FrcCatalystTest](https://github.com/TomAs-1226/FrcCatalystTest) repository.

It includes tests for:
- All utility classes (CatalystMath, InterpolatingTable, MovingAverage, etc.)
- Hardware types (MotorType specs, DCMotor creation)
- FeedforwardGains calculations
- All mechanism types (construction, commands, triggers)
- SuperstructureCoordinator state machine

```bash
# Clone and run
git clone https://github.com/TomAs-1226/FrcCatalystTest.git
cd FrcCatalystTest
./gradlew test
```
