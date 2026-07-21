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

### Timer-dependent code: `HAL.initialize` does not work here

`SlewRateLimiter` and `TimedBoolean` call WPILib's `Timer`, which needs the HAL. Older versions of
this page suggested reaching for it directly:

```java
// This does NOT work in the FrcCatalyst `test` task.
@BeforeAll
static void initHAL() {
    HAL.initialize(500, 0);   // UnsatisfiedLinkError: no wpiHaljni in java.library.path
}
```

The reason is worth understanding, because it is the same reason in your own robot project.
`HAL.initialize` is a Java method that calls straight into `wpiHaljni`, a native library. The
`edu.wpi.first.hal:hal-java` artifact contains only the Java side of that binding; the `.dll` /
`.so` comes from a separate platform-specific artifact that the GradleRIO plugin puts on the
classpath and extracts for you. FrcCatalyst's `build.gradle` applies plain `java-library` and lists
exactly three test dependencies — JUnit, the JUnit launcher, and `quickbuf-runtime` for wpimath's
geometry classes. No GradleRIO, so no natives, so the class loads and the first native call fails.
Adding a `@BeforeAll` block does not fix that; it just moves the failure earlier.

**What this means in practice.** Anything that touches the HAL — `Timer`, motor controllers,
`DriverStation`, NetworkTables-backed telemetry, the command scheduler — belongs in a simulation
run (`./gradlew simulateJava`), not in a JUnit test. Anything that is pure Java can and should be
unit-tested, and the useful move is to *design* for that boundary rather than to fight it.

### Designing for testability: how the state machine engine does it

`StateMachineCore` is the clearest example in the library, and it is worth copying. The engine
takes **no WPILib imports at all**. Two decisions bought that:

1. **Time is injected, not read.** The engine takes a `DoubleSupplier clock`. On a robot,
   `Superstructure.Builder` defaults it to `Timer::getFPGATimestamp`; in a test it is a mutable
   field the test advances by hand. Nothing anywhere calls `Timer` directly, so a test can jump
   four seconds forward to check a deadline without waiting four seconds — or ever loading the HAL.
2. **Mechanisms are behind an interface.** The engine knows only `Binding<G>`, an interface of
   plain-Java methods (`atGoal`, `measured`, `zeroed`, `label`). The WPILib-flavoured
   `Actuator<G>` — which hands out `Command` objects and owns `Subsystem`s — sits one layer up in
   `frc.lib.catalyst.statemachine.robot`. A test implements `Binding` directly with a fake whose
   position walks toward its goal a step at a time, and which can be jammed on demand to reproduce
   a stuck mechanism.

That is enough to test the parts that actually contain the bugs: whether an undeclared edge is
refused, whether a timeout leaves `current()` where the robot really is, whether the blocker string
names the right mechanism.

```java
// From the FrcCatalyst suite: no HAL, no scheduler, no NetworkTables.
enum St { STOW, MID, HIGH, INTAKE, CLIMB }

FakeClock clock = new FakeClock();          // a DoubleSupplier the test advances
FakeBinding elevator = new FakeBinding("elevator");

StateMachineCore.Builder<St> b =
    StateMachineCore.builder(St.class, "Graph").clock(clock);
Handle<Double> h = b.bind("elevator", elevator);

b.initialState(St.STOW)
 .state(St.STOW, s -> s.set(h, 0.0))
 .state(St.HIGH, s -> s.set(h, 1.0))
 .hub(St.STOW);

// validate() checks the configuration without building or touching hardware,
// and reports every problem at once rather than only the first.
ValidationReport report = b.validate();
assertFalse(report.ok());
```

`validate()` is the hook to reach for even if you write no other tests. It is the difference
between finding an undeclared state on a laptop and finding it in a queue line.

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

### The in-repo suite

`./gradlew test` on FrcCatalyst itself runs **62 JUnit tests**. They all run on a laptop with no
HAL, no NetworkTables and no command scheduler, for the reasons described above.

**46 of the 62 cover the state machine engine**, split across five files:

| Area | Tests | What it pins down |
|---|---|---|
| Graph and validation | 11 | Undeclared states and unreachable states fail the build; every problem is reported, not just the first; an undeclared edge is refused rather than attempted |
| Truth invariants | 10 | `current()` is only ever a proven state; a timeout leaves the machine where the robot actually is; `isAt` stays a live measurement rather than a latch |
| Transitions and staging | 11 | Stage N waits for stage N-1; guards, entry guards and interlocks block with the right reason; abort and override behaviour |
| Telemetry cadence | 9 | The log schema is written at the right rate, and edge-detected keys do not churn |
| Robustness | 5 | A throwing guard fails closed instead of crashing the loop; after a timeout the machine can be commanded back to its proven state; diagnostics track the goals actually being pursued |

The remaining 16 cover `AimingSolver` (8), `AllianceFlipUtil` (4) and turret math (4) — the other
pure-Java corners of the library. Mechanism classes are not in this count: they need the CTRE
Phoenix simulation runtime and are exercised through `simulateJava` instead.

### The example robot project

For a larger worked project that exercises mechanisms end to end, see the
[FrcCatalystTest](https://github.com/TomAs-1226/FrcCatalystTest) repository. It covers utility
classes, hardware types, `FeedforwardGains`, and all mechanism types (construction, commands,
triggers), and it runs under simulation because those parts need the HAL.

```bash
# Clone and run
git clone https://github.com/TomAs-1226/FrcCatalystTest.git
cd FrcCatalystTest
./gradlew test
```
