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
| TurretMechanism | Motor dynamics at the configured MOI, fed back through the gear ratio |
| FlywheelMechanism | Flywheel inertia, spin-up/spin-down dynamics |
| RollerMechanism | Motor response (no physics model needed for duty cycle) |
| WinchMechanism | Position tracking with range limits |
| ClawMechanism | Motor response, plus `setSimHasPiece(...)` — a `DCMotorSim` will not stall against a virtual game piece, so possession is set rather than inferred |
| DifferentialWristMechanism | Both motors independently, so pitch and roll come out of the differential rather than being faked |
| PneumaticMechanism | Commanded solenoid state only. There is no air model anywhere, sim or not — `isForward()` reports what was commanded, so gate on `timeInState()` rather than on arrival |
| ServoMechanism | Nothing to model — a servo has no encoder, so the measured angle reads back the commanded one on a real robot too |

### Dashboard Visualization

{: .warning }
> **Not built on the 2027 branch.** `MechanismVisualizer` publishes through
> `org.wpilib.telemetry.Telemetry`, which the released WPILib 2027 alpha-6 does not ship - the
> package exists only in the development snapshot that shares the alpha-6 name. The class is
> excluded from the source set in `build.gradle` rather than deleted, and comes back the moment a
> release carries that package. It is not on the classpath today, so code written against this
> section will not compile against Catalyst 2.x. (`Mechanism2d` itself is fine; it is the
> telemetry backend underneath that is missing.)

Use `MechanismVisualizer` to see your mechanisms in real-time on the dashboard:

```java
MechanismVisualizer viz = new MechanismVisualizer("Robot", 1.0, 2.0);
var elevatorViz = viz.addElevator("Elevator", 0.5, 0.0, 1.2, Color.BLUE);
var armViz = viz.addArm("Arm", 0.5, 0.0, 0.5, Color.RED);

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

### The HAL works in tests, and so does the command scheduler

Earlier versions of this page said the HAL could not be loaded in a JUnit test and that anything
touching it — `Timer`, `DriverStation`, the command scheduler — belonged in `simulateJava` instead.
That is no longer true, and the reason it was true is worth knowing, because the same thing will
happen in your own project.

WPILib's Java artifacts contain only the Java half of each JNI binding. The `.dll`/`.so` comes from
a separate platform artifact that GradleRIO normally extracts for you. Catalyst applies plain
`java-library`, so its `build.gradle` extracts them itself into `build/nativeLibs` and points the
test JVM at that directory.

The part that took a while: 2027 split `telemetry`, `tunables` and `datalog` out of `wpiutil` and
`ntcore` into their own libraries. `wpimath` links against the first two and `ntcore` against the
third, so leaving them out of the extraction meant **no** native library loaded at all. The
diagnostic points the wrong way —

```
wpiHaljni.dll: Can't find dependent libraries
```

— names the library that failed, never the one actually missing. If you hit this, check the import
table of the library that failed rather than adding directories to `PATH`.

The test JVM also passes the two continuation flags Commands v3 requires:

```groovy
jvmArgs '--add-opens', 'java.base/jdk.internal.vm=ALL-UNNAMED',
        '--add-opens', 'java.base/java.lang=ALL-UNNAMED'
```

Without them the scheduler cannot be constructed. See [Systemcore](../advanced/systemcore.md) — the
robot program needs the same two.

**What this means in practice.** You can schedule real commands in a unit test:

```java
Scheduler scheduler = Scheduler.createIndependentScheduler();   // not the default one
scheduler.schedule(myMechanism.raiseCommand());
for (int i = 0; i < 10; i++) {
    scheduler.run();                                            // ten robot loops
}
assertTrue(elevator.atGoal());
```

Use `createIndependentScheduler()` rather than `Scheduler.getDefault()` so tests do not inherit each
other's commands. `CommandFacadeTest` in this repo is a worked example.

One caveat that has not changed: a test JVM has no Systemcore behind the HAL, so anything that reads
hardware reports absent rather than a value. That is what `SystemCoreSim` is for — see below.

### Simulating Systemcore itself

`SystemCoreSim` stands in for the machine. It exists for two reasons.

The first is that `SystemCoreStatus` could not otherwise be tested at all: resolving Systemcore's
system NetworkTables server forces a HAL JNI load through a path that terminates the JVM when the
natives are absent, which took the whole Gradle worker down and lost every other result with it.
Reading is now separated from interpreting — `SystemCoreSource` supplies raw values, `SystemCoreStatus`
keeps every rule about units, ratios and what counts as missing — so the rules are testable.

The second is more useful day to day. The interesting code paths are the ones that only run when the
machine is in trouble, and those are exactly the ones nobody exercises, because reproducing them on
real hardware means deliberately filling a disk or pinning a core during a practice match.

```java
SystemCoreStatus.useSource(SystemCoreSim.healthy()
        .withStorage(31_500_000_000.0, 32_000_000_000.0));   // 98% full

assertTrue(healthMonitor.problems().stream()
        .anyMatch(p -> p.contains("storage")));
```

Values are set in the OS's own units — millivolts for the brownout thresholds, bytes for memory and
storage — so the conversions under test are the real ones.

A fresh `new SystemCoreSim()` reports available with no values, which is the honest model of a
Systemcore that has booted but published nothing yet. Every reading must come back empty rather than
zero: a robot that reads 0 V should not conclude the battery is flat.

`withAvailable(false)` models no Systemcore at all, which is what a desktop test run actually is.
Restore that in teardown rather than the real source — constructing the real one loads the HAL:

```java
@AfterEach
void tearDown() {
    SystemCoreStatus.useSource(SystemCoreSource.unavailable());
}
```

### Designing for testability: how the state machine engine does it

`StateMachineCore` is the clearest example in the library, and it is worth copying. The engine
takes **no WPILib imports at all**. Two decisions bought that:

1. **Time is injected, not read.** The engine takes a `DoubleSupplier clock`. On a robot,
   `Superstructure.Builder` defaults it to `Timer::getTimestamp`; in a test it is a mutable
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

`./gradlew test` on FrcCatalyst itself runs **427 JUnit tests** across 37 test classes. They all run
on a laptop with no HAL, no NetworkTables and no command scheduler, for the reasons described above.

| Package | Tests | What it covers |
|---|---|---|
| `physics` | 314 | The state estimator, contact and collision, slip and stability, projectile and prediction, parameter identification, diagnostics, and the ground-truth validation suite |
| `statemachine` | 49 | The engine — see the breakdown below |
| `identity` | 28 | What `SpecSheet` records, what it refuses to record, and the geometry `RobotIdentity` derives |
| `util` | 22 | `AimingSolver` and the vector solver, `AllianceFlipUtil`, `LoopMonitor` |
| `mechanisms` | 11 | Config validation and pure math only: flywheel torque-current config, servo config, turret continuous-angle wrap |
| `subsystems` | 3 | The swerve sim yield guard |

Physics dominates the count because physics is where a wrong answer is invisible: an estimator that
is confidently wrong looks exactly like one that is right, until the robot misses.

**49 of them cover the state machine engine**, split across five files:

| Area | Tests | What it pins down |
|---|---|---|
| Graph and validation | 12 | Undeclared states and unreachable states fail the build; every problem is reported, not just the first; an undeclared edge is refused rather than attempted |
| Truth invariants | 10 | `current()` is only ever a proven state; a timeout leaves the machine where the robot actually is; `isAt` stays a live measurement rather than a latch |
| Transitions and staging | 11 | Stage N waits for stage N-1; guards, entry guards and interlocks block with the right reason; abort and override behaviour |
| Telemetry cadence | 9 | The log schema is written at the right rate, and edge-detected keys do not churn |
| Robustness | 7 | A throwing guard fails closed instead of crashing the loop; after a timeout the machine can be commanded back to its proven state; diagnostics track the goals actually being pursued |

The mechanism *classes* are not in this count. Their configs and their maths are — a `Config`
builder is pure Java and a turret's wrap arithmetic has no motor in it — but driving a mechanism
needs the CTRE Phoenix simulation runtime, so that happens under `simulateJava` instead.

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
