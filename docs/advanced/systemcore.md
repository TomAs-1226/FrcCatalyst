---
layout: default
title: Systemcore & WPILib 2027
parent: Advanced
nav_order: 12
---

# Systemcore & WPILib 2027
{: .no_toc }

What changes when you move a Catalyst robot to Systemcore, what Catalyst absorbed for you, and the
handful of things you have to change yourself.
{: .fs-6 .fw-300 }

1. TOC
{:toc}

---

## The short version

Catalyst 2.x targets WPILib 2027 and Systemcore. **Your robot code mostly does not change.** WPILib
2027 renamed its package root, reorganised its hardware classes and replaced the command framework
outright; Catalyst absorbed all of that behind its own API so you would not have to relearn a library
because someone else reorganised theirs.

What you do have to change fits on one screen, and it is listed in [What you have to change](#what-you-have-to-change).

**Versions that go together.** These are not interchangeable — a mismatch usually looks like code
that deploys and then does nothing.

| Component | Version |
|---|---|
| Catalyst | 2.0.0-alpha.1 |
| WPILib | 2027.0.0-alpha-6 |
| Systemcore OS | beta 13 |
| Phoenix 6 | 26.50.0-alpha-1 |
| PathPlannerLib | 2027.0.0-alpha-3 |
| LimelightLib | 2.0.0-beta2 |
| Java | 25 |

Systemcore OS **beta 14 requires WPILib alpha-7**, which is not released. Until it is, pair Catalyst
2.x with OS beta 13.

---

## What you have to change

### `AutoSelector.getChooser()` returns a different type

`SendableChooser` no longer exists in WPILib. The method now returns
`org.wpilib.tunable.Selectable`. Every other `AutoSelector` method is unchanged, so this only matters
if you called that accessor.

### Five command decorators were renamed

Commands v3 took the names `until`, `andThen`, `alongWith`, `raceWith` and `withTimeout` and gave
them incompatible return types — they return group *builders* now, and Java will not let Catalyst
narrow a builder back to a finished command. Catalyst's versions are renamed rather than shadowing
v3's:

| Was | Now |
|---|---|
| `.until(condition)` | `.untilTrue(condition)` |
| `.andThen(next)` | `.then(next)` |
| `.alongWith(others)` | `.together(others)` |
| `.raceWith(others)` | `.racing(others)` |
| `.withTimeout(seconds)` | `.timeoutAfter(seconds)` |

`withName`, `finallyDo` and `beforeStarting` are unchanged.

### `ServoMechanism.getServo()` is now `getPwm()`

See [Servos do not work on Systemcore](#servos-do-not-work-on-systemcore) — this is a hardware
limitation, not a rename for its own sake.

### Subsystems implement instead of extend

`SubsystemBase` is gone. Catalyst subsystems implement `CatalystSubsystem`, which restores
`run(Runnable)`, `runOnce(Runnable)` and `startEnd(...)` so your command factories still work.

**One thing to actually do**: call `registerPeriodic()` in your constructor if you define
`periodic()`. Commands v3's `Mechanism` is an interface with no constructor to hook, so nothing
registers it for you — and a `periodic()` that never runs compiles perfectly and fails silently.

```java
public class Intake extends CatalystMechanism {
    public Intake() {
        super("Intake");
        // CatalystMechanism already does this for you. Only needed if you implement
        // CatalystSubsystem directly.
    }
}
```

---

## Things about the hardware that will catch you out

These are not Catalyst decisions. They are properties of Systemcore that are invisible from robot
code, and each one has a failure mode that does not look like its cause.

### Commands need two JVM flags, or nothing runs

Commands v3 is built on JDK continuations. They live in `jdk.internal.vm`, which is exported to
nobody, so v3 reaches them by reflection. The robot program must be launched with:

```
--add-opens java.base/jdk.internal.vm=ALL-UNNAMED
--add-opens java.base/java.lang=ALL-UNNAMED
```

Nothing fails at build time and nothing fails at startup. The robot boots, the dashboard connects,
and then the first command to be scheduled throws `ExceptionInInitializerError` from inside WPILib,
naming no class of yours, at whatever moment a driver first pressed a button.

Both flags are needed, and the second only shows itself once the first is fixed: `jdk.internal.vm`
gets the scheduler constructed, `java.lang` gets a command scheduled. Fix one, deploy, and you meet
the other on the field.

Catalyst checks for both when it builds a command and throws something legible if either is missing,
so you find out at the line that built the command rather than several seconds later. To ask
directly:

```java
CommandRuntime.isAvailable();   // false means the flags are missing
```

On Systemcore the launch line is `/home/systemcore/robotCommand`, written by GradleRIO at deploy
time. In simulation and unit tests these are the JVM's own arguments.

### The five CAN buses are three controllers

`can_s0`+`can_s1` share an SPI controller, `can_s3`+`can_s4` share another, `can_s2` is alone.
Splitting a drivetrain across `can_s0` and `can_s1` therefore buys much less than splitting it
across `can_s0` and `can_s3`, and two buses at 55% each can overload their shared controller while
both look fine individually.

```java
CANBusPlanner.validate().forEach(System.out::println);   // problems with the plan you have
CANBusPlanner.suggest();                                 // a plan that respects the pairing
```

The load model is an estimate. Measure with `CANBusHealth` on a real robot under load and feed the
numbers back with `CANBusPlanner.calibrate(bus, utilization)`.

### Servos do not work on Systemcore

Systemcore's IO pins output 3.3 V and nowhere near enough current to turn a servo. This is not a
software limitation and no library can work around it — WPILib removed the `Servo` class for exactly
this reason. Use a CAN servo hub.

`ServoMechanism` drives a raw PWM channel, which is the correct *signal* for a servo through a hub.

**Also**: Systemcore returns every PWM output to centre when the robot is disabled, in IO-chip
firmware, with no override. A mechanism that must hold position through a disable cannot hold it on
PWM.

### I²C cables need two wires swapped

Systemcore's I²C pinout swaps SCL and SDA relative to the roboRIO. An existing FRC I²C cable will
find nothing and fail silently, which reads as a dead sensor. Systemcore matches the Qwiic and REV
Control/Expansion Hub pinout, so a Qwiic cable is correct as-is.

```java
CatalystI2C.scan();   // empty result raises an alert telling you to check the cable
```

### IO pins are typed on the device

A pin is `digital_in` or `analog_in` because someone set it that way in the Systemcore web UI, and
that survives every code deploy. Declare what you expect so it is written down where a human can
compare it:

```java
SmartIO.declare(0, SmartIO.Type.DIGITAL_IN, "IntakeBeamBreak");
```

Published to `/Catalyst/IO/Pins`.

---

## What you get that you did not have

### The machine reports on itself

Systemcore measures its own processor, memory, storage, temperature and power, and publishes all of
it on its own NetworkTables server. On a roboRIO there was nothing to read, so a robot that browned
out because four seasons of match logs had filled the disk failed in a way that pointed at nothing
at all.

```java
HealthMonitor.systemCoreChecks();           // brownout, CPU, memory, storage, team number mismatch
SystemCoreStatus.getInstance().publish();   // once per loop
```

`BrownoutMonitor` now takes its floor from the hardware rather than the roboRIO's 6.8 V.

#### What you can read

Every one of these is an `Optional` and comes back empty when the machine did not report it. That is
deliberate and worth honouring in your own code: absent and zero are different facts, and only one of
them is good news.

| | |
|---|---|
| `batteryVolts()`, `isBrownedOut()` | supply, and whether the device says it browned out |
| `brownoutVolts()`, `recoveryVolts()` | the device's own thresholds, not the roboRIO's constants |
| `rail3v3Amps()` | current on the rail that feeds the IO pins |
| `cpuUtilization()`, `cpuTemperatureCelsius()` | load and SoC temperature |
| `ramFraction()`, `ramUsed()`, `ramTotal()` | memory, as a ratio and in bytes |
| `storageFraction()`, `storageUsed()`, `storageTotal()` | storage, likewise |
| `emmcLifeUsedFraction()`, `emmcPreEol()`, `emmcNeedsAttention()` | flash wear |
| `canBusUtilization()`, `canBusUtilization(int)` | **per bus**, measured rather than estimated |
| `canBusDown()`, `canBusDownCount()`, `canBusUnavailableCount()` | CAN faults, live and since boot |
| `networkInterfaces()`, `teamNumber()`, `hardwareSubRevision()` | what and where the machine is |

Two of those deserve a note.

**Per-bus CAN utilisation is measured.** `CANBusPlanner` predicts it from a device list; this is what
the machine actually saw. Comparing the two is how a plan gets corrected:

```java
SystemCoreStatus.getInstance().canBusUtilization(0)
        .ifPresent(u -> CANBusPlanner.calibrate("can_s0", u));
```

**Flash wear does not recover.** Deleting logs frees space and gives back none of the write life
already spent, and four seasons of match logs is exactly the workload that spends it. The device
reports its estimate in 10% bands rather than as a percentage, so `emmcLifeUsedFraction()` returns
the midpoint of the band it reported — treat it as "roughly", because that is all the flash said.
`emmcPreEol()` is the device's separate opinion based on blocks actually retired, and is usually the
earlier of the two signals.

#### Two publishing rates

`publish()` puts everything under `/Catalyst/Systemcore/`, on the same connection as the rest of
Catalyst's telemetry, so Console, AdvantageScope and a `.wpilog` replay all get it.

There are twenty-two readings and about a third of them move loop to loop. Publishing all of them at
50 Hz would put roughly 1100 NetworkTables writes per second on a link shared with everything else
the robot reports, to say over and over that the team number is still the same. So the ones that
move go out every loop, and the rest go out about once a second. Both go out on the first call, so a
dashboard connecting mid-match fills in immediately.

Nothing about that is tunable, and it should not need to be — call it once per loop and forget it.

#### Seeing it

Catalyst Console has a **Systemcore** page in Settings showing all of it: the four live figures,
per-bus CAN grouped by the controller each bus shares, flash wear, power, and the network
interfaces. It needs no configuration — a robot calling `publish()` fills it in.

### Running Catalyst on the Systemcore itself

Everything above is a summary, and a summary is what it is. It cannot say *which* core is pinned,
*what* filled the disk, or *how many times* the robot program has restarted — and after a crash the
robot program is not there to report anything about itself, which is exactly when somebody wants to
know what it printed.

All of that is in `/proc` and `/sys` on the machine already. `agent/` in this repository builds an
optional package that serves it:

```bash
cd agent && ./build.sh        # catalyst-agent_2.0.0.ipk
```

Install it through the Systemcore web UI's package manager. It auto-starts on port 9010 and Console
finds it on its own — no configuration on either side, and the Systemcore page simply gains
sections.

| | |
|---|---|
| Per-core load and clocks | one core at 100% and three idle averages to 25%, which reads as a quiet machine |
| Throttling flags | split into now and since-boot, so a robot that throttled last match still shows it |
| Robot program | systemd state, uptime, **restart count**, memory, and a log tail |
| Storage by directory | "94% full" is not actionable; "your logs are 9.4 GB" is |
| Heaviest processes | by CPU and by resident memory |
| CAN frame counters | errors, drops and restarts — none of which move utilisation |
| OS build, kernel, uptime | the first question when one robot behaves unlike the one beside it |

It is read-only, and that is a boundary rather than a limitation: Console is read-only by design, and
an agent that could restart the robot program would put a restart button one mis-click from a driver
during a match. It also runs at `Nice=10` with idle CPU and IO scheduling and no write access
anywhere, because a diagnostic must never be the reason a match is lost.

### A free second IMU

Systemcore has an onboard IMU — no CAN id, no wiring, no bus load. Keep the Pigeon for swerve; this
is worth having as an independent second opinion that shares no failure mode with it, and as a
fallback if the Pigeon leaves the bus mid-match.

```java
CatalystIMU backup = new SystemCoreIMU(OnboardIMU.MountOrientation.FLAT);
```

The mount orientation must match what is set in the Systemcore web UI. Getting it wrong silently
swaps which axis reports as yaw.

#### Using both at once

`DualIMU` reads the two as one. It is opt-in and it does not change how the robot drives:

```java
DualIMU imu = new DualIMU(
        pigeon, backup,
        new Translation2d(0.25, 0.0),    // where the Pigeon sits, robot coordinates, metres
        new Translation2d(-0.15, 0.1));  // where Systemcore sits

PhysicsCore physics = PhysicsCore.builder(...).dualIMU(imu).build();
```

Heading still comes from the primary alone. Yaw is an integrated quantity, and averaging two
integrals that drift at different rates gives a third drifting integral while hiding which sensor
moved — so odometry and drivetrain carry on exactly as before.

What the second sensor buys is measured, not averaged:

| | |
|---|---|
| `getYawRate()` | averaged across both — rate is a direct measurement, so this is straightforwardly better, and the two share no failure mode since the Pigeon is on CAN and Systemcore is not |
| `angularAccelerationRadPerSecSq(a1, a2)` | measured from the difference between the two accelerometers instead of by differentiating a gyro, which amplifies noise |
| `yawRateDisagreementDegPerSec()` | two gyros on one rigid body must agree; a persistent gap means a sensor failed, drifted, or physically moved, and shows up long before the pose error does |

The offsets have to be right. They are measured on the robot, in robot coordinates, and a wrong one
produces a confident wrong answer. Sensors closer together than 5 cm return empty rather than
guessing — at that separation the difference between them is mostly noise divided by a small number.

### Four cameras, no switch

Systemcore runs four independent vision instances on its USB ports.

```java
LimelightSource front = LimelightSource.usb(0, robotToFrontCamera);
```

With a Hailo accelerator on ports 0–1, object detection runs alongside AprilTag tracking:

```java
GamePieceDetector detector = new GamePieceDetector(front);
detector.best("note").ifPresent(piece -> drive.turnTo(piece.bearing()));
```

### Autos on the Driver Station

The 2027 Driver Station selects op modes directly, so the auto can be chosen on the screen the drive
team already has open:

```java
autos.publishAsOpModes();
```

---

## If vision stops working

**Check this first.** Limelight OS 2027.0 publishes results as a single MessagePack topic and
disables the classic per-key NetworkTables API by default. Code written against the old keys compiles
perfectly and sees nothing at all.

Catalyst 2.x uses LimelightLib 2 and is fine. What is *not* automatically fine is your camera
transform: 2027.0 unified every 3D space on NWU right-handed, so a transform carried over from 2026
may need its mount side and pitch signs flipped. A wrong sign produces a confidently wrong pose, not
an error.

Catalyst publishes each camera's transform at construction to
`/Catalyst/Vision/<camera>/RobotToCamera` so you can check it against the robot.

---

## What is deliberately not done yet

Stated so nobody assumes otherwise:

- **`ignoringDisable()` is a no-op.** Commands v3's disabled-mode behaviour could not be confirmed
  against the alpha jars, and guessing at what runs while disabled is not something to do quietly.
- **`PhysicsProfile.SYSTEMCORE` behaves as `BALANCED`.** The compute and the solver both exist now,
  but Physics Core has never run on a robot. Shadow mode on carpet first.
- **PhotonVision and ChoreoLib have no 2027 build.** `PhotonSource` is excluded from the compile
  rather than deleted, and `followChoreoPath()` has no path forward until ChoreoLib ships.
- **PathPlanner is still commands v2.** Catalyst bridges it with `LegacyCommands.fromV2(...)`; the
  bridge and the v2 dependency both go away when PathPlanner ships for v3.

## Two things 2027 puts where the driver is looking

### Autos the Driver Station lists

2027 replaced "autonomous is whatever `autonomousInit` does" with named **OpModes** that the Driver
Station shows and selects. A routine is a class with an annotation:

```java
@Autonomous(name = "Three Piece Left", group = "Competition")
public class ThreePieceLeft extends CommandOpMode {
    public ThreePieceLeft() {
        super(() -> RobotContainer.get().auto().threePieceLeft());
    }
}
```

and it appears on the Driver Station, grouped, ready to pick. No chooser to publish, no dashboard to
configure, and nothing to remember when a routine is added — which was the old way's failure mode: an
auto written on Friday that nobody could select on Saturday because the chooser was never updated.

Register them in one line:

```java
public class Robot extends OpModeRobot {
    public Robot() {
        addAnnotatedOpModeClasses(getClass().getPackage());
    }
}
```

`CommandOpMode` takes a **supplier**, not a command, and that is not a style preference. An OpMode is
constructed when the Driver Station lists the modes, which is at startup. A command built then
captures the pose the robot had at boot and the alliance before the FMS said — it runs, it looks
entirely plausible, and it drives the wrong way. Ending the mode also cancels what it scheduled,
without which a fifteen-second auto keeps driving into teleop while the driver wonders why the robot
will not respond.

`CatalystOpMode` is the base for anything that is not just one command. It runs the scheduler and
publishes machine status, which an OpMode does not do on its own — a mode that does not tick the
scheduler compiles, runs, and does nothing at all: no command, no default command, no subsystem
periodic, and nothing anywhere saying why.

### Status on the Driver Station itself

Everything Catalyst reports lands somewhere the driver is not looking: Console is on another screen,
AdvantageScope is for afterwards, the health dashboard is a browser tab. In the thirty seconds before
a match the driver is looking at exactly one thing, and 2027 finally allows writing to it.

```java
DriverBoard.standard()                       // what is wrong, and the battery
    .line("Auto", () -> selectedAutoName)
    .start();
```

Deliberately short. A display that scrolls is a display nobody reads, and this one competes with a
match for attention — a line earns its place only if a driver would do something differently because
of it in the next minute. Failing health checks are named while there are few enough to name and
counted after that, because a robot with nine problems has one problem and it is not any of the nine.
Battery is judged against the thresholds the machine itself publishes rather than the roboRIO's
6.8 V, which is a different number.

## Facts we could not verify

Separate from the list above, which is about work not done. These are things where the answer was
not available and a guess would have looked exactly like knowledge.

Two came off this list by being found rather than assumed. The `/sys` topic names, including
`vbrownout` and `vrecovery`, were read out of the OS image; so were the eMMC, thermal and per-bus
CAN topics that the readings above are built on.

- **Whether Systemcore terminates its own CAN buses.** The roboRIO had a 120 Ohm terminator built
  in. Nothing in Limelight's or WPILib's documentation says whether Systemcore does, on any of its
  five buses. The wiring tool asks you to check rather than assuming either way.
- **The `frcYear` a 2027 vendordep should carry.** Catalyst's says `2027`. WPILib's own
  `WPILibNewCommands.json` in the alpha test projects says `2027_alpha1` - the same string as the
  project's `projectYear` - so an exact-match check somewhere in the toolchain would reject ours.
  It could not be tested, because GradleRIO 2027 alpha-6 ships only inside the WPILib installer.
  If VS Code complains about the season on install, this is why, and the fix is to match your
  project's `projectYear` exactly.
- **Whether the eMMC health topics carry raw JEDEC codes.** The OS reads Linux's
  `/sys/class/mmc_host/*/life_time` and `/pre_eol_info`, which expose the raw registers, and Catalyst
  reads `emmc/lifetime_a`, `emmc/lifetime_b` and `emmc/pre_eol` as those codes: lifetime in ten 10%
  steps, pre-EOL as 1 normal / 2 warning / 3 urgent. That the daemon passes them through unchanged is
  read off the sysfs paths it opens, not off a running machine. If a real Systemcore reports a
  percentage instead, wear will read as roughly a tenth of its true value.
- **The camera NWU transform signs.** Reasoned from the coordinate conventions, not measured. Check
  against a known target before trusting pose estimates from a camera that is not centred.
- **`Models.singleJointedArmFromPhysicalConstants` as the successor to `createDCMotorSystem`.** It
  builds the same rotational double-integrator, since gravity lives in the sim classes rather than
  the linear system, but that equivalence is reasoned rather than read off a jar. It feeds five
  mechanism sim models, so a discrepancy shows up as simulation that does not match the robot.
