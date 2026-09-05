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

> **The installer is all you need, and the two alpha-6s are not the same thing.**
>
> There are two builds called 2027 alpha-6: the **release**, which ships inside the WPILib installer
> and is what Systemcore OS beta 13 runs, and a **development snapshot** with the same alpha number
> published to the development maven. They are materially different APIs. Read off every jar in the
> installer rather than inferred:
>
> - `org.wpilib.telemetry`, `org.wpilib.tunables` and `org.wpilib.fields` **do not exist** in the
>   release. They do in the snapshot.
> - Commands v3's `Mechanism` is a **class** in the release and an **interface** in the snapshot.
> - The release publishes `org.wpilib:commands3-java`; the snapshot publishes
>   `org.wpilib:commandsv3-java`.
>
> **Catalyst 2.0.0-alpha.1-a6 targets the release**, so the installer is sufficient and no
> development maven is needed. Five classes that can only exist on the snapshot are excluded from
> this build — see [What is not in this build](#what-is-not-in-this-build).
>
> Getting this wrong does not look like a version problem. A program built against the snapshot and
> deployed to a beta 13 board **aborts at startup**: the board's daemon checks the client's reported
> library version and rejects a mismatch, so the symptom is a robot that deploys, refuses to enable,
> and says nothing a compile error would have told you.

---

## What is not in this build

Five classes that exist in the source tree do not ship in `2.0.0-alpha.1-a6`. None of them are
deleted — every one is excluded from the source set, so it comes back unchanged the moment the thing
it needs exists in a release. They are listed here so that "why can I not import this" has an answer
that is not a compile error.

| Class | Needs | Use instead |
|---|---|---|
| `WpiTelemetrySink` | `org.wpilib.telemetry` | `NetworkTablesSink` — already the default |
| `TelemetryUtil` | `org.wpilib.telemetry` | `CatalystLog` |
| `MechanismVisualizer` | `org.wpilib.telemetry` | `SimDashboard` |
| `DriverBoard` | `DriverStationDisplay` | the Driver Station's own panels |
| `PhotonSource` | a PhotonVision 2027 vendordep | `LimelightSource` |

**The logging ones cost you nothing.** `CatalystLog` falls back to `NetworkTablesSink`, which is the
sink this library used for every season before 2027 and is still fully tested. Published key paths
are identical, so Console, the health dashboard and any AdvantageScope layout you already have
resolve exactly the same names. The only thing lost is that Catalyst telemetry does not automatically
appear in tools that read WPILib's new standard backend without being pointed at `/Catalyst/...`.

**PhotonVision is a direction, not a gap.** Catalyst is Limelight-first: the pipeline is built into
the hardware and it is the supported path. `PhotonSource` is unlikely to return.

**Choreo still works**, despite ChoreoLib having no build past alpha-2. `followChoreoPath()` reads
Choreo `.traj` files through PathPlanner's `fromChoreoTrajectory`, so it never needed the ChoreoLib
vendordep and is unaffected.

---

## What you have to change

### `AutoSelector` works, and briefly did not

It is in the build and unchanged from 1.x, `SendableChooser` return type included.

It was excluded for a while on a wrong premise, which is worth recording because the premise appears
elsewhere in these docs. `SendableChooser` was believed removed in 2027 and the class was ported to
`org.wpilib.tunable.Selectable`; that package really is absent from the released alpha-6, so the
port would not compile and the file was dropped from the source set. But the original claim was
false. Checked against the shipped jars: **`SendableChooser`, `SmartDashboard`, `Sendable` and
`SendableBuilder` are all present** in `wpilibj-java`. Only `org.wpilib.telemetry` and
`org.wpilib.tunables` are genuinely missing — those belong to the development snapshot, a different
build that happens to share the alpha-6 name.

So nothing about `AutoSelector` changes for a migrating team, and the "one forced API break" the
2027 notes recorded was not a break at all.

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

**Yaw rate also comes from the primary.** This is worth being explicit about, because the obvious
thing to do is average them and the obvious thing is wrong. Averaging two gyros only beats the
better one when they are of *similar* quality: the lowest-variance blend weights each by the inverse
of its variance, and a plain mean is that formula with the noise assumed equal. A Pigeon 2 and a
board-mounted IMU are not equal, so averaging drags a good rate toward a worse one.

If you have measured both — hold the robot still, take the standard deviation of each reported rate
over a few seconds — you can ask for a proper weighting:

```java
imu.withYawRateNoise(0.10, 0.30);    // deg/s at rest: primary, secondary
```

Expect less than you might. Inverse-variance weighting gives a sensor three times noisier a ninth of
the weight, so the combined figure moves by about 5%. Two *equal* sensors buy a factor of 1/√2, and
these are not equal. If you have not measured, leave it alone — the primary is the right answer.

So what does the second sensor actually buy?

| | |
|---|---|
| `angularAccelerationRadPerSecSq()` | **the real reason.** Measured from the difference between two accelerometers instead of by differentiating a gyro. Physics Core uses it automatically when `dualIMU()` is set. |
| `yawRateDisagreementDegPerSec()` | two gyros on one rigid body must agree; a persistent gap means a sensor failed, drifted, or physically moved, and shows up long before the pose error does. The two share no failure mode — the Pigeon is on the CAN bus and Systemcore is not. |

### Why the angular acceleration matters

Without two sensors, Physics Core gets angular acceleration by differencing the gyro rate across a
loop and smoothing the result. On a clean signal that is exact. On a real one it is not:
differentiating scales noise by `1/dt`, so at 50 Hz a rate wobbling a tenth of a degree per second
becomes several degrees per second squared of acceleration that is not happening — and the smoothing
that hides it is hiding real signal too.

A difference of two accelerometers does not amplify noise the way a derivative does. Physics Core
falls back to the derived value on any loop where either sensor is not reporting, so a dropout
degrades rather than stops.

Angular acceleration reads both sensors itself:

```java
imu.angularAccelerationRadPerSecSq();      // rad/s², or empty
imu.canMeasureAngularAcceleration();       // whether both are reporting
```

`CatalystIMU` gained an optional `getAcceleration()` for this, which `CatalystGyro` and
`SystemCoreIMU` both implement. It defaults to empty, so a heading source that only knows heading
stays a perfectly good heading source. Without it a caller had to fetch accelerations from a Pigeon
and from Systemcore separately, through two unrelated APIs in two unit conventions, and get the
frames right — enough friction that the measurement simply would not get taken.

If either sensor is silent the answer is empty rather than zero. Substituting zero for the missing
one produces a confident number out of a single accelerometer, which is the exact thing having two
sensors exists to avoid.

The offsets have to be right. They are measured on the robot, in robot coordinates, and a wrong one
produces a confident wrong answer. Sensors closer together than 5 cm return empty rather than
guessing — at that separation the difference between them is mostly noise divided by a small number.

### Four cameras, and how they actually reach the robot

The earlier draft of this section said Systemcore runs four vision instances on its USB ports and
that a Limelight 4 plugs into one. It does run four instances - `limelight_visionserver0-3` are
there for plain USB cameras - but a **Limelight 4 is not a USB camera to Systemcore.** The OS's own
notes say so ("USB Limelights (LL3A, LL3G) and ethernet Limelights (LL4, LL3, LL2)"), and it was
confirmed with four LL4s and a switch: nothing appears on the USB bus, everything appears on
`eth0`. So the LL4 wiring is Ethernet into a switch, the switch into the Systemcore's port, and the
`LimelightSource.usb(...)` factory is for the two USB models only.

What Systemcore does with an Ethernet Limelight is the useful part, and none of it is in the
Limelight docs yet:

- Its **vision aggregator** (`:4810`) discovers every Limelight on the wire and lists them at
  `http://<systemcore>:4810/api/cameras`, with each camera's address, type, frame rate, pipeline,
  and - the field that matters in the pit - `ntConnected`, whether the camera holds a
  NetworkTables session with the robot program.
- It then **alias-NATs** each camera to `.220`, `.221`, `.222`, `.223`… on *every* subnet the
  Systemcore has: `172.26.0.22x` over USB from a Windows laptop, `172.27.0.22x` from Linux or
  macOS, `172.30.0.22x` over its Wi-Fi, and `10.TE.AM.22x` on the robot network. Which means the
  Driver Station laptop can open any camera's web UI at `http://172.26.0.220:5801` through the
  Systemcore's USB cable, with no route to the robot network at all. Catalyst's on-device agent
  serves this list joined with each camera's own status (temperature, CPU, frame rate), and Console
  draws it.

The addressing that makes the cameras find the robot is the one both sets of docs already
recommend, and on a bench it is the *only* one that works: **cameras static at `10.TE.AM.11`,
`.12`, `.13`, `.14` with gateway `10.TE.AM.1`, the Systemcore static at `10.TE.AM.2`.** A camera
left on automatic addressing with no radio on the bench falls back to a link-local `169.254.x.x`
address and can never reach `10.TE.AM.2`; it shows up in the aggregator's list, streams to the
laptop, and never connects to NetworkTables - `ntConnected: false` is the tell. The Systemcore's
static address is set on its web UI's network page (it writes `dhcpcd.conf` itself); the cameras'
on their own settings page, team number included, and they need a power cycle before it takes.

```java
// Team 581's names, which this robot is a clone of. MegaTag2 needs the drivetrain's yaw every
// loop; VisionSubsystem publishes it once for all four.
VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
        .addLimelight("limelight-shooter", robotToShooterCamera)
        .addLimelight("limelight-left", robotToLeftCamera)
        .addLimelight("limelight-right", robotToRightCamera)
        .addLimelight("limelight-ground", robotToGroundCamera)
        .driveSubsystem(drive)
        .build());
```

With a Hailo accelerator on ports 0–1, object detection runs alongside AprilTag tracking:

```java
GamePieceDetector detector = new GamePieceDetector(ground);
detector.best("note").ifPresent(piece -> drive.turnTo(piece.bearing()));
```

### Vision with no drivetrain

`VisionSubsystem` used to return on the first line of `periodic()` without a Catalyst
`SwerveSubsystem` - and `VisionConfig.build()` threw before it got that far - so a tank drive, a
team's own swerve code, or four cameras on a bench got no fusion, no per-camera telemetry and no
health. The three things fusion needs from a drivetrain are now an interface, `VisionPoseSink`,
which `SwerveSubsystem` implements unchanged, and `StandaloneVisionPose` is the case with no
drivetrain at all: the pose is whatever the cameras last agreed on.

```java
StandaloneVisionPose pose = new StandaloneVisionPose();
VisionSubsystem vision = new VisionSubsystem(VisionConfig.builder()
        .addCamera(new LimelightSource("limelight-shooter", mount, false))   // MegaTag1: no gyro here
        .poseSink(pose)
        .build());
```

Use MegaTag1 (`useMegaTag2 = false`) with a standalone pose. MegaTag2 needs a yaw from something
other than itself, and fed from a standalone pose it gets its own last answer back; Catalyst warns
at construction if you do it.

### Is vision working?

Every camera now has a state - `OK`, `NO_TARGETS`, `DISCONNECTED`, `STALE`, `HOT`, `LOW_FPS`,
`REJECTING` - and vision as a whole a level, `OK`, `DEGRADED` or `BLIND`, under
`/Catalyst/Vision/Health/`. The signals were all there before; what was missing was anything that
said "the left camera has been dead since the second match". A camera that loses its session
produces no rejections, because it produces nothing, and a dashboard cannot notice a number that
has stopped moving.

The per-camera signals come from the classic API a shipping Limelight publishes: `hb`, the frame
heartbeat, is what `LimelightSource.isConnected()` now reads (LimelightLib's own answer is about the
2027 topic and reads false for every camera you can buy), and `hw` carries the camera's frame rate
and CPU temperature. Limelight 4s run hot - 70–85 °C on the bench, in free air - and throttle when
hotter, so `HOT` fires at a configurable 80 °C (`VisionConfig.builder().cameraHotCelsius(...)`).

Faults become Driver Station alerts only after they have lasted a second, and clear only after
they have been gone a second, so a dropped frame is not forty alerts a match. Console puts them in
a bar over the dashboard.

### What is on the robot, and is it answering

`/Catalyst/Devices/` is three counts and a row per device: cameras, motors, and the controller,
each as *expected* and *connected*. Every `CatalystMotor` registers itself (Phoenix says whether
it is on the bus); every camera a `VisionSubsystem` owns registers itself (the heartbeat says); the
Systemcore's system server is the controller's pulse. Console shows the three counts in its top
right corner, `4/4 · 20/20 · Systemcore`, which is the first thing anyone looks for when a robot
behaves strangely and used to be answered by walking round it.

### Is the robot where the auto thinks it is

`AutoStartCheck` compares the pose estimate against the selected auto's starting pose while the
robot is disabled, publishes the distance and heading error under `/Catalyst/Auto/StartCheck/`,
and warns when out of tolerance (0.30 m and 10° by default). It reports; it never moves the
robot or resets its pose, because a check that reset the pose would hide exactly what it exists
to reveal.

```java
AutoStartCheck startCheck = autos.startCheck(drive::getPose);   // PathPlanner autos know their start
// in disabledPeriodic():
startCheck.update();
```

### Autos on the Driver Station

The 2027 Driver Station selects op modes directly, so the auto can be chosen on the screen the drive
team already has open:

```java
autos.publishAsOpModes();
```

---

## If vision stops working

**Check `/Catalyst/Vision/Health/Summary` first.** It names the camera and the fault. If it says
`disconnected` for a camera that is plainly powered, the camera is not reaching the robot's
NetworkTables: on a Systemcore that is nearly always addressing - see *Four cameras, and how they
actually reach the robot* above - and the aggregator's `ntConnected` field confirms it from the
other side.

**Then the camera's OS version.** Catalyst 2.x reads either API a Limelight speaks, and says which
at `/Catalyst/Vision/<camera>/Api`; the rest of this section is about what each one costs.

This is the failure that costs a whole session, and it was written down backwards here until a real
camera was put in front of the library. Both halves are true:

- Limelight OS **2027** publishes results as a single topic and disables the classic per-key
  NetworkTables API by default, so code written against the old keys sees nothing.
- Limelight OS **2026** publishes only the classic per-key API, so Catalyst 2.x — which reads the
  2027 topic through LimelightLib 2 — sees nothing.

Measured on a Limelight 4 running OS 2026.0: it connects to the robot's NetworkTables server as a
healthy NT4 client and publishes fifty topics with live values while looking at a tag — `tx`, `ty`,
`ta`, `botpose_wpiblue`, `rawfiducials`, `stddevs`, all of it. LimelightLib reads none of them,
`getEstimatedPose()` returns empty forever, and **`isConnected()` reports false** even though the
camera is plainly connected, because that method tests data freshness rather than the link.

Catalyst now says so itself. After a few seconds in that state it reports a Driver Station warning
naming the version as the likely cause, and `LimelightSource.looksLikeOldLimelightOs()` exposes it
for a pit dashboard. The camera's version is in the top right of its web UI.

Once the version is right, the thing that is *not* automatically fine is your camera transform:
2027.0 unified every 3D space on NWU right-handed, so a transform carried over from 2026 may need
its mount side and pitch signs flipped. A wrong sign produces a confidently wrong pose, not an
error.

**And if poses are empty while the camera plainly sees a tag,** check that something is publishing
robot yaw every loop. MegaTag2 — Catalyst's default — resolves tags against an externally supplied
heading and silently assumes zero when none arrives, so its poses would be wrong by exactly the
robot's heading. Catalyst refuses those rather than passing them on;
`LimelightSource.isReceivingRobotOrientation()` says whether the feed is live.

Catalyst publishes each camera's transform at construction to
`/Catalyst/Vision/<camera>/RobotToCamera` so you can check it against the robot.

---

## What is deliberately not done yet

Stated so nobody assumes otherwise:

- **`ignoringDisable()` is a no-op.** Commands v3's disabled-mode behaviour could not be confirmed
  against the alpha jars, and guessing at what runs while disabled is not something to do quietly.
- **`PhysicsProfile.SYSTEMCORE` behaves as `BALANCED`.** The compute and the solver both exist now,
  but Physics Core has never run on a robot. Shadow mode on carpet first.
- **PhotonVision has no 2027 build.** `PhotonSource` is excluded from the compile rather than
  deleted. ChoreoLib has none either, but that costs nothing: `followChoreoPath()` reads Choreo
  `.traj` files through PathPlanner's `fromChoreoTrajectory`, so it never depended on ChoreoLib.
- **PathPlanner is still commands v2.** Catalyst bridges it with `LegacyCommands.fromV2(...)`; the
  bridge and the v2 dependency both go away when PathPlanner ships for v3.

## Is this robot fit to enable?

One line in the robot constructor:

```java
Preflight.run().printToConsole();
```

Everything it checks is already checkable separately, and checking all of it separately is something
nobody does. The failures share a shape: the robot boots, the dashboard connects, everything looks
normal, and the problem surfaces at the worst moment as something that does not resemble its cause.

| | |
|---|---|
| **Commands v3 runtime** | the two `--add-opens` flags. Without them the robot starts perfectly and dies on the first command a driver schedules. **Blocker.** |
| **Storage** | 95% full stops logging, then stops the robot program, and nothing about that points at the disk. **Blocker.** |
| **Battery** | against the machine's own brownout floor, not a roboRIO constant. Below it is a **blocker** — a robot browning out on the cart will not survive being driven. |
| **Flash wear** | warns. It will run this match; it is a part to order, not a reason to stop. |
| **Temperature** | warns above 80 °C, and says what it will look like — the cores throttle rather than reporting anything, so the symptom is a loop overrun. |
| **CAN plan** | conflicts and controller contention, both knowable from the device list before anything is enabled. |

It changes nothing and blocks nothing. A preflight that refused to let a robot enable would
eventually refuse during a match, and the team it happened to would delete it. It reports; the
decision stays with the people.

`report.summary()` is one line for a dashboard or the Driver Station — `"Ready"`, `"Ready, 2
warnings"`, or `"NOT READY: "` and the first thing to fix, named rather than counted.

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
