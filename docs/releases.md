---
layout: default
title: Releases
nav_order: 8
---

# Releases
{: .no_toc }

What shipped, what it needs, and what to do if you are coming from 1.x.
{: .fs-6 .fw-300 }

1. TOC
{:toc}

---

## Which line am I on?

Two lines exist and they are not compatible. This is deliberate — 2.x is a hard cut with no 2026
shims, because carrying both would have meant every class in the library branching on a season.

| | **Catalyst 1.x** | **Catalyst 2.x** |
|---|---|---|
| Controller | roboRIO | Limelight Systemcore |
| WPILib | 2026 | 2027 alpha-6 |
| Java | 17 | **25** |
| Gradle | 8.x | **9.7+** |
| Commands | v2 | **v3** |
| CAN buses | 1 | **5** |
| Latest | `v1.12.0` | `v2.0.0-alpha.1` |
| Docs | [stable](https://tomas-1226.github.io/FrcCatalyst/) | you are reading them |

{: .warning }
> **If your robot is competing, use 1.x.** 2.x targets an alpha WPILib on a beta operating system
> and has never run on hardware. It is for an offseason robot you can afford to have not work.

---

## v2.0.0-alpha.1

*23 August 2026* · [release](https://github.com/TomAs-1226/FrcCatalyst/releases/tag/v2.0.0-alpha.1)

The 2027 port.

### Install

```gradle
repositories { maven { url 'https://jitpack.io' } }
dependencies {
    implementation 'com.github.TomAs-1226:FrcCatalyst:v2.0.0-alpha.1'
}
```

or the vendordep, `https://tomas-1226.github.io/FrcCatalyst/beta/vendordep/FrcCatalyst.json`.

Then the two flags from [Installation](getting-started/installation), without which the robot boots
and dies on the first command scheduled.

### What is new because the platform is new

**Five CAN buses.** `CatalystCANBus` types them, and `CANBusPlanner` knows the thing that is
invisible from robot code: `can_s0`+`can_s1` share an SPI controller, `can_s3`+`can_s4` share
another, `can_s2` is alone. Splitting a drivetrain across a *pair* buys far less than it looks.

**A machine that reports on itself.** `SystemCoreStatus` reads processor, temperature, memory,
storage, eMMC wear, per-bus CAN utilisation, fault counters, the 3.3 V rail, and the brownout
thresholds the device holds — not the roboRIO's 6.8 V, which is a different number. A roboRIO
reported almost none of this, which is why a robot that browned out because four seasons of logs had
filled the disk used to fail pointing at nothing at all.

**Catalyst Agent.** An optional package that runs *on* the Systemcore and answers what
NetworkTables cannot: which core is pinned and by what, what filled the disk, how many times the
robot program has restarted and what it printed on the way down. Read-only by design.

**OpModes.** Autos as annotated classes the Driver Station lists and selects natively — no chooser
to publish and nothing to remember when a routine is added.

**DriverBoard.** Status on the Driver Station itself, which is the one screen a driver is certainly
looking at thirty seconds before a match.

**Dual IMU.** The Pigeon and Systemcore's onboard IMU read as one, measuring angular acceleration
from the difference between two accelerometers rather than differentiating a gyro.

Plus the onboard IMU, typed `SmartIO` pins, `CatalystI2C` with the SCL/SDA swap handled,
`GamePieceDetector` on the Hailo NPU, and `SystemCoreSim`, which makes a full disk or a pinned core
reachable in a unit test.

### What changed for you

Five decorators were renamed. Commands v3 took the names and gave them incompatible return types, so
Java will not allow an override — this is the one place API stability could not be kept:

| 1.x | 2.x |
|---|---|
| `.until(...)` | `.untilTrue(...)` |
| `.andThen(...)` | `.then(...)` |
| `.alongWith(...)` | `.together(...)` |
| `.raceWith(...)` | `.racing(...)` |
| `.withTimeout(...)` | `.timeoutAfter(...)` |

Three more, each forced by something being removed from WPILib or the hardware:

- `AutoSelector.getChooser()` returns `Selectable<String>`. `SendableChooser` no longer exists.
- `ServoMechanism.getServo()` is `getPwm()`. Servos cannot be driven from Systemcore at all — its IO
  pins are 3.3 V and nowhere near the current — so the class drives raw PWM and says so.
- `Identity/RioSerial` and `Software/RioImage` are now `ControllerSerial` and `ControllerImage`. The
  Rio-named keys are still published for one season so existing dashboards keep resolving.

`SwerveSubsystem` publishes `ModuleVelocities` and `ChassisVelocities`, with `ModuleStates` and
`ChassisSpeeds` kept as deprecated aliases for one season.

**Everything else kept its name.** Every builder, every mechanism signature, every binding call.

### Gone

- **PhotonVision.** No 2027 build, and Systemcore is Limelight-first — the pipeline is in the
  hardware. `PhotonSource` is excluded from the compile rather than deleted.
- **ChoreoLib.** No build past alpha-2, so `followChoreoPath()` has no path forward yet.
- **`ignoringDisable(boolean)`** still exists and compiles but is a **no-op**. v3's disabled-mode
  behaviour could not be confirmed against the alpha jars, and guessing at what runs while disabled
  is not something to do quietly.

PathPlanner works, bridged onto the v3 scheduler by `LegacyCommands.fromV2(...)` until PathPlanner
itself ships for v3.

### Verified, and not

540 tests pass on Windows and Linux. The example project builds. It publishes with a complete
18-dependency POM, and JitPack resolves it.

It has **not** run on a robot. See [the list of facts that could not be
verified](advanced/systemcore#facts-we-could-not-verify) — CAN termination, the camera transform
signs, the eMMC value format. Each is somewhere a guess would have looked exactly like knowledge.

---

## Coming from 1.x

Roughly in the order that will save you time.

1. **Install the 2027 toolchain.** Java 25 and Gradle 9.7+. Gradle 8 fails with
   `Unsupported class file major version 69`, which names neither the JDK nor the dependency.
2. **Add the two `--add-opens` flags.** Everything else can wait; this one stops the robot.
3. **Rename the five decorators.** A compile error each, so the compiler finds them all for you.
4. **Change your bus names.** `""` and `"rio"` no longer mean anything. `can_s0` is the default;
   run `CANBusPlanner.suggest()` before you rewire anything.
5. **Remove PhotonVision** from your vendordeps.
6. **Add Phoenix 6 and PathPlanner by hand.** Neither publishes a 2027 vendordep JSON at a
   discoverable URL yet, so the usual online install fetches a 2026 one — which installs cleanly and
   fails at build with an error naming none of this.
7. **Check `getChooser()` and `getServo()`** if you used them.

Then read [Systemcore & WPILib 2027](advanced/systemcore) once through. It is the page that explains
why things are the way they are, including the hardware behaviour that no amount of reading robot
code would reveal.

---

## 1.x

The roboRIO line, still maintained. Its release notes live on the
[stable documentation](https://tomas-1226.github.io/FrcCatalyst/) and in the
[repository README](https://github.com/TomAs-1226/FrcCatalyst#readme).

`v1.12.0` is the current release and the one to use on a competition robot.
