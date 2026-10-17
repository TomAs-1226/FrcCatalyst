---
layout: default
title: Installation
nav_order: 1
parent: Getting Started
---

# Installation
{: .no_toc }

Add FrcCatalyst to your WPILib robot project.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Prerequisites

- **WPILib 2027 alpha** installed ([download](https://github.com/wpilibsuite/allwpilib/releases)).
  Catalyst 2.x targets `2027.0.0-alpha-7`; the 2027 toolchain ships **Java 25**, and Gradle 9.7 or
  later is required to read it. Gradle 8.x fails with `Unsupported class file major version 69`.
- A **Limelight Systemcore** running the matching OS beta. The library and the OS are versioned
  together — see [Systemcore & WPILib 2027](../advanced/systemcore) before flashing anything.
- A **GradleRIO robot project** (created via the WPILib project generator).
- Vendordeps: **CTRE Phoenix 6** `26.70.0-alpha-2` or later — the first Phoenix build for WPILib
  alpha-7, published 2026-09-18. RC1 source pins **PathPlanner** `2027.0.0-alpha-4.1` as an
  optional compile-time dependency and LimelightLib `2.0.0-beta9-alpha7`.

  {: .warning }
  > **CTRE devices need 26.70.x firmware.** Phoenix 6 `26.70.0-alpha-2` is the first Phoenix build
  > for WPILib alpha-7, but it **requires 26.70.x device firmware** — every TalonFX, CANcoder and
  > Pigeon has to be re-flashed before it will talk to this line, which makes moving a robot here
  > an afternoon's work, not a `git checkout`. See
  > [Versions and compatibility](https://tomas-1226.github.io/FrcCatalyst/versions.html).
  >
  > **PathPlanner is optional.** RC1 source includes the WPILib Commands v2 bridge needed by
  > PathPlanner while Catalyst uses Commands v3. PathPlanner itself remains `compileOnly`, so a
  > consumer that uses it must supply its matching vendordep/repository. This is a source dependency
  > statement, not a claim that autonomous behavior has been qualified on hardware.
  > Verify downloaded vendor JSON targets the intended 2027 version before building.

  **PhotonVision is not used.** There is no 2027 build, and Catalyst is Limelight-first on
  Systemcore — the pipeline is built into the hardware. If you had it installed for Catalyst 1.x,
  remove it.

## Source candidate and published dependency

**Published prerelease:** `2.0.0-rc.1` (tag `v2.0.0-rc.1`).

The RC1 JitPack POM, binary jar, sources and Javadoc were verified before the published
vendordep moved from beta.2. Install the release below or build the exact tag from source.
The [GitHub prerelease](https://github.com/TomAs-1226/FrcCatalyst/releases/tag/v2.0.0-rc.1)
contains downloadable artifacts and checksums. This is a Java 25/WPILib 2027 alpha-7
Systemcore prerelease, with hardware qualification still required.

## Option 1: Vendordep (Recommended)

The easiest install. In WPILib VS Code, open the command palette and run
**WPILib: Manage Vendor Libraries → Install new libraries (online)**, then
paste:

```
https://tomas-1226.github.io/FrcCatalyst/beta/vendordep/FrcCatalyst.json
```

That adds FrcCatalyst to your project and lets WPILib check for updates.
Match Phoenix 6 and LimelightLib to the chosen Catalyst build; PathPlanner is optional.
The RC1 dependency uses the compatibility pins listed above. PhotonVision is **not** part
of this build. Optional PathPlanner users must add its matching vendor dependency.

## Option 2: JitPack (build.gradle)

If you'd rather add it by hand, put the JitPack repository and the
dependency in your robot project's `build.gradle`:

```gradle
repositories {
    // ... your existing repositories ...
    maven { url "https://jitpack.io" }
}

dependencies {
    // ... your existing dependencies ...
    implementation "com.github.TomAs-1226:FrcCatalyst:v2.0.0-rc.1"
}
```

## Option 3: Local Maven (From Source)

If you prefer to build from source or need to modify the library:

```bash
# Clone FrcCatalyst
git clone https://github.com/TomAs-1226/FrcCatalyst.git
cd FrcCatalyst
git checkout v2.0.0-rc.1
# Record and review the exact commit before building.
git rev-parse HEAD

# Test the library/example before publishing only to this machine.
./gradlew build publishToMavenLocal
```

The local Maven example below names the source candidate. Publishing to local Maven does not
publish a release remotely. An included Gradle build can avoid local Maven publication entirely.

Then in your robot project's `build.gradle`:

```gradle
repositories {
    mavenLocal()
}

dependencies {
    implementation "com.frccatalyst:FrcCatalyst:2.0.0-rc.1"
}
```

## Required: two JVM flags

Add these to your robot program's launch arguments. Without them the robot boots, the dashboard
connects, and the **first command scheduled** throws `ExceptionInInitializerError` from inside
WPILib — naming no class of yours, at whatever moment a driver first pressed a button.

```
--add-opens java.base/jdk.internal.vm=ALL-UNNAMED
--add-opens java.base/java.lang=ALL-UNNAMED
```

In `build.gradle`:

```groovy
frc {
    jvmArgs.addAll([
        "--add-opens", "java.base/jdk.internal.vm=ALL-UNNAMED",
        "--add-opens", "java.base/java.lang=ALL-UNNAMED",
    ])
}
```

{: .warning }
> **Both are needed, and the second only appears once the first is fixed.** `jdk.internal.vm` gets
> the command scheduler *constructed*; `java.lang` gets a command *scheduled*. Add one, deploy, and
> you meet the other on the field.

Commands v3 runs on JDK continuations, which live in `jdk.internal.vm` and are reached by
reflection — hence the flags. Catalyst checks for both when it builds a command and throws something
legible if either is missing, so you find out at the line that built the command rather than several
seconds later. To ask directly:

```java
CommandRuntime.isAvailable();   // false means the flags are missing
```

## Verify Installation

Create a simple test in your `RobotContainer` to confirm everything is working:

```java
import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.util.CatalystMath;

// In your RobotContainer constructor:
System.out.println("Kraken X60 free speed: "
    + MotorType.KRAKEN_X60.freeSpeedRPS() + " RPS");
System.out.println("Processed joystick: "
    + CatalystMath.processJoystick(0.5, 0.05, 2.0, 1.0));
```

If it compiles and prints the values, you're good to go!

## Next Steps

Head to the [Quick Start](quickstart) guide to build your first mechanism in under 5 minutes.

## RC1 release gates

This preparation is for Systemcore OS beta 14 / WPILib alpha-7 / Java 25, not a roboRIO 2026
drop-in. Preserve the two runtime `--add-opens` flags documented in the
[Systemcore guide](../advanced/systemcore). Device firmware must match the documented 26.70.x line.

Before announcing RC1 as installable, require a clean full build for its exact commit, passing
regressions, an example/consumer install check against the final artifact, and a verified Maven POM.
The downstream Numbers shooting simulation failure was diagnosed and repaired in an isolated
team-repository fixture and simulation model; see the
[17 October validation evidence]({% link advanced/shooting-simulation-validation.md %}).
That patch does not change the published RC1 binary or qualify hardware. Keep optional
control/observer features at their documented defaults.
ARES 1.1 denotes the current multicamera scope; the ARES 2.0 ROS 2-style prototype is deferred.
No tag, public release, deployed robot package or firmware change follows from this source guide.
