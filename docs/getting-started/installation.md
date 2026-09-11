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
  Catalyst 2.x targets `2027.0.0-alpha-6`; the 2027 toolchain ships **Java 25**, and Gradle 9.7 or
  later is required to read it. Gradle 8.x fails with `Unsupported class file major version 69`.
- A **Limelight Systemcore** running the matching OS beta. The library and the OS are versioned
  together — see [Systemcore & WPILib 2027](../advanced/systemcore) before flashing anything.
- A **GradleRIO robot project** (created via the WPILib project generator).
- Vendordeps: **CTRE Phoenix 6** and **PathPlanner**.

  {: .warning }
  > Neither publishes a 2027 vendordep JSON at a discoverable URL yet, so the usual
  > *Install new libraries (online)* flow fetches a 2026 one. PathPlanner's canonical
  > `PathplannerLib.json` still reports `frcYear 2026`, and installing it into a 2027 project
  > produces something that looks correctly configured and fails at build with an error naming
  > none of this. Add them by hand from the vendor's own 2027 instructions.

  **PhotonVision is not used.** There is no 2027 build, and Catalyst is Limelight-first on
  Systemcore — the pipeline is built into the hardware. If you had it installed for Catalyst 1.x,
  remove it.

## Option 1: Vendordep (Recommended)

The easiest install. In WPILib VS Code, open the command palette and run
**WPILib: Manage Vendor Libraries → Install new libraries (online)**, then
paste:

```
https://tomas-1226.github.io/FrcCatalyst/beta/vendordep/FrcCatalyst.json
```

That adds FrcCatalyst to your project and lets WPILib check for updates.
Make sure the Phoenix 6, PathPlanner and LimelightLib vendordeps are also
installed - Catalyst depends on them. PhotonVision is **not** one of them; see above.

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
    implementation "com.github.TomAs-1226:FrcCatalyst:v1.12.0"
}
```

## Option 3: Local Maven (From Source)

If you prefer to build from source or need to modify the library:

```bash
# Clone FrcCatalyst
git clone https://github.com/TomAs-1226/FrcCatalyst.git
cd FrcCatalyst

# Publish to local Maven
./gradlew publishToMavenLocal
```

Then in your robot project's `build.gradle`:

```gradle
repositories {
    mavenLocal()
}

dependencies {
    implementation "com.frccatalyst:FrcCatalyst:2.0.0-alpha.3"
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
