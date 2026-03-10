---
layout: default
title: Installation
nav_order: 2
parent: Getting Started
---

# Installation

## Prerequisites

- **WPILib 2026** installed ([download](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html))
- **CTRE Phoenix 6** vendordep installed in your robot project
- A **GradleRIO robot project** (created via WPILib project generator)

## Add FrcCatalyst via JitPack

Add the JitPack repository and FrcCatalyst dependency to your robot project's `build.gradle`:

```gradle
repositories {
    // ... your existing repositories ...
    maven { url "https://jitpack.io" }
}

dependencies {
    // ... your existing dependencies ...
    implementation "com.github.TomAs-1226:FrcCatalyst:v1.0.0"
}
```

## Add FrcCatalyst via Local Maven

If you prefer to build from source:

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
    implementation "com.frccatalyst:FrcCatalyst:1.0.0"
}
```

## Verify Installation

Create a simple test in your `RobotContainer`:

```java
import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.util.CatalystMath;

// In constructor:
System.out.println("Kraken X60 free speed: "
    + MotorType.KRAKEN_X60.freeSpeedRPS() + " RPS");
System.out.println("Processed joystick: "
    + CatalystMath.processJoystick(0.5, 0.05, 2.0, 1.0));
```

If it compiles and prints the values, you're good to go!

## Next Steps

Head to the [Quick Start](quickstart) guide to build your first mechanism.
