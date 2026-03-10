---
layout: default
title: Subsystems
nav_order: 4
has_children: true
---

# Subsystems
{: .no_toc }

FrcCatalyst provides three complex subsystem wrappers that integrate multiple components.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## SwerveSubsystem

Wraps CTRE Tuner X generated swerve code. Teams generate their drivetrain using Tuner X, then wrap it with `SwerveSubsystem` to get:

- **Field-centric and robot-centric drive** commands
- **Heading lock** - auto-holds heading when driver isn't rotating
- **Point-at-target** - always face a scoring target while translating
- **Drive-with-heading** - lock to a specific heading angle
- **PathPlanner integration** - one-line AutoBuilder configuration
- **Vision pose estimation** - `addVisionMeasurement()` bridge
- **Automatic telemetry** - pose, heading, speed to NetworkTables

```java
SwerveSubsystem drive = new SwerveSubsystem(
    TunerConstants.createDrivetrain(),
    4.5,
    SwerveSubsystem.PathPlannerConfig.builder()
        .translationPID(5.0, 0, 0)
        .rotationPID(5.0, 0, 0)
        .build()
);

// Heading lock drive (auto-holds heading)
drive.setDefaultCommand(drive.headingLockDrive(
    () -> -driver.getLeftY(),
    () -> -driver.getLeftX(),
    () -> -driver.getRightX(),
    0.05
));

// Point at speaker while driving
driver.rightBumper().whileTrue(drive.pointAtTarget(
    () -> -driver.getLeftY(),
    () -> -driver.getLeftX(),
    () -> new Translation2d(0.0, 5.55),
    0.05
));
```

## VisionSubsystem

Multi-camera pose estimation with Kalman filter integration. Supports both Limelight (MegaTag2) and PhotonVision cameras simultaneously.

**Features:**
- **Distance-scaled standard deviations** - trusts close targets more
- **Spin rejection** - ignores vision during fast rotation
- **Latency filtering** - rejects stale measurements
- **Per-cycle telemetry** - see which estimates are accepted/rejected

```java
VisionSubsystem vision = new VisionSubsystem(
    drive::addVisionMeasurement,
    drive::getHeading,
    drive::getChassisSpeeds,
    VisionConfig.builder()
        .addLimelight("limelight-front", frontCameraPose)
        .addPhotonCamera("cam-rear", rearCameraPose)
        .singleTagStdDevs(4, 8)
        .multiTagStdDevs(0.5, 1)
        .xyDistanceScaling(1.0)
        .rotDistanceScaling(1.5)
        .rejectDuringSpin(2.0)
        .maxLatency(0.5)
        .build()
);
```

## LEDSubsystem

Addressable LED pattern controller with pre-built effects.

**Built-in patterns:** solid, blink, rainbow, chase, breathe, alternating

```java
LEDSubsystem leds = new LEDSubsystem(
    LEDConfig.builder()
        .port(0)
        .length(60)
        .build()
);

// Alliance color by default
leds.setDefaultCommand(leds.solid(Color.kBlue));

// Rainbow when scoring
scoring.whileTrue(leds.rainbow());

// Blink green when game piece acquired
intake.hasPieceTrigger().whileTrue(leds.blink(Color.kGreen, 0.1));
```
