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
- **Heading lock** — auto-holds heading when driver isn't rotating
- **Point-at-target** — always face a scoring target while translating
- **Drive-with-heading** — lock to a specific heading angle
- **PathPlanner integration** — one-line AutoBuilder configuration
- **Vision pose estimation** — `addVisionMeasurement()` bridge
- **Automatic telemetry** — pose, heading, speed to NetworkTables
- **Skew correction** — Team 1690's pose exponential discretization
- **Slew rate limiting** — smooth acceleration with asymmetric profiles
- **Snap-to-angle** — auto-snap heading to predefined angles
- **Advanced drive** — combined deadband, slew, heading lock, skew correction, snap-to-angle
- **Slow mode** — toggleable speed multiplier for precision
- **Auto-align** — drive while auto-rotating to a target heading

```java
SwerveSubsystem drive = new SwerveSubsystem(
    TunerConstants.createDrivetrain(),
    4.5,
    SwerveSubsystem.PathPlannerConfig.builder()
        .translationPID(5.0, 0, 0)
        .rotationPID(5.0, 0, 0)
        .build()
);

// Advanced drive (recommended default command)
// Combines deadband, slew limiting, heading lock, snap-to-angle, and skew correction
drive.setSkewCorrectionEnabled(true);
drive.enableSlewRateLimiting(2.0, 5.0); // accel, decel
drive.setSnapToAngles(List.of(0.0, 90.0, 180.0, 270.0), 5.0);
drive.setDefaultCommand(drive.advancedDrive(
    () -> -driver.getLeftY(),
    () -> -driver.getLeftX(),
    () -> -driver.getRightX(),
    0.05
));

// Slow mode for precision alignment
driver.leftBumper().whileTrue(drive.slowModeWhileHeld(0.3));

// Point at speaker while driving
driver.rightBumper().whileTrue(drive.pointAtTarget(
    () -> -driver.getLeftY(),
    () -> -driver.getLeftX(),
    () -> new Translation2d(0.0, 5.55),
    0.05
));
```

{: .tip }
See the [Advanced Features](../advanced/) section for detailed documentation on skew correction, slew rate limiting, snap-to-angle, and auto-align.

## VisionSubsystem

Multi-camera pose estimation with Kalman filter integration. Supports both Limelight (MegaTag2) and PhotonVision cameras simultaneously.

**Features:**
- **Distance-scaled standard deviations** — trusts close targets more
- **Ambiguity-scaled std devs** — higher ambiguity = less trust
- **Spin rejection** — ignores vision during fast rotation
- **High-speed rejection** — ignores vision while driving fast
- **Heading divergence filtering** — rejects single-tag poses that disagree with the gyro
- **Kalman innovation tracking** — logs innovation norms for tuning
- **Latency filtering** — rejects stale measurements
- **Configurable field bounds** — custom field dimensions for bounds checking
- **Per-cycle telemetry** — see which estimates are accepted/rejected with reasons

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
        .rejectDuringHighSpeed(3.0)       // reject when > 3 m/s
        .maxHeadingDivergence(15.0)       // reject if heading disagrees > 15 deg
        .fieldDimensions(16.54, 8.21)     // custom field bounds
        .maxLatency(0.5)
        .build()
);
```

## LEDSubsystem

Addressable LED pattern controller with 14 pre-built effects.

**Basic patterns:** solid, blink, rainbow, chase, breathe, alternating

**Advanced patterns:** fire, gradient, scrolling gradient, strobe, larson scanner, dynamic progress, status indicator, alignment indicator

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

// Fire effect for celebration
scoring.whileTrue(leds.fire());

// Alignment indicator for driver
aligning.whileTrue(leds.alignmentIndicator(() -> visionOffset, 2.0));

// Progress bar for elevator height
leds.dynamicProgress(Color.kGreen, () -> elevator.getPosition() / 1.2);
```

{: .tip }
See the [Advanced Features](../advanced/) section for details on all new LED patterns.
