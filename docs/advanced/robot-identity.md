---
layout: default
title: Robot Identity
parent: Advanced
nav_order: 12
---

# Robot Identity
{: .no_toc }

The robot's spec sheet, published on NetworkTables for whatever is reading.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## One line

```java
public RobotContainer() {
    RobotIdentity.declare("Ratchet");
    // ...
}
```

That publishes everything Catalyst can work out for itself under `/Catalyst/Robot/`: team number,
season, which roboRIO this is with its serial and image, the Catalyst and WPILib versions, the
brownout threshold, the CAN inventory, the gyro, and — as soon as a `SwerveSubsystem` exists —
drivetrain type, module count and positions, track width, wheelbase, odometry rate and top speed.

None of that is a parameter, on purpose. Anything passed in can drift out of date: regear the drive,
forget to edit an identity block, and the robot publishes last month's numbers with this month's
confidence. Where the library can read a fact, it reads it.

Order does not matter. The sheet is re-derived and rewritten whenever a new source of facts appears,
so `declare(...)` can sit at the top of `RobotContainer` and still pick up a drivetrain constructed
three lines later.

## The rest of it

`RobotIdentity.named(...)` takes the handful of facts nothing can measure:

```java
RobotIdentity.named("Ratchet")
    .robotCodeVersion(BuildConstants.GIT_SHA)      // your build's stamp, not a typed string
    .frameMeters(0.74, 0.70)                       // tape measure, excluding bumpers
    .bumperThicknessMeters(0.08)
    .heightMeters(0.52)
    .battery("MK ES17-12")
    .swerveModules(TunerConstants.FrontLeft, TunerConstants.FrontRight,
                   TunerConstants.BackLeft, TunerConstants.BackRight)
    .power(pdh)
    .pdhChannel(0, "Front left drive")
    .pdhChannel(1, "Front right drive")
    .publish();
```

`swerveModules(...)` is worth passing: it is the only route to the gear ratios. Phoenix builds a
`SwerveDrivetrain` from the module constants and then does not hand them back, so a constructed
drivetrain can be asked its module positions but not its reduction. Passing the same objects the
drivetrain was built from cannot fall out of step with it, which typing the ratios in would.

`pdhChannel(...)` exists because the power distribution module knows how many channels it has and
what each is drawing, but not what is wired to them — and at boot a channel in use draws exactly as
much as one that is not.

## What is not published

**A fact Catalyst does not know is absent from the wire.** Not zero, not `-1`, not an empty string.

Catalyst Console draws a dash for a key the robot never published and a number for one it did, so an
absent key is the only honest way to say "unknown" — a placeholder arrives looking exactly like a
measurement, and nothing downstream can tell the difference. In practice:

- No `SwerveSubsystem` means **no `Drivetrain` group at all**, rather than a group full of zeros.
- No deployed PathPlanner settings means **no mass and no moment of inertia**. Catalyst holds no
  fallback figure.
- A module layout that is not rectangular publishes its **module positions and no track width**.
  `2*max|y|` by `2*max|x|` will happily answer for a diamond, and the answer describes a rectangle
  those modules do not sit on.
- `Chassis/BumperLengthMeters` is frame plus twice the bumper thickness, so it appears only when
  both of those were declared.
- A desktop or simulation has no rio, so `Identity/RioSerial`, `Identity/RioComment` and
  `Software/RioImage` are simply not there.

`SpecSheet` is what enforces this. Every setter takes an `Optional`, and there is deliberately no
overload that accepts a bare `double` — writing a placeholder takes more effort than writing the
truth rather than less.

## Publishing once is enough

An NT4 server keeps the last published value of every topic and sends it to a subscriber the moment
it subscribes. A dashboard that connects mid-match, or reconnects after the radio drops, therefore
receives the whole sheet on connect without the robot republishing anything. This is a boot-time
write, not a periodic one.

Nothing is marked persistent, and that is a decision rather than an oversight. A persistent topic is
saved to the roboRIO and republished by the server on the *next* boot — so a build that threw before
reaching its `declare(...)` would serve last week's mass with a fresh timestamp and no way for the
dashboard to tell. Absent is correct; stale-but-confident is the defect.

The sheet goes out through `CatalystLog`, so a team that has installed a WPILOG or AdvantageKit sink
gets it in their log alongside everything else.

## Keys

Everything lands under `/Catalyst/Robot/`. Units are in the key names, as everywhere else in
Catalyst.

### Identity

| Key | Source |
|-----|--------|
| `Identity/Name` | declared — the one fact only the team knows |
| `Identity/TeamNumber` | `RobotController.getTeamNumber()`, omitted when it reports 0 |
| `Identity/Season` | the year in the runtime WPILib version |
| `Identity/Controller` | `roboRIO`, `roboRIO 2` or `Simulation`, from the HAL runtime type |
| `Identity/RioSerial` | `RobotController.getSerialNumber()` |
| `Identity/RioComment` | the Imaging Tool's comments field, when a team filled it in |

### Software

| Key | Source |
|-----|--------|
| `Software/CatalystVersion` | compiled into the library by its own build |
| `Software/CatalystGitSha` | short sha, absent when the build had no repository |
| `Software/CatalystGitDirty` | whether that build's tree was clean |
| `Software/CatalystCommitTime` | ISO-8601 committer date |
| `Software/RobotCodeVersion` | declared |
| `Software/RobotCodeBuild` | declared |
| `Software/WPILibVersion` | the runtime constant, not Catalyst's build-time pin |
| `Software/JavaVersion` | `java.version` |
| `Software/RioImage` | parsed off the rio's own image metadata |
| `Software/FpgaVersion` | `RobotController.getFPGAVersion()` |

Phoenix, PathPlanner and PhotonVision versions are **not** published. They are pinned in Catalyst's
`build.gradle` and nowhere else — none of the three exposes a runtime constant — and a robot project
resolves its own copies. Reporting the pins would report what Catalyst was compiled against as
though it were what is running.

### Drivetrain

Absent in full unless a `SwerveSubsystem` exists.

| Key | Source |
|-----|--------|
| `Drivetrain/Type` | `Swerve` |
| `Drivetrain/Modules` | module count |
| `Drivetrain/ModuleLocations` | `[x0, y0, x1, y1, ...]`, robot-relative metres |
| `Drivetrain/TrackWidthMeters` | derived, rectangular layouts only |
| `Drivetrain/WheelBaseMeters` | derived, rectangular layouts only |
| `Drivetrain/MaxSpeedMps` | what the subsystem was built with |
| `Drivetrain/MaxAngularRateRadPerSec` | what the subsystem limits rotation to |
| `Drivetrain/OdometryHz` | `SwerveDrivetrain.getOdometryFrequency()` |
| `Drivetrain/CanFd` | `SwerveDrivetrain.isOnCANFD()` |
| `Drivetrain/DriveGearRatio` | module constants, when they all agree |
| `Drivetrain/SteerGearRatio` | module constants, when they all agree |
| `Drivetrain/WheelRadiusMeters` | module constants, else PathPlanner settings |
| `Drivetrain/SlipCurrentAmps` | module constants |
| `Drivetrain/WheelCof` | PathPlanner settings |
| `Drivetrain/DriveCurrentLimitAmps` | PathPlanner settings |

### Chassis

| Key | Source |
|-----|--------|
| `Chassis/MassKg` | deployed PathPlanner settings |
| `Chassis/MoiKgM2` | deployed PathPlanner settings |
| `Chassis/FrameLengthMeters` | declared |
| `Chassis/FrameWidthMeters` | declared |
| `Chassis/BumperThicknessMeters` | declared |
| `Chassis/BumperLengthMeters` | frame + 2 × thickness |
| `Chassis/BumperWidthMeters` | frame + 2 × thickness |
| `Chassis/HeightMeters` | declared |

Mass comes from the deployed PathPlanner settings because that is where a team has already typed it,
and it is the number the robot's own path follower runs on. `RobotModel` is deliberately not
consulted: it fills its unset fields with defaults and cannot say afterwards which figures the team
actually measured.

### Power

| Key | Source |
|-----|--------|
| `Power/BrownoutVolts` | `RobotController.getBrownoutVoltage()` |
| `Power/Module` | `CTRE PDP` or `REV PDH`, from an attached `PowerDistribution` |
| `Power/Channels` | channel count on that module |
| `Power/ChannelsInUse` | declared, as `channel|what` rows |
| `Power/Battery` | declared |

### Hardware

| Key | Source |
|-----|--------|
| `Hardware/CanDevices` | count, merged across sources |
| `Hardware/Inventory` | `type|count` rows |
| `Hardware/Gyro` | the drivetrain's Pigeon 2, else a registered `CatalystGyro` |
| `Hardware/GyroCanId` | its CAN id |
| `Hardware/Cameras` | camera names from `VisionSubsystem` |

The inventory merges two sources because neither is complete on its own. `CANRegistry` holds what the
mechanisms claimed as they were built and misses the drivetrain entirely — Phoenix constructs the
module motors and encoders inside `SwerveDrivetrain` and nothing tells the registry about them — so a
registry-only count reports a swerve robot's twelve largest motors as none. Devices are merged on
`(bus, id)`, so one counted twice is still counted once.

## Naming the library's own build

```java
CatalystVersion.version();     // "1.10.0"
CatalystVersion.gitSha();      // Optional["6e02513"]
CatalystVersion.describe();    // "1.10.0 (6e02513, dirty)"
```

The build generates these into a compiled constant rather than stamping the jar manifest, because a
robot project shades Catalyst into its `FRCUserProgram` fat jar — manifests merge there and the
robot's wins, so `Package.getImplementationVersion()` came back null on exactly the machine a team
wants the number on. A build made from a source archive with no repository reports no git stamp at
all, rather than a placeholder.
