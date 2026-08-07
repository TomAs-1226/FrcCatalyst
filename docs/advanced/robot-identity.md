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

## What the robot is made to do

The rest of the sheet describes what the robot is made of. `CatalystFeatures`, added in v1.11.0,
describes what it is made to do, under `/Catalyst/Robot/Catalyst/`. Two robots with the same
drivetrain and the same motor count can be completely different machines because one of them scores
on its own and the other does not, and nothing else on the sheet says so.

Nothing here is declared, and there is no builder call to add. Each entry is written by the component
that created it, as it is created:

| Feature | Recorded by | Named after |
|---------|-------------|-------------|
| `Autopilot` | `Autopilot.Builder.build()` | the builder's `name(...)`, defaulting to `Autopilot` |
| `Strategist` | `Strategist.Builder.build()` | the name passed to `Strategist.named(...)` |
| `Sequence` | `BehaviorEngine.Builder.build()` | the name passed to `BehaviorEngine.sequence(...)` |
| `Goal Director` | `GoalDirector.Builder.build()` | the default goal, when the director has one |
| `Physics Core` | `PhysicsCore.Builder.build()` | the `PhysicsProfile` it was built at |

`Autopilot.build()` records an autopilot because an autopilot was just built. A feature list a team
maintains by hand is a list of what they meant to use; this one is a list of what came up, and when
the two differ the second is the one worth seeing on a driver station. It also means a part of the
library that threw on its way up is absent rather than published as a false flag — "no autopilot" and
"an autopilot that failed to build" would otherwise look identical on the wire, and only one of them
deserves a driver's attention. `PhysicsCore` records itself after every one of its own validation
checks for exactly that reason.

The label is chosen to be the thing that changes the answer. Physics Core publishes its profile
because the same robot at `BALANCED` and at `MINIMAL` is estimating differently — slip scoring and
residual monitoring are off in the second. A goal director publishes its default goal because a robot
that idles at `STOW` and one that idles somewhere ready to score are describing different intents,
and the default is the only thing at build time that says which.

Registration normally happens **after** `declare(...)`, since teams declare at the top of
`RobotContainer` and build their behaviours further down. Each record therefore re-derives and
rewrites the sheet, the same way a drivetrain coming up does — and because NT4 hands a subscriber the
last value of every topic, a dashboard connecting mid-match still receives the finished list rather
than the half of it that existed when it happened to connect.

## What is not published

**A fact Catalyst does not know is absent from the wire.** Not zero, not `-1`, not an empty string.

Once a figure has left the robot nothing downstream can tell a placeholder from a measurement: a
brownout threshold of `0` reads exactly as confidently as one of `6.8`. An absent key is the only
honest way to say "unknown". In practice:

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
- A robot that runs none of the parts listed above publishes **no `Catalyst` group at all**, not a
  set of `false` flags. The group lists what came up; it does not enumerate what did not.
- An empty list is treated as unknown rather than as "none". A robot with no cameras and a robot
  whose cameras Catalyst never saw look identical from inside the library, and publishing an empty
  array would assert the first with confidence earned for neither.

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

## What reads it

[Catalyst Console](https://github.com/TomAs-1226/CatalystConsole) subscribes to `/Catalyst/Robot/`
and renders the sheet under **Settings → Robot**, which is what the one line buys you. The robot's
plan comes first, drawn to scale from the published bumper, frame and module positions and captioned
with which of those three it was actually drawn from; then the name with team number, season and
controller model under it; then the feature list, and the rest of the sheet grouped as
software, controller, drivetrain, chassis, traction, power and hardware, ending with the power
channel map. Nothing is typed into the console — it is your robot's own account of itself.

This is why the absence rule matters in practice. A row whose key the robot never published is left
out of the card entirely rather than dashed, because a dash next to "Gyro" reads as "this robot has
no gyro" when the truth is "it did not mention one". A robot that declared a name and no geometry
gets the card without the drawing, which is the ordinary case rather than a broken one.

Metric or imperial is a display choice made in the console, not on the robot. Everything on the wire
stays SI, because that is what WPILib works in; the toggle exists because FRC writes its frame
perimeter and height rules in inches, so a team checking whether they are legal would otherwise be
converting by hand.

The console never controls the robot and nothing here needs wiring up on the console side. Anything
else subscribing to NetworkTables sees the same keys.

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
| `Software/CatalystGitDirty` | true when that build carried uncommitted changes |
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

### Catalyst

Absent in full unless something recorded itself. Written by the components as they are built, not
declared.

| Key | Source |
|-----|--------|
| `Catalyst/InUse` | every feature recorded, sorted |
| `Catalyst/<Feature>/Count` | how many distinct names were recorded, and at least 1 |
| `Catalyst/<Feature>/Names` | those names, absent when the instance had none |

`<Feature>` is the name from `Catalyst/InUse` with its spaces removed — `Goal Director` publishes
under `Catalyst/GoalDirector/`. The constants are capitalised on each word for this reason: spelling
one of them "Goal director" would put it on the wire as `Goaldirector`, and a dashboard reading
`GoalDirector` would silently find nothing.

`Count` counts distinct names rather than instances, so two sequences a team called `ThreePiece`
report one. It falls back to 1 for a feature whose instance had no name worth publishing — a goal
director built without a default goal, say. A blank name is never added to `Names`, because an empty
string in a list of names reads as a robot that named something `""`.

`InUse` is sorted rather than left in construction order, so a dashboard diffing two robots — or one
robot across two builds — sees differences that are real rather than differences in whichever order
the constructors happened to run.

## Naming the library's own build

```java
CatalystVersion.version();     // "1.11.0"
CatalystVersion.gitSha();      // Optional["d604d51"]
CatalystVersion.describe();    // "1.11.0 (d604d51, dirty)"
```

The build generates these into a compiled constant rather than stamping the jar manifest, because a
robot project shades Catalyst into its `FRCUserProgram` fat jar — manifests merge there and the
robot's wins, so `Package.getImplementationVersion()` came back null on exactly the machine a team
wants the number on. A build made from a source archive with no repository reports no git stamp at
all, rather than a placeholder.
