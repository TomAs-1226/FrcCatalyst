---
layout: default
title: Roadmap
nav_order: 9
---

# Roadmap & Competitive Analysis
{: .no_toc }

Where Catalyst stands against the rest of the FRC software ecosystem, and
the features that would push it further ahead. Researched June 2026.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## The landscape (2026)

| Project | What it's for | Catalyst overlap |
|---|---|---|
| **YAGSL** | Generic multi-vendor swerve | Catalyst is CTRE-first by design — fewer vendordeps, tighter integration |
| **AdvantageKit** | Deterministic log + replay | Catalyst has the IO/replay *layer* (`CatalystInputs`/`CatalystLog`) and bridges to AdvantageKit; no standalone replay harness |
| **maple-sim** | Physics-engine sim w/ game pieces | Catalyst uses WPILib analytic sims (no collisions / pieces) |
| **QuestNav** | Meta Quest VIO pose (more stable than vision) | Catalyst fuses cameras only |
| **Choreo** | Time-optimal trajectories | Catalyst supports PathPlanner only |
| **PhotonVision / Limelight** | AprilTag + object detection | Integrated via `VisionSubsystem` + `LimelightTriggers` |

### Where Catalyst already leads

No mainstream FRC library bundles these. This is the moat:

- **A browser tool suite** — Builder, Tuner, Health Dashboard, Motion
  Profile, PID, MotorType, CAN ID Planner. Hosted, zero-install.
- **Behavior framework** — `BehaviorEngine` / `Strategist` / `Autopilot`
  for resilient autos and teleop co-pilot, game-agnostic.
- **Shoot-On-The-Fly** — `AimingSolver` + `TurretMechanism` with
  defense compensation, built in.
- **Health & safety kit** — `HealthCheck` / `HealthMonitor` /
  `RobotSafety` / `HealthHistory` with a live dashboard.
- **Unified mechanism layer** — nine mechanisms, live tuning, CAN
  registry, one builder call each.

The strategy below is: **close the few real gaps competitors have, then
extend the lead with features nobody else packages.**

---

## Tier 1 — differentiators (push ahead)

### 1. `SystemCheck` — pre-match self-test framework ⭐ ✅ SHIPPED (v0.7.0)

**The pitch.** Before queueing, run every subsystem through a verification
routine and get a green/red board on the dashboard. Catches the failures
that actually lose matches — a loose connector, an inverted motor, a dead
follower, an encoder reading zero, a CAN device that didn't boot.

This addresses a *documented* team pain point and **no mainstream library
packages it.** It composes directly with what Catalyst already has
(`HealthMonitor`, `CANRegistry`, the mechanisms).

```java
SystemCheck check = SystemCheck.builder()
    .test("Elevator", () -> elevator.getMotor().runForCheck(2.0))  // applies V, watches encoder move
    .expectMotion("Elevator", () -> elevator.getVelocity(), 0.05)
    .expectCurrentBelow("Elevator", () -> elevator.getMotor().getStatorCurrent(), 60)
    .test("Follower aligned", () -> compareLeaderFollowerDirection(elevator))
    .test("All planned CAN present", CANRegistry::allPresent)   // cross-check vs CANIds.java
    .build();

driver.start().onTrue(check.run());   // results publish to /Catalyst/SystemCheck/<name>
```

Output: per-test pass/fail + reason, a single "ready to compete" boolean,
and a downloadable report (matches the Health Dashboard look). **Highest
recommended item** — unique, high-reliability-impact, builds on existing
pieces.

### 2. QuestNav pose source ⭐

**The pitch.** QuestNav (Meta Quest VIO) is the breakout 2026 pose-tracking
tech — "more stable and reliable than any FRC vision solution." Add a
`QuestNavSource` that plugs into the existing `VisionSubsystem` fusion
pipeline as a pose provider, so teams get drift-free localization fused
with their AprilTag cameras through the same deterministic multi-cam path
we just hardened.

```java
VisionConfig.builder()
    .driveSubsystem(drive)
    .addQuestNav("questnav", robotToHeadset)   // NT pose stream → CameraSource
    .addLimelight("limelight-front", frontCam)  // still fuse tags for absolute correction
    .build();
```

Riding a wave the whole community is adopting *right now*. Low effort
(it's an NT pose stream → `CameraSource`), high visibility.

### 3. maple-sim physics simulation

**The pitch.** Catalyst is pushing autonomy hard (behavior framework, SOTF,
pathfinding) — but you can't validate any of it without a robot. maple-sim
brings a physics engine with **game-piece interaction and collisions**.
Integrating it lets teams test the *entire* Catalyst autonomy stack in sim:
drive the `Strategist` against simulated fuel, watch the `Autopilot` cycle,
verify SOTF math against a simulated shot.

Ship a `CatalystSimWorld` that wires maple-sim's swerve sim to
`SwerveSubsystem` and exposes a simulated game-piece field, plus templates.
Pairs with the browser tools (visualize in AdvantageScope).

### 4. Deterministic log replay harness

**The pitch.** AdvantageKit's killer feature is *replay* — re-run a match
log through modified code to debug field-only bugs. Catalyst already has
the hard part: the `CatalystInputs` IO layer with `toLog`/`fromLog`. What's
missing is a shipped **WPILOG sink** + a **replay runner** so teams get
replay without adopting all of AdvantageKit.

```java
// record:
CatalystLog.setSink(new WpilogSink("/U/logs"));
// replay (offline):
CatalystReplay.from("match-42.wpilog").run(robotProject);  // feeds recorded inputs back
```

Completes AdvantageKit-parity for the one feature teams most envy, while
staying lighter-weight.

---

### 4. Deterministic log replay harness ✅ RECORD HALF SHIPPED (v0.8.0)

`WpilogSink` now records all Catalyst logging to a standard `.wpilog`
(AdvantageScope-readable, no extra dep). Full deterministic *replay*
still routes through AdvantageKit's `Logger` via the `CatalystInputs`
bridge — reimplementing AK's replay engine isn't worth the maintenance
when the IO layer already bridges to it.

---

## Tier 2 — close real gaps  ✅ ALL SHIPPED (v0.8.0)

### 5. Swerve odometry / wheel-radius calibration ✅

A `CalibrationRoutine` that spins the robot a known number of rotations
(or drives a known distance) and back-solves the **actual** wheel radius /
track width vs the CAD value — the documented source of auto inaccuracy.
Outputs the corrected constants to NT and a copy-paste snippet.

### 6. Brownout prediction → `RobotSafety` ✅

`BrownoutMonitor` — predicts sag voltage from `Σ current × R_internal`,
gives a graceful output scale, and preemptively trips `RobotSafety`.

### 7. Choreo trajectory support ✅

`SwerveSubsystem.followChoreoPath(name)` — loads Choreo `.traj` files
through PathPlanner (no extra vendordep) and follows them.

### 8. Vision piece-pursuit helper ✅

`SwerveSubsystem.driveToPiece(Supplier<Optional<Translation2d>>)` — the
primitive the `Autopilot` "acquire" action wants.

### 9. CAN bus utilization optimization ✅

`CatalystMotor.Builder.optimizeCanBus()` — raises only the used status
signals, silences the rest. Opt-in.

---

## Tier 3 — ecosystem & tools (evaluated; mostly declined)

### 10. Auto Builder browser tool — ❌ declined

PathPlanner's GUI already does visual auto editing well, and the
`BehaviorEngine` autos are reactive (preconditions, fallbacks) in a way a
static visual editor can't capture. Building a worse version isn't worth
it.

### 11. Match replay / log scrubber tool — ❌ declined

AdvantageScope is the gold-standard WPILOG scrubber and it's free. The
right move was making Catalyst *produce* AdvantageScope-readable logs
(`WpilogSink`, #4), not building an inferior viewer.

### 12. WPILib Epilog (`@Logged`) support — ⚠️ documented, not bundled

Epilogue's annotation processor logs annotated fields to DataLog. It works
*alongside* Catalyst (both land in the same `.wpilog` once `WpilogSink` is
installed) — annotate your own classes and they show up next to Catalyst's
output. Bundling it into Catalyst's internals would create a second,
conflicting logging path, so it stays opt-in on the team side.

---

## Reference: QuestNav integration (when we build it)

Research notes for the saved-for-later QuestNav source. QuestNav exposes a
`QuestNav` class (their vendordep) with `getAllUnreadPoseFrames()` →
`PoseFrame` (a `Pose3d` + `dataTimestamp()` + `isTracking()`), in the
**headset frame**. Robot-side pattern:

```
for (frame : questNav.getAllUnreadPoseFrames())
    if (frame.isTracking())
        addVisionMeasurement(frame.pose().transformBy(ROBOT_TO_QUEST.inverse()),
                             frame.dataTimestamp(), stdDevs);
```

This maps cleanly onto Catalyst's `CameraSource` interface — a
`QuestNavSource` would implement `getEstimatedPose()` returning the latest
tracking `PoseFrame` as a `PoseEstimate`, and drop straight into the
hardened multi-camera fusion. The only wrinkle: it needs the QuestNav
vendordep on the classpath, so it'd ship as an optional integration
(present only if the team installs QuestNav). Low effort once we commit.

---

## Recommended next three

1. **`SystemCheck`** — unique, high-impact, builds on existing pieces.
2. **QuestNav source** — rides the 2026 wave, low effort, high visibility.
3. **maple-sim integration** — unlocks testing the autonomy features we've
   been shipping, which otherwise can't be validated until a robot exists.

Together these say: *Catalyst is the library that makes your robot reliable
(self-test), accurately localized (QuestNav), and testable before it's
built (maple-sim) — on top of autonomy and aiming nobody else bundles.*
