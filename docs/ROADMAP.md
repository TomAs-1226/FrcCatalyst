---
layout: default
title: Roadmap
nav_order: 9
---

# Roadmap & Competitive Analysis
{: .no_toc }

Where Catalyst stands against the rest of the FRC software ecosystem, and what comes next.
Landscape researched June 2026; roadmap current as of **v1.5.0 (August 2026)**.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## How to read this page

The 2026 roadmap is **done**. Every Tier 1 and Tier 2 item either shipped or was evaluated and
declined for a stated reason, and this page had gone stale as a result
([#31](https://github.com/TomAs-1226/FrcCatalyst/issues/31)). What follows is the closed-out record
first, then the live plan.

Items are marked ✅ shipped, ❌ declined, 🚧 in progress, or 📋 planned. Anything planned is a
statement of intent, not a promise of a date — Catalyst ships when a feature is tested, not when a
calendar says so.

---

## The landscape (2026)

| Project | What it's for | Catalyst overlap |
|---|---|---|
| **YAGSL** | Generic multi-vendor swerve | Catalyst is CTRE-first by design — fewer vendordeps, tighter integration |
| **AdvantageKit** | Deterministic log + replay | Catalyst has the IO/replay *layer* (`CatalystInputs`/`CatalystLog`) and bridges to AdvantageKit; no standalone replay harness |
| **maple-sim** | Physics-engine sim w/ game pieces | Integrated — `SwerveSubsystem` exposes the sim seam (`getModuleTargets`, `setSimPose`, `disableInternalSim`) |
| **QuestNav** | Meta Quest VIO pose (more stable than vision) | Not yet built; research notes retained below |
| **Choreo** | Time-optimal trajectories | Supported — `followChoreoPath(name)` |
| **PhotonVision / Limelight** | AprilTag + object detection | Integrated via `VisionSubsystem` + `LimelightTriggers` |

### Where Catalyst leads

No mainstream FRC library bundles these. This is the moat:

- **A browser tool suite** — Builder, Tuner, Health Dashboard, Motion Profile, PID, MotorType, CAN ID
  Planner. Hosted, zero-install, plus an optional desktop app (v1.4.0) with an MCP server for AI agents.
- **Behavior framework** — `BehaviorEngine` / `Strategist` / `Autopilot` for resilient autos and a
  teleop co-pilot, game-agnostic.
- **Universal state machine** — declarative states, guards, staged transitions, and routing, with
  validation and telemetry.
- **Shoot-On-The-Fly** — two solvers (`AimingSolver`, `AimingSolverVector`) plus `TurretMechanism`.
- **Health & safety kit** — `HealthCheck` / `HealthMonitor` / `RobotSafety` / `HealthHistory` /
  `SystemCheck` / `BrownoutMonitor` / `LoopMonitor`, with a live dashboard.
- **Physical intelligence** — Physics Core (v1.5.0): fused state with honest confidence, slip
  scoring, collision detection, shot-release prediction. Nobody else packages this either.
- **Unified mechanism layer** — nine mechanisms, live tuning, CAN registry, one builder call each.

---

## Closed out: the 2026 roadmap

### Tier 1 — differentiators

| # | Item | Status |
|---|---|---|
| 1 | `SystemCheck` — pre-match self-test framework | ✅ v0.7.0 |
| 2 | QuestNav pose source | 📋 still open — see [below](#carried-forward-questnav-pose-source) |
| 3 | maple-sim physics simulation | ✅ seam in v0.9.0-beta (`setSimPose`), hardened in v1.2.1 (`getModuleTargets` / `disableInternalSim`) + [wiring guide](advanced/simulation.md) |
| 4 | Deterministic log replay harness | ✅ record half (v0.8.0); replay routes through AdvantageKit |

On #4: `WpilogSink` records all Catalyst logging to a standard `.wpilog` (AdvantageScope-readable, no
extra dependency). Full deterministic *replay* goes through AdvantageKit's `Logger` via the
`CatalystInputs` bridge — reimplementing AK's replay engine is not worth the maintenance when the IO
layer already bridges to it.

### Tier 2 — close real gaps ✅ all shipped (v0.8.0)

| # | Item | What shipped |
|---|---|---|
| 5 | Swerve odometry / wheel-radius calibration | `WheelRadiusCalibration` — back-solves actual radius vs the CAD value |
| 6 | Brownout prediction | `BrownoutMonitor` — predicts sag from `Σ I × R`, graceful output scale, trips `RobotSafety` |
| 7 | Choreo trajectory support | `followChoreoPath(name)` — loads `.traj` through PathPlanner, no extra vendordep |
| 8 | Vision piece-pursuit helper | `driveToPiece(Supplier<Optional<Translation2d>>)` |
| 9 | CAN bus utilization optimization | `CatalystMotor.Builder.optimizeCanBus()` — opt-in |

### Tier 3 — ecosystem & tools

| # | Item | Status |
|---|---|---|
| 10 | Auto Builder browser tool | ❌ declined — PathPlanner's GUI does visual editing well, and `BehaviorEngine` autos are reactive in a way a static editor cannot capture |
| 11 | Match replay / log scrubber tool | ❌ declined — AdvantageScope is the free gold standard; the right move was making Catalyst *produce* readable logs (#4) |
| 12 | WPILib Epilogue (`@Logged`) support | ⚠️ documented, not bundled — works alongside Catalyst once `WpilogSink` is installed; bundling it would create a second, conflicting logging path |

### Shipped since the last roadmap revision

| Version | What |
|---|---|
| 1.2.x | `AimingSolverVector` (community PR [#27](https://github.com/TomAs-1226/FrcCatalyst/pull/27)), state-machine internals, maple-sim seam |
| 1.3.2 | Field-centric red-alliance flip made explicit; loop-cost guidance (team 3211 report) |
| 1.3.3 | `LoopMonitor` — measure the loop before reaching for the levers (team 3211 report) |
| 1.4.0 | **Catalyst Desktop** — optional companion app, offline vendordep install, auto-update, MCP server for AI agents |
| 1.5.0 | **Physics Core** — Phase 1, shadow mode |

---

## Now: Physics Core

Tracked in [#33](https://github.com/TomAs-1226/FrcCatalyst/issues/33). The full RFC spans five phases;
**Phase 1 shipped in v1.5.0** and the rest is staged behind validation.

The scope statement, unchanged from the RFC: *an optional collection of synchronized data pipelines,
physical models, estimation algorithms, predictive services, and digital-twin adapters that use real
robot behaviour to improve performance throughout the Catalyst ecosystem.* Not a general-purpose
rigid-body engine, not a replacement for WPILib or maple-sim, not a second command scheduler, not a
strategy engine.

### Phase 0 — feasibility and architecture ✅ v1.5.0

Scope and non-goals agreed; `RobotStateSource` / `UncertainRobotStateSource` defined and implemented
by both `SwerveSubsystem` and `PhysicsCore`; timestamped signal and observation contracts defined;
`PhysicalRobotState` drafted; a weighted complementary observer selected over an error-state EKF for
Phase 1 (an EKF's extra machinery buys nothing until there is real data to tune it against).

### Phase 1 — shadow-mode Physics Lite ✅ v1.5.0

No control authority. Synchronized signal buffers and latency-aware snapshots; fused
velocity and acceleration; slip-aware confidence and covariance; per-module slip scoring;
disturbance residuals and collision detection; a pose-observation adapter with outlier gating;
release-time state prediction; full telemetry under `Catalyst/Physics/...`. 94 HAL-free unit tests.
See the [Physics Core guide](advanced/physics.md).

### Phase 2 — advisory integrations 🚧 partly shipped

`predictLaunchState()` already drops into both aiming solvers, and the state machine and
`BehaviorEngine` consume Physics Core through their existing `guard` / `when` hooks with no API
change. Still to come: Autopilot logging path-compromised conditions, and mechanisms exposing
expected-versus-measured response.

### Phase 3 — bounded interventions 📋 explicit opt-in only

Traction-aware odometry weighting, low-confidence speed reduction, replan requests after a confirmed
disturbance, holding a shot when the predicted miss radius is too large, anti-tip and power-aware
constraints. **Gated on Phase 1 validation on a real robot** — nothing here ships before the shadow
data says it earns its place.

### Phase 4 — advanced backend 📋

Fixed-lag smoothing, delayed-measurement replay, a richer nonlinear estimator, online parameter
identification, local vision process integration, an articulated whole-robot model. This is where the
`ADVANCED` and `SYSTEMCORE` compute profiles start to mean something different from `BALANCED`.

### Phase 5 — higher-level predictive services 📋

Capability evaluation ("is this action feasible, and what will it cost?"), counterfactual replay,
predictive power scheduling, uncertainty-aware behaviour selection.

### Kill criteria

Stated up front and meant literally: **if an advanced model does not measurably outperform the simpler
implementation it replaces, it stays experimental or gets removed.** Every service must clear offline
data, then simulation, then shadow mode, then controlled robot experiments before it gets any control
authority. Acceptance targets — fused velocity beating encoder-only velocity, slip detection catching
induced slip at an acceptable false-positive rate, confidence rising during known disturbances and
falling after valid corrections, release-state prediction improving measured SOTF accuracy — are in
the [RFC](https://github.com/TomAs-1226/FrcCatalyst/issues/33).

---

## Next

### Carried forward: QuestNav pose source 📋

Still the highest-value unbuilt item, and Physics Core makes it more valuable, not less: QuestNav
gives absolute correction that survives losing sight of a tag, which is exactly what the confidence
model rewards.

QuestNav exposes a `QuestNav` class (their vendordep) with `getAllUnreadPoseFrames()` → `PoseFrame`
(a `Pose3d` + `dataTimestamp()` + `isTracking()`), in the **headset frame**:

```java
for (frame : questNav.getAllUnreadPoseFrames())
    if (frame.isTracking())
        addVisionMeasurement(frame.pose().transformBy(ROBOT_TO_QUEST.inverse()),
                             frame.dataTimestamp(), stdDevs);
```

This maps cleanly onto `CameraSource` — a `QuestNavSource` returning the latest tracking `PoseFrame`
as a `PoseEstimate` drops into the hardened multi-camera fusion, and feeds `physics.observe(...)` at
the same time. The wrinkle: it needs the QuestNav vendordep on the classpath, so it ships as an
optional integration.

### Physics Core Phase 2 completion 🚧

Autopilot path-compromised events and mechanism expected-versus-measured response — the two advisory
integrations not yet wired.

### Robot validation of Phase 1 📋

The gate on everything downstream. Nothing in Phase 3 gets built before the shadow data from a real
robot says the Phase 1 estimates are worth acting on.

### Under consideration

- **`catalyst.world`** — a game-independent world model (game pieces, field zones, targets, and their
  uncertainty), deliberately kept separate from Physics Core: Physics Core estimates physical
  reality, a world model represents season-specific entities.
- **Counterfactual replay** — "would this estimator have rejected that bad vision frame?" run against
  recorded logs. Depends on Phase 4.
- **Residual-based fault isolation** — ranking likely causes when measurements disagree with the
  model, using honest diagnostic scores rather than probabilities until they are validated.

---

## Recommended next three

1. **Validate Physics Core Phase 1 on a real robot** — everything else in the physics track is gated
   on it, and it costs nothing to run in shadow mode.
2. **QuestNav source** — still low effort, still high visibility, and now worth more than it was.
3. **Finish Phase 2 advisory integrations** — Autopilot and mechanisms, so the whole stack can see
   what Physics Core sees.

Together these say: *Catalyst is the library that makes your robot reliable (self-test), accurately
localized (vision + QuestNav + physics), testable before it's built (maple-sim), and honest about
when it does not know where it is (Physics Core)* — on top of autonomy and aiming nobody else bundles.
