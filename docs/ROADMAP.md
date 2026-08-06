---
layout: default
title: Roadmap
nav_order: 9
---

# Roadmap & Competitive Analysis
{: .no_toc }

Where Catalyst stands against the rest of the FRC software ecosystem, and what comes next.
Landscape researched June 2026; roadmap current as of **v1.8.0 (August 2026)**.
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
- **Physical intelligence** — Physics Core (v1.5.0–v1.6.0): fused state with honest confidence, slip
  and collision detection, a live centre of mass and tipping margin, online gain and battery
  identification, and pre-flight capability evaluation. Nobody else packages this either.
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
| 1.7.0 | **Physics Core validated in simulation** — a ground-truth simulator, nine scored acceptance scenarios, three real defects fixed, and a robot-measurement guide |
| 1.6.0 | **Physics Core completed** — live mass tree and stability, ballistics, online parameter identification, fault isolation, capability evaluation, power planning, replay, opt-in constraints |

---

## Now: Physics Core

Tracked in [#33](https://github.com/TomAs-1226/FrcCatalyst/issues/33). The full RFC spans five phases;
**Phase 1 shipped in v1.5.0; Phases 2, 3, and 5 and most of Phase 4 shipped in v1.6.0.** What
remains is either coprocessor-class work or gated on validation from a real robot.

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

### Phase 2 — advisory integrations ✅ v1.5.0 / v1.6.0

`predictLaunchState()` drops into both aiming solvers with no new overload; the state machine and
`BehaviorEngine` consume Physics Core through their existing `guard` / `when` hooks with no API
change; `ModelResidualMonitor` gives mechanisms expected-versus-measured response; and
`CapabilityEvaluator` supplies the feasibility input `BehaviorEngine` fallbacks wanted. Autopilot
path-compromised events remain the one loose end — see [Next](#next).

### Phase 3 — bounded interventions ✅ v1.6.0, opt-in only

`PhysicsConstraints` computes low-confidence speed reduction, slip-aware and anti-tip acceleration
limits, and power-aware caps. **It applies none of them.** There is no installer and no periodic hook:
it hands you numbers and `explain()` tells you which limit binds, so every recommended slowdown is
attributable. Teams should log `speedScale()` alongside what they actually command for a session
before acting on it.

Still deliberately unshipped from this phase: **traction-aware odometry weighting and automatic replan
requests**, both of which would mean Physics Core writing pose or driving path selection. Those stay
out until shadow data from a real robot justifies them.

### Phase 4 — advanced backend 🚧 mostly shipped

Shipped in v1.6.0: **online parameter identification** (`FeedforwardIdentifier`,
`BatteryResistanceIdentifier`, on a recursive-least-squares core) and the **articulated whole-robot
model** (`ArticulatedRobotModel`, `StabilityModel`), which makes the centre of mass and the tipping
limit live rather than static.

Outstanding, and genuinely needing a coprocessor: fixed-lag smoothing, delayed-measurement replay
inside the estimator, a richer nonlinear estimator, and local vision process integration. This is what
would finally make `ADVANCED` and `SYSTEMCORE` mean something different from `BALANCED`.

### Phase 5 — higher-level predictive services ✅ v1.6.0

`CapabilityEvaluator` (feasibility, duration, position error, tip margin, voltage — each with a
derivation rather than a heuristic), `PhysicsReplay` for counterfactual analysis over recorded
samples, `PowerPredictor` for predictive load sequencing, and uncertainty-aware behaviour selection
through the confidence-scaled constraints.

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

### Simulation validation ✅ v1.7.0

`PhysicsValidator` runs Physics Core against a ground-truth simulator with deliberately imperfect
sensors and scores it against the RFC's acceptance criteria. Nine scenarios, all passing on the
reference robot, re-run across three noise seeds.

It earned its keep immediately: building it found three real defects that 300-odd unit tests had
missed — fusion silently disabled for direct `update(PhysicsSample)` callers, a collision detector
that fired on every hard launch, and a wheel-trust floor high enough that the filter re-joined the
wheels mid-slip. The fused-velocity result went from **0.08% to 48%** better than raw encoders once
those were fixed.

This is the rung below shadow mode, not a substitute for it. It shows the estimator recovers a known
truth from imperfect sensors; it says nothing about whether the *model* matches carpet.

### Robot validation 📋 — still the gate on everything else

Physics Core is feature-complete, simulation-validated, and has **never run on a robot**. The
simulation assumes constant friction, rigid contact, and uniform slip; a real field is messier than
all three.

What the first sessions need to establish, in order:

1. **Sensor disagreement sits near zero** while driving normally. Anything else is a calibration error
   that every downstream number inherits.
2. **Slip detection catches induced slip** without firing on hard braking over rough carpet.
3. **Collision detection reports one event per impact**, and clears.
4. **Confidence tracks reality** — falls when vision is covered, recovers when it returns.
5. **The identifiers converge** to something close to the SysId gains already in the constants file.
   If they do not, one of the two is wrong and that is worth knowing.
6. **`PhysicsConstraints` would have intervened at sensible moments** — logged alongside what was
   actually commanded, before anybody applies it.

### Autopilot path-compromised events 🚧

The one advisory integration still unwired: `Autopilot` logging when a confirmed disturbance has
displaced the robot off its planned path. The detection exists; the reporting hook does not.

### Carried forward: QuestNav pose source 📋

Still the highest-value unbuilt item, and Physics Core makes it more valuable, not less: QuestNav
gives absolute correction that survives losing sight of a tag, which is exactly what the confidence
model rewards. Research notes above.

### Advanced backend, if a coprocessor arrives 📋

Fixed-lag smoothing, delayed-measurement replay inside the estimator, a nonlinear estimator, and local
vision processing. These are what would make `ADVANCED` and `SYSTEMCORE` differ from `BALANCED`, and
they are the only remaining RFC items that genuinely need more compute than a roboRIO has.

### Under consideration

- **`catalyst.world`** — a game-independent world model (game pieces, field zones, targets, and their
  uncertainty), deliberately kept separate from Physics Core: Physics Core estimates physical
  reality, a world model represents season-specific entities.
- **Traction-aware odometry weighting** — the one Phase 3 intervention held back, because it would
  mean Physics Core writing pose. Gated on validation.

---

## Recommended next three

1. **Run Physics Core in shadow mode on a real robot.** It is feature-complete and validated in
   simulation, which is as far as it can get without carpet. That remaining gap is the single largest
   one in the library. It costs nothing — no control authority — and everything else in the physics
   track is gated on what it shows. Measure the robot first
   ([guide](advanced/physics-measurement.md)) and run `PhysicsValidator` against your own model; both
   happen before the robot is on a field.
2. **QuestNav source** — still low effort, still high visibility, and now worth more than it was.
3. **Autopilot path-compromised events** — the last advisory integration, so the whole stack sees what
   Physics Core sees.

Together these say: *Catalyst is the library that makes your robot reliable (self-test), accurately
localized (vision + QuestNav + physics), testable before it's built (maple-sim), and honest about
when it does not know where it is (Physics Core)* — on top of autonomy and aiming nobody else bundles.
