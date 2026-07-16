---
layout: default
title: Home
nav_order: 1
permalink: /
---

# FrcCatalyst
{: .fs-9 }

A Java library of pre-built mechanism building blocks for FRC robots on Phoenix 6 and WPILib 2026.
{: .fs-6 .fw-300 }

[Get Started](getting-started/installation){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 }
[Tools](tools/){: .btn .fs-5 .mb-4 .mb-md-0 .mr-2 }
[GitHub](https://github.com/TomAs-1226/FrcCatalyst){: .btn .fs-5 .mb-4 .mb-md-0 }

{% include hero3d.html %}

<p style="margin-top: -8px">
  <img src="https://img.shields.io/badge/WPILib-2026.2.1-1f6feb?style=flat-square" alt="WPILib"/>
  <img src="https://img.shields.io/badge/Phoenix%206-26.1.1-e94560?style=flat-square" alt="Phoenix 6"/>
  <img src="https://img.shields.io/badge/Java-17-orange?style=flat-square&logo=openjdk" alt="Java 17"/>
  <img src="https://img.shields.io/badge/PathPlanner-2026.1.2-7c3aed?style=flat-square" alt="PathPlanner"/>
  <img src="https://img.shields.io/badge/PhotonVision-v2026.3.1-22c55e?style=flat-square" alt="PhotonVision"/>
</p>

---

## Browser tools

Eight single-file tools served from this site. Click and use — no clone, no install.

<style>
.hero-tools {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(220px, 1fr));
  gap: 14px;
  margin: 18px 0 28px;
}
.hero-tool {
  display: flex; flex-direction: column;
  padding: 18px 18px 16px;
  border: 1px solid rgba(255,255,255,0.08);
  border-radius: 12px;
  text-decoration: none !important;
  color: inherit;
  background: linear-gradient(180deg, rgba(255,255,255,0.04), rgba(255,255,255,0.01));
  position: relative; overflow: hidden;
  transition: transform 0.18s ease, border-color 0.18s ease, box-shadow 0.18s ease;
}
.hero-tool::before {
  content: ""; position: absolute; inset: 0;
  background: radial-gradient(300px 160px at 0% 0%, rgba(233,69,96,0.10), transparent 60%);
  opacity: 0; transition: opacity 0.2s ease;
  pointer-events: none;
}
.hero-tool:hover {
  transform: translateY(-3px);
  border-color: rgba(233,69,96,0.55);
  box-shadow: 0 14px 30px rgba(0,0,0,0.45);
}
.hero-tool:hover::before { opacity: 1; }
.hero-tool .icon {
  font-size: 22px;
  width: 40px; height: 40px;
  display: flex; align-items: center; justify-content: center;
  border-radius: 10px;
  background: rgba(233,69,96,0.12);
  border: 1px solid rgba(233,69,96,0.20);
  margin-bottom: 10px;
}
.hero-tool .name {
  font-weight: 700; font-size: 15px;
  display: block; margin-bottom: 4px; color: #e94560;
  letter-spacing: -0.01em;
}
.hero-tool .desc {
  font-size: 12px; line-height: 1.5;
  color: var(--body-text-color, #aaa);
}
</style>

<div class="hero-tools">
  <a class="hero-tool" href="tools/builder/"><span class="icon">🛠️</span><span class="name">Builder</span><span class="desc">Form generates ready-to-paste Java for any mechanism.</span></a>
  <a class="hero-tool" href="tools/tuner/"><span class="icon">🎚</span><span class="name">Tuner</span><span class="desc">Live PID + Motion Magic over NT4.</span></a>
  <a class="hero-tool" href="tools/health/"><span class="icon">🩺</span><span class="name">Health Dashboard</span><span class="desc">Every <code>HealthCheck</code> on the robot, live.</span></a>
  <a class="hero-tool" href="tools/motion/"><span class="icon">📈</span><span class="name">Motion Profile</span><span class="desc">Trapezoid + S-curve sketcher with copyable constants.</span></a>
  <a class="hero-tool" href="tools/pid/"><span class="icon">🎯</span><span class="name">PID Step Response</span><span class="desc">Dial gains, watch the simulated response.</span></a>
  <a class="hero-tool" href="tools/motors/"><span class="icon">⚡</span><span class="name">MotorType Browser</span><span class="desc">Every motor preset + gear-ratio calculator.</span></a>
  <a class="hero-tool" href="tools/canids/"><span class="icon">🔌</span><span class="name">CAN ID Planner</span><span class="desc">Catch CAN ID collisions before crimping.</span></a>
  <a class="hero-tool" href="tools/auto/"><span class="icon">🧭</span><span class="name">Auto Builder</span><span class="desc">Generate a behavior-framework auto.</span></a>
</div>

---

## Why

Every season teams rebuild the same elevator, arm, intake, and swerve scaffolding. Catalyst replaces it with one builder call:

```java
LinearMechanism elevator = new LinearMechanism(
    LinearMechanism.Config.builder()
        .name("Elevator").motor(13).follower(14, true)
        .motorType(MotorType.KRAKEN_X60)
        .gearRatio(10.0).drumRadius(0.0254).mass(5.0)
        .pid(50, 0, 0.5).gravityGain(0.35)
        .motionMagic(2.0, 4.0, 20.0)
        .position("STOW", 0.0).position("HIGH", 1.1)
        .build()
);
```

Motion Magic, gravity FF, sim, telemetry, command factories, health monitoring, live tuning — all wired in.

| | Raw WPILib + Phoenix | Catalyst |
|---|---|---|
| Elevator with gravity FF | ~150 lines | **8 lines** |
| Swerve + PathPlanner + Vision | ~400 lines | **15 lines** |
| Sim + telemetry per mechanism | you write it | included |
| Temperature cutoff | manual | automatic |
| Limit-switch auto-zero | manual | one call |

---

## What's in the box

### Mechanisms

| Mechanism | Use Case | Key Features |
|-----------|----------|--------------|
| **LinearMechanism** | Elevators, slides | Position control, gravity FF, limit switches, multi-follower |
| **RotationalMechanism** | Arms, wrists, turrets | Cosine gravity, hard stops, Motion Magic, multi-follower |
| **FlywheelMechanism** | Shooters | Dual motor + per-shaft followers, velocity PID, at-speed trigger |
| **RollerMechanism** | Intakes, conveyors | Stall detection, beam break, auto-stop |
| **WinchMechanism** | Climbers | Extend/retract limits, position tracking, dual-arm support |
| **ClawMechanism** | Motor-driven grippers | Stall detection, beam break, multi-follower, passive-hold |
| **DifferentialWristMechanism** | Diffy wrists (pitch + roll) | **Phoenix-6 native differential control**, separate Slot 0 / Slot 1 tuning |
| **PneumaticMechanism** | Solenoids / pistons | Double or single solenoid, optional pressure-gating, pulse / toggle commands |
| **SuperstructureCoordinator** | Multi-mechanism | State machine with safe transitions |

### Subsystems

- **SwerveSubsystem** — wraps the TunerX-generated drivetrain. Heading lock, point-at-target, skew correction, PathPlanner.
- **VisionSubsystem** — multi-camera with Kalman innovation tracking, high-speed rejection, heading-divergence filtering.
- **LEDSubsystem** — 14 patterns (fire, gradients, Larson scanner, alignment indicator).

### Advanced

| | |
|---|---|
| `StateSpaceController` | LQR + Kalman for optimal mechanism control |
| `MotionConstraintCalculator` | Physics-derived velocity / accel from motor specs |
| `SignalProcessor` | EMA, median, low-pass, composite filters |
| `PoseHistory` | Timestamped pose ring buffer with interpolation |
| `DynamicAutoBuilder` | Runtime path generation via PathPlanner |
| `TunableNumber` | Dashboard-editable constants for live PID tuning |
| `AutoSelector` | PathPlanner auto chooser with safe fallbacks |
| `GamePieceTracker` | Multi-stage piece state machine with Triggers |
| Skew correction | Pose-exponential discretization for swerve |
| Collision zones | Prevent physical mechanism collisions |
| `SimDashboard` (v1.1.0+) | Generic browser sim cockpit that adapts to any mechanism via `describe()` / `MechanismView`; sim-only, dependency-free |

### Health and safety (v0.3.3+)

| | |
|---|---|
| `HealthCheck` / `HealthMonitor` | Debounced INFO / WARN / ERROR layer; every motor gets OverCurrent / HighTemp / OverTemp by default |
| `HealthHistory` | Ring buffer of recent transitions; published for the dashboard timeline |
| `RobotSafety` | Optional watchdog with `trippedTrigger()` for one-line bindings |
| `MotorType` | NEO / Vortex / 550 / Minion / Kraken (including X44 and corrected FOC torques), plus a public constructor for custom motors |

### Driver & robot state (v0.4.0+)

| | |
|---|---|
| `DriverProfile` | Per-driver deadband + curve + speed cap + slow mode |
| `RumbleEvents` | Bind any Trigger to a controller rumble pattern |
| `RobotState` | One singleton wrapping alliance / match time / mode / battery + ready-to-bind triggers |
| `LimelightTriggers` | `Trigger.tagInView(7)`, `.detectorClass("note")`, `.horizontalErrorBelow(2.0)` |
| SysId on every motor | `mechanism.sysIdQuasistatic(Direction)` / `.sysIdDynamic(Direction)` work out of the box |
| `SwerveSetpointGenerator` | Chassis-aware accel + skid clamp for the common driver-skid case |

### Every mechanism gets

Builder config, Motion Magic or ProfiledPID, named position presets (`goTo("STOW")`),
WPILib sim, NetworkTables telemetry, temperature cutoff, limit-switch auto-zero,
HealthCheck-based fault monitoring, multi-follower support, pre-built commands,
**SysId quasistatic / dynamic routines (v0.4.0+)**,
**a self-describing `describe()` / `MechanismView` snapshot driveable in the built-in `SimDashboard` (v1.1.0+)**.

---

## Documentation

| | |
|---|---|
| [Installation](getting-started/installation) | Add Catalyst to your `build.gradle` |
| [Quick Start](getting-started/quickstart) | First mechanism in five minutes |
| [Mechanisms](mechanisms/) | All nine mechanism types |
| [Subsystems](subsystems/) | Swerve, Vision, LEDs, LimelightTriggers, SwerveSetpointGenerator |
| [Driver](driver/) | DriverProfile, RumbleEvents, controller feel |
| [Utilities](utilities/) | Health Kit, RobotSafety, RobotState, MotorType, CANRegistry, feedforward, profiles |
| [Advanced](advanced/) | Behavior framework, turret + SOTF, state-space, live tuning, health, SysId |
| [Tools](tools/) | The eight browser tools — incl. Auto Builder |
| [Examples](examples/) | Whole-robot examples |
| [Testing](testing/) | Unit-testing Catalyst-based code |

---

## Compatibility

| Component | Version |
|-----------|---------|
| WPILib | 2026.2.1 |
| CTRE Phoenix 6 | 26.1.1 |
| PhotonVision | v2026.3.1 |
| PathPlanner | 2026.1.2 |
| Java | 17+ |
