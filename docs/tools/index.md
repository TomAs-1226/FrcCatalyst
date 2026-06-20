---
layout: default
title: Tools
nav_order: 8
has_children: false
---

# Tools
{: .no_toc }

Eight single-file browser tools, hosted right here. Nothing to install.
{: .fs-6 .fw-300 }

<style>
.tool-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
  gap: 16px;
  margin: 22px 0 30px;
}
.tool-card {
  display: flex; flex-direction: column;
  padding: 22px 22px 18px;
  border: 1px solid rgba(255,255,255,0.08);
  border-radius: 14px;
  text-decoration: none !important;
  color: inherit;
  background:
    linear-gradient(180deg, rgba(255,255,255,0.04), rgba(255,255,255,0.01));
  position: relative; overflow: hidden;
  transition: transform 0.18s ease, border-color 0.18s ease, box-shadow 0.18s ease;
}
.tool-card::before {
  content: ""; position: absolute; inset: 0;
  background: radial-gradient(400px 200px at 0% 0%, rgba(233,69,96,0.10), transparent 60%);
  opacity: 0; transition: opacity 0.2s ease;
  pointer-events: none;
}
.tool-card:hover {
  transform: translateY(-3px);
  border-color: rgba(233,69,96,0.6);
  box-shadow: 0 16px 36px rgba(0,0,0,0.45);
}
.tool-card:hover::before { opacity: 1; }
.tool-card .icon {
  font-size: 28px;
  width: 48px; height: 48px;
  display: flex; align-items: center; justify-content: center;
  border-radius: 12px;
  background: rgba(233,69,96,0.12);
  border: 1px solid rgba(233,69,96,0.20);
  margin-bottom: 12px;
}
.tool-card .name {
  font-weight: 700; font-size: 17px;
  margin: 0 0 6px; color: #e94560;
  letter-spacing: -0.01em;
}
.tool-card .desc {
  font-size: 13px; line-height: 1.55;
  color: var(--body-text-color, #aaa);
  flex: 1;
}
.tool-card .tag {
  display: inline-block; margin-top: 14px;
  font-size: 10px; font-weight: 700;
  text-transform: uppercase; letter-spacing: 1.2px;
  padding: 3px 10px; border-radius: 999px;
}
.tag.live { background: rgba(74,222,128,0.16); color: #4ade80; }
.tag.gen  { background: rgba(233,69,96,0.16); color: #f0648a; }
.tag.calc { background: rgba(96,165,250,0.16); color: #60a5fa; }
.tag.plan { background: rgba(251,191,36,0.16); color: #fbbf24; }
</style>

<div class="tool-grid">

<a class="tool-card" href="builder/">
  <div class="icon">🛠️</div>
  <p class="name">Builder</p>
  <p class="desc">One-click presets for elevator, arm, shooter, intake, climber, claw, diffy wrist, piston. Persistence, <code>.java</code> download, full-subsystem-class mode, snippet import.</p>
  <span class="tag gen">Generator</span>
</a>

<a class="tool-card" href="tuner/">
  <div class="icon">🎚</div>
  <p class="name">Tuner</p>
  <p class="desc">Live PID and Motion Magic over NT4. Drag a slider, the robot reacts. Export as Java or JSON when the tune feels right.</p>
  <span class="tag live">Robot-connected</span>
</a>

<a class="tool-card" href="health/">
  <div class="icon">🩺</div>
  <p class="name">Health Dashboard</p>
  <p class="desc">Live view of every <code>HealthCheck</code> on the robot. Severity filters, search, plain-text report download.</p>
  <span class="tag live">Robot-connected</span>
</a>

<a class="tool-card" href="motion/">
  <div class="icon">📈</div>
  <p class="name">Motion Profile</p>
  <p class="desc">Analytic trapezoid and 7-segment S-curve solvers. Drag the sliders, get the position / velocity / acceleration curves and a ready-to-paste <code>.motionMagic(…)</code> line.</p>
  <span class="tag calc">Calculator</span>
</a>

<a class="tool-card" href="pid/">
  <div class="icon">🎯</div>
  <p class="name">PID Step Response</p>
  <p class="desc">Elevator, arm, or flywheel. Dial PID + feedforward, watch the closed-loop response. Rise time, settling, overshoot reported live.</p>
  <span class="tag calc">Calculator</span>
</a>

<a class="tool-card" href="motors/">
  <div class="icon">⚡</div>
  <p class="name">MotorType Browser</p>
  <p class="desc">Every <code>MotorType</code> preset (Kraken, Falcon, NEO, Vortex, 550, Minion) in one sortable table. Built-in gear-ratio calculator.</p>
  <span class="tag calc">Calculator</span>
</a>

<a class="tool-card" href="canids/">
  <div class="icon">🔌</div>
  <p class="name">CAN ID Planner</p>
  <p class="desc">Add devices, catch ID collisions, export a <code>CANIds.java</code> that pre-registers everything with <code>CANRegistry</code>. Presets for swerve and shooters.</p>
  <span class="tag plan">Planner</span>
</a>

<a class="tool-card" href="auto/">
  <div class="icon">🧭</div>
  <p class="name">Auto Builder</p>
  <p class="desc">Generate a behavior-framework auto — resilient <code>BehaviorEngine</code> sequences or a utility <code>Strategist</code>. Path-following stays with PathPlanner / Choreo.</p>
  <span class="tag gen">Generator</span>
</a>

<a class="tool-card" href="aiming/">
  <div class="icon">🎯</div>
  <p class="name">Shoot-On-The-Fly</p>
  <p class="desc">Drag the robot, set a velocity, and watch the <strong>virtual goal</strong>, lead, turret bearing and feedforward rate — the exact <code>AimingSolver</code> math. Copies the <code>track(…)</code> wiring.</p>
  <span class="tag calc">Visualizer</span>
</a>

</div>

---

## How they work

The live tools (Tuner, Health Dashboard) connect to your robot's NT4
WebSocket on port `5810`. Calculators (Builder, Motion Profile, PID,
MotorType, CAN ID) run entirely in the browser — no robot needed.

Nothing is uploaded anywhere. The pages pull msgpack from a public CDN
once and that's it.

---

## AdvantageScope tab bundles

Eight portable JSON bundles describing the NT field paths each
mechanism publishes, plus a recommended chart layout. Drop one into
AdvantageScope, Elastic, Glass, or Shuffleboard — the schema is
dashboard-agnostic.

| Mechanism | File |
|---|---|
| LinearMechanism | [linear.json](advantagescope/linear.json) |
| RotationalMechanism | [rotational.json](advantagescope/rotational.json) |
| FlywheelMechanism | [flywheel.json](advantagescope/flywheel.json) |
| RollerMechanism | [roller.json](advantagescope/roller.json) |
| WinchMechanism | [winch.json](advantagescope/winch.json) |
| ClawMechanism | [claw.json](advantagescope/claw.json) |
| DifferentialWristMechanism | [diffwrist.json](advantagescope/diffwrist.json) |
| PneumaticMechanism | [pneumatic.json](advantagescope/pneumatic.json) |

Replace `{MECH}` with the name from your Config builder (`Arm`, `Elevator`,
`Shooter`, …) and drag the listed NT keys onto the matching axes.
Schema: [`catalyst-tab-bundle.schema.json`](catalyst-tab-bundle.schema.json).
