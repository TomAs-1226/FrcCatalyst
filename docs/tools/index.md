---
layout: default
title: Tools
nav_order: 8
has_children: false
---

# Tools
{: .no_toc }

Thirteen single-file browser tools, hosted right here. Nothing to install.
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
  padding: var(--cat-space-5);
  border: 1px solid var(--cat-line);
  border-radius: var(--cat-r-card);
  text-decoration: none !important;
  color: inherit;
  background: linear-gradient(180deg, var(--cat-surface-2) 0%, var(--cat-surface-1) 58%);
  box-shadow: var(--cat-lift);
  position: relative; overflow: hidden;
  transition: transform var(--cat-dur-hold) var(--cat-ease-hold),
              border-color var(--cat-dur-effect) var(--cat-ease-effect),
              box-shadow var(--cat-dur-effect) var(--cat-ease-effect);
}
/* The key light, in the corner the identity's light comes from. It is the only thing that arrives
   on hover: a card that lifts, tints and glows at once is three answers to one gesture. */
.tool-card::before {
  content: ""; position: absolute; inset: 0;
  background: var(--cat-key-light);
  opacity: 0; transition: opacity var(--cat-dur-effect) var(--cat-ease-effect);
  pointer-events: none;
}
.tool-card:hover {
  transform: translateY(-3px);
  border-color: var(--cat-signal-line);
  box-shadow: var(--cat-lift-strong);
}
.tool-card:hover::before { opacity: 1; }
.tool-card .icon {
  font-size: 28px;
  width: 48px; height: 48px;
  display: flex; align-items: center; justify-content: center;
  border-radius: var(--cat-r-chip);
  background: var(--cat-signal-tint);
  border: 1px solid var(--cat-signal-line);
  margin-bottom: var(--cat-space-3);
}
.tool-card .name {
  font-weight: var(--cat-w-semi); font-size: var(--cat-fs-18);
  margin: 0 0 6px; color: var(--cat-ink-strong);
  letter-spacing: var(--cat-track-tight);
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
.tag.live { background: var(--cat-ok-tint); color: var(--cat-ok); }
.tag.gen  { background: var(--cat-signal-tint-2); color: var(--cat-signal-ink); }
.tag.calc { background: var(--cat-info-tint); color: var(--cat-info); }
.tag.plan { background: var(--cat-warn-tint); color: var(--cat-warn); }
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

<a class="tool-card" href="wiring/">
  <div class="icon">⚡</div>
  <p class="name">Wiring Diagram</p>
  <p class="desc">Imports your CAN ID Planner and generates a complete, optimal power tree + CAN bus diagram — breaker sizes, wire gauge, channel schedule, and termination. Printable for the build team.</p>
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

<a class="tool-card" href="statemachine/">
  <div class="icon">🔀</div>
  <p class="name">State Machine Visualizer</p>
  <p class="desc">Paste the <code>Graph/Dot</code> string — or the <code>Graph/States</code> and <code>Graph/Edges</code> arrays — your <code>Superstructure</code> logs, and see the legal-transition graph drawn. Guards are dashed; dead-ends and unreachable states are flagged so a missing edge is obvious.</p>
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

Nine portable JSON bundles describing the NT field paths each
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
| Superstructure (state machine) | [statemachine.json](advantagescope/statemachine.json) |

Replace `{MECH}` with the name from your Config builder (`Arm`, `Elevator`,
`Shooter`, …) and drag the listed NT keys onto the matching axes.
Schema: [`catalyst-tab-bundle.schema.json`](catalyst-tab-bundle.schema.json).

The state-machine bundle uses two placeholders instead of one. `{SM}` is the
log prefix you gave `Superstructure.builder(…)` — it defaults to the machine
name, so `Superstructure` unless you called `.logPrefix(…)`. `{BINDING}` is a
binding key from `b.bind("elevator", …)`; the two per-binding tabs are meant to
be copied once per mechanism you want to watch, since `Error` and `Tolerance`
are in that mechanism's own units and do not share an axis.

Ten tabs, and the first one is the one to import if you only import one: the
State timeline puts `State`, `StateConfirmed`, `Phase`, `Target` and `Blocker`
side by side, which is enough to see a transition stall and read off the name
of the mechanism that stalled it.

- [Motor History](history/) — every motor's hours, peaks and past names, pulled off the Systemcore and saved as JSON or CSV
- [Autonomy 2.0 Planner](autonomy/) — assemble tasks and power claims, see what the arbiter will actually run, and take the generated Catalyst
