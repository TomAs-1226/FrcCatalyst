# Changelog

All notable changes to FrcCatalyst are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [1.10.0] — 2026-08-06 — The robot says who it is

A dashboard could read everything a Catalyst robot was doing and nothing about what it was. No name,
no team number, no drivetrain geometry, not even which build of the library was running — the version
lived in `build.gradle` and never reached the artifact.

### Added

- **`RobotIdentity`** — a spec sheet published under `/Catalyst/Robot/`, from one line:

  ```java
  RobotIdentity.declare("Ratchet");
  ```

  That publishes the team number, the season, which roboRIO this is with its serial and image, the
  Catalyst and WPILib versions, the brownout threshold, the CAN inventory by type, the gyro, and —
  as soon as a `SwerveSubsystem` exists — drivetrain type, module count and positions, track width,
  wheelbase, odometry rate, CAN FD, and top speed. Nothing on that list is a parameter, because a
  parameter can drift: regear the drive, forget to edit the identity block, and the robot publishes
  last month's numbers with this month's confidence.

  `RobotIdentity.named(...)` takes what nothing can measure — frame perimeter, bumper thickness,
  height, your own code version, what each power distribution channel feeds, and the Tuner X module
  constants. Those last are worth passing because they are the only route to the gear ratios:
  Phoenix builds a `SwerveDrivetrain` from them and does not hand them back, so a constructed
  drivetrain can be asked its module positions but not its reduction. Passing the same objects the
  drivetrain was built from cannot fall out of step with it, which typing the ratios in would.

  **A fact Catalyst does not know is absent from the wire** — not zero, not `-1`, not an empty
  string. A robot with no swerve publishes no drivetrain group; a project with no PathPlanner
  settings publishes no mass; a diamond module layout publishes its module positions and no track
  width, because `2*max|y|` by `2*max|x|` would describe a rectangle those modules do not sit on.
  `SpecSheet` is what enforces it: every setter takes an `Optional`, and there is deliberately no
  overload that accepts a bare `double`, so writing the placeholder takes more effort than writing
  the truth rather than less.

  Written once at boot, and that is enough. An NT4 server keeps the last value of every topic and
  sends it to each subscriber the moment it subscribes, so a dashboard connecting mid-match or
  reconnecting after the radio drops receives the sheet without the robot repeating itself. Nothing
  is marked persistent, and that is a decision rather than an omission: a persistent topic is saved
  to the rio and republished by the *next* boot, so a build that threw before declaring itself would
  serve last week's mass with a fresh timestamp and no way to tell.

- **`CatalystVersion`** — which build of Catalyst is actually running.
  `CatalystVersion.describe()` returns `1.10.0 (6e02513, dirty)`. The build generates the constants
  into a source file rather than stamping the jar manifest, because a robot project shades Catalyst
  into its `FRCUserProgram` fat jar, manifests merge there, and
  `Package.getImplementationVersion()` came back null on exactly the machine a team wants the number
  on. A build made from a source archive with no repository in it reports no git stamp at all.

### Changed

- The hardware inventory merges two sources, because neither was complete. `CANRegistry` holds what
  the mechanisms claimed as they were built and misses the drivetrain entirely — Phoenix constructs
  the module motors and encoders inside `SwerveDrivetrain` and nothing tells the registry — so a
  registry-only count reported a swerve robot's twelve largest motors as none. Devices are merged on
  `(bus, id)`, and the rio bus spelled `rio` by Phoenix and `""` by the registry is the same bus.
- `CatalystGyro` claims its CAN id like every motor does. A Pigeon sharing an id with a TalonFX was a
  wiring fault that went unreported because nothing registered the gyro.

### Known gaps

- Phoenix, PathPlanner and PhotonVision versions are not published. They are pinned in Catalyst's
  own `build.gradle` and nowhere else — none of the three exposes a runtime constant — and a robot
  resolves its own copies. Reporting the pins would be reporting what Catalyst was compiled against
  as though it were what is running.
- The robot code's own version and build stamp are declared, not derived. GradleRIO writes no deploy
  metadata, so there is nothing to read; wire `robotCodeVersion(...)` to a constant your build
  generates rather than typing a string that is wrong the first time anyone forgets to edit it.

Verified against a running simulation: 37 keys on the wire, identical on a fresh connect and on a
reconnect by a brand new client, with the drivetrain and mass groups genuinely absent on a robot that
has neither. 427 tests.

---

## [1.9.2] — 2026-08-06 — Registration and contact response

Two things you could see on screen: the CAD and the collision geometry did not line up, and the robot
buzzed against walls like a game engine with a bad solver.

### Fixed

- **The CAD was drawn 0.825&nbsp;m off the collision map.** The console placed the model by centring
  its bounding box, and that box includes the **scoring table** — a 240 x 30 x 36&nbsp;in block
  standing off the +y wall with nothing matching it on -y. It dragged the centre by 0.8376&nbsp;m, so
  a robot stopped against an invisible barrier 0.8&nbsp;m short of the wall it appeared to be touching.
  The extractor now emits `modelToField` — where field (0,0,carpet) sits in model space — and the
  console places the CAD with it. Residual is 2.6&nbsp;mm on both axes, about 5% of one cell.

  The CAD's own origin was the field centre all along, and the flood fill finds it to within
  2.6&nbsp;mm. Measuring a bounding box is what threw the right answer away.

- **The robot oscillated against geometry and visibly sat inside it.** Driving into a wall with the
  throttle held, the velocity changed sign 58 times over 136 steps and the pose buzzed in a
  33.8&nbsp;mm band; recovering from a 20&nbsp;cm overlap took 7 timesteps of visible clipping. Two
  causes: {@code FieldHeightmap} returned a fixed one-cell penetration — over-correcting 45x on a
  1&nbsp;mm contact and under-correcting to a quarter on a 12&nbsp;cm one — and a single timestep
  could apply up to three impulses.

  Penetration is measured now, and exactly one impulse is applied per step with position correction
  separated from it. Exit speed matches the pair restitution to five decimal places, which is the
  measurement that proves a single impulse: two would give e squared.

- **Pose mapping used the tile's configured field size while the map declared its own.** 16.54 x 8.07
  against 16.55 x 8.05 — a further 5&nbsp;mm and 10&nbsp;mm. Poses map through the loaded map's
  dimensions now, with the configured size kept as the no-map fallback.

398 tests.

---

## [1.9.1] — 2026-08-06 — Ramps, trenches and loose fuel

Three things the field collision got wrong, all found by driving it rather than reading it.

### Fixed

- **Ramps were unclimbable because the ramp's own deck was recorded as a ceiling.** The clearance
  layer logged any surface more than 60&nbsp;mm above the carpet as something overhead, with no test
  of which way that surface faced. A ramp deck is above the carpet band, so the deck — and in 14 of
  16 sampled cells, specifically *the blue gaff tape stuck to it* — was written into the map as a bar
  7&nbsp;cm off the floor. The robot climbed 63&nbsp;mm of a 165&nbsp;mm ramp and was told it would
  not fit. Only a downward-facing triangle can be a ceiling now, and edges never write one, because an
  edge carries no facing and a wall's silhouette is a side face rather than an underside.

  Worth recording that the height layer was never at fault: the ramps were always a clean gradient
  with a 26&nbsp;mm worst step against a 60&nbsp;mm limit. The earlier measurement that the field
  needed a 170&nbsp;mm step tolerance was this same bug re-projected, not evidence about the terrain.

- **Loose fuel was baked into the terrain.** The game piece is one 150&nbsp;mm sphere instanced 456
  times; 360 of them are heaped across the centre of the field, which rasterised into a solid slab
  roughly 2&nbsp;m by 6.9&nbsp;m standing 130-155&nbsp;mm proud of the carpet. No robot could cross
  the middle of the field. Loose game pieces are excluded from the heightmap — they are handled by
  {@code SimulatedGamePiece}, which is where contact with them belongs.

- **Clearance now means what it says.** Before this, of the cells carrying a clearance value, zero had
  clearance above their own floor and 9021 had it below: every entry was a ground surface mislabelled
  as a roof. The map now carries 1820 genuine drive-under cells.

### Changed

- The example robot is 0.52&nbsp;m tall rather than 0.85&nbsp;m. The trench rails in the field CAD have
  their underside at 565&nbsp;mm, so an 850&nbsp;mm robot could never use the trench and the map looked
  broken when it was telling the truth. Teams that intend to run the trench build to fit under it.

Verified by driving the simulation the length of a perimeter lane, under a trench rail and back across
both ramps. 388 tests pass.

---

## [1.9.0] — 2026-08-05 — Field collision from the CAD

Collision geometry stopped being something somebody typed in and started being something read off the
season's field model. Hand-placed boxes never match: you measure the hub, forget the trench,
approximate the ramp as a wall, and the simulation quietly disagrees with the field it represents.

### Added

- **`FieldHeightmap`** — loads a heightmap generated from the season CAD by `npm run field-collision`
  in the Catalyst Console repo, and answers whether a robot of a given size can stand somewhere and
  which way is out if not. Two grids: **height** is the highest surface per cell, **clearance** is the
  underside of the lowest thing overhead. Together they resolve carpet, ramps, trenches and walls in
  one loop.

  The choice that makes ramps work: *blocked* is decided by a **step**, not a height. A robot at the
  top of a ramp is two metres above the carpet and perfectly happy; a robot against a six-centimetre
  lip is stuck. Every sampled cell is compared against the ground under the robot's centre.

  The clearance grid is what makes trenches work, and a height-only model gets them backwards: an FRC
  trench is not dug into the floor, it is carpet with a bar above it. Max-height sees the bar, calls
  the cell solid, and refuses passage. Now a short robot goes through and a tall one does not.

- **`SimulatedRobot.Builder.heightmap(...)`** — drive against it. Takes precedence over
  `collisionField`, and `groundHeightMeters()` reports how far up a ramp the robot has ridden.

### Changed

- The example's cockpit serves the same heightmap at `/collision` and draws the field from it, so
  what you see and what you hit are one source instead of two drawings that drift apart.
- Example compilation pinned to UTF-8; javac was reading the cockpit's em-dashes as cp1252.

### Known gaps

- Clearance counts geometry standing more than 6 cm clear of the carpet. On the current model that
  finds 2379 low-overhead cells; whether REBUILT has a true drive-under trench is unconfirmed against
  the field drawings.
- Penetration depth is one cell rather than measured, because a heightmap does not carry one.
  Resolution relies on repeated passes converging.

381 tests, all HAL-free.

---

## [1.8.0] — 2026-08-05 — Contact physics, and a driver station console

Simulated robots used to drive through walls. Game pieces did not exist. Both are now solid, and what
happens when they meet depends on what they are made of.

This release also has a companion application.

### Catalyst Console

**[Catalyst Console](https://github.com/TomAs-1226/CatalystConsole)** is a driver station dashboard for
teams running Catalyst. It sits next to the NI Driver Station and shows what the robot is doing: live
telemetry, alerts straight from `AlertManager`, live tuning of whatever the robot declares tunable,
Physics Core state, the REBUILT field in 3D with your robot on it, and Driver Station logs laid out so
they can actually be read.

It ships inside the **Catalyst desktop app** — install that and the console is there, with a *Driver
Console* entry that launches it. It is also available on its own.

Three rules shape it, and they are worth stating because they constrain what it will ever do:

- **It never controls the robot.** FRC requires the official NI Driver Station and only one DS may hold
  the robot connection. There is no code path in the console that opens that socket.
- **Nothing it does may impede driving.** No modal blocks the dashboard, no check gates anything. Every
  failure degrades to a dimmed number and a quiet chip in a corner.
- **It never invents a number.** An unpublished topic shows a dash. The `.dslog` parser refuses
  versions it does not recognise rather than decoding them into nonsense.

Most of what it displays, Catalyst already publishes — alerts, loop time, Physics Core state — so for
most teams there is nothing to wire up. The full contract, including the tunable manifest schema, is in
the console's README.

### Added — `frc.lib.catalyst.physics.contact`

- **`ContactMaterial`** — restitution, friction and rolling resistance, with presets for carpet,
  polycarbonate, aluminium, foam game pieces, bumpers and tread. Pairs combine by geometric mean, the
  rule Box2D and Bullet use. The point is that the same foam ball dies on carpet and comes back off the
  wall; model both with one global bounciness and you get one of them wrong, usually the one that
  matters.
- **`ContactResolver`** — impulse resolution. The normal impulse sets the bounce; the tangential
  impulse is clamped to the Coulomb cone, so a glancing hit differs by surface rather than only in how
  high it bounces. Two details carry more weight than they look: restitution is suppressed below a
  resting speed, or a ball on carpet bounces microscopically forever, and separating contacts are
  skipped, or resolving them glues bodies together.
- **`CollisionField`** — the solid parts of the field: a rectangle plus static obstacles, each with its
  own material. The robot is treated as an oriented box, because a robot at 45° needs noticeably more
  room than one square to the wall. Obstacle tests use the separating axis theorem and report the axis
  of least overlap, which is the shortest way out and therefore the contact normal.
- **`SimulatedGamePiece`** — a sphere with gravity, drag, floor and perimeter contact, rolling
  resistance, and a bumper interaction that pushes out along the shallowest face, so a corner and a
  flat shove a ball in visibly different directions.
- **`SimulatedRobot.Builder.collisionField(...)`** — makes the field solid. Opt-in, so every simulation
  written before this behaves exactly as it did. The velocity change is deliberately left out of the
  wheel velocity: a swerve wheel measures rolling along its own axis, so a robot stopped dead by a wall
  has encoders still claiming forward motion, and that gap is precisely what `DisturbanceEstimator`
  decomposes. The collision is only detectable because the simulation reproduces it.
- **`SimulatedRobot.Builder.startingPose(...)`** — where the robot starts and where `reset()` returns
  it. The origin was fine when nothing was solid; it is the inside of the corner once it is.

### Testing

- **`ContactTest`**, **`RobotCollisionTest`**, **`AutoPathPhysicsTest`** — 44 new tests. The last is a
  test autonomous routine driven through the simulator against ground truth: it accelerates, turns
  while translating, then stops, and asserts distance covered, that rotation follows the commanded
  sign, repeatability, that Physics Core tracks true speed within 0.75 m/s and its path within 1.5 m,
  and that a game piece in the way ends up somewhere legal. A path that only drives straight would not
  catch a rotation sign error, which is the mistake that actually ruins autos.

369 tests, all HAL-free.

### Fixed

- **`LoopMonitor` churned its over-budget alert.** The edge-triggering was right but there was no
  hysteresis, so a robot averaging exactly its budget — the common case, because that is what the loop
  is tuned to — crossed the threshold in both directions every few loops and flooded the Driver Station
  console with the same line. It now clears only once the average falls back to 90% of budget.

---

## [1.7.1] — 2026-08-05

Found while wiring [Catalyst Console](https://github.com/TomAs-1226/CatalystConsole), a driver station
companion dashboard, to a real robot: the telemetry it was written against was not at the path the docs
said it was.

### Fixed

- **Physics Core and `LoopMonitor` telemetry was landing one table too deep.** `CatalystLog` keys are
  relative — the sink supplies the `Catalyst` root table, which is why every other caller writes
  `"Vision/…"`, `"Safety/…"`, `"Health/…"`. These two prefixed it again, so everything documented as
  `Catalyst/Physics/…` and `Catalyst/Loop/…` was actually published at `/Catalyst/Catalyst/…` and no
  dashboard found it. The keys now match what the documentation has always claimed, so no docs changed
  — only the code that disagreed with them.

### Added

- **`PhysicsCore` publishes the fused pose.** It is the headline output of a state estimator and it was
  the one thing missing from the telemetry. It goes out twice on purpose: `Physics/Pose` as a `Pose2d`
  struct for AdvantageScope, and `Physics/PoseArray` as a plain `[x, y, theta]` array for the many
  dashboards that cannot read struct topics.

## [1.7.0] — 2026-08-05 — Physics Core validated in simulation

Physics Core shipped feature-complete in 1.6.0 and had never been run against a known truth. This
release builds a simulator that provides one, marks Physics Core against the RFC's acceptance criteria,
and fixes the three real defects that exercise found. It also documents how to measure the robot the
whole thing depends on.

**The headline number: fused velocity error through a slip went from 0.08% better than raw encoders to
48% better.** The first figure was what a silently-disabled fusion looks like.

### Fixed — found by the validation, not by unit tests

- **Fusion was silently disabled for anyone calling `update(PhysicsSample)` directly.** `PhysicsCore`
  inferred "this robot has no accelerometer" from whether a *builder supplier* had been configured, so
  replay, tests, and any team doing their own sampling got wheel-only estimation while supplying
  perfectly good IMU data in the sample. Availability is now decided per sample.
  `withoutAccelerometer()` still works as an explicit veto.
- **The collision detector fired every time the robot accelerated hard off the line.** It thresholded
  on the raw wheels-versus-IMU residual, and hard acceleration that breaks traction produces exactly as
  large a residual as being hit. `DisturbanceEstimator` now decomposes the residual into the part slip
  can explain and the part it cannot — slip can only ever make the robot accelerate *less* than the
  wheels claim, along the direction they are pushing, so anything outside that range is external.
  `impactEvidence()` drives collisions; `slipEvidence()` drives wheel distrust.
- **The wheel-trust floor was too high for a first-order filter.** A complementary filter settles onto
  whatever it is weighted toward, so the old 0.15 floor meant the estimate re-joined the wheels within
  about seven loops — part way through a slip, which is exactly the wrong moment. The floor is now
  0.005, bounded by a new **slip budget**: the estimate may ride the IMU for a configurable window
  (2 s by default, floor held flat for the first 60%) after which wheel trust climbs back, because
  dead reckoning that never ends is drift.
- **`MechanismModel.fixed(...)`** and two smaller fixes carried from 1.6.0 testing.

### Added — simulation validation

- **`SimulatedRobot`** (`physics.sim`). A robot that knows the truth: traction-limited dynamics, slip
  that makes the wheels over-report, odometry that drifts through it, and per-module bias for
  differential slip. Deliberately includes effects Physics Core does **not** model — wheel radius
  error, accelerometer bias and noise, encoder noise, and wheels that skid through a sideways impact
  without the encoders seeing it — because a simulator built from the estimator's own assumptions
  proves nothing. Deterministic from a seed.
- **`PhysicsValidator`** (`physics.sim`). Nine scenarios scored against the RFC's acceptance criteria,
  with the metrics behind every verdict so a near-miss is visible rather than just "fail". Run it
  against **your** `RobotModel` — the thresholds suit a typical mid-weight swerve and a robot far from
  that may need different ones.

Current results on a 55 kg reference robot: 48% less velocity error than encoders through a slip;
differential slip detected on the correct module; zero false positives in 200 loops of clean driving;
uniform slip correctly surfacing as disturbance rather than module slip; one collision event 40 ms
after impact; confidence 0.97 → 0.59 → 0.99 across a five-second vision dropout; release-state
prediction 96% better than aiming from the present.

### Added — measuring the robot

- **`ModelUncertainty`** (`physics.model`). Propagates measurement error through to the limits, in
  quadrature, and names which measurement is limiting you. It exists because the sensitivities are
  genuinely counter-intuitive: **mass does not affect the traction limit at all** — a heavier robot
  needs more force and gets proportionally more grip, and the two cancel exactly — while
  centre-of-mass height maps one-for-one onto the tipping limit and is the hardest thing to measure.
- **[Measuring Your Robot](docs/advanced/physics-measurement.md)** — a new guide. What each number
  drives, how to measure it with pit tools (tilt test and pull test for friction, tilt-and-weigh for
  centre of mass, high-speed video for release delay), the accuracy each one actually needs, a worked
  example, a checklist, and a table of what goes wrong if you get each one wrong.

The last of those is the part worth reading: the errors that make Physics Core **cautious** announce
themselves in telemetry, and the errors that make it **optimistic** — friction too high, centre of mass
too low, footprint measured from the frame rather than the contact patches — stay quiet until the robot
tips. When unsure, round toward caution.

### Changed

- **`DrivetrainModel.maxAngularAccelerationRadPerSecSq()`** and **`wheelForceFromTorque(...)`** — added
  because `RobotModel`'s `momentOfInertiaKgM2` and `wheelRadiusMeters` were stored and consumed by
  nothing. Rather than tell teams to measure values that no calculation used, they now have real uses,
  and the measurement guide says plainly that moment of inertia is worth skipping unless you need the
  angular limit.
- **`PhysicsAnalysis.disturbanceFraction()`** added, and `isDisturbed()` now consults it. Uniform slip
  leaves no per-module residual *and* pulls the fused acceleration down toward the truth once the
  estimator distrusts the wheels, so a robot sliding across the carpet could show a modest traction
  usage while the wheels and the IMU were shouting at each other.

### Testing

325 tests total, all passing, still entirely HAL-free. The validation scenarios are themselves tested
for determinism and re-run across three different noise seeds, because a single lucky seed proves
nothing.

## [1.6.0] — 2026-08-05 — Physics Core, completed

Completes the Physics Core RFC ([#33](https://github.com/TomAs-1226/FrcCatalyst/issues/33)): the
articulated mass model, closed-form ballistics, online parameter identification, fault isolation,
predictive capability evaluation, whole-robot power planning, counterfactual replay, and the opt-in
bounded-intervention layer.

**Still additive, still not intrusive, still not mandatory.** Robot code written against 1.5.0 works
identically on 1.6.0. Physics Core continues to write no pose and schedule no command, and the one new
piece that *could* change how the robot drives — `PhysicsConstraints` — has no installer and no
periodic hook: it computes limits and hands them to you. A team that never calls it gets exactly the
robot they had before.

### Added — a live centre of mass

- **`MechanismModel` / `ArticulatedRobotModel`** (`physics.model`). The robot as a transform tree of
  masses, so the centre of mass moves when the mechanisms do. Raising a 10 kg carriage 1.2 m on a
  60 kg robot moves the CoM 20 cm; a static `centerOfMassHeightMeters` cannot see that. Links compose
  through `childOf`, which also yields the **field-relative end-effector pose** placement games want
  (`fieldPoseOf`). Linear, rotational, custom, and fixed links; `build()` rejects duplicate names and
  parent cycles, and a link naming an unknown parent falls back to the chassis rather than silently
  dropping its mass.
- **`StabilityModel`**. Tipping from the **zero-moment point**: `zmp = com − (h/g)·a`, with the margin
  reported in metres, plus per-wheel normal loads and a worst-direction acceleration limit. It agrees
  exactly with `DrivetrainModel`'s closed-form `g·halfWidth/comHeight` when the mass is centred — the
  tests assert that cross-check — and diverges correctly as soon as the robot extends or carries mass
  off-centre.

### Added — ballistics, so shot gates stop guessing

- **`ProjectileModel`**. Closed-form time of flight, height at distance, apex, max range, and the two
  launch angles that reach a target, with an optional exact linear-drag model. Gives
  `LaunchState.missRadiusMeters(...)` a real flight time instead of a constant that is wrong at every
  distance but one. Quadratic drag and spin are deliberately absent — neither has a closed form, and
  fitting them needs measurements, which is what the aiming solvers' interpolating tables already are.
  Below a negligible-drag threshold the drag path falls back to the exact parabola, because the drag
  solution is a difference of two terms that both blow up like `1/k` and loses all precision there.

### Added — learning the constants the robot actually has

- **`RecursiveLeastSquares`**. Constant-time, constant-memory linear fitting with a forgetting factor.
  Convergence is judged on **mean per-parameter variance** rather than the raw covariance trace, so
  the same threshold means the same thing regardless of how many coefficients are being fitted.
- **`FeedforwardIdentifier`**. Recovers `kS`/`kV`/`kA` (and an elevator or arm gravity term) from
  normal operation. **`recommendation()` stays empty until the fit has genuinely converged** — a
  mechanism run at one speed forever cannot separate `kS` from `kV`, and no quantity of samples fixes
  that.
- **`BatteryResistanceIdentifier`**. Measures this battery's internal resistance from voltage sag —
  the number every brownout prediction rests on and nobody measures. Refuses to report under a steady
  load, where resistance is unidentifiable.
- Neither has any way to write a gain. They report; you review and apply.

### Added — diagnostics that name a cause

- **`ModelResidualMonitor`**. Separates noise from bias statistically instead of by threshold: a
  residual that averages to zero is a noisy sensor, one that sits to one side is a wrong model. Calls
  bias only when the offset beats both a tolerance and its own standard error, so a 12 cm offset
  buried in 40 cm of scatter is still caught.
- **`FaultIsolator`**. Ranks candidate causes by the residual pattern they predict, including
  `expectsQuiet` — the residual a cause should *not* disturb, which is what separates wheel slip from
  a wheel-radius calibration error. Scores are documented as **diagnostic scores, not probabilities**,
  and deliberately do not inflate with severity.
- **`JamDetector`**. Tells a jam from a successful intake the only honest way — by asking whether the
  piece is there. With no piece sensor it reports `STALLED` rather than guessing.

### Added — deciding before committing

- **`CapabilityEvaluator`**. "Is this action worth attempting, and what will it cost", answered before
  it is scheduled. Duration from an exact trapezoidal profile, position error from the estimate's own
  uncertainty compounded over that duration, tip margin from the live CoM, voltage from the power
  predictor. Anything not configured is absent rather than invented, and a robot already too fast to
  stop in the distance available is reported infeasible with the reason.
- **`PowerPredictor`**. Complements `BrownoutMonitor` rather than duplicating it: the monitor watches
  the present and backs off, this is asked about the future and sequences demands into waves that fit.
  Open-circuit voltage is inferred from the present reading, so it follows the battery down a match.

### Added — replay, injection, and the opt-in limits

- **`PhysicsConstraints`** (`physics.constraints`). Phase 3, built so it cannot engage by accident.
  Four independent limits — traction/tipping from the live CoM, confidence, slip, electrical headroom
  — with `explain()` naming whichever binds, so every recommended slowdown is attributable to a
  sentence. Conditions are exposed as plain predicates as well as WPILib `Trigger`s, which keeps the
  logic HAL-free and testable.
- **`PhysicsReplay`** (`physics.replay`). Counterfactual analysis over recorded samples: run the same
  twelve seconds against two configurations and compare. Exact and deterministic. A transform can
  remove a sensor to ask what it was worth.
- **`DisturbanceInjector`** (`physics.sim`). Injects slip and impacts on purpose, so detectors can be
  tested a hundred times in a unit test instead of once on a slick patch of carpet.

### Added — the rest of the observation contract

- **`RangeObservation`**, **`BearingObservation`**, **`ContactObservation`**, each with a residual
  helper. A bearing that is too oblique for a trustworthy 3D solve is often still a perfectly good
  angle; contact with a known wall is the most accurate absolute measurement on the field and almost
  nobody uses it.

### Changed

- **`VelocityObservation` is now genuinely fused** rather than logged. It is the one measurement that
  can settle a wheels-versus-IMU disagreement, because it is the only one that sees the robot's motion
  without going through either. Fusion is inverse-variance weighting, and the tightened variance grows
  back at a configured process noise so the benefit fades rather than being claimed forever.
  `PhysicsCore.observe(...)` now returns `false` for a velocity observation arriving before the first
  update, when there is no estimate to fuse with.
- Range, bearing, and contact observations reset the staleness clock — an independent absolute
  measurement agreeing with the estimate is exactly what staleness tracks — but are **not** fused into
  the pose, which stays out of scope.
- `MechanismModel.fixed(...)` puts the position in the link transform rather than the centre-of-mass
  offset, so `poseOf(name)` reports where the link actually is. It previously gave the right centre of
  mass and a link that appeared to sit at the robot origin.

### Documented

- The [Physics Core guide](docs/advanced/physics.md) roughly triples in length: the mass tree and its
  rotation-sign trap, the zero-moment-point derivation, ballistics scope, when to believe a parameter
  fit, fault-isolation patterns, the power planning distinction, and a seven-step adoption order.
- A **Known limits** section states plainly what it cannot do — most importantly that **uniform slip
  is invisible to per-module scoring** (all four wheels reading fast leaves no residual), so a robot
  with no accelerometer cannot detect it at all. `SlipEstimator`'s javadoc says the same.

### Testing

120 new tests (**214 physics, 299 total, all passing**), still entirely HAL-free. The parameter fits
are verified by generating data from known coefficients and checking they come back; the ballistics by
round-trip — solve for a launch angle, fly the shot, confirm it lands on the target; the stability
model by cross-checking against an independent closed-form derivation.

## [1.5.0] — 2026-08-05 — Catalyst Physics Core (Phase 1, shadow mode)

Introduces **Catalyst Physics Core**, an optional, game-independent physical-intelligence layer.
Additive and strictly advisory: it writes no pose, schedules no command, blocks no transition, and
changes no setpoint. Robot code written against 1.4.0 works identically on 1.5.0, and a team that
never configures a robot model loses nothing. Implements Phases 0 and 1 of the RFC in
[#33](https://github.com/TomAs-1226/FrcCatalyst/issues/33).

### Added

- **`frc.lib.catalyst.physics`** — the new package. `PhysicsCore` is the one object a robot project
  touches; call `update()` once per loop and ask it five questions: `state()`, `predict(h)`,
  `predictLaunchState(delay)`, `analyze()`, `health()`. Everything publishes under
  `Catalyst/Physics/...`.
  - **Fused state estimation** (`PhysicalStateEstimator`). Wheel odometry is exact at steady state
    and wrong the instant a tyre slips; integrated IMU acceleration is traction-blind but drifts. A
    weighted complementary observer blends them, with the wheels' weight falling from 0.98 to 0.15 as
    slip rises — never to zero, because an estimate running purely on integrated acceleration walks
    away within seconds. Reported acceleration is the derivative of the fused velocity, so the output
    is self-consistent at either extreme.
  - **Honest confidence** (`LocalizationQuality`). Time since the last absolute fix, wheel slip, and
    wheels-versus-IMU disagreement each subtract from confidence, capped so no single factor can zero
    it alone. Carries per-axis standard deviations, a `HIGH`/`MODERATE`/`LOW`/`LOST` band, and short
    stable text naming the dominant limit ("vision stale 4.1 s"). Every weight is documented and
    tunable.
  - **Per-module slip scoring** (`SlipEstimator`). Runs the believed chassis velocity back through
    inverse kinematics and compares each wheel against its prediction, counting only the component
    along the module's predicted direction so a module still rotating toward its setpoint is not
    mistaken for one that is slipping. `slipFactor()` (mean) drives the fusion weight;
    `peakSlip()` (max) is what to alert on. Suppressed below 0.25 m/s.
  - **Disturbance residuals and collision detection** (`DisturbanceEstimator`, `CollisionDetector`).
    The acceleration the wheels imply versus the acceleration the IMU measured. Wheels accelerate and
    the robot does not → the tyres are spinning; the robot accelerates and the wheels did not command
    it → something hit it. An event needs 70% of the traction limit for two consecutive loops, with a
    0.5 s refractory period so one collision is reported once, and reports the peak of the impact
    rather than whichever loop crossed first.
  - **Shot-release prediction** (`StatePredictor`, `LaunchStatePredictor`, `LaunchState`). A shot
    leaves ~120 ms after the trigger; at 3 m/s that is 36 cm of error before the piece is out.
    `LaunchState` exposes `pose()` and `fieldVelocity()`, so it drops into `AimingSolver` and
    `AimingSolverVector` with **no new overload and no API change**. `missRadiusMeters(flightTime)`
    compounds position and velocity uncertainty into a shot gate. The predictor integrates a
    decaying-acceleration model in closed form (a robot cannot accelerate forever), and
    `constantAcceleration()` recovers the textbook form for short horizons.
  - **Physical model** (`RobotModel`, `DrivetrainModel`). Four measurements — mass, footprint, wheel
    radius, centre-of-mass height — yield the traction limit (`mu*g`), the tipping limit
    (`g*halfWidth/comHeight`), stopping distances, and their inverse
    (`maxSpeedForStoppingDistance`). Every default is stated on the builder method that sets it.
  - **Timestamp handling** (`SignalBuffer`, `SignalSnapshot`, `TimestampSynchronizer`). Allocation-free
    ring buffers with interpolated lookup, and per-signal latency so measurements are stamped at
    capture time. Residuals computed across signals read at different instants include that skew;
    this removes it. Refuses to extrapolate outside what it holds.
  - **Observations** (`PoseObservation`, `VelocityObservation`). Timestamped, uncertainty-carrying
    evidence from outside the drivetrain. A pose observation landing more than a metre from the
    current estimate is rejected and counted — one bad frame should not convince the robot it
    teleported.
  - **Compute profiles** (`PhysicsProfile`). `MINIMAL` / `BALANCED` / `ADVANCED` / `SYSTEMCORE`, so
    behaviour is gated on a stated budget rather than a controller name. `ADVANCED` and `SYSTEMCORE`
    behave as `BALANCED` today and deliberately do not silently enable anything unvalidated.
- **`RobotStateSource` / `UncertainRobotStateSource`** — the contract for "where is the robot and how
  fast". Both `SwerveSubsystem` and `PhysicsCore` implement it, so consumers depend on neither and
  keep working whether or not a team adopts Physics Core. `UncertainRobotStateSource` adds
  `quality()` and derives a diagonal `poseCovariance()` from it by default.
- **`SwerveSubsystem.pose()` / `fieldVelocity()` / `timestampSeconds()`** — the three contract methods,
  delegating to the existing `getPose()` and `getFieldRelativeSpeeds()`. Purely additive.
- **[Physics Core guide](docs/advanced/physics.md)** — what it is for, how the fusion and the
  confidence model work, how to wire it into state-machine guards and `BehaviorEngine` preconditions,
  what deliberately did not ship, and a six-step procedure for validating it on a real robot before
  trusting it.

### Notes on scope

Phase 1 is **shadow mode**. Deliberately not in this release, and staged behind validation on a real
robot: pose fusion (your drivetrain estimator stays the single writer of `Pose2d`), dynamic
constraints, and online parameter identification. Power modelling is intentionally absent —
`BrownoutMonitor` already predicts sag from `Sum(I) * R`, and a second path would conflict with it.
Physics Core is not a rigid-body engine; there is no collision solver or contact model, and
simulation with game pieces stays with the maple-sim seam.

### Testing

94 new unit tests (179 total, all passing), every one HAL-free — no NetworkTables, no Driver Station,
no robot. Clocks are injectable and `PhysicsCore.update(PhysicsSample)` takes every measurement as an
argument, so the whole pipeline is drivable from a test or a log.

### Docs

- **[Roadmap](docs/ROADMAP.md) rewritten** ([#31](https://github.com/TomAs-1226/FrcCatalyst/issues/31),
  reported by [avrahamavraham](https://github.com/avrahamavraham)). The 2026 plan was complete — every
  Tier 1 and Tier 2 item shipped or declined — so the page had gone stale. It now closes out the old
  plan as a record, states the Physics Core phase plan with its kill criteria, and names what is
  actually next: validating Phase 1 on a robot, the QuestNav pose source, and finishing the Phase 2
  advisory integrations.

### Build

- `org.ejml:ejml-simple` added as a **test-only** runtime dependency. `SwerveDriveKinematics` builds a
  `SimpleMatrix`, which the test runtime did not otherwise resolve. No change to the published
  library's dependencies.

## [1.4.0] — 2026-07-30 — Catalyst Desktop (optional companion app)

Introduces the **optional** Catalyst desktop app. **No library API changes** — the version bump
aligns the library with the first app release; robot code written against 1.3.3 works identically on
1.4.0, and teams that never touch the app lose nothing.

### Added

- **Catalyst Desktop** — an optional, cross-platform companion app (Tauri; Windows now, macOS via CI).
  Every Catalyst browser tool in one native window; a one-click *install into your robot project* that
  drops the vendordep in from a bundled copy (works offline); offline-graceful auto-update for both the
  app and the library; and an **AI-agent connector** — a dependency-free MCP server that exposes the
  tools and a distilled knowledge graph to agents (Claude, Cursor, …). The app is entirely optional and
  ships separately from the library JAR.

## [1.3.3] — 2026-07-28 — LoopMonitor

A small, additive measurement tool for the loop-overrun problem team 3211 reported.

### Added

- **`LoopMonitor`** (`frc.lib.catalyst.util`). Call `record()` once per `robotPeriodic()` and it
  measures the real loop time, tracks the last / rolling-average / peak in milliseconds, publishes
  them under `Catalyst/Loop/<name>/...`, and raises a single `AlertManager` warning when the rolling
  average sits over the budget (20 ms by default). It alerts on the average rather than one-off
  spikes, and warns/clears only on the transition so the alert list never churns. The clock is
  injectable, so the statistics are unit-tested with no HAL, no NetworkTables, and no robot
  (`LoopMonitorTest`, 7 tests). Pairs with the "Keeping the loop under 20 ms" logging guide added in
  1.3.2: measure first, then reach for the levers.

## [1.3.2] — 2026-07-25 — Field-centric red flip & loop-cost guidance

Two fixes prompted by team 3211's competition report. Additive and backward compatible.

### Fixed

- **Field-centric drive now sets the operator perspective explicitly on the request** (reported by
  team 3211). `SwerveSubsystem.periodic()` already calls `setOperatorPerspectiveForward(...)` from
  the alliance each loop (red → `Rotation2d.k180deg`), and CTRE's `FieldCentric` request defaults
  its `ForwardPerspective` to `OperatorPerspective` — so the red-alliance flip was intended to work
  already. To make it robust and self-documenting rather than reliant on the CTRE default, the reused
  request now sets `.withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)` explicitly.
  A team whose forward direction did not flip on red should confirm they are on 1.3.2 and that their
  `SwerveSubsystem` is running (its `periodic()` is what publishes the perspective each loop).

### Docs

- **"Keeping the loop under 20 ms"** — a new section on the [logging & telemetry page](docs/advanced/logging.md)
  laying out the loop-cost levers in impact order (disable tunables at competition, profile vision,
  halve telemetry with `enableLoggingInputs(false)`, prefer a CANivore bus). Prompted by team 3211's
  loop-overrun report; the library's own per-loop cost was audited and found already optimized
  (Phoenix signal-frequency tuning, throttled health checks, change-detected tunables).

## [1.3.1] — 2026-07-24 — Audit fixes

A full adversarial audit of the library. Ten issues were raised and verified; nine are fixed here
(the tenth is reported for review, see below). Additive and backward compatible.

### Fixed

- **A throwing `zeroed()` could crash the robot loop from a rejected request** (high). Inside
  `StateMachineCore`, `describeRejection()` re-checked `Binding.zeroed()` unguarded while every other
  call on the decision path used the fail-closed `safeZeroed(...)`. A binding whose `zeroed()` threw
  (a CANcoder off the CAN bus) would throw a second time out of `request()` — documented never to
  throw — and out of `CommandScheduler.run()`. Now guarded.
- **`Action.run(Command)` could crash the robot loop on reuse** (high). The overload stores one
  command instance, and `Action.toCommand()` composes it via `until(...)`; a second `toCommand()`
  (an `Autopilot` cycle, a `Strategist` re-selection) threw "command already composed" out of the
  scheduler. `toCommand()` now fails closed to a no-op instead of throwing, and the overload is
  documented as single-use — use `run(Supplier<Command>)` for anything run more than once.
- **Disabling mid-transition then re-enabling silently dropped the transition** (medium). The state
  machine went to `HOLDING` on re-enable even with a transition still in flight, so it stopped
  advancing and the mechanisms fell back to the origin state's goals. It now resumes the transition.
- **`FlywheelBinding.atGoal` ignored the second wheel** (medium). Two dual-speed goals that shared a
  primary speed but differed in spin (backspin/topspin) were indistinguishable to `isAt`. The
  secondary setpoint is now corroborated too.
- **`FlywheelMechanism.spinUpAndWait` stopped the wheel the instant it reached speed** (medium). It
  was built on `spinUp`, whose `finallyDo` stops the motors; the `.until(atSpeed)` therefore cut the
  wheel just before a shot. It now reaches speed and ends with the velocity latched.
- **`AimingSolverVector` could emit `Infinity`/`NaN` RPS** (low). A 90-degree hood limit made the
  clamp re-derivation divide by `sin(0)`; a zero efficiency or wheel diameter divided by zero. The
  divisions are guarded and `efficiency`/`wheelDiameter` are validated at build.
- **`GhostReplay` CSV round-trip broke under comma-decimal locales** (low). `save()` now formats with
  `Locale.ROOT` to match `load()`'s `Double.parseDouble`.
- **`AlertManager`'s own javadoc example grew alerts without bound** (low). It showed a message with a
  live value; alerts de-duplicate by exact text, so that pattern never clears and grows the NT array.
  The example and contract now make clear that alert text must be invariant.
- **Corrected the `Actuator.pursueCommand` / `Superstructure` docs** (low) that claimed a build-time
  requirements-subset probe which is not performed — it is a contract the binding must honour.

### Reported, not changed

- The `advancedDrive` skew correction rotates the command by `+omega*dt/2` while its own javadoc and
  the standard pose-exponential correction say `-omega*dt/2`. The sign looks inverted, but it is a
  subtle, convention-dependent, second-order effect; it is flagged for a combined translate-plus-rotate
  simulation check rather than flipped blind.

## [1.3.0] — 2026-07-24 — Understandable state machine, servos, and live debugging

Focused on making the state machine easy to read and debug (issue
[#29](https://github.com/TomAs-1226/FrcCatalyst/issues/29)) and adding a few lean, self-contained
components. Additive and backward compatible.

### Added — understandability & debugging (issue #29)

- **`docs/advanced/statemachine-internals.md`** — a "how it works under the hood" page: why the
  package is 44 files across four layers (only `StateMachineCore` holds real logic), one 20 ms
  scheduler tick walked end to end with diagrams, the proven-arrival invariant, and a
  **symptom → log-key debug map** ("a button does nothing" → read `Blocker`/`Rejected/Last`, etc.).
- **`Superstructure.explain()` / `StateMachineCore.explain()`** — a one-call, plain-language dump of
  what you built (states, legal edges, guards, bindings) and why it is stuck right now. Print it from
  a button, a test, or a breakpoint. The direct answer to "it's impossible to debug".
- **Browser state-graph tool** (`docs/tools/statemachine/`, the 11th tool) — paste the machine's
  `Graph/Dot` or `Graph/States`+`Graph/Edges` from the log and see the graph drawn: states, legal
  edges, guards, and — flagged — dead-end and unreachable states, so "what's missing" is obvious.
  Single-file, offline, no dependencies.
- **`package-info.java` for all four state-machine subpackages** — each package now shows a short map
  of its types in Javadoc and IDE package views.

### Added — components

- **`ServoMechanism`** — a lean PWM-servo mechanism (hoods, ratchet releases, funnel flappers). Open-
  loop with the usual Catalyst ergonomics: builder with validation, named positions
  (`goTo("FAR")`), angle clamping, telemetry, and a `describe()` view. Plus a first-class state-
  machine binding (`ServoGoal` + `Mechanisms.servo(...)`), so a servo is a full superstructure member.
- **`SimDashboard.statusPanel(title, lines)`** — a live text card in the browser sim cockpit,
  decoupled from mechanisms. Feed it anything: `dash.statusPanel("Superstructure", () ->
  List.of(sm.explain().split("\n")))` shows the running state machine live in sim.

### Docs

- The `SimDashboard` example now drives a servo with a tiny state machine and shows its `explain()`
  output live, exercising all of the above under `./gradlew simulateJava`.

## [1.2.1] — 2026-07-23 — Logging plumbing, shooter + sim seams, second SOTF solver

A maintenance release folding in the open issue backlog and the ready pull requests.
Backward compatible: additive, plus one internal telemetry-routing change that preserves
every existing NetworkTables key and type.

### Added

- **Torque-current FOC velocity control** ([#20](https://github.com/TomAs-1226/FrcCatalyst/pull/20),
  closes the last [#17](https://github.com/TomAs-1226/FrcCatalyst/issues/17) gap).
  `CatalystMotor.setVelocityTorqueCurrent(rps[, ffAmps])` +
  `Builder.torqueCurrentLimits(...)`; `FlywheelMechanism` gains a torque-current mode and a
  per-loop amps-feedforward `track(velocity, feedforwardAmps)` overload for shooters that
  compensate for the piece being fed in. Slot 0 gains are **amps** in this mode (documented);
  `track(vel, ffAmps)` throws at wiring time if torque-current wasn't enabled. Requires Phoenix
  Pro. Defaults preserve voltage-mode behaviour.
- **Swerve internal-sim opt-out** ([#19](https://github.com/TomAs-1226/FrcCatalyst/pull/19),
  fixes [#18](https://github.com/TomAs-1226/FrcCatalyst/issues/18)). The 200 Hz `updateSimState()`
  thread now stands down the first time `setSimPose()` is called, so an external physics engine
  (maple-sim) is no longer silently overwritten. Explicit `disableInternalSim()` /
  `isInternalSimRunning()` controls. Adds the first swerve tests.
- **`SwerveSubsystem.getModuleTargets()` / `getModuleStates()`** — the clean "speeds-out" seam for
  bridging maple-sim at the **mechanism level** (hand it the commanded module states) instead of
  reproducing device sim state by hand. `docs/advanced/simulation.md` now recommends this path with
  a worked example.
- **`AimingSolverVector`** ([#27](https://github.com/TomAs-1226/FrcCatalyst/pull/27)) — a second,
  hardware-independent shoot-on-the-fly solver alongside the time-of-flight `AimingSolver`. It solves
  the shot as a 3D velocity vector and returns hood pitch, yaw *and* flywheel RPS. Pure math; unit
  tested. (The existing `AimingSolver` is unchanged — the contributed rename to `AimingSolverTOF` was
  intentionally not taken, as it would break existing code.)

### Changed

- **All remaining raw NetworkTables telemetry now routes through `CatalystLog`**
  ([#24](https://github.com/TomAs-1226/FrcCatalyst/issues/24)): `CatalystMotor`, `CANRegistry`,
  `GhostReplay`, `HealthHistory`, `RobotSafety`, `SimGamePieces`, `VisionSubsystem`, and
  `SwerveSubsystem`. Every published key path and NT type is unchanged — dashboards and
  AdvantageScope layouts keep working — but a team can now swap the sink once (WPILOG, AdvantageKit,
  a 2027 backend) and every telemetry stream follows. NT *inputs* (Limelight reads, `TunableNumber`,
  camera orientation) are untouched.

### Fixed

- **The v1.2.0 GitHub Release was missing** ([#28](https://github.com/TomAs-1226/FrcCatalyst/issues/28),
  [#25](https://github.com/TomAs-1226/FrcCatalyst/issues/25)) — the tag and docs referenced v1.2.0 but
  no Release object existed, so the Releases page lagged at v1.1.0. The v1.2.0 release is now
  published, and v1.2.0/v1.2.1 build cleanly on JitPack (v1.1.0 remains uninstallable there after a
  transient DNS failure at build time cached a 404 — use v1.2.1).

### Credits

Thanks to [@avrahamavraham](https://github.com/avrahamavraham) for issue #24 and the ideas behind
PRs #21 (swerve logging) and #27 (vector SOTF solver).

## [1.2.0] — 2026-07-21 — A real state machine, for the whole robot

Resolves [#22](https://github.com/TomAs-1226/FrcCatalyst/issues/22). The old
`SuperstructureCoordinator` only understood `LinearMechanism` and `RotationalMechanism`
positions, so you could not put a whole robot in it — and it kept no log worth reading
when something went wrong. The new `frc.lib.catalyst.statemachine` package takes every
mechanism type, is an actual guarded state machine, and logs enough to debug a stuck
superstructure from the driver station with no laptop attached.

Backward compatible: purely additive, plus one deprecation that removes nothing.

### Added — `frc.lib.catalyst.statemachine`

- **`Superstructure<S>`** — the entry point. Enum-typed states, a builder in the Catalyst
  house style, and `Command`/`Trigger` factories: `goTo`, `goToAndHold`, `requestOnly`,
  `waitUntilSettled`, `onlyIfSettled`, `seed`, `abort`, `clearFault`; `in`, `settledIn`,
  `arrivedAt`, `transitioning`, `faulted`, `rejected`, `overridden`.
- **Every mechanism type, plus yours.** `Mechanisms` binds all nine Catalyst mechanisms
  — linear, rotational, differential wrist, flywheel, turret, claw, roller, winch,
  pneumatic — each with a typed goal record so the compiler catches sending an elevator
  to 90 degrees. Any other subsystem plugs in through four escape hatches: `instant` for
  fire-and-forget output, `custom` for plain method calls with an arrival test, `commands`
  for a subsystem that exposes `Command` factories, and `build` for full control. The nine
  built-in bindings are ordinary implementations of the same public interface — nothing is
  reserved for library mechanisms.
- **A legal-transition graph.** `allow`, `allowBoth`, `hub`, `edge`, `via`. An edge you did
  not declare is a hard `NO_EDGE` refusal, so the graph is a real safety constraint rather
  than a hint. `Routing.SHORTEST_PATH` opts into automatic multi-hop routing, off by default.
- **Guards, entry guards and interlocks**, each carrying a reason string that appears in the
  log when it blocks something — so a button that "does nothing" says why.
- **Staged actuation per edge.** `.stage(elevator).stage(arm, wrist)` raises the elevator
  before the arm swings out, declaratively, instead of in a hand-built command group whose
  sequencing nothing can inspect.
- **Arrival is proven, not assumed.** `current()` is only ever a state whose every gating
  mechanism was measured at its goal. Deadlines are per-edge, per-state or global, and always
  finite. `FaultPolicy` decides what a blown deadline does — the default holds position and
  reports rather than cutting anything.
- **The log.** A full schema under `/Catalyst/<prefix>/`, routed through `CatalystLog` so it
  reaches every sink: state and phase timelines, `StateConfirmed`, a `Blocker` string naming
  which mechanism is holding things up, a 5 Hz-throttled `BlockerDetail` with the actual
  numbers, `WaitingOn`, `LegalTargets`, per-binding goal/measured/error/tolerance/owned, a
  50-entry `Transition/History` with outcomes and per-mechanism arrival times, counters, and a
  heartbeat. Plus one-shot Driver Station warnings and a `Sendable` pit widget, both of which
  work with no dashboard configured at all.
- **Build-time validation.** `validate()` returns every problem at once — undeclared states,
  unreachable states, out-of-range setpoints, unknown named-position presets, a roller that
  can never detect a piece — with no hardware involved. A pit deploy cycle costs the better
  part of a minute; six typos in one message beats six crashes.
- **Unit-testable without a roboRIO.** The engine imports nothing from WPILib, so a test fakes
  a mechanism in ten lines. 46 new tests ship with it, covering the graph and validation, the
  truth invariants, transitions and staging, log cadence, and robustness (throwing guards, recovery
  after a timeout). Suite is now 62 tests.

### Added — elsewhere

- `LinearMechanism.getNamedPositions()` / `getPositionTolerance()` and
  `RotationalMechanism.getNamedPositions()` / `getTolerance()`, so a named-position preset can
  be resolved and range-checked at build time instead of throwing from inside a command factory
  during a match.
- `GoalDirector.Builder.superstructure(SuperstructureLike)` — point an existing goal layer at
  either the new state machine or the old coordinator. `coordinator(...)` is unchanged.

### Changed

- `CatalystLog`'s sink is now created lazily. Previously the eager field initialiser meant that
  merely touching the class constructed a `NetworkTablesSink`, which throws with no HAL present
  and made even `setSink(fake)` unusable from a unit test.

### Deprecated

- **`SuperstructureCoordinator`** — superseded by `statemachine.robot.Superstructure`. Not
  scheduled for removal, and its behaviour is deliberately **frozen**: four known defects are
  documented in its javadoc rather than fixed, because changing them would silently alter what a
  robot already built on it physically does, mid-season. Existing code keeps working unchanged.
  It now also implements `SuperstructureLike`, which is a source- and binary-compatible addition.

## [1.1.0] — 2026-07-16 — Audit fixes from a full robot port

Resolves the 15-finding v1.0.0 audit ([#17](https://github.com/TomAs-1226/FrcCatalyst/issues/17))
from porting a full robot codebase (Team 5805's clone of team581's 2026 comp-bot)
onto Catalyst. Backward compatible: bug fixes plus additive API.

### Fixed
- **Swerve now simulates.** `SwerveSubsystem` starts a 200 Hz sim thread
  (`updateSimState`) in the constructor, so the drivetrain actually moves in the
  simulator instead of freezing with stale signals.
- **`CatalystGyro` no longer wipes the Pigeon 2 config.** The default constructors
  stopped applying a blank `Pigeon2Configuration`, which was silently erasing the
  Tuner X mount pose. A new `CatalystGyro(canId, canBus, Pigeon2Configuration)`
  constructor opts in to Catalyst-owned config.
- **`LinearMechanism.zero()` now sets `hasBeenZeroed`**, so the zeroed interlock
  and the public getter stop lying after an explicit zero.
- **`pathfindToPose()` is lazy.** It defers so the target pose is read when the
  command is scheduled, not once at construction, so both legs track a moving target.
- **`xBrake()` and `idle()` hold their requirement** (`run` instead of `runOnce`);
  `idle()` again honors the `Subsystem.idle()` run-forever contract.
- **PathPlanner path following is closed-loop and keeps its feedforwards.** The
  drive callback now uses `ApplyRobotSpeeds` (velocity control) with the wheel
  force feedforwards instead of dropping them onto an open-loop request.
- **`configurePathPlanner()` fails loudly** (stack trace + a persistent
  `AlertManager` error) instead of a quiet `reportError`.

### Added
- **Runtime current limits on `CatalystMotor`**: `setSupplyCurrentLimit`,
  `setStatorCurrentLimit`, `setCurrentLimits` (re-send the whole group so thermal
  protection is never dropped) — enables state-based power budgeting.
- **Current-spike homing**: `LinearMechanism.homeOnCurrent(...)` and
  `RotationalMechanism.homeOnCurrent(...)` drive into a hard stop until the stator
  current spikes, then seed the encoder and mark zeroed. For robots with no limit switch.
- **`RotationalMechanism.hasBeenZeroed()`** getter (parity with `LinearMechanism`).
- **`SwerveSubsystem.setMaxAngularRate(double)`** for asymmetric module layouts.

### Docs / packaging
- The vendordep now lists the **PhotonVision, PathPlanner, and Phoenix maven
  repositories**, so its transitive dependencies resolve without extra repo setup.
- `goTo(...)` javadoc now points to `goToAndWait(...)` for waiting transitions;
  documented that drive feature flags apply only to `advancedDrive()` and that one
  heading PID backs every heading drive mode.
- Corrected the advertised install coordinate (JitPack `com.github.TomAs-1226:FrcCatalyst`,
  not `com.frccatalyst`) and the test-coverage wording (16 tests cover the aiming
  solver, alliance flip, and turret math; mechanisms and swerve are verified in the
  simulator, not yet unit-tested).

## [1.0.0] — 2026-07-15 — Out of beta 🎉

**FrcCatalyst 1.0.0.** After a long beta (v0.3 through v0.10) and four release
candidates, the library is stable and ready for the 2026 season.

This is the same code as `1.0.0-rc4`, promoted after the release-candidate series
held up. Nothing changes in the API; this release marks the milestone:

- **Stable, versioned public API.** Semantic versioning starts here. Breaking
  changes are reserved for 2.0.0.
- **Proven.** Mechanisms, swerve, Shoot-On-The-Fly aiming, the behavior
  framework, logging, and the generic `SimDashboard` are covered by a real unit
  test suite (`./gradlew test`) and verified in the simulator and on a robot.
- **Documented.** Full docs at
  [tomas-1226.github.io/FrcCatalyst](https://tomas-1226.github.io/FrcCatalyst/),
  plus browser tools and per-mechanism guides.

Thank you to everyone who filed issues and PRs through the beta, especially
**@avrahamavraham** and the teams on Chief Delphi. Go build something awesome.

Everything shipped in the `1.0.0-rc1` through `1.0.0-rc4` candidates below is part
of 1.0.0.

## [1.0.0-rc4] — 2026-07-15 — Community PRs + telemetry, swerve, sim, vision

Seven community PRs merged, plus four additive feature areas. Backward compatible.

### Merged community contributions (thanks @avrahamavraham)
- Red-alliance operator-perspective fix so field-centric drive is correct on red.
- `maxAngularRate` now derives from the real module radius instead of a magic constant.
- Swerve `idle()` command and a ChassisSpeeds NT publisher.
- Struct logging on the `LogSink` chain (hardened on merge to a non-breaking default).
- `CatalystLog.enableLoggingInputs(boolean)` to cut NT traffic under loop overrun.
- Custom `WpilogSink(LogSink alsoTo)` destinations.
- A cross-platform UTF-8 Javadoc build fix.

### Added — struct + struct-array logging, end to end
- `CatalystLog.log(key, Struct, value)` and `log(key, Struct, value[])` publish any
  WPILib struct-serializable type (poses, `SwerveModuleState`, ...) as real objects
  in AdvantageScope, through both the NetworkTables (`StructArrayPublisher`) and
  WPILOG (`StructArrayLogEntry`) sinks, forwarded through `CompoundSink`.

### Added — swerve module-state telemetry
- Measured and target `SwerveModuleState[]` publish to `/Catalyst/Swerve/ModuleStates`
  and `/ModuleTargets` for the AdvantageScope swerve view. Also fixes the broken
  `WheelRadiusCalibration` javadoc link.

### Added — SimDashboard v2
- Per-mechanism **sparkline** history with a setpoint guide, a global **pause/resume**
  toggle, and **CSV export** of the live snapshot. Frontend only.

### Added — simulated vision
- `SimCameraSource`, a `CameraSource` that emits noisy, latency-delayed pose
  estimates from a supplied simulated pose, so the multi-camera fusion pipeline
  runs in the WPILib simulator with no hardware.

### WPILib 2027
- New `wpilib-2027` branch with a grounded migration plan (`MIGRATION_2027.md`):
  the dominant cost is the `edu.wpi.first` -> `org.wpilib` package rename, Catalyst
  uses none of the removed 2027 classes, and it is blocked on 2027 vendor builds.
  Main stays on WPILib 2026.

## [1.0.0-rc3] — 2026-06-28 — Configurable simulation

The simulation is no longer tied to one robot. A new generic dashboard renders
and drives **any** mechanism you register, and the four mechanisms that used to
sit motionless in sim now run real physics. **Backward compatible** — only
additions, with sensible defaults for the new sim-only config fields.

### Added — `SimDashboard`, a configurable mechanism cockpit
- New `frc.lib.catalyst.sim.SimDashboard`: register any `CatalystMechanism` with
  `dash.add(mechanism)` and it auto-discovers the mechanism's shape and renders a
  fitting live widget — a travel bar for a linear actuator, a speed readout for a
  flywheel, a state chip for a claw, an angle gauge for a turret. Works against a
  team's own `CatalystMechanism` subclass too, with no dashboard changes.
- Opt-in driving per mechanism: `.button(...)`, `.command(...)`, `.slider(...)`
  and `.toggle(...)`. Browser input is **never** run on the HTTP thread — it is
  queued and executed inside `update()` on the main/scheduler thread, so it is
  always safe to schedule Commands or mutate robot state from a control.
- **Real-robot safe**: `start()` and `update()` no-op off simulation, so the
  same calls can live in shared robot code unguarded. Default port 5805,
  configurable.

### Added — every mechanism describes itself
- New `CatalystMechanism.describe()` returns a uniform `MechanismView` snapshot
  (value, unit, setpoint, range, velocity, current, plus kind-specific extras).
  A base default is provided, and all nine built-in mechanisms override it, so
  the generic dashboard can render and drive them without knowing their type.

### Added — physics for the mechanisms that used to be inert in sim
- `RollerMechanism`, `ClawMechanism`, `WinchMechanism` and
  `DifferentialWristMechanism` now have real `simulationPeriodic()` models
  (`FlywheelSim` / `DCMotorSim` / `ElevatorSim`), so they actually move, track
  setpoints and draw current in simulation like the other mechanisms already did.
- `RollerMechanism.setSimHasPiece(boolean)` and `ClawMechanism.setSimHasPiece(boolean)`
  let you force the simulated game-piece state to exercise handoff logic without
  a physical sensor (sim only — ignored on a real robot).
- New **sim-only** config fields, additive with defaults: `WinchMechanism`
  `loadMass(kg)` and `DifferentialWristMechanism` `momentOfInertia(kg·m²)`.

### Added — `MechanismShowcase` example
- The example now serves a second cockpit at
  [localhost:5806](http://localhost:5806) that builds one of **every** mechanism
  kind and drives each through `SimDashboard` — a working template for wiring the
  dashboard to your own robot, alongside the existing game cockpit on 5805.

## [1.0.0-rc2] — 2026-06-19

### Added — `AllianceFlipUtil`
- Author field coordinates once in the **blue-origin** frame; call
  `AllianceFlipUtil.apply(...)` at runtime to get the right pose/translation/
  heading for the current alliance — no more duplicate red/blue constants.
  Configurable field size (defaults to REBUILT) and symmetry
  (`ROTATIONAL` / `MIRRORED`). Unit-tested both ways.

### Added — Shoot-On-The-Fly browser tool
- A new interactive [SOTF Visualizer](https://tomas-1226.github.io/FrcCatalyst/tools/aiming/):
  drag the robot, set a velocity, and watch the **virtual goal**, lead, turret
  bearing and feedforward rate update live — running the exact corrected
  virtual-goal math from `AimingSolver`. Copies the `track(...)` wiring.

## [1.0.0-rc1] — 2026-06-19 — Preseason Release Candidate 1

Hardening pass on the Shoot-On-The-Fly stack ahead of the season, with a real
test suite to keep it honest. **Backward compatible** — only additions and fixes.

### Fixed — SOTF solver was never exact (off-by-one in the virtual-goal iterate)
- `AimingSolver.solve(...)` reported a `virtualGoal` that lagged the reported
  `timeOfFlight` by one iteration, so even a perfectly aimed, perfectly sped
  shot landed `v · Δtof` off the target — a **velocity-dependent error that
  could never reach zero**, even in a perfect simulation. The solve now iterates
  to a true **fixed point** (tof, distance and virtual goal mutually consistent),
  so an ideal shot is exact by construction. A new closed-loop unit test sweeps
  4,459 pose/velocity cases and confirms max landing error ≈ **1.3 × 10⁻¹⁴ m**.
- `solve(...)` now **guards non-finite velocity** (a momentary pose-estimator
  glitch falls back to a stationary solve instead of producing `NaN` aim).

### Added — continuous tracking for the rest of the shooter
- **`FlywheelMechanism.track(DoubleSupplier rps)`** and
  **`RotationalMechanism.track(DoubleSupplier deg)`** — re-read the setpoint
  every loop, so flywheel RPM and hood angle follow the live distance during
  SOTF (the turret already had `track`). Closes the radial-motion gap: distance
  is now compensated, not just bearing.
- **`TurretMechanism.track(solution, headingDeg, yawRateDps)`** — a 3-arg
  overload that applies the solver's **exact analytic bearing rate** (minus yaw
  rate) as velocity feedforward, so the turret leads a moving goal even while
  the chassis is rotating.
- **`TurretMechanism.aimErrorDeg(solution, headingDeg)`** and
  **`isOnTarget(solution, headingDeg, tolDeg)`** — the "barrel on target" half
  of a shoot-while-moving readiness gate, plus a live aim error for dashboards.

### Added — turret simulation
- **`TurretMechanism` now self-simulates** (`DCMotorSim`), with optional
  `.motorType(...)` and `.simMOI(...)` config. Turret/SOTF code now moves in the
  WPILib simulator with no hardware — the one headline mechanism that didn't.

### Added — optional goal / intent layer
- **`frc.lib.catalyst.goal`** — `Goal` + `GoalDirector`: a thin, *completely
  optional* "software-defined robot" layer. The driver gives an intent
  (`director.pursue(SCORE_HIGH)`); the director drives the
  `SuperstructureCoordinator` to the right state, runs the goal's setup work,
  reports readiness, and hands control back to the driver via the standard
  command-requirement model. Publishes to `/Catalyst/Goal/*`. Skip the package
  entirely and the rest of the library is unchanged.

### Added — runnable example + browser sim cockpit
- **`example/`** — a runnable REBUILT (2026) example robot wired on the real
  library (swerve-style drive, intake, turret, flywheel, hood, the goal layer),
  plus a dependency-free **browser Sim Cockpit** (`localhost:5805`) that
  visualizes the field, the live code path, the SOTF math, telemetry, and the
  safety stack. A teaching/demo harness, not shipped robot code.

### Added — test suite
- First JUnit tests ship with the library: the SOTF closed-loop proof, solver
  self-consistency, radial/tangential lead, NaN fallback, max-range/degenerate
  cases, and the turret continuous-wrap resolver. Run with `./gradlew test`.

## [0.10.1-beta] — 2026-06-09

### Fixed — swerve helper commands no longer steal the drivetrain
- **`SwerveSubsystem.slowModeWhileHeld(...)` required the drivetrain**, so holding it interrupted the default drive command and the robot stopped moving (teams worked around it with `.proxy()`, which misbehaved in simulation). Slow mode is a *state modifier*, not a drive command — it now requires **no subsystem** and just sets the speed multiplier the drive command reads, so you keep driving while it's held. No `.proxy()` needed, correct in sim. Thanks to **tcrvo (3211)** for the report.
- `resetHeading()` and `resetPoseCommand(...)` were also momentarily reserving the drivetrain (one-tick interrupt of the default command). They're pure odometry ops now requiring no subsystem, and run while disabled.

## [0.10.0-beta] — 2026-06-09

### Added — reactive autonomous architecture (path + reactive blending)
The hard part of modern FRC auto: a PathPlanner path is a time-parameterized plan, but chasing a piece or auto-aligning takes you off it. PathPlanner removed replanning in 2025, so the right approach is three deliberate tools — now first-class in Catalyst.
- **`PathCorrection`** (`subsystems.swerve`) — bend a path toward a live target *without leaving it*, using PathPlanner 2026's feedback-override API with safe lifecycle (override set on command start, **always cleared on end** — never leaks into the next path):
  - `facingPoint(goal, poseSupplier, follow)` / `facing(headingSupplier, follow)` — face a goal while following the path's XY (shoot-on-the-move). Clean: PathPlanner still does its own rotation feedback, toward your target.
  - `nudgingXY(xCorr, yCorr, follow)` — bias X/Y toward a target while pathing (⚠️ replaces PP's XY feedback; documented sharp edge).
- **`SwerveSubsystem.followPath(name)`** — follow a pre-made path exactly (segment between known waypoints).
- **`SwerveSubsystem.pathfindThenFollowPath(name [, constraints])`** — the "rejoin the plan" primitive: pathfind from the *current* pose to the next path's start, then follow. Use this right after a reactive deviation instead of `followPath` (which assumes you start at the path's beginning).
- **Auto Builder** gains command templates mapping one-to-one onto these (Follow path / Pathfind→rejoin / Follow+face goal / Chase piece / Pathfind→pose), plus an inline architecture note.
- Docs: [Autonomous Architecture](https://tomas-1226.github.io/FrcCatalyst/advanced/autonomous.html) — the full plan-as-feedforward / reaction-as-feedback / pathfind-to-reconnect writeup, researched against current PathPlanner.

## [0.9.0-beta] — 2026-06-09

### Changed — BrownoutMonitor is now passive by default
- `BrownoutMonitor` no longer takes any action unless you opt in. `outputScale()` stays at `1.0` and nothing trips until you call `.enableThrottling()` and/or `.tripsRobotSafety(true)`. Prominent warnings added — these behaviours are aggressive (throttling cuts robot power mid-match; a too-high warn threshold can throttle a healthy robot). The passive monitor still estimates and publishes to `/Catalyst/Brownout/` so you can watch the prediction before deciding to act.

### Added — Auto Builder tool
- New [Auto Builder](https://tomas-1226.github.io/FrcCatalyst/tools/auto/) browser tool — generates a behavior-framework auto (resilient `BehaviorEngine` sequence or utility `Strategist`) as copy-paste Java. Add actions, set fallbacks / deadline / bail (sequence) or score expressions (utility), optionally scaffold `Action` stubs. **Path-following stays with PathPlanner / Choreo** — the tool generates the autonomy *wiring*, you reference your paths inside the action commands. localStorage persistence, copy, `.java` download. (PathPlanner is the general pather; this builds the reactive strategy layer on top.)

### Added — maple-sim support (dependency-free)
- `SimGamePieces` — streams simulated game-piece `Pose3d[]` to `/Catalyst/Sim/<name>` for AdvantageScope rendering. Works with any sim engine.
- `SwerveSubsystem.setSimPose(Pose2d)` — feed a physics-sim pose (e.g. maple-sim's) into Catalyst's odometry; no-op on a real robot.
- [Simulation guide](https://tomas-1226.github.io/FrcCatalyst/advanced/simulation.html) — wires maple-sim to the Catalyst autonomy stack. Catalyst provides the seam; you add maple-sim to your own robot project (not bundled — it's an unstable, fast-moving, sim-only package, and Catalyst is a consumed library).

## [0.8.0-beta] — 2026-06-09

### Added — Tier 2 batch (drive, power, logging)
- **Choreo paths** — `SwerveSubsystem.followChoreoPath(name)` follows Choreo's time-optimal `.traj` files through PathPlanner (no extra vendordep). Loads from `src/main/deploy/choreo/`; reports to the DS and no-ops if missing.
- **`SwerveSubsystem.driveToPiece(Supplier<Optional<Translation2d>>)`** — vision-pursuit primitive the `Autopilot` "acquire" action wanted. Drives onto a detected piece, stops on arrival or when it disappears.
- **`WheelRadiusCalibration`** — spins the robot and back-solves the actual wheel radius from gyro arc vs measured module arc, correcting the CAD value (a documented source of auto inaccuracy). Publishes the corrected radius + a copy-paste constant. Adds `SwerveSubsystem.getModuleDistances()`.
- **`BrownoutMonitor`** — predicts sag voltage from `Σ current × R_internal`, exposes a graceful `outputScale()`, and preemptively trips `RobotSafety` before the radio drops.
- **`CatalystMotor.Builder.optimizeCanBus()`** — raises only the status signals Catalyst reads and silences the rest, cutting CAN-bus load on high-device-count robots. Opt-in (default off) so it never starves a signal you depend on.
- **`WpilogSink`** — records all Catalyst logging to a standard `.wpilog` (opens in AdvantageScope / DataLogTool; no extra vendordep). The "record" half of replay-style debugging; full deterministic replay still routes through AdvantageKit via the existing `CatalystInputs` bridge.

### Notes
- Tier 3 evaluated: the Auto Builder and log-scrubber browser tools were **declined** (PathPlanner's GUI and AdvantageScope already do those well — building inferior versions isn't worth it); WPILib Epilog is **documented as opt-in** (it works alongside `WpilogSink` without bundling a conflicting logging path). QuestNav and maple-sim are saved for later with integration notes in the [Roadmap](https://tomas-1226.github.io/FrcCatalyst/ROADMAP.html).

## [0.7.0-beta] — 2026-06-09

### Added — `SystemCheck` pre-match self-test
- New `frc.lib.catalyst.util.SystemCheck` — run every subsystem through a verification routine from one button and get a go/no-go board before queueing. Catches the failures that actually lose matches: a loose connector, an inverted motor after a swap, a dead follower, an encoder stuck at zero.
  - `check(name, BooleanSupplier)` — instant pass/fail (battery, gyro, sensor sanity).
  - `timed(name, action, seconds, pass, cleanup)` — apply an action for a duration, confirm a result, always clean up. The classic use is "drive the motor, confirm the encoder moves and current is sane" — catching dead motors, disconnected encoders, and backwards followers.
  - Publishes per-test `PASS`/`FAIL: reason`, a `Ready` boolean, and a copy-paste `Report` to `/Catalyst/SystemCheck/<name>/`. Check lambdas are exception-guarded.
- Docs: [System Check](https://tomas-1226.github.io/FrcCatalyst/advanced/system-check.html).

### Added — Roadmap + competitive analysis
- New [docs/ROADMAP.md](https://tomas-1226.github.io/FrcCatalyst/ROADMAP.html) — researched analysis of where Catalyst stands vs YAGSL / AdvantageKit / maple-sim / QuestNav / Choreo, and a prioritized feature plan. SystemCheck is the first Tier-1 item shipped; QuestNav pose source and maple-sim physics simulation are next.

## [0.6.1-beta] — 2026-06-09

### Cleanup pass — API additions + doc/tool correctness
A full audit of every code example against the source turned up methods that were documented but never built, and two browser tools generating code that wouldn't compile. All fixed.

**Source additions (real gaps the docs assumed):**
- `SwerveSubsystem.getFieldRelativeSpeeds()` — robot-relative speeds rotated into the field frame. **This is what `AimingSolver` needs for SOTF** — the prior docs told you to pass `getChassisSpeeds()`, which is robot-relative and would mis-aim once the robot wasn't facing +X.
- `SwerveSubsystem.getMaxSpeedMPS()` / `getMaxAngularRate()` — getters the `SwerveSetpointGenerator` examples needed.
- `RobotSafety.trip(String)` + `RobotSafety.tripCommand(String)` — manual trip for conditions outside the health system (brownout / low-battery triggers).

**Tool fixes (were generating non-compiling code):**
- MotorType Browser emitted `.motorCount(n)` — no such builder method. Now emits a `// + n follower(s)` note.
- Catalyst Tuner emitted `.gravity(kG)` — the method is `.gravityGain(kG)`.

**Doc corrections:** fixed `leds.setPattern → setSolidColor`, `leds.greenCommand → solid(Color.kGreen)`, `alignmentIndicator` signature, `rumble.driver(...) → fire(..., Channel.DRIVER)`, the `VisionSubsystem` constructor example (real form is single-arg with `.driveSubsystem(drive)`), `.motorCount → .follower`, and added an explicit **PathPlanner support** section confirming the `AutoBuilder` wiring.

**Verified:** every other documented Catalyst call now resolves against the source.

## [0.6.0-beta] — 2026-06-09

### Added — Behavior framework (`frc.lib.catalyst.behavior`)
Game-agnostic autonomous + assisted-driving orchestration. The framework sequences and reacts; your `Action`s hold the game-specific work, so it carries forward to future games unchanged.
- **`Action`** — atomic capability: a `Supplier<Command>` plus a precondition (`when`), success test (`until`), estimated cost, and subsystem `requires`. Buggy lambdas are caught so one bad precondition can't crash a strategy.
- **`BehaviorEngine`** — reactive sequencer for resilient autos. Checks each action's precondition *when reached* and falls back (`orElse` substitute / `orElseSkip` / `orElseAbort`). `deadline(s)` (measured from schedule time) or `bailWhen(...)` stops the sequence and runs an `onBail(...)` action — "give up chasing, go align and shoot." Publishes `Step / Action / FellBack / Bailed` to NT.
- **`Strategist`** — utility selector. Register behaviors with score functions; the highest scorer that can start runs, switching the instant another wins. Expresses "chase scattered pieces until the goal is met or time runs short, then bail to a guaranteed score" as two crossing score curves. Publishes per-behavior scores + `Active` to NT.
- **`Autopilot`** — teleop cycle co-pilot. Hold a button → acquire → score → repeat, releases to the driver instantly. Built from an acquire action, a score action, and a `hasPiece` supplier.
- **`BehaviorContext`** — lightweight match-time / mode snapshot passed to scorers.

### Changed — multi-camera vision robustness
- `VisionSubsystem.periodic()` now fuses cameras deterministically for 4+ camera setups: each camera is snapshotted once and filtered independently, NaN/Inf poses are rejected up front, and accepted estimates are added to the pose estimator in **timestamp order** (out-of-order adds cause estimator jitter) with **quality → camera-index tiebreaks**. The published vision pose is now the single highest-quality accepted estimate rather than "whichever camera was processed last," so vision-driven behavior decisions are reproducible run-to-run.

## [0.5.1-beta] — 2026-06-09

### Added / Changed — SOTF hardening
- **`ShotCompensation`** (`util/`) — live operator aim bias + defense robustness. Turret / distance / RPM / hood bias offsets (nudge methods for D-pad binding), a `velocityScale` SOTF-aggressiveness knob, plus a **velocity deadband and clamp** so a defender's hit can't spike the measured chassis speed and fling the aim. Values publish to `/Catalyst/Aiming/<name>/...`.
- **`AimingSolver` upgrades**:
  - Accepts an optional `ShotCompensation` (`.compensation(...)`) and applies bias + conditioned velocity on every solve.
  - `.maxRange(m)` — shots beyond your tested effective range are marked infeasible.
  - `Solution` now carries `turretFieldRateDps()` — the analytic field-bearing rate (`(rᵧ·vₓ − rₓ·vᵧ)/|r|²`) for turret velocity feedforward.
  - Shooter / hood lookups now use the distance-biased lookup distance.
- **`TurretMechanism` velocity feedforward** — `track(...)` differentiates the resolved command and applies a `kV` voltage feedforward so the turret leads a moving goal rather than lagging it. Skips the unwrap-jump loop, clamps to ±2 V.

## [0.5.0-beta] — 2026-06-09

### Added — Turret + Shoot-On-The-Fly
- **`TurretMechanism`** (`mechanisms/`) — single-axis turret with continuous-angle resolution. The wrap / soft-limit "unwrap" logic picks the reachable `desired + 360·k` representation closest to the current position and only takes the long way around when the short way is blocked; clamps to a limit when the target is unreachable. Field-relative aim, vision-error lock, Motion Magic moving-goal tracking, health checks, live tuning, optional fused CANcoder for boot-time absolute homing.
  - `resolveTurretAngle(desired, current, min, max)` is exposed `static` and pure for unit testing.
  - Commands: `goToAngle`, `lockForward`, `holdAngle`, `aimAtFieldAngle`, `aimAtTarget`, `track`, `aimWithVision`, `zero`.
- **`AimingSolver`** (`util/`) — hardware-independent Shoot-On-The-Fly math using the virtual-goal method: `virtualGoal = target − v_field · timeOfFlight`, iterated to converge. Returns a `Solution` record (field aim bearing, distance, flight time, shooter RPM, hood angle, virtual goal, feasibility). Builder takes `InterpolatingTable` lookups for shot time / RPM / hood. Static and SOTF modes; no motors or NT, so it's unit-testable.
- **`TurretMechanismInputs`** (`io/`) — IO logging snapshot matching the other mechanisms.
- Docs: [Turret & Shoot-On-The-Fly](https://tomas-1226.github.io/FrcCatalyst/advanced/aiming.html) with the full SOTF derivation and a tuning checklist.

## [0.4.1-beta] — 2026-06-07

### Added — driver experience
- **`GhostReplay`** — record the live robot pose during teleop, replay it later as a ghost pose for a new driver to chase. Stores trajectories as plain CSV under the deploy directory (`ghosts/<name>.csv`), so a recording follows the codebase across deploys. Publishes the ghost pose to `/Catalyst/Ghost/Pose` for AdvantageScope field overlay.
- **Per-mechanism `bindRumble(events, pattern, channel)`** — one-line ergonomic path that pre-picks each mechanism's "obvious" event:
  - `ClawMechanism` + `RollerMechanism` → `hasPieceTrigger()`
  - `FlywheelMechanism` → `atSpeedTrigger()`
  - `DifferentialWristMechanism` → `atSetpointTrigger()`
  - The four-arg form `bindRumble(events, trigger, pattern, channel)` is still there for everything else.

## [0.4.0-beta] — 2026-06-07

### Added — driver experience
- **`RumbleEvents`** — bind any `Trigger` to an Xbox-controller rumble pattern (`SHORT`, `LONG`, `DOUBLE_TAP`, `TRIPLE_TAP`, `RAMP`). Targets driver, operator, or both. A scheduler updates the rumble state every loop so back-to-back events don't fight.
- **`DriverProfile`** — per-driver feel: radial deadband, response curve (`LINEAR` / `QUADRATIC` / `CUBIC` / `EXPO`), max-speed cap, slow-mode multiplier. Wrap a joystick `DoubleSupplier` once and the swerve drive gets a shaped supplier in return. Swap profiles to swap drivers.

### Added — library
- **`RobotState`** — singleton view of "what's the robot doing right now": mode (`isAutonomous`, `isTeleop`, `isDisabled`, …), alliance, match time, station, battery voltage, `timeSinceEnable`. Cached for 5 ms so re-reading from multiple subsystems is cheap. Exposes ready-to-bind triggers: `RobotState.lateMatch(20)`, `.lowBattery(11.0)`, `.autonomous()`, `.disabled()`.
- **SysId for every motor** — `CatalystMotor.sysIdQuasistatic(Direction)` / `.sysIdDynamic(Direction)` produce ready-to-bind Commands using WPILib's `SysIdRoutine` and Phoenix-6's `SignalLogger`. `CatalystMechanism` adds zero-arg variants that target the mechanism's primary motor — no per-mechanism boilerplate. Teams need to call `SignalLogger.start()` once in `robotInit()`.
- **`LimelightTriggers`** — `Trigger` wrappers around the Limelight NT keys: `hasTarget()`, `tagInView(int)`, `detectorClass(String)`, `targetWithinArea(double)`, `horizontalErrorBelow(double)`. Plus diagnostic readers (`tx()`, `ty()`, `ta()`, `tid()`, `latencyMs()`). Works with any Limelight on the bus — point it at the NT table name and bind.
- **`SwerveSetpointGenerator`** — light-weight chassis-aware accel/skid clamp. Wraps a requested `ChassisSpeeds` and returns one limited by max wheel speed, max angular rate, and max translational accel (per-second delta-v cap). Cheaper than a full feasibility solver, handles the common driver-induced skid case.

### Added — tools
- **Health Dashboard timeline** — `HealthHistory` events now render as a swim-lane timeline at the bottom of the dashboard. One lane per `subsystem/id`, severity-colored dots (filled = fired, hollow = cleared), hover to see the live detail string and "X.X s ago". Auto-rescales to the current event window.

## [0.3.6.1-beta] — 2026-05-18

### Fixed — silent follower loss in Linear / Rotational
- `LinearMechanism.Config.follower(canId, oppose)` and
  `RotationalMechanism.Config.follower(canId, oppose)` were **overwriting**
  the previous follower on every call. Anyone calling
  `.follower(11, true).follower(12, false)` silently lost the first
  follower. Both are now additive (matching the v0.3.5 fix to Claw and
  Flywheel). Same `(canId, oppose)` API; just call once per follower for
  3+ motor setups.
- The old workaround `additionalFollower(canId, oppose)` is now a
  `@Deprecated` shim that forwards to `follower(...)`. Existing code
  keeps compiling.

### Added
- `RollerMechanism` now supports followers — `.follower(canId, oppose)`
  is additive, same pattern as the other mechanisms. Two-motor intakes
  (master + mirrored follower) need only one builder call per follower.
  Per-follower `OverCurrent` / `HighTemp` health checks register
  automatically.

### Builder UI fixes
- The Roller schema previously emitted `.intakeVoltage(6.0)` /
  `.outtakeVoltage(-4.0)` / `.holdVoltage(...)` — none of those methods
  exist on `RollerMechanism`. Schema now generates the correct
  `.intakeSpeed(...)` / `.ejectSpeed(...)` / `.stallDetection(...)`.
- The schema also emitted `.gravity(0.35)` for Linear/Rotational kG —
  the Java method is `.gravityGain(0.35)`. Fixed.
- The Rotational tolerance emitted `.toleranceDegrees(1.0)` — Java
  method is `.tolerance(1.0)`. Fixed.
- The **Intake** preset now wires a follower at id 41 (mirrored) so the
  generated code matches the "Roller · 2-motor + beam-break" subtitle.

## [0.3.6-beta] — 2026-05-18

### Added — `CANRegistry`
- New `frc.lib.catalyst.hardware.CANRegistry` — a process-wide registry of every CAN device the robot has claimed.
- Every `CatalystMotor` (primary, followers, and any attached CANcoder) **auto-registers** at builder time. Duplicate `(bus, id)` with a different name throws `CANConflictException` with both sides named. Identical re-registrations are idempotent.
- Lookup, snapshot, and per-bus views: `CANRegistry.lookup(id, bus)`, `.all()`, `.byBus()`.
- Plan is published to `/Catalyst/CAN/Devices` as a pipe-delimited string array for the Health Dashboard and other NT viewers.

### Added — CAN ID Planner "Generate Catalyst Java"
- New output mode in the [CAN ID Planner](https://tomas-1226.github.io/FrcCatalyst/tools/canids/) emits a complete `CANIds.java`:
  - `public static final int` constants in `SCREAMING_SNAKE_CASE` per device
  - Static block that pre-registers every planned device with `CANRegistry`
  - Configurable Java package, copy-to-clipboard, download as `.java`
- Calling `CANIds.init()` once from `Robot.robotInit()` surfaces wiring mistakes — missing device, wrong name, duplicate id — at boot instead of mid-match.

## [0.3.5.1-beta] — 2026-05-18

### Tools (hosted on GitHub Pages)
- **`docs/tools/` landing page** — clean overview that links to all three live tools (Builder, Tuner, Health Dashboard). The same pages are served directly from the Pages site at `tomas-1226.github.io/FrcCatalyst/tools/...` so teams don't have to clone anything to use them.
- **Catalyst Builder enhancements**:
  - **localStorage persistence.** Form state survives page reloads.
  - **Download as `.java`** — one click writes the generated code to a properly-named file ready to drop into `src/main/java/`.
  - **Full subsystem class mode** — toggle wraps the config in a complete `public class FooSubsystem extends SubsystemBase` skeleton with `get()` accessor and `periodic()` hook.
  - **Import existing config** — paste any `Foo.Config.builder()...build()` snippet and the form populates itself.
  - **Clear all saved data** link in the footer.
- **Tuner: Download gains JSON** — saves a snapshot of every tuned value (including Motion Magic constants) to a timestamped `.json` file. Useful for archiving working tunes between events.
- **Health Dashboard: Download report** — plain-text snapshot of every check's current state. Drop into a team chat when triaging.

### Lib additions
- **`RobotSafety.trippedTrigger()`** — returns a WPILib `Trigger` for direct binding in `RobotContainer.configureBindings()`:
  ```java
  RobotSafety.trippedTrigger().onTrue(drive.stopCommand());
  ```
- **`HealthHistory`** — fixed-capacity ring buffer (default 100) of recent fire / clear events. Automatically fed by `HealthMonitor` on every transition and published as a string array at `/Catalyst/Health/History`. Queryable from team code via `HealthHistory.snapshot()` for post-match triage.

## [0.3.5-beta] — 2026-05-18

### Fixed
- **`ClawMechanism` followers are no longer capped at one.** The Config builder now appends rather than overwriting, so calling `.follower(canId, oppose)` repeatedly attaches multiple followers as advertised. Per-follower OverCurrent / HighTemp health checks register automatically.
- **`FlywheelMechanism` gained follower paths.** Use `.primaryFollower(canId, oppose)` for single-wheel shooters with two or more motors ganged on one shaft, and `.secondaryFollower(...)` when running independent top/bottom wheels each with their own followers. The independent `.secondMotor(...)` API is unchanged.
- Both fixes credited to **avrahamavraham** for opening the issue on Chief Delphi.

### Changed
- **`DifferentialWristMechanism` now uses Phoenix-6 native differential control.** Internally the left motor is the master and runs `DifferentialMotionMagicVoltage`; the right motor is in `DifferentialFollower` mode and is wired via `DifferentialSensors.RemoteTalonFX_HalfDiff`. Both targets ship in one CAN frame and the firmware keeps them coordinated — replaces the previous "two independent Motion Magic loops" pattern. Thanks to **tcrvo** for the suggestion.
  - Pitch axis tunes through the existing `.pid(...) / .feedforward(...)` builder methods (Slot 0).
  - Roll axis can now be tuned independently via `.differentialPid(...) / .differentialFeedforward(...)` (Slot 1) — defaults to the same gains as pitch when not specified.
  - Slot 1 gains are live-tunable under `/Catalyst/Tuning/<Name>/Diff/...` alongside the existing Slot 0 tunables.

### Added
- **`RobotSafety` watchdog** in `frc.lib.catalyst.util` — opt-in cross-mechanism trip. Configure with `RobotSafety.configure(...)`, supplies `isTripped()`, `reason()`, manual `reset()`, optional auto-reset, and `onTrip` / `onReset` callbacks. Driven each loop by `HealthMonitor.tick(errorCount, warnCount)`; when no config is installed it's a zero-cost no-op. Publishes to `/Catalyst/Safety/{Tripped,Reason,ErrorCount,WarnCount}`.
- **Catalyst Builder UI** at `docs/tools/builder/index.html` — single-file dark-themed form that generates ready-to-paste mechanism config code (`LinearMechanism.Config.builder()...build()` snippets) for every Catalyst mechanism. Multi-follower fields, motor-type dropdown, copy-to-clipboard. Credits tcrvo / yteam3211's original [FRC Catalyst Subsystem Generator](https://yteam3211.github.io/frc-catalyst-subsystem-generator) for the design idea.
- **More `MotorType` presets**:
  - `KRAKEN_X44`, `KRAKEN_X44_FOC` (shipped earlier in 0.3.3 but worth restating)
  - `NEO`, `NEO_VORTEX`, `NEO_550` — REV motors for teams running a mixed stack (sim + physics only; not Phoenix-controllable)
  - `MINION` — the WCP Minion
- **Hot-reload for Slot 1** — `CatalystMotor.updateSlot1(kP, kI, kD, kS, kV, kA)` for any mechanism using a differential control mode.

### Docs
- Added an **Acknowledgements** section to the README crediting outside contributors (currently tcrvo / yteam3211 and avrahamavraham). Removed scattered team-name name-drops elsewhere in the docs and source comments in favour of generic "successful teams" / "in-house" language.
- Bumped install snippets and the AdvantageScope tab bundle versions to 0.3.5.

## [0.3.3-beta] — 2026-05-14

### Fixed — IMPORTANT, READ THIS
- **`MotorType` FOC variants had the same stall torque as their non-FOC counterparts.** Phoenix 6 FOC delivers about 30% more stall torque, so any team using `MotorType.KRAKEN_X60_FOC` or `MotorType.FALCON_500_FOC` was getting wrong numbers out of `holdingVoltage(...)`, `maxMechanismTorque(...)`, `getDCMotor(...)`, and `MotionConstraintCalculator`. Concrete effect: gravity feedforward voltages were over-stated by ~30%, and sim models under-reported torque. Re-check any hand-tuned `kG` values after upgrading.
  - `KRAKEN_X60_FOC`: stall torque 7.09 → **9.37 Nm**, free speed 6000 → **5800 RPM**, stall current 366 → **483 A**
  - `FALCON_500_FOC`: stall torque 4.69 → **5.84 Nm**, stall current 257 → **304 A**, free speed → **6080 RPM**
  - Values now match CTRE's published specs and WPILib 2026's `DCMotor.getKrakenX60Foc()` / `DCMotor.getFalcon500Foc()`.

### Changed
- **`MotorType` is no longer an `enum`** — it's a regular `final class` with the same `public static final` constants (`MotorType.KRAKEN_X60` etc.) so existing code keeps compiling unchanged. The change unlocks user-declared motor specs, which teams running NEO, NEO Vortex, Minion, or anything else Catalyst doesn't ship a preset for can now use directly: `new MotorType("NEO 550", 0.97, 11000, 100, 1.4)`.

### Added
- **`MotorType.KRAKEN_X44`** and **`MotorType.KRAKEN_X44_FOC`** presets — previously missing. Specs sourced from CTRE: 4.05 / 5.45 Nm stall torque, 7530 / 7200 RPM free, 275 / 366 A stall current.
- **`CatalystMath`** gained FOC and X44 constants (`KRAKEN_X60_FOC_STALL_TORQUE`, `KRAKEN_X44_STALL_TORQUE`, `FALCON_FOC_STALL_TORQUE`, etc.). The existing non-FOC constants are unchanged.
- **Health Kit** under `frc.lib.catalyst.util`:
  - `HealthCheck` — a single debounced fault condition with `Severity` (INFO/WARN/ERROR), a `BooleanSupplier` predicate, optional live `detail` string, `debounce(seconds)`, `clearAfter(seconds)`, and `onFire`/`onClear` hooks. Built via a fluent `HealthCheck.builder(subsystem, id)` and registered with one `.register()` call.
  - `HealthMonitor` — singleton registry that ticks every check once per loop (throttled to 5 ms, so all eight built-in mechanisms calling it cost one evaluation per scheduler tick). Publishes per-check state to `Catalyst/Health/<subsystem>/<id>/{firing,severity,description,detail,firedAt}` and rollup counts to `Catalyst/Health/{ErrorCount,WarnCount,InfoCount,Healthy}`. Every fire/clear edge is relayed to the existing `AlertManager` so dashboards already wired against it keep working.
  - `HealthMonitor.standardMotorChecks(...)` — one call registers OverCurrent (WARN at 90% of stator limit, debounce 0.5 s), HighTemp (WARN, debounce 1.0 s, clearAfter 5.0 s), and OverTemp (ERROR at warn+10 °C, immediate fire, auto-calls `motor.stop()`).
- **Health checks wired into every built-in mechanism** — Linear/Rotational add Stall + NotZeroed, Flywheel adds NotSpinningUp, Pneumatic adds LowPressure (with `requirePressureAbove(psi)` gating actuation). Multi-motor mechanisms (Flywheel, Winch, Claw follower, DifferentialWrist) register per-motor checks with collision-free id suffixes.
- **Health Dashboard** at `docs/tools/health/index.html` — single-file dark-themed web viewer that connects to NT4 read-only, shows per-subsystem cards with severity-colored firing checks, filter buttons (All / Firing only / Errors only), and a search box.

### Notes
- HealthCheck predicates and detail suppliers are wrapped in try/catch — a buggy lambda from team code won't take down the whole monitor.
- All thresholds use honest seconds-based debounce/clearAfter semantics rather than loop-cycle counts, so behavior is unaffected by scheduler period.
- Backward-compatible: existing `AlertManager.error/warning/info` calls in team code keep working; the built-in mechanisms just go through HealthCheck now.

## [0.3.2-beta] — 2026-05-14

### Added
- **Live-tunable PID + Motion Magic gains by default** on every closed-loop mechanism (`LinearMechanism`, `RotationalMechanism`, `FlywheelMechanism`, `DifferentialWristMechanism`). All Slot-0 gains (`kP`, `kI`, `kD`, `kS`, `kV`, `kA`, `kG`) and Motion Magic profile constants (`CruiseVelocity`, `Acceleration`, `Jerk`) are published under `Catalyst/Tuning/<MechanismName>/...` on NetworkTables. Edit from any dashboard, change applies on the next loop. Zero user code required.
- `CatalystMotor.updateSlot0(p, i, d, s, v, a, g)` and `CatalystMotor.updateMotionMagic(cruise, accel, jerk)` — public hot-reload methods used by the new tunable wiring. Gravity model from initial builder config is preserved.
- `frc.lib.catalyst.util.TunableGains` — bundles every gain into one helper and only re-applies when a value has actually changed (cheap to call every periodic).
- `docs/advanced/tuning.md` — full guide including the recommended competition lock-down pattern.

### Notes
- Default behavior is unchanged at robot init: gains start at whatever you put in the Config builder.
- Call `TunableNumber.disableTuning()` once in `robotInit()` for competition builds. After that, `hasChanged()` always returns false, no NT reads happen, and the motor configurator is never touched. Tuning is essentially free at runtime once disabled.
- No new dependencies. No API breaks.

## [0.3.1-beta] — 2026-05-14

### Fixed
- `ClawMechanism.hasPiece()` now OR-combines beam-break and stall detection. Previously, configuring a beam-break sensor would short-circuit the method and make the stall-current latch unreachable, even though the builder docstring describes beam-break as an "alternative / additional" signal.

### Added
- `PneumaticMechanism.timeInState()` — seconds since the last forward/reverse/off transition. Lets teams sequence actions with `Commands.waitUntil(() -> piston.timeInState() > 0.25)` instead of hand-rolling timers.
- `PneumaticMechanism.getTransitionCount()` — public getter for the transition counter already tracked internally.

## [0.3.0-beta] — 2026-05-14

### Added
- **Multi-follower support** on `CatalystMotor` and `LinearMechanism`. Each `withFollower(canId, oppose)` call now appends a follower instead of overwriting the previous one. New `FollowerSpec` record + `withFollowers(FollowerSpec...)` varargs convenience.
- **In-house logging core** under `frc.lib.catalyst.logging`:
  - `CatalystLog` — static facade routing all mechanism telemetry through a single pluggable sink.
  - `LogSink` — interface with typed `log(...)` overloads plus `processInputs(...)`.
  - `NetworkTablesSink` — default sink. Preserves the v0.2 `/Catalyst/<name>/...` NetworkTables layout.
  - `CompoundSink` — fan-out to multiple sinks (e.g., NT + AK during a competition).
  - `CatalystInputs` — symmetric `toLog`/`fromLog` contract for Inputs POJOs.
  - `LogTable` — typed key/value table used for serialization.
- **AdvantageKit bridge documentation** at `docs/advanced/logging.md`. Catalyst itself takes no AK dependency; teams write a ~10-line `LogSink` to bridge.
- **IO + Inputs contract** under `frc.lib.catalyst.io`:
  - `LinearMechanismInputs` / `LinearMechanismIO`
  - `RotationalMechanismInputs` / `RotationalMechanismIO`
  - `RollerMechanismInputs` / `RollerMechanismIO`
  - `FlywheelMechanismInputs` / `FlywheelMechanismIO`
  - `WinchMechanismInputs` / `WinchMechanismIO`
  - `ClawMechanismInputs` / `ClawMechanismIO`
  - `DifferentialWristMechanismInputs` / `DifferentialWristMechanismIO`
  - `PneumaticMechanismInputs` / `PneumaticMechanismIO`
  - All five built-in mechanisms (plus the three new ones) populate their Inputs POJO each loop and ship it via `CatalystMechanism#processInputs`.
- **`ClawMechanism`** — motor-driven gripper with stall-current grip detection and a low passive hold voltage. Supports optional beam-break and follower motor.
- **`DifferentialWristMechanism`** — two-motor diffy wrist. Resolves `(pitch, roll) ↔ (left, right)` and commands both axes via Motion Magic with software pitch/roll limits and named presets.
- **`PneumaticMechanism`** — single/double solenoid wrapper with `extend()` / `retract()` / `toggle()` / `pulse(duration)` commands, optional pressure-aware actuation guard (`requirePressureAbove(psi)`), and cycle counting.
- **Forward-limit auto-zero** on `LinearMechanism` (mirrors the existing reverse-limit support).
- **Configurable tolerances** on `RotationalMechanism` via `tolerance(degrees)`; new tolerance support on `DifferentialWristMechanism`.

### Changed
- `CatalystMechanism` now routes all telemetry through `CatalystLog` instead of writing directly to NetworkTables. The `telemetryTable` field is preserved for backwards compatibility with v0.2 user code.
- `VisionSubsystem` now raises an `AlertManager` warning when constructed without a drive subsystem, instead of silently no-op'ing.

### Fixed
- `RotationalMechanism.atPosition(String)` no longer ignores the configured angular tolerance (used a hardcoded `2.0` degrees regardless of config).
- `LinearMechanism` reverse-limit auto-zero now seeds the encoder to `config.minPosition` instead of zero (was incorrect when `minPosition != 0`).
- `LinearMechanism` simulation motor count is now derived from the live follower count, so it can no longer drift out of sync when followers are added.

### Migration notes
- The default behavior is unchanged. Existing v0.2 robot code keeps working without modification.
- Teams wanting AdvantageKit telemetry can install a `LogSink` at robot init — no mechanism code changes required.

## [0.2.0-beta] — 2026 season

Initial public beta. See git history.
