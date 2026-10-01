---
layout: default
title: Autonomy 2.0
parent: Advanced
nav_order: 10.5
---

# Autonomy 2.0
{: .no_toc }

Nine small decision cores that answer *what should the robot do next*, and one board that publishes
what they decided and why. They read the robot's situation and return a record. **None of them
commands anything** — no core schedules a command, writes a pose, sets a motor output or blocks a
transition. What you do with a decision is your code's business.

{: .warning }
> New in 2.0.0-alpha.3 and not yet run on a robot. Every core is unit-tested and every one is a pure
> function or a small object with explicit memory, so they are cheap to test yourself — but nothing
> here has been driven.

1. TOC
{:toc}

---

## Why it is shaped this way

A behaviour layer that both decides and acts is one you cannot test and cannot argue with. When the
robot does something surprising, "which of the fourteen things that can command the drivetrain did
that" is the question you are left with, at a competition, with six minutes on the clock.

So the decision is separated from the acting. Every core takes a snapshot and returns a record with
a **reason string** in it. You can log the reason, print it, assert on it in a test, or read it off
the dashboard afterwards. And because a core cannot command anything, adding one cannot break a
robot that ignores its answer.

```java
// The whole pattern, in three lines.
Situation now = source.sample(Timer.getTimestamp());
TaskArbiter.Selection<Runnable, String> picked = TaskArbiter.select(candidates, 0.2);
picked.winners().forEach(w -> schedule(w.task()));   // your code, your call
```

---

## `Situation` — the seam

One immutable snapshot per loop, in five facets:

| Facet | Carries |
|---|---|
| `localization()` | `pose`, `confidence` 0..1, a `Level` band (`HIGH`/`MODERATE`/`LOW`/`LOST`), `secondsSinceAbsoluteFix` |
| `motion()` | field-relative `ChassisVelocities`, `speedMps`, `accelMpsSq` |
| `traction()` | `slipFactor`, `tractionUsage`, `tippingUsage`, plus `slipping()` and `nearTipping()` |
| `power()` | `busVolts`, `headroomAmps`, `drawAmps`, and `bindingLimit` — *which* limit is binding |
| `match()` | `enabled`, `autonomous`, `secondsRemaining` |

**Each facet carries its own `valid` flag**, and that is the part that matters. A robot with no
`PhysicsCore` has no traction estimate; a robot with no current measurement has no power estimate. An
invalid facet says so rather than reporting a plausible zero, and every core is written to do
nothing rather than something wrong when the facet it needs is invalid.

`power()` is the one worth understanding, because a robot can be part-way there. `busVolts` comes
from the robot controller and is always real - the record's own javadoc says it is "meaningful even
when the rest is not". What needs measuring is *current*, and `PowerPredictor` takes a
`DoubleSupplier` for it rather than a `PowerDistribution`, so a robot with no power module on CAN can
still feed it the sum of its Talon FX supply currents. Without any current source the facet is
`Power.unmeasured(volts)`: `valid()` is false, `AuthorityCore.fromSituation` skips its power branch,
and `ShedCore` has nothing to shed. Nothing degrades; that half of the layer simply does not run.

There is a second, subtler invalid case. A `PowerPredictor` with no breaker budget set reports a
*battery-sag* headroom - true physics, and roughly twice what a 120 A main breaker allows. The facet
stays invalid in that state rather than handing an allocator a number that reads like a budget and
is not one.

```java
Situation.blind()            // every facet invalid; what a robot with no instrumentation sees
Situation.blind(nowSeconds)  // the same, timestamped
```

Convenience predicates never fire on invalid data: `traction().slipping()` is `false` on a robot
with no traction estimate, not `false` because the wheels are gripping.

## `SituationSource` — where a snapshot comes from

```java
public interface SituationSource {
    Situation sample(double nowSeconds);

    static SituationSource blind();            // for tests and for robots with no physics layer
    default SituationSource cachedPerLoop();   // one sample per timestamp, however many cores ask
}
```

`cachedPerLoop()` is not an optimisation detail — it is a correctness one. Six cores reading the
situation six times in one loop could each see a slightly different robot. Wrap the source once and
they cannot.

`PhysicsSituationSource` builds a `Situation` from what the robot already has:

```java
SituationSource source = PhysicsSituationSource.builder()
        .physics(physicsCore)          // optional: without it, traction and motion go invalid
        .power(powerPredictor)         // optional: without it, power goes invalid
        .build()
        .cachedPerLoop();
```

---

## The cores

### `TaskArbiter` — which behaviours may run together

Greedy weighted independent set over declared requirements. Highest score wins, anything needing a
mechanism already taken is skipped **with the name of the winner that took it**.

```java
record Candidate<T, R>(T task, String name, double score, Set<R> requirements, ...)

TaskArbiter.Selection<Command, String> s = TaskArbiter.select(candidates, 0.2 /* minScore */);
s.winners();    // may be several — they share no mechanism
s.skipped();    // each with a reason: "drivetrain taken by Score"
```

Ties break by name, so the same inputs give the same answer twice. An arbiter that reordered equal
candidates between loops would make two behaviours alternate at 50 Hz.

### `ChaseCore` — which target is worth going for

Ranks by **value per second**, not by value and not by distance. A cheap target you can reach in one
second beats a rich one that takes the rest of the match.

```java
record Target<T>(T target, String name, double value, double seconds, boolean reachable, ...)

ChaseCore.Choice<Note> c = ChaseCore.choose(targets, match.secondsRemaining(), 0.5 /* minValue */);
c.chosen();     // Optional — empty when nothing clears minValue or nothing is reachable in time
c.rejected();   // the rest, in rank order
c.reason();
```

### `CycleCore` — repeating a sequence without chattering

A phase machine with **dwell** and **stall handback**. It will not leave a phase before
`dwellSeconds`, and if a phase cannot start for `stallHandbackSeconds` it gives up and says so
rather than retrying forever.

```java
CycleCore cycle = new CycleCore(3 /* phases */, 0.25 /* dwell s */, 2.0 /* handback s */);
CycleCore.Decision d = cycle.decide(now, desiredPhase, desiredCanStart);
d.act();   // RUN / HOLD / HAND_BACK
d.reason();
```

### `StepCore` — one action, with a declared fallback

```java
StepCore.Decision d = StepCore.decide("Score", scoreReady, StepCore.Fallback.SUBSTITUTE,
                                      "Stow", stowReady);
d.outcome();     // RUN / SUBSTITUTE / SKIP / ABORT
d.fellBack();
```

The fallback is a parameter rather than a policy baked into the core, because "what should happen
when this cannot start" is a match-strategy question, not a library question.

### `AuthorityCore` — how much of what you asked for the robot may have

Min-of-limiters with attribution. Several things can want to slow the robot down; the smallest wins
and the result says **which one**.

```java
AuthorityCore.Authority a = AuthorityCore.combine(List.of(
        AuthorityCore.fromSituation(now),                       // localization/traction derived
        new AuthorityCore.Limit("operator", 0.5, "precision mode")));
a.scale();     // 0..1
a.binding();   // "operator"
a.explain();
```

`fromSituation` derives a limit from confidence and traction. It is a suggestion with a name on it,
not a governor: multiplying your command by `a.scale()` is a line of code you write.

### `ShedCore` — giving current back, with floors

**Shed-only.** It never grants current and never raises a limit; it only says what to turn down when
the budget is already exceeded. Each claim declares a `floorAmps` it will not be cut below, so
shedding cannot stop a mechanism that must keep holding.

```java
ShedCore.Plan p = ShedCore.plan(claims, deficitAmps, 5.0 /* don't bother below this */);
p.cuts();        // name -> new amp ceiling
p.shortfall();   // what could not be shed without breaking a floor — this is the honest number
```

A non-zero `shortfall()` is the interesting output: it means the robot is asking for more than it
can have *and* every remaining claim is already at its floor. That is a design problem, and the core
reports it rather than hiding it by cutting through a floor.

### `IntentCore` — guessing what the driver is doing

Ranks named guesses with hysteresis (`stickiness`), and **scores itself**: `observe(actual)` feeds
back what really happened, and `hitRate()` says how often the guess was right.

```java
IntentCore intent = new IntentCore(0.55 /* minConfidence */, 0.15 /* stickiness */);
IntentCore.Reading r = intent.read(guesses);
r.best();       // Optional — empty when nothing clears minConfidence
r.explain();
intent.observe("intake");    // later, when you know
intent.hitRate();
```

A hit rate you can read is the point. An intent system nobody can score is one nobody should trust,
and this one publishes its own accuracy to the dashboard.

### `AutonomyBoard` — what happened, on the wire

Overloaded `publish(...)` for every decision type, under `/Catalyst/Autonomy/`:

```java
AutonomyBoard board = new AutonomyBoard();
board.publish(now);          // the Situation
board.publish(selection);    // TaskArbiter
board.publish(choice);       // ChaseCore
board.publish(authority);    // AuthorityCore
board.publish(plan, deficit);// ShedCore
board.publish(reading);      // IntentCore
```

Reasons are published as strings, so the dashboard shows *why* rather than only *what*. That is the
whole reason every decision record carries one.

---

## Autonomy 2.1: shared control

The driver drives. Assists help with the parts a person does badly under pressure — holding a
heading, lining up on something — without taking the robot away. `SharedControl` takes the driver's
command and the assists' *proposals* each loop and returns one `Decision`: the velocity to send, who
owns the heading, with how much authority, and why. Like every core above it **commands nothing**;
sending the decision is your line of code.

{: .warning }
> New in 2.0.0-rc.3-a6. Unit-tested and run in simulation, but not yet driven on a robot. Treat the
> rules below as what the tests prove, not as something a driver has felt.

### The rules it enforces

These are what "not intrusive" means here, and each has a test:

1. **The driver's stick always wins.** While the driver turns, the rotation is theirs, the same
   loop. A heading assist that waits for rest comes back only after the turn stick has rested
   `restSeconds` (0.3 s) *and* the robot has stopped rotating (under `settledRadps`, 0.3 rad/s) — so
   it never undoes the end of the driver's turn.
2. **An assist never makes the robot faster** than the driver asked, and never sends it against the
   driver's direction. A translation proposal that would is cut down or dropped.
3. **No steps.** A heading assist's authority ramps from zero over `rampSeconds` (0.25 s) each time
   it takes over.
4. **One owner of the heading at a time**: the first proposal in the list that may act. The order of
   the list is the priority.
5. **Bounded.** A heading assist's turn rate is capped by its own proposal.

The thresholds that switch a proposal on and off belong to whoever makes the proposal, and should
have hysteresis. Smoothing the resulting velocity is `VelocityLimiter`'s job, after this.

### Proposing a heading

A `HeadingProposal` comes in three kinds:

| Factory | Meaning |
|---|---|
| `hold(source, kP, maxRateRadps, reason)` | Hold the heading the robot has when the assist takes over. Waits for rest. |
| `to(source, targetRad, kP, maxRateRadps, reason)` | Turn to a given heading. Waits for rest. |
| `external(source, reason)` | The robot's own code drives the heading (an aim with its own controller). Does not wait for rest; the decision names it and commands no turn. |

Angles are radians, counter-clockwise positive; velocities are on the field, in m/s.

```java
SharedControl shared = new SharedControl(new SharedControl.Config());

// In the drive command's loop:
SharedControl.Driver driver = new SharedControl.Driver(
        fieldVx, fieldVy,                  // the sticks, already turned into field velocities
        -rightX * maxTurnRadps,            // the turn asked, used only while turning
        Math.abs(rightX) > 0.05);          // is the turn stick off its deadband

List<SharedControl.HeadingProposal> proposals = new ArrayList<>();
if (aiming) {
    proposals.add(SharedControl.HeadingProposal.external("AIM", "aiming at the hub"));
}
proposals.add(SharedControl.HeadingProposal.hold("HOLD", 4.0, 3.0, "holding heading"));

SharedControl.Decision decision = shared.decide(
        Timer.getTimestamp(),
        driver,
        drive.getHeading().getRadians(),
        drive.getYawRateRadPerSec(),
        proposals,
        Optional.empty());                 // no translation assist this loop

double omega = decision.external() ? aimController.omega() : decision.omega();
drive.driveFieldAbsolute(decision.vx(), decision.vy(), omega);
log.info(decision.headingOwner() + ": " + decision.reason());
```

When `decision.external()` is true the owner drives the heading itself and `omega()` is zero, so the
turn comes from your own controller (`aimController` above is yours). The list order is the
priority: here the aim beats the hold, and the hold only acts once the driver has stopped turning.

A translation assist is an `Optional<TranslationProposal>`
(`new TranslationProposal(source, vx, vy, reason)`). Rule 2 applies: it is scaled down to the speed
the driver asked for, and dropped if it points against their direction or the driver is not moving.

### `VelocityLimiter` — no lurch

A slammed stick asks for the whole top speed in one loop. `VelocityLimiter` hands the drivetrain a
velocity whose **vector** changes no faster than a limit — the size of the change, not each axis on
its own. Per-axis limiters, like `enableSlewRateLimiting`'s, let a diagonal push accelerate at 1.4
times the limit. Here, "slowing down" means the commanded *speed* is falling, whichever way the robot
points.

```java
VelocityLimiter limiter = new VelocityLimiter();

// When the drive command starts, from the robot's measured velocity:
ChassisVelocities now = drive.getFieldRelativeSpeeds();
limiter.reset(now.vx, now.vy);

// Each loop, after SharedControl:
double maxAccel = physicsConstraints.maxAccelerationMpsSq();
double[] v = limiter.limit(0.02, decision.vx(), decision.vy(), maxAccel, maxAccel);
drive.driveFieldAbsolute(v[0], v[1], omega);
```

`limit(dt, wantVx, wantVy, maxAccelMpsSq, maxDecelMpsSq)` takes separate acceleration and
deceleration limits; a limit that is not finite or not positive means no limit. Give it
`PhysicsConstraints.maxAccelerationMpsSq()` and a slammed stick becomes the hardest launch the carpet
and the robot's stability allow, and no harder — or less, for a driver who wants a gentler robot.

### Call `reset()`

`SharedControl.reset()` forgets everything held. Call it when the pose's heading is reset, and when
the drive command starts. A heading held across a reset is a target in a frame that no longer
exists; the robot would turn back to it. Reset `VelocityLimiter` with the robot's measured velocity
when the drive command starts, as above.

---

## What this deliberately is not

- **Not a scheduler.** Nothing here schedules, cancels or requires a command. `TaskArbiter` tells
  you which candidates do not conflict; scheduling them is your line of code.
- **Not a safety layer.** `AuthorityCore` returns a number and `ShedCore` returns a plan. Neither
  applies anything. `RobotSafety` is still the layer that stops things.
- **Not a replacement for the state machine.** `StateMachineCore` owns *what state the robot is in*;
  these cores answer *what to aim at next*. They compose — an arbiter winner can be a `goTo(...)`.
- **Not free of the physics layer.** Without `PhysicsCore` the traction and motion facets are
  invalid, and the cores that need them decline to decide. That is the designed behaviour, not a
  degraded one.

## Planning it visually

The desktop app ships an **Autonomy 2.0 Planner**: assemble candidates and requirements, and see
which tasks win, which are held, and which mechanism each loser lost. Its conflict matrix answers
the question worth asking before a competition — *can these two behaviours ever run together, or
does everything need the drivetrain?*
