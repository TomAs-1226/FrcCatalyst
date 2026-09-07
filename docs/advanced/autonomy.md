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
> New in 2.0.0-alpha.2 and not yet run on a robot. Every core is unit-tested and every one is a pure
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
`PhysicsCore` has no traction estimate; a robot whose PDH is not on CAN has no power estimate. An
invalid facet says so rather than reporting a plausible zero, and every core is written to do
nothing rather than something wrong when the facet it needs is invalid.

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
