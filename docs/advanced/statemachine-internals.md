---
layout: default
title: State Machine Internals
parent: Advanced
nav_order: 3.5
---

# State Machine Internals
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

This page is the answer to
[issue #29](https://github.com/TomAs-1226/FrcCatalyst/issues/29): "it looks so complicated
it is basically impossible to debug", "there are like 40 files only for this", "give me a
full page of just how does it work in the back". So this page is about the *inside* of the
machine — how a request becomes motor output, why the file count is what it is, and exactly
where to look when something misbehaves.

It is not a how-to. If you want to *build* a superstructure, read
[State Machine]({% link advanced/statemachine.md %}) first — this page assumes you have seen
`Superstructure.builder(...)`, `goTo(...)`, states, edges, and bindings, and it will not
re-explain them.

{: .note }
The honest short version of the #29 complaint: yes, there are 44 files, but 43 of them are
tiny. The actual engine — every decision, every arrival check, every timeout — lives in
**one** file, `StateMachineCore.java`. Once you know that, the package stops being scary.

---

## 1. Why so many files

Java forces one public class per file. A package that models nine mechanism types, each with
its own goal shape and its own hardware adapter, and then wraps the whole thing in a real
finite state machine with logging, is going to be a lot of *files* even though it is not much
*logic*. Counting them and being surprised is fair; but the count is almost entirely
single-purpose value types, not engine code.

The 44 files fall into four layers, and the layers matter because they are what keeps the
hard part testable:

| Layer | Package | Files | What it is | WPILib? |
|---|---|---|---|---|
| **Core engine + value types** | `statemachine` | 20 | The FSM itself plus the plain values it speaks in | **None** |
| **Goals** | `statemachine.goals` | 9 | One typed "where should this mechanism be" value per mechanism | None |
| **Mechanism adapters** | `statemachine.mech` | 10 | One WPILib adapter per mechanism, plus the `Mechanisms` facade | Yes |
| **Robot glue** | `statemachine.robot` | 5 | The `Subsystem` / `Command` / `Trigger` / logging layer | Yes |

### The core engine + value types (20 files, zero WPILib imports)

This is the only layer with real logic, and even here the logic is concentrated. Of the 20
files, exactly **one** is the engine:

| File | What it is |
|---|---|
| **`StateMachineCore.java`** | **The engine.** ~1830 lines, and everything below is a value it operates on. This is the file to read. |
| `Binding.java` | The one interface the engine knows about hardware — no WPILib types, so a test implements it in ten lines. |
| `StateSpec.java` | What one state *means*: each mechanism's goal, entry guard, settle time, timeout, fault policy, enter/exit actions. |
| `EdgeSpec.java` | What one transition means: guard, timeout, cost, and the actuation `stage(...)` ordering. |
| `StateGraph.java` | The legal-transition graph and its deterministic breadth-first routing. |
| `Handle.java` | A type-carrying token so `set(elevator, RotationalGoal.degrees(90))` fails to *compile*. |
| `Phase.java`, `RejectReason.java`, `FaultPolicy.java`, `Routing.java` | Four small enums — the machine's vocabulary. |
| `Route.java`, `TransitionResult.java`, `TransitionRecord.java`, `ArrivalReport.java`, `BindingSample.java`, `Snapshot.java` | Immutable result/record values the engine hands out. |
| `TransitionHistory.java` | The newest-first ring buffer of records. |
| `StateMachineTelemetry.java` | The sink interface the engine publishes through (so it never imports NetworkTables). |
| `ValidationReport.java`, `StateMachineConfigException.java` | Build-time error collection. |

The point of the "zero WPILib imports" rule is not purity for its own sake. It is what lets
the entire decision logic — routing, guards, arrival, timeouts, fault policy — run in a JUnit
test on a laptop with no HAL, no scheduler, and a fake clock. The engine is *told* the time
by a `DoubleSupplier` and *told* to advance by `step()`; it never reaches for
`Timer.getFPGATimestamp()` itself. That same property is what will make the eventual port to
the 2027 command framework a change to the thin robot layer instead of to the FSM.

### Goals (9 files) — one value type per mechanism

`goals/`: `LinearGoal`, `RotationalGoal`, `WristGoal`, `FlywheelGoal`, `TurretGoal`,
`ClawGoal`, `RollerGoal`, `WinchGoal`, `PneumaticGoal`.

Each is a `record` describing *where a mechanism should be* — `LinearGoal.meters(0.30)`,
`LinearGoal.preset("L4")`, `FlywheelGoal.rpm(4200)`, `ClawGoal.open()`. They are `record`s on
purpose: the engine calls `Objects.equals(want, applied)` every loop to decide whether to
re-apply actuation, so a goal type needs value-based equality. A hand-written class with
identity equality would compare unequal to itself and rebuild its pursue command 50 times a
second (`LinearGoal`'s javadoc walks through exactly this trap, including the `-0.0` vs `0.0`
edge). These files are almost all factory methods and normalisation — no control flow.

### Mechanism adapters (10 files) — one adapter per mechanism, plus a facade

`mech/`: `LinearBinding`, `RotationalBinding`, `WristBinding`, `FlywheelBinding`,
`TurretBinding`, `ClawBinding`, `RollerBinding`, `WinchBinding`, `PneumaticBinding`, and
`Mechanisms`.

Each `*Binding` implements `Actuator<ThatGoal>` and is the single place that knows how to
turn one goal into motor output for one mechanism type: build the pursue command, measure
`atGoal(...)`, resolve presets at build time, report the tolerance and the shortfall note.
`Mechanisms` is a facade of static factories — `Mechanisms.linear(mech)`,
`Mechanisms.flywheel(mech)` — so robot code writes `b.bind("elevator", Mechanisms.linear(...))`
instead of `new LinearBinding(...)`. Your own subsystem is exactly as first-class: implement
`Actuator<YourGoal>` and it drops in with no special-casing.

The 9 goals and 9 bindings are the "19 files" from the complaint. They are 19 files because
there are 9 mechanism types and Java needs a file per class, not because there are 19 things
to understand — it is the same two ideas (a value, an adapter) repeated nine times.

### Robot glue (5 files)

`robot/`: the layer that touches WPILib so the engine does not have to.

| File | What it is |
|---|---|
| `Superstructure.java` | A `SubsystemBase` that owns the engine, steps it every loop, and exposes `goTo(...)` commands and `arrivedAt(...)` triggers. |
| `GoalRunner.java` | The default command installed on each mechanism; it reads the engine's active goal and hosts the pursue/hold commands. |
| `Actuator.java` | `Binding` plus the WPILib `Command` plumbing (`pursueCommand`, `holdCommand`, `requirements`). |
| `CatalystStateMachineLog.java` | Routes the whole log schema into `CatalystLog`, plus alerts and Driver Station messages. |
| `SuperstructureLike.java` | The small interface `GoalDirector` talks to, shared with the old coordinator. |

So: four layers, one of which contains the actual machine. When you read #29's "40 files",
read it as "one engine, plus a lot of small typed values that keep the engine honest and
testable."

---

## 2. The layer diagram

Data flows down (your intent becomes motion) and truth flows back up (sensors become the
machine's belief). The engine in the middle never touches a motor and never imports WPILib;
the robot layer on either side does.

```mermaid
flowchart TB
    code["Your code<br/>button.onTrue(sm.goTo(AIM))"]
    ss["Superstructure  (SubsystemBase)<br/>periodic() → engine.step()"]
    core["StateMachineCore  (the engine)<br/>request() · step() · activeGoalOf()<br/><i>zero WPILib imports</i>"]
    graph["StateGraph / StateSpec / EdgeSpec<br/>guards · interlocks · staging"]
    runner["GoalRunner  (default command, one per mechanism)<br/>reads activeGoalOf(handle)"]
    bind["Actuator / Binding  (LinearBinding, …)<br/>pursueCommand() · atGoal()"]
    mech["The mechanism<br/>LinearMechanism, FlywheelMechanism, …"]
    log["CatalystStateMachineLog → CatalystLog<br/>NetworkTables · Alerts · Driver Station"]

    code -->|"request(target)"| ss
    ss --> core
    core <--> graph
    core -->|"active goal per mechanism"| runner
    runner -->|"hosts pursue/hold command"| bind
    bind -->|"motor output"| mech
    mech -.->|"sensors: atGoal()"| bind
    bind -.->|"measured / atGoal"| core
    core -.->|"telemetry sink"| log
```

Two things are worth pinning down here, because they are the crux of the design:

- **The engine decides, the `GoalRunner` acts.** `StateMachineCore` never calls a command. It
  computes, per mechanism, "what goal should you be pursuing right now?" and exposes it as
  `activeGoalOf(handle)`. Each `GoalRunner` — installed as that mechanism's *default command*
  — reads that answer once per loop and hosts the matching pursue command. That single seam is
  what makes driver override free: when a driver command grabs the elevator, WPILib interrupts
  the elevator's `GoalRunner`, the engine is told ownership was lost, and when the driver lets
  go the default is rescheduled and the goal re-applies. No reassertion race, no policy flag.

- **Arrival flows *up* from sensors, not down from intent.** The dotted lines are the ones
  that keep the machine honest: `current()` only ever advances because a `Binding.atGoal(...)`
  measured true, never because a command "finished."

---

## 3. One loop, start to finish

This is the most important section on the page. If you understand one 20 ms scheduler tick,
you understand the machine.

`CommandScheduler.run()` runs, in this fixed order, every loop:

1. every `Subsystem.periodic()`,
2. then it polls `Trigger`s and schedules/cancels commands,
3. then it runs every scheduled command's `execute()`.

`Superstructure` *is* a `SubsystemBase`, so its `periodic()` runs in phase 1 — **before** any
trigger is polled and **before** any `GoalRunner.execute()` runs. That ordering is not an
accident; it is the whole reason the machine is coherent within a loop:

```java
// Superstructure.periodic()
engine.setEnabled(DriverStation.isEnabled());
engine.step();
```

Here is the tick, end to end:

```mermaid
sequenceDiagram
    participant Sched as CommandScheduler.run()
    participant SS as Superstructure.periodic()
    participant Core as StateMachineCore.step()
    participant Run as GoalRunner.execute()
    participant Bind as Binding / mechanism

    Note over Sched: phase 1 — periodic()
    Sched->>SS: periodic()
    SS->>Core: setEnabled(DS.isEnabled()); step()
    activate Core
    Core->>Core: ticks++, read clock
    Core->>Core: syncGoalApplication(now)  — per-binding "goal changed?" bookkeeping
    Core->>Core: publishBindings(now)      — change-detected telemetry
    Core->>Core: if disabled: freeze deadlines, return
    Core->>Bind: advanceTransition(): isAt(hop)? (reads atGoal)
    Core->>Core: on proven arrival → write current(); else check timeout
    Core->>Core: publishTimeline(now)
    deactivate Core

    Note over Sched: phase 2 — triggers polled, commands scheduled
    Note over Sched: phase 3 — command execute()
    Sched->>Run: execute()  (one per mechanism)
    Run->>Core: activeGoalOf(handle)
    Core-->>Run: the goal for this mechanism right now (or null)
    Run->>Bind: host pursue/hold command → motor output
```

Read that as three beats:

1. **`step()` runs first** and does all the *thinking* for the loop: it advances the
   in-flight transition, measures arrival, enforces the deadline, and publishes telemetry.
   Crucially, it decides *what each mechanism's goal is* for this loop but does not command
   anything.

2. **Triggers are polled** against the state the machine just settled into — so
   `arrivedAt(CARRY).onTrue(...)` fires against this loop's truth, not last loop's.

3. **Each `GoalRunner.execute()` runs** and asks `activeGoalOf(handle)` for its answer, then
   drives the motor.

### The one-loop lag, and why it is fine

Because `step()` computes goals in phase 1 and `GoalRunner` applies them in phase 3, a goal
the engine chooses this loop reaches the motor *this same loop* — good. But arrival works the
other way: a mechanism physically reaches its target during phase 3, and `step()` does not
*measure* that until phase 1 of the **next** loop. So `current()` can lag physical arrival by
one 20 ms tick.

That is not only acceptable, it is the safe direction to be wrong in. The machine is
conservative: it would rather believe it *has not* arrived for one extra loop than believe it
*has* arrived one loop early. A 20 ms delay before `arrivedAt(...)` fires is invisible on a
robot; a state machine that declared arrival before the sensor agreed would be the exact class
of bug this package exists to kill.

---

## 4. How a transition actually flows

There are two completely separate halves to a transition, and keeping them separate is the
core design decision.

### `request()` — a pure decision, no motion

`request(target)` **decides** and nothing else. It never builds a command, never moves a
motor, and — this is a hard guarantee — **never throws**. It:

1. calls `evaluate(target)`, which runs every gate in order: enabled? faulted? seeded?
   already there? blocked by an `Interlock`? blocked by the target's `entryGuard`? any gating
   mechanism `NOT_ZEROED`? no edge / no route? The first failure returns a `RejectReason`.
2. If rejected, it counts the rejection, logs it with a reason string, pulses `blocker()` for
   one loop, and returns — the running transition (if any) is untouched.
3. If accepted, it plans the route (`planRoute`), records the origin, sets `phase = MOVING`,
   loads the edge's actuation stages, and returns an accepted `TransitionResult`. Still no
   motor has moved.

Every guard is evaluated through `safeGuard(...)`, which catches exceptions and treats a
throwing guard as *blocking* (fails closed) — a CANcoder that fell off the bus cannot take the
robot loop down with it.

### `step()` — advancing, and the moment `current()` is written

The actual motion is driven by `GoalRunner` (section 5) pursuing goals; `step()` *watches*.
Each loop, `advanceTransition(now)`:

- advances through the edge's actuation stages (`stageGatingArrived` — stage N+1 does not
  start until every gating mechanism in stage N reports at-goal);
- once stages are done, checks `isAt(hop)` — is every gating binding of the hop measured at
  goal *right now*?
- if the state declares `settleFor(...)`, requires that to stay true continuously for the
  settle window;
- **only then** executes the single most important line in the file:

```java
// advanceTransition(), the one and only place current is written on a measurement
current = hop;
stateConfirmed = true;
```

If instead the deadline expires first (`now - hopStartSeconds > effectiveTimeout(...)`), it
sets `stateConfirmed = false`, records a `TIMED_OUT`, and leaves `current()` at the last
*proven* state.

### The invariant that makes it debuggable (I1)

> **`current()` is only ever a state whose arrival was proven by every gating binding.**

A timeout, abort, interrupt, or fault sets `stateConfirmed()` to `false` and leaves
`current()` at the last proven value. The machine never claims to be somewhere it merely
*tried* to go. This sounds obvious and is the single most common defect in hand-rolled
superstructure code — including the one this package replaces, where an interrupted transition
set the current state to its target, so the *next* transition planned its route from a state
the robot was not in. Because of I1, `request()` from any state always plans from the truth.

One consequence worth knowing when reading the code: after a timeout the engine keeps pursuing
the goals it already had (`holdGoalsFrom`) rather than snapping back to the origin, so a
timed-out elevator holds where it stopped instead of being driven back down through whatever
it was stuck on. The diagnostics all measure against this held "reference state"
(`referenceState()`), which is why `WaitingOn` stays truthful after a timeout.

---

## 5. Where the goal actually reaches the motor

The engine decides; `GoalRunner` acts. `GoalRunner` is installed as the **default command** of
each bound mechanism's subsystem, so WPILib runs its `execute()` every loop the mechanism is
not owned by something else. Each loop it does this:

```java
G want = core.activeGoalOf(handle);   // what should I be doing right now?
```

`activeGoalOf(handle)` is the one method `GoalRunner` consults, and it is where all the
actuation *policy* lives (so it stays in the tested engine, not smeared across robot code). It
returns the goal for the currently-pursued hop, or `null` when the mechanism is released,
disabled, faulted-released, or parked in a later actuation stage.

When the wanted goal differs from what it is currently pursuing, `GoalRunner` builds the
mechanism's `pursueCommand(goal)` and **hosts** it. Hosting means it calls the inner command's
`initialize()` / `execute()` / `isFinished()` / `end()` **directly**, rather than handing it to
the scheduler. That is legal — those methods are public and the scheduler does nothing else to
a leaf command — but it is only correct with careful bookkeeping: never calling `execute()`
after `isFinished()` returned true, never calling `end()` twice, and swallowing every
exception so nothing escapes into `CommandScheduler.run()` and kills the robot loop. On
arrival it optionally swaps to a `holdCommand(goal)` (used by open-loop mechanisms like a winch
that would otherwise keep driving into a hard stop; closed-loop ones like the elevator return
`null` and keep pursuing, since Motion Magic holding a setpoint *is* the correct hold).

When a driver command interrupts a `GoalRunner`, its `end(interrupted)` calls
`core.noteOwned(key, false)` — that is how `Owned=false` and `Blocker="yielded:elevator"`
appear in the log. When the driver releases, WPILib reschedules the default, `initialize()`
clears the applied goal, and the engine's goal re-applies on the next loop.

---

## 6. Where do I look when X is broken

Every key below is published under `/Catalyst/<prefix>/...` by `CatalystStateMachineLog`
(the `<prefix>` defaults to the machine's name). This table is the fast path from a symptom to
the exact log key and the likely cause.

| Symptom | Read these keys | What it means / likely cause |
|---|---|---|
| **A button does nothing** | `Blocker`, `Rejected/Last`, `Rejected/LastTimestamp`, `Counters/Rejections` | The request was *refused*, not lost. `Rejected/Last` carries the `From->To`, the `RejectReason`, and a detail string. Pair it with the `rejected()` trigger on a rumble so drivers feel refusals. |
| **Refused, but why?** | `Rejected/Last` reason field | `NO_EDGE` (you never declared that edge), `GUARD_BLOCKED` / `INTERLOCK_BLOCKED` / `ENTRY_GUARD_BLOCKED` (a precondition — the detail names it), `NOT_ZEROED` (a gating mechanism has not been homed), `NOT_SEEDED` (you never called `seed(...)`), `FAULTED`, `DISABLED`. |
| **It says it arrived, but the mechanism is not there** | `Bindings/<key>/AtGoal`, `Bindings/<key>/Error`, `Bindings/<key>/Tolerance`, `Bindings/<key>/Observable` | The tolerance is too loose, or `Observable=false` means arrival for that goal is a *settle timer*, not a real sensor — it "arrived" by waiting, not by measuring. Tighten the tolerance or make arrival observable. |
| **It never arrives / it times out** | `WaitingOn`, `BlockerDetail`, `Progress`, `ElapsedSeconds` / `TimeoutSeconds`, `Counters/Timeouts` | `WaitingOn` lists the gating mechanisms still short; `BlockerDetail` gives the live numbers (`elevator err 0.520 m > 0.020 m`) plus any binding `Note`. Cause is usually a mechanism that physically can't reach the goal, an out-of-range/unresolved preset, or a deadline that is simply too short. |
| **A mechanism will not move at all** | `Bindings/<key>/Owned`, `Blocker` (`yielded:<key>`), `Counters/Yields` | `Owned=false` means a *driver command* currently holds that subsystem — the engine has yielded it. Expected during an override; if unexpected, something else set a default command or is holding the requirement. |
| **The whole machine froze** | `Ticks`, `Enabled`, `Phase` | `Ticks` is the heartbeat — if it stopped incrementing, `step()` is not being called (the `Superstructure` was never added as a subsystem, or `periodic()` throws upstream). `Enabled=false` freezes deadlines and issues no goals. |
| **It faulted** | `Faulted`, `FaultReason`, `Phase` (`FAULTED`) | A deadline blew in `strict(true)` mode, or a fault policy escalated. `FaultReason` carries the shortfall. Call `clearFault()` (or the `clearFault()` command) before it will accept requests again. Run a practice match with `strict(false)` to convert faults into warnings while you tune tolerances. |
| **State shows a `?` / unconfirmed** | `StateConfirmed`, `Phase` | `StateConfirmed=false` — the machine knows its last *proven* state but is not currently confirmed there (post-timeout, post-abort, or never seeded). This is invariant I1 doing its job, not a bug. |
| **Config error at build (won't deploy)** | The thrown `StateMachineConfigException` message; also `Graph/Warnings` | `build()` aggregates **every** problem into one exception: undeclared states, duplicate binding keys, unknown presets (with the list of valid ones), out-of-range setpoints, unreachable states, a `RECOVER_TO` with no `recoverTo(...)`, a subsystem with two bindings. Read the numbered list — every line is one fix. |
| **What can I do from here right now?** | `LegalTargets`, `Graph/Edges`, `Graph/Dot` | `LegalTargets` is the live set of targets that would be accepted this instant (guards and interlocks applied). `Graph/Dot` is Graphviz source — paste it into any viewer to see the graph you *actually* declared versus the one you thought you did. |
| **What just happened, in order?** | `Transition/History`, `Transition/Outcome`, `Transition/Trigger`, `Transition/Detail`, `Transition/Arrivals` | Newest-first record of every transition with who triggered it (`op.y`, `auto:leftThree`), the outcome (`ARRIVED` / `TIMED_OUT` / `ABORTED` / `REJECTED` / `SUPERSEDED` / `SEEDED`), and per-mechanism arrival reports. This is your black box. |

Two channels work with **no dashboard at all**, which is exactly when you need them most:
`AlertManager` shows four invariant strings ("Transition timed out", "Transition rejected",
"State is not confirmed", "Superstructure faulted"), and the Driver Station console gets one
non-deduplicated warning per fault and per rejection carrying the full detail with live
numbers.

---

## 7. Reading the code

If you want to read the engine itself, read `StateMachineCore.java` in this order. Following
the data instead of the line numbers makes the ~1830 lines tractable:

1. **`step()`** — the entry point, called once per loop. Short; it is a dispatcher. Read it
   first to see the shape of a tick, then follow what it calls.
2. **`syncGoalApplication(now)`** — the per-binding "has the goal I should pursue changed?"
   bookkeeping that makes elapsed-time arrival tests work identically under a fake clock.
3. **`activeGoalOf(handle)`** — the actuation-policy method `GoalRunner` consults. Read this to
   see how staging, release, disable, and fault-release all funnel into one goal-or-null answer.
4. **`advanceTransition(now)`** — the heart. Stage advance → `isAt(hop)` → settle window → the
   one `current = hop` write → else the timeout branch. If you read one method, read this one.
5. **`isAt(state)`** and the private `Bound.atGoal(...)` — how "arrived" is *measured*, and why
   it stays a fresh measurement rather than a latch (the `secondsSinceApplied` trick).
6. **`finishTransition(...)`** — how a transition ends (arrived, timed out, aborted,
   superseded) and why a non-arrival keeps holding its goals via `holdGoalsFrom`.
7. **`request(target)` → `evaluate(target)` → `planRoute(from, to)`** — the pure decision path.
   `evaluate` is the ordered list of gates; `planRoute` is the deterministic BFS.
8. **`publishTimeline(now)`** and `publishBindings(now)` — the change-detected publishing, so
   you understand why most keys only write when they change and `BlockerDetail` throttles to
   5 Hz.

Then read `GoalRunner.execute()` for the other side of the seam, and `Binding.java` /
`LinearBinding.java` for how one mechanism type implements arrival and pursuit. After that the
remaining 40-odd files are just the typed values these methods pass around.
