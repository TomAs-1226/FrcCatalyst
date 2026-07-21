---
layout: default
title: State Machine
parent: Advanced
nav_order: 3
---

# State Machine
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Why this exists

Catalyst has had a superstructure coordinator since early versions, and it only ever
understood two mechanism types: `LinearMechanism` and `RotationalMechanism`. If your
robot had a shooter, a claw, a turret, a winch or a solenoid — which is most robots —
those mechanisms simply could not be part of a coordinated state. So teams wrote their
own state machine every season, and every season it was a fresh pile of booleans with no
log to read when it misbehaved. That is
[issue #22](https://github.com/TomAs-1226/FrcCatalyst/issues/22), and it is a fair
complaint: a state machine that only understands half your robot is a preset applier with
a nicer name.

Version 1.2.0 adds `frc.lib.catalyst.statemachine`. It accepts all nine Catalyst
mechanism types plus any subsystem you wrote yourself, it enforces a legal-transition
graph where an edge you did not declare is a transition the robot will not make, it
proves arrival from sensors instead of assuming it, and it publishes a complete
structured log including a plain-English string naming which mechanism is holding
things up. The old `SuperstructureCoordinator` is deprecated but frozen, not removed —
robot code built on it keeps working, unchanged, with no deprecation-driven rewrite
required.

---

## Which should I use

| Your situation | Use | Why |
|---|---|---|
| New robot, starting fresh | `Superstructure` | Every mechanism type, a real graph, the full log. This page. |
| Existing robot already on `SuperstructureCoordinator` | Keep it | It still works and its behaviour is frozen. Migrate when you have a reason, not because of a deprecation warning. |
| Robot built on the goal layer (`GoalDirector`) | `GoalDirector.builder().superstructure(sm)` | One-line swap from `.coordinator(...)`. Goals still name their state as a `String`, so the two are interchangeable — and readiness becomes truthful, because `isAtState` now means measured, confirmed arrival. |

{: .note }
`Superstructure` implements `SuperstructureLike`, and so does the old
`SuperstructureCoordinator`. That shared interface is what makes the `GoalDirector` swap
a one-word change.

---

## Quick start

The smallest machine worth building: two mechanisms, three states. Start here and add
pieces as you need them.

```java
public enum ArmState { STOW, INTAKE, SCORE }

private final Superstructure<ArmState> superstructure;

public RobotContainer() {
    var b = Superstructure.builder(ArmState.class, "Superstructure");

    Handle<LinearGoal>     elevator = b.bind("elevator", Mechanisms.linear(elevatorMech));
    Handle<RotationalGoal> arm      = b.bind("arm",      Mechanisms.rotational(armMech));

    superstructure = b
        .state(ArmState.STOW, s -> s
            .set(elevator, LinearGoal.meters(0.0))
            .set(arm,      RotationalGoal.degrees(0)))

        .state(ArmState.INTAKE, s -> s
            .set(elevator, LinearGoal.meters(0.05))
            .set(arm,      RotationalGoal.degrees(-20)))

        .state(ArmState.SCORE, s -> s
            .set(elevator, LinearGoal.meters(0.90))
            .set(arm,      RotationalGoal.degrees(95)))

        .hub(ArmState.STOW)     // STOW connects to and from everything else
        .build();
}
```

Three more lines make it drivable:

```java
// Tell the machine where the robot is physically built. Once, at init.
superstructure.engine().seed(ArmState.STOW);

operator.a().onTrue(superstructure.goTo(ArmState.INTAKE, "op.a"));
operator.b().onTrue(superstructure.goTo(ArmState.SCORE,  "op.b"));
```

That is a working state machine. Note what you did **not** have to write: no
`setDefaultCommand` for either mechanism (the builder installs one per bound mechanism),
no periodic call (`Superstructure` extends `SubsystemBase` and steps the engine in
`periodic()`), and no telemetry (it is publishing the full schema under
`/Catalyst/Superstructure/` already).

{: .warning }
`seed(...)` is not optional. Before it is called the machine has never confirmed a state,
so it does not know where a route starts from, and every request is refused with
`NOT_SEEDED`. This is deliberate — guessing the starting state is how a robot moves an
arm through something on the very first press of the match.

### Growing it

Add these in roughly this order as the robot gets real:

- `.defaults(...)` — the safe posture, stated once, inherited by every state.
- `.allow(...)` / `.allowBoth(...)` instead of `.hub(...)`, once some transitions are genuinely illegal.
- `.edge(A, B, e -> e.stage(...))` — when two mechanisms would collide if they moved together.
- `.entryGuard(...)` — when a state only makes sense under a condition.
- `.settleFor(...)` — when a mechanism clips through tolerance on the way past.

---

## The full worked example

This is a nine-mechanism robot: elevator, arm, wrist, claw, intake roller, shooter
flywheel, turret, climber winch and a pneumatic funnel. It is the example from the
`Superstructure` class Javadoc.

```java
public enum SuperState { STOW, INTAKE, CARRY, AIM, SCORE, CLIMB }

var b = Superstructure.builder(SuperState.class, "Superstructure");

var elevator = b.bind("elevator", Mechanisms.linear(elevatorMech));
var arm      = b.bind("arm",      Mechanisms.rotational(armMech));
var wrist    = b.bind("wrist",    Mechanisms.rotational(wristMech));
var claw     = b.bind("claw",     Mechanisms.claw(clawMech));
var intake   = b.bind("intake",   Mechanisms.roller(intakeMech));
var shooter  = b.bind("shooter",  Mechanisms.flywheel(shooterMech));
var turret   = b.bind("turret",   Mechanisms.turret(turretMech, () -> drive.getPose().getRotation().getDegrees()));
var climber  = b.bind("climber",  Mechanisms.winch(climberMech));
var funnel   = b.bind("funnel",   Mechanisms.pneumatic(funnelMech));

superstructure = b
    // The safe posture, stated once. Every state inherits it unless it says otherwise,
    // so the mechanism you forget in one state does not stay where the last state parked it.
    .defaults(s -> s
        .set(claw,    ClawGoal.hold())
        .set(intake,  RollerGoal.idle())
        .set(shooter, FlywheelGoal.idle())
        .set(turret,  TurretGoal.forward())
        .set(climber, WinchGoal.stop())
        .set(funnel,  PneumaticGoal.retracted()))

    .state(SuperState.STOW, s -> s
        .set(elevator, LinearGoal.meters(0.0))
        .set(arm,      RotationalGoal.degrees(0))
        .set(wrist,    RotationalGoal.degrees(0)))

    .state(SuperState.INTAKE, s -> s
        .set(elevator, LinearGoal.meters(0.05))
        .set(arm,      RotationalGoal.degrees(-20))
        .set(wrist,    RotationalGoal.degrees(15))
        .set(intake,   RollerGoal.intakeUntilPiece(3.0))
        .set(claw,     ClawGoal.open()))

    .state(SuperState.CARRY, s -> s
        .set(elevator, LinearGoal.meters(0.15))
        .set(arm,      RotationalGoal.degrees(10))
        .set(wrist,    RotationalGoal.degrees(0))
        .set(claw,     ClawGoal.grip(1.5)))

    .state(SuperState.AIM, s -> s
        .set(elevator, LinearGoal.meters(0.30))
        .set(arm,      RotationalGoal.degrees(35))
        .set(wrist,    RotationalGoal.degrees(-10))
        .set(shooter,  FlywheelGoal.rpm(4200))
        .set(turret,   TurretGoal.at(drive::getPose, SPEAKER, "speaker"))
        .entryGuard(clawMech::hasPiece, "no piece"))

    .state(SuperState.SCORE, s -> s
        .set(elevator, LinearGoal.meters(0.30))
        .set(arm,      RotationalGoal.degrees(35))
        .set(shooter,  FlywheelGoal.rpm(4200))
        .set(claw,     ClawGoal.open())
        .settleFor(0.2))

    .state(SuperState.CLIMB, s -> s
        .set(elevator, LinearGoal.meters(0.0))
        .set(arm,      RotationalGoal.degrees(0))
        .set(climber,  WinchGoal.extend())
        .release(turret)                     // the driver owns the turret while climbing
        .entryGuard(() -> RobotState.matchTimeRemaining() < 30.0, "not endgame"))

    .hub(SuperState.STOW)                    // STOW connects to everything
    .allowBoth(SuperState.CARRY, SuperState.AIM)
    .allow(SuperState.AIM, SuperState.SCORE)
    .allow(SuperState.INTAKE, SuperState.CARRY)

    // Raise the elevator BEFORE the arm swings out, or the arm hits the chassis.
    .edge(SuperState.STOW, SuperState.AIM, e -> e.stage(elevator).stage(arm, wrist))

    // Once the climber is out, nothing but CLIMB and STOW is safe.
    .interlock("climberStowed", () -> !climberMech.isFullyExtended(),
               s -> s != SuperState.CLIMB && s != SuperState.STOW)

    .build();

// in robotInit, tell it where the robot is physically built:
superstructure.engine().seed(SuperState.STOW);

// bindings
operator.a().onTrue(superstructure.goTo(SuperState.INTAKE, "op.a"));
operator.y().onTrue(superstructure.goTo(SuperState.AIM,    "op.y"));
superstructure.arrivedAt(SuperState.CARRY).onTrue(leds.flash(Color.kGreen));
```

Nine mechanisms and six states in about forty lines, and every one of those forty lines
says something a person reading the code needs to know. That is what `.defaults(...)`
buys: without it, six states times nine mechanisms is fifty-four assignments, most of
them repeated, and the one you forget is the bug.

{: .note }
`SPEAKER` here is a `Translation2d` your team defines — `TurretGoal.at` takes
`(Supplier<Pose2d> robotPose, Translation2d target, String label)`. The label is what
appears in the log, so keep it stable and short.

---

## Every mechanism type

All nine Catalyst mechanisms have a typed factory in `Mechanisms` and a matching goal
type. The factories are thin — each is a one-line delegation to a public binding
constructor, so if you want the binding's behaviour without this naming you can construct
`LinearBinding` and friends directly and lose nothing.

Every factory has an overload without a key, which uses `mechanism.getMechanismName()` —
the same string the mechanism already logs under, so binding telemetry lands next to
mechanism telemetry in a log viewer.

| Mechanism | Goal type | Bind it | Use it in a state |
|---|---|---|---|
| `LinearMechanism` | `LinearGoal` | `Mechanisms.linear("elevator", elevatorMech)` | `.set(elevator, LinearGoal.meters(0.90))` |
| `RotationalMechanism` | `RotationalGoal` | `Mechanisms.rotational("arm", armMech)` | `.set(arm, RotationalGoal.degrees(95))` |
| `DifferentialWristMechanism` | `WristGoal` | `Mechanisms.wrist("wrist", wristMech)` | `.set(wrist, WristGoal.of(15.0, 0.0))` |
| `FlywheelMechanism` | `FlywheelGoal` | `Mechanisms.flywheel("shooter", shooterMech)` | `.set(shooter, FlywheelGoal.rpm(4200))` |
| `TurretMechanism` | `TurretGoal` | `Mechanisms.turret("turret", turretMech, headingDeg)` | `.set(turret, TurretGoal.robotRelative(0))` |
| `ClawMechanism` | `ClawGoal` | `Mechanisms.claw("claw", clawMech)` | `.set(claw, ClawGoal.grip(1.5))` |
| `RollerMechanism` | `RollerGoal` | `Mechanisms.roller("intake", intakeMech)` | `.set(intake, RollerGoal.intakeUntilPiece(3.0))` |
| `WinchMechanism` | `WinchGoal` | `Mechanisms.winch("climber", climberMech)` | `.set(climber, WinchGoal.extend())` |
| `PneumaticMechanism` | `PneumaticGoal` | `Mechanisms.pneumatic("funnel", funnelMech)` | `.set(funnel, PneumaticGoal.extended())` |

The goal types are records or sealed interfaces of records, so `Handle<LinearGoal>` will
not accept a `RotationalGoal` — passing the wrong goal to the wrong mechanism is a
compile error, not a match.

### Goal constructors worth knowing

```java
LinearGoal.meters(0.9);                    LinearGoal.meters(0.9, 0.01);   // explicit tolerance
LinearGoal.preset("L4");                   // resolved against the mechanism's named positions

RotationalGoal.degrees(95);                RotationalGoal.degrees(95, 1.5);
RotationalGoal.preset(ArmPos.SCORE);       // any enum

WristGoal.of(15.0, 0.0);                   WristGoal.level();              // pitch, roll

FlywheelGoal.rpm(4200);                    FlywheelGoal.rps(70);
FlywheelGoal.rpm(4200, 3800);              // primary, secondary
FlywheelGoal.idle();
FlywheelGoal.track("dist", () -> table.get(range));   // live setpoint, stable label

TurretGoal.forward();                      TurretGoal.robotRelative(30);
TurretGoal.at(drive::getPose, SPEAKER, "speaker");
TurretGoal.hold();

ClawGoal.open();  ClawGoal.close();  ClawGoal.grip(1.5);  ClawGoal.hold();  ClawGoal.idle();

RollerGoal.intakeUntilPiece(3.0);  RollerGoal.intakeContinuous();
RollerGoal.eject(0.5);             RollerGoal.idle();

WinchGoal.extend();  WinchGoal.retract();  WinchGoal.stop();  WinchGoal.speed(0.6, 1.0);

PneumaticGoal.extended();  PneumaticGoal.retracted();  PneumaticGoal.off();
```

{: .tip }
A few factories take an extra argument that turns a match-day mystery into a laptop-side
build error. `Mechanisms.roller(key, mech, RollerBinding.PieceDetection.ABSENT)` fails
the build if any state asks that roller to `intakeUntilPiece`, because a roller with no
beam break and no stall detection can only ever time out.
`Mechanisms.turret(mech, headingDeg, minDeg, maxDeg)` lets the binding notice that a
requested angle was *clamped* — `TurretMechanism.atSetpoint()` reports true once the
turret reaches the clamped angle, which means it reports true while pointing somewhere
the shot misses from.

---

## Custom mechanisms

The engine knows nothing about the nine typed bindings. It knows only `Actuator`. So your
own swerve wrapper, LED controller, vendor-SDK climber or odd-shaped subsystem reaches
exactly the same fidelity — arrival gating, blocker reporting, re-assertion, build-time
validation — through one of four tiers, chosen by how much the subsystem actually has to
say.

### Tier 1 — `instant`: fire and forget

For anything whose effect is instantaneous and whose completion there is nothing to wait
for. `atGoal` is hard-wired true, so a state carrying only this binding completes on the
loop it is entered.

```java
Actuator<LedPattern> leds =
    Mechanisms.instant("Leds", ledSubsystem::setPattern, ledSubsystem);
```

### Tier 2 — `custom`: a method call and a real sensor test

The simplest binding that can actually gate a transition. `apply` runs every loop while
the goal is active — the right default for a setter that writes a setpoint into a
controller.

```java
Actuator<HoodPreset> hood =
    Mechanisms.custom("Hood",
        want -> hoodSubsystem.setAngleDeg(want.degrees()),
        want -> Math.abs(hoodSubsystem.getAngleDeg() - want.degrees()) < 1.0,
        hoodSubsystem);
```

### Tier 3 — `commands`: the subsystem already speaks Commands

The natural tier for a subsystem written in ordinary WPILib style, where the public
surface is `Command stepCommand(ClimbStep)` rather than `void setStep(...)`.

```java
Actuator<ClimbStep> climber =
    Mechanisms.commands("Climber",
        climberSubsystem::stepCommand,
        climberSubsystem::stepComplete,
        climberSubsystem);
```

{: .warning }
The factory **must return a fresh command instance on every call**. The state machine
hosts commands rather than scheduling them, and it will initialize them many times over a
match — re-entering the state, re-asserting after a command ended early, recovering from
a driver override. A factory that caches and returns the same instance produces a command
initialized twice without being ended in between, which in WPILib is undefined behaviour
and in practice is a mechanism that stops responding partway through a match. A method
reference to a `Commands.run(...)` factory is fresh; a field holding a command is not.
For the same reason the factory must not be a `goToAndWait`-style composition: arrival is
decided by `atGoal`, never by a command finishing, and a command that ends on arrival
simply stops driving, so the mechanism sags.

### Tier 4 — `build`: everything

Everything the nine typed bindings publish is reachable here. Nothing is reserved for
library mechanisms.

```java
Actuator<ShotGoal> shooter =
    Mechanisms.<ShotGoal>build("Shooter", shooterSubsystem)
        .kind("flywheel")
        .unit("rps")
        .pursue(goal -> shooterSubsystem.spinUpCommand(goal.rps()))
        .hold(goal -> shooterSubsystem.maintainCommand(goal.rps()))
        .atGoal(goal -> Math.abs(shooterSubsystem.getRps() - goal.rps()) < 2.0)
        .measured(shooterSubsystem::getRps)
        .error(goal -> shooterSubsystem.getRps() - goal.rps())
        .tolerance(2.0)
        .range(0.0, 90.0, ShotGoal::rps)
        .label(ShotGoal::name)
        .note(goal -> shooterSubsystem.isStalled() ? "stalled" : "")
        .reassertEvery(25)
        .done();
```

What tier 4 buys over tier 2: `measured` and `error` make the shooter plottable.
`tolerance` makes the plot readable, because the band is drawn next to the trace. `range`
turns an impossible setpoint into a build failure on a laptop instead of a mechanism that
silently never arrives in a match. `note` is what appears in `BlockerDetail` when a state
refuses to complete — the difference between "the robot is stuck" and "the shooter is
stalled".

Other builder methods: `settle(seconds)` when arrival genuinely is just a timer (it also
flips `Observable` to false, so nobody reading a log mistakes a timer for a sensor),
`observable(Predicate)` to override that, `zeroed(BooleanSupplier)` so an unhomed
mechanism produces a clean `NOT_ZEROED` rejection instead of being commanded in the wrong
frame, and `onRelease(Runnable)` to stop an open-loop output when the machine hands the
mechanism back.

{: .warning }
Write the type argument explicitly: `Mechanisms.<ShotGoal>build("Shooter", sub)`. This is
a requirement, not a style preference. `G` does not appear in `build`'s parameter list,
and Java resolves a call's type arguments from that call alone — the assignment on the
far side of `done()` is too late to help. A bare `build("Shooter", sub)` infers `G` as
`Object`, and the failure surfaces one call later as "cannot find symbol" on whatever the
first lambda reads off its goal. The three shorthand factories return `Actuator<G>`
directly, with no chain in between, so they infer normally.

### The one rule a goal type cannot break

Whatever `G` you choose **must have value-based `equals` and `hashCode`** — a record, an
enum, or a sealed hierarchy of records, and never an array. The engine compares the
wanted goal against the active one with `Objects.equals` every loop to decide whether to
rebuild actuation. An identity-equality goal type would rebuild its command fifty times a
second, forever. This is not enforceable at compile time, which is why it is stated this
loudly.

---

## The transition graph

An edge you did not declare is a transition the robot will not make. That is the whole
point: the graph is a safety constraint, not a hint.

| Call | Meaning |
|---|---|
| `.allow(FROM, TO, MORE...)` | One-way edges from `FROM` to each listed state |
| `.allowBoth(A, B)` | Edges in both directions |
| `.hub(H)` | Edges from every state to `H` **and** from `H` to every state |
| `.edge(FROM, TO, e -> ...)` | Declares the edge *and* configures its guard, deadline, cost and staging |
| `.via(FROM, TO, W1, W2...)` | Pins the route for `FROM -> TO` through named waypoints instead of trusting search |
| `.allowUnreachable(S...)` | Suppresses the "state is unreachable" build error for these states |

`.edge(...)` declares the edge itself, so you do not need a matching `.allow(...)`.

### DIRECT_ONLY vs SHORTEST_PATH

`Routing.DIRECT_ONLY` is the default. A request is legal only if a declared edge connects
the current state to the target; anything else is a hard `NO_EDGE` rejection. If you
never declared `CLIMB -> SCORE`, the robot will not do it, no matter what else is
reachable.

`Routing.SHORTEST_PATH` routes through waypoints automatically. If `STOW -> SCORE` is not
declared but `STOW -> MID -> SCORE` is, the machine visits `MID` on the way and **confirms
arrival there** before continuing. Ergonomically this is the "press Y from anywhere" mode.

```java
b.routing(Routing.SHORTEST_PATH);
```

Search is deterministic — breadth-first, lowest enum ordinal first — so a route never
changes without a graph change. To keep that visible, enabling this publishes *every*
multi-hop route the graph can produce to `Graph/Routes` and into `validate().warnings()`.

{: .warning }
`hub()` creates **real edges**, in both directions, between the hub and every other
state. Under `DIRECT_ONLY` that is exactly what you want for a `STOW` state. Under
`SHORTEST_PATH` it also makes almost everything reachable in two hops — `CLIMB -> STOW ->
SCORE` becomes a legal automatic route even though you never intended `CLIMB` to lead
anywhere but `STOW`. If you use both together, read `Graph/Routes` and the
`validate()` warnings before you believe the graph you think you declared. Use
`allow`/`allowBoth` explicitly when the hub is too permissive.

`via(...)` is the escape hatch when search picks a legal-but-wrong path: pin the route
and the machine stops guessing.

---

## Guards, entry guards and interlocks

Three kinds of precondition, each with a short reason string that lands in the log when it
blocks something. Keep the reason a noun, not a sentence — it becomes
`Blocker = "reject:GUARD_BLOCKED"` with the noun in `BlockerDetail`, and a driver reads
it under pressure.

**Edge guard** — this specific transition is allowed only while the condition holds. Also
gates breadth-first routing: a blocked edge is not passable.

```java
.edge(SuperState.STOW, SuperState.CLIMB, e -> e
    .guard(() -> RobotState.matchTimeRemaining() < 30.0, "endgame"))
```

Rejected with `RejectReason.GUARD_BLOCKED`.

**Entry guard** — this *state* cannot be entered unless the condition holds, whatever
edge you came in on. Evaluated at request time.

```java
.state(SuperState.AIM, s -> s
    .set(shooter, FlywheelGoal.rpm(4200))
    .entryGuard(clawMech::hasPiece, "no piece"))
```

Rejected with `RejectReason.ENTRY_GUARD_BLOCKED`.

**Interlock** — a global precondition, named positively: `satisfied` returning true means
it is safe to proceed. The predicate says which states it blocks.

```java
.interlock("climberStowed",
           () -> !climberMech.isFullyExtended(),        // satisfied when the climber is in
           s -> s != SuperState.CLIMB && s != SuperState.STOW)   // blocks everything else
```

"The climber is deployed, so nothing but CLIMB and STOW is allowed" is one line here and
is inexpressible as a per-edge guard without touching every edge. Rejected with
`RejectReason.INTERLOCK_BLOCKED`, and the detail carries the interlock's name.

### Where they show up

Every rejection is logged, once, at the moment it happens:

- `Blocker` becomes `reject:GUARD_BLOCKED` (or the matching reason) for exactly one loop.
- `BlockerDetail` carries the full result, including your reason string.
- `Rejected/Last` holds the last rejection text and `Rejected/LastTimestamp` its clock reading.
- A `TransitionRecord` with `Outcome.REJECTED` is appended to `Transition/History`.
- `Counters/Rejections` increments.
- A one-shot Driver Station warning is printed: `[Superstructure] refused CARRY->AIM: ENTRY_GUARD_BLOCKED — no piece`.
- The `rejected()` trigger pulses for one loop.

That last one is worth binding. The single worst failure mode of a superstructure state
machine is a button that silently does nothing — without a reason code, a refused request
and a broken binding look identical from the driver's seat.

```java
superstructure.rejected().onTrue(driverRumble.pulse());
```

The full set of reasons is `RejectReason`: `NONE`, `UNKNOWN_STATE`, `NO_EDGE`,
`NO_ROUTE`, `GUARD_BLOCKED`, `INTERLOCK_BLOCKED`, `ENTRY_GUARD_BLOCKED`, `NOT_ZEROED`,
`ALREADY_THERE`, `BUSY`, `FAULTED`, `DISABLED`, `NOT_SEEDED`.

---

## Staged actuation

By default every mechanism in a state starts moving at once. When two mechanisms would
collide if they did, declare the order:

```java
.edge(SuperState.STOW, SuperState.AIM, e -> e
    .stage(elevator)        // raise first, so the arm clears the chassis
    .stage(arm, wrist))     // then deploy arm and wrist together
```

Stage N is applied only once every gating binding in stage N-1 reports at-goal. Any
handle never named in a stage moves immediately, in parallel with stage 0 — so you only
have to name the mechanisms whose ordering actually matters.

This replaces the old `addTransitionRule(...)`, where you handed the library a hand-built
command group whose sequencing nothing could inspect. Staging is declarative: the machine
knows the order, publishes the current stage as `Stage`, and reports `stage 1/2` in
`Summary`, so what the robot will do is readable before it does it.

`EdgeSpec` also carries `guard`, `timeout(seconds)`, `cost(weight)` for `SHORTEST_PATH`
routing, and `onTransit(Runnable)` which runs once when the edge is accepted and begins.

---

## Arrival, deadlines and faults

### current() vs stateConfirmed()

This is the invariant the package exists for.

`current()` is **only ever** a state whose arrival was proven by every gating mechanism
reporting at-goal. A timeout, an abort, an interrupt or a fault sets `stateConfirmed()` to
false and leaves `current()` at the last *proven* value. The machine never claims to be
somewhere it merely tried to go.

Concretely: the robot is confirmed at `MID`, you press the button for `HIGH`, the elevator
jams, the deadline blows. `current()` is still `MID`. `stateConfirmed()` is false. When
you clear the fault and press again, the machine plans from `MID` — the truth — rather
than from `HIGH`, which is where it never went.

An unconfirmed belief does not brick the machine. It still knows its last proven state, so
a retry is legal. Only a machine that has never been seeded genuinely does not know where
to route from.

Related queries:

| Call | Means |
|---|---|
| `current()` | Last *proven* state |
| `stateConfirmed()` | Is that belief currently proven? |
| `isAt(state)` | Measured right now: is every gating mechanism at that state's goal? |
| `isSettledAt(state)` | `current() == state` **and** confirmed **and** still measuring true |
| `progress()` | Fraction of the active hop's gating mechanisms at goal, in `[0,1]` |

`isAt` is a measurement, not a latch. Shove the arm out of tolerance and `isAt` goes
false immediately, while `current()` stays put — the last proven state does not change
just because the robot drifted.

### Deadlines

Every hop has a deadline: `defaultTimeout(seconds)` on the builder (default 4.0 s),
overridden by `state(...).timeout(...)`, overridden in turn by `edge(...).timeout(...)`.

Deadline accounting is frozen while the robot is disabled, so a long wait between
autonomous and teleop does not blow a timer for a robot that physically could not move.

### Fault policies

`FaultPolicy` decides what happens when a deadline blows in strict mode.

| Policy | Behaviour |
|---|---|
| `HOLD_AND_REPORT` | **Default.** Every binding keeps pursuing its current goal, motors keep holding, the state is marked unconfirmed and the log shouts. Nothing is cut. |
| `RELEASE_ALL` | Every binding is released, each `GoalRunner` idles, and the subsystem is free for a driver command. For a stuck mechanism better left limp. |
| `RECOVER_TO` | Immediately request a declared recovery state. Never the default — unexpected motion after a fault is startling. A fault *during* recovery escalates to `HOLD_AND_REPORT` rather than recursing. |

Set it per state with `.onFault(policy)`, or with the shorthand `.recoverTo(STOW)`, or
globally with `defaultFaultPolicy(...)`.

The default is deliberately the least dramatic option. Honest arrival detection turns
robots that "worked" only because nothing was checking into robots that fault — so on a
blown deadline the mechanisms keep holding and the robot behaves roughly as it did
before. The difference is that the log now says so.

A latched fault refuses all requests with `RejectReason.FAULTED` until `clearFault()`.

### strict(false) — the migration mode

```java
b.strict(false);
```

With `strict(false)`, deadlines still fire, still count and still log — they just
downgrade from a fault to a warning, and `Phase.TIMED_OUT` becomes `Phase.HOLDING`.

**The migration workflow:**

1. Build the machine with `strict(false)` and reasonable-looking tolerances.
2. Run a full practice match. Drive it hard, the way you actually would.
3. Read `Transition/History`. Every `TIMED_OUT` entry names the mechanism that missed and
   its error at the moment the deadline expired, and every entry's `Arrivals` column gives
   per-mechanism arrival times.
4. Fix what it names. Usually this is two or three tolerances that were tighter than the
   mechanism can hold, or one deadline that was shorter than the mechanism physically
   takes. `ArrivalReport` exists exactly for this: if the arm arrives in 0.88 s, the wrist
   in 1.02 s and the elevator in 1.31 s, the elevator is the only one worth tuning.
5. Turn `strict` back on.

{: .warning }
Do not ship a competition robot with `strict(false)` and call the job done. It is a
diagnostic mode: it tells you the truth and then declines to act on it. Once the
tolerances are honest, strict mode is what stops the robot from continuing an autonomous
routine on the assumption it reached a state it never reached.

---

## The log

Everything goes through `CatalystLog`, so a team that swaps in a WPILOG or AdvantageKit
sink gets the state machine's telemetry along with everything else, free. Live keys land
at `/Catalyst/<prefix>/`, where `<prefix>` defaults to the machine name you passed to
`builder(...)` and can be changed with `.logPrefix(...)`.

### Key reference

**Graph** — published once, at build.

| Key | Type | Contents |
|---|---|---|
| `Graph/Machine` | string | The machine name |
| `Graph/States` | string[] | Every enum constant, in ordinal order |
| `Graph/Edges` | string[] | `FROM->TO\|cost=1.0\|guard=endgame` per declared edge |
| `Graph/Bindings` | string[] | `key\|kind\|unit\|goal=<GoalType>\|advisory=false` per binding |
| `Graph/Routes` | string[] | Multi-hop routes, populated only under `SHORTEST_PATH` |
| `Graph/Dot` | string | Graphviz DOT source — paste it into any viewer to see your graph as a picture |
| `Graph/Warnings` | string[] | Validation warnings; each is also printed to the Driver Station |

**State and phase** — published on change.

| Key | Type | Contents |
|---|---|---|
| `State` | string | Last proven state |
| `StateOrdinal` | int | Same, numeric, for plotting |
| `StateConfirmed` | boolean | Is that belief proven right now |
| `Target` | string | Requested destination |
| `NextHop` | string | Immediate next state on the route |
| `Phase` | string | `IDLE`, `MOVING`, `SETTLING`, `HOLDING`, `REJECTED`, `TIMED_OUT`, `FAULTED`, `DISABLED` |
| `PhaseOrdinal` | int | Same, numeric |
| `Stage` | int | Active actuation stage index |
| `Transitioning` | boolean | True while `MOVING` or `SETTLING` |

**Diagnosis** — `Blocker` and `Summary` on change; `BlockerDetail` throttled to 5 Hz.

| Key | Type | Contents |
|---|---|---|
| `Blocker` | string | Stable, low-cardinality: `waiting:elevator,arm`, `yielded:elevator`, `reject:NO_EDGE`, `fault`, or empty |
| `BlockerDetail` | string | The same diagnosis with live numbers, plus any binding `note` |
| `Summary` | string | One human line: `MOVING STOW->AIM stage 1/2 2.1/4.0s waiting:elevator` |
| `WaitingOn` | string[] | Keys of gating mechanisms not yet at goal, sorted |

**Progress**

| Key | Type | Contents |
|---|---|---|
| `Progress` | double | Fraction of the active hop's gating mechanisms at goal |
| `ElapsedSeconds` | double | Time on this hop |
| `TimeoutSeconds` | double | This hop's deadline |
| `Route` | string | Rendered route for the in-flight request |
| `Trigger` | string | Who asked — the `triggerSource` you passed to `goTo` |

**Per mechanism**, under `Bindings/<key>/`.

| Key | Type | Contents |
|---|---|---|
| `Goal` | string | Low-cardinality goal label |
| `GoalDetail` | string | Goal with live values interpolated |
| `Measured` | double | Live measurement, or `NaN` |
| `Error` | double | Signed error toward the goal, or `NaN` |
| `Tolerance` | double | Tolerance band, or `NaN` |
| `AtGoal` | boolean | Is this mechanism there right now |
| `Owned` | boolean | Does the state machine currently own it (see driver override) |
| `Observable` | boolean | `false` when arrival is a settle timer rather than a sensor |
| `Gating` | boolean | Does the current state wait on this mechanism |
| `ArrivalSeconds` | double | Seconds from goal application to arrival |
| `Note` | string | Free-form runtime note — `"stalled"`, `"low pressure"` |

**Transitions** — one write per completed attempt, including rejections.

| Key | Type | Contents |
|---|---|---|
| `Transition/Seq` | int | Monotonic sequence number |
| `Transition/From` / `To` | string | Origin and requested target |
| `Transition/Route` | string | `MID>SCORE_HIGH`, or empty when direct |
| `Transition/Trigger` | string | Who asked |
| `Transition/Outcome` | string | `ARRIVED`, `REJECTED`, `TIMED_OUT`, `ABORTED`, `SUPERSEDED`, `FAULTED`, `SEEDED` |
| `Transition/OutcomeOrdinal` | int | Same, numeric |
| `Transition/Reason` | string | A `RejectReason`, or `NONE` |
| `Transition/Detail` | string | Human detail naming the culprit |
| `Transition/DurationSeconds` | double | Wall time of the attempt |
| `Transition/Arrivals` | string[] | Per-mechanism report, 9 pipe-delimited columns |
| `Transition/History` | string[] | Newest-first ring of the last 50 records, 10 columns each |
| `Rejected/Last` | string | Text of the most recent rejection |
| `Rejected/LastTimestamp` | double | Its clock reading |

`Transition/Arrivals` columns:
`key|kind|goalLabel|arrived|observable|error|tolerance|unit|arrivalSeconds`

`Transition/History` columns:
`timestamp|seq|from|to|route|trigger|outcome|reason|duration|detail`

Pipes inside any field are replaced with underscores, so the line always splits into
exactly that many columns and a dashboard table parser can rely on the shape. History
capacity is 50 by default; change it with `.historyCapacity(n)`.

**Everything else**

| Key | Type | Contents |
|---|---|---|
| `LegalTargets` | string[] | States that would be accepted right now |
| `Counters/Transitions` | int | Completed transitions |
| `Counters/Rejections` | int | Refused requests |
| `Counters/Timeouts` | int | Blown deadlines |
| `Counters/Aborts` | int | Cancelled transitions |
| `Counters/Yields` | int | Times a mechanism was taken over by another command |
| `Ticks` | int | Engine steps |
| `UptimeSeconds` | double | Since build |
| `Enabled` | boolean | Robot enabled state as the engine sees it |
| `Faulted` | boolean | Is a fault latched |
| `FaultReason` | string | Why |

### The three field-visible channels

This is the "no laptop at a regional" contract.

1. **NetworkTables / WPILOG** — the full schema above, for AdvantageScope and the pit display.
2. **`AlertManager`** — exactly four alert strings, all invariant: `Transition timed out`,
   `Transition rejected`, `State is not confirmed`, `Superstructure faulted`. They carry no
   live numbers on purpose, because `AlertManager` keys messages by their composed text and
   clears by exact match — a number in an alert makes it impossible to clear.
3. **Driver Station console** — one non-deduplicated warning per fault and per rejection,
   carrying the full summary with mechanism names and numbers. This is the channel that
   needs no dashboard setup whatsoever, which is exactly when you need it most. Turn it off
   with `.driverStationMessages(false)`.

One drag onto Shuffleboard or Elastic gives you a working pit display:

```java
superstructure.addToDashboard("Pit");   // State, Confirmed, Phase, Blocker, Summary, Progress, Faulted
```

### Debugging a stuck superstructure

The operator presses Y for `AIM` and the robot does not get there. Four keys, in order.

**1. `Blocker`** tells you the category in one glance.

```
Blocker = "waiting:elevator"
```

Not a rejection, not a fault, not an override — a gating mechanism has not arrived. If it
had read `reject:ENTRY_GUARD_BLOCKED` you would be looking at a precondition, not a
mechanism; if `yielded:elevator`, the driver is holding a button that took the elevator
away.

**2. `BlockerDetail`** turns the category into a number.

```
BlockerDetail = "elevator err 0.520 m > 0.020 m"
```

The elevator is half a metre from where it was asked to go, against a 2 cm tolerance.
That is not a tuning problem — that is a mechanism that did not move. Compare against a
detail like `elevator err 0.024 m > 0.020 m`, which *is* a tuning problem: it is basically
there and the band is too tight.

If a binding has a `note`, it is appended here. `elevator err 0.520 m > 0.020 m; shooter
stalled` immediately narrows the search.

**3. `WaitingOn`** confirms whether it is one mechanism or all of them.

```
WaitingOn = ["elevator"]
```

One entry means one mechanism. If it read `["arm", "elevator", "wrist"]` with everything
far from goal, suspect something upstream of any individual mechanism — a brownout, a
disabled robot, a CAN bus fault — rather than the elevator.

**4. `Transition/History`** tells you whether this is new.

```
128.412|7|CARRY|AIM||op.y|TIMED_OUT|NONE|4.000|elevator err 0.520 m > 0.020 m
124.108|6|AIM|CARRY||op.b|ARRIVED|NONE|0.912|
119.660|5|CARRY|AIM||op.y|ARRIVED|NONE|1.204|
```

Sequence 5 made this exact trip in 1.2 s. Sequence 7 timed out at the 4 s deadline with
the elevator half a metre short. Something changed between 119 s and 128 s of the match —
a broken chain, a tripped breaker, a mechanism that hit something. This is not a
configuration problem, and no amount of tolerance tuning will fix it.

Now cross-check `Transition/Arrivals` for sequence 7:

```
elevator|linear|0.30m|false|true|0.5200|0.0200|m|
arm|rotational|35deg|true|true|0.4000|1.0000|deg|0.8300
wrist|rotational|-10deg|true|true|0.1000|1.0000|deg|0.6100
```

The arm and wrist both arrived, in 0.83 s and 0.61 s. The elevator never did — its
`arrivalSeconds` column is empty and `arrived` is false. One mechanism, named, with the
other two explicitly cleared. Go look at the elevator.

{: .tip }
`Observable` is worth a glance whenever a mechanism "arrives" but the robot clearly did
not. `observable=false` means arrival for that goal was a settle timer, not a sensor —
the binding waited its half-second and declared victory. That is honest and sometimes
correct, but it is never evidence.

---

## Driver override

A driver jogging the elevator while the state machine wants it somewhere else is not a
conflict to resolve — it is a thing that happens every match. It works with no extra code.

The builder installs a `GoalRunner` as the **default command** of each bound mechanism's
subsystem. That one choice does everything:

```java
// This is all you write. Nothing about the state machine appears here.
driver.povUp().whileTrue(elevatorMech.jogUp(2.0));
```

Hold the button: WPILib interrupts the `GoalRunner` on the elevator alone, the runner's
`end()` tells the engine ownership was lost, `Bindings/elevator/Owned` goes false,
`Blocker` becomes `yielded:elevator`, `Counters/Yields` increments, and the `overridden()`
trigger fires. The other eight mechanisms carry on untouched.

Release the button: WPILib re-schedules the default command, `initialize()` clears the
applied goal, and the machine's goal is re-applied on the very next loop. No policy flag,
no reassertion race, no fighting the driver.

`Owned=false` in the log means precisely one thing — some other command has that
subsystem right now. It is not an error, and during an intentional jog it is the correct
state of the world. It is worth noticing when you did *not* expect it: a stray
`whileTrue` or a second `setDefaultCommand` shows up here as a mechanism the machine
permanently cannot drive.

By default an override does **not** cancel an in-flight transition, so the driver can
nudge one mechanism without abandoning the whole superstructure move. If your robot would
be unsafe finishing a transition with one mechanism under manual control, say so:

```java
b.abortOnOverride(true);
```

The transition then aborts the moment any non-advisory mechanism is taken over, with
`Outcome.ABORTED` and detail `mechanism ownership lost`.

{: .note }
If a bound mechanism already has a default command, `build()` **fails** rather than
silently replacing it. A silently stomped `setDefaultCommand(holdPosition())` is a very
confusing afternoon. Remove that call, or use `.manageDefaults(false)` and schedule the
runners from `goalRunners()` yourself.

---

## Commands and triggers

### Commands

| Command | Ends when | Use for |
|---|---|---|
| `goTo(state)` / `goTo(state, source)` | Settled at `state`, or faulted, or superseded, or refused | `onTrue` bindings, autonomous sequences |
| `goToAndHold(state)` / `(state, source)` | Never on arrival — only when interrupted or faulted | `whileTrue` bindings |
| `requestOnly(state)` | Immediately | Fire-and-forget requests that hold no requirement |
| `waitUntilSettled(state)` | Settled at `state` | Composing; holds no requirement |
| `onlyIfSettled(state, next)` | When `next` does, or immediately if not settled | The safe autonomous idiom |
| `seed(assumedState)` | Immediately (runs while disabled) | Once at robot init |
| `abort(reason)` | Immediately | Cancelling an in-flight transition |
| `clearFault()` | Immediately (runs while disabled) | Clearing a latched fault |

The `triggerSource` string is recorded in the transition history, so the log says who
asked. Use `"op.y"`, `"auto.leg2"`, `"goal"` — anything that identifies the caller when
you are reading `Transition/History` a week later. `goTo(state)` without a source records
`"code"`.

`goTo` ends immediately if the request is refused: the button visibly does nothing, the
reason is logged and announced, and the requirement is released so nothing is left
holding.

{: .note }
Both `goTo` and `goToAndHold` are built with `Commands.defer`, so the route, the guards
and the origin state are resolved when the command is **scheduled**, not when it is
constructed. Storing the result in a field and re-binding it every match works correctly —
which is precisely what the old coordinator got wrong.

{: .warning }
**Do not write `goTo(A).andThen(next)` in autonomous.** A self-detected fault cannot make
a command end "interrupted", so `next` runs anyway — the shooter fires into the floor
because the arm never got there. Use `onlyIfSettled`:
`goTo(AIM).andThen(superstructure.onlyIfSettled(AIM, shoot()))`. It is one call longer and
it is the difference between a wasted cycle and a penalty.

### Triggers

| Trigger | Fires while |
|---|---|
| `in(state)` | The machine believes it is in `state` — not necessarily confirmed |
| `settledIn(state)` | In `state`, confirmed, **and** still measuring true |
| `arrivedAt(state)` | Same condition as `settledIn`; named for how it reads with `onTrue` |
| `transitioning()` | A transition is in flight |
| `faulted()` | A fault is latched |
| `rejected()` | Pulses for one loop on each refused request |
| `overridden()` | A bound mechanism is currently held by another command |

```java
superstructure.arrivedAt(SuperState.CARRY).onTrue(leds.flash(Color.kGreen));
superstructure.faulted().onTrue(leds.strobe(Color.kRed, 8));
superstructure.rejected().onTrue(driverRumble.pulse());
```

Prefer `settledIn`/`arrivedAt` over `in` for anything that acts on the robot being
somewhere. `in(state)` is true for a belief that a timeout left unconfirmed; only
`settledIn` means the mechanisms are measurably there.

### Queries

`current()`, `stateConfirmed()`, `target()`, `phase()`, `isAt(state)`,
`isSettledAt(state)`, `isFaulted()`, `progress()`, `blocker()`, `summary()`,
`waitingOn()`, `legalTargets()`, `plan(target)`, `history()`, `snapshot()`, `graph()`.

`plan(target)` returns the `Route` a request *would* take with no side effects — useful in
a test, and useful on a dashboard button that should grey itself out.

---

## Unit testing without hardware

`StateMachineCore` imports **nothing** from WPILib. No `Command`, no `Subsystem`, no
`Trigger`, no `Timer`, no NetworkTables. It is driven by `step()` and told the time by a
`DoubleSupplier`. That means the whole of the machine's logic is testable on a laptop with
no HAL, no simulation extension and no vendor libraries loaded.

A fake `Binding` is about ten lines:

```java
static final class FakeBinding implements Binding<Double> {
    private final String key;
    double position;
    double tolerance = 0.01;
    boolean stuck;

    FakeBinding(String key) { this.key = key; }

    @Override public String key() { return key; }
    @Override public boolean atGoal(Double goal, double since) {
        return Math.abs(position - goal) < tolerance;
    }
    @Override public double measured() { return position; }

    /** Advance one step toward the goal, unless jammed. */
    void drive(Double goal) {
        if (stuck || goal == null) return;
        double delta = goal - position;
        position += Math.signum(delta) * Math.min(0.25, Math.abs(delta));
    }
}
```

Plus a fake clock, because a test that calls `Timer.getFPGATimestamp()` is a test that
needs a HAL:

```java
static final class FakeClock implements java.util.function.DoubleSupplier {
    double now;
    @Override public double getAsDouble() { return now; }
    void advance(double seconds) { now += seconds; }
}
```

Now a real test — that a blown deadline never confirms the target:

```java
@Test
void timeoutNeverConfirmsTheTarget() {
    FakeClock clock = new FakeClock();
    FakeBinding elevator = new FakeBinding("elevator");
    FakeBinding arm = new FakeBinding("arm");

    StateMachineCore.Builder<St> b = StateMachineCore.builder(St.class, "Truth").clock(clock);
    Handle<Double> hElevator = b.bind("elevator", elevator);
    Handle<Double> hArm = b.bind("arm", arm);
    StateMachineCore<St> sm = b.defaultTimeout(1.0)
            .initialState(St.STOW)
            .state(St.STOW, s -> s.set(hElevator, 0.0).set(hArm, 0.0))
            .state(St.MID,  s -> s.set(hElevator, 0.5).set(hArm, 30.0))
            .state(St.HIGH, s -> s.set(hElevator, 1.0).set(hArm, 90.0))
            .hub(St.STOW)
            .build();

    sm.seed(St.STOW);
    elevator.stuck = true;            // jam it
    sm.request(St.MID, "test");

    for (int i = 0; i < 200; i++) {
        clock.advance(0.02);
        sm.step();
        elevator.drive(sm.activeGoalOf(hElevator));
        arm.drive(sm.activeGoalOf(hArm));
    }

    assertEquals(St.STOW, sm.current(),
            "current() must stay at the last PROVEN state, not jump to the one we failed to reach");
    assertFalse(sm.stateConfirmed());
    assertFalse(sm.isSettledAt(St.MID));
    assertEquals(1, sm.timeoutCount());
}
```

No robot, no `HAL.initialize()`, no scheduler. It runs in milliseconds and it tests the
thing that actually matters.

The 41 shipped tests live in
`src/test/java/frc/lib/catalyst/statemachine/` and are worth reading as documentation:

| File | Covers |
|---|---|
| `StateMachineFixtures.java` | The fakes above, plus a recording telemetry sink |
| `StateMachineTruthTest.java` | The `current()`/`stateConfirmed()` invariant, seeding, measured-not-latched arrival, frozen deadlines while disabled |
| `StateMachineGraphTest.java` | Edges, hubs, routing, guards, interlocks, validation |
| `StateMachineTransitionTest.java` | Staging, timeouts, fault policies, aborts, supersession |
| `StateMachineTelemetryTest.java` | Publication cadence — that the log is diffed, not spammed |

You can also validate a real robot's configuration in a test without touching hardware:

```java
@Test
void superstructureConfigurationIsValid() {
    ValidationReport report = buildSuperstructureBuilder().validate();
    assertTrue(report.ok(), report.toString());
}
```

`validate()` runs every binding's `validate` hook, so out-of-range setpoints, unknown
presets and structurally impossible goals are caught at commit time, all at once, instead
of one exception per deploy cycle in a pit.

---

## Migrating from SuperstructureCoordinator

You do not have to. The old class is deprecated but frozen, not scheduled for removal, and
existing robot code keeps working unchanged. Migrate when you want the mechanism types,
the graph or the log — not because of a compiler warning.

### Before and after

```java
// BEFORE — SuperstructureCoordinator
SuperstructureCoordinator coord = new SuperstructureCoordinator()
    .withLinear("elevator", elevatorMech)
    .withRotational("arm", armMech)
    .withTimeout(3.0)
    .defineState("STOW")
        .setLinear("elevator", 0.0)
        .setRotational("arm", 0.0)
        .done()
    .defineState("SCORE_HIGH")
        .setLinear("elevator", 1.1)
        .setRotational("arm", 90.0)
        .done()
    .addTransitionRule("STOW", "SCORE_HIGH", (from, to) ->
        to.goToLinear("elevator")
          .andThen(Commands.waitUntil(elevatorMech.atPositionTrigger(1.1, 0.1)))
          .andThen(to.goToRotational("arm")));

operator.y().onTrue(coord.transitionTo("SCORE_HIGH"));
```

```java
// AFTER — Superstructure
public enum SuperState { STOW, SCORE_HIGH }

var b = Superstructure.builder(SuperState.class, "Superstructure");
var elevator = b.bind("elevator", Mechanisms.linear(elevatorMech));
var arm      = b.bind("arm",      Mechanisms.rotational(armMech));

Superstructure<SuperState> superstructure = b
    .defaultTimeout(3.0)
    .state(SuperState.STOW, s -> s
        .set(elevator, LinearGoal.meters(0.0))
        .set(arm,      RotationalGoal.degrees(0)))
    .state(SuperState.SCORE_HIGH, s -> s
        .set(elevator, LinearGoal.meters(1.1))
        .set(arm,      RotationalGoal.degrees(90)))
    .allowBoth(SuperState.STOW, SuperState.SCORE_HIGH)
    .edge(SuperState.STOW, SuperState.SCORE_HIGH, e -> e.stage(elevator).stage(arm))
    .build();

superstructure.engine().seed(SuperState.STOW);
operator.y().onTrue(superstructure.goTo(SuperState.SCORE_HIGH, "op.y"));
```

The differences worth noticing: states are enum constants, so a typo is a compile error
rather than a silent no-op. The hand-built transition rule with its opaque
`waitUntil`/`andThen` chain becomes `.stage(elevator).stage(arm)`, which the machine can
inspect, log and report progress through. And the legal-transition graph is now explicit —
under the old class every state could reach every other state, always.

### The four frozen defects

Each of these is documented on the deprecated class and deliberately left unfixed, because
changing them would silently alter what a robot already built on it physically does.

| Defect in `SuperstructureCoordinator` | How `Superstructure` avoids it |
|---|---|
| `transitionTo(String)` selects its transition rule at command **construction** time, so a command stored in a field and re-bound each match keeps using whichever rule applied when it was created. | `goTo` is built with `Commands.defer`. Route, guards and origin state resolve at **schedule** time, every time. |
| The default parallel transition completes after a single scheduler tick, whether or not any mechanism moved. | A transition completes only when every gating binding reports `atGoal`, plus any `settleFor` window. There is no path from "asked" to "done" that skips measurement. |
| `getCurrentState()` is set to the target even when a transition is interrupted or times out, so the next transition plans from a state the robot is not in. | `current()` is only ever a **proven** state. A timeout leaves it at the last confirmed value and sets `stateConfirmed()` false, so the next transition plans from the truth. |
| `isAtState(String)` returns `true` for mechanism keys that were never registered. | Handles are typed objects returned by `bind(...)`. There is no way to name a mechanism that does not exist — the code does not compile. |

If you use `GoalDirector`, the migration is one line: replace `.coordinator(coord)` with
`.superstructure(superstructure)`. Goals still name their state as a `String`, so a
director on the new engine and one on the old coordinator remain interchangeable.

---

## A note on Command V3 and 2027

The issue suggested using the state machine coming in WPILib's Command V3 for 2027. That
is a reasonable thing to want, and the engine is built so it stays possible.

`StateMachineCore` takes no WPILib imports at all — that is why the tests above run with
no HAL, and it is the same property that makes a port to a new command framework a change
to the thin robot-side adapter (`Superstructure`, `Actuator`, `GoalRunner`) rather than to
the engine. The graph, the guards, the arrival logic, the fault policies and the entire
log schema live below that line and do not care what a `Command` is.

Nothing V3-related ships in 1.2.0, and there is no date attached to this. It is a
statement about how the code is arranged, not a promise about a release.

---

## See also

- [Health Monitoring](health.html) — per-mechanism fault checks; a mechanism that is hot or stalled is usually the one in your `Blocker` string
- [Logging & AdvantageKit Bridge](logging.html) — where `/Catalyst/<prefix>/` goes and how to swap the sink
- [Behavior Framework](behavior.html) — `Action`s that use the superstructure as one of their primitives
