---
layout: default
title: Measuring Your Robot
parent: Advanced
nav_order: 11
---

# Measuring Your Robot for Physics Core
{: .no_toc }

What each number actually does, how to measure it with pit tools, and — most importantly — which ones
are worth the afternoon and which ones are not.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Read this table first

Physics Core takes six measurements. They are **not** equally important, and the ranking is not
intuitive. Before you weigh anything, know what it buys you:

| Measurement | What it drives | Error sensitivity | Worth measuring? |
|---|---|---|---|
| **Coefficient of friction** | Traction limit, stopping distance | **1:1** — 15% off means 15% off | **Yes.** Biggest single lever |
| **Centre-of-mass height** | Tipping limit, ZMP, wheel loads | **1:1 inverse** — 20% off means 20% off | **Yes**, if anything extends |
| Footprint | Tipping limit, wheel loads | 1:1, but a tape measure is accurate | Yes — it's five minutes |
| Mass | Traction *force*, wheel loads, collision force | **Zero** on the traction limit | Roughly. Nearest kg is fine |
| Mechanism masses | Live centre of mass | Proportional to how far they move | Yes, if you use the mass tree |
| Moment of inertia | Angular acceleration limit only | 1:1 inverse, on that one figure | Only if you need that number |

**The one that surprises people:** mass does not affect the traction limit at all. A heavier robot
needs more force to accelerate and gets proportionally more grip to do it with, and the two cancel
exactly:

```text
F = μ·m·g        and        a = F/m = μ·g
```

Teams re-weigh the robot hoping to sharpen their acceleration limit. It cannot help. Weigh it once to
the nearest kilogram, because it does matter for forces and wheel loads, and then go measure the
friction instead.

You can get this analysis for your own numbers:

```java
ModelUncertainty uncertainty = ModelUncertainty.builder(model)
    .massKg(2.0)                       // bathroom scale
    .centerOfMassHeightMeters(0.04)    // estimated by eye — be honest
    .coefficientOfFriction(0.15)       // not measured
    .build();

System.out.println(uncertainty.describe());
System.out.println(uncertainty.dominantSource().measurement());   // "what should I go measure?"
```

---

## Mass

**What it drives:** traction *force*, wheel normal loads, collision force, and the mass-weighted centre
of mass. **Not** the traction limit.

**How:** a bathroom scale under each corner, summed, or a single scale and a plank. Weigh it **as it
competes** — battery in, bumpers on, game piece out.

**Accuracy needed:** ±1 kg is plenty. ±5 kg is fine for everything except collision force.

**Common mistake:** weighing without bumpers. Bumpers are 5–7 kg and they sit low and outboard, so
leaving them off changes both the mass *and* the centre of mass.

```java
.massKg(54.4)      // or .massPounds(120.0)
```

> **If you use the mass tree:** `chassis().massKg()` must be everything that does **not** move. Count
> the elevator in both the chassis mass and its `MechanismModel` and you have double-counted it, which
> silently biases every centre-of-mass result toward the chassis.

---

## Footprint

**What it drives:** the tipping limit and the support polygon for wheel loads.

**How:** measure between **wheel contact patches**, not the frame perimeter. On most swerve robots the
modules are inset from the frame by 5–10 cm per side, so using the frame overstates your footprint by
10–20% and your tipping limit with it.

- `trackWidthMeters` — left to right, centre of contact patch to centre of contact patch
- `wheelBaseMeters` — front to back, same

**Accuracy needed:** ±5 mm. A tape measure beats every other tool on this page, which is why footprint
is almost never the limiting error.

```java
.footprintMeters(0.74, 0.74)
```

---

## Coefficient of friction

**What it drives:** the traction limit, directly and one-for-one. **The highest-value measurement on
this page.**

The default is 0.9, deliberately conservative. Real values run 0.9–1.1 for fresh tread on clean
carpet, and drop toward 0.7 for worn tread or a dusty field.

### Method 1 — the tilt test (best accuracy, needs a ramp)

Put the robot on a board, disable it in brake mode, and slowly lift one end until it slides.

```text
μ = tan(θ)
```

Measure the angle by measuring rise over run — a 1 m board lifted 0.8 m is θ = 38.7°, μ = 0.80. Repeat
five times and average; the spread tells you your uncertainty, which is what
`ModelUncertainty.coefficientOfFriction(...)` wants.

**Accuracy:** ±0.05, and you learn the real spread.

### Method 2 — the pull test (no ramp needed)

Disable the robot in brake mode on carpet, hook a fish scale to the frame, and pull horizontally until
it slides. Read the peak.

```text
μ = peak force / weight
```

A 55 kg robot weighs 540 N, so a 480 N peak means μ = 0.89. Use a scale rated well above that, pull
**horizontally** (any upward angle unloads the wheels and reads low), and pull along the frame axis.

**Accuracy:** ±0.1. Easier than the tilt test and good enough.

### Method 3 — from a log (free, least accurate)

Drive full-throttle from a standstill on a good surface and read the peak acceleration off the IMU. If
the wheels broke traction, that peak *is* `μ·g`.

```text
μ = peak acceleration / 9.81
```

**Accuracy:** ±0.15, and only valid if you actually slipped — if the drivetrain was torque-limited
rather than traction-limited, you measured your motors, not your tyres. Physics Core's slip detector
tells you which happened.

> **Measure it in competition condition.** Carpet friction drops noticeably over an event as the field
> gets dusty. If you measure on fresh practice carpet and compete on day three, you are running an
> optimistic number.

---

## Centre-of-mass height

**What it drives:** the tipping limit, the zero-moment point, and the wheel load distribution. The
tipping limit is **inversely proportional** to it, so a 20% error is a 20% error in the limit.

This is the hardest measurement here *and* one of the two that matter most. That is worth an afternoon.

### Method 1 — the tilt-and-weigh test (best)

1. Weigh the robot flat. Call it `W`.
2. Raise one end by a known height so the frame sits at angle `θ` (20–30° is enough), with the low-end
   wheels chocked.
3. Put a scale under the **raised** end and read the load `F`.

```text
h = L·(F/W − b/L) / tan(θ) + r
```

where `L` is the wheelbase, `b` the horizontal distance from the low axle to the centre of mass
measured flat (see below), and `r` the wheel radius. If the centre of mass is at the middle
(`b = L/2`), this simplifies to:

```text
h = L·(F/W − 0.5)/tan(θ) + r
```

**Accuracy:** ±1–2 cm. Do it stowed and again fully extended — the difference is what the mass tree
exists to model.

### Method 2 — estimate from the parts list (fast)

Take each major assembly's mass and the height of its centre, and average by mass:

```text
h = Σ(mᵢ·hᵢ) / Σmᵢ
```

Battery ~8 kg at 0.10 m, drivetrain ~20 kg at 0.08 m, superstructure, bumpers ~6 kg at 0.13 m. Ten
minutes with a CAD model or a tape measure.

**Accuracy:** ±3–5 cm — worse than it looks, because it is easy to forget something heavy.

### Method 3 — the default (do not ship with this)

`0.20 m`, which suits a low robot with everything stowed. It is a placeholder, not a measurement, and
it is optimistic for anything that extends.

```java
.centerOfMassHeightMeters(0.22)
```

### Finding the horizontal centre of mass too

Two scales, one under each axle, robot flat:

```text
distance from front axle = wheelbase × (rear reading / total)
```

Feed it in through `ArticulatedRobotModel.Builder.chassisCenterOfMass(...)`. Worth doing if your robot
is visibly nose- or tail-heavy — it makes the robot genuinely harder to tip one way than the other, and
`StabilityModel` will tell you which way.

---

## Wheel radius

**What it drives:** motor torque to wheel force, via `DrivetrainModel.wheelForceFromTorque(...)`.

**How: don't measure it with calipers.** A wheel under load is compressed, so its *effective* rolling
radius is smaller than its free radius, typically by 2–4%. Catalyst already ships the right tool:

```java
WheelRadiusCalibration.run(drive);   // spins the robot, back-solves from gyro arc vs module arc
```

That gives the effective radius under actual load. See the [utilities guide](../utilities/).

**Why this matters beyond force:** an incorrect wheel radius makes **every encoder speed wrong by the
same factor**, which is exactly what Physics Core's `SensorDisagreement` residual surfaces. If that
sits persistently non-zero while the robot drives cleanly, this is the first thing to check.

---

## Moment of inertia

**What it drives:** `DrivetrainModel.maxAngularAccelerationRadPerSecSq()` — and nothing else.

**Skip it unless you need that number.** Left unset, Catalyst estimates it as a uniform slab:

```text
I ≈ m·(w² + l²)/12
```

That runs 10–20% **light**, because a real robot carries its battery and drivetrain low and outboard
rather than spread uniformly. If you only want traction and tipping limits, the estimate is irrelevant —
neither uses it.

**If you do need it:** command a known yaw torque, measure the resulting angular acceleration from the
gyro, and use `I = τ/α`. Or accept the estimate and add 15%.

```java
.momentOfInertiaKgM2(4.8)   // measured; omit to use the slab estimate
```

---

## Mechanism masses, for the mass tree

**What it drives:** the live centre of mass, and through it the live tipping limit.

**How:** weigh the moving assembly on its own if you can — a spare elevator carriage on a scale. If it
is already installed, weigh the robot with the mechanism at both extremes of travel with a scale under
one end; the change in reading gives you `mass × travel` and you know the travel.

**Accuracy needed:** proportional to how far it moves. A 10 kg carriage travelling 1.2 m moves the CoM
of a 60 kg robot by 20 cm, so a 20% mass error there is 4 cm of CoM error. A 2 kg wrist rotating 30 cm
moves it by 1 cm, and a 20% error there is invisible.

**Rule of thumb:** model anything whose `mass × travel` exceeds about 3 kg·m. Ignore the rest.

> **Check the sign once.** Right-handed rotation about `+Y` takes `+X` toward `−Z`, so a positive angle
> pitches an arm's far end **down**. Most arms count up as positive. Raise the arm and confirm
> `centerOfMassHeightMeters()` goes **up**; if it goes down, negate the supplier or pass
> `.about(new Translation3d(0, -1, 0))`.

---

## Shooter release delay

**What it drives:** `predictLaunchState()`, and through it every shot-on-the-move solution.

**How:** high-speed phone video (240 fps is plenty) alongside the log. Count frames from the command
timestamp to the piece leaving the shooter.

**Typical values:** 80–150 ms. **Accuracy needed:** ±20 ms. At 3 m/s that is 6 cm of lead error, which
is inside most goals.

```java
.releaseDelaySeconds(0.12)
```

---

## Battery internal resistance

**What it drives:** brownout prediction (`BrownoutMonitor`) and power planning (`PowerPredictor`).

**Don't measure it by hand — Catalyst identifies it from normal driving:**

```java
battery.addSample(RobotController.getBatteryVoltage(), pdh.getTotalCurrent());
battery.recommendation().ifPresent(r -> System.out.println(r.describe()));
```

Real batteries vary by a factor of two between a fresh one and a four-year-old one, and everybody uses
0.020 Ω for all of them. Run the identifier for a few matches and label your batteries with what it
tells you. It reports only — nothing applies the value.

---

## A worked example

A 55 kg swerve robot, 0.70 m square footprint, with a 12 kg elevator that travels 1.4 m.

**Measured in an afternoon:**

| Value | Method | Result |
|---|---|---|
| Mass | Bathroom scale, competition config | 55 kg ± 1 |
| Footprint | Tape between contact patches | 0.70 × 0.70 m ± 0.005 |
| Friction | Tilt test, 5 runs | 0.95 ± 0.05 |
| CoM height, stowed | Tilt-and-weigh | 0.24 m ± 0.02 |
| Elevator mass | Spare carriage on a scale | 12 kg ± 0.5 |
| Release delay | 240 fps video | 0.115 s ± 0.015 |

**What that gives:**

```text
traction limit                   9.32 m/s^2  +/- 0.49  ( 5.3%)
tipping limit                   14.30 m/s^2  +/- 1.23  ( 8.6%)
binding limit                    9.32 m/s^2  +/- 0.49  ( 5.3%)
  limited by: coefficient of friction — the traction limit is mu*g and depends on nothing else
```

Stowed, this robot is **traction-limited**, so friction is the only thing that matters and the CoM
measurement barely shows up. Raise the elevator and the CoM climbs to 0.47 m, the tipping limit falls
to 7.3 m/s², and it becomes **tipping-limited** — at which point the CoM measurement is suddenly the
dominant error and the friction one stops mattering.

**That switchover is the whole reason to model both.** A single static number cannot represent a robot
that is traction-limited stowed and tipping-limited extended, and every robot with an elevator is both.

---

## Checklist

Fifteen minutes gets you most of the value:

- [ ] Weigh the robot in competition config — battery, bumpers, no game piece
- [ ] Tape-measure the footprint **between contact patches**, not the frame
- [ ] Run `WheelRadiusCalibration` for the effective rolling radius
- [ ] Tilt or pull test for friction, five runs, keep the spread

An afternoon gets you the rest:

- [ ] Tilt-and-weigh for CoM height, **stowed and extended**
- [ ] Weigh each mechanism whose `mass × travel` exceeds 3 kg·m
- [ ] High-speed video for the shooter release delay
- [ ] Run `ModelUncertainty.describe()` and see what is limiting you

Then confirm it:

- [ ] Run `PhysicsValidator` against your model — see [validation](physics.md#validating-against-simulation)
- [ ] Watch `SensorDisagreement` in shadow mode; if it is persistently non-zero, your wheel radius or
      gear ratio is wrong and every downstream number inherits it

---

## What happens if you get one wrong

| Error | Symptom | How you'd notice |
|---|---|---|
| Friction too high | Constraints allow acceleration that slips | `Slip/Peak` fires under normal driving |
| Friction too low | Robot drives more timidly than it needs to | `TractionUsage` never approaches 1.0 |
| CoM height too low | Tipping limit optimistic — the dangerous one | Nothing warns you until it tips |
| CoM height too high | Needlessly cautious when extended | `TippingUsage` high at modest acceleration |
| Footprint from the frame | Tipping limit optimistic by 10–20% | Same as CoM too low |
| Mass wrong | Collision forces and wheel loads off | Collision magnitudes look implausible |
| Wheel radius wrong | Every speed and distance scaled | `SensorDisagreement` persistently biased |

**The asymmetry is deliberate:** the errors that make Physics Core *cautious* announce themselves in
telemetry. The errors that make it *optimistic* — friction too high, CoM too low, footprint from the
frame — are the ones that stay quiet until the robot tips. When you are unsure, round toward caution:
lower friction, higher centre of mass, smaller footprint.
