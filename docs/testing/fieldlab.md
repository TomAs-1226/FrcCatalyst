---
layout: default
title: Field Lab
parent: Testing
nav_order: 1
---

# Field Lab
{: .no_toc }

A planned measurement campaign for the afternoon a team actually gets a field.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

{: .warning }
> **Not yet run on a real field.** `frc.lib.catalyst.fieldlab` is a new package: the framework is
> tested — 939 tests pass on a laptop, no HAL — but no campaign has been run on a robot, and nothing
> in this package has been near a Systemcore. Read this as a design that has been proven on a
> checkpoint file and a fake battery supplier, not on carpet.

## Why

A team gets a field for an afternoon, perhaps twice a season. What that afternoon produces decides
how good the robot is for the rest of it, and what usually happens is that the time goes on driving
practice and on whatever broke. The measurements that only a real field can give — odometry drift
over twelve metres, a shot from the far side, where the vision pose actually is — get taken ad hoc,
by hand, into a notebook, or not at all.

A **campaign** is that afternoon planned in advance and run by the robot instead of a clipboard. It
knows how long it expects to take. It checkpoints every finished procedure, so a battery swap costs
one procedure instead of the hour. It refuses to characterise on a flat pack rather than hand back a
number that looks real and is not. And it waits, indefinitely, for a person with a tape measure,
because the numbers a robot most needs are the ones it cannot measure itself — the gap between where
it believes it is and where it actually is belongs to the tape, not the gyro.

The output of a session is one `.wpilog` and a report. Nothing is fitted on the robot: fitting is
`tools/fieldlab.py`'s job, offline, after the field is gone — see [The offline analysis](#the-offline-analysis).

## A worked example

Assembling a campaign from the library's own procedures, plus one project-specific shot map:

```java
import frc.lib.catalyst.fieldlab.Procedures;
import frc.lib.catalyst.fieldlab.TestCampaign;
import frc.lib.catalyst.fieldlab.TestProcedure;

import java.util.List;

public final class Campaigns {
    private Campaigns() {}

    public static TestCampaign fieldDay(SwerveSubsystem drive, SysIdRoutine driveSysId,
                                         Hood hood, Shooter shooter) {
        List<TestProcedure> basics = Procedures.drivetrainBasics(
                drive, driveSysId,
                /* currentWheelRadiusMeters */ 0.0508,
                /* driveBaseRadiusMeters */ 0.43,
                /* currentSlipAmps */ 120.0,
                /* runwayMeters */ 8.0,
                () -> drive.followPathCommand("ClosedLoop"),
                /* loopSeconds */ 6.0);

        TestCampaign.Builder b = TestCampaign.builder("Field Day")
                .add(basics.toArray(new TestProcedure[0]));

        // hood and shooter are this team's own mechanisms; sweep() only asks for a setter, a
        // readiness check and a command, so it never needs to know they exist.
        b.add(Procedures.sweep("shot-map", "Shot map",
                hood::setAngle, shooter::setRpm, shooter::atSpeed,
                () -> shooter.fireOnce(),
                List.of(Procedures.Setpoint.of(2.0, 1800, 42),
                        Procedures.Setpoint.of(3.0, 2100, 38),
                        Procedures.Setpoint.of(4.0, 2400, 34)),
                "did it score? 1 = in, 0.5 = rim, 0 = miss"));

        return b.build();
    }
}
```

`TestCampaign.Builder.add` takes either one `TestProcedure` or a varargs run of them — there is no
overload for a `List`, so a group like `Procedures.drivetrainBasics(...)` needs `.toArray(...)` on
the way in. Order is priority order: put the procedures whose results everything else depends on
first, because the campaign will be cut short. It always is.

The op mode that runs it, in the shape `FieldLabOpMode`'s own javadoc gives:

```java
import org.wpilib.opmode.Utility;
import frc.lib.catalyst.fieldlab.FieldLabOpMode;

@Utility(name = "Field Lab", group = "Data",
         description = "The field-day measurement campaign; needs an operator with a tape measure")
public class FieldLab extends FieldLabOpMode {
    public FieldLab() {
        super(Campaigns.fieldDay(Robot.ROBOT.drive(), Robot.ROBOT.driveSysId(),
                Robot.ROBOT.hood(), Robot.ROBOT.shooter()));
    }
}
```

That is the whole wiring. It appears on the Driver Station under **Data**, the same group as every
other Catalyst diagnostic.

## How an operator actually uses it

- **Enable to run, disable to pause.** Disabling stops the campaign's command the same way it stops
  every command. Enabling again starts the op mode over; `CampaignRunner` reads its checkpoint and
  carries on from the next unfinished procedure. The ordinary rhythm of a field session — disable,
  swap a battery, enable — is also how the campaign is meant to be paused.
- **The prompt.** Whatever the campaign currently wants from a person is on
  `/Catalyst/FieldLab/Prompt`, with the expected unit on `/Catalyst/FieldLab/PromptUnit`. Both are
  read-only status, not something a dashboard writes.
- **Answering.** A number goes on `/Catalyst/Tuning/FieldLab/Value`; committing it means changing
  `/Catalyst/Tuning/FieldLab/Advance` to any other number. Both are ordinary tunables keys, so
  Catalyst Console and Catalyst Tab can already write them with no new API.

**Advance is a change, not a level.** `FieldLabInput.advanced()` reports whether the value has
changed since it was last asked, not whether it is currently set to something. A level-triggered
control would need the operator to set it and then clear it for every single step, and a step that
forgot to clear it would run the whole rest of the campaign through in one 20 ms loop — twenty
procedures in twenty milliseconds is a real failure mode this design avoids, not a hypothetical one.
A change is also exactly what a dashboard's number field naturally produces: type something different
and press enter.

## The checkpoint and resume

This is the thing that makes an hour of field time survivable. A campaign runs for the better part of
an hour, and the robot will be disabled, re-enabled, browned out and power-cycled several times in
that hour — a single battery swap does all four. Without a checkpoint, every one of those restarts the
campaign from the top, and in practice a campaign never gets past its third procedure before the
operator gives up and starts running things by hand.

`CampaignRunner` writes a `CampaignCheckpoint` after every procedure finishes, and reads it back on
start. It holds the session id, so a resumed run keeps logging into the same namespace, and every
measurement a person has already typed, so nobody is asked to re-measure something they already
measured. Only the procedure that was in flight is lost to a restart — nothing banked, and nothing
answered.

It lives at `/home/systemcore/catalyst/fieldlab-checkpoint.json` on a Systemcore, beside the motor
history, and in the working directory in simulation — deliberately not on the USB stick the logs go
to, because that stick gets pulled and taken to the shop, and a checkpoint that walks away mid-session
would make the campaign silently restart. It is cleared only when the campaign finishes, so a
completed run does not resume into itself the next time the op mode is enabled.

## The battery gate

`TestStep.BatteryGate` refuses to go on below a set voltage and resumes once the pack recovers.
Every procedure Catalyst ships gates itself this way (`needsBattery(...)` on the builder).

Data taken at 11 V is not slightly worse than data taken at 12.6 V — it is wrong in a direction that
looks like a real result. kV comes out high. A flywheel never reaches its commanded setpoint. A
slip-current sweep slips early, because the motors cannot deliver the current the fit assumes. Worse,
the robot can brown out mid-procedure, and the run becomes a write-off discovered later, at the shop,
when the field is gone. A gate that pauses and says which battery it wants, then carries on from
exactly where it stopped, is cheaper than any of that.

The gate pauses at `minVolts` and does not clear until the voltage reaches `resumeVolts`, which is
above `minVolts` on purpose — `BatteryGate.at(volts)` sets it 0.3 V above the floor, so a pack that
sags back down to the floor under load does not flap the gate open and shut.

## What it publishes

Under `/Catalyst/FieldLab/`, from `CampaignRunner.publish()` — call it every loop, from both
`onPeriodic()` and `onDisabledPeriodic()`, which `FieldLabOpMode` already does. Both, because the two
moments an operator is most likely to be looking at the dashboard — held at a battery gate with the
robot disabled for the swap, or waiting on a measurement — are exactly the moments no command is
running.

| Topic | Type | What it is |
|---|---|---|
| `State` | string | `IDLE`, `RUNNING`, `AWAITING_OPERATOR`, `PAUSED_BATTERY`, `DONE` or `ABORTED` |
| `Session` | string | the session id, e.g. `690123ab` — short enough to read off a dashboard and match to a `.wpilog` |
| `Campaign` | string | the campaign's name |
| `Procedure` | string | the current procedure's title |
| `ProcedureIndex` | int | index of the current procedure |
| `ProcedureCount` | int | procedures in the campaign |
| `Step` | string | the current step's label |
| `StepIndex` | int | index of the current step within the procedure |
| `StepCount` | int | steps in the current procedure |
| `Progress` | double | fraction of procedures banked, 0 to 1 |
| `ElapsedSeconds` | double | since the campaign started |
| `RemainingSeconds` | double | sum of the estimates of procedures not yet banked |
| `BatteryVolts` | double | the battery source's current reading |
| `Notes` | string[] | every note the run has accumulated |
| `Measurements` | string[] | `"<procedure>/<key>|<value>"` rows, one per measurement taken |

Two more are written outside the per-loop `publish()` call: `EstimatedSeconds` once, when a session
begins, and `LastNote` each time a note is recorded. `Prompt` and `PromptUnit` — the operator-facing
question — come from `FieldLabInput`, not `CampaignRunner`, and are covered above under
[How an operator actually uses it](#how-an-operator-actually-uses-it).

A dashboard needs nothing but this table to show a campaign's progress and its current prompt.

## The offline analysis

`CampaignRunner` fits nothing on the robot. It records, through `CatalystLog`, into whatever sink is
installed — a `WpilogSink` puts everything into the same `.wpilog` as the rest of the robot's
telemetry, so one file is one campaign with the robot's full context alongside the measurements.
`tools/fieldlab.py` reads that log afterwards:

```bash
python tools/fieldlab.py report run.wpilog                    # the report, on stdout
python tools/fieldlab.py report run.wpilog -o report.md       # and to a file
python tools/fieldlab.py json run.wpilog                      # just the numbers, as JSON
python tools/fieldlab.py diff run.wpilog --constants c.json   # what the robot believes vs. what the field measured
python tools/fieldlab.py channels run.wpilog                  # every channel in the log, with counts
```

It is standard-library-only Python 3.8+; there is no dependency to install. Two reasons the fit
happens here and not on the robot: a robot has twenty milliseconds a loop and no business running a
least-squares, and a fit you can re-run against the same log with a different model is worth more
than a number the robot computed once and threw the data away for.

**What it fits:**

- **Feedforward (kS, kV, kA)** from the SysId channels a `driveFeedforward` procedure recorded, by
  least squares over the quasistatic and dynamic segments together. The model is
  `V = kS·sign(v) + kV·v + kA·a`, with acceleration differentiated from the logged velocity — which
  is exactly why **kA is the weakest of the three**: it is a derivative of a derivative of a
  measurement, and any noise in velocity gets amplified before the fit sees it. The report gives the
  fit's residual in volts RMS and calls it out plainly: a residual under 0.25 V reads as good, and
  anything above that is labelled `poor - a line through a mess, not a measurement`. Read kS and kV
  as measurements and kA as a starting point.
- **Wheel radius and slip current** — reads what `WheelRadiusCalibration` and `SlipCurrentCalibration`
  already concluded on the robot, plus the caliper cross-check a person typed in, and says whether
  they agree.
- **Odometry drift** — the operator's lap measurements, as a mean and a spread, with a reading on
  whether the pattern looks like a systematic scale error (fix the wheel radius) or slip (fix the
  current limit or the driving).
- **A sweep** (a shot map, or anything shaped like one) — pairs each commanded setpoint with the score
  a person gave it and prints the table, plus which setpoints scored.
- **Thermal soak** — peak temperature and current per channel.

**What it refuses to fit:** anything it does not recognise. An unrecognised procedure's measurements
and notes are printed and nothing more — the tool does not guess at a model for data it was not
written to understand, which is the honest output.

## The library/robot split

The library ships the generic procedures. Anything that needs a shooter, or any other project-specific
mechanism, is configuration through `Procedures.sweep(...)`, not a new routine.

`sweep` is generic on purpose: the library cannot know what a shooter is, but every table shaped like
one — distance to RPM, distance to hood angle, angle to time-of-flight — is the same procedure:
command two setpoints, wait for them to settle, act, and have a person say what happened. A team's
shot map becomes about thirty lines of configuration (the setpoint table and the four accessor
references `sweep` takes) rather than a new procedure type.

The rest of `Procedures` — `wheelRadius`, `slipCurrent`, `driveFeedforward`, `odometryDrift`,
`thermalSoak` — wraps routines Catalyst already had (`WheelRadiusCalibration`, `SlipCurrentCalibration`,
`SysIdRoutine`). What was missing was never the measurement; it was the thing that runs nine of them
in order, waits for a person, and survives a battery swap. `odometryDrift` and `sweep` are the two
genuinely new measurements here — the ones a robot cannot take by itself, because the answer lives on
a tape measure or in what a person saw the game piece do.

Run them roughly in this order in a campaign, because each depends on the ones above it: wheel radius,
then slip current, then drive feedforward, then odometry drift (meaningless until the three above are
right), then a sweep, then thermal soak last, because it leaves everything hot.

## What it deliberately does not do

There is no way to start a campaign from a dashboard. Catalyst publishes no command triggers over
NetworkTables, on any diagnostic — this is a design commitment, not something Field Lab happened to
omit. `SystemCheck`, `Zero wheels`, `Find motor`, the wheel-radius calibration and SysId are all
Driver Station **Utility** op modes; a dashboard shows their results and, at most, names the op mode
to run. `/Catalyst/Tuning/FieldLab/Advance` looks more like a command than a tuning gain does, but it
cannot start, stop or steer anything by itself — it can only let a Utility op mode that a person
already selected and enabled move on to its next step. A campaign is the largest routine of this
shape and belongs in the same place as the rest of them, for the same reason: what runs on the robot
is chosen on the Driver Station, and a dashboard only ever watches.
