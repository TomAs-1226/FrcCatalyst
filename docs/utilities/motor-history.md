---
title: Motor history
parent: Utilities
nav_order: 60
---

# Motor history

**What every motor on the robot has been through, by serial number, kept on the robot.**

A motor's CAN id and its name are what the code knows it by, and both are one Tuner X session
away from changing. When the Catalyst X1 was first powered, its "front left steer" was the
front-right drive and its "intake roller" was a back drive motor - every label correct for a robot
those motors were no longer on. The serial number is the motor. `MotorHistory` keeps, per serial:

- every id, name, bus and firmware it has been seen with, and when - so a renumbered motor keeps
  its past, and "what was this motor before" has an answer;
- for motors: seconds powered, turning, under load and hot; revolutions; energy; the highest stator
  current and temperature ever; sticky faults ever set;
- a bounded log of recent boots (start time, seconds, revolutions, peaks), and a boot count.

## It is on by default

Anything that has a `HealthMonitor` has this: it runs on the same tick as the device roster,
enabled or not. Identity comes from the robot program's own Phoenix diagnostic server - the one
Tuner X talks to - asked every 30 s for every device on every bus, so motors the team's code never
constructs are tracked too. Usage comes from the status frames every Talon FX already sends,
sampled at 10 Hz through Phoenix objects the history makes for itself.

```java
// Optional. Defaults are sensible; change them before the first loop.
MotorHistory.Config cfg = new MotorHistory.Config();
cfg.hotCelsius = 65;               // what counts as a hot second
cfg.path = "catalyst/motor-history.json";   // beside the program; absolute paths work too
MotorHistory.configure(cfg);
```

## The file is the record

`catalyst/motor-history.json` in the program's working directory (`/home/systemcore` on a
Systemcore). It is read once at boot and written when something changes - at most every 30 s, and
always when the robot disables - through a temporary file and a rename, so a power loss mid-write
leaves the old file, not half of a new one. Nothing is re-derived from the bus at boot: a motor
that is unplugged keeps its history, and a fresh deploy does not touch it (it is not under
`deploy/`).

Copy it off whenever you like:

```
scp systemcore@10.TE.AM.2:/home/systemcore/catalyst/motor-history.json .
```

Or let the tools do it. With [catalyst-agent](../tools/systemcore-agent.md) 2.0.3 on the
Systemcore, `http://10.TE.AM.2:4800/api/motor-history` is the file and
`/api/motor-history.csv` is one row per motor; the Catalyst App's **Motor history** tool fetches
either and saves it, and the Console's **Motor history** tile shows the live table.

## What the dashboard sees

| Key under `/Catalyst/MotorHistory/` | |
|---|---|
| `Rows` | one string per device: `serial|model|kind|bus|id|name|firmware|poweredS|runningS|loadedS|revolutions|peakA|peakC|hotS|energyJ|boots|firstSeenMs|lastSeenMs|identities|stickyFaults` |
| `Count` / `Devices` | motors / all devices known |
| `Summary` | "8 motors, 12.3 h powered in total, hottest ever 74 C (FL_Drive)" |
| `File` | where the file is on the robot |
| `ClockTrusted` | false while the Systemcore's clock reads 1970; dates are then relative |
| `Discovery` | `ok`, or why the diagnostic server could not be read |

## Reading it

- **Hours** are powered hours: the motor was on the bus. Turning hours are the ones that wore the
  bearings; loaded hours (stator current over 5 A) are the ones that wore the windings.
- **Hot seconds** count time at or above `hotCelsius` (70 °C by default). A Falcon starts
  protecting itself in the 90s. A motor with far more hot seconds than its module-mates is either
  working harder or cooling worse, and both are worth a look.
- **Peak stator current** and **sticky faults** are lifetime highs: a 200 A peak and a
  `SupplyCurrLimit` fault on a drive motor say something about a match that happened.
- **Identities** are the motor's résumé. Two identities with different names on the same day is
  a rename; a different bus is a rewire; a different firmware is an update.
- **Boots** and **sessions** say how much of the total is recent. A motor with 30 hours over 200
  boots and one with 30 hours over 4 boots have lived different lives.
