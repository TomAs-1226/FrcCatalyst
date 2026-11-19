---
layout: default
title: System Check
parent: Advanced
nav_order: 7
---

# System Check
{: .no_toc }

Pre-match self-test — catch the failures that lose matches before you queue.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Why

The failures that cost matches are rarely software bugs — they're a loose
connector, an inverted motor after a swap, a follower that didn't get
flashed, an encoder reading zero, a CAN device that didn't boot. You find
them in the pit if you're lucky, on the field if you're not.

`SystemCheck` runs every subsystem through a verification routine from one
button and gives you a green/red board: a per-test pass/fail, a single
"ready to compete" boolean, and a copy-paste report.

---

## Usage

```java
SystemCheck check = SystemCheck.builder("PreMatch")
    // instant checks — evaluated once
    .check("Battery >12V", () -> RobotState.batteryVoltage() > 12.0)
    .check("Gyro alive",   () -> Double.isFinite(drive.getHeading().getDegrees()))
    .check("No CAN conflicts", () -> CANRegistry.size() > 0)

    // timed checks — apply an action, confirm the result, clean up
    .timed("Elevator moves up",
           () -> elevator.getMotor().setVoltage(1.5),     // action each loop
           1.0,                                            // for 1.0 s
           () -> elevator.getVelocity() > 0.05,            // pass: it actually moved
           () -> elevator.getMotor().stop())               // cleanup
    .timed("Elevator current sane",
           () -> elevator.getMotor().setVoltage(1.5),
           0.5,
           () -> elevator.getMotor().getStatorCurrent() < 60,
           () -> elevator.getMotor().stop())
    .timed("Intake follower aligned",
           () -> intake.runAtVoltage(2.0),
           0.5,
           () -> followerMatchesLeaderSign(intake),        // your own helper
           () -> intake.stop())
    .build();

pit.menu().onTrue(check.run());
```

Run it when nothing else is commanding the mechanisms — `SystemCheck`
declares no subsystem requirements, so it's on you to run it in the pit
rather than mid-match.

---

## Check types

| Method | When |
|---|---|
| `check(name, BooleanSupplier)` | instant pass/fail — battery, gyro, sensor sanity |
| `timed(name, action, seconds, pass, cleanup)` | apply something, confirm a result, clean up — motor moves, current sane, follower aligned |

A `timed` check applies `action` every loop for `seconds`, then evaluates
`pass`, then always runs `cleanup` (even if interrupted). The classic use is
"drive the motor and confirm the encoder velocity is non-zero" — which
catches a dead motor, a disconnected encoder, or a follower wired backwards.

---

## Reading the results

Everything publishes to `/Catalyst/SystemCheck/<name>/`:

| Key | Value |
|---|---|
| `<test name>` | `PASS` or `FAIL: <reason>` |
| `Ready` | boolean — true only if every test passed |
| `Report` | full plain-text report |

From code:

```java
SystemCheck.Results r = check.results();
boolean go = r.ready();
for (SystemCheck.Result t : r.all()) {
    System.out.println(t.name() + (t.passed() ? " ok" : " FAILED: " + t.detail()));
}
System.out.println(r.report());
```

Put `Ready` on the driver-station dashboard as a big green/red light and
you've got a go/no-go you can trust before every match.

---

## Pattern: pair with the Health Dashboard

`SystemCheck` is the *active* pre-match probe; the
[Health Dashboard](health.html) is the *passive* in-match monitor. Run the
check in the pit, then let `HealthMonitor` watch for over-current /
over-temp / stall the rest of the match. Buggy check lambdas are
exception-guarded, so a typo in a test can't crash the routine — it just
fails that test.
