---
title: Numbers desktop simulation validation
parent: Advanced
---

# Numbers desktop simulation validation

**Validation update, 17 October 2026.** The team-specific `NumbersShootingSimTest`
failure recorded during RC1 preparation was diagnosed in an isolated Numbers review.
The repair changes its test fixture and desktop drivetrain model. FrcCatalyst
`v2.0.0-rc.1`, real-robot gains, shooting tables, calibration, and shot permission
rules are unchanged. The isolated Numbers patch has not been deployed.

## Reproducer and cause

The test constructs Numbers directly, outside `RobotBase`. Its fixture omitted the
framework's NetworkTables Tunables backend and produced hundreds of warning stacks.
Long loop gaps could invalidate motion and temporarily remove aiming speed limits.
The repaired fixture installs that backend and closes native simulation resources
before HAL shutdown, including after failed assertions.

Correcting the fixture alone still failed translation release: the measured speed
was 0.1697 m/s with zero translation requested and yaw still commanded. Phoenix's
independent drive-wheel plants used only 0.001 kg·m² of output-wheel inertia, equivalent
to 1.55 kg of total translational load at the configured two-inch wheel radius.
For the installed motor model and unchanged gains, this nearly unloaded plant's
sampled feedback stability limit is approximately 1.20 ms; the callback runs at 5 ms.

The simulation now reflects the measured 139.8 lb (63.412213326 kg) chassis mass
equally across its four wheels:

`J_sim = 0.001 + (mass_kg / 4) × wheel_radius_m² = 0.0419110235494 kg·m²`.

An explicit simulation branch retains **0.001 kg·m² on real hardware** before native
configuration marshaling. PID, feedforward, current limits, timer cadence, and all
32 original shooting assertions remain unchanged.

## Recorded checks

| Check | Outcome |
|---|---|
| Correct fixture, old plant | Failed: 0.1697 m/s after translation release |
| Derived chassis load, original native shooting assertions | Passed: 0.7529 m/s for a 0.75 m/s capped request; 0.00171 m/s after translation release with continued yaw |
| Full Numbers build and package | Exit 0; 623 tests, 0 failures, 0 errors, 0 skipped |
| Fresh native/model repeat | Exit 0; 4 tests, 0 failures, 0 errors |
| `SwerveSimulationModelTest` | Actual WPILib motor plant and configured gains at 5 ms and intermittent 12 ms updates; translation release preserves requested yaw; real-hardware inertia branch unchanged |

The isolated patch is based on Numbers `1eba2894f3799019bcfb70f369ea7eaec5ca4dd7`
on `codex/ares-1.1-bug-review`. The retained review bundle contains
`Numbers-shooting-simulation-fix.patch`, `results.json`, full and repeated JUnit XML,
and the failing diagnostic traces. Those recorded checks qualify that exact desktop
patch, not a different downstream revision.

## Qualification boundary

Independent wheel plants still approximate coupled chassis motion. These checks
do not prove physical traction, bump crossing, collision behavior, sensor mounting,
or shot accuracy. Native CAN configuration-read observations from RC1 preparation
remain separate from this simulation diagnosis.

[ARES 1.1]({% link advanced/ares-version-scope.md %}) remains the incremental current
multicamera work included in RC1. The ARES 2.0 ROS 2-style prototype remains separate;
this simulation repair neither integrates it nor enables experimental control.
