---
layout: default
title: Shoot on the Move
parent: Advanced
nav_order: 5.2
---

# Shoot on the Move
{: .no_toc }

A swerve drivetrain that aims itself while the driver drives it: the whole drivebase is the turret.
{: .fs-6 .fw-300 }

{: .warning }
> Unreleased: on this line (`upgrade/alpha-7`) after 2.0.0-beta.2, so no release has it yet.
> **Ported and compiled here; not yet driven on a robot.** Phoenix 6 `26.70.0-alpha-2`, published
> 2026-09-18, is the first Phoenix build for WPILib alpha-7 — it requires **26.70.x device
> firmware** on every TalonFX, CANcoder and Pigeon — so a CTRE drivetrain can run this line now.
> What this page says about running on hardware (the Pigeon's turn rate, the facing request, the
> wiring example, the checks on the robot) was measured on `systemcore-alpha6`'s 2.0.0-alpha.4,
> now merged into this line; it has not yet been repeated on 26.70.x firmware. See
> [Versions and compatibility](https://tomas-1226.github.io/FrcCatalyst/versions.html).
> Built offline against a model of the Catalyst X1 fitted to its recordings, and **not yet
> driven**. Every number marked *(sim)* comes from that model.

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## The pieces

| Piece | Where | What it does |
|---|---|---|
| `HeadingTracker` | `util/` | Where to face and how fast that direction is turning, as a smooth reference for Phoenix's `FieldCentricFacingAngle`. Solves the shot (virtual goal, shooter-exit offset, any time-of-flight function), and adds a led feedforward and a disturbance observer. |
| `AimSpeedGovernor` | `util/` | Caps the driver's speed where the target's swing would outrun the turn the drivetrain has left, with an optional cap on closing speed. Also the "moving beyond the safe speed" gate for the shot. |
| `SwerveSetpointGenerator.Priority.ROTATION` | `subsystems/swerve/` | When translation and rotation together would take a module past its top speed, shrinks the translation and keeps the turn. |
| `VisionPoseSink.getYawRateRadPerSec()` | `subsystems/vision/` | The gyro's own turn rate. `SwerveSubsystem` reads its Pigeon 2; vision's spin gate and MegaTag2 use it too. |

`HeadingTracker` and `AimSpeedGovernor` are plain doubles, with no WPILib, Phoenix or NetworkTables
imports, so each step can be tested at a desk and the same source builds for WPILib 2026 and 2027.
Nothing publishes by itself; the [wiring example](#wiring-it) publishes the aim contract.

For a turret on its own mechanism, use `AimingSolver` and `TurretMechanism.track(...)` instead: see
[Turret & Shoot-On-The-Fly]({% link advanced/aiming.md %}). This page is for a robot with no turret,
where the drivetrain does the aiming. The drivetrain then has its own lag, turns short of what it is
told, and yaws while it strafes, and the classes here deal with all three.

---

## How it works

### The aim is solved for the shot

`HeadingTracker.solve(...)` finds the heading that lands the ball in the target.

- **Moving goal.** The ball carries the robot's velocity, so it is aimed at a *virtual goal*: the
  target less that velocity times the flight time.
- **Off-centre exit.** The ball leaves from the shooter's exit, which may sit off the robot's centre
  (`setShooterExit(x, y)`, robot frame). On a turning robot that exit also moves sideways. With the
  exit 0.29 m behind the centre and the robot turning at 2 rad/s, it moves at 0.58 m/s, which the
  ball carries. Aimed from the centre, that shot misses by 0.58 m per second of flight.
- **Any time-of-flight function.** The flight time depends on the distance to the virtual goal, and
  that distance depends on the flight time, so the solve iterates to a fixed point. A bisection would
  need the table to be monotonic; this does not. A table measured on team 5805's robot flies longer
  from 2.42 m than from 3.54 m, and the tests use it. The iteration stops at 20 steps and reports `converged == false`
  if it could not settle.

### A reference, not the raw aim

The heading the robot follows is not the aim itself, which carries all of the pose's noise.

- **Its own trajectory.** A second-order reference moves with the aim's swing and pulls itself onto
  the aim with bounded acceleration. The swing is worked out from the commanded velocity, so it
  carries no pose noise.
- **Never re-seeded.** The reference is never reset from the measured heading, so the measurement's
  noise never gets in.
- **A noise band.** While the aim stays inside a band around the reference, the reference does not
  chase it. Once the aim leaves the band, the reference closes all the way before it stops again.
- **Wide when still.** The band is wide standing still, where the aim moves only because the pose is
  noisy, and where every hundredth of a rad/s of turn command re-steers all four modules. It narrows
  as the robot speeds up. Inside the band the reference still moves with the swing, so the band costs
  no lag while driving.
- **The velocity actually delivered.** The velocity used is the commanded one, scaled by the share of
  it the drivetrain actually delivers. That share is learned slowly from the fused pose; the X1's pose
  moved 0.75-0.83 of its command.

### Feedforward, and what the gyro says

- **A led feedforward.** The reference's turn rate is fed forward, led by the drivetrain's answer time
  (`leadS`) using the reference's own acceleration.
- **A disturbance observer.** It compares the gyro's own turn rate with what the requests should have
  produced, and takes the difference out of the next feedforward. That removes the steady error a
  drivetrain leaves when it turns short of what it is told, or yaws while it translates. The X1 does
  both.
- **Resting when still.** Below 0.25 m/s the observer holds and then lets go, so it cannot wind up
  against static friction.
- **The gyro's rate.** The rate must be the gyro's, not the one the wheels report: see
  [the gyro rate](#the-gyro-rate).

### The heading loop

The heading loop itself is Phoenix's facing request, P only, in the drivetrain's odometry thread,
where the heading is fresh: 100-250 Hz against the main loop's 50. The tracker hands it:

- a direction to face, set halfway through the coming loop so the error is centred;
- a turn rate to feed forward, never more than `maxRateRadps`.

### At speed

At 2 m/s and up, a driver can translate faster than the drivetrain can turn to follow the target's
swing. Two pieces handle that, one before the tracker and one after it.

**Before the tracker: `AimSpeedGovernor`.**

- **Why speed.** Only the part of the velocity *across* the line to the target swings the aim.
  Passing a target `d` metres off at `v` m/s swings its bearing at `v/d` rad/s. Closing on it swings
  nothing.
- **The cap.** The speed is capped where that swing would use more than `1 − turnReserve` of the turn
  left. The turn left is the rate cap, or the modules' headroom above the speed, whichever is less.
- **The closing cap.** An optional cap on closing speed applies as well. The smaller of the two
  scales wins and the direction is kept.
- **After team 581.** This is how 581 structures its `DriveConstraints`: a radial and a tangential
  cap and an aim-state speed ceiling. Here the tangential cap is worked out from the drivetrain rather
  than set by hand.
- **Eased.** The cap moves at no more than `capSlewMps2`, so the robot eases down to it and back.
- **The tracker plans for it.** The tracker aims for the governed velocity, the one the robot will
  actually have.

**After the tracker: `SwerveSetpointGenerator` with `Priority.ROTATION`.**

- **Rotation kept.** The tracker says what turn it needs this loop. If that turn plus the translation
  would take any module past its top speed, the translation alone is shrunk along its own direction,
  by the largest factor that fits.
- **Solved per module.** Each module gives a quadratic, and the smallest root bounds the scale.
- **No desaturation.** Phoenix's own desaturation would shrink the turn too. With this, it never has
  to act.

**Not included: a skew or discretization term.** Phoenix's `FieldCentric` requests discretize already.
Catalyst once added its own, doubled the lateral correction, and took it back out: see the note in
`SwerveSubsystem`'s drive command. The X1 does run a 17 ms skew term of its own, measured on that
robot *after* Phoenix's discretization. It is robot-specific, and on the X1's model it destabilised an
ungoverned arc on its own, so it stays in the X1's code. [What it bought](#evidence) is below.

---

## Wiring it

This example is adapted from the X1's turret mode, simplified. It is a command that drives with the
left stick while the robot faces a target and leads the shot.

On this line the example compiles but cannot run: it drives a CTRE drivetrain, and Phoenix 6 has no
WPILib alpha-7 release yet.

```java
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.lib.catalyst.command.CatalystCommand;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.subsystems.swerve.SwerveSetpointGenerator;
import frc.lib.catalyst.subsystems.swerve.SwerveSubsystem;
import frc.lib.catalyst.util.AimSpeedGovernor;
import frc.lib.catalyst.util.HeadingTracker;
import frc.lib.catalyst.util.InterpolatingTable;
import frc.lib.catalyst.util.RobotState;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

public final class AimWhileDriving {
    /** The fastest turn the aim may ask for, rad/s: the tracker's, the governor's and the request's. */
    static final double MAX_TURN = 3.0;
    /** How long the feeder takes to get a ball to the shooter, s: Ready looks this far ahead. */
    static final double FEED_S = 0.25;
    /** How close to the aim counts as on target, rad. */
    static final double TOLERANCE = Math.toRadians(2.0);

    private final SwerveSubsystem drive;
    private final HeadingTracker tracker = new HeadingTracker();
    private final AimSpeedGovernor governor;
    private final SwerveSetpointGenerator allocation;
    private final SwerveRequest.FieldCentricFacingAngle facing;
    private double lastT = Double.NaN;

    public AimWhileDriving(SwerveSubsystem drive, InterpolatingTable shotTime, double exitX, double exitY) {
        this.drive = drive;
        Translation2d[] modules = drive.getDrivetrain().getModuleLocations();
        double radius = 0;
        for (Translation2d m : modules) {
            radius = Math.max(radius, m.getNorm());
        }
        tracker.config().maxRateRadps(MAX_TURN);
        tracker.setTimeOfFlight(shotTime::get);       // distance (m) -> flight (s); null for no lead
        tracker.setShooterExit(exitX, exitY);         // robot frame, m; (0, 0) for a centred shooter
        governor = new AimSpeedGovernor(drive.getMaxSpeedMPS(), radius);
        governor.config().maxTurnRadps(MAX_TURN);
        // The rotation is kept whole. 3% of the wheels' top speed is left for the facing request's own
        // correction, which Phoenix works out after this, in its odometry thread. No acceleration
        // limit: the governor already eases the speed.
        allocation = new SwerveSetpointGenerator(drive.getMaxSpeedMPS(), MAX_TURN,
                Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
                SwerveSetpointGenerator.Priority.ROTATION, modules);
        // P only, at the tracker's gain: its observer models this request.
        facing = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                .withTargetDirectionPerspective(SwerveRequest.TargetDirectionPerspectiveValue.BlueAlliance)
                .withHeadingPID(tracker.config().kP(), 0, 0)
                .withMaxAbsRotationalRate(MAX_TURN);
    }

    /** The sticks are -1..1 in the driver's frame; {@code target} is in field metres, blue origin. */
    public CatalystCommand command(Supplier<Translation2d> target, DoubleSupplier forward,
            DoubleSupplier left, DoubleSupplier turn) {
        return drive.run(() -> loop(target.get(), forward.getAsDouble(), left.getAsDouble(),
                        turn.getAsDouble()))
                .beforeStarting(() -> {
                    tracker.reset();
                    governor.reset();
                    allocation.reset();
                    lastT = Double.NaN;
                })
                // Whatever reads Ready must not go on reading the last loop's yes.
                .finallyDo(() -> {
                    CatalystLog.log("Aim/Ready", false);
                    CatalystLog.log("Aim/State", "IDLE");
                })
                .withName("aim while driving");
    }

    private void loop(Translation2d target, double forward, double left, double turn) {
        double now = Timer.getTimestamp();
        double dt = Double.isNaN(lastT) || now - lastT > 0.1 ? 0.02 : now - lastT;
        lastT = now;
        double speed = drive.getMaxSpeedMPS() * drive.getSpeedMultiplier();
        // The driver's frame into the blue-origin field frame everything here works in.
        double flip = RobotState.isRed() ? -1.0 : 1.0;
        Pose2d pose = drive.getPose();
        double heading = pose.getRotation().getRadians();

        // 1. Cap the driver where the target's swing would outrun the turn, so that the tracker aims for
        //    the velocity the robot will actually have.
        double[] v = governor.govern(dt, forward * speed * flip, left * speed * flip,
                pose.getX(), pose.getY(), target.getX(), target.getY());
        // 2. Where to face, and how fast that is turning, from the gyro's own rate.
        tracker.update(now, pose.getX(), pose.getY(), heading, drive.getYawRateRadPerSec(),
                v[0], v[1], target.getX(), target.getY());
        HeadingTracker.Aim aim = tracker.aim();
        if (aim == null) {
            // No usable pose this loop: the right stick turns, as it does without aiming.
            drive.driveFieldCentric(forward * speed, left * speed,
                    turn * drive.getMaxAngularRate() * drive.getSpeedMultiplier());
            CatalystLog.log("Aim/Mode", "stick");
            CatalystLog.log("Aim/Ready", false);
            return;
        }
        // 3. The turn this loop needs is the feedforward plus the heading loop's correction. It is kept
        //    whole; the translation gives way if both will not fit in the modules.
        double need = tracker.feedforwardRadps()
                + tracker.config().kP() * HeadingTracker.wrap(tracker.directionRad() - heading);
        need = Math.max(-MAX_TURN, Math.min(MAX_TURN, need));
        ChassisVelocities send = allocation.generate(new ChassisVelocities(v[0], v[1], need), dt,
                pose.getRotation());
        drive.getDrivetrain().setControl(facing
                .withVelocityX(send.vx)
                .withVelocityY(send.vy)
                .withTargetDirection(Rotation2d.fromRadians(tracker.directionRad()))
                .withTargetRateFeedforward(tracker.feedforwardRadps()));

        // 4. The aim contract, for Catalyst Console and anything else that reads it.
        ChassisVelocities moving = drive.getFieldRelativeSpeeds();
        boolean ready = tracker.onTargetIn(FEED_S, TOLERANCE)
                && !governor.movingBeyondSafeSpeed(moving.vx, moving.vy, pose.getX(), pose.getY(),
                        target.getX(), target.getY());
        boolean onTarget = Math.abs(tracker.aimErrorRad()) <= TOLERANCE;
        boolean driving = Math.hypot(moving.vx, moving.vy) > 0.25;
        CatalystLog.log("Aim/State", !onTarget ? "ALIGNING" : driving ? "SOTF" : "ALIGNED");
        CatalystLog.log("Aim/Target", new double[] {target.getX(), target.getY()});
        CatalystLog.log("Aim/AimPoint", new double[] {aim.aimX(), aim.aimY()});
        CatalystLog.log("Aim/HeadingErrorDeg", Math.toDegrees(tracker.aimErrorRad()));
        CatalystLog.log("Aim/DistanceMeters", aim.distanceM());
        CatalystLog.log("Aim/TimeOfFlightSeconds", aim.timeOfFlightS());
        CatalystLog.log("Aim/Ready", ready);
        CatalystLog.log("Aim/SpeedCapMps", governor.limiting() ? governor.capMps() : Double.NaN);
        CatalystLog.log("Aim/Mode", "V8");
    }
}
```

Things the example gets right that are easy to get wrong:

- **Two blue-alliance perspectives.** The facing request's velocity and its target direction are
  both in the blue-origin field frame, where the tracker works. Its default, the operator perspective,
  would flip the velocity on red.
- **Matching gain and cap.** The request's gain and rate cap match the tracker's `kP()` and
  `maxRateRadps()`, because the observer models that request.
- **The driver's velocity, not the measured one.** The tracker gets the commanded velocity, which is
  smooth and a loop ahead. It learns for itself how much of it the drivetrain delivers.
- **The gyro, not the wheels.** The turn rate is the gyro's, not `getChassisSpeeds().omega`.
- **Reset on start.** `reset()` runs at the start of every aiming command. The tracker also starts
  over by itself after a gap of more than 0.1 s, or a loop with anything that is not a number.

---

## The knobs

Every setting can change between loops, so each can be a live tunable (`TunableNumber` works). Each
setter clamps to the range given and ignores a value that is not a number.

### `HeadingTracker.Config`

The defaults are what the X1's harness found best across two fits of its chassis, inside the range
where the reference is stable.

| Knob | Default | Range | What it does |
|---|---:|---|---|
| `kP` | 6 (rad/s)/rad | 0-15 | The facing request's gain on the heading error. Give the request the same P. |
| `leadS` | 0.18 s | 0-0.3 | How far the feedforward leads: about how long the drivetrain takes to answer a turn. |
| `bandwidthRadps` | 7 rad/s | 1-12 | How fast the reference closes on the aim. Critically damped. |
| `maxAccelRadps2` | 12 rad/s² | 1-60 | The reference's acceleration limit. |
| `velocitySmoothingS` | 0.10 s | 0-0.5 | Smoothing on the velocity the aim swings with: a thumb holding a creep trembles. |
| `bandStillRad` | 1.2° | 0-5° | How far the aim may wander, standing still, before the reference chases it. |
| `bandMovingRad` | 0.3° | 0-5° | The same band at speed. |
| `bandFadeMps` | 0.6 m/s | 0.05-5 | The speed by which the band has narrowed from one to the other. |
| `bandInnerShare` | 0.15 | 0-1 | Once chasing, how close (as a share of the band) the reference gets before it stops. |
| `observerS` | 0.10 s | 0-2 | The disturbance observer's time constant. 0 turns it off. |
| `plantDelayS` | 0.06 s | 0-0.3 | The drivetrain's delay, as the observer models it. |
| `plantLagS` | 0.25 s | 0-0.3 | The drivetrain's lag, as the observer models it. |
| `learnDelivery` | on | | Whether the delivered share of the command is learned. |
| `deliveryS` | 0.6 s | 0.1-10 | How long that learning averages over. The share stays within 0.5-1.2. |
| `maxRateRadps` | 3 rad/s | 0.1-20 | The facing request's rate cap. The feedforward never exceeds it. |

### `AimSpeedGovernor.Config`

| Knob | Default | Range | What it does |
|---|---:|---|---|
| `maxModuleSpeedMps` | constructor | 0.5-10 | The modules' top speed (`getMaxSpeedMPS()`, Phoenix's `kSpeedAt12Volts`). |
| `moduleRadiusM` | constructor | 0.05-1 | The farthest wheel from the centre. |
| `maxTurnRadps` | 3 rad/s | 0.1-20 | The fastest turn the aim may ask for: the same as the tracker's `maxRateRadps`. |
| `enabled` | on | | Whether it caps. Off, `govern` passes the driver through; the gate still answers. |
| `turnReserve` | 0.25 | 0-0.9 | The share of the turn left that is kept back for the aim's corrections. Lower caps less. |
| `radialCapMps` | 0 (off) | 0-10 | A cap on speed toward or away from the target. |
| `capSlewMps2` | 4 m/s² | 0.5-50 | How fast the cap moves. |

`capMps()`, `capTargetMps()` and `limiting()` say what it did this loop. `safeSpeedMps(...)` and
`movingBeyondSafeSpeed(...)` answer for any velocity without changing anything. Use the gate on the
*measured* velocity: `limiting()` says the driver's command is being held down, and the gate says
whether the robot has actually slowed to the cap yet.

### `SwerveSetpointGenerator` with `Priority.ROTATION`

- **Construction.** Takes the module positions: `drive.getDrivetrain().getModuleLocations()`.
- **Which frame.** `generate(desired, dt, robotHeading)` takes a field-relative request. The older
  overloads judge the request as robot-relative.
- **What it leaves.** The rotation priority budgets the turn as if it were a little faster than asked
  (`allocationMargin(double)`, 0.5-1, default 0.97: the turn counts as 1/0.97 of itself), so the facing
  request's own correction has room in proportion to the turn. Driving straight nothing is held back, and
  straight-line top speed is never shaved. (The X1's own shaper shrinks the whole limit instead.)
- **What it did.** `getTranslationScale()` is the share of the translation it kept, 1 unless it had
  to give some up.
- **The default.** `Priority.PROPORTIONAL` is the behaviour every existing user already has.

---

## The /Catalyst/Aim contract

This is what a robot publishes about its aiming so Catalyst Console can draw it, whatever does the
aiming. The target is picked out, with a band on the carpet from the robot to it: grey while the robot
swings on, blue once it holds the target, broken into chevrons while it drives and stays on. With
`CatalystLog`, the key `Aim/Ready` publishes as `/Catalyst/Aim/Ready`.

| Key | Type | What it means | From the tracker |
|---|---|---|---|
| `State` | string | `IDLE`; `ALIGNING` while turning onto the target; `ALIGNED` once on it; `SOTF` on it while moving faster than 0.25 m/s. Console draws nothing on `IDLE`. | on target: `aimErrorRad()` within tolerance |
| `Target` | double[2] | The target, field metres `{x, y}`. | the target you passed in |
| `AimPoint` | double[2] | Where the aim actually points: the target, or while leading a shot the virtual goal. | `aim().aimX()`, `aimY()` |
| `HeadingErrorDeg` | double | How far the heading is from the aim, degrees. NaN when there is nothing to measure it from, never 0. | `aimErrorRad()` |
| `DistanceMeters` | double | From the shooter's exit to the aim point. Use it for shooter and hood lookups. | `aim().distanceM()` |
| `TimeOfFlightSeconds` | double | The ball's flight. | `aim().timeOfFlightS()` |
| `Ready` | boolean | A ball fed now would land. Set false the loop aiming ends. | `onTargetIn(feedTime, tol)` and not `movingBeyondSafeSpeed(...)` |
| `SpeedCapMps` | double | The governor's cap while it holds the driver down; NaN when it does not. | `limiting() ? capMps() : NaN` |
| `Mode` | string | Who has the rotation: the aiming controller's name (the X1 publishes `V8`, the name the tracker had there), or `stick` when there is no usable aim and the driver's stick turns the robot. | |

`Ready`, `SpeedCapMps` and `Mode` are the shoot-on-the-move additions. A robot that does not publish
them reads as "not said", and Console draws the rest as before.

---

## What to watch on the robot

These checks need a CTRE drivetrain, so on this line they wait for a Phoenix 6 release for WPILib
alpha-7. Until then they are for a robot on the alpha-6 line.

The tracker and the governor publish nothing themselves, so log what you want to watch. The X1 logs
every getter under its own table: `referenceRateRadps()`, `chasing()`, `disturbanceRadps()`,
`deliveredShare()`, `aimErrorRad()`, `trackingErrorRad()`, `feedforwardRadps()`, and the gyro rate
beside the turn requested.

1. **Park at the target, aiming, hands off.**
   - The modules should not move, `referenceRateRadps()` should read 0, and `chasing()` should go false.
   - If the modules twitch, raise `bandStillRad`.
2. **Creep along the target.**
   - No wheel should reverse.
   - `disturbanceRadps()` should stay near 0 below 0.25 m/s.
3. **Strafe at 1 m/s, then 2.**
   - Compare the heading error by speed with what you had before.
   - `deliveredShare()` should settle; on the X1's model it settled near 0.82.
   - `disturbanceRadps()` should flip sign when the strafe reverses.
   - If the heading swings, drop `kP` to 4, then `observerS` to 0.
4. **Then 3-4 m/s: a pass by the target, and an arc round it.** Watch `limiting()` and `capMps()`, which
   Console shows as `SpeedCapMps`.
   - The cap should bind only close to the target or in tight arcs, and ease the robot down smoothly.
   - If the driver finds it too eager, lower `turnReserve` toward 0, or switch the governor off.
   - `getTranslationScale()` stays at 1 until the wheels near their top speed. On the X1's model it
     never acted at 4 m/s or below.
5. **Settle the drivetrain's lag.** Record a hard 3 m/s strafe and compare the gyro rate with the turn
   requested.
   - A chassis trailing by about 0.25 s at speed matches the defaults.
   - At about 0.05 s, try `plantLagS` 0.07 and `leadS` 0.12.
6. **Keep a fallback.** Phoenix's facing request fed the plain bearing to the target (or
   `AimingSolver`'s bearing and rate) is one binding away, should anything misbehave on the carpet.

---

## Evidence

The X1 built these against a model of itself, not on the carpet.

- **The model.** The X1's `SotfPlant` was fitted to its 2026-09-17 recordings. It includes Phoenix's
  discretization and desaturation, the azimuth response, wheel spin-up, one traction budget shared by
  translation and yaw, and the chassis turning short of its wheels.
- **Two fits.** Fit A follows the recorded paths. Fit B has the chassis as quick as the X1's in-place
  turns suggest. Every controller was judged on both, so that no conclusion depends on which fit is
  right.
- **Where it lives.** The harness stays in the X1's repository, calibrated to those recordings. Its
  `docs/sotf-v8.md` has every table.

**Below 2 m/s** *(sim, fit A)*, each cell is module activity (deg/s), error RMS, and lag (ms):

- "Facing request" is Phoenix's `FieldCentricFacingAngle` on its own, with kP 8, kD 0.1 and a 0.12 s
  lookahead.
- "V7" is 5805's controller.
- "Tracker" is `HeadingTracker`, as the X1 runs it. Below 2 m/s the X1's speed shaping barely acts:
  in strafes and passes its skew term moved the aim by 0.2° at most.

| Run | Facing request | V7 | Tracker |
|---|---|---|---|
| standstill | 131 / 0.75° / - | 3985 / 0.70° / - | **131 / 0.58° / -** |
| creep | 3013 / 1.08° / −270 | 2362 / 6.91° / −280 | **1749 / 1.06° / −235** |
| strafe 0.5 m/s | 849 / 2.38° / −205 | 819 / 13.62° / −60 | **284 / 0.79° / −25** |
| strafe 1.0 m/s | 817 / 4.53° / −210 | 699 / 7.96° / +100 | **454 / 1.43° / −25** |
| strafe 1.5 m/s | 886 / 7.10° / −220 | 716 / 10.52° / +190 | **574 / 1.92° / −15** |
| arc 2.0 m/s | 883 / 12.63° / −210 | 722 / 21.06° / +275 | **616 / 3.36° / −10** |
| lead 1.8 m/s, 5805's flight table and exit | 904 / 11.98° / −155 | 807 / 24.25° / +320 | **379 / 5.75° / +80** |
| replay of the 21:10 recording | 1480 / 6.89° / −250 | 1162 / 20.86° / +400 | **1138 / 1.94° / −25** |

On the robot itself, in that 21:10 run, the facing request it ran then (kP 6, no lookahead) missed by
more the faster it went: 7.6°, 6.8°, 8.6°, 13.9° and 17.6° RMS in the <0.3, 0.3-0.8, 0.8-1.3, 1.3-2.0
and >2.0 m/s bands.

**At speed** *(sim)*, each part on its own:

- **Governor.** A 3 m/s arc round the target sits at the X1's traction limit: 4.1 m/s² of centripetal
  acceleration out of a 6 m/s² budget.
  - Without the governor the tracker loses the circle: 44° RMS (fit A) and 24° (fit B).
  - The governor alone brings it to 8.7° and 14.8°, for 15-28% of the mean speed there.
  - In straight strafes it cost almost nothing, and in a 4 m/s pass by the target about 6% of the mean
    speed.
- **Rotation priority.** In a 5 m/s pass, Phoenix desaturated 25-35% of the time without it, and never
  with it. The error went from 4.71° to 4.15° on fit A, and from 6.84° to 6.99° on fit B. At 4 m/s
  and below it never engaged.
- **The X1's skew term (not in the library).**
  - It straightened the driven line: path skew went from 0.85° to 0.22° at 2 m/s on fit A.
  - It moved the aim by 0.2° at most in every strafe and pass.
  - On fit B, the X1's full stack held the governed 3 m/s arc at 9.9°, against the governor alone's
    14.8°. The rotation priority never engaged there, so the skew term accounts for the difference.
  - On its own, without the governor, it tipped that arc on fit B from 24° to 56°.
  - A robot that wants it measures its own residual after Phoenix's discretization first.
- **An acceleration limit.** `SwerveSetpointGenerator`'s cap is the same idea, and it cost more than it
  gave. It halved the error in 3-4 m/s strafes, mostly by making the robot slower: at 4 m/s² the 4 m/s
  strafe's mean speed fell from 1.65 to 0.90 m/s. It wrecked arcs: the 2 m/s arc went from 3.3° to
  8.3° at 6 m/s², and to 46° at 5 m/s² (fit A). That is why the example passes no acceleration limit.

### The gyro rate

The pose sink's `getYawRateRadPerSec()` is what the tracker should be handed, and vision's spin gate
and MegaTag2 now read it too.

- **The wheels are wrong exactly when it matters.** In the X1's 2026-09-17 recordings, the rate the
  wheel kinematics report disagreed with the Pigeon's in 13 of 41 turning seconds, some of them in
  sign.
- **They are early.** At every turn onset the wheels reported the full rate 100-200 ms before the
  gyro saw it, while the modules fought and slipped.
- **`SwerveSubsystem` reads the Pigeon 2.** It overrides the method with the Pigeon's
  `AngularVelocityZWorld`. On this line that is compiled and, with Phoenix 6 `26.70.0-alpha-2` and
  26.70.x device firmware, can run — it just has not yet been driven on a robot.
- **Other sinks keep the wheels.** The default is still `getChassisSpeeds().omega`, so a sink written
  before this keeps working unchanged. Override it if the sink has a gyro.
