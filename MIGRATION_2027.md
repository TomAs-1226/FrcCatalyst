# WPILib 2027 migration notes

```
                     (@)
                      |
                    \ | /
                     \|/
                     |=|
                  ___|=|___
              .:'''       ''':.
             /  (@)  (@)  (@)   \
            /___________________ \
           |                     |
           |                     |
           |_____________________|
            \                   /
             '-._____________.-'

              THE CAKE IS A LIE
   (the 2027 release is promised, not yet real)
```

Status: **planning / blocked on the ecosystem.** WPILib 2027 is at `2027.0.0-alpha-6`
(alpha, not beta). This branch (`wpilib-2027`) tracks what a migration will take.
`main` stays on WPILib 2026 and remains the shipping library until 2027 stabilizes
and the vendor libraries Catalyst depends on publish 2027 builds.

## Why this isn't merged yet

The migration cannot compile today because the dependencies Catalyst pulls in do
not have 2027 releases available:

- **WPILib 2027** artifacts are not in the local `~/wpilib/2026/maven` this build
  resolves from. A 2027 SDK install (or the online 2027 maven) is required.
- **CTRE Phoenix 6** (`com.ctre.phoenix6`) has no 2027 wpiapi build yet.
- **PathPlannerLib** and **PhotonVision** have no 2027 builds yet.

Until those land, a rename would just produce code that mixes the new
`org.wpilib` types with vendor jars still compiled against `edu.wpi.first`.

## The dominant change: package rename

WPILib 2027 reorganizes every Java package from `edu.wpi.first` to `org.wpilib`.
In this repository that is:

- **321 import lines across 69 files** under `src/main/java`.

This is mechanical. When the 2027 SDK and vendor jars are available, the bulk of
the work is a find-and-replace, then fixing anything the compiler flags:

```bash
# Run from the repo root ONLY once WPILib 2027 + 2027 vendor jars are installed.
grep -rl 'edu\.wpi\.first' src example | xargs sed -i '' 's/edu\.wpi\.first/org.wpilib/g'
```

Do this on `wpilib-2027` and let the compiler drive the rest. Do not run it on
`main`.

## What Catalyst does NOT have to change (verified against the source)

Catalyst is well-positioned. A scan of `src/main/java` found **no usage** of any
class WPILib 2027 removes:

- Legacy HAL / hardware: `Servo`, `Relay`, `Counter`, `Ultrasonic`, `AnalogGyro`,
  `AnalogOutput`, `AnalogTrigger`, analog/SPI IMUs, `NidecBrushless`, `Jaguar`,
  DMA, interrupts, Digital Glitch Filter, built-in accelerometer. Catalyst drives
  everything through CTRE Phoenix 6, so none of these appear.
- Removed command/util classes: `MotorControllerGroup`, `RamseteCommand`,
  `SwerveControllerCommand`, `MecanumControllerCommand`, `AxisCamera`. None used.
- Gamepad classes (`XboxController`, `PS4Controller`, `PS5Controller`,
  `StadiaController`), which 2027 consolidates into a single `Gamepad`. Catalyst
  does not reference them; driver binding is abstracted in the driver layer.
- `robotInit()`, which 2027 removes in favor of the `Robot()` constructor. The
  example already uses a `RobotContainer` field + `robotPeriodic()`, not
  `robotInit()`.
- `MotorController.set()` -> `setThrottle()` rename. Catalyst uses CTRE control
  requests, not WPILib `MotorController`.

## Items to review during migration

- **NetworkTables.** 2027 removes the NT3 wire protocol. The Java entry API
  (`table.getEntry(...)`) is used in ~18 files (telemetry, tunables). Confirm the
  entry API survives in the 2027 NT4 build; if it is finally removed, migrate
  those call sites to publishers/subscribers. The logging sinks already use NT4
  `StructPublisher` / `StructArrayPublisher`, so the modern path is in place.
- **Field coordinate system.** WPILib has signalled a possible field origin /
  orientation change for 2027. `AllianceFlipUtil` is the one place that encodes
  field dimensions and flip math, so it is the single file to re-derive if the
  origin moves. Its unit tests will catch a regression. (Not confirmed in the
  alpha changelog as of alpha-6; treat as a watch item.)
- **Commands v3.** Additive. The v2 command-based API Catalyst builds on is not
  removed in 2027, so nothing is forced. Adopting v3 idioms can come later, per
  mechanism, once the framework and its docs are final.
- **Phoenix 6 / PathPlanner / PhotonVision** 2027 releases may carry their own API
  changes; re-check `SwerveSubsystem`, `AimingSolver`, and the vision sources when
  those ship.

## Migration checklist

- [ ] WPILib 2027 SDK installed; 2027 maven reachable
- [ ] Phoenix 6, PathPlannerLib, PhotonVision 2027 builds available
- [ ] Bump `wpilibVersion` / `phoenixVersion` / `photonVersion` / PathPlanner in `build.gradle`
- [ ] Point the maven repos at `wpilib/2027/maven`
- [ ] Run the `edu.wpi.first` -> `org.wpilib` rename (command above)
- [ ] `./gradlew compileJava` and fix the compiler fallout
- [ ] `./gradlew test` (AllianceFlipUtil / aiming / math suites) green
- [ ] Re-check NetworkTables entry usage against the 2027 NT4 API
- [ ] Re-derive `AllianceFlipUtil` if the field origin changed
- [ ] Update the example (`Robot()` constructor is fine; verify vendor deps)
- [ ] Publish a `2.0.0-2027` prerelease
