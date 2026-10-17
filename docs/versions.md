---
layout: default
title: Versions and compatibility
nav_order: 1.5
---

# Versions and compatibility

Checked **17 October 2026** against the release artifacts, active source pins and served vendordeps.
Choose the controller and WPILib season first; desktop app versions are separate from library versions.

| Robot platform | Library | Required source stack | Install |
|---|---|---|---|
| roboRIO / WPILib 2026 | **1.12.0 stable** | Java 17, WPILib 2026.2.1; use the 2026 vendor libraries | [Stable installation](getting-started/installation.html) |
| Systemcore / WPILib 2027 alpha-7 | **2.0.0-rc.1 prerelease** | Java 25, WPILib 2027.0.0-alpha-7, Phoenix 26.70.0-alpha-2 and 26.70.x device firmware | [RC1 release](https://github.com/TomAs-1226/FrcCatalyst/releases/tag/v2.0.0-rc.1), [2.x installation](https://tomas-1226.github.io/FrcCatalyst/beta/getting-started/installation.html) |
| Existing Systemcore alpha-6 project | Historical source-only line | Its original alpha-6 stack and image 13 | Keep its matching source checkout; use the 2.x migration guide before changing the stack |

The current RC1 source pairs alpha-7 with Systemcore OS beta 14. Check the controller image's own
compatibility requirements before an upgrade. A later image number by itself does not certify a
different WPILib build. The NI and 2027 Driver Stations have different OpMode/alert support; use the
matching 2027 Driver Station for Catalyst 2.x OpMode selection.

Phoenix **26.70.0-alpha-2** provides alpha-7 support. Old instructions saying that no alpha-7 Phoenix
exists are obsolete. RC1 is still a prerelease: desktop verification is separate from on-robot
qualification, physical calibration and competition readiness.

## Published dependency URLs

**roboRIO / 2026:**

```text
https://tomas-1226.github.io/FrcCatalyst/vendordep/FrcCatalyst.json
```

**Systemcore / 2027 alpha-7:**

```text
https://tomas-1226.github.io/FrcCatalyst/beta/vendordep/FrcCatalyst.json
```

The stable URL remains on 1.12.0; the beta URL installs 2.0.0-rc.1. RC1's JitPack POM and release
asset were checked to agree. The tag is immutable. See the [download hub](downloads.html) for
desktop apps and the separately versioned Systemcore agent package.

## RC1 source pins

| Component | Version |
|---|---|
| WPILib | `2027.0.0-alpha-7` |
| Java | `25` |
| Phoenix 6 | `26.70.0-alpha-2` |
| CTRE device firmware | `26.70.x` |
| LimelightLib | `2.0.0-beta9-alpha7` |
| Optional PathPlanner | `2027.0.0-alpha-4.1` |
| catalyst-agent | `2.0.3`, separate on-controller service |

Installing a desktop app does not upgrade robot firmware. Installing an agent package does not
deploy robot code. Preserve measured device IDs, wheel/hood setup and camera transforms when
moving between supported software stacks.

## ARES and validation scope

ARES 1.1 is incremental current-library multicamera work. The separate ARES 2.0 localization
prototype remains deferred and is not included in RC1. Adding cameras still requires unique
identities, measured mounts and validation before enabling autonomous assistance.

The previously reported Numbers shooting-simulation failure was isolated to missing simulated
chassis load and incomplete test setup. A desktop-only model/fixture repair passed 623 tests and
a fresh four-test repeat with the original assertions and real-robot tuning unchanged. This
does not establish real shot accuracy, wheel grip or bump-crossing performance.

## Check a checkout

```text
python tools/catalyst-versions.py --online
```

This reads branch/tag pins, dependency artifacts and the published vendordeps. Release notes
and installable artifacts are at [GitHub Releases](https://github.com/TomAs-1226/FrcCatalyst/releases).
