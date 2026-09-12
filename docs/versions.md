---
layout: default
title: Versions and compatibility
nav_order: 1.5
---

# Versions and compatibility
{: .no_toc }

Which Catalyst goes on which robot, what each version runs on, and where each one lives.
{: .fs-6 .fw-300 }

**Checked 11 September 2026** against the repository's tags and branches, JitPack, the vendordeps
this site serves and the upstream
[Systemcore compatibility matrix](https://github.com/wpilibsuite/SystemcoreTesting#software-compatibility),
whose README last changed on 5 September 2026. A clone of the library re-checks all of it with one
command, `python tools/catalyst-versions.py --online` ([below](#check-it-yourself)).

1. TOC
{:toc}

---

## If you are...

| If you are | Use | Get it from |
|:--|:--|:--|
| **Competing this season on a roboRIO** | **Catalyst 1.12.0** on WPILib 2026.2.1, the NI Driver Station and Phoenix 6 26.1.1 | JitPack: `com.github.TomAs-1226:FrcCatalyst:v1.12.0`. See [Installation](getting-started/installation). |
| **Testing on a Systemcore with CTRE motors, today** | **Catalyst 2.0.0-alpha.3** on Systemcore OS image 13, WPILib 2027.0.0-alpha-6, the 2027 Driver Station alpha-6 and Phoenix 6 26.50.0-alpha-1 | A source build of tag `v2.0.0-alpha.3` (commit `14e2080`), branch `systemcore-alpha6`. See [Building 2.0.0-alpha.3](#build-alpha3). It is not on JitPack and cannot be. |
| **Trying WPILib alpha-7 on Systemcore image 14** | **Catalyst 2.0.0-beta.1**, to port and compile your code against alpha-7 &mdash; not to drive. Phoenix 6 and PathPlannerLib have no alpha-7 release. | JitPack: `com.github.TomAs-1226:FrcCatalyst:v2.0.0-beta.1` |

{: .warning }
> **A robot with CTRE motors cannot run on Systemcore image 14 / WPILib alpha-7 yet.** Phoenix 6 has
> no alpha-7 release: its newest build, 26.50.0-alpha-1, is for alpha-5/6. On
> [SystemcoreTesting #378](https://github.com/wpilibsuite/SystemcoreTesting/issues/378)
> (5 and 6 September 2026) CTRE said its alpha-7 release will come a few business days after
> RobotPy's, that until then its release supports WPILib alpha-6 on image 13, and that editing a
> vendordep's year to force an older build into an alpha-7 project is unsupported on Systemcore.
> RobotPy has not released alpha-7; the newest on PyPI is `2027.0.0a6.post4`. WPILib's advice in the
> same thread is image 13, WPILib alpha-6 and the alpha-6 Driver Station, which is the stack
> 2.0.0-alpha.3 is built for.

---

## The three lines

| | **1.12.0** | **2.0.0-alpha.3** | **2.0.0-beta.1** |
|:--|:--|:--|:--|
| Status | Stable: the competition line | Newest code for WPILib alpha-6. Tagged `v2.0.0-alpha.3`, 11 September 2026. | Pre-season beta |
| Robot controller | roboRIO | Limelight Systemcore | Limelight Systemcore |
| Systemcore OS image | &mdash; | 10 to 13; built for 13 | 14 or newer |
| WPILib | 2026.2.1 | 2027.0.0-alpha-6 | 2027.0.0-alpha-7 |
| Driver Station | NI Driver Station | 2027 Driver Station `v2027.0.0-alpha-6`, or NI | 2027 Driver Station, or NI |
| Java, Gradle, commands | 17, 8.x, v2 | 25, 9.7+, v3 | 25, 9.7+, v3 |
| Git tag | [`v1.12.0`](https://github.com/TomAs-1226/FrcCatalyst/releases/tag/v1.12.0) at `cc5cadf` | [`v2.0.0-alpha.3`](https://github.com/TomAs-1226/FrcCatalyst/tree/v2.0.0-alpha.3) at `14e2080`, tagged 11 September 2026 | [`v2.0.0-beta.1`](https://github.com/TomAs-1226/FrcCatalyst/tree/v2.0.0-beta.1) at `36315c9` |
| Branch | `main` | `systemcore-alpha6` | `upgrade/alpha-7` |
| JitPack | builds | cannot build ([why](#source-only)) | builds |
| Vendordep URL | [`/vendordep/FrcCatalyst.json`](https://tomas-1226.github.io/FrcCatalyst/vendordep/FrcCatalyst.json), which installs **1.12.0** | none | none |
| Documentation | this site | the [beta site](https://tomas-1226.github.io/FrcCatalyst/beta/) documents 2.0.0-alpha.3 | not published yet: `docs/` on `upgrade/alpha-7` |
| Measured | <span class="proven">JitPack build</span> | <span class="proven">alpha-6 builds on a Systemcore bench</span><br><span class="attempted">driven on a robot</span> | <span class="proven">JitPack build</span><br><span class="attempted">run on image 14</span><br><span class="attempted">driven on a robot</span> |

A filled mark was measured; a dashed one has not been. Image 13 is `limelightosr-2027.0.0-beta13` on
beta Systemcore units and `alpha13` on alpha units; image 14 (`beta14`, `alpha14`) came out on
2 September 2026 titled "(REQUIRES WPILIB ALPHA 7)". The NI Driver Station works with every image, but
OpMode selection, which Catalyst 2.x's OpMode autos rely on, and Alerts need the 2027 Driver Station.

---

## Vendor libraries

Catalyst builds on **Phoenix 6** and **PathPlannerLib**, and on Systemcore its vision source is
**LimelightLib**. For WPILib 2027 the source is the upstream
[third-party table](https://github.com/wpilibsuite/SystemcoreTesting#software-compatibility); for
2026 it is Catalyst's own pins. "No release" means upstream lists no compatible build.

| Library | 1.12.0, WPILib 2026.2.1 | 2.0.0-alpha.3, WPILib alpha-6 | 2.0.0-beta.1, WPILib alpha-7 |
|:--|:--|:--|:--|
| **CTRE Phoenix 6** | 26.1.1 | 26.50.0-alpha-1 | **no release** |
| **PathPlannerLib** | 2026.1.2 | 2027.0.0-alpha-3 | **no release** |
| **LimelightLib** | not a dependency | the alpha-5/6 build; Catalyst pins 2.0.0-beta2 | the alpha-7 build; Catalyst pins 2.0.0-beta8-alpha7 |
| PhotonVision | v2026.3.1 | not in the matrix; Catalyst 2.x does not use it | not in the matrix; Catalyst 2.x does not use it |
| REVLib | a 2026 release | 2027.0.0-alpha-2 | no release |
| ReduxLib | a 2026 release | 2027.0.0-alpha-6 | no release |
| ThriftyLib | a 2026 release | 2027.0.0-alpha-1 | no release |
| ChoreoLib | a 2026 release | no release | no release |
| AdvantageKit | a 2026 release | 27.0.0-alpha-4 | 27.0.0-alpha-5 |

{: .note }
> **2.0.0-beta.1 still depends on the alpha-5/6 builds.** Its POM names Phoenix 6 26.50.0-alpha-1
> and PathPlannerLib 2027.0.0-alpha-3, the newest either vendor has. That is enough to compile against
> alpha-7. It is not a Phoenix 6 that runs a CTRE device on image 14.

---

## Apps and the agent

| | 1.12.0 | 2.0.0-alpha.3 | 2.0.0-beta.1 |
|:--|:--|:--|:--|
| **Catalyst App** | **1.4.3**, the published release (`app-v1.4.3`, branch `main`). It bundles 1.7.0; install 1.12.0 as on [Installation](getting-started/installation). | **2.6.1** for its tools. Its install button puts in 2.0.0-beta.1, which is for image 14, so add alpha.3 by hand. | **2.6.1**, which bundles 2.0.0-beta.1 |
| **Catalyst Console** | **1.0.0**, the published release (branch `main`), or 1.4.3 with the auto chooser tile at `/SmartDashboard/Auto Selector` | **1.4.3**, with the auto chooser tile at `/SmartDashboard/Auto Selector` | **1.4.3**; its defaults, `/Auto Selector` included, are beta.1's |
| **catalyst-agent** | &mdash; (Systemcore only) | **2.0.3** | **2.0.3** |

App 2.6.1 and Console 1.4.3 live on their repositories' `systemcore` branches and have not been
published as GitHub releases, so the Releases pages still offer App 1.4.3 and Console 1.0.0.
catalyst-agent is built from `agent/` in the library; 2.0.3 is the same package on 2.0.0-alpha.2,
alpha.3 and beta.1, and 2.0.0-alpha.1 carried 2.0.0.

---

## Every 2.x version

| Version | Commit, date | WPILib | Where it lives | JitPack | What it is |
|:--|:--|:--|:--|:--|:--|
| `v2.0.0-alpha.1` | `702143a`, 23 Aug | 2027.0.0-alpha-6-360-gcaf0fb2dc | tag | builds | Built on a WPILib CI snapshot, not a release. Measured on 26 August: a build on that snapshot does not start on image 13. Superseded, but the [beta vendordep](https://tomas-1226.github.io/FrcCatalyst/beta/vendordep/FrcCatalyst.json) still installs it, because it is the only 2.x tag before beta.1 that JitPack can build. |
| `v2.0.0-alpha.2` | `4c81114`, 6 Sep | 2027.0.0-alpha-6 | tag | fails | The first 2.x built on the WPILib alpha-6 release, which is what image 13 runs. Superseded by 2.0.0-alpha.3, which the beta site now documents. |
| `v2.0.0-alpha.3` | `14e2080`, 11 Sep | 2027.0.0-alpha-6 | tag; branch `systemcore-alpha6` | cannot build | alpha.2 plus Autonomy 2.0 and the fix that stops an unsolved MegaTag2 throwing away the MegaTag1 pose beside it. The newest code for image 13. |
| `v2.0.0-beta.1` | `36315c9`, 9 Sep | 2027.0.0-alpha-7 | tag; branch `upgrade/alpha-7` is this plus later docs | builds | alpha.3 plus the move to WPILib alpha-7 and the default field-width fix. |

### Why the alpha-6 builds are source-only {#source-only}

The WPILib 2027.0.0-alpha-6 release is on no public Maven repository: `frcmaven.wpi.edu` answers 404
for it in both `release` and `development` (checked 10 September 2026). JitPack therefore cannot
resolve WPILib and fails, which is what it recorded for `v2.0.0-alpha.2`. The WPILib installer
carries alpha-6 in its own local Maven, so a source build on a machine with that installer works.
Alpha-7 is on `frcmaven/release`, which is why 2.0.0-beta.1 builds on JitPack again.

---

## Building 2.0.0-alpha.3 from source {#build-alpha3}

You need JDK 25 and the WPILib 2027 alpha-6 installer on the machine (alpha-6 installs into a
`2027_alpha5` folder, and Catalyst's build looks there).

1. Clone and check out the tag:

   ```bash
   git clone https://github.com/TomAs-1226/FrcCatalyst.git
   cd FrcCatalyst
   git checkout v2.0.0-alpha.3
   ```

2. Publish it:

   ```bash
   ./gradlew publishToMavenLocal
   ```

3. Set the robot project up as in Option 3 of the beta site's
   [Installation](https://tomas-1226.github.io/FrcCatalyst/beta/getting-started/installation) page,
   with `com.frccatalyst:FrcCatalyst:2.0.0-alpha.3`.

---

## Branches and tags

Versions ship as tags. A branch is where work continues, and today there is one for each WPILib:

| Branch | Holds | WPILib |
|:--|:--|:--|
| `main` | 1.x: `v1.12.0` and the Pages workflow. Builds this site. | 2026.2.1 |
| `systemcore` | `v2.0.0-alpha.3` and later docs. Builds the [beta site](https://tomas-1226.github.io/FrcCatalyst/beta/). | 2027.0.0-alpha-6 |
| `systemcore-alpha6` | `v2.0.0-alpha.3`: `v2.0.0-alpha.2` plus Autonomy 2.0, docs and the MegaTag1 fix. Where alpha-6 work continues. | 2027.0.0-alpha-6 |
| `upgrade/alpha-7` | `systemcore-alpha6` plus the alpha-7 upgrade: `v2.0.0-beta.1` and later docs. | 2027.0.0-alpha-7 |
| `wpilib-2027` | An early start on 2027 that stopped at 1.3.2 on WPILib 2026. It is not the 2027 code. | 2026.2.1 |

A version number in a local Maven repository is whatever `build.gradle` said when that build was
published, which is not always the tag with the same number. The script below traces every local
build to the commit its sources came from.

---

## Check it yourself

From a clone of the library, on `main`, `systemcore` or `upgrade/alpha-7`:

```bash
python tools/catalyst-versions.py            # tags, branch heads and what each pins; Maven local builds
python tools/catalyst-versions.py --online   # plus the upstream matrix, JitPack and the published vendordeps
```

It only reads, and needs nothing beyond Python 3.8. Where it disagrees with this page, the script
is right and this page is out of date.
