# AGENTS.md: FrcCatalyst

Instructions for AI coding agents working in this repo: Claude, and whatever model runs when Claude
isn't available. For humans, `README.md` and `CONTRIBUTING.md` are the guides.

The shared method for every repo here — the commands, the push rules, which model does what — lives
in `~/dev/devtools/AGENTS.md`. This file is what is true about *this* repo.

## What this is

FrcCatalyst is an FRC robot library in Java: pre-built mechanisms, hardware wrappers, Physics Core
and autonomy for CTRE Phoenix 6 on WPILib. A change here reaches every robot that builds against it,
and the library's own tests are the gate.

| Path | What it is |
|---|---|
| `src/` | The library and its tests |
| `example/` | Example robot code |
| `agent/` | The **on-robot Systemcore service**, packaged as an `.ipk`. Not AI configuration |
| `docs/` | The documentation website, including the browser tools and the published vendordep |
| `tools/catalyst-versions.py` | Prints the live version map: tags, branches, Maven local builds traced to commits, JitPack, the published vendordeps |

Generated or machine-local, don't edit: `build/`, `graphify-out/`, `ctre_sim/`, `vendordeps/`,
`networktables.json`.

## Three lines at once

Run `git branch --show-current` first, then `python tools/catalyst-versions.py --online` when you need
the live picture. `docs/versions.md` on `main` is the same map written for people.

| Line | Branch | Runs on | How it installs |
|---|---|---|---|
| 1.x, the competition line | `main` | roboRIO, WPILib 2026, Java 17 | JitPack, through the stable vendordep |
| 2.0.0-alpha.3 | `systemcore-alpha6`, published from `systemcore` | Systemcore image 13, WPILib 2027 alpha-6, Java 25 | **source build only** |
| 2.0.0-beta.1 | `upgrade/alpha-7` | Systemcore image 14, WPILib 2027 alpha-7, Java 25 | JitPack; cannot drive a CTRE robot until CTRE ships alpha-7 |

- **A version names one build.** Never publish a second build under a version that already exists.
  `devtools release FrcCatalyst <version>` does the bump, the CHANGELOG date, the checks, the commit
  and the tag in one step, and stops before pushing.
- **The 2.x API is not the 1.x API** most training data describes. It targets WPILib 2027 and
  Commands v3: no `SubsystemBase`, no `CommandScheduler.getInstance()`, `ChassisSpeeds` is
  `ChassisVelocities`, `Timer.getFPGATimestamp()` is `Timer.getTimestamp()`. Confirm every name
  against `src/` on the current branch.

## Build and test

Use the toolchain wrapper, which picks the JDK from the branch's `build.gradle`:

```
python C:/Users/yu_th/dev/devtools/devtools.py gradle -- build
```

There is no `java` on PATH, and the ambient `JAVA_HOME` is JDK 21, which fits neither line (1.x needs
the WPILib 2026 JDK 17; 2.x needs Adoptium 25). Trust gradle's exit status: a failed run leaves the
previous run's `build/test-results` in place. Running 2.x robot code also needs two JVM flags; see the
README section "Catalyst 2.x needs two JVM flags".

## Docs site (`docs/`)

Jekyll, with the just-the-docs remote theme. `docs/` is the site root.

- **Publishing.** `.github/workflows/docs.yml` on `main` builds one GitHub Pages deployment: `main`'s
  `docs/` at the root and `systemcore`'s under `/beta/`. On 2.x branches that workflow only asks
  `main` to rebuild. Docs on `systemcore-alpha6` or `upgrade/alpha-7` are not published until they
  reach one of those two branches.
- **Nav and titles** come from front matter (`title`, `nav_order`, `parent`, `has_children`). Every
  page needs a `title`: CatalystApp bundles all `.md` under `docs/` into its MCP server by it.
- **Links.** `{% link %}` resolves against `docs/`: `{% link advanced/foo.md %}`, never
  `{% link docs/advanced/foo.md %}`.
- **Per-branch config.** `_config.yml` differs by branch (`baseurl`, title, aux links). Never copy it
  between branches, and leave the pinned `mermaid.version` alone.

**Never move or rename these:**

- `docs/vendordep/FrcCatalyst.json`. Every robot project fetches it by URL, and its `jsonUrl` must
  name its own site: `/beta/` on 2.x branches, the stable path on `main`.
- `docs/tools/<tool>/index.html`. CatalystApp copies these by folder name (`npm run sync-tools`).
- `docs/_includes/` and `docs/assets/`.

**The published vendordep names a version that can actually be installed.** On `main` that is the
newest tag JitPack has built. On the alpha-6 line JitPack can build nothing, so it deliberately stays
at `v2.0.0-alpha.1` — the last tag it could build — and `docs/getting-started/installation.md` says
which version it really installs. `docs/tools/versions.test.mjs` enforces exactly that rule.

**Before finishing docs work:** run `node --test "docs/tools/**/*.test.mjs"`, and grep the repo for
any path you moved.

## Writing rules

- **Describe what the code on this branch does.** Never present a feature as installable in a version
  that can't install it.
- **`CHANGELOG.md` follows Keep a Changelog**: `## [x.y.z] — YYYY-MM-DD — title`, then Added /
  Changed / Fixed. Entries explain why, with measured numbers where there are any.
- **`CONTRIBUTING.md` is out of date.** It still describes the 1.x setup (Java 17, WPILib 2026).
- **Public class names and telemetry keys are API.** Robots depend on them; don't rename them as
  cleanup.
- **Encoding.** UTF-8 without a BOM, with `—`, `·` and `←` throughout. Use an editor tool; Windows
  PowerShell 5.1 corrupts them both ways.

## Git

- Don't commit, push, tag or release unless the user asked in this session.
- `origin` carries two push URLs, GitHub and the self-hosted Forgejo, so one `git push` reaches both.
  Forgejo is often unreachable; `devtools mirror` catches it up afterwards.
- A release is not done when the tag is pushed — it is done when JitPack serves the pom. JitPack
  caches failures, so a failed build is fixed by cutting the next version, never by retrying.

## If you are a fallback model

Fine to take on: reorganizing and rewriting docs pages, nav and link fixes, CHANGELOG drafts.

Stop and leave notes for Claude on: anything in `src/` or `agent/`, robot safety, the vendordep, CI
workflows, and releases.
