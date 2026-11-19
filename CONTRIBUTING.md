# Contributing to FrcCatalyst

Thanks for your interest in contributing! FrcCatalyst is built for the FRC community, and contributions from all teams are welcome.

## Which branch

| Line | Branch | Targets |
|---|---|---|
| 2.x | `main` | Systemcore, WPILib 2027 alpha-7, Commands v3, Java 25 |
| 1.x | `1.x` | roboRIO, WPILib 2026, Java 17 |

`main` is the 2.x line. Fixes for the roboRIO line go to **`1.x`**, not `main`. The 2.x API is not the
1.x API: it uses Commands v3 and the `org.wpilib` package root (no `SubsystemBase`, no
`CommandScheduler.getInstance()`), so check every name against `src/` on the branch you are changing.

## Getting Started

1. **Fork** the repository
2. **Clone** your fork: `git clone https://github.com/YOUR-USERNAME/FrcCatalyst.git`
3. Set `JAVA_HOME` to a **Java 25** JDK (see below) and **build**: `./gradlew build`
4. Create a **feature branch**: `git checkout -b feature/my-feature`
5. Make your changes and **commit**
6. **Push** to your fork and open a **Pull Request**

## Development Setup

- **Java 25** for 2.x: Temurin 25, or the JDK inside the WPILib 2027 alpha-7 installer. Point
  `JAVA_HOME` at it; an older JDK fails to build. (`1.x` needs the WPILib 2026 JDK 17.)
- **WPILib 2027.0.0-alpha-7** and Phoenix 6 `26.70.0-alpha-2`, resolved by Gradle.
- Any Java IDE (VS Code with the WPILib extension, IntelliJ, Eclipse).
- Tests run **one JVM per class**, and some of the native-simulation tests are slow, so a full
  `./gradlew build` takes minutes. To run one class: `./gradlew test --tests '*SomeTest'`.
- Running 2.x robot code needs two `--add-opens` JVM flags; see the README.

## Code Style

- Use the **builder pattern** for configuration classes
- Mechanisms extend `CatalystMechanism`
- Provide **command factories** for actions rather than exposing raw motor calls
- Include **Javadoc** on all public methods
- Publish mechanism state as telemetry through `CatalystLog`
- Use `AlertManager` for fault reporting; alert messages must be constant strings
- Public class names and telemetry keys are API that robots depend on; don't rename them as cleanup
- **Never paste team-specific constants** (CAN IDs, field poses, tuned gains) into the library;
  they belong in the team's robot code

Files are UTF-8 without a BOM.

## What to Contribute

- **New mechanism types**
- **Bug fixes** and **performance improvements**
- **Documentation** improvements and examples
- **Simulation** improvements
- **Unit tests**

## Pull Request Guidelines

- Keep PRs focused on a single feature or fix
- Include a clear description of what changed and why
- Add an entry to `CHANGELOG.md` under `## [Unreleased]`, in [Keep a Changelog](https://keepachangelog.com)
  form (Added / Changed / Fixed). Say why, with measured numbers where you have them
- Make sure `./gradlew build` passes
- If you touch `docs/`, run `node --test "docs/tools/**/*.test.mjs"` from the repository root; it must
  stay green
- Update documentation if adding new features

## Questions?

Open a [GitHub Issue](https://github.com/TomAs-1226/FrcCatalyst/issues) or start a [Discussion](https://github.com/TomAs-1226/FrcCatalyst/discussions).
