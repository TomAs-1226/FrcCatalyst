# Contributing to FrcCatalyst

Thanks for your interest in contributing! FrcCatalyst is built for the FRC community, and contributions from all teams are welcome.

## Getting Started

1. **Fork** the repository
2. **Clone** your fork: `git clone https://github.com/YOUR-USERNAME/FrcCatalyst.git`
3. **Build** the project: `./gradlew build`
4. Create a **feature branch**: `git checkout -b feature/my-feature`
5. Make your changes and **commit**: `git commit -m "Add my feature"`
6. **Push** to your fork: `git push origin feature/my-feature`
7. Open a **Pull Request**

## Development Setup

- **Java 17+** (included with WPILib 2026)
- **WPILib 2026** installed (for local Maven dependencies)
- Any Java IDE (VS Code with WPILib extension, IntelliJ, Eclipse)

## Code Style

- Use **builder pattern** for all configuration classes
- All mechanisms should extend `CatalystMechanism`
- Provide **command factories** for all actions (not raw motor calls)
- Include **Javadoc** on all public methods
- Add **telemetry** for all mechanism state via NetworkTables
- Use `AlertManager` for fault reporting

## What to Contribute

- **New mechanism types** (e.g., differential swerve, conveyor belt)
- **Bug fixes** and **performance improvements**
- **Documentation** improvements and examples
- **Simulation** improvements
- **Unit tests**

## Pull Request Guidelines

- Keep PRs focused on a single feature or fix
- Include a clear description of what changed and why
- Make sure `./gradlew build` passes
- Update documentation if adding new features

## Questions?

Open a [GitHub Issue](https://github.com/TomAs-1226/FrcCatalyst/issues) or start a [Discussion](https://github.com/TomAs-1226/FrcCatalyst/discussions).
