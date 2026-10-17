---
layout: default
title: Downloads
nav_order: 1.6
---

# Catalyst downloads

Use the GitHub release page for the installer, release notes and platform support. The desktop
apps work on a local robot network; internet is needed to download updates, not to view local
telemetry or use saved tools.

| Product | Download | Purpose |
|---|---|---|
| Catalyst App | [Latest release](https://github.com/TomAs-1226/CatalystApp/releases/latest) | Library setup, offline tools, project discovery and agent/MCP resources |
| Catalyst Console | [Latest release](https://github.com/TomAs-1226/CatalystConsole/releases/latest) | Robot telemetry, vision/field view, declared tunables and autonomous selection |
| Catalyst Pit | [Latest release](https://github.com/TomAs-1226/CatalystPit/releases/latest) | Offline pit checklists, batteries, queue and issue tracking |
| Catalyst Setup | [Latest release](https://github.com/TomAs-1226/CatalystSetup/releases/latest) | Suite installer and updater; use each package's declared platform support |
| FrcCatalyst 1.12.0 | [Stable release](https://github.com/TomAs-1226/FrcCatalyst/releases/tag/v1.12.0) | roboRIO / WPILib 2026 library |
| FrcCatalyst 2.0.0-rc.1 | [RC1 assets and notes](https://github.com/TomAs-1226/FrcCatalyst/releases/tag/v2.0.0-rc.1) | Systemcore / WPILib 2027 alpha-7 prerelease |

Choose the correct library line on [Versions and compatibility](versions.html). A prerelease
library and a stable desktop app can have different version numbers without being out of sync.

## Other suite projects

- [Catalyst Sim releases](https://github.com/TomAs-1226/CatalystSim/releases): Windows driver-practice simulator; repository access is required. It is a separate C# policy/physics implementation, not Java/HAL execution or proof of robot calibration.
- [Detent releases](https://github.com/TomAs-1226/Detent/releases): motion library, sourced design knowledge, CLI and MCP tools.
- [Bezel releases](https://github.com/TomAs-1226/Bezel/releases): static design specimen and separately labelled Catalyst Tab previews.

Firmware previews identify their tested target and hardware limits in their own release notes.
The Systemcore agent has its own package version; it does not share the library's version number.

## Before using an update

Read the release notes for supported controllers, WPILib/vendor versions and required firmware.
Autonomous and shot tuning remain robot-specific. Advisory preflight findings never replace
Driver Station enable/disable controls or block loading a match program.
