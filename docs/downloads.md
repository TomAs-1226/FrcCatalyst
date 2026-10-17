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

## October 17 release set

These exact releases were checked on October 17, 2026. Use their release notes for package
contents and validation limits; version numbers differ because the products ship independently.

| Product | Release | Distribution |
|---|---|---|
| Catalyst App | [2.9.2](https://github.com/TomAs-1226/CatalystApp/releases/tag/app-v2.9.2) | Windows installer and universal macOS disk image; refreshed RC1 tools, docs and MCP graph |
| Catalyst Console | [2.1.1](https://github.com/TomAs-1226/CatalystConsole/releases/tag/v2.1.1) | Signed Windows installer and updater assets |
| Catalyst Pit | [0.1.1](https://github.com/TomAs-1226/CatalystPit/releases/tag/v0.1.1) | Windows installer |
| Catalyst Setup | [1.0.2](https://github.com/TomAs-1226/CatalystSetup/releases/tag/v1.0.2) | Portable setup and offline suite ZIP containing App 2.9.2, Console 2.1.1 and Pit 0.1.1 |
| Detent | [0.1.0](https://github.com/TomAs-1226/Detent/releases/tag/v0.1.0) | Source, npm-format archive, and standalone motion lab; no npm registry publication |
| Bezel web specimen | [0.1.0](https://github.com/TomAs-1226/Bezel/releases/tag/v0.1.0) | Complete static page source, separate from firmware |
| Catalyst Tab / Link | [Tab preview 1](https://github.com/TomAs-1226/Bezel/releases/tag/v0.1.0-tab5-preview.1) | ESP32-P4 firmware, Link 1.0.0 Windows installer and Python wheel; hardware qualification pending |
| Catalyst devtools | [2026.10.17](https://github.com/TomAs-1226/devtools/releases/tag/v2026.10.17) | Portable commands, shared hooks and setup guidance; private repository access required; toolchain binaries installed separately |
| Systemcore agent | [2.0.3 package](https://github.com/TomAs-1226/FrcCatalyst/releases/tag/v2.0.0-rc.1) | IPK plus checksum under the existing library RC1 release |

The offline Setup bundle intentionally omits Sim and Link. Download Link separately from its
preview release. Installer workflows, downloaded hashes and the suite ZIP's embedded versions
were verified; no installers were run and no robot or camera firmware was flashed.

## Releases requiring repository access

- [Catalyst Sim 1.7.1 practice preview](https://github.com/TomAs-1226/CatalystSim/releases/tag/v1.7.1): Windows installer/player, source and validation; the repository is private. The practice plant remains uncalibrated and is separate from Java/HAL robot execution.
- [Paired ESP32-C6 preview 1](https://github.com/TomAs-1226/Tab5C6Slave/releases/tag/c6-v0.1.0-preview.1): Wi-Fi-only companion firmware. The repository is private; an authorized GitHub login is required. Existing experimental BLE work was not included.

These restricted downloads may show a 404 page without access. Their repositories have not
been made public. Firmware previews are not field qualification or a request to flash hardware.

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
