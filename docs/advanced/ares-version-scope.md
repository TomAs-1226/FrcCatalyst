---
title: ARES 1.1 and 2.0 scope
parent: Advanced
---

# ARES 1.1 and 2.0 scope

ARES 1.1 is an incremental upgrade of the current multicamera implementation on the existing
FRC/WPILib/Phoenix stack. The October 15 library audit was based on library `6b4180b`;
its reviewed fixes are carried into the isolated `upgrade/rc1-prep-20261016` candidate. They repair
demonstrated existing defects; they do not
activate new cameras, change measured robot constants, or add a new localization architecture.

ARES 2.0 is the separate ROS 2-style localization, perception-mailbox and action-ownership
prototype retained on `codex/ares-localization`. Its staged changes and exported review patches
are preserved. They are not merged into ARES 1.1. The prototype remains opt-in, unqualified for
hardware control, and distinct from the deployable Numbers auto package.

These ARES feature names do not change the FrcCatalyst Maven version or create a release tag.
The saved auto package remains Numbers `1eba289` with library `6b4180b` until a separately reviewed
integration is authorized. The RC1 preparation branch is source for review and CI, not a release
tag or an installed robot package. Deployment, actuation, firmware and machine/network/security
changes remain outside this preparation.
