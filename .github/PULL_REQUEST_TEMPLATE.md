## What and why

<!-- What this changes on a robot, and the problem it fixes. -->

## Checks

- [ ] `./gradlew build` passes (Java 25; see CONTRIBUTING.md)
- [ ] `node --test "docs/tools/**/*.test.mjs"` passes
- [ ] A test covers the change, or the PR says why one cannot
- [ ] A CHANGELOG entry under `## [Unreleased]`
- [ ] No team-specific constants (CAN ids, field positions, tuned gains) in library code
- [ ] Targets `main` for 2.x, or `1.x` for the roboRIO line
