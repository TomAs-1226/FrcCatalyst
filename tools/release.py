#!/usr/bin/env python3
"""Release helpers, run by CI and usable by hand.

    python tools/release.py notes v1.12.0           print that version's CHANGELOG section
    python tools/release.py jitpack-wait v1.12.0    wait until JitPack has really built the tag

A tag is not a release. Two things have bitten this repository:

  * A tag with no GitHub Release. Issues #28 and #25 were filed because v1.2.0 had a tag and bumped
    docs but no Release object, so the releases page lagged.
  * A tag JitPack never built. JitPack builds on demand and **caches failures**, so retrying does
    nothing and neither does asking for a different artifact path. The escape is to cut the next
    version: a new tag resolves to a new commit, which JitPack builds fresh. v1.5.0 was stuck that
    way and v1.6.0 built first try minutes later.

Some lines cannot be on JitPack at all: their WPILib release is on no public maven. There the wait is
skipped with a notice rather than failing, because there is nothing to wait for.

Read-only apart from what it prints. Standard library only, Python 3.8+.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
COORDINATE = "com.github.TomAs-1226/FrcCatalyst"
BUILDS_API = "https://jitpack.io/api/builds/%s/%s"
POM_URL = "https://jitpack.io/com/github/TomAs-1226/FrcCatalyst/%s/FrcCatalyst-%s.pom"

# WPILib releases on no public maven: JitPack cannot resolve them, so this library cannot be built
# there by anyone but a machine with the WPILib installer.
SOURCE_ONLY_WPILIB = {"2027.0.0-alpha-6"}

# "## [1.12.0] — 2026-08-06 — The devices, not the count"; any dash spelling, date optional here so
# a missing date is reported as itself rather than as a missing section.
HEADING = re.compile(r"^##\s*\[([^\]]+)\]\s*(?:[—–-]\s*(\d{4}-\d{2}-\d{2}))?\s*(?:[—–-]\s*(.*))?$", re.M)


def version_of(tag: str) -> str:
    return tag[1:] if tag.startswith("v") else tag


def wpilib_version() -> str:
    text = (REPO / "build.gradle").read_text(encoding="utf-8")
    m = re.search(r"wpilibVersion\s*=\s*['\"]([^'\"]+)['\"]", text)
    return m.group(1) if m else ""


def section(version: str):
    """(title, body, dated) for one release, or None when the file has no section for it."""
    text = (REPO / "CHANGELOG.md").read_text(encoding="utf-8")
    matches = list(HEADING.finditer(text))
    for i, m in enumerate(matches):
        if m.group(1).strip() != version:
            continue
        end = matches[i + 1].start() if i + 1 < len(matches) else len(text)
        return (m.group(3) or "").strip(), text[m.end():end].strip(), bool(m.group(2))
    return None


def cmd_notes(args) -> int:
    version = version_of(args.tag)
    found = section(version)
    if not found:
        print("CHANGELOG.md has no section for %s. Add `## [%s] — <date> — <title>` before releasing."
              % (version, version), file=sys.stderr)
        return 1
    title, body, dated = found
    if not dated:
        print("the %s section in CHANGELOG.md has no date." % version, file=sys.stderr)
        return 1
    if not body:
        print("the %s section in CHANGELOG.md is empty." % version, file=sys.stderr)
        return 1
    if args.title:
        print(title or version)
        return 0
    print(body)
    return 0


def jitpack_status(tag: str, timeout: float = 15.0) -> dict:
    try:
        with urllib.request.urlopen(BUILDS_API % (COORDINATE, tag), timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except (urllib.error.URLError, ValueError, OSError) as exc:
        return {"status": "unreachable", "message": str(exc)}


def pom_code(tag: str, timeout: float = 60.0) -> int:
    """Fetching the pom is what asks JitPack to build; the status API never does."""
    req = urllib.request.Request(POM_URL % (tag, tag))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status
    except urllib.error.HTTPError as exc:
        return exc.code
    except (urllib.error.URLError, OSError):
        return 0


def cmd_jitpack_wait(args) -> int:
    tag = args.tag
    wpilib = wpilib_version()
    if wpilib in SOURCE_ONLY_WPILIB:
        print("This line pins WPILib %s, which is on no public maven, so JitPack cannot build it. "
              "Nothing to wait for: %s is a source-build-only release." % (wpilib, tag))
        return 0

    deadline = time.time() + args.timeout
    triggered = False
    polls = 0
    while time.time() < deadline:
        status = jitpack_status(tag)
        state = str(status.get("status", ""))
        polls += 1

        if state == "ok":
            code = pom_code(tag)
            if code == 200:
                print("JitPack has built %s: the pom is served, so the vendordep can install it." % tag)
                return 0
            print("JitPack reports ok for %s but the pom returned %s; waiting." % (tag, code))
        elif state == "Error":
            print("JitPack could not build %s: %s" % (tag, status.get("message", "")), file=sys.stderr)
            print("Log: https://jitpack.io/com/github/TomAs-1226/FrcCatalyst/%s/build.log" % tag,
                  file=sys.stderr)
            print("JitPack caches failures. Retrying will not clear it and neither will asking for a "
                  "different artifact. Cut the next version: a new tag is a new commit, which builds "
                  "fresh.", file=sys.stderr)
            return 1
        elif state in ("unreachable",):
            print("JitPack did not answer (%s); retrying." % status.get("message", ""))
        elif not triggered and polls >= 2:
            # "none" means JitPack has never been asked for this tag. Asking for the pom is the ask.
            print("JitPack has no record of %s; requesting the pom, which starts the build." % tag)
            pom_code(tag)
            triggered = True
        else:
            print("JitPack status for %s: %s" % (tag, state or "building"))

        time.sleep(args.interval)

    print("Gave up after %d seconds waiting for JitPack to build %s. Check "
          "https://jitpack.io/#TomAs-1226/FrcCatalyst" % (args.timeout, tag), file=sys.stderr)
    return 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="release.py", description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("notes", help="print a version's CHANGELOG section")
    p.add_argument("tag")
    p.add_argument("--title", action="store_true", help="print the section's title instead of its body")
    p.set_defaults(func=cmd_notes)

    p = sub.add_parser("jitpack-wait", help="wait until JitPack has built the tag")
    p.add_argument("tag")
    p.add_argument("--timeout", type=float, default=1800.0, help="seconds to wait (default 1800)")
    p.add_argument("--interval", type=float, default=30.0, help="seconds between polls")
    p.set_defaults(func=cmd_jitpack_wait)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
