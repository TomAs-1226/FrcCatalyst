#!/usr/bin/env python3
"""Build catalyst-agent_<version>.ipk.

## Why this is Python and not the shell script it replaces

An .ipk is an `ar` archive of three members, and the members are tarballs whose Unix file modes and
ownership are what actually matter — `ExecStart` points straight at `catalyst_agent.py`, so if that
file lands without its executable bit the service fails to start with `203/EXEC`, which names the
file but not the reason.

Shelling out to `tar` and `ar` gets those modes from the host filesystem. That is fine on Linux and
wrong everywhere else: Windows has no executable bit to read, and `ar` is not present in Git Bash at
all, so the shell build could not even finish. Setting every mode explicitly here means the package
is identical no matter which machine built it, which is the property a release process needs.

## What it produces

    !<arch>
    debian-binary      "2.0\\n"
    control.tar.gz     ./control ./postinst ./prerm ./postrm
    data.tar.gz        ./etc/... ./usr/...

`debian-binary` is not optional in the format even though opkg tolerates its absence. Producing a
package that happens to work on one installer and not another is the kind of thing that is found on
a competition field, so it is written.

Output is byte-for-byte reproducible: fixed timestamps, fixed ownership, sorted entries. That makes
"is this the package I tested?" a question with an answer.

    python3 build.py            # -> catalyst-agent_2.0.0.ipk
    python3 build.py --check    # verify an existing package instead of building
"""

import gzip
import io
import os
import shutil
import sys
import tarfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
OVERLAY = HERE / "overlay"
CONTROL = HERE / "control"

# Fixed so two builds of the same source produce the same bytes. The value is arbitrary but it has
# to be in the past: GNU tar warns on every entry of an archive stamped in the future, and a build
# whose normal output is a page of warnings trains people to skip reading them. 2026-01-01 UTC.
MTIME = 1767225600

# Files whose content is read by a shell or an interpreter after unpacking, and which therefore must
# arrive with LF endings whatever the build host did to them.
#
# The .gitattributes in the repo root is the real fix - it stops git rewriting these on checkout in
# the first place. This is the second line of defence, because the failure it prevents is remote and
# unhelpful: `catalyst_agent.py` is named directly by the unit's ExecStart, so one carriage return
# on the shebang line makes systemd report `bad interpreter` for a file that is plainly right there.
TEXT_SUFFIXES = {".py", ".sh", ".service", ".conf", ".json", ".md"}

# Anything matching these never ships. __pycache__ is the one that actually turned up: the build
# copied overlay/ wholesale, so a .pyc compiled by whatever Python the developer happened to run
# went into the package, stale and host-specific.
EXCLUDE_NAMES = {"__pycache__", ".DS_Store", "Thumbs.db"}
EXCLUDE_SUFFIXES = {".pyc", ".pyo", ".swp", ".orig", ".rej"}

# The executable bit is the whole reason this file sets modes by hand. Paths are relative to
# overlay/; everything not listed is 0644, every directory is 0755.
EXECUTABLE = {"usr/local/bin/catalyst-agent/catalyst_agent.py"}

# Maintainer scripts are run by the installer, so all three need it.
CONTROL_EXECUTABLE = {"postinst", "prerm", "postrm"}


def excluded(path: Path) -> bool:
    return path.name in EXCLUDE_NAMES or path.suffix in EXCLUDE_SUFFIXES


def collect(root: Path):
    """Every file under root, relative and sorted, with excluded paths pruned.

    Sorted because archive order is part of the output bytes, and `os.walk` order is filesystem
    order — which differs between machines and would defeat the reproducibility above.
    """
    out = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames if d not in EXCLUDE_NAMES)
        here = Path(dirpath)
        for name in sorted(filenames):
            f = here / name
            if not excluded(f):
                out.append(f.relative_to(root))
    return out


def read_control():
    """The control file's fields, in the order they were written."""
    fields = []
    for line in (CONTROL / "control").read_text(encoding="utf-8").splitlines():
        if ":" in line and not line.startswith((" ", "\t")):
            key, _, value = line.partition(":")
            fields.append((key.strip(), value.strip()))
    return fields


def _entry(name: str, mode: int, size: int, kind=tarfile.REGTYPE) -> tarfile.TarInfo:
    """A tar entry owned by root, with a fixed time and an explicit mode.

    Ownership matters: files built as an unprivileged user would otherwise carry that uid into the
    package, and opkg preserves it. A telemetry agent owned by uid 1000 on a machine that has no
    such user is a confusing way to fail.
    """
    ti = tarfile.TarInfo(name)
    ti.mode = mode
    ti.size = size
    ti.mtime = MTIME
    ti.type = kind
    ti.uid = ti.gid = 0
    ti.uname = ti.gname = "root"
    return ti


def _targz(add) -> bytes:
    """Run `add(tar)` and return the gzipped tarball.

    gzip is written separately with mtime=0 rather than through `tarfile.open(mode="w:gz")`, because
    that path stamps the current time into the gzip header and nothing else here would vary.
    """
    raw = io.BytesIO()
    with tarfile.open(fileobj=raw, mode="w", format=tarfile.GNU_FORMAT) as tar:
        add(tar)
    out = io.BytesIO()
    with gzip.GzipFile(fileobj=out, mode="wb", mtime=0) as gz:
        gz.write(raw.getvalue())
    return out.getvalue()


def build_data() -> tuple:
    """data.tar.gz, plus the installed size in bytes that the control file will declare."""
    files = collect(OVERLAY)
    if not files:
        sys.exit("overlay/ has no files - nothing to package")

    # Parent directories, so the archive carries them with sane modes rather than leaving opkg to
    # invent them.
    dirs = set()
    for rel in files:
        for parent in rel.parents:
            if parent != Path("."):
                dirs.add(parent.as_posix())

    installed = 0

    def add(tar):
        nonlocal installed
        for d in sorted(dirs):
            tar.addfile(_entry("./" + d, 0o755, 0, tarfile.DIRTYPE))
        for rel in files:
            data = (OVERLAY / rel).read_bytes()
            if rel.suffix in TEXT_SUFFIXES:
                data = data.replace(b"\r\n", b"\n")
            posix = rel.as_posix()
            mode = 0o755 if posix in EXECUTABLE else 0o644
            installed += len(data)
            tar.addfile(_entry("./" + posix, mode, len(data)), io.BytesIO(data))

    return _targz(add), installed, files


def build_control(installed: int) -> tuple:
    """control.tar.gz.

    Installed-Size is added here rather than kept in the source control file, because it is derived
    from the payload and a hand-maintained copy would drift. Package managers show it before
    installing; without it the Systemcore web UI reports an unknown size.
    """
    fields = read_control()
    keys = {k.lower() for k, _ in fields}
    if "package" not in keys or "version" not in keys:
        sys.exit("control/control needs both Package and Version")
    if "installed-size" not in keys:
        # After Description, which is where dpkg-style tooling expects the block to end.
        at = next((i for i, (k, _) in enumerate(fields) if k.lower() == "description"), len(fields) - 1)
        fields.insert(at + 1, ("Installed-Size", str(installed)))

    text = "".join(f"{k}: {v}\n" for k, v in fields)
    scripts = sorted(p for p in CONTROL.iterdir() if p.is_file() and p.name != "control")

    def add(tar):
        body = text.encode("utf-8")
        tar.addfile(_entry("./control", 0o644, len(body)), io.BytesIO(body))
        for p in scripts:
            # Written with LF regardless of how the file sits in the working tree. A maintainer
            # script with CRLF endings fails on the board with `/bin/sh^M: bad interpreter`, which
            # is a Windows checkout leaking into a package and is invisible until install time.
            data = p.read_bytes().replace(b"\r\n", b"\n")
            mode = 0o755 if p.name in CONTROL_EXECUTABLE else 0o644
            tar.addfile(_entry("./" + p.name, mode, len(data)), io.BytesIO(data))

    return _targz(add), dict(fields)


def ar_archive(members) -> bytes:
    """An `ar` archive. The format is small enough to write directly, and `ar` is not always here."""
    out = bytearray(b"!<arch>\n")
    for name, data in members:
        if len(name) > 16:
            sys.exit(f"ar member name too long: {name}")
        header = (
            name.ljust(16).encode()
            + str(MTIME).ljust(12).encode()
            + b"0".ljust(6)          # uid
            + b"0".ljust(6)          # gid
            + b"100644".ljust(8)     # mode
            + str(len(data)).ljust(10).encode()
            + b"`\n"
        )
        assert len(header) == 60, len(header)
        out += header + data
        if len(data) % 2:
            out += b"\n"             # members start on an even offset
    return bytes(out)


def build() -> Path:
    if not OVERLAY.is_dir():
        sys.exit("overlay/ not found - run this from the agent directory")
    if not (CONTROL / "control").is_file():
        sys.exit("control/control not found")

    data, installed, files = build_data()
    control, fields = build_control(installed)

    ipk = HERE / f"{fields['Package']}_{fields['Version']}.ipk"
    ipk.write_bytes(
        ar_archive([
            ("debian-binary", b"2.0\n"),
            ("control.tar.gz", control),
            ("data.tar.gz", data),
        ])
    )

    print(f"{ipk.name}  ({ipk.stat().st_size:,} bytes, {installed:,} installed)")
    for rel in files:
        posix = rel.as_posix()
        print(f"  {'0755' if posix in EXECUTABLE else '0644'}  /{posix}")
    return ipk


def check(path: Path) -> int:
    """Read a built package back and assert the things that fail silently.

    Worth having as a command rather than a comment: every one of these produces a package that
    installs without complaint and then does not work.
    """
    blob = path.read_bytes()
    problems = []

    if not blob.startswith(b"!<arch>\n"):
        return _report(path, ["not an ar archive"])

    members, i = {}, 8
    order = []
    while i + 60 <= len(blob):
        name = blob[i:i + 16].decode().strip().rstrip("/")
        size = int(blob[i + 48:i + 58].decode().strip())
        members[name] = blob[i + 60:i + 60 + size]
        order.append(name)
        i += 60 + size + (size % 2)

    if order[:3] != ["debian-binary", "control.tar.gz", "data.tar.gz"]:
        problems.append(f"members are {order}, expected debian-binary, control.tar.gz, data.tar.gz")
    if members.get("debian-binary") != b"2.0\n":
        problems.append("debian-binary is not '2.0'")

    contents = {}

    def entries(member):
        found = {}
        with tarfile.open(fileobj=io.BytesIO(gzip.decompress(members[member]))) as tar:
            for m in tar.getmembers():
                name = m.name.lstrip("./")
                found[name] = m
                if m.isfile():
                    contents[(member, name)] = tar.extractfile(m).read()
        return found

    data = entries("data.tar.gz")
    control = entries("control.tar.gz")

    # A carriage return on a shebang line is the failure this package is most exposed to, because it
    # survives every check that reads the file as text and only shows up on the board as
    # `bad interpreter` naming a file that is plainly present.
    for (_member, name), body in sorted(contents.items()):
        if body.startswith(b"#!") and body.split(b"\n", 1)[0].endswith(b"\r"):
            problems.append(f"{name} has a CRLF shebang; it will fail with 'bad interpreter'")

    agent = "usr/local/bin/catalyst-agent/catalyst_agent.py"
    if agent not in data:
        problems.append(f"{agent} is not in the package - the service has nothing to run")
    elif not data[agent].mode & 0o111:
        problems.append(f"{agent} is not executable ({data[agent].mode:04o}); systemd will fail 203/EXEC")

    unit = "etc/systemd/system/catalyst-agent.service"
    if unit not in data:
        problems.append(f"{unit} is missing - nothing would start it")

    for script in CONTROL_EXECUTABLE:
        if script not in control:
            problems.append(f"CONTROL/{script} is missing")
        elif not control[script].mode & 0o111:
            problems.append(f"CONTROL/{script} is not executable ({control[script].mode:04o})")

    if "control" not in control:
        problems.append("CONTROL/control is missing")

    for name in list(data) + list(control):
        if "__pycache__" in name or name.endswith(".pyc"):
            problems.append(f"build residue shipped: {name}")

    for name, m in {**data, **control}.items():
        if m.uid or m.gid:
            problems.append(f"{name} is owned by {m.uid}:{m.gid}, not root")

    return _report(path, problems)


def _report(path: Path, problems) -> int:
    if problems:
        print(f"{path.name}: {len(problems)} problem(s)")
        for p in problems:
            print(f"  - {p}")
        return 1
    print(f"{path.name}: ok")
    return 0


if __name__ == "__main__":
    if "--check" in sys.argv:
        rest = [a for a in sys.argv[1:] if a != "--check"]
        target = Path(rest[0]) if rest else next(HERE.glob("*.ipk"))
        sys.exit(check(target))
    sys.exit(check(build()))
