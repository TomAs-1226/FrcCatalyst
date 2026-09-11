#!/usr/bin/env python3
"""Print the Catalyst version map: what exists, what it runs on, and where it lives.

Catalyst's versions are spread over places that drift apart: git tags, branch heads, builds
published to Maven local under numbers picked by hand, JitPack, the vendordeps the docs site
serves, and the upstream Systemcore compatibility matrix. This reads them and prints one map, so
nobody has to rebuild it by hand again.

    python tools/catalyst-versions.py              git tags + branch heads + Maven local
    python tools/catalyst-versions.py --matrix     + the SystemcoreTesting compatibility matrix
    python tools/catalyst-versions.py --online     + the matrix, JitPack status, published vendordeps
    python tools/catalyst-versions.py --apps       + CatalystApp / CatalystConsole checked out beside it
    python tools/catalyst-versions.py --fetch      `git fetch origin` first
    python tools/catalyst-versions.py --all-tags   every tag, not only the current lines

Read-only. Nothing is written to git, to ~/.m2 or anywhere else, and the JitPack query is the
build-list API, which reports status and never triggers a build. Standard library only, Python 3.8+.
The same file is kept on main, systemcore and upgrade/alpha-7; it reads every ref, so any copy
prints the same map. The page it backs: https://tomas-1226.github.io/FrcCatalyst/versions.html

How a Maven local build is traced to a commit: every .java file in its -sources.jar is hashed the
way git hashes a blob (line endings normalised) and compared with src/main/java at every commit
reachable from any ref. A build whose sources match no commit exactly was made from uncommitted
changes, and the closest commit is shown with the number of files that differ.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import urllib.request
import xml.etree.ElementTree as ET
import zipfile
from datetime import datetime
from pathlib import Path

JITPACK_BUILDS = "https://jitpack.io/api/builds/com.github.TomAs-1226/FrcCatalyst"
PUBLISHED_VENDORDEPS = (
    ("stable", "https://tomas-1226.github.io/FrcCatalyst/vendordep/FrcCatalyst.json"),
    ("beta", "https://tomas-1226.github.io/FrcCatalyst/beta/vendordep/FrcCatalyst.json"),
)
MATRIX_README = "https://raw.githubusercontent.com/wpilibsuite/SystemcoreTesting/main/README.md"
MATRIX_HISTORY = ("https://api.github.com/repos/wpilibsuite/SystemcoreTesting/commits"
                  "?path=README.md&per_page=1")
MATRIX_PAGE = "https://github.com/wpilibsuite/SystemcoreTesting#software-compatibility"
MAVEN_PATH = Path("com") / "frccatalyst" / "FrcCatalyst"
CODE_ROOT = "src/main/java"

PIN_RE = re.compile(r"^\s*(version|wpilibVersion|phoenixVersion|pathplannerVersion|"
                    r"limelightVersion|photonVersion)\s*=\s*['\"]([^'\"]+)['\"]", re.M)
SEMVER_RE = re.compile(r"^v?(\d+)\.(\d+)\.(\d+)(?:\.(\d+))?(?:-([0-9A-Za-z.\-]+))?$")
ITERATION_RE = re.compile(r"-a\d+$")
RELEASED_2027_RE = re.compile(r"^2027\.0\.0-alpha-(\d+)$")
# Catalyst's own pins, and the name the upstream matrix gives the same library.
PIN_TO_MATRIX = (("phoenixVersion", "CTRE Phoenix 6"), ("pathplannerVersion", "PathPlannerLib"),
                 ("limelightVersion", "LimelightLib2"))
ASCII = str.maketrans({"—": "-", "–": "-", "‘": "'", "’": "'", "“": '"',
                       "”": '"', "→": "->", "≤": "<=", "≥": ">=",
                       "×": "x", "…": "...", " ": " ", "•": "*"})


def out(text: str = ""):
    """Everything goes out as ASCII: a Windows console or pipe will not mangle it."""
    print(text.translate(ASCII).encode("ascii", "replace").decode("ascii"))


def plural(n: int, word: str) -> str:
    return "%d %s%s" % (n, word, "" if n == 1 else "s")


# --------------------------------------------------------------------------------------------
# git


class Git:
    """Thin wrapper: one-shot commands plus a single long-lived `cat-file --batch` for objects."""

    def __init__(self, repo: Path):
        self.repo = str(repo)
        self._batch = None
        self._trees = {}
        self._flat = {}
        self._env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")

    def run(self, *args: str, check: bool = True) -> str:
        r = subprocess.run(["git", "-C", self.repo, *args], stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, env=self._env)
        if check and r.returncode != 0:
            raise RuntimeError("git %s: %s" % (" ".join(args),
                                               r.stderr.decode("utf-8", "replace").strip()))
        return r.stdout.decode("utf-8", "replace")

    def obj(self, name: str):
        """Any object name git accepts (a sha, `rev:path`). Returns (type, bytes) or (None, None)."""
        if self._batch is None:
            self._batch = subprocess.Popen(["git", "-C", self.repo, "cat-file", "--batch"],
                                           stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                           env=self._env)
        self._batch.stdin.write(name.encode("utf-8") + b"\n")
        self._batch.stdin.flush()
        header = self._batch.stdout.readline().decode("utf-8", "replace").split()
        if len(header) != 3:  # "<name> missing" or "<name> ambiguous"
            return None, None
        data = self._batch.stdout.read(int(header[2]))
        self._batch.stdout.read(1)
        return header[1], data

    def text(self, rev: str, path: str):
        kind, data = self.obj("%s:%s" % (rev, path))
        return data.decode("utf-8", "replace") if kind == "blob" else None

    def tree(self, sha: str):
        if sha not in self._trees:
            _, data = self.obj(sha)
            entries, i = [], 0
            while data and i < len(data):
                space = data.index(b" ", i)
                nul = data.index(b"\0", space)
                entries.append((data[i:space].decode(), data[space + 1:nul].decode("utf-8", "replace"),
                                data[nul + 1:nul + 21].hex()))
                i = nul + 21
            self._trees[sha] = entries
        return self._trees[sha]

    def subtree(self, commit: str, path: str):
        kind, data = self.obj(commit)
        if kind != "commit":
            return None
        sha = data.split(b"\n", 1)[0].split()[1].decode()
        for part in path.split("/"):
            for mode, name, child in self.tree(sha):
                if name == part and mode == "40000":
                    sha = child
                    break
            else:
                return None
        return sha

    def flatten(self, tree_sha: str):
        """{relative path: blob sha} for a tree, memoised - most subtrees are shared by many commits."""
        if tree_sha not in self._flat:
            flat = {}
            for mode, name, child in self.tree(tree_sha):
                if mode == "40000":
                    for path, blob in self.flatten(child).items():
                        flat[name + "/" + path] = blob
                elif mode in ("100644", "100755"):
                    flat[name] = child
            self._flat[tree_sha] = flat
        return self._flat[tree_sha]


def blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def pins_at(git: Git, rev: str) -> dict:
    """The version and dependency pins build.gradle declares at `rev`, plus the agent package."""
    pins = {}
    for key, value in PIN_RE.findall(git.text(rev, "build.gradle") or ""):
        pins.setdefault(key, value)
    m = re.search(r"^Version:\s*(\S+)", git.text(rev, "agent/control/control") or "", re.M)
    if m:
        pins["agent"] = m.group(1)
    return pins


def classify(git: Git, base: str, head: str) -> str:
    """What the commits between base and head changed: code, tests, docs, or nothing."""
    names = [n for n in git.run("diff", "--name-only", base, head).splitlines() if n]
    code = any(n.startswith("src/main/") for n in names)
    tests = any(n.startswith("src/test/") for n in names)
    build = False
    for line in git.run("diff", "-U0", base, head, "--", "build.gradle", "settings.gradle",
                        "gradle.properties").splitlines():
        if not line or line.startswith(("+++", "---")) or line[0] not in "+-":
            continue
        body = line[1:].strip()
        if body and not body.startswith(("//", "*", "/*")):
            build = True
            break
    if code or build:
        return "code"
    if tests:
        return "tests"
    return "docs" if names else "same"


def vkey(name: str):
    """Sort key for v1.2.3[.4][-pre]; a release sorts after its pre-releases."""
    m = SEMVER_RE.match(name)
    if not m:
        return None
    major, minor, patch, fourth, pre = m.groups()
    nums = (int(major), int(minor), int(patch), int(fourth or 0))
    if pre is None:
        return nums + ((1,),)
    ids = tuple((0, int(p), "") if p.isdigit() else (1, 0, p) for p in pre.split("."))
    return nums + ((0, ids),)


def wpilib_key(w: str):
    """Order WPILib versions: 2026.x, then the 2027 alphas, snapshots just after their alpha."""
    m = re.match(r"^(\d+)\.(\d+)\.(\d+)(?:-alpha-(\d+))?(.*)$", w or "")
    if not m:
        return (9999,)
    return (int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4) or 0), m.group(5))


# --------------------------------------------------------------------------------------------
# network (only with --matrix / --online)


def fetch(url: str, timeout: float = 15.0):
    req = urllib.request.Request(url, headers={"User-Agent": "catalyst-versions"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.read().decode("utf-8", "replace"), None
    except Exception as e:  # offline, rate-limited, DNS - report and carry on
        return None, "%s: %s" % (type(e).__name__, e)


class Matrix:
    """The SystemcoreTesting README's two compatibility tables, parsed well enough to look up."""

    def __init__(self, body: str):
        lines = body.splitlines()
        start = next((i for i, l in enumerate(lines) if re.match(r"^#+\s+Software Compatibility", l)), None)
        self.section, self.tables = [], []
        if start is None:
            return
        current = None
        for line in lines[start + 1:]:
            if re.match(r"^##\s", line):
                break
            self.section.append(line)
            if line.startswith("|"):
                cells = [c.strip() for c in line.strip().strip("|").split("|")]
                if all(re.fullmatch(r":?-*:?", c) for c in cells):
                    continue
                if current is None:
                    current = []
                    self.tables.append(current)
                current.append(cells)
            else:
                current = None

    @staticmethod
    def alphas(cell: str):
        nums = set()
        for m in re.finditer(r"alpha-(\d+(?:/\d+)*)", cell):
            nums.update(int(x) for x in m.group(1).split("/"))
        return nums, (">=" in cell or "≥" in cell)

    def _covers(self, cell: str, alpha: int) -> bool:
        nums, at_least = self.alphas(cell)
        return bool(nums) and (alpha >= min(nums) if at_least else alpha in nums)

    def image_for(self, wpilib: str) -> str:
        if (wpilib or "").startswith("2026."):
            return "roboRIO"
        m = RELEASED_2027_RE.match(wpilib or "")
        if not m:
            return "snapshot, no image"
        if self.tables:
            for row in self.tables[0][1:]:
                if len(row) > 1 and self._covers(row[1], int(m.group(1))):
                    return "image " + row[0].replace(" ", "")
        return "?"

    def missing_for(self, wpilib: str):
        """Libraries the matrix says have no release for this WPILib, or None if unknown."""
        m = RELEASED_2027_RE.match(wpilib or "")
        if not m or len(self.tables) < 2:
            return None
        header = self.tables[1][0]
        col = next((i for i, h in enumerate(header) if i and self._covers(h, int(m.group(1)))), None)
        if col is None:
            return None
        return [row[0] for row in self.tables[1][1:]
                if len(row) > col and (":x:" in row[col] or not row[col])]


# --------------------------------------------------------------------------------------------
# output


def table(rows, headers, indent="  "):
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))
    fmt = lambda row: indent + "  ".join(c.ljust(widths[i]) for i, c in enumerate(row)).rstrip()
    out(fmt(headers))
    for row in rows:
        out(fmt(row))


def section(title: str):
    out()
    out(title)
    out("-" * len(title))


# --------------------------------------------------------------------------------------------
# the map


class VersionMap:
    def __init__(self, git: Git, m2: Path, args):
        self.git, self.m2, self.args = git, m2, args
        self.notes = []
        self.jitpack = {}
        self.matrix = None
        self.commit_time = {}
        for line in git.run("log", "--all", "--format=%H%x09%ct%x09%cs").splitlines():
            sha, ct, cs = line.split("\t")
            self.commit_time[sha] = (int(ct), cs)
        self.tags = []
        fmt = "%(refname:short)%09%(objectname)%09%(*objectname)"
        for line in git.run("for-each-ref", "--format=" + fmt, "refs/tags").splitlines():
            name, obj, peeled = line.split("\t")
            self.tags.append((name, peeled or obj))
        self.tags.sort(key=lambda t: (vkey(t[0]) is None, vkey(t[0]) or (), t[0]))
        self.tag_by_commit = {}
        for name, sha in self.tags:
            self.tag_by_commit.setdefault(sha, []).append(name)
        self.tag_pins = {name: pins_at(git, sha) for name, sha in self.tags}
        self._index = None
        self.heads = []   # dicts, see collect_branches
        self.builds = []  # dicts, see collect_maven

    def short(self, sha):
        return sha[:7]

    def date(self, sha):
        return self.commit_time.get(sha, (0, "?"))[1]

    def newest_tag(self, wpilib):
        named = [n for n, _ in self.tags if vkey(n) and self.tag_pins[n].get("wpilibVersion") == wpilib]
        return max(named, key=vkey) if named else None

    # ---- online ----------------------------------------------------------------------------

    def load_jitpack(self):
        body, err = fetch(JITPACK_BUILDS)
        if err:
            self.notes.append("JitPack status unavailable (%s)" % err)
            return
        try:
            self.jitpack = next(iter(next(iter(json.loads(body).values())).values()))
        except (ValueError, StopIteration, AttributeError):
            self.notes.append("JitPack returned something unexpected")

    def load_matrix(self):
        body, err = fetch(MATRIX_README)
        if err:
            self.notes.append("SystemcoreTesting matrix unavailable (%s)" % err)
            return
        self.matrix = Matrix(body)
        history, _ = fetch(MATRIX_HISTORY)
        try:
            last = json.loads(history)[0]
            self.matrix.changed = "%s (%s)" % (last["commit"]["committer"]["date"][:10], last["sha"][:7])
        except (TypeError, ValueError, LookupError):
            self.matrix.changed = "?"

    # ---- collect ------------------------------------------------------------------------

    def collect_branches(self):
        local, remote = {}, {}
        for line in self.git.run("for-each-ref", "--format=%(refname)%09%(objectname)", "refs/heads",
                                 "refs/remotes/origin").splitlines():
            ref, sha = line.split("\t")
            if ref.startswith("refs/heads/"):
                local[ref[len("refs/heads/"):]] = sha
            elif ref != "refs/remotes/origin/HEAD":
                remote[ref[len("refs/remotes/origin/"):]] = sha
        for name in set(local) | set(remote):
            sha = remote.get(name) or local[name]
            head = {"name": name, "sha": sha, "pins": pins_at(self.git, sha), "local": "-",
                    "tag": None, "count": 0, "kind": "same"}
            if name in local and name in remote:
                if local[name] == remote[name]:
                    head["local"] = "="
                else:
                    ahead, behind = self.git.run("rev-list", "--left-right", "--count",
                                                 "%s...%s" % (local[name], remote[name])).split()
                    head["local"] = ", ".join(s for s in (
                        "ahead %s" % ahead if ahead != "0" else "",
                        "behind %s" % behind if behind != "0" else "") if s)
                    self.notes.append("your local %s is %s origin/%s; `git fetch origin %s:%s` "
                                      "fast-forwards it while it is not checked out"
                                      % (name, head["local"], name, name, name))
            elif name in local:
                head["local"] = "local only"
            tag = self.git.run("describe", "--tags", "--abbrev=0", sha, check=False).strip()
            if tag:
                head["tag"] = tag
                head["count"] = int(self.git.run("rev-list", "--count", "%s..%s" % (tag, sha)))
                head["kind"] = classify(self.git, tag, sha) if head["count"] else "same"
            within = set()
            for ref in self.git.run("branch", "-a", "--contains", sha, "--format=%(refname)").split():
                if ref.startswith("refs/remotes/origin/") and not ref.endswith("/HEAD"):
                    within.add(ref[len("refs/remotes/origin/"):])
                elif ref.startswith("refs/heads/"):
                    within.add(ref[len("refs/heads/"):])
            head["within"] = sorted(within - {name})
            newest = self.newest_tag(head["pins"].get("wpilibVersion"))
            head["stale"] = bool(not head["within"] and newest and
                                 self.commit_time[sha][0] < self.commit_time[dict(self.tags)[newest]][0])
            head["newest_tag"] = newest
            self.heads.append(head)
        self.heads.sort(key=lambda h: -self.commit_time[h["sha"]][0])

    def index(self):
        """src/main/java at every commit reachable from any ref, grouped by identical tree."""
        if self._index is None:
            trees = {}
            for sha in self.commit_time:
                tree = self.git.subtree(sha, CODE_ROOT)
                if tree:
                    trees.setdefault(tree, []).append(sha)
            union = set()
            for tree in trees:
                union.update(self.git.flatten(tree))
            self._index = (trees, union)
        return self._index

    def trace(self, sources_jar: Path):
        """(files that differ, tree, commits with that tree) for the closest src/main/java."""
        jar = {}
        with zipfile.ZipFile(sources_jar) as z:
            for n in z.namelist():
                if n.endswith(".java"):
                    data = z.read(n)
                    jar[n] = {blob_sha(data), blob_sha(data.replace(b"\r\n", b"\n"))}
        trees, union = self.index()
        generated = {p for p in jar if p not in union}  # e.g. a build-generated version class
        best = None
        for tree, commits in trees.items():
            files = self.git.flatten(tree)
            differ = sum(1 for p in jar if p not in generated and files.get(p) not in jar[p])
            extra = sum(1 for p in files if p not in jar)  # source-set excludes
            if best is None or (differ, extra) < best[0]:
                best = ((differ, extra), tree, commits)
        return (best[0][0], best[1], best[2]) if best else None

    def representative(self, commits, version):
        tagged = [c for c in commits if c in self.tag_by_commit]
        for c in tagged:
            if "v" + version in self.tag_by_commit[c]:
                return c
        return min(tagged or commits, key=lambda c: self.commit_time[c][0])

    def where(self, sha):
        if sha in self.tag_by_commit:
            return "tag " + ", ".join(self.tag_by_commit[sha])
        name = self.git.run("name-rev", "--name-only", "--no-undefined",
                            "--refs=refs/remotes/origin/*", "--exclude=refs/remotes/origin/HEAD",
                            sha, check=False).strip()
        return name.replace("remotes/origin/", "") if name else "(on no origin branch)"

    def collect_maven(self):
        base = self.m2 / MAVEN_PATH
        self.maven_base = base
        self.maven_release = None
        if not base.is_dir():
            return
        meta = base / "maven-metadata-local.xml"
        if meta.is_file():
            m = re.search(r"<release>([^<]+)</release>", meta.read_text("utf-8", "replace"))
            self.maven_release = m.group(1) if m else None
        tag_tree = {name: self.git.subtree(sha, CODE_ROOT) for name, sha in self.tags}
        found = []
        for d in base.iterdir():
            jar = d / ("FrcCatalyst-%s.jar" % d.name)
            if d.is_dir() and jar.is_file():
                found.append((jar.stat().st_mtime, d))
        found.sort(key=lambda b: (not b[1].name.startswith("1."), b[0]))
        for mtime, d in found:
            version = d.name
            build = {"version": version, "built": datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M"),
                     "mtime": mtime, "pom": self.pom(d / ("FrcCatalyst-%s.pom" % version)),
                     "source": "(no sources jar)", "sha": None, "where": "", "notes": []}
            src = d / ("FrcCatalyst-%s-sources.jar" % version)
            traced = self.trace(src) if src.is_file() else None
            if traced:
                differ, tree, commits = traced
                rep = self.representative(commits, version)
                build["sha"] = rep
                build["where"] = self.where(rep)
                if differ == 0:
                    build["source"] = "== " + self.short(rep)
                    later = [c for c in commits if self.commit_time[c][0] > self.commit_time[rep][0]]
                    if later:
                        build["notes"].append("same source in %s after it" % plural(len(later), "commit"))
                    declared = pins_at(self.git, rep).get("version")
                    if declared and declared != version:
                        build["notes"].append("build.gradle there says %s" % declared)
                else:
                    build["source"] = "~~ %s (%s differ)" % (self.short(rep), plural(differ, "file"))
                    build["notes"].append("built from uncommitted changes")
                tag = "v" + version
                if tag in tag_tree:
                    if tag_tree[tag] == tree and differ == 0:
                        build["notes"].append("is the %s tag" % tag)
                    else:
                        tag_sha = dict(self.tags)[tag]
                        build["notes"].append("NOT the %s tag (%s)" % (tag, self.short(tag_sha)))
                        self.notes.append("Maven local %s is not the %s tag (%s): its sources are %s "
                                          "(%s). Same number, different code."
                                          % (version, tag, self.short(tag_sha), self.short(rep),
                                             build["where"]))
                elif ITERATION_RE.search(version):
                    build["notes"].append("iteration build")
            self.builds.append(build)

    @staticmethod
    def pom(path: Path) -> dict:
        info = {}
        try:
            root = ET.parse(str(path)).getroot()
        except (OSError, ET.ParseError):
            return info
        ns = {"m": "http://maven.apache.org/POM/4.0.0"}
        for dep in root.iterfind(".//m:dependency", ns):
            group = (dep.findtext("m:groupId", "", ns) or "").strip()
            version = (dep.findtext("m:version", "", ns) or "").strip()
            for prefix, key in (("org.wpilib", "wpilib"), ("edu.wpi.first", "wpilib"),
                                ("com.ctre.phoenix6", "phoenix"), ("com.pathplanner", "pathplanner"),
                                ("com.limelightvision", "limelight"), ("org.photonvision", "photon")):
                if group.startswith(prefix):
                    info.setdefault(key, version)
        return info

    # ---- print ----------------------------------------------------------------------------

    def print_lines(self):
        """One row per WPILib a tag or a live branch pins: the answer to 'what runs on what'."""
        wpilibs = {p.get("wpilibVersion") for p in self.tag_pins.values() if p.get("wpilibVersion")}
        wpilibs = sorted((w for w in wpilibs if w), key=wpilib_key)
        rows = []
        for w in wpilibs:
            newest = self.newest_tag(w)
            if not newest or vkey(newest)[0] < 1:
                continue
            tag_time = self.commit_time[dict(self.tags)[newest]][0]
            # Past it: built on the newest tag, or newer than it and merged nowhere else.
            current = [h for h in self.heads if h["pins"].get("wpilibVersion") == w
                       and h["sha"] != dict(self.tags)[newest]
                       and (h["tag"] == newest or (not h["within"]
                                                   and self.commit_time[h["sha"]][0] > tag_time))]
            heads = ", ".join("%s %s (+%d %s)" % (h["name"], self.short(h["sha"]), h["count"], h["kind"])
                              if h["tag"] == newest else "%s %s" % (h["name"], self.short(h["sha"]))
                              for h in current) or "-"
            builds = [b for b in self.builds if b["pom"].get("wpilib") == w]
            latest = max(builds, key=lambda b: b["mtime"]) if builds else None
            row = [w]
            if self.matrix:
                row.append(self.matrix.image_for(w))
            row += [newest, heads,
                    ("%s %s" % (latest["version"], latest["source"])) if latest else "-"]
            if self.matrix:
                missing = self.matrix.missing_for(w)
                row.append("-" if missing is None else (", ".join(missing) or "none missing"))
                pins = self.tag_pins[newest]
                for pin, library in PIN_TO_MATRIX:
                    if missing and pins.get(pin) and library in missing:
                        self.notes.append("%s depends on %s %s, and upstream lists no %s release for "
                                          "WPILib %s" % (newest, library, pins[pin], library, w))
            rows.append(row)
            for h in current:
                if h["kind"] == "code" and h["tag"] == newest:
                    self.notes.append("the newest code for WPILib %s is %s (%s): %s of code past %s, "
                                      "and no tag" % (w, h["name"], self.short(h["sha"]),
                                                      plural(h["count"], "commit"), newest))
        headers = ["WPILib"] + (["runs on"] if self.matrix else []) + [
            "newest tag", "branches past it", "newest Maven local build"]
        if self.matrix:
            headers.append("upstream has no release of")
        section("LINES  (one per WPILib a release pins; `runs on` and the last column come from the "
                "upstream matrix)" if self.matrix else "LINES  (one per WPILib a release pins)")
        table(rows, headers)

    def print_tags(self):
        shown, hidden, by_major = [], 0, {}
        for name, sha in self.tags:
            k = vkey(name)
            if k is None or k[0] >= 2 or self.args.all_tags:
                shown.append((name, sha))
            else:
                by_major.setdefault(k[0], []).append((name, sha))
        for major in sorted(by_major, reverse=True):
            shown.insert(0, by_major[major][-1])
            hidden += len(by_major[major]) - 1
        rows = []
        for name, sha in shown:
            p = self.tag_pins[name]
            row = [name, self.short(sha), self.date(sha), p.get("wpilibVersion", "?"),
                   p.get("phoenixVersion", "-"), p.get("pathplannerVersion", "-"),
                   p.get("limelightVersion", "-"), p.get("agent", "-")]
            if self.jitpack:
                row.append(self.jitpack.get(name, self.jitpack.get(name.lstrip("v"), "not built")))
            rows.append(row)
            if vkey(name) and p.get("version", "?") != name.lstrip("v"):
                self.notes.append("tag %s: build.gradle there declares version %s" % (name, p.get("version")))
        headers = ["tag", "commit", "date", "WPILib", "Phoenix 6", "PathPlanner", "Limelight", "agent"]
        if self.jitpack:
            headers.append("JitPack")
        section("TAGS  (the pins are what build.gradle declares at the tag)")
        table(rows, headers)
        if hidden:
            out("  (+%d older tags on the 0.x/1.x lines; --all-tags lists them)" % hidden)

    def print_branches(self):
        rows = []
        for h in self.heads:
            since = "(no tag)"
            if h["tag"]:
                since = h["tag"] if not h["count"] else "%s +%d %s" % (h["tag"], h["count"], h["kind"])
            if h["stale"]:
                where = "stale: predates " + h["newest_tag"]
            elif h["within"]:
                where = "inside " + ", ".join(h["within"][:3]) + (
                    " +%d" % (len(h["within"]) - 3) if len(h["within"]) > 3 else "")
            else:
                where = ""
            rows.append([h["name"], self.short(h["sha"]), self.date(h["sha"]), since,
                         h["pins"].get("version", "?"), h["pins"].get("wpilibVersion", "?"),
                         h["local"], where])
        section("BRANCHES  (origin's head where there is one; `local` compares yours with it)")
        table(rows, ["branch", "commit", "date", "since tag", "version", "WPILib", "local", "notes"])

    def print_maven(self):
        section("MAVEN LOCAL  (%s)" % self.maven_base)
        if not self.builds:
            out("  (no Catalyst builds here)")
            return
        if self.maven_release:
            out("  maven-metadata-local.xml <release> is %s - what `latest.release` or `+` resolves to"
                % self.maven_release)
        rows = [[b["version"], b["built"], b["pom"].get("wpilib", "?"), b["source"], b["where"],
                 "; ".join(b["notes"])] for b in self.builds]
        table(rows, ["version", "built", "WPILib", "sources", "where", "notes"])
        if any(ITERATION_RE.search(b["version"]) for b in self.builds):
            out("  -aN builds were published while debugging on a robot; no tag carries that number.")

    def print_vendordeps(self):
        section("PUBLISHED VENDORDEPS  (what `Install new libraries (online)` gives a team today)")
        newest = {}
        for name, _ in self.tags:
            k = vkey(name)
            if k and (k[0] not in newest or k > vkey(newest[k[0]])):
                newest[k[0]] = name
        for label, url in PUBLISHED_VENDORDEPS:
            body, err = fetch(url)
            if err:
                out("  %-6s %s  (%s)" % (label, url, err))
                continue
            try:
                dep = json.loads(body)
            except ValueError:
                out("  %-6s %s  (not JSON)" % (label, url))
                continue
            coords = ["%s:%s:%s" % (j.get("groupId"), j.get("artifactId"), j.get("version"))
                      for j in dep.get("javaDependencies", []) if "TomAs-1226" in j.get("groupId", "")]
            out("  %-6s %s" % (label, url))
            out("         installs %s  (year %s)" % (", ".join(coords) or dep.get("version"),
                                                     dep.get("wpilibYear") or dep.get("frcYear") or "?"))
            version = "v" + str(dep.get("version", ""))
            k = vkey(version)
            if k and k[0] in newest and newest[k[0]] != version:
                self.notes.append("the %s vendordep installs %s; the newest %d.x tag is %s"
                                  % (label, version, k[0], newest[k[0]]))

    def print_matrix(self):
        section("SYSTEMCORE COMPATIBILITY  (upstream: %s)" % MATRIX_PAGE)
        if not self.matrix:
            out("  unavailable")
            return
        out("  README.md last changed %s" % getattr(self.matrix, "changed", "?"))
        for line in self.matrix.section:
            text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", line).replace(":x:", "none")
            text = text.replace("\\*", "*").rstrip()
            if text.startswith("|"):
                cells = [c.strip() for c in text.strip().strip("|").split("|")]
                if not all(re.fullmatch(r":?-*:?", c) for c in cells):
                    out("  " + " | ".join(cells))
            elif text.startswith("#"):
                out("  " + text.lstrip("#").strip())
            elif text.startswith(("*", "- ")):
                out("  " + text)

    def print_apps(self, dirs):
        section("COMPANION APPS")
        if not dirs:
            out("  (no CatalystApp or CatalystConsole checkout beside this repository; --apps DIR ...)")
            return
        rows = []
        for d in dirs:
            app = Git(d)
            try:
                refs = app.run("for-each-ref", "--format=%(refname)%09%(objectname)",
                               "refs/remotes/origin", "refs/heads").splitlines()
            except RuntimeError as e:
                out("  %s: %s" % (d, e))
                continue
            origin = {}
            entries = []
            for line in refs:
                ref, sha = line.split("\t")
                if ref.endswith("/HEAD"):
                    continue
                if ref.startswith("refs/remotes/origin/"):
                    origin[ref[len("refs/remotes/origin/"):]] = sha
                    entries.append(("origin/" + ref[len("refs/remotes/origin/"):], sha))
                else:
                    name = ref[len("refs/heads/"):]
                    if origin.get(name) != sha:
                        entries.append((name + " (local)", sha))
            for name, sha in entries:
                try:
                    version = json.loads(app.text(sha, "src-tauri/tauri.conf.json") or "{}").get("version", "?")
                except ValueError:
                    version = "?"
                bundled, runs_on = "-", ""
                vendordep = app.text(sha, "src-tauri/resources/FrcCatalyst.json")
                if vendordep:
                    try:
                        bundled = json.loads(vendordep).get("version", "?")
                    except ValueError:
                        bundled = "?"
                    w = self.tag_pins.get("v" + bundled, {}).get("wpilibVersion")
                    runs_on = w or ""
                tags = app.run("tag", "--points-at", sha).split()
                rows.append([d.name, name, sha[:7], version, bundled, runs_on, ", ".join(tags) or "-"])
        table(rows, ["repo", "branch", "commit", "app", "bundles library", "that library's WPILib",
                     "release tag"])

    def print_notes(self):
        section("NOTES")
        if not self.notes:
            out("  nothing to flag")
        for n in dict.fromkeys(self.notes):
            out("  ! " + n)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", help="the FrcCatalyst checkout (default: the one this script is in)")
    ap.add_argument("--m2", help="Maven local repository (default: ~/.m2/repository)")
    ap.add_argument("--matrix", action="store_true", help="add the SystemcoreTesting matrix")
    ap.add_argument("--online", action="store_true",
                    help="--matrix, plus JitPack status and the vendordeps the docs site serves")
    ap.add_argument("--apps", nargs="*", metavar="DIR",
                    help="CatalystApp / CatalystConsole checkouts (default: beside this repo)")
    ap.add_argument("--fetch", action="store_true", help="run `git fetch origin` first")
    ap.add_argument("--all-tags", action="store_true", help="list every tag")
    args = ap.parse_args(argv)

    repo = Path(args.repo) if args.repo else Path(__file__).resolve().parent.parent
    git = Git(repo)
    try:
        top = Path(git.run("rev-parse", "--show-toplevel").strip())
        common = Path(git.run("rev-parse", "--git-common-dir").strip())
    except RuntimeError as e:
        sys.exit("not a git checkout: %s (%s)" % (repo, e))
    if not common.is_absolute():
        common = (top / common).resolve()
    if args.fetch:
        out("git fetch origin ...")
        git.run("fetch", "origin")

    vm = VersionMap(git, Path(args.m2) if args.m2 else Path.home() / ".m2" / "repository", args)
    if args.online:
        vm.load_jitpack()
    if args.matrix or args.online:
        vm.load_matrix()
    vm.collect_branches()
    vm.collect_maven()

    fetched = common / "FETCH_HEAD"
    out("Catalyst version map, %s" % datetime.now().strftime("%Y-%m-%d %H:%M"))
    out("  repo     %s" % top)
    out("  fetched  %s  (--fetch refreshes origin's refs first)" % (
        datetime.fromtimestamp(fetched.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
        if fetched.exists() else "never"))
    vm.print_lines()
    vm.print_tags()
    vm.print_branches()
    vm.print_maven()
    if args.online:
        vm.print_vendordeps()
    if args.matrix or args.online:
        vm.print_matrix()
    if args.apps is not None:
        siblings = common.parent.parent
        dirs = [Path(p) for p in args.apps] or [
            d for d in (siblings / "CatalystApp", siblings / "CatalystConsole") if (d / ".git").exists()]
        vm.print_apps(dirs)
    vm.print_notes()


if __name__ == "__main__":
    main()
