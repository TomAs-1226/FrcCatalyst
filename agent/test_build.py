#!/usr/bin/env python3
"""The packaging invariants, checked against deliberately broken packages.

Every fault here produces a package that builds without complaint, installs without complaint, and
then does not work — on a board, over SSH, usually at a competition. That is the whole reason the
checker exists, and a checker nobody has watched fail is not evidence of anything.

So these tests do not build a good package and assert it is good. They take the good package apart,
break one thing, and assert the checker says so.

    python3 test_build.py
"""

import gzip
import importlib.util
import io
import tarfile
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent

_spec = importlib.util.spec_from_file_location("catalyst_build", HERE / "build.py")
build = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(build)


def ar_members(blob):
    """Read an ar archive back into {name: bytes}, in order."""
    out, i = [], 8
    while i + 60 <= len(blob):
        header = blob[i:i + 60]
        name = header[0:16].decode().strip().rstrip("/")
        size = int(header[48:58].decode().strip())
        out.append((name, blob[i + 60:i + 60 + size]))
        i += 60 + size + (size % 2)
    return out


def repack(targz, mutate=None, drop=None):
    """Rebuild a member tarball, optionally mutating or dropping entries."""
    raw = io.BytesIO()
    with tarfile.open(fileobj=io.BytesIO(gzip.decompress(targz))) as old, \
         tarfile.open(fileobj=raw, mode="w") as new:
        for ti in old.getmembers():
            if drop and drop(ti):
                continue
            data = old.extractfile(ti).read() if ti.isfile() else b""
            if mutate:
                ti, data = mutate(ti, data)
            new.addfile(ti, io.BytesIO(data) if ti.isfile() else None)
    out = io.BytesIO()
    with gzip.GzipFile(fileobj=out, mode="wb", mtime=0) as gz:
        gz.write(raw.getvalue())
    return out.getvalue()


class Packaging(unittest.TestCase):
    """One good package, built once, taken apart by each test."""

    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.TemporaryDirectory()
        cls.good = build.build()
        cls.blob = cls.good.read_bytes()
        cls.members = dict(ar_members(cls.blob))

    @classmethod
    def tearDownClass(cls):
        cls.dir.cleanup()

    # -- helpers ----------------------------------------------------------

    def check(self, members):
        """Write an ar archive of `members` and return the checker's problem count."""
        p = Path(self.dir.name) / f"{self.id().rsplit('.', 1)[-1]}.ipk"
        p.write_bytes(build.ar_archive(members))
        return build.check(p)

    def parts(self, control=None, data=None, debian=b"2.0\n"):
        out = []
        if debian is not None:
            out.append(("debian-binary", debian))
        out.append(("control.tar.gz", control or self.members["control.tar.gz"]))
        out.append(("data.tar.gz", data or self.members["data.tar.gz"]))
        return out

    # -- the package we actually ship -------------------------------------

    def test_the_built_package_passes(self):
        self.assertEqual(0, build.check(self.good))

    def test_it_is_reproducible(self):
        # So "is this the package I tested?" has an answer. Two builds of unchanged source must be
        # the same bytes; anything time- or host-dependent that creeps in breaks this first.
        again = build.build().read_bytes()
        self.assertEqual(self.blob, again)

    # -- the archive shape ------------------------------------------------

    def test_a_missing_debian_binary_is_caught(self):
        # opkg tolerates its absence and dpkg-based tooling does not, so a package without it works
        # on one installer and not another - found on a field, not on a bench.
        self.assertNotEqual(0, self.check([
            ("control.tar.gz", self.members["control.tar.gz"]),
            ("data.tar.gz", self.members["data.tar.gz"]),
        ]))

    def test_members_out_of_order_are_caught(self):
        self.assertNotEqual(0, self.check([
            ("debian-binary", b"2.0\n"),
            ("data.tar.gz", self.members["data.tar.gz"]),
            ("control.tar.gz", self.members["control.tar.gz"]),
        ]))

    # -- the failures that are invisible until the board runs it -----------

    def test_a_payload_without_its_executable_bit_is_caught(self):
        # ExecStart names catalyst_agent.py directly, so this is systemd 203/EXEC: an error that
        # names the file and says nothing about the mode.
        def unexec(ti, data):
            if ti.name.endswith("catalyst_agent.py"):
                ti.mode = 0o644
            return ti, data

        self.assertNotEqual(0, self.check(self.parts(
            data=repack(self.members["data.tar.gz"], mutate=unexec))))

    def test_a_maintainer_script_without_its_executable_bit_is_caught(self):
        def unexec(ti, data):
            if ti.name.endswith("postinst"):
                ti.mode = 0o644
            return ti, data

        self.assertNotEqual(0, self.check(self.parts(
            control=repack(self.members["control.tar.gz"], mutate=unexec))))

    def test_a_crlf_shebang_is_caught(self):
        # The one a Windows build host introduces silently. On the board it is `bad interpreter` for
        # a file that is plainly present, which reads as anything but a line-ending problem.
        def crlf(ti, data):
            if ti.name.endswith(".py"):
                data = data.replace(b"\n", b"\r\n")
                ti.size = len(data)
            return ti, data

        self.assertNotEqual(0, self.check(self.parts(
            data=repack(self.members["data.tar.gz"], mutate=crlf))))

    def test_files_owned_by_the_build_user_are_caught(self):
        # opkg preserves ownership, so a package built as an unprivileged user installs files owned
        # by a uid that may not exist on the board.
        def owned(ti, data):
            ti.uid = ti.gid = 1000
            ti.uname = ti.gname = "builder"
            return ti, data

        self.assertNotEqual(0, self.check(self.parts(
            data=repack(self.members["data.tar.gz"], mutate=owned))))

    def test_a_missing_service_unit_is_caught(self):
        self.assertNotEqual(0, self.check(self.parts(
            data=repack(self.members["data.tar.gz"],
                        drop=lambda ti: ti.name.endswith(".service")))))

    def test_a_missing_payload_is_caught(self):
        # The failure the .gitignore bug actually produced: a package with a unit and no program.
        self.assertNotEqual(0, self.check(self.parts(
            data=repack(self.members["data.tar.gz"],
                        drop=lambda ti: ti.name.endswith("catalyst_agent.py")))))

    def test_build_residue_is_caught(self):
        def add_pyc(_ti, _data):
            return _ti, _data

        raw = io.BytesIO()
        with tarfile.open(fileobj=io.BytesIO(gzip.decompress(self.members["data.tar.gz"]))) as old, \
             tarfile.open(fileobj=raw, mode="w") as new:
            for ti in old.getmembers():
                new.addfile(ti, io.BytesIO(old.extractfile(ti).read()) if ti.isfile() else None)
            junk = tarfile.TarInfo("./usr/local/bin/catalyst-agent/__pycache__/x.cpython-312.pyc")
            junk.size, junk.mode = 3, 0o644
            new.addfile(junk, io.BytesIO(b"abc"))
        out = io.BytesIO()
        with gzip.GzipFile(fileobj=out, mode="wb", mtime=0) as gz:
            gz.write(raw.getvalue())

        self.assertNotEqual(0, self.check(self.parts(data=out.getvalue())))

    # -- content ----------------------------------------------------------

    def test_the_control_file_declares_an_installed_size(self):
        # The Systemcore web UI shows it before installing; without it the package reports an
        # unknown size, which looks like a broken package.
        with tarfile.open(fileobj=io.BytesIO(gzip.decompress(self.members["control.tar.gz"]))) as tar:
            text = tar.extractfile("./control").read().decode()
        self.assertIn("Installed-Size:", text)
        size = int([l for l in text.splitlines() if l.startswith("Installed-Size:")][0].split(":")[1])
        self.assertGreater(size, 0)

    def test_the_unit_points_at_a_file_the_package_actually_contains(self):
        # These two are written in different files by different people and nothing else checks that
        # they agree. When they disagree the service fails at boot with 203/EXEC.
        with tarfile.open(fileobj=io.BytesIO(gzip.decompress(self.members["data.tar.gz"]))) as tar:
            names = {m.name.lstrip("./") for m in tar.getmembers()}
            unit = tar.extractfile("./etc/systemd/system/catalyst-agent.service").read().decode()

        exec_start = [l for l in unit.splitlines() if l.startswith("ExecStart=")][0]
        target = exec_start.split("=", 1)[1].split()[0].lstrip("/")
        self.assertIn(target, names, f"ExecStart points at /{target}, which is not in the package")


if __name__ == "__main__":
    unittest.main(verbosity=2)
