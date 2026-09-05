#!/usr/bin/env python3
"""Tests for the Catalyst agent's parsing.

    python agent/test_agent.py

The agent runs on Systemcore and is written on a laptop, so the parsing is the part that has to be
checked somewhere other than the robot. All of it reads Linux text formats that are easy to parse
almost correctly:

  - /proc/stat puts the command name in parentheses and the name may contain spaces, so splitting
    the line on whitespace shifts every field after it;
  - /proc/meminfo is in kibibytes with a unit suffix, and reading it as bytes understates memory by
    1024x - which looks like a machine that is fine;
  - MemFree is not the memory a program can get, because the kernel spends it all on cache.

Each of those produces a plausible number rather than an error, which is why they are worth a test.
"""

import json
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "overlay", "usr", "local", "bin", "catalyst-agent"))

import catalyst_agent as agent  # noqa: E402


def with_files(files):
    """Serve fixture text for the paths a reader asks for."""
    def fake_read(path, default=""):
        return files.get(path, default)
    return mock.patch.object(agent, "read_text", side_effect=fake_read)


class Memory(unittest.TestCase):
    MEMINFO = """MemTotal:        8123456 kB
MemFree:          412000 kB
MemAvailable:    5891234 kB
Buffers:          102400 kB
Cached:          4500000 kB
SwapTotal:             0 kB
SwapFree:              0 kB
"""

    def test_kibibytes_become_bytes(self):
        # Reading these as bytes understates memory 1024x, which reads as a machine with 8 MB of RAM
        # and 5 MB free - odd enough to notice, but the ratio still comes out right, so a dashboard
        # showing only a percentage would look perfectly healthy.
        with with_files({"/proc/meminfo": self.MEMINFO}):
            m = agent.memory()
        self.assertEqual(m["totalBytes"], 8123456 * 1024)
        self.assertEqual(m["cachedBytes"], 4500000 * 1024)

    def test_used_is_measured_against_available_not_free(self):
        # The distinction that matters on Linux. Free memory is near zero on any healthy machine
        # because the kernel uses it all for cache and hands it back on demand; computing "used" from
        # MemFree would report this 8 GB machine as 95% full and permanently in the red.
        with with_files({"/proc/meminfo": self.MEMINFO}):
            m = agent.memory()
        self.assertEqual(m["usedBytes"], (8123456 - 5891234) * 1024)
        self.assertLess(m["usedBytes"] / m["totalBytes"], 0.3)

    def test_a_machine_that_reports_nothing_reports_nothing(self):
        with with_files({}):
            m = agent.memory()
        self.assertIsNone(m["totalBytes"])
        self.assertIsNone(m["usedBytes"])


class Cpu(unittest.TestCase):
    def _stat(self, busy0, busy1, total):
        """A /proc/stat where each core has spent `busy` of `total` jiffies working.

        Both figures are cumulative since boot, which is the whole reason a delta is needed: the
        second sample has to carry a larger total than the first, or there is no interval to
        divide by.
        """
        # user nice system idle iowait irq softirq steal
        return "\n".join([
            "cpu  0 0 0 0 0 0 0 0",
            f"cpu0 {busy0} 0 0 {total - busy0} 0 0 0 0",
            f"cpu1 {busy1} 0 0 {total - busy1} 0 0 0 0",
            "intr 12345",
        ])

    def test_per_core_utilisation_is_a_delta_not_a_total(self):
        # Jiffies are cumulative since boot. Reporting them raw would show a machine that has been
        # up for a week as permanently at 100%.
        # Nothing used, then 500 of the next 1000 jiffies on core 0 and 250 on core 1.
        reads = [self._stat(0, 0, 0), self._stat(500, 250, 1000)]

        def fake_read(path, default=""):
            return reads.pop(0) if path == "/proc/stat" and reads else ""

        with mock.patch.object(agent, "read_text", side_effect=fake_read):
            with mock.patch.object(agent.time, "sleep"):
                c = agent.cpu()

        self.assertEqual(len(c["cores"]), 2)
        self.assertAlmostEqual(c["cores"][0]["percent"], 50.0, places=1)
        self.assertAlmostEqual(c["cores"][1]["percent"], 25.0, places=1)

    def test_cores_come_back_in_order(self):
        # cpu10 must not sort between cpu1 and cpu2. Rows out of order on a dashboard get read as
        # the wrong core being busy.
        stat = "\n".join(["cpu  0 0 0 0"] + [f"cpu{i} 10 0 0 90" for i in range(12)])
        with mock.patch.object(agent, "read_text", side_effect=lambda p, d="": stat
                               if p == "/proc/stat" else ""):
            with mock.patch.object(agent.time, "sleep"):
                c = agent.cpu()
        self.assertEqual([row["core"] for row in c["cores"]], list(range(12)))

    def test_no_procfs_is_no_cores_rather_than_an_exception(self):
        with with_files({}):
            with mock.patch.object(agent.time, "sleep"):
                c = agent.cpu()
        self.assertEqual(c["cores"], [])


class Throttling(unittest.TestCase):
    def test_the_flag_bits_are_split_into_now_and_since_boot(self):
        # The distinction the whole reading exists for: a robot that throttled during its last match
        # and is cool now still has the since-boot bit set, and that is the evidence.
        with mock.patch.object(agent, "run", return_value="throttled=0x50005\n"):
            t = agent._throttling()
        self.assertTrue(t["underVoltageNow"])
        self.assertTrue(t["throttledNow"])
        self.assertTrue(t["underVoltageSinceBoot"])
        self.assertTrue(t["throttledSinceBoot"])

    def test_a_healthy_soc_reports_all_clear(self):
        with mock.patch.object(agent, "run", return_value="throttled=0x0\n"):
            t = agent._throttling()
        self.assertFalse(any(t.values()))

    def test_a_machine_without_vcgencmd_says_nothing_rather_than_all_clear(self):
        # Absent is not healthy. Reporting every flag false because the tool is missing would be the
        # agent inventing an all-clear.
        with mock.patch.object(agent, "run", return_value=""):
            self.assertIsNone(agent._throttling())


class Processes(unittest.TestCase):
    def test_a_command_name_containing_spaces_does_not_shift_every_field(self):
        # The classic /proc/stat bug. Splitting on whitespace makes "(Robot Main)" two fields, so
        # utime, stime and rss all come from one position to the left - and the numbers that come
        # out are still numbers, just wrong.
        fields = " ".join(["S", "1", "1", "1", "0", "-1", "0", "0", "0", "0", "0"]
                          + ["400", "100"]              # utime, stime
                          + ["0"] * 8
                          + ["5000"]                    # rss in pages
                          + ["0"] * 10)
        stat = f"42 (Robot Main) {fields}"

        def fake_read(path, default=""):
            return stat if path == "/proc/42/stat" else ""

        with mock.patch.object(agent.os, "listdir", return_value=["42", "self"]):
            with mock.patch.object(agent, "read_text", side_effect=fake_read):
                with mock.patch.object(agent.time, "sleep"):
                    p = agent.processes()

        self.assertEqual(p["count"], 1)
        row = p["topByCpu"][0]
        self.assertEqual(row["name"], "Robot Main")
        self.assertEqual(row["pid"], 42)
        self.assertEqual(row["rssBytes"], 5000 * agent._sysconf("SC_PAGE_SIZE", 4096))

    def test_no_procfs_reports_no_processes(self):
        with mock.patch.object(agent.os, "listdir", side_effect=OSError):
            with mock.patch.object(agent.time, "sleep"):
                p = agent.processes()
        self.assertEqual(p["count"], 0)


class Identity(unittest.TestCase):
    def test_os_release_is_unquoted(self):
        with with_files({
            "/etc/os-release": 'PRETTY_NAME="Systemcore OS 2027.0.0-beta14"\nVERSION_ID="2027.0.0"\n',
            "/proc/uptime": "12345.67 98765.43\n",
            "/proc/sys/kernel/osrelease": "6.12.77-rt\n",
        }):
            i = agent.identity()
        self.assertEqual(i["os"], "Systemcore OS 2027.0.0-beta14")
        self.assertEqual(i["versionId" if "versionId" in i else "osVersion"], "2027.0.0")
        self.assertEqual(i["kernel"], "6.12.77-rt")
        self.assertAlmostEqual(i["uptimeSeconds"], 12345.67)


class Storage(unittest.TestCase):
    MOUNTS = """/dev/mmcblk0p2 / ext4 rw,relatime 0 0
proc /proc proc rw,nosuid 0 0
tmpfs /run tmpfs rw,nosuid 0 0
sysfs /sys sysfs rw,nosuid 0 0
/dev/sda1 /U vfat rw,relatime 0 0
"""

    def test_only_real_filesystems_are_reported(self):
        # A Linux box has dozens of pseudo-filesystems. Listing them as storage buries the one mount
        # anybody is asking about.
        class Stat:
            f_blocks, f_frsize, f_bavail = 1000, 4096, 400

        with with_files({"/proc/mounts": self.MOUNTS}):
            with mock.patch.object(agent.os, "statvfs", create=True, return_value=Stat()):
                with mock.patch.object(agent, "run", return_value=""):
                    with mock.patch.object(agent.os.path, "isdir", return_value=False):
                        s = agent.storage()

        points = [m["mount"] for m in s["mounts"]]
        self.assertEqual(points, ["/", "/U"])
        self.assertEqual(s["mounts"][0]["usedBytes"], (1000 - 400) * 4096)


class Endpoints(unittest.TestCase):
    def test_the_whole_snapshot_serialises(self):
        # Every value has to survive json.dumps. A stray bytes or a set from some future reader
        # would fail the request rather than the field, and the Console would see a dead agent.
        snap = agent.snapshot()
        json.dumps(snap)
        for key in ("identity", "cpu", "thermal", "memory", "storage",
                    "processes", "can", "network", "robotProgram"):
            self.assertIn(key, snap)

    def test_there_are_no_endpoints_that_change_anything(self):
        # The boundary this agent is built on, asserted rather than assumed. A do_POST appearing here
        # later should fail this test and make somebody argue for it.
        handler_methods = [n for n in dir(agent.Handler) if n.startswith("do_")]
        self.assertEqual(handler_methods, ["do_GET"])


class MotorHistory(unittest.TestCase):
    DOC = {"format": "catalyst-motor-history", "version": 1, "updatedMs": 1, "clockTrusted": True,
           "devices": [{"serial": "000E0B500C776800000A0001160000E3", "model": "Talon FX", "kind": "motor",
                        "boots": 3, "firstSeenMs": 10, "lastSeenMs": 20,
                        "identities": [{"id": 45, "name": "Intake Roller", "bus": "can_s2", "firmware": "26.1.0.0"},
                                       {"id": 45, "name": "BL, \"Drive\"", "bus": "can_s2", "firmware": "26.1.1.1"}],
                        "totals": {"poweredSeconds": 120.5, "runningSeconds": 30, "revolutions": 1500.25,
                                   "peakStatorAmps": 80, "peakTempC": 61, "hotSeconds": 0, "stickyFaults": 4}}]}

    def _with_file(self, text):
        import tempfile
        f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8")
        f.write(text)
        f.close()
        return f.name

    def test_a_missing_file_is_an_answer_not_an_exception(self):
        doc = agent.motor_history("/nonexistent/motor-history.json")
        self.assertIn("error", doc)
        self.assertEqual(doc["devices"], [])

    def test_the_file_is_handed_over_as_it_is(self):
        path = self._with_file(json.dumps(self.DOC))
        doc = agent.motor_history(path)
        self.assertNotIn("error", doc)
        self.assertEqual(doc["devices"][0]["serial"], "000E0B500C776800000A0001160000E3")
        self.assertEqual(doc["path"], path)
        os.unlink(path)

    def test_the_csv_is_the_latest_identity_and_the_totals(self):
        csv = agent.motor_history_csv(self.DOC)
        lines = csv.strip().split("\n")
        self.assertEqual(lines[0].split(",")[:6], ["serial", "model", "kind", "bus", "id", "name"])
        self.assertEqual(len(lines), 2)
        self.assertIn('"BL, ""Drive"""', lines[1], "a name with a comma and quotes is quoted")
        self.assertIn(",26.1.1.1,120.5,30,", lines[1], "the latest firmware, then the totals")
        self.assertTrue(lines[1].endswith(",2,4"), "identity count and sticky faults close the row")

    def test_garbage_is_reported_not_raised(self):
        path = self._with_file("{not json")
        doc = agent.motor_history(path)
        self.assertIn("error", doc)
        os.unlink(path)

    def test_the_new_routes_are_still_only_gets(self):
        handler_methods = [n for n in dir(agent.Handler) if n.startswith("do_")]
        self.assertEqual(handler_methods, ["do_GET"])


class Cameras(unittest.TestCase):
    AGGREGATED = {"cameras": [
        {"host": "limelight-left", "ip": "10.58.5.12", "type": "limelight4", "ntConnected": True,
         "aliasIps": ["172.30.0.220", "172.26.0.220"], "fps": 55.0, "interface": "eth",
         "uiUrl": "http://172.30.0.220:5801/", "mjpegUrl": "http://172.30.0.220:5800/",
         "pipelineType": "pipe_fiducial"},
        {"host": "limelight", "ip": "10.58.5.13", "ntConnected": False, "aliasIp": "172.26.0.221"},
    ]}

    def test_each_camera_status_is_joined_by_ip(self):
        merged = agent.merge_cameras(self.AGGREGATED, {
            "10.58.5.12": {"name": "limelight-left", "temp": 71.6, "cpu": 75.0, "fps": 56.6,
                           "ram": 63.2, "pipelineType": "pipe_fiducial", "pipelineIndex": 0},
        })
        left = next(c for c in merged if c["ip"] == "10.58.5.12")
        self.assertEqual(left["temperatureC"], 71.6)
        self.assertEqual(left["fps"], 56.6)             # the camera's own number beats the OS's
        self.assertTrue(left["ntConnected"])
        self.assertTrue(left["statusReachable"])
        self.assertEqual(left["aliasIps"], ["172.30.0.220", "172.26.0.220"])

    def test_a_camera_that_did_not_answer_still_appears(self):
        # The OS saw it, so it exists. Hiding it because its REST API timed out would turn a slow
        # camera into an absent one on the dashboard, which is the opposite of the truth.
        merged = agent.merge_cameras(self.AGGREGATED, {})
        other = next(c for c in merged if c["ip"] == "10.58.5.13")
        self.assertIsNone(other["temperatureC"])
        self.assertFalse(other["statusReachable"])
        self.assertFalse(other["ntConnected"])
        self.assertEqual(other["aliasIps"], ["172.26.0.221"])
        self.assertEqual(other["name"], "limelight")

    def test_garbage_from_the_aggregator_is_an_empty_list(self):
        self.assertEqual(agent.merge_cameras(None, {}), [])
        self.assertEqual(agent.merge_cameras({"cameras": ["nonsense", 7]}, {}), [])

    def test_an_unreachable_aggregator_is_reported_not_raised(self):
        agent._camera_cache.update(at=0.0, value=None)
        with mock.patch.object(agent, "_http_json", side_effect=OSError("connection refused")):
            answer = agent.cameras()
        self.assertFalse(answer["available"])
        self.assertEqual(answer["cameras"], [])
        self.assertIn("aggregator", answer["reason"])
        agent._camera_cache.update(at=0.0, value=None)

    def test_the_answer_is_cached_briefly(self):
        agent._camera_cache.update(at=0.0, value=None)
        calls = []

        def fake(url, timeout):
            calls.append(url)
            return self.AGGREGATED if "4810" in url else {"name": "x", "temp": 50.0}

        with mock.patch.object(agent, "_http_json", side_effect=fake):
            first = agent.cameras()
            second = agent.cameras()
        self.assertIs(first, second)
        self.assertEqual(sum(1 for u in calls if "4810" in u), 1)
        agent._camera_cache.update(at=0.0, value=None)

    def test_the_route_is_a_get_like_everything_else(self):
        src = open(agent.__file__, encoding="utf-8").read()
        self.assertIn('route == "/api/cameras"', src)
        self.assertNotIn("def do_POST", src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
