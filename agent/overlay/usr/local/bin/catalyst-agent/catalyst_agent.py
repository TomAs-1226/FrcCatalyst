#!/usr/bin/env python3
"""Read-only system telemetry for Catalyst Console.

Systemcore publishes a useful summary of itself on its NetworkTables server, and Catalyst mirrors it
so it arrives with everything else. But that summary is a summary: one CPU figure for four cores, a
storage percentage with no idea what filled it, and nothing at all about the robot program's own
process. The questions a pit crew actually asks in the ninety seconds before a match are the ones it
cannot answer.

  - The processor is pinned. Which core, and by what?
  - Storage is at 94%. What is using it, and can I delete it?
  - The robot program restarted. When, how many times, and what did it say on the way down?
  - CAN looks fine but a motor dropped out. Were there bus errors?

All of that is sitting in /proc and /sys on the machine already. This serves it over HTTP so the
Console can ask.

## Read-only, deliberately

There are no endpoints that change anything. Not a simplification - a stated boundary. Catalyst
Console is read-only by design, and an agent that could restart the robot program would put a
"restart" button one mis-click from a driver during a match. Diagnosis and control are different
tools, and this is the diagnosis one.

## What it costs

Sampled on request, not on a timer, so an idle agent is an idle process. The one place it blocks is
the CPU sample, which needs two reads of /proc/stat a short interval apart; that interval is the
agent's entire cost and it is 100 ms. Nothing here touches the robot program, the CAN buses, or the
control loop.

Standard library only. The device has Python 3 and nothing else is guaranteed.
"""

import concurrent.futures
import http.server
import json
import os
import re
import signal
import socket
import socketserver
import subprocess
import sys
import time
import urllib.request

PORT = 9010

# How long to wait between the two /proc/stat reads that a per-core figure needs. Long enough to be
# a real measurement, short enough that the Console's poll does not feel like a request that hung.
CPU_SAMPLE_SECONDS = 0.1

# Reading every process twice is the expensive part of this whole file, so the list is trimmed to
# what a person would actually look at.
TOP_PROCESSES = 8

# A log tail long enough to hold a stack trace, short enough not to make the response unwieldy.
LOG_LINES = 60


# --------------------------------------------------------------------------- helpers

def read_text(path, default=""):
    """Read a file, or return the default. Nothing here is worth failing a whole response over."""
    try:
        with open(path, "r", errors="replace") as handle:
            return handle.read()
    except OSError:
        return default


def read_int(path):
    raw = read_text(path).strip()
    try:
        return int(raw)
    except ValueError:
        return None


def run(argv, timeout=3):
    """Run a command, returning stdout or "" — never raising, and never hanging the response."""
    try:
        done = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, check=False)
        return done.stdout
    except (OSError, subprocess.SubprocessError):
        return ""


# --------------------------------------------------------------------------- identity

def identity():
    """What this machine is and what it is running.

    None of this reaches NetworkTables, and it is the first thing anyone asks when a robot behaves
    differently from the one next to it: are they on the same OS build?
    """
    os_release = {}
    for line in read_text("/etc/os-release").splitlines():
        if "=" in line:
            key, _, value = line.partition("=")
            os_release[key.strip()] = value.strip().strip('"')

    uptime_raw = read_text("/proc/uptime").split()
    uptime = float(uptime_raw[0]) if uptime_raw else None

    # /proc/version is long and mostly the compiler. The release is the part anyone reads.
    kernel = read_text("/proc/sys/kernel/osrelease").strip() or None

    model = read_text("/proc/device-tree/model").strip("\x00 \n") or None

    return {
        "hostname": socket.gethostname(),
        "os": os_release.get("PRETTY_NAME") or os_release.get("NAME"),
        "osVersion": os_release.get("VERSION_ID"),
        "kernel": kernel,
        "model": model,
        "uptimeSeconds": uptime,
        "agentVersion": "2.0.2",
    }


# --------------------------------------------------------------------------- cpu

def _cpu_times():
    """Per-core busy and total jiffies from /proc/stat."""
    times = {}
    for line in read_text("/proc/stat").splitlines():
        if not line.startswith("cpu"):
            continue
        parts = line.split()
        name = parts[0]
        if name == "cpu":
            continue
        try:
            values = [int(v) for v in parts[1:]]
        except ValueError:
            continue
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)
        times[name] = (total - idle, total)
    return times


def cpu():
    """Per-core utilisation, clock speeds, and whether the SoC is throttling.

    Per-core matters because the summary figure hides the shape of the problem: one core pinned at
    100% while three idle averages to 25% and reads as a quiet machine, but it is a thread stuck in
    a loop and the robot program is on that core.
    """
    first = _cpu_times()
    time.sleep(CPU_SAMPLE_SECONDS)
    second = _cpu_times()

    cores = []
    for name in sorted(first, key=lambda n: int(n[3:] or 0)):
        if name not in second:
            continue
        busy0, total0 = first[name]
        busy1, total1 = second[name]
        span = total1 - total0
        percent = round(100.0 * (busy1 - busy0) / span, 1) if span > 0 else None

        index = name[3:]
        khz = read_int(f"/sys/devices/system/cpu/cpu{index}/cpufreq/scaling_cur_freq")
        cores.append({
            "core": int(index) if index.isdigit() else index,
            "percent": percent,
            "mhz": round(khz / 1000) if khz else None,
        })

    # Load average is the queue, not the usage. A machine at 40% CPU with a load of 6 is waiting on
    # something - usually IO - and that reads completely differently from 40% with a load of 0.5.
    try:
        one, five, fifteen = os.getloadavg()
        load = [round(one, 2), round(five, 2), round(fifteen, 2)]
    except (OSError, AttributeError):
        # AttributeError on platforms without it. The agent only ever runs on Systemcore, but it is
        # developed and tested elsewhere, and a diagnostic tool that cannot be run off-target is one
        # nobody checks before shipping.
        load = None

    model = None
    for line in read_text("/proc/cpuinfo").splitlines():
        if line.lower().startswith("model name"):
            model = line.partition(":")[2].strip()
            break

    return {"cores": cores, "loadAverage": load, "model": model, "throttling": _throttling()}


def _throttling():
    """Whether the SoC is currently throttling, from the Pi firmware's own flag.

    This is the reason a thermal problem does not look thermal: Linux slows the cores and reports
    nothing, so the symptom that reaches robot code is a loop overrun.
    """
    out = run(["vcgencmd", "get_throttled"])
    match = re.search(r"throttled=0x([0-9a-fA-F]+)", out)
    if not match:
        return None
    bits = int(match.group(1), 16)
    return {
        "underVoltageNow": bool(bits & 0x1),
        "frequencyCappedNow": bool(bits & 0x2),
        "throttledNow": bool(bits & 0x4),
        "softTempLimitNow": bool(bits & 0x8),
        "throttledSinceBoot": bool(bits & 0x40000),
        "underVoltageSinceBoot": bool(bits & 0x10000),
    }


def thermal():
    """Every thermal zone the kernel exposes, named."""
    zones = []
    base = "/sys/class/thermal"
    try:
        names = sorted(n for n in os.listdir(base) if n.startswith("thermal_zone"))
    except OSError:
        return zones

    for name in names:
        milli = read_int(f"{base}/{name}/temp")
        if milli is None:
            continue
        zones.append({
            "zone": read_text(f"{base}/{name}/type").strip() or name,
            "celsius": round(milli / 1000.0, 1),
        })
    return zones


# --------------------------------------------------------------------------- memory

def memory():
    """/proc/meminfo, in bytes.

    MemAvailable rather than MemFree: free memory on Linux is a meaningless number, because the
    kernel spends it all on cache and gives it back on demand. Available is what a program can
    actually get, and it is the one that answers "is this machine short of memory".
    """
    values = {}
    for line in read_text("/proc/meminfo").splitlines():
        key, _, rest = line.partition(":")
        parts = rest.split()
        if parts and parts[0].isdigit():
            values[key.strip()] = int(parts[0]) * 1024

    total = values.get("MemTotal")
    available = values.get("MemAvailable")
    return {
        "totalBytes": total,
        "availableBytes": available,
        "usedBytes": (total - available) if total and available is not None else None,
        "cachedBytes": values.get("Cached"),
        "swapTotalBytes": values.get("SwapTotal"),
        "swapFreeBytes": values.get("SwapFree"),
    }


# --------------------------------------------------------------------------- storage

# Where a robot's own data ends up. Reported separately from the filesystem total, because "the disk
# is 94% full" is not actionable and "your logs are 22 GB of it" is.
DATA_DIRS = ["/home/systemcore", "/var/log", "/U", "/V"]


def storage():
    mounts = []
    seen = set()
    for line in read_text("/proc/mounts").splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        device, point, kind = parts[0], parts[1], parts[2]
        # Only real filesystems. The rest are kernel bookkeeping and reporting them as "storage"
        # would bury the one mount anybody cares about.
        if kind in ("proc", "sysfs", "devtmpfs", "devpts", "cgroup", "cgroup2", "overlay",
                    "tmpfs", "squashfs", "debugfs", "tracefs", "securityfs", "pstore",
                    "bpf", "configfs", "fusectl", "mqueue", "autofs", "ramfs"):
            continue
        if point in seen:
            continue
        seen.add(point)
        try:
            stat = os.statvfs(point)
        except OSError:
            continue
        total = stat.f_blocks * stat.f_frsize
        free = stat.f_bavail * stat.f_frsize
        if total == 0:
            continue
        mounts.append({
            "mount": point,
            "device": device,
            "filesystem": kind,
            "totalBytes": total,
            "usedBytes": total - free,
            "freeBytes": free,
        })

    return {"mounts": mounts, "directories": _directory_sizes()}


def _directory_sizes():
    """What is actually taking the space.

    `du` with a one-second budget per directory. If it does not finish, the entry is reported without
    a size rather than making the caller wait - a slow answer to a diagnostic question is worse than
    an incomplete one when a match is about to start.
    """
    out = []
    for path in DATA_DIRS:
        if not os.path.isdir(path):
            continue
        text = run(["du", "-sb", path], timeout=1)
        size = None
        if text:
            head = text.split()
            if head and head[0].isdigit():
                size = int(head[0])
        out.append({"path": path, "bytes": size})
    return out


# --------------------------------------------------------------------------- processes

def _sysconf(name, default):
    try:
        return os.sysconf(name) or default
    except (AttributeError, OSError, ValueError):
        return default


def processes():
    """The heaviest processes, by CPU over a sample and by resident memory.

    This is the half of "the processor is pinned" that the NetworkTables figure cannot answer. It is
    also how a leak in robot code becomes visible: the same process, growing, match after match.
    """
    # sysconf is POSIX-only. The agent only runs on Systemcore, but it is developed off-target and a
    # diagnostic tool nobody can run before shipping is one nobody runs.
    clock = _sysconf("SC_CLK_TCK", 100)
    page = _sysconf("SC_PAGE_SIZE", 4096)

    def sample():
        out = {}
        try:
            pids = os.listdir("/proc")
        except OSError:
            # No procfs. Reporting no processes is honest; failing the whole response is not.
            return out
        for pid in pids:
            if not pid.isdigit():
                continue
            stat = read_text(f"/proc/{pid}/stat")
            if not stat:
                continue
            # The command name is in parentheses and may itself contain spaces, so split around it
            # rather than on whitespace.
            open_paren = stat.find("(")
            close_paren = stat.rfind(")")
            if open_paren < 0 or close_paren < 0:
                continue
            name = stat[open_paren + 1:close_paren]
            fields = stat[close_paren + 2:].split()
            if len(fields) < 22:
                continue
            try:
                utime, stime = int(fields[11]), int(fields[12])
                rss_pages = int(fields[21])
            except ValueError:
                continue
            out[pid] = {"name": name, "jiffies": utime + stime, "rssBytes": rss_pages * page}
        return out

    first = sample()
    time.sleep(CPU_SAMPLE_SECONDS)
    second = sample()

    rows = []
    for pid, now in second.items():
        before = first.get(pid)
        used = (now["jiffies"] - before["jiffies"]) if before else 0
        percent = round(100.0 * used / clock / CPU_SAMPLE_SECONDS, 1)
        rows.append({
            "pid": int(pid),
            "name": now["name"],
            "cpuPercent": percent,
            "rssBytes": now["rssBytes"],
        })

    by_cpu = sorted(rows, key=lambda r: r["cpuPercent"], reverse=True)[:TOP_PROCESSES]
    by_memory = sorted(rows, key=lambda r: r["rssBytes"], reverse=True)[:TOP_PROCESSES]
    return {"count": len(rows), "topByCpu": by_cpu, "topByMemory": by_memory}


# --------------------------------------------------------------------------- can

def can_interfaces():
    """Frame-level CAN counters straight off the interfaces.

    Utilisation says how busy a bus is. These say whether it is healthy, which is a different
    question and the one that matters when a motor drops out on a bus reading a comfortable 30%:
    error frames, restarts, and the bus-off state do not move utilisation at all.
    """
    out = []
    base = "/sys/class/net"
    try:
        names = sorted(n for n in os.listdir(base) if n.startswith("can"))
    except OSError:
        return out

    for name in names:
        stats = f"{base}/{name}/statistics"
        state = read_text(f"{base}/{name}/operstate").strip() or None

        # Bitrate and error counters come from `ip`, which reports the CAN-specific fields the
        # generic statistics directory does not carry.
        detail = run(["ip", "-details", "-statistics", "link", "show", name])
        bitrate = None
        can_state = None
        restarts = None
        match = re.search(r"bitrate (\d+)", detail)
        if match:
            bitrate = int(match.group(1))
        match = re.search(r"state (\S+)", detail)
        if match:
            can_state = match.group(1)
        match = re.search(r"restarts (\d+)", detail)
        if match:
            restarts = int(match.group(1))

        out.append({
            "name": name,
            "up": state == "up",
            "state": can_state,
            "bitrate": bitrate,
            "restarts": restarts,
            "rxPackets": read_int(f"{stats}/rx_packets"),
            "txPackets": read_int(f"{stats}/tx_packets"),
            "rxErrors": read_int(f"{stats}/rx_errors"),
            "txErrors": read_int(f"{stats}/tx_errors"),
            "rxDropped": read_int(f"{stats}/rx_dropped"),
            "txDropped": read_int(f"{stats}/tx_dropped"),
        })
    return out


# --------------------------------------------------------------------------- network

def network():
    out = []
    base = "/sys/class/net"
    try:
        names = sorted(n for n in os.listdir(base) if n != "lo" and not n.startswith("can"))
    except OSError:
        return out

    addresses = _addresses()
    wireless = _wireless()

    for name in names:
        out.append({
            "name": name,
            "up": read_text(f"{base}/{name}/operstate").strip() == "up",
            "mac": read_text(f"{base}/{name}/address").strip() or None,
            "speedMbps": read_int(f"{base}/{name}/speed"),
            "addresses": addresses.get(name, []),
            "wireless": wireless.get(name),
        })
    return out


def _addresses():
    """IPv4 addresses per interface. The pit's first question when a robot will not connect."""
    out = {}
    for line in run(["ip", "-4", "-oneline", "addr", "show"]).splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[2] == "inet":
            out.setdefault(parts[1], []).append(parts[3])
    return out


def _wireless():
    """Link quality, which is the difference between "connected" and "connected on the field"."""
    out = {}
    lines = read_text("/proc/net/wireless").splitlines()[2:]
    for line in lines:
        parts = line.split()
        if len(parts) < 4:
            continue
        name = parts[0].rstrip(":")
        try:
            out[name] = {
                "linkQuality": float(parts[2].rstrip(".")),
                "signalDbm": float(parts[3].rstrip(".")),
            }
        except ValueError:
            continue
    return out


# --------------------------------------------------------------------------- robot program

# The unit the robot program runs as. Read off the OS image rather than assumed.
ROBOT_UNIT = "robot.service"


def robot_program():
    """How the robot program itself is doing, which NetworkTables cannot report.

    Notably it can report this after a crash, when the robot program is not there to report anything
    about itself - which is exactly the moment somebody wants to know what it said.
    """
    show = run(["systemctl", "show", ROBOT_UNIT,
                "--property=ActiveState,SubState,NRestarts,ExecMainStartTimestampMonotonic,"
                "MemoryCurrent,MainPID"])
    props = {}
    for line in show.splitlines():
        key, _, value = line.partition("=")
        props[key] = value

    def as_int(key):
        try:
            return int(props.get(key, ""))
        except ValueError:
            return None

    started_monotonic = as_int("ExecMainStartTimestampMonotonic")
    running_for = None
    if started_monotonic:
        uptime_raw = read_text("/proc/uptime").split()
        if uptime_raw:
            running_for = round(float(uptime_raw[0]) - started_monotonic / 1e6, 1)

    memory_current = as_int("MemoryCurrent")
    return {
        "unit": ROBOT_UNIT,
        "state": props.get("ActiveState") or None,
        "subState": props.get("SubState") or None,
        # Restarts are the signal people miss. A robot program that crashes and comes straight back
        # looks fine from the driver's station, and looks like this from here.
        "restarts": as_int("NRestarts"),
        "runningForSeconds": running_for if running_for and running_for >= 0 else None,
        "memoryBytes": memory_current if memory_current and memory_current > 0 else None,
        "pid": as_int("MainPID") or None,
        "log": _robot_log(),
    }


def _robot_log():
    """The tail of the robot program's log, newest last."""
    out = run(["journalctl", "-u", ROBOT_UNIT, "-n", str(LOG_LINES),
               "--no-pager", "--output=short-iso"], timeout=4)
    lines = [line for line in out.splitlines() if line.strip()]
    return lines[-LOG_LINES:]


# --------------------------------------------------------------------------- cameras

# Systemcore's vision aggregator discovers every Limelight on the robot network, gives each one an
# alias address on every interface the Driver Station might arrive by, and knows whether the camera
# holds a NetworkTables session with the robot program. It does not know how hot the camera is.
# The camera does: its REST API on 5807 answers /status with temperature, CPU, frame rate and the
# pipeline it is running. This joins the two, so one poll from the Console answers "is every
# camera up, is every camera talking to the robot, and is any of them about to throttle".
AGGREGATOR_URL = "http://127.0.0.1:4810/api/cameras"
CAMERA_STATUS_PORT = 5807
AGGREGATOR_TIMEOUT = 0.8
CAMERA_STATUS_TIMEOUT = 0.6

# The Console polls every three seconds and four cameras cost up to four round trips, so the answer
# is held for two. Long enough that two dashboards do not double the traffic; short enough that a
# camera dropping out is visible before anyone reaches for it.
CAMERA_CACHE_SECONDS = 2.0
_camera_cache = {"at": 0.0, "value": None}


def _http_json(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8", "replace"))


def camera_status(ip, timeout=CAMERA_STATUS_TIMEOUT):
    """One camera's own account of itself, or None if it did not answer in time."""
    try:
        status = _http_json("http://%s:%d/status" % (ip, CAMERA_STATUS_PORT), timeout)
        return status if isinstance(status, dict) else None
    except Exception:                       # noqa: BLE001 - one silent camera must not hide the rest
        return None


def merge_cameras(aggregated, statuses):
    """Join the OS's camera list with each camera's status, by IP. Pure, so it is testable."""
    cameras = aggregated.get("cameras", []) if isinstance(aggregated, dict) else []
    out = []
    for cam in cameras:
        if not isinstance(cam, dict):
            continue
        ip = cam.get("ip")
        st = statuses.get(ip) or {}
        aliases = cam.get("aliasIps") or ([cam["aliasIp"]] if cam.get("aliasIp") else [])
        out.append({
            "name": st.get("name") or cam.get("host") or cam.get("name"),
            "host": cam.get("host"),
            "ip": ip,
            "type": cam.get("type"),
            "interface": cam.get("interface"),
            "ntConnected": bool(cam.get("ntConnected")),
            "ntName": cam.get("ntName") or None,
            "aliasIps": list(aliases),
            "uiUrl": cam.get("uiUrl"),
            "streamUrl": cam.get("mjpegUrl"),
            "fps": st.get("fps", cam.get("fps")),
            "temperatureC": st.get("temp"),
            "cpuPercent": st.get("cpu"),
            "ramPercent": st.get("ram"),
            "pipelineType": st.get("pipelineType") or cam.get("pipelineType"),
            "pipelineIndex": st.get("pipelineIndex"),
            "statusReachable": bool(st),
        })
    out.sort(key=lambda c: (c["name"] or "", c["ip"] or ""))
    return out


def cameras():
    """Every Limelight the OS can see, with what each says about itself. Cached briefly."""
    now = time.time()
    cached = _camera_cache["value"]
    if cached is not None and now - _camera_cache["at"] < CAMERA_CACHE_SECONDS:
        return cached
    try:
        aggregated = _http_json(AGGREGATOR_URL, AGGREGATOR_TIMEOUT)
    except Exception as exc:                # noqa: BLE001 - reported, not raised
        value = {"available": False, "cameras": [],
                 "reason": "vision aggregator did not answer (%s)" % exc.__class__.__name__,
                 "sampledAt": now}
        _camera_cache.update(at=now, value=value)
        return value

    ips = []
    for cam in aggregated.get("cameras", []) if isinstance(aggregated, dict) else []:
        if isinstance(cam, dict) and cam.get("ip"):
            ips.append(cam["ip"])
    statuses = {}
    if ips:
        # In parallel, because they are independent machines and four sequential 0.6 s timeouts
        # would be a 2.4 s stall inside a poll that expects to take milliseconds.
        with concurrent.futures.ThreadPoolExecutor(max_workers=min(8, len(ips))) as pool:
            for ip, status in zip(ips, pool.map(camera_status, ips)):
                if status:
                    statuses[ip] = status
    value = {"available": True, "cameras": merge_cameras(aggregated, statuses), "sampledAt": now}
    _camera_cache.update(at=now, value=value)
    return value


# --------------------------------------------------------------------------- assembly

# --------------------------------------------------------------------------- motor history

# The robot program (FrcCatalyst's MotorHistory) keeps what every motor has been through, by serial
# number, in one JSON file beside itself. It is the record - loaded at boot, never re-derived - so
# the right thing for this agent to do is hand the file over exactly as it is, plus a CSV of the
# totals for a spreadsheet. The Console shows a summary of it from the snapshot; the App fetches
# the whole thing and saves it wherever the team keeps such things.
MOTOR_HISTORY_PATH = os.environ.get("CATALYST_MOTOR_HISTORY", "/home/systemcore/catalyst/motor-history.json")
MOTOR_HISTORY_COLUMNS = ("serial", "model", "kind", "bus", "id", "name", "firmware", "poweredSeconds",
                         "runningSeconds", "loadedSeconds", "revolutions", "peakStatorAmps", "peakTempC",
                         "hotSeconds", "energyJoules", "boots", "firstSeenMs", "lastSeenMs", "identities",
                         "stickyFaults")


def motor_history(path=None):
    """The file as it is on disk, with where it came from. A missing file is an answer, not a fault."""
    path = path or MOTOR_HISTORY_PATH
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    except FileNotFoundError:
        return {"error": "no motor history yet - the robot program writes it once it has seen a motor",
                "path": path, "devices": []}
    except (OSError, ValueError) as exc:
        return {"error": str(exc), "path": path, "devices": []}
    if not isinstance(doc, dict):
        return {"error": "not a motor history file", "path": path, "devices": []}
    doc["path"] = path
    try:
        doc["fileModifiedAt"] = os.path.getmtime(path)
    except OSError:
        pass
    return doc


def _device_row(d):
    """One device flattened to the CSV columns: the latest identity and the totals."""
    identities = d.get("identities") or []
    latest = identities[-1] if identities else {}
    totals = d.get("totals") or {}
    return {
        "serial": d.get("serial", ""), "model": d.get("model", ""), "kind": d.get("kind", ""),
        "bus": latest.get("bus", ""), "id": latest.get("id", ""), "name": latest.get("name", ""),
        "firmware": latest.get("firmware", ""),
        "poweredSeconds": totals.get("poweredSeconds", 0), "runningSeconds": totals.get("runningSeconds", 0),
        "loadedSeconds": totals.get("loadedSeconds", 0), "revolutions": totals.get("revolutions", 0),
        "peakStatorAmps": totals.get("peakStatorAmps", 0), "peakTempC": totals.get("peakTempC", 0),
        "hotSeconds": totals.get("hotSeconds", 0), "energyJoules": totals.get("energyJoules", 0),
        "boots": d.get("boots", 0), "firstSeenMs": d.get("firstSeenMs", 0), "lastSeenMs": d.get("lastSeenMs", 0),
        "identities": len(identities), "stickyFaults": totals.get("stickyFaults", 0),
    }


def motor_history_csv(doc):
    """One row per device, the totals only. Commas and quotes in names are quoted the RFC way."""
    def cell(v):
        s = "" if v is None else str(v)
        return '"' + s.replace('"', '""') + '"' if any(c in s for c in ',"\n') else s
    lines = [",".join(MOTOR_HISTORY_COLUMNS)]
    for d in doc.get("devices") or []:
        row = _device_row(d)
        lines.append(",".join(cell(row[c]) for c in MOTOR_HISTORY_COLUMNS))
    return "\n".join(lines) + "\n"


_history_cache = {"mtime": None, "summary": None}


def motor_history_summary():
    """What the snapshot carries: the totals per device, re-read only when the file changes."""
    try:
        mtime = os.path.getmtime(MOTOR_HISTORY_PATH)
    except OSError:
        return {"devices": [], "present": False}
    if _history_cache["mtime"] == mtime and _history_cache["summary"] is not None:
        return _history_cache["summary"]
    doc = motor_history()
    summary = {
        "present": "error" not in doc,
        "updatedMs": doc.get("updatedMs"),
        "clockTrusted": doc.get("clockTrusted"),
        "devices": [_device_row(d) for d in doc.get("devices") or []],
    }
    _history_cache["mtime"] = mtime
    _history_cache["summary"] = summary
    return summary


def snapshot():
    return {
        "identity": identity(),
        "cpu": cpu(),
        "thermal": thermal(),
        "memory": memory(),
        "storage": storage(),
        "processes": processes(),
        "can": can_interfaces(),
        "network": network(),
        "robotProgram": robot_program(),
        "cameras": cameras(),
        "motorHistory": motor_history_summary(),
        "sampledAt": time.time(),
    }


class Handler(http.server.BaseHTTPRequestHandler):
    # Every response is JSON and every route is a GET. There is no POST, PUT or DELETE anywhere in
    # this file, and there should not be - see the module docstring.
    protocol_version = "HTTP/1.1"

    def _send_text(self, text, content_type, status=200, filename=None):
        body = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        if filename:
            self.send_header("Content-Disposition", 'attachment; filename="%s"' % filename)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send(self, payload, status=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        # The Console is served from somewhere else, so it needs this to read the response at all.
        # Safe because the agent has nothing to protect: it exposes no secrets and accepts no
        # commands, and anyone who can reach this port is already on the robot's network.
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route = self.path.split("?")[0].rstrip("/") or "/"
        try:
            if route in ("/", "/api", "/api/system"):
                self._send(snapshot())
            elif route == "/api/health":
                # A cheap route the Console can use to find out whether the agent is here at all,
                # without paying for a full sample.
                self._send({"ok": True, "agent": "catalyst-agent", "version": "2.0.3"})
            elif route == "/api/robot":
                self._send(robot_program())
            elif route == "/api/cameras":
                self._send(cameras())
            elif route == "/api/motor-history":
                doc = motor_history()
                self._send(doc, status=404 if "error" in doc else 200)
            elif route == "/api/motor-history.csv":
                doc = motor_history()
                self._send_text(motor_history_csv(doc), "text/csv; charset=utf-8",
                                status=404 if "error" in doc else 200, filename="motor-history.csv")
            else:
                self._send({"error": "not found", "path": route}, status=404)
        except Exception as exc:            # noqa: BLE001 - a diagnostic tool must not die diagnosing
            self._send({"error": str(exc)}, status=500)

    def log_message(self, fmt, *args):
        # journald already stamps and routes these, and a poll every second would otherwise fill the
        # log with the fact that it was polled.
        pass


class Server(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    server = Server(("0.0.0.0", PORT), Handler)

    def stop(*_):
        server.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    server.serve_forever()


if __name__ == "__main__":
    main()
