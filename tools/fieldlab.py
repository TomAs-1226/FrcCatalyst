#!/usr/bin/env python3
"""Turn a Field Lab campaign's .wpilog into a report and a set of recommended constants.

    python tools/fieldlab.py report run.wpilog                  the report, on stdout
    python tools/fieldlab.py report run.wpilog -o report.md     and to a file
    python tools/fieldlab.py json run.wpilog                    just the numbers, as JSON
    python tools/fieldlab.py diff run.wpilog --constants c.json compare against what the robot believes
    python tools/fieldlab.py channels run.wpilog                every channel in the log, with counts

Why this is offline
-------------------

`CampaignRunner` fits nothing on the robot. It records, and this fits. Two reasons, and the second is
the one that matters:

  * A robot has twenty milliseconds a loop and no business running a least-squares.
  * A fit you can re-run against the same log with a different model is worth more than a number the
    robot computed once and threw the data away for. Every time a fit here turns out to be wrong -
    the wrong segment, the wrong window, an outlier nobody saw - the log is still there.

So the robot's job is to collect honestly and the analysis happens here, at the shop, as many times as
it takes.

What it will and will not tell you
----------------------------------

It fits what is unambiguous and refuses to guess at the rest:

  * **Feedforward (kS, kV, kA)** from the SysId channels, by least squares, quasistatic and dynamic
    segments separately. Reported with the residual, because a kV with a large residual is not a
    measurement, it is a line drawn through a mess.
  * **Wheel radius** - reads what `WheelRadiusCalibration` concluded and the caliper number a person
    typed, and says whether they agree.
  * **Slip current** - reads what `SlipCurrentCalibration` concluded.
  * **Odometry drift** - the operator's lap measurements, as a mean and a spread, and as a percentage
    of the distance driven when the log says how far that was.
  * **A sweep** (a shot map, say) - pairs each commanded setpoint with the score a person gave it and
    prints the table, plus the best setpoint per distance.
  * **Thermal soak** - the peak temperature and current per channel.

It does NOT invent a model for anything else. An unrecognised procedure gets its measurements and its
notes printed and nothing more, which is the honest output.

Standard library only, Python 3.8+. Reads the log; writes only what you ask for.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from pathlib import Path

# --------------------------------------------------------------------------- the .wpilog reader
#
# WPILib's DataLog format. There is no reader for it anywhere in this repository - Catalyst Console
# has one in Rust and the tablet has one in C, and neither is reachable from here - so this is a
# third. It is about eighty lines, which is cheaper than a dependency.

_HEADER_MAGIC = b"WPILOG"


class DataLogError(Exception):
    """The file is not a readable .wpilog."""


def _read_str(buf: bytes, pos: int) -> "tuple[str, int]":
    """A length-prefixed UTF-8 string inside a control record."""
    if pos + 4 > len(buf):
        raise DataLogError("string length runs past the end of a control record")
    (n,) = struct.unpack_from("<I", buf, pos)
    pos += 4
    if pos + n > len(buf):
        raise DataLogError("string runs past the end of a control record")
    return buf[pos:pos + n].decode("utf-8", "replace"), pos + n


def read_wpilog(path: Path):
    """Yield ``(name, type, timestamp_seconds, value)`` for every data record, in file order.

    Records for an entry that was never started are skipped rather than raising: a log truncated by a
    power cut - which is exactly how a field-day log ends - has a valid beginning and a torn end, and
    the beginning is the part with the data in it.
    """
    raw = path.read_bytes()
    if not raw.startswith(_HEADER_MAGIC):
        raise DataLogError(f"{path} does not start with WPILOG; is it a .dslog?")

    pos = len(_HEADER_MAGIC)
    (version,) = struct.unpack_from("<H", raw, pos)
    pos += 2
    (extra_len,) = struct.unpack_from("<I", raw, pos)
    pos += 4
    pos += extra_len

    entries = {}  # entry id -> (name, type)

    while pos < len(raw):
        try:
            bitfield = raw[pos]
            pos += 1
            id_len = (bitfield & 0x03) + 1
            size_len = ((bitfield >> 2) & 0x03) + 1
            ts_len = ((bitfield >> 4) & 0x07) + 1

            need = id_len + size_len + ts_len
            if pos + need > len(raw):
                break  # torn tail

            entry_id = int.from_bytes(raw[pos:pos + id_len], "little")
            pos += id_len
            payload_size = int.from_bytes(raw[pos:pos + size_len], "little")
            pos += size_len
            timestamp_us = int.from_bytes(raw[pos:pos + ts_len], "little")
            pos += ts_len

            if pos + payload_size > len(raw):
                break  # torn tail
            payload = raw[pos:pos + payload_size]
            pos += payload_size
        except (IndexError, struct.error):
            break

        if entry_id == 0:
            _apply_control(payload, entries)
            continue

        known = entries.get(entry_id)
        if known is None:
            continue
        name, type_str = known
        value = _decode(type_str, payload)
        if value is not _UNDECODABLE:
            yield name, type_str, timestamp_us / 1e6, value


class _Undecodable:
    def __repr__(self):
        return "<undecodable>"


_UNDECODABLE = _Undecodable()


def _apply_control(payload: bytes, entries: dict) -> None:
    if not payload:
        return
    kind = payload[0]
    pos = 1
    try:
        if kind == 0:  # Start
            (entry_id,) = struct.unpack_from("<I", payload, pos)
            pos += 4
            name, pos = _read_str(payload, pos)
            type_str, pos = _read_str(payload, pos)
            entries[entry_id] = (name, type_str)
        elif kind == 1:  # Finish
            (entry_id,) = struct.unpack_from("<I", payload, pos)
            entries.pop(entry_id, None)
        # kind == 2 is SetMetadata; nothing here needs metadata.
    except (struct.error, DataLogError):
        return


def _decode(type_str: str, payload: bytes):
    try:
        if type_str == "double":
            return struct.unpack("<d", payload)[0]
        if type_str == "float":
            return struct.unpack("<f", payload)[0]
        if type_str in ("int64", "int"):
            return int.from_bytes(payload, "little", signed=True)
        if type_str == "boolean":
            return bool(payload[0]) if payload else False
        if type_str in ("string", "json"):
            return payload.decode("utf-8", "replace")
        if type_str == "double[]":
            return list(struct.unpack("<%dd" % (len(payload) // 8), payload[:len(payload) // 8 * 8]))
        if type_str == "float[]":
            return list(struct.unpack("<%df" % (len(payload) // 4), payload[:len(payload) // 4 * 4]))
        if type_str == "int64[]":
            return [int.from_bytes(payload[i:i + 8], "little", signed=True)
                    for i in range(0, len(payload) - 7, 8)]
        if type_str == "boolean[]":
            return [bool(b) for b in payload]
        if type_str == "string[]":
            if len(payload) < 4:
                return []
            (n,) = struct.unpack_from("<I", payload, 0)
            pos = 4
            out = []
            for _ in range(n):
                s, pos = _read_str(payload, pos)
                out.append(s)
            return out
        # A struct topic decodes as a run of little-endian doubles, which is how both Catalyst
        # dashboards read them; the widths are the struct's business, not this reader's.
        if type_str.startswith("struct:"):
            count = len(payload) // 8
            return list(struct.unpack("<%dd" % count, payload[:count * 8])) if count else []
    except (struct.error, DataLogError, IndexError):
        return _UNDECODABLE
    return _UNDECODABLE


# --------------------------------------------------------------------------- the campaign

FIELDLAB = "/FieldLab/"


class Campaign:
    """Everything a Field Lab campaign left in the log."""

    def __init__(self):
        self.session = ""
        self.name = ""
        self.measurements = {}   # "procedure/key" -> value
        self.units = {}          # "procedure/key" -> unit
        self.notes = []
        self.states = []         # (t, state)
        self.channels = {}       # full channel name -> list of (t, value)
        self.started = {}        # procedure id -> t
        self.finished = {}       # procedure id -> t

    @property
    def procedures(self):
        return sorted(set(self.started) | set(self.finished))

    def duration(self):
        if not self.states:
            return 0.0
        return self.states[-1][0] - self.states[0][0]


def load(path: Path, keep_all: bool = False) -> Campaign:
    """Read a log into a :class:`Campaign`.

    ``keep_all`` keeps every numeric channel, not just the Field Lab ones. The report needs a few
    non-Field Lab channels (the calibrations publish their own conclusions, and SysId writes its own),
    so the default keeps anything numeric and drops only the high-rate noise this never looks at.
    """
    c = Campaign()
    for name, type_str, t, value in read_wpilog(path):
        short = name
        for prefix in ("/Catalyst", "NT:/Catalyst", "NT:"):
            if short.startswith(prefix):
                short = short[len(prefix):]
                break

        if short.startswith(FIELDLAB) or short.startswith("FieldLab/"):
            _fieldlab_channel(c, short, value)

        if isinstance(value, (int, float)) and not isinstance(value, bool):
            c.channels.setdefault(short, []).append((t, float(value)))
        elif keep_all:
            c.channels.setdefault(short, []).append((t, value))
    return c


def _fieldlab_channel(c: Campaign, short: str, value) -> None:
    tail = short.split("FieldLab/", 1)[-1]

    if tail == "Session" and isinstance(value, str) and value:
        c.session = value
    elif tail == "Campaign" and isinstance(value, str) and value:
        c.name = value
    elif tail == "Notes" and isinstance(value, list):
        for n in value:
            if n not in c.notes:
                c.notes.append(n)
    elif tail == "State" and isinstance(value, str):
        c.states.append((0.0, value))
    elif tail.endswith("/Started"):
        c.started[tail[: -len("/Started")]] = value
    elif tail.endswith("/Finished"):
        c.finished[tail[: -len("/Finished")]] = value
    elif tail.endswith(".unit") and isinstance(value, str):
        c.units[_strip_session(tail[: -len(".unit")], c.session)] = value
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        key = _strip_session(tail, c.session)
        if key.count("/") == 1 and not key.endswith(("Index", "Count", "Seconds", "Progress", "Volts")):
            c.measurements[key] = float(value)


def _strip_session(key: str, session: str) -> str:
    """``<session>/<procedure>/<step>`` -> ``<procedure>/<step>``."""
    if session and key.startswith(session + "/"):
        return key[len(session) + 1:]
    parts = key.split("/")
    # A session id is eight hex digits; drop a leading one even if the Session channel came later in
    # the log than the measurement did, which happens when a campaign resumes.
    if len(parts) == 3 and len(parts[0]) == 8 and all(ch in "0123456789abcdef" for ch in parts[0]):
        return "/".join(parts[1:])
    return key


# --------------------------------------------------------------------------- fits


def _lstsq(rows, targets):
    """Least squares by normal equations. Returns (coefficients, rms_residual) or None.

    Normal equations rather than a QR: these systems are two or three columns of well-scaled data,
    and pulling in numpy for a 3x3 solve would make this script need an install.
    """
    n_cols = len(rows[0]) if rows else 0
    if len(rows) <= n_cols or n_cols == 0:
        return None

    ata = [[0.0] * n_cols for _ in range(n_cols)]
    atb = [0.0] * n_cols
    for row, y in zip(rows, targets):
        for i in range(n_cols):
            atb[i] += row[i] * y
            for j in range(n_cols):
                ata[i][j] += row[i] * row[j]

    # Gauss-Jordan with partial pivoting.
    aug = [ata[i][:] + [atb[i]] for i in range(n_cols)]
    for col in range(n_cols):
        pivot = max(range(col, n_cols), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-12:
            return None
        aug[col], aug[pivot] = aug[pivot], aug[col]
        div = aug[col][col]
        aug[col] = [v / div for v in aug[col]]
        for r in range(n_cols):
            if r != col and aug[r][col]:
                factor = aug[r][col]
                aug[r] = [a - factor * b for a, b in zip(aug[r], aug[col])]
    coeffs = [aug[i][n_cols] for i in range(n_cols)]

    total = 0.0
    for row, y in zip(rows, targets):
        predicted = sum(c * v for c, v in zip(coeffs, row))
        total += (predicted - y) ** 2
    return coeffs, math.sqrt(total / len(rows))


def fit_feedforward(c: Campaign):
    """kS, kV and kA from the SysId channels, if they are in the log.

    Model: ``V = kS*sign(v) + kV*v + kA*a``. Acceleration is differentiated from velocity, so kA is the
    weakest of the three and is reported as such.
    """
    volts = _find_channel(c, ("sysid", "voltage"), ("Drive", "AppliedVolts"), ("Swerve", "Volts"))
    vel = _find_channel(c, ("sysid", "velocity"), ("Swerve", "SpeedMPS"), ("Drive", "Velocity"))
    if not volts or not vel:
        return None

    samples = _align(volts, vel)
    if len(samples) < 20:
        return None

    rows, targets = [], []
    for i in range(1, len(samples) - 1):
        t, v_volts, v_vel = samples[i]
        dt = samples[i + 1][0] - samples[i - 1][0]
        if dt <= 0:
            continue
        accel = (samples[i + 1][2] - samples[i - 1][2]) / dt
        if abs(v_vel) < 0.02:
            continue  # kS is undefined at rest and a sign() there is noise
        rows.append([1.0 if v_vel > 0 else -1.0, v_vel, accel])
        targets.append(v_volts)

    fit = _lstsq(rows, targets)
    if not fit:
        return None
    (ks, kv, ka), residual = fit
    return {
        "kS": ks, "kV": kv, "kA": ka,
        "residual_volts_rms": residual,
        "samples": len(rows),
        "confidence": "good" if residual < 0.25 else "poor - a line through a mess, not a measurement",
    }


def _find_channel(c: Campaign, *needle_sets):
    for needles in needle_sets:
        for name, series in c.channels.items():
            lowered = name.lower()
            if all(n.lower() in lowered for n in needles) and len(series) > 10:
                return series
    return None


def _align(a, b):
    """Pair two series on nearest timestamp. Both are logged on the same 20 ms loop, so this is a
    zip with a tolerance rather than an interpolation."""
    out = []
    j = 0
    for t, va in a:
        while j + 1 < len(b) and abs(b[j + 1][0] - t) <= abs(b[j][0] - t):
            j += 1
        if j < len(b) and abs(b[j][0] - t) < 0.03:
            out.append((t, va, b[j][1]))
    return out


def summarise_drift(c: Campaign):
    laps = sorted((k, v) for k, v in c.measurements.items()
                  if k.startswith("odometry-drift/") and "error" in k)
    if not laps:
        return None
    values = [v for _, v in laps]
    mean = sum(values) / len(values)
    spread = max(values) - min(values)
    return {
        "laps": {k.split("/")[-1]: v for k, v in laps},
        "mean_error_m": mean,
        "spread_m": spread,
        "reading": ("consistent - looks like a systematic scale error, so check the wheel radius"
                    if spread < 0.3 * max(mean, 1e-9)
                    else "scattered - looks like slip rather than a scale error"),
    }


def summarise_sweep(c: Campaign, procedure: str):
    scores = {k.split("/")[-1]: v for k, v in c.measurements.items()
              if k.startswith(procedure + "/") and k.split("/")[-1].startswith("at-")}
    if not scores:
        return None
    at = c.channels.get("/FieldLab/%s/At" % procedure) or c.channels.get("FieldLab/%s/At" % procedure)
    first = c.channels.get("/FieldLab/%s/First" % procedure) or c.channels.get("FieldLab/%s/First" % procedure)
    second = c.channels.get("/FieldLab/%s/Second" % procedure) or c.channels.get("FieldLab/%s/Second" % procedure)

    rows = []
    for label, score in sorted(scores.items()):
        rows.append({"setpoint": label.replace("at-", ""), "score": score})
    commanded = []
    if at and first and second:
        for (t, a), (_, f), (_, s) in zip(at, first, second):
            commanded.append({"at": a, "first": f, "second": s, "t": t})
    return {"rows": rows, "commanded": commanded}


def summarise_thermal(c: Campaign):
    out = {}
    for name, series in c.channels.items():
        low = name.lower()
        if "temperaturec" in low or low.endswith("/tempc"):
            out.setdefault("temperature_c", {})[name] = max(v for _, v in series)
        elif "statorcurrent" in low or low.endswith("/currentamps"):
            out.setdefault("current_a", {})[name] = max(v for _, v in series)
    return out or None


def analyse(c: Campaign) -> dict:
    result = {
        "session": c.session,
        "campaign": c.name,
        "procedures": c.procedures,
        "measurements": c.measurements,
        "units": c.units,
        "notes": c.notes,
        "channels": len(c.channels),
    }

    ff = fit_feedforward(c)
    if ff:
        result["feedforward"] = ff

    drift = summarise_drift(c)
    if drift:
        result["odometry_drift"] = drift

    thermal = summarise_thermal(c)
    if thermal:
        result["thermal"] = thermal

    for key, label in (("Calibration/WheelRadius/CorrectedRadiusInches", "wheel_radius_in"),
                       ("Calibration/WheelRadius/PercentChange", "wheel_radius_percent_change"),
                       ("Calibration/SlipCurrent/BreakAmps", "slip_current_a")):
        series = c.channels.get("/" + key) or c.channels.get(key)
        if series:
            result.setdefault("calibrations", {})[label] = series[-1][1]

    caliper = c.measurements.get("wheel-radius/caliper-diameter-in")
    radius = result.get("calibrations", {}).get("wheel_radius_in")
    if caliper and radius:
        result["wheel_radius_cross_check"] = {
            "caliper_radius_in": caliper / 2.0,
            "calibrated_radius_in": radius,
            "disagreement_in": abs(caliper / 2.0 - radius),
            "reading": ("they agree" if abs(caliper / 2.0 - radius) < 0.05
                        else "they disagree - the drive-base radius the routine was told is the usual cause"),
        }

    for procedure in c.procedures:
        sweep = summarise_sweep(c, procedure)
        if sweep:
            result.setdefault("sweeps", {})[procedure] = sweep

    return result


# --------------------------------------------------------------------------- output


def report(c: Campaign, a: dict) -> str:
    L = []
    L.append("# Field Lab report")
    L.append("")
    L.append("| | |")
    L.append("|---|---|")
    L.append("| Session | `%s` |" % (a["session"] or "unknown"))
    L.append("| Campaign | %s |" % (a["campaign"] or "unknown"))
    L.append("| Procedures in the log | %d |" % len(a["procedures"]))
    L.append("| Measurements taken | %d |" % len(a["measurements"]))
    L.append("| Channels recorded | %d |" % a["channels"])
    L.append("")

    if not a["procedures"]:
        L.append("> No Field Lab procedures found in this log. Either the campaign never started, or")
        L.append("> this is not the log the campaign wrote - check that the robot had a `WpilogSink`")
        L.append("> installed and that you are reading the right file.")
        L.append("")

    if "feedforward" in a:
        ff = a["feedforward"]
        L.append("## Drive feedforward")
        L.append("")
        L.append("| Gain | Measured |")
        L.append("|---|---|")
        L.append("| kS | %.4f |" % ff["kS"])
        L.append("| kV | %.4f |" % ff["kV"])
        L.append("| kA | %.4f |" % ff["kA"])
        L.append("")
        L.append("Fitted from %d samples; residual %.3f V RMS — **%s**."
                 % (ff["samples"], ff["residual_volts_rms"], ff["confidence"]))
        L.append("")
        L.append("kA is differentiated from velocity, so it is the least trustworthy of the three.")
        L.append("Treat kS and kV as measurements and kA as a starting point.")
        L.append("")

    if "calibrations" in a:
        L.append("## Calibrations")
        L.append("")
        L.append("| What | Value |")
        L.append("|---|---|")
        for k, v in a["calibrations"].items():
            L.append("| %s | %.4f |" % (k.replace("_", " "), v))
        L.append("")

    if "wheel_radius_cross_check" in a:
        x = a["wheel_radius_cross_check"]
        L.append("**Wheel radius cross-check:** the calipers say %.3f in, the routine concluded %.3f in,"
                 " a difference of %.3f in — %s."
                 % (x["caliper_radius_in"], x["calibrated_radius_in"], x["disagreement_in"], x["reading"]))
        L.append("")

    if "odometry_drift" in a:
        d = a["odometry_drift"]
        L.append("## Odometry drift")
        L.append("")
        for lap, value in d["laps"].items():
            L.append("- %s: %.3f m" % (lap, value))
        L.append("")
        L.append("Mean %.3f m, spread %.3f m — **%s**." % (d["mean_error_m"], d["spread_m"], d["reading"]))
        L.append("")

    for procedure, sweep in a.get("sweeps", {}).items():
        L.append("## Sweep: %s" % procedure)
        L.append("")
        L.append("| Setpoint | Score |")
        L.append("|---|---|")
        for row in sweep["rows"]:
            L.append("| %s | %s |" % (row["setpoint"], row["score"]))
        L.append("")
        best = [r for r in sweep["rows"] if r["score"] >= 0.99]
        if best:
            L.append("Scored at: %s." % ", ".join(r["setpoint"] for r in best))
        else:
            L.append("Nothing scored a full 1.0. The best was %s."
                     % max(sweep["rows"], key=lambda r: r["score"])["setpoint"])
        L.append("")

    if "thermal" in a:
        L.append("## Peaks")
        L.append("")
        for group, values in a["thermal"].items():
            L.append("### %s" % group.replace("_", " "))
            L.append("")
            for name, value in sorted(values.items(), key=lambda kv: -kv[1])[:12]:
                L.append("- `%s` peaked at %.1f" % (name, value))
            L.append("")

    if a["measurements"]:
        L.append("## Every measurement")
        L.append("")
        L.append("| Key | Value | Unit |")
        L.append("|---|---|---|")
        for k in sorted(a["measurements"]):
            L.append("| `%s` | %s | %s |" % (k, a["measurements"][k], a["units"].get(k, "")))
        L.append("")

    if a["notes"]:
        L.append("## Notes the run recorded")
        L.append("")
        for n in a["notes"]:
            L.append("- %s" % n)
        L.append("")
        L.append("A note is the runner telling you something it could not fix: a gate that fired, a step")
        L.append("that hit its cap, a value outside its expected range. Read these before trusting a fit.")
        L.append("")

    return "\n".join(L)


def diff(a: dict, constants: dict) -> str:
    """What the robot believes, against what the field said."""
    L = ["# What to change", "", "| Constant | Robot believes | Field measured | Change |",
         "|---|---|---|---|"]
    measured = {}
    if "feedforward" in a:
        measured["kS"] = a["feedforward"]["kS"]
        measured["kV"] = a["feedforward"]["kV"]
        measured["kA"] = a["feedforward"]["kA"]
    for k, v in a.get("calibrations", {}).items():
        measured[k] = v

    any_row = False
    for name, believed in sorted(constants.items()):
        found = measured.get(name)
        if found is None:
            continue
        any_row = True
        if isinstance(believed, (int, float)) and believed:
            pct = (found - believed) / abs(believed) * 100.0
            change = "%+.1f%%" % pct
        else:
            change = "-"
        L.append("| %s | %s | %.4f | %s |" % (name, believed, found, change))

    if not any_row:
        L.append("| _nothing matched_ | | | |")
        L.append("")
        L.append("None of the constants you supplied share a name with anything measured. The keys this")
        L.append("recognises are kS, kV, kA and the calibration names printed by `json`.")
    return "\n".join(L)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    for name in ("report", "json", "channels"):
        p = sub.add_parser(name)
        p.add_argument("log", type=Path)
        p.add_argument("-o", "--out", type=Path)

    p = sub.add_parser("diff")
    p.add_argument("log", type=Path)
    p.add_argument("--constants", type=Path, required=True,
                   help="JSON of what the robot believes, e.g. {\"kV\": 0.124, \"slip_current_a\": 120}")
    p.add_argument("-o", "--out", type=Path)

    args = ap.parse_args(argv)

    # The report contains em dashes and a degree sign, and Windows' console defaults to cp1252, which
    # renders them as replacement characters or raises outright. The file output is already written as
    # UTF-8; this makes stdout agree with it.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass  # Python 3.6 or a stream that cannot be reconfigured; the file path still works.

    if not args.log.is_file():
        print("no such log: %s" % args.log, file=sys.stderr)
        return 2

    try:
        campaign = load(args.log)
    except DataLogError as e:
        print("could not read %s: %s" % (args.log, e), file=sys.stderr)
        return 2

    if args.cmd == "channels":
        lines = ["%6d  %s" % (len(series), name)
                 for name, series in sorted(campaign.channels.items(), key=lambda kv: -len(kv[1]))]
        text = "\n".join(lines)
    else:
        analysis = analyse(campaign)
        if args.cmd == "json":
            text = json.dumps(analysis, indent=2, sort_keys=True)
        elif args.cmd == "diff":
            text = diff(analysis, json.loads(args.constants.read_text(encoding="utf-8")))
        else:
            text = report(campaign, analysis)

    if getattr(args, "out", None):
        args.out.write_text(text + "\n", encoding="utf-8")
        print("wrote %s" % args.out)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
