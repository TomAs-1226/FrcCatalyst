# Catalyst Agent

An optional package that runs **on the Systemcore** and answers the questions NetworkTables cannot.

Systemcore publishes a summary of itself and Catalyst mirrors it under `/Catalyst/Systemcore/`, so
Console shows processor, memory, storage, temperature, power, flash wear and per-bus CAN with no
extra software. That covers most of what a pit crew needs.

What it does not cover is the detail behind a number that has gone the wrong way:

| The summary says | The agent answers |
|---|---|
| the processor is at 92% | which of the four cores, and which process |
| storage is at 94% | what is using it, per directory |
| CAN utilisation is 30% | whether the bus is dropping frames or restarting |
| *(nothing)* | how many times the robot program has restarted, and what it printed |
| *(nothing)* | which OS build this machine is on, and how long it has been up |
| a camera is "connected" | whether each Limelight is talking to the robot program, and how hot it is |

All of it is already in `/proc` and `/sys`. The agent serves it over HTTP so Console can ask.

## Motor history

`/api/motor-history` is the robot program's `catalyst/motor-history.json` exactly as it is on
disk - what every motor has been through, by serial number (see the library's *Motor history*
page). `/api/motor-history.csv` is one row per device with the totals, for a spreadsheet. The
`/api/system` snapshot carries a summary of it (`motorHistory`) that the Console shows. The path
can be moved with the `CATALYST_MOTOR_HISTORY` environment variable. Like everything here, it is
read-only: the robot program owns the file.

## Building and installing

```bash
cd agent
./build.sh                  # produces catalyst-agent_2.0.0.ipk
./build.sh --check          # verify a package without rebuilding it
```

Install it through the Systemcore web UI's package manager, or with `opkg install` over SSH. It
registers itself as an auto-start service on port 9010; Console finds it on its own.

The build sets every file mode and owner explicitly rather than reading them off the build host, so
the package is the same bytes whether it was built on Linux, macOS or Windows. That matters more
than it sounds: `ExecStart` names `catalyst_agent.py` directly, and a Windows build that lost the
executable bit produces a service that fails at boot with `203/EXEC` — an error naming the file and
saying nothing about why. Every build ends by reading its own output back and checking exactly that
class of thing.

## Cameras

`/api/cameras` is the Systemcore vision aggregator's camera list (`:4810/api/cameras` on the
device - every Limelight it discovered, the alias addresses it gave each one on each interface,
and whether the camera holds a NetworkTables session with the robot program) joined with each
camera's own `/status` from its REST API on port 5807: temperature, CPU, frame rate, pipeline.
The join is by IP and is pure, so it is tested without a camera. Answers are held for two
seconds; the per-camera fetches run in parallel with a 0.6 s timeout each, because four
sequential timeouts would be a stall inside a poll that expects milliseconds. A camera that did
not answer still appears, marked `statusReachable: false` - hiding it would turn a slow camera into
an absent one.

It is also part of `/api/system`, so Console's existing poll gets it.

## What it does not do

**There are no endpoints that change anything.** Not an oversight — a stated boundary. Catalyst
Console is read-only by design, and an agent that could restart the robot program would put a
restart button one mis-click away from a driver during a match. Diagnosis and control are different
tools, and this is the diagnosis one. `test_agent.py` asserts that `do_GET` is the only handler, so
adding a `do_POST` later fails the tests and makes somebody argue for it.

It is also deliberately the lowest-priority thing on the machine: `Nice=10`, idle CPU and IO
scheduling, and a read-only view of the filesystem. The robot program runs at real-time priority
under PREEMPT_RT, and a diagnostic tool must never be the reason a match is lost.

## Cost

Sampled on request rather than on a timer, so an idle agent is an idle process. The one place it
blocks is the CPU and process sample, which needs two reads of `/proc/stat` a short interval apart —
that interval is 100 ms and it is the agent's entire cost. Console polls it every three seconds, and
only while somebody has the Systemcore page open.

## Tests

```bash
python agent/test_agent.py     # the parsing
python agent/test_build.py     # the packaging
```

`test_build.py` takes the built package apart, breaks one thing, and asserts the checker notices —
a missing payload, a lost executable bit, a CRLF shebang, files owned by the build user, members in
the wrong order. Every one of those installs without complaint and then does not work, which is why
none of them can be checked by building a good package and looking at it.

`test_agent.py` is fifteen tests over the parsing, which is the part that has to be checked somewhere other than a
robot. They cover the Linux text formats that are easy to parse *almost* correctly: a command name
containing spaces in `/proc/stat`, which shifts every field after it; `/proc/meminfo` in kibibytes;
and `MemFree` versus `MemAvailable`, where using the first reports every healthy Linux machine as
nearly out of memory. Each of those produces a plausible wrong number rather than an error.
