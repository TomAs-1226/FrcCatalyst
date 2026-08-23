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

All of it is already in `/proc` and `/sys`. The agent serves it over HTTP so Console can ask.

## Building and installing

```bash
cd agent
./build.sh                  # produces catalyst-agent_2.0.0.ipk
```

Install it through the Systemcore web UI's package manager, or with `opkg install` over SSH. It
registers itself as an auto-start service on port 9010; Console finds it on its own.

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
python agent/test_agent.py
```

Fifteen tests over the parsing, which is the part that has to be checked somewhere other than a
robot. They cover the Linux text formats that are easy to parse *almost* correctly: a command name
containing spaces in `/proc/stat`, which shifts every field after it; `/proc/meminfo` in kibibytes;
and `MemFree` versus `MemAvailable`, where using the first reports every healthy Linux machine as
nearly out of memory. Each of those produces a plausible wrong number rather than an error.
