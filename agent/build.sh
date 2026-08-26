#!/bin/sh
# Build catalyst-agent_<version>.ipk.
#
# The work is in build.py. This existed as a shell script first and could not do the job: an .ipk's
# tarballs carry Unix file modes and ownership, and reading those from the host filesystem is wrong
# on any machine that does not have them. On Windows there is no executable bit to read, and `ar` is
# not present in Git Bash at all, so the build could not finish. build.py sets every mode explicitly
# and writes the ar archive itself, so the package is identical wherever it was built.
#
# Kept as build.sh because that is what the README and everyone's muscle memory says.
set -e
cd "$(dirname "$0")"

# Each candidate is asked to run something, not merely found on PATH. Windows ships a `python3` in
# WindowsApps that exists, resolves, and does nothing but print an advert for the Microsoft Store -
# so `command -v` says yes and the build then fails with a message about app execution aliases.
for py in python3 python py; do
    if "$py" -c "import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)" >/dev/null 2>&1; then
        exec "$py" build.py "$@"
    fi
done

echo "No working python3 on PATH. build.py needs one (standard library only)." >&2
exit 1
