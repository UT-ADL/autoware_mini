#!/bin/bash
# One-time setup for the wheel force-feedback daemon: adds your user to the
# 'input' group (so it can read /dev/input/event* without sudo) and installs its
# Python deps. Idempotent. Log out and back in once afterwards for the group
# change to take effect.
#
#   ./setup.sh
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ME="$(id -un)"

# Read /dev/input/event* (root:input) without sudo.
if id -nG "$ME" | grep -qw input; then
    echo "$ME is already in the 'input' group"
else
    sudo usermod -aG input "$ME"
    echo "Added $ME to 'input' — log out and back in (or run 'newgrp input') to apply"
fi

# (deps are also available via apt — python3-evdev python3-aiohttp python3-docopt
#  — if the system Python is externally managed (PEP 668).)
pip install -r "$DIR/requirements.txt"

echo "Wheel FFB setup complete."
