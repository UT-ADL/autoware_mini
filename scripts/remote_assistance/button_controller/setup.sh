#!/bin/bash
# One-time setup for the StreamDeck button controller: HID libs plus a udev rule
# so it talks to the device without sudo, and its Python deps. Idempotent.
#
#   ./setup.sh
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# HID libs + udev rule so the StreamDeck is reachable without sudo.
sudo apt-get update
sudo apt-get install -y libhidapi-libusb0 libhidapi-dev
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0fd9", MODE="0660", GROUP="plugdev"' \
    | sudo tee /etc/udev/rules.d/50-elgato.rules > /dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger

pip install -r "$DIR/requirements.txt"

echo "Button controller setup complete."
