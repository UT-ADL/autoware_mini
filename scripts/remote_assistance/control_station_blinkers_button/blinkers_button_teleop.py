#!/usr/bin/env python3
"""Operator-PC peripheral bridge: serial devices -> browser, over localhost WS.

The teleop UI (web/index.html) runs in the operator's browser on the remote PC.
Two USB-serial peripherals are wired to that same PC:

    --button  /dev/ttyACM0   push-button switch (engage / disengage teleop)
    --lever   /dev/ttyACM1   blinker lever (turn signals)

Firefox has no Web Serial API (it is Chromium-only), so the page cannot read
/dev/ttyACM* itself. This daemon runs alongside the browser, reads both ports
by exact path, and pushes JSON events the UI already knows how to consume
(setTeleopDisabled / onBlinkerEvent). While the WebSocket is open the UI treats
the bridge as the "enable hardware present" gate; if it drops the UI
force-disengages.

Wire protocol (raw bytes from the devices):
    button : 'K' (0x4b) = RELEASED = engage,   'E' (0x45) = PRESSED = disengage
    lever  : 0x00 / 0x01 = LEFT turn,           0x02 / 0x03 = RIGHT turn
             (any other byte is treated as "lever returned to neutral", which
              re-arms the next same-side push so the UI can toggle it off)

WebSocket protocol (JSON text frames, bridge -> browser):
    {"type":"engage",  "value": true|false}        # true = engage teleop
    {"type":"blinker", "side":  "left"|"right"}
    {"type":"hello",   "engage": bool|null,         # state snapshot on connect
                       "blinker": "left"|"right"|null}

Safety: engage is delivered on EDGES only. The button's resting state is
"released = engage", but auto-engaging teleop the instant a browser connects is
unsafe, so the first sample of the button only establishes a silent baseline.
An actual press->release (or release->press) transition is required to move the
latch. Blinker events fire on every change, including the first push.

Run on the operator machine (needs `pip install pyserial websockets`):
    python3 teleop_operator.py
    python3 teleop_operator.py --button /dev/ttyACM0 --lever /dev/ttyACM1 \
                               --baud 115200 --host 127.0.0.1 --port 8765
"""

import argparse
import asyncio
import json
import logging
import threading

import serial          # pyserial
import websockets

log = logging.getLogger("teleop-periph")

# Wire-protocol byte constants.
BTN_RELEASED = 0x4B    # 'K' -> engage
BTN_PRESSED  = 0x45    # 'E' -> disengage
LEVER_LEFT   = (0x00, 0x01)
LEVER_RIGHT  = (0x02, 0x03)

# Idle watchdog: if no lever byte arrives this long after a side latch, force
# the latch back to "neutral". Self-heals a dropped neutral byte over USB —
# without it, a single missed release would stick _lever_pos to "left"/"right"
# and the next same-side push could never re-fire the blinker event.
LEVER_IDLE_TIMEOUT = 0.2


class Bridge:
    """Shared state + fan-out to all connected browser clients."""

    def __init__(self, loop):
        self._loop = loop
        self._clients = set()
        # None = "not yet observed". engage starts None so the resting button
        # state is a silent baseline (see module docstring).
        self.engage = None
        # Physical lever position: "left" | "right" | "neutral". This tracks the
        # DEVICE only — never the logical blinker (which the browser owns and may
        # clear on steering auto-cancel). Keeping it physical means it can't
        # desync: a fresh push is always a real neutral->side transition here.
        self._lever_pos = "neutral"
        # Pending asyncio TimerHandle for the side-latch idle watchdog (loop-thread only).
        self._lever_idle_handle = None

    # -- client lifecycle -------------------------------------------------
    async def serve_client(self, ws):
        self._clients.add(ws)
        log.info("browser connected (%d total)", len(self._clients))
        try:
            await ws.send(json.dumps({
                "type": "hello",
                "engage": self.engage,
                "blinker": self._lever_pos if self._lever_pos in ("left", "right") else None,
            }))
            async for _ in ws:   # we never expect inbound frames; just drain
                pass
        finally:
            self._clients.discard(ws)
            log.info("browser disconnected (%d total)", len(self._clients))

    def _broadcast(self, msg):
        """Called from the asyncio loop thread only."""
        if not self._clients:
            return
        text = json.dumps(msg)
        for ws in list(self._clients):
            # fire-and-forget; a slow/closing client must not block others
            asyncio.create_task(self._safe_send(ws, text))

    @staticmethod
    async def _safe_send(ws, text):
        try:
            await ws.send(text)
        except Exception:
            pass

    def _post(self, msg):
        """Thread-safe entry point for the serial reader threads."""
        self._loop.call_soon_threadsafe(self._broadcast, msg)

    # -- byte handlers (invoked from serial reader threads) ---------------
    def on_button_byte(self, b):
        log.debug("button byte: 0x%02x", b)
        if b == BTN_RELEASED:
            val = True
        elif b == BTN_PRESSED:
            val = False
        else:
            return
        if self.engage is None:
            self.engage = val          # silent baseline; no auto-engage
            log.info("button baseline: %s", "engage" if val else "disengage")
            return
        if val == self.engage:
            return
        self.engage = val
        log.info("button edge -> %s", "ENGAGE" if val else "DISENGAGE")
        self._post({"type": "engage", "value": val})

    def on_lever_byte(self, b):
        # Raw-byte trace for diagnosing lever irregularities (run with --log DEBUG):
        # shows EXACTLY what the device emits at rest / left / right, so a mismatch
        # against the LEVER_LEFT/RIGHT mapping (e.g. an idle 0x00 read as LEFT) is
        # immediately visible.
        log.debug("lever byte: 0x%02x", b)
        if b in LEVER_LEFT:
            pos = "left"
        elif b in LEVER_RIGHT:
            pos = "right"
        else:
            pos = "neutral"

        # Every lever byte (re-)arms the idle watchdog. While the lever is at a
        # side, the timer keeps getting bumped; once bytes stop, it fires and
        # snaps _lever_pos back to neutral so the next push is a fresh edge.
        self._loop.call_soon_threadsafe(self._rearm_lever_idle)

        # Edge-detect on the PHYSICAL lever position. Repeats while held collapse
        # to nothing; we only act when the lever actually moves to a new position.
        if pos == self._lever_pos:
            return
        self._lever_pos = pos
        log.debug("lever pos -> %s", pos)

        # Returning to centre emits no blinker event — the browser owns the
        # blinker and cancels it on steering return (NO_BLINKER), not on lever
        # release. We only fire when the lever enters a side; that neutral->side
        # transition is a fresh push every time, so it re-fires correctly even
        # after the browser has cleared the signal on its own.
        if pos == "neutral":
            return
        log.info("lever -> %s", pos.upper())
        self._post({"type": "blinker", "side": pos})

    # -- lever idle watchdog (loop thread only) ---------------------------
    def _rearm_lever_idle(self):
        if self._lever_idle_handle is not None:
            self._lever_idle_handle.cancel()
            self._lever_idle_handle = None
        if self._lever_pos in ("left", "right"):
            self._lever_idle_handle = self._loop.call_later(
                LEVER_IDLE_TIMEOUT, self._lever_idle_fire)

    def _lever_idle_fire(self):
        self._lever_idle_handle = None
        if self._lever_pos in ("left", "right"):
            log.debug("lever idle timeout -> forcing neutral (was %s)", self._lever_pos)
            self._lever_pos = "neutral"


def reader_thread(name, port, baud, on_byte, stop):
    """Open `port`, feed each received byte to `on_byte`. Reconnect on error."""
    while not stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                log.info("%s: opened %s @ %d", name, port, baud)
                while not stop.is_set():
                    data = ser.read(1)
                    if data:
                        on_byte(data[0])
        except serial.SerialException as e:
            log.warning("%s: %s (%s) — retrying in 2s", name, e, port)
            stop.wait(2.0)
        except Exception as e:                       # pragma: no cover
            log.exception("%s: unexpected error: %s", name, e)
            stop.wait(2.0)


async def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--button", default="/dev/ttyACM0", help="engage push-button port")
    ap.add_argument("--lever",  default="/dev/ttyACM1", help="blinker lever port")
    ap.add_argument("--baud",   type=int, default=115200)
    ap.add_argument("--host",   default="127.0.0.1", help="WS bind addr (localhost only by default)")
    ap.add_argument("--port",   type=int, default=8765, help="WS port")
    ap.add_argument("--log",    default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=getattr(logging, args.log.upper(), logging.INFO),
                        format="%(asctime)s %(levelname)s %(message)s")

    loop = asyncio.get_running_loop()
    bridge = Bridge(loop)
    stop = threading.Event()

    # Startup summary: which role maps to which serial port. Printed before the
    # reader threads connect, so a failed open (or a swapped ACM0/ACM1) is easy
    # to spot against this baseline. Each successful open is confirmed again by
    # reader_thread ("<role>: opened <port>").
    log.info("serial assignment: button=%s  lever=%s  @ %d baud",
             args.button, args.lever, args.baud)

    threads = [
        threading.Thread(target=reader_thread, name="button",
                         args=("button", args.button, args.baud,
                               bridge.on_button_byte, stop), daemon=True),
        threading.Thread(target=reader_thread, name="lever",
                         args=("lever", args.lever, args.baud,
                               bridge.on_lever_byte, stop), daemon=True),
    ]
    for t in threads:
        t.start()

    log.info("WebSocket bridge listening on ws://%s:%d", args.host, args.port)
    try:
        async with websockets.serve(bridge.serve_client, args.host, args.port):
            await asyncio.Future()      # run forever
    finally:
        stop.set()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
