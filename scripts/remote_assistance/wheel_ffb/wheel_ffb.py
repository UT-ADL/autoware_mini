#!/usr/bin/env python3
"""Wheel force-feedback daemon — drives an FFB wheel to a target angle via
PID on FF_CONSTANT, with Coulomb friction feedforward (pure PID can't push
past gearbox stiction on most wheels).

Listens on a localhost WebSocket. The browser pushes target angles; the
daemon broadcasts status + telemetry. With no FFB wheel it reports no_device
but keeps polling, recovering automatically on a later hot-plug.

Usage:
    wheel_ffb.py [--device=PATH] [--port=PORT]
                 [--kp=K] [--ki=K] [--kd=K]
                 [--max-gain=G] [--watchdog-ms=MS] [--hz=HZ]
                 [--telemetry-hz=HZ]
                 [--friction-comp=F] [--friction-deadband=D]
                 [--target-lpf-hz=HZ] [--invert]
                 [--detect-timeout-s=S]
                 [--config=PATH] [--vendor=NAME]

Options:
    --device=PATH           evdev path or auto-detect by name [default: auto]
    --port=PORT             localhost WebSocket port [default: 8766]
    --kp=K                  PID proportional gain [default: 1.0]
    --ki=K                  PID integral gain [default: 0.1]
    --kd=K                  PID derivative gain [default: 0.20]
    --max-gain=G            Hard clamp on |torque| in [0, 1] [default: 0.5]
    --watchdog-ms=MS        Stop FFB if no msg for this long [default: 250]
    --hz=HZ                 PID/output loop rate [default: 500]
    --telemetry-hz=HZ       Wheel-state broadcast rate [default: 50]
    --friction-comp=F       Static-friction feedforward; halves while moving,
                            ramps to 2.5× when stuck. T300 ~0.08, G29 ~0.18.
                            [default: 0.0]
    --friction-deadband=D   Skip friction-comp when |err| < D [default: 0.005]
    --target-lpf-hz=HZ      Low-pass cutoff (1st-order exp) on the commanded
                            target. 0 disables. [default: 0]
    --invert                Flip torque sign (T300+hid-tmff2 and Logitech
                            evdev have opposite motor/encoder polarity).
    --detect-timeout-s=S    Grace period before broadcasting no_device; the
                            daemon keeps polling after, so a later hot-plug
                            recovers without a restart. [default: 8]
    --config=PATH           Path to wheel_configs.json. Defaults to the
                            file next to this script.
    --vendor=NAME           Force a specific vendor preset from the JSON;
                            otherwise picked by name match against the
                            detected device.

CLI flags override JSON preset values; JSON preset values override the
docopt defaults shown above.

Setup (one-time): run scripts/remote_assistance/setup.sh — it installs the deps
(evdev, aiohttp, docopt) and adds your user to the 'input' group for read access
to the wheel's /dev/input/eventN node.
"""

import asyncio
import errno
import json
import logging
import math
import signal
import sys
import time
from pathlib import Path

from aiohttp import web, WSMsgType
from docopt import docopt
from evdev import InputDevice, ecodes, ff, list_devices

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "wheel_configs.json"

logging.basicConfig(
    level=logging.INFO,
    format="[ffb %(asctime)s.%(msecs)03d] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("ffb")

FF_LEVEL_MAX = 32767


def load_wheel_configs(path):
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        log.info(f"No config at {path} — using docopt defaults")
        return {}
    except json.JSONDecodeError as e:
        log.warning(f"Malformed {path}: {e} — using docopt defaults")
        return {}


def pick_vendor_preset(configs, device_name, forced_vendor=None):
    """Return (vendor_name, preset_dict) for the matching vendor, or
    (None, None) if the device isn't listed in wheel_configs.json.
    forced_vendor wins if set; otherwise match device_name against each
    preset's `match` list (case-insensitive substring)."""
    if forced_vendor:
        preset = configs.get(forced_vendor)
        if isinstance(preset, dict):
            return forced_vendor, preset
        log.warning(f"--vendor={forced_vendor} not in config")
        return None, None

    name_lc = (device_name or "").lower()
    for vendor, preset in configs.items():
        if vendor.startswith("_") or not isinstance(preset, dict):
            continue
        for hint in preset.get("match", []) or []:
            if hint.lower() in name_lc:
                return vendor, preset
    return None, None


def cli_flag_names(argv):
    """Return the set of --flag names the user passed (without =value).
    Used to suppress preset overrides for flags the user set explicitly."""
    seen = set()
    for arg in argv:
        if arg.startswith("--"):
            seen.add(arg.split("=", 1)[0])
    return seen


def apply_preset(args, preset, explicit_flags):
    """Mutate args (the docopt dict) with values from `preset` for flags the
    user did NOT pass on the command line. Preset keys are flag names
    without the leading `--` (e.g. `kp`, `max-gain`, `invert`)."""
    for key, value in preset.items():
        if key in ("match", "_comment") or key.startswith("_"):
            continue
        flag = f"--{key}"
        if flag in explicit_flags:
            continue
        if flag not in args:
            # Preset references a flag the daemon doesn't know about — skip.
            continue
        if isinstance(value, bool):
            args[flag] = bool(value)
        else:
            args[flag] = str(value)


# --- Device detection -------------------------------------------------------

def find_configured_wheel(arg, configs, forced_vendor=None):
    """Look for an attached FFB wheel whose name matches a vendor entry in
    `configs`. Returns (device, vendor_name, preset) or None. Never raises
    on missing device — that case is handled by the detection loop."""
    if arg != "auto":
        try:
            d = InputDevice(arg)
        except OSError as e:
            log.warning(f"Could not open {arg}: {e!r}")
            return None
        vendor, preset = pick_vendor_preset(configs, d.name, forced_vendor)
        if vendor is None:
            log.warning(f"{arg} [{d.name}] not listed in wheel_configs.json")
            d.close()
            return None
        return d, vendor, preset

    for path in list_devices():
        try:
            d = InputDevice(path)
        except OSError:
            continue
        if ecodes.EV_FF not in d.capabilities():
            d.close()
            continue
        vendor, preset = pick_vendor_preset(configs, d.name, forced_vendor)
        if vendor is not None:
            return d, vendor, preset
        d.close()
    return None


# --- FFB effect & controller ------------------------------------------------

def read_wheel_norm(dev, axis_code=ecodes.ABS_X):
    info = dev.absinfo(axis_code)
    span = (info.max - info.min) / 2.0
    if span <= 0:
        return 0.0
    center = (info.max + info.min) / 2.0
    return max(-1.0, min(1.0, (info.value - center) / span))


def make_constant_effect(level_i16, effect_id=-1):
    """FF_CONSTANT effect; sign of `level` selects direction.

    direction=0x4000 is required for hid-tmff2 (multiplies level by
    sin(direction); 0 zeroes the force). Other drivers ignore direction.
    Replay length=0 = infinite — works for every wheel we support."""
    level_i16 = int(max(-FF_LEVEL_MAX, min(FF_LEVEL_MAX, level_i16)))
    return ff.Effect(
        ecodes.FF_CONSTANT, effect_id, 0x4000,
        ff.Trigger(0, 0),
        ff.Replay(0, 0),
        ff.EffectType(ff_constant_effect=ff.Constant(
            level=level_i16, envelope=ff.Envelope(0, 0, 0, 0))),
    )


class FFBController:
    """PID position controller with friction feedforward. Writes FF_CONSTANT
    at `hz` to drive the wheel to `self.target`."""

    VEL_MOVING_THRESHOLD = 0.05   # norm/s; above this the wheel is "moving"
    STUCK_BOOST_RATE = 5.0        # 1.0×→2.5× over ~300 ms of being stuck
    STUCK_BOOST_MAX = 1.5
    I_CONTRIB_MAX = 0.15          # integrator torque contribution capped at
                                  # this fraction of max_gain (anti-windup)

    def __init__(self, dev, kp, ki, kd, max_gain, watchdog_s, hz,
                 invert=False, friction_comp=0.0, friction_deadband=0.005,
                 target_lpf_hz=0.0):
        self.dev = dev
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_gain = max_gain
        self.watchdog_s = watchdog_s
        self.dt = 1.0 / hz
        self.invert = invert
        self.friction_comp = friction_comp
        self.friction_deadband = friction_deadband
        # Target low-pass: 1st-order exp, alpha = dt / (RC + dt), RC = 1/(2π·fc).
        # fc=0 disables (alpha=1 → filtered == raw).
        self._target_lpf_alpha = (
            self.dt / (1.0 / (2.0 * math.pi * target_lpf_hz) + self.dt)
            if target_lpf_hz > 0 else 1.0)

        self.mode = "target"          # "target" | "torque"
        self.target = 0.0
        self._target_filtered = 0.0
        self.direct_torque = 0.0
        self.last_msg_ts = 0.0
        self.last_err = 0.0
        self.err_integral = 0.0
        self.last_applied = 0.0
        self.effect_id = None
        self._effect_playing = False
        self._last_current = None
        self._stuck_time = 0.0
        self._running = False
        self._msg_count = 0

    def set_mode(self, m):
        if m not in ("target", "torque") or m == self.mode:
            return
        self.last_err = 0.0
        self.err_integral = 0.0
        self.mode = m

    def set_target(self, t):
        self.target = max(-1.0, min(1.0, float(t)))
        self.last_msg_ts = time.monotonic()
        self._msg_count += 1

    def set_torque(self, t):
        self.direct_torque = max(-1.0, min(1.0, float(t)))
        self.last_msg_ts = time.monotonic()
        self._msg_count += 1

    def _upload(self, value_norm):
        physical = -value_norm if self.invert else value_norm
        is_new = self.effect_id is None
        eid_arg = -1 if is_new else self.effect_id
        level = int(physical * FF_LEVEL_MAX)
        new_id = self.dev.upload_effect(make_constant_effect(level, eid_arg))
        if is_new:
            self.effect_id = new_id
        if not self._effect_playing:
            self.dev.write(ecodes.EV_FF, self.effect_id, 1)
            self._effect_playing = True
        self.last_applied = value_norm

    def _stop_playing(self):
        if self.effect_id is not None and self._effect_playing:
            try:
                self.dev.write(ecodes.EV_FF, self.effect_id, 0)
            except OSError:
                pass
            self._effect_playing = False
        self.last_applied = 0.0

    def _compute_output(self, current):
        if self.mode == "torque":
            torque = self.direct_torque
            if self.friction_comp > 0.0 and abs(torque) > self.friction_deadband:
                torque += self.friction_comp if torque > 0 else -self.friction_comp
            return max(-self.max_gain, min(self.max_gain, torque))

        # Low-pass the target so per-tick slider jitter doesn't slam kp×err.
        self._target_filtered += self._target_lpf_alpha * (self.target - self._target_filtered)
        prev_err = self.last_err
        err = self._target_filtered - current
        self.last_err = err

        if self._last_current is None:
            self._last_current = current
        d_current = (current - self._last_current) / self.dt
        wheel_vel = abs(d_current)
        self._last_current = current

        # Anti-windup: zero integrator on zero-crossing of err so a wound-up
        # integrator from holding the wheel doesn't park it offset on release.
        if prev_err * err < 0:
            self.err_integral = 0.0

        if self.ki > 0.0:
            self.err_integral += err * self.dt
            i_limit = self.I_CONTRIB_MAX * self.max_gain / self.ki
            self.err_integral = max(-i_limit, min(i_limit, self.err_integral))

        # Derivative on measurement (not on error): kd reacts to wheel motion,
        # ignoring target jumps. d/dt(err) = -d/dt(current) at steady target.
        # With d/dt(err) instead, slider jitter feeds kd → audible buzz.
        torque = self.kp * err + self.ki * self.err_integral - self.kd * d_current

        if self.friction_comp > 0.0 and abs(err) > self.friction_deadband:
            if wheel_vel > self.VEL_MOVING_THRESHOLD:
                base = self.friction_comp * 0.5
                self._stuck_time = 0.0
            else:
                self._stuck_time += self.dt
                boost = 1.0 + min(self.STUCK_BOOST_MAX,
                                  self._stuck_time * self.STUCK_BOOST_RATE)
                base = self.friction_comp * boost
            torque += base if err > 0 else -base
        else:
            self._stuck_time = 0.0

        return max(-self.max_gain, min(self.max_gain, torque))

    async def run(self):
        self._running = True
        # FF_GAIN default may be 0 on some drivers (effects come out silent);
        # FF_AUTOCENTER would oppose every commanded force.
        for code, val in ((ecodes.FF_GAIN, 65535), (ecodes.FF_AUTOCENTER, 0)):
            try:
                self.dev.write(ecodes.EV_FF, code, val)
            except OSError as e:
                log.warning(f"Could not set EV_FF {code}: {e!r}")

        log.info(f"FFB active on {self.dev.path} ({self.dev.name})")
        last_log = 0.0
        upload_errs = 0
        msgs_at_last_log = 0
        try:
            while self._running:
                now = time.monotonic()
                try:
                    current = read_wheel_norm(self.dev)
                except OSError as e:
                    # Wheel went away (unplugged, or a Fanatec switched to PS4
                    # mode re-enumerates the USB device). Return so the
                    # supervisor tears this controller down and re-detects.
                    log.warning(f"wheel read failed ({e!r}) — disconnect; stopping controller")
                    break

                stale = (now - self.last_msg_ts) > self.watchdog_s
                if stale or self.last_msg_ts == 0.0:
                    self._stop_playing()
                    self.last_err = 0.0
                    self.err_integral = 0.0
                    out = 0.0
                else:
                    out = self._compute_output(current)
                    try:
                        self._upload(out)
                    except OSError as e:
                        if e.errno == errno.ENODEV:
                            log.warning("wheel gone during upload; stopping controller")
                            break
                        upload_errs += 1
                        if upload_errs <= 3 or upload_errs % 500 == 0:
                            log.error(f"FFB upload failed (#{upload_errs}): {e!r}")
                        self.last_applied = out

                if now - last_log >= 1.0:
                    since = (now - self.last_msg_ts) * 1000.0 if self.last_msg_ts > 0 else -1.0
                    rate = self._msg_count - msgs_at_last_log
                    msgs_at_last_log = self._msg_count
                    log.info(
                        f"loop mode={self.mode} target={self.target:+.3f} "
                        f"cur={current:+.3f} out={out:+.3f} "
                        f"since_msg={since:.0f}ms msgs/s={rate} "
                        f"playing={self._effect_playing} upload_errs={upload_errs}"
                    )
                    last_log = now

                await asyncio.sleep(self.dt)
        finally:
            self.stop()

    def stop(self):
        self._running = False
        if self.effect_id is not None:
            try:
                self.dev.write(ecodes.EV_FF, self.effect_id, 0)
                self.dev.erase_effect(self.effect_id)
            except OSError:
                pass
            self.effect_id = None
            self._effect_playing = False


# --- WS server --------------------------------------------------------------

def _parse_value(data):
    try:
        return float(data["value"]) if "value" in data else None
    except (TypeError, ValueError):
        return None


async def _broadcast_status(app, state, **extra):
    msg = {"type": "status", "state": state, **extra}
    state_dict = app["state"]
    state_dict["last_status"] = msg
    log.info(f"status → {state} {extra if extra else ''}")
    payload = json.dumps(msg)
    dead = []
    for ws in list(state_dict["clients"]):
        try:
            await ws.send_str(payload)
        except (ConnectionResetError, RuntimeError):
            dead.append(ws)
    for ws in dead:
        state_dict["clients"].discard(ws)


async def ws_handler(request):
    state_dict = request.app["state"]
    ws = web.WebSocketResponse(heartbeat=60)
    await ws.prepare(request)
    state_dict["clients"].add(ws)
    peer = request.remote
    log.info(f"WS connected from {peer}")

    # Greet the new client with the cached status so it knows where we are.
    last = state_dict.get("last_status")
    if last is not None:
        try:
            await ws.send_str(json.dumps(last))
        except ConnectionResetError:
            pass

    telemetry_dt = state_dict["telemetry_dt"]

    async def push_telemetry():
        try:
            while not ws.closed:
                ctl: FFBController = state_dict.get("ctl")
                if ctl is not None and ctl.dev is not None:
                    await ws.send_str(json.dumps({
                        "type": "telemetry",
                        "wheel": read_wheel_norm(ctl.dev),
                        "torque": ctl.last_applied,
                        "mode": ctl.mode,
                        "max_gain": ctl.max_gain,
                    }))
                await asyncio.sleep(telemetry_dt)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        except Exception as e:
            log.warning(f"telemetry push stopped: {e!r}")

    push_task = asyncio.create_task(push_telemetry())

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            ctl: FFBController = state_dict.get("ctl")
            if ctl is None:
                # Still in detection phase — accept and discard.
                continue
            if "mode" in data:
                ctl.set_mode(data["mode"])
            if "target" in data:
                ctl.set_mode("target")
                ctl.set_target(data["target"])
            elif "torque" in data:
                ctl.set_mode("torque")
                ctl.set_torque(data["torque"])
            else:
                v = _parse_value(data)
                if v is not None:
                    (ctl.set_torque if ctl.mode == "torque" else ctl.set_target)(v)
    finally:
        if push_task is not None:
            push_task.cancel()
        state_dict["clients"].discard(ws)
        log.info(f"WS disconnected ({peer})")
    return ws


# --- Detection + lifecycle --------------------------------------------------

async def supervise(args, app, stop_event):
    """Keep an FFB controller bound to an attached, configured wheel for the
    daemon's lifetime. Detect on startup; if none within the timeout, report
    no_device but KEEP polling — so a wheel plugged in later (or a Fanatec
    switched back from PS4 to PC mode, which re-enumerates the USB device) is
    picked up automatically, without restarting the daemon/app. When the active
    wheel disconnects, tear the controller down, report searching, and
    re-detect. Never exits on its own (only on stop_event)."""
    configs = load_wheel_configs(args["--config"] or DEFAULT_CONFIG_PATH)
    forced_vendor = args["--vendor"]
    explicit_flags = app["state"]["explicit_flags"]
    timeout_s = float(args["--detect-timeout-s"])
    state_dict = app["state"]

    await _broadcast_status(app, "searching")
    deadline = time.monotonic() + timeout_s
    announced_no_device = False

    while not stop_event.is_set():
        found = find_configured_wheel(args["--device"], configs, forced_vendor)
        if found is None:
            # No wheel yet. After the initial grace period, announce no_device
            # ONCE (browser falls back to read-only) but keep polling so a
            # later hot-plug / PC-mode switch recovers on its own.
            if not announced_no_device and time.monotonic() >= deadline:
                await _broadcast_status(
                    app, "no_device",
                    reason=f"no configured FFB wheel within {timeout_s:.0f}s")
                announced_no_device = True
            await asyncio.sleep(1.0)
            continue

        # Wheel found — apply its preset (for flags the user didn't pass; the
        # vendor isn't known until detection) and run a controller until the
        # device drops.
        dev, vendor, preset = found
        announced_no_device = False
        apply_preset(args, preset, explicit_flags)
        log.info(f"Loaded preset '{vendor}' for [{dev.name}]: "
                 + ", ".join(f"{k}={v}" for k, v in preset.items()
                             if not k.startswith("_") and k != "match"))
        ctl = FFBController(
            dev=dev,
            kp=float(args["--kp"]),
            ki=float(args["--ki"]),
            kd=float(args["--kd"]),
            max_gain=float(args["--max-gain"]),
            watchdog_s=float(args["--watchdog-ms"]) / 1000.0,
            hz=float(args["--hz"]),
            invert=bool(args["--invert"]),
            friction_comp=float(args["--friction-comp"]),
            friction_deadband=float(args["--friction-deadband"]),
            target_lpf_hz=float(args["--target-lpf-hz"]),
        )
        state_dict["ctl"] = ctl
        await _broadcast_status(app, "ready", device=dev.name or "unknown",
                                vendor=vendor or "")
        try:
            await ctl.run()   # returns when the wheel drops or stop() is called
        finally:
            ctl.stop()
            state_dict["ctl"] = None
            try:
                dev.close()
            except OSError:
                pass

        # ctl.run() returned. If we're not shutting down, the wheel dropped —
        # go back to searching (with a fresh grace period before re-announcing
        # no_device, so a quick PS4↔PC toggle just shows "searching").
        if not stop_event.is_set():
            log.info("wheel disconnected — searching for a wheel again")
            await _broadcast_status(app, "searching")
            deadline = time.monotonic() + timeout_s
            announced_no_device = False


async def run_server(args):
    port = int(args["--port"])
    app = web.Application()
    app["state"] = {
        "ctl": None,
        "clients": set(),
        "telemetry_dt": 1.0 / float(args["--telemetry-hz"]),
        "explicit_flags": cli_flag_names(sys.argv[1:]),
        "last_status": None,
    }
    app.router.add_get("/ws", ws_handler)

    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, "127.0.0.1", port).start()
    url = f"http://127.0.0.1:{port}/"
    log.info(f"Listening on {url}   (WebSocket: /ws)")

    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(sig, stop_event.set)

    supervise_task = asyncio.create_task(supervise(args, app, stop_event))
    await stop_event.wait()

    log.info("Shutting down")
    state_dict = app["state"]
    ctl = state_dict.get("ctl")
    if ctl is not None:
        ctl.stop()             # let ctl.run() return so the supervisor unwinds
    supervise_task.cancel()    # break out of the supervise loop / any sleep
    try:
        await supervise_task
    except asyncio.CancelledError:
        pass
    await runner.cleanup()
    if ctl is not None and ctl.dev is not None:
        try:
            ctl.dev.close()
        except OSError:
            pass


def main():
    args = docopt(__doc__)
    asyncio.run(run_server(args))


if __name__ == "__main__":
    main()
