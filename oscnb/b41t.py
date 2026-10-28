"""The OWON B41T+ handheld meter: its Bluetooth packet, a background reader,
a logger that writes one CSV row per packet, and the loader for those logs.

The meter sends a 6-byte packet per reading, three little-endian u16 words.
Word 0 holds the function (bits 6..9), the range scale (bits 3..5) and the
decimals (bits 0..2). Word 2 is the reading, bit 15 its sign. The scale maps
to a power of ten per function family. Temperature comes as displayed.

Ohms are read by pushing about a milliamp through the part, so an ohms
reading taken while something else drives the same terminals means nothing.

On Linux the reader talks to bluez: it can pick the adapter (BT_ADAPTER),
power cycle it when a scan sticks, and reconnect by the address the registry
holds. On macOS bleak uses CoreBluetooth, which has one radio, no adapter
choice and no Bluetooth address, only a per-host UUID, so the meter is found
by its name on every run. The first run there asks for the terminal's
Bluetooth permission in System Settings.
"""
import asyncio
import csv
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pandas as pd

FUNCS = {0: "DC V", 1: "AC V", 2: "DC A", 3: "AC A", 4: "ohm", 5: "F", 6: "Hz",
         7: "%", 8: "C", 9: "F(temp)", 10: "diode", 11: "cont", 12: "hFE", 13: "NCV"}
VOLT_EXP = {3: -3, 4: 0}
AMP_EXP = {2: -6, 3: -3, 4: 0}
OHM_EXP = {4: 0, 5: 3, 6: 6}
NAME_HINTS = ("B41", "BDM", "OWON", "LILLIPUT")
SCAN_S = 8.0
SCAN_TRIES = 5
LINUX = sys.platform.startswith("linux")


def adapter():
    """The bluez adapter name, hciN. BT_ADAPTER pins it; otherwise the first
    one the kernel lists. A dongle that resets on USB comes back with a new
    number, so this is read when needed, not once."""
    env = os.environ.get("BT_ADAPTER")
    if env:
        return env
    names = sorted(os.listdir("/sys/class/bluetooth")) if os.path.isdir("/sys/class/bluetooth") else []
    return names[0] if names else "hci0"


def backend():
    """The keyword arguments bleak's scanner and client take on this host.
    Picking an adapter is a bluez feature; CoreBluetooth has one radio."""
    return {"adapter": adapter()} if LINUX else {}


def decode(packet):
    """(function name, value in base units or None, raw dict) of one packet,
    given as bytes or as its hex string."""
    d = bytes.fromhex(packet) if isinstance(packet, str) else bytes(packet)
    if len(d) < 6:
        return None, None, {}
    w0 = d[1] << 8 | d[0]
    w2 = d[5] << 8 | d[4]
    func = (w0 >> 6) & 0x0F
    scale = (w0 >> 3) & 0x07
    dec = w0 & 0x07
    meas = w2 if w2 < 0x7FFF else -(w2 & 0x7FFF)
    val = meas / (10 ** dec)
    if func in (0, 1):
        exp = VOLT_EXP.get(scale)
    elif func in (2, 3):
        exp = AMP_EXP.get(scale)
    elif func == 4:
        exp = OHM_EXP.get(scale)
    elif func in (8, 9):
        exp = 0
    else:
        exp = None
    value = val * (10 ** exp) if exp is not None else None
    return FUNCS.get(func, f"f{func}"), value, {"func": func, "scale": scale, "dec": dec, "meas": meas}


def encode(func, scale, dec, meas):
    """The inverse of `decode`, for tests and synthetic logs."""
    w0 = (func & 0x0F) << 6 | (scale & 0x07) << 3 | (dec & 0x07)
    w2 = meas if meas >= 0 else 0x8000 | (-meas)
    return bytes([w0 & 0xFF, w0 >> 8, 0, 0, w2 & 0xFF, w2 >> 8])


REGISTRY = Path(__file__).resolve().parents[1] / "equipment" / "registry.json"


def known_address():
    """The meter's Bluetooth address from B41T_ADDR, else from the equipment
    registry, else None (then the reader scans for it). The registry holds
    the bluez address; on macOS B41T_ADDR can carry the host's UUID for it."""
    addr = os.environ.get("B41T_ADDR")
    if addr:
        return addr
    if REGISTRY.exists():
        return json.loads(REGISTRY.read_text()).get("dmm", {}).get("address")
    return None


def adapter_power_cycle():
    """Clear a stuck bluez discovery state. Needs no privileges on a desktop."""
    for state in ("off", "on"):
        subprocess.run(["bluetoothctl", "power", state], capture_output=True, timeout=10)
        time.sleep(3.0)


class Meter:
    """Background reader. `latest()` is the newest (host time, function, value,
    raw hex); `on_packet`, if given, is called with the same tuple for every
    packet. The meter advertises as "BDM" until a first connection, then as
    "LILLIPUT"."""

    def __init__(self, on_packet=None):
        self.on_packet = on_packet
        self._last = None
        self._err = None
        self._ready = threading.Event()
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        try:
            asyncio.run(self._main())
        except Exception as e:
            self._err = e
            self._ready.set()

    async def _find(self):
        """The meter's advertisement. It stops advertising while connected and
        for a while after, so the scan is retried. Bluez sometimes refuses to
        start a discovery after another client used one, and an adapter left
        idle can scan and see nothing at all; a power cycle of the adapter
        clears both, and is done once here."""
        from bleak import BleakScanner
        from bleak.exc import BleakDBusError
        names = set()
        cycled = False
        for _ in range(SCAN_TRIES):
            try:
                devs = await BleakScanner.discover(timeout=SCAN_S, **backend())
            except BleakDBusError as e:
                if "InProgress" in str(e) and not cycled:
                    cycled = True
                    adapter_power_cycle()
                    continue
                raise
            dev = next((d for d in devs if d.name and any(h in d.name.upper() for h in NAME_HINTS)), None)
            if dev:
                return dev
            if not devs and LINUX and not cycled:
                cycled = True
                adapter_power_cycle()
                continue
            names |= {d.name for d in devs if d.name}
            await asyncio.sleep(2.0)
        raise RuntimeError(f"no OWON meter found in {SCAN_TRIES} scans; saw {sorted(names)}")

    async def _known(self):
        """The meter as bluez remembers it, if it does. Handing that object to
        the client connects without a scan; a bare address string would make
        the library scan first. Bluez forgets an unpaired device a short while
        after it was last seen, and the meter refuses a connection for a while
        after a disconnect; in both cases this returns None and the scan path
        takes over."""
        from bleak import BleakClient
        from bleak.backends.device import BLEDevice
        from bleak.exc import BleakError
        addr = known_address()
        if not addr:
            return None
        if not LINUX:
            if ":" in addr:
                return None     # a bluez address; CoreBluetooth knows the meter by a UUID of its own
            from bleak import BleakScanner
            return await BleakScanner.find_device_by_address(addr, timeout=SCAN_S)
        path = f"/org/bluez/{adapter()}/dev_{addr.replace(':', '_')}"
        dev = BLEDevice(addr, "B41T+", {"path": path, "props": {}})
        try:
            async with BleakClient(dev, timeout=5.0, **backend()):
                pass
        except asyncio.TimeoutError:
            return None
        except BleakError as e:
            if "not found" in str(e):
                return None
            raise
        return dev

    async def _main(self):
        from bleak import BleakClient
        dev = await self._known() or await self._find()
        self.address, self.name = dev.address, dev.name or "?"
        async with BleakClient(dev, **backend()) as cl:
            chars = [c for s in cl.services for c in s.characteristics if "notify" in c.properties]
            if not chars:
                raise RuntimeError("meter has no notify characteristic")

            def on(_, data):
                name, value, _raw = decode(bytes(data))
                row = (time.time(), name, value, bytes(data).hex())
                self._last = row
                if self.on_packet:
                    self.on_packet(row)
                self._ready.set()

            for c in chars:
                await cl.start_notify(c, on)
            while not self._stop:
                await asyncio.sleep(0.2)

    def wait(self, timeout=SCAN_TRIES * (SCAN_S + 4.0)):
        self._ready.wait(timeout)
        if self._err:
            raise self._err
        return self._last

    def latest(self):
        if self._err:
            raise self._err
        return self._last

    def close(self):
        """Stop and wait for the disconnect. A meter left connected at the
        bluez level stops advertising and the next scan cannot find it."""
        self._stop = True
        self._t.join(timeout=10.0)

    def __enter__(self):
        self.wait()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def fresh(self, timeout=5.0):
        """The next packet to arrive, so a dial change made since the last one
        is seen."""
        last = self._last
        t_end = time.time() + timeout
        while self._last is last and time.time() < t_end:
            time.sleep(0.05)
        return self.latest()

    def log(self, path, expect=None):
        """A Logger on this connection. The meter stays connected between
        logs, which avoids the quiet stretch it goes through after a
        disconnect."""
        return Logger(path, expect, meter=self)


class Logger:
    """A meter log on disk, one row per packet: host time of arrival, decoded
    function, value in base units (volts, amps, ohms, degrees C) and the raw
    packet in hex. Use as a context manager. On its own it connects on entry
    and disconnects on exit; given a connected `meter` it only attaches to it,
    and `Meter.log` is the usual way to get one. With `expect` it refuses to
    start when the meter's dial is on another function, before anything is
    driven."""

    def __init__(self, path, expect=None, meter=None):
        self.path = Path(path)
        self.expect = expect
        self.meter = meter
        self._own = meter is None
        self.rows = 0

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = open(self.path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["t_host", "function", "value", "raw"])
        self._f.flush()
        if self._own:
            self.meter = Meter(self._on_packet)
            row = self.meter.wait()
        else:
            row = self.meter.fresh()
            self.meter.on_packet = self._on_packet
        if self.expect and (row is None or row[1] != self.expect):
            self.__exit__()
            seen = "nothing" if row is None else row[1]
            raise RuntimeError(f"the DMM is on {seen}, this run needs it on {self.expect}")
        return self

    def _on_packet(self, row):
        self._w.writerow([f"{row[0]:.3f}", row[1], "" if row[2] is None else row[2], row[3]])
        self._f.flush()
        self.rows += 1

    def __exit__(self, *exc):
        if self._own:
            self.meter.close()
        else:
            self.meter.on_packet = None
        self._f.close()
        return False


def load(path):
    """A log as a DataFrame: t_host (s), function, value (NaN where the packet
    had no known unit), raw. `t` is seconds from the first row."""
    df = pd.read_csv(path)
    df["value"] = pd.to_numeric(df["value"], errors="coerce")
    df["t"] = df["t_host"] - df["t_host"].iloc[0]
    return df
