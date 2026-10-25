"""The OWON B41T+ handheld meter over Bluetooth LE.

  python equipment/b41t.py read             connect and print one reading
  python equipment/b41t.py log <out.csv>    log every packet until Ctrl-C

The meter sends a 6-byte packet per reading: three little-endian u16 words,
word 0 holds the function (bits 6..9), the range scale (bits 3..5) and the
decimals (bits 0..2), word 2 the reading with bit 15 as its sign. The log
keeps the host time of arrival, the decoded function and value in base units
(volts, amps, ohms, degrees C), and the raw packet in hex so a row can be
re-decoded. It advertises as "BDM" until a first connection, then as
"LILLIPUT".

Ohms are read by pushing about a milliamp through the part, so an ohms
reading taken while something else drives the same terminals means nothing.
"""
import asyncio
import csv
import sys
import threading
import time

FUNCS = {0: "DC V", 1: "AC V", 2: "DC A", 3: "AC A", 4: "ohm", 5: "F", 6: "Hz",
         7: "%", 8: "C", 9: "F(temp)", 10: "diode", 11: "cont", 12: "hFE", 13: "NCV"}
VOLT_EXP = {3: -3, 4: 0}
AMP_EXP = {2: -6, 3: -3, 4: 0}
OHM_EXP = {4: 0, 5: 3, 6: 6}
NAME_HINTS = ("B41", "BDM", "OWON", "LILLIPUT")


def decode(d: bytes):
    """(function name, value in base units or None, raw dict) of one packet."""
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


class Meter:
    """Background reader. `latest()` is the newest (host time, function, value, raw hex);
    `on_packet`, if given, is called with the same tuple for every packet."""

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

    async def _main(self):
        from bleak import BleakClient, BleakScanner
        devs = await BleakScanner.discover(timeout=8.0)
        dev = next((d for d in devs if d.name and any(h in d.name.upper() for h in NAME_HINTS)), None)
        if dev is None:
            names = sorted({d.name for d in devs if d.name})
            raise RuntimeError(f"no OWON meter found; saw {names}")
        self.address, self.name = dev.address, dev.name
        async with BleakClient(dev) as cl:
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

    def wait(self, timeout=25.0):
        self._ready.wait(timeout)
        if self._err:
            raise self._err
        return self._last

    def latest(self):
        if self._err:
            raise self._err
        return self._last

    def close(self):
        self._stop = True


def main(argv):
    cmd = argv[1] if len(argv) > 1 else "read"
    if cmd == "read":
        m = Meter()
        row = m.wait()
        print(m.name, m.address, "->", row)
        m.close()
        return 0
    if cmd == "log" and len(argv) > 2:
        f = open(argv[2], "w", newline="")
        w = csv.writer(f)
        w.writerow(["t_host", "function", "value", "raw"])
        n = [0]

        def on_packet(row):
            w.writerow([f"{row[0]:.3f}", row[1], "" if row[2] is None else row[2], row[3]])
            f.flush()
            n[0] += 1

        m = Meter(on_packet)
        m.wait()
        print(f"logging {m.name} to {argv[2]}, Ctrl-C to stop", file=sys.stderr)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        m.close()
        f.close()
        print(f"{n[0]} rows", file=sys.stderr)
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
