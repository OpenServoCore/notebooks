"""The FNIRSI DPS-150 bench supply over USB serial.

`steps` enables the output at the first voltage, holds each voltage for the
given seconds, and disables it at the end. It writes `events.csv` (the host
time just before each set command, and the voltage) and `dps.csv` (the host
time, voltage and current readback at about 5 Hz) into the directory.

The supply enumerates as an Artery AT32 virtual com port. On Linux that is
/dev/serial/by-id/usb-Artery_AT32_Virtual_Com_Port_*, on macOS /dev/cu.usbmodem*.
Set DPS150_PORT to pin a port. The voltage readback agrees with a meter within
millivolts. The current readback reads well under the true current and is not
used as a measurement anywhere in this repo.
"""
import csv
import glob
import os
import time
from pathlib import Path

from fnirsi_dps150 import DPS150

PORT_GLOBS = ("/dev/serial/by-id/usb-Artery_AT32_Virtual_Com_Port_*", "/dev/cu.usbmodem*")


def ports():
    env = os.environ.get("DPS150_PORT")
    if env:
        return [env]
    return [p for g in PORT_GLOBS for p in sorted(glob.glob(g))]


def open_supply():
    tried = ports()
    for port in tried:
        p = DPS150(port)
        try:
            p.connect()
            p.read_measurements()
            return p
        except Exception:
            try:
                p.close()
            except Exception:
                pass
    raise RuntimeError(f"no DPS-150 answered on {tried}")


def read(p):
    """A fresh reading. The supply also streams unsolicited packets, so the
    library's request can match a stale one still buffered; drop the backlog
    first."""
    p._drain_input()
    return p.read_measurements()


def off(p):
    p.disable_output()
    time.sleep(1)


def on(p, volts, amps=2.5):
    p.set_current(amps)
    p.set_voltage(volts)
    p.enable_output()
    time.sleep(4)


def _log(p, out, hold_s, setter, levels, column):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "events.csv", "w", newline="") as fe, open(out / "dps.csv", "w", newline="") as fd:
        ev, log = csv.writer(fe), csv.writer(fd)
        ev.writerow(["t_host", column])
        log.writerow(["t_host", "volts", "amps"])
        setter(levels[0])
        ev.writerow([f"{time.time():.3f}", levels[0]])
        fe.flush()
        p.enable_output()
        for k, v in enumerate(levels):
            if k:
                ev.writerow([f"{time.time():.3f}", v])
                fe.flush()
                setter(v)
            t_end = time.time() + hold_s
            while time.time() < t_end:
                m = read(p)
                log.writerow([f"{time.time():.3f}", f"{m.output_voltage:.3f}", f"{m.output_current:.3f}"])
                fd.flush()
                time.sleep(0.2)
        p.disable_output()


def steps(p, out, hold_s, amps, volts):
    """Step the voltage with the current limit fixed. `events.csv` has a
    `volts` column."""
    p.set_current(amps)
    _log(p, out, hold_s, p.set_voltage, volts, "volts")


def current_steps(p, out, hold_s, volts, amps):
    """Step the current limit with the voltage fixed high enough that the
    supply sits in constant current. For a load small against `volts`, such
    as the leads alone. `events.csv` has an `amps` column."""
    p.set_voltage(volts)
    _log(p, out, hold_s, p.set_current, amps, "amps")
