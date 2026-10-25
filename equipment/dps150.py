"""The FNIRSI DPS-150 bench supply over USB serial.

  python equipment/dps150.py read            print the output voltage and current
  python equipment/dps150.py on <V> [A]      set the voltage, the current limit (default 2.5 A), enable
  python equipment/dps150.py off             disable the output

The supply enumerates as an Artery AT32 virtual com port. On Linux that is
/dev/serial/by-id/usb-Artery_AT32_Virtual_Com_Port_*, on macOS /dev/cu.usbmodem*.
Set DPS150_PORT to pin a port. The voltage readback agrees with a meter within
millivolts; the current readback reads well under the true current and is not
used as a measurement anywhere in this repo.
"""
import glob
import os
import sys
import time

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


def main(argv):
    cmd = argv[1] if len(argv) > 1 else "read"
    p = open_supply()
    try:
        if cmd == "on":
            volts = float(argv[2])
            amps = float(argv[3]) if len(argv) > 3 else 2.5
            p.set_current(amps)
            p.set_voltage(volts)
            p.enable_output()
            time.sleep(4)
        elif cmd == "off":
            p.disable_output()
            time.sleep(1)
        elif cmd != "read":
            print(__doc__)
            return 2
        m = read(p)
        print(f"{cmd} {m.output_voltage:.3f} V {m.output_current:.3f} A")
        return 0
    finally:
        p.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
