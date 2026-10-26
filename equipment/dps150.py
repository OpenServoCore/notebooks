"""The FNIRSI DPS-150 bench supply over USB serial.

  uv run python equipment/dps150.py read                        print the output voltage and current
  uv run python equipment/dps150.py on <V> [A]                  set the voltage and the current limit (default 2.5 A), enable
  uv run python equipment/dps150.py off                         disable the output
  uv run python equipment/dps150.py steps <dir> <hold s> <A> <V>...   step the voltage, logging

`steps` enables the output at the first voltage, holds each voltage for the
given seconds, and disables it at the end. It writes `events.csv` (the host
time just before each set command, and the voltage) and `dps.csv` (the host
time, voltage and current readback at about 5 Hz) into the directory.

The port, the readback caveats and the functions behind these commands are
in `oscnb.dps150`, which the capture scripts import directly.
"""
import sys

from oscnb import dps150


def main(argv):
    cmd = argv[1] if len(argv) > 1 else "read"
    p = dps150.open_supply()
    try:
        if cmd == "on":
            dps150.on(p, float(argv[2]), float(argv[3]) if len(argv) > 3 else 2.5)
        elif cmd == "off":
            dps150.off(p)
        elif cmd == "steps" and len(argv) > 5:
            dps150.steps(p, argv[2], float(argv[3]), float(argv[4]), [float(v) for v in argv[5:]])
            return 0
        elif cmd != "read":
            print(__doc__)
            return 2
        m = dps150.read(p)
        print(f"{cmd} {m.output_voltage:.3f} V {m.output_current:.3f} A")
        return 0
    finally:
        p.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
