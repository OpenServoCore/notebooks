"""The OWON B41T+ handheld meter over Bluetooth LE.

  uv run python equipment/b41t.py read             connect and print one reading
  uv run python equipment/b41t.py log <out.csv>    log every packet until Ctrl-C or SIGTERM

The log has one row per packet: the host time of arrival, the decoded
function and value in base units (volts, amps, ohms, degrees C), and the raw
packet in hex. The protocol, the reader and the logger live in `oscnb.b41t`,
which the capture scripts import directly.

The meter's Bluetooth address comes from `equipment/registry.json` (or B41T_ADDR)
and the reader then connects without a scan. Without one it scans for the
meter's name. The reader disconnects on exit, since a meter left connected stops
advertising, and it power cycles the adapter once if bluez refuses to scan.
"""
import signal
import sys
import time

from oscnb.b41t import Logger, Meter


def main(argv):
    cmd = argv[1] if len(argv) > 1 else "read"
    if cmd == "read":
        m = Meter()
        row = m.wait()
        print(m.name, m.address, "->", row)
        m.close()
        return 0
    if cmd == "log" and len(argv) > 2:
        stop = [False]
        signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
        with Logger(argv[2]) as log:
            print(f"logging {log.meter.name} to {argv[2]}", file=sys.stderr)
            try:
                while not stop[0]:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
        print(f"{log.rows} rows", file=sys.stderr)
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
