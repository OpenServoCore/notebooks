"""The `osc` tool from a capture script.

The tool is on the PATH (`cargo install --path tools/osc` in the firmware
repo) or named by the OSC environment variable. Every call addresses one
servo id, OSC_ID or 1. The tool prints its connection status on stderr and
its output on stdout, so stdout is parsed and stderr is passed through.
"""
import os
import re
import subprocess
from pathlib import Path

TOOL = os.environ.get("OSC", "osc")
ID = os.environ.get("OSC_ID", "1")


def run(*args, capture=False, check=True):
    """Run `osc -i <id> <args>`; returns stdout when `capture`."""
    cmd = [TOOL, "-i", ID, *map(str, args)]
    r = subprocess.run(cmd, stdout=subprocess.PIPE if capture else None, text=True, check=check)
    return r.stdout if capture else None


def bus_baud():
    """The bus rate the tool finds, from its connection line on stderr."""
    r = subprocess.run([TOOL, "-i", ID, "ping"], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    m = re.search(r"bus at (\d+) baud", r.stderr)
    if r.returncode != 0 or not m:
        raise RuntimeError(f"no servo answers: {r.stderr.strip()}")
    return int(m.group(1))


def require_3m():
    """A TEL stream at the control rate needs the 3 Mbps bus; refuse to
    record on a slower one rather than lose frames."""
    baud = bus_baud()
    if baud < 3_000_000:
        raise SystemExit(f"the bus is at {baud} baud; TEL needs 3000000 (osc baud 3000000, then osc save)")


def get(field):
    """One register's decoded value, as the tool prints it after `=`."""
    out = run("get", field, capture=True)
    m = re.search(rf"^{re.escape(field)} = (\S+)", out, re.M)
    if not m:
        raise RuntimeError(f"osc get {field}: unexpected output {out!r}")
    v = m.group(1)
    try:
        return int(v, 0)
    except ValueError:
        return v


def set_(field, value):
    """Write one register, volatile until a SAVE."""
    run("set", field, value)


def status():
    return run("status", capture=True)


def sweep(out, duty, window_ms=150, baseline_ms=1000, rest_ms=500, dirs="both", decay="slow",
          tel_chunks=None, static_load=True):
    """`osc sweep` into `out` (sweep.csv + meta.json). `duty` is the step list
    as the tool takes it, percent, with `then:` and `@MS` forms allowed."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    args = ["sweep", "--out", out, "--duty-pct", ",".join(map(str, duty)), "--window-ms", window_ms,
            "--baseline-ms", baseline_ms, "--rest-ms", rest_ms, "--dirs", dirs, "--decay", decay]
    if tel_chunks:
        args += ["--tel-chunks", tel_chunks]
    if static_load:
        args.append("--static-load")
    run(*args)
    return out


# The default TEL row: pos, then current_raw, vmotor_a, vmotor_b, vbus_raw, duty_ticks.
# A row holds 12 bytes, so the NTC rides along only in place of the position,
# which a static load has no use for.
CHUNKS_DEFAULT = "0x238+2,0x240+10"
CHUNKS_NTC = "0x23e+2,0x240+10"


def bursts(out, pct, repeats=3, chans="diff"):
    """`osc ident burst --static-load`: the ADC free running on the shunt and
    the taps through PWM periods around a duty step, `burst-K.csv` files in
    `out`."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    run("ident", "--out", out, "--burst-pct", ",".join(map(str, pct)), "--burst-repeats", repeats,
        "--burst-chans", chans, "burst", "--static-load")
    return out


class Limit:
    """A volatile current limit for a sitting: a resistor load draws more than
    the limit a servo holds, so the laps run under a wider one and the saved
    value comes back at the end. Use as a context manager."""

    def __init__(self, counts):
        self.counts = counts

    def __enter__(self):
        self.saved = get("current_limit_counts")
        set_("current_limit_counts", self.counts)
        return self

    def __exit__(self, *exc):
        set_("current_limit_counts", self.saved)
