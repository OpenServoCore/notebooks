"""The `osc` and `osc-lab` tools from a capture script.

`osc-lab` is the capture instrument: `sweep()` and `bursts()` run it. It
installs with `cargo install --locked --path osc-lab` from this repo, and is
found on the PATH or named by the OSC_LAB environment variable. Every other call
(get, set, status, ping, baud) runs `osc`, on the PATH (`cargo install
--path tools/osc` in the firmware repo) or named by the OSC environment
variable. Every call addresses one servo id, OSC_ID or 1. The tools print
their connection status on stderr and their output on stdout, so stdout is
parsed and stderr is passed through.
"""
import os
import re
import subprocess
from pathlib import Path

TOOL = os.environ.get("OSC", "osc")
LAB = os.environ.get("OSC_LAB", "osc-lab")
ID = os.environ.get("OSC_ID", "1")


def run(*args, capture=False, check=True, tool=TOOL):
    """Run `<tool> -i <id> <args>`, `osc` unless `tool` says otherwise;
    returns stdout when `capture`."""
    cmd = [tool, "-i", ID, *map(str, args)]
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


# The TEL row a static load wants, one chunk: vbus_raw, ntc_raw, current_raw,
# vmotor_a, vmotor_b, duty_ticks. The six are contiguous in the table and fill
# the 12-byte row, so the position, which a resistor has no use for, stays out
# and every stream carries the rail and the board temperature.
CHUNKS_RAW = "0x23c+12"

# The TEL row a rest stream wants: vref_raw, then the same six less the duty,
# which a rest has no use for. vref_raw is the ADC's own 1.2 V reference, so
# a rest stream carries VDD, which every other count is a share of.
CHUNKS_REST = "0x23a+12"


def sweep(out, duty, window_ms=150, baseline_ms=1000, rest_ms=500, dirs="both", decay="slow",
          tel_chunks=CHUNKS_RAW, static_load=True):
    """`osc-lab sweep` into `out` (sweep.csv + meta.json). `duty` is the step list
    as the tool takes it, percent, with `then:` and `@MS` forms allowed."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    args = ["sweep", "--out", out, "--duty-pct", ",".join(map(str, duty)), "--window-ms", window_ms,
            "--baseline-ms", baseline_ms, "--rest-ms", rest_ms, "--dirs", dirs, "--decay", decay]
    if tel_chunks:
        args += ["--tel-chunks", tel_chunks]
    if static_load:
        args.append("--static-load")
    run(*args, tool=LAB)
    return out


def bursts(out, pct, repeats=3, chans="diff", dirs="both", rest_ms=150):
    """`osc-lab burst` into `out` (burst-K.csv + meta.json): the ADC free running
    on the shunt and the taps through PWM periods around a duty step from
    rest, each duty by sign by repeat. The raw instrument, no fit."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    run("burst", "--out", out, "--duty-pct", ",".join(map(str, pct)), "--repeats", repeats,
        "--chans", chans, "--dirs", dirs, "--rest-ms", rest_ms, tool=LAB)
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
