"""The `osc` and `osc-lab` tools from a capture script.

`osc-lab` is the capture instrument: `sweep()`, `bursts()` and `track()` run
it. It installs with `cargo install --locked --path osc-lab` from this repo,
and is found on the PATH or named by the OSC_LAB environment variable. Every
other call (get, set, read, write, status, ping, baud) runs `osc`, on the PATH (`cargo install
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


def start(*args, tool=TOOL):
    """`<tool> -i <id> <args>` started and left running: the Popen, for a
    caller that acts on the bench while the tool drives."""
    return subprocess.Popen([tool, "-i", ID, *map(str, args)], text=True)


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


def write(addr, data):
    """`osc write`: raw bytes at a table address, committed."""
    run("write", f"0x{addr:x}", bytes(data).hex())


def read(addr, n):
    """`osc read`: `n` raw bytes from a table address, off its hex dump."""
    out = run("read", f"0x{addr:x}", n, capture=True)
    data = b"".join(bytes.fromhex(m.group(1)) for m in re.finditer(r"^0x[0-9a-f]+\s+(.+)$", out, re.M))
    if len(data) != n:
        raise RuntimeError(f"osc read 0x{addr:x} {n}: unexpected output {out!r}")
    return data


def status():
    return run("status", capture=True)


# The TEL row a static load wants, one chunk: vbus_raw, ntc_raw, current_raw,
# vmotor_a, vmotor_b, duty_ticks. The six are contiguous in the table and fill
# the 12-byte row, so the position, which a resistor has no use for, stays out
# and every stream carries the rail and the board temperature.
CHUNKS_RAW = "0x23c+12"
# the rail, then current_raw to current_trough: the crest and the trough of the shunt with the taps and the duty, no NTC
CHUNKS_TROUGH = "0x23c+2,0x240+10"

# The TEL row a rest stream wants: vref_raw, then the same six less the duty,
# which a rest has no use for. vref_raw is the ADC's own 1.2 V reference, so
# a rest stream carries VDD, which every other count is a share of.
CHUNKS_REST = "0x23a+12"

# The TEL row a servo capture wants: pos, then current_raw to current_trough,
# the crest and the trough of the shunt with the taps and the duty. The rail
# and the NTC stay out of the row; the runner polls them before each segment.
CHUNKS_SERVO = "0x238+2,0x240+10"


def sweep(out, duty, window_ms=150, baseline_ms=1000, rest_ms=500,
          dirs="both", decay="slow", tel_chunks=CHUNKS_RAW, static_load=True,
          stall=False, guard_lo=None, guard_hi=None, settle_ms=None,
          park=True, background=False):
    """`osc-lab sweep` into `out` (sweep.csv + meta.json). `duty` is the step
    list as the tool takes it, percent, with `then:`, `hold:`, `coast:` and
    `@MS` forms allowed. `stall` seeks the end stop in each direction and
    presses the rungs into it, `guard_lo` and `guard_hi` set the seek
    envelope in counts, `settle_ms` the still time before a rung, all as the
    tool takes them. `park` False leaves the shaft where the last rung left
    it, a stop press still wound, for the caller to center. `background`
    returns the running Popen instead of waiting."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    args = ["sweep", "--out", out, "--duty-pct", ",".join(map(str, duty)),
            "--window-ms", window_ms, "--baseline-ms", baseline_ms,
            "--rest-ms", rest_ms, "--dirs", dirs, "--decay", decay]
    if tel_chunks:
        args += ["--tel-chunks", tel_chunks]
    if static_load:
        args.append("--static-load")
    if stall:
        args.append("--stall")
    if not park:
        args.append("--no-park")
    for flag, value in (("--guard-lo", guard_lo), ("--guard-hi", guard_hi),
                        ("--settle-ms", settle_ms)):
        if value is not None:
            args += [flag, value]
    if background:
        return start(*args, tool=LAB)
    run(*args, tool=LAB)
    return out


def track(out, mode, schedule, tel_chunks=CHUNKS_SERVO, current_cap_ma=None,
          current_cap_ms=None, ntc_max_c=None, guard_lo=None, guard_hi=None,
          deadman_ms=None, deadman_check=False, center=True):
    """`osc-lab track` into `out` (track.csv, segments.csv + meta.json): the
    production image's `mode` loop (position, velocity or current) driven
    through `schedule`, the items as the tool takes them (`set:FIELD=VALUE`,
    `goal:VALUE@MS`, `hold@MS`, `rest@MS`), a list or one comma-joined
    string. The watchdog caps, the guard in counts and the dead-man as the
    tool takes them, None for the tool's default. `deadman_check` proves the
    image honors the dead-man first; `center` False leaves the shaft where
    the last item left it. A refusal (exit 2) or an abort (exit 3) raises."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    if not isinstance(schedule, str):
        schedule = ",".join(map(str, schedule))
    args = ["track", "--out", out, "--mode", mode, "--schedule", schedule,
            "--tel-chunks", tel_chunks]
    for flag, value in (("--current-cap-ma", current_cap_ma), ("--current-cap-ms", current_cap_ms),
                        ("--ntc-max-c", ntc_max_c), ("--guard-lo", guard_lo),
                        ("--guard-hi", guard_hi), ("--deadman-ms", deadman_ms)):
        if value is not None:
            args += [flag, value]
    if deadman_check:
        args.append("--deadman-check")
    if not center:
        args.append("--no-center")
    run(*args, tool=LAB)
    return out


def clapper(path, flashes):
    """`osc clapper`: `flashes` flashes of the STAT lamp, one CSV line per
    flash with the host time either side, into `path`."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        subprocess.run([TOOL, "-i", ID, "clapper", "--flashes", str(flashes)],
                       stdout=f, text=True, check=True)
    return path


def bursts(out, pct, repeats=3, chans="diff", dirs="both", rest_ms=150, decay="slow", spin_ms=0):
    """`osc-lab burst` into `out` (burst-K.csv + meta.json): the ADC free running
    on the shunt and the taps through PWM periods around a duty step from
    rest, each duty by sign by repeat, in `decay`. The raw instrument, no fit.
    `spin_ms` drives each step's own duty that long before its arm, so a free
    rotor is captured spinning in steady state on both sides of the step."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    args = ["burst", "--out", out, "--duty-pct", ",".join(map(str, pct)), "--repeats", repeats,
            "--chans", chans, "--dirs", dirs, "--rest-ms", rest_ms]
    # slow is the tool's default, so an osc-lab from before --decay still runs a slow set
    if decay != "slow":
        args += ["--decay", decay]
    if spin_ms:
        args += ["--spin-ms", spin_ms]
    run(*args, tool=LAB)
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
