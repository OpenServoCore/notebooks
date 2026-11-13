"""A rung of settings: CONFIG fields applied volatile, put back by a reboot.

A settings file is `{"fields": {name: value, ...}, "note": "..."}`. `apply`
needs torque off. It reads and keeps every field's old value, writes each
with `osc set` and reads it back, then blesses the set with `osc stamp`
without `--save`, so a closed loop may start on it, and checks that
`osc status` reads the data state clean. Nothing is ever saved, so
`restore` is `osc reboot`: the servo comes back on its saved images.

    uv run python -m oscnb.settings apply <file.json>
    uv run python -m oscnb.settings show <file.json>
    uv run python -m oscnb.settings restore
"""
import argparse
import json
import re
import time
from pathlib import Path

from . import osc

BOOT_WAIT_S = 0.5       # s, before the first ping after a reboot
BOOT_TRIES = 10         # pings, BOOT_WAIT_S apart, before giving up


class Refused(RuntimeError):
    """The servo is not in a state the step may run in, or a write did not
    land; the message says which."""


def load(path):
    """(fields, note) of a settings file."""
    doc = json.loads(Path(path).read_text())
    fields = doc.get("fields")
    if not isinstance(fields, dict) or not fields:
        raise ValueError(f"{path}: no fields")
    return fields, doc.get("note", "")


def same(want, got):
    """Whether a read-back `got` is the value `want` written: as integers
    when both read as one, else as the tool's words (on/off, a variant)."""
    if isinstance(want, bool):
        want = "on" if want else "off"
    try:
        return int(str(want), 0) == int(str(got), 0)
    except ValueError:
        return str(want).lower() == str(got).lower()


def data_state(status):
    """The data state line of an `osc status` report: "clean" or the
    reasons' names."""
    m = re.search(r"^\s*data\s+(.+)$", status, re.M)
    if not m:
        raise RuntimeError(f"osc status: no data line in {status!r}")
    return m.group(1).strip()


def plant_stamp(status):
    """The stamp line of an `osc status` report, after "stamp"."""
    m = re.search(r"^\s*stamp\s+(.+)$", status, re.M)
    return m.group(1).strip() if m else None


def require_clean():
    """The `osc status` report, refused unless its data state is clean."""
    status = osc.status()
    state = data_state(status)
    if state != "clean":
        raise Refused(f"data state {state}: closed loop is refused")
    return status


def apply(fields, note=""):
    """Write `fields` volatile and stamp them; returns the old values. A
    refusal past the first write leaves the servo half set: `restore` puts
    it back."""
    torque = osc.get("torque_enable")
    if not same("off", torque):
        raise Refused(f"torque_enable reads {torque}: apply runs with torque off")
    old = {name: osc.get(name) for name in fields}
    for name, value in fields.items():
        osc.set_(name, value)
        back = osc.get(name)
        if not same(value, back):
            raise Refused(f"{name}: wrote {value}, read back {back}")
    osc.run("stamp")
    require_clean()
    if note:
        print(note)
    for name, value in fields.items():
        print(f"{name}: {old[name]} -> {value}")
    return old


def restore():
    """Reboot onto the saved images, wait for the bus, check the data state
    is clean; returns the stamp line."""
    osc.run("reboot")
    for _ in range(BOOT_TRIES):
        time.sleep(BOOT_WAIT_S)
        try:
            osc.bus_baud()
            break
        except RuntimeError:
            continue
    else:
        raise Refused(f"no answer {BOOT_TRIES * BOOT_WAIT_S:g} s after the reboot")
    stamp = plant_stamp(require_clean())
    print(f"plant stamp {stamp}")
    return stamp


def show(fields):
    """The servo's current value of every field named; returns them."""
    now = {name: osc.get(name) for name in fields}
    for name, value in now.items():
        print(f"{name} = {value}")
    return now


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.settings",
                                 description="apply a set of CONFIG fields volatile, or reboot it away")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for cmd, doc in (("apply", "write and stamp the file's fields, no SAVE"),
                     ("show", "the current values of the file's fields")):
        sub.add_parser(cmd, help=doc).add_argument("file", type=Path)
    sub.add_parser("restore", help="reboot onto the saved images")
    a = ap.parse_args(argv)
    try:
        if a.cmd == "restore":
            restore()
            return
        fields, note = load(a.file)
        if a.cmd == "apply":
            apply(fields, note)
        else:
            show(fields)
    except Refused as e:
        raise SystemExit(f"refused: {e}")


if __name__ == "__main__":
    main()
