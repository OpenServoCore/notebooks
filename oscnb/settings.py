"""A rung of settings: CONFIG fields applied volatile, put back by a reboot.

A settings file is `{"fields": {name: value, ...}, "note": "..."}`. `apply`
needs torque off. It reads and keeps every field's old value, writes each
with `osc-bus set` and reads it back; a closed loop runs on the set as written.
Nothing is ever saved, so `restore` is `osc-bus reboot`: the servo comes back
on its saved images.

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
LOADED = "loaded"       # the settings word of a servo that booted on both of its saved images


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


def line(status, word):
    """What the line of an `osc-bus status` report that `word` leads says."""
    m = re.search(rf"^\s*{word}\s+(.+)$", status, re.M)
    if not m:
        raise RuntimeError(f"osc-bus status: no {word} line in {status!r}")
    return m.group(1).strip()


def settings(status):
    """The settings word of an `osc-bus status` report: "loaded", or the
    images on their defaults ("defaults: CONFIG, CALIB")."""
    return line(status, "settings")


def refused(status):
    """The modes an `osc-bus status` report says the servo refuses: "none"
    or their names, lowest first ("Current, Velocity, Position")."""
    return line(status, "refused")


def unsaved(status):
    """Whether an `osc-bus status` report says the servo runs what a reboot
    would not load (`unsaved true`)."""
    m = re.search(r"\bunsaved\s+(true|false)\b", status)
    if not m:
        raise RuntimeError(f"osc-bus status: no unsaved word in {status!r}")
    return m.group(1) == "true"


def apply(fields, note=""):
    """Write `fields` volatile; returns the old values. A refusal past the
    first write leaves the servo half set: `restore` puts it back."""
    torque = osc.get("torque_enable")
    if not same("off", torque):
        raise Refused(f"torque_enable reads {torque}: apply runs with torque off")
    old = {name: osc.get(name) for name in fields}
    for name, value in fields.items():
        osc.set_(name, value)
        back = osc.get(name)
        if not same(value, back):
            raise Refused(f"{name}: wrote {value}, read back {back}")
    if note:
        print(note)
    for name, value in fields.items():
        print(f"{name}: {old[name]} -> {value}")
    return old


def await_boot():
    """Wait for the bus to answer after a reboot, refused when it stays silent."""
    for _ in range(BOOT_TRIES):
        time.sleep(BOOT_WAIT_S)
        try:
            osc.bus_baud()
            return
        except RuntimeError:
            continue
    raise Refused(f"no answer {BOOT_TRIES * BOOT_WAIT_S:g} s after the reboot")


def restore():
    """Reboot onto the saved images and wait for the bus; returns the
    settings word, information."""
    osc.run("reboot")
    await_boot()
    word = settings(osc.status())
    print(f"rebooted: settings {word}")
    return word


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
    for cmd, doc in (("apply", "write the file's fields volatile, no SAVE"),
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
