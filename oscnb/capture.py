"""Shared pieces of the capture scripts.

A capture script lives next to its notebook, is run from anywhere as
`uv run python stages/<stage>/<experiment>/capture.py`, and writes one
dataset folder under `telemetry/`. Stage 0 has no servo and no board, so its
datasets are `telemetry/instruments/<experiment>/capture-N`; later stages
use `telemetry/<servo>__<board>/<experiment>/capture-N`.
"""
import csv
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def dataset(group, experiment, argv=None, default=1):
    """The dataset folder for this run, `telemetry/<group>/<experiment>/capture-N`.
    N is the first command line argument when given."""
    argv = sys.argv if argv is None else argv
    n = int(argv[1]) if len(argv) > 1 else default
    out = ROOT / "telemetry" / group / experiment / f"capture-{n}"
    if out.exists() and any(out.iterdir()):
        # a re-record replaces a dataset in place, but never by accident: name the folder first
        prompt(f"{out.relative_to(ROOT)} already holds a dataset and this run overwrites it.")
    out.mkdir(parents=True, exist_ok=True)
    return out


def prompt(text):
    """Wait for Enter. Used where the hands have to do something first."""
    input(text + " Enter when ready. ")


def hold(seconds, text=""):
    if text:
        print(f"{text} ({seconds:.0f} s)")
    time.sleep(seconds)


class Marks:
    """`marks.csv`: the host time of each named moment, such as the tip going
    into the ice water."""

    def __init__(self, out, append=False):
        self.path = Path(out) / "marks.csv"
        append = append and self.path.exists()
        self._f = open(self.path, "a" if append else "w", newline="")
        self._w = csv.writer(self._f)
        if not append:
            self._w.writerow(["t_host", "mark"])

    def mark(self, name):
        self._w.writerow([f"{time.time():.3f}", name])
        self._f.flush()

    def close(self):
        self._f.close()
