"""A sitting: every plan that wants one bench state, run in one go.

The hands set the bench once per state and the runner does the rest. The
states are the rotor free, the rotor free with the meter in series on the
MOT_A side, and the rotor locked by the wrench bar, which the runner walks
through the twelve holes of the rig's bolt circle. A notebook that captures
in a state declares its plans in a `plan.py` beside it, each with the
rungs it wants as parameters, and writes its dataset where the notebook
reads it, so a notebook never knows which sitting produced its data.

  uv run python -m oscnb.sitting free [N] [--only a,b] [--skip a,b] [--include a,b]
  uv run python -m oscnb.sitting meter [N] ...
  uv run python -m oscnb.sitting locked [N] ...
  uv run python -m oscnb.sitting list

N numbers every dataset of the sitting, `<name>-N`, and its manifest,
`telemetry/<motor>__<board>/sittings/<state>-N.csv`, one row per prompt,
rail trim, dataset, hole and check with its host time, the record of what
each capture ran under. OSC_MOTOR picks the motor as before.

The rules every plan runs under live here and not in the plans: no spin
puts more than the motor's rated voltage on it as rail times duty, unless
the plan is marked `over_rating`, the sacrificial exception that runs last
and only when named; the motor rests REST_S torque-off between cycles and
between holes; a `bracket` plan runs at the open and the close of the
sitting as `<name>-open-N` and `<name>-close-N`, so a drift through the
sitting shows against the same rung; an `optional` plan runs only when
named by --include. The meter state refuses to start unless a short forward
spin reads positive on the meter, since the notebooks pair the meter's
spins with the board's by that sign, and the locked state refuses unless a
lap into the rotor draws a stall current.
"""
import argparse
import csv
import importlib.util
import sys
import time
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path

from . import b41t, bench, boards, capture, motors, osc, tel

STATES = ("free", "meter", "locked")
REST_S = 20                   # torque-off between cycles and between holes, the bench rule for a bare motor
HOLES = list(range(12))       # the rig's bolt circle, 30 degrees apart
POLARITY = (5.0, 30, 1000)    # the meter check: rail, duty, ms of one forward spin
STALL = (3.3, 300, 300)       # the locked check: rail, lap ms, counts over the zero a locked rotor draws (a free one at 100 % stays under half)
ENTRY = {
    "free": "Motor leads on the board's motor connector, the bar OFF the pinion, the rotor free, no meter.",
    "meter": "Bar OFF, rotor free. DMM on DC mA held on the 220.00 mA range with RANGE, red probe in the mA jack, in series "
             "on the MOT_A side with its red lead toward MOT_A: MOT_A -> meter -> motor lead -> motor -> other lead -> MOT_B. "
             "Bluetooth on.",
    "locked": f"Motor leads on the board's motor connector, the board on the DPS. Bar on the pinion at hole {HOLES[0]}, clamp snug.",
}


@dataclass
class Plan:
    """One experiment's capture in one state. `run(sitting, out)` for the free
    and meter states. In the locked state `setup(sitting, out)` once,
    `at_hole(sitting, out, k)` at every hole, `tail(sitting, out)` after the
    last hole with the bar still in it, `finish(sitting, out)` at the end."""

    name: str             # the dataset's name, `<name>-N`
    experiment: str       # the telemetry folder, the notebook's name
    state: str
    doc: str
    run: object = None
    setup: object = None
    at_hole: object = None
    tail: object = None
    finish: object = None
    order: int = 50       # within the state, low first
    bracket: bool = False
    optional: bool = False
    over_rating: bool = False

    def __post_init__(self):
        if self.state not in STATES:
            raise ValueError(f"{self.name}: state {self.state!r} is not one of {STATES}")
        if self.over_rating:
            self.optional = True
            self.order += 1000


class Sitting:
    """The bench through one sitting: the rail, the DPS, the meter when the
    state has one, and the manifest."""

    def __init__(self, state, n, motor=None, board=None, root=None):
        self.state, self.n = state, n
        self.motor = motor or motors.selected()
        self.board = board or boards.BOARDS[boards.DEFAULT]
        self.root = Path(root) if root else capture.ROOT
        self.group = f"{self.motor.key}__{self.board.key}"
        self.meter = None
        self.level = None
        self.rail_read = None
        self.powered = False
        self._f = self._w = None

    # --- the manifest ---
    def open(self):
        folder = self.root / "telemetry" / self.group / "sittings"
        folder.mkdir(parents=True, exist_ok=True)
        self._f = open(folder / f"{self.state}-{self.n}.csv", "w", newline="")
        self._w = csv.writer(self._f, lineterminator="\n")
        self._w.writerow(["t_host", "event", "detail"])
        self.note("start", f"motor {self.motor.key}, board {self.board.key}, state {self.state}")

    def close(self):
        self.note("end")
        self._f.close()

    def note(self, event, detail=""):
        self._w.writerow([f"{time.time():.3f}", event, detail])
        self._f.flush()
        print(f"[{event}] {detail}" if detail else f"[{event}]")

    def prompt(self, text):
        capture.prompt(text)
        self.note("prompt", text)

    def dataset(self, plan, suffix=""):
        out = capture.dataset(self.group, plan.experiment, ["", str(self.n)], name=plan.name + suffix)
        self.note("dataset", f"{plan.name}{suffix}: {out.relative_to(capture.ROOT)}")
        return out

    # --- the bench ---
    def rail(self, volts):
        """The rail at `volts`, the DPS powered up on the first call. Returns
        (DPS volts read, rail read) as the trim left them."""
        if not self.powered:
            bench.power_up(volts + bench.DIODE_V)
            self.powered = True
            self.note("dps", f"on at {volts + bench.DIODE_V:.2f} V")
        if volts != self.level:
            dps_read, vsys = bench.set_vsys(volts)
            self.level, self.rail_read = volts, (dps_read, vsys)
            self.note("rail", f"{volts} V: reads {vsys:.3f} V, DPS {dps_read:.3f} V")
        return self.rail_read

    def power_down(self):
        if self.powered:
            bench.power_down()
            self.powered, self.level = False, None
            self.note("dps", "off")

    def rest(self, text=""):
        capture.hold(REST_S, text)

    def cap(self, volts, duty, over_rating=False):
        if not over_rating and volts * duty / 100 > self.motor.rated_v:
            raise SystemExit(f"{volts} V at {duty} % is {volts * duty / 100:.2f} V effective, over the motor's rated {self.motor.rated_v} V")

    def meter_log(self, out):
        """The meter's log into `out/dmm.csv` while a plan runs, nothing when
        the state has no meter."""
        return self.meter.log(out / "dmm.csv", expect="DC A") if self.meter else nullcontext()

    def cycles(self, out, rungs, spin_ms, coast_ms, repeats=3, folder="cycles", kick=None, over_rating=False):
        """Run `rungs`, (rail volts, duty %) pairs, `repeats` cycles each, into
        `out/<folder>/K` with one row per cycle in `out/<folder>.csv`. A cycle is
        a spin and a coast in both directions as one sweep. With `kick`, duty by
        rail, each spin opens with KICK_MS at that duty and drops to the rung's
        duty chained, no torque-off between."""
        with open(out / f"{folder}.csv", "w", newline="") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(["k", "volts_set", "dps_read", "vsys_read", "duty_pct", "repeat", "t_start", "t_end", "folder", "kick_pct"])
            k = 0
            for v, d in rungs:
                self.cap(v, max(d, kick[v] if kick else 0), over_rating)
                dps_read, vsys = self.rail(v)
                for r in range(repeats):
                    sub = f"{folder}/{k}"
                    t0 = time.time()
                    drive = [f"{kick[v]}@{KICK_MS}", f"then:{d}@{spin_ms}"] if kick else [f"{d}@{spin_ms}"]
                    osc.sweep(out / sub, drive + [f"coast:{coast_ms}"], baseline_ms=500, rest_ms=REST_S * 1000, dirs="both")
                    w.writerow([k, v, f"{dps_read:.3f}", f"{vsys:.3f}", d, r, f"{t0:.3f}", f"{time.time():.3f}", sub, kick[v] if kick else 0])
                    f.flush()
                    k += 1
                    self.rest(f"cycle {k - 1} done ({v} V, {d} %), the motor resting")


KICK_MS = 300                 # a kicked spin's opening, long enough to reach the kick's speed


# --- the checks at a state's entry ---
def polarity_check(s):
    """One forward spin with the meter logging: the meter must read it
    positive, the sign the notebooks pair the meter's spins with the board's by."""
    v, d, ms = POLARITY
    s.cap(v, d)
    s.rail(v)
    out = s.root / "telemetry" / s.group / "sittings" / f"{s.state}-{s.n}-polarity"
    with s.meter.log(out / "dmm.csv", expect="DC A"):
        osc.sweep(out, [f"{d}@{ms}", "coast:300"], baseline_ms=500, rest_ms=500, dirs="fwd")
    with open(out / "dmm.csv", newline="") as f:
        amps = [float(r["value"]) for r in csv.DictReader(f) if abs(float(r["value"])) > 2e-3]
    if not amps:
        raise SystemExit("the meter logged no current through the forward spin: is it in the loop, on DC mA, Bluetooth on?")
    mean = sum(amps) / len(amps)
    if mean <= 0:
        raise SystemExit(f"the meter reads the forward spin at {mean * 1e3:+.1f} mA: turn the meter around, red toward MOT_A")
    s.note("check", f"meter polarity: the forward spin reads {mean * 1e3:+.1f} mA")
    s.rest("the check's spin done, the motor resting")


def stall_check(s):
    """One lap into the rotor: it must draw a stall current, or the bar is
    not holding."""
    v, ms, counts = STALL
    s.rail(v)
    out = s.root / "telemetry" / s.group / "sittings" / f"{s.state}-{s.n}-stall"
    osc.sweep(out, [f"100@{ms}"], baseline_ms=500, rest_ms=500, dirs="fwd")
    df, _ = tel.load(out, s.board)
    drawn = float(df[df["seg"] > 0]["current_raw"].mean() - tel.baseline(df)["current_raw"].median())
    if drawn < counts:
        raise SystemExit(f"the lap drew {drawn:.0f} counts over the zero, under the {counts} a locked rotor draws: is the bar clamped?")
    s.note("check", f"locked rotor: the lap drew {drawn:.0f} counts over the zero")
    s.rest("the check's lap done, the copper resting")


# --- the plans ---
def discover(stages=None):
    """Every plan declared in a `stages/*/*/plan.py`, in path order."""
    stages = Path(stages) if stages else capture.ROOT / "stages"
    plans = []
    for p in sorted(stages.glob("*/*/plan.py")):
        spec = importlib.util.spec_from_file_location(f"plan_{p.parent.name.replace('-', '_')}", p)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        plans.extend(mod.PLANS)
    keys = [(p.experiment, p.name, p.state) for p in plans]
    if len(keys) != len(set(keys)):
        raise SystemExit("two plans of one experiment share a name and a state")
    return plans


def named(p, tokens):
    """Whether a plan is among `tokens`, each a name, shared by the main
    `capture` of every experiment, or `experiment/name` for one of them."""
    return p.name in tokens or f"{p.experiment}/{p.name}" in tokens


def select(plans, state, only=(), skip=(), include=()):
    """The plans of `state` in running order: by `order`, the optional ones
    only when named by `include`, `only` and `skip` by name."""
    in_state = [p for p in plans if p.state == state]
    for t in (*only, *skip, *include):
        if not any(named(p, (t,)) for p in in_state):
            raise SystemExit(f"no plan {t!r} in state {state}; the plans are "
                             f"{', '.join(sorted(f'{p.experiment}/{p.name}' for p in in_state))}")
    chosen = [p for p in in_state if not named(p, skip) and (not p.optional or named(p, include))
              and (not only or named(p, only) or p.bracket)]
    return sorted(chosen, key=lambda p: p.order)


def run_free(s, plans):
    s.prompt(ENTRY[s.state])
    brackets = [p for p in plans if p.bracket]
    body = [p for p in plans if not p.bracket]
    with (b41t.Meter() if s.state == "meter" else nullcontext()) as meter:
        s.meter = meter
        if meter:
            polarity_check(s)
        for p in brackets:
            s.note("plan", f"{p.name} (open)")
            p.run(s, s.dataset(p, "-open"))
        for p in body:
            s.note("plan", p.name)
            p.run(s, s.dataset(p))
        for p in brackets:
            s.note("plan", f"{p.name} (close)")
            p.run(s, s.dataset(p, "-close"))


def run_locked(s, plans):
    s.prompt(ENTRY[s.state])
    stall_check(s)
    outs = {p.name: s.dataset(p) for p in plans}
    for p in plans:
        if p.setup:
            p.setup(s, outs[p.name])
    for k in HOLES:
        if k != HOLES[0]:
            s.prompt(f"Pins out, bar turned to hole {k}, pins in.")
        s.note("hole", str(k))
        for p in plans:
            if p.at_hole:
                p.at_hole(s, outs[p.name], k)
        s.rest(f"hole {k} done, the copper resting")
    for p in plans:
        if p.tail:
            s.note("tail", p.name)
            p.tail(s, outs[p.name])
    for p in plans:
        if p.finish:
            p.finish(s, outs[p.name])


def run(state, n, only=(), skip=(), include=(), plans=None):
    plans = select(plans if plans is not None else discover(), state, only, skip, include)
    if not plans:
        raise SystemExit(f"no plan runs in state {state} with that selection")
    s = Sitting(state, n)
    print("motor", s.motor.key, "board", s.board.key, "state", state, "plans", ", ".join(p.name for p in plans))
    s.open()
    try:
        (run_locked if state == "locked" else run_free)(s, plans)
    finally:
        s.power_down()
        s.close()
    print("wrote", s.root / "telemetry" / s.group / "sittings" / f"{state}-{n}.csv")


def listing(plans):
    for state in STATES:
        print(state)
        for p in sorted((p for p in plans if p.state == state), key=lambda p: p.order):
            flags = ", ".join(f for f, on in (("bracket", p.bracket), ("optional", p.optional), ("over rating", p.over_rating)) if on)
            print(f"  {p.name:14s} {p.experiment:16s} {p.doc}" + (f" [{flags}]" if flags else ""))


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.sitting", description="run every plan of one bench state")
    ap.add_argument("state", choices=(*STATES, "list"))
    ap.add_argument("n", nargs="?", type=int, default=1, help="the sitting's number, N in every <name>-N it writes")
    split = lambda a: tuple(x for x in a.split(",") if x)
    ap.add_argument("--only", type=split, default=(), help="run these plans alone, by name (bracket plans still run)")
    ap.add_argument("--skip", type=split, default=(), help="leave these plans out")
    ap.add_argument("--include", type=split, default=(), help="run these optional plans too")
    a = ap.parse_args(argv)
    if a.state == "list":
        listing(discover())
        return
    run(a.state, a.n, a.only, a.skip, a.include)


if __name__ == "__main__":
    main()
