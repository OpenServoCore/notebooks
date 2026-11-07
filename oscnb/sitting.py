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
  uv run python -m oscnb.sitting servo [N] ...
  uv run python -m oscnb.sitting servo-camera [N] ...
  uv run python -m oscnb.sitting servo-load [N] ...
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

The servo states are the assembled servo of stage 3, OSC_SERVO picking it,
its datasets under `telemetry/<servo>__<board>/`. `servo` is the servo on
the board and the DPS with no instrument, `servo-camera` the servo in the
printed rig under the phone, `servo-load` the rig on its side with the arm
and its mass on the horn. The horn lock is no state: a plan marked `lock`
runs with the horn held at mid travel, and the runner centers the horn and
asks for the lock on before the first such plan and off before the next
free one. On top of the rated voltage, a lap into the horn lock draws at
most the servo's lock hold, any other held stall at most its hold current
and a lap into an end stop at most its lap cap, as rail times duty over the
type resistance, refused before anything moves.
Every sweep and burst set is preceded by `pos`, `ntc_raw` and `vbus_raw`
read into the manifest. A free rung's window is sized from a pilot spin so
its travel at a bound on its speed, the coast included, ends RUNWAY_MARGIN
counts short of the far guard, since nothing stops a rung that outruns its
window but the end stop. A seat that moves past SEAT_SLIP_COUNTS skips the
sitting's remaining seated plans.
"""
import argparse
import csv
import functools
import importlib.util
import sys
import time
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import b41t, bench, boards, capture, motors, osc, results, servos, tel

SERVO_STATES = ("servo", "servo-camera", "servo-load")
STATES = ("free", "meter", "locked", *SERVO_STATES)
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
    "servo": "The servo's motor and pot leads on the board, the board on "
             "the DPS, the adapter on the bus, no instrument. The horn free "
             "and the lock off, the lock to hand.",
    "servo-camera": "The servo in the printed rig with the tag horn, the "
                    "reference pad beside it, the board on the DPS, the "
                    "phone overhead in SLO-MO at 240 fps with the tags and "
                    "the STAT lamp in view, nothing in the swept circle.",
    "servo-load": "The rig on its side with the shaft horizontal, the arm "
                  "on the horn with the mass at its hole and the hub tag on "
                  "top, the board on the DPS, nothing in the arm's swing.",
}
LOCK_ON = ("The horn is at mid travel: put the horn lock on without "
           "turning the horn.")
LOCK_OFF = "Take the horn lock off; the horn free."

# --- the assembled servo ---
PROBES = ("pos", "ntc_raw", "vbus_raw")   # read into the manifest per segment
GUARD_INSET = 100     # counts inside the soft limits, the seek envelope
BAND_HALF = 75        # osc-lab's start band, a rung opens this far past it
RUNWAY_MARGIN = 300   # counts short of the far guard a cut lands
# The spin-down after a cut as speed x time: a bare MG90 rotor coasting
# from full speed covers what 93 ms at that speed would; a train only
# shortens it.
COAST_TAU_S = 0.1
PILOT = (5.0, 30, 300)    # rail, duty, ms: the spin every window scales from
PILOT_COAST_MS = 300
PILOT_MIN_MS = 150    # the pilot's settled last third still holds 50 ms
# The highest effective volts a servo's speed line can cross zero at; a
# floor above it makes a rung over the pilot faster than its bound. Twice
# the bare motor's breakaway, for the train's drag.
FLOOR_V = 1.0
KE_MARGIN = 0.9       # the bound's Ke as a share of the type's: a faster motor
SPEED_MARGIN = 1.2    # over the pilot's read, for its noise and the climb
LOAD_ASSIST = 1.5     # the arm's mass drives the lowering side past free speed
WINDOW_MIN_MS = 30    # a rung the runway cuts under this reads nothing, skipped
SPEED_MIN_ROWS = 200  # rows a speed line needs, 10 ms at the tick rate
SEAT_SLIP_COUNTS = 10     # a held seat that moves this far ends the seated work
MID_TOL = 100         # counts from mid travel the lock goes on within
CENTER_MS = 50        # the centering sweep's one still segment
CLAPPER_FLASHES = 12  # at a clip's start, as many as clock-sync fitted


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
    lock: bool = False    # runs with the horn lock on, the servo state only
    seated: bool = False  # presses a held seat; a moved seat skips it
    minutes: float = 0    # the bench time it takes, about
    mb: float = 0         # the dataset's size, about

    def __post_init__(self):
        if self.state not in STATES:
            raise ValueError(f"{self.name}: state {self.state!r} is not one of {STATES}")
        if self.over_rating:
            self.optional = True
            self.order += 1000
        if self.lock and self.state != "servo":
            raise ValueError(f"{self.name}: the horn lock is a servo state "
                             f"prompt, not {self.state!r}")
        if self.lock:
            self.seated = True


class Skip(Exception):
    """A plan that cannot run as asked here: the runner notes why and goes
    on with the next."""


class SeatMoved(Skip):
    """A held seat moved past SEAT_SLIP_COUNTS: the sitting's remaining
    seated plans are skipped."""


class Sitting:
    """The bench through one sitting: the rail, the DPS, the meter when the
    state has one, and the manifest."""

    def __init__(self, state, n, motor=None, board=None, root=None,
                 servo=None):
        self.state, self.n = state, n
        if state in SERVO_STATES:
            self.servo = servo or servos.selected()
            self.motor = motor or self.servo.motor_entry
            unit = self.servo.key
        else:
            self.servo = None
            self.motor = motor or motors.selected()
            unit = self.motor.key
        self.board = board or boards.BOARDS[boards.DEFAULT]
        self.root = Path(root) if root else capture.ROOT
        self.group = f"{unit}__{self.board.key}"
        self.locked = False
        self.seat_moved = False
        self.found = {}       # what one plan reads for the next, by name
        self._guards = None
        self._speed = None
        self._seats = {}
        self._centers = 0
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
        unit = f"servo {self.servo.key}, " if self.servo else ""
        self.note("start", f"{unit}motor {self.motor.key}, "
                           f"board {self.board.key}, state {self.state}")

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

    @property
    def rated_v(self):
        return self.servo.rated_v if self.servo else self.motor.rated_v

    def cap(self, volts, duty, over_rating=False):
        if not over_rating and volts * duty / 100 > self.rated_v:
            raise SystemExit(f"{volts} V at {duty} % is "
                             f"{volts * duty / 100:.2f} V effective, over the "
                             f"rated {self.rated_v} V")

    def meter_log(self, out):
        """The meter's log into `out/dmm.csv` while a plan runs, nothing when
        the state has no meter."""
        return self.meter.log(out / "dmm.csv", expect="DC A") if self.meter else nullcontext()

    def cycles(self, out, rungs, spin_ms, coast_ms, repeats=3, folder="cycles", kick=None, over_rating=False, tel_chunks=None):
        """Run `rungs`, (rail volts, duty %) pairs, `repeats` cycles each, into
        `out/<folder>/K` with one row per cycle in `out/<folder>.csv`. A cycle is
        a spin and a coast in both directions as one sweep. With `kick`, duty by
        rail, each spin opens with KICK_MS at that duty and drops to the rung's
        duty chained, no torque-off between. `tel_chunks` picks the TEL row,
        the R/L set unless given."""
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
                    osc.sweep(out / sub, drive + [f"coast:{coast_ms}"], baseline_ms=500, rest_ms=REST_S * 1000, dirs="both",
                              **({"tel_chunks": tel_chunks} if tel_chunks else {}))
                    w.writerow([k, v, f"{dps_read:.3f}", f"{vsys:.3f}", d, r, f"{t0:.3f}", f"{time.time():.3f}", sub, kick[v] if kick else 0])
                    f.flush()
                    k += 1
                    self.rest(f"cycle {k - 1} done ({v} V, {d} %), the motor resting")

    # --- the assembled servo ---
    @functools.cached_property
    def r_type(self):
        """The type motor's terminal resistance, the caps' ohms."""
        doc = results.read("2-motor", "rest-resistance",
                           pair=self.servo.type_motor)
        return doc["r_term_median_ohm"]["value"]

    @functools.cached_property
    def free_per_v(self):
        """Counts/s per effective volt the motor could turn at with no drag
        and a Ke KE_MARGIN under the type's: the speed no rung can pass."""
        doc = results.read("2-motor", "bemf-coast", pair=self.servo.type_motor)
        ke_v_per_rev_s = doc["ke_mv_per_rev_s"]["value"] / 1e3
        assist = LOAD_ASSIST if self.state == "servo-load" else 1.0
        return (self.servo.counts_per_motor_rev * assist
                / (ke_v_per_rev_s * KE_MARGIN))

    def press(self, volts, duty, cap_a, what):
        """Refuse a pressed drive whose stall current, rail times duty over
        the type resistance, passes `cap_a`."""
        amps = volts * duty / 100 / self.r_type
        if amps > cap_a:
            raise SystemExit(f"{what} at {volts} V and {duty} % draws "
                             f"{amps:.2f} A over {self.r_type:.2f} ohm, "
                             f"over the {cap_a} A it may")

    def check(self, volts, steps, seat=None):
        """Every drive step of a schedule under the rated voltage, and under
        the lock hold (`seat` "lock"), the hold current of any other held
        stall ("hold") or the lap cap ("stop")."""
        for duty in step_duties(steps):
            self.cap(volts, duty)
            if seat == "lock":
                self.press(volts, duty, self.servo.lock_hold_a, "a locked lap")
            elif seat == "hold":
                self.press(volts, duty, self.servo.stop_hold_a, "a held stall")
            elif seat == "stop":
                self.press(volts, duty, self.servo.lap_cap_a,
                           "a lap into the stop")

    def probe(self, label):
        """PROBES read from the servo into the manifest, before a segment."""
        got = {f: osc.get(f) for f in PROBES}
        detail = ", ".join(f"{k} {v}" for k, v in got.items())
        self.note("probe", f"{label}: {detail}")
        return got

    def guards(self):
        """The seek envelope, GUARD_INSET inside the servo's soft limits,
        read once a sitting."""
        if self._guards is None:
            lo = osc.get("pos_min_soft_counts") + GUARD_INSET
            hi = osc.get("pos_max_soft_counts") - GUARD_INSET
            if hi - lo <= 2 * (BAND_HALF + RUNWAY_MARGIN):
                raise SystemExit(f"the guards {lo} to {hi} leave no runway: "
                                 "are the soft limits set?")
            self._guards = (lo, hi)
            self.note("guards", f"{lo} to {hi} counts")
        return self._guards

    def narrow(self, lo, hi):
        """The seek envelope for the rest of the sitting narrowed to `lo` to
        `hi` counts, the pilot's and every default sweep's."""
        glo, ghi = self.guards()
        lo, hi = max(lo, glo), min(hi, ghi)
        if (lo, hi) != (glo, ghi):
            self._guards = (lo, hi)
            self.note("guards", f"narrowed to {lo} to {hi} counts")
        return self._guards

    def runway(self, guards=None):
        """Counts a rung from one guard may travel toward the other."""
        lo, hi = guards or self.guards()
        return hi - lo - BAND_HALF - RUNWAY_MARGIN

    def label(self, out):
        out = Path(out)
        try:
            return str(out.relative_to(self.root))
        except ValueError:
            return str(out)

    def drive(self, out, volts, steps, dirs="both", seat=None, guards=None,
              rest_ms=REST_S * 1000, baseline_ms=500, background=False):
        """One `osc-lab sweep` on the servo: the schedule checked, the rail
        set, the probes read, the servo's TEL row. `seat` "lock" drives the
        locked horn with no seek, "stop" presses into the end stops, None
        is a free rotor from the guards. `background` returns the running
        sweep instead of waiting."""
        self.check(volts, steps, seat)
        self.rail(volts)
        self.probe(self.label(out))
        if seat == "lock":
            kw = {"static_load": True}
        else:
            lo, hi = guards or self.guards()
            kw = {"static_load": False, "stall": seat == "stop",
                  "guard_lo": lo, "guard_hi": hi}
        return osc.sweep(out, steps, baseline_ms=baseline_ms, rest_ms=rest_ms,
                         dirs=dirs, tel_chunks=osc.CHUNKS_SERVO,
                         background=background, **kw)

    def bursts(self, out, volts, pct, dirs, repeats, seat="lock",
               rest_ms=150):
        """One `osc-lab burst` set into a held seat, checked as a pressed
        drive, the probes read first."""
        self.check(volts, [str(d) for d in pct], seat)
        self.rail(volts)
        self.probe(self.label(out))
        return osc.bursts(out, pct, repeats=repeats, chans="diff", dirs=dirs,
                          rest_ms=rest_ms)

    def sittings_dir(self, what):
        return (self.root / "telemetry" / self.group / "sittings"
                / f"{self.state}-{self.n}-{what}")

    def speed(self):
        """The pilot's speed, counts/s, the faster direction, run once a
        sitting before the first free rung, its window cut so even the
        drag-free motor stays on the runway."""
        if self._speed is None:
            volts, duty, want = PILOT
            out = self.sittings_dir("pilot")
            reach = self.free_per_v * volts * duty / 100
            ms = min(want, int(1000 * (self.runway() / reach - COAST_TAU_S)))
            if ms < PILOT_MIN_MS:
                raise SystemExit("the runway leaves the pilot itself "
                                 f"{ms} ms, under {PILOT_MIN_MS}")
            self.drive(out, volts, [f"{duty}@{ms}", f"coast:{PILOT_COAST_MS}"])
            self._speed = drive_speed(out)
            self.note("pilot", f"{volts} V {duty} % for {ms} ms: "
                               f"{self._speed:.0f} counts/s")
            self.rest("the pilot done, the motor resting")
        return self._speed

    def speed_bound(self, v_eff):
        """A speed no rung at `v_eff` effective volts passes, counts/s: the
        pilot's through zero below it, through FLOOR_V above it, and never
        over the drag-free motor."""
        v_pilot = PILOT[0] * PILOT[1] / 100
        pilot = self.speed() * SPEED_MARGIN
        line = pilot * v_eff / v_pilot
        floor = pilot * (v_eff - FLOOR_V) / (v_pilot - FLOOR_V)
        return min(max(line, floor), self.free_per_v * v_eff)

    def fit(self, volts, segments, fixed=(), runway=None):
        """Windows for one chain: `segments`, (duty, wanted ms), scaled by one
        factor so the chain's travel at the bound speed, with the `fixed`
        (duty, ms) segments and the spin-down from the fastest, stays on
        the runway. Skip when a window would fall under WINDOW_MIN_MS."""
        room = self.runway() if runway is None else runway
        fastest = 0.0
        for duty, ms in fixed:
            speed = self.speed_bound(volts * abs(duty) / 100)
            room -= speed * ms / 1000
            fastest = max(fastest, speed)
        travel = 0.0
        for duty, ms in segments:
            speed = self.speed_bound(volts * abs(duty) / 100)
            travel += speed * ms / 1000
            fastest = max(fastest, speed)
        room -= fastest * COAST_TAU_S
        scale = min(1.0, room / travel) if travel > 0 and room > 0 else 0.0
        windows = [int(ms * scale) for _, ms in segments]
        if min(windows) < WINDOW_MIN_MS:
            raise Skip(f"{volts} V {[d for d, _ in segments]} %: the runway "
                       f"leaves {min(windows)} ms, under {WINDOW_MIN_MS}")
        return windows

    def servo_cycles(self, out, rungs, want_ms, coast_ms, repeats=3,
                     folder="cycles", kick=None, kick_ms=0, guards=None):
        """`cycles` on the servo: (rail, duty) rungs, every one checked
        before the first moves, each spin's window sized from the pilot
        under `want_ms` (ms, or ms by duty), `kick` a duty by rail opening
        each spin for `kick_ms`, `guards` the envelope when not the
        servo's. A rung the runway cuts too short is noted and left out."""
        def kick_of(v):
            return kick[v] if kick else 0

        for v, d in rungs:
            self.check(v, [str(d), str(kick_of(v))])
        with open(out / f"{folder}.csv", "w", newline="") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(["k", "volts_set", "dps_read", "vsys_read", "duty_pct",
                        "repeat", "t_start", "t_end", "folder", "kick_pct",
                        "kick_ms", "spin_ms"])
            k = 0
            for v, d in rungs:
                want = want_ms[d] if isinstance(want_ms, dict) else want_ms
                fixed = [(kick_of(v), kick_ms)] if kick else []
                try:
                    (ms,) = self.fit(v, [(d, want)], fixed,
                                     self.runway(guards))
                except Skip as e:
                    self.note("skip", f"rung {v} V {d} %: {e}")
                    continue
                if kick:
                    steps = [f"{kick_of(v)}@{kick_ms}", f"then:{d}@{ms}"]
                else:
                    steps = [f"{d}@{ms}"]
                for r in range(repeats):
                    sub = f"{folder}/{k}"
                    t0 = time.time()
                    self.drive(out / sub, v, steps + [f"coast:{coast_ms}"],
                               guards=guards)
                    dps_read, vsys = self.rail_read
                    w.writerow([k, v, f"{dps_read:.3f}", f"{vsys:.3f}", d, r,
                                f"{t0:.3f}", f"{time.time():.3f}", sub,
                                kick_of(v), kick_ms if kick else 0, ms])
                    f.flush()
                    k += 1
                    self.rest(f"cycle {k - 1} done ({v} V, {d} %), resting")

    def seat(self, out, key, pos):
        """One seat reading into `out/seats.csv`; SeatMoved when it sits
        more than SEAT_SLIP_COUNTS from the sitting's first reading of
        `key`, the lock's from when it last went on."""
        first = self._seats.setdefault(key, pos)
        shift = pos - first
        log_row(Path(out) / "seats.csv", {"t_host": f"{time.time():.3f}",
                                          "seat": key, "pos": pos,
                                          "shift_counts": shift})
        if abs(shift) > SEAT_SLIP_COUNTS:
            self.note("seat", f"{key} moved {shift:+} counts")
            raise SeatMoved(f"the {key} seat moved {shift:+} counts, over "
                            f"{SEAT_SLIP_COUNTS}")
        return shift

    def center(self):
        """The horn to mid travel: a sweep of one still segment, which
        osc-lab opens with a seek to the middle band and ends with a park
        at the soft limits' center. Refuses unless pos reads there. A horn
        already there is left alone: a lock put on before the sitting holds
        it within MID_TOL but not within the park's band, and the park would
        drive against it."""
        lo, hi = self.guards()
        mid = (lo + hi) // 2
        pos = self.probe("center")["pos"]
        if abs(pos - mid) > MID_TOL:
            self._centers += 1
            out = self.sittings_dir(f"center-{self._centers}")
            self.drive(out, PILOT[0], [f"coast:{CENTER_MS}"], dirs="fwd",
                       rest_ms=0, baseline_ms=CENTER_MS)
            pos = self.probe("centered")["pos"]
        if abs(pos - mid) > MID_TOL:
            raise SystemExit(f"the horn reads {pos}, not within {MID_TOL} of "
                             f"mid travel {mid}")
        return pos

    def set_lock(self, on):
        """The horn lock on at mid travel, or off, at the prompt."""
        if on:
            self.center()
            self._seats.pop("lock", None)
            self.prompt(LOCK_ON)
            self.probe("lock on")
        else:
            self.prompt(LOCK_OFF)
        self.locked = on
        self.note("lock", "on" if on else "off")

    @contextmanager
    def clip(self, out, name):
        """One camera clip around a block: the recording started at the
        prompt, the clapper flashed, the clip copied in at the end."""
        self.prompt(f"Start the phone recording for {name}.")
        osc.clapper(Path(out) / f"{name}-clapper.csv", CLAPPER_FLASHES)
        self.note("clip", f"{name} started")
        try:
            yield
        finally:
            self.prompt(f"Stop the recording. Copy the clip into "
                        f"{self.label(out)} as {name}.mp4 "
                        "(equipment/camera/iphone.md).")
            self.note("clip", f"{name} stopped")


KICK_MS = 300                 # a kicked spin's opening, long enough to reach the kick's speed


def log_row(path, row):
    """One row of a plan's table into `path`, the header first when the
    file is new."""
    path = Path(path)
    new = not path.exists()
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row), lineterminator="\n")
        if new:
            w.writeheader()
        w.writerow(row)


def step_duties(steps):
    """The duty magnitude of every driving step of an osc-lab schedule."""
    out = []
    for step in steps:
        head = str(step).split("@")[0]
        if head.startswith(("coast:", "brake:")):
            continue
        out.append(abs(int(head.split(":")[-1])))
    return out


def drive_speed(folder):
    """The fastest pos speed of a sweep's driven segments, counts/s: a line
    through the last third of each, where the spin has settled."""
    df, _ = tel.load(folder)
    best = 0.0
    for _, g in df[df["cmd_duty_q15"] != 0].groupby("seg"):
        tail = g.iloc[len(g) - len(g) // 3:].dropna(subset=["pos"])
        if len(tail) < SPEED_MIN_ROWS:
            continue
        slope = np.polyfit(tail["t"], tail["pos"].astype(float), 1)[0]
        best = max(best, abs(float(slope)))
    if best == 0.0:
        raise SystemExit(f"{folder}: no driven segment carries a pos speed")
    return best


def seat_pos(folder):
    """The median pos over the driven rows of a sweep, by direction: where
    a pressed seat sat."""
    df, _ = tel.load(folder)
    driven = df[df["cmd_duty_q15"] != 0].dropna(subset=["pos"])
    return {int(d): float(g["pos"].median()) for d, g in driven.groupby("dir")}


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
    only when named by `include` or `only`, `skip` by name."""
    in_state = [p for p in plans if p.state == state]
    for t in (*only, *skip, *include):
        if not any(named(p, (t,)) for p in in_state):
            raise SystemExit(f"no plan {t!r} in state {state}; the plans are "
                             f"{', '.join(sorted(f'{p.experiment}/{p.name}' for p in in_state))}")
    chosen = [p for p in in_state if not named(p, skip) and (not p.optional or named(p, include) or named(p, only))
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


def run_servo(s, plans):
    """A servo state: the brackets at the open and, mirrored, at the close;
    the horn lock on and off at the prompt as the plans ask; a plan that
    cannot run here noted and passed, and after a moved seat every seated
    plan left out."""
    s.prompt(ENTRY[s.state])
    brackets = [p for p in plans if p.bracket]
    body = [p for p in plans if not p.bracket]
    sequence = ([(p, "-open") for p in brackets] + [(p, "") for p in body]
                + [(p, "-close") for p in reversed(brackets)])
    for p, suffix in sequence:
        if p.seated and s.seat_moved:
            s.note("skip", f"{p.experiment}/{p.name}: a seat moved earlier")
            continue
        if p.lock != s.locked:
            s.set_lock(p.lock)
        s.note("plan", p.name + (f" ({suffix[1:]})" if suffix else ""))
        try:
            p.run(s, s.dataset(p, suffix))
        except Skip as e:
            s.note("skip", f"{p.experiment}/{p.name}: {e}")
            if isinstance(e, SeatMoved):
                s.seat_moved = True
    if s.locked:
        s.set_lock(False)


def run(state, n, only=(), skip=(), include=(), plans=None):
    plans = select(plans if plans is not None else discover(), state, only, skip, include)
    if not plans:
        raise SystemExit(f"no plan runs in state {state} with that selection")
    s = Sitting(state, n)
    unit = s.servo.key if s.servo else s.motor.key
    print("unit", unit, "board", s.board.key, "state", state, "plans",
          ", ".join(p.name for p in plans))
    s.open()
    try:
        if state == "locked":
            run_locked(s, plans)
        elif state in SERVO_STATES:
            run_servo(s, plans)
        else:
            run_free(s, plans)
    finally:
        s.power_down()
        s.close()
    print("wrote", s.root / "telemetry" / s.group / "sittings" / f"{state}-{n}.csv")


def listing(plans):
    for state in STATES:
        print(state)
        for p in sorted((p for p in plans if p.state == state), key=lambda p: p.order):
            marks = (("bracket", p.bracket), ("optional", p.optional),
                     ("over rating", p.over_rating), ("lock", p.lock))
            flags = ", ".join(f for f, on in marks if on)
            cost = f" ({p.minutes:g} min, {p.mb:g} MB)" if p.minutes else ""
            tail = f" [{flags}]" if flags else ""
            print(f"  {p.name:14s} {p.experiment:16s} {p.doc}{cost}{tail}")


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
