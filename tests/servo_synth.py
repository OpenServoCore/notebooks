"""A synthetic stage 3 servo, for drafting and testing the stage 3 notebooks
before the bench, in the pattern of `sweep_synth.py`.

`Model` is a servo of given constants: the winding's R and L, Ke, the
inertia at the motor with the train reflected, Coulomb and viscous friction
as motor currents with a breakaway over the sliding level, the gear ratio,
a pot table and the backlash between the train and the pot. `sweep()`
drives it through an `osc-lab sweep` schedule (the drive, `then:`, `hold:`
and `coast:` steps, free from the guards, pressed into a stop, or held by
the horn lock) and writes what the tool writes: `sweep.csv`, `segments.csv`
and `meta.json`. `write_sitting()` writes a small `servo` sitting's worth
of datasets in the stage 3 plans' layouts, at the three rails, with its
manifest and the model beside it as `synthetic.json`.

A test fixture, not a simulator of record. The PWM is averaged, so the
crest sample reads the mean current with the commutation ripple on top at
`ripple_per_rev` per motor revolution and no reading term; a dropout row is
a crest sample at the zero, drawn at `dropouts_per_rev` per revolution; the
taps read the rail across the motor while it is driven and the EMF while
it coasts; the train has no compliance and stops dead at the end stops.

  uv run python tests/servo_synth.py OUT    writes OUT/telemetry/<group>/...
"""
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from oscnb import boards

HEADER = ("seg,cmd_duty_q15,dir,tick,window_valid,pos,current,current_trough,"
          "duty_counts,vdiff,vbus,current_raw,vmotor_a,vmotor_b,vbus_raw,"
          "ntc_raw,vref_raw,pos_lin").split(",")
TICK_HZ = 20_000
DT = 1 / TICK_HZ
SEEK_S = 1.0          # the host time a seek and its settle take, about
BASELINE_MS = 100     # the torque-off segment, short to keep the files small
RAILS = (3.7, 5.0, 7.4)


@dataclass(frozen=True)
class Model:
    """The servo's constants, the truth a notebook drafted on it recovers."""
    r_ohm: float = 4.46
    l_h: float = 0.83e-3
    ke_v_per_rev_s: float = 7.34e-3
    j_kg_m2: float = 8.0e-9            # rotor plus the train at the motor
    coulomb_a: float = 0.06            # sliding friction, motor current
    breakaway_a: float = 0.08          # what a still shaft needs to turn
    viscous_a_per_rev_s: float = 4.0e-4
    gear_ratio: float = 308.0533
    counts_per_deg: float = 19.8
    travel_deg: float = 184.0
    stop_lo_counts: float = 160.0      # the pot at the low stop
    pot_bow_counts: float = 12.0       # the table's departure from a line
    backlash_deg: float = 0.8
    ripple_per_rev: int = 6
    ripple_depth: float = 0.25         # current ripple, share of the mean
    emf_ripple_depth: float = 0.04
    dropouts_per_rev: float = 0.3
    zero_counts: float = 1207.0
    noise_counts: float = 1.3
    pos_noise_counts: float = 0.4
    soft_inset_counts: int = 60        # the soft limits inside the stops

    def pot(self, out_deg):
        """The pot table: counts at an output angle from the low stop."""
        x = np.asarray(out_deg, float)
        bow = self.pot_bow_counts * np.sin(np.pi * x / self.travel_deg)
        return self.stop_lo_counts + self.counts_per_deg * x + bow

    def angle_of(self, counts):
        """The output angle a pot reading sits at, by the line alone."""
        return (counts - self.stop_lo_counts) / self.counts_per_deg

    @property
    def stops(self):
        return (round(float(self.pot(0.0))),
                round(float(self.pot(self.travel_deg))))

    @property
    def soft(self):
        lo, hi = self.stops
        return lo + self.soft_inset_counts, hi - self.soft_inset_counts

    @property
    def guards(self):
        lo, hi = self.soft
        return lo + 100, hi - 100

    def steady_rev_s(self, v_eff):
        """The free motor's settled speed at `v_eff` effective volts."""
        v = abs(v_eff) - self.r_ohm * self.coulomb_a
        if abs(v_eff) <= self.r_ohm * self.breakaway_a or v <= 0:
            return 0.0
        k = self.ke_v_per_rev_s + self.r_ohm * self.viscous_a_per_rev_s
        return math.copysign(v / k, v_eff)

    @property
    def counts_per_motor_rev(self):
        return self.counts_per_deg * 360 / self.gear_ratio


class Shaft:
    """The servo's state between rows: motor angle and speed, the winding's
    current, the pot side of the backlash."""

    def __init__(self, model, out_deg):
        self.m = model
        self.out_deg = out_deg
        self.theta = out_deg * model.gear_ratio / 360   # motor revolutions
        self.omega = 0.0                               # rev/s
        self.i = 0.0

    def place(self, counts):
        """Put the shaft at rest at a pot reading, as a seek leaves it."""
        self.__init__(self.m, float(self.m.angle_of(counts)))

    def run(self, n, v, held=False, driven=True):
        """`n` ticks at `v` volts across the motor: arrays of the current,
        the speed and the motor angle at each, and the output angle."""
        m = self.m
        decay = math.exp(-DT * m.r_ohm / m.l_h)
        kt = m.ke_v_per_rev_s / (2 * math.pi)
        accel = kt / m.j_kg_m2 / (2 * math.pi) * DT
        half = m.backlash_deg / 2
        cur, spd, ang, out = (np.empty(n) for _ in range(4))
        i, w, th, o = self.i, self.omega, self.theta, self.out_deg
        for k in range(n):
            if driven:
                ss = (v - m.ke_v_per_rev_s * w) / m.r_ohm
                i = ss + (i - ss) * decay
            else:
                i = 0.0
            if held:
                w = 0.0
            elif w == 0.0:
                if abs(i) > m.breakaway_a:
                    w = accel * (i - math.copysign(m.coulomb_a, i))
            else:
                drag = math.copysign(m.coulomb_a, w) + m.viscous_a_per_rev_s * w
                nw = w + accel * (i - drag)
                w = 0.0 if nw * w < 0 else nw
            th += w * DT
            gear_deg = th * 360 / m.gear_ratio
            o = min(max(o, gear_deg - half), gear_deg + half)
            if o < 0 or o > m.travel_deg:
                o = min(max(o, 0.0), m.travel_deg)
                th = o * m.gear_ratio / 360
                w = 0.0
            cur[k], spd[k], ang[k], out[k] = i, w, th, o
        self.i, self.omega, self.theta, self.out_deg = i, w, th, o
        return cur, spd, ang, out


def parse(step):
    """(kind, signed pct, ms or None) of one osc-lab schedule entry."""
    head, _, ms = str(step).partition("@")
    ms = int(ms) if ms else None
    for kind in ("coast", "brake"):
        if head.startswith(kind + ":"):
            return kind, 0, int(head.split(":")[1])
    for kind in ("then", "hold"):
        if head.startswith(kind + ":"):
            return kind, int(head.split(":")[1]), ms
    return "drive", int(head), ms


def q15(pct):
    return int(pct * 32767 / 100)


def sweep(folder, model, rail, steps, dirs="both", seat=None, guards=None,
          window_ms=150, baseline_ms=BASELINE_MS, rest_ms=0, board=None,
          t0=None, seed=0):
    """One `osc-lab sweep` of `steps` on the model into `folder`. `seat`
    None runs the free shaft from the guards, "stop" presses it into the
    end stop of each direction, "lock" holds the horn where it is, mid
    travel unless placed. Returns the folder."""
    board = board or boards.BOARDS[boards.DEFAULT]
    rng = np.random.default_rng(seed)
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    lo, hi = guards or model.guards
    stop_lo, stop_hi = model.stops
    mid = (model.soft[0] + model.soft[1]) / 2
    shaft = Shaft(model, float(model.angle_of(mid)))
    vb = board.bias_nom_counts
    vtpc = board.v_term_per_count
    cpa = board.counts_per_a
    t_host = time.time() if t0 is None else t0
    tick = 0
    frames, times = [], []

    def rows(seg, d, cmd, n, volts, kind):
        nonlocal tick
        driven = kind != "coast"
        cur, spd, ang, out = shaft.run(n, volts, held=seat is not None,
                                       driven=driven)
        phase = np.cos(2 * np.pi * model.ripple_per_rev * ang)
        noise = rng.normal(0, model.noise_counts, n)
        if driven and cmd:
            crest = cur * (1 + model.ripple_depth * phase)
            p_drop = model.dropouts_per_rev * np.abs(spd) * DT
            crest[rng.random(n) < p_drop] = 0.0
            diff = np.sign(cmd) * rail * np.ones(n)
            duty_counts = np.sign(cmd) * int(board.drive_ticks(cmd))
        else:
            crest = np.zeros(n)
            diff = model.ke_v_per_rev_s * spd * (
                1 + model.emf_ripple_depth * phase)
            duty_counts = 0
        pos = model.pot(out) + rng.normal(0, model.pos_noise_counts, n)
        frame = pd.DataFrame({
            "seg": seg, "cmd_duty_q15": cmd, "dir": d,
            "tick": tick + np.arange(n), "window_valid": int(bool(cmd)),
            "pos": np.round(pos).astype(int),
            "current_trough": np.round(model.zero_counts + noise / 2),
            "duty_counts": duty_counts,
            "current_raw": np.round(model.zero_counts + crest * cpa + noise),
            "vmotor_a": np.round(vb + np.clip(diff, 0, None) / vtpc),
            "vmotor_b": np.round(vb + np.clip(-diff, 0, None) / vtpc),
        })
        tick += n
        return frame

    def segment(seg, d, cmd, ms, volts, kind):
        nonlocal t_host
        n = round(ms * TICK_HZ / 1000)
        start = t_host
        if kind == "hold":
            shaft.run(n, volts, held=seat is not None)
            streamed, samples = 0, 0
        else:
            frames.append(rows(seg, d, cmd, n, volts, kind))
            streamed, samples = 1, n
        t_host += ms / 1000
        times.append({"seg": seg, "dir": d, "cmd_duty_q15": cmd,
                      "streamed": streamed, "samples": samples,
                      "t_start": f"{start:.3f}", "t_end": f"{t_host:.3f}"})

    segment(0, 0, 0, baseline_ms, 0.0, "coast")
    seg = 1
    for d in {"both": (1, -1), "fwd": (1,), "rev": (-1,)}[dirs]:
        for step in steps:
            kind, pct, ms = parse(step)
            ms = window_ms if ms is None else ms
            if kind == "drive":
                t_host += SEEK_S
                if seat == "stop":
                    shaft.place(stop_hi if d > 0 else stop_lo)
                elif seat is None:
                    shaft.place(lo if d > 0 else hi)
            cmd = d * q15(pct)
            volts = rail * d * pct / 100
            segment(seg, d, cmd, ms, volts, kind)
            seg += 1
        shaft.omega, shaft.i = 0.0, 0.0
        t_host += rest_ms / 1000
    df = pd.concat(frames, ignore_index=True).reindex(columns=HEADER)
    df.to_csv(folder / "sweep.csv", index=False)
    pd.DataFrame(times).to_csv(folder / "segments.csv", index=False)
    meta = {"model": 257, "fw": 65, "tick_hz": TICK_HZ,
            "pwm_period_ticks": board.pwm_arr, "sense": board.sense,
            "schedule": [str(s) for s in steps], "dirs": dirs,
            "stall": seat == "stop", "static_load": seat == "lock",
            "guard": [lo, hi], "tel_chunks": [[0x238, 2], [0x240, 10]],
            "window_ms": window_ms, "baseline_ms": baseline_ms,
            "rest_ms": rest_ms, "settle_ms": 300, "decay": "slow",
            "rows_dropped": 0, "host_git_sha": "synthetic"}
    (folder / "meta.json").write_text(json.dumps(meta, indent=2))
    return folder


class Manifest:
    """The sitting's manifest, `sittings/<state>-N.csv`, on a fake clock."""

    def __init__(self, group_dir, state, n, t0):
        self.path = group_dir / "sittings" / f"{state}-{n}.csv"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.t = t0
        self.rows = []

    def note(self, event, detail=""):
        self.rows.append({"t_host": f"{self.t:.3f}", "event": event,
                          "detail": detail})
        self.t += 0.01

    def close(self):
        pd.DataFrame(self.rows).to_csv(self.path, index=False)


def window(model, v_eff, want_ms, margin=0.8):
    """The plan's window cut so the model's own settled speed stays on the
    runway, the generator's stand-in for the runner's pilot sizing."""
    lo, hi = model.guards
    speed = abs(model.steady_rev_s(v_eff)) * model.counts_per_motor_rev
    if speed == 0:
        return want_ms
    return min(want_ms, int(1000 * margin * (hi - lo - 375) / speed))


def write_sitting(root, model=Model(), n=1, servo="mg90-d", board=None,
                  rails=RAILS, duties=(20, 40, 60), low=(0.6, 0.3),
                  repeats=1, spin_ms=400, coast_ms=300, seed=0):
    """A small `servo` sitting of `model` under `root/telemetry/`: the run-up
    and ref brackets, the ladder's capture and low rungs at `rails`, the
    locked and stop resistance chains, one steps chain, the manifest, and
    `synthetic.json` with the model. Returns the group folder."""
    board = board or boards.BOARDS[boards.DEFAULT]
    group = Path(root) / "telemetry" / f"{servo}__{board.key}"
    t = 1.8e9
    man = Manifest(group, "servo", n, t)
    man.note("start", f"servo {servo}, motor {servo}, board {board.key}, "
                      "state servo")
    man.note("guards", f"{model.guards[0]} to {model.guards[1]} counts")
    k_seed = [seed]

    def run(folder, rail, steps, **kw):
        nonlocal t
        k_seed[0] += 1
        man.note("probe", f"{folder.relative_to(root)}: pos "
                          f"{round(float(np.mean(model.soft)))}, "
                          f"ntc_raw 2139, vbus_raw "
                          f"{round(rail / board.v_rail_per_count)}")
        sweep(folder, model, rail, steps, board=board, t0=t,
              seed=k_seed[0], **kw)
        t += 30
        man.t = t

    def cycles(out, rungs, want, coast, kick=None, kick_ms=0):
        rows = []
        k = 0
        for v, d in rungs:
            ms = window(model, v * d / 100, want)
            kd = kick[v] if kick else 0
            steps = ([f"{kd}@{kick_ms}", f"then:{d}@{ms}"] if kick
                     else [f"{d}@{ms}"]) + [f"coast:{coast}"]
            for r in range(repeats):
                start = t
                run(out / "cycles" / str(k), v, steps)
                rows.append({"k": k, "volts_set": v, "dps_read":
                             f"{v + 0.25:.3f}", "vsys_read": f"{v:.3f}",
                             "duty_pct": d, "repeat": r,
                             "t_start": f"{start:.3f}",
                             "t_end": f"{t:.3f}", "folder": f"cycles/{k}",
                             "kick_pct": kd, "kick_ms": kick_ms,
                             "spin_ms": ms})
                k += 1
        pd.DataFrame(rows).to_csv(out / "cycles.csv", index=False)

    def dataset(experiment, name):
        out = group / experiment / f"{name}-{n}"
        out.mkdir(parents=True, exist_ok=True)
        man.note("dataset", f"{name}: {out.relative_to(root)}")
        return out

    def chains(out, seat, laps):
        """`laps`, (rail, side) pairs, as the resistance plans chain them."""
        mid = round(float(np.mean(model.soft)))
        rows, seats = [], []
        for k, (rail, side) in enumerate(laps):
            low_d, high_d = (round(100 * v / rail) for v in (1.2, 1.8))
            if seat == "lock":
                steps = ["100@300"]
                row = {"duty_pct": 100, "pos_before": mid, "pos_after": mid}
                pos = mid
            else:
                steps = [f"{low_d}@300", f"then:{high_d}@300",
                         f"then:{low_d}@300"]
                pos = model.stops[side == "fwd"]
                row = {"duty_low_pct": low_d, "duty_high_pct": high_d,
                       "stop": side, "seat_pos": pos}
            start = t
            run(out / "chains" / str(k), rail, steps, dirs=side, seat=seat)
            row = {"k": k, "volts_set": rail, "dps_read": f"{rail + 0.25:.3f}",
                   "vsys_read": f"{rail:.3f}", **row,
                   "t_start": f"{start:.3f}", "t_end": f"{t:.3f}",
                   "folder": f"chains/{k}"}
            rows.append(row)
            seats.append({"t_host": f"{t:.3f}",
                          "seat": "lock" if seat == "lock" else side,
                          "pos": pos, "shift_counts": 0})
        pd.DataFrame(rows).to_csv(out / "chains.csv", index=False)
        pd.DataFrame(seats).to_csv(out / "seats.csv", index=False)

    def brackets(suffix, order):
        for experiment, name in order:
            man.note("plan", f"{name} ({suffix})")
            out = dataset(experiment, f"{name}-{suffix}")
            if name == "runup":
                cycles(out, [(5.0, 100)], 200, 500)
            else:
                cycles(out, [(5.0, 30)], spin_ms, coast_ms)

    def body():
        man.note("lock", "on")
        chains(dataset("resistance", "locked"), "lock",
               ((3.3, "both"), (3.7, "both")))
        man.note("lock", "off")
        chains(dataset("resistance", "stop"), "stop",
               ((5.0, "rev"), (5.0, "fwd")))
        cycles(dataset("ladder", "capture"),
               [(v, d) for v in rails for d in duties], spin_ms, coast_ms)
        kick = {3.7: 40, 5.0: 30, 7.4: 20}
        cycles(dataset("ladder", "low"),
               [(v, max(1, round(100 * e / v))) for v in rails for e in low],
               spin_ms, coast_ms, kick=kick, kick_ms=150)
        out = dataset("steps", "capture")
        start = t
        run(out / "chains" / "0", 5.0,
            ["30@200", "then:35@150", "then:30@150"])
        pd.DataFrame([{"k": 0, "volts_set": 5.0, "start_share": "0.000",
                       "guard_lo": model.guards[0],
                       "guard_hi": model.guards[1], "base_pct": 30,
                       "step_pct": 5, "windows_ms": "200/150/150",
                       "t_start": f"{start:.3f}", "t_end": f"{t:.3f}",
                       "folder": "chains/0"}]).to_csv(out / "chains.csv",
                                                      index=False)

    brackets("open", (("run-up", "runup"), ("ladder", "ref")))
    body()
    brackets("close", (("ladder", "ref"), ("run-up", "runup")))
    man.note("end")
    man.close()
    truth = {"model": asdict(model), "stops": list(model.stops),
             "soft": list(model.soft), "guards": list(model.guards)}
    (group / "synthetic.json").write_text(json.dumps(truth, indent=2) + "\n")
    return group


if __name__ == "__main__":
    print("wrote", write_sitting(Path(sys.argv[1])))
