"""Synthetic `osc-lab track` captures for the scorecard's tests: the files
the tool writes (track.csv, segments.csv, meta.json) and the plan.json the
runner puts beside them, from rows a test chooses or from a fake servo
that answers a schedule, a board's wide-code result file, and the
constant state a fake servo answers `osc` with.

A test fixture, not a model of record. `fake_track` runs each goal as a
first-order approach at a speed cap, its duty the error's sign at a fixed
share of the period, its current a fixed count per duty.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

from oscnb import boards, postable, scorecard, widecodes

HEADER = ("seg,goal,tick,window_valid,pos,current,current_trough,duty_counts,vdiff,vbus,"
          "current_raw,vmotor_a,vmotor_b,vbus_raw,ntc_raw,vref_raw").split(",")
SEG_HEADER = ("seg,kind,field,value,ms,tick0,rows,rows_dropped,t_start,t_end,vbus_raw,ntc_raw,"
              "theta_hat_q16,omega_hat_cps,t_winding_cc,i_lim_counts,limit_flags,fault_flags,pos,"
              "i_tick_counts,current_bias_counts,fault_code,omega_hat_src").split(",")
ZERO = 1210           # the shunt's raw zero
I_PER_DUTY = 0.2      # raw current counts per duty count
WINDOW_MIN = 64       # a drive window under this many duty counts reads no current
BOARD = boards.BOARDS[boards.DEFAULT]
GOAL_PERIOD_S = scorecard.testset().GOAL_PERIOD_S


def meta(center, guard, tick_hz, config=None, lut_state="IDENTITY"):
    return {"center": center, "guard": list(guard), "tick_hz": tick_hz, "ended": "ok",
            "config": config or {"p_kp_q88": 40212, "pos_deadband_counts": 4},
            "sense": BOARD.sense,
            "plant": {"lut_state": lut_state}, "pwm_period_ticks": 1200}


def write(out, segs, center, guard, tick_hz=20_000, tick0=60_000, gap_ticks=500,
          plan=None, config=None, lut_state="IDENTITY", polls=None):
    """One capture into `out`. `segs` are (item, goal, pos, duty) per
    schedule item, the arrays one value a row; tick0 starts at `tick0` and
    each segment opens `gap_ticks` after the last, wrapping at 2^16, its
    t_start the host's clock on the same axis. `polls` are a column of
    segments.csv each, one value an item; the rest are left empty."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    frames, seg_rows = [], []
    t = tick0
    for k, (item, goal, pos, duty) in enumerate(segs):
        n = len(pos)
        duty = np.asarray(duty, int)
        valid = np.abs(duty) >= WINDOW_MIN
        raw = np.round(ZERO + np.abs(duty) * I_PER_DUTY * valid).astype(int)
        frames.append(pd.DataFrame({
            "seg": k, "goal": goal, "tick": np.arange(n), "window_valid": valid.astype(int),
            "pos": np.round(pos).astype(int), "current_trough": ZERO, "duty_counts": duty,
            "current_raw": raw, "vmotor_a": 600, "vmotor_b": 600}))
        kind, rest = item.split("@")[0], item.split("@")[1]
        value = kind.split(":")[1] if kind.startswith("goal:") else ""
        seg_rows.append({"seg": k, "kind": "goal" if value else "hold", "value": value, "ms": rest,
                         "tick0": t % 65536, "rows": n, "rows_dropped": 0,
                         "t_start": (t - tick0) / tick_hz, "t_end": (t - tick0 + n) / tick_hz,
                         **{c: v[k] for c, v in (polls or {}).items()}})
        t += n + gap_ticks
    pd.concat(frames).reindex(columns=HEADER).to_csv(out / "track.csv", index=False)
    pd.DataFrame(seg_rows).reindex(columns=SEG_HEADER).to_csv(out / "segments.csv", index=False)
    (out / "meta.json").write_text(json.dumps(meta(center, guard, tick_hz, config, lut_state)))
    if plan is not None:
        (out / "plan.json").write_text(json.dumps(plan))
    return out


def rows_of(ms, tick_hz):
    return int(ms * tick_hz / 1000)


def fake_track(out, schedule, start, tick_hz=2_000, tau_s=0.03, max_cps=1500.0,
               duty_counts=600, deadband=4):
    """A servo that answers `schedule` from `start` counts; writes the
    capture (no plan) and returns where the shaft ended. Its meta names
    the middle of the guard as the center, as `osc-lab track` does."""
    plan = json.loads((Path(out) / "plan.json").read_text())
    guard = plan["guard_counts"]
    center = guard[0] + (guard[1] - guard[0]) // 2
    dt = 1 / tick_hz
    pos = float(start)
    goal = round(start)
    segs = []
    for item in schedule:
        n = rows_of(int(item.split("@")[1]), tick_hz)
        if item.startswith("goal:"):
            goal = int(item.split(":")[1].split("@")[0])
        p = np.empty(n)
        d = np.empty(n, int)
        for i in range(n):
            err = goal - pos
            step = np.clip(err * dt / tau_s, -max_cps * dt, max_cps * dt)
            pos += step
            p[i] = pos
            d[i] = 0 if abs(err) <= deadband else int(np.sign(err)) * duty_counts
        segs.append((item, goal, p, d))
    # the burst and the host's overhead, as a 20 ms goal item really runs
    gap = round((GOAL_PERIOD_S - 0.020) * tick_hz)
    write(out, segs, center, guard, tick_hz=tick_hz, gap_ticks=gap, plan=plan)
    return pos


def wide_result(path, codes, root=None):
    """A board's wide-code result file at `path`, the bands and the slots
    as `widecodes find` writes them. `codes` are (stuck, skipped) or
    (stuck, skipped, ratio), the ratio that of a code reading its own rows
    and the skipped codes' when left out. Returns what a rung names: the
    file, from `root` when given, and its sha1."""
    bands = [widecodes.Band(c[0], c[1], float(c[2] if len(c) > 2 else 1 + c[1])) for c in codes]
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({
        "wide_codes": [{"stuck_code": b.stuck, "skipped": b.skipped, "ratio": b.ratio, "wide": b.wide,
                        "representable": b.representable, "slot": b.slot, "value_q4": b.value} for b in bands],
        widecodes.FIELD: list(widecodes.slots(bands))}))
    return {"file": str(path.relative_to(root) if root else path),
            "sha1": hashlib.sha1(path.read_bytes()).hexdigest()}


class Servo:
    """The constant state a fake servo answers `osc` with: a few CONFIG
    and CALIB fields where its descriptor places them, a byte no field
    names among them, the first fields of CONTROL after them, the
    position table behind its window and the status report. The places
    are not a real servo's, CONTROL starting at 0xc0, so a span read off
    this servo comes from its descriptor alone."""

    # name: (addr, width, access, kind), in table order
    FIELDS = {"model_number": (0x00, 2, "ro", "uint"), "p_kp_q88": (0x40, 2, "rw", "uint"),
              "pos_deadband_counts": (0x42, 2, "rw", "uint"), "drive_polarity": (0x4b, 1, "rw", "bool"),
              "r_q12": (0x80, 2, "rw", "uint"), "plant_stamp": (0x8a, 2, "rw", "uint"),
              widecodes.FIELD: (0x91, 16, "rw", "bytes"),
              "torque_enable": (0xc0, 1, "rw", "bool"), "pos_lut_state": (0xde, 1, "ro", "uint")}
    STORED = {"model_number": 257, "p_kp_q88": 40212, "pos_deadband_counts": 4, "drive_polarity": 1,
              "r_q12": 17100, "plant_stamp": 0x9c45, widecodes.FIELD: bytes([0, 0, 0, 88] + [0] * 12)}
    UNNAMED = 0x7f

    def __init__(self):
        self.pages = [bytes(2 * postable.PAGE_POINTS)] * postable.PAGES
        self.window = bytes(2 * postable.PAGE_POINTS)
        self.stamp, self.data = "0x9c45 = live set", "clean"
        self.table = bytearray(0x100)
        self.log = []
        self.reboot()

    def reboot(self):
        """The stored fields back, as `osc reboot` brings them."""
        for name, v in self.STORED.items():
            self.set(name, v)

    def set(self, name, v):
        addr, width, _, kind = self.FIELDS[name]
        self.table[addr:addr + width] = v if kind == "bytes" else int(v).to_bytes(width, "little")

    def decoded(self, name):
        addr, width, _, kind = self.FIELDS[name]
        raw = bytes(self.table[addr:addr + width])
        if kind == "bytes":
            return raw.hex(" ")
        v = int.from_bytes(raw, "little")
        return ("on" if v else "off") if kind == "bool" else v

    def get(self, name):
        self.log.append(("get", name))
        return self.decoded(name)

    def dump(self):
        self.log.append(("dump",))
        return [(addr, name, access, self.decoded(name)) for name, (addr, _, access, _) in self.FIELDS.items()]

    def field(self, name):
        self.log.append(("field", name))
        if name not in self.FIELDS:
            return None
        addr, width = self.FIELDS[name][:2]
        return addr, width, bytes(self.table[addr:addr + width])

    def write(self, addr, data):
        self.log.append(("write", addr, bytes(data)))
        assert addr == postable.WINDOW_ADDR and data[1] == postable.FETCH, "the fake window only fetches"
        self.window = self.pages[data[0]]

    def read(self, addr, n):
        self.log.append(("read", addr, n))
        if addr == postable.POINTS_ADDR:
            return self.window[:n]
        return bytes(self.table[addr:addr + n])

    def status(self):
        self.log.append(("status",))
        return f"id 1: model 0x0101\n      data  {self.data}\n      stamp {self.stamp}\n"

    def stub(self, monkeypatch, regs=dict):
        """`osc`'s reads and its window write answered by this servo;
        `get` reads `regs()` first, a fixture's own registers. Returns the
        servo."""
        def get(name):
            own = regs()
            return own[name] if name in own else self.get(name)

        monkeypatch.setattr(scorecard.osc, "get", get)
        for name in ("dump", "field", "write", "read", "status"):
            monkeypatch.setattr(scorecard.osc, name, getattr(self, name))
        return self
