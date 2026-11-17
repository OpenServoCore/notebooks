"""Plumbing of the window-term notebooks: stages/3-servo/window-term, the
window conductance from OpenLoop onsets from rest, and
stages/5-closed-loop/cruise-friction, the friction line the velocity loop holds
under the window term, from Velocity cruises.

The window term is the part of a crest shunt sample that is not winding
current: a share of the drive window the motor's loss conductance g
carries across its inductance, g x (1 - D) x V. The image the captures ran
takes it off every reading at the source, so a reading is the settle-gained
sample over the zero less (K0 - k x ticks) >> 16, signed by the drive.

Loading and checking a capture, cutting the onsets from rest, the
pulse-train model of the winding, the image's integer reading, the
MEDIUM-period phase and the speed windows of a cruise live here. The fits,
the replay of the reference and the lines stay in the notebooks.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

from . import boards, constants as cn, postable, results, scales, widecodes
from . import scorecard as sc

SWEEP_COLS = ["seg", "dir", "cmd_duty_q15", "tick", "window_valid", "pos",
              "current", "current_trough", "duty_counts", "current_raw",
              "vmotor_a", "vmotor_b"]
TRACK_COLS = ["seg", "goal", "tick", "window_valid", "pos", "current_trough",
              "duty_counts", "current_raw", "vmotor_a", "vmotor_b"]


def board_of(meta, where):
    """The board a capture's sense block names, checked against it."""
    board = boards.from_meta(meta)
    if board is None:
        raise ValueError(f"{where}: no board matches the sense block")
    board.check_meta(meta, where)
    return board


def _ticks_whole(rows, where):
    if (rows.groupby("seg")["tick"].diff().dropna() != 1).any():
        raise ValueError(f"{where}: a tick is missing inside a segment")


def load_sweep(folder):
    """An osc-lab OpenLoop sweep (sweep.csv), checked: the board from its
    meta, every tick inside a segment, no rows dropped, the live set's
    stamp. Rows, meta, board, stage 1 scales, segments and the torque-off
    baseline's crest and trough means over its second half."""
    folder = Path(folder)
    where = str(results.rel(folder))
    meta = json.loads((folder / "meta.json").read_text())
    board = board_of(meta, where)
    if meta["rows_dropped"] or meta["plant"]["stamp_verdict"] != "match":
        raise ValueError(f"{where}: rows dropped or not the live set")
    rows = pd.read_csv(folder / "sweep.csv", usecols=SWEEP_COLS)
    _ticks_whole(rows, where)
    rest = rows[rows["seg"] == 0]
    rest = rest.iloc[len(rest) // 2:]
    return {"name": folder.name, "folder": folder, "rows": rows, "meta": meta,
            "board": board, "sc": scales.load(board),
            "arr": meta["pwm_period_ticks"], "hz": meta["tick_hz"],
            "segs": pd.read_csv(folder / "segments.csv").set_index("seg"),
            "rest_crest": float(rest["current_raw"].mean()),
            "rest_trough": float(rest["current_trough"].mean())}


def rail_v(cap):
    """The capture's rail at rest, V: meta's vbus_counts is in tap units
    (the image rescales the rail to the terminal chain), so it reads
    through the tap scale. Used for the edge lags' rail line only."""
    return cap["meta"]["vbus_counts"] / cap["sc"].terminal_counts_per_v


def onsets(cap, after=60):
    """Every drive onset of a sweep, the first row of non-zero duty of a
    drive segment, as (rows, info); a segment commanded to zero, a brake
    of a chain, has none. rows: the segment's rows from its first to
    `after` past the onset,
    `k` the row from the onset. info: one line per onset with what the
    gates read: the zero-duty rows before it, their largest tap difference
    along the drive, the pot's move over the first millisecond, the drive's
    sign over 21 rows, the ramp's first 14 widths. `ok` is set by `gate`."""
    r = cap["rows"]
    out, info = [], []
    for seg, g in r[r["seg"] > 0].groupby("seg"):
        if g["cmd_duty_q15"].iloc[0] == 0:
            continue
        g = g.reset_index(drop=True)
        d = g["duty_counts"].to_numpy()
        nz = np.flatnonzero(d != 0)
        if len(nz) == 0:
            continue
        on = int(nz[0])
        w = g.iloc[: on + after].copy()
        w["k"] = np.arange(len(w)) - on
        w["cap"], w["onset"] = cap["name"], int(seg)
        out.append(w)
        sgn = int(np.sign(d[on]))
        pre, post = w[w.k < 0], w[w.k >= 0]
        vd = (w["vmotor_a"] - w["vmotor_b"]) * sgn
        pos = post["pos"].to_numpy()
        info.append({
            "cap": cap["name"], "onset": int(seg), "sgn": sgn,
            "cmd_q15": int(g["cmd_duty_q15"].iloc[0]),
            "pre_rows": len(pre),
            "pre_vd_max": float(vd[w.k < 0].abs().max()) if len(pre)
            else np.nan,
            "moved": float(abs(np.median(pos[16:21])
                               - np.median(pos[0:5]))),
            "same_sign": bool((np.sign(post["duty_counts"].to_numpy()[:21])
                               == sgn).all()),
            "seq": tuple(int(x) for x in
                         np.abs(post["duty_counts"].to_numpy()[:14])),
            "valid0": int(post["window_valid"].iloc[0])})
    info = pd.DataFrame(info)
    return pd.concat(out), info


def gate(info, tap_gate=6, pot_gate=6):
    """The gates of an onset from rest: a still row before it with no
    entry back-EMF (tap difference within `tap_gate` counts), the pot
    within `pot_gate` counts over the first millisecond, one sign, and the
    set's common ramp."""
    seq = info["seq"].mode().iloc[0]
    ok = ((info.pre_rows >= 1) & (info.pre_vd_max <= tap_gate)
          & (info.moved <= pot_gate) & info.same_sign & (info.seq == seq))
    return info.assign(ok=ok), seq


def tick_seconds(arr, hz):
    """Seconds per timer tick: two ticks of `arr` a PWM period."""
    return 1.0 / (hz * 2 * arr)


def pulse_train(h, L, R, lag_on, lag_off, sample, arr, hz, r_brake,
                first_late=0.0):
    """The winding current per volt of on-window at each row's shunt
    sample, through a train of same-sign whole pulses from an empty
    winding: the pulse around crest k conducts from crest - h[k] + lag_on
    to crest + h[k] + lag_off, the winding is shorted through the brake
    loop `r_brake` between pulses, and the sample closes `sample` ticks
    after the crest. h: half-widths in ticks, one a row. L and R
    broadcastable arrays (one call fills a fit grid). `first_late`: ticks
    the first pulse after rest starts later than `lag_on` puts it.
    Returns an array of shape (len(h), *L.shape)."""
    h = np.asarray(h, float)
    L, R = np.broadcast_arrays(np.asarray(L, float), np.asarray(R, float))
    dt = tick_seconds(arr, hz)
    period = 2.0 * arr
    out = np.empty((len(h),) + L.shape)
    i = np.zeros(L.shape)
    t = -period / 2.0
    k_drive, k_brake = dt * R / L, dt * (R + r_brake) / L

    def brake(i, t0, t1):
        return i * np.exp(-max(t1 - t0, 0.0) * k_brake)

    def drive(i, t0, t1):
        return 1 / R + (i - 1 / R) * np.exp(-max(t1 - t0, 0.0) * k_drive)

    started = False
    for k in range(len(h)):
        crest = k * period
        s_at = crest + sample
        if h[k] == 0:
            out[k] = brake(i, t, s_at)
            continue
        a, b = crest - h[k] + lag_on, crest + h[k] + lag_off
        if not started:
            a += first_late
        i = brake(i, t, a)
        if s_at <= a:
            out[k] = brake(i, a, s_at) if s_at < a else i
        elif s_at <= b:
            out[k] = drive(i, a, s_at)
        i = drive(i, a, b)
        t = b
        started = True
        if s_at > b:
            out[k] = brake(i, b, s_at)
    return out


# --- cruises ---

def set_tables(pair):
    """The pair's set of record and what its counts are defined by: (doc,
    fields as plain values, the position table, the wide-code slots), the
    table file checked against the sha1 the set names and its points
    against the set's."""
    doc = cn.of_record(pair)
    fields = {k: e["value"] for k, e in doc["fields"].items()}
    tin = doc["inputs"]["pos_table"]
    path = results.ROOT / tin["file"]
    if sc.file_sha1(path) != tin["sha1"]:
        raise ValueError(f"{tin['file']} is not the table file the set names")
    table, _ = postable.load_file(path)
    points = np.asarray(table.points, "<i2").tobytes()
    if hashlib.sha1(points).hexdigest() != doc["pos_table"]["points_sha1"]:
        raise ValueError("the table file's points are not the set's")
    return doc, fields, table, bytes(doc["wide_codes"]["slots"])


def load_track(folder, table, slots):
    """An osc-lab Velocity track (track.csv), checked: the board from its
    meta, the live table, the live set's stamp, no rows dropped, every
    tick inside a segment. Rows with the pot through the wide-code slots
    and the table (`pos_lin`), meta, the polls at each segment's start
    (segments.csv), board and stage 1 scales."""
    folder = Path(folder)
    where = str(results.rel(folder))
    meta = json.loads((folder / "meta.json").read_text())
    board = board_of(meta, where)
    plant = meta["plant"]
    if (plant["lut_state"] != "LIVE" or plant["stamp_verdict"] != "match"
            or meta["rows_dropped"]):
        raise ValueError(f"{where}: not the live table, or rows dropped")
    rows = pd.read_csv(folder / "track.csv", usecols=TRACK_COLS)
    _ticks_whole(rows, where)
    pos = widecodes.compensated(rows["pos"].to_numpy(), slots)
    rows["pos_lin"] = table.lin(pos)
    polls = pd.read_csv(folder / "segments.csv").set_index("seg")
    return {"name": folder.name, "rows": rows, "meta": meta, "polls": polls,
            "board": board, "sc": scales.load(board),
            "arr": meta["pwm_period_ticks"], "hz": meta["tick_hz"]}


def sitting_fields(folder):
    """The fields a sitting read off the servo (fields.json beside its
    captures): what it read, with the words it set in their place."""
    doc = json.loads((Path(folder) / "fields.json").read_text())
    return {**doc["read"], **doc.get("set", {})}


def medium_phase(i_ref, tick, decim=cn.DECIM_MED):
    """The MEDIUM period's phase in FAST ticks: the one at which a replayed
    reference is flattest inside each period (it moves once a period).
    (phase, median sd inside a period there, the same at the worst)."""
    sd = [pd.Series(i_ref).groupby((tick - ph) // decim).std().median()
          for ph in range(decim)]
    best = int(np.argmin(sd))
    return best, sd[best], max(sd)


def window_speeds(pos, direction, hz, rows):
    """The speed along the travel over consecutive windows of `rows`
    rows, a line through the pot each, c/s."""
    t = np.arange(rows) / hz
    return np.array([direction * np.polyfit(t, pos[k:k + rows], 1)[0]
                     for k in range(0, len(pos) - rows + 1, rows)])


def block_se(x, blocks=8):
    """The standard error of a mean of autocorrelated rows: the sd of
    `blocks` block means over root blocks."""
    m = [b.mean() for b in np.array_split(np.asarray(x, float), blocks)]
    return float(np.std(m, ddof=1) / np.sqrt(blocks))
