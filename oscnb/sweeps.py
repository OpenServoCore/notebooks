"""Every sweep of a pair, in time order, and its rows.

A pair's folder (motor__board under telemetry/) holds one folder per
sweep, with `sweep.csv`, `meta.json` and `segments.csv` as `osc-lab sweep`
writes them. `index` lists every sweep in time order and `load` reads one
with the bridge state of every row.

`row_store` lays chosen columns of every sweep end to end in time order
(the global row index), with each sweep's offsets and baseline and the run
table: a run is contiguous rows of one bridge state inside one segment,
cut again wherever time passes that no row covers. It builds in memory;
`cache_dir` keeps the arrays on disk between calls.

Bridge states: ASLEEP where dir == 0 (torque off), COAST where the driver
is on with no window (dir != 0, duty_counts == 0), else DRIVE (slow decay,
|duty_counts| timer ticks either side of the crest).

The store keeps |duty_counts| as `duty`; `duty_sign` is the drive's
direction, which differs from `dirn` through a reversal segment (`dir`
stays put while the duty's sign flips), and `tick` the tick counter as the
rows carry it (it restarts in every segment).
"""
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from . import boards, tel

ASLEEP, COAST, DRIVE = 0, 1, 2
SETTLE_TICKS = 32     # ticks after a bridge change, stage 1's zero settle
TAIL_S = 0.1          # s, the tail of a rest
BASE_DROP_S = 0.2     # s, the baseline's head, where the arm disturbs it
SCHEDULE_CHARS = 80   # chars of the schedule the index keeps
I4_MAX = 2**31 - 1


def index(pair_dir):
    """One row per folder holding a sweep.csv, in time order: folder
    (under pair_dir), experiment, dataset, t_start, t_end, segments, rows,
    streamed, duty_pct_max, schedule, rows_dropped, gap_s (rest since the
    sweep before), day."""
    pair_dir = Path(pair_dir)
    out = []
    for sw in sorted(pair_dir.rglob("sweep.csv")):
        f = sw.parent
        rel = f.relative_to(pair_dir)
        seg = pd.read_csv(f / "segments.csv")
        meta = json.loads((f / "meta.json").read_text())
        drives = seg[seg["cmd_duty_q15"] != 0]
        duty_max = (round(drives["cmd_duty_q15"].abs().max() / 327.68, 1)
                    if len(drives) else 0.0)
        out.append({
            "folder": str(rel), "experiment": rel.parts[0],
            "dataset": rel.parts[1], "t_start": seg["t_start"].min(),
            "t_end": seg["t_end"].max(), "segments": len(seg),
            "rows": int(seg["samples"].sum()),
            "streamed": int(seg["streamed"].sum()),
            "duty_pct_max": duty_max,
            "schedule": " ".join(map(str, meta.get("schedule", [])))
            [:SCHEDULE_CHARS],
            "rows_dropped": meta.get("rows_dropped")})
    ix = pd.DataFrame(out).sort_values("t_start").reset_index(drop=True)
    ix["gap_s"] = ix["t_start"] - ix["t_end"].shift()
    ix["day"] = pd.to_datetime(ix["t_start"], unit="s").dt.strftime("%m-%d")
    return ix


def bridge_of(rows):
    """The bridge state of every row, from dir and duty_counts."""
    return np.where(rows["dir"] == 0, ASLEEP,
                    np.where(rows["duty_counts"] == 0, COAST, DRIVE))


def load(pair_dir, folder, board=None):
    """(rows, meta, segments) of one sweep by its index folder, with a
    `bridge` column. With a board, the sweep's sense block is checked."""
    f = Path(pair_dir) / folder
    df, meta = tel.load(f, board)
    seg = pd.read_csv(f / "segments.csv").set_index("seg")
    df["bridge"] = bridge_of(df)
    return df, meta, seg


def board_of(pair_dir):
    """The board a pair's folder names after its `__`."""
    key = Path(pair_dir).name.split("__")[-1]
    if key not in boards.BOARDS:
        raise ValueError(f"{pair_dir}: no board entry {key}")
    return boards.BOARDS[key]


def segment_gaps(seg, segs, tick_hz):
    """Ticks no row covers before each row, nonzero only on a segment's
    first row: `tick` restarts in every segment, so the segment clock
    gives the time (unstreamed segments, seeks and rests)."""
    gap = np.zeros(len(seg), np.int64)
    first = np.flatnonzero(np.r_[False, seg[1:] != seg[:-1]])
    prev_a = 0
    for a in first:
        t_prev = segs.t_start[seg[prev_a]] + (a - prev_a) / tick_hz
        gap[a] = max(0, int(round((segs.t_start[seg[a]] - t_prev)
                                  * tick_hz)))
        prev_a = a
    return gap


def run_starts(bridge, seg, gap):
    """(first row, end row) of every run: a change of bridge state, of
    segment, or a gap no row covers starts a new one."""
    ch = np.r_[True, (bridge[1:] != bridge[:-1]) | (seg[1:] != seg[:-1])
               | (gap[1:] > 0)]
    st = np.flatnonzero(ch)
    return st, np.r_[st[1:], len(bridge)]


@dataclass
class _Sweep:
    """One sweep's rows and what its columns are made from."""
    rows: pd.DataFrame
    segs: pd.DataFrame
    board: boards.Board
    gap: np.ndarray       # ticks no row covers before each row
    hi: np.ndarray        # the driven terminal tap, counts
    vb_counts: float      # the bias node, the baseline's own tap
    t_start: float        # s, the sweep's first segment start


def _rail(c):
    """V, the driven terminal at the crest, on drive rows with a valid
    window, else NaN: the only rail a row carries."""
    ok = ((c.rows["bridge"].to_numpy() == DRIVE)
          & (c.rows["window_valid"].to_numpy() > 0))
    v = np.full(len(c.rows), np.nan, np.float32)
    v[ok] = c.board.term_v(c.hi[ok].astype(float), c.vb_counts)
    return v


def _tsw(c):
    """s since the sweep's start: the segment clock plus the row's time."""
    t_seg = (c.segs.t_start.to_numpy()[c.rows["seg"].to_numpy()]
             - c.t_start)
    return t_seg + c.rows["t"].to_numpy()


# name -> (stored dtype, the column from a _Sweep)
COLUMNS = {
    "bridge": ("i1", lambda c: c.rows["bridge"].to_numpy()),
    "duty": ("i2", lambda c: np.abs(c.rows["duty_counts"].to_numpy())),
    "dirn": ("i1", lambda c: c.rows["dir"].to_numpy()),
    "raw": ("i2", lambda c: c.rows["current_raw"].to_numpy()),
    "trough": ("i2", lambda c: c.rows["current_trough"].to_numpy()),
    "hi": ("i2", lambda c: c.hi),
    "pos": ("i2", lambda c: c.rows["pos"].to_numpy()),
    "pre_gap": ("i4", lambda c: np.clip(c.gap, 0, I4_MAX)),
    "rail": ("f4", _rail),
    "tsw": ("f4", _tsw),
    "ph": ("i1", lambda c: c.rows["tick"].to_numpy() % 16),
    "va": ("i2", lambda c: c.rows["vmotor_a"].to_numpy()),
    "vb": ("i2", lambda c: c.rows["vmotor_b"].to_numpy()),
    "valid": ("i1", lambda c: c.rows["window_valid"].to_numpy()),
    "duty_sign": ("i1",
                  lambda c: np.sign(c.rows["duty_counts"].to_numpy())),
    "tick": ("i4", lambda c: c.rows["tick"].to_numpy()),
}
ZERO_COLUMNS = ("bridge", "duty", "dirn", "raw", "trough", "hi", "pos",
                "pre_gap", "rail", "tsw", "ph")


def _runs(s, o, c):
    """The run table rows of one sweep `s` starting at global row `o`."""
    rows = c.rows
    b = rows["bridge"].to_numpy()
    sg = rows["seg"].to_numpy()
    dirn = rows["dir"].to_numpy()
    d = np.abs(rows["duty_counts"].to_numpy())
    raw = rows["current_raw"].to_numpy()
    tr = rows["current_trough"].to_numpy()
    # under the window floor the tap is not the rail
    wv = rows["window_valid"].to_numpy() > 0
    pos = rows["pos"].to_numpy()
    hi = c.hi
    out = []
    for a, e in zip(*run_starts(b, sg, c.gap)):
        h = a + (e - a) // 2
        vbus = np.nan
        if b[a] == DRIVE and wv[a:e].sum() > SETTLE_TICKS:
            vbus = float(c.board.term_v(np.median(hi[a:e][wv[a:e]]),
                                        c.vb_counts))
        out.append(dict(
            sweep=s, seg=int(sg[a]), kind=int(b[a]), i0=o + a, i1=o + e,
            n=e - a, gap_before=int(c.gap[a]), dirn=int(dirn[a]),
            duty=float(np.median(d[a:e])), duty_min=int(d[a:e].min()),
            duty_max=int(d[a:e].max()), vbus=vbus,
            raw=float(raw[a:e].mean()), trough=float(tr[a:e].mean()),
            raw_s=float(raw[h:e].mean()), trough_s=float(tr[h:e].mean()),
            raw_s_sd=float(raw[h:e].std()), trough_s_sd=float(tr[h:e].std()),
            pos0=int(pos[a]), pos1=int(pos[e - 1]),
            pos_rng_s=int(np.ptp(pos[h:e]))))
    return out


def _baseline(o, c, tick_hz):
    """The sweep table's columns of one sweep: its offsets and baseline."""
    b = c.rows["bridge"].to_numpy()
    raw = c.rows["current_raw"].to_numpy()
    tr = c.rows["current_trough"].to_numpy()
    n = len(b)
    drop = int(round(BASE_DROP_S * tick_hz))
    tail = int(round(TAIL_S * tick_hz))
    a0 = drop if (b[:drop + tail] == ASLEEP).all() else SETTLE_TICKS
    nb0 = int(np.argmax(b != ASLEEP)) if (b != ASLEEP).any() else n
    have = nb0 > a0
    return dict(i0=o, i1=o + n, base_n=nb0,
                base_raw=float(raw[a0:nb0].mean()) if have else np.nan,
                base_trough=float(tr[a0:nb0].mean()) if have else np.nan,
                vb_counts=c.vb_counts)


@dataclass
class RowStore:
    """Columns of every sweep of a pair end to end in time order.

    arrays: column name -> one value per row (the global row index).
    sweeps: the index with each sweep's rows i0..i1, its baseline (base_n
    rows before the first awake row; base_raw, base_trough their means
    past the head), vb_counts (the bias node, the median terminal tap
    while asleep), rail (the median drive run's rail, V), ds, sub, stall,
    sload. runs: one row per run, rows i0..i1, its kind (bridge state),
    the gap before it in ticks, duty (the median window, timer ticks),
    vbus (the driven terminal at the crest, V; the sweep's rail where the
    run has no valid window), the means of the crest and trough columns
    over the run and over its second half (_s), the pot at its ends.
    """
    arrays: dict
    sweeps: pd.DataFrame
    runs: pd.DataFrame
    board: boards.Board
    tick_hz: int
    pwm_arr: int


def _build(pair_dir, columns, board):
    ix = index(pair_dir)
    n_tot = int(ix["rows"].sum())
    arr = {k: np.empty(n_tot, np.dtype(COLUMNS[k][0])) for k in columns}
    runs, srec, o, clocks = [], [], 0, set()
    for s, f in enumerate(ix["folder"]):
        rows, meta, segs = load(pair_dir, f, board)
        tick_hz = int(meta["tick_hz"])
        clocks.add((tick_hz, int(meta.get("pwm_period_ticks",
                                          board.pwm_arr))))
        n = len(rows)
        if o + n > n_tot or not isinstance(rows.index, pd.RangeIndex):
            raise ValueError(f"{f}: rows do not match segments.csv")
        b = rows["bridge"].to_numpy()
        hi = np.maximum(rows["vmotor_a"].to_numpy(),
                        rows["vmotor_b"].to_numpy())
        asl = np.flatnonzero(b == ASLEEP)[int(round(BASE_DROP_S
                                                   * tick_hz)):]
        vbc = (float(np.median(rows["vmotor_a"].to_numpy()[asl]))
               if len(asl) else np.nan)
        c = _Sweep(rows, segs, board,
                   segment_gaps(rows["seg"].to_numpy(), segs, tick_hz),
                   hi, vbc, float(ix["t_start"].iat[s]))
        for k in columns:
            arr[k][o:o + n] = COLUMNS[k][1](c)
        runs += _runs(s, o, c)
        p = f.split("/")
        srec.append(dict(_baseline(o, c, tick_hz), ds=p[1],
                         sub="/".join(p[2:]), stall=bool(meta.get("stall")),
                         sload=bool(meta.get("static_load"))))
        o += n
    if o != n_tot:
        raise ValueError(f"{pair_dir}: {o} rows, segments.csv says {n_tot}")
    if len(clocks) != 1:
        raise ValueError(f"{pair_dir}: sweeps on clocks {sorted(clocks)}")
    sw = pd.concat([ix, pd.DataFrame(srec)], axis=1)
    r = pd.DataFrame(runs)
    drive = r["kind"] == DRIVE
    sw["rail"] = (r[drive].groupby("sweep")["vbus"].median()
                  .reindex(range(len(sw))).to_numpy())
    fill = drive & r["vbus"].isna()
    r.loc[fill, "vbus"] = sw["rail"].to_numpy()[r["sweep"][fill]]
    (tick_hz, pwm_arr), = clocks
    return arr, sw, r, tick_hz, pwm_arr


def row_store(pair_dir, columns=ZERO_COLUMNS, board=None, cache_dir=None):
    """The RowStore of a pair's folder with `columns` (names of COLUMNS).

    board: the board the sweeps ran on (default: the one the folder
    names), each sweep's sense block checked against it. cache_dir: keep
    the arrays and tables there and read them back on the next call; the
    default keeps nothing on disk.
    """
    pair_dir = Path(pair_dir)
    board = board or board_of(pair_dir)
    columns = tuple(columns)
    bad = [k for k in columns if k not in COLUMNS]
    if bad:
        raise ValueError(f"unknown columns {bad}")
    got = _read_cache(cache_dir, pair_dir, columns, board)
    if got is None:
        got = _build(pair_dir, columns, board)
        _write_cache(cache_dir, pair_dir, columns, board, *got)
    arr, sw, runs, tick_hz, pwm_arr = got
    if "tsw" in arr:
        t0 = sw["t_start"].to_numpy()[runs["sweep"].to_numpy()]
        tsw = np.asarray(arr["tsw"])
        runs["t0"] = t0 + tsw[runs["i0"].to_numpy()]
        runs["t1"] = t0 + tsw[runs["i1"].to_numpy() - 1]
    return RowStore(arr, sw, runs, board, tick_hz, pwm_arr)


def _cache_key(pair_dir, board):
    return {"pair": Path(pair_dir).name, "board": board.key}


def _read_cache(cache_dir, pair_dir, columns, board):
    if cache_dir is None:
        return None
    d = Path(cache_dir)
    try:
        info = json.loads((d / "store.json").read_text())
    except FileNotFoundError:
        return None
    if (any(info.get(k) != v for k, v in _cache_key(pair_dir, board).items())
            or not set(columns) <= set(info["columns"])):
        return None
    arr = {k: np.load(d / f"{k}.npy", mmap_mode="r") for k in columns}
    return (arr, pd.read_pickle(d / "sweeps.pkl"),
            pd.read_pickle(d / "runs.pkl"), info["tick_hz"], info["pwm_arr"])


def _write_cache(cache_dir, pair_dir, columns, board, arr, sw, runs,
                 tick_hz, pwm_arr):
    if cache_dir is None:
        return
    d = Path(cache_dir)
    d.mkdir(parents=True, exist_ok=True)
    for k, a in arr.items():
        np.save(d / f"{k}.npy", a)
    sw.to_pickle(d / "sweeps.pkl")
    runs.to_pickle(d / "runs.pkl")
    info = dict(_cache_key(pair_dir, board), columns=list(columns),
                tick_hz=tick_hz, pwm_arr=pwm_arr)
    (d / "store.json").write_text(json.dumps(info))
