"""identification.py a second time, on the second motor, mg90-c.

identification.py reads its motor through motor-model's motor_model.py and brush_drop.py, which bind the registry's
default motor at import. motor-model's second_motor.py executes those two files again with every motor constant and
folder pointed at mg90-c. This module executes identification.py again on top of those two copies, into a module of
its own, `idn` here, so every constant it reads comes from mg90-c's result files and every folder is mg90-c's. No
function body is changed. Five pieces are replaced where mg90-c's datasets differ from mg90-b's.

- `MECH`: every mg90-c dataset ran with the black lead on MOT_A, the orientation mg90-b's capture-1 ran with, so the
  mechanical direction is the board's direction everywhere but the swapped block of swapped-1.
- `rest_resistance_cells`: rest-resistance's own cells, run with the registry's default motor set to mg90-c while
  they run, so they read mg90-c's stall pass. The one-body split of the hole map is left out, since rest-resistance
  does not run it on mg90-c.
- `inductance_cells`: inductance's mg90-b cells cannot read mg90-c, which had no reversal or run-up read, so this
  runs the cells inductance's own mg90-c section needs and that section's two cells, which read mg90-c's bursts.
- `runup_windows`: speed-torque's estimator cells, run once as brush_drop runs them, then its capture, run-up and
  count cells again with the capture and the constants rebound to mg90-c, as speed-torque's own mg90-c section
  rebinds them.
- `cycle_folders`: capture-1 on the cycles bemf-coast's mg90-c result carries, as second_motor.py's reduce_jobs.

A worker process imports this module through the pool's initializer, which builds every copy before a job arrives.
"""
import contextlib
import functools
import os
import sys
import types
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from oscnb import motors, results

HERE = Path(__file__).resolve().parent
_MM_DIR = HERE.parent / "motor-model"
if str(_MM_DIR) not in sys.path:
    sys.path.insert(0, str(_MM_DIR))
import second_motor as sm  # noqa: E402

KEY = sm.KEY
mm, bd = sm.mm, sm.bd


def _exec(name, file):
    # identification.py executed into a module of its own, with motor-model's mg90-c copies as its imports
    mod = types.ModuleType(name)
    mod.__file__ = str(HERE / file)
    sys.modules[name] = mod
    saved = {k: sys.modules.get(k) for k in ("motor_model", "brush_drop")}
    sys.modules["motor_model"], sys.modules["brush_drop"] = mm, bd
    try:
        exec(compile((HERE / file).read_text(), mod.__file__, "exec"), mod.__dict__)
    finally:
        for k, v in saved.items():
            if v is None:
                del sys.modules[k]
            else:
                sys.modules[k] = v
    return mod


idn = _exec("second_motor_idn", "identification.py")
assert idn.MOTOR.key == KEY and idn.PAIR == sm.PAIR

idn.MECH = {"capture-1": 1, "speed-torque": 1, "meter-1": 1, "high-1": 1, "swapped-wired": 1, "swapped-swapped": -1}

# rest-resistance's cells without the one-body split (19018c53), which rest-resistance does not run on mg90-c
RR_CELLS = tuple(c for c in idn.RR_CELLS if c != "19018c53")
# inductance's mg90-b cells that its mg90-c section reads, then that section's reading of mg90-c's bursts
IND_CELLS = idn.IND_CELLS + ("b82dfc38", "7529243b", "4f7b00df", "44173f2e", "fb66b760", "d7374be2", "1660e598",
                             "754f6d7f", "ed33bff0")
# speed-torque's cells that read a capture: its cycles, its run-ups with their dropout rows, and the count
ST_RECOUNT = ("98a52b0f", "4c4b548a", "64a4b80c", "1331f6dd")


@contextlib.contextmanager
def _default_motor(key):
    # the registry's default motor for the cells that bind it at their first cell
    saved = motors.DEFAULT
    motors.DEFAULT = key
    try:
        yield
    finally:
        motors.DEFAULT = saved


@functools.lru_cache(maxsize=None)
def rest_resistance_cells():
    with _default_motor(KEY):
        ns = idn.notebook_cells("rest-resistance", RR_CELLS)
    assert ns["MOTOR"].key == KEY
    return ns


@functools.lru_cache(maxsize=None)
def inductance_cells():
    """inductance's mg90-c reading, under the names identification reads from mg90-b's: FITS, C2 and CHANGE"""
    ns = idn.notebook_cells("inductance", IND_CELLS)
    assert ns["MOTOR_C"].key == KEY
    return dict(FITS=ns["FITS_C"], C2=ns["C2C"], CHANGE=ns["CHANGE"], R_HOLE=ns["R_HOLE_C"])


def _cells_into(g, nb, ids):
    # the named cells of a notebook executed into the namespace g, in the notebook's folder, output kept out
    import json
    import warnings
    import matplotlib.pyplot as plt
    from IPython.utils.capture import capture_output
    path = idn.NB2 / nb / f"{nb}.ipynb"
    cells = {c.get("id"): c for c in json.loads(path.read_text())["cells"]}
    here, show0, figs = os.getcwd(), plt.show, set(plt.get_fignums())
    plt.show = lambda *a, **k: None
    try:
        os.chdir(path.parent)
        with capture_output(), warnings.catch_warnings():
            warnings.simplefilter("ignore")
            for c in ids:
                exec(compile("".join(cells[c]["source"]), f"{nb}.ipynb cell {c}", "exec"), g)
    finally:
        os.chdir(here)
        plt.show = show0
        for n in set(plt.get_fignums()) - figs:
            plt.close(n)


@functools.lru_cache(maxsize=None)
def runup_cells():
    """speed-torque's namespace after its count of mg90-c's capture-1"""
    g = {"__name__": "speed_torque_cells"}
    _cells_into(g, "speed-torque", bd.ST_CELLS[:1])
    g["show"] = lambda *a, **k: None
    _cells_into(g, "speed-torque", bd.ST_CELLS[1:])
    # rebound as speed-torque's mg90-c section rebinds them: the capture, Ke, Kt, the drag curves, L and R
    nl = results.read("2-motor", "no-load-current", pair=KEY)
    nl_b, g["NL"] = g["NL"], nl
    g["I0"], g["I0_HI"] = g["drag_curve"]("5 V"), g["drag_curve"]("7.4 V")
    g["NL"] = nl_b
    g["CAPTURE"] = sm.PAIR / "speed-torque" / "capture-1"
    g["KE"] = results.read("2-motor", "bemf-coast", pair=KEY)["ke_v_s_per_rad"]["value"]
    g["KT"] = nl["kt_n_m_per_a"]["value"]
    g["L_H"] = results.read("2-motor", "inductance", pair=KEY)["l_mh"]["value"] * 1e-3
    g["RR"] = results.read("2-motor", "rest-resistance", pair=KEY)
    _cells_into(g, "speed-torque", ST_RECOUNT)
    assert len(g["RUNS"]) == 2 * len(g["cycles"])                 # both directions of every cycle
    return g


def runup_windows():
    """speed-torque's turn-window table of mg90-c's capture-1, as brush_drop's runup_windows returns it"""
    g = runup_cells()
    return g["REV"].copy(), g["RUNS"], g["L_H"], g["T_OFFSET"], g["V_PER_COUNT"], g["SETTLED"]


_cycle_folders = idn.cycle_folders


def cycle_folders():
    """identification's cycle_folders with capture-1 on the cycles bemf-coast's mg90-c result carries"""
    return [f for f in _cycle_folders() if f[0] != "capture-1" or int(f[1]["k"]) in sm._KEPT]


idn.rest_resistance_cells, idn.inductance_cells = rest_resistance_cells, inductance_cells
idn.runup_windows, idn.cycle_folders = runup_windows, cycle_folders


def _init():
    """the pool's initializer: importing this module in the worker built the copies"""


def run_parallel(jobs, workers=None, chunksize=1, cost=None):
    """motor_model's run_parallel, each worker holding the mg90-c copies of all three modules"""
    workers = workers or os.cpu_count()
    order = list(range(len(jobs))) if cost is None else sorted(range(len(jobs)), key=lambda i: -cost[i])
    with ProcessPoolExecutor(max_workers=workers, initializer=_init) as ex:
        res = list(ex.map(mm._call, [jobs[i] for i in order], chunksize=chunksize))
    out = [None] * len(jobs)
    for i, r in zip(order, res):
        out[i] = r
    return out


mm.run_parallel = run_parallel                      # the identification copy calls mm.run_parallel
