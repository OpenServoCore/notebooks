"""motor_model.py and brush_drop.py a second time, on the second motor, mg90-c.

Both modules read their motor constants once, at import, for the registry's default motor, and the notebook's
sections run on them as imported. The second-motor section runs the same code on mg90-c. This module executes the
two files again into modules of their own, `mm` and `bd` here, and points every motor constant and capture folder at
mg90-c's own result files and telemetry. The board's constants are shared, since both motors ran on board D. No
function body is changed. Four functions are replaced where mg90-c's datasets differ from mg90-b's.

- `slow_folds`: the motor's slow decay is fast-1's two slow groups and inductance's capture-1 at hole 11. mg90-c had
  no reversal run.
- `job_variant`: the R variant mg90-b read from the meter's ohms at rest reads the stall pass's graded lines here,
  since the meter's route was not run on mg90-c.
- `coast_jobs` and brush_drop's `reduce_jobs`: the same four captures under mg90-c's folder, capture-1 on the cycles
  bemf-coast's mg90-c result carries. Its ladder ran a 3.7 V level that mg90-b's did not, and bemf-coast left out the
  three cycles at 3.7 V 15 %, whose coast falls under the turn threshold before the window holds its ripple cycles.

bemf-coast's mg90-c file carries no spinning run, so `spinning_by_point` is empty here and the spinning points are
the coast and meter runs alone. The teardown's constants (the lead zero, the hand readings) serve only the coil
runs, which mg90-c did not get, and are set to None so any use fails. A worker process imports this module through
the pool's initializer, which registers the two copies before any job arrives.
"""
import os
import sys
import types
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from oscnb import motors, results

KEY = "mg90-c"
HERE = Path(__file__).resolve().parent


def _exec(name, file):
    # one of the two files executed into a module of its own, registered so a worker can unpickle its functions
    mod = types.ModuleType(name)
    mod.__file__ = str(HERE / file)
    sys.modules[name] = mod
    exec(compile((HERE / file).read_text(), mod.__file__, "exec"), mod.__dict__)
    return mod


mm = _exec("second_motor_mm", "motor_model.py")
_real = sys.modules.get("motor_model")
sys.modules["motor_model"] = mm                    # brush_drop's `import motor_model as mm` binds this copy
try:
    bd = _exec("second_motor_bd", "brush_drop.py")
finally:
    if _real is not None:
        sys.modules["motor_model"] = _real
    else:
        del sys.modules["motor_model"]

# ---------------------------------------------------------------- mg90-c's constants, each from its own result file

MOTOR = motors.MOTORS[KEY]
RR = results.read("2-motor", "rest-resistance", pair=KEY)
BEMF = dict(results.read("2-motor", "bemf-coast", pair=KEY), spinning_by_point={})   # no spinning run on mg90-c
SPT = results.read("2-motor", "speed-torque", pair=KEY)
NLC = results.read("2-motor", "no-load-current", pair=KEY)
IND_RES = results.read("2-motor", "inductance", pair=KEY)
PAIR = mm.TEL / f"{KEY}__{mm.BOARD.key}"
R_GRADED = RR["r_stall_graded_lines_ohm"]["value"]   # the stall pass's graded lines, standing in for the ohms route

mm.MOTOR, mm.RR, mm.BEMF, mm.SPT, mm.NLC = MOTOR, RR, BEMF, SPT, NLC
mm.TD = mm.LEAD_ZERO = mm.HAND = mm.HAND_DIR = mm.LV = None   # the teardown's, for the coil runs alone
mm.KE = BEMF["ke_v_s_per_rad"]["value"]
mm.J = SPT["j_kg_m2"]["value"]
mm.N_R = MOTOR.ripple_per_rev
mm.PAIR = PAIR
mm.IND = PAIR / "inductance"
mm.SETS = {}
mm.OLD = {"motor": mm.IND / "fast-1" / "hole-11"}
mm.BC = PAIR / "bemf-coast"
mm.STQ = PAIR / "speed-torque" / "capture-1"
mm.IND_RUNS = {"cap-11": mm.IND / "capture-1" / "hole-11", "motor": mm.IND / "fast-1" / "hole-11"}


def slow_folds(s):
    """the motor's slow-decay folds: fast-1's slow groups and capture-1 at hole 11"""
    SL = mm.pick(mm.cached_folds(s), "slow")
    if s == "motor":
        SL = SL + list(mm.cached_folds("cap-11"))
    return SL


VARIANTS = [("band", a) for a in (100, 150, 200, 250)] + [("R", "graded lines"), ("R", "stall R x 0.95")]


def job_variant(s, key, which, x0):
    """one variant of the motor's fast-decay fit: ("band", start) or ("R", name), the graded lines' R in place of
    the meter's ohms"""
    m = mm.model_of(s, key)
    if which[0] == "band":
        return mm.fit(m, mm.fast_folds(s), a=which[1], x0=x0)
    fn = {"graded lines": _r_graded, "stall R x 0.95": _r_low}[which[1]]
    return mm.fit(m, mm.fast_folds(s), a=mm.band_of(s), rs=fn, x0=x0)


def _r_graded(f):
    return R_GRADED


def _r_low(f):
    return 0.95 * mm.rs_of(f)


_KEPT = {int(r["k"]) for r in BEMF["cycles"]}       # capture-1's cycles in bemf-coast's mg90-c result
_coast_jobs, _reduce_jobs = mm.coast_jobs, bd.reduce_jobs


def _kept(jobs):
    # the jobs of capture-1's cycles that bemf-coast's mg90-c result carries, and every job of the other runs
    return [j for j in jobs if j[1][0] != "capture-1" or int(j[1][2]["k"]) in _KEPT]


def coast_jobs():
    return _kept(_coast_jobs())


def reduce_jobs(runs=("capture-1", "speed-torque", "meter-1", "high-1")):
    return _kept(_reduce_jobs(runs))


mm.slow_folds, mm.job_variant, mm.coast_jobs = slow_folds, job_variant, coast_jobs
mm.cached_folds.cache_clear()

bd.BOARD, bd.MOTOR, bd.KE, bd.J = mm.BOARD, MOTOR, mm.KE, mm.J
bd.RR, bd.BEMF, bd.SPT, bd.IND = RR, BEMF, SPT, IND_RES
bd.KE_LO, bd.KE_HI = BEMF["ke_v_s_per_rad"]["lo"], BEMF["ke_v_s_per_rad"]["hi"]
bd.R_LOCKED = RR["r_term_median_ohm"]["value"]
bd.R_LOCKED_LO, bd.R_LOCKED_HI = RR["r_term_median_ohm"]["lo"], RR["r_term_median_ohm"]["hi"]
bd.R_LOCKED_AMPS = RR["r_term_amps"]["value"]
bd.R_RUNUP = bd.R_RUNUP_AMPS = bd.R_OHMS = np.nan    # not read on mg90-c
bd.TAIL_ZERO = {int(k): v["value"] for k, v in BEMF["tail_zero_counts_by_direction"].items()}
bd.TELE = PAIR
bd.RUNS = {"capture-1": PAIR / "bemf-coast" / "capture-1", "meter-1": PAIR / "bemf-coast" / "meter-1",
           "high-1": PAIR / "bemf-coast" / "high-1", "speed-torque": PAIR / "speed-torque" / "capture-1"}
bd.SPUN = None
bd.reduce_jobs = reduce_jobs


def _init():
    """the pool's initializer: importing this module in the worker built the two copies"""


def run_parallel(jobs, workers=None, chunksize=1, cost=None):
    """motor_model's run_parallel, each worker holding the mg90-c copies"""
    workers = workers or os.cpu_count()
    order = list(range(len(jobs))) if cost is None else sorted(range(len(jobs)), key=lambda i: -cost[i])
    with ProcessPoolExecutor(max_workers=workers, initializer=_init) as ex:
        res = list(ex.map(mm._call, [jobs[i] for i in order], chunksize=chunksize))
    out = [None] * len(jobs)
    for i, r in zip(order, res):
        out[i] = r
    return out
