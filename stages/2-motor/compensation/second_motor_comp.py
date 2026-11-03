"""compensation.py a second time, on the second motor, mg90-c.

compensation.py reads its motor through identification.py, which binds the registry's default motor at import.
identification's second_motor_ident.py executes identification.py again with every motor constant and folder pointed
at mg90-c, on top of motor-model's second_motor.py copies. This module executes compensation.py again on top of that
copy, into a module of its own, `cp` here, so every constant it reads comes from mg90-c's result files and every
folder is mg90-c's. No function body is changed.

Two pieces are added where mg90-c's datasets differ from mg90-b's.

- `split_ladder`: mg90-c's ladder ran a 3.7 V level beside 5.0 and 7.4 V. The V_b floor compares the low rungs of two
  rails, so every V_b fit reads the 5.0 and 7.4 V rungs, as on mg90-b, and the 3.7 V rungs are scored and never
  fitted.
- `score_pass`: compensation's score_pass with the 3.7 V rungs as one more held-out set.

A worker process imports this module's copies through second_motor_ident's pool, whose initializer builds them.
"""
import sys
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
_ID_DIR = HERE.parent / "identification"
if str(_ID_DIR) not in sys.path:
    sys.path.insert(0, str(_ID_DIR))
import second_motor_ident as smi  # noqa: E402

KEY = smi.KEY
idn = smi.idn


def _exec(name, file):
    # compensation.py executed into a module of its own, with identification's mg90-c copy as its import
    mod = types.ModuleType(name)
    mod.__file__ = str(HERE / file)
    sys.modules[name] = mod
    saved = sys.modules.get("identification")
    sys.modules["identification"] = idn
    try:
        exec(compile((HERE / file).read_text(), mod.__file__, "exec"), mod.__dict__)
    finally:
        if saved is None:
            del sys.modules["identification"]
        else:
            sys.modules["identification"] = saved
    return mod


cp = _exec("second_motor_cp", "compensation.py")
assert cp.MOTOR.key == KEY and cp.idn is idn

FIT_RAILS = (5.0, 7.4)          # the rails every V_b fit reads, the two whose low rungs the floor compares
LOW_RAIL = 3.7                  # the rail mg90-c's ladder adds, scored and never fitted
CAPS = ["capture-1", "capture-1 3.7 V"] + cp.CAPS[1:]


def split_ladder(T):
    """capture-1's spins, mechanical direction tagged, as the fitted rails and the 3.7 V rail"""
    T = cp.tag_mech(T, "capture-1")
    return T[T["level"].isin(FIT_RAILS)], T[T["level"] == LOW_RAIL]


def score_pass(C1_test, C37_test, SW, RU, RS, P, form="table"):
    """compensation's score_pass for one pass P, with the 3.7 V rungs C37_test scored like capture-1's"""
    out = cp.score_pass(C1_test, SW, RU, RS, P, form)
    S = cp.score_spins(C37_test, P.get("fit_c1", P["fit"]), P.get("ftab_c1", P["ftab"]), "capture-1", P["r"],
                       P["ke"], form)
    out["capture-1 3.7 V"] = cp.bands(S)
    return {c: out[c] for c in CAPS}
