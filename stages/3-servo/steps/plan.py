"""Stage 3, steps: the current's and the speed's response to duty steps of
the free servo, for the plant stage 5 designs its loops on. Free-rotor
plans of `python -m oscnb.sitting servo`, the horn lock off, 5 V.

`capture` runs chains of a base duty, a step up and back, `d0`, then
`d0 + s`, then `d0`, chained with no release, at bases 20, 30 and 40 % and
steps of 2, 5 and 10 %, both directions, from three starts: the guards,
then the guards moved in by a sixth and by a third of the travel. A chain
is 400, 300 and 300 ms wanted, all three cut by one factor so the chain
stays on the runway from its start.

`reversal` runs `d` then `-d`, chained, at 10, 20 and 30 %, each half the
same window, so the backlash crossing sits in the middle of one segment
pair. Only the outbound half counts against the runway, the return heads
back toward the start.

A dataset is `chains.csv`, one row per sweep, and `chains/K` sweeps.
"""
import time

from oscnb.sitting import Plan, Skip, log_row

LEVEL = 5.0
BASES = (20, 30, 40)
STEP_SIZES = (2, 5, 10)
CHAIN_MS = (400, 300, 300)      # base, stepped, back, wanted
STARTS = (0.0, 1 / 6, 1 / 3)    # share of the travel the guards move in by
REVERSALS = (10, 20, 30)
REVERSAL_MS = 300


def start_guards(s, share):
    """The guards moved in by `share` of the travel from each end, and the
    runway a chain from either has toward the far guard."""
    lo, hi = s.guards()
    inset = round((hi - lo) * share)
    return (lo + inset, hi - inset), s.runway((lo + inset, hi))


def capture(s, out):
    for d0 in BASES:
        for step in STEP_SIZES:
            s.check(LEVEL, [str(d0), str(d0 + step)])
    k = 0
    for share in STARTS:
        guards, runway = start_guards(s, share)
        for d0 in BASES:
            for step in STEP_SIZES:
                segments = list(zip((d0, d0 + step, d0), CHAIN_MS))
                try:
                    base, up, back = s.fit(LEVEL, segments, runway=runway)
                except Skip as e:
                    s.note("skip", f"chain {d0} + {step} %: {e}")
                    continue
                steps = [f"{d0}@{base}", f"then:{d0 + step}@{up}",
                         f"then:{d0}@{back}"]
                sub = f"chains/{k}"
                t0 = time.time()
                s.drive(out / sub, LEVEL, steps, guards=guards)
                log_row(out / "chains.csv", {
                    "k": k, "volts_set": LEVEL, "start_share": f"{share:.3f}",
                    "guard_lo": guards[0], "guard_hi": guards[1],
                    "base_pct": d0, "step_pct": step,
                    "windows_ms": f"{base}/{up}/{back}",
                    "t_start": f"{t0:.3f}", "t_end": f"{time.time():.3f}",
                    "folder": sub})
                k += 1
                s.rest(f"chain {k - 1} done, the motor resting")


def reversal(s, out):
    for d in REVERSALS:
        s.check(LEVEL, [str(d)])
    for k, d in enumerate(REVERSALS):
        try:
            (ms,) = s.fit(LEVEL, [(d, REVERSAL_MS)])
        except Skip as e:
            s.note("skip", f"reversal {d} %: {e}")
            continue
        sub = f"chains/{k}"
        t0 = time.time()
        s.drive(out / sub, LEVEL, [f"{d}@{ms}", f"then:{-d}@{ms}"])
        log_row(out / "chains.csv", {
            "k": k, "volts_set": LEVEL, "duty_pct": d, "window_ms": ms,
            "t_start": f"{t0:.3f}", "t_end": f"{time.time():.3f}",
            "folder": sub})
        s.rest(f"reversal {k} done, the motor resting")


PLANS = [
    Plan("capture", "steps", "servo",
         "base 20 to 40 %, steps of 2 to 10 % up and back, three starts",
         run=capture, order=50, minutes=20, mb=110),
    Plan("reversal", "steps", "servo",
         "d then -d at 10, 20 and 30 %, chained",
         run=reversal, order=55, minutes=3, mb=12),
]
