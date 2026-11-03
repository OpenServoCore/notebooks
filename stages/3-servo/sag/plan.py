"""Stage 3, sag: the servo under drive as the rail falls from a full 2S pack
to its floor, for the rail-tracking duty cap, the zero tracker's step with
the rail and the undervolt behavior stage 6 designs. Optional plans of
`python -m oscnb.sitting servo`, the horn lock off, run on request.

`capture` is the first pass: thirteen rails 0.2 V apart from 8.4 V down to
6.0 V, and at each a lap chain of 1.5 V effective into the low stop, one
crawl cycle both ways, a kick then a hold at 0.5 V effective, and one 30 %
cycle. The DPS steps only between sweeps, with torque off: a DPS step down
has reset the servo once, so no drive spans one.

`held`, run only when the first pass came through clean, steps the DPS
down the same rails while osc-lab holds a low stall at the low stop, the
hold unstreamed between two short windows, so a rail falling under drive
is recorded where the first pass only ever saw a still bridge.

A dataset is `levels.csv`, one row per sweep, and `levels/<rail>/<what>`
sweeps; `held` is one sweep with the DPS steps in `steps.csv`.
"""
import time

from oscnb import bench
from oscnb.sitting import Plan, Skip, log_row

LEVELS = tuple(round(8.4 - 0.2 * k, 1) for k in range(13))
STOP_V = 1.5           # effective volts of the stop chain, 0.34 A
STOP_MS = 300
CRAWL_V = 0.5          # the crawl's hold, effective volts
KICK_V = 1.5           # the crawl's kick, effective volts
KICK_MS = 150
CRAWL_MS = 1200
CYCLE_DUTY = 30
CYCLE_MS = 700
COAST_MS = 400
HELD_V = 1.0           # the held pass's stall, effective volts on the top rail
HELD_WINDOW_MS = 100
HELD_STEP_S = 2.0      # each DPS level held this long under the stall
HELD_LEAD_S = 5.0      # the seek and the first window before the first step


def duty(volts, effective):
    return max(1, round(100 * effective / volts))


def schedules(s, v):
    """(what, steps, dirs, seat) at rail `v`, the free windows sized."""
    stop = [f"{duty(v, STOP_V)}@{STOP_MS}"]
    kick, crawl = duty(v, KICK_V), duty(v, CRAWL_V)
    (hold_ms,) = s.fit(v, [(crawl, CRAWL_MS)], fixed=[(kick, KICK_MS)])
    (cycle_ms,) = s.fit(v, [(CYCLE_DUTY, CYCLE_MS)])
    return [
        ("stop", stop, "rev", "stop"),
        ("crawl", [f"{kick}@{KICK_MS}", f"then:{crawl}@{hold_ms}",
                   f"coast:{COAST_MS}"], "both", None),
        ("cycle", [f"{CYCLE_DUTY}@{cycle_ms}", f"coast:{COAST_MS}"], "both",
         None),
    ]


def capture(s, out):
    for v in LEVELS:
        s.check(v, [str(duty(v, STOP_V))], seat="stop")
        s.check(v, [str(duty(v, KICK_V)), str(CYCLE_DUTY)])
    k = 0
    for v in LEVELS:
        try:
            plan = schedules(s, v)
        except Skip as e:
            s.note("skip", f"{v} V: {e}")
            continue
        for what, steps, dirs, seat in plan:
            sub = f"levels/{v:g}V/{what}"
            t0 = time.time()
            s.drive(out / sub, v, steps, dirs=dirs, seat=seat)
            dps_read, vsys = s.rail_read
            log_row(out / "levels.csv", {
                "k": k, "volts_set": v, "dps_read": f"{dps_read:.3f}",
                "vsys_read": f"{vsys:.3f}", "what": what,
                "schedule": ",".join(steps), "t_start": f"{t0:.3f}",
                "t_end": f"{time.time():.3f}", "folder": sub})
            k += 1
            s.rest(f"{v} V {what} done, the motor resting")


def held(s, out):
    d = duty(LEVELS[0], HELD_V)
    hold_ms = int((HELD_LEAD_S + HELD_STEP_S * len(LEVELS)) * 1000)
    steps = [f"{d}@{HELD_WINDOW_MS}", f"hold:{d}@{hold_ms}",
             f"then:{d}@{HELD_WINDOW_MS}"]
    s.prompt("The DPS now steps down under a stall at the low stop; a step "
             "down has reset the servo once. Hands clear of the horn.")
    proc = s.drive(out / "held", LEVELS[0], steps, dirs="rev", seat="stop",
                   background=True)
    time.sleep(HELD_LEAD_S)
    for v in LEVELS[1:]:
        t0 = time.time()
        read = bench.set_volts(v + bench.DIODE_V, hold_s=HELD_STEP_S)
        log_row(out / "steps.csv", {"volts_set": v, "dps_read": f"{read:.3f}",
                                    "t_start": f"{t0:.3f}",
                                    "t_end": f"{time.time():.3f}"})
        s.note("dps", f"{v} V under the hold: DPS reads {read:.3f} V")
    s.level = None
    if proc.wait() != 0:
        raise SystemExit(f"the held sweep ended with {proc.returncode}: "
                         "did the servo reset?")


PLANS = [
    Plan("capture", "sag", "servo",
         "8.4 down to 6.0 V: a stop chain, a crawl and a 30 % cycle at each",
         run=capture, order=70, optional=True, minutes=24, mb=205),
    Plan("held", "sag", "servo",
         "the DPS stepped down 8.4 to 6.0 V under a stall at the low stop",
         run=held, order=75, optional=True, seated=True, minutes=2, mb=1),
]
