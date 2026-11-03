"""Stage 2, bemf-coast: the free motor spun at a set duty, then the bridge
opened, the terminal voltage decaying as the rotor coasts. The spin's tail
is the no-load current, read by the no-load-current notebook from the same
datasets. Every plan here is a free-rotor plan, with or without the meter,
run by `python -m oscnb.sitting free` or `meter`; a cycle is one `osc-lab
sweep`, a spin and a coast in both directions, and a dataset is
`cycles/K` folders with one row per cycle in `cycles.csv`.

In the free state:

`ref`, the bracket, is three cycles of 30 % on 5.0 V at the open and the
close of every free sitting, so a drift of the coast's drag or the brush
drop through the sitting shows against the same rung.

`capture` is the ladder: three rails at five duties, three cycles each, a
spin of a second and a half, long enough for the speed to settle, then a
coast of most of a second. The rails are the 1S and 2S levels and 3.7 V,
the cell floor; the duties stay under the motor's rated voltage as rail
times duty, which the sitting enforces.

`low` is the slow rungs. The duty that starts a motor from rest is set by
its breakaway, and a turning motor holds a lower duty than that, so each
cycle kicks the rotor for 300 ms at a duty that turns it near 150 rev/s and
then drops to the rung's duty, chained with no torque-off between, holds
it for two seconds and coasts. The last 200 ms of the hold are the rung,
whether the rotor kept turning or stopped, which the ripple tach tells.

`swapped`, optional, runs the 40 and 60 % cycles on 7.4 V with the leads
as wired, then again with the motor's two leads swapped at the board's
connector. A split between the directions that follows the rotor is the
motor's, one that follows the board's direction is the board's. On the
first motor it followed the rotor.

`threshold-70` and `threshold-75` are the sacrificial exception to the
rated voltage, named only when a motor's other captures are done: three
cycles of one second at 7.4 V and that duty, between the 67 % that reads
like 5 V at 100 % and the 80 % that changed a motor's brushes within
seconds, each followed by three 5 V 100 % run-ups in `runup/` so the duty
the change starts at is bracketed.

In the meter state, the DMM on DC mA in series with one motor lead and
logging over Bluetooth beside the spins into `dmm.csv`, for the true mean
current of a switching drive against the board's reading of it:

`meter` is 30 % on 5.0 V and 20 % on 7.4 V, which turn near the same
speed, and 40 % on 7.4 V, with spins of three seconds, the longest a TEL
stream holds, so the meter has a settled stretch to read in each.

`high` is 60, 80 and 100 % on 5.0 V, how the board's reading term falls
toward full duty. At 100 % nothing switches, so the crest sample is the
mean, and what the board and the meter still disagree by there is the
zero of a driven bridge.

`high7` is 50, 60 and 67 % on 7.4 V with spins of one second, the dropout
rate and the dropout rows' zero at the rail where the two zeros part,
every rung under the rated voltage.

`low-meter` is the slow rungs again, kicked and held as in the
free state, at 8 to 30 % on the three rails, the duties the meter runs
never reached and the rail they never saw. The meter's burden, near ten
ohms in a loop of four, holds the rotor well under the free rungs' speeds
and stops it at the lowest, which the ripple tach tells; the term is the
crest against the meter's mean either way.

`spinning`, optional, records `osc-lab burst` captures of the free rotor
already turning at the meter run's three points, the shunt and both taps
through whole PWM periods, in slow and in fast decay, so the board's own
period mean sits beside the meter's.
"""
from oscnb import capture, osc
from oscnb.sitting import Plan

LEVELS = [3.7, 5.0, 7.4]
DUTIES = [15, 20, 30, 40, 60]
SPIN_MS = 1500
COAST_MS = 800
REF = [(5.0, 30)]
# (rail volts, duty %): 30 % on 5.0 V and 20 % on 7.4 V turn near 150 rev/s, 40 % on 7.4 V is the zero question's spin
METER = [(5.0, 30), (7.4, 20), (7.4, 40)]
METER_SPIN_MS = 3000
METER_COAST_MS = 300
HIGH = [(5.0, 60), (5.0, 80), (5.0, 100)]
HIGH7 = [(7.4, 50), (7.4, 60), (7.4, 67)]
HIGH7_SPIN_MS = 1000
LOW = [(5.0, 12), (5.0, 10), (5.0, 8), (5.0, 6), (7.4, 8), (7.4, 6), (7.4, 4)]
KICK = {3.7: 40, 5.0: 30, 7.4: 20}    # the kick's duty by rail, near 150 rev/s on each
LOW_HOLD_MS = 2000
LOW_COAST_MS = 600
# the lowest rungs the meter's burden still lets the rotor hold, down to 0.6 V effective
LOW_METER = [(7.4, 15), (7.4, 12), (7.4, 10), (7.4, 8), (5.0, 20), (5.0, 15), (5.0, 12), (5.0, 10), (3.7, 30), (3.7, 20), (3.7, 16)]
LOW_METER_HOLD_MS = 2500
SWAPPED = [(7.4, 40), (7.4, 60)]
THRESHOLD = [70, 75]
RUNUP = [(5.0, 100)]
RUNUP_SPIN_MS = 300
RUNUP_COAST_MS = 600
BURST_SPIN_MS = 1500
BURST_REST_MS = 1500         # the rotor coasts to a stop between captures


def ref(s, out):
    s.cycles(out, REF, SPIN_MS, COAST_MS)


def ladder(s, out):
    s.cycles(out, [(v, d) for v in LEVELS for d in DUTIES], SPIN_MS, COAST_MS)


def low(s, out):
    s.cycles(out, LOW, LOW_HOLD_MS, LOW_COAST_MS, kick=KICK)


def swapped(s, out):
    s.cycles(out, SWAPPED, SPIN_MS, COAST_MS, folder="wired")
    s.power_down()
    s.prompt("Swap the motor's two leads at the board's connector.")
    s.cycles(out, SWAPPED, SPIN_MS, COAST_MS, folder="swapped")
    s.power_down()
    s.prompt("Swap the motor's leads back as usually wired.")


def threshold(pct):
    def run(s, out):
        s.prompt(f"This plan puts 7.4 V at {pct} % on the motor, over its rated {s.motor.rated_v} V, and may change its brushes for good.")
        s.cycles(out, [(7.4, pct)], HIGH7_SPIN_MS, METER_COAST_MS, over_rating=True)
        s.cycles(out, RUNUP, RUNUP_SPIN_MS, RUNUP_COAST_MS, folder="runup")
    return run


def metered(rungs, spin_ms, kick=None):
    def run(s, out):
        with s.meter_log(out):
            s.cycles(out, rungs, spin_ms, METER_COAST_MS, kick=kick)
    return run


def spinning(s, out):
    marks = capture.Marks(out)
    with s.meter_log(out):
        for v, d in METER:
            s.cap(v, d)
            s.rail(v)
            for decay in ("slow", "fast"):
                group = f"{v:.1f}v-{d}-{decay}"
                marks.mark(group)
                osc.bursts(out / group, [d], repeats=3, chans="diff", dirs="both", rest_ms=BURST_REST_MS, decay=decay, spin_ms=BURST_SPIN_MS)
                s.rest(f"{group} done, the motor resting")
    marks.close()


PLANS = [
    Plan("ref", "bemf-coast", "free", "three cycles of 30 % on 5 V at the open and the close", run=ref, order=0, bracket=True),
    Plan("capture", "bemf-coast", "free", "the ladder: 3.7, 5 and 7.4 V at 15 to 60 %, three cycles each", run=ladder, order=10),
    Plan("low", "bemf-coast", "free", "kick then hold at 4 to 12 %, the slow rungs", run=low, order=30),
    Plan("swapped", "bemf-coast", "free", "7.4 V 40 and 60 % as wired, then with the leads swapped", run=swapped, order=40, optional=True),
    *[Plan(f"threshold-{p}", "bemf-coast", "free", f"7.4 V at {p} %, over the rating, then 5 V run-ups", run=threshold(p), order=p, over_rating=True)
      for p in THRESHOLD],
    Plan("meter", "bemf-coast", "meter", "5 V 30 %, 7.4 V 20 and 40 %, 3 s spins", run=metered(METER, METER_SPIN_MS), order=10),
    Plan("high", "bemf-coast", "meter", "5 V 60, 80 and 100 %", run=metered(HIGH, METER_SPIN_MS), order=20),
    Plan("high7", "bemf-coast", "meter", "7.4 V 50, 60 and 67 %, 1 s spins", run=metered(HIGH7, HIGH7_SPIN_MS), order=30),
    Plan("low-meter", "bemf-coast", "meter", "kick then hold at 8 to 30 % on the three rails, metered", run=metered(LOW_METER, LOW_METER_HOLD_MS, kick=KICK), order=35),
    Plan("spinning", "bemf-coast", "meter", "bursts of the turning rotor at the meter points", run=spinning, order=40, optional=True),
]
