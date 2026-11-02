"""Stage 2, bemf-coast: the free motor spun at a set duty, then the bridge
opened, the terminal voltage decaying as the rotor coasts. The spin's tail
is the no-load current, read by the no-load-current notebook from this
same dataset. Needs the bare motor in the rig with the wrench bar OFF, the
board on the DPS and the `osc` and `osc-lab` tools; no meter.

  uv run python stages/2-motor/bemf-coast/capture.py [N]            writes telemetry/<motor>__<board>/bemf-coast/capture-N
  uv run python stages/2-motor/bemf-coast/capture.py meter [N]      writes telemetry/<motor>__<board>/bemf-coast/meter-N
  uv run python stages/2-motor/bemf-coast/capture.py high [N]       writes telemetry/<motor>__<board>/bemf-coast/high-N
  uv run python stages/2-motor/bemf-coast/capture.py swapped [N]    writes telemetry/<motor>__<board>/bemf-coast/swapped-N
  uv run python stages/2-motor/bemf-coast/capture.py spinning [N]   writes telemetry/<motor>__<board>/bemf-coast/spinning-N
  uv run python stages/2-motor/bemf-coast/capture.py high7 [N]      writes telemetry/<motor>__<board>/bemf-coast/high7-N
  uv run python stages/2-motor/bemf-coast/capture.py ref [N]        writes telemetry/<motor>__<board>/bemf-coast/ref-N
  uv run python stages/2-motor/bemf-coast/capture.py low [N]        writes telemetry/<motor>__<board>/bemf-coast/low-N

A capture on a motor other than the registry's default sets OSC_MOTOR to
its key. `nometer` after `high` or `high7` runs the plan on the board's
readings alone, no meter in the loop, into `<mode>-nometer-N`.

One cycle is a spin of a second and a half, long enough for the speed to
settle, then a coast of most of a second, both directions, as one sweep.
Cycles rest twenty seconds apart, the bench rule for a bare motor.

`meter` puts the DMM on DC mA in series with one motor lead and logs it
over Bluetooth beside the spins, for the true mean current of a switching
drive against the board's reading of it. The winding's time constant is
long against the 50 us PWM period, so the meter's mean is the winding's
mean. The duties are the ones the no-load-current and bemf-coast open
questions name, 30 % on 5.0 V and 20 % on 7.4 V near the same speed, and
40 % on 7.4 V. The spin is three seconds, the longest a TEL stream holds,
so the meter has a settled stretch to read in each. The meter's burden sits
in the loop and slows the motor a little, and both readings see the same
current.

`high` is the meter run again at 60, 80 and 100 % on 5.0 V, how the
board's reading term falls toward full duty. At 100 % nothing switches, so
the crest sample is the mean, and what the board and the meter still
disagree by there is the zero of a driven bridge.

`high7` is `high` on 7.4 V with spins of one second, for the dropout rate
and the dropout rows' zero at the rail where the two zeros part. No free
spin puts more than the motor's rated voltage on it as rail times duty,
the registry's `rated_v`: on mg90-c the 7.4 V 80 and 100 % spins, 625 to
780 rev/s, raised the dropout rate of the 5 V 100 % run-up that followed
two to four times and slowed it 2 to 3 %, where 7.4 V 60 % and 5 V 100 %
left it unchanged, so `cycles` refuses a rung over that line.

`ref` is three cycles of 30 % on 5.0 V, run at the start and the end of a
sitting, so a drift of the coast's drag or the brush drop through the
sitting shows against the same rung.

`low` is the slow rungs. The duty that starts a motor from rest is set by
its breakaway, and a turning motor holds a lower duty than that, so each
cycle kicks the rotor for 300 ms at a duty that turns it near 150 rev/s and
then drops to the rung's duty, chained with no torque-off between, holds
it for two seconds and coasts. The last 200 ms of the hold are the rung,
whether the rotor kept turning or stopped, which the ripple tach tells.

`swapped` runs the 40 and 60 % cycles on 7.4 V with the leads as wired,
then again with the motor's two leads swapped at the board's connector. A
split between the directions that follows the rotor is the motor's, and
one that follows the board's direction is the board's. The leads go back
at the end.

`spinning` keeps the meter in series and records `osc-lab burst` captures
of the free rotor already turning at the meter run's three points, the
shunt and both taps through whole PWM periods, in slow and in fast decay.
Slow decay shows the current's shape inside the window the crest sample
reads. Fast decay returns the current through the shunt for the whole
period, so the board's own period mean sits beside the meter's. Each
capture spins 1.5 s at its duty first, and the groups rest twenty seconds
apart.
"""
import csv
import sys
import time

from oscnb import b41t, bench, boards, capture, motors, osc

LEVELS = [3.7, 5.0, 7.4]
DUTIES = [15, 20, 30, 40, 60]
REPEATS = 3
SPIN_MS = 1500
COAST_MS = 800
REST_S = 20
# (rail volts, duty %): 30 % on 5.0 V and 20 % on 7.4 V turn near 150 rev/s, 40 % on 7.4 V is the zero question's spin
METER = [(5.0, 30), (7.4, 20), (7.4, 40)]
METER_SPIN_MS = 3000
METER_COAST_MS = 300
HIGH = [(5.0, 60), (5.0, 80), (5.0, 100)]
HIGH7 = [(7.4, 50), (7.4, 60), (7.4, 67)]
HIGH7_SPIN_MS = 1000
REF = [(5.0, 30)]
LOW = [(5.0, 12), (5.0, 10), (5.0, 8), (5.0, 6), (7.4, 8), (7.4, 6), (7.4, 4)]
KICK = {5.0: 30, 7.4: 20}    # the kick's duty by rail, about 150 rev/s on mg90-b
KICK_MS = 300
LOW_HOLD_MS = 2000
LOW_COAST_MS = 600
SWAPPED = [(7.4, 40), (7.4, 60)]
SPINNING = METER
BURST_SPIN_MS = 1500
BURST_REST_MS = 1500         # the rotor coasts to a stop between captures


def cycles(out, plan, spin_ms, coast_ms, folder="cycles", kick=None):
    """Run `plan`, (rail volts, duty %) pairs, REPEATS cycles each, into
    `out/<folder>/K` with one row per cycle in `out/<folder>.csv`. With
    `kick`, duty by rail, each spin opens with KICK_MS at that duty and
    drops to the rung's duty chained, no torque-off between."""
    with open(out / f"{folder}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["k", "volts_set", "dps_read", "vsys_read", "duty_pct", "repeat", "t_start", "t_end", "folder", "kick_pct"])
        k, level = 0, None
        for v, d in plan:
            if v * max(d, kick[v] if kick else 0) / 100 > motor.rated_v:
                raise SystemExit(f"{v} V at {d} % is {v * d / 100:.2f} V effective, over the motor's rated {motor.rated_v} V")
            if v != level:
                dps_read, vsys = bench.set_vsys(v)
                print(f"VSYS {v} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
                level = v
            for r in range(REPEATS):
                sub = f"{folder}/{k}"
                t0 = time.time()
                drive = [f"{kick[v]}@{KICK_MS}", f"then:{d}@{spin_ms}"] if kick else [f"{d}@{spin_ms}"]
                osc.sweep(out / sub, drive + [f"coast:{coast_ms}"], baseline_ms=500, rest_ms=REST_S * 1000, dirs="both")
                w.writerow([k, v, f"{dps_read:.3f}", f"{vsys:.3f}", d, r, f"{t0:.3f}", f"{time.time():.3f}", sub, kick[v] if kick else 0])
                f.flush()
                k += 1
                capture.hold(REST_S, f"cycle {k - 1} done ({v} V, {d} %), the motor resting")


mode = next((a for a in sys.argv[1:] if a in ("meter", "high", "high7", "swapped", "spinning", "ref", "low")), None)
nometer = "nometer" in sys.argv[1:]
n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.selected()
board = boards.BOARDS[boards.DEFAULT]
out = capture.dataset(f"{motor.key}__{board.key}", "bemf-coast", ["", *n], name=(mode or "capture") + ("-nometer" if nometer else ""))
print("motor", motor.key, "board", board.key)
if mode in ("meter", "high", "high7"):
    plan = {"meter": METER, "high": HIGH, "high7": HIGH7}[mode]
    spin_ms = HIGH7_SPIN_MS if mode == "high7" else METER_SPIN_MS
    if nometer:
        capture.prompt("Bar OFF, rotor free, no meter. Motor leads on the board's connector as usually wired.")
        bench.power_up(plan[0][0] + bench.DIODE_V)
        cycles(out, plan, spin_ms, METER_COAST_MS)
    else:
        capture.prompt("Bar OFF, rotor free. DMM on DC mA held on the 220.00 mA range with RANGE, red probe in the mA jack, "
                       "in series with one motor lead: MOT_A -> meter -> white lead -> motor -> black lead -> MOT_B. Bluetooth on.")
        bench.power_up(plan[0][0] + bench.DIODE_V)
        with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="DC A"):
            cycles(out, plan, spin_ms, METER_COAST_MS)
elif mode == "ref":
    capture.prompt("Motor leads on the board's motor connector, the bar OFF the pinion, the rotor free.")
    bench.power_up(REF[0][0] + bench.DIODE_V)
    cycles(out, REF, SPIN_MS, COAST_MS)
elif mode == "low":
    capture.prompt("Motor leads on the board's motor connector, the bar OFF the pinion, the rotor free.")
    bench.power_up(LOW[0][0] + bench.DIODE_V)
    cycles(out, LOW, LOW_HOLD_MS, LOW_COAST_MS, kick=KICK)
elif mode == "spinning":
    capture.prompt("Bar OFF, rotor free. DMM on DC mA held on the 220.00 mA range with RANGE, red probe in the mA jack, "
                   "in series with one motor lead: MOT_A -> meter -> white lead -> motor -> black lead -> MOT_B. Bluetooth on.")
    bench.power_up(SPINNING[0][0] + bench.DIODE_V)
    marks = capture.Marks(out)
    with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="DC A"):
        level = None
        for v, d in SPINNING:
            if v != level:
                dps_read, vsys = bench.set_vsys(v)
                print(f"VSYS {v} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
                level = v
            for decay in ("slow", "fast"):
                group = f"{v:.1f}v-{d}-{decay}"
                marks.mark(group)
                osc.bursts(out / group, [d], repeats=REPEATS, chans="diff", dirs="both", rest_ms=BURST_REST_MS,
                           decay=decay, spin_ms=BURST_SPIN_MS)
                capture.hold(REST_S, f"{group} done, the motor resting")
    marks.close()
elif mode == "swapped":
    capture.prompt("Bar OFF, rotor free, no meter. Motor leads on the board's connector as usually wired.")
    bench.power_up(SWAPPED[0][0] + bench.DIODE_V)
    cycles(out, SWAPPED, SPIN_MS, COAST_MS, folder="wired")
    bench.power_down()
    capture.prompt("Swap the motor's two leads at the board's connector.")
    bench.power_up(SWAPPED[0][0] + bench.DIODE_V)
    cycles(out, SWAPPED, SPIN_MS, COAST_MS, folder="swapped")
    bench.power_down()
    capture.prompt("Swap the motor's leads back as usually wired.")
else:
    capture.prompt("Motor leads on the board's motor connector, the bar OFF the pinion, the rotor free.")
    bench.power_up(LEVELS[0] + bench.DIODE_V)
    cycles(out, [(v, d) for v in LEVELS for d in DUTIES], SPIN_MS, COAST_MS)
bench.power_down()
print("wrote", out)
