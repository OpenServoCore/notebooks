"""Stage 2, teardown: what the motor is made of, read with the rotor out of
its can, and the bare copper heated by its own current and read while it
cools, the copper's resistance its own thermometer once no brush is in the
path.

  uv run python stages/2-motor/teardown/capture.py terminals [N]       writes telemetry/<motor>/teardown/terminals-N
  uv run python stages/2-motor/teardown/capture.py cool winding [N]    writes telemetry/<motor>__<board>/teardown/cool-winding-N
  uv run python stages/2-motor/teardown/capture.py cool core [N]       writes telemetry/<motor>__<board>/teardown/cool-core-N

`terminals` looks for a path across the motor's terminals that is not the
winding, on the end cap with its brush card seated back on the empty can
once the rotor is out, so the brushes touch nothing and a part from a
terminal to the case stays in place. The meter reads ohms and capacitance across the two
terminal tabs and ohms from each tab to the can. A part that conducts only
above a volt or two reads open on the meter's ohms, so the DPS then puts 3.7,
5.0 and 7.4 V across the tabs, the levels the bridge drives, with the meter
on DC mA in series. The DPS output comes up at its last setting for a moment
when enabled, so the leads go on after it is on. Last, after a lead zero,
the motor's two leads in series, unsoldered from the tabs and clipped
together at their tab ends, which every reading of the assembled motor
carried, then the rotor's three tab pairs on ohms, one coil against the other two in each
reading, for a coil that differs from the others, and a look at the back of
the commutator for a ring varistor, which sits across the segments where
the end cap's readings never reach it.

`cool` needs the rotor with two leads soldered to commutator tabs 1 and 2,
held by its shaft in still air, and the thermocouple tip taped on the
winding (`winding`) or on the iron of a pole face (`core`), one place a run
since the tip stays put through one. First the meter reads the copper at
room: the thermocouple, a lead zero, the tabs, the leads' free ends, the
thermocouple again. Then the leads go to the board's motor connector and the
meter stays on the thermocouple for the rest of the run. The board probes
the copper at room with 5 ms windows at 100 %, then heats it at 100 % on
the 3.3 V rail, each second a 100 ms window and then a held drive. A first
heat of FIRST_S reads how fast the copper warms, and a second heat, sized
from that rate, takes it to about RISE_C over room, unless the duty folded
back in the first. Each heat runs on in the same chain into 5 ms probes
0.2 s apart for six seconds, with no pause after the drive, since the
copper's fast exchange with the iron is over in a few seconds. Then the
probes go on once a second to a minute and once every five seconds to ten
minutes. The resistance over the textbook copper coefficient is the
copper's temperature, so `winding` checks that route against the tip on
the winding, and `core` reads the iron beside it. The run checks the
method, and the shape of the heating, whether the copper runs seconds ahead
of the iron, is the part a brushed motor of this class shares. The numbers
themselves belong to this rotor in open air and feed no calibration.

The masses, the photos and the counts of the teardown go into the notebook
by hand.
"""
import csv
import json
import statistics
import sys
import time

from oscnb import b41t, bench, boards, capture, dps150, motors, osc

IV_LEVELS = [3.7, 5.0, 7.4]
IV_AMPS = 0.1                # the DPS limit: a part across the tabs passes tens of mA, two touching brushes no more than this
IV_WINDOW_S = 20

ZERO_S = 20
OHMS_S = 40
TEMP_S = 60
RAIL_V = 3.3                 # the stall pass's lowest level: about 0.85 A through the copper, the current it held at 100 %
ALPHA = 0.00393              # copper's resistance coefficient per C at 20 C, the textbook value
RISE_C = 40                  # the most the copper may rise over room
FIRST_S = 5                  # the first heat, which reads the warming rate
HEAT_MAX_S = 60
ROOM_PROBES = 10
PROBE_MS = 5                 # at 100 %, 25 times the winding's L/R, about 15 mJ into the copper
TAIL = 30                    # probes 0.2 s apart closing each heat, the copper's first six seconds of cooling
COOL = [(60, 1000), (108, 5000)]   # probes and their spacing in ms: once a second to a minute, then every 5 s to 10 min

args = sys.argv[1:]
n = [a for a in args if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
OHMS = "DMM on ohms, manual 220 ohm range."
TEMP = "DMM on temperature (C)."


def window(meter, marks, out, name, expect, seconds):
    with meter.log(out / f"{name.replace(' ', '-')}.csv", expect=expect):
        marks.mark(name)
        capture.hold(seconds, name)


def resistances(folder):
    """(host time, R in raw counts, duty full, rows) for each streamed drive window
    of an osc-lab sweep in `folder`, over the window's second half so the
    onset is out of it. R is the terminal taps' difference over the shunt
    above its torque-off zero, both in raw counts, so a ratio of two of them
    is a ratio of resistances with every scale cancelled."""
    meta = json.loads((folder / "meta.json").read_text())
    times = {r["seg"]: r for r in csv.DictReader(open(folder / "segments.csv"))}
    rows = {}
    for r in csv.DictReader(open(folder / "sweep.csv")):
        rows.setdefault(r["seg"], []).append(r)
    zero = statistics.median(int(r["current_raw"]) for r in rows["0"])
    out = []
    for seg, rs in rows.items():
        if int(times[seg]["cmd_duty_q15"]) <= 0:
            continue
        tail = rs[len(rs) // 2:]
        v = statistics.median(int(r["vmotor_a"]) - int(r["vmotor_b"]) for r in tail)
        i = statistics.median(int(r["current_raw"]) for r in tail) - zero
        full = all(int(r["duty_counts"]) >= meta["pwm_period_ticks"] for r in tail)
        out.append((float(times[seg]["t_start"]), v / i, full, len(rs)))
    return out


def heat(folder, seconds):
    """`seconds` of 100 % in one-second chains, the last chain running on
    into the TAIL probes with no pause. Returns the copper's rise over room,
    in C by its resistance, at the last heating window, and whether the duty
    held 100 % through every heating window."""
    steps = ["100@100", "hold:100@900"] * seconds + [f"then:100@{PROBE_MS}", f"hold:0@{200 - PROBE_MS}"] * TAIL
    osc.sweep(folder, steps, baseline_ms=500, rest_ms=0, dirs="fwd")
    w = [x for x in resistances(folder) if x[3] > 1000]     # the 100 ms windows, not the probes
    return (w[-1][1] / r_room - 1) / ALPHA, all(x[2] for x in w)


if "terminals" in args:
    out = capture.dataset(motor.key, "teardown", ["", *n], name="terminals")
    print("motor", motor.key)
    capture.prompt("The rotor out, the end cap with its brush card seated back on the empty can, so a part from a terminal "
                   "to the case is in place. If the brushes spring together, a slip of kapton between them. "
                   "One terminal tab marked with a pen as tab A, the other is tab B. "
                   "DMM on ohms, auto range, probes on the two terminal tabs. Bluetooth on.")
    marks = capture.Marks(out)
    with b41t.Meter() as meter:
        window(meter, marks, out, "tabs ohm", "ohm", IV_WINDOW_S)
        capture.prompt("DMM on capacitance, probes on the two terminal tabs.")
        window(meter, marks, out, "tabs cap", "F", IV_WINDOW_S)
        capture.prompt("DMM on ohms, one probe on tab A, the other on the can.")
        window(meter, marks, out, "tab a can ohm", "ohm", IV_WINDOW_S)
        capture.prompt("DMM on ohms, one probe on tab B, the other on the can.")
        window(meter, marks, out, "tab b can ohm", "ohm", IV_WINDOW_S)
        capture.prompt("DMM on DC mA, 220.00 mA range held with RANGE, red probe in the mA jack. "
                       "DPS leads OFF the tabs for now: the DPS turns on next.")
        p = dps150.open_supply()
        try:
            dps150.on(p, IV_LEVELS[0], IV_AMPS)
            capture.prompt("DPS + -> meter red, meter COM -> tab A, tab B -> DPS -.")
            for v in IV_LEVELS:
                p.set_voltage(v)
                time.sleep(2)
                print(f"DPS {dps150.read(p).output_voltage:.2f} V")
                window(meter, marks, out, f"iv {v:.1f}", "DC A", IV_WINDOW_S)
        finally:
            dps150.off(p)
            p.close()
        print("DPS off")
        capture.prompt(f"Probes shorted together. {OHMS}")
        window(meter, marks, out, "pair zero", "ohm", IV_WINDOW_S)
        capture.prompt("The motor's two loose leads clipped together at the ends that were soldered to the tabs, "
                       "probes on their two free ends.")
        window(meter, marks, out, "leads ohm", "ohm", IV_WINDOW_S)
        for pair in ("1-2", "2-3", "3-1"):
            capture.prompt(f"Probes on the rotor's commutator tabs {pair}, the tabs numbered round from one marked with a pen.")
            window(meter, marks, out, f"tabs {pair} ohm", "ohm", IV_WINDOW_S)
    capture.prompt("Photograph the back face of the commutator. A ring varistor is a thin disc or ring, "
                   "often orange, gray or black, pressed on behind the segments. Note it in the notebook either way.")
    marks.mark("end")
    marks.close()
    print("wrote", out)
    sys.exit()

if "cool" not in args or not ({"winding", "core"} & set(args)):
    sys.exit("usage: capture.py terminals [N] | cool winding [N] | cool core [N]")
place = "winding" if "winding" in args else "core"
board = boards.BOARDS[boards.DEFAULT]
out = capture.dataset(f"{motor.key}__{board.key}", "teardown", ["", *n], name=f"cool-{place}")
print("motor", motor.key, "board", board.key, "tip on the", place)
tip = {"winding": "taped on the enamel of a coil, away from the commutator tabs",
       "core": "taped on the iron of a pole face, clear of the copper"}[place]
capture.prompt("The rotor held by its shaft in still air, two leads soldered to commutator tabs 1 and 2, "
               f"their other ends off the board. The thermocouple tip {tip}. Nothing has heated the rotor for "
               f"at least twenty minutes. {TEMP} Bluetooth on.")
marks = capture.Marks(out)
with b41t.Meter() as meter:
    window(meter, marks, out, "room c1", "C", TEMP_S)
    capture.prompt(f"Probes shorted together. {OHMS}")
    window(meter, marks, out, "zero", "ohm", ZERO_S)
    capture.prompt("Probes on the solder of tabs 1 and 2.")
    window(meter, marks, out, "tabs ohm", "ohm", OHMS_S)
    capture.prompt("Probes on the two leads' free ends, where they plug into the board.")
    window(meter, marks, out, "leads ohm", "ohm", OHMS_S)
    capture.prompt(TEMP)
    window(meter, marks, out, "room c2", "C", TEMP_S)
    capture.prompt("Leads on the board's motor connector. Hands off the rotor from here to the end.")
    bench.power_up(RAIL_V + bench.DIODE_V)
    try:
        dps_read, vsys = bench.set_vsys(RAIL_V)
        print(f"DPS {dps_read:.3f} V, rail {vsys:.3f} V")
        board_out = out / "board"
        with meter.log(out / "temp.csv", expect="C"):
            marks.mark("room probes")
            osc.sweep(board_out / "room", [f"100@{PROBE_MS}", f"hold:0@{1000 - PROBE_MS}"] * ROOM_PROBES,
                      baseline_ms=500, rest_ms=0, dirs="fwd")
            r_room = statistics.median(x[1] for x in resistances(board_out / "room"))
            marks.mark("heat 0")
            rise, full = heat(board_out / "heat-0", FIRST_S)
            print(f"heated {FIRST_S} s, the copper {rise:+.1f} C over room by its resistance")
            # A straight line from the first heat overstates the second, since the copper
            # has cooled through the tail and loses more as it warms, so it errs short.
            more = min(HEAT_MAX_S - FIRST_S, int((RISE_C - rise) / (rise / FIRST_S))) if rise > 0 else 0
            if not full:
                print("the duty folded back under 100 %, no second heat")
            elif more >= 1:
                marks.mark("heat 1")
                rise, _ = heat(board_out / "heat-1", more)
                print(f"heated {more} s more, the copper {rise:+.1f} C over room by its resistance")
            marks.mark("cool")
            cool = []
            for count, gap in COOL:
                cool += [f"100@{PROBE_MS}", f"hold:0@{gap - PROBE_MS}"] * count
            osc.sweep(board_out / "cool", cool, baseline_ms=500, rest_ms=0, dirs="fwd")
            marks.mark("cool end")
    finally:
        bench.power_down()
marks.mark("end")
marks.close()
print("wrote", out)
