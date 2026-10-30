"""Stage 1, bridge-states: what the shunt and the terminal taps read in each
bridge state when the load carries a real inductive current. The load is the
RL dummy, the bare 5 ohm resistor in series with one or both of the two 1 mH
power inductors, with the resistor alone as the control on the same rail.
Needs the board on the DPS, the bare load, the two inductors and the `osc`
and `osc-lab` tools. No meter.

  uv run python stages/1-board/bridge-states/capture.py [N]    writes telemetry/<board>/bridge-states/capture-N

Four wirings, MOT_A to the resistor to the inductors to MOT_B, in an order
where each change adds or removes one part: `r` the resistor alone, `ra`
with inductor A, `rab` with A and B in series, `rb` with B. Each wiring has
its own folder with the same parts in it, and `marks.csv` holds the host
time each wiring went on and each part began.

- `laps/`, `levels.csv`: DC laps at 100 % on a 3.7 and a 4.2 V rail, both
  directions. At 100 % the bridge stops switching and the inductance drops
  out, so the laps give the dummy's DC resistance.
- `bursts-slow/`, `bursts-fast/`: `osc-lab burst` at 25 and 40 % on a 5 V
  rail, both signs, five repeats each, the shunt and both taps interleaved,
  one set per decay mode.
- `sweep-slow/`, `sweep-fast/`: 25 and 40 % with the 100 % rung as the
  control, both directions, 150 ms a rung on a 5 V rail. The row carries the
  trough shunt beside the crest samples, the steady state the bursts end
  before.

One inductor with the resistor is about 1 mH over 5.2 ohm, a time constant
of 0.19 ms or four PWM periods of 50 us. Both are about 2 mH over 5.4 ohm,
0.37 ms or seven periods. A slow-decay burst runs about 9 periods past the
step, so it ends near 90 % of the final current on one inductor and near
70 % on both. Both curves bend inside the capture, so the rise across
periods fits L/R, and the ramp inside each drive window fits L from its
slope, about 5 mA per us at 5 V on 1 mH. The bare resistor's own inductance
did not show in sample-timing's bursts, where the shunt rose with the tap,
so on `r` the bursts read the amplifier's step and the method should find
next to none.

Under slow decay the off phase shorts the load through the two low-side
switches. On the RL loads the current keeps circulating there, out of the
shunt's path, so the trough shunt reads zero while the load carries about
the crest's current, and the crest sample at the middle of the drive reads
the mean of a ripple of 50 to 70 counts on 1 mH. On `r` nothing flows in the
off phase. The bursts see the whole period, with the taps in the off phase
carrying the circulating current's drop across the low-side switches, and
the sweeps see the crest and the trough in the settled state.

Under fast decay the off phase opens all four switches and the current
returns to the rail through the body diodes, against the rail plus two diode
drops, more than the drive puts across the load. Below 50 % it reaches zero
before the next drive, so every period starts from zero on every load
whatever the inductance, and a fast-decay burst is periodic from its first
period. Its fold carries the ramp up in the drive window, about 5 mA per us
on 1 mH, the bridge's turn-off, the return through the diodes on the taps
and where it ends, the amplifier's tail, and the coast after it. The drive
window sits at the trough, so the sweep's trough shunt reads the ramp at
half the window, about 30 mA at 25 % and 50 mA at 40 % on 1 mH, and its
crest sample reads the off phase. At 40 % the return ends close to the
crest. On `r` nothing returns, so its fast set is the turn-off and the tail
with no inductive current. The returned charge, about 0.8 uC a period at
40 % on one inductor, goes into the board's capacitance behind the feed
diode and the next drive draws it back. The average draw stays positive,
about a fifth of what the drive window takes, and the feed diode keeps the
return off the DPS, which sinks no current.

The resistor takes 3.5 W on a 4.2 V lap and 4.9 W on the 100 % rung at 5 V,
under its 12 W. The inductors carry at most about 1 A, against the 3.5 A
where their inductance has fallen to 380 uH.
"""
from oscnb import bench, boards, capture, laps, osc, results

LAP_LEVELS = [3.7, 4.2]
LEVEL = 5.0
DUTIES = [25, 40]
REPEATS = 5
LADDER = DUTIES + [100]
# vbus_raw, then current_raw, vmotor_a, vmotor_b, duty_ticks and current_trough,
# 12 B. The trough shunt is the off phase under slow decay and the drive under
# fast. The rail tap takes ntc_raw's place, and osc-lab writes no trough taps.
ROW = "0x23c+2,0x240+10"
# (folder, load between MOT_A and MOT_B), each change adding or removing one part
WIRINGS = [
    ("r", "the bare resistor alone"),
    ("ra", "the resistor and inductor A"),
    ("rab", "the resistor, inductor A and inductor B"),
    ("rb", "the resistor and inductor B"),
]

out = capture.dataset(boards.DEFAULT, "bridge-states")
capture.prompt(f"Mark one inductor A and the other B. MOT_A -> {WIRINGS[0][1]} -> MOT_B.")
bench.power_up(LAP_LEVELS[0] + bench.DIODE_V)
marks = capture.Marks(out)
for name, load in WIRINGS:
    if name != WIRINGS[0][0]:
        # a DPS step down resets the board, so it steps before the hands go on the terminals
        bench.set_vsys(LAP_LEVELS[0])
        capture.prompt(f"Rewire to MOT_A -> {load} -> MOT_B, the bridge off and the rail at {LAP_LEVELS[0]} V.")
    marks.mark(f"{name}/wired")
    with osc.Limit(laps.LIMIT_COUNTS):
        marks.mark(f"{name}/laps")
        laps.run(out / name, LAP_LEVELS, name)
        dps_read, vsys = bench.set_vsys(LEVEL)
        print(f"{name}: VSYS {LEVEL} V, rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
        for decay in ("slow", "fast"):
            marks.mark(f"{name}/bursts-{decay}")
            osc.bursts(out / name / f"bursts-{decay}", DUTIES, repeats=REPEATS, chans="diff", decay=decay)
        for decay in ("slow", "fast"):
            marks.mark(f"{name}/sweep-{decay}")
            osc.sweep(out / name / f"sweep-{decay}", LADDER, window_ms=150, baseline_ms=1000, rest_ms=500,
                      dirs="both", decay=decay, tel_chunks=ROW)
    marks.mark(f"{name}/done")
marks.close()
bench.power_down()
print("wrote", results.rel(out))
