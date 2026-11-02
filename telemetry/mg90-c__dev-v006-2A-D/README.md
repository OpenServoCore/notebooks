# mg90-c on dev-v006-2A-D: the captures and what each one represents

The second motor of the MG90 clone, bare in the motor rig on board D, the
experiment image, the DPS as the supply. Every dataset below ran under the
scripts in `stages/2-motor/*/capture.py` with `OSC_MOTOR=mg90-c`, the bar
OFF unless said, no meter unless said. The motor's rated voltage is 5 V and,
from `high7-nometer-1` on, no free spin exceeds it as rail times duty; that
one dataset is the exception and the evidence for the rule.

The motor changed state during the first sitting, so the datasets fall into
three states:

- **A, as delivered.** Fresh out of the servo, never run. The +1 direction
  was still seating in during the first reference rung.
- **B, after the 7.4 V spins.** `high7-nometer-1` ran the motor free at
  7.4 V 80 and 100 %, 625 to 780 rev/s, about 18 s in all, above its rated
  voltage. The 5 V 100 % run-up that followed read two to four times the
  dropouts and ran 2 to 3 % slower at less current. Below about 320 rev/s
  nothing changed; from about 450 rev/s up the dropout rate and the speed
  did. Ten minutes of slow running did not undo it.
- **D, after the threshold spins.** The sacrificial end of the record: 7.4 V
  at 70 % for 3 s, then at 75 % for 3 s, each bracketed by 5 V 100 % run-ups.
  The 70 % spins alone, 5.2 V effective, raised the heavy direction's run-up
  dropout rate by half and took 5 mA off its crest current, so the change
  starts between 67 % (4.96 V effective, harmless through high7-1) and 70 %,
  at the rated voltage within a few percent.
- **C, after half a day at rest.** The heavy direction's dropout rate fell
  to under its state A value and the direction asymmetry reversed, while the
  speed stayed 4 to 5 % under state A at a higher current, so a mechanical
  part remained after a contact part relaxed.

## Sitting 1, in capture order

| Dataset | Run | State | Conditions | What it represents |
|---|---|---|---|---|
| bemf-coast/ref-1 | 5 V 30 %, 3 cycles | A, seating in | first spins of the motor's life | the opening reference; +1 drifted 141 -> 131 rev/s over the three cycles |
| bemf-coast/capture-1 | the ladder, 3.7 / 5.0 / 7.4 V at 15 to 60 %, 3 repeats | A | ascending rails | the motor as delivered, every rung |
| speed-torque/capture-1 | 5 V 100 % run-ups, 3 cycles | A | right after the ladder | the before point for the 7.4 V question |
| bemf-coast/high7-nometer-1 | 7.4 V 60 / 80 / 100 %, 1 s spins, 3 cycles each, no meter | A -> B | the only dataset over the rated voltage | the dropout rate and the dropout-row zero at 7.4 V, and the run that changed the brushes: the 60 % rung still reads as state A, the 80 % rung's rate climbs cycle by cycle |
| speed-torque/capture-2 | 5 V 100 % run-ups | B | two minutes after the 7.4 V spins | the after point: dropouts 627 / 213 -> 1060 / 827 per s by direction, speed -2 to -3 % |
| bemf-coast/low-1 | kick 300 ms then hold 2 s, 5 V 12 / 10 / 8 / 6 %, 7.4 V 8 / 6 / 4 %, coast 600 ms | B | kick 30 % at 5 V, 20 % at 7.4 V | the slow rungs, 2 to 36 rev/s, under drive from above the sustaining floor |
| speed-torque/capture-3 | 5 V 100 % run-ups | B | after the low run's ten minutes of gentle running | whether B relaxes in minutes: it did not (1053 / 627 per s) |
| bemf-coast/capture-2 | the ladder again | B | two minutes after capture-3; the 3.7 V level ran first and carried extra drag that faded by the 5 V level | the motor after the change, rung for rung against capture-1 |
| bemf-coast/ref-2 | 5 V 30 %, 3 cycles | B | the close of sitting 1 | 144 / 144 rev/s, 73 to 76 mA, both directions alike |

A swap run was started after capture-3 and stopped before its lead swap; its
partial folder was removed and it is not part of the record.

## Sitting 2, after about half a day at rest

| Dataset | Run | State | Conditions | What it represents |
|---|---|---|---|---|
| speed-torque/capture-4 | 5 V 100 % run-ups | C | the first drive after the rest, motor cold | the overnight read: 473 / 533 per s, 520 / 520 rev/s, 107 to 109 mA |
| bemf-coast/ref-3 | 5 V 30 %, 3 cycles | C | minutes after capture-4 | the opening reference of sitting 2: 140 / 140 rev/s, 78 mA, 3 % under ref-2 |
| speed-torque/capture-5 | 5 V 100 % run-ups | C, warm | minutes after ref-3 | capture-4 repeated: the heavy direction's rate climbed back from 440 to 500 toward 740 to 820 per s within minutes, so the overnight drop was mostly a cold first run |
| bemf-coast/swapped-1 | 7.4 V 40 and 60 %, 3 cycles each, leads as wired then swapped at the board's connector | C, warm | right after capture-5 | whether the direction split follows the rotor or the board |
| rest-resistance/capture-1 | 100 % laps of 300 ms into the locked rotor, 3.3 / 3.7 / 4.2 V, both directions, twelve holes | C | bar on the pinion, hole 0 through 11, 20 s rest between holes, board NTC 21.6 to 22.9 C; driven by hand at the prompts | the terminal resistance over the rotor's angle: 4.40 ohm median in direction +1 (4.28 to 4.44 over the holes), direction -1 1.3 % over it, against 4.46 ohm on mg90-b at a board 3 C warmer |
| inductance/capture-1 | duty-step bursts into the locked rotor at 5 V, 25 and 40 %, five groups a hole, twelve holes | C | bar on the pinion, the clamp never loosened since the stall pass, so hole k is the stall pass's hole k; board at room | the winding's rise against the same twelve seats as the resistance map: time constant 197 us against 193 on mg90-b by the same first read |
| inductance/fast-1 | the four main groups in fast decay then two slow ones, hole 11 | C | the bar left where the main run ended, no prompt between | the winding's L on its own, the cross-check of the main run |
| bemf-coast/meter-1 | 5 V 30 %, 7.4 V 20 and 40 %, 3 s spins, 3 cycles each, DMM in series on the 220 mA range | C, warm | bar off; meter on the MOT_A side, its probes reversed against the earlier captures by a wrong instruction, so the log's `value` column was negated after the run to the convention that direction +1 reads positive; the `raw` packet bytes are untouched | the true mean current at the reading term's anchor points: 52 mA at 5 V 30 % and at 7.4 V 20 %, 70 to 72 mA at 7.4 V 40 % |
| bemf-coast/high-1 | 5 V 60, 80, 100 %, 1.5 s spins, 3 cycles each, DMM in series | C, warm | right after meter-1, the same wiring, the same sign fix applied to `value`; the meter chirped over-range at each spin's start, no over-range packet reached the log | the top of the speed table with the meter: 76, 81 to 86, 82 to 90 mA |
| bemf-coast/high7-1 | 7.4 V 50, 60, 67 %, 1 s spins, 3 cycles each, DMM in series, probes the right way round | C, warm | right after high-1; every rung under the rated voltage as rail times duty (67 % is 4.96 V effective); the 1 s spins leave the meter about one settled packet per spin, so its means are thin and two of them straddle a spin's edge | the 7.4 V zero confirmation from the meter beside the dropout rows, and the dropout rate below the damaging rungs: +1 / -1 about 55 / 6 per s at 50 %, 345 / 65 at 60 %, 447 / 133 at 67 % |
| bemf-coast/ref-4 | 5 V 30 %, 3 cycles | C, warm | meter out, leads straight on the connector; the close of the required sitting | the closing reference: 85 to 86 mA crest against ref-3's 87, dropouts a few per second |
| speed-torque/capture-6 | 5 V 100 % run-ups | C, warm | right after ref-4; the before point of the threshold runs | 653 / 350 dropouts per s by direction, 112 mA crest both ways |
| bemf-coast/threshold-70-1 | 7.4 V 70 %, 1 s spins, 3 cycles | C -> D | the first deliberate run over the rated voltage since high7-nometer-1, 5.18 V effective, 3 s in all | 585 to 732 / 321 to 434 dropouts per s during the spins, climbing cycle by cycle in the heavy direction |
| speed-torque/capture-7 | 5 V 100 % run-ups | D | two minutes after the 70 % spins | 970 / 560 dropouts per s, crest 107 / 111 mA: the heavy direction's rate up half over capture-6 and its current down 5 mA, so 70 % already changed the brushes |
| bemf-coast/threshold-75-1 | 7.4 V 75 %, 1 s spins, 3 cycles | D | two minutes after capture-7, 5.55 V effective | 721 to 916 / 521 to 573 dropouts per s, climbing again |
| speed-torque/capture-8 | 5 V 100 % run-ups | D | two minutes after the 75 % spins; the last drive of the motor's record | 1080 / 637 dropouts per s, crest 105 / 109 mA |

The stall pass and the two inductance runs share one bar seat: the clamp went on at hole 0 before the stall pass and the pins alone moved between holes, so a hole number is the same rotor angle in all three datasets, which the two 12-hole runs of mg90-b, a day apart on a new seat, could not claim.

Lead orientation at the board's connector, as wired for every dataset but the swapped block of `swapped-1`: the black lead on MOT_A, the white lead on MOT_B. The swapped block has them reversed.
