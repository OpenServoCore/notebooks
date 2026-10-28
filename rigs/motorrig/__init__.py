"""The bare-motor rig for the stage 2 experiments: a base plate with a boss
that seats the motor can and twelve pillars on a bolt circle, a slit wrench
bar that clamps the rotor by its pinion and is pinned to two pillars at one
of the twelve angles, and a fit coupon that reads the printer's bore against
the pinion in hand.

Nothing in the rig pushes or pulls along the shaft. The motor sits on its
can's rear face, with reliefs under the rear boss and the terminal tabs, so a
load on the motor goes into the can. The leads are soldered to the tabs
before the motor goes in; they drop through the tab slots into a channel in
the base's underside and leave at the edge below hole 0, so the base sits
flat on the table with the leads under it and nothing pulls on the tabs.
The bar is one piece with a slit along its length through the bore, from
well past the hub to short of the pin holes, so the two ends that carry the
pins stay solid and the two halves around the bore can close. Two M2 screws
cross the slit through the hub, one each side of the bore, each into a nut
dropped into a slot from the hub's top, so no printed thread carries the
tension. The bore slips over the pinion and the screws close the halves on
it; the halves flex in the arm, where each is a beam some 4 mm square and
30 mm long, and a tenth of a millimeter of closure takes a few tens of
newtons, well inside an M2 screw. A screw only squeezes what sits between
its head and its nut, so the gap it closes has to contain the pinion's axis,
and that puts the screws across the bar, not along the pinion. Backing the
screws off lets the bar's own spring open the slit, and the bar lifts
straight off; nothing levers the pinion. Between angles the bar turns with
the rotor and the clamp stays tight; only the two pins move. The torque a
locked rotor puts on the bar is a few millinewton meters, a few hundredths of
a newton at the pins, so the bar's flex under it is thousandths of a degree
and the angle at a hole is set by the pin clearance alone. The pinion stays
on the shaft for every experiment; it only comes off when the motor is torn
down.

Frame: X east, Y north, Z up, the shaft on Z. Hole 0 of the bolt circle is on
+X and the holes count counterclockwise seen from above, the sign the
notebooks use for angle. The base prints on its underside with the boss and
the pillars standing, the bar on its underside with the hub standing and the
nut slots opening upward, the coupon on its top face, so every bore is vertical
and no part needs support.

Motor dimensions default to the SG90-class flat can and pinion measured in
the firmware repo's mechanical/sg90/measurements.py; the motor at hand is
measured with calipers and the numbers go in as Spec fields or command line
overrides before the print.

    uv run --group cad python -m rigs.motorrig [--pinion-d 2.3 ...]    writes rigs/motorrig/build/
"""
