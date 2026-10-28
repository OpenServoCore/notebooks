"""The bare-motor rig for the stage 2 experiments: a base plate with a boss
that seats the motor can and twelve pillars on a bolt circle, a wrench bar
with a cap that clamps the rotor by its pinion and is pinned to two pillars
at one of the twelve angles, and a fit coupon that reads the printer's bore
against the pinion in hand.

Nothing in the rig pushes or pulls along the shaft. The motor sits on its
can's rear face, with reliefs under the rear boss and the terminal tabs, so a
load on the motor goes into the can. The leads are soldered to the tabs
before the motor goes in; they drop through the tab slots into a channel in
the base's underside and leave at the edge below hole 0, so the base sits
flat on the table with the leads under it and nothing pulls on the tabs.
The bar's socket is a two-piece pinch clamp: the hub on the bar carries one
half of the bore, a separate cap the other half, and two M2 screws along the
split pull the cap onto the pinion, so the grip is screw tension through two
rigid halves and nothing has to flex. The bore is cut a tenth under the
pinion so the halves grip before their faces meet. The screws sit high in
the hub, so a driver runs across the bar well above the plate. The bar comes
off by loosening the cap, never by levering the pinion. Between angles the
bar turns with the rotor and the clamp stays tight; only the two pins move,
so no thread in the rig cycles more than a few times. The pinion stays on
the shaft for every experiment; it only comes off when the motor is torn
down.

Frame: X east, Y north, Z up, the shaft on Z. Hole 0 of the bolt circle is on
+X and the holes count counterclockwise seen from above, the sign the
notebooks use for angle. The base prints on its underside with the boss and
the pillars standing, the bar and the cap on their undersides with the hub
standing, the coupon on its top face, so every bore is vertical and no part needs
support.

Motor dimensions default to the SG90-class flat can and pinion measured in
the firmware repo's mechanical/sg90/measurements.py; the motor at hand is
measured with calipers and the numbers go in as Spec fields or command line
overrides before the print.

    uv run --group cad python -m rigs.motorrig [--pinion-d 2.3 ...]    writes rigs/motorrig/build/
"""
