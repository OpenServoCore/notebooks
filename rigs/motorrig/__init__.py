"""The bare-motor rig for the stage 2 experiments: a base plate with a boss
that seats the motor can and twelve pillars on a bolt circle, a wrench bar
that holds the rotor by its pinion at one of the twelve angles, and a fit
coupon that picks the bar's bore for the pinion in hand.

Nothing in the rig pushes or pulls along the shaft. The motor sits on its
can's rear face, with reliefs under the rear boss and the terminal tabs, so a
load on the motor goes into the can. The leads are soldered to the tabs
before the motor goes in; they drop through the tab slots into a channel in
the base's underside and leave at the edge below hole 0, so the base sits
flat on the table with the leads under it and nothing pulls on the tabs.
The bar's socket is a slip bore over the pinion inside a wrench hub, closed
by a clamp screw across a slit through the hub's side, high enough above the
plate for a driver along the bar; the grip is radial and the bar comes off
by loosening the screw, never by levering the pinion. The pinion stays on the shaft for every
experiment; it only comes off when the motor is torn down.

Frame: X east, Y north, Z up, the shaft on Z. Hole 0 of the bolt circle is on
+X and the holes count counterclockwise seen from above, the sign the
notebooks use for angle. The base prints on its underside with the boss and
the pillars standing, the bar on its underside with the hub standing, the
coupon on its top face, so every bore is vertical and no part needs
support.

Motor dimensions default to the SG90-class flat can and pinion measured in
the firmware repo's mechanical/sg90/measurements.py; the motor at hand is
measured with calipers and the numbers go in as Spec fields or command line
overrides before the print.

    uv run --group cad python -m rigs.motorrig [--pinion-d 2.3 ...]    writes rigs/motorrig/build/
"""
