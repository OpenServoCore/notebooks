"""The three parts, each a build function over one Spec."""
import math
from dataclasses import dataclass

from build123d import (Align, Box, Circle, Cylinder, FontStyle, Part, Polygon, Pos, Rectangle, RectangleRounded,
                       Rot, Text, extrude)


@dataclass(frozen=True)
class Spec:
    # motor: the flat can, rear face down, shaft up; the can and the pinion are mg90-b measured with calipers,
    # the rest the SG90-class dimensions (sg90/measurements.py, [cal]/[dwg]) until measured
    can_d: float = 9.95          # across the round sides
    can_flat: float = 7.95       # across the flats
    can_l: float = 12.3          # rear face to front face
    front_boss_d: float = 4.0
    front_boss_l: float = 0.6
    rear_boss_d: float = 4.0
    rear_boss_l: float = 1.0
    shaft_d: float = 1.0
    shaft_l: float = 2.7         # stickout past the front boss
    term_span: float = 7.3       # tab centers, along the round axis (X)
    term_w: float = 1.5          # tab width, along Y
    term_t: float = 0.3          # tab thickness, along X
    term_l: float = 1.0          # tab stickout past the rear face, before solder
    pinion_d: float = 2.25       # over the teeth
    pinion_h: float = 2.17
    # fits, per side
    can_clear: float = 0.10      # the can drops in and sits square
    bore_clear: float = 0.05     # the bar drops over the pinion; the slit closes the rest
    # base
    floor: float = 3.5           # under the can's rear face; the lead channel runs in it
    deck_h: float = 8.0          # the solid plate, half the base's height, so the boss and the ring stand short
    pocket_frac: float = 2 / 3   # of the can's length in the boss; the rest stands above it, to pull the motor by the can
    recess: float = 0.8          # the can's front face this far below the ring's top, so the bar lies on the ring over it
    bolt_r: float = 22.0
    holes: int = 12
    pin_d: float = 2.1           # a loose M2 screw or a 2 mm pin drops through the bar into the ring, no thread
    pin_depth: float = 6.0
    rim: float = 5.0             # plate past the bolt circle
    boss_wall: float = 2.5       # the boss around the pocket, standing on the deck
    ring_w: float = 6.0          # the ring on the bolt circle with the pin holes, up to the bar's level
    label_r: float = 13.5        # the hole numbers, raised on the deck between the boss and the ring
    tab_room: float = 1.5        # the well past each tab center along X: the tab, its solder and the lead
    well_y: float = 4.4          # the well across: the rear boss with its clearance; the can's flats carry the motor
    well_depth: float = 1.6      # tab plus solder; the well opens into the lead channel under it
    channel_w: float = 2.5       # the lead channel in the underside, under the well, out to the +X edge
    channel_depth: float = 2.3   # meets the well; a lead on the table stays a millimeter under the can
    label_h: float = 0.6         # the numbers and the arrow stand this high, three layers
    label_font: float = 4.5
    # bar
    bar_w: float = 8.0
    bar_t: float = 4.0
    bar_end: float = 4.0         # past each hole
    bar_hole_d: float = 2.4      # M2 clearance
    socket_roof: float = 0.4     # the socket is this much deeper than the pinion reaches
    # the clamp: one slit along the bar through the bore, two M2 screws across it, each into a nut trapped in the hub
    hub_x: float = 15.0          # the hub's span along the bar
    hub_w: float = 10.0          # across the bar
    hub_h: float = 8.0           # from the bar's underside
    slit_w: float = 0.5
    slit_l: float = 38.0         # centered on the bore, up to the pin holes, so the arm halves flex over their whole length
    clamp_x: float = 4.3         # the two screws, at +/- this along the bar
    clamp_z: float = 3.5         # the screw axis above the bar's underside, as low as the nut floor allows: nearer the pinion, less tilt
    screw_clear_d: float = 2.2   # through the whole hub, along Y
    nut_af: float = 4.0          # M2 nut across flats
    nut_t: float = 1.6
    nut_clear: float = 0.25      # on the slot's width and thickness
    nut_wall: float = 1.5        # between the slit and the nut slot

    @property
    def pocket_depth(self):
        return self.pocket_frac * self.can_l

    @property
    def boss_h(self):
        return self.floor + self.pocket_depth

    @property
    def base_h(self):
        """The ring's top, the bar's level."""
        return self.floor + self.can_l + self.recess

    @property
    def well_x(self):
        return self.term_span + self.tab_room

    @property
    def pinion_top(self):
        """The pinion's top above the base top."""
        return self.front_boss_l + self.shaft_l - self.recess

    @property
    def pinion_bottom(self):
        return self.pinion_top - self.pinion_h

    @property
    def socket_depth(self):
        return self.pinion_top + self.socket_roof

    @property
    def bore_d(self):
        return self.pinion_d + 2 * self.bore_clear

    @property
    def plate_d(self):
        return 2 * (self.bolt_r + self.rim)

    @property
    def ring_r(self):
        """The ring's inner and outer radii."""
        return self.bolt_r - self.ring_w / 2, self.bolt_r + self.ring_w / 2

    @property
    def boss_d(self):
        return self.can_d + 2 * self.can_clear + 2 * self.boss_wall

    @property
    def bar_l(self):
        return 2 * (self.bolt_r + self.bar_end)

    @property
    def nut_corner(self):
        return self.nut_af / math.cos(math.radians(30))

    @property
    def nut_slot_x(self):
        return self.nut_af + self.nut_clear

    @property
    def nut_slot_y(self):
        return self.nut_t + self.nut_clear

    @property
    def nut_y0(self):
        """The nut slot's near face, on the +Y side of the slit."""
        return self.slit_w / 2 + self.nut_wall

    @property
    def nut_z0(self):
        """The nut slot's floor: the nut's corners clear the screw axis."""
        return self.clamp_z - self.nut_corner / 2 - self.nut_clear / 2

    @property
    def screw_l(self):
        """Head on the -Y face, tip flush with the +Y face."""
        return self.hub_w

    def check(self):
        if self.recess <= 0 or self.recess >= self.front_boss_l + self.shaft_l - self.pinion_h:
            raise ValueError("The recess must leave the whole pinion above the base top")
        if self.socket_depth >= self.bar_t:
            raise ValueError("The socket must not break through the bar's top")
        if self.pinion_bottom <= self.front_boss_l - self.recess:
            raise ValueError("The socket would reach the front boss")
        if self.pocket_depth >= self.can_l - 2.0 or self.pocket_depth <= self.can_l / 2:
            raise ValueError("The boss must hold the can by more than half and leave a few millimeters to pull it by")
        if self.well_y >= self.can_flat - 2.0 or self.well_x >= self.can_d - 0.5:
            raise ValueError("The well must leave the can's rear face a ledge along both flats")
        if self.well_y <= self.rear_boss_d or self.well_x <= self.term_span + self.term_t:
            raise ValueError("The well must clear the rear boss and both tabs")
        if self.channel_depth + self.well_depth <= self.floor:
            raise ValueError("The well must open into the lead channel")
        if self.well_depth >= self.floor or self.channel_depth >= self.floor:
            raise ValueError("The well and the channel must leave floor under them")
        if self.pin_depth >= self.base_h:
            raise ValueError("The pin holes must not break through the base")
        if self.ring_w <= self.pin_d + 2.0:
            raise ValueError("The ring must leave a wall around its pin holes")
        if self.deck_h <= self.floor or self.deck_h >= min(self.boss_h, self.base_h - self.pin_depth):
            raise ValueError("The deck must stand above the floor and below the boss top and the pin holes")
        if self.label_r + self.label_font / 2 >= self.ring_r[0] or self.label_r - self.label_font / 2 <= self.boss_d / 2:
            raise ValueError("The hole numbers must fit between the boss and the ring")
        if 2 * math.pi * self.label_r / self.holes <= 1.5 * self.label_font:
            raise ValueError("The hole numbers must not run into their neighbors")
        if self.bore_clear <= 0:
            raise ValueError("The bore must slip over the pinion; the slit does the gripping")
        if self.slit_l / 2 + 1.5 >= self.bolt_r - self.bar_hole_d / 2:
            raise ValueError("The slit must stop short of the pin holes, so the ends stay one piece")
        if self.slit_l / 2 < self.hub_x / 2 + 2.0:
            raise ValueError("The slit must run past the hub, so the halves flex in the arm and not in the hub")
        if self.hub_w <= self.bar_w:
            raise ValueError("The hub must be wider than the arm")
        if self.clamp_x - self.nut_slot_x / 2 <= self.bore_d / 2 + 0.8:
            raise ValueError("The nut slots must leave a wall to the bore")
        if self.clamp_x + self.nut_slot_x / 2 >= self.hub_x / 2 - 1.0:
            raise ValueError("The nut slots must leave a wall to the hub's end")
        if self.nut_y0 + self.nut_slot_y >= self.hub_w / 2 - 1.0:
            raise ValueError("The nut slots must leave a wall to the hub's face")
        if self.nut_z0 <= 1.0 or self.clamp_z + self.screw_clear_d / 2 >= self.hub_h - 1.0:
            raise ValueError("The screws must sit inside the hub with a floor under the nuts")
        if self.plate_d > 250:
            raise ValueError("The base exceeds the printer bed")


def flat_profile(d, flat):
    """The can's section: a circle cut by two flats."""
    return Circle(d / 2) & Rectangle(2 * d, flat)


def box(x, y, z, w, l, h):
    return Pos(x, y, z) * Box(w, l, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def cyl(x, y, z, d, h):
    return Pos(x, y, z) * Cylinder(d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def label(text, x, y, z, h, size, angle=0):
    t = Text(text, font_size=size, font_style=FontStyle.BOLD, align=(Align.CENTER, Align.CENTER))
    return Pos(x, y, z) * Rot(0, 0, angle) * extrude(t, h)


def build_base(s: Spec = Spec()) -> Part:
    s.check()
    # a deck, a boss around the pocket to two thirds of the can, and a ring on the bolt circle to the bar's level
    p = extrude(Circle(s.plate_d / 2), s.deck_h)
    p += cyl(0, 0, 0, s.boss_d, s.boss_h)
    p += extrude(Circle(s.ring_r[1]) - Circle(s.ring_r[0]), s.base_h)
    # the can pocket, open at the top, the can's rear face on the floor
    p -= Pos(0, 0, s.floor) * extrude(flat_profile(s.can_d + 2 * s.can_clear, s.can_flat + 2 * s.can_clear),
                                      s.pocket_depth + 1)
    # one well in the floor under the rear boss and both tabs, so the rear face carries the motor along its flats
    p -= box(0, 0, s.floor - s.well_depth, s.well_x, s.well_y, s.well_depth + 0.1)
    # the lead channel in the underside: under the well, along X, out through the +X edge
    x0 = -s.well_x / 2
    p -= box((x0 + s.plate_d / 2 + 1) / 2, 0, -0.1, s.plate_d / 2 + 1 - x0, s.channel_w, s.channel_depth + 0.1)
    # the bolt circle, hole 0 on +X, counterclockwise from above
    for k in range(s.holes):
        a = 2 * math.pi * k / s.holes
        x, y = s.bolt_r * math.cos(a), s.bolt_r * math.sin(a)
        p -= cyl(x, y, s.base_h - s.pin_depth, s.pin_d, s.pin_depth + 0.1)
        p += label(str(k), s.label_r * math.cos(a), s.label_r * math.sin(a), s.deck_h, s.label_h,
                   s.label_font, angle=math.degrees(a) - 90)
    p.label = "motorrig_base"
    return p


def build_bar(s: Spec = Spec()) -> Part:
    s.check()
    p = extrude(RectangleRounded(s.bar_l, s.bar_w, s.bar_w / 2 - 0.01), s.bar_t)
    for sx in (-1, 1):
        p -= cyl(sx * s.bolt_r, 0, -0.1, s.bar_hole_d, s.bar_t + 0.2)
    p += box(0, 0, 0, s.hub_x, s.hub_w, s.hub_h)
    # the socket, a blind bore from the underside
    p -= cyl(0, 0, -0.1, s.bore_d, s.socket_depth + 0.1)
    # the slit along the bar through the bore and the whole height, so the two halves close on the pinion
    p -= box(0, 0, -0.1, s.slit_l, s.slit_w, s.hub_h + 0.2)
    for sx in (-1, 1):
        # the screw across the slit, head on the -Y face, through to the +Y face
        p -= Pos(sx * s.clamp_x, -s.hub_w / 2 - 0.1, s.clamp_z) * Rot(-90, 0, 0) * Cylinder(
            s.screw_clear_d / 2, s.hub_w + 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
        # the nut slot in the +Y half, open at the hub's top, flats along the bar so the nut cannot turn
        p -= box(sx * s.clamp_x, s.nut_y0 + s.nut_slot_y / 2, s.nut_z0, s.nut_slot_x, s.nut_slot_y, s.hub_h - s.nut_z0 + 0.1)
    # the index arrow on the top face at the +X end, pointing out along the bar
    ax = s.bolt_r - s.bar_hole_d / 2 - 3.0
    p += Pos(0, 0, s.bar_t) * extrude(Pos(ax, 0) * Polygon((0, -1.2), (2.0, 0), (0, 1.2), align=None), s.label_h)
    p.label = "motorrig_bar"
    return p

