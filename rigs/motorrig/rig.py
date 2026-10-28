"""The three parts, each a build function over one Spec."""
import math
from dataclasses import dataclass

from build123d import (Align, Box, Circle, Cylinder, FontStyle, Part, Polygon, Pos, Rectangle, RectangleRounded,
                       Rot, Text, extrude)


@dataclass(frozen=True)
class Spec:
    # motor: the flat can, rear face down, shaft up (sg90/measurements.py, [cal]/[dwg])
    can_d: float = 10.0          # across the round sides
    can_flat: float = 8.0        # across the flats
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
    pinion_d: float = 2.23       # over the teeth
    pinion_h: float = 2.17
    # fits, per side
    can_clear: float = 0.15      # the can drops in and sits square
    bore_clear: float = 0.10     # the bar slides over the pinion with no force; the clamp grips
    # base
    floor: float = 3.5
    recess: float = 0.8          # the can's front face this far below the base top, so the bar lies flat on the base
    bolt_r: float = 22.0
    holes: int = 12
    pin_d: float = 2.1           # a loose M2 screw or a 2 mm pin drops through the bar into the pillar, no thread
    pin_depth: float = 6.0
    rim: float = 5.0             # plate past the bolt circle
    boss_wall: float = 2.5       # the boss around the pocket, standing on the plate
    pillar_d: float = 6.0        # one pillar per hole, up to the bar's level
    label_r: float = 17.0        # the hole numbers, engraved in the plate inside the bolt circle
    tab_slot_x: float = 1.0      # tab slot, X by Y, in the pocket floor
    tab_slot_y: float = 2.3
    tab_slot_depth: float = 1.6  # tab plus solder
    boss_relief_clear: float = 0.2
    boss_relief_depth: float = 1.2
    channel_w: float = 2.4       # the lead channel in the underside, under the tabs, out to the +X edge
    channel_depth: float = 2.3   # meets the boss relief; a lead on the table stays a millimeter under the boss
    label_h: float = 0.4         # engraving depth, two layers
    label_font: float = 3.0
    # bar
    bar_w: float = 8.0
    bar_t: float = 4.0
    bar_end: float = 4.0         # past each hole
    bar_hole_d: float = 2.4      # M2 clearance
    socket_roof: float = 0.4     # the socket is this much deeper than the pinion reaches
    hub_d: float = 12.0          # the wrench hub around the socket, standing on the bar
    hub_h: float = 12.0          # from the bar's underside; the clamp screw sits high enough for a driver over the plate
    slit_w: float = 0.5          # from the bore out through the -Y edge: the +Y side is the hinge
    clamp_y: float = -3.2        # the clamp screw runs along X across the slit here
    clamp_pilot_len: float = 3.0
    clamp_z: float = 8.0
    clamp_pilot_d: float = 1.8
    clamp_clear_d: float = 2.2
    clamp_seat_d: float = 4.2    # a flat seat for the screw head in the hub's round side
    # coupon
    coupon_bores: tuple = (2.2, 2.3, 2.4, 2.5, 2.6)
    coupon_t: float = 4.0

    @property
    def pocket_depth(self):
        return self.can_l + self.recess

    @property
    def base_h(self):
        return self.floor + self.pocket_depth

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
    def boss_d(self):
        return self.can_d + 2 * self.can_clear + 2 * self.boss_wall

    @property
    def bar_l(self):
        return 2 * (self.bolt_r + self.bar_end)

    def check(self):
        if self.recess <= 0 or self.recess >= self.front_boss_l + self.shaft_l - self.pinion_h:
            raise ValueError("The recess must leave the whole pinion above the base top")
        if self.socket_depth >= self.bar_t:
            raise ValueError("The socket must not break through the bar's top")
        if self.pinion_bottom <= self.front_boss_l - self.recess:
            raise ValueError("The socket would reach the front boss")
        if self.channel_w < self.tab_slot_y:
            raise ValueError("The lead channel must take the whole tab slot")
        if self.channel_depth + self.boss_relief_depth < self.floor:
            raise ValueError("The lead channel must reach the boss relief, or a skin too thin to print is left between")
        if self.channel_depth + self.tab_slot_depth <= self.floor:
            raise ValueError("The tab slots must open into the lead channel")
        if max(self.boss_relief_depth, self.tab_slot_depth) >= self.floor or self.channel_depth >= self.floor:
            raise ValueError("Every relief must leave floor under it")
        if self.pin_depth >= self.base_h:
            raise ValueError("The pin holes must not break through the base")
        if self.pillar_d <= self.pin_d + 2.0:
            raise ValueError("A pillar must leave a wall around its pin hole")
        if self.label_r + self.label_font / 2 >= self.bolt_r - self.pillar_d / 2 or self.label_r - self.label_font / 2 <= self.boss_d / 2:
            raise ValueError("The hole numbers must fit between the boss and the pillars")
        if -self.clamp_y - self.clamp_clear_d / 2 <= self.bore_d / 2 + 0.8 or -self.clamp_y + self.clamp_clear_d / 2 >= self.hub_d / 2 - 1.5:
            raise ValueError("The clamp screw must cross the slit between the bore and the hub's side with a wall each way")
        if self.clamp_z - self.clamp_clear_d / 2 <= self.bar_t or self.clamp_z + self.clamp_seat_d / 2 >= self.hub_h:
            raise ValueError("The clamp screw must sit in the hub above the bar")
        if self.hub_d <= self.bar_w:
            raise ValueError("The hub must stand proud of the bar's sides")
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
    # a plate, a boss around the pocket and one pillar per hole, all to the bar's level
    p = extrude(Circle(s.plate_d / 2), s.floor)
    p += cyl(0, 0, 0, s.boss_d, s.base_h)
    for k in range(s.holes):
        a = 2 * math.pi * k / s.holes
        p += cyl(s.bolt_r * math.cos(a), s.bolt_r * math.sin(a), 0, s.pillar_d, s.base_h)
    # the can pocket, open at the top, the can's rear face on the floor
    p -= Pos(0, 0, s.floor) * extrude(flat_profile(s.can_d + 2 * s.can_clear, s.can_flat + 2 * s.can_clear),
                                      s.pocket_depth + 1)
    # reliefs under the rear boss and the tabs, so the rear face carries the motor
    p -= cyl(0, 0, s.floor - s.boss_relief_depth, s.rear_boss_d + 2 * s.boss_relief_clear, s.boss_relief_depth + 0.1)
    for sx in (-1, 1):
        p -= box(sx * s.term_span / 2, 0, s.floor - s.tab_slot_depth, s.tab_slot_x, s.tab_slot_y, s.tab_slot_depth + 0.1)
    # the lead channel in the underside: under both tab slots, along X, out through the +X edge
    x0 = -s.term_span / 2 - s.channel_w / 2
    p -= box((x0 + s.plate_d / 2 + 1) / 2, 0, -0.1, s.plate_d / 2 + 1 - x0, s.channel_w, s.channel_depth + 0.1)
    # the bolt circle, hole 0 on +X, counterclockwise from above
    for k in range(s.holes):
        a = 2 * math.pi * k / s.holes
        x, y = s.bolt_r * math.cos(a), s.bolt_r * math.sin(a)
        p -= cyl(x, y, s.base_h - s.pin_depth, s.pin_d, s.pin_depth + 0.1)
        p -= label(str(k), s.label_r * math.cos(a), s.label_r * math.sin(a), s.floor - s.label_h, s.label_h + 0.1,
                   s.label_font, angle=math.degrees(a) - 90)
    p.label = "motorrig_base"
    return p


def build_bar(s: Spec = Spec()) -> Part:
    s.check()
    p = extrude(RectangleRounded(s.bar_l, s.bar_w, s.bar_w / 2 - 0.01), s.bar_t)
    for sx in (-1, 1):
        p -= cyl(sx * s.bolt_r, 0, -0.1, s.bar_hole_d, s.bar_t + 0.2)
    # the wrench hub around the socket, and the socket as a blind bore from the underside
    p += cyl(0, 0, 0, s.hub_d, s.hub_h)
    p -= cyl(0, 0, -0.1, s.bore_d, s.socket_depth + 0.1)
    # the slit from the bore out through the -Y side, full height: the +Y side is the hinge
    p -= box(0, -s.hub_d / 4 - 1, -0.1, s.slit_w, s.hub_d / 2 + 2, s.hub_h + 0.2)
    # the clamp screw along X across the slit: a flat seat and a clearance hole on the -X side, a pilot on the +X side
    p -= Pos(-s.hub_d / 2 + 1.0, s.clamp_y, s.clamp_z) * Rot(0, -90, 0) * Cylinder(
        s.clamp_seat_d / 2, s.hub_d, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(0.1, s.clamp_y, s.clamp_z) * Rot(0, -90, 0) * Cylinder(
        s.clamp_clear_d / 2, s.hub_d, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(-0.1, s.clamp_y, s.clamp_z) * Rot(0, 90, 0) * Cylinder(
        s.clamp_pilot_d / 2, s.clamp_pilot_len + 0.1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # the index arrow on the top face at the +X end, pointing out along the bar
    ax = s.bolt_r - s.bar_hole_d / 2 - 3.0
    p += Pos(0, 0, s.bar_t) * extrude(Pos(ax, 0) * Polygon((0, -1.2), (2.0, 0), (0, 1.2), align=None), s.label_h)
    p.label = "motorrig_bar"
    return p


def build_coupon(s: Spec = Spec()) -> Part:
    pitch = 6.0
    n = len(s.coupon_bores)
    p = extrude(RectangleRounded(pitch * n + 2, 8.0, 1.0), s.coupon_t)
    for i, d in enumerate(s.coupon_bores):
        x = (i - (n - 1) / 2) * pitch
        p -= cyl(x, 1.0, -0.1, d, s.socket_depth + 0.1)
        p -= label(f"{d:.1f}", x, -2.6, s.coupon_t - s.label_h, s.label_h + 0.1, 1.8)
    p.label = "motorrig_coupon"
    return p
