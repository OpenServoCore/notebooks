"""The four parts and the servo envelope, each built over one Spec."""
import math
from dataclasses import dataclass

from build123d import (Align, Box, Circle, Cylinder, Part, Polygon, Pos,
                       Rectangle, RectangleRounded, Rot, SlotCenterToCenter,
                       Text, extrude, offset)

BED_X = 250.0      # MK3S bed
BED_Y = 210.0
BED_Z = 210.0      # MK3S build height
EPS = 0.1          # cutter overrun past a face

# Printed fits, per side, and the M3 button head hardware, as the stencil
# jig in the firmware repo's mechanical/stenciljig/common.py defines them.
FIT_TIGHT = 0.05   # push-in, holds inverted
FIT_LOOSE = 0.15   # free slide
M3_HOLE = 3.4      # vertical M3 clearance, prints about 3.25
BTN_CBORE = 1.65 + FIT_TIGHT    # ISO 7380 head height: the dome sits flush
BTN_CBORE_D = 6.5  # drop-in room around the 5.7 head

# Worth a caliper check on the servo and the horn at hand before the base
# prints; the defaults come from a STEP model of the Tower Pro MG90S.
MEASURE = (
    "body_h", "ear_z", "ear_span", "ear_t", "ear_w", "shaft_offset",
    "horn_above_body", "horn_hub_above", "head_d", "head_h",
)
# The test print's pick, before the base: settable from the command line.
FITS = ("key_fit", "servo_clear")


@dataclass(frozen=True)
class Arm:
    """One horn arm, out from the axis along its own direction: straight
    edges from the axis to a round tip."""
    angle: float                # degrees from +X at mid travel
    reach: float                # axis to the tip
    holes: tuple                # axis to each hole
    tip_r: float                # the round tip, half the width there
    axis_half_w: float          # half width where the edges, carried
    #                             straight, cross the axis


@dataclass(frozen=True)
class Spec:
    # servo: the Tower Pro MG90S STEP model (GrabCAD, "Tower Pro MG90S
    # micro servo", the case and its four-arm horn); the model's case is
    # 22.55 x 12.15 below the ears and 22.6 long above them, against the
    # 22.8 x 12.2 the datasheets quote, so the fit print tries the pocket
    servo_w: float = 12.15
    servo_l: float = 22.6
    servo_clear: float = 0.2        # per side: at 0.05 the fit print's
    #                                 pocket took the body but not pushed
    #                                 home; two screws through the ears
    #                                 locate the servo, not the pocket
    ear_hole_dy: float = 13.75      # body center to each ear's screw hole
    ear_pilot_wall: float = 1.2     # least wall from a pilot to the pocket
    body_h: float = 22.55           # case bottom to case top, no output boss
    ear_z: float = 18.4             # case bottom to ear underside
    ear_span: float = 32.2          # across both ears, 2.5 holes 27.5 apart
    ear_t: float = 2.5
    ear_w: float = 12.1             # ear plan width
    shaft_offset: float = 5.375     # case center to the output axis
    case_boss_d: float = 11.8       # output boss; envelope only
    case_boss_h: float = 5.75       # case top to the boss top; the spline
    #                                 stands 3.95 over it, 4.9 across
    horn_above_body: float = 10.0   # case top to the arm plate's top face:
    #                                 the 5.75 boss, the horn's 2.2 collar
    #                                 resting on it, the 2.05 plate; a horn
    #                                 that bottoms on the spline instead
    #                                 sits up to 1.75 higher, so MEASURE
    # horn: the model draws two 7-hole long arms, 36.35 tip to tip; the
    # horn at hand has 6 holes on one of them, so that arm is one 2.0 hole
    # pitch shorter with the same edges. The long arms run straight through
    # the hub, 6.65 wide at the axis; the short arms are parallel sided.
    # The owner's calipers on the horn at hand: 34.14 tip to tip (model
    # 34.35), 16.65 across the short arms (16.25), plate 1.90 (2.05),
    # collar bottom to plate top 4.40 (4.25), collar bottom to hub top 5.28
    # (4.55); the fit print settles the key, the recess over the hub covers
    # both hub readings. The longer arm points +X under the lock ear at mid
    # travel.
    horn_arms: tuple = (
        Arm(0, 18.175, (4.5, 6.5, 8.5, 10.5, 12.5, 14.5, 16.5), 2.0, 3.325),
        Arm(90, 8.125, (4.5, 6.5), 1.85, 1.85),
        Arm(180, 16.175, (4.5, 6.5, 8.5, 10.5, 12.5, 14.5), 2.18, 3.325),
        Arm(270, 8.125, (4.5, 6.5), 1.85, 1.85),
    )
    horn_hole_d: float = 1.3
    horn_plate_t: float = 2.05      # the arm plate
    horn_collar_h: float = 2.2      # the splined collar under the plate
    horn_hub_above: float = 0.3     # the hub over the plate's top face,
    #                                 around a 4.55 x 0.9 screw counterbore
    horn_hub_d: float = 6.65        # hub and collar
    head_d: float = 3.8             # the servo's horn screw head, in the
    #                                 hub's counterbore: an M2 pan head;
    #                                 MEASURE, an M2.5 head is 4.5 across
    head_h: float = 1.1             # head top over the hub's top face: a
    #                                 2.0 tall M2.5 pan head in the 0.9
    #                                 counterbore; an M2 head sits 0.5 over
    # heights
    horn_above_base: float = 1.5    # owner's band: 1 to 2 mm
    shim_room: float = 0.5          # ledge cut this much low; card shims
    #                                 under the ears set the horn height
    bottom_clear: float = 1.0       # shimmed case bottom above the underside
    ear_gap: float = 0.5            # mount ear over the base top: one M3
    #                                 washer (DIN 125) fills it at the lock
    # tag: stages/0-instruments/camera-tag, DICT_4X4_50 at 250 mm
    tag_edge: float = 25.0          # black square
    tag_cells: int = 6              # 4 x 4 bits plus a one-cell black border
    quiet_cells: float = 1.5        # white border inside the cut line
    working_distance: float = 250.0     # lens to tags
    frame_per_distance: float = 1.4     # 1x lens: 28 cm wide from 20 cm
    frame_aspect: float = 9 / 16        # 1080p
    paper_clear: float = 0.25       # per side: the cut paper in the pad's
    #                                 pocket, placement slop on the mount
    #                                 and the arm, whose tag faces print on
    #                                 the bed and stay flat, no pocket
    paper_pocket: float = 0.2       # one layer in the pad; the paper is 0.1
    plate_rim: float = 1.2          # plate past the paper
    # tag mount and load arm, the shared horn seat
    key_fit: float = 0.25           # per side on the arms: the fit print's
    #                                 0.25 pocket pressed on by hand and
    #                                 held; 0.05 and 0.15 did not go on
    key_depth: float = 1.5          # into the 2.05 plate: the skirt stays
    #                                 clear of the boss under the plate
    key_wall: float = 1.6
    hub_clear: float = 0.4          # around the hub and the screw head, so
    #                                 the arms carry the seat
    head_clear: float = 1.0         # over the hub and the screw head: the
    #                                 recess ceilings never bear, and 1.0
    #                                 covers a hub 0.9 over the plate (the
    #                                 owner's calipers) as well as the
    #                                 model's 0.3
    plate_t: float = 7.0            # arm plate's top face to the tag plane
    retain_d: float = 1.4           # optional retaining screw into a long
    #                                 arm's 1.3 hole: M1.4 or 1.4 self-tapper
    retain_clear: float = 0.3       # per side: retains, never locates
    retain_head_d: float = 3.2      # drop-in room around a 2.6 pan head
    retain_floor: float = 2.0       # under the retaining screw heads
    retain_screws: bool = True
    retain_holes: tuple = ((0, 8.5), (180, 8.5))   # (angle, radius): the
    #                                 long arms' third holes, opposite
    # lock: one ear, one M3 button head screw into one heat-set insert
    slot_deg: float = 3.0           # angular play of the slot each way
    ear_wall: float = 1.2           # around the head counterbore
    ear_head_sink: float = 0.3      # head top below the tag plane
    ear_clamp: float = 2.4          # ear under the head, twelve layers
    access_clear: float = 0.5       # head counterbore past the clip edge
    insert_hole_d: float = 4.0      # M3 x 5.7 heat-set insert
    insert_od: float = 4.6
    insert_l: float = 5.7
    insert_relief: float = 1.0      # melt room under the insert
    min_wall: float = 1.6
    # tag clip
    clip_lip_t: float = 0.6         # three layers over the tag plane
    clip_wall: float = 1.2
    clip_skirt: float = 3.0         # down the plate's side
    clip_fit: float = FIT_TIGHT     # friction, holds inverted
    clip_overlap: float = 1.0       # lip over the paper's quiet border
    notch_clear: float = 0.5        # skirt notch past the ear or the beam
    # fit test print: three key pockets and a short servo pocket
    fit_trials: tuple = (0.05, 0.15, 0.25, 0.35)  # key fits to try, per
    #                                 side; 0.35 covers short arms 0.2 longer
    #                                 than the model's, as the calipers read
    fit_floor: float = 1.5          # under the key pockets
    fit_margin: float = 2.5         # plate past the pockets and the labels
    fit_web: float = 3.0            # between neighboring pockets
    fit_body_h: float = 10.0        # servo pocket below the ear ledge
    fit_wall: float = 2.0           # around the servo pocket
    label_h: float = 3.5            # font size of the fit labels
    label_depth: float = 0.4        # two layers into the plate
    # load arm: four stacks of US quarters (5.670 g, 24.26 x 1.75 mm) in
    # a 2 x 2 block of cups centered on arm_r, two along the arm and two
    # across it; stacks go in by pairs, so each pair balances across the
    # arm, and the inner pair, the outer pair and both make the load ladder
    arm_r: float = 55.0             # axis to the block's center
    stack_n: int = 12               # coins a cup holds
    unit_g: float = 5.670
    unit_d: float = 24.26
    unit_t: float = 1.75
    cup_clear: float = 1.0          # on the diameter: the coins drop in
    cup_room: float = 0.5           # over the stack, under the lid's plug
    cup_wall: float = 1.6
    cup_floor: float = 1.6
    # lid: a plate dropped into a pocket in the block's open face, four
    # plugs into the bores, one M2 self-tapping screw into the block's
    # center, where the four walls meet
    lid_t: float = 2.0
    lid_fit: float = FIT_LOOSE      # per side in its pocket
    lid_margin: float = 0.3         # pocket past the bores
    plug_h: float = 1.5
    plug_clear: float = 0.3         # per side in the bore
    lid_hole_d: float = 2.2         # M2 clearance
    beam_w: float = 10.0
    travel_deg: float = 180.0       # servo travel, mid travel centered
    travel_margin: float = 10.0     # past each end, for the sweep check
    arm_board_clear: float = 2.0    # loaded arm to the board pocket at the
    #                                 ends of travel
    # base
    well_clear: float = 0.75        # horn well around the mount's skirt
    well_floor_clear: float = 1.0   # under the skirt
    ear_fit: float = FIT_LOOSE      # ear well, per side
    wire_w: float = 7.0             # bus and power lead grooves, the test
    #                                 harness's lead slot
    servo_lead_w: float = 4.5       # the servo's three-wire ribbon, 3 x
    #                                 1.3 mm, so the wall above bridges less
    lead_boot_w: float = 6.5        # the lead's strain relief boot where it
    #                                 leaves the case, 5.9 x 1.7 in the
    #                                 model, has to pass the end wall's slot
    lead_slot_depth: float = 4.0    # into the pocket's end wall: the boot
    #                                 and the ribbon's bend down the slot
    channel_depth: float = 3.0      # servo lead under the base
    groove_depth: float = 3.0       # bus and power leads in the base top
    pad_margin: float = 1.0         # reference pad past the keep-out circle
    board_gap: float = 4.0          # web between the pad and the board
    rim: float = 5.0
    back_rim: float = 12.0          # behind the pad and the board: ties
    # board: osc-dev-v006 (osc-dev-v006.kicad_pcb), pocket, pads and pilots
    # as the test harness's, the board turned so its long side runs along X
    board_l: float = 50.0           # along X
    board_w: float = 40.0           # along Y
    board_r: float = 4.0            # corner radius
    board_t: float = 1.6
    board_clear: float = 0.15       # per side in the pocket
    board_recess: float = 1.6       # pocket top to the board's seat: flush
    underside_clear: float = 2.8    # under the board, for its bottom parts
    hole_dx: float = 21.0           # mounting holes from the board center
    hole_dy: float = 16.0
    pad_r: float = 2.8              # the pads the board seats on
    pilot_d: float = 1.8            # M2 self-tapping pilot
    pilot_depth: float = 6.0        # below the seat, the harness's 0.8 floor
    #                                 under its 6.8 seat
    # bus J9/J10 and power J1 along X from the board center, the board
    # turned so J9/J10 face the back (osc-dev-v006.kicad_pcb)
    bus_dx: float = -8.8
    power_dx: float = 18.3
    tie_slot_w: float = 2.0         # 2.5 mm zip tie, band 1.0 thick
    tie_slot_l: float = 3.5
    tie_web: float = 2.0            # between a tie slot and its lead
    tie_band_depth: float = 1.5     # band groove, so the band lies flush
    wing_reach: float = 22.0        # past the base end: a clamp jaw
    wing_t: float = 6.0
    wing_depth: float = 30.0        # flat leg, back from the front face
    wing_h: float = 20.0            # standing leg, up from the underside

    @property
    def cell(self):
        return self.tag_edge / self.tag_cells

    @property
    def quiet(self):
        return self.quiet_cells * self.cell

    @property
    def paper(self):
        """The cut tag: black square plus its quiet border."""
        return self.tag_edge + 2 * self.quiet

    @property
    def paper_side(self):
        return self.paper + 2 * self.paper_clear

    @property
    def plate_side(self):
        return self.paper_side + 2 * self.plate_rim

    @property
    def clip_side(self):
        return self.plate_side + 2 * (self.clip_fit + self.clip_wall)

    @property
    def clip_window(self):
        return self.paper - 2 * self.clip_overlap

    @property
    def base_top(self):
        return (self.bottom_clear + self.body_h + self.horn_above_body
                - self.horn_above_base)

    @property
    def horn_top(self):
        return self.base_top + self.horn_above_base

    @property
    def tag_plane(self):
        """The mount's top face and the reference pad's: both papers."""
        return self.horn_top + self.plate_t

    @property
    def case_top(self):
        return self.horn_top - self.horn_above_body

    @property
    def case_bottom(self):
        return self.case_top - self.body_h

    @property
    def ledge_z(self):
        return self.case_bottom + self.ear_z - self.shim_room

    @property
    def body_y(self):
        """The case center; the axis sits at its back end."""
        return -self.shaft_offset

    @property
    def body_end(self):
        """The case's back end wall, where the lead leaves."""
        return self.body_y + self.servo_l / 2 + self.servo_clear

    @property
    def hub_top(self):
        return self.horn_top + self.horn_hub_above

    @property
    def head_top(self):
        return self.hub_top + self.head_h

    @property
    def collar_bottom(self):
        return self.horn_top - self.horn_plate_t - self.horn_collar_h

    @property
    def horn_reach(self):
        return max(arm.reach for arm in self.horn_arms)

    @property
    def skirt_r(self):
        return self.horn_reach + self.key_fit + self.key_wall

    @property
    def well_r(self):
        return self.skirt_r + self.well_clear

    @property
    def well_floor(self):
        return self.horn_top - self.horn_plate_t - self.well_floor_clear

    @property
    def ear_bottom(self):
        return self.base_top + self.ear_gap

    @property
    def insert_r(self):
        return self.clip_side / 2 + BTN_CBORE_D / 2 + self.access_clear

    @property
    def slot_half(self):
        return self.insert_r * math.tan(math.radians(self.slot_deg))

    @property
    def ear_cbore(self):
        return BTN_CBORE + self.ear_head_sink

    @property
    def lock_w(self):
        """The ear's width across the axis."""
        return 2 * (self.slot_half + BTN_CBORE_D / 2 + self.ear_wall)

    @property
    def ear_tip(self):
        return self.insert_r + BTN_CBORE_D / 2 + self.ear_wall

    @property
    def lock_clamp(self):
        """Under the screw head: ear plus washer."""
        return self.tag_plane - self.ear_cbore - self.ear_bottom + self.ear_gap

    @property
    def sweep_r(self):
        """Farthest point of the mount with its clip, ear corners included."""
        return max(math.hypot(self.ear_tip, self.lock_w / 2),
                   self.clip_side / math.sqrt(2))

    @property
    def keep_out_r(self):
        return self.sweep_r + self.quiet

    @property
    def arm_sweep_y(self):
        """How far toward the back the load arm reaches over its travel
        plus the margin, in plan."""
        half = self.block_side / 2
        pts = [(sx * half, -(self.arm_r + sy * half))
               for sx in (-1, 1) for sy in (-1, 1)]
        pts += [(sx * self.plate_side / 2, sy * self.plate_side / 2)
                for sx in (-1, 1) for sy in (-1, 1)]
        end = self.travel_deg / 2 + self.travel_margin
        angles = [math.radians(a / 10) for a in range(-round(10 * end),
                                                       round(10 * end) + 1)]
        return max(x * math.sin(a) + y * math.cos(a)
                   for x, y in pts for a in angles)

    @property
    def ref_y(self):
        """The reference pad and the board share this Y: past the keep-out
        circle, and the board pocket past the loaded arm's sweep."""
        return max(self.keep_out_r + self.pad_margin + self.plate_side / 2,
                   self.arm_sweep_y + self.arm_board_clear
                   + self.board_pocket[1] / 2)

    @property
    def board_x(self):
        return self.plate_side / 2 + self.board_gap + self.board_pocket[0] / 2

    @property
    def board_pocket(self):
        """Long axis along X: the bus edge faces the back slots."""
        return (self.board_l + 2 * self.board_clear,
                self.board_w + 2 * self.board_clear)

    @property
    def x_min(self):
        return -max(self.plate_side / 2, self.well_r) - self.rim

    @property
    def x_max(self):
        return self.board_x + self.board_pocket[0] / 2 + self.rim

    @property
    def y_min(self):
        ear_well = self.ear_span / 2 + self.ear_fit
        return min(self.body_y - ear_well, -self.well_r) - self.rim

    @property
    def y_max(self):
        return self.ref_y + self.plate_side / 2 + self.back_rim

    @property
    def tie_y(self):
        return self.y_max - self.back_rim / 2

    def tie_dx(self, lead_w):
        """A tie slot's center from a lead's centerline."""
        return lead_w / 2 + self.tie_web + self.tie_slot_w / 2

    @property
    def stack_g(self):
        return self.stack_n * self.unit_g

    @property
    def cup_bore(self):
        return self.unit_d + self.cup_clear

    @property
    def cup_pitch(self):
        """Neighboring cups share one wall."""
        return self.cup_bore + self.cup_wall

    @property
    def block_side(self):
        """The 2 x 2 cup block, square in plan."""
        return self.cup_pitch + self.cup_bore + 2 * self.cup_wall

    @property
    def lid_pocket(self):
        return self.cup_pitch + self.cup_bore + 2 * self.lid_margin

    @property
    def lid_side(self):
        return self.lid_pocket - 2 * self.lid_fit

    @property
    def arm_top(self):
        """The cup floors and the hub tag: one plane, the arm's print
        face; the coins stack from the lid's plugs up to the floors."""
        return (self.ear_bottom + self.lid_t + self.plug_h
                + self.stack_n * self.unit_t + self.cup_room + self.cup_floor)

    @property
    def arm_reach(self):
        """Farthest point of the load arm from the axis."""
        return math.hypot(self.arm_r + self.block_side / 2,
                          self.block_side / 2)

    def stack_radii(self):
        """Axis to the inner and the outer pair of cups."""
        return (self.arm_r - self.cup_pitch / 2, self.arm_r + self.cup_pitch / 2)

    @property
    def notch_w(self):
        return max(self.lock_w, self.beam_w) + 2 * self.notch_clear

    @property
    def frame(self):
        w = self.frame_per_distance * self.working_distance
        return w, w * self.frame_aspect

    def check(self):
        if not 1.0 <= self.horn_above_base <= 2.0:
            raise ValueError("The horn's top face must sit 1 to 2 mm above "
                             "the base top")
        if not 0 < self.shim_room <= self.bottom_clear:
            raise ValueError("Unshimmed, the case must stay above the "
                             "underside")
        if not 0 < self.ear_gap < self.horn_above_base:
            raise ValueError("The ear must clear the base top and sit below "
                             "the horn's top face")
        if self.horn_top - self.horn_plate_t - self.well_floor <= 0:
            raise ValueError("The horn well must clear the mount's skirt")
        if self.horn_hub_above < 0 or self.horn_collar_h < 0:
            raise ValueError("The hub and the collar stand off the plate")
        if self.collar_bottom < self.case_top + self.case_boss_h:
            raise ValueError("The horn's collar must clear the output boss")
        if not 0 < self.key_depth <= self.horn_plate_t:
            raise ValueError("The key must stay inside the arm plate")
        for arm in self.horn_arms:
            if max(arm.holes) + self.horn_hole_d / 2 >= arm.reach:
                raise ValueError("Each arm's holes must sit on the arm")
            if not self.horn_hole_d / 2 < arm.tip_r <= arm.axis_half_w:
                raise ValueError("The arms narrow toward the tips, past a "
                                 "hole")
            if arm.axis_half_w > self.horn_hub_d / 2:
                raise ValueError("Each arm must meet the hub")
        holes = {(arm.angle, r) for arm in self.horn_arms for r in arm.holes}
        if not set(self.retain_holes) <= holes:
            raise ValueError("Each retaining screw must go into a horn hole")
        wall = (min(r for _, r in self.retain_holes) - self.retain_head_d / 2
                - self.horn_hub_d / 2 - self.hub_clear)
        if wall < self.min_wall:
            raise ValueError("The retaining heads must leave a wall to the "
                             "hub recess")
        if self.skirt_r > self.plate_side / 2:
            raise ValueError("The skirt must stay under the plate, which "
                             "prints it without support")
        if not self.case_bottom < self.ledge_z < self.base_top:
            raise ValueError("The ear ledge must sit inside the base")
        if self.ledge_z + self.shim_room + self.ear_t >= self.case_top:
            raise ValueError("The ears must sit below the case top")
        roof = self.tag_plane - self.head_top - self.head_clear
        if roof < self.min_wall:
            raise ValueError("The plate must leave a roof over the screw head")
        if self.retain_floor >= self.plate_t:
            raise ValueError("The retaining heads must sink into the plate")
        if self.lock_clamp - self.ear_gap < self.ear_clamp:
            raise ValueError("The ear must leave a clamp under the head")
        if self.clip_skirt >= self.plate_t:
            raise ValueError("The clip's skirt must stay on the plate's side")
        if self.clip_window < self.tag_edge + 2 * self.cell:
            raise ValueError("The clip must leave one white cell around the "
                             "black square")
        if self.clip_overlap <= self.paper_clear:
            raise ValueError("The clip's lip must reach over the paper")
        if self.clip_side - self.notch_w < 2 * 2 * self.clip_wall:
            raise ValueError("The clip's corner grips must stay longer than "
                             "their walls")
        if self.insert_r - self.insert_od / 2 < self.well_r + self.min_wall:
            raise ValueError("The insert must sit clear of the horn well")
        ear_well_x = self.ear_w / 2 + self.ear_fit
        if self.insert_r - self.insert_od / 2 < ear_well_x + self.min_wall:
            raise ValueError("The insert must sit clear of the ear well")
        if self.insert_l + self.insert_relief >= self.base_top:
            raise ValueError("The insert hole must stay in the base")
        if self.wire_w >= self.servo_w or self.lead_boot_w >= self.servo_w:
            raise ValueError("The lead slots must leave the pocket's corners")
        wall = (self.ear_hole_dy - self.pilot_d / 2 - self.servo_l / 2
                - self.servo_clear)
        if wall < self.ear_pilot_wall:
            raise ValueError("The ear screw pilots must leave a wall to the "
                             "pocket")
        if self.ear_hole_dy + self.pilot_d / 2 + self.ear_pilot_wall > (
                self.ear_span / 2 + self.ear_fit):
            raise ValueError("The ear screw pilots must sit under the ears")
        if self.groove_depth >= self.board_recess + self.underside_clear:
            raise ValueError("The lead grooves must open into the board "
                             "pocket's wall")
        if (abs(self.bus_dx - self.power_dx)
                < 2 * self.tie_dx(self.wire_w) + self.tie_slot_w):
            raise ValueError("The bus and power ties must not overlap")
        bx, by = self.board_pocket
        for dx in (self.bus_dx, self.power_dx):
            out = abs(dx) + self.tie_dx(self.wire_w) + self.tie_slot_w / 2
            if out > bx / 2 + self.rim - self.min_wall:
                raise ValueError("Each lead tie must stay inside the base")
        back = self.ref_y + max(self.plate_side, by) / 2
        if self.tie_y - self.tie_slot_l / 2 < back + self.tie_web:
            raise ValueError("The ties must sit behind the pad and the board")
        near = math.hypot(self.board_x - bx / 2, self.ref_y - by / 2)
        if near <= self.keep_out_r:
            raise ValueError("The board must sit outside the keep-out circle")
        if self.beam_w >= self.block_side:
            raise ValueError("The beam must run into the cup block")
        if self.arm_r - self.block_side / 2 <= self.skirt_r:
            raise ValueError("The cup block must sit clear of the hub")
        if self.lid_margin >= self.cup_wall:
            raise ValueError("The lid pocket must leave the block's outer "
                             "wall")
        web = self.cup_pitch / math.sqrt(2) - self.cup_bore / 2
        if web - self.pilot_d / 2 < self.min_wall:
            raise ValueError("The lid screw's pilot must sit in the web "
                             "between the four cups")
        if self.plug_clear <= 0 or self.plug_h >= self.lid_t + self.plug_h:
            raise ValueError("The plugs must enter the bores freely")
        if self.arm_top - self.plate_t <= self.head_top + self.head_clear:
            raise ValueError("The arm's hub plate must sit above the horn")
        if self.wing_h > self.base_top or self.wing_t >= self.wing_depth:
            raise ValueError("The wings must stay below the base top")
        width = self.x_max - self.x_min + 2 * self.wing_reach
        depth = self.y_max - self.y_min
        if max(width, depth) > BED_X or min(width, depth) > BED_Y:
            raise ValueError("The base exceeds the MK3S 250 x 210 mm bed")
        scene_x = self.board_x + bx / 2 + self.sweep_r
        scene_y = self.ref_y + self.plate_side / 2 + self.sweep_r
        fw, fh = self.frame
        if max(scene_x, scene_y) > fw or min(scene_x, scene_y) > fh:
            raise ValueError("Both tags and the board must fit the frame")


def box(x, y, z, w, l, h):
    return Pos(x, y, z) * Box(w, l, h,
                              align=(Align.CENTER, Align.CENTER, Align.MIN))


def cyl(x, y, z, d, h):
    return Pos(x, y, z) * Cylinder(
        d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def horn(s: Spec, grow=0.0):
    """The horn's plan: the hub and four arms with round tips."""
    plan = Circle(s.horn_hub_d / 2)
    for arm in s.horn_arms:
        x, tip, root = arm.reach - arm.tip_r, arm.tip_r, arm.axis_half_w
        plan += Rot(0, 0, arm.angle) * (
            Polygon((0, -root), (x, -tip), (x, tip), (0, root), align=None)
            + Pos(x, 0) * Circle(tip))
    return offset(plan, grow) if grow else plan


def horn_plate(s: Spec) -> Part:
    """The arm plate, the hub through it included."""
    return Pos(0, 0, s.horn_top - s.horn_plate_t) * extrude(horn(s),
                                                            s.horn_plate_t)


def horn_seat(p: Part, s: Spec, top: float) -> Part:
    """Cut the key, the hub recess, the head counterbore and the
    retaining holes into a part standing on the horn."""
    z = s.horn_top
    p -= Pos(0, 0, z - s.key_depth - EPS) * extrude(
        horn(s, s.key_fit), s.key_depth + EPS)
    p -= cyl(0, 0, z - EPS, s.horn_hub_d + 2 * s.hub_clear,
             s.hub_top - z + s.head_clear + EPS)
    p -= cyl(0, 0, z - EPS, s.head_d + 2 * s.hub_clear,
             s.head_top - z + s.head_clear + EPS)
    if s.retain_screws:
        for angle, r in s.retain_holes:
            p -= Rot(0, 0, angle) * cyl(
                r, 0, z - EPS, s.retain_d + 2 * s.retain_clear,
                top - z + 2 * EPS)
            p -= Rot(0, 0, angle) * cyl(
                r, 0, z + s.retain_floor, s.retain_head_d,
                top - z - s.retain_floor + EPS)
    return p


def paper_pocket(s: Spec, x, y, top):
    return box(x, y, top - s.paper_pocket, s.paper_side, s.paper_side,
               s.paper_pocket + EPS)


def build_mount(s: Spec = Spec()) -> Part:
    s.check()
    z, top = s.horn_top, s.tag_plane
    p = box(0, 0, z, s.plate_side, s.plate_side, s.plate_t)
    p += cyl(0, 0, z - s.key_depth, 2 * s.skirt_r, s.key_depth + EPS)
    p += box(s.ear_tip / 2, 0, s.ear_bottom, s.ear_tip, s.lock_w,
             top - s.ear_bottom)
    p = horn_seat(p, s, top)
    slot = 2 * s.slot_half
    p -= Pos(s.insert_r, 0, s.ear_bottom - EPS) * extrude(
        SlotCenterToCenter(slot, M3_HOLE, rotation=90),
        top - s.ear_bottom + 2 * EPS)
    p -= Pos(s.insert_r, 0, top - s.ear_cbore) * extrude(
        SlotCenterToCenter(slot, BTN_CBORE_D, rotation=90),
        s.ear_cbore + EPS)
    p.label = "servorig_mount"
    return p


def cup_centers(s: Spec):
    h = s.cup_pitch / 2
    return [(x, -s.arm_r + y) for x in (-h, h) for y in (-h, h)]


def block(s: Spec, side, r, z, h):
    return Pos(0, -s.arm_r, z) * extrude(RectangleRounded(side, side, r), h)


def build_arm(s: Spec = Spec()) -> Part:
    s.check()
    z, top, bottom = s.horn_top, s.arm_top, s.ear_bottom
    plate_z = top - s.plate_t
    p = box(0, 0, plate_z, s.plate_side, s.plate_side, s.plate_t)
    p += cyl(0, 0, z - s.key_depth, 2 * s.skirt_r,
             plate_z - z + s.key_depth + EPS)
    p += box(0, -s.arm_r / 2, bottom, s.beam_w, s.arm_r, top - bottom)
    p += block(s, s.block_side, s.cup_bore / 2 + s.cup_wall, bottom,
               top - bottom)
    p = horn_seat(p, s, top)
    # the cups open toward the base: their floors print on the bed, top
    # down, instead of bridging the bore
    for x, y in cup_centers(s):
        p -= cyl(x, y, bottom - EPS, s.cup_bore,
                 top - bottom - s.cup_floor + EPS)
    p -= block(s, s.lid_pocket, s.cup_bore / 2 + s.lid_margin, bottom - EPS,
               s.lid_t + EPS)
    p -= cyl(0, -s.arm_r, bottom - EPS, s.pilot_d,
             s.lid_t + s.pilot_depth + EPS)
    p.label = "servorig_arm"
    return p


def build_lid(s: Spec = Spec()) -> Part:
    """The coin lid, in place on the arm."""
    s.check()
    z = s.ear_bottom
    p = block(s, s.lid_side, s.cup_bore / 2 + s.lid_margin - s.lid_fit, z,
              s.lid_t)
    for x, y in cup_centers(s):
        p += cyl(x, y, z + s.lid_t - EPS, s.cup_bore - 2 * s.plug_clear,
                 s.plug_h + EPS)
    p -= cyl(0, -s.arm_r, z - EPS, s.lid_hole_d, s.lid_t + 2 * EPS)
    p.label = "servorig_lid"
    return p


def build_clip(s: Spec = Spec()) -> Part:
    s.check()
    top = s.tag_plane
    inner = s.plate_side + 2 * s.clip_fit
    p = Pos(0, 0, top) * extrude(
        Rectangle(s.clip_side, s.clip_side)
        - Rectangle(s.clip_window, s.clip_window), s.clip_lip_t)
    p += Pos(0, 0, top - s.clip_skirt) * extrude(
        Rectangle(s.clip_side, s.clip_side) - Rectangle(inner, inner),
        s.clip_skirt + EPS)
    edge = (s.clip_side + inner) / 4
    for a in (0, 90, 180, 270):
        p -= Rot(0, 0, a) * box(edge, 0, top - s.clip_skirt - EPS,
                                s.clip_side - inner + EPS, s.notch_w,
                                s.clip_skirt + EPS)
    p.label = "servorig_clip"
    return p


def build_fit(s: Spec = Spec()) -> Part:
    """A test print: the key pocket at each trial fit, labeled, and a
    short servo pocket with its ear ledge and lead slot."""
    s.check()
    plan = horn(s, max(s.fit_trials)).bounding_box()
    pitch = plan.size.Y + s.fit_web
    label_x = plan.max.X + s.fit_margin + s.label_h / 2
    x0 = plan.min.X - s.fit_margin
    x1 = label_x + s.label_h / 2 + s.fit_margin
    y0 = plan.min.Y - s.fit_margin
    y1 = plan.max.Y + (len(s.fit_trials) - 1) * pitch + s.fit_margin
    t = s.key_depth + s.fit_floor
    p = box((x0 + x1) / 2, (y0 + y1) / 2, 0, x1 - x0, y1 - y0, t)
    for i, fit in enumerate(s.fit_trials):
        y = i * pitch
        p -= Pos(0, y, t - s.key_depth) * extrude(horn(s, fit),
                                                  s.key_depth + EPS)
        # the hub and the screw head pass through
        p -= cyl(0, y, -EPS, s.horn_hub_d + 2 * s.hub_clear, t + 2 * EPS)
        p -= Pos(label_x, y, t - s.label_depth) * extrude(
            Rot(0, 0, 90) * Text(f"{fit:.2f}", font_size=s.label_h),
            s.label_depth + EPS)
    ear_w, ear_l = s.ear_w + 2 * s.ear_fit, s.ear_span + 2 * s.ear_fit
    ear_depth = s.ear_t + s.shim_room
    h = s.fit_body_h + ear_depth
    cx = x1 - EPS + ear_w / 2 + s.fit_wall
    cy = (y0 + y1) / 2
    p += box(cx, cy, 0, ear_w + 2 * s.fit_wall, ear_l + 2 * s.fit_wall, h)
    p -= box(cx, cy, -EPS, s.servo_w + 2 * s.servo_clear,
             s.servo_l + 2 * s.servo_clear, h + 2 * EPS)
    p -= box(cx, cy, s.fit_body_h, ear_w, ear_l, ear_depth + EPS)
    end = cy + s.servo_l / 2 + s.servo_clear
    p -= box(cx, end + (s.lead_slot_depth - EPS) / 2, -EPS, s.lead_boot_w,
             s.lead_slot_depth + EPS, s.fit_body_h + EPS)
    p = ear_pilots(p, s, cx, cy, s.fit_body_h)
    p.label = "servorig_fit"
    return p


def ear_pilots(p: Part, s: Spec, x, y, ledge) -> Part:
    """Two pilots down from the ear ledge, under the servo's ear holes,
    for the M2 screws that hold the servo."""
    for sy in (-1, 1):
        p -= cyl(x, y + sy * s.ear_hole_dy, ledge - s.pilot_depth, s.pilot_d,
                 s.pilot_depth + EPS)
    return p


def tie(p: Part, s: Spec, x, band_z, lead_w) -> Part:
    """Two zip tie slots through the base either side of a lead at x, and
    the band's groove across the face opposite the lead."""
    dx = s.tie_dx(lead_w)
    for sx in (-1, 1):
        p -= box(x + sx * dx, s.tie_y, -EPS, s.tie_slot_w,
                 s.tie_slot_l, s.base_top + 2 * EPS)
    span = 2 * dx + s.tie_slot_w
    z = -EPS if band_z == 0 else band_z - s.tie_band_depth
    return p - box(x, s.tie_y, z, span, s.tie_slot_l,
                   s.tie_band_depth + EPS)


def build_base(s: Spec = Spec()) -> Part:
    s.check()
    top = s.base_top
    p = box((s.x_min + s.x_max) / 2, (s.y_min + s.y_max) / 2, 0,
            s.x_max - s.x_min, s.y_max - s.y_min, top)
    for sx, x in ((-1, s.x_min), (1, s.x_max)):
        wx = x + sx * s.wing_reach / 2
        p += box(wx, s.y_min + s.wing_depth / 2, 0, s.wing_reach + EPS,
                 s.wing_depth, s.wing_t)
        p += box(wx, s.y_min + s.wing_t / 2, 0, s.wing_reach + EPS,
                 s.wing_t, s.wing_h)
    p += box(0, s.ref_y, top - EPS, s.plate_side, s.plate_side,
             s.tag_plane - top + EPS)
    p -= paper_pocket(s, 0, s.ref_y, s.tag_plane)
    # the servo: body through to the underside, ears on a ledge
    p -= box(0, s.body_y, -EPS, s.servo_w + 2 * s.servo_clear,
             s.servo_l + 2 * s.servo_clear, s.ledge_z + EPS)
    p -= box(0, s.body_y, s.ledge_z, s.ear_w + 2 * s.ear_fit,
             s.ear_span + 2 * s.ear_fit, top - s.ledge_z + EPS)
    p -= cyl(0, 0, s.well_floor, 2 * s.well_r, top - s.well_floor + EPS)
    p -= box(0, s.body_end + (s.lead_slot_depth - EPS) / 2, -EPS,
             s.lead_boot_w, s.lead_slot_depth + EPS, s.ledge_z + EPS)
    p = ear_pilots(p, s, 0, s.body_y, s.ledge_z)
    p -= box(0, (s.body_end + s.y_max) / 2, -EPS, s.servo_lead_w,
             s.y_max - s.body_end + EPS, s.channel_depth + EPS)
    p = tie(p, s, 0, top, s.servo_lead_w)
    insert_depth = s.insert_l + s.insert_relief
    p -= cyl(s.insert_r, 0, top - insert_depth, s.insert_hole_d,
             insert_depth + EPS)
    # the board, recessed flush on its pads and pilots
    bx, by = s.board_pocket
    floor = top - s.board_recess - s.underside_clear
    seat = top - s.board_recess
    p -= Pos(s.board_x, s.ref_y, floor) * extrude(RectangleRounded(
        bx, by, s.board_r + s.board_clear), top - floor + EPS)
    for dx in (-s.hole_dx, s.hole_dx):
        for dy in (-s.hole_dy, s.hole_dy):
            x, y = s.board_x + dx, s.ref_y + dy
            p += cyl(x, y, floor - EPS, 2 * s.pad_r, seat - floor + EPS)
            p -= cyl(x, y, seat - s.pilot_depth, s.pilot_d,
                     s.pilot_depth + EPS)
    back = s.ref_y + by / 2
    for dx in (s.bus_dx, s.power_dx):
        x = s.board_x + dx
        p -= box(x, (back + s.y_max) / 2, top - s.groove_depth,
                 s.wire_w, s.y_max - back + 2 * EPS, s.groove_depth + EPS)
        p = tie(p, s, x, 0, s.wire_w)
    p.label = "servorig_base"
    return p


def build_servo(s: Spec = Spec()) -> Part:
    """The seated servo, its horn and the horn screw as envelopes, for
    the checks."""
    p = box(0, s.body_y, s.case_bottom, s.servo_w, s.servo_l, s.body_h)
    p += box(0, s.body_y, s.ledge_z + s.shim_room, s.ear_w, s.ear_span,
             s.ear_t)
    p += cyl(0, 0, s.case_top, s.case_boss_d, s.case_boss_h)
    p += cyl(0, 0, s.collar_bottom, s.horn_hub_d,
             s.hub_top - s.collar_bottom)
    p += horn_plate(s)
    p += cyl(0, 0, s.hub_top, s.head_d, s.head_h)
    p.label = "servorig_servo"
    return p
