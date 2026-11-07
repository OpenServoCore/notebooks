"""Build: uv run --group cad python -m rigs.servorig [--head-h H ...]"""
import argparse
import math
from dataclasses import fields, replace
from pathlib import Path

from build123d import (CenterOf, Compound, Pos, Rot, export_step,
                       export_stl)

from .rig import (BED_X, BED_Y, BED_Z, FITS, MEASURE, Spec, box, build_base,
                  build_block, build_camera_mount, build_cap_base,
                  build_cap_top, build_fit, build_hole_fit, build_lid,
                  build_load_mount, build_servo, build_stall_mount, cyl,
                  horn_plate, nut_pocket)

TOUCH = 1e-3                # mm^3: faces that only meet
DROP = 0.01                 # mm: a seat lowered this far meets its datum
CONTACT = 1.0               # mm^2: the least seat an arm must carry
MOUNT_STEP = 15             # degrees between camera mount poses, a full turn
ARM_STEP = 10               # degrees between load mount poses over its travel
STALL_PLAY = 5              # degrees either side of the lock position
PLA_G_PER_CM3 = 1.24                # solid; infill prints lighter
BUTTON_HEAD = (5.7, 1.65)           # ISO 7380 M3 button head: d, height
M3_NUT = (5.5, 2.4)                 # across flats, thickness
TIE_BAND = (2.5, 1.0)               # zip tie band: width, thickness
TOP_DOWN = Rot(180, 0, 0)
PRINT = {"servorig_fit": None, "servorig_holes": None, "servorig_base": None,
         "servorig_camera_mount": TOP_DOWN,
         "servorig_stall_mount": TOP_DOWN, "servorig_load_mount": TOP_DOWN,
         "servorig_block": None, "servorig_lid": TOP_DOWN,
         "servorig_cap_base": None,
         "servorig_cap_top": TOP_DOWN}


def print_pose(part):
    """The part turned to its print pose and dropped onto the bed."""
    rot = PRINT[part.label]
    q = rot * part if rot else part
    bb = q.bounding_box()
    return Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2,
               -bb.min.Z) * q


def preview(stl, png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import trimesh
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    mesh = trimesh.load(stl)
    views = ((35, -50), (-35, 130))
    fig = plt.figure(figsize=(6 * len(views), 6))
    light = np.array([0.4, -0.6, 0.7])
    light /= np.linalg.norm(light)
    for i, (el, az) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        shade = 0.25 + 0.75 * np.clip(mesh.face_normals @ light, 0, 1)
        ax.add_collection3d(Poly3DCollection(
            mesh.vertices[mesh.faces],
            facecolors=plt.cm.Greys(0.25 + 0.6 * (1 - shade)),
            edgecolor="none"))
        lo, hi = mesh.bounds
        c, r = (lo + hi) / 2, (hi - lo).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)
        ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az)
        ax.set_axis_off()
        ax.set_box_aspect((1, 1, 1))
    plt.tight_layout()
    plt.savefig(png, dpi=110, facecolor="white")
    plt.close(fig)


def overlap(a, b):
    return (a & b).volume


def seat_areas(s, part, plate):
    """The part's contact on each arm's top face, out past the hub."""
    low = Pos(0, 0, -DROP) * part
    hub_r = s.horn_hub_d / 2
    areas = []
    for a in s.horn_arms:
        zone = Rot(0, 0, a.angle) * box(
            (hub_r + a.reach) / 2, 0, s.horn_top - s.horn_plate_t,
            a.reach - hub_r, s.horn_hub_d, s.horn_plate_t)
        areas.append(overlap(low, plate & zone) / DROP)
    return areas, overlap(low, build_servo(s)) - overlap(low, plate)


def verify(s, fit, base, mount, stall, arm, block, lid, cap_base, cap_top):
    """Every geometric check; returns report lines, raises on a failure."""
    servo = build_servo(s)
    ribbed = cap_top
    cap_top = build_cap_top(s, ribs=False)
    cap = cap_base + cap_top
    on_pad = Pos(0, s.ref_y, 0) * cap
    loaded = arm + block + lid + cap
    tall = s.block_top + 50.0
    lines = []

    def need(ok, what):
        if not ok:
            raise RuntimeError(f"check failed: {what}")
        lines.append(f"  ok  {what}")

    above = cyl(0, 0, s.base_top + TOUCH, 2 * s.keep_out_r, tall)
    need(overlap(base, above) < TOUCH,
         f"nothing of the base rises inside r {s.keep_out_r:.1f}")
    outside = (mount + cap) - cyl(0, 0, 0, 2 * (s.sweep_r + TOUCH), tall)
    need(outside.volume < TOUCH,
         f"camera mount and cap stay inside the swept r {s.sweep_r:.1f}")
    worst = max(max(overlap(Rot(0, 0, a) * mount, base),
                    overlap(Rot(0, 0, a) * cap, base))
                for a in range(0, 360, MOUNT_STEP))
    need(worst < TOUCH,
         "camera mount and cap clear the base over a full turn")
    worst = max(overlap(Rot(0, 0, a) * stall, base)
                for a in range(-STALL_PLAY, STALL_PLAY + 1))
    need(worst < TOUCH, f"stall mount clears the base within "
                        f"+/-{STALL_PLAY} deg of the lock position")
    head = cyl(s.stall_r, 0, s.stall_seat, *BUTTON_HEAD)
    shank = cyl(s.stall_r, 0, s.stall_end, 3.0, s.stall_l)
    need(overlap(head, stall) < TOUCH and overlap(shank, stall) > TOUCH
         and overlap(shank, base) > TOUCH,
         "stall screw's head sits on the ear, its thread bites the ear and "
         "the base")
    if s.retain_screws:
        pins = [Rot(0, 0, a) * cyl(r, 0, s.horn_top, s.retain_d,
                                   s.retain_floor)
                for a, r in s.retain_holes]
        need(min(overlap(p, stall) for p in pins) > TOUCH
             and max(overlap(p, mount) for p in pins) < TOUCH,
             "retaining screws thread into the stall mount and pass the "
             "camera mount free")
    down = [f for f in base.faces()
            if f.normal_at().Z < -0.99 and f.center().Z > TOUCH]
    need(not down, "nothing on the base's underside overhangs the bed")
    nut = nut_pocket(s.stall_r, 0, s.stall_nut_top - M3_NUT[1],
                     M3_NUT[0] / 2, M3_NUT[1])
    need(overlap(nut, base) < TOUCH
         and overlap(Pos(0, 0, 0.1) * nut, base) > TOUCH,
         f"M3 nut seats under the pocket's roof, the screw "
         f"{s.stall_tip_past:.1f} past it and "
         f"{s.stall_end:.2f} over the underside")
    ties = [(0, s.servo_lead_w)] + [(s.board_x + dx, s.wire_w)
                                    for dx in (s.bus_dx, s.power_dx)]
    bands = [box(x, s.tie_y, 0, 2 * s.tie_dx(w) + s.tie_slot_w, TIE_BAND[0],
                 TIE_BAND[1]) for x, w in ties]
    need(max(overlap(b, base) for b in bands) < TOUCH,
         "zip tie bands lie in their grooves under the base")
    bx, by = s.board_pocket
    board = box(s.board_x, s.ref_y, s.base_top, bx, by, tall)
    half = s.travel_deg / 2 + s.travel_margin
    angles = range(-round(half), round(half) + 1, ARM_STEP)
    worst = max(max(overlap(Rot(0, 0, a) * loaded, base),
                    overlap(Rot(0, 0, a) * loaded, board)) for a in angles)
    need(worst < TOUCH, f"loaded mount, block, lid and cap clear the base "
                        f"and the board over +/-{half:.0f} deg")
    need(overlap(lid, block) < TOUCH and len((lid + block).solids()) == 1,
         "lid sits in the block's pocket, plugs in the bores")
    seat = overlap(Pos(0, 0, -DROP) * block, arm) / DROP
    need(overlap(block, arm) < TOUCH and seat > CONTACT,
         f"block sits on the beam: {seat:.0f} mm^2")
    tip = s.block_bottom + s.flange_t - s.joint_l
    need(s.beam_bottom - 1e-6 <= tip <= s.plate_top - s.nut_roof - s.nut_t,
         f"joint screws M2.5 x {s.joint_l:.0f} pass their nuts, tips "
         f"{tip - s.beam_bottom:.2f} inside the beam, "
         f"{s.beam_bottom - s.base_top:.2f} over the base")
    need(overlap(servo, base) < TOUCH, "servo seats in its pocket")
    need(max(overlap(servo, p) for p in (mount, stall, arm)) < TOUCH,
         "camera, stall and load mounts sit on the horn without touching "
         "the rest")
    plate = horn_plate(s)
    for part in (mount, stall, arm):
        areas, rest = seat_areas(s, part, plate)
        need(min(areas) > CONTACT and rest < TOUCH,
             f"{part.label} seats on the plate's top face alone, on all "
             f"four arms: {' / '.join(f'{a:.0f}' for a in areas)} mm^2")
    need(overlap(cap_base, cap_top) < TOUCH,
         "cap base sits in the cap top's skirt")
    need(overlap(ribbed, cap_base) < TOUCH and overlap(ribbed, mount) > TOUCH,
         f"ribs bite {s.rib_squeeze:.2f} per side into the plate and clear "
         f"the cap base")
    need(max(overlap(cap, mount), overlap(on_pad, base),
             overlap(cap, arm), overlap(cap, block)) < TOUCH,
         "cap goes over the camera mount, the pad and the load mount's hub")
    low = Pos(0, 0, -DROP) * cap_base
    seats = (overlap(low, mount) / DROP,
             overlap(Pos(0, s.ref_y, 0) * low, base) / DROP,
             overlap(low, arm) / DROP)
    need(min(seats) > CONTACT,
         f"cap base seats on the camera mount, the pad and the load mount: "
         f"{' / '.join(f'{a:.0f}' for a in seats)} mm^2")
    for part in (fit, build_hole_fit(s), base, mount, stall, arm, block, lid,
                 cap_base, cap_top):
        bb = print_pose(part).bounding_box()
        plan = sorted((bb.size.X, bb.size.Y), reverse=True)
        need(plan[0] <= BED_X and plan[1] <= BED_Y and bb.size.Z <= BED_Z,
             f"{part.label} fits the bed: {bb.size.X:.1f} x {bb.size.Y:.1f}"
             f" x {bb.size.Z:.1f}")
    return lines


def summary(s):
    longest = s.stall_seat
    shortest = s.stall_seat - (s.stall_nut_top - M3_NUT[1])
    arms = " / ".join(f"{a.reach:.2f}" for a in s.horn_arms)
    tips = " / ".join(f"{2 * a.tip_r:.2f}" for a in s.horn_arms)
    retain = " and ".join(f"r {r:.1f} at {a:.0f} deg"
                          for a, r in s.retain_holes)
    return [
        f"horn arms at 0 / 90 / 180 / 270 deg reach {arms}, {tips} wide at "
        f"the tips; hub {s.horn_hub_d:.2f} standing "
        f"{s.hub_top - s.horn_top:.2f} over a {s.horn_plate_t:.2f} plate, "
        f"head top {s.head_top - s.horn_top:.2f} over the plate; collar "
        f"{s.collar_bottom - s.case_top - s.case_boss_h:.2f} over the boss",
        f"key {s.key_depth:.2f} into the plate; retaining screws "
        + (f"at {retain}" if s.retain_screws else "left out"),
        f"base top {s.base_top:.2f}, horn top {s.horn_top:.2f}, tag plane "
        f"{s.tag_plane:.2f} on a cap seated at {s.plate_top:.2f} (cap top "
        f"{s.plate_top + s.cap_t + s.cap_lip_t:.2f})",
        f"ledge {s.ledge_z:.2f} with {s.shim_room:.1f} of shim under the "
        f"ears; case bottom {s.case_bottom:.2f} over the underside",
        f"tag {s.tag_edge:.1f}, quiet {s.quiet:.2f}, paper {s.paper:.2f}, "
        f"plate {s.plate_side:.2f}, cap {s.cap_side:.2f} window "
        f"{s.cap_window:.2f}",
        f"skirt r {s.skirt_r:.2f}, well r {s.well_r:.2f}, swept r "
        f"{s.sweep_r:.2f}, keep-out r {s.keep_out_r:.2f}",
        f"stall mount to r {s.stall_tip:.2f}, {s.stall_w:.2f} wide at the "
        f"screw; M3 x {s.stall_l:.0f} button head at r {s.stall_r:.1f} over "
        f"one washer into an M3 nut seated at {s.stall_nut_top:.2f} "
        f"(fits {shortest:.1f} to {longest:.1f}); 0.1 of play is "
        f"{math.degrees(0.1 / s.stall_r):.2f} deg",
        f"reference pad at y {s.ref_y:.2f}, tags {s.ref_y:.1f} apart; board "
        f"at ({s.board_x:.2f}, {s.ref_y:.2f}), four M2 into "
        f"{s.pilot_depth:.1f} pilots under a {s.board_t:.1f} board",
        f"load mount: 4 x {s.stack_n} coins = {4 * s.stack_g:.0f} g at "
        f"{s.arm_r:.0f} mm, {4 * s.stack_g * 9.81 * s.arm_r / 1e3:.0f} mN m "
        f"level; one stack {s.stack_g:.1f} g gives "
        + " / ".join(f"{s.stack_g * 9.81 * r / 1e3:.0f}"
                     for r in s.stack_radii())
        + f" mN m at r {' / '.join(f'{r:.1f}' for r in s.stack_radii())}; "
        f"reach {s.arm_reach:.1f}, block {s.block_bottom:.2f} to "
        f"{s.block_top:.2f}; joint at r {s.joint_r:.2f}, beam underside "
        f"{s.beam_bottom:.2f}",
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for f in fields(Spec):
        if f.name in MEASURE or f.name in FITS or f.name == "horn_above_base":
            parser.add_argument("--" + f.name.replace("_", "-"), type=float,
                                default=getattr(Spec, f.name))
    parser.add_argument("--no-retain-screws", action="store_true",
                        help="leave out the two retaining screw holes")
    parser.add_argument("--preview", action="store_true",
                        help="also write a PNG of each part")
    parser.add_argument("--assembly", action="store_true",
                        help="also write the assembly STEP")
    args = parser.parse_args()
    measured = {k: v for k, v in vars(args).items()
                if k not in ("no_retain_screws", "preview", "assembly")}
    spec = replace(Spec(), retain_screws=not args.no_retain_screws,
                   **measured)
    out = Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    parts = [build_fit(spec), build_hole_fit(spec), build_base(spec),
             build_camera_mount(spec), build_stall_mount(spec),
             build_load_mount(spec), build_block(spec), build_lid(spec),
             build_cap_base(spec),
             build_cap_top(spec)]
    for part in parts:
        if not part.is_valid or len(part.solids()) != 1:
            raise RuntimeError(f"{part.label} must be one valid solid")
        export_step(part, str(out / f"{part.label}.step"))
        export_stl(print_pose(part), str(out / f"{part.label}.stl"),
                   tolerance=0.01)
        bb = part.bounding_box()
        print(f"{part.label}: {bb.size.X:.1f} x {bb.size.Y:.1f} x "
              f"{bb.size.Z:.1f} mm, {part.volume / 1000:.1f} cm^3")
        if args.preview:
            preview(str(out / f"{part.label}.stl"),
                    str(out / f"{part.label}.png"))
    fit, holes, base, mount, stall, arm, block, lid, cap_base, cap_top = parts
    loaded = arm + block + lid
    arm_g = loaded.volume / 1000 * PLA_G_PER_CM3
    arm_y = loaded.center(CenterOf.MASS).Y
    print(f"load mount, block and lid, printed solid: {arm_g:.0f} g with the mass "
          f"center {-arm_y:.1f} mm out, {arm_g * 9.81 * -arm_y / 1e3:.0f} "
          f"mN m level on top of the coins; weigh the print")
    if args.assembly:
        cap = [cap_base, cap_top]
        on_pad = [Pos(0, spec.ref_y, 0) * c for c in cap]
        export_step(Compound([base, build_servo(spec), mount, *cap,
                              *on_pad]),
                    str(out / "servorig_assembly.step"))
    print("\n".join(summary(spec)))
    print("checks:")
    print("\n".join(verify(spec, fit, base, mount, stall, arm, block, lid,
                            cap_base, cap_top)))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
