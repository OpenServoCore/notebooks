"""Build: uv run --group cad python -m rigs.servorig [--head-h H ...]"""
import argparse
from dataclasses import fields, replace
from pathlib import Path

from build123d import (CenterOf, Compound, Pos, Rot, export_step,
                       export_stl)

from .rig import (BED_X, BED_Y, BED_Z, FITS, MEASURE, Spec, box, build_arm,
                  build_base, build_clip, build_fit, build_mount, build_servo,
                  cyl, horn_plate)

TOUCH = 1e-3                # mm^3: faces that only meet
DROP = 0.01                 # mm: a seat lowered this far meets its datum
CONTACT = 1.0               # mm^2: the least seat an arm must carry
MOUNT_STEP = 15             # degrees between mount poses over a full turn
ARM_STEP = 10               # degrees between load arm poses over its travel
M3_LENGTHS = (6, 8, 10, 12, 16)     # button heads as sold
PLA_G_PER_CM3 = 1.24                # solid; infill prints lighter
TOP_DOWN = Rot(180, 0, 0)
PRINT = {"servorig_fit": None, "servorig_base": None,
         "servorig_mount": TOP_DOWN, "servorig_arm": TOP_DOWN,
         "servorig_clip": TOP_DOWN}


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


def verify(s, fit, base, mount, arm, clip):
    """Every geometric check; returns report lines, raises on a failure."""
    servo = build_servo(s)
    tall = s.arm_top + 50.0
    lines = []

    def need(ok, what):
        if not ok:
            raise RuntimeError(f"check failed: {what}")
        lines.append(f"  ok  {what}")

    above = cyl(0, 0, s.base_top + TOUCH, 2 * s.keep_out_r, tall)
    need(overlap(base, above) < TOUCH,
         f"nothing of the base rises inside r {s.keep_out_r:.1f}")
    outside = (mount + clip) - cyl(0, 0, 0, 2 * (s.sweep_r + TOUCH), tall)
    need(outside.volume < TOUCH,
         f"mount and clip stay inside the swept r {s.sweep_r:.1f}")
    worst = max(max(overlap(Rot(0, 0, a) * mount, base),
                    overlap(Rot(0, 0, a) * clip, base))
                for a in range(0, 360, MOUNT_STEP))
    need(worst < TOUCH, "mount and clip clear the base over a full turn")
    bx, by = s.board_pocket
    board = box(s.board_x, s.ref_y, s.base_top, bx, by, tall)
    half = s.travel_deg / 2 + s.travel_margin
    angles = range(-round(half), round(half) + 1, ARM_STEP)
    worst = max(max(overlap(Rot(0, 0, a) * arm, base),
                    overlap(Rot(0, 0, a) * arm, board)) for a in angles)
    need(worst < TOUCH, f"load arm clears the base and the board over "
                        f"+/-{half:.0f} deg")
    need(overlap(servo, base) < TOUCH, "servo seats in its pocket")
    need(overlap(servo, mount) < TOUCH and overlap(servo, arm) < TOUCH,
         "mount and arm sit on the horn without touching the rest")
    plate = horn_plate(s)
    for part in (mount, arm):
        areas, rest = seat_areas(s, part, plate)
        need(min(areas) > CONTACT and rest < TOUCH,
             f"{part.label} seats on the plate's top face alone, on all "
             f"four arms: {' / '.join(f'{a:.0f}' for a in areas)} mm^2")
    need(overlap(clip, mount) < TOUCH, "clip goes over the mount's plate")
    for part in (fit, base, mount, arm, clip):
        bb = print_pose(part).bounding_box()
        plan = sorted((bb.size.X, bb.size.Y), reverse=True)
        need(plan[0] <= BED_X and plan[1] <= BED_Y and bb.size.Z <= BED_Z,
             f"{part.label} fits the bed: {bb.size.X:.1f} x {bb.size.Y:.1f}"
             f" x {bb.size.Z:.1f}")
    return lines


def summary(s):
    lock_min = s.lock_clamp + s.insert_l / 2
    lock_max = s.lock_clamp + s.insert_l + s.insert_relief
    lock = min(M3_LENGTHS, key=lambda n: abs(n - (lock_min + lock_max) / 2))
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
        f"{s.tag_plane:.2f} (clip top {s.tag_plane + s.clip_lip_t:.2f})",
        f"ledge {s.ledge_z:.2f} with {s.shim_room:.1f} of shim under the "
        f"ears; case bottom {s.case_bottom:.2f} over the underside",
        f"tag {s.tag_edge:.1f}, quiet {s.quiet:.2f}, paper {s.paper:.2f}, "
        f"plate {s.plate_side:.2f}, clip {s.clip_side:.2f} window "
        f"{s.clip_window:.2f}",
        f"skirt r {s.skirt_r:.2f}, well r {s.well_r:.2f}, ear tip "
        f"{s.ear_tip:.2f} x {s.lock_w:.2f} wide, swept r {s.sweep_r:.2f}, "
        f"keep-out r {s.keep_out_r:.2f}",
        f"insert at r {s.insert_r:.2f}, slot +/-{s.slot_deg:.0f} deg; lock "
        f"M3 x {lock} button head over one washer (fits "
        f"{lock_min:.1f} to {lock_max:.1f})",
        f"reference pad at y {s.ref_y:.2f}, tags {s.ref_y:.1f} apart; board "
        f"at ({s.board_x:.2f}, {s.ref_y:.2f}), four M2 into "
        f"{s.pilot_depth:.1f} pilots under a {s.board_t:.1f} board",
        f"load arm: {s.mass_stacks} x {s.stack_n} coins = "
        f"{s.mass_stacks * s.stack_n * s.unit_g:.0f} g at {s.arm_r:.0f} mm, "
        f"{s.mass_stacks * s.stack_n * s.unit_g * 9.81 * s.arm_r / 1e3:.0f}"
        f" mN m level; reach {s.arm_reach:.1f}, hub tag at "
        f"{s.arm_top:.2f}",
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
    parts = [build_fit(spec), build_base(spec), build_mount(spec),
             build_arm(spec), build_clip(spec)]
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
    fit, base, mount, arm, clip = parts
    arm_g = arm.volume / 1000 * PLA_G_PER_CM3
    arm_y = arm.center(CenterOf.MASS).Y
    print(f"load arm itself, printed solid: {arm_g:.0f} g with its mass "
          f"center {-arm_y:.1f} mm out, {arm_g * 9.81 * -arm_y / 1e3:.0f} "
          f"mN m level on top of the coins; weigh the print")
    if args.assembly:
        export_step(Compound([base, build_servo(spec), mount, clip]),
                    str(out / "servorig_assembly.step"))
    print("\n".join(summary(spec)))
    print("checks:")
    print("\n".join(verify(spec, fit, base, mount, arm, clip)))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
