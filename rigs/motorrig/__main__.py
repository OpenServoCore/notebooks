"""Build every part: uv run --group cad python -m rigs.motorrig [--pinion-d D ...]"""
import argparse
from dataclasses import fields, replace
from pathlib import Path

from build123d import Pos, Rot, export_step, export_stl

from .rig import Spec, build_bar, build_base, build_cap, build_coupon

MOTOR_FIELDS = ("can_d", "can_flat", "can_l", "front_boss_l", "shaft_l", "pinion_d", "pinion_h", "term_span")
PRINT = {"motorrig_base": None, "motorrig_bar": None, "motorrig_cap": None, "motorrig_coupon": Rot(180, 0, 0)}   # the bar and the cap stand on their undersides


def print_pose(part):
    """The part rotated to its print pose and dropped onto the bed."""
    rot = PRINT[part.label]
    q = rot * part if rot else part
    bb = q.bounding_box()
    return Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * q


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
    light = np.array([0.4, -0.6, 0.7]); light /= np.linalg.norm(light)
    for i, (el, az) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        shade = 0.25 + 0.75 * np.clip(mesh.face_normals @ light, 0, 1)
        ax.add_collection3d(Poly3DCollection(mesh.vertices[mesh.faces], facecolors=plt.cm.Greys(0.25 + 0.6 * (1 - shade)),
                                             edgecolor="none"))
        lo, hi = mesh.bounds; c, r = (lo + hi) / 2, (hi - lo).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az); ax.set_axis_off(); ax.set_box_aspect((1, 1, 1))
    plt.tight_layout(); plt.savefig(png, dpi=110, facecolor="white"); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for f in fields(Spec):
        if f.name in MOTOR_FIELDS or f.name in ("bore_clear", "can_clear", "bolt_r"):
            parser.add_argument("--" + f.name.replace("_", "-"), type=float, default=getattr(Spec, f.name))
    parser.add_argument("--preview", action="store_true", help="also write a PNG of each part")
    args = parser.parse_args()
    spec = replace(Spec(), **{k: v for k, v in vars(args).items() if k != "preview"})
    out = Path(__file__).parent / "build"
    out.mkdir(exist_ok=True)
    for build in (build_base, build_bar, build_cap, build_coupon):
        part = build(spec)
        if not part.is_valid or len(part.solids()) != 1:
            raise RuntimeError(f"{part.label} must be one valid solid")
        export_step(part, str(out / f"{part.label}.step"))
        posed = print_pose(part)
        export_stl(posed, str(out / f"{part.label}.stl"), tolerance=0.01)
        bb = part.bounding_box()
        print(f"{part.label}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {part.volume / 1000:.1f} cm^3 -> {out}")
        if args.preview:
            preview(str(out / f"{part.label}.stl"), str(out / f"{part.label}.png"))
    print(f"bore {spec.bore_d:.2f} over a {spec.pinion_d:.2f} pinion, socket {spec.socket_depth:.1f} deep, "
          f"pinion top {spec.pinion_top:.1f} above the base")


if __name__ == "__main__":
    main()
