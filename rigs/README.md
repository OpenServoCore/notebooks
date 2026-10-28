# Rigs

The printed fixtures the experiments run on, as parametric CAD in
[build123d](https://github.com/gumyr/build123d), one folder per rig. Each
folder's docstring is its design story, its `Spec` holds every dimension with
the motor or board it was measured from, and `python -m rigs.<rig>` writes
STEP, STL and a preview of every part to the rig's `build/` folder, which is
not committed. The STLs are exported in their print pose, so they slice as
they are.

```sh
uv sync --group cad
uv run --group cad python -m rigs.motorrig --preview
uv run --group cad python -m rigs.motorrig --pinion-d 2.3 --can-l 12.0   # the motor at hand
```

- [motorrig](motorrig/__init__.py): the bare-motor rig for stage 2. A base
  that seats the motor can with its leads under it, a wrench bar and cap
  that pinch the rotor's pinion between two rigid halves and pin the bar at
  one of twelve indexed angles, and a fit coupon that reads the printer's
  bore against the pinion in hand.
