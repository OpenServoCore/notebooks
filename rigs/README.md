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
uv run --group cad python -m rigs.servorig --preview
uv run --group cad python -m rigs.servorig --horn-arm-root 5.2 --head-h 1.6   # the servo at hand
```

- [motorrig](motorrig/__init__.py): the bare-motor rig for stage 2. A base
  that seats the motor can with its leads under it and carries a ring of
  twelve pin holes, and a slit wrench bar that clamps the rotor's pinion with
  two screws and captured nuts and pins to the ring at one of the twelve
  angles.
- [servorig](servorig/__init__.py): the servo rig for stage 3. A base that
  seats one MG90 on its ears with the horn just above the base top, and
  carries the reference tag, the board and the stall lock's nut outside the
  circle the horn sweeps, three mounts keyed to the four-arm horn (a camera
  mount for the horn's tag, a stall mount whose long arm bolts the horn to
  the base, and a load mount with a screwed-on block of four coin cups), a
  two-piece cap that carries a tag paper flat onto the camera mount, the
  load mount or the pad,
  and a fit test print that tries
  the key and the servo pocket first. What to measure, the print and the
  assembly are in its [README](servorig/README.md).
