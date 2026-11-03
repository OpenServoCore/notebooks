# servorig

The printed rig for the stage 3 camera experiments: one MG90, one
osc-dev-v006 board and two camera tags in one rigid piece. The design story,
the frame and the reasons behind each part live in the package docstring,
[`__init__.py`](__init__.py). This page covers what to measure, the build,
the print, the hardware, the assembly and the acceptance.

There are four printed parts, the base, the tag mount, the load arm and the
tag clip. The rest is the stock cross horn and a few screws.

## Measure before printing

The servo's heights and the cross horn are not measured yet. Every one of
these is a guess from the SG90 measurements or a stock 9 g horn, so measure
the servo at hand with calipers first and pass the numbers on the command
line.

| Flag | What to measure | Default |
|---|---|---|
| `--body-h` | case bottom to case top, without the output boss | 22.45 |
| `--ear-z` | case bottom to the underside of the mounting ears | 13.41 |
| `--ear-span` / `--ear-t` / `--ear-w` | both ears end to end, ear thickness, ear width | 32.12 / 2.47 / 12.2 |
| `--shaft-offset` | case center to the output axis, along the case | 6.0 |
| `--horn-above-body` | case top to the cross horn's top face, horn screwed on | 8.0 |
| `--horn-reach` | axis to the tip of the longest arm | 10.0 |
| `--horn-arm-root` / `--horn-arm-tip` | arm width at the hub and at the tip | 5.5 / 3.6 |
| `--horn-arm-t` | arm thickness | 1.8 |
| `--horn-hub-d` | hub diameter | 7.0 |
| `--horn-hole-r` | axis to the outermost arm hole | 8.5 |
| `--head-d` / `--head-h` | M2 center screw head, diameter and height | 3.8 / 1.4 |

`--horn-above-base` (1.5) moves the horn up or down in the 1 to 2 mm band.
`--no-retain-screws` leaves out the two optional retaining screw holes.

## Build

Run from the repo root:

```sh
uv sync --group cad
uv run --group cad python -m rigs.servorig      # writes rigs/servorig/build/
uv run --group cad python -m rigs.servorig --horn-reach 10.4 --horn-arm-t 1.6 --preview
```

It writes a STEP of each part in the rig's frame, an STL of each part in its
print pose, `servorig_assembly.step` with the servo envelope and the mount
on it, and with `--preview` a PNG of each part. It also runs the checks and
stops with an error if one fails:

- nothing of the base rises inside the keep-out circle, which is the circle
  the mount and the clip sweep (ear corners included) plus the tag's quiet
  border
- the mount and the clip clear the base over a full turn
- the load arm clears the base and the board over the servo's travel plus
  10 degrees each way
- the servo envelope seats in its pocket, the mount and the arm seat on the
  cross and touch nothing else, the clip goes over the mount
- every part fits the MK3S bed, and both tags and the board fit the phone's
  frame at the working distance

## Print

- **Base** flat, underside on the bed. The underside and the front face
  are the two datums.
- **Tag mount** top down, tag face on the bed. The bed gives the flattest
  tag face, and the seat on the cross prints as a top skin, so no support
  anywhere. The lock screw counterbore and the retaining screw counterbores
  open at the bed and print as short bridges.
- **Load arm** top down, the hub tag and the cup rims on the bed. The cup
  floors print as 25 mm bridges.
- **Tag clip** lip down, its top on the bed.

## Hardware

- the MG90's stock cross horn and its M2 center screw
- one M3 heat-set insert, 5.7 mm long for a 4.0 mm hole
- one M3 x 8 button head screw and one M3 washer (0.5 mm), the lock
- four M2 x 6 screws for the board, into the 1.8 mm pilots
- two small self-tapping screws, about 1.7 x 5 mm, to retain the mount on
  the horn (optional)
- three 2.5 mm zip ties
- 22 US quarters and some tape, for the load arm
- a few strips of 0.5 mm card, for shims under the servo's ears

## Assembly

1. Melt the insert into the base top beside the servo pocket, flush.
2. Screw the board into its pocket, face up, with the bus connectors (J9,
   J10) to the back slots.
3. Center the servo. Push the cross horn onto the spline with its arms as
   square to the base as the spline allows, and screw it down.
4. Lay the card shims on the ear ledge, one strip under each ear. Feed the
   lead down the slot in the pocket's back wall and press the servo in until
   the ears sit on the shims. The lead goes along the channel under the base
   and out the back. Zip tie it at the back.
5. Lay the bus and power leads in their grooves and zip tie them.
6. Glue the reference tag into the pad's pocket and the horn tag into the
   mount's pocket, both square to the base edges.
7. Press the tag mount over the cross, ear pointing at the insert. Add the
   two retaining screws if you use them. Push the clip on.
8. Clamp the base to the desk by its wings, and set the phone above it at
   250 mm.

The ear lands over the insert at whatever angle the spline left the cross
at, within one tooth of mid travel. That angle is the lock position. Command
the servo there before putting the lock screw in (washer under the ear).

For the load arm, swap the mount for the arm, pointing at the front face at
mid travel. Fill the two cups with 11 quarters each and tape over them. Then
stand the base on its front face at the desk edge, horn face out past the
edge, and clamp it by the wings. The arm hangs down at mid travel and swings
out past the edge.

If the horn sits outside the 1 to 2 mm band, change the shims under the
ears. Without shims the horn drops 0.5 mm.

## Acceptance

- Both tags detected in every frame over a full turn.
- Plate tilt under 1 degree for both tags.
- Equal distances from the lens to both tags.
- No frame where a wire or the board crosses a tag.
- STAT flashes found by the clock-sync extraction in the same clip.
- The lock holds a 100 % lap at 3.7 V with the pot reading unchanged.
