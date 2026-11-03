# servorig

The printed rig for the stage 3 camera experiments: one MG90, one
osc-dev-v006 board and two camera tags in one rigid piece. The design story,
the frame and the reasons behind each part live in the package docstring,
[`__init__.py`](__init__.py). This page covers what to measure, the build,
the print, the hardware, the assembly and the acceptance.

There are four printed parts, the base, the tag mount, the load arm and the
tag clip, and a fit test print that goes first. The rest is the SG90 horn #3
with its four unequal arms and a few screws.

## Measure before printing

The horn is already in the Spec as nominal values: the owner's calipers,
rounded, for the arms' reach, the plate and the hub, and the SG90 horn #3
drawing (Aus Electronics Direct TA0132, page 5) for the hole radii and the
long arms' edge tapers.

| Measured | Calipers | Nominal |
|---|---|---|
| tip to tip, the two long arms | 34.14 | 34.0: arms of 16.0 and 18.0 |
| across the two short arms | 16.65 | 16.5: arms of 8.25 |
| arm plate thickness | 1.90 | 2.0 |
| hub, collar underside to hub top | 5.28 | 5.3 |
| spline socket, collar underside to its end | 4.40 | 4.4 |

How far the collar reaches below the plate is not measured. The Spec takes
it flush with the plate's underside, so the hub stands 3.3 above the plate's
top face; a collar that reaches lower leaves the hub lower, and the recess
still clears it.

The rest are guesses, from the SG90 measurements, a photo of the horn or a
stock part, so measure the servo at hand with calipers and pass the numbers
on the command line.

| Flag | What to measure | Default | Source |
|---|---|---|---|
| `--body-h` | case bottom to case top, without the output boss | 22.45 | SG90 donor, calipers (22.7 on the SG90 drawing) |
| `--ear-z` | case bottom to the underside of the mounting ears | 13.41 | SG90 donor, calipers |
| `--ear-span` / `--ear-t` / `--ear-w` | both ears end to end, ear thickness, ear width | 32.12 / 2.47 / 12.2 | SG90 donor, calipers |
| `--shaft-offset` | case center to the output axis, along the case | 6.0 | SG90 pot-to-arbor spacing as designed |
| `--horn-above-body` | case top to the top face of the horn's arm plate, horn screwed on | 6.4 | boss 4.3 and spline 4.5 from the SG90 drawing, socket 4.4 and plate 2.0 from the calipers |
| `--horn-hub-d` | hub diameter, a spot-check | 6.9 | SG90 horn #3 drawing |
| `--horn-arm-root` / `--horn-arm-tip` | a short arm's width where it meets the hub, every arm's width at its round tip | 5.0 / 3.8 | the owner's photo against the hub |
| `--head-d` / `--head-h` | M2 center screw head: diameter, and its top above the hub's top face | 3.8 / 1.4 | a stock M2 head |

The long arms widen from the tip width toward the hub at the drawing's
taper, which makes the 16 mm arm 5.1 wide at the hub and the 18 mm arm 6.3;
the build prints the widths it uses. If the 18 mm arm measures well under
6.3 at the hub, its key slot is loose, and the other three arms locate the
mount.

`--horn-above-base` (1.5) moves the horn up or down in the 1 to 2 mm band.
`--no-retain-screws` leaves out the two optional retaining screw holes.

## Build

Run from the repo root:

```sh
uv sync --group cad
uv run --group cad python -m rigs.servorig      # writes rigs/servorig/build/
uv run --group cad python -m rigs.servorig --horn-arm-root 5.2 --head-h 1.6 --preview
```

It writes a STEP of each part in the rig's frame, an STL of each part in its
print pose, with `--preview` a PNG of each part and with `--assembly`
`servorig_assembly.step`, the servo envelope with the mount on it. It also
runs the checks and stops with an error if one fails:

- nothing of the base rises inside the keep-out circle, which is the circle
  the mount and the clip sweep (ear corners included) plus the tag's quiet
  border
- the mount and the clip clear the base over a full turn
- the load arm clears the base and the board over the servo's travel plus
  10 degrees each way
- the servo envelope seats in its pocket, the mount and the arm sit on the
  horn and touch nothing else, the clip goes over the mount
- the mount and the arm, lowered 0.01 mm, meet only the top face of the
  horn's arm plate, and meet it on all four arms
- every part, the fit test print included, fits the MK3S bed, and both tags
  and the board fit the phone's frame at the working distance

## Print

- **Fit test** first, flat, pockets up, about 11 cm^3 (under 14 g even
  solid). It carries the tag mount's key pocket three times, labeled with
  its fit per side, 0.05, 0.15 and 0.25, and a 10 mm section of the servo
  pocket with the ear ledge and the lead slot, open underneath. Press the
  horn, screw in, into each key pocket, hub down: the hub and the screw head
  pass through the plate, so hold it clear of the desk. Pick the fit that
  presses on by hand and does not rock, and try the servo body and ears in
  the short pocket. The fit you pick goes in as `--key-fit` and
  `--servo-clear` (both 0.05 by default) for every part after it. The servo
  pocket is cut at `--servo-clear`, so print the test again with the new
  value if the body does not go in.
- **Base** flat, underside on the bed. The underside and the front face
  are the two datums.
- **Tag mount** top down, tag face on the bed. The bed gives the flattest
  tag face, and the seat on the horn prints as a top skin, so no support
  anywhere. The lock screw counterbore and the retaining screw counterbores
  open at the bed and print as short bridges.
- **Load arm** top down, the hub tag and the cup rims on the bed. The cup
  floors print as 25 mm bridges.
- **Tag clip** lip down, its top on the bed.

## Hardware

- the SG90 horn #3, four unequal arms, and the servo's M2 center screw
- one M3 heat-set insert, 5.7 mm long for a 4.0 mm hole
- one M3 x 12 button head screw and one M3 washer (0.5 mm), the lock
- four M2 x 6 screws for the board, into the 1.8 mm pilots
- two M1.4 x 4 screws or 1.4 x 4 mm self-tapping screws, into the long
  arms' 1.0 mm holes at 8.5 and 8.0 mm from the axis, to retain the mount
  on the horn (optional)
- three 2.5 mm zip ties
- 22 US quarters and some tape, for the load arm
- a few strips of 0.5 mm card, for shims under the servo's ears

## Assembly

1. Melt the insert into the base top beside the servo pocket, flush.
2. Screw the board into its pocket, face up, with the bus connectors (J9,
   J10) to the back slots.
3. Center the servo. Push the horn onto the spline with its longer arm
   pointing at the insert, as square to the base as the spline allows, and
   screw it down.
4. Lay the card shims on the ear ledge, one strip under each ear. Feed the
   lead down the slot in the pocket's back wall and press the servo in until
   the ears sit on the shims. The lead goes along the channel under the base
   and out the back. Zip tie it at the back.
5. Lay the bus and power leads in their grooves and zip tie them.
6. Glue the reference tag into the pad's pocket and the horn tag into the
   mount's pocket, both square to the base edges.
7. Press the tag mount over the horn, ear over the longer arm; it goes on
   one way only. Add the two retaining screws if you use them. Push the
   clip on.
8. Clamp the base to the desk by its wings, and set the phone above it at
   250 mm.

The ear lands over the insert at whatever angle the spline left the horn
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
