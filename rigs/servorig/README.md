# servorig

The printed rig for the stage 3 camera experiments: one MG90, one
osc-dev-v006 board and two camera tags in one rigid piece. The design story,
the frame and the reasons behind each part live in the package docstring,
[`__init__.py`](__init__.py). This page covers what to measure, the build,
the print, the hardware, the assembly and the acceptance.

There are five printed parts, the base, the tag mount, the load arm, its
coin lid and the tag clip, and a fit test print that goes first. The rest is the MG90S
four-arm horn and a few screws.

## Measure before printing

The servo and the horn are in the Spec from a STEP model of the Tower Pro
MG90S (GrabCAD, "Tower Pro MG90S micro servo", the case and its four-arm
horn). The model's horn has 7 holes on both long arms; the horn at hand has
6 on one, so that arm is one 2.0 mm hole pitch shorter in the Spec, with
the same edges. The owner's calipers on that horn differ from the model by
a few tenths, which the fit test print absorbs:

| Measured | Calipers | Model |
|---|---|---|
| tip to tip, the two long arms | 34.14 | 34.35: arms of 16.175 and 18.175 |
| across the two short arms | 16.65 | 16.25: arms of 8.125 |
| arm plate thickness | 1.90 | 2.05 |
| collar underside to the plate's top face | 4.40 | 4.25 |
| collar underside to hub top | 5.28 | 4.55 |

The hub stands 0.3 above the plate's top face in the model and 0.9 by the
calipers; the recess over the hub and the screw head is cut 1.0 deeper than
the model, so it clears either.

The heights below set how deep the servo sinks and where the horn lands, so
check them on the servo at hand with calipers and pass any that differ on
the command line.

| Flag | What to measure | Default | Source |
|---|---|---|---|
| `--body-h` | case bottom to case top, without the output boss | 22.55 | model |
| `--ear-z` | case bottom to the underside of the mounting ears | 18.4 | model |
| `--ear-span` / `--ear-t` / `--ear-w` | both ears end to end, ear thickness, ear width | 32.2 / 2.5 / 12.1 | model |
| `--shaft-offset` | case center to the output axis, along the case | 5.375 | model |
| `--horn-above-body` | case top to the top face of the horn's arm plate, horn screwed on | 10.0 | model: the 5.75 boss, the horn's 2.2 collar resting on it, the 2.05 plate; a horn that bottoms on the spline first sits up to 1.75 higher |
| `--horn-hub-above` | hub top above the plate's top face | 0.3 | model (calipers: 0.9) |
| `--head-d` / `--head-h` | center screw head: diameter, and its top above the hub's top face | 3.8 / 1.1 | an M2 pan head; the Spec leaves room for an M2.5 head standing in the hub's 0.9 counterbore |

`--horn-above-base` (1.5) moves the horn up or down in the 1 to 2 mm band.
`--no-retain-screws` leaves out the two optional retaining screw holes.

## Build

Run from the repo root:

```sh
uv sync --group cad
uv run --group cad python -m rigs.servorig      # writes rigs/servorig/build/
uv run --group cad python -m rigs.servorig --horn-above-body 10.4 --head-h 1.6 --preview
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

- **Fit test** first, flat, pockets up, about 13 cm^3 (under 17 g even
  solid). Done once on the horn and servo at hand: the 0.25 key pocket
  pressed on and held, and the 0.05 servo pocket bound, which set the
  defaults below. It carries the tag mount's key pocket four times, labeled with
  its fit per side, 0.05 to 0.35, and a 10 mm section of the servo
  pocket with the ear ledge and the lead slot, open underneath. Press the
  horn, screw in, into each key pocket, hub down: the hub and the screw head
  pass through the plate, so hold it clear of the desk. Pick the fit that
  presses on by hand and does not rock, and try the servo body and ears in
  the short pocket. The fit you pick goes in as `--key-fit` (0.25) and
  `--servo-clear` (0.2) for every part after it. The servo pocket is cut at
  `--servo-clear`, so print the test again with a new value if the body
  does not slide home; the two ear screws locate the servo, so the pocket
  only needs to let it drop in.
- **Base** flat, underside on the bed. The underside and the front face
  are the two datums.
- **Tag mount** top down, tag face on the bed. The bed gives the flattest
  tag face, and the seat on the horn prints as a top skin, so no support
  anywhere. The lock screw counterbore and the retaining screw counterbores
  open at the bed and step down to their holes over short bridges.
- **Load arm** top down, the hub tag face and the cup floors on the bed.
  The cups open the other way, toward the base, so nothing bridges.
- **Coin lid** flat, plugs up.
- **Tag clip** lip down, its top on the bed.

## Hardware

- the MG90S four-arm horn and the servo's center screw
- one M3 heat-set insert, 5.7 mm long for a 4.0 mm hole
- one M3 x 12 button head screw and one M3 washer (0.5 mm), the lock
- five M2 x 6 screws, four for the board and one for the coin lid, and
  two M2 x 8 through the servo's ears, all into 1.8 mm pilots
- two M1.4 x 4 screws or 1.4 x 4 mm self-tapping screws, into the long
  arms' 1.3 mm third holes, 8.5 mm from the axis, to retain the mount on
  the horn (optional)
- three 2.5 mm zip ties
- 48 US quarters, four stacks of 12, for the load arm
- a few strips of 0.5 mm card, for shims under the servo's ears

## Assembly

1. Melt the insert into the base top beside the servo pocket, flush.
2. Screw the board into its pocket, face up, with the bus connectors (J9,
   J10) to the back slots.
3. Center the servo. Push the horn onto the spline with its longer arm
   pointing at the insert, as square to the base as the spline allows, and
   screw it down.
4. Lay the card shims on the ear ledge, one strip under each ear. Feed the
   lead down the slot in the pocket's back wall first, boot and all, and
   keep pulling it through under the base as the servo goes down, so it
   never folds at the case. Lower the servo until the ears sit on the
   shims and screw the ears down. The lead goes along the channel under
   the base and out the back. Zip tie it at the back.
5. Lay the bus and power leads in their grooves and zip tie them.
6. Glue the reference tag into the pad's pocket, square to the base
   edges, and the horn tag onto the mount's flat face, centered by eye:
   the clip's lip covers its edges.
7. Press the tag mount over the horn, ear over the longer arm; it goes on
   one way only. Add the two retaining screws if you use them. Push the
   clip on.
8. Clamp the base to the desk by its wings, and set the phone above it at
   250 mm.

The ear lands over the insert at whatever angle the spline left the horn
at, within one tooth of mid travel. That angle is the lock position. Command
the servo there before putting the lock screw in (washer under the ear).

For the load arm, drop stacks of 12 quarters into the cups first; they
open toward the base. Load by pairs so the arm stays balanced: the inner
pair, the outer pair or all four, and a diagonal pair for the middle step.
Fit the lid, plugs into the bores, and screw it down at the center. Then
swap the mount for the arm, pointing at the front face at mid travel, stand
the base on its front face at the desk edge, horn face out past the edge,
and clamp it by the wings. The arm hangs down at mid travel and swings out
past the edge, the coins lying on the cup walls.

If the horn sits outside the 1 to 2 mm band, change the shims under the
ears. Without shims the horn drops 0.5 mm.

## Acceptance

- Both tags detected in every frame over a full turn.
- Plate tilt under 1 degree for both tags.
- Equal distances from the lens to both tags.
- No frame where a wire or the board crosses a tag.
- STAT flashes found by the clock-sync extraction in the same clip.
- The lock holds a 100 % lap at 3.7 V with the pot reading unchanged.
