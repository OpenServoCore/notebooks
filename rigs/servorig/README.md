# servorig

The printed rig for the stage 3 camera experiments: one MG90, one
osc-dev-v006 board and two camera tags in one rigid piece. The design story,
the frame and the reasons behind each part live in the package docstring,
[`__init__.py`](__init__.py). This page covers what to measure, the build,
the print, the hardware, the assembly and the acceptance.

There are eight printed parts, the base, three mounts that take turns on
the horn, the camera mount, the stall mount and the load mount, the load
mount's coin block and the block's lid, and the cap base and cap top that
carry a tag, plus a fit test print that goes first. Print two caps, one for
the reference pad and one for the horn that moves between the camera mount
and the load mount; the stall mount takes no cap. The rest is the MG90S
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
`servorig_assembly.step`, the servo envelope with the camera mount and both caps
on it. It also
runs the checks and stops with an error if one fails:

- nothing of the base rises inside the keep-out circle, which is the circle
  the camera mount's cap sweeps plus the tag's quiet border
- the camera mount and its cap clear the base over a full turn
- the stall mount clears the base within 5 degrees of the lock position;
  its screw head sits on the ear, its thread bites the ear and the base,
  and it ends past the M3 nut, which seats under its pocket's roof
- the zip tie bands lie in their grooves under the base
- the load mount, its block, lid and cap clear the base and the board over
  the servo's travel plus 10 degrees each way
- the lid sits in the block's pocket, the block sits on the beam, and the
  joint screws pass their nuts and end inside the beam
- the servo envelope seats in its pocket, the three mounts sit on the
  horn and touch nothing else
- the cap base sits in the cap top's skirt, the cap goes over the camera
  mount, the reference pad and the load mount's hub, and the cap base
  seats on each of their top faces
- the three mounts, lowered 0.01 mm, meet only the top face of the horn's
  arm plate, and meet it on all four arms
- every part, the fit test print included, fits the MK3S bed, and both tags
  and the board fit the phone's frame at the working distance

## Print

- **Fit test** first, flat, pockets up, about 13 cm^3 (under 17 g even
  solid). Done once on the horn and servo at hand: the 0.25 key pocket
  pressed on and held, and the 0.05 servo pocket bound, which set the
  defaults below. It carries the mounts' key pocket four times, labeled with
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
- **Hole test** flat, pockets up, about 5 cm^3. Five strips, each with
  the long arms' key pocket, the shorter arm's retaining hole and one
  trial radius on the longer arm, engraved. Press the loose horn into a
  pocket, hub down, and drive an M1.4 into each trial hole from below. The
  middle of the trials it goes into straight is the longer arm's hole;
  set it in `retain_holes` (and the load mount's hole row) before printing
  the mounts.
- **Base** flat, underside on the bed. The underside and the front face
  are the two datums. Everything that opens at the bed, the lead channel,
  the tie bands' grooves and the stall nut's pocket, closes under a 45
  degree roof, so nothing bridges.
- **Camera mount** top down, cap seat on the bed. The bed gives the
  flattest seat for the cap, and the seat on the horn prints as a top
  skin, so no support anywhere. The retaining screw counterbores open at
  the bed and step down to their holes over short bridges.
- **Stall mount** top down, like the camera mount. One solid arm tapers
  from the head's 40.4 mm width to the 8.9 mm ear at the screw, 70 mm out.
  The screw's hole is 2.8, as in the base: 0.2 under the M3's major
  diameter, like the board's M2 pilots, so the screw cuts its own thread
  in both, its head on the ear's top face. The two retaining holes are
  1.2, tap size for the M1.4, so those screws thread in too. Print it with
  extra perimeters or solid infill so the arm does not flex under a lap.
- **Load mount** top down, the hub's cap seat and the beam's top on the
  bed. The nut pockets open upward.
- **Coin block** floor down, the flange on the bed, cups open up.
- **Coin lid** flat, plugs up.
- **Cap base** flat, seat on the bed, paper pocket up.
- **Cap top** lip down, its top on the bed.

The cap tops hold by crush ribs, 0.3 mm of squeeze per side by default.
Print one cap top first and press it onto the camera mount: if it goes on loose,
rebuild with a larger `--rib-squeeze` (0.4); if it will not go on by hand,
a smaller one (0.2).

## Hardware

- the MG90S four-arm horn and the servo's center screw
- one M3 x 25 button head screw, one M3 washer (0.5 mm) and one M3 hex
  nut, the stall lock
- five M2 x 6 screws, four for the board and one for the coin lid, and
  the servo's two long screws through its ears, all into 1.8 mm pilots
- two M2.5 x 8 pan head screws and two M2.5 hex nuts, the block to the
  load mount
- two M1.4 x 4 pan head screws, machine or self-tapping, head under 3 mm
  across, into the long arms' 1.3 mm third holes, 8.8 mm from the axis on the
  longer arm and 8.5 on the shorter:
  they retain the load mount on the horn and the camera mount if you
  like, and pin the stall mount to it, which needs them. Bought
  separately; the servo's two long screws are for its ears. No longer
  than 4 mm: a 5 mm tip touches the floor of the horn's well
- three 2.5 mm zip ties, heads on the top face so the base sits flat
- 48 US quarters, four stacks of 12, for the load mount
- a few strips of 0.5 mm card, for shims under the servo's ears

## Assembly

1. Thread the M3 nut a few turns onto the M3 x 25, push it up the hex
   pocket in the base's underside until it stops under the roof, and back
   the screw out; the tight pocket holds the nut there.
2. Screw the board into its pocket, face up, with the bus connectors (J9,
   J10) to the back slots.
3. Center the servo. Push the horn onto the spline with its longer arm
   pointing at the stall nut's hole, as square to the base as the spline
   allows, and screw it down.
4. Lay the card shims on the ear ledge, one strip under each ear. Feed the
   lead down the slot in the pocket's back wall first, boot and all, and
   keep pulling it through under the base as the servo goes down, so it
   never folds at the case. Lower the servo until the ears sit on the
   shims and screw the ears down. The lead goes along the channel under
   the base and out the back. Zip tie it at the back.
5. Lay the bus and power leads in their grooves and zip tie them.
6. Glue each tag into a cap base's pocket and press a cap top over it:
   the lip covers the paper's edges and the skirt holds the cap base.
7. Press the camera mount over the horn; it goes on one way only. Add the
   two retaining screws if you use them, before the cap, which covers
   them. Push one cap onto the camera mount and the other onto the
   reference pad.
8. Clamp the base to the desk by the flat tabs at its back corners, and set
   the phone above it at 250 mm.

For the locked sitting, swap the camera mount for the stall mount. Screw
the M3 x 25 down through the ear's hole until it stands out below, slip
the washer onto it, and press the mount over the horn, the screw into the
base's hole. The ear lands near the hole at whatever angle the spline left
the horn at, within one tooth of mid travel. Command the servo to the
angle that puts the screw over the hole; that angle is the lock position.
Drive the two M1.4 x 4 into the mount's retaining holes, where they cut
their thread and pin the mount to the horn. Then keep screwing the M3
until it picks up the nut and the head comes down on the ear, and do it
up firmly.

For the load mount, press the two M2.5 nuts into the hex pockets in the
beam's underside, set the block on the beam with its flange over them,
and screw the two M2.5 x 8 down through the flange. Drop stacks of 12
quarters into the cups, by pairs so the arm stays balanced: the inner
pair, the outer pair or all four, and a diagonal pair for the middle step.
Fit the lid, plugs into the bores, and screw it down at the center. Then
swap the camera mount for the load mount, moving the horn's cap onto its hub with
the notches over the beam, pointing at the front face at mid travel, stand
the base on its front face at the desk edge, horn face out past the edge,
and clamp it by the uprights at its front corners, which now lie on the
desk. The arm hangs down at mid travel and swings out past the edge, the
coins lying on the cup walls.

If the horn sits outside the 1 to 2 mm band, change the shims under the
ears. Without shims the horn drops 0.5 mm.

## Acceptance

- Both tags detected in every frame over a full turn.
- Plate tilt under 1 degree for both tags.
- Equal distances from the lens to both tags.
- No frame where a wire or the board crosses a tag.
- STAT flashes found by the clock-sync extraction in the same clip.
- The lock holds a 100 % lap at 3.7 V with the pot reading unchanged.
