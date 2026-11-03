"""The servo rig for the stage 3 camera experiments: a base that holds one
MG90, one osc-dev-v006 board and a reference tag in one rigid piece, a tag
mount that presses over the servo's four-arm horn, a load arm that takes the
mount's place, and a clip that holds a tag paper flat.

A phone films the rig from straight above, and the camera reads the horn's
angle from the tag on the mount against the tag on the base. That reading is
the independent reference for the potentiometer, so nothing may move between
clips and no wire may cross a tag. The tags are 25 mm ArUco squares with a
6.25 mm white border, filmed from 250 mm; the size and the distance come from
the camera-tag notebook of stage 0.

The servo sinks into the base and sits on its mounting ears on a ledge, with
the body snug in a pocket that runs through to the underside, so the servo
pushes out from below. The ledge is cut half a millimeter low and card shims
under the ears set the horn's height, so the top face of the horn's arm
plate stands 1.5 mm above the base top, inside a 1 to 2 mm band, turning in
a shallow well. The
servo lead drops through a slot in the pocket's back wall and runs to the
back in a channel under the base. The base top carries the reference tag's
pad, the board's pocket with its pads and pilots, one M3 heat-set insert for
the lock and two grooves at the back for the bus and power leads. Each of
the three leads gets a pair of zip tie slots at the back. An L-shaped clamp
wing at each end holds the base to the desk, lying flat or standing on its
front face.

The horn is the SG90's horn #3, four unequal arms on one 2 mm plate: two
long arms of 16 and 18 mm and two short ones of 8.25 mm at right angles to
them, with a hub that stands 3.3 mm above the plate. The tag mount is keyed
to all four arms, so it goes on one way only, the longer arm under the lock
ear. The key reaches 1.5 mm into the plate, which keeps the skirt around it
clear of the boss under the horn; the skirt is a full circle that encloses
the long arms and stays inside the tag plate's square. The mount's underside
sits on the plate's top face across all four arms, which makes that face the
flatness reference. The hub rises into a clearance recess and the M2 center
screw's head into a counterbore above it, neither touched. The mount's top
is the tag plane, 7 mm over the plate, with a one-layer pocket for the paper
at the height of the reference pad's own. One ear reaches out over the
insert with a slot for the lock screw, 3 degrees of play each way. The ear
lands wherever the spline left the horn, within one tooth of mid travel, and
that angle is the lock position. Two optional M1.4 screws through the long
arms' holes at 8.5 and 8.0 mm, opposite each other so the clamp balances,
retain the mount on the horn; they never locate it, and they stay because
the hanging load arm pries at a press fit 1.5 mm deep. Out there their
counterbores leave a full wall to the hub's recess.

The load arm has the same keyed seat on the horn and a beam to two coin cups at
55 mm from the axis, about 120 g of US quarters. The base stands on its front
face at the desk edge and the arm hangs down at mid travel. A tag pocket on
the arm's hub keeps the camera reading the horn under load. That hub tag
sits at the cup rims, about 13 mm above the tag plane and closer to the lens
than the reference tag, which lets the arm print top down. The clip is a
square frame that pushes onto the mount's plate or the arm's hub and holds
the paper flat by its quiet border. Its lip stands 0.6 mm above the tag
plane; both papers sit in the same 0.2 mm pocket depth, so the papers are
level with each other and the clip top is not.

Nothing of the base rises inside the keep-out circle, which is the circle
the mount and its clip sweep, ear corners included, plus the tag's quiet
border. In plan, the cups pass 10 mm from the board pocket at the end of a
180 degree travel, and 1 mm with the 10 degrees past each end that the
checks allow. The arm rides above the board, but a servo with more travel
needs a look.

Frame: X east, Y north (the back), Z up, the horn axis on Z and Z = 0 the
base's underside. At mid travel the mount's ear points +X over the insert
and the load arm points -Y, toward the front face. The base prints on its
underside and the mount, the arm and the clip print top down, so each tag
face comes off the bed flat and no part needs support.

A fit test print goes first: a thin plate with the mount's key pocket at
three fits, 0.05, 0.15 and 0.25 mm per side, each labeled, and a 10 mm
section of the servo pocket with its ear ledge and lead slot. The real horn
and servo pick the fit, which becomes the key and pocket clearance of every
part after it.

Servo dimensions default to the SG90-class case measured in the firmware
repo's mechanical/sg90/measurements.py. The horn's arm reaches, plate, hub
and spline socket are the owner's calipers rounded to nominal, its hole
radii and edge tapers the SG90 horn #3 drawing, and its arm widths a photo.
What is still a guess is measured with calipers and goes in as Spec fields
or command line overrides before the print (README.md lists them).

    uv run --group cad python -m rigs.servorig [--head-h 1.6 ...]
    writes rigs/servorig/build/
"""
