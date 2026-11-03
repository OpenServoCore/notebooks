"""The servo rig for the stage 3 camera experiments: a base that holds one
MG90, one osc-dev-v006 board and a reference tag in one rigid piece, a tag
mount that presses over the servo's cross horn, a load arm that takes the
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
under the ears set the horn's height, so the cross's top face stands 1.5 mm
above the base top, inside a 1 to 2 mm band, turning in a shallow well. The
servo lead drops through a slot in the pocket's back wall and runs to the
back in a channel under the base. The base top carries the reference tag's
pad, the board's pocket with its pads and pilots, one M3 heat-set insert for
the lock and two grooves at the back for the bus and power leads. Each of
the three leads gets a pair of zip tie slots at the back. An L-shaped clamp
wing at each end holds the base to the desk, lying flat or standing on its
front face.

The tag mount is keyed to the cross's four arms. Its underside sits on the
cross's top face across all four arms, which makes that face the flatness
reference, and it clears the M2 center screw's head without touching it. Its
top is the tag plane, with a one-layer pocket for the paper at the height of
the reference pad's own. One ear reaches out over the insert with a slot for
the lock screw, 3 degrees of play each way. The ear lands wherever the spline
left the cross, within one tooth of mid travel, and that angle is the lock
position. Two optional self-tapping screws through the outer arm holes
retain the mount on the horn; they never locate it.

The load arm has the same seat on the cross and a beam to two coin cups at
55 mm from the axis, about 120 g of US quarters. The base stands on its front
face at the desk edge and the arm hangs down at mid travel. A tag pocket on
the arm's hub keeps the camera reading the horn under load. That hub tag
sits at the cup rims, about 16 mm above the tag plane and closer to the lens
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

Servo dimensions default to the SG90-class case measured in the firmware
repo's mechanical/sg90/measurements.py, and the horn to a stock 9 g cross
horn; the servo at hand is measured with calipers and the numbers go in as
Spec fields or command line overrides before the print (README.md lists
them).

    uv run --group cad python -m rigs.servorig [--horn-reach 10.4 ...]
    writes rigs/servorig/build/
"""
