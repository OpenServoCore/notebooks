# iPhone as the camera

How to set up an iPhone for the tag videos, and how to get the clips onto
the host. Every experiment that films tags uses the same setting, because
the lens calibration from one clip holds only for the setting it was filmed
in. The reasons behind each setting are in the camera-tag notebook's
Procedure.

Any iPhone from the last several years records 1080p slow motion at 240 fps.
A Pro model is not needed. Menu names below are from recent iOS versions and
can differ slightly on older ones.

## Once, in the Settings app

1. **Slow motion format.** Settings > Camera > Record Slo-mo > choose
   **1080p HD at 240 fps**.
2. **File format.** Settings > Camera > Formats > **High Efficiency**. 1080p
   at 240 fps is only offered in this format.
3. **HDR video off.** Settings > Camera > Record Video > turn **HDR Video**
   off. Not every model has this switch; if it is missing, there is nothing
   to turn off.
4. **Level on.** Settings > Camera > turn **Level** on (or **Grid**, on
   older iOS). It shows when the phone points straight down.
5. **Macro switching off** (Pro models, and other models with a macro
   button). Settings > Camera > turn **Macro Control** on. This puts a
   flower button in the Camera app, which is used in the next section to
   keep macro off. Then Settings > Camera > Preserve Settings > turn
   **Macro Control** on, so macro stays off between sessions.
6. **Keep the camera in slow motion.** Settings > Camera > Preserve Settings
   > turn **Camera Mode** on, so the Camera app opens in the mode it was left
   in.
7. **Original files on transfer.** Settings > Apps > Photos (Settings >
   Photos on older iOS) > Transfer to Mac or PC > **Keep Originals**.

## Each filming session, in the Camera app

1. Fix the phone with the lens pointing straight down at the tags, at the
   experiment's working distance (about 20 cm for the tag experiments).
   Most tripods cannot point straight down that close to the desk; an
   overhead phone arm clamped to the desk edge works, and so does bridging
   the phone across two stacks of books with the lens over the gap. With
   Level on, two crosshairs show in the middle of the screen; tilt the phone
   until they line up and turn yellow.
2. Swipe the mode bar at the bottom to **SLO-MO**. If a corner of the screen
   shows the format (for example "HD 240"), tap it until it reads 240.
3. Use the **1x** lens. If the flower button shows, tap it so it is not
   yellow. Macro switches to the ultrawide lens, which changes the lens in
   the middle of a clip.
4. Long-press on the tags until **AE/AF LOCK** shows at the top. Focus and
   exposure now stay where they are. Tapping the screen again clears the
   lock, so after that, start and stop recording only with the shutter
   button or a remote.
5. Film the clips the experiment asks for, without touching the zoom or the
   lock between them.

Recording can be started without touching the phone, which keeps it from
moving on the tripod:

- an Apple Watch, with its Camera Remote app
- a Bluetooth shutter button, which works as the volume button and starts
  or stops a video

The recording does not need to start at a known moment. The clips carry
their own frame times, and the clapper lines them up with the host's log.

## Getting the clips onto the host

Do not edit the clips in Photos and do not share them through a messaging
app. Either can re-encode a slow motion clip and change its timing.

### Linux, over USB

The tools are libimobiledevice and ifuse. On Arch:

```
sudo pacman -S libimobiledevice ifuse usbmuxd
```

1. Plug the phone in and unlock it. Keep it unlocked for the whole copy:
   iOS cuts the USB data link when the phone locks, the mount on the host
   then returns I/O errors until it is remounted, and a copy in progress
   fails. Settings > Display & Brightness > Auto-Lock > Never for the
   session, and back afterwards, avoids this.
2. Pair it once. Tap **Trust** on the phone and enter the passcode when
   asked.
   ```
   idevicepair pair
   ```
3. Mount the phone's photo storage and find the newest clips.
   ```
   mkdir -p ~/iphone
   ifuse ~/iphone
   ls -lt ~/iphone/DCIM/*/ | head
   ```
4. Copy each clip into the capture folder under the name the experiment
   asks for. The files are `.MOV`. A rename on copy is enough, with no
   conversion.
   ```
   cp ~/iphone/DCIM/100APPLE/IMG_1234.MOV telemetry/instruments/camera-tag/capture-1/board.mp4
   ```
5. Unmount when done.
   ```
   fusermount -u ~/iphone
   ```
6. The experiment's `capture.py` runs the extraction once the clips are
   there. The originals stay on this machine, outside git; the tables and
   previews it writes are what gets committed.

If iCloud Photos is set to Optimize iPhone Storage, an older clip may only
be on iCloud and not show up in `DCIM`. Open it in Photos once to download
it, then copy it.

### Mac

Import the clips with the Image Capture app, which keeps the original
files. AirDrop works too: in the share sheet, tap Options and turn on **All
Photos Data**, so the original is sent rather than an edited copy.
