"""Stage 0, clock-sync: the camera's clock against the host's. The host
flashes the servo's STAT lamp through `stat_flash_ticks` (the `osc clapper`
command, which logs the host time either side of each write) while the
phone films the lamp. Needs a servo board on the bus running a firmware
with that register, the `osc` tool on the PATH (`cargo install --path
tools/osc` in the firmware repo), the DPS on the board's power leads, and
the phone on its mount with the STAT lamp in view.

The script powers the board from the DPS at the bench rail once the leads
are confirmed connected, runs the flashes, and switches the DPS off after
the clip is in.

  uv run python stages/0-instruments/clock-sync/capture.py [N]    writes telemetry/instruments/clock-sync/capture-N
"""
import os
import subprocess
import time

from oscnb import capture, clips, dps150

FLASHES = 12
TICKS = 2000  # 100 ms at the 20 kHz control tick
VOLTS = 7.4  # the bench rail, a 2S pack's nominal
AMPS = 1.0  # the board idles with torque off; the lamp is the only load

out = capture.dataset("instruments", "clock-sync")
osc = os.environ.get("OSC", "osc")
capture.prompt(f"DPS leads on the board's power input. The script sets {VOLTS} V and turns the output on.")
dps = dps150.open_supply()
try:
    dps150.on(dps, VOLTS, AMPS)
    m = dps150.read(dps)
    print(f"DPS on, {m.output_voltage:.2f} V")
finally:
    dps.close()
capture.prompt("Servo on the bus, phone on its mount with the STAT lamp in view, in SLO-MO at 240 fps.")
capture.prompt("Start recording.")
with open(out / "marks.csv", "w") as f:
    subprocess.run([osc, "clapper", "--flashes", str(FLASHES), "--ticks", str(TICKS)], stdout=f, check=True)
print(f"{FLASHES} flashes written, host times in {out / 'marks.csv'}")
capture.prompt("Stop recording. Copy the clip into the folder as lamp.mp4 (equipment/camera/iphone.md).")
while not (out / "lamp.mp4").exists() or (out / "lamp.mp4").stat().st_size == 0:
    time.sleep(5)
clips.extract(out)
dps = dps150.open_supply()
try:
    dps150.off(dps)
    print("DPS off")
finally:
    dps.close()
print("wrote", out)
