"""Stage 1, ntc-curve: the board NTC against the thermocouple while the
board warms and cools. Needs the board on a controlled heat source (a 3D
printer's heated bed, a hot plate with a thermostat, a filament dryer,
anything that holds a set temperature with the whole board at one
temperature) with the thermocouple tip taped to the board next to the NTC,
the DMM on temperature, the board on the DPS and the `osc` tool. The heater
runs at two settings and then off; the script polls the NTC's raw count the
whole time and records the conversion constants the board holds, so the
notebook computes the firmware's reading itself (the experiment image
publishes no estimate).

  uv run python stages/1-board/ntc-curve/capture.py [N]    writes telemetry/<board>/ntc-curve/capture-N
"""
import csv
import json
import threading
import time

from oscnb import b41t, bench, boards, capture, osc

VOLTS = 7.4
POLL_S = 2.0
PLATEAUS = ["room", "warm", "hot", "room again"]
# the quadratic the firmware converts the count with, from its calibration block
NTC_CONSTANTS = ["ntc_raw_ref", "ntc_t_ref_cc", "ntc_k1_q88", "ntc_k2_q24"]


def poll(path, stop):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_host", "ntc_raw"])
        while not stop.is_set():
            w.writerow([f"{time.time():.3f}", osc.get("ntc_raw")])
            f.flush()
            stop.wait(POLL_S)


out = capture.dataset(boards.DEFAULT, "ntc-curve")
capture.prompt("Thermocouple tip taped to the board next to the NTC, DMM on temperature (C), Bluetooth on. "
               "Board on the heater, heater off.")
bench.power_up(VOLTS + bench.DIODE_V)
(out / "ntc-constants.json").write_text(json.dumps({k: osc.get(k) for k in NTC_CONSTANTS}, indent=1))
stop = threading.Event()
with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="C"):
    marks = capture.Marks(out)
    t = threading.Thread(target=poll, args=(out / "ntc.csv", stop), daemon=True)
    t.start()
    for p in PLATEAUS:
        marks.mark(p)
        hint = {"room": "Logging at room. Set the heater to about 45 C.",
                "warm": "Press Enter when the thermocouple has settled. Then set the heater to about 60 C.",
                "hot": "Press Enter when it has settled again. Then switch the heater off.",
                "room again": "Press Enter when the board is back near room."}[p]
        capture.prompt(hint)
    marks.mark("end")
    marks.close()
    stop.set()
    t.join()
bench.power_down()
print("wrote", out)
