"""Stage 1, ntc-curve: the board NTC against the thermocouple while the
board warms and cools. Needs the board in the filament dryer with the
thermocouple tip taped to the board next to the NTC, the DMM on temperature,
the board on the DPS and the `osc` tool. The dryer runs at two settings
and then off with the lid open; the script polls the NTC the whole time.

  uv run python stages/1-board/ntc-curve/capture.py [N]    writes telemetry/<board>/ntc-curve/capture-N
"""
import csv
import threading
import time

from oscnb import b41t, bench, boards, capture, osc

VOLTS = 7.4
POLL_S = 2.0
PLATEAUS = ["room", "warm", "hot", "room again"]


def poll(path, stop):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_host", "ntc_raw", "t_ntc_cc"])
        while not stop.is_set():
            w.writerow([f"{time.time():.3f}", osc.get("ntc_raw"), osc.get("t_ntc_cc")])
            f.flush()
            stop.wait(POLL_S)


out = capture.dataset(boards.DEFAULT, "ntc-curve")
capture.prompt("Thermocouple tip taped to the board next to the NTC, DMM on temperature (C), Bluetooth on. "
               "Board in the dryer, lid open, dryer off.")
bench.power_up(VOLTS + bench.DIODE_V)
stop = threading.Event()
with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="C"):
    marks = capture.Marks(out)
    t = threading.Thread(target=poll, args=(out / "ntc.csv", stop), daemon=True)
    t.start()
    for p in PLATEAUS:
        marks.mark(p)
        hint = {"room": "Logging at room. Close the lid and set the dryer to about 45 C.",
                "warm": "Press Enter when the thermocouple has settled. Then set the dryer to about 60 C.",
                "hot": "Press Enter when it has settled again. Then switch the dryer off and open the lid.",
                "room again": "Press Enter when the board is back near room."}[p]
        capture.prompt(hint)
    marks.mark("end")
    marks.close()
    stop.set()
    t.join()
bench.power_down()
print("wrote", out)
