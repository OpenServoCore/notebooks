"""Stage 1, ntc-curve: the board NTC against the thermocouple while the
board warms and cools. Needs the board on a controlled heat source (a 3D
printer's heated bed, a hot plate with a thermostat, a filament dryer,
anything that holds a set temperature with the whole board at one
temperature) with the thermocouple tip taped to the board next to the NTC,
the DMM on temperature, the board on the DPS and the `osc` tool. The heater
runs at two settings and then off; the script polls the NTC's raw count the
whole time.

  uv run python stages/1-board/ntc-curve/capture.py [board] [N]    writes telemetry/<board>/ntc-curve/capture-N

board is a key of the board registry (oscnb/boards.py), the default board
when left out, so one script serves every board of a revision in turn.
"""
import csv
import sys
import threading
import time

from oscnb import b41t, bench, boards, capture, osc

VOLTS = 7.4
POLL_S = 2.0
PLATEAUS = ["room", "warm", "hot", "room again"]


def poll(path, stop):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_host", "ntc_raw"])
        while not stop.is_set():
            w.writerow([f"{time.time():.3f}", osc.get("ntc_raw")])
            f.flush()
            stop.wait(POLL_S)


board = next((a for a in sys.argv[1:] if a in boards.BOARDS), boards.DEFAULT)
n = [a for a in sys.argv[1:] if a.isdigit()]
out = capture.dataset(board, "ntc-curve", ["", *n])
print("board", board)
capture.prompt("Thermocouple tip taped to the board next to the NTC, DMM on temperature (C), Bluetooth on. "
               "Board on the heater, heater off.")
bench.power_up(VOLTS + bench.DIODE_V)
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
