"""The stage 1 bench: one board on the DPS, a load on its motor terminals,
the osc adapter on its bus, no servo.

The DPS output comes up at its previous setting for a moment when it is
enabled (the dmm notebook saw a 12 V pulse), so the board's leads go on
after the output is on at the first level, never before. Levels then change
with the output left on. A TEL stream at the control rate needs the 3 Mbps
bus, so the board is checked to be there before anything records.
"""
import time

from . import boards, capture, dps150, osc

DIODE_V = 0.25     # the dev board's feed diode at rest, the first guess for the DPS over VSYS


def power_up(volts, amps=2.5):
    """DPS on at `volts` with the board's leads connected, then the bus checked.
    The DPS output comes up at its previous setting for a moment when enabled,
    which the board's rail tolerates, so the leads stay on."""
    capture.prompt(f"Board power leads on the DPS, adapter on the bus. The DPS turns on at {volts} V now.")
    p = dps150.open_supply()
    try:
        dps150.on(p, volts, amps)
        m = dps150.read(p)
    finally:
        p.close()
    print(f"DPS on, {m.output_voltage:.2f} V")
    time.sleep(1)
    print(f"bus at {osc.bus_baud()} baud")
    osc.require_3m()


def set_volts(volts, hold_s=2.0):
    p = dps150.open_supply()
    try:
        p.set_voltage(volts)
        time.sleep(hold_s)
        return dps150.read(p).output_voltage
    finally:
        p.close()


def read_vsys(board=None):
    """The rail as the board's own tap reads it, at the VDD its 1.2 V reference
    reads. Below the regulator's dropout the tap alone reads high, since every
    count is a share of a sagging VDD, and a trim on it walks the wrong way."""
    board = board or boards.BOARDS[boards.DEFAULT]
    return float(board.rail_v(osc.get("vbus_raw"), osc.get("vref_raw")))


def set_vsys(target, tol=0.03, tries=4):
    """Trim the DPS until the rail behind the feed diode reads `target`. The
    levels are rail volts, since a swap board has no diode and sees the
    battery directly. Returns (dps volts read, vsys read)."""
    dps = target + DIODE_V
    for _ in range(tries):
        read = set_volts(dps)
        vsys = read_vsys()
        if abs(vsys - target) <= tol:
            break
        dps += target - vsys
    return read, vsys


def power_down():
    p = dps150.open_supply()
    try:
        dps150.off(p)
    finally:
        p.close()
    print("DPS off")
