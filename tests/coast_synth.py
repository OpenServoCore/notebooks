"""A synthetic coast EMF, for the coast and ripple tests.

The EMF falls on a straight line from the cut to zero at the stop, the
rotor's speed in proportion to it, with a six-per-revolution ripple riding
on it whose frequency follows the speed exactly. Dropout rows read zero.
"""
import numpy as np

FS = 20_000
EMF_V = 3.0                 # at the cut
REV_S = 500.0               # at the cut
PER_REV = 6
STOP_S = 0.25               # the line reaches zero here
ROWS = int(0.3 * FS)
RIPPLE_SHARE = 0.03         # the ripple's amplitude as a share of the EMF
DROPOUTS = (300, 301, 1500, 2800)


def line():
    """The EMF line, volts by row from the cut."""
    t = np.arange(ROWS) / FS
    return EMF_V * np.maximum(1 - t / STOP_S, 0.0)


def ripple_hz(t):
    """The ripple's frequency at t seconds past the cut."""
    return REV_S * PER_REV * np.maximum(1 - np.asarray(t) / STOP_S, 0.0)


def emf(dropouts=DROPOUTS, phase0=0.3):
    """The line with its ripple, and zero on the dropout rows."""
    t = np.arange(ROWS) / FS
    held = np.minimum(t, STOP_S)
    f0 = REV_S * PER_REV
    phase = 2 * np.pi * f0 * (held - held ** 2 / (2 * STOP_S)) + phase0
    e = line() * (1 + RIPPLE_SHARE * np.sin(phase))
    e[list(dropouts)] = 0.0
    return e
