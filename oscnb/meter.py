"""A DMM in series with the drive: its packet log, `dmm.csv`.

One packet about every 0.51 s, `value` in amps signed by the current's
direction and `t_host` the host's clock, the same clock a capture's
cycles.csv carries. A spin shows as a run of packets with current, the rest
between spins as packets near zero. The first packet of a spin carries the
spin-up and the last can straddle the cut, so a spin is read over its
packets after the first, less any well under their median.
"""
import numpy as np

FLOOR_A = 2e-3      # A, a packet at or under this reads the rotor at rest
GAP_S = 0.8         # s, over the 0.51 s packet spacing: a new spin starts
KEEP_FRAC = 0.85    # a packet under this share of the median straddles an end
MARGIN_S = 1.0      # s, a cycle's packets read this far past its two ends
MA_PER_A = 1e3


def window(log, t0, t1, margin_s=MARGIN_S):
    """The log's packets inside (t0 - margin_s, t1 + margin_s)."""
    t = log["t_host"]
    return log[(t > t0 - margin_s) & (t < t1 + margin_s)]


def group(packets, scale=MA_PER_A):
    """[dict(dir, t0, values, t)]: the spins in a stretch of packets."""
    p = packets[packets["value"].abs() > FLOOR_A]
    spin = (p["t_host"].diff() > GAP_S).cumsum()
    out = []
    for _, g in p.groupby(spin):
        v = g["value"].to_numpy() * scale
        t = g["t_host"].to_numpy()
        after_first = np.abs(v[1:])
        keep = after_first > KEEP_FRAC * np.median(after_first)
        out.append({"dir": int(np.sign(v.mean())), "t0": t[0],
                    "values": after_first[keep], "t": t[1:][keep]})
    return out


def spins(log, t0, t1, margin_s=MARGIN_S, scale=MA_PER_A):
    """The spins the meter saw in one cycle, in time order, mA by default."""
    return group(window(log, t0, t1, margin_s), scale)


def by_direction(found):
    """{direction: mean}, a later spin of a direction replacing an earlier."""
    return {s["dir"]: float(s["values"].mean()) for s in found}


def pooled_by_direction(found, directions=(1, -1)):
    """{direction: the mean of its spins' means}, NaN for one with none."""
    out = {}
    for d in directions:
        means = [s["values"].mean() for s in found if s["dir"] == d]
        out[d] = float(np.mean(means)) if means else np.nan
    return out
