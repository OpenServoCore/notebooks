"""The stage 1 conversions of one board, read from its result files.

current-scale grades the current channel: counts over the lap's own
torque-off zero = drive_offset_counts + counts_per_a x amps. terminal-chain
grades the tap difference, = terminal_offset_counts + terminal_counts_per_v
x volts under DC current, and the rail tap. sample-timing gives the
bridge's edge lags on each graded rail, bridge-states the fast-decay edges.
A later notebook reads its counts through these lines inverted, never
through the registry's nominal conversions.

The zero a reading is taken over stays an argument: the current over the
lap's rest zero, the tap difference over the terminal offset under drive or
over its own rest difference on a coast.

The volts a motor sees at partial duty are formed from the crest volts and
the duty. Construction A is the duty times the crest volts. B takes the
edge lags off the window, the on-share, and the brake's drop through the
low-side loop between the windows off the volts. C is B with a crest term
taken off the current.
"""
from dataclasses import dataclass

import numpy as np

from . import results

STAGE = "1-board"
# the two rails sample-timing graded; a rail between them, 5.0 V among
# them, reads at the mean of the two
GRADED_RAILS = ("7v4", "3v7")
# the same rails low first with their volts, for a lag read on the rail
RAIL_LINE = (("3v7", 3.7), ("7v4", 7.4))


@dataclass(frozen=True)
class Scales:
    """The measured conversions of one board, one result file each."""
    board: str
    current_scale: dict
    terminal_chain: dict
    sample_timing: dict | None = None
    bridge_states: dict | None = None

    @property
    def counts_per_a(self):
        return self.current_scale["counts_per_a"]["value"]

    @property
    def a_per_count(self):
        return 1 / self.counts_per_a

    @property
    def drive_offset_counts(self):
        return self.current_scale["drive_offset_counts"]["value"]

    @property
    def terminal_counts_per_v(self):
        return self.terminal_chain["terminal_counts_per_v"]["value"]

    @property
    def v_per_count(self):
        return 1 / self.terminal_counts_per_v

    @property
    def terminal_offset_counts(self):
        return self.terminal_chain["terminal_offset_counts"]["value"]

    @property
    def rail_counts_per_v(self):
        return self.terminal_chain["rail_counts_per_v"]["value"]

    @property
    def rail_offset_counts(self):
        return self.terminal_chain["rail_offset_counts"]["value"]

    def amps(self, counts, zero, drive=True):
        """Amps from current counts over `zero`, the in-drive offset out."""
        over = counts - zero
        if drive:
            over = over - self.drive_offset_counts
        return over / self.counts_per_a

    def volts(self, counts, zero):
        """Terminal volts from a signed tap difference over `zero`."""
        return (counts - zero) / self.terminal_counts_per_v

    def rail_v(self, counts):
        """Rail volts from the rail tap, terminal-chain's line at rest."""
        counts = np.asarray(counts, float)
        return (counts - self.rail_offset_counts) / self.rail_counts_per_v

    def _timing(self):
        if self.sample_timing is None:
            raise ValueError(f"{self.board}: no sample-timing result")
        return self.sample_timing

    def edge_lags(self, rails=GRADED_RAILS):
        """The bridge's slow-decay edges in ticks, the mean over `rails`."""
        timing = self._timing()
        keys = {"on": "on_edge_lag", "tap_on": "tap_on_edge_lag",
                "off": "off_edge_lag", "settle": "shunt_settle"}
        lags = {}
        for name, key in keys.items():
            values = [timing[f"{key}_{r}"]["value"] for r in rails]
            lags[name] = np.mean(values)
        return lags

    def edge_lags_at(self, rail_v):
        """(ON, OFF, lowest ON - OFF, highest ON - OFF) tap edge lags in
        ticks on the line through the graded rails, extrapolated past them."""
        timing = self._timing()
        (_, lo_v), (_, hi_v) = RAIL_LINE
        f = (rail_v - lo_v) / (hi_v - lo_v)

        def at(key, end):
            a, b = (timing[f"{key}_{r}"][end] for r, _ in RAIL_LINE)
            return a + f * (b - a)

        on = at("tap_on_edge_lag", "value")
        off = at("off_edge_lag", "value")
        lo = at("tap_on_edge_lag", "lo") - at("off_edge_lag", "hi")
        hi = at("tap_on_edge_lag", "hi") - at("off_edge_lag", "lo")
        return on, off, lo, hi

    def share_of(self, half_ticks, rail_v, period):
        """(on-share, lowest, highest) of a half window on `rail_v`, the
        bounds at the widest ON - OFF the lags' bounds allow."""
        on, off, lo, hi = self.edge_lags_at(rail_v)
        h = np.asarray(half_ticks, float)
        return (on_share(h, period, on, off), (2 * h - hi) / period,
                (2 * h - lo) / period)

    def edge_ticks_at(self, rail_v, which="value"):
        """The tap ON lag less the OFF lag in ticks, interpolated between
        the graded rails and held flat outside; "lo" and "hi" the widest
        combination of the two lags' bounds."""
        timing = self._timing()
        on, off = {"value": ("value", "value"), "lo": ("lo", "hi"),
                   "hi": ("hi", "lo")}[which]
        lags = [timing[f"tap_on_edge_lag_{r}"][on]
                - timing[f"off_edge_lag_{r}"][off] for r, _ in RAIL_LINE]
        return float(np.interp(rail_v, [v for _, v in RAIL_LINE], lags))

    def fast_edge_lags(self):
        """The bridge's fast-decay edges in ticks, from bridge-states."""
        if self.bridge_states is None:
            raise ValueError(f"{self.board}: no bridge-states result")
        edges = self.bridge_states["fast_edge_lags_ticks"]
        return {"on": edges["on"]["value"], "off": edges["off"]["value"]}

    def describe(self):
        """Print the lines and edges a notebook reads its counts through."""
        cpa = self.current_scale["counts_per_a"]
        graded = self.current_scale["amps_range"]
        cpv = self.terminal_chain["terminal_counts_per_v"]
        print(f"current: counts over the zero = "
              f"{self.drive_offset_counts:+.1f} + {cpa['value']:.1f} x amps "
              f"({cpa['lo']:.1f} to {cpa['hi']:.1f}), graded over "
              f"{graded['lo']:.2f} to {graded['hi']:.2f} A")
        print(f"terminals: tap difference = "
              f"{self.terminal_offset_counts:+.1f} + {cpv['value']:.2f} x "
              f"volts ({cpv['lo']:.2f} to {cpv['hi']:.2f})")
        print(f"rail: tap = {self.rail_offset_counts:+.1f} + "
              f"{self.rail_counts_per_v:.2f} x volts at rest")
        if self.sample_timing is not None:
            lag = self.edge_lags()
            print(f"bridge ON edge {lag['tap_on']:.0f} ticks after the "
                  f"commanded start, shunt crossing {lag['on']:.0f}, OFF edge "
                  f"{lag['off']:.0f} after the commanded end, shunt settled "
                  f"{lag['settle']:.0f} after the start")
        if self.bridge_states is not None:
            fast = self.fast_edge_lags()
            print(f"fast decay: ON edge {fast['on']:.0f} ticks after the "
                  f"commanded start, OFF edge {fast['off']:.0f} after the end")


def on_share(half_ticks, period, on, off):
    """Construction B's on-share: the window shortened by the edge lags."""
    h = np.asarray(half_ticks, float)
    return (2 * h - on + off) / period


def volts_ab(duty, share, v_on, amps, r_brake):
    """The applied volts under construction A and under B, the brake's
    drop through the low-side loop r_brake between the on-windows."""
    return duty * v_on, share * v_on - (1 - share) * r_brake * amps


def constructions(duty, share, v_on, amps, share_lo, share_hi, r_brake,
                  g_spin):
    """The applied volts under A, B and C and C's current, B also at the
    lags' two bounds; C is B with the crest term g_spin x (1 - D) x V_on
    taken off the current."""
    v_a, v_b = volts_ab(duty, share, v_on, amps, r_brake)
    i_c = amps - g_spin * (1 - duty) * v_on
    return {"v_a": v_a, "v_b": v_b, "i_c": i_c,
            "v_c": volts_ab(duty, share, v_on, i_c, r_brake)[1],
            "v_b_lo": volts_ab(duty, share_lo, v_on, amps, r_brake)[1],
            "v_b_hi": volts_ab(duty, share_hi, v_on, amps, r_brake)[1]}


def _optional(experiment, key):
    if not results.path(STAGE, experiment, key).exists():
        return None
    return results.read(STAGE, experiment, key)


def load(board):
    """The stage 1 conversions of `board`, a registry entry or its key."""
    key = getattr(board, "key", board)
    return Scales(
        board=key,
        current_scale=results.read(STAGE, "current-scale", key),
        terminal_chain=results.read(STAGE, "terminal-chain", key),
        sample_timing=_optional("sample-timing", key),
        bridge_states=_optional("bridge-states", key),
    )
