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
"""
from dataclasses import dataclass

import numpy as np

from . import results

STAGE = "1-board"
# the two rails sample-timing graded; a rail between them, 5.0 V among
# them, reads at the mean of the two
GRADED_RAILS = ("7v4", "3v7")


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

    def edge_lags(self, rails=GRADED_RAILS):
        """The bridge's slow-decay edges in ticks, the mean over `rails`."""
        if self.sample_timing is None:
            raise ValueError(f"{self.board}: no sample-timing result")
        keys = {"on": "on_edge_lag", "tap_on": "tap_on_edge_lag",
                "off": "off_edge_lag", "settle": "shunt_settle"}
        lags = {}
        for name, key in keys.items():
            values = [self.sample_timing[f"{key}_{r}"]["value"] for r in rails]
            lags[name] = np.mean(values)
        return lags

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
