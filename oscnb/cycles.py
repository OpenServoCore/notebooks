"""A capture's table of sweeps: one row per sweep, its folder named relative
to the capture.

The sitting runner writes `cycles.csv`, a cycle numbered `k` with its rail
level and duty; a lap capture writes `levels.csv` in each pass's folder, a
lap numbered `k` with its level. Both carry `folder`, so one reader serves
both and a notebook asks the table for a sweep's folder by its number.
"""
from pathlib import Path

import pandas as pd

TABLE = "cycles.csv"


class Cycles(pd.DataFrame):
    """A sweep table that remembers the capture folder it was read from."""
    _metadata = ["root"]

    @property
    def _constructor(self):
        return Cycles

    def folder(self, k):
        """The folder of the sweep numbered `k`."""
        return self.root / self.loc[self["k"] == k, "folder"].iloc[0]


def load(capture, table=TABLE):
    """The sweep table of a capture, `cycles.csv` unless `table` names one."""
    capture = Path(capture)
    out = Cycles(pd.read_csv(capture / table))
    out.root = capture
    return out
