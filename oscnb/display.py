"""Showing a log in a notebook.

The logs keep `t_host` as Unix seconds, which pandas prints as 1.79e+09.
`show` renders it as local wall-clock time with milliseconds and leaves the
frame itself untouched.
"""
from datetime import datetime

import pandas as pd


def clock(t_host):
    """Unix seconds to a local HH:MM:SS.mmm string."""
    local = pd.to_datetime(t_host, unit="s", utc=True).dt.tz_convert(datetime.now().astimezone().tzinfo)
    return local.dt.strftime("%H:%M:%S.%f").str[:-3]


def show(df, n=5):
    """Display the first `n` rows (all rows when n is None) with `t_host` as
    clock time and other floats to three decimals."""
    out = df if n is None else df.head(n)
    out = out.copy()
    if "t_host" in out:
        out["t_host"] = clock(out["t_host"])
    from IPython.display import display
    with pd.option_context("display.float_format", "{:.3f}".format):
        display(out)
