"""The house figure style: the palette, the direction markers and the
scaffolds several notebooks draw the same way, so a figure cell holds its
data loop and nothing else.

A direction is the marker's shape and fill, forward a filled circle and
reverse an open triangle, so the color stays free for the rail, the duty or
the wiring a figure compares.
"""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colormaps

from . import tel

BLUE = "#2a78d6"
ORANGE = "#eb6834"
GREEN = "#1baf7a"
INK = "#0b0b0b"
MUTED = "#898781"
RED = "#d03030"
HOUSE = {"blue": BLUE, "orange": ORANGE, "green": GREEN, "ink": INK,
         "muted": MUTED, "red": RED}
DIRECTION_MARKER = {1: "o", -1: "^"}
GRID_ALPHA = 0.3            # a grid the data sits on, not in front of
TRACE_LW = 0.4              # a raw 20 kHz trace, thin enough to show its noise
EDGE_STYLE = {"color": MUTED, "lw": 0.6, "ls": ":"}
HIGHLIGHT_STYLE = {"color": RED, "lw": 0.8, "ls": "--"}
# a sequential map's span: the light end still shows on white, the dark end
# stops short of black
SHADE_LO = 0.35
SHADE_HI = 0.95
OVERVIEW_WIDTH_IN = 13
OVERVIEW_ROW_IN = 2.7
ROUTES_WIDTH_IN = 10
ROUTE_ROW_IN = 0.5
ROUTES_MIN_IN = 3
CAP_PT = 3


def direction_style(direction, color):
    """Plot keywords for one direction: filled forward, open reverse."""
    return {"ls": "none", "marker": DIRECTION_MARKER[direction],
            "color": color, "mfc": color if direction == 1 else "none"}


def label(ax, x=None, y=None, title=None, grid=True):
    """Name an axis's x, y and title where given, and draw its grid."""
    if x is not None:
        ax.set_xlabel(x)
    if y is not None:
        ax.set_ylabel(y)
    if title is not None:
        ax.set_title(title)
    if grid:
        ax.grid(alpha=GRID_ALPHA)


def finish(fig=None):
    """Lay the figure out and show it, the current figure by default."""
    if fig is None:
        fig = plt.gcf()
    fig.tight_layout()
    plt.show()


def shades(cmap, n, lo=SHADE_LO, hi=SHADE_HI):
    """`n` colors from light to dark along a sequential map."""
    if isinstance(cmap, str):
        cmap = colormaps[cmap]
    return cmap(np.linspace(lo, hi, n))


def sweep_overview(df, fs, channels, title=None, axes=None, edges=True):
    """A sweep's raw channels against its rows, segment edges dotted.

    `channels` holds one (y label, traces) per panel, each trace a
    (values, color) or (values, color, label). Without `axes` the panels
    stack on one shared time axis; with them, the overview draws into the
    axes given and leaves the rest of the figure to the caller."""
    if axes is None:
        height = OVERVIEW_ROW_IN * len(channels)
        fig, axes = plt.subplots(len(channels), 1, sharex=True, squeeze=False,
                                 figsize=(OVERVIEW_WIDTH_IN, height))
        axes = list(axes[:, 0])
    else:
        axes = list(axes)
        fig = axes[0].figure
    rows = np.arange(len(df)) / fs
    starts = tel.segment_edges(df) / fs if edges else []
    for ax, (y_label, traces) in zip(axes, channels):
        for values, color, *name in traces:
            ax.plot(rows, values, lw=TRACE_LW, color=color,
                    label=name[0] if name else None)
        if any(len(trace) > 2 for trace in traces):
            ax.legend(loc="upper right")
        for start in starts:
            ax.axvline(start, **EDGE_STYLE)
        label(ax, y=y_label)
    label(axes[len(channels) - 1], x="time through the sweep, rows only (s)",
          grid=False)
    if title is not None:
        axes[0].set_title(title)
    return fig, axes


def routes_errorbar(routes, x_label, highlight=None, scale=1.0, title=None,
                    ax=None):
    """One quantity by every route, one row each with its bounds.

    `routes` holds (label, value, lo, hi) or (label, value, lo, hi, color),
    ink by default; values are multiplied by `scale` for the axis.
    `highlight` is a value drawn as a dashed line, the one carried on."""
    if ax is None:
        height = max(ROUTES_MIN_IN, ROUTE_ROW_IN * len(routes))
        fig, ax = plt.subplots(figsize=(ROUTES_WIDTH_IN, height))
    else:
        fig = ax.figure
    for row, route in enumerate(routes):
        _, value, lo, hi, color = (*route, INK)[:5]
        below = (value - lo) * scale
        above = (hi - value) * scale
        ax.errorbar(value * scale, row, xerr=[[below], [above]], fmt="o",
                    color=color, capsize=CAP_PT)
    ax.set_yticks(range(len(routes)), [route[0] for route in routes])
    ax.invert_yaxis()
    if highlight is not None:
        ax.axvline(highlight * scale, **HIGHLIGHT_STYLE)
    label(ax, x=x_label, title=title, grid=False)
    ax.grid(alpha=GRID_ALPHA, axis="x")
    return fig, ax
