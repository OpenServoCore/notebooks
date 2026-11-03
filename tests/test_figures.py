"""The figure scaffolds under a headless backend: the axes they make, the
labels they set and the marks they draw."""
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
import sweep_synth as synth  # noqa: E402

from oscnb import figures, tel  # noqa: E402

FS = 20_000


@pytest.fixture(autouse=True)
def shown(monkeypatch):
    calls = []
    monkeypatch.setattr(plt, "show", lambda: calls.append(True))
    yield calls
    plt.close("all")


@pytest.fixture
def sweep(tmp_path):
    folder = synth.write(tmp_path / "sweep", baseline_rows=400,
                         rung_rows=600, coast_rows=200, tail_rows=5)
    return tel.load(folder)[0]


def test_direction_style_fills_forward_and_opens_reverse():
    forward = figures.direction_style(1, figures.BLUE)
    reverse = figures.direction_style(np.int64(-1), figures.ORANGE)
    assert forward == {"ls": "none", "marker": "o", "color": figures.BLUE,
                       "mfc": figures.BLUE}
    assert reverse["marker"] == "^" and reverse["mfc"] == "none"
    fig, ax = plt.subplots()
    (line,) = ax.plot([0, 1], [0, 1], **reverse)
    assert line.get_linestyle() == "None"


def test_label_and_finish(shown):
    fig, ax = plt.subplots(1, 2)
    figures.label(ax[0], x="speed (rev/s)", y="current (mA)", title="t")
    figures.label(ax[1], y="volts", grid=False)
    assert ax[0].get_xlabel() == "speed (rev/s)"
    assert ax[0].get_ylabel() == "current (mA)"
    assert ax[0].get_title() == "t"
    assert ax[0].xaxis.get_gridlines()[0].get_visible()
    assert not ax[1].xaxis.get_gridlines()[0].get_visible()
    figures.finish(fig)
    assert shown == [True]


def test_shades_run_light_to_dark():
    colors = figures.shades("Blues", 5)
    assert colors.shape == (5, 4)
    brightness = colors[:, :3].sum(axis=1)
    assert np.all(np.diff(brightness) < 0)
    assert np.array_equal(figures.shades(plt.cm.Oranges, 3, 0.3, 1.0),
                          plt.cm.Oranges(np.linspace(0.3, 1.0, 3)))


def test_sweep_overview_stacks_one_panel_per_channel(sweep):
    channels = [
        ("current_raw (counts)", [(sweep["current_raw"], figures.BLUE)]),
        ("terminal taps (counts)", [(sweep["vmotor_a"], figures.BLUE, "A"),
                                    (sweep["vmotor_b"], figures.ORANGE, "B")]),
        ("duty_counts (ticks)", [(sweep["duty_counts"], figures.INK)]),
    ]
    fig, axes = figures.sweep_overview(sweep, FS, channels, title="cycle 0")
    assert len(fig.axes) == 3 and len(axes) == 3
    assert [a.get_ylabel() for a in axes] == [c[0] for c in channels]
    assert axes[-1].get_xlabel() == "time through the sweep, rows only (s)"
    assert axes[0].get_title() == "cycle 0"
    edges = tel.segment_edges(sweep)
    assert len(edges) == 4
    for ax, (_, traces) in zip(axes, channels):
        assert len(ax.lines) == len(traces) + len(edges)
    assert axes[1].get_legend() is not None
    assert axes[0].get_legend() is None
    assert axes[0].lines[0].get_xdata()[-1] == (len(sweep) - 1) / FS


def test_sweep_overview_draws_into_given_axes(sweep):
    fig, ax = plt.subplots(1, 3)
    channels = [("current_raw (counts)", [(sweep["current_raw"], "k")]),
                ("taps (counts)", [(sweep["vmotor_a"], "b")])]
    out, axes = figures.sweep_overview(sweep, FS, channels, axes=ax[:2],
                                       edges=False)
    assert out is fig and len(fig.axes) == 3
    assert len(axes[0].lines) == 1 and len(ax[2].lines) == 0


def test_routes_errorbar_one_row_per_route():
    routes = [("ratio at the cut", 2.0e-9, 1.8e-9, 2.3e-9),
              ("line at the cut", 2.1e-9, 2.0e-9, 2.2e-9, figures.MUTED)]
    fig, ax = figures.routes_errorbar(routes, "J (1e-9 kg m$^2$)",
                                      highlight=2.05e-9, scale=1e9,
                                      title="every route")
    assert len(fig.axes) == 1
    assert [t.get_text() for t in ax.get_yticklabels()] == [
        "ratio at the cut", "line at the cut"]
    assert ax.yaxis_inverted()
    assert ax.get_xlabel() == "J (1e-9 kg m$^2$)"
    assert ax.get_title() == "every route"
    assert len(ax.containers) == 2
    first = ax.containers[0].lines[0]
    assert first.get_xdata()[0] == pytest.approx(2.0)
    assert ax.lines[-1].get_xdata()[0] == pytest.approx(2.05)
    assert ax.containers[1].lines[0].get_color() == figures.MUTED
