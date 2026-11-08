"""A sitting's manifest read back: the probes the runner logged.

Before every sweep `python -m oscnb.sitting` reads the board's registers
with torque off and logs a `probe` row whose detail is
`<where>: pos P, ntc_raw N, vbus_raw V`. `<where>` is the sweep's folder as
the manifest names it, `telemetry/<pair>/<experiment>/<dataset>/<folder>`,
or a moment of the sitting such as `center`.
"""
import pandas as pd


def manifest_probes(path):
    """{where: {"t_probe", "pos", "ntc_raw", "vbus_raw"}} of a manifest."""
    rows = {}
    manifest = pd.read_csv(path)
    for _, r in manifest[manifest["event"] == "probe"].iterrows():
        where, rest = r["detail"].split(": ", 1)
        values = dict(kv.split(" ") for kv in rest.split(", "))
        rows[where] = {"t_probe": r["t_host"],
                       **{k: int(v) for k, v in values.items()}}
    return rows


def with_probes(cycles, probes, dataset):
    """`cycles` with the probe before each sweep, `<dataset>/<folder>`,
    merged in as floats, nan where a sweep has none."""
    prefix = f"{dataset}/"
    rows = [{"folder": where[len(prefix):],
             **{k: float(v) for k, v in p.items()}}
            for where, p in probes.items() if where.startswith(prefix)]
    return cycles.merge(pd.DataFrame(rows), how="left", on="folder")
