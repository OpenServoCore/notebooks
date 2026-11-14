"""The ADC's wide-code finder on synthetic sweeps, bands at and off the 256
carries, at the range's edges and too wide for a byte, and a thin sweep
refused; the slots; the step on a recording's readings and its load; the
bench sweep's code histogram; the sweep's schedule and its capture with
`osc` stubbed; the install against a fake bus and the slots read back
off it; and the scorecard's `--wide-codes` rung."""
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from oscnb import boards, results, scorecard as sc, servos, tel
from oscnb import widecodes as wc

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scorecard_synth as synth  # noqa: E402
import sweep_synth  # noqa: E402

FIXTURE = Path(__file__).resolve().parent / "widecodes_bench_hist.json"
SERVO = servos.SERVOS["mg90-d"]
BOARD = boards.BOARDS[boards.DEFAULT]
PAIR = sc.pair_of(SERVO, BOARD)
KEY = BOARD.key
TS = sc.testset()


def sweep(lo, hi, wide, rows_per_code=200):
    """Raw pot rows of a slow sweep from `lo` to `hi` and back through a
    converter that sticks at each `wide` code over the codes it skips."""
    x = np.linspace(lo - 3, hi + 3, (hi - lo + 6) * rows_per_code // 2)
    code = np.floor(np.r_[x, x[::-1]]).astype(int)
    for stuck, skipped in wide.items():
        code[(code > stuck) & (code <= stuck + skipped)] = stuck
    return code


def bands(f):
    return [(b.stuck, b.skipped, b.representable, b.slot) for b in f.bands]


def test_bands_at_and_off_the_carries_are_found_and_only_the_carries_fit_the_field():
    f = wc.find(sweep(900, 2700, {1023: 11, 1500: 3, 2047: 10, 2303: 40}), 900, 2700)
    assert bands(f) == [(1023, 11, True, 3), (1500, 3, False, None), (2047, 10, True, 7),
                        (2303, 40, False, None)]
    assert f.median_rows == pytest.approx(200, rel=0.01)
    # the stuck code reads its own rows and the skipped codes'
    assert f.bands[0].ratio == pytest.approx(12, rel=0.02)
    assert [b.value for b in f.bands] == [88, 24, 80, 320]


def test_a_band_within_the_edge_margin_is_not_read_one_just_inside_is():
    m = wc.EDGE_MARGIN_COUNTS
    f = wc.find(sweep(1000, 2000, {1002: 3, 1000 + m - 1: 2, 2000 - m - 2: 2, 1995: 2}), 1000, 2000)
    assert [(b.stuck, b.skipped) for b in f.bands] == [(1000 + m - 1, 2), (2000 - m - 2, 2)]
    assert f.inside == (1000 + m, 2000 - m)


def test_a_run_straddling_the_margin_is_left_out():
    m = wc.EDGE_MARGIN_COUNTS
    f = wc.find(sweep(1000, 2000, {1000 + m - 3: 5}), 1000, 2000)
    assert f.bands == []


def test_a_thin_sweep_is_refused():
    pos = sweep(900, 2700, {1023: 11}, rows_per_code=wc.MIN_ROWS_PER_CODE - 20)
    with pytest.raises(ValueError, match="median .* rows per code"):
        wc.find(pos, 900, 2700)
    with pytest.raises(ValueError, match="no codes inside"):
        wc.find(sweep(1000, 1020, {}), 1000, 1020)


def test_a_code_never_read_under_a_code_that_does_not_read_wide_takes_no_slot():
    pos = sweep(900, 2000, {1023: 4})
    pos = pos[pos != 1792]              # a thin stretch: 1792 is never read, 1791 reads its plain share
    f = wc.find(pos, 900, 2000)
    thin = next(b for b in f.bands if b.stuck == 1791)
    assert (thin.skipped, thin.wide, thin.representable, thin.slot) == (1, False, True, None)
    wide = next(b for b in f.bands if b.stuck == 1023)
    assert wide.wide and wide.slot == 3
    s = wc.slots(f.bands)
    assert s[6] == 0 and s[3] == 4 * wc.Q4_PER_SKIPPED


def test_slots_hold_the_wide_representable_bands_only():
    b = [wc.Band(1023, 11, 12.0), wc.Band(1500, 3, 4.0), wc.Band(0x7FF, 31, 32.0), wc.Band(0xAFF, 32, 33.0),
         wc.Band(0xFFF, 1, 2.0), wc.Band(0x3FF + 0x800, 5, 1.0)]
    s = wc.slots(b)
    assert len(s) == 16 and isinstance(s, bytes)
    assert list(s) == [0, 0, 0, 88, 0, 0, 0, 248, 0, 0, 0, 0, 0, 0, 0, 0]
    assert wc.slots([]) == bytes(16)


# --- the step on a recording's readings ---

def test_a_stuck_code_gains_its_slot_and_the_codes_beside_it_do_not():
    slots = bytes([0, 48, 0, 80] + [0] * 12)
    pos = np.array([1022, 1023, 1024, 511, 510, 512, 2047])
    out = wc.compensated(pos, slots)
    assert out.dtype == float
    assert list(out) == [1022, 1023 + 5.0, 1024, 511 + 3.0, 510, 512, 2047]


def test_a_zero_slot_leaves_its_stuck_code_and_an_all_zero_map_is_the_identity():
    pos = np.arange(4096)
    np.testing.assert_array_equal(wc.compensated(pos, bytes(16)), pos.astype(float))
    slots = np.zeros(16, np.uint8)
    slots[7] = 88
    out = wc.compensated(pos, slots)
    assert out[0x7FF] == 0x7FF + 5.5
    assert out[0x3FF] == 0x3FF and out[0xFFF] == 0xFFF
    assert np.count_nonzero(out != pos) == 1


def test_the_slots_of_a_meta_are_its_boards_and_a_meta_with_no_board_is_refused(monkeypatch):
    seen = []
    monkeypatch.setattr(wc, "slots_of_record", lambda key: seen.append(key) or bytes(range(16)))
    meta = {"sense": dict(BOARD.sense)}
    assert list(wc.slots_of_meta(meta)) == list(range(16))
    assert seen == [boards.from_meta(meta).key]
    with pytest.raises(ValueError, match="no board matches"):
        wc.slots_of_meta({"sense": {**BOARD.sense, "gain_milli": -1}})


def test_a_load_with_the_step_keeps_the_raw_code_and_a_plain_load_is_unchanged(monkeypatch, tmp_path):
    sweep_synth.write(tmp_path)
    df = pd.read_csv(tmp_path / "sweep.csv")
    codes = np.resize([1022, 1023, 1024, 2047, 3000], len(df))
    df.assign(pos=codes).to_csv(tmp_path / "sweep.csv", index=False)
    slots = bytes([0, 0, 0, 80, 0, 0, 0, 88] + [0] * 8)
    monkeypatch.setattr(wc, "slots_of_record", lambda key: slots)
    plain, _ = tel.load(tmp_path)
    assert "pos_raw" not in plain and plain["pos"].dtype == np.int64
    stepped, _ = tel.load(tmp_path, wide_codes=True)
    np.testing.assert_array_equal(stepped["pos_raw"], codes)
    np.testing.assert_array_equal(stepped["pos"], wc.compensated(codes, slots))
    assert set(stepped["pos"] - stepped["pos_raw"]) == {0.0, 5.0, 5.5}
    assert stepped.drop(columns=["pos", "pos_raw"]).equals(plain.drop(columns="pos"))


def bench_rows():
    d = json.loads(FIXTURE.read_text())
    codes = np.arange(d["first_code"], d["first_code"] + len(d["rows"]))
    return np.repeat(codes, d["rows"]), d["swept_counts"]


def test_the_bench_sweep_reads_its_five_carries():
    pos, (lo, hi) = bench_rows()
    assert len(pos) == 540_000
    f = wc.find(pos, lo, hi)
    assert [(b.stuck, b.skipped) for b in f.bands] == [(1023, 11), (1279, 2), (2047, 10), (2303, 2), (2559, 6)]
    assert all(b.representable for b in f.bands)
    assert [b.wide for b in f.bands] == [True, False, True, False, True]
    assert list(wc.slots(f.bands)) == [0, 0, 0, 88, 0, 0, 0, 80, 0, 48, 0, 0, 0, 0, 0, 0]
    assert f.median_rows == 234
    assert min(b.ratio for b in f.bands) > 1


# --- the sweep's schedule and capture ---

def test_the_sweep_opens_then_passes_the_range_in_items_under_the_tools_limit():
    cpd, center, lo, hi = 18.9, 1937, 409, 3465
    segs = wc.segments(center, cpd, lo, hi, TS)
    assert [s["goal_counts"] for s in segs[:2]] == [center + wc.OPENER_COUNTS, center]
    ms = [int(s["item"].split("@")[1]) for s in segs]
    assert max(ms) <= wc.ITEM_MAX_MS < 3276
    passes = [s for s in segs if s["test"] == "pass"]
    assert sorted({s["pass_k"] for s in passes}) == list(range(wc.PASSES))
    assert [s["goal_counts"] for s in passes if s["pass_k"] in (0, 1)][::3] == [lo, hi]
    for k in range(wc.PASSES):
        took = sum(int(s["item"].split("@")[1]) for s in passes if s["pass_k"] == k)
        assert took >= (hi - lo) / wc.SPEED_CPS * 1000 + wc.TURN_MS
    assert segs[-1]["goal_counts"] == center and segs[-1]["test"] == "home"
    assert wc.swept({"segments": segs}) == (lo, hi)


@pytest.fixture
def bench(monkeypatch, tmp_path):
    """A servo at rest on a 3 Mbps bus at 5 V; the track writes a capture's
    meta and logs its schedule; settings apply and restore logged."""
    state = {"torque": "off", "log": []}
    monkeypatch.setattr(results, "ROOT", tmp_path)
    monkeypatch.setattr(sc.osc, "bus_baud", lambda: 3_000_000)
    monkeypatch.setattr(sc.osc, "get", lambda f: {"torque_enable": state["torque"], "pos_min_soft_counts": 314,
                                                   "pos_max_soft_counts": 3560}[f])
    monkeypatch.setattr(sc.bench, "read_vsys", lambda board=None: 5.05)
    inp = {"counts_per_deg": 18.9, "no_load_deg_s": 600.0, "current_cap_ma": 850, "current_cap_ms": 100,
           "sources": {}}
    monkeypatch.setattr(TS, "inputs", lambda servo, pair: inp)
    monkeypatch.setattr(sc, "testset", lambda: TS)

    def track(out, mode, schedule, **kw):
        state["log"].append(("track", list(schedule), kw["deadman_check"]))
        if state.get("abort"):
            raise RuntimeError("aborted")
        plan = json.loads((Path(out) / "plan.json").read_text())
        (Path(out) / "meta.json").write_text(json.dumps(
            synth.meta(plan["center_counts"], plan["guard_counts"], 20_000)))

    monkeypatch.setattr(sc.osc, "track", track)
    monkeypatch.setattr(wc.settings, "apply", lambda fields, note="", stamp=True: state["log"].append(
        ("apply", dict(fields))))
    monkeypatch.setattr(wc.settings, "restore", lambda: state["log"].append(("restore",)))
    return state


def test_a_capture_sweeps_the_soft_range_less_the_margin_under_the_speed_and_restores(bench, tmp_path):
    f = tmp_path / "gains.json"
    f.write_text(json.dumps({"fields": {"p_kp_q88": 5026}}))
    out = wc.capture(settings_file=f, servo=SERVO, board=BOARD)
    assert out == tmp_path / "telemetry" / KEY / "wide-codes" / "sweep-1"
    assert [c[0] for c in bench["log"]] == ["apply", "track", "restore"]
    assert bench["log"][0][1] == {"p_kp_q88": 5026, "velocity_limit_cps": wc.SPEED_CPS}
    assert bench["log"][1][2] is True
    margin = TS.FIT_MARGIN_DEG * 18.9
    lo, hi = math.ceil(314 + margin), math.floor(3560 - margin)
    plan = json.loads((out / "plan.json").read_text())
    assert wc.swept(plan) == (lo, hi)
    doc = json.loads((out / "sweep.json").read_text())
    assert doc["swept_counts"] == [lo, hi] and doc["passes"] == wc.PASSES
    assert doc["fields"]["velocity_limit_cps"] == wc.SPEED_CPS
    assert (doc["board"], doc["ramp_servo"]) == (KEY, SERVO.key)
    assert wc.next_label(KEY) == "sweep-2"
    assert wc.latest(KEY) == out


def test_a_capture_restores_after_an_abort_and_refuses_before_anything_moves(bench):
    bench["abort"] = True
    with pytest.raises(RuntimeError, match="aborted"):
        wc.capture(label="a", servo=SERVO, board=BOARD)
    assert [c[0] for c in bench["log"]] == ["apply", "track", "restore"]
    bench["log"].clear()
    with pytest.raises(SystemExit, match="already holds a sweep"):
        wc.capture(label="a", servo=SERVO, board=BOARD)
    bench["torque"] = "on"
    with pytest.raises(SystemExit, match="torque is on"):
        wc.capture(label="b", servo=SERVO, board=BOARD)
    assert bench["log"] == []


def test_find_writes_the_boards_result_with_the_bands_slots_and_ramp_servo(monkeypatch, tmp_path):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    folder = wc.captures_dir(KEY) / "sweep-1"
    folder.mkdir(parents=True)
    (folder / "sweep.json").write_text(json.dumps({"board": KEY, "ramp_servo": SERVO.key}))
    pos, (lo, hi) = bench_rows()
    (folder / "track.csv").write_text("pos\n" + "\n".join(map(str, pos)) + "\n")
    segs = wc.segments(1937, 18.9, lo, hi, TS)
    (folder / "plan.json").write_text(json.dumps({"segments": segs}))
    f = wc.find_capture(folder)
    assert "1023 11 11.5 yes 3 88 yes" in " ".join(wc.show(f).split())
    path = wc.write_result(KEY, folder, f)
    assert str(path) == f"stages/1-board/wide-codes/results/{KEY}.json"
    doc = results.read("1-board", "wide-codes", KEY)
    assert doc["capture"] == f"telemetry/{KEY}/wide-codes/sweep-1" and doc["label"] == "sweep-1"
    assert doc["ramp_servo"] == SERVO.key
    assert doc["adc_wide_mid_q4"] == [0, 0, 0, 88, 0, 0, 0, 80, 0, 48, 0, 0, 0, 0, 0, 0]
    assert doc["wide_codes"][0] == {"stuck_code": 1023, "skipped": 11, "ratio": pytest.approx(11.5, abs=0.1),
                                    "wide": True, "representable": True, "slot": 3, "value_q4": 88}
    assert (doc["swept_lo_counts"]["value"], doc["swept_hi_counts"]["value"]) == (lo, hi)
    assert doc["median_rows_per_code"]["value"] == 234
    assert wc.slots_of_record(KEY) == bytes(doc["adc_wide_mid_q4"])


# --- the install ---

ADDR = 0x1F0
SLOTS_OF_RECORD = bytes([0, 0, 0, 88, 16, 0, 0, 80, 16, 48, 0, 0, 0, 0, 0, 0])


class FakeBus:
    """The servo's control table behind `osc`: `get` by name prints the
    descriptor's line, or the tool's error when the image has no field;
    writes land in the table, `flip` xor-ed into the first byte read back."""

    def __init__(self, torque="off", width=16, has_field=True, flip=0):
        self.torque, self.width, self.has_field, self.flip = torque, width, has_field, flip
        self.table = bytearray(0x400)
        self.log = []

    def get(self, field):
        self.log.append(("get", field))
        return {"torque_enable": self.torque}[field]

    def run(self, *args, capture=False, check=True, **kw):
        self.log.append(("run", *args))
        assert args[:2] == ("get", wc.FIELD) and capture and not check
        if not self.has_field:
            return "note: descriptor 0.1 is fw 0.1, servo reports fw 0.2; fields added since are unnamed\n"
        raw = " ".join(f"{b:02x}" for b in self.table[ADDR:ADDR + self.width])
        return f"{wc.FIELD} = {raw.replace(' ', '')}  (addr {ADDR:#06x}, {self.width} B, raw {raw})\n"

    def write(self, addr, data):
        self.log.append(("write", addr, bytes(data)))
        self.table[addr:addr + len(data)] = data

    def read(self, addr, n):
        self.log.append(("read", addr, n))
        out = bytearray(self.table[addr:addr + n])
        out[0] ^= self.flip
        return bytes(out)

    def writes(self):
        return [c for c in self.log if c[0] == "write"]


@pytest.fixture
def wide_bus(monkeypatch, tmp_path):
    path = tmp_path / "wide.json"
    path.write_text(json.dumps({"capture": "x", wc.FIELD: list(SLOTS_OF_RECORD)}))
    monkeypatch.setattr(wc, "result_file", lambda board: path if board == KEY else tmp_path / "none.json")

    def make(**kw):
        b = FakeBus(**kw)
        for name in ("get", "run", "write", "read"):
            monkeypatch.setattr(wc.osc, name, getattr(b, name))
        return b
    make.path = path
    return make


def test_install_writes_the_slots_at_the_descriptors_address_and_reads_them_back(wide_bus):
    b = wide_bus()
    b.table[ADDR:ADDR + 16] = bytes(range(16))
    data, old = wc.install(KEY)
    assert data == SLOTS_OF_RECORD and old == bytes(range(16))
    assert b.writes() == [("write", ADDR, SLOTS_OF_RECORD)]
    assert b.log[-1] == ("read", ADDR, 16)
    assert not any(c[0] == "run" and c[1] in ("save", "stamp") for c in b.log)


@pytest.mark.parametrize("kw, match", [
    ({"torque": "on"}, "torque_enable reads on"),
    ({"has_field": False}, "names no adc_wide_mid_q4"),
    ({"width": 8}, "is 8 B on this servo"),
])
def test_install_refuses_before_anything_is_written(wide_bus, kw, match):
    b = wide_bus(**kw)
    with pytest.raises(wc.settings.Refused, match=match):
        wc.install(KEY)
    assert b.writes() == []


def test_install_refuses_when_an_older_image_rejects_the_write(wide_bus, monkeypatch):
    wide_bus()

    def rejected(addr, data):
        raise subprocess.CalledProcessError(1, ["osc", "write"])
    monkeypatch.setattr(wc.osc, "write", rejected)
    with pytest.raises(wc.settings.Refused, match="refused the write of adc_wide_mid_q4 at 0x1f0"):
        wc.install(KEY)


def test_install_refuses_a_read_back_that_differs(wide_bus):
    wide_bus(flip=0x01)
    with pytest.raises(wc.settings.Refused, match="reads back 01000058"):
        wc.install(KEY)


@pytest.mark.parametrize("doc, match", [
    (None, "no ADC wide-code result for board"),
    ({wc.FIELD: [0] * 15}, "is not 16 values"),
    ({wc.FIELD: [0] * 15 + [256]}, "is not 16 values"),
])
def test_install_refuses_a_result_without_sixteen_slots(wide_bus, doc, match):
    b = wide_bus()
    if doc is None:
        wide_bus.path.unlink()
    else:
        wide_bus.path.write_text(json.dumps(doc))
    with pytest.raises(wc.settings.Refused, match=match):
        wc.install(KEY)
    assert b.log == []


def test_the_slots_read_back_off_the_servos_own_descriptor(wide_bus):
    b = wide_bus()
    b.table[ADDR:ADDR + 16] = SLOTS_OF_RECORD
    assert wc.on_servo() == SLOTS_OF_RECORD
    assert b.log == [("run", "get", wc.FIELD)]
    wide_bus(has_field=False)
    assert wc.on_servo() is None


def test_osc_field_reads_the_descriptors_place_off_get(monkeypatch):
    seen = []
    line = "adc_wide_mid_q4 = 0000  (addr 0x01f0, 2 B, raw 00 58)\n"
    monkeypatch.setattr(wc.osc, "run", lambda *a, **kw: seen.append((a, kw)) or line)
    assert wc.osc.field("adc_wide_mid_q4") == (0x1F0, 2, bytes([0, 0x58]))
    assert seen == [(("get", "adc_wide_mid_q4"), {"capture": True, "check": False})]
    monkeypatch.setattr(wc.osc, "run", lambda *a, **kw: "")
    assert wc.osc.field("adc_wide_mid_q4") is None


# --- the scorecard's flag ---

@pytest.fixture
def rung_bench(monkeypatch, tmp_path):
    """The fake servo of the scorecard's tests, a wide-code result on disk
    and every settings, table, slots and reboot call logged in order."""
    state = {"shaft": 1950.0, "log": [], "calls": []}
    monkeypatch.setattr(sc.osc, "bus_baud", lambda: 3_000_000)
    synth.Servo().stub(monkeypatch, lambda: {"torque_enable": "off", "pos_min_soft_counts": 314,
                                             "pos_max_soft_counts": 3560})
    monkeypatch.setattr(sc.bench, "read_vsys", lambda board=None: 5.05)

    def track(out, mode, schedule, **kw):
        state["log"].append(Path(out).name)
        state["shaft"] = synth.fake_track(out, schedule, state["shaft"])
        return out

    monkeypatch.setattr(sc.osc, "track", track)
    monkeypatch.setattr(sc, "rung_dir", lambda pair, label: tmp_path / "telemetry" / pair / "scorecard" / label)
    path = tmp_path / "wide.json"
    path.write_text(json.dumps({"capture": "x", wc.FIELD: list(SLOTS_OF_RECORD)}))
    monkeypatch.setattr(wc, "result_file", lambda board: path if board == KEY else tmp_path / "none.json")
    calls = state["calls"]
    monkeypatch.setattr(sc.settings, "apply",
                        lambda fields, note="", stamp=True: calls.append(("apply", dict(fields), stamp)))
    monkeypatch.setattr(sc.postable, "install", lambda pair: calls.append(("table", pair)))
    monkeypatch.setattr(sc.settings, "stamp_live", lambda: calls.append(("stamp",)))
    monkeypatch.setattr(sc.widecodes, "install", lambda board: calls.append(("wide", board)))
    monkeypatch.setattr(sc.settings, "restore", lambda: calls.append(("restore", len(state["log"]))))
    state["path"] = path
    return state


def test_a_wide_codes_rung_installs_last_records_the_file_and_reboots(rung_bench, tmp_path):
    f = tmp_path / "gains.json"
    f.write_text(json.dumps({"fields": {"p_kp_q88": 5026}}))
    rung = sc.run("04-wide", "abc123", "f" * 40, runs=1, settings_file=f, pos_table=True, wide_codes=True,
                  servo=SERVO, board=BOARD)
    assert rung_bench["calls"] == [("apply", {"p_kp_q88": 5026}, False), ("table", PAIR), ("stamp",),
                                   ("wide", KEY), ("restore", 3)]
    doc = json.loads((rung / "rung.json").read_text())
    assert doc["wide_codes"] == {"file": str(rung_bench["path"]),
                                 "sha1": hashlib.sha1(rung_bench["path"].read_bytes()).hexdigest()}


def test_a_wide_codes_rung_alone_installs_and_reboots_and_a_plain_rung_names_none(rung_bench):
    sc.run("04-wide", "abc123", "f" * 40, runs=1, wide_codes=True, servo=SERVO, board=BOARD)
    assert rung_bench["calls"] == [("wide", KEY), ("restore", 3)]
    rung = sc.run("00-plain", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    assert json.loads((rung / "rung.json").read_text())["wide_codes"] is None
    assert rung_bench["calls"] == [("wide", KEY), ("restore", 3)]


def test_a_wide_codes_rung_without_a_result_is_refused_before_anything_moves(rung_bench):
    rung_bench["path"].unlink()
    with pytest.raises(SystemExit, match="--wide-codes: no ADC wide-code result for board"):
        sc.run("04-wide", "abc123", "f" * 40, runs=1, wide_codes=True, servo=SERVO, board=BOARD)
    assert rung_bench["log"] == [] and rung_bench["calls"] == []


def test_ab_installs_a_sides_slots_around_its_own_runs_only(rung_bench, monkeypatch, tmp_path):
    image = tmp_path / "a.bin"
    image.write_bytes(b"a" * 64)
    monkeypatch.setattr(sc.osc, "flash", lambda image: rung_bench["calls"].append(("flash",)))
    monkeypatch.setattr(sc.osc, "status", lambda: "      data  clean\n      stamp 0x9c45 = live set\n")
    monkeypatch.setattr(sc.settings, "await_boot", lambda: None)
    a = {"label": "05-a", "fw": "abc123", "image": image}
    b = {"label": "05-b", "fw": "abc123", "image": image, "wide_codes": True}
    ra, rb = sc.ab(a, b, rounds=2, servo=SERVO, board=BOARD)
    assert rung_bench["calls"] == [("flash",), ("wide", KEY), ("restore", 6), ("wide", KEY), ("restore", 12)]
    assert json.loads((ra / "rung.json").read_text())["wide_codes"] is None
    assert json.loads((rb / "rung.json").read_text())["wide_codes"]["file"] == str(rung_bench["path"])


def test_the_cli_passes_the_flags(monkeypatch):
    seen = []
    monkeypatch.setattr(sc, "run", lambda *a, **kw: seen.append(a) or Path("x"))
    sc.main(["run", "--label", "l", "--fw", "f", "--image-sha", "s", "--wide-codes"])
    assert seen[-1][-1] is True
    monkeypatch.setattr(sc, "ab", lambda *sides, rounds: seen.append(sides) or (Path("a"), Path("b")))
    sc.main(["ab", "--a-label", "a", "--a-fw", "f", "--a-image", "i", "--b-label", "b", "--b-fw", "f",
             "--b-image", "i", "--b-wide-codes"])
    assert [s["wide_codes"] for s in seen[-1]] == [False, True]
