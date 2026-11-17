"""The constant set of a pair with `osc` stubbed by a fake servo: the
first set adopted from a record of its state, the file's bytes and its
three hashes, what `load` refuses, the set against its input files, the
servo read against the set, the set installed on it (the order of the
writes, the protection gate, the read-back ahead of the stamp and the
reboot after a refusal), and the set built by its rules: what a rule
reads recorded, the encoder, the bands, the fields held, and the one
hash a build moves; the rules of the motor block, the feedforwards and
the observer gains, a hold and its release by a deviation; the rules
of the NTC curve and the current limits, the protection steps a
deviation may move, and what each curve reads on the bench board; the
rules of the winding model's four fields, their one step, and what the
estimate reads under the four held and under the four wanted; the
rules of the screens' times and counts, the supply terms, the choices
in words, the current loop and the records a host keeps, and the rail
walk the undervolt floor stands over; the set of record against the set
a servo stores from before its releases, and the deviations of a bench
session over it."""
import copy
import csv
import dataclasses
import hashlib
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import pytest

from oscnb import constants as cn
from oscnb import boards, ntc, postable, results, scorecard as sc, settings, winding
from oscnb import widecodes as wc

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scorecard_synth as synth  # noqa: E402

PAIR = "synth__board"
SLOTS = bytes([0, 0, 0, 88] + [0] * 12)
NO_POINTS = hashlib.sha1(bytes(512)).hexdigest()
PAGE_3 = b"\x01" + bytes(63)
ADOPTED = {"class": "none", "origin": "servo-state"}


class Servo(synth.Servo):
    """The fake servo with a field of each kind a set tells apart: the
    read-only front and a read-only CALIB field, the bus id, the stamp,
    the slots, protection fields, per-count fields, one that is both."""

    FIELDS = {"model_number": (0x00, 2, "ro", "uint"), "id": (0x10, 1, "rw", "uint"),
              "pos_max_soft_counts": (0x2c, 4, "rw", "uint"), "p_kp_q88": (0x40, 2, "rw", "uint"),
              "pos_deadband_counts": (0x42, 2, "rw", "uint"), "current_limit_counts": (0x48, 2, "rw", "uint"),
              "drive_polarity": (0x4b, 1, "rw", "bool"), "stall_omega_max_cps": (0x50, 2, "rw", "uint"),
              "r_q12": (0x80, 2, "rw", "uint"), "recip_ke_q": (0x82, 2, "rw", "uint"),
              "plant_stamp": (0x8a, 2, "rw", "uint"), wc.FIELD: (0x91, 16, "rw", "bytes"),
              "tick_hz": (0xa2, 2, "ro", "uint"),
              "torque_enable": (0xc0, 1, "rw", "bool"), "pos_lut_state": (0xde, 1, "ro", "uint")}
    STORED = {**synth.Servo.STORED, "id": 1, "pos_max_soft_counts": 3560, "current_limit_counts": 554,
              "stall_omega_max_cps": 500, "recip_ke_q": 11817, "tick_hz": 20000}


OWNED = ["pos_max_soft_counts", "p_kp_q88", "pos_deadband_counts", "current_limit_counts", "drive_polarity",
         "stall_omega_max_cps", "r_q12", "recip_ke_q"]


@pytest.fixture
def servo(monkeypatch):
    return Servo().stub(monkeypatch)


def record(tmp_path, change=None):
    """The servo's state as a scorecard run records it, at a path."""
    state = sc.servo_state()
    if change:
        change(state)
    path = tmp_path / "servo.json"
    path.write_text(json.dumps(state, indent=2) + "\n")
    return path


@pytest.fixture
def adopted(servo, tmp_path):
    """(the set adopted from the fake servo's state, its path)."""
    return cn.adopt(PAIR, record(tmp_path), root=tmp_path)


# --- the file and adopt ---

def test_adopt_reproduces_the_servo_state(servo, tmp_path):
    rec = record(tmp_path)
    state = json.loads(rec.read_text())
    doc, path = cn.adopt(PAIR, rec, root=tmp_path)
    assert path == tmp_path / "stages" / "5-closed-loop" / "calibration" / f"{PAIR}.constants.json"
    # every field a host may write past the bus front, in table order: not the read-only model number
    # and tick rate, not the id, not the stamp, and the slots as the slots
    assert list(doc["fields"]) == OWNED
    assert all(e == {"value": state["fields"][n], **ADOPTED, "protection": cn.protection_of(n),
                     **({"scale": "unknown"} if "scale" in e else {})} for n, e in doc["fields"].items())
    # a protection field says which way protects less, a field of the plant that it is none
    assert doc["fields"]["current_limit_counts"]["protection"] == {"looser": "up"}
    assert doc["fields"]["stall_omega_max_cps"]["protection"] == {"looser": "down"}
    assert doc["fields"]["r_q12"]["protection"] is None
    assert doc["fields"]["drive_polarity"]["value"] == "on" and doc["fields"]["r_q12"]["value"] == 17100
    assert [n for n, e in doc["fields"].items() if e.get("scale") == "unknown"] == [
        "pos_max_soft_counts", "stall_omega_max_cps", "recip_ke_q"]
    assert doc["wide_codes"] == {"slots": list(SLOTS), **ADOPTED}
    assert doc["pos_table"] == {"state": "IDENTITY", "points_sha1": NO_POINTS, **ADOPTED}
    assert doc["origins"] == {"servo-state": {"from": "the servo's state before the path",
                                              "sha1": sc.file_sha1(rec), "state_hash": state["hash"]}}
    assert (doc["format"], doc["pair"], doc["inputs"], doc["previous"]) == (1, PAIR, {}, None)
    assert doc["hash"] == cn.hashes(doc) and len(set(doc["hash"].values())) == 3
    # the same record gives the same bytes, and the file is what load reads and render writes
    _, again = cn.adopt(PAIR, rec, root=tmp_path / "again")
    assert again.read_bytes() == path.read_bytes()
    assert cn.load(path) == doc and cn.render(cn.load(path)) == path.read_text()
    # nothing of the machine: the record is named by its sha1 and in words, not by where it lay
    assert str(tmp_path) not in path.read_text() and "servo.json" not in path.read_text()


def test_the_file_holds_a_field_a_line_in_one_key_order(adopted):
    doc, path = adopted
    lines = path.read_text().splitlines()
    assert ('  "r_q12": {"value": 17100, "class": "none", "origin": "servo-state", "protection": null}'
            in (s.rstrip(",") for s in lines))
    assert ('  "recip_ke_q": {"value": 11817, "class": "none", "origin": "servo-state", "protection": null, '
            '"scale": "unknown"}') in lines
    assert ('  "current_limit_counts": {"value": 554, "class": "none", "origin": "servo-state", '
            '"protection": {"looser": "up"}},') in lines
    # the bytes do not depend on the order a document was put together in
    shuffled = {k: doc[k] for k in reversed(list(doc))}
    shuffled["fields"] = {n: dict(reversed(list(e.items()))) for n, e in doc["fields"].items()}
    assert cn.render(shuffled) == path.read_text()
    with pytest.raises(ValueError, match="format 1 holds no utc"):
        cn.render({**doc, "utc": "now"})


def test_every_field_of_an_adopted_set_is_in_exactly_one_hash(adopted):
    doc, _ = adopted
    plant, protection = cn.hashed(doc)
    assert sorted([*plant["fields"], *protection]) == sorted(doc["fields"])
    assert set(protection) == {"current_limit_counts", "stall_omega_max_cps"} == set(doc["fields"]) & cn.PROTECTION

    def moved(change):
        other = copy.deepcopy(doc)
        change(other)
        return [k for k, v in cn.hashes(other).items() if v != doc["hash"][k]]

    for name in doc["fields"]:
        side = "protection" if name in cn.PROTECTION else "plant"
        assert moved(lambda d: d["fields"][name].update(value=1)) == [side, "set"], name
    # the table and the slots are the plant's
    assert moved(lambda d: d["pos_table"].update(state="LIVE")) == ["plant", "set"]
    assert moved(lambda d: d["pos_table"].update(points_sha1="0" * 40)) == ["plant", "set"]
    assert moved(lambda d: d["wide_codes"].update(slots=[0] * 16)) == ["plant", "set"]
    # where a value came from is in no hash
    assert moved(lambda d: d["origins"]["servo-state"].update(sha1="0" * 40)) == []


@pytest.mark.parametrize("change, match", [
    (lambda d: d["fields"]["r_q12"].update(value=16000), "hashes are not those of its content"),
    (lambda d: d["pos_table"].update(state="LIVE"), "hashes are not those of its content"),
    (lambda d: d["fields"]["r_q12"].update({"class": "guessed"}), "r_q12 holds no value of one class"),
    (lambda d: d["fields"]["r_q12"].pop("value"), "r_q12 holds no value of one class"),
    (lambda d: d["origins"].clear(), "is adopted from 'servo-state', which the file does not hold"),
    (lambda d: d["fields"]["r_q12"].update({"class": "bench"}), "r_q12 is of class bench and names no record"),
    (lambda d: d["fields"]["r_q12"].update({"class": "datasheet", "inputs": ["a-file:r"]}),
     "r_q12 is of class datasheet and names no record or no document"),
    (lambda d: d["fields"]["current_limit_counts"].update(protection=None),
     "current_limit_counts says None of protection, the tool {'looser': 'up'}"),
    (lambda d: d["fields"]["r_q12"].update(protection={"looser": "up"}), "r_q12 says .* of protection, the tool None"),
    (lambda d: d["fields"]["r_q12"].pop("protection"), "r_q12 says nothing of protection, the tool None"),
    (lambda d: d.update(deviation={"file": "deviations/x.json", "sha1": "0" * 40, "fields": {"r_q12": [1, 2]}}),
     "its deviation is not the fields it moved, each as \\[of record, deviated\\]"),
    (lambda d: d.update(format=2), "format 2, this tool reads 1"),
], ids=["a field edited", "the table edited", "a class the format has none of", "no value", "no origin",
        "a bench value with no record", "a datasheet value with no document",
        "a protection field marked as the plant's", "a plant field marked as protection",
        "a field that says nothing of protection", "a deviation that is not what the fields hold",
        "another format"])
def test_load_refuses_a_file_that_is_not_a_set_as_the_tool_writes_one(adopted, change, match):
    doc, path = adopted
    change(doc)
    path.write_text(cn.render(doc))
    with pytest.raises(ValueError, match=match):
        cn.load(path)


def test_a_pair_without_a_set_is_refused(tmp_path):
    with pytest.raises(ValueError, match=f"no constant set for {PAIR}"):
        cn.of_record(PAIR, tmp_path)


def unclean(state):
    state["data_state"] = "STAMP_MISMATCH"
    state["hash"] = sc.state_hash(state)


@pytest.mark.parametrize("change, match", [
    (unclean, "the record's data state is STAMP_MISMATCH"),
    (lambda s: s.update(raw="00" + s["raw"][2:]), "the record's hash is not that of its content"),
    (lambda s: s.pop("writable"), "the record names no writable fields: record the servo's state again"),
], ids=["a state that is not clean", "a record edited", "a record without its writable fields"])
def test_adopt_refuses_a_record_it_cannot_stand_on(servo, tmp_path, change, match):
    with pytest.raises(ValueError, match=match):
        cn.adopt(PAIR, record(tmp_path, change), root=tmp_path)
    assert not cn.set_path(PAIR, tmp_path).exists()


def test_adopt_writes_a_pairs_first_set_only(adopted, tmp_path):
    _, path = adopted
    before = path.read_bytes()
    with pytest.raises(ValueError, match="writes a pair's first set only"):
        cn.adopt(PAIR, tmp_path / "servo.json", root=tmp_path)
    assert path.read_bytes() == before


def test_a_servo_whose_descriptor_names_no_slots_has_none_in_its_set(servo, monkeypatch, tmp_path):
    monkeypatch.setattr(sc.osc, "field", lambda name: None)
    doc, path = cn.adopt(PAIR, record(tmp_path), root=tmp_path)
    assert doc["wide_codes"] is None and cn.load(path) == doc
    assert cn.described(doc)[2] == "wide-code slots: none"


def test_every_set_of_record_loads_as_the_tool_writes_it(adopted):
    # the sets the repo holds, and one written here so the checks run whether or not it holds any
    for path in [*sorted((results.ROOT / cn.FOLDER).glob("*.constants.json")), adopted[1]]:
        text = path.read_text()
        doc = cn.load(path)
        assert cn.render(doc) == text and path.name == f"{doc['pair']}.constants.json"
        assert "/Users/" not in text and "/home/" not in text


# --- status ---

def built(root):
    """A set as a build over results writes one, put together by hand:
    two fields over one result file, one over another, one over the
    registry alone, one adopted."""
    files = {"3-servo/ladder": "stages/3-servo/ladder/results/synth__board.json",
             "1-board/current-scale": "stages/1-board/current-scale/results/board.json"}
    for f in files.values():
        (root / f).parent.mkdir(parents=True)
        (root / f).write_text('{"notes": ""}\n')
    doc = {
        "format": 1, "pair": PAIR,
        "inputs": {**{name: {"file": f, "sha1": sc.file_sha1(root / f)} for name, f in files.items()},
                   "registry": {"servo": {"gear_ratio": 308.05}}},
        "origins": {"servo-state": {"from": "a record", "sha1": "0" * 40, "state_hash": "1" * 64}},
        "wide_codes": None,
        "pos_table": {"state": "IDENTITY", "points_sha1": NO_POINTS, **ADOPTED},
        "fields": {
            "current_limit_counts": {"value": 554, "class": "policy", "rule": "current_limit.class",
                                     "inputs": ["1-board/current-scale:counts_per_a", "registry:gear_class"]},
            "recip_ke_q": {"value": 11817, "class": "measured", "rule": "recip_ke.ladder",
                           "inputs": ["3-servo/ladder:ke_v_s_per_rad", "registry:gear_ratio", "scale"]},
            "ke_vpc_q": {"value": 355, "class": "measured", "rule": "ke_vpc.ladder",
                         "inputs": ["3-servo/ladder:ke_v_s_per_rad"]},
            "gear_ratio_centi": {"value": 30805, "class": "measured", "rule": "gear.registry",
                                 "inputs": ["registry:gear_ratio"]},
            "p_kp_q88": {"value": 5026, **ADOPTED},
        },
        "previous": None,
    }
    for name, e in doc["fields"].items():
        e["protection"] = cn.protection_of(name)
    doc["hash"] = cn.hashes(doc)
    return cn.load(cn.write(doc, cn.set_path(PAIR, root))), files


def test_status_names_a_moved_input_and_its_fields(tmp_path):
    doc, files = built(tmp_path)
    reg = {"servo": {"gear_ratio": 308.05}}
    lines, stale = cn.status(doc, tmp_path, reg)
    assert not stale and lines == [
        "2 input files, none moved",
        "5 fields: 3 measured, 0 bench, 0 datasheet, 1 policy, 0 pinned, 1 none (no source); 0 per-count at an unknown "
        "scale",
        f"position table IDENTITY, points sha1 {NO_POINTS} [none]",
        "wide-code slots: none",
        f"hash: plant {doc['hash']['plant'][:12]}, protection {doc['hash']['protection'][:12]} (1 fields), "
        f"set {doc['hash']['set'][:12]}",
        # the debt: every field with no source, with why it has none and what would give it one
        "no source, 1 fields, 0 of which a maker's value on record could supply:",
        "  p_kp_q88 = 5026: the servo held it before its set existed; needs a rule over a result, a bench record or "
        "a decision",
        f"adopted from servo-state: a record (record sha1 {'0' * 40}, state hash {'1' * 12})",
    ]
    rest = lines[1:]
    ladder = tmp_path / files["3-servo/ladder"]
    ladder.write_text('{"notes": "re-run"}\n')
    lines, stale = cn.status(doc, tmp_path, reg)
    assert stale and lines == [
        f"stale: 3-servo/ladder ({files['3-servo/ladder']}) is no longer the file the set read, sha1 "
        f"{sc.file_sha1(ladder)}: recip_ke_q, ke_vpc_q", *rest]
    # a file that is gone is named too, the inputs in name order
    (tmp_path / files["1-board/current-scale"]).unlink()
    lines, stale = cn.status(doc, tmp_path, reg)
    assert stale and lines[:2] == [
        f"stale: 1-board/current-scale ({files['1-board/current-scale']}) is gone: current_limit_counts",
        f"stale: 3-servo/ladder ({files['3-servo/ladder']}) is no longer the file the set read, sha1 "
        f"{sc.file_sha1(ladder)}: recip_ke_q, ke_vpc_q"]
    assert cn.moved(doc, tmp_path)[0][2] is None
    # a registry value the set read that the registry no longer holds, or holds no entry for
    lines, stale = cn.status(doc, tmp_path, {"servo": {"gear_ratio": 310.0}})
    assert stale and lines[2] == "stale: the registry's servo gear_ratio was 308.05, it is 310.0: " \
                                 "recip_ke_q, gear_ratio_centi"
    assert cn.registry_moved(doc) == [("servo", "gear_ratio", 308.05, None)]


def test_an_adopted_set_has_no_input_to_move(adopted, tmp_path):
    doc, _ = adopted
    lines, stale = cn.status(doc, tmp_path)
    o = doc["origins"]["servo-state"]
    assert not stale and lines[0] == "0 input files, none moved"
    assert lines[1] == "8 fields: 0 measured, 0 bench, 0 datasheet, 0 policy, 0 pinned, 8 none (no source); " \
                       "3 per-count at an unknown scale"
    assert lines[5] == "no source, 8 fields, 0 of which a maker's value on record could supply:" and len(lines) == 15
    assert lines[-1] == (f"adopted from servo-state: the servo's state before the path (record sha1 {o['sha1']}, "
                         f"state hash {o['state_hash'][:12]})")


# --- check ---

def test_check_is_clean_on_the_adopted_servo(adopted, servo, capsys):
    doc, _ = adopted
    servo.log.clear()
    assert cn.check(doc) == []
    h = doc["hash"]
    assert capsys.readouterr().out.splitlines() == [
        "8 fields: 0 measured, 0 bench, 0 datasheet, 0 policy, 0 pinned, 8 none (no source); 3 per-count at an "
        "unknown scale",
        f"position table IDENTITY, points sha1 {NO_POINTS} [none]",
        f"wide-code slots {SLOTS.hex(' ')} [none]",
        f"hash: plant {h['plant'][:12]}, protection {h['protection'][:12]} (2 fields), set {h['set'][:12]}",
        "in no set (the bus front and the stamp): id = 1, plant_stamp = 40005",
    ]
    # it reads only: the state's own 21 calls, every write a FETCH of a table page into the window
    assert len(servo.log) == 21
    assert all(c[0] in ("dump", "read", "get", "field", "status")
               or (c[0] == "write" and c[1:] == (postable.WINDOW_ADDR, bytes([c[2][0], postable.FETCH])))
               for c in servo.log)


def test_check_names_a_changed_field_the_table_and_the_slots(adopted, servo, capsys):
    doc, _ = adopted
    servo.set("p_kp_q88", 5026)
    servo.set("current_limit_counts", 600)
    servo.set("drive_polarity", 0)
    servo.set("pos_lut_state", 2)
    servo.pages[3] = PAGE_3
    servo.set(wc.FIELD, bytes(16))
    points = hashlib.sha1(bytes(3 * 64) + PAGE_3 + bytes(4 * 64)).hexdigest()
    named = [
        "p_kp_q88: servo 5026, set 40212 [none]",
        "current_limit_counts: servo 600, set 554 [none]",
        "drive_polarity: servo off, set on [none]",
        "position table state: servo LIVE, set IDENTITY [none]",
        f"position table points sha1: servo {points}, set {NO_POINTS} [none]",
        f"wide-code slots: servo {bytes(16).hex(' ')}, set {SLOTS.hex(' ')} [none]",
    ]
    assert cn.check(doc) == named
    assert capsys.readouterr().out.splitlines()[:len(named)] == named


def test_a_difference_names_the_fields_class_and_rule_and_a_field_the_servo_lacks(adopted, servo):
    doc, _ = adopted
    doc["fields"]["r_q12"] = {"value": 4541, "class": "measured", "rule": "r.rails"}
    doc["fields"]["ke_vpc_q"] = {"value": 355, **ADOPTED}
    assert cn.differences(doc, sc.servo_state()) == [
        "r_q12: servo 17100, set 4541 [measured, r.rails]",
        "ke_vpc_q: servo names no such field, set 355 [none]",
    ]


def test_check_names_what_the_servo_holds_outside_the_set(adopted, servo, monkeypatch):
    doc, _ = adopted
    del doc["fields"]["r_q12"]
    assert cn.unowned(doc, sc.servo_state()) == [
        "not in the set: r_q12 = 17100",
        "in no set (the bus front and the stamp): id = 1, plant_stamp = 40005",
    ]
    # a servo whose descriptor names no slots against a set that holds them
    monkeypatch.setattr(sc.osc, "field", lambda name: None)
    assert cn.differences(doc, sc.servo_state()) == [
        f"wide-code slots: servo names none, set {SLOTS.hex(' ')} [none]"]


def test_check_refuses_a_data_state_that_is_not_clean(adopted, servo, capsys):
    doc, _ = adopted
    servo.data, servo.stamp = "STAMP_MISMATCH", "0x9c45 != live set 0x1234"
    with pytest.raises(settings.Refused, match="data state STAMP_MISMATCH, not clean"):
        cn.check(doc)
    assert capsys.readouterr().out.splitlines()[0].startswith("8 fields: ")
    # what differs is named ahead of the refusal
    servo.set("r_q12", 16000)
    with pytest.raises(settings.Refused, match="data state STAMP_MISMATCH, not clean"):
        cn.check(doc)
    assert capsys.readouterr().out.splitlines()[0] == "r_q12: servo 16000, set 17100 [none]"


# --- the command line ---

@pytest.fixture
def cli(servo, monkeypatch, tmp_path):
    """The verbs run as commands for PAIR, the sets under tmp_path."""
    monkeypatch.setattr(cn, "set_path", lambda pair, root=None: tmp_path / f"{pair}.constants.json")
    return lambda *args: cn.main(["--pair", PAIR, *args])


def test_the_verbs_as_commands(cli, servo, tmp_path, capsys):
    with pytest.raises(SystemExit, match=f"refused: no constant set for {PAIR}"):
        cli("check")
    cli("adopt", "--from", str(record(tmp_path)), "--origin", "the bench servo as stored")
    out = capsys.readouterr().out.splitlines()
    assert out[0].endswith(f"{PAIR}.constants.json") and out[0].startswith("wrote ")
    assert out[1].startswith("8 fields: ") and len(out) == 5
    doc = cn.load(tmp_path / f"{PAIR}.constants.json")
    assert doc["origins"]["servo-state"]["from"] == "the bench servo as stored"
    cli("status")
    cli("check")
    assert capsys.readouterr().out.splitlines()[0] == "0 input files, none moved"
    # a difference is exit 1, a data state that is not clean a refusal
    servo.set("r_q12", 16000)
    with pytest.raises(SystemExit) as e:
        cli("check")
    assert e.value.code == 1
    servo.data = "STAMP_MISMATCH"
    with pytest.raises(SystemExit, match="refused: data state STAMP_MISMATCH, not clean"):
        cli("check")
    with pytest.raises(SystemExit, match="refused: .*writes a pair's first set only"):
        cli("adopt", "--from", str(tmp_path / "servo.json"))


def test_status_of_a_stale_set_exits_1(cli, tmp_path, monkeypatch, capsys):
    cli("adopt", "--from", str(record(tmp_path)))
    monkeypatch.setattr(cn, "status", lambda doc, root: (["stale: an input"], True))
    with pytest.raises(SystemExit) as e:
        cli("status")
    assert e.value.code == 1 and capsys.readouterr().out.splitlines()[-1] == "stale: an input"


# --- install ---

Bench = synth.Bench


POINTS = [0] * 100 + [3] + [0] * 155
NEW_STOPS = (90, 3780)
NEW_SLOTS = bytes([0, 0, 0, 48] + [0] * 12)
TARGET = {"pos_min_soft_counts": 3600, "pos_max_soft_counts": 3700, "p_kp_q88": 5026, "current_limit_counts": 500,
          "stall_response": 1, "r_q12": 16000, "raw_min": NEW_STOPS[0], "raw_max": NEW_STOPS[1]}


def table_file(root, points=POINTS, stops=NEW_STOPS, fitted=NEW_SLOTS):
    """The pair's table file as `postable.load_file` reads one, its
    lookup fitted on the slots `fitted`; None for a file written before
    table files named them."""
    path = cn.set_path(PAIR, root).with_name(f"{PAIR}.pos_lut.json")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({
        "inputs": {"stops": {"raw_min": stops[0], "raw_max": stops[1]},
                   **({} if fitted is None else {"fitted_on_slots_sha1": postable.slots_sha1(fitted)})},
        "target": {"theta_at_stops_deg": [-90.0, 90.0]},
        "points_by_page": [points[32 * p:32 * (p + 1)] for p in range(postable.PAGES)]}))
    return path


@pytest.fixture
def bench(monkeypatch):
    return Bench().stub(monkeypatch)


@pytest.fixture
def stored(bench, tmp_path):
    """The set adopted from the fake servo as it boots: an identity table."""
    doc, _ = cn.adopt(PAIR, record(tmp_path), root=tmp_path)
    bench.log.clear()
    return doc


@pytest.fixture
def target(bench, tmp_path):
    """A set that differs from the servo as it boots in the slots, the
    stops, the table and six fields, two of them protection fields: the
    set adopted from the servo brought to that state by hand, the pair's
    table file beside it, and the servo rebooted off it."""
    for name, v in TARGET.items():
        bench.set(name, v)
    bench.set(wc.FIELD, NEW_SLOTS)
    bench.pages = [postable.page_bytes(POINTS, p) for p in range(postable.PAGES)]
    bench.set("pos_lut_state", postable.STATES.index("LIVE"))
    doc, _ = cn.adopt(PAIR, record(tmp_path), root=tmp_path)
    table_file(tmp_path)
    bench.reboot()
    bench.log.clear()
    return doc


def deviated(doc, **values):
    """`doc` with other values in the named fields, as a set of its own."""
    other = copy.deepcopy(doc)
    for name, v in values.items():
        other["fields"][name]["value"] = v
    return {**other, "hash": cn.hashes(other)}


NAMED = {"protection": ["current_limit_counts", "stall_response"], "loosen": ["stall_response"]}


def test_install_from_blank_equals_the_set(target, bench, tmp_path, capsys):
    assert len(cn.differences(target, sc.servo_state())) == 11
    bench.log.clear()
    wrote = cn.install(target, **NAMED, root=tmp_path)
    # the order a count is defined in, then the rest in table order; the low soft limit, refused while
    # the high one stood under it, once more after the rest
    assert wrote == ["wide-code slots", "raw_min", "raw_max", "position table", "pos_max_soft_counts", "p_kp_q88",
                     "current_limit_counts", "stall_response", "r_q12", "pos_min_soft_counts", "stamp"]
    changed = bench.wrote()
    assert changed[0] == ("write", 0x91, NEW_SLOTS)
    assert changed[1:3] == [("set", "raw_min", "90"), ("set", "raw_max", "3780")]
    # the table by postable's own steps: 8 STOREs, the COMMIT, and every page fetched back
    stores = [postable.page_bytes(POINTS, p) for p in range(postable.PAGES)]
    assert changed[3:12] == [*(("write", postable.WINDOW_ADDR, bytes([p, postable.STORE]) + stores[p])
                              for p in range(postable.PAGES)),
                             ("write", postable.WINDOW_ADDR, bytes([0, postable.COMMIT]))]
    assert changed[12:] == [("set", "pos_min_soft_counts", "3600"), ("set", "pos_max_soft_counts", "3700"),
                            ("set", "p_kp_q88", "5026"), ("set", "current_limit_counts", "500"),
                            ("set", "stall_response", "Yield"), ("set", "r_q12", "16000"),
                            ("set", "pos_min_soft_counts", "3600"), ("run", "stamp")]
    state = sc.servo_state()
    assert cn.differences(target, state) == []
    assert (state["data_state"], state["stamp"]) == ("clean", "0x1234 = live set")
    assert state["pos_table"]["state"] == "LIVE" and state["pos_table"]["points_sha1"] == cn.points_sha1(POINTS)
    out = capsys.readouterr().out.splitlines()
    assert out[0] == f"wide-code slots: {SLOTS.hex(' ')} -> {NEW_SLOTS.hex(' ')}"
    assert "stall_response: Fault (0) -> Yield (1)" in out
    assert "position table: IDENTITY -> LIVE, 8 pages read back" in out
    assert out[-1] == "stamped: data state clean, stamp 0x1234 = live set"
    assert cn.check(target) == []
    # the servo now holds the set: the same install, unnamed, writes nothing
    bench.log.clear()
    assert cn.install(target, root=tmp_path) == [] and bench.wrote() == []


def test_install_on_an_equal_servo_writes_nothing(stored, bench, tmp_path):
    assert cn.install(stored, root=tmp_path) == [] and bench.wrote() == []
    # it reads the torque and the state, and no more
    assert bench.log[0] == ("get", "torque_enable") and len(bench.log) == 22
    # a set that differs in two fields is two writes, the read-back and the stamp
    two = deviated(stored, p_kp_q88=4000, r_q12=15000)
    assert cn.install(two, root=tmp_path) == ["p_kp_q88", "r_q12", "stamp"]
    assert bench.wrote() == [("set", "p_kp_q88", "4000"), ("set", "r_q12", "15000"), ("run", "stamp")]
    bench.log.clear()
    assert cn.install(two, root=tmp_path) == [] and bench.wrote() == []
    # an equal servo whose data state is not clean is stamped, nothing else written
    bench.data = "STAMP_MISMATCH"
    assert cn.install(two, root=tmp_path) == ["stamp"] and bench.wrote() == [("run", "stamp")]


def test_a_protection_change_needs_its_name(stored, bench, tmp_path):
    tighter = deviated(stored, current_limit_counts=500, p_kp_q88=5026)
    with pytest.raises(settings.Refused) as e:
        cn.install(tighter, root=tmp_path)
    assert str(e.value) == ("the protection gate:\n  current_limit_counts: servo 554, set 500, protects more "
                            "(less is up): name it with --protection current_limit_counts")
    # nothing was written, the plant field beside it neither, and nothing rebooted
    assert bench.wrote() == []
    # a name that is no protection field of the set is refused too
    with pytest.raises(settings.Refused, match="p_kp_q88 is not a protection field of the set"):
        cn.install(tighter, protection=["current_limit_counts", "p_kp_q88"], root=tmp_path)
    with pytest.raises(settings.Refused, match="cutoff_cc is not a protection field of the set"):
        cn.install(tighter, protection=["current_limit_counts", "cutoff_cc"], root=tmp_path)
    assert bench.wrote() == []
    assert cn.install(tighter, protection=["current_limit_counts"], root=tmp_path) == [
        "p_kp_q88", "current_limit_counts", "stamp"]
    # named and already there is no refusal: the same command runs twice
    assert cn.install(tighter, protection=["current_limit_counts"], root=tmp_path) == []


def test_a_looser_limit_needs_loosen(stored, bench, tmp_path):
    higher = deviated(stored, current_limit_counts=600)
    line = "current_limit_counts: servo 554, set 600, protects less (less is up): name it with "
    for names, need in (({}, "--protection current_limit_counts and --loosen current_limit_counts"),
                        ({"protection": ["current_limit_counts"]}, "--loosen current_limit_counts")):
        with pytest.raises(settings.Refused) as e:
            cn.install(higher, **names, root=tmp_path)
        assert str(e.value) == f"the protection gate:\n  {line}{need}"
    # the second name does not stand in for the first
    with pytest.raises(settings.Refused, match="named to loosen and not as a protection change"):
        cn.install(higher, loosen=["current_limit_counts"], root=tmp_path)
    # a field that protects less downward, and one no direction says it for
    slower = deviated(stored, stall_omega_max_cps=400, stall_response="Yield (1)")
    with pytest.raises(settings.Refused) as e:
        cn.install(slower, protection=["stall_omega_max_cps", "stall_response"], root=tmp_path)
    assert str(e.value).splitlines()[1:] == [
        "  stall_response: servo Fault (0), set Yield (1), judged, taken as protecting less: name it with "
        "--loosen stall_response",
        "  stall_omega_max_cps: servo 500, set 400, protects less (less is down): name it with "
        "--loosen stall_omega_max_cps"]
    assert bench.wrote() == []
    assert cn.install(higher, protection=["current_limit_counts"], loosen=["current_limit_counts"],
                      root=tmp_path) == ["current_limit_counts", "stamp"]
    assert bench.decoded("current_limit_counts") == 600


@pytest.mark.parametrize("name, on_servo, wanted, less", [
    ("current_limit_counts", 554, 600, True), ("current_limit_counts", 554, 500, False),
    ("stall_omega_max_cps", 500, 400, True), ("stall_omega_max_cps", 500, 600, False),
    ("th_rise_q016", 1143, 494, True), ("v_undervolt_counts", 720, 800, False),
    ("stall_response", "Fault (0)", "Yield (1)", True), ("stall_response", "Yield (1)", "Fault (0)", True),
    ("i_cont_counts", 277, 0, True), ("i_cont_counts", 277, 200, True), ("rail_fold_q8", 1430, 0, True),
    ("ntc_k1_q88", 563, 212, True), ("cutoff_cc", 10000, "9000", True), ("cutoff_cc", 10000, 9000, False),
    ("derate_start_cc", 8000, 9000, True), ("speed_ceiling_cps", 8051, 8244, True),
    ("speed_ceiling_cps", 8051, 4025, False),
])
def test_looser_reads_a_direction_and_takes_what_it_cannot_read_as_less(name, on_servo, wanted, less):
    assert cn.looser(name, on_servo, wanted) is less


def test_every_protection_field_has_one_way_to_protect_less():
    assert cn.PROTECTION == {*cn.LOOSER, *cn.RETIRED} and set(cn.LOOSER.values()) == {"up", "down", "judged"}
    assert len(cn.LOOSER) == 26 and not cn.PROTECTION & {*cn.COMMS, cn.STAMP, wc.FIELD}
    # a field the firmware no longer names keeps the way it had, for the sets written while it stood: it is
    # no field of the table now, and nothing binds it
    assert cn.RETIRED == {"derate_start_cc": "up"} and not set(cn.RETIRED) & {*cn.LOOSER, *cn.RULES}


def test_a_refusal_after_the_first_write_reboots(stored, bench, tmp_path):
    two = deviated(stored, p_kp_q88=5026, r_q12=16000)
    bench.drop = {"r_q12"}
    with pytest.raises(settings.Refused) as e:
        cn.install(two, root=tmp_path)
    assert str(e.value) == "r_q12: wrote 16000, read back 17100; rebooted: the servo is back on its saved state"
    assert bench.wrote() == [("set", "p_kp_q88", "5026"), ("set", "r_q12", "16000"), ("run", "reboot")]
    assert bench.decoded("p_kp_q88") == 40212 and bench.data == "clean"
    assert cn.differences(stored, sc.servo_state()) == []
    # a write the servo refuses twice, after one that landed
    bench.log.clear()
    with pytest.raises(settings.Refused) as e:
        cn.install(deviated(stored, pos_min_soft_counts=3600, p_kp_q88=5026), root=tmp_path)
    assert str(e.value) == ("pos_min_soft_counts: the servo refused 3600; rebooted: the servo is back on its "
                            "saved state")
    assert bench.wrote() == [("set", "pos_min_soft_counts", "3600"), ("set", "p_kp_q88", "5026"),
                             ("set", "pos_min_soft_counts", "3600"), ("run", "reboot")]
    # a refused write with nothing landed before it reboots nothing
    bench.log.clear()
    with pytest.raises(settings.Refused) as e:
        cn.install(deviated(stored, pos_min_soft_counts=3600), root=tmp_path)
    assert str(e.value) == "pos_min_soft_counts: the servo refused 3600"
    assert bench.wrote() == [("set", "pos_min_soft_counts", "3600")] * 2


def test_a_reboot_that_does_not_answer_is_said(stored, bench, tmp_path, monkeypatch):
    bench.drop = {"r_q12"}

    def silent():
        raise RuntimeError("no servo answers")

    monkeypatch.setattr(sc.osc, "bus_baud", silent)
    with pytest.raises(settings.Refused, match="read back 17100; the reboot did not answer: the servo holds a "
                                               "half-written set until it is power cycled"):
        cn.install(deviated(stored, r_q12=16000), root=tmp_path)


def test_the_stamp_is_written_only_after_the_read_back_matches(stored, bench, tmp_path):
    two = deviated(stored, p_kp_q88=5026, r_q12=16000)
    cn.install(two, root=tmp_path)
    kinds = [c[0] if c[0] != "run" else c[1] for c in bench.log]
    # every write, then the state whole, then the stamp, then the status that must read clean
    assert kinds.index("dump", kinds.index("set")) > max(i for i, k in enumerate(kinds) if k == "set")
    back = max(i for i, k in enumerate(kinds) if k == "dump")
    assert kinds[back + 21:] == ["stamp", "status"] and kinds.count("stamp") == 1
    assert bench.data == "clean"
    # a write that lands and reads back, while the state whole is not the set: no stamp, a reboot
    bench.reboot()
    bench.log.clear()
    bench.drift = {"r_q12": ("p_kp_q88", 1)}
    with pytest.raises(settings.Refused) as e:
        cn.install(two, root=tmp_path)
    assert str(e.value) == ("the state read back is not the set: p_kp_q88: servo 1, set 5026 [none]; rebooted: "
                            "the servo is back on its saved state")
    assert [c for c in bench.wrote() if c[0] == "run"] == [("run", "reboot")]


def stale(doc, root):
    doc["inputs"]["3-servo/ladder"] = {"file": "stages/3-servo/ladder/results/gone.json", "sha1": "0" * 40}


def unnamed(doc, root):
    doc["fields"]["ke_vpc_q"] = {"value": 355, **ADOPTED}
    doc["fields"]["tick_hz"] = {"value": 20000, **ADOPTED}


@pytest.mark.parametrize("change, match", [
    (lambda doc, root: None, "torque_enable reads on: a set is installed with torque off"),
    (stale, "the set is stale, 3-servo/ladder moved"),
    (unnamed, "the servo's descriptor lets a host write no ke_vpc_q, tick_hz"),
    (lambda doc, root: doc["pos_table"].update(state="REJECT_ENDS"),
     "the set's table is REJECT_ENDS .* and the servo's IDENTITY .*: an install writes a LIVE or an IDENTITY "
     "table only"),
    (lambda doc, root: table_file(root).unlink(), "the set's table is LIVE and the pair has no table file"),
    (lambda doc, root: table_file(root, [0] * 256), "holds points of sha1 .*, the set's table is"),
    (lambda doc, root: table_file(root, stops=(82, 3792)),
     "was built against stops 82 and 3792, the set's are 90 and 3780"),
], ids=["torque on", "a stale set", "fields the servo has none of or lets no host write",
        "a table of a state no host writes", "no table file", "a table file of other points",
        "a table built against other stops"])
def test_install_refuses_before_anything_is_written(target, bench, tmp_path, monkeypatch, change, match):
    if "torque" in match:
        bench.set("torque_enable", 1)
    change(target, tmp_path)
    with pytest.raises(settings.Refused, match=match):
        cn.install(target, **NAMED, root=tmp_path)
    assert bench.wrote() == []


def identity(doc):
    """`doc` with the identity for its table, as a set of its own."""
    other = copy.deepcopy(doc)
    other["pos_table"].update(state="IDENTITY", points_sha1=NO_POINTS)
    return {**other, "hash": cn.hashes(other)}


def test_install_returns_a_live_table_to_the_identity(target, bench, tmp_path, capsys):
    cn.install(target, **NAMED, root=tmp_path)
    bench.log.clear()
    capsys.readouterr()
    # all-zero points against the set's stops, stored a page at a time and committed, each page read back
    assert cn.install(identity(target), root=tmp_path) == ["position table", "stamp"]
    zero = bytes(2 * postable.PAGE_POINTS)
    assert bench.wrote() == [*(("write", postable.WINDOW_ADDR, bytes([p, postable.STORE]) + zero)
                               for p in range(postable.PAGES)),
                             ("write", postable.WINDOW_ADDR, bytes([0, postable.COMMIT])), ("run", "stamp")]
    assert capsys.readouterr().out.splitlines()[0] == "position table: LIVE -> IDENTITY, 8 pages read back"
    state = sc.servo_state()
    assert state["pos_table"]["state"] == "IDENTITY" and state["pos_table"]["points_sha1"] == NO_POINTS
    assert cn.differences(identity(target), state) == [] and state["data_state"] == "clean"


def test_a_servo_that_lands_an_all_zero_table_live_is_refused_as_that(target, bench, tmp_path):
    cn.install(target, **NAMED, root=tmp_path)
    bench.zero_live = True
    bench.log.clear()
    with pytest.raises(settings.Refused) as e:
        cn.install(identity(target), root=tmp_path)
    assert str(e.value) == ("the COMMIT of an all-zero table landed LIVE, not IDENTITY: the servo's firmware lands "
                            "an all-zero table LIVE, and only a boot returns its table to IDENTITY; rebooted: the "
                            "servo is back on its saved state")
    # nothing is read back or stamped past the COMMIT: the reboot is the next thing the servo is told
    assert bench.wrote()[-2:] == [("write", postable.WINDOW_ADDR, bytes([0, postable.COMMIT])), ("run", "reboot")]


def test_a_table_fitted_on_other_slots_is_refused(target, bench, tmp_path, capsys):
    path = table_file(tmp_path, fitted=SLOTS)
    with pytest.raises(settings.Refused) as e:
        cn.install(target, **NAMED, root=tmp_path)
    assert str(e.value) == (f"the lookup was fitted on other slots (sha1 {postable.slots_sha1(SLOTS)}, the set's "
                            f"are {postable.slots_sha1(NEW_SLOTS)}): re-run stage 4 position, then `postable build`")
    assert bench.wrote() == []
    # a table file that names no slots is said, as not recorded, and taken
    table_file(tmp_path, fitted=None)
    capsys.readouterr()
    assert "position table" in cn.install(target, **NAMED, root=tmp_path)
    assert capsys.readouterr().out.splitlines()[0] == (
        f"{path}: the wide-code slots its lookup was fitted on are not recorded")
    assert cn.differences(target, sc.servo_state()) == []


def test_install_as_a_command(cli, bench, tmp_path, capsys):
    cli("adopt", "--from", str(record(tmp_path)))
    path = cn.set_path(PAIR)
    path.write_text(cn.render(deviated(cn.load(path), current_limit_counts=600, p_kp_q88=5026)))
    capsys.readouterr()
    with pytest.raises(SystemExit) as e:
        cli("install")
    assert str(e.value).startswith("refused: the protection gate:\n  current_limit_counts: servo 554, set 600")
    cli("install", "--protection", "current_limit_counts", "--loosen", "current_limit_counts")
    out = capsys.readouterr().out.splitlines()
    assert out[:3] == ["p_kp_q88: 40212 -> 5026", "current_limit_counts: 554 -> 600",
                       "stamped: data state clean, stamp 0x1234 = live set"]
    assert out[3].startswith("12 fields: ") and out[-1] == "volatile, not saved: `osc reboot` returns the saved state"
    cli("install")
    assert capsys.readouterr().out.splitlines()[-1] == "the servo holds the set: nothing written"
    cli("check")


# --- the rules and build ---

class Geared(Bench):
    """The fake servo with the angle map and the gear fields the first
    rules bind."""

    FIELDS = {**{n: f for n, f in Bench.FIELDS.items() if f[0] < 0xc0},
              "angle_min_cdeg": (0xa8, 2, "rw", "uint"), "angle_max_cdeg": (0xaa, 2, "rw", "uint"),
              "gear_ratio_centi": (0xac, 2, "rw", "uint"), "gear_class": (0xae, 1, "rw", "class"),
              **{n: f for n, f in Bench.FIELDS.items() if f[0] >= 0xc0}}
    STORED = {**Bench.STORED, "angle_min_cdeg": 0, "angle_max_cdeg": 18774, "gear_ratio_centi": 30805,
              "gear_class": 2}

    def decoded(self, name):
        if self.FIELDS[name][3] != "class":
            return super().decoded(name)
        v = self.table[self.FIELDS[name][0]]
        return f"{v} ({cn.GEAR_CLASSES[v]})"

    def get(self, name):
        # `osc get` prints a class as its number alone
        v = super().get(name)
        return int(v.split()[0]) if self.FIELDS[name][3] == "class" else v


REG = {"servo": {"gear_ratio": 308.0533, "gear_class": "metal"}}
STOPS = (82, 3792)
THETA = (-90.0, 90.0)      # the tag angle at the stops: 180 deg of travel, 3710 counts
SCALE = (STOPS[1] - STOPS[0]) / 180
WIDE = "stages/1-board/wide-codes/results/board.json"
CURRENT = "stages/1-board/current-scale/results/board.json"
# what a build says of a board with no current-scale result: the set holds no scale to score a current with
NO_SCALE = "current_counts_per_a: left out, 1-board/current-scale has no result for board"
# and of a pair with none of the results the motor block's rules read and a policy that names neither the
# resistance's rule nor the observer: the two fields of it the fake servo holds are held, the rest left out
WAYS = ("rails_fit", "locked_laps", "small_step", "take_back")
NO_RULE = "policy.json names no rule for {} of " + ", ".join(f"{{}}.{w}" for w in WAYS)
NO_LADDER = "3-servo/ladder has no result for synth__board"
# and of a servo that names no class: no value is read off a maker's sheet
NO_CLASS = "the registry has no servo datasheet_class for synth__board"
NO_LIMIT = f"current_limit_counts: held, {NO_CLASS}"
NO_CEILING = f"speed_ceiling_cps: left out, {NO_CLASS}"
NO_R = f"r_q12: held, {NO_RULE.format('r_q12', *['r'] * 4)}"
# and of the fields in counts of position under a policy with none of their entries: the range, the hold band
# and the stall speed, which the fake servo holds, are held, the rest left out
NO_RANGE = [*(f"pos_{end}_soft_counts: held, policy.json has no soft_margin_share" for end in ("min", "max")),
            "pos_deadband_counts: held, policy.json has no hold_band_noise_multiple"]
NO_STALL = "stall_omega_max_cps: held, policy.json has no stall_omega_max_deg_s"
NO_RESPONSE = "stall_response: held, policy.json has no stall_response"
NO_COUNTS = [f"{n}: left out, policy.json has no {key}" for n, key in (
    ("velocity_limit_cps", "velocity_limit_deg_s"), ("accel_limit_q88", "accel_limit_deg_s2"),
    ("pos_error_counts", "pos_error_deg"), ("sensor_delta_max", "sensor_delta_max_counts"))]
HELD_BLOCK = [*NO_RANGE, NO_LIMIT, NO_RESPONSE, NO_STALL, NO_R, f"recip_ke_q: held, {NO_LADDER}"]
# the current loop's gains under a policy with no bandwidth, and the times, the counts of samples and the
# choices in words under one with none of their entries
NO_LOOP = [f"{n}: left out, policy.json has no current_loop_hz" for n in ("i_kp_q88", "i_ki_q412", "i_kaw_q412")]
NO_CHOICES = [f"{n}: left out, policy.json has no {key}" for n, key in (
    ("stall_time_ticks", "stall_time_ms"), ("pos_error_time_ticks", "pos_error_time_ms"),
    ("oc_trip_ticks", "oc_trip_samples"), ("sensor_bad_count", "sensor_bad_samples"),
    ("openloop_zero_brake", "openloop_zero_brake"), ("duty_max_q15", "duty_max_share"))]
NO_BLOCK = [
    f"r_bemf_q12: left out, {NO_RULE.format('r_bemf_q12', *['r_bemf'] * 4)}",
    f"b_i_q313: left out, {NO_LADDER}",
    *(f"{n}: left out, 3-servo/friction has no result for synth__board" for n in ("fric_fc_counts", "fric_fv_q016")),
    *(f"{n}: left out, {NO_LADDER}" for n in ("ke_vpc_q", "j_ff_q88")),
    *(f"{n}: left out, policy.json has no observer_hz" for n in ("l1_q016", "l2_q88", "l3_q88")),
    *NO_LOOP,
    "r0_q12: left out, 3-servo/resistance has no result for synth__board",
    f"ke_uvs_per_rad: left out, {NO_LADDER}",
    "fric_breakaway_counts: left out, 3-servo/friction has no result for synth__board",
    *NO_COUNTS,
    *(f"{n}: left out, 1-board/ntc-curve has no result for board" for n in ("ntc_raw_ref", "ntc_t_ref_cc",
                                                                           "ntc_k1_q88", "ntc_k2_q24")),
    *(f"{n}_counts: left out, policy.json has no {n}_ma" for n in ("i_cont", "stall_release", "oc_trip")),
    "stall_tau_trip_counts: left out, policy.json has no collision_trip",
    "cutoff_cc: left out, policy.json has no cutoff_c",
    *(f"{n}: left out, 4-estimators/winding-temperature has no result for synth__board" for n in (
        "thermal_tau_s", "thermal_tau2_s")),
    *(f"{n}: left out, policy.json has no winding_plateau_reading" for n in ("th_rise_q016", "th_rise2_q016")),
    *NO_CHOICES,
    "v_undervolt_counts: left out, 1-board/terminal-chain has no result for board",
    "rail_fold_q8: left out, 1-board/current-scale has no result for board", NO_CEILING]
# the registry with the board's tick rate and the class the fake servo is of, whose datasheet record a build
# reads the speed ceiling and the current limit off, and the policy entries the rules of the motor block and
# of the protection fields read: the current limit is the stall at 4.8 V through the rails fit's 4 ohm, at
# half, 600 counts at 1000 counts per A
CLASS = "synth-class"
CLASS_SHEET = f"stages/5-closed-loop/calibration/{CLASS}.datasheet.json"
DOCUMENT = "the maker's page, at its address, read on a day"
BOARD_REG = {"servo": {**REG["servo"], "datasheet_class": CLASS}, "board": {"tick_hz": 20_000}}
LIMITS_MA = {"datasheet_derate_default": 0.5, "i_cont_ma": 250.0, "stall_release_ma": 90.0, "oc_trip_ma": 2200.0}
BLOCK_POLICY = {"observer_hz": 15.0, "rule": {"r_q12": "r.rails_fit", "r_bemf_q12": "r_bemf.small_step"},
                **LIMITS_MA, "collision_trip": "off", "cutoff_c": 90.0, "winding_plateau_reading": "high",
                "stall_response": "fault"}
# the policy entries the rules of the fields in counts of position read, and the pot's noise at rest
COUNT_POLICY = {"velocity_limit_deg_s": 75.0, "accel_limit_deg_s2": 1500.0, "stall_omega_max_deg_s": 25.0,
                "pos_error_deg": 20.0, "soft_margin_share": 0.0625, "hold_band_noise_multiple": 2.5,
                "hold_band_floor_counts": 4, "sensor_delta_max_counts": 256}
POT_NOISE = {"3-servo/pot-table": {"pot_rest_sd_counts": (1.0, 0.0, 1.5)}}
# the policy entries the other rules read: the current loop's bandwidth, the times, the counts of samples,
# the choices in words and the supply terms
CHOICE_POLICY = {"current_loop_hz": 1000.0, "stall_time_ms": 500.0, "pos_error_time_ms": 100.0, "oc_trip_samples": 8,
                 "sensor_bad_samples": 4, "openloop_zero_brake": "off",
                 "duty_max_share": 1.0, "v_undervolt_v": 3.0, "rail_source_ohm": 0.5}
# what a build of the plant over those inputs says, in the set's order: the fields in counts of position,
# the current loop's gains and the choices, whose entries that policy lacks, and the protection fields the
# inputs give a value, the one the fake servo holds carried with what it would be, the rest, which nothing
# carries, left to a build that names them
BLOCK_SAID = [
    *NO_RANGE, "current_limit_counts: carried at 554, it would be 600: name it with --protection", NO_STALL,
    *NO_LOOP, *NO_COUNTS,
    *(f"{n}: left out, name it with --protection" for n in (
        "ntc_raw_ref", "ntc_t_ref_cc", "ntc_k1_q88", "ntc_k2_q24", "i_cont_counts", "stall_release_counts",
        "oc_trip_counts", "stall_tau_trip_counts", "cutoff_cc", "thermal_tau_s", "thermal_tau2_s", "th_rise_q016",
        "th_rise2_q016")),
    *NO_CHOICES,
    "v_undervolt_counts: left out, policy.json has no v_undervolt_v",
    "rail_fold_q8: left out, policy.json has no rail_source_ohm",
    "speed_ceiling_cps: left out, policy.json has no datasheet_derate_speed"]
PLANT = {
    "1-board/current-scale": {"counts_per_a": (1000.0, 990.0, 1010.0)},
    "1-board/ntc-curve": {"design_raw_ref": (2048.0,), "design_t_ref_cc": (2600.0,), "design_k1_q88": (200.0,),
                          "design_k2_q24": (30000.0,)},
    # the tap's scale and the rail's line
    "1-board/terminal-chain": {"terminal_counts_per_v": (250.0,), "rail_counts_per_v": (248.0, 247.0, 249.0),
                               "rail_offset_counts": (6.0,)},
    # the rails fit with the board temperature it was taken at
    "3-servo/resistance": {"r_rails_at_measured_e_ohm": (4.0, 3.9, 4.2), "r_term_ohm": (4.1,),
                           "board_ntc_c": (27.0,)},
    "3-servo/steps": {"r_step_a_ohm": (5.0, 4.0, 6.0), "r_takeback_ohm": (4.8,)},
    "3-servo/inductance": {"l_mh": (0.8, 0.75, 0.85)},
    "3-servo/ladder": {"ke_v_s_per_rad": (0.001,)},
    "3-servo/run-up": {"j_kg_m2": (6e-9, 5e-9, 7e-9)},
    "3-servo/friction": {"firmware_pair": {"pooled": {"coulomb_uNm": (50.0, 40.0, 60.0),
                                                      "viscous_uNm_s_per_rad": (0.05,)}},
                         "load_static_friction_uNm_motor": (100.0, 80.0, 120.0)},
    # the fast node's gain beside each reading's slow node, which the quantity carries with it
    "4-estimators/winding-temperature": {
        "one_node_tau_s": (2.4, 1.2, 3.2), "family_slow_tau_s": (128.9, 121.0, 135.0),
        "joined_low_fast_gain_c_per_w": {"value": 21.0, "lo": 16.0, "hi": 21.0, "slow_gain_c_per_w": 32.0},
        "joined_high_fast_gain_c_per_w": {"value": 20.0, "lo": 15.0, "hi": 20.0, "slow_gain_c_per_w": 60.0}},
}


def quantities(entry):
    """A result's entries with each (value, lo, hi) as the quantity a
    result file holds, a number as it is."""
    if isinstance(entry, dict):
        return {k: quantities(v) for k, v in entry.items()}
    if not isinstance(entry, tuple):
        return entry
    value, lo, hi = (*entry, None, None)[:3]
    return {"value": value, "lo": lo, "hi": hi, "unit": ""}


def plant_path(root, name):
    stage, experiment = name.split("/")
    return root / f"stages/{stage}/{experiment}/results/{'board' if stage == '1-board' else PAIR}.json"


def plant(root, values=PLANT):
    """The stage 1, stage 3 and stage 4 results the rules read, under
    `root`, each naming what it read: nothing."""
    for name, doc in values.items():
        path = plant_path(root, name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"inputs": {}, **quantities(doc)}))


def inputs(root, slots=NEW_SLOTS, theta=THETA, policy=None, fitted=NEW_SLOTS, wide_inputs=None):
    """The inputs of record a build reads, under `root`: the board's
    wide-code result (none for `slots` None), the pair's table file
    (none for `theta` None) and the policy file."""
    if slots is not None:
        (root / WIDE).parent.mkdir(parents=True, exist_ok=True)
        (root / WIDE).write_text(json.dumps({
            **({} if wide_inputs is None else {"inputs": wide_inputs}), wc.FIELD: list(slots)}))
    if theta is not None:
        path = table_file(root, stops=STOPS, fitted=fitted)
        doc = json.loads(path.read_text())
        doc["target"]["theta_at_stops_deg"] = list(theta)
        path.write_text(json.dumps(doc))
    path = cn.set_path(PAIR, root).with_name("policy.json")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"angle_cw_stop_deg": 0.0, "hold": {}, **(policy or {})}))


@pytest.fixture
def geared(monkeypatch):
    return Geared().stub(monkeypatch)


@pytest.fixture
def first(geared, tmp_path):
    """The set adopted from the fake servo as it boots, the set a build
    stands on."""
    doc, _ = cn.adopt(PAIR, record(tmp_path), root=tmp_path)
    geared.log.clear()
    return doc


def built_on(previous, root, **kw):
    return cn.build(PAIR, previous, root=root, registry=REG, **kw)


def moved_hashes(a, b):
    return [k for k in ("plant", "protection") if a["hash"][k] != b["hash"][k]]


def test_build_binds_what_defines_a_count_and_carries_the_rest(first, tmp_path):
    inputs(tmp_path)
    doc, notes = built_on(first, tmp_path)
    sha = cn.points_sha1(POINTS)
    wide = f"1-board/wide-codes:{wc.FIELD}"
    assert doc["wide_codes"] == {"slots": list(NEW_SLOTS), "class": "measured", "rule": "slots.wide_codes",
                                 "inputs": [wide]}
    assert doc["pos_table"] == {"state": "LIVE", "points_sha1": sha, "class": "measured", "rule": "table.pos_lut",
                                "fitted_on_slots_sha1": postable.slots_sha1(NEW_SLOTS), "inputs": ["pos_table"]}
    assert doc["scale"] == {"kind": "table", "counts_per_deg": pytest.approx(SCALE), "points_sha1": sha}
    f = doc["fields"]
    angle = {"rule_rev": 1, "unit": "cdeg", "q": 0, "pos_unit": -1, "scale_points_sha1": sha, "protection": None}
    # the end at the clockwise stop is the policy's angle, the other one the measured travel past it
    assert f["angle_min_cdeg"] == {"value": 0, "class": "policy", "rule": "angle_min.table", **angle,
                                   "inputs": ["policy:angle_cw_stop_deg", "pos_table:theta_at_stops_deg"]}
    assert f["angle_max_cdeg"] == {
        "value": 18000, "class": "measured", "rule": "angle_max.table", **angle,
        "inputs": ["policy:angle_cw_stop_deg", "pos_table:theta_at_stops_deg", "pos_table:stops", "scale"]}
    # the registry's gear ratio is a choice of which count stands for the unit, its gear class read off the part
    assert f["gear_ratio_centi"] == {
        "value": 30805, "class": "policy", "rule": "gear_ratio.registry", "rule_rev": 1,
        "unit": "motor rev per output rev x100", "q": 0, "pos_unit": 0, "inputs": ["registry:gear_ratio"],
        "protection": None}
    assert (f["gear_class"]["value"], f["gear_class"]["class"], f["gear_class"]["inputs"]) == (
        "2 (metal)", "bench", ["registry:gear_class"])
    # every other field stands as it was adopted, in the previous set's order, the stops among them
    rules = set(cn.RULES)
    assert list(f) == list(first["fields"])
    assert all(f[n] == first["fields"][n] for n in f if n not in rules)
    assert f["raw_min"]["class"] == f["p_kp_q88"]["class"] == "none"
    assert all("protection" in e for e in f.values())
    # what the build read, each file by its sha1, the registry by its values, and the origin still used
    assert doc["inputs"] == {
        "1-board/wide-codes": {"file": WIDE, "sha1": sc.file_sha1(tmp_path / WIDE), "inputs_recorded": False},
        "policy": {"file": "stages/5-closed-loop/calibration/policy.json",
                   "sha1": sc.file_sha1(cn.set_path(PAIR, tmp_path).with_name("policy.json"))},
        "pos_table": {"file": f"stages/5-closed-loop/calibration/{PAIR}.pos_lut.json",
                      "sha1": sc.file_sha1(cn.set_path(PAIR, tmp_path).with_name(f"{PAIR}.pos_lut.json"))},
        "registry": REG}
    assert doc["origins"] == first["origins"] and doc["previous"] == first["hash"]
    assert notes == ["1-board/wide-codes: the results it read are not recorded", NO_SCALE, *HELD_BLOCK, *NO_BLOCK]
    # a field of the motor block whose rule has nothing to read is held where it stood, at its value
    assert f["recip_ke_q"] == {"value": 11817, "class": "none", "rule": "recip_ke.ladder", "held": NO_LADDER,
                               "carried_from": first["hash"]["set"], "scale": "unknown", "protection": None}
    # the plant moved and the protection fields did not; the same inputs give the same bytes; the file loads
    # a protection field whose rule has nothing to read is held where it stood too: no value of it moves
    assert f["current_limit_counts"] == {"value": 554, "class": "none", "rule": "current_limit.stall_derate",
                                         "held": NO_CLASS,
                                         "carried_from": first["hash"]["set"], "protection": {"looser": "up"}}
    assert moved_hashes(first, doc) == ["plant"]
    assert cn.render(built_on(first, tmp_path)[0]) == cn.render(doc)
    path = cn.write(doc, tmp_path / "derived" / "x.constants.json")
    assert cn.load(path) == doc and str(tmp_path) not in path.read_text()
    assert cn.described(doc)[3] == f"scale: {SCALE:.4f} counts/deg, the table one"
    lines, stale = cn.status(doc, tmp_path, REG)
    assert not stale and lines[:2] == [
        "3 input files, none moved",
        "1-board/wide-codes: the results it read are not recorded: wide-code slots"]
    assert cn.changes(first, doc) == [
        f"wide-code slots: {SLOTS.hex(' ')} -> {NEW_SLOTS.hex(' ')} [measured, slots.wide_codes]",
        "position table state: IDENTITY -> LIVE [measured, table.pos_lut]",
        f"position table points sha1: {NO_POINTS} -> {sha} [measured, table.pos_lut]",
        f"scale: none -> {SCALE:.4f} counts/deg, the table one",
        "pos_min_soft_counts: 314 [none -> none, held, soft_min.stops]",
        "pos_max_soft_counts: 3560 [none -> none, held, soft_max.stops]",
        "pos_deadband_counts: 4 [none -> none, held, deadband.pot_noise]",
        "current_limit_counts: 554 [none -> none, held, current_limit.stall_derate]",
        "stall_response: Fault (0) [none -> none, held, stall_response.policy]",
        "stall_omega_max_cps: 500 [none -> none, held, stall_omega_max.policy]",
        "r_q12: 17100 [none -> none, held]",
        "recip_ke_q: 11817 [none -> none, held, recip_ke.ladder]",
        "angle_min_cdeg: 0 [none -> policy, angle_min.table]",
        "angle_max_cdeg: 18774 -> 18000 [measured, angle_max.table]",
        "gear_ratio_centi: 30805 [none -> policy, gear_ratio.registry]",
        "gear_class: 2 (metal) [none -> bench, gear_class.registry]",
        f"hash: plant moved: plant {first['hash']['plant'][:12]} -> {doc['hash']['plant'][:12]}, protection "
        f"{first['hash']['protection'][:12]} -> {doc['hash']['protection'][:12]}, set "
        f"{first['hash']['set'][:12]} -> {doc['hash']['set'][:12]}"]


def test_a_built_set_installs_and_checks_clean(first, geared, tmp_path):
    inputs(tmp_path)
    doc, _ = built_on(first, tmp_path)
    assert cn.install(doc, root=tmp_path) == ["wide-code slots", "position table", "angle_max_cdeg", "stamp"]
    assert cn.check(doc) == []


def context(root, k=1.0, policy=None, **kw):
    """The Context a build gives its rules over `inputs(**kw)` and the
    plant's results, the scale of the count times `k`."""
    inputs(root, policy={**BLOCK_POLICY, **COUNT_POLICY, **CHOICE_POLICY, **DERATE, **(policy or {})}, **kw)
    plant(root, {**PLANT, **POT_NOISE})
    datasheet(root)
    c = cn.Context(PAIR, root, BOARD_REG)
    cn.table_of(c, None, cn.slots_of(c, None))
    c.scale = {**c.scale, "counts_per_deg": c.scale["counts_per_deg"] * k}
    return c


def scales_by_its_unit(r, root, k, theta):
    """Whether rule `r`'s value at k times the counts per deg is its
    value times k to the power of its position unit, and it read the
    scale exactly when it declares one."""
    values, read = [], []
    for scale in (1.0, k):
        c = context(root, scale, theta=theta)
        c.reads = []
        values.append(r.fn(c))
        read.append(any(name == "scale" for name, _ in c.reads))
    x, xk = values
    if isinstance(x, str):
        return x == xk and r.pos_unit == 0 and not any(read)
    same = xk == pytest.approx(x * k ** r.pos_unit, rel=1e-9, abs=1e-9)
    # a value that is zero at every scale says nothing of the scale, and need not read it
    return same and (x == xk == 0 or read == [bool(r.pos_unit)] * 2)


@pytest.mark.parametrize("theta", [THETA, THETA[::-1]], ids=["ascending", "descending"])
def test_every_rule_scales_by_its_position_unit(tmp_path, theta):
    for r in cn.rules():
        assert scales_by_its_unit(r, tmp_path, 1.25, theta), r.name
        # the check has teeth: the same rule declared at another power fails it
        if not isinstance(r.fn(context(tmp_path, theta=theta)), str):
            wrong = dataclasses.replace(r, pos_unit=r.pos_unit + 1)
            other = context(tmp_path, theta=theta)
            other.reads = []
            assert r.fn(other) == 0 or not scales_by_its_unit(wrong, tmp_path, 1.25, theta), r.name


def test_the_angle_map_puts_the_clockwise_stop_at_the_policys_angle(tmp_path):
    def angles(key="value", **kw):
        c = context(tmp_path, **kw)
        return [cn.evaluate(cn.rule_of(c, n), c)[key] for n in ("angle_min_cdeg", "angle_max_cdeg")]

    assert angles() == [0, 18000]
    # a tag angle that falls as the count rises: the high stop is the clockwise one
    assert angles(theta=THETA[::-1]) == [18000, 0]
    assert angles(policy={"angle_cw_stop_deg": -90.0}) == [-9000, 9000]
    # the end at the clockwise stop is the policy's and reads no travel, the other one is measured
    assert angles("class") == ["policy", "measured"] and angles("class", theta=THETA[::-1]) == ["measured", "policy"]


def test_a_field_that_does_not_fit_is_refused(first, tmp_path):
    inputs(tmp_path, theta=(-200.0, 200.0))
    with pytest.raises(ValueError) as e:
        built_on(first, tmp_path)
    assert str(e.value) == ("angle_max_cdeg: angle_max.table gives 40000 cdeg, 40000 as the field holds it, which "
                            "does not fit its i16 (-32768..32767)")
    # the encoder rounds to the field's Q format, half away from zero, and passes text; it never clamps
    r = dataclasses.replace(cn.RULES["angle_max_cdeg"]["angle_max.table"], kind="u16", q=8)
    assert [cn.encode(r, x) for x in (1.5, 0.001953125, 0.0019, 255.998)] == [384, 1, 0, 65535]
    assert cn.encode(dataclasses.replace(r, kind="i16", q=0), -2.5) == -3
    assert cn.encode(r, "2 (metal)") == "2 (metal)"
    # a cap derated from a maker's value rounds down, never past its derate: the two datasheet derates
    cap = dataclasses.replace(r, cap=True)
    assert [cn.encode(cap, x) for x in (1.5, 0.0039, 255.998)] == [384, 0, 65535]
    assert {r.name for r in cn.rules() if r.cap} == {"speed_ceiling.datasheet", "current_limit.stall_derate"}
    for x in (256.0, -0.01):
        with pytest.raises(ValueError, match="does not fit its u16"):
            cn.encode(r, x)


def test_a_missing_result_holds_the_field_and_says_why(first, tmp_path):
    # a board with no wide-code result and a pair with no table: what the set held stands, and says why
    inputs(tmp_path, slots=None, theta=None)
    doc, notes = built_on(first, tmp_path)
    carried = {"class": "none", "carried_from": first["hash"]["set"]}
    assert doc["wide_codes"] == {"slots": list(SLOTS), **carried, "rule": "slots.wide_codes",
                                 "held": "1-board/wide-codes has no result for board"}
    assert doc["pos_table"] == first["pos_table"] and "scale" not in doc
    no_table = "the pair has no position table: which stop is the clockwise one is not known"
    assert doc["fields"]["angle_max_cdeg"] == {"value": 18774, **carried, "rule": "angle_max.table",
                                               "held": no_table, "protection": None}
    assert doc["fields"]["gear_ratio_centi"]["class"] == "policy"
    assert notes == [
        "wide-code slots: held, 1-board/wide-codes has no result for board",
        "scale: none, 3-servo/pot-table has no result for synth__board", NO_SCALE, *HELD_BLOCK,
        f"angle_min_cdeg: held, {no_table}", f"angle_max_cdeg: held, {no_table}", *NO_BLOCK]
    # the gear's two fields have a source, the registry; every other one has none, 10 of them held
    assert cn.described(doc)[0].startswith("16 fields: 0 measured, 1 bench, 0 datasheet, 1 policy, 0 pinned, 14 none")
    assert sum("held" in e for e in doc["fields"].values()) == 10
    assert moved_hashes(first, doc) == [] and cn.load(cn.write(doc, tmp_path / "x.constants.json")) == doc
    # held again on top of that set, the value still names the set it first came from
    again, _ = built_on(doc, tmp_path)
    assert again["fields"]["angle_max_cdeg"]["carried_from"] == first["hash"]["set"]
    # a registry with no such entry holds the gear fields the same way
    lone, _ = cn.build(PAIR, first, root=tmp_path, registry={})
    assert lone["fields"]["gear_class"] == {"value": "2 (metal)", **carried, "rule": "gear_class.registry",
                                            "held": "the registry has no servo gear_class for synth__board",
                                            "protection": None}
    # with nothing to carry the field is left out, and said
    new, notes = cn.build(PAIR, None, root=tmp_path, registry=REG)
    assert list(new["fields"]) == ["gear_ratio_centi", "gear_class"] and new["wide_codes"] is None
    assert new["pos_table"] == {"state": "IDENTITY", "points_sha1": NO_POINTS, "class": "measured",
                                "rule": "table.none"} and new["previous"] is None
    assert f"angle_max_cdeg: left out, {no_table}" in notes and notes[0].startswith("wide-code slots: left out, ")


def test_a_field_the_policy_holds_is_carried_with_what_its_rule_would_give(first, tmp_path):
    inputs(tmp_path, policy={"hold": {"angle_max_cdeg": "the hosts read the old travel until they are moved"}})
    doc, notes = built_on(first, tmp_path)
    assert doc["fields"]["angle_max_cdeg"] == {
        "value": 18774, "class": "none", "rule": "angle_max.table", "would_be": 18000,
        "held": "the hosts read the old travel until they are moved", "carried_from": first["hash"]["set"],
        "inputs": ["policy:angle_cw_stop_deg", "pos_table:theta_at_stops_deg", "pos_table:stops", "scale"],
        "protection": None}
    assert ("angle_max_cdeg: held, the hosts read the old travel until they are moved; its rule "
            "gives 18000") in notes


def test_a_measured_field_at_another_table_is_refused(first, tmp_path):
    inputs(tmp_path)
    doc, _ = built_on(first, tmp_path)
    path = tmp_path / "x.constants.json"
    other = copy.deepcopy(doc)
    other["fields"]["angle_max_cdeg"]["scale_points_sha1"] = NO_POINTS
    with pytest.raises(ValueError, match=f"angle_max_cdeg was computed for the table of points sha1 {NO_POINTS}, "
                                         f"the set's scale is that of {cn.points_sha1(POINTS)}"):
        cn.load(cn.write(other, path))
    # nor does a set hold a scale of one table and the points of another
    other = copy.deepcopy(doc)
    other["pos_table"]["points_sha1"] = NO_POINTS
    other["hash"] = cn.hashes(other)
    with pytest.raises(ValueError, match="its scale is that of the table of points sha1"):
        cn.load(cn.write(other, path))


def test_build_refuses_a_table_that_is_not_the_sets(first, tmp_path):
    inputs(tmp_path, fitted=SLOTS)
    with pytest.raises(ValueError) as e:
        built_on(first, tmp_path)
    assert str(e.value) == (f"the lookup was fitted on other slots (sha1 {postable.slots_sha1(SLOTS)}, the set's "
                            f"are {postable.slots_sha1(NEW_SLOTS)}): re-run stage 4 position, then `postable build`")
    # a table file that records no slots is said, not refused
    inputs(tmp_path, fitted=None)
    doc, notes = built_on(first, tmp_path)
    assert doc["pos_table"]["fitted_on_slots_sha1"] is None
    assert "position table: the wide-code slots its lookup was fitted on are not recorded" in notes
    # the stops are the set's, adopted, and the table has to be built against them
    table_file(tmp_path, stops=NEW_STOPS)
    with pytest.raises(ValueError, match="was built against stops 90 and 3780, the set's are 82 and 3792"):
        built_on(first, tmp_path)


def limit(c):
    return 2 * c.q("1-board/current-scale", "half_limit_counts")


@pytest.fixture
def limited(monkeypatch, tmp_path):
    """A rule for a protection field, over a banded result that names
    what it read."""
    monkeypatch.setitem(cn.RULES, "current_limit_counts", {"current_limit.class": cn.Rule(
        "current_limit_counts", "current_limit.class", "u16", "ccounts", 0, 0, 1, "policy", "none", limit)})
    path = tmp_path / "stages/1-board/current-scale/results/board.json"
    path.parent.mkdir(parents=True)

    def write(value, read=None):
        path.write_text(json.dumps({"inputs": {} if read is None else read,
                                    "half_limit_counts": {"value": value, "lo": value - 5, "hi": value + 10}}))
    write(250)
    return write


def test_one_build_moves_one_hash(first, limited, tmp_path):
    inputs(tmp_path)
    # the plant moves: the protection field stands, with what its rule would give beside it
    doc, notes = built_on(first, tmp_path)
    assert moved_hashes(first, doc) == ["plant"]
    assert doc["fields"]["current_limit_counts"] == {**first["fields"]["current_limit_counts"], "would_be": 500}
    assert "current_limit_counts: carried at 554, it would be 500: name it with --protection" in notes
    # named, it moves, alone: the plant is the previous set's to the byte
    named, notes = built_on(doc, tmp_path, protection=["current_limit_counts"])
    # a field its rule leaves out is said once, by the rule
    assert moved_hashes(doc, named) == ["protection"] and notes == [
        "1-board/wide-codes: the results it read are not recorded",
        "current_counts_per_a: left out, 1-board/current-scale has no counts_per_a",
        *(n for n in HELD_BLOCK if n != NO_LIMIT), *NO_BLOCK[:-2],
        "rail_fold_q8: left out, 1-board/current-scale has no counts_per_a", NO_CEILING]
    assert named["fields"]["current_limit_counts"] == {
        "value": 500, "class": "policy", "rule": "current_limit.class", "rule_rev": 1, "unit": "ccounts", "q": 0,
        "pos_unit": 0, "inputs": ["1-board/current-scale:half_limit_counts"], "band": [490, 520],
        "protection": {"looser": "up"}}
    assert all(named["fields"][n] == doc["fields"][n] for n in doc["fields"] if n != "current_limit_counts")
    assert [named[k] for k in ("wide_codes", "pos_table", "scale")] == [doc[k] for k in ("wide_codes", "pos_table",
                                                                                         "scale")]
    assert named["previous"] == doc["hash"] and cn.load(cn.write(named, tmp_path / "x.constants.json")) == named
    # a plant build on top of it leaves the limit where it was put
    assert moved_hashes(named, built_on(named, tmp_path)[0]) == []
    # a plant field its rule would move stays put in a protection build, and is said
    limited(260)
    inputs(tmp_path, theta=(-80.0, 100.0), policy={"angle_cw_stop_deg": 10.0})
    both, notes = built_on(named, tmp_path, protection=["current_limit_counts"])
    assert moved_hashes(named, both) == ["protection"] and both["fields"]["current_limit_counts"]["value"] == 520
    assert both["fields"]["angle_min_cdeg"] == {**named["fields"]["angle_min_cdeg"], "would_be": 1000}
    assert "angle_min_cdeg: carried at 0, it would be 1000" in notes
    # what defines a count has to stand: a protection build does not carry a table that would move
    inputs(tmp_path, theta=(-100.0, 100.0))
    with pytest.raises(ValueError, match="the slots, the table, the scale or what the set is scored with would "
                                         "move: build the plant first"):
        built_on(named, tmp_path, protection=["current_limit_counts"])
    for names, match in ((["p_kp_q88"], "p_kp_q88 is no protection field"),):
        with pytest.raises(ValueError, match=match):
            built_on(named, tmp_path, protection=names)
    with pytest.raises(ValueError, match="there is no set to carry the plant from"):
        cn.build(PAIR, None, protection=["current_limit_counts"], root=tmp_path, registry=REG)


def test_a_field_the_firmware_no_longer_names_leaves_in_the_build_that_names_it(first, geared, tmp_path):
    inputs(tmp_path)
    plain, _ = built_on(first, tmp_path)
    # a set written while the firmware had the field: it holds it under the protection hash, and loads
    old = copy.deepcopy(plain)
    old["fields"]["derate_start_cc"] = {"value": 8000, **ADOPTED, "protection": cn.protection_of("derate_start_cc")}
    old["hash"] = cn.hashes(old)
    assert old["fields"]["derate_start_cc"]["protection"] == {"looser": "up"}
    assert moved_hashes(plain, old) == ["protection"] and cn.load(cn.write(old, tmp_path / "old.constants.json")) == old
    # a build of the plant carries it as it stood, like any protection field, and says what names it
    doc, notes = built_on(old, tmp_path)
    assert doc["fields"]["derate_start_cc"] == old["fields"]["derate_start_cc"] and doc["hash"] == old["hash"]
    assert ("derate_start_cc: carried at 8000, the firmware names no such field: name it with --protection to "
            "leave it out") in notes
    # named, it leaves, and the protection hash moves alone
    named, notes = built_on(old, tmp_path, protection=["derate_start_cc"])
    assert "derate_start_cc" not in named["fields"] and moved_hashes(old, named) == ["protection"]
    assert "derate_start_cc: left out, the firmware names no such field" in notes
    assert cn.changes(old, named)[0] == "derate_start_cc: 8000 -> not in the set"
    assert named["hash"] == plain["hash"] and cn.load(cn.write(named, tmp_path / "x.constants.json")) == named
    # no build after brings it back, named or not, and no pin types it in
    again, notes = built_on(named, tmp_path, protection=["derate_start_cc"])
    assert again["hash"] == named["hash"] and not any(n.startswith("derate_start_cc") for n in notes)
    pins = cn.set_path(PAIR, tmp_path).with_name(f"{PAIR}.pins.json")
    pins.write_text(json.dumps({"derate_start_cc": pin(8000)}))
    with pytest.raises(ValueError, match="the firmware names no derate_start_cc"):
        built_on(named, tmp_path)
    pins.unlink()
    # the older set is refused by a servo whose table has no such field, before anything is written
    with pytest.raises(settings.Refused, match="the servo's descriptor lets a host write no derate_start_cc"):
        cn.install(old, root=tmp_path)
    assert geared.wrote() == []


def test_a_result_that_read_a_file_since_moved_is_stale(first, limited, tmp_path):
    inputs(tmp_path)
    limited(250, {WIDE: "0" * 40})
    with pytest.raises(ValueError) as e:
        built_on(first, tmp_path)
    assert str(e.value) == (f"1-board/current-scale is stale: it read {WIDE} at sha1 {'0' * 40}, which is "
                            f"{sc.file_sha1(tmp_path / WIDE)} now: re-run 1-board/current-scale")
    # one that names what it read, and it stands, says nothing
    limited(250, {WIDE: sc.file_sha1(tmp_path / WIDE)})
    doc, notes = built_on(first, tmp_path)
    assert "inputs_recorded" not in doc["inputs"]["1-board/current-scale"]
    assert not any("current-scale" in n and "not recorded" in n for n in notes)


def test_a_pin_binds_its_field_and_shows_as_pinned(first, tmp_path):
    inputs(tmp_path)
    pins = cn.set_path(PAIR, tmp_path).with_name(f"{PAIR}.pins.json")
    pins.write_text(json.dumps({"p_kp_q88": {"value": 5026, "why": "the 3 Hz loop", "rung": "01b"},
                                "angle_max_cdeg": {"value": 18433, "why": "the travel", "rung": "12a"}}))
    doc, _ = built_on(first, tmp_path)
    assert doc["fields"]["p_kp_q88"] == {"value": 5026, "class": "pinned", "why": "the 3 Hz loop", "rung": "01b",
                                         "protection": None}
    # a pin stands ahead of the field's rule
    assert doc["fields"]["angle_max_cdeg"]["class"] == "pinned" and doc["fields"]["angle_max_cdeg"]["value"] == 18433
    assert doc["inputs"]["pins"] == {"file": f"stages/5-closed-loop/calibration/{PAIR}.pins.json",
                                     "sha1": sc.file_sha1(pins)}
    assert cn.described(doc)[0].startswith("16 fields: 0 measured, 1 bench, 0 datasheet, 2 policy, 2 pinned, 11 none")
    pins.write_text(json.dumps({"p_kp_q88": {"value": 5026}}))
    with pytest.raises(ValueError, match="p_kp_q88 lacks its value, its why or its rung"):
        built_on(first, tmp_path)


# --- where a value comes from: a record, a decision, or nothing ---

RUNG = "stages/5-closed-loop/scorecard/results/synth__board/01b.json"       # a rung's results file
READING = f"stages/5-closed-loop/calibration/{PAIR}.stops.json"             # a dated hand reading
SHEET = "equipment/parts/servo.json"                                        # a maker's values, where the repo has them


def sources(root, **blocks):
    """The pair's sources file under `root`, and the records its
    entries name: a rung's results file, a hand reading and a file of
    a maker's values."""
    path = cn.set_path(PAIR, root).with_name(f"{PAIR}.sources.json")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(blocks))
    for rel, doc in ((RUNG, {"settings": {"p_kp_q88": 5026}}), (READING, {"raw_min": 82, "weak": "three reads a stop"}),
                     (SHEET, {"rated": {"gear_class": "2 (metal)"}})):
        (root / rel).parent.mkdir(parents=True, exist_ok=True)
        (root / rel).write_text(json.dumps(doc))
    return path


def test_a_bench_record_gives_its_field_the_value_it_holds(first, tmp_path):
    inputs(tmp_path)
    path = sources(tmp_path, bench={
        "p_kp_q88": {"record": RUNG, "key": "settings.p_kp_q88", "what": "rung 01b, kept"},
        "raw_min": {"record": READING, "key": "raw_min", "what": "the low stop, a hand reading"}})
    doc, notes = built_on(first, tmp_path)
    f = doc["fields"]
    # the value is the record's, read and never typed: the rung's setting where the servo held another
    assert f["p_kp_q88"] == {"value": 5026, "class": "bench", "record": "rung 01b, kept", "protection": None,
                             "inputs": ["sources:bench.p_kp_q88", f"{RUNG}:settings.p_kp_q88"]}
    assert first["fields"]["p_kp_q88"]["value"] == 40212 and "p_kp_q88: 40212 -> 5026 [bench]" in cn.changes(first, doc)
    # a record that says of itself that it is a weak one is carried as that, with what it awaits
    assert f["raw_min"] == {"value": 82, "class": "bench", "record": "the low stop, a hand reading",
                            "weak": "three reads a stop", "protection": None,
                            "inputs": ["sources:bench.raw_min", f"{READING}:raw_min"]}
    assert cn.weak(doc) == {"three reads a stop": ["raw_min"]}
    # the sources file and each record are inputs of the set, by their sha1: a record that moves makes it stale
    assert {n: doc["inputs"][n] for n in ("sources", RUNG, READING)} == {
        "sources": {"file": f"stages/5-closed-loop/calibration/{PAIR}.sources.json", "sha1": sc.file_sha1(path)},
        RUNG: {"file": RUNG, "sha1": sc.file_sha1(tmp_path / RUNG)},
        READING: {"file": READING, "sha1": sc.file_sha1(tmp_path / READING)}}
    assert cn.load(cn.write(doc, tmp_path / "x.constants.json")) == doc
    lines, stale = cn.status(doc, tmp_path, REG)
    assert not stale and "on a weak record: raw_min: three reads a stop" in lines
    (tmp_path / RUNG).write_text(json.dumps({"settings": {"p_kp_q88": 6000}}))
    lines, stale = cn.status(doc, tmp_path, REG)
    assert stale and lines[0].startswith(f"stale: {RUNG} ({RUNG}) is no longer the file the set read")
    assert lines[0].endswith(": p_kp_q88")
    # and the build after binds the record's value again: the set's value is the record's
    assert built_on(doc, tmp_path)[0]["fields"]["p_kp_q88"]["value"] == 6000
    # a pin stands ahead of a record, and a record ahead of a rule
    pins = cn.set_path(PAIR, tmp_path).with_name(f"{PAIR}.pins.json")
    pins.write_text(json.dumps({"p_kp_q88": pin(7000)}))
    assert built_on(doc, tmp_path)[0]["fields"]["p_kp_q88"]["class"] == "pinned"
    pins.unlink()
    sources(tmp_path, bench={"gear_class": {"record": SHEET, "key": "rated.gear_class", "what": "read off the part",
                                            "weak": "a look at one unit"}})
    lone = built_on(doc, tmp_path)[0]["fields"]
    assert (lone["gear_class"]["record"], lone["gear_class"]["weak"]) == ("read off the part", "a look at one unit")
    # a field the sources no longer name is carried at its value, with no source
    assert lone["p_kp_q88"] == {"value": 5026, "class": "none", "protection": None}


@pytest.mark.parametrize("blocks, match", [
    ({"bench": {"p_kp_q88": {"record": RUNG, "key": "settings.p_kp_q88"}}}, "and p_kp_q88 is not"),
    ({"bench": {"p_kp_q88": {"record": "nowhere.json", "key": "p_kp_q88", "what": "a rung"}}},
     "the bench record of p_kp_q88, nowhere.json, is not there"),
    ({"bench": {"p_kp_q88": {"record": RUNG, "key": "settings.v_kp_q88", "what": "a rung"}}},
     "the bench record of p_kp_q88, .*01b.json, holds no value of a field at settings.v_kp_q88"),
    ({"bench": {"p_kp_q88": {"record": RUNG, "key": "settings", "what": "a rung"}}},
     "holds no value of a field at settings"),
    ({"bench": {"p_kp_q88": {"record": "../out.json", "key": "p_kp_q88", "what": "a rung"}}},
     "is not a file of the repo: a bench record is named by its path in it"),
    ({"datasheet": {"gear_class": {"record": SHEET, "key": "rated.gear_class", "what": "the maker's class"}}},
     "`datasheet` with its record, key, what, document.*and gear_class is not"),
    ({"bench": {"p_kp_q88": {"record": RUNG, "key": "settings.p_kp_q88", "what": "a rung"}},
      "none": {"p_kp_q88": {"reason": "typed", "needs": "a decision"}}}, "and p_kp_q88 is not"),
    ({"none": {"p_kp_q88": {"reason": "typed"}}}, "`none` with its reason, needs.*and p_kp_q88 is not"),
    ({"guessed": {"p_kp_q88": {"reason": "typed", "needs": "a decision"}}}, "and p_kp_q88 is not"),
], ids=["a record with no word on what it is", "a record that is not there", "a key the record does not hold",
        "a key that holds no value", "a record outside the repo", "a maker's value with no document",
        "a field with a record and with no source", "no source with no need", "a block the file has none of"])
def test_the_sources_of_a_pair_name_a_record_or_say_why_there_is_none(first, tmp_path, blocks, match):
    inputs(tmp_path)
    sources(tmp_path, **blocks)
    with pytest.raises(ValueError, match=match):
        built_on(first, tmp_path)


def test_a_datasheet_value_names_its_document_and_where_the_repo_records_it(first, tmp_path):
    inputs(tmp_path)
    sources(tmp_path, datasheet={"gear_class": {"record": SHEET, "key": "rated.gear_class", "what": "the maker's class",
                                                "document": "the maker's sheet, rev B"}})
    doc, _ = built_on(first, tmp_path)
    # the maker's value, read from where the repo records it, with the document it is stated in: ahead of the
    # field's rule, and no measurement
    assert doc["fields"]["gear_class"] == {
        "value": "2 (metal)", "class": "datasheet", "record": "the maker's class",
        "document": "the maker's sheet, rev B",
        "inputs": ["sources:datasheet.gear_class", f"{SHEET}:rated.gear_class"], "protection": None}
    assert cn.classes(doc)["datasheet"] == 1 and cn.CLASSES == ("measured", "bench", "datasheet", "policy", "pinned",
                                                               "none")
    assert cn.load(cn.write(doc, tmp_path / "x.constants.json")) == doc
    assert "gear_class" not in [name for name, *_ in cn.unsourced(doc)]


def test_a_field_with_no_source_says_why_and_what_would_give_it_one(first, tmp_path):
    held = "it moves on a rung of its own"
    why = {"reason": "boot default restated, no decision", "needs": "a decision on the speed"}
    inputs(tmp_path, policy={**COUNT_POLICY, "hold": {"pos_max_soft_counts": held},
                             "undecided": {"accel_limit_deg_s2": why, "soft_margin_share": why}})
    plant(tmp_path, POT_NOISE)
    sources(tmp_path, none={
        "r_q12": {"reason": "old identification fit of another servo", "needs": "an existing result to bind"},
        "p_kp_q88": {"reason": "boot default", "needs": "a decision",
                     "datasheet_candidate": "the maker's rated gain: a file of the repo, no document named"},
        "pos_max_soft_counts": {"reason": "typed, no source", "needs": "a bench session"},
        "pos_deadband_counts": {"reason": "boot default", "needs": "a decision"}})
    doc, notes = cn.build(PAIR, first, root=tmp_path, registry=BOARD_REG)
    f = doc["fields"]
    # carried from the servo's state: the reason and the need the pair's sources state, and the maker's value
    # that could supply it, a candidate and not its source
    assert f["p_kp_q88"] == {**first["fields"]["p_kp_q88"], "reason": "boot default", "needs": "a decision",
                             "datasheet_candidate": "the maker's rated gain: a file of the repo, no document named"}
    # held by a missing input, and held by the policy, where the pair's word stands over the policy's
    assert {k: f["r_q12"][k] for k in ("class", "held", "reason")} == {
        "class": "none", "held": NO_RULE.format("r_q12", *["r"] * 4),
        "reason": "old identification fit of another servo"}
    assert {k: f["pos_max_soft_counts"][k] for k in ("class", "held", "would_be", "reason", "needs")} == {
        "class": "none", "held": held, "would_be": 3560, "reason": "typed, no source", "needs": "a bench session"}
    # bound by a rule that reads an entry nobody decided: the rule's value, and no source, the policy's word
    assert {k: f[n][k] for n in ("accel_limit_q88", "pos_min_soft_counts") for k in ("class", "reason")} == {
        "class": "none", "reason": why["reason"]}
    assert (f["accel_limit_q88"]["value"], f["accel_limit_q88"]["rule"], "held" in f["accel_limit_q88"]) == (
        3957, "accel_limit.policy", False)
    # a field its rule binds from decisions has a source, and the sources' word that it has none is said
    assert f["pos_deadband_counts"]["class"] == "policy" and "reason" not in f["pos_deadband_counts"]
    assert "pos_deadband_counts: the pair's sources say it has no source, and deadband.pot_noise binds it" in notes
    # the debt, in the set's order: every field with no source, a line each, a count a later set can be held to
    debt = cn.unsourced(doc)
    assert [name for name, *_ in debt] == [n for n, e in f.items() if e["class"] == "none"]
    assert len(debt) == cn.classes(doc)["none"] == 12
    said = dict((name, rest) for name, *rest in debt)
    assert said["p_kp_q88"] == [40212, "boot default", "a decision",
                                "the maker's rated gain: a file of the repo, no document named"]
    # one that states no reason says what carries its value
    assert said["drive_polarity"] == ["on", "the servo held it before its set existed",
                                      "a rule over a result, a bench record or a decision", None]
    assert said["recip_ke_q"] == [11817, f"held, {NO_LADDER}", "what its rule reads", None]
    lines, _ = cn.status(doc, tmp_path, BOARD_REG)
    at = lines.index("no source, 12 fields, 1 of which a maker's value on record could supply:")
    assert lines[at + 1:at + 13] == [
        f"  {name} = {value}: {reason}; needs {needs}"
        + ("" if maker is None else f"; datasheet candidate, not its source: {maker}")
        for name, value, reason, needs, maker in debt]
    assert cn.load(cn.write(doc, tmp_path / "x.constants.json")) == doc


def test_a_set_written_before_a_class_said_the_source_loads_as_it_was_written():
    # the copy the reference rung ran on: its held and adopted fields are counted with no source
    path = results.ROOT / "telemetry" / postable.default_pair() / "scorecard" / "14a-set-of-record" / sc.SET_COPY
    old = cn.load(path)
    was = {c: sum(e["class"] == c for e in old["fields"].values()) for c in cn.WAS}
    assert was == {"held": 17, "adopted": 34} and cn.render(old) == path.read_text()
    assert cn.classes(old) == {"measured": 6, "bench": 0, "datasheet": 0, "policy": 6, "pinned": 0, "none": 51}
    assert len(cn.unsourced(old)) == 51 and cn.tagged(old["fields"]["r_q12"]) == "held, r.rails_fit"


def test_the_set_of_record_names_a_source_for_every_field_or_lists_it_as_debt():
    doc = cn.of_record(postable.default_pair())
    f = doc["fields"]
    assert not any(e["class"] in cn.WAS for e in f.values())
    assert cn.classes(doc) == {"measured": 16, "bench": 10, "datasheet": 1, "policy": 34, "pinned": 1, "none": 1}
    by = {c: [n for n, e in f.items() if e["class"] == c] for c in cn.CLASSES}
    assert by["measured"] == ["j_ff_q88", "thermal_tau_s", "r_q12", "recip_ke_q", "b_i_q313", "fric_fc_counts",
                              "fric_fv_q016", "ke_vpc_q", "r0_q12", "th_rise_q016", "ntc_raw_ref", "ntc_t_ref_cc",
                              "ntc_k1_q88", "ntc_k2_q24", "ke_uvs_per_rad", "angle_max_cdeg"]
    # the one pin: the board's nominal supply word, which a capture is matched to its board by
    assert by["pinned"] == ["vdd_mv"]
    # the one value of the maker's: the speed ceiling, read off the class's page
    assert by["datasheet"] == ["speed_ceiling_cps"]
    # the gains of the current loop and of the observer among them: a bandwidth is a target, and the policy
    # holds each at the pick of its bench sweep; and the limits of motion, the owner's default move speed
    assert by["policy"] == ["pos_min_soft_counts", "pos_max_soft_counts", "i_kp_q88", "i_ki_q412", "i_kaw_q412",
                            "duty_max_q15", "pos_deadband_counts", "velocity_limit_cps", "accel_limit_q88",
                            "current_limit_counts", "stall_response",
                            "stall_time_ticks", "stall_omega_max_cps", "i_cont_counts", "stall_release_counts",
                            "stall_tau_trip_counts", "oc_trip_counts", "oc_trip_ticks", "openloop_decay",
                            "openloop_zero_brake", "cutoff_cc", "v_undervolt_counts", "thermal_tau2_s",
                            "pos_error_time_ticks", "pos_error_counts", "sensor_delta_max", "sensor_bad_count",
                            "rail_fold_q8", "l1_q016", "l2_q88", "l3_q88", "th_rise2_q016", "angle_min_cdeg",
                            "gear_ratio_centi"]
    # the bench fields: the loop gains a rung chose, read from its results file, the stops a dated hand
    # reading, the drive's sign the state a rung of record ran on, the gear class read off the part
    rung = "stages/5-closed-loop/scorecard/results/mg90-d__dev-v006-2A-D/01b-gains-3hz-50hz.json"
    stops = "stages/5-closed-loop/calibration/mg90-d__dev-v006-2A-D.stops-2026-11-05.json"
    ran = "telemetry/mg90-d__dev-v006-2A-D/scorecard/14a-set-of-record/run-1/servo.json"
    records = {"p_kp_q88": f"{rung}:settings.p_kp_q88", "v_kp_q88": f"{rung}:settings.v_kp_q88",
               "v_ki_q412": f"{rung}:settings.v_ki_q412", "v_kaw_q412": f"{rung}:settings.v_kaw_q412",
               "pos_min_phys_counts": f"{stops}:pos_min_phys_counts",
               "pos_max_phys_counts": f"{stops}:pos_max_phys_counts", "raw_min": f"{stops}:raw_min",
               "raw_max": f"{stops}:raw_max", "drive_polarity": f"{ran}:fields.drive_polarity",
               "gear_class": "registry:gear_class"}
    assert {n: f[n]["inputs"][-1] for n in by["bench"]} == records
    for name, pointer in records.items():
        if name != "gear_class":
            file, _, key = pointer.partition(":")
            at = json.loads((results.ROOT / file).read_text())
            for k in key.split("."):
                at = at[k]
            # the set's value is the record's, and the record is an input of the set
            assert f[name]["value"] == at and doc["inputs"][file]["file"] == file and f[name]["record"], name
    # the hand reading says of itself that it is a weak record, and the drive's sign has no run that found it
    reading = json.loads((results.ROOT / stops).read_text())
    assert reading["date"] == "2026-11-05" and reading["how"] and "three recorded reads a stop" in reading["weak"]
    assert cn.weak(doc) == {reading["weak"]: ["pos_min_phys_counts", "pos_max_phys_counts", "raw_min", "raw_max"],
                            f["drive_polarity"]["weak"]: ["drive_polarity"]}
    # the debt: 1 field with no source, with its reason and what would give it one from the set's own
    # entry. A unit that gives a field a source brings the count down
    debt = cn.unsourced(doc)
    assert len(debt) == 1 and [name for name, *_ in debt] == by["none"] == ["fric_breakaway_counts"]
    assert all(f[n].get("reason") and f[n].get("needs") for n in by["none"])
    # it waits for a capture no sitting has taken
    waits = {name: needs for name, _, _, needs, _ in debt}
    assert waits["fric_breakaway_counts"].startswith("a driven-ramp breakaway capture")
    reasons = {}
    for name, _, reason, _, _ in debt:
        reasons.setdefault(reason, []).append(name)
    assert {r: len(names) for r, names in reasons.items()} == {"never written": 1}
    # no maker's value the repo records could supply it
    assert [name for name, *_, maker in debt if maker] == []
    lines, stale = cn.status(doc)
    assert not stale and "no source, 1 fields, 0 of which a maker's value on record could supply:" in lines
    assert sum(line.startswith("  ") for line in lines) == 1
    # it is held beside what its rule gives: the breakaway's rule reads a result that is not the field's
    # quantity
    held = [n for n in by["none"] if "held" in f[n]]
    assert held == ["fric_breakaway_counts"]
    assert all(f[n]["would_be"] != f[n]["value"] for n in held)
    policy = json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    assert set(policy["hold"]) == {*held, "r_bemf_q12"} and "r_bemf_q12" not in f
    # the breakaway: the result on record is the back-driven static friction of the load arm, the field a
    # driven ramp's breakaway, so the hold says the result is not its quantity and the field stays a zero
    # never written
    assert (f["fric_breakaway_counts"]["value"], f["fric_breakaway_counts"]["reason"]) == (0, "never written")
    assert all(word in f["fric_breakaway_counts"]["held"] for word in ("driven ramp's breakaway", "back-driven"))
    assert "back-driven" in waits["fric_breakaway_counts"] and "is no breakaway" in waits["fric_breakaway_counts"]


# how a why ends when the entry is the pick of a bench sweep and no decision of the owner's
SWEEP_PICK = ": the pick of a bench sweep under a rule written before its session"


def test_the_policy_of_record_holds_what_the_rules_read():
    policy = json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    assert isinstance(policy["angle_cw_stop_deg"], float) and isinstance(policy["hold"], dict)
    assert set(policy["hold"]) <= set(cn.RULES)
    assert isinstance(policy["observer_hz"], float) and all(policy["hold"].values())
    # a choice that is a decision says so, under `why`, by the name of its entry, or of the field where the
    # value is not the policy file's: the registry's gear ratio, the two loop gains benched in counts, the
    # winding model's slow pair, the family's terms as a stage 4 result quotes them
    fields = cn.of_record(postable.default_pair())["fields"]
    assert policy["why"]["cutoff_c"] == "one thermal limit, at 90 C: the owner's decision"
    assert policy["cutoff_c"] == 90.0
    assert {k for k in policy["why"] if k not in policy} == {"gear_ratio_centi", "v_kp_q88", "v_ki_q412",
                                                             "thermal_tau2_s", "th_rise2_q016"} < set(fields)
    # a why ends in whose pick the entry is: the owner's decision, or a bench sweep's under a rule written
    # before its session. A why of the second kind names the sweep's rungs, each a results file in the repo,
    # and does not call the pick the owner's
    assert all(why.endswith((": the owner's decision", SWEEP_PICK)) for why in policy["why"].values())
    swept = {k: re.findall(r"zz-sweep-[a-z0-9-]+", why) for k, why in policy["why"].items()
             if why.endswith(SWEEP_PICK)}
    assert swept == {"observer_hz": [f"zz-sweep-observer-{hz}hz" for hz in (10, 15, 25)],
                     "current_loop_hz": [f"zz-sweep-current-loop-{hz}hz" for hz in (500, 1000, 1500)]}
    assert all((results.ROOT / "stages" / sc.STAGE / "scorecard" / "results" / postable.default_pair()
                / f"{rung}.json").exists() for rungs in swept.values() for rung in rungs)
    assert not any("the owner's decision" in policy["why"][k] for k in swept)
    assert {k: policy[k] for k in ("i_cont_ma", "stall_release_ma", "collision_trip", "hold_band_floor_counts")} == {
        "i_cont_ma": 250.0, "stall_release_ma": 90.0, "collision_trip": "off", "hold_band_floor_counts": 4}
    # an entry that restates a default nobody decided says that, under `undecided`, with what it needs: it
    # is no decision, so it has no why, and a field whose rule reads it has no source. None is left: the two
    # bandwidths left the list with their sweeps, and the two limits of motion with the owner's decision
    assert policy["undecided"] == {}
    # the default move speed and the acceleration that reaches it in 50 ms, each a decision with its why,
    # which names the sweep under the speed ceiling it rests on and says it is interim
    motion = {"velocity_limit_deg_s": 200.0, "accel_limit_deg_s2": 4000.0}
    assert {k: policy[k] for k in motion} == motion and motion["accel_limit_deg_s2"] * 0.05 == motion[
        "velocity_limit_deg_s"]
    assert re.findall(r"zz-sweep-[a-z0-9-]+", policy["why"]["velocity_limit_deg_s"]) == [
        f"zz-sweep-motion-{speed}" for speed in (75, 200, 400)]
    assert all("interim" in policy["why"][k] and policy["why"][k].endswith(": the owner's decision") for k in motion)
    # the decisions on the screens, the supply terms and the open loop, each an entry in the unit a person
    # sets it in, with its why
    decided = {"stall_response": "yield", "stall_time_ms": 500.0, "stall_omega_max_deg_s": 24.84, "pos_error_deg": 9.9,
               "pos_error_time_ms": 100.0, "sensor_delta_max_counts": 64, "sensor_bad_samples": 4,
               "v_undervolt_v": 3.7, "rail_source_ohm": 0.8, "duty_max_share": 1.0, "oc_trip_ma": 2200.0,
               "oc_trip_samples": 8, "openloop_decay": "slow", "openloop_zero_brake": "off"}
    assert {k: policy[k] for k in decided} == decided and set(decided) <= set(policy["why"])
    assert sum("interim" in policy["why"][k] for k in decided) == 3
    # where a field has several rules the policy names one of them, and names none for a field with one
    assert {n: policy["rule"][n] in of for n, of in cn.RULES.items() if len(of) > 1} == {
        "r_q12": True, "r_bemf_q12": True}
    assert set(policy["rule"]) == {"r_q12", "r_bemf_q12"}


def test_build_as_a_command(cli, geared, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    monkeypatch.setattr(cn, "registry_of", lambda pair: REG)
    cli("adopt", "--from", str(record(tmp_path)))
    inputs(tmp_path)
    # the cli fixture keeps the sets flat under tmp_path, the table file beside them
    table_file(tmp_path, stops=STOPS).rename(tmp_path / f"{PAIR}.pos_lut.json")
    of_record = (tmp_path / f"{PAIR}.constants.json").read_bytes()
    capsys.readouterr()
    cli("build", "--out", str(tmp_path / "derived.json"))
    out = capsys.readouterr().out.splitlines()
    assert out[0].endswith("derived.json") and "angle_max_cdeg: 18774 -> 18000 [measured, angle_max.table]" in out
    assert any(line.startswith("hash: plant moved: ") for line in out)
    assert (tmp_path / f"{PAIR}.constants.json").read_bytes() == of_record
    # without --out it is the set of record that is written, on top of the one that stood
    cli("build")
    doc = cn.load(tmp_path / f"{PAIR}.constants.json")
    assert doc["previous"] == cn.load(tmp_path / "derived.json")["previous"] and doc["pos_table"]["state"] == "LIVE"
    cli("status")
    assert "3 input files, none moved" in capsys.readouterr().out.splitlines()
    # a deviation is built on the set of record, to a path of its own
    f = dev_file(tmp_path, "soft", pins={"p_kp_q88": pin(5026)})
    cli("build", "--deviation", str(f), "--out", str(tmp_path / "soft.constants.json"))
    out = capsys.readouterr().out.splitlines()
    assert "p_kp_q88: 40212 -> 5026 [pinned]" in out
    assert "derived, not a set of record: deviations/soft.json moves p_kp_q88 40212 -> 5026" in out
    assert cn.load(tmp_path / "soft.constants.json")["deviation"]["fields"] == {"p_kp_q88": [40212, 5026]}
    assert cn.load(tmp_path / f"{PAIR}.constants.json") == doc
    with pytest.raises(SystemExit, match="refused: a deviation is built on the set of record, to --out"):
        cli("build", "--deviation", str(f))


# --- what a set is scored with ---

def current_scale(root, value=1135.0, **more):
    """The board's stage 1 current-scale result under `root`."""
    path = root / CURRENT
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"inputs": {}, "counts_per_a": {"value": value, "lo": value - 1.2, "hi": value + 1.2,
                                                               "unit": "counts/A"}, **more}))
    return path


def test_a_set_carries_the_current_scale_it_is_scored_with_under_a_hash_of_its_own(first, tmp_path):
    inputs(tmp_path)
    bare, _ = built_on(first, tmp_path)
    assert "scales" not in bare and "scales" not in bare["hash"]
    path = current_scale(tmp_path)
    doc, notes = built_on(bare, tmp_path)
    # the board's measured counts per A with its band, the result among the set's inputs
    assert doc["scales"] == {"current_counts_per_a": {
        "value": 1135.0, "lo": 1133.8, "hi": 1136.2, "unit": "counts/A", "class": "measured",
        "rule": "scales.current", "inputs": ["1-board/current-scale:counts_per_a"]}}
    assert doc["inputs"]["1-board/current-scale"] == {"file": CURRENT, "sha1": sc.file_sha1(path)}
    assert not any("current_counts_per_a" in n for n in notes)
    # it is on no servo: the plant, the protection and the set hash stand, and a hash of its own says it
    assert {k: doc["hash"][k] for k in bare["hash"]} == bare["hash"]
    assert doc["hash"]["scales"] == sc.config_hash({"current_counts_per_a": 1135.0})
    assert cn.changes(bare, doc) == [
        "scored with current_counts_per_a: not in the set -> 1135.0",
        f"hash: scales moved: plant {bare['hash']['plant'][:12]} -> {bare['hash']['plant'][:12]}, protection "
        f"{bare['hash']['protection'][:12]} -> {bare['hash']['protection'][:12]}, set {bare['hash']['set'][:12]} "
        f"-> {bare['hash']['set'][:12]}, scales none -> {doc['hash']['scales'][:12]}"]
    assert cn.described(doc)[-2:] == [
        "scored with current_counts_per_a: 1135.0000 counts/A [measured]",
        f"hash: plant {doc['hash']['plant'][:12]}, protection {doc['hash']['protection'][:12]} (3 fields), set "
        f"{doc['hash']['set'][:12]}, scales {doc['hash']['scales'][:12]}"]
    # the file renders, loads and builds again to the same bytes
    out = cn.write(doc, tmp_path / "x.constants.json")
    assert cn.load(out) == doc and cn.render(built_on(bare, tmp_path)[0]) == out.read_text()
    assert cn.install(doc, root=tmp_path)[-1] == "stamp" and cn.check(doc) == []
    # a scale that moves is a build that moves that hash alone, and the set it stood in is stale
    current_scale(tmp_path, 1140.0)
    lines, stale = cn.status(doc, tmp_path, REG)
    assert stale and lines[0] == (f"stale: 1-board/current-scale ({CURRENT}) is no longer the file the set read, "
                                  f"sha1 {sc.file_sha1(path)}: current_counts_per_a")
    again, _ = built_on(doc, tmp_path)
    assert [k for k in doc["hash"] if doc["hash"][k] != again["hash"][k]] == ["scales"]
    assert cn.changes(doc, again)[0] == "scored with current_counts_per_a: 1135.0 -> 1140.0 [measured, scales.current]"
    # a build of protection fields carries it, and is refused while it would move
    with pytest.raises(ValueError, match="what the set is scored with would move: build the plant first"):
        built_on(doc, tmp_path, protection=["current_limit_counts"])
    assert built_on(again, tmp_path, protection=["current_limit_counts"])[0]["scales"] == again["scales"]
    # without its result the scale is held at what the set had, the missing input named
    path.unlink()
    held, notes = built_on(again, tmp_path)
    assert held["scales"]["current_counts_per_a"] == {
        "value": 1140.0, "lo": 1138.8, "hi": 1141.2, "unit": "counts/A", "class": "none", "rule": "scales.current",
        "held": "1-board/current-scale has no result for board", "carried_from": again["hash"]["set"]}
    assert "current_counts_per_a: held, 1-board/current-scale has no result for board" in notes
    # an edited scale is not the set the tool wrote
    edited = copy.deepcopy(doc)
    edited["scales"]["current_counts_per_a"]["value"] = 1108.5
    out.write_text(cn.render(edited))
    with pytest.raises(ValueError, match="hashes are not those of its content"):
        cn.load(out)


# --- the values read off the maker's sheet ---

CLASS_REG = BOARD_REG
RAIL_REG = {**CLASS_REG, "servo": {**CLASS_REG["servo"], "rated_v": 5.0}}     # and the rail it runs on
DERATE = {"datasheet_derate_speed": [2, 3]}
CEILING = math.floor(400 * SCALE)   # two thirds of 60 deg in 0.10 s, counts per second on the fake's table, down
CEILING_6V = math.floor(500 * SCALE)    # the same at the 6.0 V row: 10305.6 counts per second, rounded down
NO_TIME = "the datasheet record states no one no-load time at its operating voltage, 5 V"
VOLTS_RANGE = (4.8, 6.6)            # a range of operating voltage, with both of the sheet's two speeds in it
BY_RAIL = {**DERATE, "datasheet_speed_voltage": "rated_v"}
IN_RANGE = "in its operating range, 4.8 V to 6.6 V"


def datasheet(root, volts=4.8):
    """The datasheet record of the fake servo's class under `root`: an
    operating voltage, a range of it for a (min, max), and the no-load
    time over 60 deg and the stall torque at two."""
    path = root / CLASS_SHEET
    path.parent.mkdir(parents=True, exist_ok=True)
    of = dict(zip(("min", "max"), volts)) if isinstance(volts, tuple) else {"value": volts}
    path.write_text(json.dumps({
        "what": "what the maker states of the class", "class": CLASS, "document": DOCUMENT,
        "stated": {"operating_voltage": {**of, "unit": "V"},
                   "no_load_time": [{"value": 0.10, "unit": "s per 60 deg", "per_deg": 60, "at_v": 4.8},
                                    {"value": 0.08, "unit": "s per 60 deg", "per_deg": 60, "at_v": 6.0}],
                   "stall_torque": [{"value": 1.8, "unit": "kg cm", "at_v": 4.8},
                                    {"value": 2.2, "unit": "kg cm", "at_v": 6.0}]}}))
    return path


def classed(previous, root, registry=CLASS_REG):
    """A build for the fake servo as one of its class: the plant on
    `previous`, then the speed ceiling, a protection field, by a build
    that names it."""
    plain, _ = cn.build(PAIR, previous, root=root, registry=registry)
    return cn.build(PAIR, plain, protection=[cn.CEILING], root=root, registry=registry)


# what the entry of the ceiling reads, in order, of a sheet with one operating voltage
SHEET_READS = ["registry:datasheet_class", "datasheet:operating_voltage", "datasheet:no_load_time",
               "policy:datasheet_derate_speed", "scale"]


def test_the_speed_ceiling_is_the_makers_no_load_speed_derated_at_the_sets_scale(first, tmp_path):
    inputs(tmp_path, policy=DERATE)
    path = datasheet(tmp_path)
    # a servo that names no class has no sheet: its set holds no ceiling, and the build says why
    bare, notes = built_on(first, tmp_path)
    assert cn.CEILING not in bare["fields"] and "limits" not in bare and NO_CEILING in notes
    # a protection field: a build of the plant leaves it to a build that names it
    plain, notes = cn.build(PAIR, bare, root=tmp_path, registry=CLASS_REG)
    assert cn.CEILING not in plain["fields"] and "speed_ceiling_cps: left out, name it with --protection" in notes
    doc, notes = classed(bare, tmp_path)
    # 60 deg in 0.10 s at the sheet's operating voltage is 600 deg/s, two thirds of it 400 deg/s, and that in
    # counts per second at the set's scale: the record, the policy and the scale, no number typed
    assert doc["fields"][cn.CEILING] == {
        "value": CEILING, "class": "datasheet", "rule": "speed_ceiling.datasheet", "rule_rev": 1, "unit": "count/s",
        "q": 0, "pos_unit": 1, "scale_points_sha1": cn.points_sha1(POINTS), "inputs": SHEET_READS,
        "deg_s": 400.0, "stated_deg_s": 600.0, "at_v": 4.8, "derate": [2, 3],
        "record": "what the maker states of the class", "document": DOCUMENT, "protection": {"looser": "up"}}
    assert CEILING == 8244 and not any(n.startswith(cn.CEILING) for n in notes)
    # the record is an input of the set by its sha1, the class the registry's
    assert doc["inputs"]["datasheet"] == {"file": CLASS_SHEET, "sha1": sc.file_sha1(path)}
    assert doc["inputs"]["registry"]["servo"] == CLASS_REG["servo"]
    # the servo holds it: the protection hash moves and the plant stands, and no hash of a limit is left
    assert moved_hashes(plain, doc) == ["protection"] and "limits" not in doc["hash"]
    assert cn.changes(plain, doc)[0] == f"speed_ceiling_cps: not in the set -> {CEILING}"
    # status says the ceiling with its source: the maker's value, the derate, the document, what it bounds
    lines, stale = cn.status(doc, tmp_path, CLASS_REG)
    assert not stale and lines[lines.index(cn.described(doc)[-1]) + 1] == (
        f"speed ceiling: {CEILING} count/s, 400 deg/s at the set's scale [datasheet, speed_ceiling.datasheet]: the "
        f"maker's no-load speed at 4.8 V, 600 deg/s, x 2/3 by the policy (datasheet_derate_speed); {DOCUMENT}; it "
        f"bounds velocity_limit_cps, not in the set")
    # the file renders, loads and builds again to the same bytes
    out = cn.write(doc, tmp_path / "x.constants.json")
    assert cn.load(out) == doc and cn.render(classed(bare, tmp_path)[0]) == out.read_text()
    # a record that moves makes the set stale, and the build after moves the protection hash alone: at
    # another operating voltage the ceiling is the speed the sheet states there, derated
    datasheet(tmp_path, volts=6.0)
    lines, stale = cn.status(doc, tmp_path, CLASS_REG)
    assert stale and lines[0] == (f"stale: datasheet ({CLASS_SHEET}) is no longer the file the set read, sha1 "
                                  f"{sc.file_sha1(path)}: speed_ceiling_cps")
    again, _ = cn.build(PAIR, doc, protection=[cn.CEILING], root=tmp_path, registry=CLASS_REG)
    e = again["fields"][cn.CEILING]
    assert (e["stated_deg_s"], e["deg_s"], e["at_v"], e["value"]) == (750.0, 500.0, 6.0, CEILING_6V)
    assert CEILING_6V == 10305 and round(500 * SCALE) == 10306
    assert moved_hashes(doc, again) == ["protection"]
    assert cn.changes(doc, again)[0] == (f"speed_ceiling_cps: {CEILING} -> {CEILING_6V} "
                                         f"[datasheet, speed_ceiling.datasheet]")
    # an edited ceiling is not the set the tool wrote
    for change, match in (
            (lambda e: e.update(value=60000), "hashes are not those of its content"),
            (lambda e: e.update(protection=None), "speed_ceiling_cps says None of protection"),
            (lambda e: e.pop("document"), "speed_ceiling_cps is of class datasheet and names no record or no document"),
            (lambda e: e.update(scale_points_sha1="0" * 40), "speed_ceiling_cps was computed for the table of")):
        edited = copy.deepcopy(doc)
        change(edited["fields"][cn.CEILING])
        out.write_text(cn.render(edited))
        with pytest.raises(ValueError, match=match):
            cn.load(out)


def test_a_set_written_while_the_ceiling_was_a_limit_loads_and_its_limit_stands_until_the_field_is_named(
        first, tmp_path):
    inputs(tmp_path, policy=DERATE)
    datasheet(tmp_path)
    doc, _ = classed(first, tmp_path)
    # the set as a build wrote it while the table had no field for the ceiling: a bound under `limits`, the
    # fields it bounds named, in no hash of what the servo is to hold and under a hash of its own
    old = copy.deepcopy(doc)
    e = old["fields"].pop(cn.CEILING)
    old["limits"] = {cn.CEILING: {**{k: v for k, v in e.items() if k != "protection"},
                                  "bounds": ["velocity_limit_cps"]}}
    old["hash"] = cn.hashes(old)
    assert old["hash"]["limits"] == sc.config_hash({cn.CEILING: CEILING})
    path = cn.write(old, tmp_path / "old.constants.json")
    assert cn.load(path) == old and cn.ceiling_said(old)[0].startswith(f"speed ceiling: {CEILING} count/s, 400 deg/s")
    # it bounds what it bounded, and is no limit as the tool wrote one without the fields it bounds
    over = copy.deepcopy(old)
    over["fields"]["velocity_limit_cps"] = {"value": CEILING + 1, "class": "pinned", "why": "a test's", "rung": "x",
                                            "protection": None}
    assert cn.over_limit(over) == [f"velocity_limit_cps: {CEILING + 1} count/s is over the set's speed_ceiling_cps, "
                                   f"{CEILING} count/s (400 deg/s): a field stays at or under the limit that bounds it"]
    bare = copy.deepcopy(old)
    bare["limits"][cn.CEILING]["bounds"] = []
    bare["hash"] = cn.hashes(bare)
    with pytest.raises(ValueError, match="speed_ceiling_cps is no limit as the tool writes one"):
        cn.load(cn.write(bare, path))
    # a build of the plant carries the limit as it stood, and says what names the field
    plain, notes = cn.build(PAIR, old, root=tmp_path, registry=CLASS_REG)
    assert plain["limits"] == old["limits"] and plain["hash"] == old["hash"] and cn.CEILING not in plain["fields"]
    assert f"limit speed_ceiling_cps: carried at {CEILING}, the set holds no field of it: name it with --protection" \
        in notes
    # named, the field joins the set and the limit leaves it: the protection hash moves, the limits' goes
    named, notes = cn.build(PAIR, old, protection=[cn.CEILING], root=tmp_path, registry=CLASS_REG)
    assert named["fields"][cn.CEILING] == doc["fields"][cn.CEILING] and "limits" not in named
    assert moved_hashes(old, named) == ["protection"] and "limits" not in named["hash"]
    assert cn.changes(old, named)[:2] == [f"limit speed_ceiling_cps: {CEILING} -> not in the set",
                                          f"speed_ceiling_cps: not in the set -> {CEILING}"]
    # the copy a rung of record ran on while the ceiling was a limit loads as it was written
    rung = results.ROOT / "telemetry" / postable.default_pair() / "scorecard" / "19a-lifecycle-heat-mean" / sc.SET_COPY
    ran = cn.load(rung)
    assert ran["limits"][cn.CEILING]["value"] == 8051 and cn.CEILING not in ran["fields"]
    assert cn.render(ran) == rung.read_text()


def test_a_ceiling_whose_input_is_missing_is_held_and_does_not_guess(first, tmp_path):
    inputs(tmp_path, policy=DERATE)
    path = datasheet(tmp_path)
    doc, _ = classed(first, tmp_path)
    # the sheet states no time at its operating voltage: the ceiling the set had stands, the input named
    datasheet(tmp_path, volts=5.0)
    held, notes = classed(doc, tmp_path)
    assert held["fields"][cn.CEILING] == {
        "value": CEILING, "class": "none", "rule": "speed_ceiling.datasheet", "held": NO_TIME,
        "carried_from": doc["hash"]["set"], "protection": {"looser": "up"}}
    assert f"speed_ceiling_cps: held, {NO_TIME}" in notes and held["hash"] == doc["hash"]
    assert cn.load(cn.write(held, tmp_path / "x.constants.json")) == held
    assert (f"speed ceiling: {CEILING} count/s [none, held, speed_ceiling.datasheet]: {NO_TIME}; it bounds "
            f"velocity_limit_cps, not in the set") in cn.status(held, tmp_path, CLASS_REG)[0]
    # with nothing to carry it is left out and said: a record that is not there, a derate nobody decided
    path.unlink()
    none, notes = classed(first, tmp_path)
    assert cn.CEILING not in none["fields"]
    assert f"speed_ceiling_cps: left out, the servo class {CLASS} has no datasheet record ({CLASS_SHEET})" in notes
    datasheet(tmp_path)
    inputs(tmp_path)
    none, notes = classed(first, tmp_path)
    assert cn.CEILING not in none["fields"]
    assert "speed_ceiling_cps: left out, policy.json has no datasheet_derate_speed" in notes
    # a derate is a ratio of at most one, written as it was decided
    for bad in (0.6667, [3, 2], [2, 3, 4], [2.0, 3.0]):
        inputs(tmp_path, policy={"datasheet_derate_speed": bad})
        with pytest.raises(ValueError, match="datasheet_derate_speed is .*: a derate is a ratio of at most one"):
            classed(first, tmp_path)


def test_a_range_of_operating_voltage_reads_the_speed_at_the_voltage_the_policy_says(first, tmp_path):
    def at(policy, rail=5.0, volts=VOLTS_RANGE, previous=first):
        inputs(tmp_path, policy=policy)
        datasheet(tmp_path, volts)
        return classed(previous, tmp_path, {**RAIL_REG, "servo": {**RAIL_REG["servo"], "rated_v": rail}})

    def read(doc):
        e = doc["fields"][cn.CEILING]
        return e["at_v"], e["stated_deg_s"], e["deg_s"], e["value"]

    # a sheet with one operating voltage names one speed, the policy and the rail unread, and one with a range
    # that holds both of its speeds names none: the policy says the stated voltage nearest the rail the
    # registry holds, 4.8 V on 5 V, and the ceiling is the single sheet's
    single, _ = at(DERATE, volts=4.8)
    assert "voltage" not in single["fields"][cn.CEILING] and "rated_v" not in single["inputs"]["registry"]["servo"]
    doc, notes = at(BY_RAIL)
    assert doc["fields"][cn.CEILING] == {
        **single["fields"][cn.CEILING], "voltage": "rated_v",
        "inputs": ["registry:datasheet_class", "datasheet:operating_voltage", "datasheet:no_load_time",
                   "policy:datasheet_speed_voltage", "registry:rated_v", "policy:datasheet_derate_speed", "scale"]}
    assert read(doc) == (4.8, 600.0, 400.0, CEILING) and not any(n.startswith(cn.CEILING) for n in notes)
    assert doc["inputs"]["registry"] == {"servo": RAIL_REG["servo"]}
    assert doc["hash"] == single["hash"]
    lines, stale = cn.status(doc, tmp_path, RAIL_REG)
    assert not stale and (
        f"speed ceiling: {CEILING} count/s, 400 deg/s at the set's scale [datasheet, speed_ceiling.datasheet]: the "
        f"maker's no-load speed at 4.8 V, the stated voltage nearest the servo's rated_v of 5 V by the policy "
        f"(datasheet_speed_voltage), 600 deg/s, x 2/3 by the policy (datasheet_derate_speed); {DOCUMENT}; it "
        f"bounds velocity_limit_cps, not in the set") in lines
    out = cn.write(doc, tmp_path / "x.constants.json")
    assert cn.load(out) == doc and cn.render(at(BY_RAIL)[0]) == out.read_text()
    # the rail is the registry's: one that moves makes the set stale, and the nearest stated voltage follows
    # it, the lower of two as near and never one outside the range
    lines, stale = cn.status(doc, tmp_path, {**RAIL_REG, "servo": {**RAIL_REG["servo"], "rated_v": 6.0}})
    assert stale and lines[0].startswith("stale: the registry's servo rated_v was 5.0, it is 6.0")
    for rail, row in ((4.0, 4.8), (5.3, 4.8), (5.4, 4.8), (5.5, 6.0), (6.0, 6.0), (7.4, 6.0)):
        assert read(at(BY_RAIL, rail)[0])[0] == row, rail
    assert read(at(BY_RAIL, 6.0)[0]) == (6.0, 750.0, 500.0, CEILING_6V)
    assert read(at(BY_RAIL, 7.4, volts=(4.8, 5.5))[0]) == (4.8, 600.0, 400.0, CEILING)
    # a number of volts is the stated voltage itself, and status says the policy named it
    named, _ = at({**DERATE, "datasheet_speed_voltage": 6})
    assert read(named) == (6.0, 750.0, 500.0, CEILING_6V)
    assert named["fields"][cn.CEILING]["voltage"] == 6 and type(named["fields"][cn.CEILING]["at_v"]) is float
    assert "registry:rated_v" not in named["fields"][cn.CEILING]["inputs"]
    assert any("the maker's no-load speed at 6 V, the voltage the policy names (datasheet_speed_voltage), 750 deg/s"
               in line for line in cn.status(named, tmp_path, RAIL_REG)[0])
    # a voltage the record has no speed at, an entry nobody decided, a rail the registry does not hold and a
    # range with no speed in it: the ceiling the set had stands, the input named, and is left out with none
    for policy, volts, why in (
            ({**DERATE, "datasheet_speed_voltage": 5.0}, VOLTS_RANGE,
             f"the datasheet record states no one no-load time at 5 V, the voltage policy.json names "
             f"(datasheet_speed_voltage), {IN_RANGE}"),
            ({**DERATE, "datasheet_speed_voltage": 7.4}, VOLTS_RANGE,
             f"the datasheet record states no one no-load time at 7.4 V, the voltage policy.json names "
             f"(datasheet_speed_voltage), {IN_RANGE}"),
            ({**DERATE, "datasheet_speed_voltage": 6.0}, (4.8, 5.5),
             "the datasheet record states no one no-load time at 6 V, the voltage policy.json names "
             "(datasheet_speed_voltage), in its operating range, 4.8 V to 5.5 V"),
            (DERATE, VOLTS_RANGE, "policy.json has no datasheet_speed_voltage"),
            ({**DERATE, "datasheet_speed_voltage": "pack_v"}, VOLTS_RANGE,
             "the registry has no servo pack_v for synth__board"),
            (BY_RAIL, (5.0, 5.5),
             "the datasheet record states no one no-load time at a voltage of its operating range, 5 V to 5.5 V")):
        held, notes = at(policy, volts=volts, previous=doc)
        assert held["fields"][cn.CEILING] == {
            "value": CEILING, "class": "none", "rule": "speed_ceiling.datasheet", "held": why,
            "carried_from": doc["hash"]["set"], "protection": {"looser": "up"}}
        assert f"speed_ceiling_cps: held, {why}" in notes and held["hash"] == doc["hash"]
        none, notes = at(policy, volts=volts)
        assert cn.CEILING not in none["fields"] and f"speed_ceiling_cps: left out, {why}" in notes
    # the entry is a number of volts or the registry's name of a voltage, and nothing else
    for bad in ([4.8], True, None, "gear_class"):
        with pytest.raises(ValueError, match="datasheet_speed_voltage is .*: it is a voltage the sheet states a "
                                             "no-load time at, in volts, or the registry's name of a voltage"):
            at({**DERATE, "datasheet_speed_voltage": bad})


def test_a_set_whose_speed_limit_is_over_the_ceiling_is_refused(first, tmp_path):
    datasheet(tmp_path)
    plant(tmp_path, POT_NOISE)

    def at(deg_s):
        inputs(tmp_path, policy={**COUNT_POLICY, **DERATE, "velocity_limit_deg_s": deg_s})
        return classed(first, tmp_path)[0]

    # the move's speed limit binds under the ceiling and at it, and status says where it stands
    doc = at(75.0)
    assert doc["fields"]["velocity_limit_cps"]["value"] == round(75 * SCALE) < CEILING and cn.over_limit(doc) == []
    doc = at(400.0)
    assert doc["fields"]["velocity_limit_cps"]["value"] == doc["fields"][cn.CEILING]["value"] == CEILING
    assert any(line.endswith(f"; it bounds velocity_limit_cps = {CEILING}")
               for line in cn.status(doc, tmp_path, CLASS_REG)[0])
    # over it the set is refused, by the build, by a deviation and by load
    over = (f"velocity_limit_cps: {{}} count/s is over the set's speed_ceiling_cps, {CEILING} count/s "
            f"\\(400 deg/s\\): a field stays at or under the limit that bounds it")
    with pytest.raises(ValueError, match=over.format(round(401 * SCALE))):
        at(401.0)
    base = at(75.0)
    cn.write(base, cn.set_path(PAIR, tmp_path))
    fast = dev_file(tmp_path, "fast", pins={"velocity_limit_cps": pin(CEILING + 1)})
    with pytest.raises(ValueError, match=over.format(CEILING + 1)):
        cn.derive(base, fast, tmp_path, CLASS_REG)
    faster = dev_file(tmp_path, "faster", policy={"velocity_limit_deg_s": 500.0})
    with pytest.raises(ValueError, match=over.format(round(500 * SCALE))):
        cn.derive(base, faster, tmp_path, CLASS_REG)
    top = dev_file(tmp_path, "top", pins={"velocity_limit_cps": pin(CEILING)})
    assert cn.derive(base, top, tmp_path, CLASS_REG)[0]["deviation"]["fields"] == {
        "velocity_limit_cps": [round(75 * SCALE), CEILING]}
    # a deviation that lowers the ceiling under the move's speed limit is refused the same way
    low = dev_file(tmp_path, "low", pins={cn.CEILING: pin(round(75 * SCALE) - 1)})
    with pytest.raises(ValueError, match=f"velocity_limit_cps: {round(75 * SCALE)} count/s is over the set's "
                                         f"speed_ceiling_cps, {round(75 * SCALE) - 1} count/s: a field stays"):
        cn.derive(base, low, tmp_path, CLASS_REG)
    path = tmp_path / "over.constants.json"
    path.write_text(cn.render(deviated(base, velocity_limit_cps=CEILING + 1)))
    with pytest.raises(ValueError, match=over.format(CEILING + 1)):
        cn.load(path)


def test_the_set_of_record_holds_the_speed_ceiling_of_its_servo_class():
    pair = postable.default_pair()
    doc = cn.of_record(pair)
    servo = cn.servos.SERVOS[pair.partition("__")[0]]
    path = cn.sheet_path(pair, servo.datasheet_class)
    sheet = json.loads(path.read_text())
    # the record: the servo's own maker's page with its address, the day it was last edited and the day it
    # was read, what it states and what it does not
    assert (sheet["class"], sheet["maker"], sheet["read"]) == (servo.datasheet_class, "Vorpal Robotics", "2026-11-14")
    assert servo.datasheet_class == "vorpal-mg90" and sheet["last_edited"] == "2018-08-01"
    assert sheet["url"] == "https://www.vorpalrobotics.com/wiki/index.php/Vorpal_MG90_Micro_Servo"
    assert all(sheet[k] in sheet["document"] for k in ("url", "last_edited", "read"))
    assert "the maker's own page" in sheet["document"]
    assert sheet["stated"] == {
        "operating_voltage": {"min": 4.8, "max": 6.6, "unit": "V"},
        "no_load_time": [{"value": 0.10, "unit": "s per 60 deg", "per_deg": 60, "at_v": 4.8},
                         {"value": 0.08, "unit": "s per 60 deg", "per_deg": 60, "at_v": 6.0}],
        "stall_torque": [{"value": 1.8, "unit": "kg cm", "at_v": 4.8}, {"value": 2.2, "unit": "kg cm", "at_v": 6.0}]}
    assert "Operating voltage: 4.8V to 6.6V" in sheet["as_printed"]
    assert "Operating speed: 0.10sec/60degree (4.8V); 0.08sec/60degree (6.0V)" in sheet["as_printed"]
    assert all(word in sheet["not_stated"] for word in ("current", "travel", "temperature range"))
    # the servo is the maker's own, and nothing says it is a clone with no sheet
    assert "its own maker's statement" in sheet["stands_for"][servo.key]
    assert "clone" not in path.read_text() and "no sheet" not in path.read_text()
    # what the maker says of the TowerPro MG90S is quoted as the maker's claim, and no value rests on it
    claim = sheet["maker_claim"]
    assert claim["quoted"] == ["We contracted directly with Tower Pro to create these servos",
                               "The gears and plastic housing are identical to standard Tower Pro MG90S servos"]
    assert claim["standing"].startswith("the maker's claim, quoted") and "no value" in claim["standing"]
    assert "/Users/" not in path.read_text() and "/home/" not in path.read_text() and path.read_text().isascii()
    # this servo's derates and the voltages its speed and its stall are read at are decisions for it, with
    # their why, and no rule for another servo; the speed voltage's why says what the other row would give and
    # that it is not taken
    policy = json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    assert (policy["datasheet_derate_speed"], policy["datasheet_derate_default"]) == ([2, 3], 0.8)
    assert policy[cn.SPEED_VOLTAGE] == policy[cn.STALL_VOLTAGE] == "rated_v" and servo.rated_v == 5.0
    for key in ("datasheet_derate_speed", "datasheet_derate_default", cn.SPEED_VOLTAGE, cn.STALL_VOLTAGE):
        assert "for this servo and no rule for another" in policy["why"][key] and key not in policy["undecided"]
    why = policy["why"][cn.SPEED_VOLTAGE]
    assert "nearest the rail this servo runs on" in why and why.endswith(": the owner's decision")
    assert "The 6.0 V row, 750 deg/s, would give 500 deg/s and is not taken" in why
    # the ceiling: of the two voltages the page states a speed at, 4.8 V is the nearest the servo's rail,
    # 600 deg/s no load there, two thirds of it, at the pair's own scale
    e = doc["fields"][cn.CEILING]
    speeds = {q["at_v"]: q["per_deg"] / q["value"] for q in sheet["stated"]["no_load_time"]}
    assert min(speeds, key=lambda v: abs(v - servo.rated_v)) == e["at_v"] == 4.8
    assert (e["stated_deg_s"], e["voltage"], e["derate"], e["deg_s"]) == (speeds[4.8], "rated_v", [2, 3], 400.0)
    assert speeds == {4.8: 600.0, 6.0: 750.0}
    # 400 deg/s is 8050.65 counts per second at the pair's scale, rounded down: a derated cap never passes its
    # derate
    assert 400 * doc["scale"]["counts_per_deg"] == pytest.approx(8050.65, abs=0.01)
    assert e["value"] == math.floor(400 * doc["scale"]["counts_per_deg"]) == 8050 and e["unit"] == "count/s"
    assert (e["class"], e["document"], e["record"]) == ("datasheet", sheet["document"], sheet["what"])
    assert e["scale_points_sha1"] == doc["scale"]["points_sha1"] and e["protection"] == {"looser": "up"}
    assert e["inputs"] == ["registry:datasheet_class", "datasheet:operating_voltage", "datasheet:no_load_time",
                           "policy:datasheet_speed_voltage", "registry:rated_v", "policy:datasheet_derate_speed",
                           "scale"]
    assert doc["inputs"]["datasheet"] == {"file": path.relative_to(results.ROOT).as_posix(),
                                          "sha1": sc.file_sha1(path)}
    assert doc["inputs"]["registry"]["servo"]["datasheet_class"] == "vorpal-mg90"
    assert doc["inputs"]["registry"]["servo"]["rated_v"] == servo.rated_v
    # it is a field the servo holds, under the protection hash, and the set holds no limit beside its fields:
    # the move's speed limit is the owner's default move speed, a policy entry of its own, under the ceiling
    assert "limits" not in doc and "limits" not in doc["hash"] and cn.CEILING in cn.PROTECTION
    limit = doc["fields"]["velocity_limit_cps"]
    assert (limit["value"], limit["class"]) == (4025, "policy") and limit["value"] <= e["value"]
    assert limit["inputs"] == ["policy:velocity_limit_deg_s", "scale"] and cn.classes(doc)["datasheet"] == 1
    assert cn.over_limit(doc) == []
    lines, stale = cn.status(doc)
    assert not stale and (
        "speed ceiling: 8050 count/s, 400 deg/s at the set's scale [datasheet, speed_ceiling.datasheet]: the "
        "maker's no-load speed at 4.8 V, the stated voltage nearest the servo's rated_v of 5 V by the policy "
        "(datasheet_speed_voltage), 600 deg/s, x 2/3 by the policy (datasheet_derate_speed); "
        f"{sheet['document']}; it bounds velocity_limit_cps = 4025") in lines
    assert "Vorpal Robotics wiki" in sheet["document"]


def test_the_set_of_record_holds_the_current_limit_the_makers_stall_gives_at_its_derate():
    doc, c, _ = of_the_bench_pair()
    pair = postable.default_pair()
    sheet = json.loads(cn.sheet_path(pair, cn.servos.SERVOS[pair.partition("__")[0]].datasheet_class).read_text())
    policy = json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    e = doc["fields"]["current_limit_counts"]
    # the voltage the page states the stall torque at nearest the servo's rail, 4.8 V on 5.0 V, over the
    # winding's resistance of the stage 3 rails fit, at the 0.8 the policy takes a maker's value at, in the
    # board's measured counts per A: the owner's 0.92 A, about 0.8 x 4.8 V / 4.177 ohm
    ohm = c.q(*cn.RESISTANCES["rails_fit"])
    counts_per_a = doc["scales"][cn.CURRENT]["value"]
    assert [q["at_v"] for q in sheet["stated"]["stall_torque"]] == [4.8, 6.0]
    assert (e["at_v"], e["voltage"], e["derate"]) == (4.8, "rated_v", policy["datasheet_derate_default"]) == (
        4.8, "rated_v", 0.8)
    assert round(ohm, 3) == 4.177 and round(counts_per_a, 2) == 1135.01
    assert e["ma"] == pytest.approx(1000 * 0.8 * 4.8 / ohm) and round(e["ma"]) == 919
    # 1043.51 counts, rounded down: a derated cap never passes its derate
    assert 0.8 * 4.8 / ohm * counts_per_a == pytest.approx(1043.505, abs=1e-3)
    assert (e["value"], e["class"], e["rule"], e["protection"]) == (1043, "policy", "current_limit.stall_derate",
                                                                   {"looser": "up"})
    assert (e["record"], e["document"]) == (sheet["what"], sheet["document"]) and e["band"] == [1039, 1045]
    assert e["inputs"] == ["registry:datasheet_class", "datasheet:operating_voltage", "datasheet:stall_torque",
                           "policy:datasheet_stall_voltage", "registry:rated_v",
                           "3-servo/resistance:r_rails_at_measured_e_ohm", "policy:datasheet_derate_default",
                           "1-board/current-scale:counts_per_a"]
    assert cn.evaluate(cn.rule_of(c, "current_limit_counts"), c)["value"] == e["value"]
    # the 500 mA it replaces is no entry of the policy, and no field reads it
    assert "current_limit_ma" not in policy and "current_limit_ma" not in policy["why"]
    assert "500 mA it replaces, which was conservative" in policy["why"][cn.STALL_VOLTAGE]
    # a derate is a share of a maker's value, and a stall voltage is chosen as a speed's is
    for bad in (0, 1.2, "0.8", [4, 5]):
        c.policies = {**c.policies, cn.DERATE: bad}
        with pytest.raises(ValueError, match="datasheet_derate_default is .*: a derate is a share of a maker's value"):
            cn.current_limit_a(c)
    c.policies = {**c.policies, cn.DERATE: 0.8, cn.STALL_VOLTAGE: [4.8]}
    with pytest.raises(ValueError, match="datasheet_stall_voltage is .*: it is a voltage the sheet states a stall "
                                         "torque at"):
        cn.current_limit_a(c)


def test_a_datasheet_record_stands_for_the_servos_that_name_its_class():
    folder = results.ROOT / cn.FOLDER
    records = {path.name.removesuffix(".datasheet.json"): json.loads(path.read_text())
               for path in sorted(folder.glob("*.datasheet.json"))}
    assert sorted(records) == ["mg90s", "vorpal-mg90"]
    # a record is of its class, names its page and the day it was read, and stands for the servos of the
    # registry that name the class, each with why, and for no other
    for name, sheet in records.items():
        assert sheet["class"] == name and sheet["url"] in sheet["document"] and sheet["read"] in sheet["document"]
        assert sorted(sheet["stands_for"]) == sorted(k for k, s in cn.servos.SERVOS.items()
                                                     if s.datasheet_class == name)
        assert all(sheet["stands_for"].values())
    # the TowerPro page is the record of its own class and stands for no servo: the bench servo has a maker
    # of its own, and nothing says it is a clone with no sheet
    sheet, text = records["mg90s"], (folder / "mg90s.datasheet.json").read_text()
    assert sheet["stands_for"] == {} and "clone" not in text and "mg90-d" not in text
    assert (sheet["maker"], sheet["read"]) == ("TowerPro", "2026-11-14")
    assert sheet["url"].startswith("https://www.towerpro.com.tw/product/mg90s")
    assert sheet["stated"] == {
        "operating_voltage": {"value": 4.8, "unit": "V"},
        "no_load_time": [{"value": 0.10, "unit": "s per 60 deg", "per_deg": 60, "at_v": 4.8},
                         {"value": 0.08, "unit": "s per 60 deg", "per_deg": 60, "at_v": 6.0}],
        "stall_torque": [{"value": 1.8, "unit": "kg cm", "at_v": 4.8}],
        "temperature_min": {"value": 0, "unit": "C"}, "temperature_max": {"value": 55, "unit": "C"}}
    assert all(word in sheet["not_stated"] for word in ("current", "travel", "voltage range"))
    assert "/Users/" not in text and "/home/" not in text and text.isascii()
    # the two pages state the same two speeds, so the ceiling stands where the TowerPro record put it
    assert sheet["stated"]["no_load_time"] == records["vorpal-mg90"]["stated"]["no_load_time"]


# --- a deviation ---

def pin(value, rung="a-rung"):
    return {"value": value, "why": "a test's", "rung": rung}


def dev_file(root, name, **doc):
    path = root / "deviations" / f"{name}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc))
    return path


@pytest.fixture
def base(first, tmp_path):
    """The pair's set of record as a build gives it: on its table, its
    slots and its angle map."""
    inputs(tmp_path)
    doc, _ = built_on(first, tmp_path)
    cn.write(doc, cn.set_path(PAIR, tmp_path))
    return doc


def test_a_deviation_of_policy_goes_through_the_rules_and_is_named_in_the_set(base, tmp_path):
    f = dev_file(tmp_path, "cw-5", note="the clockwise stop at 5 deg", policy={"angle_cw_stop_deg": 5.0})
    doc, _ = cn.derive(base, f, tmp_path, REG)
    # the entry moves what its rules give, both ends of the angle map at the set's scale: no count is typed
    assert doc["deviation"] == {"file": "deviations/cw-5.json", "sha1": sc.file_sha1(f),
                                "fields": {"angle_min_cdeg": [0, 500], "angle_max_cdeg": [18000, 18500]}}
    sha = cn.points_sha1(POINTS)
    assert (doc["fields"]["angle_max_cdeg"]["class"], doc["fields"]["angle_max_cdeg"]["scale_points_sha1"]) == (
        "measured", sha)
    # it is the set with that one thing changed: the file among its inputs, the set before it named
    assert doc["inputs"] == {**base["inputs"], "deviation": {"file": "deviations/cw-5.json", "sha1": sc.file_sha1(f)}}
    assert doc["previous"] == base["hash"] and moved_hashes(base, doc) == ["plant"]
    assert all(doc["fields"][n]["value"] == e["value"] for n, e in base["fields"].items()
               if n not in doc["deviation"]["fields"])
    assert all(doc[k] == base[k] for k in ("wide_codes", "pos_table", "scale"))
    # the same inputs give the same bytes, the file loads, and every verb says what it is
    path = cn.write(doc, tmp_path / "derived.constants.json")
    assert cn.load(path) == doc and cn.render(cn.derive(base, f, tmp_path, REG)[0]) == path.read_text()
    assert cn.described(doc)[0] == ("derived, not a set of record: deviations/cw-5.json moves angle_min_cdeg 0 -> "
                                    "500, angle_max_cdeg 18000 -> 18500")
    # the deviation file moved, the derived set is stale
    assert cn.status(doc, tmp_path, REG)[1] is False
    f.write_text(json.dumps({"policy": {"angle_cw_stop_deg": 6.0}}))
    lines, stale = cn.status(doc, tmp_path, REG)
    assert stale and lines[0].startswith("stale: deviation (deviations/cw-5.json) is no longer the file the set read")


def test_a_deviation_pins_literals_and_a_protection_field_moves_alone(base, tmp_path):
    f = dev_file(tmp_path, "bemf", pins={"recip_ke_q": pin(12000), "r_bemf_q12": pin(16500, "a-later-rung")})
    doc, _ = cn.derive(base, f, tmp_path, REG)
    # a field the set does not hold yet joins it, after the others, with nothing of record before it
    assert doc["deviation"]["fields"] == {"recip_ke_q": [11817, 12000], "r_bemf_q12": [None, 16500]}
    assert doc["fields"]["recip_ke_q"] == {"value": 12000, "class": "pinned", "why": "a test's", "rung": "a-rung",
                                           "protection": None}
    assert list(doc["fields"])[-1] == "r_bemf_q12" and doc["fields"]["r_bemf_q12"]["rung"] == "a-later-rung"
    assert moved_hashes(base, doc) == ["plant"]
    # a protection field is a deviation of its own, and moves the protection hash alone
    limit, _ = cn.derive(base, dev_file(tmp_path, "limit", pins={"current_limit_counts": pin(500)}), tmp_path, REG)
    assert moved_hashes(base, limit) == ["protection"]
    assert limit["deviation"]["fields"] == {"current_limit_counts": [554, 500]}


@pytest.mark.parametrize("doc, match", [
    ({"pins": {"current_limit_counts": pin(500), "p_kp_q88": pin(5026)}},
     "a deviation that moves a protection field \\(current_limit_counts\\) may move nothing else"),
    ({"pins": {"current_limit_counts": pin(500)}, "policy": {"angle_cw_stop_deg": 5.0}},
     "a deviation that moves a protection field \\(current_limit_counts\\) may move nothing else"),
    ({"pins": {"p_kp_q88": pin(40212)}}, "the deviation moves no field of the set"),
    ({"policy": {"angle_cw_stop_deg": 0.0}}, "the deviation moves no field of the set"),
    ({"fields": {"p_kp_q88": 5026}}, "a deviation holds policy entries, pins or holds to release, and no fields"),
    ({"note": "nothing"}, "a deviation holds policy entries, pins or holds to release, and this one none"),
    ({"release": "p_kp_q88"}, "`release` is a list of the fields whose hold it releases"),
    ({"release": ["p_kp_q88"]}, "p_kp_q88 is not held: a deviation releases a hold of policy.json"),
    ({"pins": {"p_kp_q88": {"value": 5026}}}, "p_kp_q88 lacks its value, its why or its rung"),
    ({"pins": {"p_kp_q88": pin(70000)}}, None),
], ids=["a protection field and one of the plant", "a protection field and a policy entry", "a pin of the value "
        "the set holds", "a policy entry the set is built on", "a list of raw fields", "nothing", "a release that "
        "is no list", "a release of a field that is not held", "a pin with no "
        "why", "a pin is a literal: what fits its field is the install's to say"])
def test_a_deviation_that_is_not_one_change_by_the_rules_is_refused(base, tmp_path, doc, match):
    f = dev_file(tmp_path, "x", **doc)
    if match is None:
        assert cn.derive(base, f, tmp_path, REG)[0]["deviation"]["fields"] == {"p_kp_q88": [40212, 70000]}
        return
    with pytest.raises(ValueError, match=match):
        cn.derive(base, f, tmp_path, REG)


def test_a_deviation_is_built_on_a_set_of_record_as_a_build_gives_it_and_only_one(first, base, tmp_path):
    f = dev_file(tmp_path, "soft", pins={"p_kp_q88": pin(5026)})
    # on the adopted set the rules would bind the slots, the table and more: more than the one change
    with pytest.raises(ValueError, match="the set is not what a build of its inputs gives now \\(wide-code slots: "
                                         ".*\\): `build` it first, a deviation is one change"):
        cn.derive(first, f, tmp_path, REG)
    doc, _ = cn.derive(base, f, tmp_path, REG)
    with pytest.raises(ValueError, match="the set already holds the deviation deviations/soft.json"):
        cn.derive(doc, dev_file(tmp_path, "other", pins={"r_q12": pin(16000)}), tmp_path, REG)
    with pytest.raises(ValueError, match="is not a file of the repo"):
        cn.derive(base, dev_file(tmp_path.parent / f"{tmp_path.name}-out", "soft", pins={"p_kp_q88": pin(5026)}),
                  tmp_path, REG)


def test_a_derived_set_is_checked_and_installed_by_its_path(geared, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    monkeypatch.setattr(cn, "registry_of", lambda pair: REG)
    run = lambda *args: cn.main(["--pair", PAIR, *args])   # noqa: E731
    run("adopt", "--from", str(record(tmp_path)))
    inputs(tmp_path)
    derived = tmp_path / "elsewhere" / "derived.json"
    run("build", "--out", str(derived))
    of_record = cn.set_path(PAIR, tmp_path).read_bytes()
    # the set of record is what the servo holds; the derived set is not, until it is installed
    run("check")
    with pytest.raises(SystemExit) as e:
        run("--set", str(derived), "check")
    assert e.value.code == 1
    capsys.readouterr()
    run("--set", str(derived), "install")
    out = capsys.readouterr().out.splitlines()
    assert "angle_max_cdeg: 18774 -> 18000" in out and "position table: IDENTITY -> LIVE, 8 pages read back" in out
    run("--set", str(derived), "check")
    run("--set", str(derived), "status")
    assert cn.set_path(PAIR, tmp_path).read_bytes() == of_record
    # a protection build with a table that fell away: what defines a count would move
    cn.set_path(PAIR, tmp_path).with_name(f"{PAIR}.pos_lut.json").unlink()
    with pytest.raises(ValueError, match="would move: build the plant first"):
        cn.build(PAIR, cn.load(derived), protection=["current_limit_counts"], root=tmp_path, registry=REG)


# --- the motor block, the feedforwards and the observer gains ---

CPRAD = SCALE * 180 / math.pi / REG["servo"]["gear_ratio"]      # counts per motor rad on the fake's table
# the fields of the block the fake servo does not hold, in the rules' order, the records a host keeps after them
JOINED = ["r_bemf_q12", "b_i_q313", "fric_fc_counts", "fric_fv_q016", "ke_vpc_q", "j_ff_q88", "l1_q016", "l2_q88",
          "l3_q88", "r0_q12", "ke_uvs_per_rad", "fric_breakaway_counts"]
HOLDS = {"r_q12": "it moves on a rung of its own", "recip_ke_q": "it moves with the estimator's change",
         "r_bemf_q12": "the servo's image has no such field yet"}


def raw(x, q=0):
    """A value as its field holds it at `q` fraction bits."""
    return int(x * 2 ** q + 0.5)


def block(first, root, **policy):
    """(the set, what the build says) of a build on `first` over the
    plant's results, the policy naming the observer and the rules of
    the resistance."""
    inputs(root, policy={**BLOCK_POLICY, **policy})
    plant(root)
    datasheet(root)
    return cn.build(PAIR, first, root=root, registry=BOARD_REG)


def test_the_motor_block_binds_from_the_record_through_the_stage_1_scales_the_gear_and_the_count(first, tmp_path):
    doc, notes = block(first, tmp_path)
    f, sha = doc["fields"], cn.points_sha1(POINTS)
    assert notes == ["1-board/wide-codes: the results it read are not recorded", *BLOCK_SAID]
    # the resistance, ohm x tap counts per V / shunt counts per A, by the rule the policy names, its band the
    # rule again at each end of the resistance's band and of the shunt scale's
    assert f["r_q12"] == {
        "value": 4096, "class": "measured", "rule": "r.rails_fit", "rule_rev": 1, "unit": "vcounts/ccount", "q": 12,
        "pos_unit": 0, "band": [3994, 4301], "protection": None,
        "inputs": ["policy:rule.r_q12", "3-servo/resistance:r_rails_at_measured_e_ohm",
                   "1-board/terminal-chain:terminal_counts_per_v", "1-board/current-scale:counts_per_a"]}
    ke = 0.001 * 250 / CPRAD                    # tap counts per count/s
    b = 0.001 / 1000 / 6e-9 * CPRAD / 2000      # (count/s) a MEDIUM tick per shunt count
    pole = math.exp(-2 * math.pi * 15 / 2000)
    # field: (the value in its unit, its fraction bits, the power of counts per deg in it, its class)
    want = {"recip_ke_q": (1 / ke, 10, 1, "measured"), "ke_vpc_q": (ke, 12, -1, "measured"),
            "b_i_q313": (b, 13, 1, "measured"), "j_ff_q88": (1 / b, 8, -1, "measured"),
            "fric_fc_counts": (50e-6 / 0.001 * 1000, 0, 0, "measured"),
            "fric_fv_q016": (0.05e-6 / 0.001 * 1000 / CPRAD, 16, -1, "measured"),
            "l1_q016": (1 - pole ** 3, 16, 0, "policy"),
            "l2_q88": (1.5 * (1 - pole) ** 2 * (1 + pole) * 2000, 8, 0, "policy"),
            "l3_q88": ((1 - pole) ** 3 * 2000 / (2 * b), 8, -1, "policy"),
            "r_bemf_q12": (5.0 * 250 / 1000, 12, 0, "measured")}
    for name, (x, q, pos_unit, cls) in want.items():
        e = f[name]
        assert (e["value"], e["q"], e["pos_unit"], e["class"]) == (raw(x, q), q, pos_unit, cls), name
        # a per-count field names the table it was computed for
        assert e.get("scale_points_sha1") == (sha if pos_unit else None), name
    assert (f["fric_fc_counts"]["value"], f["r_bemf_q12"]["rule"]) == (50, "r_bemf.small_step")
    assert f["b_i_q313"]["band"] == [raw(b * 6 / 7, 13), raw(b * 6 / 5, 13)]
    assert f["fric_fc_counts"]["inputs"] == ["3-servo/friction:firmware_pair.pooled.coulomb_uNm",
                                             "3-servo/ladder:ke_v_s_per_rad", "1-board/current-scale:counts_per_a"]
    # the fields the servo holds stand in their places, the ones it does not come after, in the rules' order
    assert list(f) == [*first["fields"], *JOINED]
    assert cn.load(cn.write(doc, tmp_path / "x.constants.json")) == doc
    # a result a rule read is an input of the set: moved, it names the fields that read it
    assert cn.status(doc, tmp_path, BOARD_REG)[1] is False
    plant(tmp_path, {"3-servo/run-up": {"j_kg_m2": (7e-9,)}})
    lines, stale = cn.status(doc, tmp_path, BOARD_REG)
    assert stale and lines[0].endswith(": b_i_q313, j_ff_q88, l3_q88")


def test_the_policy_names_which_of_a_fields_rules_binds(first, tmp_path):
    def r_q12(way):
        doc, _ = block(first, tmp_path, rule={**BLOCK_POLICY["rule"], "r_q12": f"r.{way}"})
        return doc["fields"]["r_q12"]

    # the same quantity each way it is on record, through the same scales
    assert {w: r_q12(w)["value"] for w in WAYS} == {
        "rails_fit": 4096, "locked_laps": raw(4.1 / 4, 12), "small_step": 5120, "take_back": raw(4.8 / 4, 12)}
    assert r_q12("take_back")["inputs"][:2] == ["policy:rule.r_q12", "3-servo/steps:r_takeback_ohm"]
    assert [list(cn.RULES[n]) for n in ("r_q12", "r_bemf_q12")] == [[f"r.{w}" for w in WAYS],
                                                                   [f"r_bemf.{w}" for w in WAYS]]
    # a policy that names none does not choose for it: the field is held, the back-EMF one left out
    doc, notes = block(first, tmp_path, rule={})
    assert doc["fields"]["r_q12"] == {"value": 17100, "class": "none", "held": NO_RULE.format("r_q12", *["r"] * 4),
                                      "carried_from": first["hash"]["set"], "protection": None}
    assert "r_bemf_q12" not in doc["fields"]
    assert [NO_R, NO_BLOCK[0]] == [n for n in notes if n.startswith(("r_q12", "r_bemf_q12"))]
    with pytest.raises(ValueError, match="policy.json: r.guess is no rule of r_q12 \\(r.rails_fit, r.locked_laps"):
        block(first, tmp_path, rule={"r_q12": "r.guess"})


@pytest.fixture
def holding(first, tmp_path):
    """(the pair's set of record built over the plant with HOLDS in the
    policy, what the build said)."""
    doc, notes = block(first, tmp_path, hold=HOLDS)
    cn.write(doc, cn.set_path(PAIR, tmp_path))
    return doc, notes


def test_a_held_field_stands_beside_what_its_rule_gives_and_one_with_nothing_to_carry_stays_out(first, holding):
    doc, notes = holding
    would = raw(CPRAD / (0.001 * 250), 10)
    assert doc["fields"]["recip_ke_q"] == {
        "value": 11817, "class": "none", "rule": "recip_ke.ladder", "would_be": would, "held": HOLDS["recip_ke_q"],
        "carried_from": first["hash"]["set"], "scale": "unknown", "protection": None,
        "inputs": ["3-servo/ladder:ke_v_s_per_rad", "1-board/terminal-chain:terminal_counts_per_v", "scale",
                   "registry:gear_ratio"]}
    assert (doc["fields"]["r_q12"]["value"], doc["fields"]["r_q12"]["would_be"]) == (17100, 4096)
    assert [n for n in notes if n.startswith(("r_q12", "recip_ke_q", "r_bemf_q12"))] == [
        f"r_q12: held, {HOLDS['r_q12']}; its rule gives 4096",
        f"recip_ke_q: held, {HOLDS['recip_ke_q']}; its rule gives {would}",
        "r_bemf_q12: left out, the servo's image has no such field yet; its rule gives 5120"]
    assert list(doc["fields"]) == [*first["fields"], *JOINED[1:]]
    # a value carried from before the path is still at a scale nothing recorded: holding it gave it none
    unknown = [cn.described(d)[0].split("; ")[1] for d in (first, doc)]
    assert unknown == ["4 per-count at an unknown scale"] * 2
    # the two the policy holds, and the range, the hold band and the stall speed, whose entries it lacks:
    # held, they have no source still, and the four fields of the angle map and the gear have one
    assert sum("held" in e for e in doc["fields"].values()) == 6
    assert cn.classes(doc)["none"] == cn.classes(first)["none"] - 4 == len(cn.unsourced(doc))


def test_a_deviation_releases_a_hold_and_the_rest_of_the_policy_stands(holding, tmp_path):
    base, _ = holding
    would = base["fields"]["recip_ke_q"]["would_be"]
    f = dev_file(tmp_path, "ke", note="the back-EMF constant of the record", release=["recip_ke_q"])
    doc, _ = cn.derive(base, f, tmp_path, BOARD_REG)
    # the released field binds what its rule gives, of record beside deviated like any field a deviation moves
    assert doc["deviation"] == {"file": "deviations/ke.json", "sha1": sc.file_sha1(f),
                                "fields": {"recip_ke_q": [11817, would]}}
    e = doc["fields"]["recip_ke_q"]
    assert (e["value"], e["class"], e["rule"], e["scale_points_sha1"]) == (
        would, "measured", "recip_ke.ladder", cn.points_sha1(POINTS))
    assert not any(k in e for k in ("held", "would_be", "scale"))
    # the holds it does not name stand as the policy file has them: nothing of the policy is restated
    assert doc["fields"]["r_q12"] == base["fields"]["r_q12"] and "r_bemf_q12" not in doc["fields"]
    assert moved_hashes(base, doc) == ["plant"] and cn.load(cn.write(doc, tmp_path / "d.constants.json")) == doc
    assert cn.described(doc)[0] == f"derived, not a set of record: deviations/ke.json moves recip_ke_q 11817 -> {would}"
    # a released field the set does not hold joins it
    both, _ = cn.derive(base, dev_file(tmp_path, "r", release=["r_q12", "r_bemf_q12"]), tmp_path, BOARD_REG)
    assert both["deviation"]["fields"] == {"r_q12": [17100, 4096], "r_bemf_q12": [None, 5120]}
    # refused: a field no hold stands on, and a hold whose rule has no value to bind
    with pytest.raises(ValueError, match="deviations/free.json: l1_q016 is not held: a deviation releases a hold"):
        cn.derive(base, dev_file(tmp_path, "free", release=["l1_q016"]), tmp_path, BOARD_REG)
    plant_path(tmp_path, "3-servo/ladder").unlink()
    with pytest.raises(ValueError, match=f"recip_ke_q: its hold is released and its rule has no value: {NO_LADDER}"):
        cn.derive(base, f, tmp_path, BOARD_REG)


def test_a_release_of_a_protection_field_moves_it_alone(first, limited, tmp_path):
    inputs(tmp_path, policy={"hold": {"current_limit_counts": "the limit moves on a rung of its own",
                                      "angle_max_cdeg": "the hosts read the old travel"}})
    base, _ = built_on(first, tmp_path)
    cn.write(base, cn.set_path(PAIR, tmp_path))
    assert (base["fields"]["current_limit_counts"]["value"], base["fields"]["current_limit_counts"]["would_be"]) == (
        554, 500)
    doc, _ = cn.derive(base, dev_file(tmp_path, "limit", release=["current_limit_counts"]), tmp_path, REG)
    assert doc["deviation"]["fields"] == {"current_limit_counts": [554, 500]}
    assert moved_hashes(base, doc) == ["protection"] and doc["fields"]["angle_max_cdeg"] == base["fields"][
        "angle_max_cdeg"]
    with pytest.raises(ValueError, match="a deviation that moves a protection field \\(current_limit_counts\\) may "
                                         "move nothing else"):
        cn.derive(base, dev_file(tmp_path, "both", release=["current_limit_counts", "angle_max_cdeg"]), tmp_path,
                  REG)


def test_the_rules_reproduce_the_firmware_repos_synthesis_on_its_own_test_plant(tmp_path):
    """ident/src/gains.rs of the firmware repo, `encode(&synthesize(
    &rig_plant(), &BwTargets::default()))`: that file's test plant, in
    the count domain, and the raw fields its code gives for it, as a
    build of the file printed them. A rule starts from SI, so the plant
    is put here as the SI record that maps to it through the fake's
    scales, gear and count; the fields are the ones both compute."""
    rust = {"r_q12": 13804, "ke_vpc_q": 819, "recip_ke_q": 5120, "b_i_q313": 819, "fric_fc_counts": 20,
            "fric_fv_q016": 66, "j_ff_q88": 2560, "l1_q016": 8600, "l2_q88": 3165, "l3_q88": 247}
    # rig_plant(): r_vpc, ke_vpc, fc, fv, b, f_med; BwTargets::default().f_o is the policy's 15 Hz
    r_vpc, ke_vpc, fc, fv, b, f_med = 3.37, 0.2, 20.0, 0.001, 0.1, 2010.0
    cpa, cpv = 1000.0, 250.0
    ke = ke_vpc * CPRAD / cpv
    # the current loop's three are that file's placement too, i_kp = w L_cd, i_ki = w R / tick and i_kaw twice
    # it at BwTargets::default().f_ci, evaluated here on rig_plant()'s l_cd, r_loop_vpc and tick_hz, rounded
    # as its enc rounds: no build printed these
    w, l_cd, tick = 2 * math.pi * 1000.0, 3.58e-4, f_med * cn.DECIM_MED
    loop = {"i_kp_q88": raw(w * l_cd, 8), "i_ki_q412": raw(w * r_vpc / tick, 12),
            "i_kaw_q412": raw(2 * w * r_vpc / tick, 12)}
    assert loop == {"i_kp_q88": 576, "i_ki_q412": 4315, "i_kaw_q412": 8630}
    rust.update(loop)
    inputs(tmp_path, policy={**BLOCK_POLICY, "current_loop_hz": 1000.0})
    plant(tmp_path, {
        "1-board/current-scale": {"counts_per_a": (cpa,)},
        "1-board/terminal-chain": {"terminal_counts_per_v": (cpv,)},
        "3-servo/resistance": {"r_rails_at_measured_e_ohm": (r_vpc * cpa / cpv,)},
        "3-servo/inductance": {"l_mh": (l_cd * cpa / cpv * 1000,)},
        "3-servo/ladder": {"ke_v_s_per_rad": (ke,)},
        "3-servo/run-up": {"j_kg_m2": (ke / cpa * CPRAD / f_med / b,)},
        "3-servo/friction": {"firmware_pair": {"pooled": {
            "coulomb_uNm": (fc * ke / cpa * 1e6,), "viscous_uNm_s_per_rad": (fv * ke * CPRAD / cpa * 1e6,)}}}})
    c = cn.Context(PAIR, tmp_path, {**REG, "board": {"tick_hz": int(f_med * cn.DECIM_MED)}})
    cn.table_of(c, None, cn.slots_of(c, None))
    assert {n: cn.evaluate(cn.rule_of(c, n), c)["value"] for n in rust} == rust


def test_the_bench_pairs_block_is_what_its_record_gives_on_its_table():
    pair = postable.default_pair()
    doc = cn.of_record(pair)
    f = doc["fields"]
    c = cn.Context(pair)
    cn.table_of(c, doc, cn.slots_of(c, doc))
    # field: (what the servo held before the set, what its rule gives from the results of record, the rule's band)
    block_of_record = {
        "r_q12": (4541, 3744, [3735, 3758]), "recip_ke_q": (11817, 13315, [13233, 13399]),
        "ke_vpc_q": (355, 315, [313, 317]), "b_i_q313": (2657, 2434, [2213, 2672]),
        "fric_fc_counts": (100, 54, [42, 64]), "fric_fv_q016": (289, 812, [634, 1084]),
        "j_ff_q88": (789, 862, [785, 948]), "l1_q016": (8640, 8640, None), "l2_q88": (3180, 3180, None),
        "l3_q88": (77, 84, [77, 92])}
    ran = ran_on()
    for name, (stored, gives, band) in block_of_record.items():
        e = cn.evaluate(cn.rule_of(c, name), c)
        assert (e["value"], e.get("band")) == (gives, band), name
        # the set of record binds every field of the block at what its rule gives, none held: the two whose
        # release was benched and kept, and the rest by the release of their holds
        assert (f[name]["value"], f[name].get("would_be"), "held" in f[name]) == (gives, None, False), name
        if name not in ("fric_fv_q016", "ke_vpc_q"):
            # what the servo held is what the reference rung ran on
            assert ran[name] == stored, name
        # measured, but for the observer's three: bound at the policy's bandwidth, they are the policy's
        if name in OBSERVER:
            assert f[name]["class"] == "policy" and "reason" not in f[name], name
        else:
            assert f[name]["class"] == "measured" and "reason" not in f[name], name
    # the resistance each way it is on record; the back-EMF boxcar's own takes the small step's, on no image yet
    ways = {r.name: cn.evaluate(r, c) for r in cn.RULES["r_bemf_q12"].values()}
    assert {n: e["value"] for n, e in ways.items()} == {
        "r_bemf.rails_fit": 3744, "r_bemf.locked_laps": 3781, "r_bemf.small_step": 4799, "r_bemf.take_back": 4557}
    assert cn.rule_of(c, "r_bemf_q12").name == "r_bemf.small_step" and "r_bemf_q12" not in f
    assert ways["r_bemf.small_step"]["band"] == [4036, 5562]


# --- the protection rules: the NTC curve and the current limits ---

CURVE = ("ntc_t_ref_cc", "ntc_k1_q88", "ntc_k2_q24")        # the words of the curve that are fitted
LIMITS = ("current_limit_counts", "i_cont_counts", "stall_release_counts", "oc_trip_counts")


def test_the_protection_rules_bind_in_a_build_that_names_them(first, tmp_path):
    plain, _ = block(first, tmp_path)
    # a build of the plant leaves them: the one the servo holds carried with what it would be
    assert plain["fields"]["current_limit_counts"] == {**first["fields"]["current_limit_counts"], "would_be": 600}
    assert not any(n in plain["fields"] for n in (*CURVE, *LIMITS[1:]))
    doc, _ = cn.build(PAIR, plain, protection=[*CURVE, *LIMITS], root=tmp_path, registry=BOARD_REG)
    f = doc["fields"]
    assert moved_hashes(plain, doc) == ["protection"]
    # the cap: the current the voltage the maker states the stall at drives through the rails fit, at the
    # policy's derate of a maker's value, through the board's measured counts per A, its band the
    # resistance's and the scale's; the entry names the record, the voltage and the derate
    assert f["current_limit_counts"] == {
        "value": 600, "class": "policy", "rule": "current_limit.stall_derate", "rule_rev": 1, "unit": "ccounts",
        "q": 0, "pos_unit": 0, "inputs": [
            "registry:datasheet_class", "datasheet:operating_voltage", "datasheet:stall_torque",
            "3-servo/resistance:r_rails_at_measured_e_ohm", "policy:datasheet_derate_default",
            "1-board/current-scale:counts_per_a"],
        "band": [571, 615], "ma": 600.0, "at_v": 4.8, "derate": 0.5, "record": "what the maker states of the class",
        "document": DOCUMENT, "protection": {"looser": "up"}}
    # the limits under it: the policy's mA through the same counts per A
    assert {n: (f[n]["value"], f[n]["rule"]) for n in LIMITS[1:]} == {
        "i_cont_counts": (250, "i_cont.policy"), "stall_release_counts": (90, "stall_release.policy"),
        "oc_trip_counts": (2200, "oc_trip.policy")}
    assert f["i_cont_counts"]["protection"] == {"looser": "judged"}
    # the curve: the design quadratic's three words as the board's result holds them, each in its field's unit
    assert f["ntc_k1_q88"] == {
        "value": 200, "class": "measured", "rule": "ntc_k1.design", "rule_rev": 1, "unit": "cC/count", "q": 8,
        "pos_unit": 0, "inputs": ["1-board/ntc-curve:design_k1_q88"], "protection": {"looser": "judged"}}
    assert {n: (f[n]["value"], f[n]["q"]) for n in CURVE} == {
        "ntc_t_ref_cc": (2600, 0), "ntc_k1_q88": (200, 8), "ntc_k2_q24": (30000, 24)}
    # the reference count is the design's too, the count the quadratic is written about: named, it joins them
    assert "ntc_raw_ref" not in f and cn.load(cn.write(doc, tmp_path / "x.constants.json")) == doc
    ref, _ = cn.build(PAIR, doc, protection=["ntc_raw_ref"], root=tmp_path, registry=BOARD_REG)
    assert moved_hashes(doc, ref) == ["protection"] and ref["fields"]["ntc_raw_ref"] == {
        "value": 2048, "class": "measured", "rule": "ntc_raw_ref.design", "rule_rev": 1, "unit": "counts", "q": 0,
        "pos_unit": 0, "inputs": ["1-board/ntc-curve:design_raw_ref"], "protection": {"looser": "judged"}}
    # the one thermal limit waits for its own name: the policy's temperature, centi-C as the field holds it
    assert "cutoff_cc" not in f
    hot, _ = cn.build(PAIR, doc, protection=["cutoff_cc"], root=tmp_path, registry=BOARD_REG)
    assert moved_hashes(doc, hot) == ["protection"] and hot["fields"]["cutoff_cc"] == {
        "value": 9000, "class": "policy", "rule": "cutoff.policy", "rule_rev": 1, "unit": "cC", "q": 0,
        "pos_unit": 0, "inputs": ["policy:cutoff_c"], "protection": {"looser": "up"}}
    # the collision trip is off by the policy, which is the field's largest value: a level has no rule yet
    off, _ = cn.build(PAIR, doc, protection=["stall_tau_trip_counts"], root=tmp_path, registry=BOARD_REG)
    assert moved_hashes(doc, off) == ["protection"] and off["fields"]["stall_tau_trip_counts"] == {
        "value": 65535, "class": "policy", "rule": "stall_tau_trip.policy", "rule_rev": 1, "unit": "ccounts", "q": 0,
        "pos_unit": 0, "inputs": ["policy:collision_trip"], "protection": {"looser": "up"}}
    level = context(tmp_path, policy={"collision_trip": 900})
    with pytest.raises(ValueError, match="policy.json: collision_trip is 900: the collision trip is `off`"):
        cn.evaluate(cn.rule_of(level, "stall_tau_trip_counts"), level)
    inputs(tmp_path, policy=BLOCK_POLICY)
    # on a servo that holds a higher one it protects more: its install names it, and not to loosen
    state = {"fields": {**{n: e["value"] for n, e in hot["fields"].items()}, "cutoff_cc": 10000}}
    assert cn.gate(hot, state) == ["cutoff_cc: servo 10000, set 9000, protects more (less is up): name it with "
                                   "--protection cutoff_cc"]
    assert cn.gate(hot, state, protection=["cutoff_cc"]) == []
    # a scale that moves moves the limits with it, and the set that stood is stale by the fields that read it
    plant(tmp_path, {"1-board/current-scale": {"counts_per_a": (1100.0,)}})
    lines, stale = cn.status(doc, tmp_path, BOARD_REG)
    assert stale and all(n in lines[0] for n in LIMITS)
    with pytest.raises(ValueError, match="what the set is scored with would move: build the plant first"):
        cn.build(PAIR, doc, protection=list(LIMITS), root=tmp_path, registry=BOARD_REG)
    moved, _ = cn.build(PAIR, cn.build(PAIR, doc, root=tmp_path, registry=BOARD_REG)[0], protection=list(LIMITS),
                        root=tmp_path, registry=BOARD_REG)
    assert [moved["fields"][n]["value"] for n in LIMITS] == [660, 275, 99, 2420]


def test_a_held_protection_field_is_held_by_a_build_of_the_plant_and_moves_no_hash(first, tmp_path):
    why = "it moves on a protection step of its own"
    doc, notes = block(first, tmp_path, hold={"current_limit_counts": why, "ntc_k1_q88": why})
    assert doc["fields"]["current_limit_counts"] == {
        "value": 554, "class": "none", "rule": "current_limit.stall_derate", "would_be": 600, "held": why,
        "carried_from": first["hash"]["set"], "protection": {"looser": "up"},
        "inputs": ["registry:datasheet_class", "datasheet:operating_voltage", "datasheet:stall_torque",
                   "3-servo/resistance:r_rails_at_measured_e_ohm", "policy:datasheet_derate_default",
                   "1-board/current-scale:counts_per_a"]}
    assert f"current_limit_counts: held, {why}; its rule gives 600" in notes
    # a held field the set never held stays out, said with what its rule gives
    assert "ntc_k1_q88" not in doc["fields"] and f"ntc_k1_q88: left out, {why}; its rule gives 200" in notes
    assert doc["hash"]["protection"] == first["hash"]["protection"]
    # it stands as held in every build after, of the plant or of protection fields, until it is released
    again, _ = cn.build(PAIR, doc, root=tmp_path, registry=BOARD_REG)
    named, _ = cn.build(PAIR, doc, protection=["current_limit_counts"], root=tmp_path, registry=BOARD_REG)
    assert again["fields"] == named["fields"] == doc["fields"] and again["hash"] == doc["hash"]


@pytest.fixture
def steps(first, tmp_path):
    """The pair's set of record with the curve and the limits in it and
    held, each rule giving another value than the set holds."""
    plain, _ = block(first, tmp_path)
    bound, _ = cn.build(PAIR, plain, protection=[*CURVE, *LIMITS], root=tmp_path, registry=BOARD_REG)
    why = "it moves on a protection step of its own"
    inputs(tmp_path, policy={**BLOCK_POLICY, "datasheet_derate_default": 0.51, "i_cont_ma": 256.0,
                             "stall_release_ma": 92.0,
                             "oc_trip_ma": 2250.0, "hold": dict.fromkeys((*CURVE, *LIMITS), why)})
    plant(tmp_path, {"1-board/ntc-curve": {"design_t_ref_cc": (2430.0,), "design_k1_q88": (474.0,),
                                           "design_k2_q24": (19934.0,)}})
    doc, _ = cn.build(PAIR, bound, root=tmp_path, registry=BOARD_REG)
    cn.write(doc, cn.set_path(PAIR, tmp_path))
    assert doc["hash"] == bound["hash"] and all("held" in doc["fields"][n] for n in (*CURVE, *LIMITS))
    return doc


def test_a_deviation_moves_the_protection_fields_of_one_step_and_releases_its_holds_together(steps, tmp_path):
    def derived(name, **doc):
        return cn.derive(steps, dev_file(tmp_path, name, **doc), tmp_path, BOARD_REG)[0]

    # the curve's three words are one curve, and the limits one scale: each group is released whole
    curve = derived("curve", note="the curve of record", release=list(CURVE))
    assert curve["deviation"]["fields"] == {"ntc_t_ref_cc": [2600, 2430], "ntc_k1_q88": [200, 474],
                                            "ntc_k2_q24": [30000, 19934]}
    limits = derived("limits", release=list(LIMITS))
    assert limits["deviation"]["fields"] == {"current_limit_counts": [600, 612], "i_cont_counts": [250, 256],
                                             "stall_release_counts": [90, 92], "oc_trip_counts": [2200, 2250]}
    assert moved_hashes(steps, curve) == moved_hashes(steps, limits) == ["protection"]
    assert all(limits["fields"][n] == steps["fields"][n] for n in CURVE)
    # a part of a group left held is refused: a curve of two fits is no curve
    with pytest.raises(ValueError, match="the holds of the ntc curve are released together, and ntc_k2_q24 would "
                                         "stay held"):
        derived("half", release=list(CURVE[:2]))
    with pytest.raises(ValueError, match="the holds of the current scale are released together, and i_cont_counts, "
                                         "stall_release_counts, oc_trip_counts would stay held"):
        derived("one", release=["current_limit_counts"])
    # two steps in one deviation are refused, grouped or not
    with pytest.raises(ValueError, match="a deviation moves the protection fields of one step, and these are of "
                                         "2: ntc curve, current scale"):
        derived("two", release=[*CURVE, *LIMITS])
    with pytest.raises(ValueError, match="of 2: stall_response, stall_omega_max_cps"):
        derived("lone", pins={"stall_response": pin("Yield (1)"), "stall_omega_max_cps": pin(400)})
    with pytest.raises(ValueError, match="of 2: current scale, stall_omega_max_cps"):
        derived("mixed", pins={"current_limit_counts": pin(400), "stall_omega_max_cps": pin(400)})
    # a pin is a literal typed for a rung: one limit alone, the others standing held
    lone = derived("limit", pins={"current_limit_counts": pin(400)})
    assert lone["deviation"]["fields"] == {"current_limit_counts": [600, 400]}
    assert lone["fields"]["i_cont_counts"] == steps["fields"]["i_cont_counts"]
    # what the install of each step has to be told, against a servo that holds the set of record
    state = {"fields": {n: e["value"] for n, e in steps["fields"].items()}}
    assert cn.gate(limits, state) == [
        "current_limit_counts: servo 600, set 612, protects less (less is up): name it with --protection "
        "current_limit_counts and --loosen current_limit_counts",
        "i_cont_counts: servo 250, set 256, judged, taken as protecting less: name it with --protection "
        "i_cont_counts and --loosen i_cont_counts",
        "stall_release_counts: servo 90, set 92, protects less (less is up): name it with --protection "
        "stall_release_counts and --loosen stall_release_counts",
        "oc_trip_counts: servo 2200, set 2250, protects less (less is up): name it with --protection "
        "oc_trip_counts and --loosen oc_trip_counts"]
    assert cn.gate(limits, state, protection=LIMITS, loosen=LIMITS) == []
    assert [line.split(":")[0] for line in cn.gate(curve, state, protection=CURVE)] == list(CURVE)
    assert cn.gate(curve, state, protection=CURVE, loosen=CURVE) == []


def of_the_bench_pair():
    """(the bench pair's set of record, a Context on its table, its
    board)."""
    pair = postable.default_pair()
    doc = cn.of_record(pair)
    c = cn.Context(pair)
    cn.table_of(c, doc, cn.slots_of(c, doc))
    return doc, c, boards.BOARDS[pair.partition("__")[2]]


REFERENCE = "14a-set-of-record"     # the rung of record that ran on what the servo held before the releases


def ran_on():
    """The fields of the state the reference rung ran on, run 1: what
    the servo held of every field a release or a protection step has
    since bound at another value."""
    folder = results.ROOT / "telemetry" / postable.default_pair() / "scorecard" / REFERENCE
    return json.loads((folder / "run-1" / "servo.json").read_text())["fields"]


def as_held(doc, names):
    """The state of a servo that holds the set `doc` but for `names`,
    each at the value the reference rung ran on."""
    ran = ran_on()
    return {"fields": {n: ran[n] if n in names else e["value"] for n, e in doc["fields"].items()}}


def asked(doc, state):
    """What the protection gate asks an install of `doc` on a servo in
    `state` to name, by field."""
    return {line.split(":")[0]: line.rpartition("name it with ")[2] for line in cn.gate(doc, state)}


def test_the_bench_pairs_current_limits_are_the_ma_they_were_typed_for_at_the_measured_scale():
    doc, c, board = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    nominal, measured = board.counts_per_a, doc["scales"][cn.CURRENT]["value"]
    # the cap was typed for 500 mA on the board's nominal parts too, and is read off the maker's sheet now
    # (`test_the_set_of_record_holds_the_current_limit_the_makers_stall_gives_at_its_derate`)
    assert [ma for ma in range(1, 4000) if raw(ma * nominal / 1000) == 554] == [500]
    assert ran_on()["current_limit_counts"] == 554 and f["current_limit_counts"]["value"] == 1043
    # field: (what the servo held before the set, the policy's entry, what the rule gives)
    limits = {"i_cont_counts": (277, "i_cont_ma", 284), "stall_release_counts": (100, "stall_release_ma", 102),
              "oc_trip_counts": (2439, "oc_trip_ma", 2497)}
    assert ("current_limit_counts", *limits) == LIMITS == cn.GROUPS["current scale"]
    for name, (stored, key, gives) in limits.items():
        # the mA the stored counts were typed for: the one whole mA the board's nominal parts take to them
        typed = [ma for ma in range(1, 4000) if raw(ma * nominal / 1000) == stored]
        assert typed == [policy[key]], name
        e = cn.evaluate(cn.rule_of(c, name), c)
        assert e["value"] == gives == raw(policy[key] * measured / 1000), name
        # bound: the set holds each at the policy's mA through the measured scale, of class `policy`: none
        # of the four mA is a default nobody decided
        assert key not in policy["undecided"], name
        assert (f[name]["value"], f[name]["class"], f[name].get("would_be")) == (gives, "policy", None), name
        # more counts on the same mA: every one of them moved the way that protects less, or is judged
        assert gives > stored and cn.LOOSER[name] in ("up", "judged"), name
    # what the servo's earlier counts were at the measured scale, and how far the rule's are over them, mA
    now = {n: round(1000 * stored / measured, 1) for n, (stored, _, _) in limits.items()}
    assert now == {"i_cont_counts": 244.0, "stall_release_counts": 88.1, "oc_trip_counts": 2148.9}
    more = {n: round(1000 * (gives - stored) / measured, 1) for n, (stored, _, gives) in limits.items()}
    assert more == {"i_cont_counts": 6.2, "stall_release_counts": 1.8, "oc_trip_counts": 51.1}
    assert policy["why"]["oc_trip_ma"].startswith("a wiring or bridge fault detector")
    # the collision trip is off, no current: the policy says so, and off is the field's largest value
    e = cn.evaluate(cn.rule_of(c, "stall_tau_trip_counts"), c)
    assert (e["value"], e["class"], e["inputs"]) == (65535, "policy", ["policy:collision_trip"])
    assert {k: f["stall_tau_trip_counts"][k] for k in ("value", "class", "rule")} == {
        "value": 65535, "class": "policy", "rule": "stall_tau_trip.policy"}


def test_what_the_stored_curve_and_the_curve_of_record_read_on_the_bench_board():
    doc, c, board = of_the_bench_pair()
    f = doc["fields"]
    assert cn.GROUPS["ntc curve"] == ("ntc_raw_ref", *CURVE)
    words = cn.GROUPS["ntc curve"]
    stored = tuple(ran_on()[n] for n in words)
    record = tuple(cn.evaluate(cn.rule_of(c, n), c)["value"] for n in words)
    assert stored == (2048, 2500, 563, 5779) and record == (2048, 2430, 474, 19934)
    # the three fitted words are bound at what the design curve gives, measured, one protection step in place
    # of the tangent form the servo held. The reference count the design is written about is the count the
    # servo held, so it bound as it stood
    assert tuple(f[n]["value"] for n in words) == record
    assert all("held" not in f[n] and (f[n]["class"], f[n].get("would_be")) == ("measured", None) for n in CURVE)
    # on a servo that holds the stored curve the install names the three, and each to loosen: they are judged
    state = as_held(doc, CURVE)
    assert asked(doc, state) == {n: f"--protection {n} and --loosen {n}" for n in CURVE}
    assert cn.gate(doc, state, protection=CURVE, loosen=CURVE) == []
    assert {k: f["ntc_raw_ref"][k] for k in ("value", "class", "rule", "inputs")} == {
        "value": 2048, "class": "measured", "rule": "ntc_raw_ref.design",
        "inputs": ["1-board/ntc-curve:design_raw_ref"]}
    # the temperature each reports with the board at a true one, the part's nominal equation as the truth
    reads = {t: (ntc.reads_c(board, stored, t), ntc.reads_c(board, record, t)) for t in (25, 40, 60, 80, 100)}
    assert reads == {25: (25.0, 24.3), 40: (40.19, 40.64), 60: (57.31, 65.14), 80: (68.53, 83.54),
                     100: (75.08, 94.95)}
    # the true temperature at which each reaches the cutoff, the one thermal limit, as the set holds it
    cutoff = f["cutoff_cc"]["value"]
    assert cutoff == 9000
    reaches = [ntc.reaches_c(board, curve, cutoff) for curve in (stored, record)]
    assert reaches == [None, pytest.approx(90.1, abs=0.05)]
    # the stored curve tops out under the cutoff: short of the rail check no board temperature reads it
    counts = range(ntc.RAIL_COUNTS + 1, ntc.ADC_FULL - 1 - ntc.RAIL_COUNTS)
    assert max(ntc.ntc_cc(r, *stored) for r in counts) == 8416
    assert float(board.ntc_c_beta(ntc.RAIL_COUNTS)) == pytest.approx(290.2, abs=0.05)
    # where the curve of record reads under the stored one, which is where it protects less
    under = [r for r in counts if ntc.ntc_cc(r, *record) < ntc.ntc_cc(r, *stored)]
    assert [round(float(board.ntc_c_beta(r)), 1) for r in (under[-1], under[0])] == [21.8, 38.2]
    worst = min(under, key=lambda r: ntc.ntc_cc(r, *record) - ntc.ntc_cc(r, *stored))
    assert (ntc.ntc_cc(worst, *record) - ntc.ntc_cc(worst, *stored), round(float(board.ntc_c_beta(worst)), 1)) == (
        -107, 29.9)
    # and the cold side of the quadratic: it turns at a true 8 C and reads warmer as the board cools below it
    lowest = min(counts, key=lambda r: ntc.ntc_cc(r, *record))
    assert (ntc.ntc_cc(lowest, *record), round(float(board.ntc_c_beta(lowest)), 1)) == (1707, 8.3)
    assert (ntc.reads_c(board, record, 0), ntc.reads_c(board, stored, 0)) == (18.37, 4.84)


def test_the_bench_pairs_thermal_limit_is_the_policys_and_the_only_one():
    doc, c, _ = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    e = cn.evaluate(cn.rule_of(c, "cutoff_cc"), c)
    assert (e["value"], e["inputs"]) == (100 * policy["cutoff_c"], ["policy:cutoff_c"]) == (9000, ["policy:cutoff_c"])
    assert (f["cutoff_cc"]["value"], f["cutoff_cc"]["class"], f["cutoff_cc"].get("would_be")) == (9000, "policy", None)
    # no field of the set is one the firmware no longer names
    assert not set(cn.RETIRED) & set(f) and sum(n in cn.PROTECTION for n in f) == 26
    # against a servo at the 100 C it held before, the set's limit protects more: named, and not to loosen
    state = {"fields": {**{n: x["value"] for n, x in f.items()}, "cutoff_cc": 10000}}
    assert cn.gate(doc, state) == ["cutoff_cc: servo 10000, set 9000, protects more (less is up): name it with "
                                   "--protection cutoff_cc"]
    assert cn.gate(doc, state, protection=["cutoff_cc"]) == []


def test_the_protection_steps_of_record_name_what_their_install_needs():
    doc, _, _ = of_the_bench_pair()
    f, ran = doc["fields"], ran_on()
    folder = results.ROOT / "stages" / sc.STAGE / "scorecard" / "deviations"
    # step: (what it moved, as [what the servo held, the set of record], the fields its install names to
    # loosen as well: the ones that are judged, and the one that moves the way that protects less)
    steps = {
        "protection-ntc-curve": ({"ntc_t_ref_cc": [2500, 2430], "ntc_k1_q88": [563, 474], "ntc_k2_q24": [5779, 19934]},
                                 CURVE),
        "protection-stall-response": ({"stall_response": ["Fault (0)", "Yield (1)"]}, ("stall_response",)),
        "protection-position-error": ({"pos_error_time_ticks": [10000, 2000], "pos_error_counts": [400, 199]}, ()),
        "protection-sensor-delta": ({"sensor_delta_max": [256, 64]}, ()),
        "protection-undervolt": ({"v_undervolt_counts": [720, 922]}, ()),
        "protection-rail-fold": ({"rail_fold_q8": [1430, 1467]}, ("rail_fold_q8",)),
    }
    # with the winding model's, which has a test of its own, they are the protection steps of record
    assert {p.stem for p in folder.glob("protection-*.json")} == {*steps, "protection-winding-model"}
    for name, (moves, loosen) in steps.items():
        file = json.loads((folder / f"{name}.json").read_text())
        # the step is built: the set of record binds each field at what its release gave, from what the
        # reference rung ran on, and each has a source
        assert {n: [ran[n], f[n]["value"]] for n in moves} == moves and file["note"], name
        assert set(moves) <= cn.PROTECTION, name
        assert all("held" not in f[n] and f[n]["class"] in ("measured", "policy") for n in moves), name
        # its file released the fields it moved, the holds of one step
        assert set(moves) == set(file["release"]) <= set(cn.group_of(next(iter(moves)))[1]), name
        # on a servo that holds what it held before the step, every field it moved is named as a protection
        # change, and to loosen where it is judged or protects less
        state = as_held(doc, moves)
        assert asked(doc, state) == {
            n: f"--protection {n}" + (f" and --loosen {n}" if n in loosen else "") for n in moves}, name
        assert cn.gate(doc, state, protection=tuple(moves), loosen=loosen) == [], name
    # the curve's step leaves the reference count where it is: bound, at the count the design is written about
    assert cn.group_of("ntc_raw_ref")[1] == ("ntc_raw_ref", *CURVE) and "held" not in doc["fields"]["ntc_raw_ref"]


# --- the winding model ---

MODEL = ("thermal_tau_s", "th_rise_q016", "thermal_tau2_s", "th_rise2_q016")
HIGH = dict(zip(MODEL, (2, 524, 129, 1573)))        # what the rules give on the plant's results, by the reading
LOW = dict(zip(MODEL, (2, 551, 129, 839)))
REST = [f"the rest after 600 s at 554 counts, {s} s in" for s in cn.REST_S]


class Warm(Geared):
    """The fake servo with the winding model's four fields, one node on
    the slow leak, and the continuous current beside the limit."""

    FIELDS = dict(sorted({**Geared.FIELDS, "i_cont_counts": (0x52, 2, "rw", "uint"),
                          "thermal_tau_s": (0x54, 2, "rw", "uint"), "thermal_tau2_s": (0x56, 2, "rw", "uint"),
                          "th_rise_q016": (0xb0, 2, "rw", "uint"), "th_rise2_q016": (0xb2, 2, "rw", "uint")}.items(),
                         key=lambda item: item[1][0]))
    STORED = {**Geared.STORED, "i_cont_counts": 277, "thermal_tau_s": 115, "th_rise_q016": 1143, "thermal_tau2_s": 0,
              "th_rise2_q016": 0}


@pytest.fixture
def warm(monkeypatch, tmp_path):
    """(the fake servo, the set adopted from it as it boots)."""
    servo = Warm().stub(monkeypatch)
    doc, _ = cn.adopt(PAIR, record(tmp_path), root=tmp_path)
    servo.log.clear()
    return servo, doc


def test_the_winding_models_four_are_formed_from_the_stage_4_result_by_the_reading_the_policy_names(tmp_path):
    c = context(tmp_path)
    es = {n: cn.evaluate(cn.rule_of(c, n), c) for n in MODEL}
    assert {n: e["value"] for n, e in es.items()} == HIGH
    result = "4-estimators/winding-temperature"
    # a leak is the result's time constant in whole seconds: the fast node's the pair's own fit, measured,
    # the slow node's the family's, a decision until this motor's is measured
    assert es["thermal_tau_s"] == {
        "value": 2, "class": "measured", "rule": "thermal_tau.fast_node", "rule_rev": 1, "unit": "s", "q": 0,
        "pos_unit": 0, "inputs": [f"{result}:one_node_tau_s"], "band": [1, 3]}
    assert es["thermal_tau2_s"] == {
        "value": 129, "class": "policy", "rule": "thermal_tau2.family", "rule_rev": 1, "unit": "s", "q": 0,
        "pos_unit": 0, "inputs": [f"{result}:family_slow_tau_s"], "band": [121, 135]}
    # a rise is a gain in C/W over the winding's resistance, the current in shunt counts: 100 G R / (counts
    # per A)^2 centi-C per count squared, Q0.16. The fast gain is the one fitted beside the reading's slow node
    per_c_w = 100 * 4.0 / 1000.0 ** 2
    assert HIGH["th_rise_q016"] == raw(20.0 * per_c_w, 16) and HIGH["th_rise2_q016"] == raw(60.0 * per_c_w, 16)
    scales = ["3-servo/resistance:r_rails_at_measured_e_ohm", "1-board/current-scale:counts_per_a"]
    assert es["th_rise_q016"] == {
        "value": 524, "class": "measured", "rule": "th_rise.fast_node", "rule_rev": 1, "unit": "cC/count^2", "q": 16,
        "pos_unit": 0, "inputs": ["policy:winding_plateau_reading", f"{result}:joined_high_fast_gain_c_per_w",
                                  *scales],
        "band": [raw(15.0 * per_c_w, 16), raw(20.0 * 100 * 4.2 / 1000.0 ** 2, 16)]}
    assert es["th_rise2_q016"] == {
        "value": 1573, "class": "policy", "rule": "th_rise2.family", "rule_rev": 1, "unit": "cC/count^2", "q": 16,
        "pos_unit": 0, "inputs": ["policy:winding_plateau_reading",
                                  f"{result}:joined_high_fast_gain_c_per_w.slow_gain_c_per_w", *scales],
        "band": [raw(60.0 * 100 * 3.9 / 1000.0 ** 2, 16), raw(60.0 * 100 * 4.2 / 1000.0 ** 2, 16)]}
    # each names the kernel function its field is read by, and the four are one step
    assert {n: cn.rule_of(c, n).mirrors for n in MODEL} == {
        "thermal_tau_s": "core kernel/phases/loops/limits.rs tau_shift",
        "th_rise_q016": "core estimator/thermal.rs rise_cc",
        "thermal_tau2_s": "core kernel/phases/loops/limits.rs tau_shift",
        "th_rise2_q016": "core estimator/thermal.rs rise_cc"}
    assert cn.GROUPS[cn.MODEL] == MODEL and all(cn.group_of(n) == (cn.MODEL, MODEL) for n in MODEL)
    # the other reading is that one policy entry: the two rises move, the leaks stand
    low = context(tmp_path, policy={"winding_plateau_reading": "low"})
    assert {n: cn.evaluate(cn.rule_of(low, n), low)["value"] for n in MODEL} == LOW
    assert LOW["th_rise_q016"] == raw(21.0 * per_c_w, 16) and LOW["th_rise2_q016"] == raw(32.0 * per_c_w, 16)
    other = context(tmp_path, policy={"winding_plateau_reading": "middle"})
    with pytest.raises(ValueError, match="policy.json: winding_plateau_reading is 'middle': a reading of the "
                                         "plateau is one of low, high"):
        cn.evaluate(cn.rule_of(other, "th_rise_q016"), other)
    inputs(tmp_path, policy=BLOCK_POLICY)


def test_the_winding_model_moves_as_one_step_said_with_what_the_estimate_reads(warm, tmp_path):
    # the pair's set of record with the four held, each rule giving another value than the servo holds
    doc, _ = block(warm[1], tmp_path, hold=dict.fromkeys(MODEL, "the four move as one step"))
    cn.write(doc, cn.set_path(PAIR, tmp_path))
    f = doc["fields"]
    stored = {"thermal_tau_s": 115, "th_rise_q016": 1143, "thermal_tau2_s": 0, "th_rise2_q016": 0}
    assert {n: (f[n]["value"], f[n]["class"], f[n]["would_be"]) for n in MODEL} == {
        n: (stored[n], "none", HIGH[n]) for n in MODEL}
    derived, notes = cn.derive(doc, dev_file(tmp_path, "model", release=list(MODEL)), tmp_path, BOARD_REG)
    assert derived["deviation"]["fields"] == {n: [stored[n], HIGH[n]] for n in f if n in MODEL}
    assert moved_hashes(doc, derived) == ["protection"]
    assert {n: derived["fields"][n]["class"] for n in MODEL} == {
        "thermal_tau_s": "measured", "th_rise_q016": "measured", "thermal_tau2_s": "policy", "th_rise2_q016": "policy"}
    # the build says the step as what the estimate reads, at the set's own continuous current and limit
    said = notes[notes.index("winding model: thermal_tau_s, th_rise_q016, thermal_tau2_s, th_rise2_q016 held at "
                             "115, 1143, 0, 0, wanted at 2, 524, 129, 1573"):]
    assert said[1] == ("winding model: the leaks run at 131.072 s, wanted at 2.048 s and 131.072 s (the nearest "
                       "power of two of the 16 ms period)")
    assert said[2] == ("winding model: the estimate's rise over the board NTC, C, held -> wanted, by the kernel's "
                       "integers:")
    assert len(said) == 3 + 33 + 1 and all(line.startswith("  ") for line in said[3:-1])
    assert said[3].startswith("  a hold from rest at 277 counts, 1 s in: ")
    assert said[12].startswith("  a hold from rest at 554 counts, 1 s in: ")
    assert [line.partition(":")[0].strip() for line in said[21:28]] == REST
    assert said[-1] == "winding model: the wanted four read at least as hot in every case"
    # a part of the model left held is refused: two fields of one model and two of another are no model
    with pytest.raises(ValueError, match="the holds of the winding model are released together, and thermal_tau2_s, "
                                         "th_rise2_q016 would stay held"):
        cn.derive(doc, dev_file(tmp_path, "half", release=["thermal_tau_s", "th_rise_q016"]), tmp_path, BOARD_REG)
    # what the install has to be told: a leak is judged, the fast rise falls, and the slow rise only rises
    state = {"fields": {n: e["value"] for n, e in f.items()}}
    asked = cn.gate(derived, state)
    assert [line.split(":")[0] for line in asked] == [n for n in f if n in MODEL]
    loosen = [n for n in MODEL if n != "th_rise2_q016"]
    assert {line.split(":")[0]: line.rpartition("name it with ")[2] for line in asked} == {
        **{n: f"--protection {n} and --loosen {n}" for n in loosen}, "th_rise2_q016": "--protection th_rise2_q016"}
    assert cn.gate(derived, state, protection=MODEL, loosen=loosen) == []
    # a step that moves none of the four says nothing of the model
    soft, notes = cn.derive(doc, dev_file(tmp_path, "soft", pins={"p_kp_q88": pin(5026)}), tmp_path, BOARD_REG)
    assert soft["deviation"]["fields"] == {"p_kp_q88": [40212, 5026]}
    assert not any(line.startswith("winding model") for line in notes)


def test_an_install_that_moves_the_winding_model_says_what_the_estimate_reads(warm, tmp_path, capsys):
    servo, doc = warm
    # the two readings of the bench pair's plateau, on a servo that holds one node on the slow leak
    high, low = (deviated(doc, **dict(zip(MODEL, four))) for four in ((2, 434, 129, 1274), (2, 461, 129, 680)))
    with pytest.raises(settings.Refused) as e:
        cn.install(high, root=tmp_path)
    said = str(e.value).splitlines()
    # the gate's lines, a field each, then the step as the estimate reads it under the servo's and the set's
    assert said[0] == "the protection gate:" and [line.split(":")[0].strip() for line in said[1:5]] == [
        "thermal_tau_s", "thermal_tau2_s", "th_rise_q016", "th_rise2_q016"]
    assert said[5] == ("  winding model: thermal_tau_s, th_rise_q016, thermal_tau2_s, th_rise2_q016 held at 115, "
                       "1143, 0, 0, wanted at 2, 434, 129, 1274")
    assert said[-1] == "  winding model: the wanted four read at least as hot in every case"
    assert servo.wrote() == []
    # named, the step is said ahead of its writes
    capsys.readouterr()
    loosen = [n for n in MODEL if n != "th_rise2_q016"]
    assert cn.install(high, protection=MODEL, loosen=loosen, root=tmp_path) == [
        "thermal_tau_s", "thermal_tau2_s", "th_rise_q016", "th_rise2_q016", "stamp"]
    out = capsys.readouterr().out.splitlines()
    assert out[0] == said[5].strip() and out.index("thermal_tau_s: 115 -> 2") == out.index(said[-1].strip()) + 1
    # a servo that holds the four as the set does is told nothing of the model
    assert cn.install(high, root=tmp_path) == [] and "winding model" not in capsys.readouterr().out
    # the other reading from the same servo state: it reads cooler through the whole rest, and that is named
    servo.reboot()
    with pytest.raises(settings.Refused) as e:
        cn.install(low, protection=MODEL, root=tmp_path)
    verdict = str(e.value).splitlines()[-1]
    assert verdict.startswith("  winding model: the wanted four read cooler in ") and all(
        f"{case} (-" in verdict for case in REST)
    # a set without the limits the cases run at says so and compares nothing
    bare = {"fields": {n: e for n, e in high["fields"].items() if n != "i_cont_counts"}}
    assert cn.winding_step(bare, {n: 0 for n in MODEL}) == [
        "winding model: the set holds no current limit to compare the estimate at"]


def test_the_bench_pairs_winding_model_is_the_stage_4_high_reading_bound_as_one_step():
    doc, c, _ = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    assert cn.GROUPS[cn.MODEL] == MODEL
    result = results.read("4-estimators", "winding-temperature", pair=doc["pair"])
    published = {k: [v[n] for n in MODEL] for k, v in result["joined_fields"].items()}

    def formed(reading):
        c.policies = {**c.policies, cn.READING: reading}
        return [cn.evaluate(cn.rule_of(c, n), c) for n in MODEL]

    # the rules give the four the stage 4 result publishes for each reading, the policy's the high one
    low, high = ([e["value"] for e in formed(reading)] for reading in cn.READINGS)
    assert policy[cn.READING] == "high" and policy["why"][cn.READING].startswith("the high reading")
    assert high == published["e_high"] == [2, 434, 129, 1274] and low == published["d_low"] == [2, 461, 129, 680]
    es = dict(zip(MODEL, formed(policy[cn.READING])))
    assert {n: (e["class"], e["band"]) for n, e in es.items()} == {
        "thermal_tau_s": ("measured", [1, 3]), "th_rise_q016": ("measured", [331, 436]),
        "thermal_tau2_s": ("policy", [121, 135]), "th_rise2_q016": ("policy", [1271, 1279])}
    # the seconds written run at the leaks the fast gain was fitted beside
    fast = result[f"joined_{policy[cn.READING]}_fast_gain_c_per_w"]
    assert [winding.leak_s(high[0]), winding.leak_s(high[2])] == [fast["tau_s"], fast["slow_tau_s"]] == [2.048, 131.072]
    # the set binds the four as one step, in place of what the servo held, one node on the slow leak: the
    # fast pair measured, the slow pair the policy's, no hold left
    ran = ran_on()
    stored = [ran[n] for n in MODEL]
    assert stored == published["f_one_slow"] == [115, 1143, 0, 0]
    assert [f[n]["value"] for n in MODEL] == high
    assert all("held" not in f[n] and "would_be" not in f[n] for n in MODEL) and not set(MODEL) & set(policy["hold"])
    assert {n: f[n]["class"] for n in MODEL} == {n: e["class"] for n, e in es.items()}
    assert set(MODEL) <= cn.PROTECTION
    cont, limit = f["i_cont_counts"]["value"], f["current_limit_counts"]["value"]
    assert (cont, limit) == (284, 1043)
    # the file of the step released the four together, and an install on a servo that holds the stored four
    # says the step as what the estimate reads
    folder = results.ROOT / "stages" / sc.STAGE / "scorecard" / "deviations"
    assert json.loads((folder / "protection-winding-model.json").read_text())["release"] == list(MODEL)
    lines, cooler = cn.winding_compared(stored, high, cont, limit)
    assert cn.winding_step(doc, ran) == lines and cooler == []
    assert cn.winding_step(doc, {n: e["value"] for n, e in f.items()}) == []
    assert lines[-1] == "winding model: the wanted four read at least as hot in every case"
    assert lines[3:6] == ["  a hold from rest at 284 counts, 1 s in: 0.10 -> 2.17",
                          "  a hold from rest at 284 counts, 3 s in: 0.31 -> 4.52",
                          "  a hold from rest at 284 counts, 10 s in: 1.03 -> 6.59"]
    # at the cap an hour's hold and the rest after 600 s read the estimate's largest centi-C under the high
    # four: the cutoff latches long before
    assert lines[20:22] == ["  a hold from rest at 1043 counts, 3600 s in: 301.47 -> 327.67",
                            "  the rest after 600 s at 1043 counts, 1 s in: 295.48 -> 327.67"]
    assert lines[-3:-1] == ["  30 s on, 30 s off at 1043 counts, the peak of a cycle: 166.18 -> 301.64",
                            "  30 s on, 30 s off at 1043 counts, the mean of a cycle: 149.05 -> 225.25"]
    # the cases are the stage 4 result's own tables, to the centi-C the estimate counts in, tabled at the cap
    # of their day, 568 counts, the 500 mA the cap was before it was read off the maker's sheet
    tables = result["joined_fields_vs_one_slow_node_c"]
    tabled = max(row["counts"] for row in tables["hold"])
    assert tabled == 568 and {row["counts"] for row in tables["hold"]} == {cont, tabled}
    reads = {"f_one_slow": cn.winding_cases(stored, cont, tabled), "e_high": cn.winding_cases(high, cont, tabled)}
    for row in tables["hold"]:
        case = f"a hold from rest at {row['counts']} counts, {row['s']} s in"
        assert all(reads[k][case] == row[k] for k in reads), case
    for row in tables["cooling"]:
        case = f"the rest after 600 s at {tabled} counts, {row['s_into_rest']} s in"
        assert all(reads[k][case] == row[k] for k in reads), case
    for row in (r for r in tables["duty"] if r["cand"] in reads):
        got = [reads[row["cand"]][f"{row['cycle']} at {tabled} counts, the {what} of a cycle"]
               for what in ("peak", "mean")]
        assert got == [row["peak_c"], row["mean_c"]], row["cycle"]
    # the low reading reads cooler than the servo's four through the whole rest, and a hair under them on the
    # plateau: its step would loosen, and the comparison names where
    lines, cooler = cn.winding_compared(stored, low, cont, tabled)
    rest = [f"the rest after 600 s at {tabled} counts, {s} s in" for s in cn.REST_S]
    assert cooler == [f"a hold from rest at {cont} counts, 3600 s in",
                      f"a hold from rest at {tabled} counts, 3600 s in", *rest]
    assert lines[-1].startswith("winding model: the wanted four read cooler in 9 of 33 cases: ")
    assert all(f"{case} (-" in lines[-1] for case in rest)
    assert "the rest after 600 s at 568 counts, 10 s in (-25.81 C)" in lines[-1]
    # what the install of the set has to be told, against a servo that holds the stored four
    state = as_held(doc, MODEL)
    loosen = [n for n in MODEL if n != "th_rise2_q016"]
    assert asked(doc, state) == {
        **{n: f"--protection {n} and --loosen {n}" for n in loosen}, "th_rise2_q016": "--protection th_rise2_q016"}
    assert cn.gate(doc, state, protection=MODEL, loosen=loosen) == []


# --- the fields in counts of position ---

MOTION = ("velocity_limit_cps", "accel_limit_q88")
SCREENS = ("stall_omega_max_cps", "pos_error_counts")       # the protection fields that carry the scale
RANGE = ("pos_min_soft_counts", "pos_max_soft_counts")


def count_rule(c, name):
    return cn.evaluate(cn.rule_of(c, name), c)


@pytest.mark.parametrize("theta", [THETA, THETA[::-1]], ids=["ascending", "descending"])
def test_a_policy_in_degrees_is_that_many_counts_at_the_scale_of_the_count(tmp_path, theta):
    c = context(tmp_path, theta=theta)
    sha = cn.points_sha1(POINTS)
    medium = BOARD_REG["board"]["tick_hz"] / cn.DECIM_MED
    # a speed, an acceleration as the speed one MEDIUM period adds, and an angle, each times the counts per deg
    gives = {"velocity_limit_cps": raw(75.0 * SCALE), "accel_limit_q88": raw(1500.0 * SCALE / medium, 8),
             "stall_omega_max_cps": raw(25.0 * SCALE), "pos_error_counts": raw(20.0 * SCALE)}
    assert gives == {"velocity_limit_cps": 1546, "accel_limit_q88": 3957, "stall_omega_max_cps": 515,
                     "pos_error_counts": 412}
    keys = {"velocity_limit_cps": "velocity_limit_deg_s", "accel_limit_q88": "accel_limit_deg_s2",
            "stall_omega_max_cps": "stall_omega_max_deg_s", "pos_error_counts": "pos_error_deg"}
    for name, value in gives.items():
        e = count_rule(c, name)
        assert (e["value"], e["class"], e["pos_unit"], e["scale_points_sha1"]) == (value, "policy", 1, sha), name
        assert e["inputs"][:2] == [f"policy:{keys[name]}", "scale"], name
    assert count_rule(c, "accel_limit_q88")["inputs"] == ["policy:accel_limit_deg_s2", "scale", "registry:tick_hz"]
    assert (cn.RULES["accel_limit_q88"]["accel_limit.policy"].q, count_rule(c, "accel_limit_q88")["unit"]) == (
        8, "(count/s)/tick")
    # the range is a share of the travel inside each stop, on a count that reads the stops as themselves:
    # it takes the stops and no scale, whichever way the angle runs
    span = STOPS[1] - STOPS[0]
    soft = [count_rule(c, n) for n in RANGE]
    assert [e["value"] for e in soft] == [raw(STOPS[0] + span / 16), raw(STOPS[1] - span / 16)] == [314, 3560]
    assert all(e["inputs"] == ["policy:soft_margin_share", "pos_table:stops"] and e["pos_unit"] == 0
               and "scale_points_sha1" not in e for e in soft)
    wide = context(tmp_path, theta=theta, policy={"soft_margin_share": 0.125})
    assert [count_rule(wide, n)["value"] for n in RANGE] == [raw(82 + span / 8), raw(3792 - span / 8)]
    # the screens that protect say which way protects less, and the limits of motion are of the plant
    assert [cn.LOOSER[n] for n in SCREENS] == ["down", "up"] and cn.LOOSER["sensor_delta_max"] == "up"
    assert not {*MOTION, *RANGE, "pos_deadband_counts"} & cn.PROTECTION


def test_the_hold_band_and_the_sensor_screen_are_in_the_pots_raw_counts_whatever_the_scale(tmp_path):
    def band(sd, **policy):
        c = context(tmp_path, policy=policy)
        plant(tmp_path, {"3-servo/pot-table": {"pot_rest_sd_counts": sd}})
        return count_rule(c, "pos_deadband_counts")

    # the firmware repo's own synthesis (ident/src/gains.rs synthesize and its test): 2.5 times the pot's
    # noise at rest and no less than 4 counts, so a quiet pot sits on the floor and a noisy one clears it
    quiet, noisy = band((1.0, 0.0, 1.5)), band((6.9, 6.0, 8.0))
    assert (quiet["value"], quiet["band"]) == (4, [4, 4])
    assert (noisy["value"], noisy["band"]) == (raw(2.5 * 6.9), [15, 20])
    assert quiet["inputs"] == ["policy:hold_band_noise_multiple", "3-servo/pot-table:pot_rest_sd_counts",
                               "policy:hold_band_floor_counts"]
    assert band((6.9,), hold_band_noise_multiple=1.0, hold_band_floor_counts=10)["value"] == 10
    # neither reads the scale: the kernel takes the band through the table's local gain itself, and screens
    # the pot sample before any table
    c = context(tmp_path, k=3.0)
    screen = count_rule(c, "sensor_delta_max")
    assert (screen["value"], screen["inputs"], screen["unit"]) == (256, ["policy:sensor_delta_max_counts"],
                                                                 "raw counts/tick")
    assert count_rule(c, "pos_deadband_counts")["value"] == 4
    assert all(e["pos_unit"] == 0 and "scale_points_sha1" not in e and e["class"] == "policy"
               for e in (quiet, noisy, screen))
    assert not {"pos_deadband_counts", "sensor_delta_max"} & cn.PER_COUNT


def test_a_pair_with_no_table_takes_its_speeds_at_the_raw_pots_scale_and_has_no_range(tmp_path):
    inputs(tmp_path, theta=None, policy=COUNT_POLICY)
    plant(tmp_path, {"3-servo/pot-table": {"pot_scale_counts_per_deg": (19.0, 18.9, 19.1)}})
    c = cn.Context(PAIR, tmp_path, BOARD_REG)
    cn.table_of(c, None, cn.slots_of(c, None))
    e = count_rule(c, "velocity_limit_cps")
    assert (e["value"], e["scale_points_sha1"]) == (raw(75.0 * 19.0), None)
    with pytest.raises(cn.Missing, match="the pair has no position table: the stops its count runs between"):
        count_rule(c, "pos_max_soft_counts")


def test_a_build_binds_the_range_and_the_band_and_carries_the_screens_to_a_build_that_names_them(first, tmp_path):
    why = "the motion limits move together"
    inputs(tmp_path, policy={**COUNT_POLICY, "hold": {"velocity_limit_cps": why}})
    plant(tmp_path, POT_NOISE)
    doc, notes = cn.build(PAIR, first, root=tmp_path, registry=BOARD_REG)
    f = doc["fields"]
    # the range and the band at the values the servo holds: the class moves and no hash of the fields does
    assert {n: (f[n]["value"], f[n]["class"]) for n in (*RANGE, "pos_deadband_counts")} == {
        "pos_min_soft_counts": (314, "policy"), "pos_max_soft_counts": (3560, "policy"),
        "pos_deadband_counts": (4, "policy")}
    assert all("scale" not in f[n] and first["fields"][n]["scale"] == "unknown" for n in RANGE)
    # the stall speed stands with what its rule gives beside it, the screens the servo lacks wait to be named
    assert f["stall_omega_max_cps"] == {**first["fields"]["stall_omega_max_cps"], "would_be": 515,
                                        "protection": {"looser": "down"}}
    assert "stall_omega_max_cps: carried at 500, it would be 515: name it with --protection" in notes
    assert all(f"{n}: left out, name it with --protection" in notes for n in ("pos_error_counts", "sensor_delta_max"))
    # a limit of motion the policy holds, which the set never held, stays out; the other binds
    assert f"velocity_limit_cps: left out, {why}; its rule gives 1546" in notes and "velocity_limit_cps" not in f
    assert (f["accel_limit_q88"]["value"], f["accel_limit_q88"]["class"]) == (3957, "policy")
    assert doc["hash"]["protection"] == first["hash"]["protection"]
    # named, the stall speed moves alone, up, which protects more than the servo's
    named, _ = cn.build(PAIR, doc, protection=["stall_omega_max_cps"], root=tmp_path, registry=BOARD_REG)
    assert moved_hashes(doc, named) == ["protection"]
    assert (named["fields"]["stall_omega_max_cps"]["value"], named["fields"]["stall_omega_max_cps"]["class"]) == (
        515, "policy")
    state = {"fields": {**{n: e["value"] for n, e in named["fields"].items()}, "stall_omega_max_cps": 500}}
    assert cn.gate(named, state) == ["stall_omega_max_cps: servo 500, set 515, protects more (less is down): name "
                                     "it with --protection stall_omega_max_cps"]


def test_the_bench_pairs_fields_in_counts_are_the_policys_speeds_and_angle_on_the_sets_count():
    doc, c, board = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    typed_at = results.read_file(results.path("3-servo", "pot-table", doc["pair"]))["pot_scale_counts_per_deg"]["value"]
    table, medium = doc["scale"]["counts_per_deg"], board.tick_hz / cn.DECIM_MED
    assert (round(typed_at, 3), round(table, 3), round(table / typed_at, 4)) == (18.913, 20.127, 1.0642)
    # the limits of motion: the owner's default move speed and the acceleration that reaches it in 50 ms,
    # bound by their rules on the set's count, of class `policy`, at a scale that is on record: (the policy's
    # entry, what the rule gives)
    motion = {"velocity_limit_cps": ("velocity_limit_deg_s", 4025), "accel_limit_q88": ("accel_limit_deg_s2", 10305)}
    for name, (key, gives) in motion.items():
        e = count_rule(c, name)
        assert e["value"] == gives and f[name] == {**e, "protection": None}, name
        assert (e["class"], e["scale_points_sha1"], e["inputs"][0]) == (
            "policy", doc["scale"]["points_sha1"], f"policy:{key}"), name
        assert name not in policy["hold"] and key in policy["why"] and key not in policy["undecided"], name
    assert (raw(policy["velocity_limit_deg_s"] * table), raw(policy["accel_limit_deg_s2"] * table / medium, 8)) == (
        4025, 10305)
    # the boot default's counts the servo held before, 1500 count/s and 3840 count/s a MEDIUM tick in Q8, were
    # typed for these at the raw pot's mid travel scale, four figures, and are 6.4 % less of each on the
    # table's count
    stored = {"velocity_limit_cps": 1500, "accel_limit_q88": 3840 / 256 * medium}
    assert {n: float(f"{counts / typed_at:.4g}") for n, counts in stored.items()} == {
        "velocity_limit_cps": 79.31, "accel_limit_q88": 1586.0}
    assert {n: float(f"{counts / table:.4g}") for n, counts in stored.items()} == {
        "velocity_limit_cps": 74.53, "accel_limit_q88": 1491.0}
    # the stall speed is decided at the speed its counts are on the set's count, so its rule gives the counts
    # the servo holds: it binds, of class `policy`, at a scale that is on record, and has no step
    stall = count_rule(c, "stall_omega_max_cps")
    assert policy["stall_omega_max_deg_s"] == float(f"{500 / table:.4g}") == 24.84
    assert stall["value"] == 500 and f["stall_omega_max_cps"] == {**stall, "protection": {"looser": "down"}}
    assert (stall["class"], stall["scale_points_sha1"]) == ("policy", doc["scale"]["points_sha1"])
    assert "stall_omega_max_cps" not in policy["hold"] and "stall_omega_max_deg_s" in policy["why"]
    # the position-error screen is decided at an angle, which is fewer counts than the servo held: bound by
    # its step at what the angle is on the set's count, of class `policy`, at a scale that is on record
    ran = ran_on()
    error = count_rule(c, "pos_error_counts")
    assert error["value"] == raw(policy["pos_error_deg"] * table) == 199 and float(f"{400 / table:.4g}") == 19.87
    assert ran["pos_error_counts"] == 400 and f["pos_error_counts"] == {**error, "protection": {"looser": "up"}}
    assert (error["class"], error["scale_points_sha1"]) == ("policy", doc["scale"]["points_sha1"])
    assert not {"pos_error_counts", "pos_error_time_ticks"} & set(policy["hold"])
    # the range is what it was typed as, a sixteenth of the stops' span inside each stop, and binds as it stands
    lo, hi = (f[n]["value"] for n in cn.STOPS)
    assert policy["soft_margin_share"] == 1 / 16
    assert [(f[n]["value"], f[n]["class"]) for n in RANGE] == [(raw(lo + (hi - lo) / 16), "policy"),
                                                               (raw(hi - (hi - lo) / 16), "policy")]
    assert [f[n]["value"] for n in RANGE] == [314, 3560]
    # on the raw pot those counts sat nearer the stops than on the table, deg inside the low and the high one
    inside = [(float(c.table.lin(314)) - lo) / table, (hi - float(c.table.lin(3560))) / table]
    assert [round(x, 2) for x in inside] == [10.4, 10.86] and round((314 - lo) / table, 2) == 11.53
    # the band on its floor over the pot's noise, and the sensor screen, in the pot's raw counts
    noise = results.read_file(results.path("3-servo", "pot-table", doc["pair"]))["pot_rest_sd_counts"]
    assert max(policy["hold_band_noise_multiple"] * noise[k] for k in ("value", "lo", "hi")) < 4
    # both are decisions: the band binds at the value the servo held, the screen by its step at the decision's,
    # a quarter of the boot default the servo held
    assert [(f[n]["value"], f[n]["class"], f[n]["rule"]) for n in ("pos_deadband_counts", "sensor_delta_max")] == [
        (4, "policy", "deadband.pot_noise"), (64, "policy", "sensor_delta.policy")]
    assert f["sensor_delta_max"]["value"] == policy["sensor_delta_max_counts"] and ran["sensor_delta_max"] == 256
    assert "would_be" not in f["sensor_delta_max"] and "held" not in f["sensor_delta_max"]
    # the velocity loop's two gains are a rung's, benched in counts: no field is left at a scale nothing
    # recorded
    assert all(f[n]["class"] == "bench" and "scale" not in f[n] for n in ("v_kp_q88", "v_ki_q412"))
    assert [n for n, e in f.items() if e.get("scale") == "unknown"] == []
    assert cn.described(doc)[0] == ("63 fields: 16 measured, 10 bench, 1 datasheet, 34 policy, 1 pinned, 1 none "
                                    "(no source); 0 per-count at an unknown scale")


def test_the_steps_of_the_fields_in_counts_name_what_their_install_needs():
    doc, _, _ = of_the_bench_pair()
    state = {"fields": {n: e["value"] for n, e in doc["fields"].items()}}
    folder = results.ROOT / "stages" / sc.STAGE / "scorecard" / "deviations"
    # the limits of motion are of the plant: on a servo that holds the boot default's counts, the set's install
    # moves them with no gate
    held = {"fields": {**state["fields"], "velocity_limit_cps": 1500, "accel_limit_q88": 3840}}
    assert not set(MOTION) & cn.PROTECTION and cn.gate(doc, held) == []
    # the position-error screen trips on a smaller error held for a shorter time: one protection step of the
    # angle and the time, built into the set of record. It protects more, so on a servo that holds the boot
    # default the install names the two and loosens nothing
    screen = ("pos_error_counts", "pos_error_time_ticks")
    f = doc["fields"]
    assert json.loads((folder / "protection-position-error.json").read_text())["release"] == list(screen)
    assert {n: f[n]["value"] for n in screen} == {"pos_error_counts": 199, "pos_error_time_ticks": 2000}
    assert set(screen) <= cn.PROTECTION
    before = as_held(doc, screen)
    assert cn.gate(doc, before) == [
        "pos_error_time_ticks: servo 10000, set 2000, protects more (less is up): name it with --protection "
        "pos_error_time_ticks",
        "pos_error_counts: servo 400, set 199, protects more (less is up): name it with --protection "
        "pos_error_counts"]
    assert cn.gate(doc, before, protection=screen) == []
    assert all(f[n]["class"] == "policy" and "scale" not in f[n] and "held" not in f[n] for n in screen)
    assert [cn.group_of(n) for n in screen] == [("position error", screen)] * 2
    # the stall speed is bound at the counts the servo holds: a step alone by its group, with nothing to move
    # and no deviation of record
    assert cn.group_of("stall_omega_max_cps") == ("stall_omega_max_cps", ("stall_omega_max_cps",))
    assert "held" not in doc["fields"]["stall_omega_max_cps"] and not (folder / "protection-stall-speed.json").exists()


# --- the screens' times and counts, the supply terms, the choices in words, the current loop, the records ---

class Screened(Geared):
    """The fake servo with the position-error screen's two fields."""

    FIELDS = dict(sorted({**Geared.FIELDS, "pos_error_time_ticks": (0x60, 4, "rw", "uint"),
                          "pos_error_counts": (0x64, 2, "rw", "uint")}.items(), key=lambda item: item[1][0]))
    STORED = {**Geared.STORED, "pos_error_time_ticks": 10000, "pos_error_counts": 400}


def test_a_time_a_count_of_samples_a_choice_in_words_and_the_supply_terms_are_the_policys(tmp_path):
    def gives(name, **policy):
        c = context(tmp_path, policy=policy)
        return cn.evaluate(cn.rule_of(c, name), c)

    # a time in ms is that many kernel ticks, a count of samples is the field
    assert {n: gives(n)["value"] for n in ("stall_time_ticks", "pos_error_time_ticks", "oc_trip_ticks",
                                           "sensor_bad_count")} == {
        "stall_time_ticks": 10000, "pos_error_time_ticks": 2000, "oc_trip_ticks": 8, "sensor_bad_count": 4}
    assert gives("stall_time_ticks")["inputs"] == ["policy:stall_time_ms", "registry:tick_hz"]
    assert gives("pos_error_time_ticks", pos_error_time_ms=0.25)["value"] == 5
    # a choice between named behaviors is the policy's word, written as `osc dump` prints the field
    assert [gives("stall_response", stall_response=w)["value"] for w in ("fault", "yield")] == [
        "Fault (0)", "Yield (1)"]
    assert [gives("openloop_zero_brake", openloop_zero_brake=w)["value"] for w in ("off", "on")] == ["off", "on"]
    for name, word, match in (("stall_response", "coast", "stall_response is 'coast': it is one of fault, yield"),
                              ("openloop_zero_brake", "no", "openloop_zero_brake is 'no': it is off or on")):
        with pytest.raises(ValueError, match=f"policy.json: {match}"):
            gives(name, **{name: word})
    # the duty cap is a share of the rail, 1 the whole of it, which is the field's largest value
    assert [gives("duty_max_q15", duty_max_share=x)["value"] for x in (1.0, 0.5)] == [32767, 16384]
    for share in (0, 1.01):
        with pytest.raises(ValueError, match="duty_max_share is .*: a share of the rail, over 0 and at most 1"):
            gives("duty_max_q15", duty_max_share=share)
    # the undervolt floor is the policy's volts at the rail on the board's rail line, its offset with it,
    # and the rail fold's slope the shunt counts a rail count of sag gives through the policy's source
    floor = gives("v_undervolt_counts")
    assert (floor["value"], floor["band"], floor["unit"]) == (raw(6.0 + 3.0 * 248.0), [747, 753], "vcounts")
    assert floor["inputs"] == ["1-board/terminal-chain:rail_offset_counts", "policy:v_undervolt_v",
                               "1-board/terminal-chain:rail_counts_per_v"]
    fold = gives("rail_fold_q8")
    assert (fold["value"], fold["q"]) == (raw(1000.0 / (248.0 * 0.5), 8), 8)
    assert fold["band"] == [raw(990.0 / (248.0 * 0.5), 8), raw(1010.0 / (248.0 * 0.5), 8)]
    assert gives("rail_fold_q8", rail_source_ohm=1.0)["value"] == raw(1000.0 / 248.0, 8)
    # every one of them is a decision, and all but the open loop's brake flag are protection fields
    named = ("stall_response", "stall_time_ticks", "pos_error_time_ticks", "oc_trip_ticks", "sensor_bad_count",
             "duty_max_q15", "v_undervolt_counts", "rail_fold_q8", "openloop_zero_brake")
    assert all(gives(n)["class"] == "policy" and gives(n)["pos_unit"] == 0 for n in named)
    assert [n for n in named if n not in cn.PROTECTION] == ["openloop_zero_brake"]


def test_the_position_error_screen_moves_as_one_step_and_the_stall_response_as_one_alone(monkeypatch, tmp_path):
    Screened().stub(monkeypatch)
    first, _ = cn.adopt(PAIR, record(tmp_path), root=tmp_path)
    screen = ("pos_error_counts", "pos_error_time_ticks")
    why = "it moves on a protection step of its own"
    inputs(tmp_path, policy={**BLOCK_POLICY, **COUNT_POLICY, **CHOICE_POLICY, "stall_response": "yield",
                             "hold": dict.fromkeys((*screen, "stall_response"), why)})
    plant(tmp_path, {**PLANT, **POT_NOISE})
    doc, _ = cn.build(PAIR, first, root=tmp_path, registry=BOARD_REG)
    cn.write(doc, cn.set_path(PAIR, tmp_path))
    f = doc["fields"]
    assert {n: (f[n]["value"], f[n]["would_be"]) for n in (*screen, "stall_response")} == {
        "pos_error_counts": (400, 412), "pos_error_time_ticks": (10000, 2000),
        "stall_response": ("Fault (0)", "Yield (1)")}
    assert doc["hash"]["protection"] == first["hash"]["protection"]

    def derived(name, *release):
        return cn.derive(doc, dev_file(tmp_path, name, release=list(release)), tmp_path, BOARD_REG)[0]

    # the angle and the time are one test: released together, or not at all
    assert cn.GROUPS["position error"] == screen and cn.group_of("pos_error_time_ticks") == ("position error", screen)
    step = derived("screen", *screen)
    assert step["deviation"]["fields"] == {"pos_error_time_ticks": [10000, 2000], "pos_error_counts": [400, 412]}
    assert moved_hashes(doc, step) == ["protection"]
    for name, left in zip(screen, screen[::-1]):
        with pytest.raises(ValueError, match=f"the holds of the position error are released together, and {left} "
                                             f"would stay held"):
            derived("half", name)
    with pytest.raises(ValueError, match="of 2: position error, stall_response"):
        derived("two", *screen, "stall_response")
    # a shorter time protects more and a wider angle less: the install names both, the angle to loosen as well
    state = {"fields": {n: e["value"] for n, e in f.items()}}
    assert {line.split(":")[0]: line.rpartition("name it with ")[2] for line in cn.gate(step, state)} == {
        "pos_error_time_ticks": "--protection pos_error_time_ticks",
        "pos_error_counts": "--protection pos_error_counts and --loosen pos_error_counts"}
    # the stall response is a step alone, and no direction says which of its two protects less
    yields = derived("yield", "stall_response")
    assert yields["deviation"]["fields"] == {"stall_response": ["Fault (0)", "Yield (1)"]}
    assert yields["fields"]["stall_response"] == {
        "value": "Yield (1)", "class": "policy", "rule": "stall_response.policy", "rule_rev": 1, "unit": "response",
        "q": 0, "pos_unit": 0, "inputs": ["policy:stall_response"], "protection": {"looser": "judged"}}
    assert cn.gate(yields, state) == ["stall_response: servo Fault (0), set Yield (1), judged, taken as protecting "
                                      "less: name it with --protection stall_response and --loosen stall_response"]


def test_the_bench_pairs_decisions_are_bound_the_ones_the_servo_held_and_the_ones_a_step_moved():
    doc, c, board = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    # a decision the servo already held: (the policy's entry, what its rule gives)
    stand = {"duty_max_q15": ("duty_max_share", 32767), "stall_time_ticks": ("stall_time_ms", 10000),
             "stall_omega_max_cps": ("stall_omega_max_deg_s", 500), "oc_trip_counts": ("oc_trip_ma", 2497),
             "oc_trip_ticks": ("oc_trip_samples", 8), "sensor_bad_count": ("sensor_bad_samples", 4),
             "openloop_zero_brake": ("openloop_zero_brake", "off")}
    for name, (key, gives) in stand.items():
        e = cn.evaluate(cn.rule_of(c, name), c)
        assert (e["value"], e["class"], e["inputs"][0]) == (gives, "policy", f"policy:{key}"), name
        # bound as it stands, with the why of its entry: no value moves, and the field leaves the debt
        assert f[name] == {**e, "protection": cn.protection_of(name)} and key in policy["why"], name
    # 500 ms and 0.4 ms in the kernel's ticks, and the whole rail as the field holds it
    assert policy["stall_time_ms"] * board.tick_hz / 1000 == 10000
    assert 1000 * policy["oc_trip_samples"] / board.tick_hz == 0.4 and "0.4 ms" in policy["why"]["oc_trip_samples"]
    assert policy["duty_max_share"] == 1 and f["duty_max_q15"]["value"] == cn.DUTY_FULL_Q15
    # a decision that is another value than the servo held: (the policy's entry, what the servo held, what
    # its rule gives)
    ran = ran_on()
    moves = {"stall_response": ("stall_response", "Fault (0)", "Yield (1)"),
             "pos_error_time_ticks": ("pos_error_time_ms", 10000, 2000),
             "pos_error_counts": ("pos_error_deg", 400, 199),
             "sensor_delta_max": ("sensor_delta_max_counts", 256, 64),
             "v_undervolt_counts": ("v_undervolt_v", 720, 922),
             "rail_fold_q8": ("rail_source_ohm", 1430, 1467)}
    for name, (key, stored, gives) in moves.items():
        e = cn.evaluate(cn.rule_of(c, name), c)
        assert (e["value"], e["class"]) == (gives, "policy") and f"policy:{key}" in e["inputs"], name
        # bound at the decision's value by a protection step of its own, with the why of its entry: the
        # field left the debt, and the value the servo held is the reference rung's
        assert f[name] == {**e, "protection": cn.protection_of(name)} and ran[name] == stored != gives, name
        assert key in policy["why"] and name not in policy["hold"], name
    assert set(moves) <= cn.PROTECTION
    # the rail fold's slope is the shunt counts a rail count of sag gives through the policy's source, on the
    # board's measured counts per A and rail counts per V
    chain = results.read_file(results.path("1-board", "terminal-chain", board.key))
    measured, rail = doc["scales"][cn.CURRENT]["value"], chain["rail_counts_per_v"]["value"]
    assert raw(measured / (rail * policy["rail_source_ohm"]), 8) == 1467
    assert cn.evaluate(cn.rule_of(c, "rail_fold_q8"), c)["band"] == [1466, 1469]
    # the slope the servo held is the same source through the board's nominal parts, by the firmware's own
    # integers (core regions/config.rs rail_fold_q8): the one whole mohm they take to it
    cpa, cpv = raw(board.counts_per_a), raw(1 / board.v_rail_per_count)
    typed = [z for z in range(1, 2000) if (cpa * 1000 * 256 + cpv * z // 2) // (cpv * z) == 1430]
    assert (cpa, cpv) == (1108, 248) and typed == [1000 * policy["rail_source_ohm"]]
    # which on the measured scales is a softer source, so the rule's slope is steeper and folds later
    assert round(256 * measured / (rail * 1430), 3) == 0.821 and cn.LOOSER["rail_fold_q8"] == "judged"


def test_the_bench_pairs_undervolt_floor_is_over_the_floor_of_the_rail_taps_reading():
    doc, c, board = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    chain = results.read_file(results.path("1-board", "terminal-chain", board.key))
    scale, offset = chain["rail_counts_per_v"]["value"], chain["rail_offset_counts"]["value"]
    # the floor is the policy's volts at the rail as the rail tap reads them, on the board's measured line
    e = cn.evaluate(cn.rule_of(c, "v_undervolt_counts"), c)
    assert e["value"] == raw(offset + policy["v_undervolt_v"] * scale) == 922 and e["band"] == [922, 922]
    assert (e["class"], e["inputs"]) == ("policy", [
        "1-board/terminal-chain:rail_offset_counts", "policy:v_undervolt_v",
        "1-board/terminal-chain:rail_counts_per_v"])
    # which that result grades from 3.685 V up, so the floor is on the stretch the line is true over
    assert chain["rail_range_v"]["lo"] <= policy["v_undervolt_v"] <= chain["rail_range_v"]["hi"]
    # the rail walk is a dated bench record beside the set, and its numbers are the ones its rows hold
    record = json.loads((results.ROOT / cn.FOLDER / f"{doc['pair']}.rail-walk-2026-11-14.json").read_text())
    assert record["date"] == "2026-11-14" and record["how"] and record["says"] and record["not_measured"]
    rows = []
    for file in record["rows"]:
        with open(results.ROOT / file, newline="") as fh:
            rows += [{k: float(v) for k, v in row.items()} for row in csv.DictReader(fh)]
    assert [len(record["rows"]), len(rows)] == [2, 39 + 51] and all(row["good_reads"] == 3 for row in rows)
    assert not any(Path(file).is_absolute() for file in record["rows"])

    def band(key, keep):
        values = [row[key] for row in rows if keep(row["dps_set_v"])]
        return {"lo": min(values), "hi": max(values)}

    # down to the edge the chip's supply, the ADC's reference, holds, so the reading follows the rail; under
    # it the supply follows the rail down and the reading stops, the same on the three passes over the edge
    edge = record["edge_supply_v"]
    assert band("vref_raw", lambda v: v >= edge) == record["vref_in_regulation_counts"]
    assert min(row["vref_raw"] for row in rows if row["dps_set_v"] < edge) > record["vref_in_regulation_counts"]["hi"]
    assert band("vbus_raw", lambda v: v == edge) == record["edge_vbus_counts"] == {"lo": 860, "hi": 861}
    assert sum(row["dps_set_v"] == edge for row in rows) == 3
    assert band("vbus_raw", lambda v: v <= edge) == record["floor_vbus_counts"] == {"lo": 857, "hi": 862}
    # the threshold the servo held is under that floor, so it could not trip on a rail the chip survives, and
    # the decided one, which the set binds, is over it: 0.25 V over the edge, at the rail, by the measured line
    floor = record["floor_vbus_counts"]
    assert ran_on()["v_undervolt_counts"] == 720 < floor["lo"]
    assert floor["hi"] < e["value"] == f["v_undervolt_counts"]["value"]
    edge_v = [(record["edge_vbus_counts"][k] - offset) / scale for k in ("lo", "hi")]
    assert [round(v, 2) for v in edge_v] == [3.45, 3.45] and round(policy["v_undervolt_v"] - edge_v[1], 2) == 0.25
    assert "0.25 V over the edge the rail walk of 2026-11-14 found" in policy["why"]["v_undervolt_v"]
    # bound by its step: against the floor the servo held it protects more, named and not to loosen
    assert "held" not in f["v_undervolt_counts"] and cn.looser("v_undervolt_counts", 720, 922) is False
    assert asked(doc, as_held(doc, ["v_undervolt_counts"])) == {"v_undervolt_counts": "--protection v_undervolt_counts"}


LOOP = ("i_kp_q88", "i_ki_q412", "i_kaw_q412")
OBSERVER = ("l1_q016", "l2_q88", "l3_q88")


def swept(c, key, names, values):
    """What the rules of `names` give with the policy's `key` at each
    of `values`, the rest of the policy as it stands."""
    was, out = c.policies, {}
    for value in values:
        c.policies = {**was, key: value}
        out[value] = [cn.evaluate(cn.rule_of(c, n), c)["value"] for n in names]
    c.policies = was
    return out


def test_the_bench_pairs_current_loop_is_the_placement_on_this_motor_at_the_bandwidth_its_sweep_kept():
    doc, c, board = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    henry = results.read("3-servo", "inductance", pair=doc["pair"])["l_mh"]["value"] / 1000
    ohm = results.read("3-servo", "resistance", pair=doc["pair"])["r_rails_at_measured_e_ohm"]["value"]
    chain = results.read_file(results.path("1-board", "terminal-chain", board.key))
    per_ohm = chain["terminal_counts_per_v"]["value"] / doc["scales"][cn.CURRENT]["value"]
    # the firmware repo's placement (ident/src/gains.rs synthesize): w L, w R a FAST tick and twice that, in
    # counts through the measured scales, at the policy's bandwidth, 1 kHz
    w = 2 * math.pi * policy["current_loop_hz"]
    by_hand = [raw(w * henry * per_ohm, 8), raw(w * ohm * per_ohm / board.tick_hz, 12),
               raw(2 * w * ohm * per_ohm / board.tick_hz, 12)]
    es = {n: cn.evaluate(cn.rule_of(c, n), c) for n in LOOP}
    assert [e["value"] for e in es.values()] == by_hand == [289, 1176, 2352] and policy["current_loop_hz"] == 1000.0
    assert (round(1000 * henry, 2), round(ohm, 2)) == (0.82, 4.18)
    # the band of the proportional gain is the inductance's, of the other two the resistance's
    assert [e["band"] for e in es.values()] == [[275, 308], [1174, 1181], [2347, 2361]]
    assert es["i_kp_q88"]["inputs"] == ["policy:current_loop_hz", "3-servo/inductance:l_mh",
                                        "1-board/terminal-chain:terminal_counts_per_v",
                                        "1-board/current-scale:counts_per_a"]
    # the resistance is the one the policy names for r_q12, the rails fit, read through the same scales
    assert policy["rule"]["r_q12"] == "r.rails_fit"
    assert es["i_ki_q412"]["inputs"] == es["i_kaw_q412"]["inputs"] == [
        "policy:current_loop_hz", *f["r_q12"]["inputs"][1:], "registry:tick_hz"]
    # the three are bound at the placement, in place of the servo's old fit of another servo. The bandwidth
    # is a target, the policy's: its sweep ran and kept the 1 kHz, so the three are the policy's by their rule
    ran, why = ran_on(), policy["why"]["current_loop_hz"]
    assert [ran[n] for n in LOOP] == [275, 1580, 3160]
    for name, e in es.items():
        assert f[name] == {**e, "class": "policy", "protection": None}, name
    assert not set(LOOP) & set(policy["hold"]) and not set(LOOP) & cn.PROTECTION
    assert "current_loop_hz" not in policy["undecided"] and why.endswith(SWEEP_PICK)
    assert why.startswith("1 kHz, the middle of the sweep's 500, 1000 and 1500 Hz")
    # a deviation's policy entry sweeps the bandwidth, and every gain is in proportion to it
    assert swept(c, "current_loop_hz", LOOP, (500.0, 1000.0, 1500.0)) == {
        500.0: [144, 588, 1176], 1000.0: [289, 1176, 2352], 1500.0: [433, 1764, 3528]}


def test_the_bench_pairs_observer_gains_move_with_the_policys_bandwidth():
    doc, c, _ = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    # what a deviation's `observer_hz` gives: the three gains bind at it, the disturbance gain with them now
    # that its hold is released
    assert swept(c, "observer_hz", OBSERVER, (10.0, 15.0, 25.0)) == {
        10.0: [5894, 1446, 25], 15.0: [8640, 3180, 84], 25.0: [13757, 8433, 371]}
    assert policy["observer_hz"] == 15.0 and "observer_hz" not in policy["undecided"]
    assert [f[n]["value"] for n in OBSERVER] == [8640, 3180, 84] and all(f[n]["class"] == "policy" for n in OBSERVER)
    assert ["held" in f[n] for n in OBSERVER] == [False, False, False]
    # the bandwidth is a target, the policy's: its sweep ran and kept the 15 Hz
    why = policy["why"]["observer_hz"]
    assert why.startswith("15 Hz, the middle of the sweep's 10, 15 and 25 Hz") and why.endswith(SWEEP_PICK)
    assert not any("reason" in f[n] for n in OBSERVER)
    # the release that held the disturbance gain's hold is built into the set: no hold of its three is left
    release = json.loads((results.ROOT / "stages" / sc.STAGE / "scorecard" / "deviations"
                          / "motor-model.json").read_text())["release"]
    assert "l3_q88" in release and not set(release) & set(policy["hold"])
    assert ran_on()["l3_q88"] == 77


def test_the_bench_pairs_host_records_are_its_results_of_record_but_the_breakaway():
    doc, c, board = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    resistance = results.read("3-servo", "resistance", pair=doc["pair"])
    chain = results.read_file(results.path("1-board", "terminal-chain", board.key))
    ke = results.read("3-servo", "ladder", pair=doc["pair"])["ke_v_s_per_rad"]["value"]
    static = results.read("3-servo", "friction", pair=doc["pair"])["load_static_friction_uNm_motor"]["value"]
    per_a = doc["scales"][cn.CURRENT]["value"]
    # the cold resistance: the rails fit, the winding's at the board temperature its result states, taken to
    # the 25 C the field is stated at along copper's slope, in counts like r_q12
    ohm, at_c = resistance["r_rails_at_measured_e_ohm"]["value"], resistance["board_ntc_c"]["value"]
    cold = winding.r_of_t(cn.R0_AT_C, ohm, at_c)
    assert (cn.R0_AT_C, round(at_c, 2), round(ohm, 3), round(cold, 3)) == (25.0, 26.26, 4.177, 4.156)
    # field: (what the servo held, what its rule gives, the rule's band)
    ran = ran_on()
    records = {"r0_q12": (0, raw(cold * chain["terminal_counts_per_v"]["value"] / per_a, 12), [3717, 3740]),
               "ke_uvs_per_rad": (0, raw(1e6 * ke), [1152, 1166]),
               "fric_breakaway_counts": (0, raw(static * 1e-6 / ke * per_a), [79, 114])}
    assert {n: gives for n, (_, gives, _) in records.items()} == {
        "r0_q12": 3725, "ke_uvs_per_rad": 1159, "fric_breakaway_counts": 97}
    for name, (stored, gives, band) in records.items():
        e = cn.evaluate(cn.rule_of(c, name), c)
        assert (e["value"], e["band"], e["class"], e["pos_unit"]) == (gives, band, "measured", 0), name
        assert ran[name] == stored, name
        if name == "fric_breakaway_counts":
            continue
        # bound at the result's, measured, in place of a zero never written
        assert f[name] == {**e, "protection": None} and name not in policy["hold"], name
    # the breakaway stays held at the zero never written, with no source: its rule reads the static friction
    # of record, which is the back-driven one of the load arm, and the field means a driven ramp's breakaway
    e = f["fric_breakaway_counts"]
    assert (e["value"], e["class"], e["would_be"], e["rule"]) == (0, "none", 97, "fric_breakaway.static")
    assert e["held"] == policy["hold"]["fric_breakaway_counts"] and "back-driven" in e["held"]
    assert e["needs"].startswith("a driven-ramp breakaway capture") and "3-servo/friction" in e["inputs"][0]
    # the warm resistance of record, which r_q12's rule gives, is the same fit at its own temperature
    assert f["r_q12"]["value"] == 3744 and f["r0_q12"]["inputs"] == [
        *f["r_q12"]["inputs"][1:], "3-servo/resistance:board_ntc_c"]
    assert not set(records) & cn.PROTECTION


def test_the_bench_pairs_supply_word_is_the_boards_sense_word_pinned_and_no_measurement():
    doc, _, board = of_the_bench_pair()
    f = doc["fields"]
    # a capture carries the sense block of the servo it ran on, and is matched to its board by it: every word
    # of the block the set holds as a field is the board entry's, or the set's captures match no board
    held = {key: f[key]["value"] for key in board.sense if key in f}
    assert held == {"vdd_mv": board.sense["vdd_mv"]} == {"vdd_mv": board.vdd_mv} == {"vdd_mv": 3300}
    # it is the pair's pin, with its why and the rung that showed it, and what the servo held before
    pins = json.loads(cn.set_path(doc["pair"]).with_name(f"{doc['pair']}.pins.json").read_text())
    assert list(pins) == ["vdd_mv"] and f["vdd_mv"] == {**pins["vdd_mv"], "class": "pinned", "protection": None}
    assert all(word in pins["vdd_mv"]["why"] for word in ("nominal supply", "sense block", "oscnb/boards.py"))
    assert pins["vdd_mv"]["rung"] == "16b-constants-with-sources" and ran_on()["vdd_mv"] == board.vdd_mv
    # no rule binds it: the supply as measured stays in the board's stage 1 result, and is another number
    chain = results.read_file(results.path("1-board", "terminal-chain", board.key))
    assert "vdd_mv" not in cn.RULES and raw(1000 * chain["vdd_v"]["value"]) == 3293 != board.vdd_mv
    assert not any("vdd_v" in i for e in f.values() for i in e.get("inputs", ()))
    # so a field left without its pin is carried as it stands, with why it has no rule, and never bound at
    # the measured supply: the pair's sources say so
    said = json.loads(cn.set_path(doc["pair"]).with_name(f"{doc['pair']}.sources.json").read_text())["none"]["vdd_mv"]
    assert "no rule gives it" in said["reason"] and said["needs"].startswith("nothing measured")
    # a capture of a rung on the set matches the board, and one of a set at the measured supply would match
    # no board entry, which is how the rung that showed it could not be scored
    folder = results.ROOT / "telemetry" / doc["pair"] / "scorecard" / "17a-constants-of-record"
    meta = json.loads((folder / "run-1" / "steps" / "meta.json").read_text())
    assert meta["sense"]["vdd_mv"] == f["vdd_mv"]["value"] and boards.from_meta(meta) is not None
    board.check_meta(meta)
    assert boards.from_meta({"sense": {**board.sense, "vdd_mv": 3293}}) is None
    # and no deviation of record moves it
    for path in sorted(DEVIATIONS.glob("*.json")):
        file = json.loads(path.read_text())
        assert "vdd_mv" not in file.get("pins", {}) and "vdd_mv" not in file.get("release", []), path.name


# --- the set of record over the set before its releases, and a session's deviations over it ---

DEVIATIONS = results.ROOT / "stages" / sc.STAGE / "scorecard" / "deviations"
BEFORE = "297f3cb4a544"     # the set from before the releases, as a save names it: its hash's first 12 characters
AS_HELD = "the value the board held, for the A/B against the constants of record"
BISECT = "the value the board held, for the bisect of the constants of record"
STEPS = ("stall_response", "thermal_tau_s", "v_undervolt_counts", "thermal_tau2_s", "pos_error_time_ticks",
         "pos_error_counts", "sensor_delta_max", "rail_fold_q8", "th_rise_q016", "th_rise2_q016", "ntc_t_ref_cc",
         "ntc_k1_q88", "ntc_k2_q24")    # the fields the seven protection steps moved, in the set's order
SHEET_STEP = ("current_limit_counts", "speed_ceiling_cps")     # and the step read off the maker's sheet
LOOSEN = ("stall_response", "thermal_tau_s", "thermal_tau2_s", "rail_fold_q8", "th_rise_q016", "ntc_t_ref_cc",
          "ntc_k1_q88", "ntc_k2_q24")   # of those, the judged ones and the fast rise, which falls


def before_the_releases(doc):
    """(the set of record as it was before its holds were released, the
    fields of the plant the release moved, the protection fields the
    steps moved): every field `plant-as-held.json` pins, every field
    the file of a protection step released and the limits of motion,
    released since, at the value the reference rung ran on; the cap at
    the 500 mA it was before it was read off the maker's sheet, and no
    speed ceiling, which the table had no field for."""
    plant = list(json.loads((DEVIATIONS / "plant-as-held.json").read_text())["pins"])
    steps = [n for path in sorted(DEVIATIONS.glob("protection-*.json"))
             for n in json.loads(path.read_text())["release"]]
    ran = ran_on()
    cap = raw(0.5 * doc["scales"][cn.CURRENT]["value"])
    before = deviated(doc, **{n: ran[n] for n in (*plant, *steps, *MOTION)}, current_limit_counts=cap)
    del before["fields"][cn.CEILING]
    return {**before, "hash": cn.hashes(before)}, plant, steps


def test_an_install_of_the_set_of_record_over_the_set_before_its_releases_names_every_protection_step():
    doc, _, _ = of_the_bench_pair()
    f = doc["fields"]
    before, plant, steps = before_the_releases(doc)
    # the set a servo stores from before the releases: the plant the reference rung ran on, under the hash
    # a save kept it by, and it differs from the set of record in those fields and no other
    rung = json.loads((results.ROOT / "telemetry" / doc["pair"] / "scorecard" / REFERENCE / "rung.json").read_text())
    assert before["hash"]["plant"] == rung["set"]["hash"]["plant"] != doc["hash"]["plant"]
    assert before["hash"]["set"][:12] == BEFORE != doc["hash"]["set"][:12]
    state = {"fields": {n: e["value"] for n, e in before["fields"].items()}}
    differ = [n for n in f if state["fields"].get(n) != f[n]["value"]]
    assert differ == [n for n in f if n in (*plant, *steps, *MOTION, *SHEET_STEP)] and (len(plant), len(steps)) == (
        11, 13)
    assert not set(plant) & cn.PROTECTION and [n for n in f if n in steps] == list(STEPS)
    # the step read off the maker's sheet is one of its own: the cap rises, which protects less, and the
    # ceiling joins the set, a field the table of that day had none of. Over a servo that holds it, the gate
    # asks for the seven steps alone
    assert cn.CEILING not in state["fields"] and asked(doc, {"fields": {**state["fields"], cn.CEILING: f[
        cn.CEILING]["value"]}})["current_limit_counts"] == ("--protection current_limit_counts and --loosen "
                                                            "current_limit_counts")
    state["fields"].update({n: f[n]["value"] for n in SHEET_STEP})
    # the gate asks for every field the seven steps moved as a protection change, in the set's order, and
    # again to loosen for the eight that are judged or protect less
    assert asked(doc, state) == {n: f"--protection {n}" + (f" and --loosen {n}" if n in LOOSEN else "") for n in STEPS}
    assert list(asked(doc, state)) == list(STEPS)
    assert cn.gate(doc, state, protection=STEPS, loosen=LOOSEN) == []
    # the five it does not ask to loosen protect more: a floor and a rise that go up, three screens that
    # come down
    tighter = [n for n in STEPS if n not in LOOSEN]
    assert tighter == ["v_undervolt_counts", "pos_error_time_ticks", "pos_error_counts", "sensor_delta_max",
                       "th_rise2_q016"]
    assert not any(cn.looser(n, state["fields"][n], f[n]["value"]) for n in tighter)
    # a name short of the list is refused, the field named
    assert cn.gate(doc, state, protection=STEPS[1:], loosen=LOOSEN[1:]) == [
        "stall_response: servo Fault (0), set Yield (1), judged, taken as protecting less: name it with "
        "--protection stall_response and --loosen stall_response"]
    assert cn.gate(doc, state, protection=STEPS, loosen=LOOSEN[:-1]) == [
        "ntc_k2_q24: servo 5779, set 19934, judged, taken as protecting less: name it with --loosen ntc_k2_q24"]
    # and the install says the winding model's step as what the estimate reads
    assert cn.winding_step(doc, state["fields"])[-1] == (
        "winding model: the wanted four read at least as hot in every case")


# what each deviation of the session moves over the set of record, as [of record, deviated]
SIDES = {
    "plant-as-held": {
        "i_kp_q88": [289, 275], "i_ki_q412": [1176, 1580], "i_kaw_q412": [2352, 3160], "j_ff_q88": [862, 789],
        "l3_q88": [84, 77], "r_q12": [3744, 4541], "recip_ke_q": [13315, 11817], "b_i_q313": [2434, 2657],
        "fric_fc_counts": [54, 100], "r0_q12": [3725, 0], "ke_uvs_per_rad": [1159, 0]},
    "current-gains-as-held": {"i_kp_q88": [289, 275], "i_ki_q412": [1176, 1580], "i_kaw_q412": [2352, 3160]},
    "ke-as-held": {"recip_ke_q": [13315, 11817]},
    "model-block-as-held": {"b_i_q313": [2434, 2657], "j_ff_q88": [862, 789], "l3_q88": [84, 77]},
    "friction-as-held": {"fric_fc_counts": [54, 100]},
    "r-as-held": {"r_q12": [3744, 4541]},
    "emf-pair-as-held": {"r_q12": [3744, 4541], "recip_ke_q": [13315, 11817]},
    "current-loop-500hz": {"i_kp_q88": [289, 144], "i_ki_q412": [1176, 588], "i_kaw_q412": [2352, 1176]},
    "current-loop-1500hz": {"i_kp_q88": [289, 433], "i_ki_q412": [1176, 1764], "i_kaw_q412": [2352, 3528]},
    "observer-10hz": {"l1_q016": [8640, 5894], "l2_q88": [3180, 1446], "l3_q88": [84, 25]},
    "observer-25hz": {"l1_q016": [8640, 13757], "l2_q88": [3180, 8433], "l3_q88": [84, 371]},
}
# the policy entries a point of a sweep states, and the holds it releases
SWEPT = {
    "current-loop-500hz": ({"current_loop_hz": 500.0}, []), "current-loop-1500hz": ({"current_loop_hz": 1500.0}, []),
    "observer-10hz": ({"observer_hz": 10.0}, []), "observer-25hz": ({"observer_hz": 25.0}, []),
}


@pytest.mark.parametrize("name", SIDES)
def test_a_deviation_of_the_session_is_one_change_of_the_plant_over_the_set_of_record(name):
    doc, _, _ = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    file = json.loads((DEVIATIONS / f"{name}.json").read_text())
    derived, _ = cn.derive(doc, DEVIATIONS / f"{name}.json")
    assert derived["deviation"]["fields"] == SIDES[name] and file["note"]
    assert moved_hashes(doc, derived) == ["plant"] and not set(SIDES[name]) & cn.PROTECTION
    # it moves no protection field: on a servo that holds the set of record its install names nothing, and on
    # one that stores the set from before the releases it names what the set of record itself does
    assert cn.gate(derived, {"fields": {n: e["value"] for n, e in f.items()}}) == []
    before = before_the_releases(doc)[0]
    state = {"fields": {**{n: e["value"] for n, e in before["fields"].items()}, **{n: f[n]["value"]
                                                                                   for n in SHEET_STEP}}}
    assert cn.gate(derived, state) == cn.gate(doc, state) and list(asked(derived, state)) == list(STEPS)
    assert cn.gate(derived, state, protection=STEPS, loosen=LOOSEN) == []
    if name in SWEPT:
        # a point of a sweep: policy entries at another value, through the rules. A bandwidth is the policy's,
        # the pick of the sweep the point is a side of, so what it moves is of class policy
        entries, release = SWEPT[name]
        assert (file.get("policy"), file.get("release", []), file.get("pins")) == (entries, release, None)
        assert all(policy[k] != v for k, v in entries.items())
        picked = all(policy["why"].get(k, "").endswith(SWEEP_PICK) for k in entries)
        assert picked == (not release) and picked != (set(entries) <= set(policy["undecided"]))
        assert all(derived["fields"][n]["class"] == ("policy" if picked else "none")
                   and "held" not in derived["fields"][n] for n in SIDES[name])
        assert all(n in policy["hold"] for n in release)
        return
    # a side as the board held it: every pin the value the reference rung ran on, with the why of the A/B
    # or of the bisect and that rung, and nothing else in the file
    ran = ran_on()
    why = AS_HELD if name in ("plant-as-held", "current-gains-as-held") else BISECT
    assert set(file) == {"note", "pins"} and list(file["pins"]) == list(SIDES[name])
    assert file["pins"] == {n: {"value": ran[n], "why": why, "rung": REFERENCE} for n in SIDES[name]}
    assert all(derived["fields"][n]["class"] == "pinned" for n in SIDES[name])
    whole = list(SIDES["plant-as-held"])
    if name == "plant-as-held":
        # every field of the plant the releases moved, the supply word not among them: the set pins it at what
        # the board held. The limits of motion moved since, and a rung runs at the scorecard's own, which on the
        # set's count are the counts the reference rung ran at, so the plant a rung on it runs is the one the
        # reference rung ran on
        rung = json.loads((results.ROOT / "telemetry" / doc["pair"] / "scorecard" / REFERENCE
                           / "rung.json").read_text())
        ran_set, motion = sc.run_motion(derived, sc.testset())
        assert motion["from"] == "test set" and derived["hash"]["plant"] != rung["set"]["hash"]["plant"]
        assert ran_set["hash"]["plant"] == rung["set"]["hash"]["plant"] == before["hash"]["plant"]
        assert [n for n in f if n not in cn.PROTECTION and ran[n] != f[n]["value"]] == [
            n for n in f if n in (*whole, *MOTION)]
        assert whole == [n for n in f if n in whole] and len(whole) == 11
        # the arms of the bisect and the current gains' side cut it in groups: the resistance, the back-EMF
        # constant, the model block and the friction term each alone, the current loop's three, and the two
        # records a host keeps, which no arm carries
        arms = [tuple(SIDES[arm]) for arm in ("r-as-held", "ke-as-held", "model-block-as-held", "friction-as-held",
                                              "current-gains-as-held")]
        assert sorted(n for arm in arms for n in arm) == sorted(n for n in whole
                                                                if n not in ("r0_q12", "ke_uvs_per_rad"))
        assert sorted(SIDES["emf-pair-as-held"]) == sorted((*SIDES["r-as-held"], *SIDES["ke-as-held"]))
    else:
        # a part of the plant as held, at the same values
        assert set(SIDES[name]) < set(whole) and all(SIDES[name][n] == SIDES["plant-as-held"][n] for n in SIDES[name])
        assert (name != "current-gains-as-held") or tuple(SIDES[name]) == LOOP


def test_a_speed_limit_at_the_ceiling_passes_and_one_over_it_is_refused(monkeypatch, tmp_path):
    doc, _, _ = of_the_bench_pair()
    ceiling = doc["fields"][cn.CEILING]
    files = {speed: json.loads((DEVIATIONS / f"motion-{speed}.json").read_text())["policy"] for speed in (200, 400)}
    # the acceleration limit of each side of the sweep is the speed limit reached in 50 ms
    assert all(entries == {"velocity_limit_deg_s": speed, "accel_limit_deg_s2": speed / 0.05}
               for speed, entries in files.items())
    # the set of record holds the 200 side's limits as its default, under the ceiling
    assert {n: doc["fields"][n]["value"] for n in MOTION} == {"velocity_limit_cps": 4025, "accel_limit_q88": 10305}
    assert doc["fields"]["velocity_limit_cps"]["value"] < ceiling["value"] and cn.over_limit(doc) == []
    # the 400 side's entries over the set of record, its release left out, since the set holds the two no
    # longer: 400 deg/s is the ceiling's speed, but the move speed rounds to the nearest count, 8051, and the
    # ceiling, a derated cap, down to 8050, so the side is one count over it and refused
    read, entries = cn.deviation_of, {}
    monkeypatch.setattr(cn, "deviation_of", lambda path: {
        "note": read(path)["note"], "policy": {**read(path)["policy"], **entries}})
    assert files[400]["velocity_limit_deg_s"] == ceiling["deg_s"] and ceiling["value"] == 8050
    over = ("velocity_limit_cps: {} count/s is over the set's speed_ceiling_cps, 8050 count/s (400 deg/s): a "
            "field stays at or under the limit that bounds it")
    with pytest.raises(ValueError, match=re.escape(over.format(8051))):
        cn.derive(doc, DEVIATIONS / "motion-400.json")
    # a speed at the ceiling's count is not over it
    entries["velocity_limit_deg_s"] = 8050 / doc["scale"]["counts_per_deg"]
    top, _ = cn.derive(doc, DEVIATIONS / "motion-400.json")
    assert top["fields"]["velocity_limit_cps"]["value"] == ceiling["value"] and cn.over_limit(top) == []
    # the same entries a deg/s over the ceiling are refused by their build, and a set that holds such a limit
    # by load
    over = over.format(8071)
    entries["velocity_limit_deg_s"] = 401.0
    with pytest.raises(ValueError, match=re.escape(over)):
        cn.derive(doc, DEVIATIONS / "motion-400.json")
    past = deviated({k: v for k, v in top.items() if k != "deviation"}, velocity_limit_cps=8071)
    assert cn.over_limit(past) == [over]
    path = tmp_path / "over.constants.json"
    path.write_text(cn.render(past))
    with pytest.raises(ValueError, match=re.escape(over)):
        cn.load(path)


# --- save and restore ---

OLD_POINTS = [0] * 120 + [5] + [0] * 135
BOTH = {"protection": NAMED["protection"], "loosen": NAMED["protection"]}


@pytest.fixture
def committed(monkeypatch):
    """`git status` says the set's file is committed as it stands."""
    monkeypatch.setattr(cn, "uncommitted", lambda path: "")


def backup(doc, root):
    return root / "telemetry" / PAIR / "constants" / doc["hash"]["set"][:12]


def stored_live(bench):
    """The fake's flash holding its board defaults under a LIVE table of
    other points than the set's; returns the state it boots with."""
    bench.pages = [postable.page_bytes(OLD_POINTS, p) for p in range(postable.PAGES)]
    bench.set("pos_lut_state", postable.STATES.index("LIVE"))
    bench.run("save")
    bench.reboot()
    bench.log.clear()
    return sc.servo_state()


def held(state):
    return {k: state[k] for k in ("fields", "pos_table", "wide_codes", "data_state")}


def test_save_backs_up_saves_and_reads_back(target, bench, tmp_path, committed):
    boot = sc.servo_state()
    assert boot["pos_table"]["state"] == "IDENTITY"
    bench.set("r_q12", 1)     # volatile: what is recorded is the state a boot gives, not this
    bench.log.clear()
    stored, after = cn.save(PAIR, **NAMED, root=tmp_path)
    folder = backup(target, tmp_path)
    assert (stored, after) == (folder / "stored.json", folder / "after.json")
    assert json.loads(stored.read_text()) == boot and boot["pos_table"]["points"] == bytes(512).hex()
    back = json.loads(after.read_text())
    assert cn.differences(target, back) == [] and back["data_state"] == "clean"
    assert back["pos_table"]["state"] == "LIVE" and back["wide_codes"] == NEW_SLOTS.hex()
    assert str(tmp_path) not in stored.read_text() + after.read_text()
    # torque read, the reboot onto the stored state, the install with its stamp, the SAVE, the reboot
    wrote = bench.wrote()
    assert bench.log[0] == ("get", "torque_enable") and wrote[0] == ("run", "reboot")
    assert wrote[-3:] == [("run", "stamp"), ("run", "save"), ("run", "reboot")] and wrote.count(("run", "save")) == 1
    # what a reboot returns from now on is the set
    bench.set("r_q12", 1)
    bench.reboot()
    assert cn.check(target) == []
    # and the record puts the state the servo had stored back, its identity table with it
    assert cn.restore(stored, PAIR, **BOTH, root=tmp_path)[-1] == "stamp"
    assert held(sc.servo_state()) == held(boot)
    # a second save of the same set keeps its files beside the first, which stand as they were
    first = stored.read_text()
    again = cn.save(PAIR, **NAMED, root=tmp_path)
    assert again == (folder / "2" / "stored.json", folder / "2" / "after.json") and stored.read_text() == first
    assert held(json.loads(again[0].read_text())) == held(back)


def test_save_refuses_a_deviation(cli, bench, tmp_path, monkeypatch):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    cli("adopt", "--from", str(record(tmp_path)))
    path = cn.set_path(PAIR)
    derived = tmp_path / "derived.json"
    cn.write(deviated(cn.load(path), p_kp_q88=5026), derived)
    bench.log.clear()
    with pytest.raises(SystemExit, match="refused: .*derived.json is a derived set: only the pair's set of record "
                                         "is saved"):
        cli("--set", str(derived), "save")
    # nor does a derived set put in the place of the set of record pass: it is not the committed file
    of_record = path.read_text()
    monkeypatch.setattr(cn, "uncommitted", lambda p: f"M {p.name}")
    derived.replace(path)
    with pytest.raises(SystemExit, match=f"refused: .*{PAIR}.constants.json is not committed as it stands "
                                         f"\\(M {PAIR}.constants.json\\)"):
        cli("save")
    # and a set that holds a deviation is never saved, committed or not: the file says what it is
    path.write_text(of_record)
    f = dev_file(tmp_path, "soft", pins={"p_kp_q88": pin(5026)})
    cn.write(cn.derive(cn.load(path), f, tmp_path)[0], path)
    monkeypatch.setattr(cn, "uncommitted", lambda p: "")
    with pytest.raises(SystemExit, match=f"refused: .*{PAIR}.constants.json holds the deviation "
                                         "deviations/soft.json: a deviation is never saved"):
        cli("save")
    assert bench.log == [] and not (tmp_path / "telemetry").exists()


def test_save_refuses_a_stale_set_with_the_bus_untouched_and_then_torque_on(stored, bench, tmp_path, committed):
    bench.set("torque_enable", 1)
    with pytest.raises(settings.Refused, match="torque_enable reads on: a save reboots the servo and needs torque "
                                               "off"):
        cn.save(PAIR, root=tmp_path)
    assert bench.wrote() == []
    bench.log.clear()
    stored["inputs"]["3-servo/ladder"] = {"file": "stages/3-servo/ladder/results/gone.json", "sha1": "0" * 40}
    cn.write(stored, cn.set_path(PAIR, tmp_path))
    with pytest.raises(settings.Refused, match="the set is stale: stale: 3-servo/ladder .* is gone"):
        cn.save(PAIR, root=tmp_path)
    assert bench.log == [] and bench.flash is None and not (tmp_path / "telemetry").exists()


def test_save_refuses_a_protection_change_from_the_stored_state_that_is_not_named(
        target, bench, tmp_path, committed):
    boot = stored_live(bench)
    flash = bench.flash
    with pytest.raises(settings.Refused) as e:
        cn.save(PAIR, root=tmp_path)
    stored = backup(target, tmp_path) / "stored.json"
    said = str(e.value).splitlines()
    assert said[0] == "the protection gate:" and len(said) == 4
    assert said[-1] == (f"nothing was saved: a reboot returns the state the servo has stored, which {stored} "
                        f"records (`uv run python -m oscnb.constants restore --from {stored}` installs, volatile, "
                        "the state the servo had stored)")
    # the stored state recorded, nothing written, nothing saved
    assert json.loads(stored.read_text()) == boot and not stored.with_name("after.json").exists()
    assert bench.wrote() == [("run", "reboot")] and bench.flash is flash
    # named but not as a loosening, it is refused the same way; an install that fails half way leaves the
    # servo rebooted onto what it has stored
    with pytest.raises(settings.Refused, match="stall_response: .*\\nnothing was saved"):
        cn.save(PAIR, protection=NAMED["protection"], root=tmp_path)
    bench.drop = {"r_q12"}
    with pytest.raises(settings.Refused, match="r_q12: wrote 16000, read back .*; rebooted: the servo is back on "
                                               "its saved state\\nnothing was saved"):
        cn.save(PAIR, **NAMED, root=tmp_path)
    assert bench.flash is flash and held(sc.servo_state()) == held(boot)
    # each attempt recorded the stored state in a folder of its own
    assert [json.loads((stored.parent / n / "stored.json").read_text()) for n in ("2", "3")] == [boot, boot]


def test_a_stored_identity_is_not_put_back_on_a_servo_that_lands_an_all_zero_table_live(
        target, bench, tmp_path, committed):
    stored, _ = cn.save(PAIR, **NAMED, root=tmp_path)
    bench.zero_live = True
    with pytest.raises(settings.Refused, match="the servo's firmware lands an all-zero table LIVE, and only a boot "
                                               "returns its table to IDENTITY; rebooted: the servo is back on its "
                                               "saved state"):
        cn.restore(stored, PAIR, **BOTH, root=tmp_path)
    assert cn.check(target) == []


def test_a_save_that_does_not_read_back_says_what_flash_holds_and_what_puts_the_old_state_back(
        target, bench, tmp_path, committed):
    stored_live(bench)
    bench.torn = {"r_q12": 123}
    with pytest.raises(settings.Refused) as e:
        cn.save(PAIR, **NAMED, root=tmp_path)
    stored = backup(target, tmp_path) / "stored.json"
    after = stored.with_name("after.json")
    assert str(e.value) == (
        "after the SAVE and a reboot the servo is not the set (data state clean; r_q12: servo 123, set 16000 "
        f"[none]): the save did not land as verified. Flash now holds the new state, the one {after} records, "
        f"and no longer the old one. `uv run python -m oscnb.constants restore --from {stored}` installs, "
        "volatile, the state the servo had stored; to make that permanent again, adopt that record as the pair's "
        "set of record (`adopt --from`, which writes a pair's first set only: the present set's file out of the "
        "way first), commit it and `save` it")
    assert json.loads(after.read_text())["fields"]["r_q12"] == 123
    # the restore is behind the gate, and writes the table from the record's own points: the pair's table
    # file holds the set's and is not read
    with pytest.raises(settings.Refused, match="the protection gate"):
        cn.restore(stored, PAIR, root=tmp_path)
    cn.set_path(PAIR, tmp_path).with_name(f"{PAIR}.pos_lut.json").unlink()
    wrote = cn.restore(stored, PAIR, **BOTH, root=tmp_path)
    assert wrote[:4] == ["wide-code slots", "raw_min", "raw_max", "position table"] and wrote[-1] == "stamp"
    assert held(sc.servo_state()) == held(json.loads(stored.read_text()))
    # a record that holds only the sha1 of its points cannot give its table back
    old = json.loads(stored.read_text())
    del old["pos_table"]["points"]
    stored.write_text(json.dumps(old))
    bench.reboot()
    with pytest.raises(ValueError, match="holds the sha1 of its table's points and not the points"):
        cn.restore(stored, PAIR, **BOTH, root=tmp_path)


def test_save_and_restore_as_commands(cli, bench, tmp_path, committed, monkeypatch, capsys):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    cli("adopt", "--from", str(record(tmp_path)))
    cli("save")
    out = capsys.readouterr().out.splitlines()[-1]
    folder = next((tmp_path / "telemetry" / PAIR / "constants").iterdir())
    shown = folder.relative_to(tmp_path)
    assert out == (f"saved: after a reboot the servo reads as the set, data state clean; {shown / 'stored.json'} "
                   f"holds the state it had stored before, {shown / 'after.json'} the state it boots with now")
    bench.set("r_q12", 1)
    cli("restore", "--from", str(folder / "stored.json"))
    assert capsys.readouterr().out.splitlines()[-1] == "volatile, not saved: `osc reboot` returns the saved state"
    assert bench.decoded("r_q12") == 17100
    cli("restore", "--from", str(folder / "stored.json"))
    assert capsys.readouterr().out.splitlines()[-1] == "the servo holds the record's state: nothing written"
    # the flags of the gate are the install's
    bench.reboot()
    cn.write(deviated(cn.load(cn.set_path(PAIR)), current_limit_counts=500), cn.set_path(PAIR))
    with pytest.raises(SystemExit, match="refused: the protection gate:"):
        cli("save")
    cli("save", "--protection", "current_limit_counts")
    # the refused attempt recorded the stored state too, so the save's files are the second of that set
    second = backup(cn.load(cn.set_path(PAIR)), tmp_path)
    assert bench.decoded("current_limit_counts") == 500
    assert (second / "stored.json").exists() and (second / "2" / "after.json").exists()


def test_git_says_whether_a_set_file_is_committed(tmp_path):
    # outside a repository git cannot say, and that is a refusal, not a pass
    path = tmp_path / "x.constants.json"
    path.write_text("{}")
    with pytest.raises(settings.Refused, match="git cannot say whether .*x.constants.json is committed"):
        cn.uncommitted(path)

