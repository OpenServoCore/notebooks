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
deviation may move, and what each curve reads on the bench board."""
import copy
import dataclasses
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

from oscnb import constants as cn
from oscnb import boards, ntc, postable, results, scorecard as sc, settings
from oscnb import widecodes as wc

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scorecard_synth as synth  # noqa: E402

PAIR = "synth__board"
SLOTS = bytes([0, 0, 0, 88] + [0] * 12)
NO_POINTS = hashlib.sha1(bytes(512)).hexdigest()
PAGE_3 = b"\x01" + bytes(63)
ADOPTED = {"class": "adopted", "origin": "servo-state"}


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
    assert ('  "r_q12": {"value": 17100, "class": "adopted", "origin": "servo-state", "protection": null}'
            in (s.rstrip(",") for s in lines))
    assert ('  "recip_ke_q": {"value": 11817, "class": "adopted", "origin": "servo-state", "protection": null, '
            '"scale": "unknown"}') in lines
    assert ('  "current_limit_counts": {"value": 554, "class": "adopted", "origin": "servo-state", '
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
    (lambda d: d["fields"]["current_limit_counts"].update(protection=None),
     "current_limit_counts says None of protection, the tool {'looser': 'up'}"),
    (lambda d: d["fields"]["r_q12"].update(protection={"looser": "up"}), "r_q12 says .* of protection, the tool None"),
    (lambda d: d["fields"]["r_q12"].pop("protection"), "r_q12 says nothing of protection, the tool None"),
    (lambda d: d.update(deviation={"file": "deviations/x.json", "sha1": "0" * 40, "fields": {"r_q12": [1, 2]}}),
     "its deviation is not the fields it moved, each as \\[of record, deviated\\]"),
    (lambda d: d.update(format=2), "format 2, this tool reads 1"),
], ids=["a field edited", "the table edited", "a class the format has none of", "no value", "no origin",
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
        "5 fields: 3 measured, 1 policy, 0 pinned, 0 held, 1 adopted (no source); 0 per-count at an unknown scale",
        f"position table IDENTITY, points sha1 {NO_POINTS} [adopted]",
        "wide-code slots: none",
        f"hash: plant {doc['hash']['plant'][:12]}, protection {doc['hash']['protection'][:12]} (1 fields), "
        f"set {doc['hash']['set'][:12]}",
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
    assert lines[1] == "8 fields: 0 measured, 0 policy, 0 pinned, 0 held, 8 adopted (no source); " \
                       "3 per-count at an unknown scale"
    assert lines[-1] == (f"adopted from servo-state: the servo's state before the path (record sha1 {o['sha1']}, "
                         f"state hash {o['state_hash'][:12]})")


# --- check ---

def test_check_is_clean_on_the_adopted_servo(adopted, servo, capsys):
    doc, _ = adopted
    servo.log.clear()
    assert cn.check(doc) == []
    h = doc["hash"]
    assert capsys.readouterr().out.splitlines() == [
        "8 fields: 0 measured, 0 policy, 0 pinned, 0 held, 8 adopted (no source); 3 per-count at an unknown scale",
        f"position table IDENTITY, points sha1 {NO_POINTS} [adopted]",
        f"wide-code slots {SLOTS.hex(' ')} [adopted]",
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
        "p_kp_q88: servo 5026, set 40212 [adopted]",
        "current_limit_counts: servo 600, set 554 [adopted]",
        "drive_polarity: servo off, set on [adopted]",
        "position table state: servo LIVE, set IDENTITY [adopted]",
        f"position table points sha1: servo {points}, set {NO_POINTS} [adopted]",
        f"wide-code slots: servo {bytes(16).hex(' ')}, set {SLOTS.hex(' ')} [adopted]",
    ]
    assert cn.check(doc) == named
    assert capsys.readouterr().out.splitlines()[:len(named)] == named


def test_a_difference_names_the_fields_class_and_rule_and_a_field_the_servo_lacks(adopted, servo):
    doc, _ = adopted
    doc["fields"]["r_q12"] = {"value": 4541, "class": "measured", "rule": "r.rails"}
    doc["fields"]["ke_vpc_q"] = {"value": 355, **ADOPTED}
    assert cn.differences(doc, sc.servo_state()) == [
        "r_q12: servo 17100, set 4541 [measured, r.rails]",
        "ke_vpc_q: servo names no such field, set 355 [adopted]",
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
        f"wide-code slots: servo names none, set {SLOTS.hex(' ')} [adopted]"]


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
    assert capsys.readouterr().out.splitlines()[0] == "r_q12: servo 16000, set 17100 [adopted]"


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
    ("ntc_k1_q88", 563, 212, True), ("cutoff_cc", 10000, "9000", True),
])
def test_looser_reads_a_direction_and_takes_what_it_cannot_read_as_less(name, on_servo, wanted, less):
    assert cn.looser(name, on_servo, wanted) is less


def test_every_protection_field_has_one_way_to_protect_less():
    assert cn.PROTECTION == set(cn.LOOSER) and set(cn.LOOSER.values()) == {"up", "down", "judged"}
    assert len(cn.PROTECTION) == 26 and not cn.PROTECTION & {*cn.COMMS, cn.STAMP, wc.FIELD}


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
    assert str(e.value) == ("the state read back is not the set: p_kp_q88: servo 1, set 5026 [adopted]; rebooted: "
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
NO_LIMIT = "current_limit_counts: held, policy.json has no current_limit_ma"
NO_R = f"r_q12: held, {NO_RULE.format('r_q12', *['r'] * 4)}"
# and of the fields in counts of position under a policy with none of their entries: the range, the hold band
# and the stall speed, which the fake servo holds, are held, the rest left out
NO_RANGE = [*(f"pos_{end}_soft_counts: held, policy.json has no soft_margin_share" for end in ("min", "max")),
            "pos_deadband_counts: held, policy.json has no hold_band_noise_multiple"]
NO_STALL = "stall_omega_max_cps: held, policy.json has no stall_omega_max_deg_s"
NO_COUNTS = [f"{n}: left out, policy.json has no {key}" for n, key in (
    ("velocity_limit_cps", "velocity_limit_deg_s"), ("accel_limit_q88", "accel_limit_deg_s2"),
    ("pos_error_counts", "pos_error_deg"), ("sensor_delta_max", "sensor_delta_max_counts"))]
HELD_BLOCK = [*NO_RANGE, NO_LIMIT, NO_STALL, NO_R, f"recip_ke_q: held, {NO_LADDER}"]
NO_BLOCK = [
    f"r_bemf_q12: left out, {NO_RULE.format('r_bemf_q12', *['r_bemf'] * 4)}",
    f"b_i_q313: left out, {NO_LADDER}",
    *(f"{n}: left out, 3-servo/friction has no result for synth__board" for n in ("fric_fc_counts", "fric_fv_q016")),
    *(f"{n}: left out, {NO_LADDER}" for n in ("ke_vpc_q", "j_ff_q88")),
    *(f"{n}: left out, policy.json has no observer_hz" for n in ("l1_q016", "l2_q88", "l3_q88")),
    *NO_COUNTS,
    *(f"{n}: left out, 1-board/ntc-curve has no result for board" for n in ("ntc_t_ref_cc", "ntc_k1_q88",
                                                                           "ntc_k2_q24")),
    *(f"{n}_counts: left out, policy.json has no {n}_ma" for n in ("i_cont", "stall_release", "oc_trip"))]
# the registry with the board's tick rate, and the policy entries the rules of the motor block and of the
# protection fields read
BOARD_REG = {**REG, "board": {"tick_hz": 20_000}}
LIMITS_MA = {"current_limit_ma": 500.0, "i_cont_ma": 250.0, "stall_release_ma": 90.0, "oc_trip_ma": 2200.0}
BLOCK_POLICY = {"observer_hz": 15.0, "rule": {"r_q12": "r.rails_fit", "r_bemf_q12": "r_bemf.small_step"},
                **LIMITS_MA}
# the policy entries the rules of the fields in counts of position read, and the pot's noise at rest
COUNT_POLICY = {"velocity_limit_deg_s": 75.0, "accel_limit_deg_s2": 1500.0, "stall_omega_max_deg_s": 25.0,
                "pos_error_deg": 20.0, "soft_margin_share": 0.0625, "hold_band_noise_multiple": 2.5,
                "hold_band_floor_counts": 4, "sensor_delta_max_counts": 256}
POT_NOISE = {"3-servo/pot-table": {"pot_rest_sd_counts": (1.0, 0.0, 1.5)}}
# what a build of the plant over those inputs says, in the set's order: the fields in counts of position,
# whose entries that policy lacks, and the protection fields the inputs give a value, the one the fake servo
# holds carried with what it would be, the rest, which nothing carries, left to a build that names them
BLOCK_SAID = [
    *NO_RANGE, "current_limit_counts: carried at 554, it would be 500: name it with --protection", NO_STALL,
    *NO_COUNTS,
    *(f"{n}: left out, name it with --protection" for n in (
        "ntc_t_ref_cc", "ntc_k1_q88", "ntc_k2_q24", "i_cont_counts", "stall_release_counts", "oc_trip_counts"))]
PLANT = {
    "1-board/current-scale": {"counts_per_a": (1000.0, 990.0, 1010.0)},
    "1-board/ntc-curve": {"design_t_ref_cc": (2600.0,), "design_k1_q88": (200.0,), "design_k2_q24": (30000.0,)},
    "1-board/terminal-chain": {"terminal_counts_per_v": (250.0,)},
    "3-servo/resistance": {"r_rails_at_measured_e_ohm": (4.0, 3.9, 4.2), "r_term_ohm": (4.1,)},
    "3-servo/steps": {"r_step_a_ohm": (5.0, 4.0, 6.0), "r_takeback_ohm": (4.8,)},
    "3-servo/ladder": {"ke_v_s_per_rad": (0.001,)},
    "3-servo/run-up": {"j_kg_m2": (6e-9, 5e-9, 7e-9)},
    "3-servo/friction": {"firmware_pair": {"pooled": {"coulomb_uNm": (50.0, 40.0, 60.0),
                                                      "viscous_uNm_s_per_rad": (0.05,)}}},
}


def quantities(entry):
    """A result's entries with each (value, lo, hi) as the quantity a
    result file holds."""
    if isinstance(entry, dict):
        return {k: quantities(v) for k, v in entry.items()}
    value, lo, hi = (*entry, None, None)[:3]
    return {"value": value, "lo": lo, "hi": hi, "unit": ""}


def plant_path(root, name):
    stage, experiment = name.split("/")
    return root / f"stages/{stage}/{experiment}/results/{'board' if stage == '1-board' else PAIR}.json"


def plant(root, values=PLANT):
    """The stage 1 and stage 3 results the motor block's rules read,
    under `root`, each naming what it read: nothing."""
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
    angle = {"class": "measured", "rule_rev": 1, "unit": "cdeg", "q": 0, "pos_unit": -1, "scale_points_sha1": sha,
             "protection": None}
    assert f["angle_min_cdeg"] == {"value": 0, "rule": "angle_min.table", **angle,
                                   "inputs": ["policy:angle_cw_stop_deg", "pos_table:theta_at_stops_deg"]}
    assert f["angle_max_cdeg"] == {
        "value": 18000, "rule": "angle_max.table", **angle,
        "inputs": ["policy:angle_cw_stop_deg", "pos_table:theta_at_stops_deg", "pos_table:stops", "scale"]}
    assert f["gear_ratio_centi"] == {
        "value": 30805, "class": "measured", "rule": "gear_ratio.registry", "rule_rev": 1,
        "unit": "motor rev per output rev x100", "q": 0, "pos_unit": 0, "inputs": ["registry:gear_ratio"],
        "protection": None}
    assert (f["gear_class"]["value"], f["gear_class"]["inputs"]) == ("2 (metal)", ["registry:gear_class"])
    # every other field stands as it was adopted, in the previous set's order, the stops among them
    rules = set(cn.RULES)
    assert list(f) == list(first["fields"])
    assert all(f[n] == first["fields"][n] for n in f if n not in rules)
    assert f["raw_min"]["class"] == f["p_kp_q88"]["class"] == "adopted"
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
    assert f["recip_ke_q"] == {"value": 11817, "class": "held", "rule": "recip_ke.ladder", "held": NO_LADDER,
                               "carried_from": first["hash"]["set"], "scale": "unknown", "protection": None}
    # the plant moved and the protection fields did not; the same inputs give the same bytes; the file loads
    # a protection field whose rule has nothing to read is held where it stood too: no value of it moves
    assert f["current_limit_counts"] == {"value": 554, "class": "held", "rule": "current_limit.policy",
                                         "held": "policy.json has no current_limit_ma",
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
        "pos_min_soft_counts: 314 [adopted -> held, soft_min.stops]",
        "pos_max_soft_counts: 3560 [adopted -> held, soft_max.stops]",
        "pos_deadband_counts: 4 [adopted -> held, deadband.pot_noise]",
        "current_limit_counts: 554 [adopted -> held, current_limit.policy]",
        "stall_omega_max_cps: 500 [adopted -> held, stall_omega_max.policy]",
        "r_q12: 17100 [adopted -> held]",
        "recip_ke_q: 11817 [adopted -> held, recip_ke.ladder]",
        "angle_min_cdeg: 0 [adopted -> measured, angle_min.table]",
        "angle_max_cdeg: 18774 -> 18000 [measured, angle_max.table]",
        "gear_ratio_centi: 30805 [adopted -> measured, gear_ratio.registry]",
        "gear_class: 2 (metal) [adopted -> measured, gear_class.registry]",
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
    inputs(root, policy={**BLOCK_POLICY, **COUNT_POLICY, **(policy or {})}, **kw)
    plant(root, {**PLANT, **POT_NOISE})
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
    def angles(**kw):
        c = context(tmp_path, **kw)
        return [cn.evaluate(cn.rule_of(c, n), c)["value"] for n in ("angle_min_cdeg", "angle_max_cdeg")]

    assert angles() == [0, 18000]
    # a tag angle that falls as the count rises: the high stop is the clockwise one
    assert angles(theta=THETA[::-1]) == [18000, 0]
    assert angles(policy={"angle_cw_stop_deg": -90.0}) == [-9000, 9000]


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
    for x in (256.0, -0.01):
        with pytest.raises(ValueError, match="does not fit its u16"):
            cn.encode(r, x)


def test_a_missing_result_holds_the_field_and_says_why(first, tmp_path):
    # a board with no wide-code result and a pair with no table: what the set held stands, and says why
    inputs(tmp_path, slots=None, theta=None)
    doc, notes = built_on(first, tmp_path)
    carried = {"class": "held", "carried_from": first["hash"]["set"]}
    assert doc["wide_codes"] == {"slots": list(SLOTS), **carried, "rule": "slots.wide_codes",
                                 "held": "1-board/wide-codes has no result for board"}
    assert doc["pos_table"] == first["pos_table"] and "scale" not in doc
    no_table = "the pair has no position table: which stop is the clockwise one is not known"
    assert doc["fields"]["angle_max_cdeg"] == {"value": 18774, **carried, "rule": "angle_max.table",
                                               "held": no_table, "protection": None}
    assert doc["fields"]["gear_ratio_centi"]["class"] == "measured"
    assert notes == [
        "wide-code slots: held, 1-board/wide-codes has no result for board",
        "scale: none, 3-servo/pot-table has no result for synth__board", NO_SCALE, *HELD_BLOCK,
        f"angle_min_cdeg: held, {no_table}", f"angle_max_cdeg: held, {no_table}", *NO_BLOCK]
    assert cn.described(doc)[0].startswith("16 fields: 2 measured, 0 policy, 0 pinned, 9 held, 5 adopted")
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
        "value": 18774, "class": "held", "rule": "angle_max.table", "would_be": 18000,
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
        *(n for n in HELD_BLOCK if n != NO_LIMIT), *NO_BLOCK]
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
    assert cn.described(doc)[0].startswith("16 fields: 3 measured, 0 policy, 2 pinned, 7 held, 4 adopted")
    pins.write_text(json.dumps({"p_kp_q88": {"value": 5026}}))
    with pytest.raises(ValueError, match="p_kp_q88 lacks its value, its why or its rung"):
        built_on(first, tmp_path)


def test_the_policy_of_record_holds_what_the_rules_read():
    policy = json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    assert isinstance(policy["angle_cw_stop_deg"], float) and isinstance(policy["hold"], dict)
    assert set(policy["hold"]) <= set(cn.RULES)
    assert isinstance(policy["observer_hz"], float) and all(policy["hold"].values())
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
        "value": 1140.0, "lo": 1138.8, "hi": 1141.2, "unit": "counts/A", "class": "held", "rule": "scales.current",
        "held": "1-board/current-scale has no result for board", "carried_from": again["hash"]["set"]}
    assert "current_counts_per_a: held, 1-board/current-scale has no result for board" in notes
    # an edited scale is not the set the tool wrote
    edited = copy.deepcopy(doc)
    edited["scales"]["current_counts_per_a"]["value"] = 1108.5
    out.write_text(cn.render(edited))
    with pytest.raises(ValueError, match="hashes are not those of its content"):
        cn.load(out)


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
JOINED = ["r_bemf_q12", "b_i_q313", "fric_fc_counts", "fric_fv_q016", "ke_vpc_q", "j_ff_q88", "l1_q016", "l2_q88",
          "l3_q88"]       # the fields of the block the fake servo does not hold, in the rules' order
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
    assert doc["fields"]["r_q12"] == {"value": 17100, "class": "held", "held": NO_RULE.format("r_q12", *["r"] * 4),
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
        "value": 11817, "class": "held", "rule": "recip_ke.ladder", "would_be": would, "held": HOLDS["recip_ke_q"],
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
    # the two the policy holds, and the range, the hold band and the stall speed, whose entries it lacks
    assert cn.classes(doc)["held"] == 6 and cn.classes(doc)["adopted"] == cn.classes(first)["adopted"] - 10


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
    inputs(tmp_path, policy=BLOCK_POLICY)
    plant(tmp_path, {
        "1-board/current-scale": {"counts_per_a": (cpa,)},
        "1-board/terminal-chain": {"terminal_counts_per_v": (cpv,)},
        "3-servo/resistance": {"r_rails_at_measured_e_ohm": (r_vpc * cpa / cpv,)},
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
    for name, (stored, gives, band) in block_of_record.items():
        e = cn.evaluate(cn.rule_of(c, name), c)
        assert (e["value"], e.get("band")) == (gives, band), name
        # the set of record binds the two fields whose release was benched and kept at what their rules give,
        # holds every other field its rule would move at the servo's value, and binds the two it would not
        if name in ("fric_fv_q016", "ke_vpc_q"):
            assert (f[name]["value"], f[name]["class"], f[name].get("would_be")) == (gives, "measured", None), name
            continue
        want = {"class": "held", "would_be": gives} if gives != stored else {"class": "policy", "would_be": None}
        assert f[name]["value"] == stored and {k: f[name].get(k) for k in want} == want, name
    # the resistance each way it is on record; the back-EMF boxcar's own takes the small step's, on no image yet
    ways = {r.name: cn.evaluate(r, c) for r in cn.RULES["r_bemf_q12"].values()}
    assert {n: e["value"] for n, e in ways.items()} == {
        "r_bemf.rails_fit": 3744, "r_bemf.locked_laps": 3781, "r_bemf.small_step": 4799, "r_bemf.take_back": 4557}
    assert cn.rule_of(c, "r_bemf_q12").name == "r_bemf.small_step" and "r_bemf_q12" not in f
    assert ways["r_bemf.small_step"]["band"] == [4036, 5562]


# --- the protection rules: the NTC curve and the current limits ---

CURVE = ("ntc_t_ref_cc", "ntc_k1_q88", "ntc_k2_q24")        # the words of the curve a rule binds
LIMITS = ("current_limit_counts", "i_cont_counts", "stall_release_counts", "oc_trip_counts")


def test_the_protection_rules_bind_in_a_build_that_names_them(first, tmp_path):
    plain, _ = block(first, tmp_path)
    # a build of the plant leaves them: the one the servo holds carried with what it would be
    assert plain["fields"]["current_limit_counts"] == {**first["fields"]["current_limit_counts"], "would_be": 500}
    assert not any(n in plain["fields"] for n in (*CURVE, *LIMITS[1:]))
    doc, _ = cn.build(PAIR, plain, protection=[*CURVE, *LIMITS], root=tmp_path, registry=BOARD_REG)
    f = doc["fields"]
    assert moved_hashes(plain, doc) == ["protection"]
    # a limit: the policy's mA through the board's measured counts per A, its band the scale's
    assert f["current_limit_counts"] == {
        "value": 500, "class": "policy", "rule": "current_limit.policy", "rule_rev": 1, "unit": "ccounts", "q": 0,
        "pos_unit": 0, "inputs": ["policy:current_limit_ma", "1-board/current-scale:counts_per_a"],
        "band": [495, 505], "protection": {"looser": "up"}}
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
    # the reference count has no rule: the result names none for its design curve
    assert "ntc_raw_ref" not in cn.RULES and cn.load(cn.write(doc, tmp_path / "x.constants.json")) == doc
    # a scale that moves moves the limits with it, and the set that stood is stale by the fields that read it
    plant(tmp_path, {"1-board/current-scale": {"counts_per_a": (1100.0,)}})
    lines, stale = cn.status(doc, tmp_path, BOARD_REG)
    assert stale and all(n in lines[0] for n in LIMITS)
    with pytest.raises(ValueError, match="what the set is scored with would move: build the plant first"):
        cn.build(PAIR, doc, protection=list(LIMITS), root=tmp_path, registry=BOARD_REG)
    moved, _ = cn.build(PAIR, cn.build(PAIR, doc, root=tmp_path, registry=BOARD_REG)[0], protection=list(LIMITS),
                        root=tmp_path, registry=BOARD_REG)
    assert [moved["fields"][n]["value"] for n in LIMITS] == [550, 275, 99, 2420]


def test_a_held_protection_field_is_held_by_a_build_of_the_plant_and_moves_no_hash(first, tmp_path):
    why = "it moves on a protection step of its own"
    doc, notes = block(first, tmp_path, hold={"current_limit_counts": why, "ntc_k1_q88": why})
    assert doc["fields"]["current_limit_counts"] == {
        "value": 554, "class": "held", "rule": "current_limit.policy", "would_be": 500, "held": why,
        "carried_from": first["hash"]["set"], "protection": {"looser": "up"},
        "inputs": ["policy:current_limit_ma", "1-board/current-scale:counts_per_a"]}
    assert f"current_limit_counts: held, {why}; its rule gives 500" in notes
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
    inputs(tmp_path, policy={**BLOCK_POLICY, "current_limit_ma": 512.0, "i_cont_ma": 256.0, "stall_release_ma": 92.0,
                             "oc_trip_ma": 2250.0, "hold": dict.fromkeys((*CURVE, *LIMITS), why)})
    plant(tmp_path, {"1-board/ntc-curve": {"design_t_ref_cc": (2584.0,), "design_k1_q88": (212.0,),
                                           "design_k2_q24": (28463.0,)}})
    doc, _ = cn.build(PAIR, bound, root=tmp_path, registry=BOARD_REG)
    cn.write(doc, cn.set_path(PAIR, tmp_path))
    assert doc["hash"] == bound["hash"] and all(doc["fields"][n]["class"] == "held" for n in (*CURVE, *LIMITS))
    return doc


def test_a_deviation_moves_the_protection_fields_of_one_step_and_releases_its_holds_together(steps, tmp_path):
    def derived(name, **doc):
        return cn.derive(steps, dev_file(tmp_path, name, **doc), tmp_path, BOARD_REG)[0]

    # the curve's three words are one curve, and the limits one scale: each group is released whole
    curve = derived("curve", note="the curve of record", release=list(CURVE))
    assert curve["deviation"]["fields"] == {"ntc_t_ref_cc": [2600, 2584], "ntc_k1_q88": [200, 212],
                                            "ntc_k2_q24": [30000, 28463]}
    limits = derived("limits", release=list(LIMITS))
    assert limits["deviation"]["fields"] == {"current_limit_counts": [500, 512], "i_cont_counts": [250, 256],
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
    assert lone["deviation"]["fields"] == {"current_limit_counts": [500, 400]}
    assert lone["fields"]["i_cont_counts"] == steps["fields"]["i_cont_counts"]
    # what the install of each step has to be told, against a servo that holds the set of record
    state = {"fields": {n: e["value"] for n, e in steps["fields"].items()}}
    assert cn.gate(limits, state) == [
        "current_limit_counts: servo 500, set 512, protects less (less is up): name it with --protection "
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


def test_the_bench_pairs_current_limits_are_the_ma_they_were_typed_for_at_the_measured_scale():
    doc, c, board = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    nominal, measured = board.counts_per_a, doc["scales"][cn.CURRENT]["value"]
    # field: (what the servo held before the set, the policy's entry, what the rule gives)
    limits = {"current_limit_counts": (554, "current_limit_ma", 568), "i_cont_counts": (277, "i_cont_ma", 284),
              "stall_release_counts": (100, "stall_release_ma", 102), "oc_trip_counts": (2439, "oc_trip_ma", 2497)}
    assert tuple(limits) == LIMITS == cn.GROUPS["current scale"]
    for name, (stored, key, gives) in limits.items():
        # the mA the stored counts were typed for: the one whole mA the board's nominal parts take to them
        typed = [ma for ma in range(1, 4000) if raw(ma * nominal / 1000) == stored]
        assert typed == [policy[key]], name
        e = cn.evaluate(cn.rule_of(c, name), c)
        assert e["value"] == gives == raw(policy[key] * measured / 1000), name
        # bound: the set holds each at the policy's mA through the measured scale
        assert (f[name]["value"], f[name]["class"], f[name].get("would_be")) == (gives, "policy", None), name
        # more counts on the same mA: every one of them moved the way that protects less, or is judged
        assert gives > stored and cn.LOOSER[name] in ("up", "judged"), name
    # what the servo's earlier counts were at the measured scale, and how far the rule's are over them, mA
    now = {n: round(1000 * stored / measured, 1) for n, (stored, _, _) in limits.items()}
    assert now == {"current_limit_counts": 488.1, "i_cont_counts": 244.0, "stall_release_counts": 88.1,
                   "oc_trip_counts": 2148.9}
    more = {n: round(1000 * (gives - stored) / measured, 1) for n, (stored, _, gives) in limits.items()}
    assert more == {"current_limit_counts": 12.3, "i_cont_counts": 6.2, "stall_release_counts": 1.8,
                    "oc_trip_counts": 51.1}
    # the collision trip is off on the servo, no current, and has no rule
    assert f["stall_tau_trip_counts"]["value"] == 65535 and "stall_tau_trip_counts" not in cn.RULES


def test_what_the_stored_curve_and_the_curve_of_record_read_on_the_bench_board():
    doc, c, board = of_the_bench_pair()
    f = doc["fields"]
    assert cn.GROUPS["ntc curve"] == ("ntc_raw_ref", *CURVE)
    stored = tuple(f[n]["value"] for n in cn.GROUPS["ntc curve"])
    record = (stored[0], *(cn.evaluate(cn.rule_of(c, n), c)["value"] for n in CURVE))
    assert stored == (2048, 2500, 563, 5779) and record == (2048, 2584, 212, 28463)
    assert all((f[n]["class"], f[n]["would_be"]) == ("held", v) for n, v in zip(CURVE, record[1:]))
    assert f["ntc_raw_ref"]["class"] == "adopted"
    # the temperature each reports with the board at a true one, the part's nominal equation as the truth
    reads = {t: (ntc.reads_c(board, stored, t), ntc.reads_c(board, record, t)) for t in (25, 40, 60, 80, 100)}
    assert reads == {25: (25.0, 25.84), 40: (40.19, 37.75), 60: (57.31, 61.79), 80: (68.53, 81.64),
                     100: (75.08, 94.39)}
    # the true temperature at which each reaches the derate and the cutoff as the set holds them
    derate, cutoff = f["derate_start_cc"]["value"], f["cutoff_cc"]["value"]
    assert (derate, cutoff) == (8000, 10000)
    reaches = [[ntc.reaches_c(board, curve, at) for at in (derate, cutoff)] for curve in (stored, record)]
    assert reaches == [[pytest.approx(130.3, abs=0.05), None],
                       [pytest.approx(78.0, abs=0.05), pytest.approx(113.8, abs=0.05)]]
    # the stored curve tops out under the cutoff: short of the rail check no board temperature reads it
    counts = range(ntc.RAIL_COUNTS + 1, ntc.ADC_FULL - 1 - ntc.RAIL_COUNTS)
    assert max(ntc.ntc_cc(r, *stored) for r in counts) == 8416
    assert float(board.ntc_c_beta(ntc.RAIL_COUNTS)) == pytest.approx(290.2, abs=0.05)
    # where the curve of record reads under the stored one, which is where it protects less
    under = [r for r in counts if ntc.ntc_cc(r, *record) < ntc.ntc_cc(r, *stored)]
    assert [round(float(board.ntc_c_beta(r)), 1) for r in (under[-1], under[0])] == [26.5, 49.4]
    worst = min(under, key=lambda r: ntc.ntc_cc(r, *record) - ntc.ntc_cc(r, *stored))
    assert (ntc.ntc_cc(worst, *record) - ntc.ntc_cc(worst, *stored), round(float(board.ntc_c_beta(worst)), 1)) == (
        -264, 37.4)
    # and the cold side of the quadratic: it turns at a true 20 C and reads warmer as the board cools below it
    lowest = min(counts, key=lambda r: ntc.ntc_cc(r, *record))
    assert (ntc.ntc_cc(lowest, *record), round(float(board.ntc_c_beta(lowest)), 1)) == (2482, 20.2)
    assert (ntc.reads_c(board, record, 0), ntc.reads_c(board, stored, 0)) == (37.51, 4.84)


def test_the_protection_steps_of_record_name_what_their_install_needs():
    doc, _, _ = of_the_bench_pair()
    state = {"fields": {n: e["value"] for n, e in doc["fields"].items()}}
    folder = results.ROOT / "stages" / sc.STAGE / "scorecard" / "deviations"
    for name, fields in (("protection-ntc-curve", CURVE),):
        derived, _ = cn.derive(doc, folder / f"{name}.json")
        assert tuple(derived["deviation"]["fields"]) == fields, name
        assert [k for k in ("plant", "protection") if derived["hash"][k] != doc["hash"][k]] == ["protection"]
        # every field of the step is named as a protection change and to loosen: it is judged, or it protects less
        asked = cn.gate(derived, state)
        assert [line.split(":")[0] for line in asked] == list(fields), name
        assert all(line.endswith(f"name it with --protection {n} and --loosen {n}") for n, line in zip(fields, asked))
        assert len(cn.gate(derived, state, protection=fields)) == len(fields)
        assert cn.gate(derived, state, protection=fields, loosen=fields) == []


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


def test_the_bench_pairs_fields_in_counts_are_held_beside_what_they_were_typed_for():
    doc, c, board = of_the_bench_pair()
    f, policy = doc["fields"], json.loads((results.ROOT / cn.FOLDER / cn.POLICY).read_text())
    typed_at = results.read_file(results.path("3-servo", "pot-table", doc["pair"]))["pot_scale_counts_per_deg"]["value"]
    table, medium = doc["scale"]["counts_per_deg"], board.tick_hz / cn.DECIM_MED
    assert (round(typed_at, 3), round(table, 3), round(table / typed_at, 4)) == (18.913, 20.127, 1.0642)
    # field: (what the servo holds, in its unit per count of position, the policy's entry, what the rule gives)
    held = {"velocity_limit_cps": (1500, 1500, "velocity_limit_deg_s", 1596),
            "accel_limit_q88": (3840, 3840 / 256 * medium, "accel_limit_deg_s2", 4086),
            "stall_omega_max_cps": (500, 500, "stall_omega_max_deg_s", 532),
            "pos_error_counts": (400, 400, "pos_error_deg", 426)}
    for name, (stored, counts, key, gives) in held.items():
        # the policy is what the stored counts were typed for at the raw pot's mid travel scale, four figures
        assert policy[key] == float(f"{counts / typed_at:.4g}"), name
        assert count_rule(c, name)["value"] == gives, name
        assert (f[name]["value"], f[name]["class"], f[name]["would_be"]) == (stored, "held", gives), name
        assert f[name]["scale"] == "unknown" and name in policy["hold"], name
    assert {n: policy[key] for n, (_, _, key, _) in held.items()} == {
        "velocity_limit_cps": 79.31, "accel_limit_q88": 1586.0, "stall_omega_max_cps": 26.44,
        "pos_error_counts": 21.15}
    # and what the stored counts are on the table's count: 6.4 % less of each
    assert {n: float(f"{counts / table:.4g}") for n, (_, counts, _, _) in held.items()} == {
        "velocity_limit_cps": 74.53, "accel_limit_q88": 1491.0, "stall_omega_max_cps": 24.84,
        "pos_error_counts": 19.87}
    # the range is what it was typed as, a sixteenth of the stops' span inside each stop, and binds as it stands
    lo, hi = (f[n]["value"] for n in cn.STOPS)
    assert policy["soft_margin_share"] == 1 / 16
    assert [(f[n]["value"], f[n]["class"]) for n in RANGE] == [(raw(lo + (hi - lo) / 16), "policy"),
                                                               (raw(hi - (hi - lo) / 16), "policy")]
    assert [f[n]["value"] for n in RANGE] == [314, 3560]
    # on the raw pot those counts sat nearer the stops than on the table, deg inside the low and the high one
    inside = [(float(c.table.lin(314)) - lo) / table, (hi - float(c.table.lin(3560))) / table]
    assert [round(x, 2) for x in inside] == [10.4, 10.86] and round((314 - lo) / table, 2) == 11.53
    # the band on its floor over the pot's noise, and the sensor screen, in the pot's raw counts: bound as held
    noise = results.read_file(results.path("3-servo", "pot-table", doc["pair"]))["pot_rest_sd_counts"]
    assert max(policy["hold_band_noise_multiple"] * noise[k] for k in ("value", "lo", "hi")) < 4
    assert [(f[n]["value"], f[n]["class"], f[n]["rule"]) for n in ("pos_deadband_counts", "sensor_delta_max")] == [
        (4, "policy", "deadband.pot_noise"), (256, "policy", "sensor_delta.policy")]
    # what is left with no source at a scale nothing recorded: the velocity loop's two gains
    assert [n for n, e in f.items() if e["class"] == "adopted" and e.get("scale") == "unknown"] == [
        "v_kp_q88", "v_ki_q412"]
    assert cn.described(doc)[0] == ("63 fields: 6 measured, 10 policy, 0 pinned, 13 held, 34 adopted (no source); "
                                    "10 per-count at an unknown scale")


def test_the_steps_of_the_fields_in_counts_name_what_their_install_needs():
    doc, _, _ = of_the_bench_pair()
    state = {"fields": {n: e["value"] for n, e in doc["fields"].items()}}
    folder = results.ROOT / "stages" / sc.STAGE / "scorecard" / "deviations"
    # the limits of motion are of the plant: a rung's own step, no gate
    motion, _ = cn.derive(doc, folder / "motion-limits.json")
    assert motion["deviation"]["fields"] == {"velocity_limit_cps": [1500, 1596], "accel_limit_q88": [3840, 4086]}
    assert moved_hashes(doc, motion) == ["plant"] and cn.gate(motion, state) == []
    # the position-error screen trips later on more counts: a protection step that protects less
    error, _ = cn.derive(doc, folder / "protection-position-error.json")
    assert error["deviation"]["fields"] == {"pos_error_counts": [400, 426]} and moved_hashes(doc, error) == [
        "protection"]
    assert cn.gate(error, state, protection=["pos_error_counts"]) == [
        "pos_error_counts: servo 400, set 426, protects less (less is up): name it with --loosen pos_error_counts"]
    assert cn.gate(error, state, protection=["pos_error_counts"], loosen=["pos_error_counts"]) == []
    # the stall speed takes a press as a stall up to a higher count: a protection step that protects more
    stall, _ = cn.derive(doc, folder / "protection-stall-speed.json")
    assert stall["deviation"]["fields"] == {"stall_omega_max_cps": [500, 532]} and moved_hashes(doc, stall) == [
        "protection"]
    assert len(cn.gate(stall, state)) == 1 and cn.gate(stall, state, protection=["stall_omega_max_cps"]) == []
    # each is a step alone: the two screens do not move in one deviation
    assert [cn.group_of(n) for n in SCREENS] == [(n, (n,)) for n in SCREENS]


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
        f"[adopted]): the save did not land as verified. Flash now holds the new state, the one {after} records, "
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

