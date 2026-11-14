"""The constant set of a pair with `osc` stubbed by a fake servo: the
first set adopted from a record of its state, the file's bytes and its
three hashes, what `load` refuses, the set against its input files, the
servo read against the set, and the set installed on it: the order of
the writes, the protection gate, the read-back ahead of the stamp and
the reboot after a refusal."""
import copy
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import pytest

from oscnb import constants as cn
from oscnb import postable, results, scorecard as sc, settings
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
    assert all(e == {"value": state["fields"][n], **ADOPTED, **({"scale": "unknown"} if "scale" in e else {})}
               for n, e in doc["fields"].items())
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
    assert '  "r_q12": {"value": 17100, "class": "adopted", "origin": "servo-state"}' in (s.rstrip(",") for s in lines)
    assert '  "recip_ke_q": {"value": 11817, "class": "adopted", "origin": "servo-state", "scale": "unknown"}' in lines
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
    (lambda d: d.update(format=2), "format 2, this tool reads 1"),
], ids=["a field edited", "the table edited", "a class the format has none of", "no value", "no origin",
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
    doc["hash"] = cn.hashes(doc)
    return cn.load(cn.write(doc, cn.set_path(PAIR, root))), files


def test_status_names_a_moved_input_and_its_fields(tmp_path):
    doc, files = built(tmp_path)
    lines, stale = cn.status(doc, tmp_path)
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
    lines, stale = cn.status(doc, tmp_path)
    assert stale and lines == [
        f"stale: 3-servo/ladder ({files['3-servo/ladder']}) is no longer the file the set read, sha1 "
        f"{sc.file_sha1(ladder)}: recip_ke_q, ke_vpc_q", *rest]
    # a file that is gone is named too, the inputs in name order
    (tmp_path / files["1-board/current-scale"]).unlink()
    lines, stale = cn.status(doc, tmp_path)
    assert stale and lines[:2] == [
        f"stale: 1-board/current-scale ({files['1-board/current-scale']}) is gone: current_limit_counts",
        f"stale: 3-servo/ladder ({files['3-servo/ladder']}) is no longer the file the set read, sha1 "
        f"{sc.file_sha1(ladder)}: recip_ke_q, ke_vpc_q"]
    assert cn.moved(doc, tmp_path)[0][2] is None


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
    monkeypatch.setattr(cn, "status", lambda doc: (["stale: an input"], True))
    with pytest.raises(SystemExit) as e:
        cli("status")
    assert e.value.code == 1 and capsys.readouterr().out.splitlines()[-1] == "stale: an input"


# --- install ---

class Bench(Servo):
    """The fake servo with the writes an install makes: `osc set` by
    name, refused where it would put the low soft limit at or past the
    high one, the table's STORE and COMMIT behind the window, the slots,
    the stamp and the reboot. A write of a CONFIG or CALIB field or a
    COMMIT leaves the data state STAMP_MISMATCH until a stamp. `drop`
    names fields whose write is answered and not taken, `drift` a field
    whose write moves another too."""

    FIELDS = {"model_number": (0x00, 2, "ro", "uint"), "id": (0x10, 1, "rw", "uint"),
              "pos_min_soft_counts": (0x28, 4, "rw", "uint"), "pos_max_soft_counts": (0x2c, 4, "rw", "uint"),
              "p_kp_q88": (0x40, 2, "rw", "uint"), "pos_deadband_counts": (0x42, 2, "rw", "uint"),
              "current_limit_counts": (0x48, 2, "rw", "uint"), "stall_response": (0x4a, 1, "rw", "enum"),
              "drive_polarity": (0x4b, 1, "rw", "bool"), "stall_omega_max_cps": (0x50, 2, "rw", "uint"),
              "r_q12": (0x80, 2, "rw", "uint"), "recip_ke_q": (0x82, 2, "rw", "uint"),
              "plant_stamp": (0x8a, 2, "rw", "uint"), wc.FIELD: (0x91, 16, "rw", "bytes"),
              "tick_hz": (0xa2, 2, "ro", "uint"), "raw_min": (0xa4, 2, "rw", "uint"),
              "raw_max": (0xa6, 2, "rw", "uint"),
              "torque_enable": (0xc0, 1, "rw", "bool"), "pos_lut_page": (0xdc, 1, "rw", "uint"),
              "pos_lut_state": (0xde, 1, "ro", "uint")}
    STORED = {**Servo.STORED, "pos_min_soft_counts": 314, "stall_response": 0, "raw_min": 82, "raw_max": 3792}
    VARIANTS = ("Fault", "Yield")

    def __init__(self):
        self.drop, self.drift = set(), {}
        super().__init__()

    def reboot(self):
        super().reboot()
        self.pages = [bytes(2 * postable.PAGE_POINTS)] * postable.PAGES
        self.set("pos_lut_state", 0)
        self.stamp, self.data = "0x9c45 = live set", "clean"

    def decoded(self, name):
        if self.FIELDS[name][3] != "enum":
            return super().decoded(name)
        v = self.table[self.FIELDS[name][0]]
        return f"{self.VARIANTS[v]} ({v})"

    def get(self, name):
        # `osc get` is read up to the first space, an enum as its name
        v = super().get(name)
        return v.split()[0] if self.FIELDS[name][3] == "enum" else v

    def set_(self, name, value):
        self.log.append(("set", name, value))
        kind = self.FIELDS[name][3]
        v = (self.VARIANTS.index(value) if kind == "enum" else {"on": 1, "off": 0}[value] if kind == "bool"
             else int(value))
        soft = {n: self.decoded(n) for n in ("pos_min_soft_counts", "pos_max_soft_counts")}
        if name in soft and {**soft, name: v}["pos_min_soft_counts"] >= {**soft, name: v}["pos_max_soft_counts"]:
            raise subprocess.CalledProcessError(1, ["osc", "set", name, value])
        if name not in self.drop:
            self.set(name, v)
        if name in self.drift:
            self.set(*self.drift[name])
        self.data, self.stamp = "STAMP_MISMATCH", "0x9c45 != live set 0x1234"

    def write(self, addr, data):
        self.log.append(("write", addr, bytes(data)))
        if addr == self.FIELDS[wc.FIELD][0]:
            self.table[addr:addr + len(data)] = data
            return
        assert addr == postable.WINDOW_ADDR
        page, cmd = data[0], data[1]
        if cmd == postable.STORE:
            self.pages[page] = bytes(data[2:])
            self.set("pos_lut_state", postable.STATES.index("LOADING"))
        elif cmd == postable.COMMIT:
            self.set("pos_lut_state", postable.STATES.index("LIVE"))
            self.data = "STAMP_MISMATCH"
        else:
            self.window = self.pages[page]

    def run(self, *args, **kw):
        self.log.append(("run", *args))
        if args[0] == "stamp":
            self.stamp, self.data = "0x1234 = live set", "clean"
        elif args[0] == "reboot":
            self.reboot()

    def bus_baud(self):
        return 3_000_000

    def stub(self, monkeypatch, regs=dict):
        super().stub(monkeypatch, regs)
        for name in ("set_", "run", "bus_baud"):
            monkeypatch.setattr(sc.osc, name, getattr(self, name))
        monkeypatch.setattr(settings.time, "sleep", lambda s: None)
        return self

    def wrote(self):
        """The calls that change the servo, the FETCH of a page left out."""
        return [c for c in self.log if c[0] in ("set", "run")
                or (c[0] == "write" and not (c[1] == postable.WINDOW_ADDR and c[2][1] == postable.FETCH))]


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
    (lambda doc, root: doc["pos_table"].update(state="IDENTITY", points_sha1=NO_POINTS),
     "the set's table is IDENTITY .* and the servo's LIVE .*: an install writes a LIVE table only"),
    (lambda doc, root: table_file(root).unlink(), "the set's table is LIVE and the pair has no table file"),
    (lambda doc, root: table_file(root, [0] * 256), "holds points of sha1 .*, the set's table is"),
    (lambda doc, root: table_file(root, stops=(82, 3792)),
     "was built against stops 82 and 3792, the set's are 90 and 3780"),
], ids=["torque on", "a stale set", "fields the servo has none of or lets no host write", "an identity table",
        "no table file", "a table file of other points", "a table built against other stops"])
def test_install_refuses_before_anything_is_written(target, bench, tmp_path, monkeypatch, change, match):
    if "torque" in match:
        bench.set("torque_enable", 1)
    if "IDENTITY" in match:
        bench.set("pos_lut_state", postable.STATES.index("LIVE"))
    change(target, tmp_path)
    with pytest.raises(settings.Refused, match=match):
        cn.install(target, **NAMED, root=tmp_path)
    assert bench.wrote() == []


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
