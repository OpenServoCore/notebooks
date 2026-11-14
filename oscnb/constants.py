"""The constant set of a servo and board pair: one generated file that
says what the servo is to hold, and the verbs that read against it.

stages/5-closed-loop/calibration/<pair>.constants.json holds every
CONFIG and CALIB field a host may write past the bus front, each with
its value as `osc dump` prints it, its class and where the value came
from, with the ADC wide-code slots, the position table's state and the
sha1 of its points, and three hashes. It is generated, never typed: the
same inputs give the same bytes, and `load` refuses a file whose hashes
are not those of its content.

A field has one class. `measured` is a rule over results of record,
`policy` a rule over a general choice, `pinned` a literal typed for the
pair, `held` a value carried while its rule may not bind, and `adopted`
a value the servo held before its set existed, which has no source. The
count of adopted fields is the count with no source, and every verb
prints it. An adopted field that carries a position unit (PER_COUNT)
was fitted at a scale nothing recorded, so it is marked as at an unknown
scale and counted too.

`hash.protection` covers the values of the protection fields
(PROTECTION), `hash.plant` every other field's value with the table's
state and points and the slots, and `hash.set` the two, so a change of
a limit and a change of the plant never share a hash. A field's entry
says which it is under `protection`: null for the plant, the way the
value moves to protect less for a protection field, as LOOSER has it.

    uv run python -m oscnb.constants adopt --from SERVO_JSON [--origin TEXT]
    uv run python -m oscnb.constants build [--out PATH] [--protection NAME ...]
    uv run python -m oscnb.constants status
    uv run python -m oscnb.constants check
    uv run python -m oscnb.constants install [--protection NAME ...] [--loosen NAME ...]
    uv run python -m oscnb.constants save [--protection NAME ...] [--loosen NAME ...]
    uv run python -m oscnb.constants restore --from STATE_JSON [--protection NAME ...] [--loosen NAME ...]

`status`, `check` and `install` read the pair's set of record, or the
derived set named with `--set PATH` ahead of the verb.

`adopt` writes the first set of a servo that already runs, every field
`adopted`, from a record of its state (`oscnb.scorecard.servo_state`,
the `servo.json` of a scorecard run). The record is not kept: the set
names its sha1, the hash of the state and where it came from in words.

`build` is offline and writes the set from the results of record, on
top of the pair's set of record, which it replaces, or to `--out` as a
derived set. A field is bound by a rule (`rule`, RULES), one function a
field, that reads only through a Context: a result's quantity, the
registry, the policy file, the scale of the count. Each read is
recorded, so a field's `inputs` are what its rule read, and the set's
`inputs` name every file by its sha1. One encoder rounds a rule's value
to the field's Q format and refuses one that does not fit; it never
clamps. A rule's band is the rule again at each end of every banded
input. The order is the one a count is defined in: the slots from the
board's wide-code result, then the table from the pair's table file,
which must be built against the set's stops and fitted on the set's
slots, then the scale (the table's counts per deg, the raw pot's on a
pair with no table), then the fields, a per-count one naming the table
it was computed for. A field with no rule is carried as it stands.
`policy.json`, beside the sets and one for every pair, holds the
choices that are no measurement, in physical units, and the fields it
holds back with the reason; `<pair>.pins.json` the literals typed for
one pair, each with its why and the rung that chose it. A rule whose
input is missing does not guess: the field is `held` at the value the
set had, the missing input named, or left out and said where there is
nothing to carry. A result that names a file it read which has moved
since is stale, and refused; one written before results named what
they read is said, not refused. One build moves one hash: the plant by
default, every protection field carried with what its rule would give
beside it (`would_be`), or with `--protection` the fields named and
nothing of the plant.

`status` is offline: the sha1 of every input file against the file now,
and the registry values read against the registry, a moved one named
with the fields that list it. `check` reads the servo
and changes nothing it holds, its only writes the FETCH that brings a
page of the table into the window (protocol sec 5.7): each field of the
set by name, the table and the slots against the servo's, a line a
difference, exit 1 when there is one, and refused when the servo's data
state is not clean.

`install` makes the servo equal the set, volatile, and writes only what
differs. Torque must be off. It is refused before anything is written
when an input file of the set moved, when the servo's descriptor lets a
host write no field the set holds, when the table cannot be installed,
or by the protection gate: a protection field the install would change
has to be named with `--protection`, and named again with `--loosen`
when the set's value protects less than the servo's (LOOSER; a field
no one direction says it for is always taken as protecting less). Then,
in the order a count is defined: the slots, the stops, the table
(`postable`'s own steps, every page read back; an IDENTITY table is
written as all-zero points, whose COMMIT must land IDENTITY, and a
servo whose firmware lands it LIVE is refused as that), the other
fields in table order, each read back, a write the servo refuses tried
once more after the rest. The whole state is then read back, and only
when it is the set does `osc stamp` bless it, no save, after which the
data state must read clean. A refusal after the first write reboots the
servo, which returns its saved state.

`save` is a verb of its own, never a flag of `install`, and saves the
pair's set of record alone. Before the bus is touched it refuses a
derived set, a set with a moved input and a set file that is not
committed as it stands (a read only `git status` of that file); then
torque must be off. It reboots the servo, waits for the bus and writes
the state the servo has stored, its table's points in it, to
`telemetry/<pair>/constants/<set hash, 12 characters>/stored.json`
(a save of the same set after the first keeps its files in a numbered
folder inside that one). It installs the set from there, the same
install behind the same gate, `--protection` and `--loosen` passed
through, and the `check` after it must be clean. Then MGMT SAVE (`osc
save`, protocol sec 9.4), a reboot, the wait, the state to `after.json`
and its comparison with the set. A SAVE keeps CONFIG and CALIB whole,
the slots, the stamp and the table's points with them, so it also keeps
what the set does not hold of those regions, the bus id and rate among
it. A refusal before the SAVE leaves flash as it was. A servo that does
not come back as the set is reported as that: flash then holds the new
state, `restore --from stored.json` installs the old one, volatile, and
the old one is made permanent again by adopting that record as the
pair's set of record and saving it. `restore` puts a recorded state
back, volatile: the install of the set adopted from the record, the
table from the record's own points, behind the same protection gate.
"""
import argparse
import copy
import dataclasses
import hashlib
import json
import math
import re
import subprocess
from pathlib import Path

from . import boards, osc, postable, results, servos, settings, widecodes
from . import scorecard as sc

FORMAT = 1
CLASSES = ("measured", "policy", "pinned", "held", "adopted")
FOLDER = postable.OUT_DIR.relative_to(results.ROOT)

# What no set holds, by the descriptor's names: the comms fields of the
# bus front (protocol sec 5), which say where a servo answers and not
# what it is, and the stamp, which the servo computes over the rest.
COMMS = ("id", "baud_rate_idx", "response_deadline_us")
STAMP = "plant_stamp"

# The fields whose change is a change of what protects the motor, the
# train and the rail, by the descriptor's names, each with the way its
# value moves to protect less: "up", "down", or "judged" where no one
# direction says it. Read off the kernel's use of each field (core
# kernel/phases/loops/limits.rs and faults.rs, kernel/phases/rail.rs,
# slow.rs and fast.rs, estimator/thermal.rs).
LOOSER = {
    # the cap of the current band; the collision trip over the disturbance torque
    "current_limit_counts": "up", "stall_tau_trip_counts": "up",
    # the press arms over the continuous current and folds to it, and 0 reads as the cap; it lets go under the release
    "i_cont_counts": "judged", "stall_release_counts": "up",
    # fold or fault, neither the tighter one; the time a press takes; the speed a press is stalled under
    "stall_response": "judged", "stall_time_ticks": "up", "stall_omega_max_cps": "down",
    "oc_trip_counts": "up", "oc_trip_ticks": "up",
    "duty_max_q15": "up",
    "derate_start_cc": "up", "cutoff_cc": "up",
    # the fault under the floor; the slope of the fold over it, where 0 turns the fold off
    "v_undervolt_counts": "down", "rail_fold_q8": "judged",
    "pos_error_counts": "up", "pos_error_time_ticks": "up",
    "sensor_delta_max": "up", "sensor_bad_count": "up",
    # one curve of four words: a coefficient moves the reading up on one side of the reference, down on the other
    "ntc_raw_ref": "judged", "ntc_t_ref_cc": "judged", "ntc_k1_q88": "judged", "ntc_k2_q24": "judged",
    # a longer leak heats later and cools later, and a body time of 0 drops the body node; a rise adds to the estimate
    "thermal_tau_s": "judged", "thermal_tau2_s": "judged", "th_rise_q016": "down", "th_rise2_q016": "down",
}
PROTECTION = frozenset(LOOSER)

# The fields whose value is per position count, or a position in counts:
# a table that changes what a count means changes what they mean.
PER_COUNT = frozenset((
    "pos_min_soft_counts", "pos_max_soft_counts", "v_kp_q88", "v_ki_q412", "j_ff_q88", "velocity_limit_cps",
    "accel_limit_q88", "stall_omega_max_cps", "pos_error_counts", "l3_q88", "recip_ke_q", "b_i_q313",
    "fric_fv_q016", "ke_vpc_q",
))

ORIGIN = "servo-state"      # the origin an adopted value names
BEFORE = "the servo's state before the path"
TOP = ("format", "pair", "inputs", "origins", "wide_codes", "pos_table", "scale", "fields", "hash", "previous")
BLOCKS = ("inputs", "origins", "fields")    # written an entry a line
LEAD = ("value", "slots", "state", "kind", "counts_per_deg", "points_sha1", "class")
TABLE_STATE, TABLE_POINTS, SLOTS = "position table state", "position table points sha1", "wide-code slots"
STOPS = ("raw_min", "raw_max")      # the stops a table is taken against, written ahead of it


def set_path(pair, root=results.ROOT):
    return Path(root) / FOLDER / f"{pair}.constants.json"


# --- the file ---

def hashed(doc):
    """What `hash.plant` and `hash.protection` of a set cover: a field's
    value goes to protection when PROTECTION names it and to the plant
    otherwise, the table and the slots with the plant."""
    fields = doc["fields"]
    slots = doc["wide_codes"]
    plant = {"fields": {n: e["value"] for n, e in fields.items() if n not in PROTECTION},
             "pos_table": {k: doc["pos_table"][k] for k in ("state", "points_sha1")},
             "wide_codes": None if slots is None else slots["slots"]}
    return plant, {n: e["value"] for n, e in fields.items() if n in PROTECTION}


def protection_of(name):
    """What a field's entry says of protection: None for a field of the
    plant, the way its value moves to protect less for a protection
    field. LOOSER is the source; the file repeats it to be read."""
    return {"looser": LOOSER[name]} if name in PROTECTION else None


def hashes(doc):
    plant, protection = (sc.config_hash(x) for x in hashed(doc))
    return {"plant": plant, "protection": protection,
            "set": sc.config_hash({"plant": plant, "protection": protection})}


def entry(e):
    """One entry on a line: what it holds first, then its class, the
    rest by name."""
    keys = [k for k in LEAD if k in e] + sorted(k for k in e if k not in LEAD)
    return json.dumps({k: e[k] for k in keys}, allow_nan=False)


def render(doc):
    """The set's file, byte for byte: the keys in one order whatever
    order `doc` was put together in, the fields in the table's."""
    extra = [k for k in doc if k not in TOP]
    if extra:
        raise ValueError(f"format {FORMAT} holds no {', '.join(extra)}")
    lines = []
    for key in (k for k in TOP if k in doc):
        v = doc[key]
        if key in BLOCKS and v:
            names = v if key == "fields" else sorted(v)
            body = ",\n".join(f"  {json.dumps(n)}: {entry(v[n])}" for n in names)
            lines.append(f' "{key}": {{\n{body}\n }}')
        else:
            lines.append(f' "{key}": {entry(v) if isinstance(v, dict) else json.dumps(v)}')
    return "{\n" + ",\n".join(lines) + "\n}\n"


def write(doc, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render(doc))
    return path


def load(path):
    """A set file's document, refused when it is not a set as the tool
    writes one: another format, a field without a value or one class, an
    adopted value whose origin the file does not hold, a field that
    says another thing of protection than LOOSER, a measured or policy
    field computed for another table than the set's, or hashes that are
    not those of its content."""
    path = Path(path)
    where = sc.shown(path)
    doc = json.loads(path.read_text())
    if doc.get("format") != FORMAT:
        raise ValueError(f"{where}: format {doc.get('format')}, this tool reads {FORMAT}")
    for name, e in doc["fields"].items():
        if "value" not in e or e.get("class") not in CLASSES:
            raise ValueError(f"{where}: {name} holds no value of one class of {', '.join(CLASSES)}")
    for name, e in (*doc["fields"].items(), (SLOTS, doc["wide_codes"] or {}), ("position table", doc["pos_table"])):
        if e.get("class") == "adopted" and e.get("origin") not in doc["origins"]:
            raise ValueError(f"{where}: {name} is adopted from {e.get('origin')!r}, which the file does not hold")
    scale, table = doc.get("scale") or {}, doc["pos_table"]["points_sha1"]
    if scale.get("kind") == "table" and scale["points_sha1"] != table:
        raise ValueError(f"{where}: its scale is that of the table of points sha1 {scale['points_sha1']}, its "
                         f"table is {table}")
    for name, e in doc["fields"].items():
        if "protection" not in e or e["protection"] != protection_of(name):
            raise ValueError(f"{where}: {name} says {e.get('protection', 'nothing')} of protection, the tool "
                             f"{protection_of(name)}")
        if e["class"] in ("measured", "policy") and e.get("pos_unit") \
                and e.get("scale_points_sha1") != scale.get("points_sha1"):
            raise ValueError(f"{where}: {name} was computed for the table of points sha1 "
                             f"{e.get('scale_points_sha1')}, the set's scale is that of {scale.get('points_sha1')}")
    if doc.get("hash") != hashes(doc):
        raise ValueError(f"{where}: its hashes are not those of its content: a set is generated, never edited "
                         f"(or the protection list moved since it was written)")
    return doc


def of_record(pair, root=results.ROOT):
    """The pair's set of record, refused when it has none."""
    path = set_path(pair, root)
    if not path.exists():
        raise ValueError(f"no constant set for {pair} ({sc.shown(path)}): `adopt` writes a running servo's first")
    return load(path)


def described(doc):
    """What every verb says of a set: its fields by class, the adopted
    ones being those with no source, the per-count ones at an unknown
    scale, the table and the slots, and the three hashes in short."""
    fields = doc["fields"].values()
    by_class = ", ".join(f"{sum(e['class'] == c for e in fields)} {c}" for c in CLASSES)
    unknown = sum(e.get("scale") == "unknown" for e in fields)
    table, slots, h, scale = doc["pos_table"], doc["wide_codes"], doc["hash"], doc.get("scale")
    return [
        f"{len(fields)} fields: {by_class} (no source); {unknown} per-count at an unknown scale",
        f"position table {table['state']}, points sha1 {table['points_sha1']} [{table['class']}]",
        "wide-code slots: none" if slots is None
        else f"wide-code slots {bytes(slots['slots']).hex(' ')} [{slots['class']}]",
        *([f"scale: {scale['counts_per_deg']:.4f} counts/deg, the {scale['kind']} one"] if scale else []),
        f"hash: plant {h['plant'][:12]}, protection {h['protection'][:12]} "
        f"({sum(n in PROTECTION for n in doc['fields'])} fields), set {h['set'][:12]}",
    ]


# --- adopt ---

def adopted(record, pair, origin):
    """The first set of a servo that already runs, from a record of its
    state: every field the record names writable past the bus front, the
    stamp left out, each at the value the record holds and of class
    `adopted`, with the table and the slots. `origin` is what the set
    says of the record. A record that names no writable fields, whose
    data state is not clean, or whose hash is not its content's, is
    refused."""
    if "writable" not in record:
        raise ValueError("the record names no writable fields: record the servo's state again "
                         "(`scorecard.servo_state`)")
    if record.get("hash") != sc.state_hash(record):
        raise ValueError("the record's hash is not that of its content")
    if record["data_state"] != "clean":
        raise ValueError(f"the record's data state is {record['data_state']}: a set is adopted from a state the "
                         f"servo runs closed loop on")
    own = {"class": "adopted", "origin": ORIGIN}
    fields = {}
    for name in record["writable"]:
        if name in COMMS or name in (STAMP, widecodes.FIELD):
            continue
        fields[name] = {"value": record["fields"][name], **own, "protection": protection_of(name)}
        if name in PER_COUNT:
            fields[name]["scale"] = "unknown"
    slots = record["wide_codes"]
    doc = {
        "format": FORMAT, "pair": pair, "inputs": {}, "origins": {ORIGIN: origin},
        "wide_codes": None if slots is None else {"slots": list(bytes.fromhex(slots)), **own},
        "pos_table": {**{k: record["pos_table"][k] for k in ("state", "points_sha1")}, **own},
        "fields": fields, "previous": None,
    }
    return {**doc, "hash": hashes(doc)}


def adopt(pair, record_path, words=BEFORE, root=results.ROOT):
    """Write the pair's first set from the state record at `record_path`
    (`adopted`); returns (the set, its path). The set names the record
    by its sha1, the hash of the state it holds and `words`, and holds no
    path to it. Refused when the pair has a set."""
    path = set_path(pair, root)
    if path.exists():
        raise ValueError(f"{sc.shown(path)} exists: `adopt` writes a pair's first set only")
    record = json.loads(Path(record_path).read_text())
    origin = {"from": words, "sha1": sc.file_sha1(record_path), "state_hash": record.get("hash")}
    doc = adopted(record, pair, origin)
    return doc, write(doc, path)


# --- the rules ---

KINDS = {"u8": (0, 0xFF), "i8": (-0x80, 0x7F), "u16": (0, 0xFFFF), "i16": (-0x8000, 0x7FFF),
         "u32": (0, 0xFFFFFFFF), "i32": (-0x80000000, 0x7FFFFFFF)}
RULES = {}      # {field: Rule}, in the order the rules are written
POLICY = "policy.json"
ZERO_POINTS = hashlib.sha1(bytes(2 * postable.INTERVALS)).hexdigest()   # the points of an identity table
WIDE = f"{widecodes.STAGE}/{widecodes.EXPERIMENT}"
RAW_SCALE = ("3-servo/pot-table", "pot_scale_counts_per_deg")   # counts per deg of a pair with no table
# the byte of each gear class, as the firmware's client names them (client calib.rs GearClass)
GEAR_CLASSES = ("unknown", "plastic", "metal")


class Missing(Exception):
    """An input a rule reads is not there; the message says which."""


@dataclasses.dataclass(frozen=True)
class Rule:
    """What binds one field: its name in a set, the field as the
    firmware declares it (`kind` its storage, `unit`, `q` its fraction
    bits, `rev` moved by a firmware change of how the field is used),
    the power of counts per deg its value carries, the class its value
    is of, the firmware code it mirrors, and the function of a Context
    that gives the value in the field's unit."""
    field: str
    name: str
    kind: str
    unit: str
    q: int
    pos_unit: int
    rev: int
    cls: str
    mirrors: str
    fn: object


def rule(field, name, *, kind, unit, mirrors, q=0, pos_unit=0, rev=1, cls="measured"):
    """Register the function under it as the rule of `field`."""
    def register(fn):
        RULES[field] = Rule(field, name, kind, unit, q, pos_unit, rev, cls, mirrors, fn)
        return fn
    return register


def encode(r, x, fit=True):
    """A rule's value as the field holds it: rounded to its Q format,
    half away from zero, and refused, never clamped, when it does not
    fit the field. Text, a value `osc dump` prints in words, passes."""
    if isinstance(x, str):
        return x
    y = x * (1 << r.q)
    v = int(math.copysign(math.floor(abs(y) + 0.5), y))
    lo, hi = KINDS[r.kind]
    if fit and not lo <= v <= hi:
        raise ValueError(f"{r.field}: {r.name} gives {x:g} {r.unit}, {v} as the field holds it, which does not "
                         f"fit its {r.kind} ({lo}..{hi})")
    return v


def registry_of(pair):
    """The registry entries of a pair's servo and board, as plain
    fields; a kind the registry has no entry for is left out."""
    servo, _, board = pair.partition("__")
    return {kind: dataclasses.asdict(entries[key])
            for kind, entries, key in (("servo", servos.SERVOS, servo), ("board", boards.BOARDS, board))
            if key in entries}


class Context:
    """What a rule reads through, every read recorded as it is made, so
    a field's inputs are what its rule read: results of record (`q`,
    `raw`), the registry (`reg`), the policy file (`policy`), and the
    scale of the count (`counts_per_deg`, `travel_deg`, `ascending`).
    `inputs` gathers what a set names under `inputs`, `notes` what a
    build has to say. A read of something that is not there raises
    Missing."""

    def __init__(self, pair, root=results.ROOT, registry=None):
        self.pair, self.root = pair, Path(root)
        self.registry = registry_of(pair) if registry is None else registry
        self.inputs, self.notes, self.docs = {}, [], {}
        self.reads, self.side = [], {}
        self.scale = self.table = None
        beside = set_path(pair, root).parent.relative_to(self.root)     # the policy and the pins lie beside the set
        self.policies = self.file("policy", beside / POLICY) or {}
        self.pins = self.file("pins", beside / f"{pair}.pins.json") or {}
        loose = [n for n, pin in self.pins.items() if not all(k in pin for k in ("value", "why", "rung"))]
        if loose:
            raise ValueError(f"the pins of {pair}: {', '.join(loose)} lacks its value, its why or its rung")

    def file(self, name, rel):
        """The document of an input file, None when there is none; the
        file is named in the set's inputs with its sha1."""
        if name not in self.docs:
            path = self.root / rel
            self.docs[name] = results.read_file(path) if path.exists() else None
            if self.docs[name] is not None:
                self.inputs[name] = {"file": Path(rel).as_posix(), "sha1": sc.file_sha1(path)}
        return self.docs[name]

    def result(self, name):
        """The result `name` (stage/experiment) of the pair, or of its
        board for a stage 1 result. A result written before results
        named what they read is said, not refused; one that names a
        file which has moved since is stale, and refused."""
        seen = name in self.docs
        stage, experiment = name.split("/")
        key = self.pair.partition("__")[2] if stage == widecodes.STAGE else self.pair
        doc = self.file(name, results.path(stage, experiment, key).relative_to(results.ROOT))
        if doc is None:
            raise Missing(f"{name} has no result for {key}")
        if not seen:
            if "inputs" not in doc:
                self.inputs[name]["inputs_recorded"] = False
                self.notes.append(f"{name}: the results it read are not recorded")
            for file, sha in doc.get("inputs", {}).items():
                now = sc.file_sha1(self.root / file) if (self.root / file).exists() else "gone"
                if now != sha:
                    raise ValueError(f"{name} is stale: it read {file} at sha1 {sha}, which is {now} now: "
                                     f"re-run {name}")
        return doc

    def raw(self, name, key):
        """`key` of result `name`, as the file holds it."""
        doc = self.result(name)
        if key not in doc:
            raise Missing(f"{name} has no {key}")
        self.reads.append((f"{name}:{key}", doc[key]))
        return doc[key]

    def q(self, name, key):
        """The value of quantity `key` of result `name`; its `lo` or
        `hi` while the rule is evaluated at that end of its band."""
        quantity = self.raw(name, key)
        return quantity[self.side.get(f"{name}:{key}", "value")]

    def reg(self, kind, key):
        """`key` of the pair's registry entry of `kind`."""
        if key not in self.registry.get(kind, {}):
            raise Missing(f"the registry has no {kind} {key} for {self.pair}")
        value = self.registry[kind][key]
        self.inputs.setdefault("registry", {}).setdefault(kind, {})[key] = value
        self.reads.append((f"registry:{key}", None))
        return value

    def policy(self, key):
        if key not in self.policies:
            raise Missing(f"{POLICY} has no {key}")
        self.reads.append((f"policy:{key}", None))
        return self.policies[key]

    @property
    def counts_per_deg(self):
        """The scale of the count: the table's, or the raw pot's on a
        pair with no table."""
        if self.scale is None:
            raise Missing("the pair has no scale: no position table and no stage 3 pot-table result")
        self.reads.append(("scale", None))
        return self.scale["counts_per_deg"]

    @property
    def travel_deg(self):
        """Stop to stop, deg, at the scale of the count."""
        if self.table is None:
            raise Missing("the pair has no position table: the stops its scale runs between are not known")
        self.reads.append(("pos_table:stops", None))
        return (self.table.raw_max - self.table.raw_min) / self.counts_per_deg

    @property
    def ascending(self):
        """Whether the tag angle rises with the count: the camera sees
        the horn's face, so its counterclockwise is the horn's."""
        if self.table is None:
            raise Missing("the pair has no position table: which stop is the clockwise one is not known")
        self.reads.append(("pos_table:theta_at_stops_deg", None))
        return self.table.theta_max > self.table.theta_min


def evaluate(r, c):
    """Rule `r`'s entry of a set, from Context `c`: its value encoded,
    the inputs it read, the table its value was computed for when it
    carries a position unit, and its band, the rule evaluated again at
    each end of every banded input it read."""
    def at(side):
        c.reads, c.side = [], side
        return r.fn(c)

    value = encode(r, at({}))
    reads = c.reads
    ends = [encode(r, at({name: end}), fit=False) for name, quantity in reads if isinstance(quantity, dict)
            for end in ("lo", "hi") if quantity.get(end) is not None]
    c.side = {}
    e = {"value": value, "class": r.cls, "rule": r.name, "rule_rev": r.rev, "unit": r.unit, "q": r.q,
         "pos_unit": r.pos_unit, "inputs": list(dict.fromkeys(name for name, _ in reads))}
    if r.pos_unit:
        e["scale_points_sha1"] = c.scale["points_sha1"]
    if ends and not isinstance(value, str):
        e["band"] = [min(value, *ends), max(value, *ends)]
    return e


# The angle map: centi-deg at each stop, rising counterclockwise facing
# the horn, the clockwise stop at the policy's angle and the other one
# the travel past it. The travel is the stops' span at the scale of the
# count, so a host that divides the map's span by the stops' gets the
# table's counts per deg (ident kinematics.rs deg_per_count).
@rule("angle_min_cdeg", "angle_min.table", kind="i16", unit="cdeg", pos_unit=-1,
      mirrors="core regions/calib.rs CalibKinematics, tools/osc cal/mod.rs rail_angles")
def angle_min(c):
    return 100 * (c.policy("angle_cw_stop_deg") + (0 if c.ascending else c.travel_deg))


@rule("angle_max_cdeg", "angle_max.table", kind="i16", unit="cdeg", pos_unit=-1,
      mirrors="core regions/calib.rs CalibKinematics, tools/osc cal/mod.rs rail_angles")
def angle_max(c):
    return 100 * (c.policy("angle_cw_stop_deg") + (c.travel_deg if c.ascending else 0))


@rule("gear_ratio_centi", "gear_ratio.registry", kind="u16", unit="motor rev per output rev x100",
      mirrors="core regions/calib.rs CalibKinematics, tools/osc cal/mod.rs ratio_to_centi")
def gear_ratio(c):
    return 100 * c.reg("servo", "gear_ratio")


@rule("gear_class", "gear_class.registry", kind="u8", unit="class",
      mirrors="client calib.rs GearClass, tools/osc main.rs dump")
def gear_class(c):
    name = c.reg("servo", "gear_class")
    if name not in GEAR_CLASSES[1:]:
        raise ValueError(f"gear_class: the registry's {name!r} is no gear class of {', '.join(GEAR_CLASSES[1:])}")
    return f"{GEAR_CLASSES.index(name)} ({name})"


# --- build ---

def held(what, rule_name, carried, why, previous, would_be=None):
    """The entry of a field (`what` "value") or of the slots ("slots")
    whose rule does not bind: what the previous set held, why, the set
    it was first carried from, and what the rule would give where it
    gave a value."""
    e = {what: carried[what], "class": "held", "rule": rule_name, "held": why,
         "carried_from": carried.get("carried_from", previous["hash"]["set"])}
    return e if would_be is None else {**e, "would_be": would_be}


def slots_of(c, previous):
    """The set's wide-code slots, from the board's stage 1 result; held
    at the previous set's where the board has none, and None where
    there is nothing to carry either."""
    name, carried = "slots.wide_codes", previous and previous["wide_codes"]
    c.reads = []
    try:
        slots = c.raw(WIDE, widecodes.FIELD)
    except Missing as m:
        if carried is None:
            c.notes.append(f"{SLOTS}: left out, {m}")
            return None
        c.notes.append(f"{SLOTS}: held, {m}")
        return held("slots", name, carried, str(m), previous)
    if not (isinstance(slots, list) and len(slots) == widecodes.SLOTS
            and all(type(v) is int and 0 <= v <= widecodes.SLOT_MAX for v in slots)):
        raise ValueError(f"{WIDE}: {widecodes.FIELD} is not {widecodes.SLOTS} values 0..{widecodes.SLOT_MAX}")
    return {"slots": slots, "class": "measured", "rule": name, "inputs": [f"{WIDE}:{widecodes.FIELD}"]}


def table_of(c, previous, slots):
    """The set's position table and the scale of its count, set on `c`
    for the rules. With the pair's table file: LIVE, its points, refused
    when its lookup was fitted on other slots than the set's or it was
    built against other stops than the set's; a file that records no
    slots is said. Without one: the previous set's table carried, the
    identity on a pair with none, and where that is the identity the
    raw pot's scale from stage 3."""
    path = set_path(c.pair, c.root).with_name(postable.out_path(c.pair).name)
    if not path.exists():
        table = copy.deepcopy(previous["pos_table"]) if previous else {
            "state": "IDENTITY", "points_sha1": ZERO_POINTS, "class": "measured", "rule": "table.none"}
        if table["state"] == "IDENTITY":
            try:
                c.scale = {"kind": "raw", "counts_per_deg": abs(c.q(*RAW_SCALE)), "points_sha1": None}
            except Missing as m:
                c.notes.append(f"scale: none, {m}")
        else:
            c.notes.append(f"scale: none, the pair has no table file for its {table['state']} table")
        return table
    c.table, file = postable.load_file(path)
    c.inputs["pos_table"] = {"file": path.relative_to(c.root).as_posix(), "sha1": sc.file_sha1(path)}
    fitted = file["inputs"].get("fitted_on_slots_sha1")
    mine = postable.slots_sha1(slots["slots"] if slots is not None else bytes(widecodes.SLOTS))
    if fitted is None:
        c.notes.append(f"position table: {postable.fitted_on_said(file)}")
    elif fitted != mine:
        raise ValueError(f"the lookup was fitted on other slots (sha1 {fitted}, the set's are {mine}): re-run "
                         f"stage 4 position, then `postable build`")
    stops = [previous["fields"][n]["value"] for n in STOPS if previous and n in previous["fields"]]
    if stops and stops != [c.table.raw_min, c.table.raw_max]:
        raise ValueError(f"{sc.shown(path)} was built against stops {c.table.raw_min} and {c.table.raw_max}, the "
                         f"set's are {stops[0]} and {stops[1]}: `postable build` against the set's")
    points = points_sha1(c.table.points)
    c.scale = {"kind": "table", "counts_per_deg": abs(c.table.scale), "points_sha1": points}
    return {"state": "LIVE", "points_sha1": points, "fitted_on_slots_sha1": fitted, "class": "measured",
            "rule": "table.pos_lut", "inputs": ["pos_table"]}


def bound(c, name, carried, previous):
    """The entry a build gives field `name`, None when it has none: a
    pin of the pair, else its rule's value, held at the carried value
    where the policy holds it or an input is missing, else the previous
    set's entry as it stands."""
    if name in c.pins:
        pin = c.pins[name]
        return {"value": pin["value"], "class": "pinned", "why": pin["why"], "rung": pin["rung"]}
    r = RULES.get(name)
    if r is None:
        return copy.deepcopy(carried)
    try:
        e = evaluate(r, c)
    except Missing as m:
        c.notes.append(f"{name}: {'left out' if carried is None else 'held'}, {m}")
        return None if carried is None else held("value", r.name, carried, str(m), previous)
    why = c.policies.get("hold", {}).get(name)
    if why is None:
        return e
    c.notes.append(f"{name}: {'left out' if carried is None else 'held'}, {why}; its rule gives {e['value']}")
    return None if carried is None else held("value", r.name, carried, why, previous, e["value"])


def build(pair, previous=None, protection=(), root=results.ROOT, registry=None):
    """(the pair's set, what there is to say of it), built offline from
    the results of record, the registry, the policy file and the pair's
    pins, on top of `previous`, its set of record: first what defines a
    count, the slots, then the table and its scale, then every field,
    in the previous set's order with new ones after. A field with no
    rule and no pin is carried as it stands.

    One build moves one hash. Without `protection` it moves the plant:
    every protection field is carried, with `would_be` where its rule
    or pin gives another value. With `protection` it moves the fields
    named there and carries everything else, the slots, the table and
    the scale with it. A set with nothing before it is built whole."""
    wrong = [n for n in protection if n not in PROTECTION]
    if wrong or (protection and previous is None):
        raise ValueError(f"{', '.join(wrong)} is no protection field" if wrong
                         else "there is no set to carry the plant from")
    c = Context(pair, root, registry)
    slots = slots_of(c, previous)
    table = table_of(c, previous, slots)
    scale = c.scale
    if protection:
        # the plant stands: what defines a count is the previous set's, and must be what it would be now
        stood = [previous["wide_codes"], previous["pos_table"], previous.get("scale")]
        if any((a is None) != (b is None) or (a is not None and any(a[k] != b[k] for k in keys))
               for a, b, keys in zip((slots, table, scale), stood,
                                     (("slots",), ("state", "points_sha1"), ("counts_per_deg",)))):
            raise ValueError("the slots, the table or the scale would move: build the plant first, then the "
                             "protection fields")
        slots, table, scale = (copy.deepcopy(x) for x in stood)
    before = previous["fields"] if previous else {}
    fields = {}
    for name in dict.fromkeys([*before, *RULES, *c.pins]):
        e = bound(c, name, before.get(name), previous)
        moves = (name in protection) if name in PROTECTION else not protection
        if previous is not None and not moves:
            # the side this build does not move: as it stood, with what it would be beside it
            would = None
            if e is not None and name in before:
                would = e["value"] if e["value"] != before[name]["value"] else e.get("would_be")
            e = copy.deepcopy(before.get(name))
            if e is None:
                c.notes.append(f"{name}: left out, "
                               f"{'name it with --protection' if name in PROTECTION else 'a field of the plant'}")
            else:
                e.pop("would_be", None)
                if would is not None:
                    e["would_be"] = would
                    c.notes.append(f"{name}: carried at {e['value']}, it would be {would}"
                                   f"{': name it with --protection' if name in PROTECTION else ''}")
        if e is not None:
            fields[name] = {**e, "protection": protection_of(name)}
    named = {e.get("origin") for e in (*fields.values(), slots or {}, table)}
    doc = {"format": FORMAT, "pair": pair, "inputs": c.inputs,
           "origins": {k: v for k, v in (previous["origins"] if previous else {}).items() if k in named},
           "wide_codes": slots, "pos_table": table, "fields": fields,
           "previous": previous["hash"] if previous else None}
    if scale is not None:
        doc["scale"] = scale
    return {**doc, "hash": hashes(doc)}, c.notes


def changes(previous, doc):
    """What a built set changes against the one before it, a line each:
    a value that moves, and a value that stands whose class does."""
    def said(e, what):
        return str(e[what]) if what != "slots" else bytes(e[what]).hex(" ")

    def tag(e):
        return ", ".join(str(e[k]) for k in ("class", "rule") if k in e)

    out = []
    for label, key, whats in ((SLOTS, "wide_codes", ("slots",)), ("position table", "pos_table",
                                                                  ("state", "points_sha1"))):
        a, b = previous.get(key), doc.get(key)
        if a is None or b is None:
            if a is not b:
                out.append(f"{label}: {'none' if a is None else tag(a)} -> {'none' if b is None else tag(b)}")
            continue
        for what in whats:
            name = label if what == "slots" else f"{label} {what.replace('_', ' ')}"
            if a[what] != b[what]:
                out.append(f"{name}: {said(a, what)} -> {said(b, what)} [{tag(b)}]")
            elif tag(a) != tag(b):
                out.append(f"{name}: {said(b, what)} [{tag(a)} -> {tag(b)}]")
    a, b = previous.get("scale"), doc.get("scale")
    if a != b:
        out.append("scale: " + " -> ".join("none" if x is None else f"{x['counts_per_deg']:.4f} counts/deg, the "
                                           f"{x['kind']} one" for x in (a, b)))
    for name in dict.fromkeys([*previous["fields"], *doc["fields"]]):
        a, b = previous["fields"].get(name), doc["fields"].get(name)
        if a is None or b is None:
            out.append(f"{name}: {'not in the set' if a is None else a['value']} -> "
                       f"{'not in the set' if b is None else b['value']}")
        elif a["value"] != b["value"]:
            out.append(f"{name}: {a['value']} -> {b['value']} [{tag(b)}]")
        elif tag(a) != tag(b):
            out.append(f"{name}: {b['value']} [{tag(a)} -> {tag(b)}]")
    moved_ = [k for k in ("plant", "protection") if previous["hash"][k] != doc["hash"][k]]
    out.append("hash: " + (f"{' and '.join(moved_)} moved" if moved_ else "neither moved") + ": "
               + ", ".join(f"{k} {previous['hash'][k][:12]} -> {doc['hash'][k][:12]}"
                           for k in ("plant", "protection", "set")))
    return out


# --- status ---

def users(doc, name):
    """What of a set lists input `name`: its fields, the slots, the
    table."""
    blocks = ((SLOTS, doc["wide_codes"] or {}), ("position table", doc["pos_table"]))
    return [n for n, e in (*doc["fields"].items(), *blocks)
            if any(i.split(":")[0] == name for i in e.get("inputs", ()))]


def moved(doc, root=results.ROOT):
    """The input files that are no longer the ones the set read, each as
    (input, file, its sha1 now or None when it is gone, what lists the
    input)."""
    out = []
    for name in sorted(doc["inputs"]):
        inp = doc["inputs"][name]
        if "file" not in inp:
            continue
        p = Path(root) / inp["file"]
        now = sc.file_sha1(p) if p.exists() else None
        if now != inp["sha1"]:
            out.append((name, inp["file"], now, users(doc, name)))
    return out


def registry_moved(doc, registry=None):
    """The registry values the set read that the registry no longer
    holds, each as (kind, key, the value read, the value now)."""
    now = registry_of(doc["pair"]) if registry is None else registry
    return [(kind, key, value, now.get(kind, {}).get(key))
            for kind, read in sorted(doc["inputs"].get("registry", {}).items())
            for key, value in sorted(read.items()) if now.get(kind, {}).get(key) != value]


def status(doc, root=results.ROOT, registry=None):
    """(lines, stale): the set against its inputs as they are now, a
    moved or missing file and a moved registry value named with what
    lists it, then the inputs that record no inputs of their own, what
    the set holds and where its adopted values came from."""
    stale = moved(doc, root)
    lines = [f"stale: {name} ({file}) is {'gone' if now is None else f'no longer the file the set read, sha1 {now}'}"
             f": {', '.join(listed) or 'nothing lists it'}" for name, file, now, listed in stale]
    reg = registry_moved(doc, registry)
    lines += [f"stale: the registry's {kind} {key} was {was}, it is {now}: "
              f"{', '.join(n for n, e in doc['fields'].items() if f'registry:{key}' in e.get('inputs', ()))}"
              for kind, key, was, now in reg]
    files = sum("file" in i for i in doc["inputs"].values())
    if not stale and not reg:
        lines.append(f"{files} input files, none moved")
    lines += [f"{name}: the results it read are not recorded: {', '.join(users(doc, name))}"
              for name, inp in sorted(doc["inputs"].items()) if inp.get("inputs_recorded") is False]
    lines += described(doc)
    for name in sorted(doc["origins"]):
        o = doc["origins"][name]
        lines.append(f"adopted from {name}: {o['from']} (record sha1 {o['sha1']}, state hash {o['state_hash'][:12]})")
    return lines, bool(stale or reg)


# --- check ---

def differences(doc, state):
    """Where the servo's `state` (`scorecard.servo_state`) is not the
    set, a line each as `name: servo X, set Y [class, rule]`: every
    field of the set by name, the table's state and points, the slots."""
    def line(name, got, e, want):
        tag = ", ".join(str(e[k]) for k in ("class", "rule") if k in e)
        return f"{name}: servo {got}, set {want} [{tag}]"

    out = []
    for name, e in doc["fields"].items():
        got = state["fields"].get(name, "names no such field")
        if got != e["value"]:
            out.append(line(name, got, e, e["value"]))
    table = doc["pos_table"]
    for label, key in ((TABLE_STATE, "state"), (TABLE_POINTS, "points_sha1")):
        if state["pos_table"][key] != table[key]:
            out.append(line(label, state["pos_table"][key], table, table[key]))
    slots = doc["wide_codes"]
    if slots is not None:
        want = bytes(slots["slots"]).hex(" ")
        got = "names none" if state["wide_codes"] is None else bytes.fromhex(state["wide_codes"]).hex(" ")
        if got != want:
            out.append(line(SLOTS, got, slots, want))
    return out


def unowned(doc, state):
    """The fields a host may write on the servo that the set does not
    hold, with the value the servo has for each: first any the set is
    missing, then the ones no set holds."""
    own = set(doc["fields"]) | ({widecodes.FIELD} if doc["wide_codes"] is not None else set())
    out = [n for n in state["writable"] if n not in own]
    never = [n for n in out if n in COMMS or n == STAMP]

    def shown(names):
        return ", ".join(f"{n} = {state['fields'][n]}" for n in names)

    lines = []
    if len(out) > len(never):
        lines.append(f"not in the set: {shown(n for n in out if n not in never)}")
    if never:
        lines.append(f"in no set (the bus front and the stamp): {shown(never)}")
    return lines


def check(doc, state=None):
    """The servo against the set, nothing on the servo changed; prints
    the differences, what the set holds and what the servo holds outside
    it, and returns the differences. Refused, after the lines, when the
    servo's data state is not clean."""
    state = sc.servo_state() if state is None else state
    out = differences(doc, state)
    print("\n".join([*out, *described(doc), *unowned(doc, state)]))
    if state["data_state"] != "clean":
        raise settings.Refused(f"data state {state['data_state']}, not clean: the servo's constants are not a "
                               f"state it runs closed loop on")
    return out


# --- install ---

class Rejected(settings.Refused):
    """The servo refused a write: nothing landed."""


def plain(value):
    """A value as `osc set` takes it: `osc dump`'s text less the note in
    parentheses it prints after an enum's name or a class's number."""
    return re.sub(r"\s*\(.*\)$", "", str(value))


def looser(name, on_servo, wanted):
    """Whether moving protection field `name` from the servo's value to
    the set's protects less: by its direction on two integers, and
    always for a judged field or a value that is no integer."""
    way = LOOSER[name]
    if way == "judged" or not all(type(v) is int for v in (on_servo, wanted)):
        return True
    return wanted > on_servo if way == "up" else wanted < on_servo


def gate(doc, state, protection=(), loosen=()):
    """The protection gate against the servo's `state`: a line for every
    name that is no protection field of the set, and for every
    protection field the set would change that `protection` does not
    name, or, where the change protects less, `loosen` does not name as
    well. Empty when the install may go on."""
    out = [f"{n} is not a protection field of the set" for n in dict.fromkeys([*protection, *loosen])
           if n not in PROTECTION or n not in doc["fields"]]
    out += [f"{n} is named to loosen and not as a protection change: name it with --protection too"
            for n in loosen if n not in protection]
    for name, e in doc["fields"].items():
        got = state["fields"][name]
        if name not in PROTECTION or got == e["value"]:
            continue
        way = LOOSER[name]
        less = looser(name, got, e["value"])
        said = ("judged, taken as protecting less" if way == "judged"
                else f"protects {'less' if less else 'more'} (less is {way})")
        need = [f for f, names, wanted in (("--protection", protection, True), ("--loosen", loosen, less))
                if wanted and name not in names]
        if need:
            out.append(f"{name}: servo {got}, set {e['value']}, {said}: name it with "
                       f"{' and '.join(f'{f} {name}' for f in need)}")
    return out


def points_sha1(points):
    """The sha1 of a table's 256 points as the servo holds them, i16 LE
    in a row: what a state record names its points by."""
    return hashlib.sha1(b"".join(postable.page_bytes(points, p) for p in range(postable.PAGES))).hexdigest()


def table_to_write(doc, state, root=results.ROOT, table=None):
    """The Table an install has to write, None when the servo holds the
    set's. An IDENTITY table is all-zero points against the set's
    stops. A LIVE one is `table` when it is given and holds the set's
    points, else the one of the pair's table file, which must hold the
    set's points, be built against the set's stops, and name no other
    slots than the set's as the ones its lookup was fitted on; a file
    that records none is said and taken. No other state can be
    written."""
    want, have = doc["pos_table"], state["pos_table"]
    if all(have[k] == want[k] for k in ("state", "points_sha1")):
        return None
    if want["state"] not in ("LIVE", "IDENTITY"):
        raise settings.Refused(f"the set's table is {want['state']} (points sha1 {want['points_sha1']}) and the "
                               f"servo's {have['state']} ({have['points_sha1']}): an install writes a LIVE or an "
                               f"IDENTITY table only")
    stops = [doc["fields"][n]["value"] if n in doc["fields"] else state["fields"].get(n) for n in STOPS]
    if want["state"] == "IDENTITY":
        table = postable.from_points(bytes(2 * postable.INTERVALS), *stops)
    if table is not None:
        if points_sha1(table.points) != want["points_sha1"]:
            raise settings.Refused(f"the table at hand holds points of sha1 {points_sha1(table.points)}, the "
                                   f"set's table is {want['points_sha1']}")
        return table
    path = set_path(doc["pair"], root).with_name(postable.out_path(doc["pair"]).name)
    if not path.exists():
        raise settings.Refused(f"the set's table is LIVE and the pair has no table file ({sc.shown(path)})")
    t, file = postable.load_file(path)
    fitted = file["inputs"].get("fitted_on_slots_sha1")
    slots = doc["wide_codes"]["slots"] if doc["wide_codes"] is not None else bytes(widecodes.SLOTS)
    if fitted is None:
        print(f"{sc.shown(path)}: {postable.fitted_on_said(file)}")
    elif fitted != postable.slots_sha1(slots):
        raise settings.Refused(f"the lookup was fitted on other slots (sha1 {fitted}, the set's are "
                               f"{postable.slots_sha1(slots)}): re-run stage 4 position, then `postable build`")
    if points_sha1(t.points) != want["points_sha1"]:
        raise settings.Refused(f"{sc.shown(path)} holds points of sha1 {points_sha1(t.points)}, the set's table "
                               f"is {want['points_sha1']}")
    if stops != [t.raw_min, t.raw_max]:
        raise settings.Refused(f"{sc.shown(path)} was built against stops {t.raw_min} and {t.raw_max}, the set's "
                               f"are {stops[0]} and {stops[1]}")
    return t


def install(doc, protection=(), loosen=(), root=results.ROOT, table=None):
    """Make the servo equal the set, volatile; prints each write as it
    lands and returns what was written, by name, in order. Nothing is
    written unless torque is off, no input file of the set moved, the
    servo lets a host write every field the set holds, the table can be
    installed (`table_to_write`) and the protection gate passes
    (`gate`). The writes are the slots, the stops, the table, then the
    other fields that differ in the servo's table order, each read
    back, a field the servo refuses tried once more after the rest. The
    state read back whole must then be the set, and only then is it
    stamped, no save, and its data state required clean. A refusal after the first write
    reboots the servo onto its saved state and says so."""
    torque = osc.get("torque_enable")
    if not settings.same("off", torque):
        raise settings.Refused(f"torque_enable reads {torque}: a set is installed with torque off")
    stale = moved(doc, root)
    if stale:
        raise settings.Refused(f"the set is stale, {', '.join(name for name, *_ in stale)} moved: `status` names "
                               f"the fields")
    state = sc.servo_state()
    slots = doc["wide_codes"]
    missing = [n for n in doc["fields"] if n not in state["writable"]]
    if slots is not None and state["wide_codes"] is None:
        missing.append(widecodes.FIELD)
    if missing:
        raise settings.Refused(f"the servo's descriptor lets a host write no {', '.join(missing)}")
    refused = gate(doc, state, protection, loosen)
    if refused:
        raise settings.Refused("the protection gate:\n  " + "\n  ".join(refused))
    table = table_to_write(doc, state, root, table)
    differ = [n for n in state["writable"] if n in doc["fields"] and state["fields"][n] != doc["fields"][n]["value"]]
    wrote = []

    def put(name):
        arg = plain(doc["fields"][name]["value"])
        try:
            osc.set_(name, arg)
        except subprocess.CalledProcessError:
            raise Rejected(f"{name}: the servo refused {arg}") from None
        wrote.append(name)
        back = osc.get(name)
        if not settings.same(arg, back):
            raise settings.Refused(f"{name}: wrote {arg}, read back {back}")
        print(f"{name}: {state['fields'][name]} -> {doc['fields'][name]['value']}")

    try:
        if slots is not None and bytes.fromhex(state["wide_codes"]) != bytes(slots["slots"]):
            wrote.append(SLOTS)
            data, old = widecodes.write_slots(bytes(slots["slots"]))
            print(f"{SLOTS}: {old.hex(' ')} -> {data.hex(' ')}")
        for name in (n for n in STOPS if n in differ):
            put(name)
        if table is not None:
            wrote.append("position table")
            postable.install_table(table)
            print(f"position table: {state['pos_table']['state']} -> {doc['pos_table']['state']}, "
                  f"{postable.PAGES} pages read back")
        pending = [n for n in differ if n not in STOPS]
        for last in (False, True):
            again = []
            for name in pending:
                try:
                    put(name)
                except Rejected:
                    if last:
                        raise
                    again.append(name)
            pending = again
        if wrote:
            state = sc.servo_state()
            left = differences(doc, state)
            if left:
                raise settings.Refused(f"the state read back is not the set: {'; '.join(left)}")
        if wrote or state["data_state"] != "clean":
            wrote.append("stamp")
            stamp = settings.plant_stamp(settings.stamp_live())
            print(f"stamped: data state clean, stamp {stamp}")
    except (settings.Refused, subprocess.CalledProcessError, RuntimeError) as e:
        if not wrote:
            raise
        raise settings.Refused(f"{e}; {rebooted()}") from None
    return wrote


def rebooted():
    """Reboot the servo off a half-written set; what became of it, in
    words."""
    try:
        osc.run("reboot")
        settings.await_boot()
    except (settings.Refused, subprocess.CalledProcessError):
        return "the reboot did not answer: the servo holds a half-written set until it is power cycled"
    return "rebooted: the servo is back on its saved state"


# --- save and restore ---

def uncommitted(path):
    """What `git status` says of one file, read only: empty when the
    file is committed as it stands."""
    r = subprocess.run(["git", "--no-optional-locks", "-C", str(Path(path).parent), "status", "--porcelain",
                        "--ignored", "--", Path(path).name], capture_output=True, text=True)
    if r.returncode != 0:
        raise settings.Refused(f"git cannot say whether {sc.shown(path)} is committed: {r.stderr.strip()}")
    return r.stdout.strip()


def backup_dir(doc, root=results.ROOT):
    """Where a save of the set keeps the state the servo had stored and
    the state after it: a folder by the set's hash, and for each save of
    the same set after the first a numbered folder inside it, so no
    record of a stored state is written over."""
    base = Path(root) / "telemetry" / doc["pair"] / "constants" / doc["hash"]["set"][:12]
    folder, n = base, 2
    while (folder / "stored.json").exists():
        folder, n = base / str(n), n + 1
    return folder


def puts_back(stored):
    return (f"`uv run python -m oscnb.constants restore --from {sc.shown(stored)}` installs, volatile, the state "
            f"the servo had stored")


def save(pair, protection=(), loosen=(), root=results.ROOT):
    """Persist the pair's set of record on the servo, from the state it
    has stored. Refused with the bus untouched when an input of the set
    moved or the set's file is not committed as it stands, then unless
    torque is off. The servo is rebooted and the state it boots with
    written to `stored.json` under `backup_dir`, the record a `restore`
    puts back. The set is installed from there (`install`, its gate,
    `protection` and `loosen` passed through) and checked, then MGMT
    SAVE (`osc save`, protocol sec 9.4) programs CONFIG and CALIB with
    the slots, the stamp and the table's points into flash, and with
    them whatever else of those regions the servo holds. After a reboot
    the state is written to `after.json` and must be the set. Returns
    the two paths."""
    path = set_path(pair, root)
    doc = of_record(pair, root)
    lines, stale = status(doc, root)
    if stale:
        raise settings.Refused(f"the set is stale: {'; '.join(s for s in lines if s.startswith('stale: '))}")
    dirty = uncommitted(path)
    if dirty:
        raise settings.Refused(f"{sc.shown(path)} is not committed as it stands ({dirty}): what is saved to "
                               f"flash is a set of record, commit it first")
    torque = osc.get("torque_enable")
    if not settings.same("off", torque):
        raise settings.Refused(f"torque_enable reads {torque}: a save reboots the servo and needs torque off")
    osc.run("reboot")
    settings.await_boot()
    state = sc.servo_state()
    folder = backup_dir(doc, root)
    folder.mkdir(parents=True, exist_ok=True)
    stored, after = folder / "stored.json", folder / "after.json"
    stored.write_text(json.dumps(state, indent=2) + "\n")
    try:
        install(doc, protection, loosen, root)
        left = check(doc)
        if left:
            raise settings.Refused(f"after the install the servo is not the set: {'; '.join(left)}")
    except settings.Refused as e:
        raise settings.Refused(f"{e}\nnothing was saved: a reboot returns the state the servo has stored, which "
                               f"{sc.shown(stored)} records ({puts_back(stored)})") from None
    try:
        osc.run("save")
    except subprocess.CalledProcessError:
        raise settings.Refused(f"the servo refused the SAVE: nothing was saved, and it holds the set, volatile. "
                               f"A reboot returns the state it has stored, which {sc.shown(stored)} "
                               f"records") from None
    osc.run("reboot")
    settings.await_boot()
    state = sc.servo_state()
    after.write_text(json.dumps(state, indent=2) + "\n")
    differ = differences(doc, state)
    if differ or state["data_state"] != "clean":
        raise settings.Refused(
            f"after the SAVE and a reboot the servo is not the set (data state {state['data_state']}"
            f"{''.join('; ' + d for d in differ)}): the save did not land as verified. Flash now holds the new "
            f"state, the one {sc.shown(after)} records, and no longer the old one. {puts_back(stored)}; to make "
            f"that permanent again, adopt that record as the pair's set of record (`adopt --from`, which writes "
            f"a pair's first set only: the present set's file out of the way first), commit it and `save` it")
    return stored, after


def restore(record_path, pair, protection=(), loosen=(), root=results.ROOT):
    """Put back, volatile, the state a record holds (`save`'s
    `stored.json`, any `servo.json`): the install of the set adopted
    from the record, its table written from the record's own points and
    not from a file. The protection gate stands as for any install.
    Returns what was written."""
    record = json.loads(Path(record_path).read_text())
    doc = adopted(record, pair, {"from": "a record restored", "sha1": sc.file_sha1(record_path),
                                 "state_hash": record.get("hash")})
    table = None
    if doc["pos_table"]["state"] == "LIVE":
        if "points" not in record["pos_table"]:
            raise ValueError("the record holds the sha1 of its table's points and not the points: its table "
                             "cannot be put back from it")
        stops = [record["fields"].get(n) for n in STOPS]
        table = postable.from_points(bytes.fromhex(record["pos_table"]["points"]), *stops)
    return install(doc, protection, loosen, root, table)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.constants", description=__doc__.split("\n\n")[0])
    ap.add_argument("--pair", default=postable.default_pair(), help="servo__board, the default pair if omitted")
    ap.add_argument("--set", type=Path, dest="derived",
                    help="for status, check and install: a derived set's file, not the pair's set of record")
    sub = ap.add_subparsers(dest="cmd", required=True)
    ad = sub.add_parser("adopt", help="the first set of a servo that already runs, every field adopted")
    ad.add_argument("--from", dest="record", type=Path, required=True,
                    help="a record of the servo's state, a scorecard run's servo.json")
    ad.add_argument("--origin", default=BEFORE, help="where the record came from, in words the set keeps")
    b = sub.add_parser("build", help="offline: the set from the results of record, on top of the set of record")
    b.add_argument("--out", type=Path, help="write a derived set here and leave the set of record as it is")
    b.add_argument("--protection", nargs="+", default=[], metavar="NAME",
                   help="move these protection fields and carry the plant, not the other way about")
    sub.add_parser("status", help="offline: the set's input files against the files now")
    sub.add_parser("check", help="the servo read against the set, a line a difference; changes nothing")
    ins = sub.add_parser("install", help="make the servo equal the set, volatile, verified, then stamped")
    ins.add_argument("--protection", nargs="+", default=[], metavar="NAME",
                     help="the protection fields the install may change")
    ins.add_argument("--loosen", nargs="+", default=[], metavar="NAME",
                     help="of those, the ones it may set to protect less than the servo's value")
    sv = sub.add_parser("save", help="persist the set of record on the servo: the stored state recorded, the "
                                     "set installed, SAVE, reboot, read back")
    sv.add_argument("--protection", nargs="+", default=[], metavar="NAME",
                    help="the protection fields the save may change from the stored state")
    sv.add_argument("--loosen", nargs="+", default=[], metavar="NAME",
                    help="of those, the ones it may set to protect less than the stored value")
    res = sub.add_parser("restore", help="put back, volatile, the state a record holds")
    res.add_argument("--from", dest="record", type=Path, required=True,
                     help="a record of the servo's state, a save's stored.json")
    res.add_argument("--protection", nargs="+", default=[], metavar="NAME",
                     help="the protection fields the restore may change")
    res.add_argument("--loosen", nargs="+", default=[], metavar="NAME",
                     help="of those, the ones it may set to protect less than the servo's value")
    a = ap.parse_args(argv)
    root = results.ROOT

    def the_set():
        return load(a.derived) if a.derived else of_record(a.pair, root)

    try:
        if a.cmd == "adopt":
            doc, path = adopt(a.pair, a.record, a.origin, root)
            print("\n".join([f"wrote {sc.shown(path)}", *described(doc)]))
        elif a.cmd == "build":
            path = set_path(a.pair, root)
            previous = load(path) if path.exists() else None
            doc, notes = build(a.pair, previous, a.protection, root)
            out = write(doc, a.out or path)
            print("\n".join([f"wrote {sc.shown(out)}", *(changes(previous, doc) if previous else []), *notes,
                             *described(doc)]))
        elif a.cmd == "status":
            lines, stale = status(the_set(), root)
            print("\n".join(lines))
            if stale:
                raise SystemExit(1)
        elif a.cmd == "check":
            if check(the_set()):
                raise SystemExit(1)
        elif a.cmd == "save":
            if a.derived:
                raise ValueError(f"{sc.shown(a.derived)} is a derived set: only the pair's set of record is saved")
            stored, after = save(a.pair, a.protection, a.loosen, root)
            print(f"saved: after a reboot the servo reads as the set, data state clean; {sc.shown(stored)} holds "
                  f"the state it had stored before, {sc.shown(after)} the state it boots with now")
        elif a.cmd == "restore":
            wrote = restore(a.record, a.pair, a.protection, a.loosen, root)
            print("the servo holds the record's state: nothing written" if not wrote
                  else "volatile, not saved: `osc reboot` returns the saved state")
        else:
            doc = the_set()
            wrote = install(doc, a.protection, a.loosen, root)
            print("\n".join([*described(doc),
                             "the servo holds the set: nothing written" if not wrote
                             else "volatile, not saved: `osc reboot` returns the saved state"]))
    except (ValueError, settings.Refused) as e:
        raise SystemExit(f"refused: {e}")


if __name__ == "__main__":
    main()
