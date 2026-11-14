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
a limit and a change of the plant never share a hash.

    uv run python -m oscnb.constants adopt --from SERVO_JSON [--origin TEXT]
    uv run python -m oscnb.constants status
    uv run python -m oscnb.constants check
    uv run python -m oscnb.constants install [--protection NAME ...] [--loosen NAME ...]

`adopt` writes the first set of a servo that already runs, every field
`adopted`, from a record of its state (`oscnb.scorecard.servo_state`,
the `servo.json` of a scorecard run). The record is not kept: the set
names its sha1, the hash of the state and where it came from in words.
`status` is offline: the sha1 of every input file against the file now,
a moved one named with the fields that list it. `check` reads the servo
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
(`postable`'s own steps, every page read back), the other fields in
table order, each read back, a write the servo refuses tried once more
after the rest. The whole state is then read back, and only when it is
the set does `osc stamp` bless it, no save, after which the data state
must read clean. A refusal after the first write reboots the servo,
which returns its saved state.
"""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

from . import osc, postable, results, settings, widecodes
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
TOP = ("format", "pair", "inputs", "origins", "wide_codes", "pos_table", "fields", "hash", "previous")
BLOCKS = ("inputs", "origins", "fields")    # written an entry a line
LEAD = ("value", "slots", "state", "points_sha1", "class")
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
    adopted value whose origin the file does not hold, or hashes that
    are not those of its content."""
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
    table, slots, h = doc["pos_table"], doc["wide_codes"], doc["hash"]
    return [
        f"{len(fields)} fields: {by_class} (no source); {unknown} per-count at an unknown scale",
        f"position table {table['state']}, points sha1 {table['points_sha1']} [{table['class']}]",
        "wide-code slots: none" if slots is None
        else f"wide-code slots {bytes(slots['slots']).hex(' ')} [{slots['class']}]",
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
        fields[name] = {"value": record["fields"][name], **own}
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


# --- status ---

def moved(doc, root=results.ROOT):
    """The input files that are no longer the ones the set read, each as
    (input, file, its sha1 now or None when it is gone, the fields that
    list the input)."""
    out = []
    for name in sorted(doc["inputs"]):
        inp = doc["inputs"][name]
        if "file" not in inp:
            continue
        p = Path(root) / inp["file"]
        now = sc.file_sha1(p) if p.exists() else None
        if now != inp["sha1"]:
            out.append((name, inp["file"], now, [n for n, e in doc["fields"].items()
                                                 if any(i.split(":")[0] == name for i in e.get("inputs", ()))]))
    return out


def status(doc, root=results.ROOT):
    """(lines, stale): the set against its input files as they are now,
    a moved or missing one named with the fields that list it, then what
    the set holds and where its adopted values came from."""
    stale = moved(doc, root)
    lines = [f"stale: {name} ({file}) is {'gone' if now is None else f'no longer the file the set read, sha1 {now}'}"
             f": {', '.join(users) or 'no field lists it'}" for name, file, now, users in stale]
    files = sum("file" in i for i in doc["inputs"].values())
    if not stale:
        lines.append(f"{files} input files, none moved")
    lines += described(doc)
    for name in sorted(doc["origins"]):
        o = doc["origins"][name]
        lines.append(f"adopted from {name}: {o['from']} (record sha1 {o['sha1']}, state hash {o['state_hash'][:12]})")
    return lines, bool(stale)


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


def table_to_write(doc, state, root=results.ROOT):
    """The Table an install has to write, None when the servo holds the
    set's. Only a LIVE table can be written, from the pair's table file,
    which must hold the set's points, be built against the set's stops,
    and name no other slots than the set's as the ones its lookup was
    fitted on; a file that records none is said and taken."""
    want, have = doc["pos_table"], state["pos_table"]
    if all(have[k] == want[k] for k in ("state", "points_sha1")):
        return None
    if want["state"] != "LIVE":
        raise settings.Refused(f"the set's table is {want['state']} (points sha1 {want['points_sha1']}) and the "
                               f"servo's {have['state']} ({have['points_sha1']}): an install writes a LIVE table "
                               f"only")
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
    stops = [doc["fields"][n]["value"] if n in doc["fields"] else state["fields"].get(n) for n in STOPS]
    if stops != [t.raw_min, t.raw_max]:
        raise settings.Refused(f"{sc.shown(path)} was built against stops {t.raw_min} and {t.raw_max}, the set's "
                               f"are {stops[0]} and {stops[1]}")
    return t


def install(doc, protection=(), loosen=(), root=results.ROOT):
    """Make the servo equal the set, volatile; prints each write as it
    lands and returns what was written, by name, in order. Nothing is
    written unless torque is off, no input file of the set moved, the
    servo lets a host write every field the set holds, the table can be
    installed (`table_to_write`) and the protection gate passes
    (`gate`). The writes are the slots, the stops, the table, then the
    other fields that differ in table order, each read back, a field the
    servo refuses tried once more after the rest. The state read back
    whole must then be the set, and only then is it stamped, no save,
    and its data state required clean. A refusal after the first write
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
    table = table_to_write(doc, state, root)
    differ = [n for n, e in doc["fields"].items() if state["fields"][n] != e["value"]]
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
            print(f"position table: {state['pos_table']['state']} -> LIVE, {postable.PAGES} pages read back")
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


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.constants", description=__doc__.split("\n\n")[0])
    ap.add_argument("--pair", default=postable.default_pair(), help="servo__board, the default pair if omitted")
    sub = ap.add_subparsers(dest="cmd", required=True)
    ad = sub.add_parser("adopt", help="the first set of a servo that already runs, every field adopted")
    ad.add_argument("--from", dest="record", type=Path, required=True,
                    help="a record of the servo's state, a scorecard run's servo.json")
    ad.add_argument("--origin", default=BEFORE, help="where the record came from, in words the set keeps")
    sub.add_parser("status", help="offline: the set's input files against the files now")
    sub.add_parser("check", help="the servo read against the set, a line a difference; changes nothing")
    ins = sub.add_parser("install", help="make the servo equal the set, volatile, verified, then stamped")
    ins.add_argument("--protection", nargs="+", default=[], metavar="NAME",
                     help="the protection fields the install may change")
    ins.add_argument("--loosen", nargs="+", default=[], metavar="NAME",
                     help="of those, the ones it may set to protect less than the servo's value")
    a = ap.parse_args(argv)
    try:
        if a.cmd == "adopt":
            doc, path = adopt(a.pair, a.record, a.origin)
            print("\n".join([f"wrote {sc.shown(path)}", *described(doc)]))
        elif a.cmd == "status":
            lines, stale = status(of_record(a.pair))
            print("\n".join(lines))
            if stale:
                raise SystemExit(1)
        elif a.cmd == "check":
            if check(of_record(a.pair)):
                raise SystemExit(1)
        else:
            doc = of_record(a.pair)
            wrote = install(doc, a.protection, a.loosen)
            print("\n".join([*described(doc),
                             "the servo holds the set: nothing written" if not wrote
                             else "volatile, not saved: `osc reboot` returns the saved state"]))
    except (ValueError, settings.Refused) as e:
        raise SystemExit(f"refused: {e}")


if __name__ == "__main__":
    main()
