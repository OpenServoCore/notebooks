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
"""
import argparse
import json
from pathlib import Path

from . import postable, results, settings, widecodes
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
# train and the rail, by the descriptor's names.
PROTECTION = frozenset((
    "current_limit_counts", "stall_tau_trip_counts",                # the current limit and trip
    "i_cont_counts", "stall_release_counts",                        # the continuous current and its release
    "stall_response", "stall_time_ticks", "stall_omega_max_cps",    # the stall policy, its time and its speed
    "oc_trip_counts", "oc_trip_ticks",                              # the overcurrent pair
    "duty_max_q15",                                                 # the duty ceiling
    "derate_start_cc", "cutoff_cc",                                 # the derate and the cutoff
    "v_undervolt_counts", "rail_fold_q8",                           # the undervolt floor and the rail fold
    "pos_error_counts", "pos_error_time_ticks",                     # the position-error fault
    "sensor_delta_max", "sensor_bad_count",                         # the sensor screen
    "ntc_raw_ref", "ntc_t_ref_cc", "ntc_k1_q88", "ntc_k2_q24",      # the NTC curve
    "thermal_tau_s", "thermal_tau2_s", "th_rise_q016", "th_rise2_q016",     # the thermal model
))

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
    if record.get("hash") != sc.config_hash({k: record[k] for k in sc.HASHED}):
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
        elif check(of_record(a.pair)):
            raise SystemExit(1)
    except (ValueError, settings.Refused) as e:
        raise SystemExit(f"refused: {e}")


if __name__ == "__main__":
    main()
