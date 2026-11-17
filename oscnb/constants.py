"""The constant set of a servo and board pair: one generated file that
says what the servo is to hold, and the verbs that read against it.

stages/5-closed-loop/calibration/<pair>.constants.json holds every
CONFIG and CALIB field a host may write past the bus front, each with
its value as `osc dump` prints it, its class and where the value came
from, with the ADC wide-code slots, the position table's state and the
sha1 of its points, and three hashes. It is generated, never typed: the
same inputs give the same bytes, and `load` refuses a file whose hashes
are not those of its content.

A field has one class, which says where its value comes from.
`measured` is a rule over results of record, `bench` a recorded bench
result on this servo, read from the record and never typed,
`datasheet` a value the maker of the servo or of a part states, a
rating or a nominal and no measurement, with the document named and
read from where the repo records it, `policy` a rule over a decision
or over a target a bench sweep picked, `pinned` a literal typed for
the pair, and `none`
a value with no source. A value has none when the servo held it before
its set existed (`origin`), when it is carried while its rule may not
bind (`held`), or when its rule reads a policy entry nobody decided;
what the servo happened to hold, an old tool's output, a boot default
and zero are no source. A field with none says why (`reason`) and what
would give it one (`needs`) where the pair's sources file or the policy
states them, and where a maker's value the repo records could supply
it, which one (`datasheet_candidate`): a candidate, not its source.
Every verb prints the count per class, and `status` lists
the fields with none. A field with no source that carries a position
unit (PER_COUNT) was fitted at a scale nothing recorded, so it is
marked as at an unknown scale and counted too.

`hash.protection` covers the values of the protection fields
(PROTECTION), `hash.plant` every other field's value with the table's
state and points and the slots, and `hash.set` the two, so a change of
a limit and a change of the plant never share a hash. Under `scales`
the set carries what a capture of the pair is scored with (SCALES), the
shunt's counts per A from the board's stage 1 result. A scale is on no
servo, so it is in neither of the two and `hash.scales` covers it
alone: a scale that moves leaves `hash.set` where it was. A field's entry
says which it is under `protection`: null for the plant, the way the
value moves to protect less for a protection field, as LOOSER has it.

Under `limits` the set carries a bound on its own fields that the
control table has no field for yet. The first is the speed ceiling
(`speed_ceiling_cps`), the most the output is ever to turn at, which
protects the brushes: the no-load speed the maker of the servo's class
states at the sheet's operating voltage, times the derate `policy.json`
decides for this servo, in deg/s and in the counts per second of the
set's scale. Where the sheet states a range of operating voltage, the
policy says which stated voltage of it the speed is read at
(`datasheet_speed_voltage`): a number of volts, or the registry's name
of the voltage the servo runs on, the stated one nearest which is
taken. What a maker states of a class is a record beside the
sets, `<class>.datasheet.json`, with the document, where it was read
and when; the registry names the class a servo is of, and a servo that
names none has no limit from a sheet. A limit is on no servo, so
`hash.limits` covers it alone, and a set whose field is over the limit
that bounds it (BOUNDS: the move's speed limit, `velocity_limit_cps`,
under the speed ceiling) is refused by `build` and by `load`. `status`
prints the ceiling with its source.

    uv run python -m oscnb.constants adopt --from SERVO_JSON [--origin TEXT]
    uv run python -m oscnb.constants build [--out PATH [--deviation FILE]] [--protection NAME ...]
    uv run python -m oscnb.constants status
    uv run python -m oscnb.constants check
    uv run python -m oscnb.constants install [--protection NAME ...] [--loosen NAME ...]
    uv run python -m oscnb.constants save [--protection NAME ...] [--loosen NAME ...]
    uv run python -m oscnb.constants restore --from STATE_JSON [--protection NAME ...] [--loosen NAME ...]

`status`, `check` and `install` read the pair's set of record, or the
derived set named with `--set PATH` ahead of the verb.

`adopt` writes the first set of a servo that already runs, every field
of class `none` and adopted from a record of its state
(`oscnb.scorecard.servo_state`, the `servo.json` of a scorecard run).
The record is not kept: the set names its sha1, the hash of the state
and where it came from in words.

`build` is offline and writes the set from the results of record, on
top of the pair's set of record, which it replaces, or to `--out` as a
derived set. A field is bound by a rule (`rule`, RULES), a function
that reads only through a Context: a result's quantity, the registry,
the policy file, the scale of the count. A field may have several
rules, one a way its quantity is on record, and the policy file names
under `rule` the one that binds; with none named the field is held.
Each read is recorded, so a field's `inputs` are what its rule read,
and the set's `inputs` name every file by its sha1. One encoder rounds
a rule's value
to the field's Q format and refuses one that does not fit; it never
clamps. A rule's band is the rule again at each end of every banded
input. The order is the one a count is defined in: the slots from the
board's wide-code result, then the table from the pair's table file,
which must be built against the set's stops and fitted on the set's
slots, then the scale (the table's counts per deg, the raw pot's on a
pair with no table), then the fields, a per-count one naming the table
it was computed for. A field with no rule, no pin and no record is
carried at its value, with no source.
`policy.json`, beside the sets and one for every pair, holds the
choices that are no measurement, in physical units: under `why` the
reason of each one that is a decision, or the pick of a bench sweep
under a rule written before its session with the sweep's rungs named,
under `undecided` the entries
that restate a default nobody decided, each with its `reason` and what
it `needs`, so a field whose rule reads one has no source, and under
`hold` the fields it holds back with the reason: a held field stays at
the value the set had, with what its rule gives beside it (`would_be`)
and what the rule read still listed, and one the set never held stays
out of it, said with what its rule gives; `<pair>.pins.json` the
literals typed for one pair, each with its why and the rung that chose
it. `<pair>.sources.json` says of one pair's fields what no rule can:
under `bench` the fields whose value a bench record holds, each with
the record's file in the repo, the key of the value in it and what the
record is, a scorecard rung's results file or a dated reading, and a
`weak` line where the record awaits a better one; under `datasheet`
the fields whose value a maker states, each the same way with the
`document` named; under `none` the fields with no source, each with
its `reason`, what it `needs` and its `datasheet_candidate` where it
has one. A bench or datasheet field takes the record's value, the
record's file is among the set's inputs, and a record that moves makes
the set stale. A pin stands ahead of a record, and a record ahead of a
rule. A
rule whose
input is missing does not guess: the field is held at the value the
set had, the missing input named, or left out and said where there is
nothing to carry. A result that names a file it read which has moved
since is stale, and refused; one written before results named what
they read is said, not refused. One build moves one hash: the plant by
default, every protection field carried with what its rule would give
beside it (`would_be`), or with `--protection` the fields named and
nothing of the plant. A protection field a firmware change took out of
the table (RETIRED) is carried the same way until a build names it with
`--protection`, which leaves it out; a set written while the field
stood loads as it was written.

A deviation is one change to a set, for a rung: a file of policy
entries, pins and holds to release, `{"policy": {...}, "pins": {field:
{"value", "why", "rung"}}, "release": [field, ...]}`, never a list of
raw values. `build --deviation FILE --out PATH` (`derive`) builds the
set of record again with the file's entries over the policy file's and
its pins over the pair's, so a policy entry moves what its rules give,
at the set's scale, and with each hold it releases bound at what the
field's rule gives, the rest of the policy as it stands. A release of
a field that is not held, or whose rule has no value, is refused. The
derived set names the file among its inputs and, under `deviation`,
every field it moved as [of record, deviated]. It is refused when the
set of record is not what a build of its inputs gives now, when it
moves nothing, and when it moves a protection field and anything else.
The protection fields a deviation moves are of one step (GROUPS: the
words of the NTC curve, the current limits, the four fields of the
winding model, the position-error screen's angle and time, any other
field alone), and a release of one held field of a step releases them
all. A deviation that moves the winding model
is said with what the estimate reads under the four held and under the
four wanted (`winding_step`).
A derived set is never a set of record and is never saved.

`status` is offline: the sha1 of every input file against the file now,
and the registry values read against the registry, a moved one named
with the fields that list it, then the count per class and every field
with no source, with its reason, what would give it one and the
maker's value that could where there is one, and the fields whose
record is a weak one. `check` reads the servo
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
no one direction says it for is always taken as protecting less). An
install that would change the winding model's four prints, with the
gate's lines or ahead of its writes, what the estimate reads under the
servo's four and under the set's (`winding_step`). Then,
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

from . import boards, osc, postable, results, servos, settings, widecodes, winding
from . import scorecard as sc

FORMAT = 1
CLASSES = ("measured", "bench", "datasheet", "policy", "pinned", "none")
# The two classes of a set written before a class said where a value
# comes from. Neither named a source, so such a set loads as it was
# written and its fields of either count with `none`.
WAS = ("held", "adopted")
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
    # the one thermal limit: the overtemperature fault latches while the winding estimate reads it or more
    "cutoff_cc": "up",
    # the fault under the floor; the slope of the fold over it, where 0 turns the fold off
    "v_undervolt_counts": "down", "rail_fold_q8": "judged",
    "pos_error_counts": "up", "pos_error_time_ticks": "up",
    "sensor_delta_max": "up", "sensor_bad_count": "up",
    # one curve of four words: a coefficient moves the reading up on one side of the reference, down on the other
    "ntc_raw_ref": "judged", "ntc_t_ref_cc": "judged", "ntc_k1_q88": "judged", "ntc_k2_q24": "judged",
    # a longer leak heats later and cools later, and a body time of 0 drops the body node; a rise adds to the estimate
    "thermal_tau_s": "judged", "thermal_tau2_s": "judged", "th_rise_q016": "down", "th_rise2_q016": "down",
}

# The protection fields a firmware change took out of the table, each
# with the way it moved to protect less while it stood. A set written
# before holds one under the protection hash, so the set a rung of
# record ran on loads as it was written. A build carries one until it is
# named with `--protection`, and then leaves it out.
RETIRED = {
    # where the winding estimate began to fold the current limit, under the cutoff
    "derate_start_cc": "up",
}
PROTECTION = frozenset((*LOOSER, *RETIRED))

# The protection fields that are one step together: the four words of
# the NTC curve, which are one curve, the current limits, which one
# shunt scale takes from amps to counts, the four fields of the
# winding estimate, which are one model of the winding's heat, a leak
# and a rise for each of its two nodes, and the position-error screen,
# which is one test, an angle held for a time. A deviation moves the
# protection fields of one group, a field in none being a group of its
# own, and it releases a group's holds together or not at all.
MODEL = "winding model"
GROUPS = {
    "ntc curve": ("ntc_raw_ref", "ntc_t_ref_cc", "ntc_k1_q88", "ntc_k2_q24"),
    "current scale": ("current_limit_counts", "i_cont_counts", "stall_release_counts", "oc_trip_counts"),
    MODEL: ("thermal_tau_s", "th_rise_q016", "thermal_tau2_s", "th_rise2_q016"),
    "position error": ("pos_error_counts", "pos_error_time_ticks"),
}

# The fields whose value is per position count, or a position in counts:
# a table that changes what a count means changes what they mean.
PER_COUNT = frozenset((
    "pos_min_soft_counts", "pos_max_soft_counts", "v_kp_q88", "v_ki_q412", "j_ff_q88", "velocity_limit_cps",
    "accel_limit_q88", "stall_omega_max_cps", "pos_error_counts", "l3_q88", "recip_ke_q", "b_i_q313",
    "fric_fv_q016", "ke_vpc_q",
))

ORIGIN = "servo-state"      # the origin a value adopted from a record of the servo's state names
BEFORE = "the servo's state before the path"
TOP = ("format", "pair", "inputs", "origins", "wide_codes", "pos_table", "scale", "scales", "limits", "fields",
       "hash", "previous", "deviation")
BLOCKS = ("inputs", "origins", "scales", "limits", "fields")    # written an entry a line
LEAD = ("value", "slots", "state", "kind", "counts_per_deg", "points_sha1", "class", "reason", "needs",
        "datasheet_candidate")
TABLE_STATE, TABLE_POINTS, SLOTS = "position table state", "position table points sha1", "wide-code slots"
STOPS = ("raw_min", "raw_max")      # the stops a table is taken against, written ahead of it
# what the pair's sources file says of a field, by the block it is under: a record of the repo for the two
# classes a record gives, the maker's document with it for a datasheet value, and why a field has no source
SOURCES = {"bench": ("record", "key", "what"), "datasheet": ("record", "key", "what", "document"),
           "none": ("reason", "needs")}


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


def group_of(name):
    """(the group protection field `name` moves in, its fields): one of
    GROUPS, or the field alone."""
    return next(((g, fields) for g, fields in GROUPS.items() if name in fields), (name, (name,)))


def way(name):
    """The way protection field `name` moves to protect less: LOOSER's,
    or RETIRED's for a field the firmware no longer names."""
    return LOOSER[name] if name in LOOSER else RETIRED[name]


def protection_of(name):
    """What a field's entry says of protection: None for a field of the
    plant, the way its value moves to protect less for a protection
    field. LOOSER and RETIRED are the source; the file repeats it to be
    read."""
    return {"looser": way(name)} if name in PROTECTION else None


def hashes(doc):
    """The hashes of a set. `set` covers `plant` and `protection`, what
    the servo is to hold. `scales`, on a set that holds any, covers the
    values a capture of the servo is scored with: they are on no servo,
    so a scale that moves leaves `set` where it was. `limits`, the same
    way, covers the bounds no field of the table holds yet."""
    plant, protection = (sc.config_hash(x) for x in hashed(doc))
    out = {"plant": plant, "protection": protection,
           "set": sc.config_hash({"plant": plant, "protection": protection})}
    if doc.get("scales"):
        out["scales"] = sc.config_hash(values(doc["scales"]))
    if doc.get("limits"):
        out["limits"] = sc.config_hash(values(doc["limits"]))
    return out


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
    writes one: another format, a field without a value or one class
    (CLASSES, or WAS in a set written before them), an adopted value
    whose origin the file does not hold, a bench or datasheet value
    that names no record, a datasheet value that names no document, a
    field that says another thing of protection than LOOSER, a field a
    rule computed for another table than the set's, a limit that is
    not one as the tool writes it or a field over the limit that bounds
    it (`limits_loaded`), or hashes that are not those of its content."""
    path = Path(path)
    where = sc.shown(path)
    doc = json.loads(path.read_text())
    if doc.get("format") != FORMAT:
        raise ValueError(f"{where}: format {doc.get('format')}, this tool reads {FORMAT}")
    for name, e in (*doc["fields"].items(), *doc.get("scales", {}).items()):
        if "value" not in e or e.get("class") not in (*CLASSES, *WAS):
            raise ValueError(f"{where}: {name} holds no value of one class of {', '.join(CLASSES)}")
    for name, e in (*doc["fields"].items(), (SLOTS, doc["wide_codes"] or {}), ("position table", doc["pos_table"])):
        if "origin" in e and e["origin"] not in doc["origins"]:
            raise ValueError(f"{where}: {name} is adopted from {e['origin']!r}, which the file does not hold")
        if e.get("class") in ("bench", "datasheet") and not (e.get("inputs") and (e["class"] == "bench"
                                                                                   or e.get("document"))):
            raise ValueError(f"{where}: {name} is of class {e['class']} and names no record"
                             f"{' or no document' if e['class'] == 'datasheet' else ''}")
    scale, table = doc.get("scale") or {}, doc["pos_table"]["points_sha1"]
    if scale.get("kind") == "table" and scale["points_sha1"] != table:
        raise ValueError(f"{where}: its scale is that of the table of points sha1 {scale['points_sha1']}, its "
                         f"table is {table}")
    for name, e in doc["fields"].items():
        if "protection" not in e or e["protection"] != protection_of(name):
            raise ValueError(f"{where}: {name} says {e.get('protection', 'nothing')} of protection, the tool "
                             f"{protection_of(name)}")
        if e.get("pos_unit") and e.get("scale_points_sha1") != scale.get("points_sha1"):
            raise ValueError(f"{where}: {name} was computed for the table of points sha1 "
                             f"{e.get('scale_points_sha1')}, the set's scale is that of {scale.get('points_sha1')}")
    limits_loaded(doc, where)
    if doc.get("hash") != hashes(doc):
        raise ValueError(f"{where}: its hashes are not those of its content: a set is generated, never edited "
                         f"(or the protection list moved since it was written)")
    dev = doc.get("deviation")
    if dev is not None and (set(dev) != {"file", "sha1", "fields"} or not dev["fields"]
                            or any(n not in doc["fields"] or doc["fields"][n]["value"] != pair[1]
                                   for n, pair in dev["fields"].items())):
        raise ValueError(f"{where}: its deviation is not the fields it moved, each as [of record, deviated]")
    return doc


def of_record(pair, root=results.ROOT):
    """The pair's set of record, refused when it has none."""
    path = set_path(pair, root)
    if not path.exists():
        raise ValueError(f"no constant set for {pair} ({sc.shown(path)}): `adopt` writes a running servo's first")
    return load(path)


def source(e):
    """The class of an entry as where its value comes from: its class,
    and `none` for the two of a set written before a class said it."""
    return "none" if e["class"] in WAS else e["class"]


def classes(doc):
    """How many fields of a set are of each class."""
    return {c: sum(source(e) == c for e in doc["fields"].values()) for c in CLASSES}


def tagged(e):
    """What a line says of an entry after its value: its class, that it
    is held where it is, and its rule where it has one."""
    held_ = ["held"] if "held" in e and e["class"] == "none" else []
    return ", ".join([e["class"], *held_, *([str(e["rule"])] if "rule" in e else [])])


def unsourced(doc):
    """The fields of a set with no source, in the set's order, each as
    (name, value, why it has none, what would give it one, the maker's
    value that could supply it or None): the reason and the need its
    entry states, and where it states none, what carries the value."""
    out = []
    for name, e in doc["fields"].items():
        if source(e) != "none":
            continue
        if "held" in e:
            why = f"held, {e['held']}"
            needs = f"its hold released: its rule gives {e['would_be']}" if "would_be" in e else "what its rule reads"
        else:
            why = "the servo held it before its set existed" if "origin" in e else "nothing binds it"
            needs = "a rule over a result, a bench record or a decision"
        out.append((name, e["value"], e.get("reason", why), e.get("needs", needs), e.get("datasheet_candidate")))
    return out


def weak(doc):
    """The fields of a set whose record is a weak one, by what the
    record awaits: {what it awaits: [field, ...]}."""
    out = {}
    for name, e in doc["fields"].items():
        if "weak" in e:
            out.setdefault(e["weak"], []).append(name)
    return out


def described(doc):
    """What every verb says of a set: its fields by class, the last the
    ones with no source, the per-count ones at an unknown scale, the
    table and the slots, and the three hashes in short."""
    fields = doc["fields"].values()
    by_class = ", ".join(f"{n} {c}" for c, n in classes(doc).items())
    unknown = sum(e.get("scale") == "unknown" for e in fields)
    table, slots, h, scale = doc["pos_table"], doc["wide_codes"], doc["hash"], doc.get("scale")
    dev = doc.get("deviation")
    return [
        *([f"derived, not a set of record: {dev['file']} moves "
           + ", ".join(f"{n} {a} -> {b}" for n, (a, b) in dev["fields"].items())] if dev else []),
        f"{len(fields)} fields: {by_class} (no source); {unknown} per-count at an unknown scale",
        f"position table {table['state']}, points sha1 {table['points_sha1']} [{table['class']}]",
        "wide-code slots: none" if slots is None
        else f"wide-code slots {bytes(slots['slots']).hex(' ')} [{slots['class']}]",
        *([f"scale: {scale['counts_per_deg']:.4f} counts/deg, the {scale['kind']} one"] if scale else []),
        *(f"scored with {n}: {e['value']:.4f} {e.get('unit', '')} [{e['class']}]".replace("  [", " [")
          for n, e in doc.get("scales", {}).items()),
        f"hash: plant {h['plant'][:12]}, protection {h['protection'][:12]} "
        f"({sum(n in PROTECTION for n in doc['fields'])} fields), set {h['set'][:12]}"
        + "".join(f", {k} {h[k][:12]}" for k in ("scales", "limits") if k in h),
    ]


# --- adopt ---

def adopted(record, pair, origin):
    """The first set of a servo that already runs, from a record of its
    state: every field the record names writable past the bus front, the
    stamp left out, each at the value the record holds and of class
    `none`, adopted from the record, with the table and the slots: a
    value the servo held has no source. `origin` is what the set
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
    own = {"class": "none", "origin": ORIGIN}
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
RULES = {}      # {field: {rule name: Rule}}, in the order the rules are written
POLICY = "policy.json"
ZERO_POINTS = hashlib.sha1(bytes(2 * postable.INTERVALS)).hexdigest()   # the points of an identity table
WIDE = f"{widecodes.STAGE}/{widecodes.EXPERIMENT}"
RAW_SCALE = ("3-servo/pot-table", "pot_scale_counts_per_deg")   # counts per deg of a pair with no table
# the byte of each gear class, as the firmware's client names them (client calib.rs GearClass)
GEAR_CLASSES = ("unknown", "plastic", "metal")
CURRENT = "current_counts_per_a"    # the shunt's counts per A, the scale a capture's current is scored with
# what a set is scored with, each a quantity of a stage 1 result of the board: (result, key, rule)
SCALES = {CURRENT: ("1-board/current-scale", "counts_per_a", "scales.current")}


class Missing(Exception):
    """An input a rule reads is not there; the message says which."""


@dataclasses.dataclass(frozen=True)
class Rule:
    """What binds one field: its name in a set, the field as the
    firmware declares it (`kind` its storage, `unit`, `q` its fraction
    bits, `rev` moved by a firmware change of how the field is used),
    the power of counts per deg its value carries, the class its value
    is of, or a function of what the rule read that gives it, the
    firmware code it mirrors, and the function of a Context that gives
    the value in the field's unit."""
    field: str
    name: str
    kind: str
    unit: str
    q: int
    pos_unit: int
    rev: int
    cls: object
    mirrors: str
    fn: object


def rule(field, name, *, kind, unit, mirrors, q=0, pos_unit=0, rev=1, cls="measured"):
    """Register the function under it as rule `name` of `field`. A
    field may have several, one a way its quantity is on record."""
    def register(fn):
        RULES.setdefault(field, {})[name] = Rule(field, name, kind, unit, q, pos_unit, rev, cls, mirrors, fn)
        return fn
    return register


def rules():
    """Every rule there is, the rules of one field together."""
    return [r for of in RULES.values() for r in of.values()]


def rule_of(c, field):
    """The rule that binds `field`: its one rule, or of several the one
    the policy names for the field under `rule`."""
    of = RULES[field]
    if len(of) == 1:
        return next(iter(of.values()))
    name = c.policies.get("rule", {}).get(field)
    if name is None:
        raise Missing(f"{POLICY} names no rule for {field} of {', '.join(of)}")
    if name not in of:
        raise ValueError(f"{POLICY}: {name} is no rule of {field} ({', '.join(of)})")
    return of[name]


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
    scale of the count (`counts_per_deg`, `travel_deg`, `stops`,
    `ascending`).
    `inputs` gathers what a set names under `inputs`, `notes` what a
    build has to say. A read of something that is not there raises
    Missing. `sources` is the pair's sources file: the fields a record
    gives (`bench`, `datasheet`) and the ones it says have no source
    (`none`). With a `deviation` (`deviation_of`) its policy entries
    stand over the policy file's and its pins over the pair's, the
    holds it releases are `released`, and the file is one of the
    inputs."""

    def __init__(self, pair, root=results.ROOT, registry=None, deviation=None):
        self.pair, self.root = pair, Path(root)
        self.registry = registry_of(pair) if registry is None else registry
        self.inputs, self.notes, self.docs = {}, [], {}
        self.reads, self.side = [], {}
        self.scale = self.table = None
        beside = set_path(pair, root).parent.relative_to(self.root)     # the policy and the pins lie beside the set
        self.policies = self.file("policy", beside / POLICY) or {}
        self.pins = self.file("pins", beside / f"{pair}.pins.json") or {}
        self.sources = self.file("sources", beside / f"{pair}.sources.json") or {}
        self.released = ()
        if deviation is not None:
            dev = deviation_of(deviation)
            self.file("deviation", in_repo(deviation, self.root))
            self.policies = {**self.policies, **dev.get("policy", {})}
            self.pins = {**self.pins, **dev.get("pins", {})}
            self.released = tuple(dev.get("release", ()))
            free = [n for n in self.released if n not in self.policies.get("hold", {})]
            if free:
                raise ValueError(f"{sc.shown(deviation)}: {', '.join(free)} is not held: a deviation releases a "
                                 f"hold of {POLICY}")
        loose = [n for n, pin in self.pins.items() if not all(k in pin for k in ("value", "why", "rung"))]
        if loose:
            raise ValueError(f"the pins of {pair}: {', '.join(loose)} lacks its value, its why or its rung")
        gone = [n for n in self.pins if n in RETIRED]
        if gone:
            raise ValueError(f"the pins of {pair}: the firmware names no {', '.join(gone)}")
        said = [(n, block, e) for block in self.sources for n, e in self.sources[block].items()]
        loose = [n for n, block, e in said
                 if not all(isinstance(e.get(k), str) and e[k] for k in SOURCES.get(block, ("no such block",)))]
        loose += [n for n in dict.fromkeys(n for n, _, _ in said) if sum(n == m for m, _, _ in said) > 1]
        if loose:
            raise ValueError(f"the sources of {pair}: a field is under one of "
                             + "; ".join(f"`{block}` with its {', '.join(keys)}" for block, keys in SOURCES.items())
                             + f", and {', '.join(dict.fromkeys(loose))} is not")

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

    def raw(self, name, *path):
        """What result `name` holds at `path`, a key or the keys down
        to a nested entry, as the file holds it."""
        at = self.result(name)
        for k, key in enumerate(path):
            if not isinstance(at, dict) or key not in at:
                raise Missing(f"{name} has no {'.'.join(path[:k + 1])}")
            at = at[key]
        self.reads.append((f"{name}:{'.'.join(path)}", at))
        return at

    def q(self, name, *path):
        """The value of the quantity result `name` holds at `path`; its
        `lo` or `hi` while the rule is evaluated at that end of its
        band."""
        quantity = self.raw(name, *path)
        return quantity[self.side.get(f"{name}:{'.'.join(path)}", "value")]

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
    def stops(self):
        """The raw counts of the two stops, low then high, which the
        table reads as themselves."""
        if self.table is None:
            raise Missing("the pair has no position table: the stops its count runs between are not known")
        self.reads.append(("pos_table:stops", None))
        return self.table.raw_min, self.table.raw_max

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
    ends = [encode(r, at({name: end}), fit=False) for name, quantity in dict(reads).items()
            if isinstance(quantity, dict) for end in ("lo", "hi") if quantity.get(end) is not None]
    c.side = {}
    inputs = list(dict.fromkeys(name for name, _ in reads))
    e = {"value": value, "class": r.cls(inputs) if callable(r.cls) else r.cls, "rule": r.name, "rule_rev": r.rev,
         "unit": r.unit, "q": r.q, "pos_unit": r.pos_unit, "inputs": inputs}
    if r.pos_unit:
        e["scale_points_sha1"] = c.scale["points_sha1"]
    if ends and not isinstance(value, str):
        e["band"] = [min(value, *ends), max(value, *ends)]
    return e


# The angle map: centi-deg at each stop, rising counterclockwise facing
# the horn, the clockwise stop at the policy's angle and the other one
# the travel past it. The travel is the stops' span at the scale of the
# count, so a host that divides the map's span by the stops' gets the
# table's counts per deg (ident kinematics.rs deg_per_count). The end
# at the clockwise stop is the policy's angle and reads no travel; the
# other one is measured, the travel being the table's.
def by_travel(inputs):
    return "measured" if "scale" in inputs else "policy"


@rule("angle_min_cdeg", "angle_min.table", kind="i16", unit="cdeg", pos_unit=-1, cls=by_travel,
      mirrors="core regions/calib.rs CalibKinematics, tools/osc cal/mod.rs rail_angles")
def angle_min(c):
    return 100 * (c.policy("angle_cw_stop_deg") + (0 if c.ascending else c.travel_deg))


@rule("angle_max_cdeg", "angle_max.table", kind="i16", unit="cdeg", pos_unit=-1, cls=by_travel,
      mirrors="core regions/calib.rs CalibKinematics, tools/osc cal/mod.rs rail_angles")
def angle_max(c):
    return 100 * (c.policy("angle_cw_stop_deg") + (c.travel_deg if c.ascending else 0))


# The registry's gear ratio is a choice of which count stands for the
# unit, the family's tooth count where the unit's own teeth are not
# counted, so the field is of class `policy`. Its gear class is read off
# the part, and the registry's entry is that bench record.
@rule("gear_ratio_centi", "gear_ratio.registry", kind="u16", unit="motor rev per output rev x100", cls="policy",
      mirrors="core regions/calib.rs CalibKinematics, tools/osc cal/mod.rs ratio_to_centi")
def gear_ratio(c):
    return 100 * c.reg("servo", "gear_ratio")


@rule("gear_class", "gear_class.registry", kind="u8", unit="class", cls="bench",
      mirrors="client calib.rs GearClass, tools/osc main.rs dump")
def gear_class(c):
    name = c.reg("servo", "gear_class")
    if name not in GEAR_CLASSES[1:]:
        raise ValueError(f"gear_class: the registry's {name!r} is no gear class of {', '.join(GEAR_CLASSES[1:])}")
    return f"{GEAR_CLASSES.index(name)} ({name})"


# --- the motor block, the feedforwards and the observer gains ---
#
# The kernel works in counts: tap counts (vcounts), shunt counts
# (ccounts), position counts, a MEDIUM tick. A rule takes the SI
# quantity of record there through the board's stage 1 scales, the gear
# and the scale of the count:
#   R  vcounts/ccount            = R [ohm] x tap counts per V / shunt counts per A
#   Ke vcounts/(count/s)         = Ke [V s/rad] x tap counts per V / counts per motor rad
#   B  (count/s)/tick per ccount = Ke [N m/A] / shunt counts per A / J x counts per motor rad / MEDIUM rate
#   Fc ccounts                   = Tc / Ke x shunt counts per A
#   Fv ccounts/(count/s)         = b / Ke x shunt counts per A / counts per motor rad
# The inertia feedforward and the observer's gains follow from B and the
# observer's bandwidth the way the firmware repo's own synthesis has
# them (ident/src/gains.rs synthesize), which the parity test holds.

DECIM_MED = 10      # FAST ticks a MEDIUM period (core kernel/scheduler/table.rs DECIM_MED)
LADDER, FRICTION = "3-servo/ladder", "3-servo/friction"
CHAIN = "1-board/terminal-chain"    # the board's tap, rail and supply scales
# the winding's resistance as it is on record, by the way it was taken: (result, key)
RESISTANCES = {
    "rails_fit": ("3-servo/resistance", "r_rails_at_measured_e_ohm"),
    "locked_laps": ("3-servo/resistance", "r_term_ohm"),
    "small_step": ("3-servo/steps", "r_step_a_ohm"),
    "take_back": ("3-servo/steps", "r_takeback_ohm"),
}


def counts_per_motor_rad(c):
    return c.counts_per_deg * 180 / math.pi / c.reg("servo", "gear_ratio")


def tap_counts_per_v(c):
    return c.q(CHAIN, "terminal_counts_per_v")


def shunt_counts_per_a(c):
    return c.q(*SCALES[CURRENT][:2])


def medium_hz(c):
    return c.reg("board", "tick_hz") / DECIM_MED


def ke_v_s_per_rad(c):
    """The back-EMF constant, V s/rad, which is the torque constant in
    N m/A."""
    return c.q(LADDER, "ke_v_s_per_rad")


def ke_vcounts(c):
    """Ke, tap counts per count/s."""
    return ke_v_s_per_rad(c) * tap_counts_per_v(c) / counts_per_motor_rad(c)


def b_gain(c):
    """B, the speed a shunt count of current adds in a MEDIUM tick,
    (count/s) per tick per ccount."""
    return (ke_v_s_per_rad(c) / shunt_counts_per_a(c) / c.q("3-servo/run-up", "j_kg_m2")
            * counts_per_motor_rad(c) / medium_hz(c))


def friction(c, key):
    """The friction line the firmware's model takes, SI: the pooled fit
    with the bounds the friction result gives that pair."""
    return c.q(FRICTION, "firmware_pair", "pooled", key) * 1e-6


def observer_pole(c):
    """The observer's three coincident poles, at the policy's bandwidth
    on the MEDIUM tick."""
    return math.exp(-2 * math.pi * c.policy("observer_hz") / medium_hz(c))


def resistance(result, key):
    def r_vcounts(c):
        return c.q(result, key) * tap_counts_per_v(c) / shunt_counts_per_a(c)
    return r_vcounts


# One quantity, two fields: r_q12 is the resistance the back-EMF boxcar
# subtracts and the one the stall-safe open-loop duty is sized from;
# r_bemf_q12 is the boxcar's alone, on an image that gives it a field.
# Each way the resistance is on record is a rule of both, and the policy
# names the one that binds each.
for _field, _lead, _mirrors in (
        ("r_q12", "r", "core estimator/bemf.rs close_half, kernel/phases/limits.rs limit"),
        ("r_bemf_q12", "r_bemf", "core estimator/bemf.rs close_half, the resistance of its subtract")):
    for _way, _source in RESISTANCES.items():
        rule(_field, f"{_lead}.{_way}", kind="u16", unit="vcounts/ccount", q=12, mirrors=_mirrors)(
            resistance(*_source))


@rule("recip_ke_q", "recip_ke.ladder", kind="u16", unit="(count/s)/vcount", q=10, pos_unit=+1,
      mirrors="core estimator/bemf.rs close_half")
def recip_ke(c):
    return 1 / ke_vcounts(c)


@rule("b_i_q313", "b_i.run_up", kind="u16", unit="(count/s)/tick per ccount", q=13, pos_unit=+1,
      mirrors="core estimator/fusion.rs step")
def b_i(c):
    return b_gain(c)


@rule("fric_fc_counts", "fric_fc.friction", kind="u16", unit="ccounts",
      mirrors="core estimator/fusion.rs fric_c, kernel/phases/loops/velocity.rs fric_ff")
def fric_fc(c):
    return friction(c, "coulomb_uNm") / ke_v_s_per_rad(c) * shunt_counts_per_a(c)


@rule("fric_fv_q016", "fric_fv.friction", kind="u16", unit="ccounts/(count/s)", q=16, pos_unit=-1,
      mirrors="core kernel/phases/loops/velocity.rs fric_ff")
def fric_fv(c):
    return (friction(c, "viscous_uNm_s_per_rad") / ke_v_s_per_rad(c) * shunt_counts_per_a(c)
            / counts_per_motor_rad(c))


@rule("ke_vpc_q", "ke_vpc.ladder", kind="u16", unit="vcounts/(count/s)", q=12, pos_unit=-1,
      mirrors="core kernel/phases/loops/current.rs u_ff_vc")
def ke_vpc(c):
    return ke_vcounts(c)


@rule("j_ff_q88", "j_ff.run_up", kind="u16", unit="ccounts/((count/s)/tick)", q=8, pos_unit=-1,
      mirrors="core kernel/phases/loops/velocity.rs step, ident gains.rs synthesize")
def j_ff(c):
    return 1 / b_gain(c)


@rule("l1_q016", "l1.observer", kind="u16", unit="1", q=16, cls="policy",
      mirrors="core estimator/fusion.rs step, ident gains.rs synthesize")
def l1(c):
    return 1 - observer_pole(c) ** 3


@rule("l2_q88", "l2.observer", kind="u16", unit="(count/s)/count", q=8, cls="policy",
      mirrors="core estimator/fusion.rs step, ident gains.rs synthesize")
def l2(c):
    pole = observer_pole(c)
    return 1.5 * (1 - pole) ** 2 * (1 + pole) * medium_hz(c)


@rule("l3_q88", "l3.observer", kind="u16", unit="ccounts/count", q=8, pos_unit=-1, cls="policy",
      mirrors="core estimator/fusion.rs step, ident gains.rs synthesize")
def l3(c):
    return (1 - observer_pole(c)) ** 3 * medium_hz(c) / (2 * b_gain(c))


# --- the current loop ---
#
# The firmware repo's own placement (ident/src/gains.rs synthesize): the
# proportional gain cancels the winding's electrical pole, w x L, the
# integral gain sets the crossover, w x R a FAST tick, and the
# anti-windup gain is twice the integral one. w is the policy's
# bandwidth, L the stage 3 locked bursts' inductance and R the
# resistance of the rails fit, the winding's, each in counts through the
# board's stage 1 scales as that file's l_cd_from_si takes them.
INDUCTANCE = ("3-servo/inductance", "l_mh")


def loop_rad_s(c):
    return 2 * math.pi * c.policy("current_loop_hz")


def loop_ki(c):
    """The integral gain, vcounts per ccount a FAST tick."""
    return loop_rad_s(c) * resistance(*RESISTANCES["rails_fit"])(c) / c.reg("board", "tick_hz")


@rule("i_kp_q88", "i_kp.bandwidth", kind="u16", unit="vcounts/ccount", q=8, cls="policy",
      mirrors="core kernel/phases/loops/current.rs step, ident gains.rs synthesize, l_cd_from_si")
def i_kp(c):
    return loop_rad_s(c) * c.q(*INDUCTANCE) / 1000 * tap_counts_per_v(c) / shunt_counts_per_a(c)


rule("i_ki_q412", "i_ki.bandwidth", kind="u16", unit="vcounts/ccount per tick", q=12, cls="policy",
     mirrors="core kernel/phases/loops/current.rs step, ident gains.rs synthesize")(loop_ki)
rule("i_kaw_q412", "i_kaw.bandwidth", kind="u16", unit="vcounts/ccount per tick", q=12, cls="policy",
     mirrors="core kernel/phases/loops/current.rs step, ident gains.rs synthesize")(lambda c: 2 * loop_ki(c))


# --- the records a host keeps on the servo ---
#
# Three fields the kernel never reads, each a quantity of record a host
# reads back: the winding's cold resistance, at the 25 C the field is
# stated at, the back-EMF constant in uV s/rad, and the static friction
# as a current. The resistance of the rails fit is the winding's at the
# board temperature its result states, the pair the identification
# takes together (oscnb.speed.motor_of), and goes to 25 C along
# copper's slope (oscnb.winding.r_of_t).
#
# The supply word, vdd_mv, is no such record and has no rule. It is a
# word of the sense block a capture carries in its meta.json and is
# matched to its board by (oscnb.boards.Board.sense), so it is the
# board entry's nominal supply, typed for the pair as a pin. The supply
# as measured is the stage 1 terminal chain's `vdd_v` and stays in that
# result: a set that binds it writes a servo whose captures match no
# board entry.
R0_AT_C = 25.0      # C, where the cold resistance is stated (core regions/calib.rs CalibWinding)


@rule("r0_q12", "r0.rails_fit", kind="u16", unit="vcounts/ccount", q=12, mirrors="core regions/calib.rs CalibWinding")
def r0(c):
    return winding.r_of_t(R0_AT_C, resistance(*RESISTANCES["rails_fit"])(c), c.q("3-servo/resistance", "board_ntc_c"))


@rule("ke_uvs_per_rad", "ke_uvs.ladder", kind="u16", unit="uV s/rad", mirrors="core regions/calib.rs CalibMotorExt")
def ke_uvs(c):
    return ke_v_s_per_rad(c) * 1e6


@rule("fric_breakaway_counts", "fric_breakaway.static", kind="u16", unit="ccounts",
      mirrors="core regions/calib.rs CalibMotorExt")
def fric_breakaway(c):
    return c.q(FRICTION, "load_static_friction_uNm_motor") * 1e-6 / ke_v_s_per_rad(c) * shunt_counts_per_a(c)


# --- the fields in counts of position ---
#
# The kernel's estimate and its profile run in the count of the scale:
# the table's count, the raw pot's on a pair with no table (core
# kernel/phases/loops/trajectory.rs, limits.rs, phases/rail.rs). A
# speed, an acceleration and an angle the policy states in degrees are
# that many counts there, so each rule carries the scale once. The soft
# limits are positions and no lengths: a share of the travel inside
# each stop, on a count that reads the stops as themselves, so they
# take the stops and not the scale. Two fields are in the raw counts of
# the pot whatever the table, and carry no scale: the hold band, which
# the kernel takes through the table's local gain itself (core
# kernel/phases/loops/position.rs band_q16), and the sensor screen,
# which reads the pot sample before any table (core
# kernel/phases/loops/faults.rs sensor_sample).
POT = "3-servo/pot-table"


def in_counts(key, per_tick=False):
    """A length, a speed or an acceleration the policy states in deg,
    in counts at the scale of the count; an acceleration, `per_tick`,
    as the speed it adds in one MEDIUM period."""
    def counts(c):
        return c.policy(key) * c.counts_per_deg / (medium_hz(c) if per_tick else 1)
    return counts


def soft_limit(high):
    """A soft limit, counts: the policy's share of the stops' span
    inside the low stop, or inside the high one."""
    def counts(c):
        share = c.policy("soft_margin_share")
        lo, hi = c.stops
        return hi - share * (hi - lo) if high else lo + share * (hi - lo)
    return counts


rule("velocity_limit_cps", "velocity_limit.policy", kind="u16", unit="count/s", pos_unit=+1, cls="policy",
     mirrors="core kernel/phases/loops/trajectory.rs vel_cap")(in_counts("velocity_limit_deg_s"))
rule("accel_limit_q88", "accel_limit.policy", kind="u16", unit="(count/s)/tick", q=8, pos_unit=+1, cls="policy",
     mirrors="core kernel/phases/loops/trajectory.rs step_position")(in_counts("accel_limit_deg_s2", per_tick=True))
# the limits of motion, which move together, each with the policy entry its rule reads
MOTION = {"velocity_limit_cps": "velocity_limit_deg_s", "accel_limit_q88": "accel_limit_deg_s2"}


def motion(doc, speed_deg_s, accel_deg_s2, root=results.ROOT, registry=None):
    """The entries of the limits of motion (MOTION) that their rules
    give at a speed limit, deg/s, and an acceleration limit, deg/s^2,
    on set `doc`'s count, the policy file's entries and holds for the
    two set aside."""
    c = Context(doc["pair"], root, registry)
    c.scale = doc.get("scale")
    c.policies = {**c.policies, MOTION["velocity_limit_cps"]: speed_deg_s, MOTION["accel_limit_q88"]: accel_deg_s2}
    return {n: {**evaluate(rule_of(c, n), c), "protection": protection_of(n)} for n in MOTION}


def motion_deg(doc, values, root=results.ROOT, registry=None):
    """(deg/s, deg/s^2): what the limits of motion `values`, {field: the
    value as it holds it}, are on set `doc`'s count, the acceleration
    the speed one MEDIUM tick adds at the MEDIUM rate."""
    c = Context(doc["pair"], root, registry)
    c.scale = doc.get("scale")
    q = rule_of(c, "accel_limit_q88").q
    return (values["velocity_limit_cps"] / c.counts_per_deg,
            values["accel_limit_q88"] / (1 << q) * medium_hz(c) / c.counts_per_deg)


def overlaid(doc, entries):
    """Set `doc` with `entries`, {field: entry}, in place of its own
    entries of those fields, its hashes those of the result: what a
    servo holds where a run asks its own values of fields of the set.
    It is never written as a set. Refused for a field the set does not
    hold, and where a field would be over a limit that bounds it."""
    missing = [n for n in entries if n not in doc["fields"]]
    if missing:
        raise ValueError(f"the set holds no {', '.join(missing)}")
    out = {**doc, "fields": {n: copy.deepcopy(entries.get(n, e)) for n, e in doc["fields"].items()}}
    over = over_limit(out)
    if over:
        raise ValueError("; ".join(over))
    return {**out, "hash": hashes(out)}


rule("stall_omega_max_cps", "stall_omega_max.policy", kind="u16", unit="count/s", pos_unit=+1, cls="policy",
     mirrors="core kernel/phases/loops/limits.rs fold")(in_counts("stall_omega_max_deg_s"))
rule("pos_error_counts", "pos_error.policy", kind="u16", unit="counts", pos_unit=+1, cls="policy",
     mirrors="core kernel/phases/rail.rs rail")(in_counts("pos_error_deg"))
rule("pos_min_soft_counts", "soft_min.stops", kind="i32", unit="counts", cls="policy",
     mirrors="core kernel/phases/loops/trajectory.rs goal_q16, limits.rs fold")(soft_limit(False))
rule("pos_max_soft_counts", "soft_max.stops", kind="i32", unit="counts", cls="policy",
     mirrors="core kernel/phases/loops/trajectory.rs goal_q16, limits.rs fold")(soft_limit(True))


@rule("pos_deadband_counts", "deadband.pot_noise", kind="u16", unit="raw counts", cls="policy",
      mirrors="core kernel/phases/loops/position.rs band_q16, ident gains.rs synthesize")
def deadband(c):
    """The hold band: the policy's multiple of the pot's noise at rest,
    and no less than its floor, a band a quiet pot cannot make too
    tight to hold."""
    return max(c.policy("hold_band_noise_multiple") * c.q(POT, "pot_rest_sd_counts"),
               c.policy("hold_band_floor_counts"))


@rule("sensor_delta_max", "sensor_delta.policy", kind="u16", unit="raw counts/tick", cls="policy",
      mirrors="core kernel/phases/loops/faults.rs sensor_sample")
def sensor_delta(c):
    return c.policy("sensor_delta_max_counts")


# --- the protection rules ---
#
# The board NTC's curve is the design quadratic of the board's stage 1
# ntc-curve result, the fixed-point form the kernel carries (core
# estimator/thermal.rs ntc_cc): the centi-C at the reference count, the
# slope there and the curvature about the reference count, as that
# result holds the four.
NTC = "1-board/ntc-curve"


@rule("ntc_raw_ref", "ntc_raw_ref.design", kind="u16", unit="counts", mirrors="core estimator/thermal.rs ntc_cc")
def ntc_raw_ref(c):
    return c.q(NTC, "design_raw_ref")


@rule("ntc_t_ref_cc", "ntc_t_ref.design", kind="i16", unit="cC", mirrors="core estimator/thermal.rs ntc_cc")
def ntc_t_ref(c):
    return c.q(NTC, "design_t_ref_cc")


@rule("ntc_k1_q88", "ntc_k1.design", kind="i16", unit="cC/count", q=8, mirrors="core estimator/thermal.rs ntc_cc")
def ntc_k1(c):
    return c.q(NTC, "design_k1_q88") / (1 << 8)


@rule("ntc_k2_q24", "ntc_k2.design", kind="u16", unit="cC/count^2", q=24,
      mirrors="core estimator/thermal.rs ntc_cc")
def ntc_k2(c):
    return c.q(NTC, "design_k2_q24") / (1 << 24)


def limit_counts(key):
    """A current the policy states in mA, in shunt counts over the
    awake zero the kernel's reading subtracts (core estimator/bias.rs),
    through the board's measured counts per A."""
    def counts(c):
        return c.policy(key) / 1000 * shunt_counts_per_a(c)
    return counts


# The current-denominated limits: each a policy entry in mA, the kernel
# comparing its own reading in shunt counts against the field.
for _field, _key, _mirrors in (
        ("current_limit_counts", "current_limit_ma", "core kernel/phases/loops/limits.rs ceiling"),
        ("i_cont_counts", "i_cont_ma", "core kernel/phases/loops/limits.rs fold, i_cont_or_cap"),
        ("stall_release_counts", "stall_release_ma", "core kernel/phases/loops/limits.rs fold"),
        ("oc_trip_counts", "oc_trip_ma", "core kernel/phases/fast.rs measure")):
    rule(_field, f"{_field.removesuffix('_counts')}.policy", kind="u16", unit="ccounts", cls="policy",
         mirrors=_mirrors)(limit_counts(_key))


# The collision trip: a disturbance estimate over the field counts as a
# stall at once. The policy states it off, which is the field's largest
# value, one no estimate passes (core regions/config.rs
# STALL_TAU_TRIP_OFF). A level in counts has no rule yet.
@rule("stall_tau_trip_counts", "stall_tau_trip.policy", kind="u16", unit="ccounts", cls="policy",
      mirrors="core kernel/phases/loops/limits.rs fold")
def stall_tau_trip(c):
    if c.policy("collision_trip") != "off":
        raise ValueError(f"{POLICY}: collision_trip is {c.policies['collision_trip']!r}: the collision trip is "
                         f"`off`, a level has no rule yet")
    return KINDS["u16"][1]


# The one thermal limit: the policy's temperature in C, which the kernel
# compares in centi-C against its winding estimate, the overtemperature
# fault latching while the estimate reads it or more.
@rule("cutoff_cc", "cutoff.policy", kind="i16", unit="cC", cls="policy",
      mirrors="core kernel/phases/slow.rs cutoff")
def cutoff(c):
    return 100 * c.policy("cutoff_c")


# The winding estimate is the board NTC plus two heat nodes on one
# input, each a leak in seconds and a rise in centi-C per count squared
# of its heat (core estimator/thermal.rs, kernel/phases/loops/limits.rs
# heat). Its four fields are one model, read from the pair's stage 4
# winding-temperature result. The fast node is the pair's own: its time
# constant the one-node fit's, its gain the one refitted beside a slow
# node at the leaks a stored set runs. The slow node is the family's,
# which that result quotes from earlier work on another servo of the
# type: its time constant, and its gain by the reading of the plateau
# the policy names, the can's rise alone (`low`) or the earlier model's
# two modes together (`high`). So the fast pair is `measured` and the
# slow pair `policy`, the family's terms taken by a decision until a
# lock hold measures this motor's. A gain in C/W is a rise per count
# squared through the winding's resistance and the shunt's counts per A
# (oscnb.winding.rise_per_count2). A time constant is written in whole
# seconds and runs at the nearest power of two of the kernel's 16 ms
# period (oscnb.winding.leak_s, the mirror of tau_shift), so what a set
# of the four reads is said by the kernel's integers (`winding_step`) and
# not by the seconds written.
WINDING = "4-estimators/winding-temperature"
READING = "winding_plateau_reading"     # the policy entry that names the reading of the plateau
READINGS = ("low", "high")


def joined(c):
    """The stage 4 quantity of the reading of the plateau the policy
    names: the fast node's gain, C/W, fitted beside that reading's slow
    node, whose gain the quantity carries as `slow_gain_c_per_w`."""
    name = c.policy(READING)
    if name not in READINGS:
        raise ValueError(f"{POLICY}: {READING} is {name!r}: a reading of the plateau is one of "
                         f"{', '.join(READINGS)}")
    return f"joined_{name}_fast_gain_c_per_w"


def rise(gain):
    """A node's gain, C/W, as its rise in centi-C per count squared:
    over the winding's resistance of the stage 3 rails fit, the current
    in the board's measured shunt counts."""
    def per_count2(c):
        return winding.rise_per_count2(gain(c), c.q(*RESISTANCES["rails_fit"]), shunt_counts_per_a(c))
    return per_count2


@rule("thermal_tau_s", "thermal_tau.fast_node", kind="u16", unit="s",
      mirrors="core kernel/phases/loops/limits.rs tau_shift")
def thermal_tau(c):
    return c.q(WINDING, "one_node_tau_s")


@rule("thermal_tau2_s", "thermal_tau2.family", kind="u16", unit="s", cls="policy",
      mirrors="core kernel/phases/loops/limits.rs tau_shift")
def thermal_tau2(c):
    return c.q(WINDING, "family_slow_tau_s")


rule("th_rise_q016", "th_rise.fast_node", kind="u16", unit="cC/count^2", q=16,
     mirrors="core estimator/thermal.rs rise_cc")(rise(lambda c: c.q(WINDING, joined(c))))
rule("th_rise2_q016", "th_rise2.family", kind="u16", unit="cC/count^2", q=16, cls="policy",
     mirrors="core estimator/thermal.rs rise_cc")(rise(lambda c: c.raw(WINDING, joined(c), "slow_gain_c_per_w")))


# --- the screens' times and counts, the supply terms and the choices in words ---
#
# A time the policy states in ms is that many kernel ticks (core
# regions/config.rs ms_ticks), and a count of samples is the field. A
# choice between named behaviors is the policy's word, written as `osc
# dump` prints the field: an enum's name with its number, a flag as on
# or off.
STALL_RESPONSES = ("fault", "yield")    # core regions/config.rs StallResponse, by number
DUTY_FULL_Q15 = 32767                   # the whole rail, the field's largest value (core regions/config.rs)


def in_ticks(key):
    def ticks(c):
        return c.policy(key) * c.reg("board", "tick_hz") / 1000
    return ticks


def decided(key):
    def value(c):
        return c.policy(key)
    return value


def named(key, names):
    def text(c):
        name = c.policy(key)
        if name not in names:
            raise ValueError(f"{POLICY}: {key} is {name!r}: it is one of {', '.join(names)}")
        return f"{name.capitalize()} ({names.index(name)})"
    return text


rule("stall_response", "stall_response.policy", kind="u8", unit="response", cls="policy",
     mirrors="core kernel/phases/loops/limits.rs fold")(named("stall_response", STALL_RESPONSES))
rule("stall_time_ticks", "stall_time.policy", kind="u32", unit="ticks", cls="policy",
     mirrors="core kernel/phases/loops/limits.rs fold")(in_ticks("stall_time_ms"))
rule("pos_error_time_ticks", "pos_error_time.policy", kind="u32", unit="ticks", cls="policy",
     mirrors="core kernel/phases/rail.rs rail")(in_ticks("pos_error_time_ms"))
rule("oc_trip_ticks", "oc_trip_ticks.policy", kind="u8", unit="samples", cls="policy",
     mirrors="core kernel/phases/fast.rs measure")(decided("oc_trip_samples"))
rule("sensor_bad_count", "sensor_bad.policy", kind="u8", unit="samples", cls="policy",
     mirrors="core kernel/phases/loops/faults.rs sensor_sample")(decided("sensor_bad_samples"))


@rule("openloop_zero_brake", "openloop_zero_brake.policy", kind="u8", unit="flag", cls="policy",
      mirrors="core kernel/phases/fast.rs drive")
def zero_brake(c):
    flag = c.policy("openloop_zero_brake")
    if flag not in ("off", "on"):
        raise ValueError(f"{POLICY}: openloop_zero_brake is {flag!r}: it is off or on")
    return flag


# The duty cap is the policy's share of the rail, 1 being no cap.
@rule("duty_max_q15", "duty_max.policy", kind="u16", unit="Q15 of the rail", cls="policy",
      mirrors="core kernel/phases/loops/current.rs step")
def duty_max(c):
    share = c.policy("duty_max_share")
    if not 0 < share <= 1:
        raise ValueError(f"{POLICY}: duty_max_share is {share!r}: a share of the rail, over 0 and at most 1")
    return share * DUTY_FULL_Q15


# The undervolt floor is the policy's volts at the rail as the rail tap
# reads them, through the board's measured rail line, its offset with
# it (core kernel/phases/slow.rs compares the filtered tap). The rail
# fold's slope is the admittance of the source the policy designs for,
# shunt counts per rail count: the ceiling it puts on the current
# reaches zero at the floor (core kernel/phases/loops/limits.rs
# update_rail).
@rule("v_undervolt_counts", "v_undervolt.rail", kind="u16", unit="vcounts", cls="policy",
      mirrors="core kernel/phases/slow.rs slow, regions/config.rs vmotor_counts")
def v_undervolt(c):
    return c.q(CHAIN, "rail_offset_counts") + c.policy("v_undervolt_v") * c.q(CHAIN, "rail_counts_per_v")


@rule("rail_fold_q8", "rail_fold.source", kind="u16", unit="ccounts/vcount", q=8, cls="policy",
      mirrors="core kernel/phases/loops/limits.rs update_rail, regions/config.rs rail_fold_q8")
def rail_fold(c):
    return shunt_counts_per_a(c) / (c.q(CHAIN, "rail_counts_per_v") * c.policy("rail_source_ohm"))


# --- the limits no field holds ---
#
# A limit is a bound of the set on its own fields that the control
# table has no field for yet: the set carries it under `limits`, in no
# hash of what the servo is to hold, and a set with a field over the
# limit that bounds it is refused. The one limit so far is the speed
# ceiling, the most the output is ever to turn at, which protects the
# brushes: a speed, and no effective voltage. It is the no-load speed
# the maker of the servo's class states at the sheet's operating
# voltage, times the policy's derate of a datasheet speed on this
# servo, in the counts per second of the set's scale, and it bounds the
# move's speed limit (core kernel/phases/loops/trajectory.rs vel_cap).
# A sheet that states a range of operating voltage names no one speed
# by it: the policy then says which stated voltage of the range the
# speed is read at.
#
# What a maker states of a servo class is a record of the repo beside
# the sets, `<class>.datasheet.json`: the document, where it was read
# and when, and under `stated` each value as a quantity, one stated at
# several voltages as a list with the voltage of each, a range as its
# `min` and `max`. The registry names the class a servo is of
# (`datasheet_class`); a servo that names none has no sheet, and so no
# limit from one.
SHEET = "datasheet"                 # the input a set names the class's datasheet record by
CEILING = "speed_ceiling_cps"
SPEED_DERATE = "datasheet_derate_speed"     # the policy entry, a ratio as [numerator, denominator]
# the policy entry a range of operating voltage is resolved by: volts, or the registry's name of a servo voltage
SPEED_VOLTAGE = "datasheet_speed_voltage"
BOUNDS = {CEILING: ("velocity_limit_cps",)}     # the fields each limit bounds: none is ever over it


def sheet_path(pair, name, root=results.ROOT):
    """The datasheet record of servo class `name`, beside the sets."""
    return set_path(pair, root).with_name(f"{name}.datasheet.json")


def sheet(c):
    """The datasheet record of the class the registry says the pair's
    servo is of, the file among the set's inputs by its sha1."""
    name = c.reg("servo", "datasheet_class")
    rel = sheet_path(c.pair, name, c.root).relative_to(c.root)
    doc = c.file(SHEET, rel)
    if doc is None:
        raise Missing(f"the servo class {name} has no datasheet record ({rel.as_posix()})")
    return doc


def stated(c, key):
    """What the maker states under `key`, as the record holds it."""
    doc = sheet(c)
    if key not in doc.get("stated", {}):
        raise Missing(f"the datasheet record of {doc.get('class')} states no {key}")
    c.reads.append((f"{SHEET}:{key}", None))
    return doc["stated"][key]


def no_load(c):
    """(the no-load speed the maker states at the sheet's operating
    voltage, deg/s, that voltage): the angle the sheet times over the
    time it states for it. Of a range of operating voltage it is the
    stated voltage the policy says (SPEED_VOLTAGE): a number of volts
    is that voltage, and the registry's name of a voltage of the servo,
    the one it runs on, is the stated voltage nearest it, the lower of
    two as near. A time stated outside the range is none of it."""
    op = stated(c, "operating_voltage")
    lo, hi = (op["value"], op["value"]) if "value" in op else (op["min"], op["max"])
    times = [q for q in stated(c, "no_load_time") if lo <= q["at_v"] <= hi]
    volts, where = lo, f"its operating voltage, {lo:g} V"
    if lo != hi:
        at = c.policy(SPEED_VOLTAGE)
        rail = c.reg("servo", at) if isinstance(at, str) else None
        if type(rail) in (int, float):
            volts = min((q["at_v"] for q in times), key=lambda v: (round(abs(v - rail), 9), v), default=None)
            where = f"a voltage of its operating range, {lo:g} V to {hi:g} V"
        elif type(at) in (int, float):
            volts, where = at, (f"{at:g} V, the voltage {POLICY} names ({SPEED_VOLTAGE}), in its operating range, "
                                f"{lo:g} V to {hi:g} V")
        else:
            raise ValueError(f"{POLICY}: {SPEED_VOLTAGE} is {at!r}: it is a voltage the sheet states a speed at, in "
                             f"volts, or the registry's name of a voltage of the servo, the stated one nearest "
                             f"which is taken")
    at = [q for q in times if q["at_v"] == volts]
    if len(at) != 1:
        raise Missing(f"the datasheet record states no one no-load time at {where}")
    return at[0]["per_deg"] / at[0]["value"], at[0]["at_v"]


def speed_ceiling_deg_s(c):
    """The speed ceiling, deg/s: the maker's no-load speed at the
    policy's derate of a datasheet speed."""
    ratio = c.policy(SPEED_DERATE)
    if not (isinstance(ratio, list) and len(ratio) == 2 and all(type(n) is int for n in ratio)
            and 0 < ratio[0] <= ratio[1]):
        raise ValueError(f"{POLICY}: {SPEED_DERATE} is {ratio!r}: a derate is a ratio of at most one, written "
                         f"[numerator, denominator]")
    return no_load(c)[0] * ratio[0] / ratio[1]


SPEED_CEILING = Rule(CEILING, "speed_ceiling.datasheet", "u16", "count/s", 0, +1, 1, "datasheet",
                     "no field yet: the bound of velocity_limit_cps, core kernel/phases/loops/trajectory.rs vel_cap",
                     lambda c: speed_ceiling_deg_s(c) * c.counts_per_deg)


def limits_of(c, previous):
    """The set's limits, None for a set with none: the speed ceiling of
    a pair whose servo names a datasheet class, by its rule, with the
    speed in deg/s, what the maker states and at which voltage, the
    policy's choice of that voltage where the sheet states a range, the
    derate, the record and its document, and the fields it bounds. With
    an input missing it is held at the previous set's value, the input
    named, and left out where there is nothing to carry."""
    if not c.registry.get("servo", {}).get("datasheet_class"):
        return None
    carried = ((previous or {}).get("limits") or {}).get(CEILING)
    try:
        e = evaluate(SPEED_CEILING, c)
        (speed, volts), doc = no_load(c), sheet(c)
        e = {**e, "deg_s": speed_ceiling_deg_s(c), "stated_deg_s": speed, "at_v": volts,
             **({"voltage": c.policies[SPEED_VOLTAGE]} if f"policy:{SPEED_VOLTAGE}" in e["inputs"] else {}),
             "derate": c.policies[SPEED_DERATE], "record": doc["what"], "document": doc["document"]}
    except Missing as m:
        c.notes.append(f"{CEILING}: {'left out' if carried is None else 'held'}, {m}")
        if carried is None:
            return None
        e = {**held("value", SPEED_CEILING.name, carried, str(m), previous), "unit": carried["unit"]}
    return {CEILING: {**e, "bounds": list(BOUNDS[CEILING])}}


def over_limit(doc):
    """A line for every field of a set whose value is over a limit of
    the set that bounds it (BOUNDS)."""
    return [f"{name}: {doc['fields'][name]['value']} {e['unit']} is over the set's {limit}, {e['value']} {e['unit']}"
            + (f" ({e['deg_s']:g} deg/s)" if "deg_s" in e else "") + ": a field stays at or under the limit that "
            "bounds it"
            for limit, e in (doc.get("limits") or {}).items() for name in BOUNDS[limit]
            if name in doc["fields"] and type(doc["fields"][name]["value"]) is int
            and doc["fields"][name]["value"] > e["value"]]


def limits_loaded(doc, where):
    """Refuse a set file whose limits are not limits as the tool writes
    them: one the tool has none of, without a value of one class or the
    fields it bounds, a datasheet value that names no record or no
    document, one computed for another table than the set's, or a field
    over the limit that bounds it."""
    scale = doc.get("scale") or {}
    for name, e in (doc.get("limits") or {}).items():
        if name not in BOUNDS or "value" not in e or e.get("class") not in CLASSES \
                or e.get("bounds") != list(BOUNDS[name]):
            raise ValueError(f"{where}: {name} is no limit as the tool writes one: a value of one class of "
                             f"{', '.join(CLASSES)} and the fields it bounds")
        if e["class"] == "datasheet" and not (e.get("inputs") and e.get("document")):
            raise ValueError(f"{where}: {name} is of class datasheet and names no record or no document")
        if e.get("pos_unit") and e.get("scale_points_sha1") != scale.get("points_sha1"):
            raise ValueError(f"{where}: {name} was computed for the table of points sha1 "
                             f"{e.get('scale_points_sha1')}, the set's scale is that of {scale.get('points_sha1')}")
    over = over_limit(doc)
    if over:
        raise ValueError(f"{where}: {'; '.join(over)}")


def ceiling_said(doc):
    """What `status` says of a set's speed ceiling, a line, none for a
    set without: its counts per second and deg/s, what the maker states
    and the derate that give it, how the policy chose the voltage of a
    range, the document, and the field it bounds at the value the set
    holds."""
    e = (doc.get("limits") or {}).get(CEILING)
    if e is None:
        return []
    bounds = ", ".join(f"{n} = {doc['fields'][n]['value']}" if n in doc["fields"] else f"{n}, not in the set"
                       for n in e["bounds"])
    if "held" in e:
        return [f"speed ceiling: {e['value']} {e['unit']} [{tagged(e)}]: {e['held']}; it bounds {bounds}"]
    num, den = e["derate"]
    by, chosen = e.get("voltage"), ""
    if isinstance(by, str):
        chosen = (f", the stated voltage nearest the servo's {by} of {doc['inputs']['registry']['servo'][by]:g} V "
                  f"by the policy ({SPEED_VOLTAGE})")
    elif by is not None:
        chosen = f", the voltage the policy names ({SPEED_VOLTAGE})"
    return [f"speed ceiling: {e['value']} {e['unit']}, {e['deg_s']:g} deg/s at the set's scale [{tagged(e)}]: the "
            f"maker's no-load speed at {e['at_v']:g} V{chosen}, {e['stated_deg_s']:g} deg/s, x {num}/{den} by the "
            f"policy ({SPEED_DERATE}); {e['document']}; it bounds {bounds}"]


# --- build ---

def held(what, rule_name, carried, why, previous, would_be=None):
    """The entry of a field (`what` "value") or of the slots ("slots")
    whose rule does not bind: what the previous set held, why, the set
    it was first carried from, what the rule would give where it gave a
    value, and the rule's name where one is chosen. Its class is
    `none`: no rule gives the value it carries. A carried value at
    an unknown scale stays marked as that: holding it gave it none."""
    e = {what: carried[what], "class": "none", **({} if rule_name is None else {"rule": rule_name}), "held": why,
         "carried_from": carried.get("carried_from", previous["hash"]["set"]),
         **({"scale": carried["scale"]} if "scale" in carried else {})}
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


def values(block):
    """The values of a set's `scales`, by name; none for a set without."""
    return {n: e["value"] for n, e in (block or {}).items()}


def scales_of(c, previous):
    """The conversions a capture of the pair is scored with (SCALES),
    each the board's measured quantity with its band, read through `c`
    so the result is among the set's inputs; held at the previous set's
    where the result is missing, left out where there is nothing to
    carry. None for a set with none."""
    out = {}
    for name, (result, key, rule_name) in SCALES.items():
        carried = ((previous or {}).get("scales") or {}).get(name)
        c.reads = []
        try:
            value = c.q(result, key)
        except Missing as m:
            c.notes.append(f"{name}: {'left out' if carried is None else 'held'}, {m}")
            if carried is not None:
                out[name] = {**held("value", rule_name, carried, str(m), previous),
                             **{k: carried[k] for k in ("lo", "hi", "unit") if k in carried}}
            continue
        quantity = c.reads[-1][1]
        out[name] = {"value": value, **{k: quantity[k] for k in ("lo", "hi") if quantity.get(k) is not None},
                     "unit": quantity.get("unit", ""), "class": "measured", "rule": rule_name,
                     "inputs": [f"{result}:{key}"]}
    return out or None


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


def no_source(c, name, e, said=None):
    """Entry `e` of field `name` as one with no source: of class
    `none`, with why it has none, what would give it one and the
    maker's value that could where the pair's sources file says them of
    the field, else where `said`, the policy's word on an entry its
    rule read, does."""
    said = c.sources.get("none", {}).get(name) or said or {}
    return {**e, "class": "none", **{k: said[k] for k in ("reason", "needs", "datasheet_candidate") if k in said}}


def undecided(c, inputs):
    """The policy's word on the first entry among `inputs` that it
    restates with no decision, its `reason` and what it `needs`; None
    where every policy entry read is a decision."""
    block = c.policies.get("undecided", {})
    keys = (i.partition(":")[2] for i in inputs if i.startswith("policy:"))
    return next((block[k] for k in keys if k in block), None)


def recorded(c, name, cls):
    """The entry of a field the pair's sources file gives a record
    under `cls`, `bench` or `datasheet`: the value the record's file
    holds at the declared key, read and never typed, what the record
    is, the maker's document for a datasheet value, and what the record
    awaits where the sources or the record itself call it `weak`. The
    file is among the set's inputs, so a record that moves makes the
    set stale. Refused where the record or its key is not there, or the
    value is not one a field holds."""
    said = c.sources[cls][name]
    rel = in_repo(c.root / said["record"], c.root, f"a {cls} record").as_posix()
    at = doc = c.file(rel, rel)
    for key in said["key"].split("."):
        at = at.get(key) if isinstance(at, dict) else None
    if type(at) not in (int, str):
        raise ValueError(f"the sources of {c.pair}: the {cls} record of {name}, {rel}, "
                         + ("is not there" if doc is None else f"holds no value of a field at {said['key']}"))
    awaits = said.get("weak", doc.get("weak") if isinstance(doc, dict) else None)
    return {"value": at, "class": cls, "record": said["what"],
            **({"document": said["document"]} if cls == "datasheet" else {}),
            **({} if awaits is None else {"weak": awaits}), "inputs": [f"sources:{cls}.{name}", f"{rel}:{said['key']}"]}


def bound(c, name, carried, previous):
    """The entry a build gives field `name`, None when it has none: a
    pin of the pair, else the pair's bench or datasheet record of it,
    else the value of its rule, the one the policy names where it has
    several, held at the carried value where the policy holds it or an
    input is missing. A field with none of them is carried at its
    value, with no source, a pinned one as it stands. A field the
    policy holds lists what its rule read beside what it would be. A
    hold a deviation releases does not hold, and is refused where the
    rule has no value to bind. A field whose rule reads a policy entry
    nobody decided is bound at what the rule gives, with no source."""
    if name in c.pins:
        pin = c.pins[name]
        return {"value": pin["value"], "class": "pinned", "why": pin["why"], "rung": pin["rung"]}
    for cls in ("bench", "datasheet"):
        if name in c.sources.get(cls, {}):
            return recorded(c, name, cls)
    if name not in RULES:
        if carried is None or carried["class"] == "pinned":
            return copy.deepcopy(carried)
        return no_source(c, name, {k: carried[k] for k in ("value", "origin", "scale") if k in carried})
    r = None
    try:
        r = rule_of(c, name)
        e = evaluate(r, c)
    except Missing as m:
        if name in c.released:
            raise ValueError(f"{name}: its hold is released and its rule has no value: {m}") from None
        c.notes.append(f"{name}: {'left out' if carried is None else 'held'}, {m}")
        return None if carried is None else no_source(c, name, held("value", r and r.name, carried, str(m), previous))
    if len(RULES[name]) > 1:
        e["inputs"] = [f"policy:rule.{name}", *e["inputs"]]
    open_ = undecided(c, e["inputs"])
    why = None if name in c.released else c.policies.get("hold", {}).get(name)
    if why is None:
        if open_ is None and name in c.sources.get("none", {}):
            c.notes.append(f"{name}: the pair's sources say it has no source, and {r.name} binds it")
        return e if open_ is None else no_source(c, name, e, open_)
    c.notes.append(f"{name}: {'left out' if carried is None else 'held'}, {why}; its rule gives {e['value']}")
    # what the rule read stays listed: the value it would be moves with it
    return None if carried is None else no_source(
        c, name, {**held("value", r.name, carried, why, previous, e["value"]), "inputs": e["inputs"]}, open_)


def build(pair, previous=None, protection=(), root=results.ROOT, registry=None, deviation=None):
    """(the pair's set, what there is to say of it), built offline from
    the results of record, the registry, the policy file and the pair's
    pins and sources, on top of `previous`, its set of record: first
    what defines a count, the slots, then the table and its scale, then
    every field, in the previous set's order with new ones after. A
    field with no rule, no pin and no record is carried at its value,
    with no source. Then the limits no field holds (`limits_of`), which
    either kind of build forms anew, and a set with a field over the
    limit that bounds it is refused (`over_limit`).

    One build moves one hash. Without `protection` it moves the plant:
    every protection field is carried, with `would_be` where its rule
    or pin gives another value. A held field stands as held on either
    side, its value the one that stood. With `protection` it moves the fields
    named there and carries everything else, the slots, the table and
    the scale with it. A protection field the firmware no longer names
    (RETIRED) is such a change: carried as it stood until it is named,
    then left out. A set with nothing before it is built whole.
    `deviation` is a deviation file the build reads over the policy and
    the pins (`derive`)."""
    wrong = [n for n in protection if n not in PROTECTION]
    if wrong or (protection and previous is None):
        raise ValueError(f"{', '.join(wrong)} is no protection field" if wrong
                         else "there is no set to carry the plant from")
    c = Context(pair, root, registry, deviation)
    slots = slots_of(c, previous)
    table = table_of(c, previous, slots)
    scale = c.scale
    scales = scales_of(c, previous)
    if protection:
        # the plant stands: what defines a count is the previous set's, and must be what it would be now,
        # and so must what the set is scored with
        stood = [previous["wide_codes"], previous["pos_table"], previous.get("scale")]
        was = previous.get("scales")
        if any((a is None) != (b is None) or (a is not None and any(a[k] != b[k] for k in keys))
               for a, b, keys in zip((slots, table, scale), stood,
                                     (("slots",), ("state", "points_sha1"), ("counts_per_deg",)))) \
                or values(scales) != values(was):
            raise ValueError("the slots, the table, the scale or what the set is scored with would move: build "
                             "the plant first, then the protection fields")
        slots, table, scale = (copy.deepcopy(x) for x in stood)
        scales = copy.deepcopy(was)
    before = previous["fields"] if previous else {}
    fields = {}
    for name in dict.fromkeys([*before, *RULES, *c.pins, *c.sources.get("bench", {}),
                               *c.sources.get("datasheet", {})]):
        if name in RETIRED:
            if name in before and name not in protection:
                fields[name] = copy.deepcopy(before[name])
                c.notes.append(f"{name}: carried at {before[name]['value']}, the firmware names no such field: "
                               f"name it with --protection to leave it out")
            elif name in before:
                c.notes.append(f"{name}: left out, the firmware names no such field")
            continue
        e = bound(c, name, before.get(name), previous)
        moves = (name in protection) if name in PROTECTION else not protection
        # a held entry stands on either side: its value is the one that stood, so it moves no hash
        stands = e is not None and "held" in e and name in before
        if previous is not None and not moves and not stands:
            # the side this build does not move: as it stood, with what it would be beside it
            would = None
            if e is not None and name in before:
                would = e["value"] if e["value"] != before[name]["value"] else e.get("would_be")
            gave, e = e, copy.deepcopy(before.get(name))
            if e is None:
                # a field its rule left out is said already
                if gave is not None:
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
    if scales:
        doc["scales"] = scales
    limits = limits_of(c, previous)
    if limits:
        doc["limits"] = limits
    over = over_limit(doc)
    if over:
        raise ValueError("; ".join(over))
    return {**doc, "hash": hashes(doc)}, c.notes


def changes(previous, doc):
    """What a built set changes against the one before it, a line each:
    a value that moves, and a value that stands whose class does."""
    def said(e, what):
        return str(e[what]) if what != "slots" else bytes(e[what]).hex(" ")

    out = []
    for label, key, whats in ((SLOTS, "wide_codes", ("slots",)), ("position table", "pos_table",
                                                                  ("state", "points_sha1"))):
        a, b = previous.get(key), doc.get(key)
        if a is None or b is None:
            if a is not b:
                out.append(f"{label}: {'not in the set' if a is None else tagged(a)} -> "
                           f"{'not in the set' if b is None else tagged(b)}")
            continue
        for what in whats:
            name = label if what == "slots" else f"{label} {what.replace('_', ' ')}"
            if a[what] != b[what]:
                out.append(f"{name}: {said(a, what)} -> {said(b, what)} [{tagged(b)}]")
            elif tagged(a) != tagged(b):
                out.append(f"{name}: {said(b, what)} [{tagged(a)} -> {tagged(b)}]")
    a, b = previous.get("scale"), doc.get("scale")
    if a != b:
        out.append("scale: " + " -> ".join("none" if x is None else f"{x['counts_per_deg']:.4f} counts/deg, the "
                                           f"{x['kind']} one" for x in (a, b)))
    for block, label in (("scales", "scored with "), ("limits", "limit "), ("fields", "")):
        was, now = previous.get(block) or {}, doc.get(block) or {}
        for name in dict.fromkeys([*was, *now]):
            a, b = was.get(name), now.get(name)
            if a is None or b is None:
                out.append(f"{label}{name}: {'not in the set' if a is None else a['value']} -> "
                           f"{'not in the set' if b is None else b['value']}")
            elif a["value"] != b["value"]:
                out.append(f"{label}{name}: {a['value']} -> {b['value']} [{tagged(b)}]")
            elif tagged(a) != tagged(b):
                out.append(f"{label}{name}: {b['value']} [{tagged(a)} -> {tagged(b)}]")
    both = [k for k in ("plant", "protection", "set", "scales", "limits")
            if k in previous["hash"] or k in doc["hash"]]
    moved_ = [k for k in both if k != "set" and previous["hash"].get(k) != doc["hash"].get(k)]
    out.append("hash: " + (f"{' and '.join(moved_)} moved" if moved_ else "neither moved") + ": "
               + ", ".join(f"{k} {(previous['hash'].get(k) or 'none')[:12]} -> {(doc['hash'].get(k) or 'none')[:12]}"
                           for k in both))
    return out


# --- the winding model's step ---
#
# No one direction says what a step of the winding model's four does to
# the protection: LOOSER reads a field alone, and here one rise falls
# while the other rises and both leaks are judged. What the step does
# is what the estimate then reads, so it is printed as that, by the
# kernel's own integers (oscnb.winding.stored_rise_c): the rise over
# the board NTC under the four held and under the four wanted, in a
# hold from rest at the continuous current and at the limit, the rest
# after one, and duty cycles at the limit. The gate stands as it is,
# every field named; this is what is read before naming them.
HOLD_S = (1, 3, 10, 30, 60, 120, 300, 600, 3600)    # s into a hold from rest
REST_FROM_S = 600                                   # s at the limit before the rest
REST_S = (1, 3, 10, 30, 60, 120, 300)               # s into the rest
CYCLES_S = ((0.5, 4.5), (3.0, 3.0), (3.0, 57.0), (30.0, 30.0))      # s on, s off, at the limit
CYCLED_S = 1800                                     # s of cycles, the last one read


def winding_cases(four, cont, limit):
    """What a stored set of the winding model's four (GROUPS's order)
    reads over the board NTC, C, by case: a hold from rest at `cont`
    and at `limit`, counts, the rest after REST_FROM_S at the limit,
    and the peak and the mean of the last cycle of CYCLED_S of each
    duty cycle at the limit."""
    def at(trace, s):
        return float(trace[round(s / winding.STORED_PERIOD_S) - 1])

    out = {}
    for counts in dict.fromkeys((cont, limit)):
        trace = winding.stored_rise_c(four, [(HOLD_S[-1], counts)])
        out.update({f"a hold from rest at {counts} counts, {s} s in": at(trace, s) for s in HOLD_S})
    trace = winding.stored_rise_c(four, [(REST_FROM_S, limit), (REST_S[-1], 0)])
    out.update({f"the rest after {REST_FROM_S} s at {limit} counts, {s} s in": at(trace, REST_FROM_S + s)
                for s in REST_S})
    for on, off in CYCLES_S:
        cycle = [(on, limit), (off, 0)]
        last = winding.stored_rise_c(four, cycle * math.ceil(CYCLED_S / (on + off)))[
            -round((on + off) / winding.STORED_PERIOD_S):]
        out.update({f"{on:g} s on, {off:g} s off at {limit} counts, the {what} of a cycle": float(v)
                    for what, v in (("peak", last.max()), ("mean", last.mean()))})
    return out


def winding_compared(held_, wanted, cont, limit):
    """(the lines a step of the winding model is said with, the cases
    the wanted four read cooler in): the four as held and as wanted,
    the leak each node runs at, every case of `winding_cases` under both,
    and whether the wanted four read at least as hot in every one."""
    def leaks(four):
        # a second leak of 0 drops the second node; a first of 0 runs one period
        return " and ".join(f"{winding.leak_s(tau):g} s" for tau in (four[0], four[2])[:2 if four[2] else 1])

    was, now = (winding_cases(four, cont, limit) for four in (held_, wanted))
    cooler = [case for case in was if now[case] < was[case]]
    lines = [f"{MODEL}: {', '.join(GROUPS[MODEL])} held at {', '.join(map(str, held_))}, wanted at "
             f"{', '.join(map(str, wanted))}",
             f"{MODEL}: the leaks run at {leaks(held_)}, wanted at {leaks(wanted)} (the nearest power of two of "
             f"the {1000 * winding.STORED_PERIOD_S:g} ms period)",
             f"{MODEL}: the estimate's rise over the board NTC, C, held -> wanted, by the kernel's integers:",
             *(f"  {case}: {was[case]:.2f} -> {now[case]:.2f}" for case in was)]
    lines.append(f"{MODEL}: the wanted four read at least as hot in every case" if not cooler
                 else f"{MODEL}: the wanted four read cooler in {len(cooler)} of {len(was)} cases: "
                 + "; ".join(f"{case} ({now[case] - was[case]:+.2f} C)" for case in cooler))
    return lines, cooler


def winding_step(doc, was):
    """The lines a step of the winding model is said with
    (`winding_compared`), from the four as `was`, {field: value}, holds
    them to the four of set `doc`: none where the two hold the same
    four or either lacks one. The cases run at the set's own continuous
    current, the limit where that is 0 (core
    kernel/phases/loops/limits.rs i_cont_or_cap), and at its current
    limit."""
    f = doc["fields"]
    if not all(n in f and n in was for n in GROUPS[MODEL]):
        return []
    held_, wanted = [was[n] for n in GROUPS[MODEL]], [f[n]["value"] for n in GROUPS[MODEL]]
    if held_ == wanted:
        return []
    if not all(n in f for n in ("i_cont_counts", "current_limit_counts")):
        return [f"{MODEL}: the set holds no current limit to compare the estimate at"]
    limit = f["current_limit_counts"]["value"]
    return winding_compared(held_, wanted, f["i_cont_counts"]["value"] or limit, limit)[0]


# --- a deviation ---

def in_repo(path, root=results.ROOT, what="a deviation"):
    """A file's path from the repo root, as a set or a rung names it;
    refused for a file outside the repo, which no record could name."""
    try:
        return Path(path).resolve().relative_to(Path(root).resolve())
    except ValueError:
        raise ValueError(f"{path} is not a file of the repo: {what} is named by its path in it") from None


def deviation_of(path):
    """A deviation file's document: `{"policy": {...}, "pins": {field:
    {"value", "why", "rung"}}, "release": [field, ...]}`, any of the
    three, and a `note`. It names no raw list of values: a policy entry
    goes through the rules, at the set's scale, a pin is a literal with
    its reason and the rung that chose it, and a release names fields
    the policy file holds, each then bound at what its rule gives, the
    rest of the policy standing as it is."""
    doc = json.loads(Path(path).read_text())
    extra = [k for k in doc if k not in ("note", "policy", "pins", "release")]
    release = doc.get("release", [])
    if extra or not (doc.get("policy") or doc.get("pins") or release):
        raise ValueError(f"{sc.shown(path)}: a deviation holds policy entries, pins or holds to release"
                         f"{', and no ' + ', '.join(extra) if extra else ', and this one none'}")
    if not (isinstance(release, list) and all(isinstance(n, str) for n in release)):
        raise ValueError(f"{sc.shown(path)}: `release` is a list of the fields whose hold it releases")
    return doc


def derive(base, deviation, root=results.ROOT, registry=None):
    """(the set `base` with one deviation, what the build has to say):
    `base` built again with the deviation file's policy entries and
    pins over its own and the holds it releases bound at their rules'
    values, the file named among its inputs, and under `deviation` the
    file, its sha1 and every field it moved as [of record, deviated].
    Refused when `base` already holds a deviation, when `base` is not
    what a build of its inputs gives now, so the derived set would
    differ by more than the deviation, when the deviation moves
    nothing, when it releases a field that is not held or whose rule
    has no value, and when it pins or releases a protection field and
    anything else: a deviation that moves a protection field may move
    nothing else, moves the protection fields of one step (GROUPS, a
    field in no group being a step alone), and releases the holds of
    that step together. A deviation that moves the winding model is
    said with what the estimate reads before and after (`winding_step`)."""
    where = sc.shown(deviation)
    if base.get("deviation"):
        raise ValueError(f"the set already holds the deviation {base['deviation']['file']}: a rung runs on a set "
                         f"or on a set with one deviation")
    dev = deviation_of(deviation)
    moves = [*dev.get("pins", {}), *dev.get("release", [])]
    named = [n for n in moves if n in PROTECTION]
    if named and (len(named) < len(moves) or dev.get("policy")):
        raise ValueError(f"{where}: a deviation that moves a protection field ({', '.join(named)}) may move "
                         f"nothing else")
    steps = list(dict.fromkeys(group_of(n)[0] for n in named))
    if len(steps) > 1:
        raise ValueError(f"{where}: a deviation moves the protection fields of one step, and these are of "
                         f"{len(steps)}: {', '.join(steps)}")
    plain, _ = build(base["pair"], base, named, root, registry)
    if plain["hash"] != base["hash"] or plain.get("scale") != base.get("scale"):
        raise ValueError("the set is not what a build of its inputs gives now ("
                         + "; ".join(changes(base, plain)[:-1]) + "): `build` it first, a deviation is one change")
    doc, notes = build(base["pair"], base, named, root, registry, deviation)
    released = [n for n in named if n in dev.get("release", [])]
    left = [n for n in (group_of(released[0])[1] if released else ())
            if "held" in doc["fields"].get(n, {}) and "would_be" in doc["fields"][n]]
    if left:
        raise ValueError(f"{where}: the holds of the {steps[0]} are released together, and {', '.join(left)} "
                         f"would stay held")
    fields = {n: [base["fields"].get(n, {}).get("value"), e["value"]] for n, e in doc["fields"].items()
              if n not in base["fields"] or base["fields"][n]["value"] != e["value"]}
    if not fields or doc.get("scale") != base.get("scale") \
            or any(hashed(doc)[0][k] != hashed(base)[0][k] for k in ("wide_codes", "pos_table")):
        raise ValueError(f"{where}: the deviation moves no field of the set" if not fields
                         else f"{where}: a deviation moves fields, not the slots, the table or the scale")
    doc["deviation"] = {"file": in_repo(deviation, root).as_posix(), "sha1": sc.file_sha1(deviation),
                        "fields": fields}
    return doc, [*notes, *winding_step(doc, {n: e["value"] for n, e in base["fields"].items()})]


# --- status ---

def users(doc, name):
    """What of a set lists input `name`: its fields, the slots, the
    table, what it is scored with and its limits."""
    blocks = ((SLOTS, doc["wide_codes"] or {}), ("position table", doc["pos_table"]))
    return [n for n, e in (*doc["fields"].items(), *blocks, *doc.get("scales", {}).items(),
                           *doc.get("limits", {}).items())
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
    the set holds, its speed ceiling with its source (`ceiling_said`),
    every field with no source with why it has none,
    what would give it one and the maker's value that could supply it
    where the set names one, the fields on a weak record, and where its
    adopted values came from."""
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
    lines += ceiling_said(doc)
    debt = unsourced(doc)
    if debt:
        could = sum(maker is not None for *_, maker in debt)
        lines.append(f"no source, {len(debt)} fields, {could} of which a maker's value on record could supply:")
        lines += [f"  {name} = {value}: {why}; needs {needs}"
                  + ("" if maker is None else f"; datasheet candidate, not its source: {maker}")
                  for name, value, why, needs, maker in debt]
    lines += [f"on a weak record: {', '.join(names)}: {awaits}" for awaits, names in weak(doc).items()]
    for name in sorted(doc["origins"]):
        o = doc["origins"][name]
        lines.append(f"adopted from {name}: {o['from']} (record sha1 {o['sha1']}, state hash {o['state_hash'][:12]})")
    return lines, bool(stale or reg)


# --- check ---

def differences(doc, state):
    """Where the servo's `state` (`scorecard.servo_state`) is not the
    set, a line each as `name: servo X, set Y [class, rule]`
    (`tagged`): every field of the set by name, the table's state and
    points, the slots."""
    def line(name, got, e, want):
        return f"{name}: servo {got}, set {want} [{tagged(e)}]"

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
    less = way(name)
    if less == "judged" or not all(type(v) is int for v in (on_servo, wanted)):
        return True
    return wanted > on_servo if less == "up" else wanted < on_servo


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
        less = looser(name, got, e["value"])
        said = ("judged, taken as protecting less" if way(name) == "judged"
                else f"protects {'less' if less else 'more'} (less is {way(name)})")
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


def live_table(doc, stops, root=results.ROOT, quiet=False):
    """The Table of a set whose table is LIVE, from the pair's table
    file, which must hold the set's points, be built against `stops`
    and name no other slots than the set's as the ones its lookup was
    fitted on; a file that records none is said, unless `quiet`, and
    taken."""
    want = doc["pos_table"]
    path = set_path(doc["pair"], root).with_name(postable.out_path(doc["pair"]).name)
    if not path.exists():
        raise settings.Refused(f"the set's table is LIVE and the pair has no table file ({sc.shown(path)})")
    t, file = postable.load_file(path)
    fitted = file["inputs"].get("fitted_on_slots_sha1")
    slots = doc["wide_codes"]["slots"] if doc["wide_codes"] is not None else bytes(widecodes.SLOTS)
    if fitted is None:
        if not quiet:
            print(f"{sc.shown(path)}: {postable.fitted_on_said(file)}")
    elif fitted != postable.slots_sha1(slots):
        raise settings.Refused(f"the lookup was fitted on other slots (sha1 {fitted}, the set's are "
                               f"{postable.slots_sha1(slots)}): re-run stage 4 position, then `postable build`")
    if points_sha1(t.points) != want["points_sha1"]:
        raise settings.Refused(f"{sc.shown(path)} holds points of sha1 {points_sha1(t.points)}, the set's table "
                               f"is {want['points_sha1']}")
    if list(stops) != [t.raw_min, t.raw_max]:
        raise settings.Refused(f"{sc.shown(path)} was built against stops {t.raw_min} and {t.raw_max}, the set's "
                               f"are {stops[0]} and {stops[1]}")
    return t


def table_to_write(doc, state, root=results.ROOT, table=None):
    """The Table an install has to write, None when the servo holds the
    set's. An IDENTITY table is all-zero points against the set's
    stops. A LIVE one is `table` when it is given and holds the set's
    points, else the one of the pair's table file (`live_table`). No
    other state can be written."""
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
    return live_table(doc, stops, root)


def protection_moves(doc, state):
    """The protection fields of the set the servo's `state` holds at
    another value, each as [the servo's, the set's]."""
    return {n: [state["fields"][n], e["value"]] for n, e in doc["fields"].items()
            if n in PROTECTION and state["fields"].get(n) != e["value"]}


def install(doc, protection=(), loosen=(), root=results.ROOT, table=None, state=None):
    """Make the servo equal the set, volatile; prints each write as it
    lands and returns what was written, by name, in order. Nothing is
    written unless torque is off, no input file of the set moved, the
    servo lets a host write every field the set holds, the table can be
    installed (`table_to_write`) and the protection gate passes
    (`gate`). An install that would change the winding model's four
    says what the estimate reads under the servo's and under the set's
    (`winding_step`), with the gate's lines or ahead of the writes. The
    writes are the slots, the stops, the table, then the other fields
    that differ in the servo's table order, each read back, a field
    the servo refuses tried once more after the rest. The
    state read back whole must then be the set, and only then is it
    stamped, no save, and its data state required clean. A refusal
    after the first write reboots the servo onto its saved state and
    says so. `state` is the servo's state where the caller has just
    read it."""
    torque = osc.get("torque_enable")
    if not settings.same("off", torque):
        raise settings.Refused(f"torque_enable reads {torque}: a set is installed with torque off")
    stale = moved(doc, root)
    if stale:
        raise settings.Refused(f"the set is stale, {', '.join(name for name, *_ in stale)} moved: `status` names "
                               f"the fields")
    state = sc.servo_state() if state is None else state
    slots = doc["wide_codes"]
    missing = [n for n in doc["fields"] if n not in state["writable"]]
    if slots is not None and state["wide_codes"] is None:
        missing.append(widecodes.FIELD)
    if missing:
        raise settings.Refused(f"the servo's descriptor lets a host write no {', '.join(missing)}")
    refused = gate(doc, state, protection, loosen)
    step = winding_step(doc, state["fields"])
    if refused:
        raise settings.Refused("the protection gate:\n  " + "\n  ".join([*refused, *step]))
    if step:
        print("\n".join(step))
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
    if doc.get("deviation"):
        raise settings.Refused(f"{sc.shown(path)} holds the deviation {doc['deviation']['file']}: a deviation is "
                               f"never saved")
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
    ad = sub.add_parser("adopt", help="the first set of a servo that already runs, every field adopted from a "
                                      "record of its state")
    ad.add_argument("--from", dest="record", type=Path, required=True,
                    help="a record of the servo's state, a scorecard run's servo.json")
    ad.add_argument("--origin", default=BEFORE, help="where the record came from, in words the set keeps")
    b = sub.add_parser("build", help="offline: the set from the results of record, on top of the set of record")
    b.add_argument("--out", type=Path, help="write a derived set here and leave the set of record as it is")
    b.add_argument("--deviation", type=Path,
                   help="with --out: the set of record with this deviation file's policy entries, pins and "
                        "released holds")
    b.add_argument("--protection", nargs="+", default=[], metavar="NAME",
                   help="move these protection fields and carry the plant, not the other way about")
    sub.add_parser("status", help="offline: the set's input files against the files now, and its fields with "
                                  "no source")
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
            if a.deviation and (a.out is None or a.protection or previous is None):
                raise ValueError("a deviation is built on the set of record, to --out, and names its own "
                                 "protection fields: it is never the set of record")
            doc, notes = (derive(previous, a.deviation, root) if a.deviation
                          else build(a.pair, previous, a.protection, root))
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
