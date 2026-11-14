"""Which tool an `oscnb.osc` call runs: `osc` by default, the one `using`
names for the calls of its body and no longer, `osc-lab` for the capture
verbs whatever `using` says, and what a record holds of a tool."""
import hashlib
import subprocess

import pytest

from oscnb import osc


@pytest.fixture
def ran(monkeypatch):
    """Every command line the module starts, none run."""
    cmds = []

    def fake_run(cmd, **kw):
        cmds.append(cmd)
        return subprocess.CompletedProcess(cmd, 0, stdout="p_kp_q88 = 5\n", stderr="bus at 3000000 baud\n")

    monkeypatch.setattr(osc.subprocess, "run", fake_run)
    monkeypatch.setattr(osc.subprocess, "Popen", lambda cmd, **kw: cmds.append(cmd))
    return cmds


def every_call(tmp_path):
    """One of each kind of call: the verbs through `run`, the ping, the
    flash, the clapper and a started tool, then the three capture verbs."""
    osc.get("p_kp_q88")
    osc.set_("p_kp_q88", 5)
    osc.run("stamp")
    osc.bus_baud()
    osc.flash(tmp_path / "fw.bin")
    osc.clapper(tmp_path / "clap.csv", 3)
    osc.start("status")
    osc.track(tmp_path / "t", "position", ["hold@500"])
    osc.bursts(tmp_path / "b", [10])
    osc.sweep(tmp_path / "s", [10])
    osc.sweep(tmp_path / "s", [10], background=True)


def test_using_names_the_tool_of_every_osc_call_in_its_body_and_no_other(ran, tmp_path):
    labs = [osc.LAB] * 4
    every_call(tmp_path)
    assert [c[0] for c in ran] == [osc.TOOL] * 8 + labs and osc.tool() == osc.TOOL
    ran.clear()
    with osc.using("new/osc"):
        assert osc.tool() == "new/osc"
        every_call(tmp_path)
        with osc.using(None):
            osc.run("status")
        with osc.using(tmp_path / "other"):
            osc.run("status")
        osc.run("status")
    # the flash pings first, with the tool that flashes
    assert [c[0] for c in ran] == ["new/osc"] * 8 + labs + ["new/osc", str(tmp_path / "other"), "new/osc"]
    ran.clear()
    with pytest.raises(RuntimeError, match="stop"):
        with osc.using("new/osc"):
            raise RuntimeError("stop")
    osc.run("status")
    assert ran == [[osc.TOOL, "-i", osc.ID, "status"]]


def tool_file(path, body):
    path.write_text(f"#!/bin/sh\n{body}\n")
    path.chmod(0o755)
    return path


def test_a_record_holds_a_tools_sha1_and_its_version_line_and_no_path(tmp_path):
    new = tool_file(tmp_path / "osc-new", 'echo "osc 0.2.0 (a98647e8)"')
    old = tool_file(tmp_path / "osc-old", "echo 'error: unexpected argument --version' >&2; exit 2")
    with osc.using(new):
        assert osc.identity() == {"sha1": hashlib.sha1(new.read_bytes()).hexdigest(),
                                  "version": "osc 0.2.0 (a98647e8)"}
    # a tool with no such flag is still named by its binary
    with osc.using(old):
        assert osc.identity() == {"sha1": hashlib.sha1(old.read_bytes()).hexdigest(), "version": None}
    with osc.using(tmp_path / "nowhere"):
        assert osc.identity() is None
