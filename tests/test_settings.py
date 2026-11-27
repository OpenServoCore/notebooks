"""A settings rung with `osc` stubbed: the order of the calls, the old
values kept, the refusals, and the reboot that puts it all back."""
import pytest

from oscnb import settings

CLEAN = "id 1: model 0x0101\n      fault 0x00  unsaved false  trim 0\n      settings loaded\n      refused  none\n"
FRESH = ("id 1: model 0x0101\n      fault 0x00  unsaved true  trim 0\n      settings defaults: CONFIG, CALIB\n"
         "      refused  Current, Velocity, Position\n"
         "               closed mode refused: Ke, the current cap or the speed ceiling is 0, unset; run an "
         "identification: osc-ident run\n"
         "               changes not saved: a reboot loses them\n      fault config (7)\n")


class FakeOsc:
    def __init__(self, regs, status=CLEAN, boot_pings=0):
        self.regs = dict(regs)
        self.log = []
        self.report = status
        self.boot_pings = boot_pings    # pings that go unanswered after a reboot

    def get(self, field):
        self.log.append(("get", field))
        return self.regs[field]

    def set_(self, field, value):
        self.log.append(("set", field, value))
        self.regs[field] = value

    def run(self, *args, **kw):
        self.log.append(("run", *args))

    def status(self):
        self.log.append(("status",))
        return self.report

    def bus_baud(self):
        self.log.append(("ping",))
        if self.boot_pings:
            self.boot_pings -= 1
            raise RuntimeError("no servo answers")
        return 3_000_000


@pytest.fixture
def fake(monkeypatch):
    def make(**kw):
        f = FakeOsc({"torque_enable": "off", "p_kp_q88": 4000, "v_kp_q88": 300}, **kw)
        for name in ("get", "set_", "run", "status", "bus_baud"):
            monkeypatch.setattr(settings.osc, name, getattr(f, name))
        monkeypatch.setattr(settings.time, "sleep", lambda s: None)
        return f
    return make


def test_apply_reads_then_writes_in_that_order_and_nothing_else(fake):
    f = fake()
    old = settings.apply({"p_kp_q88": 5026, "v_kp_q88": 320}, "a rung")
    assert old == {"p_kp_q88": 4000, "v_kp_q88": 300}
    assert f.regs["p_kp_q88"] == 5026 and f.regs["v_kp_q88"] == 320
    assert f.log == [
        ("get", "torque_enable"),
        ("get", "p_kp_q88"), ("get", "v_kp_q88"),
        ("set", "p_kp_q88", 5026), ("get", "p_kp_q88"),
        ("set", "v_kp_q88", 320), ("get", "v_kp_q88"),
    ]
    assert not any("save" in str(c) or "stamp" in str(c) for c in f.log)


def test_apply_refuses_with_torque_on_and_writes_nothing(fake):
    f = fake()
    f.regs["torque_enable"] = "on"
    with pytest.raises(settings.Refused, match="torque"):
        settings.apply({"p_kp_q88": 5026})
    assert f.log == [("get", "torque_enable")]
    assert f.regs["p_kp_q88"] == 4000


def test_apply_refuses_a_write_that_did_not_land(fake, monkeypatch):
    f = fake()
    monkeypatch.setattr(settings.osc, "set_", lambda field, value: f.log.append(("set", field, value)))
    with pytest.raises(settings.Refused, match="read back 4000"):
        settings.apply({"p_kp_q88": 5026})


def test_a_servo_on_its_defaults_is_written_like_any(fake):
    f = fake(status=FRESH)
    settings.apply({"p_kp_q88": 5026})
    assert f.regs["p_kp_q88"] == 5026


def test_restore_reboots_waits_for_the_bus_and_says_the_settings(fake):
    f = fake(boot_pings=2)
    assert settings.restore() == "loaded"
    assert f.log == [("run", "reboot"), ("ping",), ("ping",), ("ping",), ("status",)]
    fake(status=FRESH)
    assert settings.restore() == "defaults: CONFIG, CALIB", "information, no refusal"


def test_the_settings_and_the_refused_modes_read_their_own_lines():
    assert (settings.settings(CLEAN), settings.refused(CLEAN)) == ("loaded", "none")
    # the operator line that says a mode is refused is not the refused line
    assert (settings.settings(FRESH), settings.refused(FRESH)) == ("defaults: CONFIG, CALIB",
                                                                   "Current, Velocity, Position")
    with pytest.raises(RuntimeError, match="no settings line"):
        settings.settings("id 1: model 0x0101\n      data  clean\n")


def test_unsaved_reads_the_health_line():
    assert settings.unsaved(FRESH) is True
    assert settings.unsaved(CLEAN) is False
    with pytest.raises(RuntimeError):
        settings.unsaved("id 1: model 0x0101\n")


def test_restore_gives_up_on_a_silent_bus(fake):
    fake(boot_pings=settings.BOOT_TRIES)
    with pytest.raises(settings.Refused, match="no answer"):
        settings.restore()


def test_show_reads_only(fake):
    f = fake()
    assert settings.show({"p_kp_q88": 1, "v_kp_q88": 2}) == {"p_kp_q88": 4000, "v_kp_q88": 300}
    assert all(c[0] == "get" for c in f.log)


def test_read_back_compares_numbers_and_words():
    assert settings.same(5026, "5026") and settings.same(-3, -3)
    assert settings.same(True, "on") and settings.same("off", "off")
    assert not settings.same(5026, 5025)


def test_load_wants_fields(tmp_path):
    p = tmp_path / "rung.json"
    p.write_text('{"fields": {"p_kp_q88": 5026}, "note": "kp up"}')
    assert settings.load(p) == ({"p_kp_q88": 5026}, "kp up")
    p.write_text('{"note": "nothing"}')
    with pytest.raises(ValueError):
        settings.load(p)
