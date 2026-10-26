from oscnb.b41t import decode, encode


def test_dc_volts_round_trip():
    name, value, raw = decode(encode(0, 4, 3, 7400))
    assert name == "DC V" and abs(value - 7.400) < 1e-9 and raw["dec"] == 3


def test_negative_millivolts():
    name, value, _ = decode(encode(0, 3, 2, -1234))
    assert name == "DC V" and abs(value - (-0.01234)) < 1e-12


def test_ohms_in_kilohm_range():
    name, value, _ = decode(encode(4, 5, 3, 4870))
    assert name == "ohm" and abs(value - 4870.0) < 1e-9


def test_temperature_as_displayed():
    name, value, _ = decode(encode(8, 0, 1, 236))
    assert name == "C" and abs(value - 23.6) < 1e-9


def test_unknown_scale_gives_no_value():
    name, value, _ = decode(encode(2, 7, 0, 5))
    assert name == "DC A" and value is None


def test_hex_string_is_accepted():
    pkt = encode(0, 4, 3, 7400).hex()
    assert decode(pkt)[1] == decode(bytes.fromhex(pkt))[1]
