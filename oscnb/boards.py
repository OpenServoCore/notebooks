"""The board registry: one entry per physical controller board.

A board is the measurement chain between the motor and the firmware: the
shunt and its amplifier, the terminal and rail dividers, the bias node the
taps return to, the NTC, the ADC and its reference, and the PWM timer the
drive windows are counted on. The entry holds the parts' nominal values,
the firmware's published constants for the same parts, and the conversions
from counts to volts and amps that follow from them. Measured corrections
to those conversions are the stage 1 results, read from the results files;
nothing measured is typed in here.

Authority: every capture's meta.json carries a `sense` block read from the
servo at capture time, and `check_meta` fails loudly when a capture was
taken on parts other than the entry declares. Never edit a number here to
silence that error; fix the entry or the selection.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Board:
    key: str
    label: str
    uid: str
    # current chain: the shunt and the op-amp gain, as the firmware publishes them
    shunt_mohm: int
    gain_milli: int
    # terminal taps, top to the terminal, bottom returned to the VB bias node
    vmotor_div_top: int
    vmotor_div_bot: int
    bias_top: int
    bias_bot: int
    bias_nom_counts: int
    # supply rail tap, to ground, no bias
    vbus_div_top: int
    vbus_div_bot: int
    rail_drop_mv: int
    # board NTC: pull-up to VDD, the part's R25 and beta
    ntc_pullup_ohm: int
    ntc_r25_ohm: int
    ntc_beta: int
    # op-amp input pull-ups to VDD, one on each input, which set the inputs' common mode;
    # 0 on a board without them
    opa_pullup_ohm: int = 0
    # converter and timer
    vdd_mv: int = 3300
    vref_v: float = 1.2       # the chip's internal reference, CH32V006 datasheet nominal
    adc_counts: int = 4096
    tick_hz: int = 20_000
    pwm_arr: int = 1200          # the counter's crest; a window is `drive_ticks` either side of it
    i_window_min_ticks: int = 64
    v_window_min_ticks: int = 93
    notes: str = ""

    # --- volts ---
    @property
    def v_adc_per_count(self):
        return self.vdd_mv / 1e3 / self.adc_counts

    @property
    def div_ratio(self):
        """r in the tap formula V_term = r x tap - (r - 1) x VB."""
        return (self.vmotor_div_top + self.vmotor_div_bot) / self.vmotor_div_bot

    @property
    def v_term_per_count(self):
        """Terminal volts per count of tap difference; vA - vB is bias free."""
        return self.v_adc_per_count * self.div_ratio

    @property
    def vbus_ratio(self):
        return (self.vbus_div_top + self.vbus_div_bot) / self.vbus_div_bot

    @property
    def v_rail_per_count(self):
        return self.v_adc_per_count * self.vbus_ratio

    @property
    def vb_nom_v(self):
        return self.vdd_mv / 1e3 * self.bias_bot / (self.bias_top + self.bias_bot)

    # --- amps ---
    @property
    def a_per_count(self):
        return self.v_adc_per_count / (self.gain_milli / 1e3 * self.shunt_mohm / 1e3)

    @property
    def counts_per_a(self):
        return 1 / self.a_per_count

    # --- duty ---
    def drive_ticks(self, q15):
        """The firmware's window half-width for a Q15 duty: (|d| x arr + 2^14) >> 15."""
        return (np.abs(np.asarray(q15, dtype=np.int64)) * self.pwm_arr + (1 << 14)) >> 15

    def duty_pct_of_ticks(self, ticks):
        return np.asarray(ticks, float) * 100 / self.pwm_arr

    @property
    def i_floor_pct(self):
        return self.duty_pct_of_ticks(self.i_window_min_ticks)

    @property
    def v_floor_pct(self):
        return self.duty_pct_of_ticks(self.v_window_min_ticks)

    # --- conversions ---
    def term_v(self, tap_counts, vb_counts):
        """Absolute terminal volts from one tap against the bias node."""
        r = self.div_ratio
        return (r * np.asarray(tap_counts, float) - (r - 1) * np.asarray(vb_counts, float)) * self.v_adc_per_count

    def diff_v(self, tap_a_counts, tap_b_counts):
        """Terminal to terminal volts, bias free."""
        return (np.asarray(tap_a_counts, float) - np.asarray(tap_b_counts, float)) * self.v_term_per_count

    def amps(self, current_raw, zero_counts):
        return (np.asarray(current_raw, float) - zero_counts) * self.a_per_count

    def vdd_v(self, vref_raw):
        """VDD from the chip's 1.2 V reference as the ADC reads it (`vref_raw`
        in a TEL row). The ADC's reference is VDD, so a count is VDD / 4096 and
        the 1.2 V reference reads lower as VDD rises."""
        return self.vref_v * self.adc_counts / np.asarray(vref_raw, float)

    def rail_v(self, vbus_raw, vref_raw=None):
        """Rail volts from the raw rail tap (`vbus_raw` in a TEL row). Not the
        firmware's `vbus_counts`, which is already scaled into tap units. The
        tap is a share of VDD, so with `vref_raw` the measured VDD replaces the
        nominal one, and the reading stays right when the regulator drops out."""
        v = np.asarray(vbus_raw, float) * self.v_rail_per_count
        if vref_raw is None:
            return v
        return v * self.vdd_v(vref_raw) / (self.vdd_mv / 1e3)

    def ntc_ohm(self, ntc_raw):
        """The NTC's resistance from its raw count, pull-up to VDD."""
        raw = np.asarray(ntc_raw, float)
        return self.ntc_pullup_ohm * raw / (self.adc_counts - raw)

    def ntc_c_beta(self, ntc_raw):
        """The NTC's temperature from the part's beta, degrees C."""
        r = self.ntc_ohm(ntc_raw)
        inv = 1 / 298.15 + np.log(r / self.ntc_r25_ohm) / self.ntc_beta
        return 1 / inv - 273.15

    # --- guard ---
    @property
    def sense(self):
        """The meta.json `sense` block a capture on this board carries."""
        return {"shunt_r_mohm": self.shunt_mohm, "gain_milli": self.gain_milli,
                "vmotor_div_top": self.vmotor_div_top, "vmotor_div_bot": self.vmotor_div_bot,
                "vdd_mv": self.vdd_mv}

    def check_meta(self, meta, where=""):
        got = {k: meta["sense"].get(k) for k in self.sense}
        if got != self.sense:
            raise ValueError(f"{where}: meta.json sense {got} != board {self.key} {self.sense}")


BOARDS = {
    "dev-v006-2A-D": Board(
        key="dev-v006-2A-D",
        label="osc-dev-v006 rev 2A, board D",
        uid="cdabf60e22bce576",
        shunt_mohm=60, gain_milli=14_884,
        vmotor_div_top=6_400, vmotor_div_bot=1_600,
        bias_top=430, bias_bot=100, bias_nom_counts=773,
        vbus_div_top=6_400, vbus_div_bot=1_600, rail_drop_mv=250,
        ntc_pullup_ohm=10_000, ntc_r25_ohm=10_000, ntc_beta=3950,
        opa_pullup_ohm=1_500,
        i_window_min_ticks=64, v_window_min_ticks=93,
        notes="Rs1 60 mOhm with the bare op-amp at Rf 6k4 / Rg 430. Terminal taps 6k4 over 1k6 "
              "returned to the VB node from 430 / 100. VSNS 6k4 / 1k6 to ground. 1k5 pull-ups from "
              "3V3 to both op-amp inputs, hand-soldered 1 % thick film, no Rs1 pads. Every value "
              "here but the pull-ups is what the firmware publishes in its calib table; the stage 1 "
              "results carry the measured corrections.",
    ),
}

DEFAULT = "dev-v006-2A-D"


def from_meta(meta):
    """The entry whose sense block a capture carries, None when no entry
    matches. Boards of one revision share a sense block, so a dataset names
    its board in its path and this is the check, not the selection."""
    for b in BOARDS.values():
        if all(meta.get("sense", {}).get(k) == v for k, v in b.sense.items()):
            return b
    return None
