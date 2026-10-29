//! The sense chain's scales: shunt counts to amps, terminal-tap counts to
//! volts, from the servo's own calib constants.

/// The calib constants the scales come from.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SenseParams {
    pub shunt_r_mohm: u16,
    pub gain_milli: u16,
    pub vmotor_div_top: u16,
    pub vmotor_div_bot: u16,
    pub vdd_mv: u16,
}

/// ADC lsb in volts: 4096 = 12-bit full scale, ref = VDD.
fn adc_lsb_v(sense: &SenseParams) -> f64 {
    sense.vdd_mv as f64 / 1000.0 / 4096.0
}

/// Shunt + PGA scale; 0.0 on a degenerate front end.
pub fn amps_per_count(sense: &SenseParams) -> f64 {
    let denom = sense.gain_milli as f64 / 1000.0 * (sense.shunt_r_mohm as f64 / 1000.0);
    if denom > 0.0 {
        adc_lsb_v(sense) / denom
    } else {
        0.0
    }
}

/// Volts per count behind the vmotor taps' divider; 0.0 on an unset
/// divider bottom.
pub fn volts_per_count(sense: &SenseParams) -> f64 {
    let (top, bot) = (sense.vmotor_div_top, sense.vmotor_div_bot);
    if bot == 0 {
        return 0.0;
    }
    adc_lsb_v(sense) * (top as f64 + bot as f64) / bot as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bench board: 60 mohm at G 15, 6k8/3k3 taps, 3.3 V.
    #[test]
    fn scales_on_the_bench_board() {
        let s = SenseParams {
            shunt_r_mohm: 60,
            gain_milli: 15000,
            vmotor_div_top: 6800,
            vmotor_div_bot: 3300,
            vdd_mv: 3300,
        };
        assert!((1.0 / amps_per_count(&s) - 1117.0).abs() < 0.5);
        assert!((volts_per_count(&s) * 1000.0 - 2.466).abs() < 1e-3);
        let none = SenseParams {
            gain_milli: 0,
            vmotor_div_bot: 0,
            ..s
        };
        assert_eq!((amps_per_count(&none), volts_per_count(&none)), (0.0, 0.0));
    }
}
