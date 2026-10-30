//! Register addresses sweep and burst read and write, mirrored from the
//! servo core's control table. The cross-check test below fails loudly on
//! any drift from the pinned firmware, so these consts are safe to hardcode.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Reg {
    pub addr: u16,
    pub width: u8,
}

const fn reg(addr: u16, width: u8) -> Reg {
    Reg { addr, width }
}

pub mod config {
    use super::{Reg, reg};

    pub const POS_MIN_SOFT_COUNTS: Reg = reg(0x0028, 4);
    pub const POS_MAX_SOFT_COUNTS: Reg = reg(0x002c, 4);
    pub const CURRENT_LIMIT_COUNTS: Reg = reg(0x0048, 2);
    /// 0 = a stall latches a fault, 1 = it folds the limit to the
    /// continuous current.
    pub const STALL_RESPONSE: Reg = reg(0x004a, 1);
    pub const DRIVE_POLARITY: Reg = reg(0x004b, 1);
    pub const STALL_TIME_TICKS: Reg = reg(0x004c, 4);
    /// The continuous current: a press over it on a still shaft trips and
    /// folds to it; 0 reads as the limit.
    pub const I_CONT_COUNTS: Reg = reg(0x0052, 2);
    /// The fold holds while the held current reads this or more.
    pub const STALL_RELEASE_COUNTS: Reg = reg(0x0054, 2);
    /// The collision trip on the disturbance estimate; `u16::MAX` (the boot
    /// value) trips nothing.
    pub const STALL_TAU_TRIP_COUNTS: Reg = reg(0x0056, 2);
}

pub mod calib {
    use super::{Reg, reg};

    pub const R_Q12: Reg = reg(0x0080, 2);
    /// The pot stops.
    pub const RAW_MIN: Reg = reg(0x00a0, 2);
    pub const RAW_MAX: Reg = reg(0x00a2, 2);
    pub const SHUNT_R_MOHM: Reg = reg(0x00ac, 2);
    pub const GAIN_MILLI: Reg = reg(0x00ae, 2);
    pub const VMOTOR_DIV_TOP: Reg = reg(0x00b0, 2);
    pub const VMOTOR_DIV_BOT: Reg = reg(0x00b2, 2);
    pub const VDD_MV: Reg = reg(0x00b4, 2);
    pub const TICK_HZ: Reg = reg(0x00b6, 2);
    pub const I_WINDOW_MIN_TICKS: Reg = reg(0x00b8, 2);
    pub const VBUS_DIV_TOP_OHM: Reg = reg(0x00bc, 2);
    pub const VBUS_DIV_BOT_OHM: Reg = reg(0x00be, 2);
    pub const RAIL_DROP_MV: Reg = reg(0x00c8, 2);
    /// The PWM period, compare counts: TEL `duty_counts`' full scale. RO.
    pub const PWM_PERIOD_TICKS: Reg = reg(0x00ce, 2);
}

pub mod control {
    use super::{Reg, reg};

    pub const TORQUE_ENABLE: Reg = reg(0x0180, 1);
    /// Deliberate stall: drops the stall trip and the endstop band, keeps the
    /// current limit and the thermal derate. RAM only, cleared by a reboot.
    /// Firmware that leases it grants about a second per write of true with
    /// torque on, and drops it on torque off
    /// ([`crate::instrument::limits::PermitLease`]).
    pub const STALL_PERMIT: Reg = reg(0x0181, 1);
    pub const MODE: Reg = reg(0x0184, 1);
    pub const GOAL_DUTY: Reg = reg(0x0186, 2);
    pub const GOAL_VELOCITY: Reg = reg(0x018c, 4);
    pub const GOAL_CURRENT: Reg = reg(0x0190, 2);
    /// Bus dead-man, kernel ticks: on the experiment image a nonzero value
    /// under torque counts down to a `deadman` fault unless an instruction
    /// reloads it or a TEL burst holds it; the production image ignores it
    /// ([`crate::instrument::limits::DEADMAN_TICKS`]).
    pub const BUS_DEADMAN_TICKS: Reg = reg(0x0192, 2);
    pub const BURST_DUTY_Q15: Reg = reg(0x0196, 2);
    pub const BURST_ARM: Reg = reg(0x0198, 1);
    pub const BURST_PAGE: Reg = reg(0x0199, 1);
    /// Extras behind the shunt: bit 0 vmotor_a, bit 1 vmotor_b, bit 2 vbus.
    /// Latched at the arm; values above 7 reject with Validation.
    pub const BURST_CHANS: Reg = reg(0x019a, 1);
    /// The TEL row's chunks, then the arm: one WRITE of all five
    /// ([`osc_protocol::tel::Chunks::to_le_bytes`] + `tel_count`) sets the row
    /// and starts the burst.
    pub const CHUNK0_ADDR: Reg = reg(0x01e2, 2);
    /// Nonzero only while a burst is armed or streaming.
    pub const TEL_COUNT: Reg = reg(0x01ea, 2);
}

/// BURST readback section, all RO. One READ of `PAGE_ECHO..=RESTORE_DIR` is
/// 252 B: the selected page plus the whole header.
pub mod burst {
    use super::{Reg, reg};

    pub const PAGE_ECHO: Reg = reg(0x02c0, 1);
    pub const STATE: Reg = reg(0x02c1, 1);
    /// 120 LE u16 raw shunt codes.
    pub const SAMPLES: Reg = reg(0x02c2, 240);
    pub const SAMPLES_LEN: Reg = reg(0x03b2, 2);
    pub const STEP_INDEX: Reg = reg(0x03b4, 2);
    pub const START_CNT: Reg = reg(0x03b6, 2);
    pub const PWM_ARR: Reg = reg(0x03b8, 2);
    pub const START_DIR: Reg = reg(0x03ba, 1);
    pub const RESTORE_DIR: Reg = reg(0x03bb, 1);
    /// One past the page READ, constant for the capture: the mask it used
    /// and its frame length (1 + popcount).
    pub const CHANS_ECHO: Reg = reg(0x03bc, 1);
    pub const FRAME_LEN: Reg = reg(0x03bd, 1);
}

pub mod telemetry {
    use super::{Reg, reg};

    pub const FAULT_FLAGS: Reg = reg(0x0200, 1);
    pub const FAULT_CODE: Reg = reg(0x0221, 1);
    /// The tick block, `I_TICK_COUNTS..=SAMPLE_TICK`, written every control
    /// tick: what a TEL row's first chunk copies.
    pub const I_TICK_COUNTS: Reg = reg(0x024a, 2);
    pub const VDIFF_COUNTS: Reg = reg(0x024c, 2);
    /// Signed PWM compare counts; full scale is `calib::PWM_PERIOD_TICKS`.
    pub const DUTY_TICKS: Reg = reg(0x0246, 2);
    pub const POS: Reg = reg(0x0238, 2);
    pub const CURRENT_TROUGH: Reg = reg(0x0248, 2);
    pub const CURRENT_RAW: Reg = reg(0x0240, 2);
    pub const VMOTOR_A: Reg = reg(0x0242, 2);
    pub const VMOTOR_B: Reg = reg(0x0244, 2);
    pub const VBUS_RAW: Reg = reg(0x023c, 2);
    pub const NTC_RAW: Reg = reg(0x023e, 2);
    pub const VREF_RAW: Reg = reg(0x023a, 2);
    /// `osc_servo_core::regions::telemetry::tick_flag` bits.
    pub const TICK_FLAGS: Reg = reg(0x0252, 2);
    /// Published at the medium step: a TEL row carries it as last
    /// published, up to a tick old.
    pub const VBUS_COUNTS: Reg = reg(0x0266, 2);
    pub const CURRENT_BIAS_COUNTS: Reg = reg(0x026c, 2);
    pub const VMOTOR_BIAS_COUNTS: Reg = reg(0x026e, 2);
    /// What shaped the command; bit 3 the stall permit lease is live.
    pub const LIMIT_FLAGS: Reg = reg(0x027c, 1);
    /// Smallest duty whose drive window the shunt reads, Q15: where the
    /// OpenLoop ceiling restarts and its blind band begins; 0 = not
    /// published.
    pub const WINDOW_FLOOR_Q15: Reg = reg(0x027e, 2);
    /// TEL rows the servo dropped because both stream buffers were waiting
    /// for the wire; wraps.
    pub const TEL_DROP_COUNT: Reg = reg(0x03c6, 2);
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_servo_core::regions::ControlTable;

    /// Every const above with its control table field name.
    const ALL: &[(&str, Reg)] = &[
        ("pos_min_soft_counts", config::POS_MIN_SOFT_COUNTS),
        ("pos_max_soft_counts", config::POS_MAX_SOFT_COUNTS),
        ("current_limit_counts", config::CURRENT_LIMIT_COUNTS),
        ("stall_response", config::STALL_RESPONSE),
        ("drive_polarity", config::DRIVE_POLARITY),
        ("stall_time_ticks", config::STALL_TIME_TICKS),
        ("i_cont_counts", config::I_CONT_COUNTS),
        ("stall_release_counts", config::STALL_RELEASE_COUNTS),
        ("stall_tau_trip_counts", config::STALL_TAU_TRIP_COUNTS),
        ("r_q12", calib::R_Q12),
        ("raw_min", calib::RAW_MIN),
        ("raw_max", calib::RAW_MAX),
        ("shunt_r_mohm", calib::SHUNT_R_MOHM),
        ("gain_milli", calib::GAIN_MILLI),
        ("vmotor_div_top", calib::VMOTOR_DIV_TOP),
        ("vmotor_div_bot", calib::VMOTOR_DIV_BOT),
        ("vdd_mv", calib::VDD_MV),
        ("tick_hz", calib::TICK_HZ),
        ("i_window_min_ticks", calib::I_WINDOW_MIN_TICKS),
        ("vbus_div_top_ohm", calib::VBUS_DIV_TOP_OHM),
        ("vbus_div_bot_ohm", calib::VBUS_DIV_BOT_OHM),
        ("rail_drop_mv", calib::RAIL_DROP_MV),
        ("pwm_period_ticks", calib::PWM_PERIOD_TICKS),
        ("torque_enable", control::TORQUE_ENABLE),
        ("stall_permit", control::STALL_PERMIT),
        ("mode", control::MODE),
        ("goal_duty", control::GOAL_DUTY),
        ("goal_velocity", control::GOAL_VELOCITY),
        ("goal_current", control::GOAL_CURRENT),
        ("bus_deadman_ticks", control::BUS_DEADMAN_TICKS),
        ("duty_q15", control::BURST_DUTY_Q15),
        ("arm", control::BURST_ARM),
        ("page", control::BURST_PAGE),
        ("chans", control::BURST_CHANS),
        ("chunk0_addr", control::CHUNK0_ADDR),
        ("tel_count", control::TEL_COUNT),
        ("page_echo", burst::PAGE_ECHO),
        ("state", burst::STATE),
        ("samples", burst::SAMPLES),
        ("samples_len", burst::SAMPLES_LEN),
        ("step_index", burst::STEP_INDEX),
        ("start_cnt", burst::START_CNT),
        ("pwm_arr", burst::PWM_ARR),
        ("start_dir", burst::START_DIR),
        ("restore_dir", burst::RESTORE_DIR),
        ("chans_echo", burst::CHANS_ECHO),
        ("frame_len", burst::FRAME_LEN),
        ("fault_flags", telemetry::FAULT_FLAGS),
        ("fault_code", telemetry::FAULT_CODE),
        ("i_tick_counts", telemetry::I_TICK_COUNTS),
        ("vdiff_counts", telemetry::VDIFF_COUNTS),
        ("duty_ticks", telemetry::DUTY_TICKS),
        ("pos", telemetry::POS),
        ("current_trough", telemetry::CURRENT_TROUGH),
        ("current_raw", telemetry::CURRENT_RAW),
        ("vmotor_a", telemetry::VMOTOR_A),
        ("vmotor_b", telemetry::VMOTOR_B),
        ("vbus_raw", telemetry::VBUS_RAW),
        ("ntc_raw", telemetry::NTC_RAW),
        ("vref_raw", telemetry::VREF_RAW),
        ("tick_flags", telemetry::TICK_FLAGS),
        ("vbus_counts", telemetry::VBUS_COUNTS),
        ("current_bias_counts", telemetry::CURRENT_BIAS_COUNTS),
        ("vmotor_bias_counts", telemetry::VMOTOR_BIAS_COUNTS),
        ("limit_flags", telemetry::LIMIT_FLAGS),
        ("window_floor_q15", telemetry::WINDOW_FLOOR_Q15),
        ("tel_drop_count", telemetry::TEL_DROP_COUNT),
    ];

    #[test]
    fn consts_match_the_control_table() {
        for (name, reg) in ALL {
            let f = ControlTable::FIELDS
                .iter()
                .find(|f| f.name == *name)
                .unwrap_or_else(|| panic!("{name} missing from the control table"));
            assert_eq!(f.addr, reg.addr, "{name} addr");
            assert_eq!(f.width, reg.width as u16, "{name} width");
        }
    }
}
