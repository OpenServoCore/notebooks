//! The brake idiom and the supply a rail comes from.

/// The brake idiom: under slow decay a token duty against the motion, 3 PWM
/// ticks at ARR 1200, holds the idle leg high for the rest of each period,
/// so the winding shorts through the bridge and the rotor stops in a few
/// hundred counts. A duty of 0 coasts. Pointed away from the wall the drive
/// approached, the soft-limit clamp can never zero it.
pub const BRAKE_DUTY_Q15: i16 = 82;
/// The brake holds until two polls `BRAKE_POLL_MS` apart differ by less
/// than this, counts, or for `BRAKE_POLLS` polls.
pub const BRAKE_REST_EPS: u16 = 4;
pub const BRAKE_POLL_MS: u32 = 20;
pub const BRAKE_POLLS: u32 = 50;

/// What powers the servo.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Supply {
    Usb,
    TwoS,
}

impl Supply {
    /// The supply a rail of `rail_mv` comes from: USB is 4.0 to 5.5 V, a
    /// 2S pack 6.0 to 8.7 V. None for anything else.
    pub fn of_rail(rail_mv: f64) -> Option<Self> {
        if (4000.0..=5500.0).contains(&rail_mv) {
            Some(Supply::Usb)
        } else if (6000.0..=8700.0).contains(&rail_mv) {
            Some(Supply::TwoS)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rail_names_its_supply() {
        assert_eq!(Supply::of_rail(4390.0), Some(Supply::Usb));
        assert_eq!(Supply::of_rail(7900.0), Some(Supply::TwoS));
        assert_eq!(Supply::of_rail(5800.0), None);
        assert_eq!(Supply::of_rail(0.0), None);
    }
}
