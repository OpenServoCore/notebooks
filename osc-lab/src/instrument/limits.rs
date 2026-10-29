//! The servo's own limits, as a drive plans against them: the current limit
//! and the stall settings from CONFIG, the travel from the soft limits, and
//! the winding R and the rail that turn a duty into a stall current. The
//! host reads the fields; everything decided from them lives here.

use core::fmt;

use super::regs::{Reg, control};

/// How often a pump rewrites a held stall permit, ms. The firmware grants
/// about a second per write, so a missed rewrite or two still leaves the
/// permit standing.
pub const PERMIT_REFRESH_MS: u32 = 250;

/// The longest TEL stream a held permit may ride, ms: nothing is rewritten
/// while a stream runs, and the refresh before it may already be due.
pub const PERMIT_STREAM_MAX_MS: u32 = 750;

/// What a pump writes into `bus_deadman_ticks` on the experiment image:
/// 20000 kernel ticks at 20 kHz is one second of bus silence before the
/// servo coasts, rewritten every [`PERMIT_REFRESH_MS`] while torque is on.
pub const DEADMAN_TICKS: u16 = 20_000;

/// Full scale of the 12-bit pot ADC; soft limits spanning it are the board
/// default, not a calibration.
pub const POT_MAX: i32 = 4095;

/// Soft-to-guard inset, counts. A sweep's seek band spans 75 counts either
/// side of a guard, so the guard sits 25 counts more than that inside soft,
/// or the band reaches the firmware's soft-limit clamp and the seek fights
/// it.
pub const GUARD_INSET: u16 = 100;

const Q15: f64 = 32767.0;

/// The lowest winding R the servo class carries, ohms. A stall at
/// `i_lim x CLASS_R_MIN / vbus` draws at most the limit on any servo of the
/// class, so a drive at that duty is safe before anything measured R.
pub const CLASS_R_MIN: f64 = 3.0;

/// A duty fraction as whole percent, rounded down: a planned duty never
/// stalls over what it was planned for.
pub fn pct_floor(duty: f64) -> u8 {
    (duty.clamp(0.0, 1.0) * 100.0 + 1e-9).floor() as u8
}

/// The travel guard for a servo's soft limits: each end `GUARD_INSET`
/// counts inside.
pub fn guards(soft: (u16, u16)) -> (u16, u16) {
    (
        soft.0.saturating_add(GUARD_INSET),
        soft.1.saturating_sub(GUARD_INSET),
    )
}

/// Soft limits spanning the whole pot: the board default, or limits a
/// widened run never put back.
pub fn is_board_default(soft: (i32, i32)) -> bool {
    soft.1 - soft.0 >= POT_MAX
}

/// Duty, as a fraction of full scale, whose stall current is `i_counts`:
/// a stalled winding passes duty x vbus over R. `r_vpc` is vcounts per
/// ccount, `vbus` vcounts.
pub fn duty_for(i_counts: f64, r_vpc: f64, vbus: f64) -> f64 {
    if vbus <= 0.0 {
        return 0.0;
    }
    (i_counts * r_vpc / vbus).clamp(0.0, 1.0)
}

/// The current, counts, a duty fraction draws with the shaft stalled.
pub fn stall_counts(duty: f64, r_vpc: f64, vbus: f64) -> f64 {
    if r_vpc <= 0.0 {
        return f64::INFINITY;
    }
    duty.abs() * vbus / r_vpc
}

/// What the servo allows, read from its table.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ServoLimits {
    /// `current_limit_counts`.
    pub i_lim: u16,
    /// `i_cont_counts`: the continuous current, the press trigger and the
    /// limit a press folds back to.
    pub i_cont: u16,
    /// `stall_tau_trip_counts`: the collision trip.
    pub tau_trip: u16,
    pub soft: (i32, i32),
    /// The pot stops, `raw_min` and `raw_max`.
    pub raw: (u16, u16),
    /// Winding R, vcounts per ccount in Q12; 0 until identified.
    pub r_q12: u16,
    /// The rail, vcounts.
    pub vbus: u16,
    /// `window_floor_q15`: the smallest duty whose drive window the shunt
    /// reads, as the servo publishes it; 0 when it publishes none.
    pub window_floor_q15: u16,
    /// Shunt scale; 0 when the sense constants are unset.
    pub amps_per_count: f64,
}

/// Why a drive is refused before anything moves.
#[derive(Clone, Debug, PartialEq)]
pub enum Refusal {
    /// The soft limits are not a calibrated range.
    Uncalibrated { soft: (i32, i32) },
    /// A deliberate stall that would draw more than the current limit.
    StallOverLimit {
        what: String,
        duty: f64,
        stall: f64,
        i_lim: u16,
        allowed: f64,
        ma: Ma,
    },
    /// A TEL stream too long to hold the stall permit through.
    StreamOverLease { ms: u32 },
    /// The servo publishes no window floor.
    NoWindowFloor,
}

/// Counts to milliamps for a message; says nothing when the scale is
/// unknown.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Ma(pub f64);

impl Ma {
    pub fn of(&self, counts: f64) -> String {
        if self.0 > 0.0 {
            format!("{counts:.0} counts ({:.0} mA)", counts * self.0 * 1000.0)
        } else {
            format!("{counts:.0} counts")
        }
    }
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Refusal::Uncalibrated { soft } => write!(
                f,
                "the servo's soft position limits {}..{} are not a calibrated range: run \
                 `osc cal` first",
                soft.0, soft.1
            ),
            Refusal::StallOverLimit {
                what,
                duty,
                stall,
                i_lim,
                allowed,
                ma,
            } => write!(
                f,
                "{what} stalls the motor at {:.0}% duty, which draws about {}, over this servo's \
                 current limit of {}; on this supply the limit holds a stall only up to {:.0}% \
                 duty",
                duty * 100.0,
                ma.of(*stall),
                ma.of(*i_lim as f64),
                (allowed * 100.0).floor()
            ),
            Refusal::StreamOverLease { ms } => write!(
                f,
                "a {ms} ms capture with the stall permit held is longer than the \
                 {PERMIT_STREAM_MAX_MS} ms the permit can be held without a rewrite: shorten \
                 the capture"
            ),
            Refusal::NoWindowFloor => write!(
                f,
                "the servo did not report its current sensor floor, the smallest duty whose \
                 current it can read (window_floor_q15 reads 0): no drive is planned without \
                 it; update the servo's firmware"
            ),
        }
    }
}

impl std::error::Error for Refusal {}

/// The host half of the stall permit lease. The firmware grants a permit
/// written true with torque on for about a second, a rewrite extends it,
/// and torque off drops it; firmware whose permit is a plain level takes
/// the rewrites as no-ops. A pump mirrors every write it sends through
/// [`PermitLease::wrote`] and rewrites the permit whenever it is `due`. The
/// clock is the caller's: pass milliseconds on any monotone timebase.
///
/// On the experiment image there is no permit: the held register is
/// `bus_deadman_ticks`, a keepalive the same cadence feeds, and a stream of
/// any length rides it, since a burst holds the countdown on the servo.
#[derive(Copy, Clone, Debug, Default)]
pub struct PermitLease {
    experiment: bool,
    torque: bool,
    held: bool,
    last_ms: f64,
}

impl PermitLease {
    /// A lease on the production image, or on the experiment image.
    pub fn new(experiment: bool) -> Self {
        Self {
            experiment,
            ..Self::default()
        }
    }

    /// The register the lease is held behind.
    pub fn held_reg(&self) -> Reg {
        if self.experiment {
            control::BUS_DEADMAN_TICKS
        } else {
            control::STALL_PERMIT
        }
    }

    /// Mirror one write sent at `now_ms`.
    pub fn wrote(&mut self, reg: Reg, value: i32, now_ms: f64) {
        if reg == control::TORQUE_ENABLE {
            self.torque = value != 0;
            self.held &= self.torque;
        } else if reg == self.held_reg() {
            // written with torque off it grants nothing
            self.held = value != 0 && self.torque;
            self.last_ms = now_ms;
        }
    }

    #[cfg(test)]
    pub fn held(&self) -> bool {
        self.held
    }

    /// A rewrite of the held permit is due.
    pub fn due(&self, now_ms: f64) -> bool {
        self.held && now_ms - self.last_ms >= PERMIT_REFRESH_MS as f64
    }

    /// Refuse a stream of `ms` the held permit would lapse inside; the
    /// dead-man holds through a burst, so the experiment image refuses none.
    pub fn check_stream(&self, ms: u32) -> Result<(), Refusal> {
        if self.held && !self.experiment && ms > PERMIT_STREAM_MAX_MS {
            return Err(Refusal::StreamOverLease { ms });
        }
        Ok(())
    }
}

impl ServoLimits {
    pub fn ma(&self) -> Ma {
        Ma(self.amps_per_count)
    }

    /// Winding R, vcounts per ccount, once identified and with a rail read.
    pub fn r_vpc(&self) -> Option<f64> {
        (self.r_q12 != 0 && self.vbus != 0).then(|| self.r_q12 as f64 / 4096.0)
    }

    /// Soft limits inside the pot and in order: `osc cal` has run.
    pub fn calibrated(&self) -> bool {
        let (lo, hi) = self.soft;
        !is_board_default(self.soft) && lo >= 0 && hi <= POT_MAX && lo < hi
    }

    /// The guard inside the soft limits.
    pub fn guard(&self) -> Result<(u16, u16), Refusal> {
        if !self.calibrated() {
            return Err(Refusal::Uncalibrated { soft: self.soft });
        }
        Ok(guards((self.soft.0 as u16, self.soft.1 as u16)))
    }

    /// The abort threshold a run takes by default: a quarter over the
    /// limit. The firmware holds a stall AT the limit - window means 0.85
    /// to 1.0 of it, peaks to 1.1 - so an abort at the limit trips on the
    /// hold's own noise.
    pub fn abort_default(&self) -> i16 {
        let lim = self.i_lim as u32;
        (lim + lim / 4).min(i16::MAX as u32) as i16
    }

    /// The travel guard for a drive: each end of `guard` as given, else the
    /// servo's soft limits inset. Refused on an uncalibrated servo either
    /// way.
    pub fn envelope(&self, guard: (Option<u16>, Option<u16>)) -> Result<(u16, u16), Refusal> {
        let inset = self.guard()?;
        Ok((guard.0.unwrap_or(inset.0), guard.1.unwrap_or(inset.1)))
    }

    /// Refuse a deliberate stall at `duty_q15` whose current the limit
    /// cannot hold, by a winding R of `r`, vcounts per ccount.
    pub fn check_stall_at(&self, what: &str, duty_q15: i16, r: f64) -> Result<(), Refusal> {
        let duty = duty_q15.unsigned_abs() as f64 / Q15;
        let vbus = self.vbus as f64;
        let stall = stall_counts(duty, r, vbus);
        if stall <= self.i_lim as f64 {
            return Ok(());
        }
        Err(Refusal::StallOverLimit {
            what: what.into(),
            duty,
            stall,
            i_lim: self.i_lim,
            allowed: duty_for(self.i_lim as f64, r, vbus),
            ma: self.ma(),
        })
    }

    /// Refuse a servo that publishes no window floor: no board constant
    /// stands in for it.
    pub fn check_floor(&self) -> Result<(), Refusal> {
        if self.window_floor_q15 == 0 {
            return Err(Refusal::NoWindowFloor);
        }
        Ok(())
    }

    /// The stall-safe plan for a drive at a stop: from the R the servo
    /// carries, else from the class's lowest, `class_r_vpc`, which errs safe
    /// on any winding of the class.
    pub fn stall_plan(&self, class_r_vpc: f64) -> DutyPlan {
        DutyPlan::new(self, self.r_vpc().unwrap_or(class_r_vpc))
    }

    /// Settings that leave the current limit the only protection a stall
    /// meets, in plain words.
    pub fn warnings(&self) -> Vec<String> {
        let ma = self.ma();
        let lim = ma.of(self.i_lim as f64);
        let mut w = Vec::new();
        if self.i_cont >= self.i_lim {
            w.push(format!(
                "the continuous current, {}, is not below the current limit of {lim}: a press \
                 never trips or backs off (set i_cont_counts under the limit)",
                ma.of(self.i_cont as f64)
            ));
        }
        // u16::MAX is the boot value: no trip until identification sets it
        if self.tau_trip != u16::MAX && self.tau_trip as u32 > 2 * self.i_lim as u32 {
            w.push(format!(
                "the collision trip, {}, is over twice the current limit of {lim}: a collision \
                 never reaches it (set stall_tau_trip_counts near the limit)",
                ma.of(self.tau_trip as f64)
            ));
        }
        w
    }
}

/// The duty a rail of `rail_mv` turns `mv` into.
pub fn duty_of_mv(mv: f64, rail_mv: f64) -> f64 {
    if rail_mv <= 0.0 {
        return 0.0;
    }
    (mv / rail_mv).clamp(0.0, 1.0)
}

/// What a burst may apply. A burst drives the winding for the capture's
/// 0.52 ms and the kernel limiter does not tick through it, so it is bounded
/// instead by the volts it applies.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BurstAllowance;

impl BurstAllowance {
    /// Applied volts, duty x rail, as the firmware caps an arm.
    pub const MAX_MV: f64 = 3200.0;
    /// The share of the volts cap a step is sized under. The rail is read
    /// once, before the run, and the firmware judges each arm on the rail of
    /// that instant in whole vcounts, so a step sized to the cap exactly is
    /// rejected by a rise of one count, 2.5 mV. 1% leaves the rail 79 mV to
    /// rise at 7.9 V and 44 mV at 4.39 V (32 and 18 vcounts) - a pack
    /// recovering from the drive before, and the rail read's own noise.
    pub const ARM_MARGIN: f64 = 0.01;

    /// The most a burst drives on a rail of `rail_mv`: the volts cap less
    /// [`Self::ARM_MARGIN`].
    pub fn top(rail_mv: f64) -> f64 {
        duty_of_mv(Self::MAX_MV * (1.0 - Self::ARM_MARGIN), rail_mv)
    }
}

/// The stall-safe duties a run plans, fractions of full scale: each stalls
/// at or under the current limit on the supply in use. Free rungs are not
/// capped by stall current: the firmware limiter, the stall timer and the
/// travel guard bound them.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DutyPlan {
    /// The seek's duty: the stall-safe cap.
    pub seek: f64,
    /// The highest stall-safe duty: its stall draws the limit. Stop seeks
    /// and the breakout escalation.
    pub stop_cap: f64,
    pub r_vpc: f64,
}

impl DutyPlan {
    pub fn new(lim: &ServoLimits, r_vpc: f64) -> Self {
        let stop_cap = duty_for(lim.i_lim as f64, r_vpc, lim.vbus as f64);
        Self {
            seek: stop_cap,
            stop_cap,
            r_vpc,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bench MG90 as SAVEd: limit 280 with the SG90-era stall settings,
    /// R 4.9 ohm, a 7.9 V rail, 60 mohm at G 15.
    fn mg90() -> ServoLimits {
        ServoLimits {
            i_lim: 280,
            i_cont: 545,
            tau_trip: 2182,
            soft: (432, 3626),
            raw: (209, 3849),
            r_q12: 7270,
            vbus: 3204,
            window_floor_q15: 4356,
            amps_per_count: 3.3 / 4096.0 / (15.0 * 0.060),
        }
    }

    #[test]
    fn guards_come_from_the_soft_limits() {
        assert_eq!(guards((432, 3626)), (532, 3526));
        assert_eq!(mg90().envelope((None, None)).unwrap(), (532, 3526));
        assert_eq!(
            mg90().envelope((Some(600), None)).unwrap(),
            (600, 3526),
            "each end overrides alone"
        );
    }

    #[test]
    fn the_abort_sits_a_quarter_over_the_current_limit() {
        assert_eq!(mg90().abort_default(), 350);
        let big = ServoLimits {
            i_lim: u16::MAX,
            ..mg90()
        };
        assert_eq!(big.abort_default(), i16::MAX);
    }

    #[test]
    fn every_stall_safe_duty_stalls_under_the_limit() {
        let r = 7270.0 / 4096.0;
        for (i_lim, vbus) in [
            (280, 3204),
            (280, 1780),
            (335, 3204),
            (150, 2400),
            (600, 3400),
        ] {
            let lim = ServoLimits {
                i_lim,
                vbus,
                ..mg90()
            };
            let plan = DutyPlan::new(&lim, r);
            for d in [plan.seek, plan.stop_cap] {
                for q in [d, pct_floor(d) as f64 / 100.0] {
                    let stall = stall_counts(q, r, vbus as f64);
                    assert!(
                        stall <= i_lim as f64 + 1e-9,
                        "{plan:?}: {q} stalls at {stall}"
                    );
                }
            }
        }
        let plan = DutyPlan::new(&mg90(), r);
        assert!((plan.stop_cap - 0.1551).abs() < 1e-3);
        assert_eq!(plan.seek, plan.stop_cap);
    }

    #[test]
    fn plan_over_the_limit_is_refused_in_plain_words() {
        let lim = mg90();
        let r = 7270.0 / 4096.0;
        // 12% stalls at 217 counts on 2S: under the limit
        assert_eq!(lim.check_stall_at("the hold", 3932, r), Ok(()));
        let err = lim.check_stall_at("resistance", 11468, r).unwrap_err();
        assert_eq!(
            err.to_string(),
            "resistance stalls the motor at 35% duty, which draws about 632 counts (566 mA), \
             over this servo's current limit of 280 counts (251 mA); on this supply the limit \
             holds a stall only up to 15% duty"
        );
    }

    #[test]
    fn board_default_limits_are_refused() {
        for soft in [(0, 4095), (-4096, 8191), (-10, 3000), (3000, 1000)] {
            let lim = ServoLimits { soft, ..mg90() };
            let err = lim.envelope((None, None)).unwrap_err();
            assert_eq!(err, Refusal::Uncalibrated { soft });
            assert!(err.to_string().ends_with("run `osc cal` first"), "{err}");
        }
    }

    #[test]
    fn a_servo_without_a_floor_is_refused_in_plain_words() {
        assert_eq!(mg90().check_floor(), Ok(()));
        let lim = ServoLimits {
            window_floor_q15: 0,
            ..mg90()
        };
        let err = lim.check_floor().unwrap_err();
        assert_eq!(err, Refusal::NoWindowFloor);
        assert_eq!(
            err.to_string(),
            "the servo did not report its current sensor floor, the smallest duty whose current \
             it can read (window_floor_q15 reads 0): no drive is planned without it; update \
             the servo's firmware"
        );
    }

    #[test]
    fn stall_math_round_trips() {
        let r = 7270.0 / 4096.0;
        let d = duty_for(280.0, r, 3204.0);
        assert!((d - 0.1551).abs() < 1e-3, "{d}");
        assert!((stall_counts(d, r, 3204.0) - 280.0).abs() < 1e-9);
        assert_eq!(stall_counts(0.5, 0.0, 3204.0), f64::INFINITY);
        assert_eq!(duty_for(280.0, r, 0.0), 0.0);
    }

    /// The firmware's arm check on the osc-dev-v006 taps: `(|duty| x
    /// vbus_counts) >> 15` against the 3.2 V cap in vcounts, 1298 at
    /// 2.466 mV a count.
    fn arms(duty_q15: i16, vbus: u32) -> bool {
        (duty_q15.unsigned_abs() as u32 * vbus) >> 15 <= 1298
    }

    /// Every rail from USB to a full 2S pack, a vcount at a time: the top
    /// step sized from the rail as read still arms when the rail has risen
    /// by 1% before the arm. Sized to the cap exactly, the 40% step on a
    /// rail read at 8.00 V is rejected by a rise of 10 mV.
    #[test]
    fn the_top_burst_step_clears_the_cap_on_a_rising_rail() {
        let mv = 3.3 / 4096.0 * (6_800.0 + 3_300.0) / 3_300.0 * 1000.0;
        let q = |pct: u8| (pct as i32 * 32767 / 100) as i16;
        for vbus in 1600u32..=3500 {
            let top = pct_floor(BurstAllowance::top(vbus as f64 * mv));
            let rise = vbus / 100;
            for up in 0..=rise {
                assert!(arms(q(top), vbus + up), "{vbus} + {up}: {top}");
            }
        }
        let at_cap = pct_floor(duty_of_mv(BurstAllowance::MAX_MV, 3244.0 * mv));
        assert_eq!(at_cap, 40);
        assert!(arms(q(40), 3244) && !arms(q(40), 3248));
    }

    #[test]
    fn the_lease_is_held_only_behind_torque_on() {
        let mut l = PermitLease::default();
        l.wrote(control::STALL_PERMIT, 1, 0.0);
        assert!(!l.held(), "a permit written with torque off grants nothing");
        l.wrote(control::TORQUE_ENABLE, 1, 10.0);
        assert!(!l.held());
        l.wrote(control::STALL_PERMIT, 1, 20.0);
        assert!(l.held());
        assert!(!l.due(269.0));
        assert!(l.due(270.0));
        assert_eq!(l.check_stream(750), Ok(()));
        assert_eq!(
            l.check_stream(800).unwrap_err().to_string(),
            "a 800 ms capture with the stall permit held is longer than the 750 ms the permit \
             can be held without a rewrite: shorten the capture"
        );
        l.wrote(control::TORQUE_ENABLE, 0, 30.0);
        assert!(!l.held() && !l.due(1000.0), "torque off drops it");
        assert_eq!(l.check_stream(3000), Ok(()));
    }

    /// On the experiment image the lease is held behind `bus_deadman_ticks`
    /// at the permit's cadence, the permit byte holds nothing, no stream is
    /// refused, and torque off drops it.
    #[test]
    fn the_experiment_lease_is_the_deadman() {
        let mut l = PermitLease::new(true);
        assert_eq!(l.held_reg(), control::BUS_DEADMAN_TICKS);
        l.wrote(control::TORQUE_ENABLE, 1, 0.0);
        l.wrote(control::STALL_PERMIT, 1, 10.0);
        assert!(!l.held(), "the permit byte is not the lease here");
        l.wrote(control::BUS_DEADMAN_TICKS, DEADMAN_TICKS.into(), 20.0);
        assert!(l.held());
        assert!(!l.due(269.0));
        assert!(l.due(270.0));
        assert_eq!(l.check_stream(3000), Ok(()), "a burst holds the count");
        l.wrote(control::BUS_DEADMAN_TICKS, 0, 30.0);
        assert!(!l.held(), "a zero field is off");
        l.wrote(control::BUS_DEADMAN_TICKS, DEADMAN_TICKS.into(), 40.0);
        l.wrote(control::TORQUE_ENABLE, 0, 50.0);
        assert!(!l.held() && !l.due(1000.0), "torque off drops it");
        let mut off = PermitLease::new(true);
        off.wrote(control::BUS_DEADMAN_TICKS, DEADMAN_TICKS.into(), 0.0);
        assert!(!off.held(), "written with torque off it holds nothing");
        assert_eq!(PermitLease::new(false).held_reg(), control::STALL_PERMIT);
    }

    /// The bench MG90's SG90-era continuous current and trip: both warned
    /// about; the boot trip value (off) is not.
    #[test]
    fn stall_settings_above_the_limit_are_warned() {
        let w = mg90().warnings();
        assert_eq!(w.len(), 2, "{w:?}");
        assert_eq!(
            w[0],
            "the continuous current, 545 counts (488 mA), is not below the current limit of \
             280 counts (251 mA): a press never trips or backs off (set i_cont_counts under \
             the limit)"
        );
        assert!(w[1].starts_with("the collision trip, 2182 counts (1953 mA), is over twice"));
        let tidy = ServoLimits {
            i_cont: 168,
            tau_trip: 280,
            ..mg90()
        };
        assert!(tidy.warnings().is_empty());
        let unset = ServoLimits {
            i_cont: 168,
            tau_trip: u16::MAX,
            ..mg90()
        };
        assert!(unset.warnings().is_empty());
    }
}
