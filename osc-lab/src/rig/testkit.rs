//! The bench servo in-process for the tests: its table is the in-process
//! servo stack's, behind osc-client's fake adapter, and a drive's clock is
//! the test's own, so nothing sleeps.

use anyhow::Result;
use osc_client::blocking::Client;
use osc_client::fake::{FakePipe, TelSample};
use osc_client::{BaudRate, Id, Inst, Opcode};
use osc_protocol::tel::Chunks;
use osc_servo_core::kernel::phases::loops::faults::{BIT_DEADMAN, CODE_DEADMAN};

use super::pump::{BurstCap, BurstStats, exchange_tel_burst, read_snapshot, write_reg};
use super::servo::{Between, Memo, Servo};
use crate::instrument::frame::{TelFrame, TelemetrySnapshot};
use crate::instrument::regs::{Reg, calib, config, control};
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::regions::control::{INJECT_HALF_PERIODS_MAX, Mode as CtMode};

/// The in-process servo stack at `rate`, calibrated and identified like the
/// bench servo: limit 280, speed ceiling 1500 c/s, stall yield 168
/// released under 84, soft limits 432..3626 inside stops 232..3849, its
/// rail ADC reading `vbus_raw` (1961 is 7.9 V, 1090 4.39 V) and its
/// published rail 3204 vcounts.
pub(crate) fn table_at(rate: BaudRate, vbus_raw: u16) -> (Client<FakePipe>, Id) {
    let mut pipe = FakePipe::new(rate, &[1]);
    pipe.seed_calibrated(0);
    let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
    rail_at(&mut c, vbus_raw, 3204);
    c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
        t.config.loop_position.speed_ceiling_cps = 1500;
    });
    for (reg, v) in [
        (config::CURRENT_LIMIT_COUNTS, 280),
        (config::STALL_RESPONSE, 1),
        (config::YIELD_CURRENT_COUNTS, 168),
        (config::STALL_RELEASE_COUNTS, 84),
        (config::POS_MIN_SOFT_COUNTS, 432),
        (config::POS_MAX_SOFT_COUNTS, 3626),
        (calib::RAW_MIN, 232),
        (calib::RAW_MAX, 3849),
    ] {
        write_reg(&mut c, id, reg, v).unwrap();
    }
    // the sim runs no kernel to publish it
    c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
        t.telemetry.limits.window_floor_q15 = 4356;
    });
    (c, id)
}

/// [`table_at`] on a 1 Mbaud bus.
pub(crate) fn table(vbus_raw: u16) -> (Client<FakePipe>, Id) {
    table_at(BaudRate::B1000000, vbus_raw)
}

/// Every later read of the rail ADC sees `vbus_raw`, and of the published
/// rail `vbus_counts`, the shaft at 2029.
pub(crate) fn rail_at(c: &mut Client<FakePipe>, vbus_raw: u16, vbus_counts: u16) {
    let mut s = TelSample {
        vbus_counts,
        ..TelSample::default()
    };
    (s.tick.pos, s.tick.vbus_raw) = (2029, vbus_raw);
    c.pipe_mut().set_track(0, vec![s]);
}

/// The loops a margin sweep closes, played on the rig: a Position goal is
/// where the shaft goes at once; in Velocity mode a burst moves it at the
/// committed goal, the observer's speed half the reference's departure from
/// the goal a period late and its position the speed's sum. On an image
/// that honors the stimulus, under torque and a loaded dead-man with both
/// fields set, the reference carries the kernel's triangle; on any other it
/// reads as the table holds it.
#[derive(Copy, Clone, Debug)]
pub(crate) struct Plant {
    pub(crate) shaft: i32,
    pub(crate) honors_stimulus: bool,
}

/// What a test does to the table after a burst, given the burst's index.
pub(crate) type AfterBurst = Box<dyn FnMut(&mut Client<FakePipe>, u32)>;

/// How a timed rig's clock moves on its own, for a test of a drive's
/// timing: every exchange takes `exchange_ms`, a burst its rows' span, the
/// host stalls a `stalls` entry's ms ahead of the staged writes of the
/// burst it names, and the servo's tick runs at `tick_ratio` of its
/// nominal rate on that clock. A carrier lands `landing_ms.0` after the
/// host sent it, give or take a seeded normal scatter of sd `landing_ms.1`,
/// never before it was sent: the burst starts there, and its `stamp0` is
/// the servo's tick there.
#[derive(Clone, Debug)]
pub(crate) struct Timed {
    pub(crate) exchange_ms: f64,
    pub(crate) stalls: Vec<(u32, f64)>,
    pub(crate) tick_ratio: f64,
    pub(crate) landing_ms: (f64, f64),
}

/// An instruction the host put on the wire: a READ's span, the address of
/// a WRITE or of one staged under HOLD, a COMMIT.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Sent {
    Read(u16, u16),
    Write(u16),
    Hold(u16),
    Commit,
}

/// The scatter's seed: any odd constant, the same run after run.
const SEED: u64 = 0x9e37_79b9_7f4a_7c15;

/// The bench servo as a drive sees it: the table on the fake adapter at
/// 3 Mbaud, a clock that moves only when the drive sleeps or waits for a
/// slot unless the rig is [`Timed`], every write logged, and `drop` TEL
/// rows lost from each burst.
///
/// The sim runs no kernel, so with `deadman_honored` the rig plays the
/// dead-man's part of one: bus silence under torque for the count last
/// written latches the `deadman` fault, and a torque enable acks it. With
/// a [`Plant`] it plays the loops' part too ([`Rig::plant_at`]).
pub(crate) struct Rig {
    pub(crate) c: Client<FakePipe>,
    id: Id,
    pub(crate) t_ms: f64,
    pub(crate) writes: Vec<(Reg, i32)>,
    pub(crate) experiment: bool,
    pub(crate) drop: u16,
    pub(crate) cap: Option<BurstCap>,
    pub(crate) deadman_honored: bool,
    pub(crate) after_burst: Option<AfterBurst>,
    pub(crate) timed: Option<Timed>,
    /// What each timed burst's carrier took to land, ms.
    pub(crate) landings: Vec<f64>,
    pub(crate) plant: Option<Plant>,
    rng: u64,
    memo: Memo,
    bursts: u32,
    torque: bool,
    deadman_ticks: u16,
    silent_ms: f64,
}

impl Rig {
    pub(crate) fn bench() -> Self {
        let (c, id) = table_at(BaudRate::B3000000, 1961);
        Self {
            c,
            id,
            t_ms: 0.0,
            writes: Vec::new(),
            experiment: false,
            drop: 0,
            cap: None,
            deadman_honored: false,
            after_burst: None,
            timed: None,
            landings: Vec::new(),
            plant: None,
            rng: SEED,
            memo: Memo::default(),
            bursts: 0,
            torque: false,
            deadman_ticks: 0,
            silent_ms: 0.0,
        }
    }

    /// The bench rig, refusing no mode, with a [`Plant`] whose shaft
    /// rests at `shaft` linearized counts (the bench table is the identity).
    pub(crate) fn plant_at(shaft: i32, honors_stimulus: bool) -> Self {
        let mut r = Self::bench();
        r.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.mode.refused_modes = 0;
        });
        r.plant = Some(Plant {
            shaft,
            honors_stimulus,
        });
        r.publish_shaft();
        r
    }

    /// The plant's shaft at rest in the table: the pot sample every read
    /// and burst sees, at room temperature on the bench rail, and the
    /// observer's estimate.
    fn publish_shaft(&mut self) {
        let Some(p) = &self.plant else {
            return;
        };
        let shaft = p.shaft;
        let mut s = TelSample {
            vbus_counts: 3204,
            ..TelSample::default()
        };
        (s.tick.pos, s.tick.vbus_raw, s.tick.ntc_raw) = (shaft as u16, 1961, 1913);
        self.c.pipe_mut().set_track(0, vec![s]);
        self.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.estimates.theta_hat_q16 = shaft << 16;
            t.telemetry.estimates.omega_hat_cps = 0;
        });
    }

    /// A burst's rows as the plant drives them in Velocity mode: the shaft
    /// at the committed goal from the burst's first row, and, on an image
    /// that honors the stimulus, the triangle on the reference as the
    /// kernel sums it ([`Plant`]).
    fn drive_plant(&mut self, frames: &mut [TelFrame]) {
        let Some(p) = self.plant else {
            return;
        };
        let (mode, goal, accel, half, deadman) =
            self.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
                let c = &t.control;
                (
                    c.lifecycle.mode,
                    c.lifecycle.goal_velocity,
                    c.inject.inject_accel_q88,
                    c.inject.inject_half_periods,
                    c.lifecycle.bus_deadman_ticks,
                )
            });
        if mode != CtMode::Velocity || frames.is_empty() {
            return;
        }
        let on = p.honors_stimulus && self.torque && deadman > 0 && accel > 0 && half > 0;
        let n = half.min(INJECT_HALF_PERIODS_MAX) as i64;
        let fm = (osc_servo_core::kernel::TICK_HZ / DECIM_MED as u32) as i64;
        let g = (goal as i64) << 16;
        let periods = (frames.last().map_or(0, |f| f.tick) / DECIM_MED as u64 + 1) as usize;
        let (mut phase, mut prev, mut theta) = (0i64, g, (p.shaft as i64) << 16);
        let mut at = Vec::with_capacity(periods);
        for _ in 0..periods {
            let mut r = g;
            if on {
                phase += 1;
                if phase >= 2 * n {
                    phase = 0;
                }
                r += (accel as i64 * ((n >> 1) - (phase - n).abs())) << 8;
            }
            let w = g + (prev - g) / 2;
            at.push((theta, w, r));
            theta += w / fm;
            prev = r;
        }
        for f in frames.iter_mut() {
            let (th, w, r) = at[(f.tick / DECIM_MED as u64) as usize];
            f.theta_hat_q16 = f.theta_hat_q16.map(|_| th as i32);
            f.omega_hat_cps = f.omega_hat_cps.map(|_| w as i32);
            if on {
                f.omega_ref_cps = f.omega_ref_cps.map(|_| r as i32);
            }
        }
        if let Some(p) = self.plant.as_mut() {
            p.shaft = (theta >> 16) as i32;
        }
        self.publish_shaft();
    }

    /// The `fault_flags` bit and `fault_code` the kernel latches, or 0, 0.
    pub(crate) fn latch(&mut self, flags: u8, code: u8) {
        self.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.common.fault_flags = flags;
            t.telemetry.mode.fault_code = code;
        });
    }

    /// Every instruction the host put on the wire since the last call, in
    /// order: one exchange each.
    pub(crate) fn sent(&mut self) -> Vec<Sent> {
        let frames = self.c.pipe_mut().take_frames();
        let sent = |b: &[u8]| {
            // the break's ring byte, ID, LEN, INST, then the payload
            let [_, _, _, inst, payload @ ..] = b else {
                return None;
            };
            let inst = Inst(*inst);
            let at = |k: usize| Some(u16::from_le_bytes([*payload.get(k)?, *payload.get(k + 1)?]));
            match inst.opcode()? {
                Opcode::Read => Some(Sent::Read(at(0)?, at(2)?)),
                Opcode::Write if inst.0 & Inst::FLAG_HOLD != 0 => Some(Sent::Hold(at(0)?)),
                Opcode::Write => Some(Sent::Write(at(0)?)),
                Opcode::Commit => Some(Sent::Commit),
                _ => None,
            }
        };
        frames.iter().filter_map(|f| sent(&f.bytes)).collect()
    }

    /// The span, address and length, of every READ among [`Rig::sent`].
    pub(crate) fn reads(&mut self) -> Vec<(u16, u16)> {
        let read = |s: Sent| match s {
            Sent::Read(addr, len) => Some((addr, len)),
            _ => None,
        };
        self.sent().into_iter().filter_map(read).collect()
    }

    /// One exchange's time on a timed rig's clock.
    fn exchange(&mut self) {
        self.silent_ms = 0.0;
        if let Some(t) = &self.timed {
            self.t_ms += t.exchange_ms;
        }
    }

    /// A standard normal draw, near enough: twelve uniform draws of a
    /// xorshift summed, less their mean.
    fn normal(&mut self) -> f64 {
        let mut sum = 0.0;
        for _ in 0..12 {
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 7;
            self.rng ^= self.rng << 17;
            sum += (self.rng >> 11) as f64 / (1u64 << 53) as f64;
        }
        sum - 6.0
    }

    /// `ms` of bus silence: under torque, on a rig that honors the
    /// dead-man, the count last written running out latches its fault.
    fn silence(&mut self, ms: f64) {
        self.silent_ms += ms;
        let ticks_per_ms = osc_servo_core::kernel::TICK_HZ as f64 / 1000.0;
        if self.deadman_honored
            && self.torque
            && self.deadman_ticks > 0
            && self.silent_ms * ticks_per_ms >= self.deadman_ticks as f64
        {
            self.latch(BIT_DEADMAN, CODE_DEADMAN);
        }
    }
}

impl Servo for Rig {
    type P = FakePipe;

    fn client(&mut self) -> &mut Client<FakePipe> {
        self.exchange();
        &mut self.c
    }

    fn id(&self) -> Id {
        self.id
    }

    fn now_ms(&self) -> f64 {
        self.t_ms
    }

    fn write(&mut self, reg: Reg, value: i32) -> Result<()> {
        self.writes.push((reg, value));
        self.exchange();
        if reg == control::TORQUE_ENABLE {
            let on = value != 0;
            if on && !self.torque && self.deadman_honored {
                self.latch(0, 0);
            }
            self.torque = on;
        } else if reg == control::BUS_DEADMAN_TICKS {
            self.deadman_ticks = value as u16;
        }
        write_reg(&mut self.c, self.id, reg, value)?;
        if reg == control::GOAL_POSITION
            && let Some(p) = self.plant.as_mut()
        {
            p.shaft = value;
            self.publish_shaft();
        }
        Ok(())
    }

    fn snapshot(&mut self) -> Result<TelemetrySnapshot> {
        self.exchange();
        read_snapshot(&mut self.c, self.id)
    }

    /// The rows the servo drops land in the table's `tel_drop_count`, where
    /// [`Servo::stream`] reads them. The wait for an instant is bus silence.
    fn burst(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        chunks: Chunks,
        not_before_ms: Option<f64>,
        between: Option<Between<'_, FakePipe>>,
    ) -> Result<(Vec<TelFrame>, BurstStats)> {
        if let Some((reg, value)) = goal {
            self.writes.push((reg, value));
        }
        let line = self.tel_line()?;
        let timed = self.timed.clone();
        let mut staged = self.t_ms;
        if let Some(t) = &timed {
            let stall = t.stalls.iter().find(|s| s.0 == self.bursts);
            staged += stall.map_or(0.0, |s| s.1);
            if goal.is_some() {
                staged += 2.0 * t.exchange_ms;
            }
            if between.is_some() {
                staged += t.exchange_ms;
            }
        }
        let sent = not_before_ms.map_or(staged, |at| at.max(staged));
        let r = exchange_tel_burst(&mut self.c, self.id, line, samples, goal, chunks, |c| {
            between.map_or(Ok(()), |between| between(c))
        })?;
        let (mut frames, mut stats) = r;
        self.drive_plant(&mut frames);
        (stats.staged_ms, stats.sent_ms) = (staged, sent);
        self.t_ms = sent;
        self.silent_ms = 0.0;
        self.silence(sent - staged);
        self.silent_ms = 0.0;
        if let Some(t) = &timed {
            let (mean, sd) = t.landing_ms;
            let landing = (mean + sd * self.normal()).max(0.0);
            self.landings.push(landing);
            let ticks_per_ms = osc_servo_core::kernel::TICK_HZ as f64 / 1000.0 * t.tick_ratio;
            stats.stamp0 = Some(((sent + landing) * ticks_per_ms).round() as u64 as u16);
            self.t_ms += landing + samples as f64 / ticks_per_ms;
        }
        let r = (frames, stats);
        if let Some(f) = self.after_burst.as_mut() {
            f(&mut self.c, self.bursts);
        }
        self.bursts += 1;
        let drop = self.drop;
        self.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            let h = &mut t.telemetry_ext.health;
            h.tel_drop_count = h.tel_drop_count.wrapping_add(drop);
        });
        Ok(r)
    }

    fn sleep(&mut self, ms: u32) {
        self.t_ms += ms as f64;
        self.silence(ms as f64);
    }

    fn memo(&mut self) -> &mut Memo {
        &mut self.memo
    }

    fn burst_cap(&mut self) -> Result<BurstCap> {
        if let Some(cap) = self.cap {
            return Ok(cap);
        }
        let d = crate::state::descriptor(&mut self.c, self.id)?;
        let cap = BurstCap::of(&d, self.tick_hz()?)?;
        self.cap = Some(cap);
        Ok(cap)
    }

    fn experiment(&mut self) -> Result<bool> {
        Ok(self.experiment)
    }
}
