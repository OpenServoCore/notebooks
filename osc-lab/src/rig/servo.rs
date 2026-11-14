//! The servo a drive talks to: its table over the bus for everything that
//! moves nothing, and the moves a drive makes - a write, a telemetry read,
//! a TEL burst, a wait - on the one clock the servo moves on. On the bus
//! that clock is the wall; a test servo keeps its own. Every TEL burst
//! carries the rows the servo dropped from it, read off its table, and
//! what a burst needs of the servo that no drive changes is read once.

use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;
use osc_protocol::tel::Chunks;

use super::pump::{
    BurstCap, BurstStats, TelLine, exchange_tel_burst, identity, read_snapshot, write_reg,
};
use super::snapshot::read_u16;
use crate::instrument::frame::{TelFrame, TelemetrySnapshot};
use crate::instrument::regs::{Reg, calib, control, telemetry};

/// What a drive reads off its servo once and keeps.
#[derive(Default)]
pub(crate) struct Memo {
    line: Option<TelLine>,
    /// `tel_drop_count` as the last stream left it.
    drops: Option<u16>,
}

pub(crate) trait Servo {
    type P: Pipe;

    /// The table, for reads and writes that move nothing.
    fn client(&mut self) -> &mut Client<Self::P>;
    fn id(&self) -> Id;
    /// Ms on the clock the moves run on.
    fn now_ms(&self) -> f64;
    fn write(&mut self, reg: Reg, value: i32) -> Result<()>;
    fn snapshot(&mut self) -> Result<TelemetrySnapshot>;
    /// One TEL burst of `samples` fast ticks of `chunks` rows, the `goal`
    /// write applied in the instant the burst starts. With `not_before_ms`
    /// the carrier waits for that instant on the servo's clock
    /// ([`Servo::now_ms`]) behind the staged writes, and goes out at once
    /// when they are staged after it. The stats say when the writes were
    /// staged and when the carrier went out. Drives call [`Servo::stream`].
    fn burst(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        chunks: Chunks,
        not_before_ms: Option<f64>,
    ) -> Result<(Vec<TelFrame>, BurstStats)>;
    fn sleep(&mut self, ms: u32);
    fn memo(&mut self) -> &mut Memo;

    /// The line a burst streams on ([`TelLine`]), read once.
    fn tel_line(&mut self) -> Result<TelLine> {
        if let Some(line) = self.memo().line {
            return Ok(line);
        }
        let id = self.id();
        let line = TelLine::read(self.client(), id)?;
        self.memo().line = Some(line);
        Ok(line)
    }

    /// The servo's fast tick rate, Hz: one TEL row each.
    fn tick_hz(&mut self) -> Result<f64> {
        let id = self.id();
        Ok(read_u16(self.client(), id, calib::TICK_HZ)? as f64)
    }

    /// The longest TEL burst the servo arms ([`BurstCap`]).
    fn burst_cap(&mut self) -> Result<BurstCap>;

    /// The servo runs the experiment image ([`identity`]): its lease is the
    /// bus dead-man, not the stall permit (`rig::pump::Lease`).
    fn experiment(&mut self) -> Result<bool>;

    /// The PWM period the servo's TEL duty counts are of.
    fn pwm_period_ticks(&mut self) -> Result<u16> {
        let id = self.id();
        match read_u16(self.client(), id, calib::PWM_PERIOD_TICKS)? {
            0 => bail!("the servo publishes no PWM period: its firmware predates TEL duty counts"),
            p => Ok(p),
        }
    }

    /// [`Servo::burst`], its stats carrying the rows the servo dropped from
    /// it: `tel_drop_count` after it less the count the stream before it
    /// left, read ahead of a drive's first. Rows drop only inside a stream,
    /// every stream of a drive is one of these and no drive writes the
    /// counter, so a drive's streams add up to every row dropped: one the
    /// servo publishes after the read, or one dropped from a stream that
    /// failed, is in the next stream's count. Firmware without the health
    /// block reads 0 there, so it drops none. A burst over the servo's cap
    /// is refused before anything is sent.
    fn stream_at(
        &mut self,
        samples: u32,
        goal: Option<(Reg, i32)>,
        chunks: Chunks,
        not_before_ms: Option<f64>,
    ) -> Result<(Vec<TelFrame>, BurstStats)> {
        let samples = self.burst_cap()?.check(samples)?;
        let id = self.id();
        let before = match self.memo().drops {
            Some(n) => n,
            None => read_u16(self.client(), id, telemetry::TEL_DROP_COUNT)?,
        };
        let (frames, mut stats) = self.burst(samples, goal, chunks, not_before_ms)?;
        let after = read_u16(self.client(), id, telemetry::TEL_DROP_COUNT)?;
        self.memo().drops = Some(after);
        stats.rows_dropped = after.wrapping_sub(before);
        Ok((frames, stats))
    }

    /// [`Servo::stream_at`] as soon as the host is ready.
    fn stream(
        &mut self,
        samples: u32,
        goal: Option<(Reg, i32)>,
        chunks: Chunks,
    ) -> Result<(Vec<TelFrame>, BurstStats)> {
        self.stream_at(samples, goal, chunks, None)
    }
}

/// A client, owned or borrowed.
pub(crate) trait AsClient {
    type P: Pipe;
    fn get(&mut self) -> &mut Client<Self::P>;
}

impl<P: Pipe> AsClient for Client<P> {
    type P = P;
    fn get(&mut self) -> &mut Client<P> {
        self
    }
}

impl<P: Pipe> AsClient for &mut Client<P> {
    type P = P;
    fn get(&mut self) -> &mut Client<P> {
        self
    }
}

/// The servo on the bus, on the wall clock.
pub(crate) struct Wire<C> {
    c: C,
    id: Id,
    t0: Instant,
    cap: Option<BurstCap>,
    experiment: Option<bool>,
    memo: Memo,
}

impl<C: AsClient> Wire<C> {
    pub(crate) fn new(c: C, id: Id) -> Self {
        Self {
            c,
            id,
            t0: Instant::now(),
            cap: None,
            experiment: None,
            memo: Memo::default(),
        }
    }
}

fn ms_since(t0: Instant) -> f64 {
    t0.elapsed().as_secs_f64() * 1000.0
}

/// The last of a wait spins.
const SPIN: Duration = Duration::from_millis(1);

/// Wait to `ms` past `t0`, to the instant. A sleep returns late, and on a
/// host that coalesces timers by half its own length, so each sleep is
/// half of what is left of the wait and the last [`SPIN`] of it spins.
fn wait_until(t0: Instant, ms: f64) {
    let at = t0 + Duration::from_secs_f64(ms.max(0.0) / 1000.0);
    loop {
        let left = at.saturating_duration_since(Instant::now());
        if left <= SPIN {
            break;
        }
        std::thread::sleep(left / 2);
    }
    while Instant::now() < at {
        std::hint::spin_loop();
    }
}

impl<C: AsClient> Servo for Wire<C> {
    type P = C::P;

    fn client(&mut self) -> &mut Client<C::P> {
        self.c.get()
    }

    fn id(&self) -> Id {
        self.id
    }

    fn now_ms(&self) -> f64 {
        ms_since(self.t0)
    }

    fn write(&mut self, reg: Reg, value: i32) -> Result<()> {
        write_reg(self.c.get(), self.id, reg, value)
    }

    fn snapshot(&mut self) -> Result<TelemetrySnapshot> {
        read_snapshot(self.c.get(), self.id)
    }

    fn burst(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        chunks: Chunks,
        not_before_ms: Option<f64>,
    ) -> Result<(Vec<TelFrame>, BurstStats)> {
        let line = self.tel_line()?;
        let t0 = self.t0;
        let (mut staged, mut sent) = (0.0, 0.0);
        let r = exchange_tel_burst(self.c.get(), self.id, line, samples, goal, chunks, || {
            staged = ms_since(t0);
            sent = match not_before_ms {
                Some(at) if at > staged => {
                    wait_until(t0, at);
                    ms_since(t0)
                }
                _ => staged,
            };
        });
        let (frames, mut stats) = r?;
        (stats.staged_ms, stats.sent_ms) = (staged, sent);
        Ok((frames, stats))
    }

    fn sleep(&mut self, ms: u32) {
        std::thread::sleep(Duration::from_millis(ms as u64));
    }

    fn memo(&mut self) -> &mut Memo {
        &mut self.memo
    }

    /// Read once, at the first burst.
    fn burst_cap(&mut self) -> Result<BurstCap> {
        if let Some(cap) = self.cap {
            return Ok(cap);
        }
        let d = crate::state::descriptor(self.c.get(), self.id)?;
        let cap = BurstCap::of(&d, self.tick_hz()?)?;
        self.cap = Some(cap);
        Ok(cap)
    }

    /// Read once, at the first lease: the connect line says so.
    fn experiment(&mut self) -> Result<bool> {
        if let Some(experiment) = self.experiment {
            return Ok(experiment);
        }
        let experiment = identity(self.c.get(), self.id)?.experiment();
        if experiment {
            eprintln!(
                "id {}: experiment image, the bus dead-man in place of the stall permit",
                self.id.as_byte()
            );
        }
        self.experiment = Some(experiment);
        Ok(experiment)
    }
}

/// Run `f`, then force the servo safe (duty and goals zero, torque, stall
/// permit and TEL off) whether it succeeded, failed, or was ctrl-c'd.
/// A hard kill skips this - a leased permit then runs out within a second
/// (a plain-level one stays until a reboot), and the servo's own
/// protections are the backstop.
pub(crate) fn guard<S: Servo, T>(s: &mut S, f: impl FnOnce(&mut S) -> Result<T>) -> Result<T> {
    let r = f(s);
    for (reg, v) in SAFE {
        let _ = s.write(reg, v);
    }
    r
}

/// What leaves a servo safe, in the order it is written.
pub(crate) const SAFE: [(Reg, i32); 6] = [
    (control::GOAL_DUTY, 0),
    (control::GOAL_CURRENT, 0),
    (control::GOAL_VELOCITY, 0),
    (control::TORQUE_ENABLE, 0),
    (control::STALL_PERMIT, 0),
    (control::TEL_COUNT, 0),
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::instrument::frame::TEL_RAW;
    use crate::rig::testkit::{self, Rig};

    /// A stream reads the rows the servo dropped from it off the table: the
    /// counter's change across it, through its wrap. The counter is read
    /// once ahead of the first stream and once after each, and every row
    /// dropped is in one stream's count.
    #[test]
    fn a_stream_carries_the_rows_the_servo_dropped_from_it() {
        let mut b = Rig::bench();
        b.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry_ext.health.tel_drop_count = u16::MAX - 4;
        });
        b.reads();
        let mut dropped = Vec::new();
        for drop in [0, 3, 0, 2, 5] {
            b.drop = drop;
            dropped.push(b.stream(200, None, TEL_RAW).unwrap().1.rows_dropped);
        }
        assert_eq!(dropped, [0, 3, 0, 2, 5]);
        let counter = (telemetry::TEL_DROP_COUNT.addr, 2);
        assert_eq!(b.reads().iter().filter(|r| **r == counter).count(), 6);
        let id = b.id();
        assert_eq!(
            read_u16(&mut b.c, id, telemetry::TEL_DROP_COUNT).unwrap(),
            5,
            "through the wrap"
        );
    }

    /// What a burst needs of the servo ahead of its arm is read once, at
    /// the first burst: the identity block and the window floor. A burst
    /// after it reads the drop counter alone.
    #[test]
    fn the_line_a_burst_streams_on_is_read_once() {
        use osc_protocol::table::{BAUD_RATE_IDX, CAPABILITY_FLAGS};
        let identity = (CAPABILITY_FLAGS, BAUD_RATE_IDX + 1 - CAPABILITY_FLAGS);
        let floor = (calib::I_WINDOW_MIN_TICKS.addr, 2);
        let mut b = Rig::bench();
        b.reads();
        b.stream(200, None, TEL_RAW).unwrap();
        let first = b.reads();
        let count = |span| first.iter().filter(|r| **r == span).count();
        assert_eq!((count(identity), count(floor)), (1, 1));
        for _ in 0..3 {
            b.stream(200, Some((control::GOAL_DUTY, 0)), TEL_RAW)
                .unwrap();
            assert_eq!(b.reads(), [(telemetry::TEL_DROP_COUNT.addr, 2)]);
        }
    }

    /// On the wire a carrier with an instant ahead goes out no sooner than
    /// it, and one staged after its instant goes out at once.
    #[test]
    fn the_wire_waits_for_an_instant_ahead_and_no_other() {
        let (mut c, id) = testkit::table_at(osc_client::BaudRate::B3000000, 1961);
        let mut w = Wire::new(&mut c, id);
        let goal = Some((control::GOAL_DUTY, 0));
        let (_, st) = w.stream(40, goal, TEL_RAW).unwrap();
        assert_eq!(st.sent_ms, st.staged_ms);
        let ahead = w.now_ms() + 15.0;
        let (_, st) = w.stream_at(40, goal, TEL_RAW, Some(ahead)).unwrap();
        assert!(st.sent_ms >= ahead && st.sent_ms >= st.staged_ms);
        let past = w.now_ms() - 1.0;
        let (_, st) = w.stream_at(40, goal, TEL_RAW, Some(past)).unwrap();
        assert_eq!(st.sent_ms, st.staged_ms);
    }

    /// The cap is the descriptor's `tel_count` max, else the field's whole
    /// range, timed by the servo's own tick rate.
    #[test]
    fn the_burst_cap_comes_from_the_descriptor_and_the_tick_rate() {
        let mut d = crate::descriptor::builtin();
        let mut set = |max: Option<i64>| {
            for f in d.fields.iter_mut().filter(|f| f.name == "tel_count") {
                f.max = max;
            }
            d.clone()
        };
        let cap = BurstCap::of(&set(Some(20_000)), 20_000.0).unwrap();
        assert_eq!((cap.rows, cap.ms()), (20_000, 1000.0));
        assert_eq!(
            BurstCap::of(&set(Some(20_000)), 10_000.0).unwrap().ms(),
            2000.0
        );
        assert_eq!(BurstCap::of(&set(Some(5000)), 20_000.0).unwrap().rows, 5000);
        assert_eq!(BurstCap::of(&set(None), 20_000.0).unwrap().rows, u16::MAX);
        assert!(BurstCap::of(&set(Some(0)), 20_000.0).is_err());
        assert!(BurstCap::of(&set(None), 0.0).is_err());
    }

    /// A burst over the servo's cap is refused before anything is sent,
    /// in rows and ms, on servos of more than one cap and tick rate; one
    /// at the cap streams.
    #[test]
    fn a_burst_over_the_cap_is_refused_before_the_arm() {
        for (rows, tick_hz) in [(20_000, 20_000.0), (4000, 10_000.0)] {
            let mut b = Rig::bench();
            b.cap = Some(BurstCap { rows, tick_hz });
            let over = rows as u32 + 1;
            let e = b.stream(over, None, TEL_RAW).err().unwrap().to_string();
            let ms = |n: u32| format!("{:.0}", n as f64 * 1000.0 / tick_hz);
            assert_eq!(
                e,
                format!(
                    "a TEL burst of {} rows ({} ms) is over the {rows} rows ({} ms) the servo \
                     arms at most (its tel_count max at {tick_hz:.0} Hz): refused before the arm",
                    over,
                    ms(over),
                    ms(rows.into())
                )
            );
            let id = b.id();
            assert_eq!(read_u16(&mut b.c, id, control::TEL_COUNT).unwrap(), 0);
            assert!(b.stream(400, None, TEL_RAW).is_ok());
        }

        // on the wire the cap is read off the servo's own descriptor, and
        // the arm never goes out
        let (mut c, id) = testkit::table(1961);
        let mut w = Wire::new(&mut c, id);
        let cap = w.burst_cap().unwrap();
        let over = cap.rows as u32 + 1;
        let e = w.stream(over, None, TEL_RAW).err().unwrap().to_string();
        assert!(e.starts_with(&format!("a TEL burst of {over} rows")), "{e}");
        assert_eq!(read_u16(&mut c, id, control::TEL_COUNT).unwrap(), 0);
        assert_eq!(read_u16(&mut c, id, control::CHUNK0_ADDR).unwrap(), 0);
    }

    /// The guard writes SAFE whatever the drive returned.
    #[test]
    fn the_guard_leaves_the_servo_safe_on_any_exit() {
        let mut b = Rig::bench();
        let r: Result<()> = guard(&mut b, |s| {
            s.write(control::TORQUE_ENABLE, 1)?;
            bail!("interrupted")
        });
        assert_eq!(r.unwrap_err().to_string(), "interrupted");
        assert_eq!(b.writes[1..], SAFE);
        let id = b.id();
        assert_eq!(b.c.read(id, control::TORQUE_ENABLE.addr, 1).unwrap(), [0]);
    }
}
