//! The bench servo in-process for the tests: its table is the in-process
//! servo stack's, behind osc-client's fake adapter, and a drive's clock is
//! the test's own, so nothing sleeps.

use anyhow::Result;
use osc_client::blocking::Client;
use osc_client::fake::{FakePipe, TelSample};
use osc_client::{BaudRate, Id};
use osc_protocol::tel::Chunks;

use super::pump::{BurstCap, BurstStats, exchange_tel_burst, read_snapshot, write_reg};
use super::servo::Servo;
use crate::instrument::frame::{TelFrame, TelemetrySnapshot};
use crate::instrument::regs::{Reg, calib, config};

/// The in-process servo stack at `rate`, calibrated and identified like the
/// bench servo: limit 280, stall yield 168 released under 84, soft limits
/// 432..3626 inside stops 232..3849, its rail ADC reading `vbus_raw`
/// (1961 is 7.9 V, 1090 4.39 V) and its published rail 3204 vcounts.
pub(crate) fn table_at(rate: BaudRate, vbus_raw: u16) -> (Client<FakePipe>, Id) {
    let mut pipe = FakePipe::new(rate, &[1]);
    pipe.seed_calibrated(0);
    let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
    rail_at(&mut c, vbus_raw, 3204);
    for (reg, v) in [
        (config::CURRENT_LIMIT_COUNTS, 280),
        (config::STALL_RESPONSE, 1),
        (config::I_CONT_COUNTS, 168),
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

/// The bench servo as a drive sees it: the table on the fake adapter at
/// 3 Mbaud, a clock that moves only when the drive sleeps, every write
/// logged, and `drop` TEL rows lost from each burst.
pub(crate) struct Rig {
    pub(crate) c: Client<FakePipe>,
    id: Id,
    pub(crate) t_ms: f64,
    pub(crate) writes: Vec<(Reg, i32)>,
    pub(crate) experiment: bool,
    pub(crate) drop: u16,
    pub(crate) cap: Option<BurstCap>,
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
        }
    }
}

impl Servo for Rig {
    type P = FakePipe;

    fn client(&mut self) -> &mut Client<FakePipe> {
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
        write_reg(&mut self.c, self.id, reg, value)
    }

    fn snapshot(&mut self) -> Result<TelemetrySnapshot> {
        read_snapshot(&mut self.c, self.id)
    }

    /// The rows the servo drops land in the table's `tel_drop_count`, where
    /// [`Servo::stream`] reads them.
    fn burst(
        &mut self,
        samples: u16,
        goal: Option<(Reg, i32)>,
        chunks: Chunks,
    ) -> Result<(Vec<TelFrame>, BurstStats)> {
        if let Some((reg, value)) = goal {
            self.writes.push((reg, value));
        }
        let r = exchange_tel_burst(&mut self.c, self.id, samples, goal, chunks)?;
        let drop = self.drop;
        self.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            let h = &mut t.telemetry_ext.health;
            h.tel_drop_count = h.tel_drop_count.wrapping_add(drop);
        });
        Ok(r)
    }

    fn sleep(&mut self, ms: u32) {
        self.t_ms += ms as f64;
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
