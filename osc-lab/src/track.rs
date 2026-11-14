//! `osc-lab track` - drive one closed loop of the production image through a
//! schedule of goals and record the raw TEL rows: each goal is staged with
//! its burst's arm under HOLD and one COMMIT applies both, so the step edge
//! lands inside the capture (protocol sec 5.6). A `set:` item is written
//! and committed on its own before what follows, since a burst never shares
//! a tick with a CONFIG take. Nothing here scores: the rows, the segments
//! and the set the servo ran on land in --out.
//!
//! A goal's COMMIT goes out when the host's exchanges around the burst
//! before it are done, so a train of short goals runs at the host's pace.
//! With `--goal-period-ms` the goals of a train go out on a grid of that
//! period, kept on the servo's own tick ([`Pace`]).
//!
//! The closed loops keep every firmware protection: no stall permit is ever
//! written. The host's own guard is the bus dead-man (written before torque
//! on, fed between bursts) and a watchdog that judges every burst's rows and
//! every poll's registers; any condition it names turns torque off before
//! anything else and nothing drives again.
//!
//! Positions are linearized counts, the unit of the goals and the soft
//! limits: the raw pot the rows and polls carry goes through the servo's own
//! position table on the host.

use std::fmt;
use std::io::Write;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::data_state::{allows, fault};
use osc_client::descriptor::{self as desc, Access, Descriptor, Kind, Value};
use osc_client::pipe::Pipe;
use osc_protocol::tel::Chunks;
use osc_servo_core::estimator::thermal::{NtcCfg, ntc_cc};
use osc_servo_core::kernel::TICK_HZ;
use osc_servo_core::kernel::phases::loops::faults::{BIT_DEADMAN, CODE_DEADMAN};
use osc_servo_core::pos_lut::{POINTS, interp_q4};
use osc_servo_core::regions::{CONFIG_BASE_ADDR, CONFIG_REGION_SIZE, control::Mode as CtMode};

use crate::instrument::frame::TelFrame;
use crate::instrument::limits::{PERMIT_REFRESH_MS, Refusal, ServoLimits};
use crate::instrument::regs::{Reg, calib, config, control, telemetry};
use crate::rig::battery::{self, Verdict};
use crate::rig::plant::Lut;
use crate::rig::pump::{self, BurstStats, Lease, STOP};
use crate::rig::servo::{Between, Servo, Wire, guard};
use crate::rig::snapshot::read_u16;
use crate::sweep::{RowFields, chunks_json, samples_of_ms, servo_meta, unix_s};

/// The longest capture one item may ask for: a burst's `tel_count` is a
/// u16, one row a kernel tick.
pub(crate) const MAX_BURST_MS: u32 = u16::MAX as u32 * 1000 / TICK_HZ;

/// The cadence the watchdog polls at through a rest and the centering.
const POLL_MS: u32 = 50;

/// The centering is done once the shaft reads within the deadband plus
/// this many counts of the middle for `CENTER_STILL` polls in a row, and
/// gives up after `CENTER_POLLS` polls, 10 s.
const CENTER_TOL: i32 = 20;
const CENTER_STILL: u32 = 4;
const CENTER_POLLS: u32 = 200;

/// The closed loop a run drives.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub(crate) enum Mode {
    Position,
    Velocity,
    Current,
}

impl Mode {
    fn value(self) -> i32 {
        (match self {
            Mode::Position => CtMode::Position,
            Mode::Velocity => CtMode::Velocity,
            Mode::Current => CtMode::Current,
        }) as i32
    }

    /// The register a `goal:` item writes.
    fn goal(self) -> Reg {
        match self {
            Mode::Position => control::GOAL_POSITION,
            Mode::Velocity => control::GOAL_VELOCITY,
            Mode::Current => control::GOAL_CURRENT,
        }
    }

    /// The data state gates these on the identified set (protocol sec 5.7).
    fn closed(self) -> bool {
        self != Mode::Current
    }

    fn as_str(self) -> &'static str {
        match self {
            Mode::Position => "position",
            Mode::Velocity => "velocity",
            Mode::Current => "current",
        }
    }
}

/// One schedule item.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Item {
    Set { field: String, value: String },
    Goal { value: i32, ms: u32 },
    Hold { ms: u32 },
    Rest { ms: u32 },
}

impl Item {
    fn kind(&self) -> &'static str {
        match self {
            Item::Set { .. } => "set",
            Item::Goal { .. } => "goal",
            Item::Hold { .. } => "hold",
            Item::Rest { .. } => "rest",
        }
    }

    /// The span of a captured item.
    fn burst_ms(&self) -> Option<u32> {
        match self {
            Item::Goal { ms, .. } | Item::Hold { ms } => Some(*ms),
            Item::Set { .. } | Item::Rest { .. } => None,
        }
    }
}

impl fmt::Display for Item {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Item::Set { field, value } => write!(f, "set:{field}={value}"),
            Item::Goal { value, ms } => write!(f, "goal:{value}@{ms}"),
            Item::Hold { ms } => write!(f, "hold@{ms}"),
            Item::Rest { ms } => write!(f, "rest@{ms}"),
        }
    }
}

impl std::str::FromStr for Item {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        parse_item(s)
    }
}

pub(crate) fn parse_item(s: &str) -> Result<Item, String> {
    if let Some(v) = s.strip_prefix("set:") {
        return match v.split_once('=') {
            Some((field, value)) if !field.is_empty() && !value.is_empty() => Ok(Item::Set {
                field: field.into(),
                value: value.into(),
            }),
            _ => Err(format!("set: is set:FIELD=VALUE, got {s:?}")),
        };
    }
    let grammar =
        || format!("an item is set:FIELD=VALUE, goal:VALUE@MS, hold@MS or rest@MS, got {s:?}");
    let (head, ms) = s.split_once('@').ok_or_else(grammar)?;
    let ms: u32 = ms.parse().map_err(|_| format!("bad ms {ms:?} in {s:?}"))?;
    if ms == 0 {
        return Err(format!("{s:?}: the span must be nonzero"));
    }
    let burst = |ms: u32| {
        if ms > MAX_BURST_MS {
            return Err(format!(
                "{s:?}: one TEL burst carries at most {MAX_BURST_MS} ms (65535 rows at {TICK_HZ} \
                 Hz): split the capture into items"
            ));
        }
        Ok(ms)
    };
    match head {
        "hold" => Ok(Item::Hold { ms: burst(ms)? }),
        "rest" => Ok(Item::Rest { ms }),
        _ => {
            let v = head.strip_prefix("goal:").ok_or_else(grammar)?;
            let value = v
                .parse()
                .map_err(|_| format!("bad goal {v:?} in {s:?}: signed device counts"))?;
            Ok(Item::Goal {
                value,
                ms: burst(ms)?,
            })
        }
    }
}

/// `osc-lab track` args. `--baud`/`--id` come from the top-level globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir; track.csv, segments.csv and meta.json land here directly.
    #[arg(long)]
    out: PathBuf,
    /// The closed loop to drive. Torque comes on holding the present
    /// position (position) or at a zero goal (velocity, current).
    #[arg(long, value_enum)]
    mode: Mode,
    /// Items, comma separated, run in order. `set:FIELD=VALUE` writes one
    /// CONFIG register or CONTROL goal by descriptor name (integer, 0x hex,
    /// true/false or an enum name), volatile, committed before what
    /// follows. `goal:VALUE@MS` commits the mode's goal (signed device
    /// counts) with a TEL burst of MS. `hold@MS` captures MS with no goal
    /// written. `rest@MS` captures nothing while the watchdog polls. A
    /// capture is at most one burst, 3276 ms. Every CONFIG field a `set:`
    /// wrote is put back to its value before the run once torque is off at
    /// the end. Until then a `set:` of a plant-stamp field (a gain, a limit,
    /// the deadband) reads STAMP_MISMATCH, so a closed-loop entry is refused,
    /// a centering from velocity or current mode included.
    #[arg(long, value_delimiter = ',', value_parser = parse_item, required = true)]
    schedule: Vec<Item>,
    /// TEL row: table chunks as addr+len, at most two, comma-joined (hex
    /// ok). Default = the raw six: pos, raw current, vmotor_a, vmotor_b,
    /// applied duty, trough.
    #[arg(long, default_value = "0x238+2,0x240+10")]
    tel_chunks: String,
    /// Watchdog: abort on |current| over this, mA [default: a quarter over
    /// the servo's current limit].
    #[arg(long)]
    current_cap_ma: Option<f64>,
    /// Watchdog: for longer than this, ms: consecutive rows of a burst, or
    /// consecutive polls.
    #[arg(long, default_value_t = 50)]
    current_cap_ms: u32,
    /// Watchdog: abort on the board NTC over this, degrees C, through the
    /// servo's own NTC curve [default: its derate_start_cc].
    #[arg(long)]
    ntc_max_c: Option<f64>,
    /// Watchdog: abort on the shaft under this, linearized counts
    /// [default: the servo's low soft limit].
    #[arg(long, allow_negative_numbers = true)]
    guard_lo: Option<i32>,
    /// High end, counts [default: the servo's high soft limit].
    #[arg(long, allow_negative_numbers = true)]
    guard_hi: Option<i32>,
    /// Bus dead-man, ms: written before torque on and fed between bursts,
    /// so a host that dies leaves a servo that coasts; 0 = no dead-man.
    /// Honored by the experiment image and a production image built with
    /// `deadman`, ignored by any other.
    #[arg(long, default_value_t = 1000)]
    deadman_ms: u32,
    /// Prove the image honors the dead-man before the schedule: torque on
    /// holding, silent for 1.5 x the dead-man, then the `deadman` fault must
    /// have latched; refused (exit 2) when it did not.
    #[arg(long)]
    deadman_check: bool,
    /// Leave the shaft where the last item left it instead of a Position
    /// move to the middle of the soft range.
    #[arg(long)]
    no_center: bool,
    /// Hold a train of goals to this period, ms: a `goal:` item right
    /// after a `goal:` item whose burst is shorter than the period is
    /// committed on a grid of the period from the train's first goal, on
    /// the servo's tick, the COMMIT waiting for its grid point behind the
    /// staged writes. A goal the host has not staged by its point goes out
    /// at once and is counted late, and the goals after it aim at their
    /// own points. Inside a train there is one poll between two bursts,
    /// taken once the next goal is staged, ahead of the wait. At most 250
    /// ms, the keepalive's cadence: the wait is bus silence [default: none,
    /// a goal goes out as soon as the host is ready].
    #[arg(long)]
    goal_period_ms: Option<f64>,
}

/// A run refused before anything moved, or a dead-man check that failed:
/// exit 2.
#[derive(Debug)]
pub(crate) struct Refused(pub(crate) String);

impl fmt::Display for Refused {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "refused: {}", self.0)
    }
}

impl std::error::Error for Refused {}

fn refuse<T>(msg: impl Into<String>) -> Result<T> {
    Err(Refused(msg.into()).into())
}

/// Where a run stopped: torque went off there and nothing drove again.
/// Exit 3.
#[derive(Debug, Clone)]
pub(crate) struct Abort {
    /// The schedule item, None for the dead-man check and the centering.
    pub(crate) seg: Option<usize>,
    pub(crate) at: String,
    pub(crate) reason: String,
}

impl fmt::Display for Abort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "aborted at {}: {}; torque is off and nothing drives again",
            self.at, self.reason
        )
    }
}

impl std::error::Error for Abort {}

/// The exit code a run's error maps to, None for any other failure.
pub(crate) fn exit_code(e: &anyhow::Error) -> Option<i32> {
    if e.downcast_ref::<Refused>().is_some() {
        Some(2)
    } else if e.downcast_ref::<Abort>().is_some() {
        Some(3)
    } else {
        None
    }
}

/// A watchdog condition, with what was captured of the item it struck in.
struct Tripped {
    reason: String,
    seg: Option<Box<Seg>>,
}

impl fmt::Debug for Tripped {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.reason)
    }
}

impl fmt::Display for Tripped {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.reason)
    }
}

impl std::error::Error for Tripped {}

fn trip<T>(reason: impl Into<String>) -> Result<T> {
    Err(Tripped {
        reason: reason.into(),
        seg: None,
    }
    .into())
}

/// The registers a poll reads, one read of the telemetry region.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct Poll {
    pub(crate) fault_flags: u8,
    pub(crate) fault_code: u8,
    pub(crate) pos: u16,
    pub(crate) vbus_raw: u16,
    pub(crate) ntc_raw: u16,
    pub(crate) i_tick_counts: i16,
    pub(crate) theta_hat_q16: i32,
    pub(crate) omega_hat_cps: i32,
    pub(crate) i_lim_counts: u16,
    pub(crate) t_winding_cc: i16,
    pub(crate) current_bias_counts: u16,
    pub(crate) limit_flags: u8,
    pub(crate) omega_hat_src: u8,
}

const POLL_BASE: u16 = telemetry::FAULT_FLAGS.addr;
const POLL_LEN: u16 = telemetry::WINDOW_FLOOR_Q15.addr + 2 - POLL_BASE;

impl Poll {
    fn parse(raw: &[u8]) -> Option<Self> {
        use telemetry as t;
        let at = |r: Reg| -> Option<u32> {
            let off = (r.addr - POLL_BASE) as usize;
            let b = raw.get(off..off + r.width as usize)?;
            let mut w = [0u8; 4];
            w[..b.len()].copy_from_slice(b);
            Some(u32::from_le_bytes(w))
        };
        Some(Self {
            fault_flags: at(t::FAULT_FLAGS)? as u8,
            fault_code: at(t::FAULT_CODE)? as u8,
            pos: at(t::POS)? as u16,
            vbus_raw: at(t::VBUS_RAW)? as u16,
            ntc_raw: at(t::NTC_RAW)? as u16,
            i_tick_counts: at(t::I_TICK_COUNTS)? as u16 as i16,
            theta_hat_q16: at(t::THETA_HAT_Q16)? as i32,
            omega_hat_cps: at(t::OMEGA_HAT_CPS)? as i32,
            i_lim_counts: at(t::I_LIM_COUNTS)? as u16,
            t_winding_cc: at(t::T_WINDING_CC)? as u16 as i16,
            current_bias_counts: at(t::CURRENT_BIAS_COUNTS)? as u16,
            limit_flags: at(t::LIMIT_FLAGS)? as u8,
            omega_hat_src: at(t::OMEGA_HAT_SRC)? as u8,
        })
    }

    fn read<S: Servo>(s: &mut S) -> Result<Self> {
        let id = s.id();
        Self::read_on(s.client(), id)
    }

    fn read_on<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Self> {
        let raw = c.read(id, POLL_BASE, POLL_LEN).context("telemetry poll")?;
        Self::parse(&raw).context("telemetry poll parse (short read?)")
    }
}

fn fault_name(code: u8) -> &'static str {
    match code {
        CODE_DEADMAN => "deadman",
        c => fault::name(c).unwrap_or("unknown"),
    }
}

/// What the watchdog judges by, and its run of polls over the current cap.
pub(crate) struct Watchdog {
    pub(crate) guard: (i32, i32),
    /// |current| cap, current counts.
    pub(crate) cap: f64,
    pub(crate) cap_rows: u32,
    pub(crate) cap_polls: u32,
    pub(crate) ntc: NtcCfg,
    pub(crate) ntc_max_cc: i16,
    pub(crate) lut: [i16; POINTS],
    polls_over: u32,
}

impl Watchdog {
    /// Raw pot counts as the kernel linearizes them.
    pub(crate) fn lin(&self, raw: u16) -> i32 {
        (interp_q4(raw, &self.lut) >> 4) as i32
    }

    fn check_pos(&self, raw: u16) -> Result<(), String> {
        let (lo, hi) = self.guard;
        let pos = self.lin(raw);
        if pos < lo || pos > hi {
            return Err(format!(
                "the shaft at {pos} counts is outside the guard {lo}..{hi}"
            ));
        }
        Ok(())
    }

    fn check_ntc(&self, raw: u16) -> Result<(), String> {
        match ntc_cc(raw, &self.ntc) {
            None => Err(format!(
                "the board NTC reads {raw}, a rail: no temperature to judge by"
            )),
            Some(cc) if cc > self.ntc_max_cc => Err(format!(
                "the board NTC reads {:.1} C, over the {:.1} C cap",
                cc as f64 / 100.0,
                self.ntc_max_cc as f64 / 100.0
            )),
            Some(_) => Ok(()),
        }
    }

    /// One poll's registers.
    pub(crate) fn poll(&mut self, p: &Poll) -> Result<(), String> {
        if p.fault_flags != 0 {
            return Err(format!(
                "the servo latched a fault: flags {:#04x} code {} ({})",
                p.fault_flags,
                p.fault_code,
                fault_name(p.fault_code)
            ));
        }
        self.check_pos(p.pos)?;
        if (p.i_tick_counts.unsigned_abs() as f64) > self.cap {
            self.polls_over += 1;
        } else {
            self.polls_over = 0;
        }
        if self.polls_over >= self.cap_polls {
            return Err(format!(
                "|current| {} counts over the {:.0} count cap for {} polls",
                p.i_tick_counts.unsigned_abs(),
                self.cap,
                self.polls_over
            ));
        }
        self.check_ntc(p.ntc_raw)
    }

    /// One burst's rows: the guard on every row that carries `pos`, the
    /// current cap on consecutive rows, the NTC on rows that carry it. A
    /// row's current is its held current when it carries one, else its raw
    /// shunt off `bias` either way on a window the shunt read (the amplifier
    /// reads under its bias while current flows back); a row that reads no
    /// current leaves a run over the cap as it was.
    pub(crate) fn rows(&self, frames: &[TelFrame], bias: u16) -> Result<(), String> {
        let mut run = 0u32;
        for f in frames {
            if let Some(pos) = f.pos {
                self.check_pos(pos)
                    .map_err(|e| format!("{e} at row tick {}", f.tick))?;
            }
            let i = match (f.current, f.current_raw) {
                (Some(i), _) => Some(i.unsigned_abs() as f64),
                (None, Some(raw)) if f.window_valid => {
                    Some((raw as i32 - bias as i32).unsigned_abs() as f64)
                }
                _ => None,
            };
            match i {
                Some(i) if i > self.cap => {
                    run += 1;
                    if run >= self.cap_rows {
                        return Err(format!(
                            "|current| over the {:.0} count cap for {run} rows, to row tick {}",
                            self.cap, f.tick
                        ));
                    }
                }
                Some(_) => run = 0,
                None => {}
            }
            if let Some(ntc) = f.ntc_raw {
                self.check_ntc(ntc)
                    .map_err(|e| format!("{e} at row tick {}", f.tick))?;
            }
        }
        Ok(())
    }
}

/// What one run drives, every refusal already made.
pub(crate) struct Cfg {
    pub(crate) mode: Mode,
    pub(crate) items: Vec<Item>,
    /// Each `set:` item resolved, by index.
    pub(crate) sets: Vec<Option<SetReg>>,
    /// `data_flags` at preflight, what the put-back is judged against.
    pub(crate) data_flags: u8,
    pub(crate) chunks: Chunks,
    pub(crate) tick_hz: f64,
    pub(crate) wd: Watchdog,
    pub(crate) cap_ma: Option<f64>,
    pub(crate) cap_ms: u32,
    pub(crate) deadman_ms: u32,
    pub(crate) deadman_ticks: u16,
    pub(crate) deadman_check: bool,
    pub(crate) center: Option<i32>,
    pub(crate) deadband: u16,
    pub(crate) pace: Option<Pace>,
}

/// The goal period's grid. The first goal of a train (a `goal:` item with
/// no `goal:` item right before it, or one whose burst outlasts the
/// period) goes out when the host is ready, and the grid starts there:
/// point n is n periods after it. The host aims by its own clock, at n
/// periods after the instant it sent the first goal, plus a trim. The trim
/// keeps that grid on the servo's tick: each goal's `stamp0` says where it
/// landed against point n on the tick, and the trim takes a [`TRIM_GAIN`]
/// share of that error, less what the host knows it sent late by. So the
/// two clocks' disagreement is taken out as it grows and never adds up,
/// and one landing's scatter moves the next aim by an eighth of itself.
///
/// A goal staged after its point goes out at once, late. The grid stands:
/// the goal after it aims at the next point, so a stall is made up at the
/// rate the period leaves room for. A goal a whole period late or more
/// has missed the points it went past, and the goal after it aims at the
/// first point still ahead of it.
#[derive(Clone, Debug)]
pub(crate) struct Pace {
    pub(crate) period_ms: f64,
    ticks_per_ms: f64,
    train: Option<Train>,
    /// Goals aimed at a grid point, and those of them sent late.
    pub(crate) held: u32,
    pub(crate) late: u32,
}

/// The trim's share of a landing's error. The clocks disagree by up to
/// 0.15 %, which this gain holds to a standing 1.2 % of a period (0.3 ms
/// of 25.6), constant, so the spacing's mean is the period's; and the
/// trim's own motion adds under 7 % to the spacing's scatter over what
/// two landings' scatter alone gives it.
const TRIM_GAIN: f64 = 0.125;

/// A train as its last goal left it.
#[derive(Copy, Clone, Debug)]
struct Train {
    /// The last goal's burst, ms: one that outlasts the period ends the
    /// train.
    burst_ms: u32,
    /// When the host sent the first goal, ms on the servo's clock.
    origin_ms: f64,
    trim_ms: f64,
    /// The grid point the last goal took, the first goal's 0.
    point: f64,
    /// When the host sent the last goal, its stamp, and the ticks from
    /// the first goal's landing to its own.
    sent_ms: f64,
    stamp: u16,
    ticks: f64,
}

/// A goal aimed at a grid point, as it went out.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct Held {
    /// How long before its point the host was ready to send it, the room
    /// the period left it; 0 when late.
    pub(crate) wait_ms: f64,
    /// How long after its point the COMMIT went out.
    pub(crate) late_ms: f64,
    /// The whole periods of that: the grid points it went past.
    pub(crate) slots_missed: u32,
}

impl Pace {
    fn new(period_ms: f64, tick_hz: f64) -> Self {
        Self {
            period_ms,
            ticks_per_ms: tick_hz / 1000.0,
            train: None,
            held: 0,
            late: 0,
        }
    }

    /// The instant on the host's clock a train's `point` is aimed at.
    fn aim(&self, t: &Train, point: f64) -> f64 {
        t.origin_ms + point * self.period_ms + t.trim_ms
    }

    /// The instant the next goal's COMMIT is due; None for the first goal
    /// of a train.
    fn slot(&self) -> Option<f64> {
        let t = self
            .train
            .filter(|t| (t.burst_ms as f64) < self.period_ms)?;
        Some(self.aim(&t, t.point + 1.0))
    }

    /// Anything but a goal ends the train.
    fn rest(&mut self) {
        self.train = None;
    }

    /// A goal's burst is back. Aimed at `slot`, it took the point its
    /// send fell in the period after, and it is late when its COMMIT went
    /// out more than a tick after the slot, the least the servo can tell.
    /// With no slot it starts a train. The stamp is 16 bits of the tick,
    /// so the ticks since the goal before are the stamps' difference plus
    /// the whole wraps the host's clock counts between the two sends.
    fn landed(&mut self, burst_ms: u32, slot: Option<f64>, stats: &BurstStats) -> Option<Held> {
        let Some(stamp) = stats.stamp0 else {
            self.train = None;
            return None;
        };
        let (Some(mut t), Some(slot)) = (self.train, slot) else {
            self.train = Some(Train {
                burst_ms,
                origin_ms: stats.sent_ms,
                trim_ms: 0.0,
                point: 0.0,
                sent_ms: stats.sent_ms,
                stamp,
                ticks: 0.0,
            });
            return None;
        };
        const WRAP: f64 = 65536.0;
        let late_ms = stats.sent_ms - slot;
        let missed = (late_ms / self.period_ms).floor().max(0.0);
        let ticks = stamp.wrapping_sub(t.stamp) as f64;
        let host = (stats.sent_ms - t.sent_ms) * self.ticks_per_ms;
        t.ticks += ticks + WRAP * ((host - ticks) / WRAP).round();
        t.point += 1.0 + missed;
        let landed_past = t.ticks / self.ticks_per_ms - t.point * self.period_ms;
        let sent_past = stats.sent_ms - self.aim(&t, t.point);
        t.trim_ms -= TRIM_GAIN * (landed_past - sent_past);
        (t.burst_ms, t.sent_ms, t.stamp) = (burst_ms, stats.sent_ms, stamp);
        self.train = Some(t);
        self.held += 1;
        self.late += (late_ms * self.ticks_per_ms > 1.0) as u32;
        Some(Held {
            wait_ms: (slot - stats.staged_ms).max(0.0),
            late_ms,
            slots_missed: missed as u32,
        })
    }
}

/// A `set:` item resolved: its register and value, whether it is a CONFIG
/// field the run puts back, and whether the field reads back signed.
#[derive(Clone, Debug)]
pub(crate) struct SetReg {
    pub(crate) field: String,
    pub(crate) reg: Reg,
    pub(crate) value: i32,
    pub(crate) config: bool,
    pub(crate) signed: bool,
}

/// A CONFIG field a `set:` wrote: its value before the run, and the value
/// it was last set to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Kept {
    pub(crate) field: String,
    pub(crate) reg: Reg,
    pub(crate) signed: bool,
    pub(crate) before: i32,
    pub(crate) last: i32,
}

impl Kept {
    /// `v` as the field reads: its width's bytes, sign-extended or not.
    fn shown(&self, v: i32) -> i64 {
        let bits = 8 * self.reg.width as u32;
        let raw = (v as u32 as u64) & ((1u64 << bits) - 1);
        if self.signed && raw >> (bits - 1) != 0 {
            raw as i64 - (1i64 << bits)
        } else {
            raw as i64
        }
    }
}

/// A `set:` item against the descriptor: the field, the value as its kind
/// takes it, and the register a write lands on. Identity and comms, the
/// lifecycle the run owns and the rest of CONTROL are refused.
fn resolve_set(d: &Descriptor, field: &str, value: &str) -> Result<(Reg, i32), String> {
    let f = crate::descriptor::field(d, field).map_err(|e| e.to_string())?;
    let goals = [
        control::GOAL_DUTY,
        control::GOAL_POSITION,
        control::GOAL_VELOCITY,
        control::GOAL_CURRENT,
    ];
    let in_config = f.addr >= config::POS_MIN_PHYS_COUNTS.addr
        && f.end() <= CONFIG_BASE_ADDR + CONFIG_REGION_SIZE;
    if !in_config && !goals.iter().any(|g| g.addr == f.addr) {
        return Err(format!(
            "set:{field} is not a CONFIG register past identity and comms, nor a goal: the run \
             owns torque, mode, the dead-man and the streams"
        ));
    }
    if f.access != Access::Rw || f.kind == Kind::Bytes || f.width > 4 {
        return Err(format!("set:{field} is not a writable scalar"));
    }
    let int = |v: &str| -> Option<i64> {
        let (neg, mag) = v.strip_prefix('-').map_or((false, v), |m| (true, m));
        let n = match mag.strip_prefix("0x") {
            Some(h) => i64::from_str_radix(h, 16).ok()?,
            None => mag.parse().ok()?,
        };
        Some(if neg { -n } else { n })
    };
    let bad = || format!("set:{field}={value}: not a value of the field");
    let v = match f.kind {
        Kind::Uint => Value::Uint(u64::try_from(int(value).ok_or_else(bad)?).map_err(|_| bad())?),
        Kind::Int => Value::Int(int(value).ok_or_else(bad)?),
        Kind::Bool => Value::Bool(match value {
            "true" | "1" => true,
            "false" | "0" => false,
            _ => return Err(bad()),
        }),
        Kind::Enum => {
            let n = match f.variants.iter().find(|v| v.name == value) {
                Some(v) => v.value,
                None => int(value)
                    .and_then(|n| u8::try_from(n).ok())
                    .ok_or_else(bad)?,
            };
            Value::Enum(n)
        }
        Kind::Bytes => return Err(bad()),
    };
    let n = match &v {
        Value::Uint(n) => *n as i64,
        Value::Int(n) => *n,
        Value::Bool(b) => *b as i64,
        Value::Enum(n) => *n as i64,
        Value::Bytes(_) => 0,
    };
    if f.min.is_some_and(|m| n < m) || f.max.is_some_and(|m| n > m) {
        return Err(format!(
            "set:{field}={value} is outside the field's {}..{}",
            f.min.map_or("".into(), |m| m.to_string()),
            f.max.map_or("".into(), |m| m.to_string())
        ));
    }
    desc::encode(f, &v).map_err(|e| format!("set:{field}={value}: {e}"))?;
    // the write truncates to the field's width, so a u32 past i32::MAX wraps
    // to the same bytes
    let reg = Reg {
        addr: f.addr,
        width: f.width as u8,
    };
    Ok((reg, n as i32))
}

/// Read the servo and refuse, before any torque, every run it cannot
/// take: an experiment image, a line that streams no TEL, a flat pack, a
/// data state that refuses the mode, a standing fault, a shaft outside the
/// guard, and any item, cap or dead-man the servo cannot carry.
pub(crate) fn prepare<S: Servo>(s: &mut S, a: &Args) -> Result<Cfg> {
    let id = s.id();
    if s.experiment()? {
        return refuse(
            "the servo runs the experiment image, which has no closed loops: flash a production \
             image",
        );
    }
    let chunks = pump::parse_tel_chunks(&a.tel_chunks).context("--tel-chunks")?;
    if let Err(e) = s.tel_line()?.admit(chunks) {
        return refuse(e.to_string());
    }
    s.pwm_period_ticks()?;
    let (rail, pack) = battery::read(s.client(), id)?;
    if let Verdict::Refuse(m) = battery::drive_gate(rail, pack) {
        return refuse(m);
    }
    let data = crate::state::check(s.client(), id)?;
    if !allows(data.flags, a.mode.closed()) {
        return refuse(format!(
            "the servo's data state ({}) refuses {} mode: {}",
            data.names(),
            a.mode.as_str(),
            data.message().unwrap_or_default()
        ));
    }
    let lim = crate::rig::limits::read(s.client(), id).map_err(as_refusal)?;
    let tick_hz = s.tick_hz()?;
    let d = crate::state::descriptor(s.client(), id)?;
    let lut = Lut::read(s.client(), id, &d)?;
    let mut points = [0i16; POINTS];
    points[..lut.points.len()].copy_from_slice(&lut.points);

    let guard = guard_of(&lim, a.guard_lo, a.guard_hi)?;
    let cap = match a.current_cap_ma {
        None => lim.abort_default() as f64,
        Some(_) if lim.amps_per_count <= 0.0 => {
            return refuse(
                "--current-cap-ma needs the servo's shunt scale, and its sense constants are unset",
            );
        }
        Some(ma) => ma / 1000.0 / lim.amps_per_count,
    };
    let c = s.client();
    let ntc = NtcCfg {
        raw_ref: read_u16(c, id, calib::NTC_RAW_REF)?,
        t_ref_cc: read_u16(c, id, calib::NTC_T_REF_CC)? as i16,
        k1_q88: read_u16(c, id, calib::NTC_K1_Q88)? as i16,
        k2_q24: read_u16(c, id, calib::NTC_K2_Q24)?,
    };
    if ntc.k1_q88 == 0 {
        return refuse(
            "the servo carries no board NTC curve (ntc_k1_q88 0), so the NTC watchdog has \
             nothing to judge by",
        );
    }
    let ntc_max_cc = match a.ntc_max_c {
        Some(c) => (c * 100.0).round().clamp(i16::MIN as f64, i16::MAX as f64) as i16,
        None => read_u16(c, id, config::DERATE_START_CC)? as i16,
    };
    let deadband = read_u16(c, id, config::POS_DEADBAND_COUNTS)?;

    let deadman_ticks = deadman_ticks(a.deadman_ms, tick_hz)?;
    if a.deadman_check && deadman_ticks == 0 {
        return refuse("--deadman-check needs a dead-man: --deadman-ms is 0");
    }

    let pace = match a.goal_period_ms {
        None => None,
        Some(ms) if ms > 0.0 && ms <= PERMIT_REFRESH_MS as f64 => Some(Pace::new(ms, tick_hz)),
        Some(ms) => {
            return refuse(format!(
                "--goal-period-ms {ms} is not over 0 and at most {PERMIT_REFRESH_MS} ms, the \
                 keepalive's cadence: a commit's wait for its slot is bus silence"
            ));
        }
    };

    let cap_rows = samples_of_ms(a.current_cap_ms, tick_hz).max(1);
    let cap_polls = a.current_cap_ms.div_ceil(POLL_MS).max(1);
    let wd = Watchdog {
        guard,
        cap,
        cap_rows,
        cap_polls,
        ntc,
        ntc_max_cc,
        lut: points,
        polls_over: 0,
    };

    let mut sets = Vec::with_capacity(a.schedule.len());
    let burst_cap = s.burst_cap()?;
    for item in &a.schedule {
        sets.push(match item {
            Item::Set { field, value } => {
                let (reg, value) = resolve_set(&d, field, value).or_else(refuse)?;
                Some(SetReg {
                    field: field.clone(),
                    reg,
                    value,
                    config: reg.addr < CONFIG_BASE_ADDR + CONFIG_REGION_SIZE,
                    signed: d.field(field).is_some_and(|f| f.kind == Kind::Int),
                })
            }
            _ => None,
        });
        if let Some(ms) = item.burst_ms()
            && let Err(e) = burst_cap.check(samples_of_ms(ms, tick_hz))
        {
            return refuse(format!("{item}: {e}"));
        }
        if let Item::Goal { value, .. } = item {
            check_goal(a.mode, *value, guard, lim.i_lim).or_else(refuse)?;
        }
    }

    let p = Poll::read(s)?;
    if p.fault_flags != 0 {
        return refuse(format!(
            "the servo has a fault standing: flags {:#04x} code {} ({}); ack it with a torque \
             enable once its cause is gone",
            p.fault_flags,
            p.fault_code,
            fault_name(p.fault_code)
        ));
    }
    wd.check_pos(p.pos).or_else(refuse)?;

    let (lo, hi) = lim.soft;
    Ok(Cfg {
        mode: a.mode,
        items: a.schedule.clone(),
        sets,
        data_flags: data.flags,
        chunks,
        tick_hz,
        wd,
        cap_ma: a.current_cap_ma,
        cap_ms: a.current_cap_ms,
        deadman_ms: a.deadman_ms,
        deadman_ticks,
        deadman_check: a.deadman_check,
        center: (!a.no_center).then_some(lo + (hi - lo) / 2),
        deadband,
        pace,
    })
}

fn as_refusal(e: anyhow::Error) -> anyhow::Error {
    match e.downcast_ref::<Refusal>() {
        Some(r) => Refused(r.to_string()).into(),
        None => e,
    }
}

/// The guard: each end as asked, else the servo's soft limits, which must
/// be a calibrated range.
fn guard_of(lim: &ServoLimits, lo: Option<i32>, hi: Option<i32>) -> Result<(i32, i32)> {
    if !lim.calibrated() {
        return refuse(Refusal::Uncalibrated { soft: lim.soft }.to_string());
    }
    let g = (lo.unwrap_or(lim.soft.0), hi.unwrap_or(lim.soft.1));
    if g.0 >= g.1 {
        return refuse(format!("the guard {}..{} is empty", g.0, g.1));
    }
    Ok(g)
}

/// A goal the mode's register takes and the guard allows.
fn check_goal(mode: Mode, value: i32, guard: (i32, i32), i_lim: u16) -> Result<(), String> {
    match mode {
        Mode::Position if value < guard.0 || value > guard.1 => Err(format!(
            "goal:{value} is outside the guard {}..{}",
            guard.0, guard.1
        )),
        Mode::Current if value.unsigned_abs() > i_lim as u32 => Err(format!(
            "goal:{value} is over the servo's current limit of {i_lim} counts"
        )),
        _ => Ok(()),
    }
}

/// The dead-man's count for `ms`; 0 is no dead-man. It must outlast twice
/// the keepalive's cadence and fit the u16 field.
fn deadman_ticks(ms: u32, tick_hz: f64) -> Result<u16> {
    if ms == 0 {
        return Ok(0);
    }
    if ms < 2 * PERMIT_REFRESH_MS {
        return refuse(format!(
            "--deadman-ms {ms} is under {} ms, twice the {PERMIT_REFRESH_MS} ms keepalive",
            2 * PERMIT_REFRESH_MS
        ));
    }
    let ticks = samples_of_ms(ms, tick_hz);
    u16::try_from(ticks).or_else(|_| {
        refuse(format!(
            "--deadman-ms {ms} is {ticks} ticks, over the field's 65535 ({:.0} ms)",
            u16::MAX as f64 * 1000.0 / tick_hz
        ))
    })
}

/// One schedule item as it ran: the poll before it, and a captured item's
/// rows.
pub(crate) struct Seg {
    pub(crate) idx: usize,
    pub(crate) item: Item,
    /// The goal in force through the item.
    pub(crate) goal: i32,
    pub(crate) poll: Poll,
    pub(crate) frames: Vec<TelFrame>,
    pub(crate) stats: BurstStats,
    pub(crate) samples: u32,
    pub(crate) t_start: f64,
    pub(crate) t_end: f64,
    /// A goal aimed at a grid point; None for an item off the grid.
    pub(crate) held: Option<Held>,
}

/// What a run reports beside its segments.
#[derive(Default)]
pub(crate) struct Ran {
    pub(crate) rows_dropped: u16,
    /// The dead-man check's verdict, None when it did not run.
    pub(crate) deadman: Option<&'static str>,
    /// CONFIG and the CONTROL lifecycle, read once the `set:` items ran.
    pub(crate) set: Option<serde_json::Value>,
    /// The CONFIG fields `set:` wrote, in the order each was last set.
    pub(crate) kept: Vec<Kept>,
    /// What the put-back wrote, last-set first.
    pub(crate) restored: Vec<Kept>,
    /// `data_flags` once the put-back ran.
    pub(crate) data_flags_after: Option<u8>,
}

/// Before a `set:` of a CONFIG field, its value as the run found it, read
/// once; a field set again moves to the back, as the last one set.
fn remember<S: Servo>(s: &mut S, ran: &mut Ran, set: &SetReg) -> Result<()> {
    if let Some(k) = ran.kept.iter().position(|k| k.reg == set.reg) {
        let mut kept = ran.kept.remove(k);
        kept.last = set.value;
        ran.kept.push(kept);
        return Ok(());
    }
    let id = s.id();
    let raw = s
        .client()
        .read(id, set.reg.addr, set.reg.width as u16)
        .with_context(|| format!("read {} before its set:", set.field))?;
    let mut w = [0u8; 4];
    let n = raw.len().min(4);
    w[..n].copy_from_slice(&raw[..n]);
    ran.kept.push(Kept {
        field: set.field.clone(),
        reg: set.reg,
        signed: set.signed,
        before: i32::from_le_bytes(w),
        last: set.value,
    });
    Ok(())
}

/// Polls of `data_flags` the put-back waits through for the servo's stamp
/// checkpoint, which runs in its main loop after each write's reply.
const DATA_POLLS: u32 = 5;
const DATA_POLL_MS: u32 = 10;

/// With torque off, write every kept field back, last-set first, then read
/// `data_flags` and warn about a mismatch or any reason new since
/// preflight. Never turns torque on or changes mode. Every write is tried;
/// Err is the first that failed.
fn put_back<S: Servo>(s: &mut S, cfg: &Cfg, ran: &mut Ran) -> Result<()> {
    use osc_client::data_state::{STAMP_MISMATCH, reasons};
    let mut first: Option<anyhow::Error> = None;
    for k in std::mem::take(&mut ran.kept).into_iter().rev() {
        match s.write(k.reg, k.before) {
            Ok(()) => ran.restored.push(k),
            Err(e) => {
                first.get_or_insert(e.context(format!("put {} back", k.field)));
            }
        }
    }
    let id = s.id();
    let flags = (|| -> Result<u8> {
        let d = crate::state::descriptor(s.client(), id)?;
        let mut flags = s.client().data_state(id, &d)?.flags;
        for _ in 0..DATA_POLLS {
            if flags & STAMP_MISMATCH == 0 && flags & !cfg.data_flags == 0 {
                break;
            }
            s.sleep(DATA_POLL_MS);
            flags = s.client().data_state(id, &d)?.flags;
        }
        Ok(flags)
    })();
    match flags {
        Ok(f) => {
            ran.data_flags_after = Some(f);
            if f & STAMP_MISMATCH != 0 || f & !cfg.data_flags != 0 {
                let names: Vec<_> = reasons(f).iter().map(|r| r.name()).collect();
                eprintln!(
                    "warning: id {}: after the put-back data_flags names {} (at preflight {:#04x}): \
                     the next closed-loop entry is refused until the set matches its stamp",
                    id.as_byte(),
                    names.join(" | "),
                    cfg.data_flags
                );
            }
        }
        Err(e) => {
            first.get_or_insert(e);
        }
    }
    first.map_or(Ok(()), Err)
}

/// The goal torque comes on at: the present position, or zero. The
/// position is the servo's own estimate, rounded to the count: one `pos`
/// sample read over the bus sits a few counts off the shaft's rest, since
/// the read's own traffic rings the pot channel, and a goal that far from
/// the estimate is a move the loop then makes.
fn start_goal<S: Servo>(s: &mut S, cfg: &Cfg) -> Result<i32> {
    Ok(match cfg.mode {
        Mode::Position => (Poll::read(s)?.theta_hat_q16 + (1 << 15)) >> 16,
        Mode::Velocity | Mode::Current => 0,
    })
}

/// A poll, judged.
fn watch<S: Servo>(s: &mut S, wd: &mut Watchdog) -> Result<Poll> {
    let p = Poll::read(s)?;
    if let Err(reason) = wd.poll(&p) {
        return trip(reason);
    }
    Ok(p)
}

fn stopped() -> Result<()> {
    if STOP.load(std::sync::atomic::Ordering::SeqCst) {
        return trip("interrupted");
    }
    Ok(())
}

/// Poll through `ms` with nothing captured.
fn rest<S: Servo>(s: &mut S, lease: &mut Lease, wd: &mut Watchdog, ms: u32) -> Result<()> {
    let mut left = ms;
    while left > 0 {
        stopped()?;
        let slice = left.min(POLL_MS);
        s.sleep(slice);
        left -= slice;
        lease.keep(s)?;
        watch(s, wd)?;
    }
    Ok(())
}

/// Polls of `fault_flags` the dead-man check waits through after its ack:
/// the servo clears the latch at the enable edge its control phase reads
/// and publishes it a phase later, so a read right behind the write can
/// still carry the old flags.
const ACK_POLLS: u32 = 5;
const ACK_POLL_MS: u32 = 2;

/// Torque on holding, then silence for 1.5 x the dead-man: an image that
/// honors it has latched `deadman` and coasts, and the ack restarts it
/// holding. Anything else turns torque off and is refused.
fn check_deadman<S: Servo>(s: &mut S, cfg: &Cfg, lease: &mut Lease) -> Result<&'static str> {
    let mut left = cfg.deadman_ms * 3 / 2;
    while left > 0 {
        stopped()?;
        let slice = left.min(20);
        s.sleep(slice);
        left -= slice;
    }
    let p = Poll::read(s)?;
    if p.fault_flags & BIT_DEADMAN == 0 || p.fault_code != CODE_DEADMAN {
        lease.write(s, control::TORQUE_ENABLE, 0)?;
        let seen = match p.fault_flags {
            0 => "no fault latched".to_string(),
            f => format!(
                "it latched flags {f:#04x} code {} ({}) instead",
                p.fault_code,
                fault_name(p.fault_code)
            ),
        };
        return refuse(format!(
            "this image does not honor the bus dead-man: {} ms of silence under torque and \
             {seen}",
            cfg.deadman_ms * 3 / 2
        ));
    }
    lease.write(s, control::TORQUE_ENABLE, 0)?;
    let goal = start_goal(s, cfg)?;
    s.write(cfg.mode.goal(), goal)?;
    lease.torque_on(s)?;
    let mut p = Poll::read(s)?;
    for _ in 0..ACK_POLLS {
        if p.fault_flags == 0 {
            break;
        }
        s.sleep(ACK_POLL_MS);
        p = Poll::read(s)?;
    }
    if p.fault_flags != 0 {
        lease.write(s, control::TORQUE_ENABLE, 0)?;
        return refuse(format!(
            "the dead-man latched, and its ack left flags {:#04x} code {} ({}) standing",
            p.fault_flags,
            p.fault_code,
            fault_name(p.fault_code)
        ));
    }
    Ok("honored")
}

/// One item: the poll before it, then its write, rest or burst, the burst's
/// rows and the poll after them judged.
///
/// Inside a train held to a period there is one poll between two bursts.
/// A goal the next goal follows on the grid leaves the poll after its
/// burst to that goal, which takes it once its own writes are staged,
/// ahead of the wait: mid-gap, where the poll before a goal sits without a
/// period, and not at the gap's start. It is judged there, before the
/// COMMIT. On a trip torque goes off ahead of the carrier, so the staged
/// goal lands on a servo that no longer drives, nothing stays staged, and
/// the item's rows are the coast.
#[allow(clippy::too_many_arguments)]
fn run_item<S: Servo>(
    s: &mut S,
    cfg: &mut Cfg,
    lease: &mut Lease,
    k: usize,
    goal: &mut i32,
    anchor: f64,
    ran: &mut Ran,
    on_seg: &mut impl FnMut(&Seg) -> Result<()>,
) -> Result<()> {
    stopped()?;
    lease.keep(s)?;
    let item = cfg.items[k].clone();
    let slot = match (&item, &mut cfg.pace) {
        (Item::Goal { .. }, Some(pace)) => pace.slot(),
        (_, Some(pace)) => {
            pace.rest();
            None
        }
        (_, None) => None,
    };
    let poll = match slot {
        Some(_) => Poll::default(),
        None => watch(s, &mut cfg.wd)?,
    };
    let t_start = anchor + s.now_ms() / 1000.0;
    let mut seg = Seg {
        idx: k,
        item: item.clone(),
        goal: *goal,
        poll,
        frames: Vec::new(),
        stats: BurstStats::default(),
        samples: 0,
        t_start,
        t_end: t_start,
        held: None,
    };
    match &item {
        Item::Set { .. } => {
            if let Some(set) = cfg.sets[k].clone() {
                if set.config {
                    remember(s, ran, &set)?;
                }
                s.write(set.reg, set.value)?;
            }
        }
        Item::Rest { ms } => rest(s, lease, &mut cfg.wd, *ms)?,
        Item::Goal { ms, .. } | Item::Hold { ms } => {
            let samples = samples_of_ms(*ms, cfg.tick_hz);
            let g = match &item {
                Item::Goal { value, .. } => {
                    *goal = *value;
                    seg.goal = *value;
                    Some((cfg.mode.goal(), *value))
                }
                _ => None,
            };
            lease.keep(s)?;
            let id = s.id();
            let mut polled = None;
            let burst = {
                let wd = &mut cfg.wd;
                let mut staged = |c: &mut Client<S::P>| -> Result<()> {
                    let poll = Poll::read_on(c, id)?;
                    let verdict = wd.poll(&poll);
                    if verdict.is_err() {
                        pump::write_reg(c, id, control::TORQUE_ENABLE, 0)?;
                    }
                    polled = Some((poll, verdict));
                    Ok(())
                };
                let between: Option<Between<'_, S::P>> = match slot {
                    Some(_) => Some(&mut staged),
                    None => None,
                };
                s.stream_at(samples, g, cfg.chunks, slot, between)
            };
            if let Some((poll, verdict)) = polled {
                seg.poll = poll;
                if let Err(reason) = verdict {
                    if let Ok((frames, stats)) = burst {
                        (seg.frames, seg.stats, seg.samples) = (frames, stats, samples);
                    }
                    return Err(Tripped {
                        reason,
                        seg: Some(Box::new(seg)),
                    }
                    .into());
                }
            }
            let (frames, stats) = burst?;
            if let (Some(_), Some(pace)) = (g, &mut cfg.pace) {
                seg.held = pace.landed(*ms, slot, &stats);
            }
            ran.rows_dropped = ran.rows_dropped.saturating_add(stats.rows_dropped);
            (seg.frames, seg.stats, seg.samples) = (frames, stats, samples);
            seg.t_end = anchor + s.now_ms() / 1000.0;
            let got = seg.frames.len() as u32 + seg.stats.rows_dropped as u32;
            let verdict = if got < samples {
                Err(format!(
                    "the burst came back short: {} of {samples} rows ({} seq holes, {} garble \
                     bytes)",
                    seg.frames.len(),
                    seg.stats.holes,
                    seg.stats.garble
                ))
            } else {
                cfg.wd.rows(&seg.frames, seg.poll.current_bias_counts)
            };
            // the goal after this one takes the poll when it follows on the grid
            let follows = matches!(cfg.items.get(k + 1), Some(Item::Goal { .. }))
                && cfg.pace.as_ref().is_some_and(|p| p.slot().is_some());
            let verdict = match verdict {
                Ok(()) if g.is_some() && follows => Ok(()),
                Ok(()) => watch(s, &mut cfg.wd).map(|_| ()),
                Err(reason) => trip(reason),
            };
            if let Err(e) = verdict {
                let reason = match e.downcast::<Tripped>() {
                    Ok(t) => t.reason,
                    Err(e) => format!("{e:#}"),
                };
                return Err(Tripped {
                    reason,
                    seg: Some(Box::new(seg)),
                }
                .into());
            }
        }
    }
    seg.t_end = seg.t_end.max(anchor + s.now_ms() / 1000.0);
    on_seg(&seg)
}

/// The Position move to the middle, at the velocity limit in force, held
/// until the shaft reads there or the bound runs out.
fn center<S: Servo>(s: &mut S, cfg: &mut Cfg, lease: &mut Lease, at: i32) -> Result<()> {
    s.write(control::GOAL_POSITION, at)?;
    if cfg.mode != Mode::Position {
        s.write(control::MODE, Mode::Position.value())?;
    }
    let tol = cfg.deadband as i32 + CENTER_TOL;
    let mut still = 0;
    let mut pos = at;
    for _ in 0..CENTER_POLLS {
        stopped()?;
        s.sleep(POLL_MS);
        lease.keep(s)?;
        let raw = watch(s, &mut cfg.wd)?.pos;
        pos = cfg.wd.lin(raw);
        still = if (pos - at).abs() <= tol {
            still + 1
        } else {
            0
        };
        if still >= CENTER_STILL {
            println!("center: {pos} counts (middle {at})");
            return Ok(());
        }
    }
    eprintln!(
        "warning: the shaft read {pos} counts, not within {tol} of the middle {at}, after {} s; \
         torque goes off where it is",
        CENTER_POLLS * POLL_MS / 1000
    );
    Ok(())
}

/// CONFIG and the CONTROL lifecycle as the servo holds them, decoded by
/// the descriptor.
fn read_set<S: Servo>(s: &mut S) -> Result<serde_json::Value> {
    let id = s.id();
    let d = crate::state::descriptor(s.client(), id)?;
    let lifecycle = (
        control::TORQUE_ENABLE.addr,
        control::BUS_DEADMAN_TICKS.addr + control::BUS_DEADMAN_TICKS.width as u16,
    );
    let config = region(
        s,
        &d,
        (CONFIG_BASE_ADDR, CONFIG_BASE_ADDR + CONFIG_REGION_SIZE),
    )?;
    let control = region(s, &d, lifecycle)?;
    Ok(serde_json::json!({ "config": config, "control": control }))
}

fn region<S: Servo>(s: &mut S, d: &Descriptor, (lo, hi): (u16, u16)) -> Result<serde_json::Value> {
    let id = s.id();
    let raw = s.client().read(id, lo, hi - lo).context("region read")?;
    let mut m = serde_json::Map::new();
    for f in d.fields.iter().filter(|f| f.addr >= lo && f.end() <= hi) {
        let at = (f.addr - lo) as usize;
        let Some(b) = raw.get(at..at + f.width as usize) else {
            bail!("region read {lo:#06x}..{hi:#06x} returned {} B", raw.len());
        };
        let v = match desc::decode(f, b) {
            Ok(Value::Uint(n)) => n.into(),
            Ok(Value::Int(n)) => n.into(),
            Ok(Value::Bool(b)) => b.into(),
            Ok(Value::Enum(n)) => f
                .variant(n)
                .map_or_else(|| n.into(), |v| v.name.clone().into()),
            Ok(Value::Bytes(b)) => b.into(),
            Err(e) => bail!("decode {}: {e}", f.name),
        };
        m.insert(f.name.clone(), v);
    }
    Ok(m.into())
}

/// Turn torque off first, then keep what the item captured, then report
/// where the run stopped.
fn halt<S: Servo>(
    s: &mut S,
    lease: &mut Lease,
    seg: Option<usize>,
    at: String,
    e: anyhow::Error,
    on_seg: &mut impl FnMut(&Seg) -> Result<()>,
) -> anyhow::Error {
    let _ = lease.write(s, control::TORQUE_ENABLE, 0);
    if e.downcast_ref::<Refused>().is_some() {
        return e;
    }
    let reason = match e.downcast::<Tripped>() {
        Ok(t) => {
            if let Some(g) = t.seg {
                let _ = on_seg(&g);
            }
            t.reason
        }
        Err(e) => format!("{e:#}"),
    };
    Abort { seg, at, reason }.into()
}

/// Torque on, the dead-man check, every item, the centering, torque off,
/// then the `set:` fields put back. Each item reaches `on_seg` as it lands,
/// so an abort keeps what was captured before it. Leaves the servo guarded
/// and torqued off. On an abort or a refusal the put-back is best effort
/// and the run's own error stands.
pub(crate) fn record<S: Servo>(
    s: &mut S,
    cfg: &mut Cfg,
    ran: &mut Ran,
    mut on_seg: impl FnMut(&Seg) -> Result<()>,
) -> Result<()> {
    let mut lease = match cfg.deadman_ticks {
        0 => Lease::new(false, false),
        t => Lease::deadman(t),
    };
    let anchor = unix_s() - s.now_ms() / 1000.0;
    let r = guard(s, |s| {
        let goal0 = start_goal(s, cfg)?;
        s.write(control::MODE, cfg.mode.value())?;
        s.write(cfg.mode.goal(), goal0)?;
        lease.torque_on(s)?;
        if cfg.deadman_check {
            let at = "the dead-man check".to_string();
            ran.deadman = Some(
                check_deadman(s, cfg, &mut lease)
                    .map_err(|e| halt(s, &mut lease, None, at, e, &mut on_seg))?,
            );
        }
        let mut goal = goal0;
        for k in 0..cfg.items.len() {
            if let Err(e) = run_item(s, cfg, &mut lease, k, &mut goal, anchor, ran, &mut on_seg) {
                let at = format!("segment {k} ({})", cfg.items[k]);
                let e = halt(s, &mut lease, Some(k), at, e, &mut on_seg);
                ran.set = read_set(s).ok();
                return Err(e);
            }
        }
        ran.set = Some(read_set(s)?);
        if let Some(at) = cfg.center
            && let Err(e) = center(s, cfg, &mut lease, at)
        {
            let where_ = "the centering".to_string();
            return Err(halt(s, &mut lease, None, where_, e, &mut on_seg));
        }
        lease.write(s, control::TORQUE_ENABLE, 0)
    });
    let restored = put_back(s, cfg, ran);
    r.and(restored)
}

/// track.csv: one row a captured tick, the row's fields under sweep.csv's
/// names.
pub(crate) const CSV_HEADER: &str = "seg,goal,tick,window_valid,pos,current,current_trough,duty_counts,vdiff,vbus,current_raw,vmotor_a,vmotor_b,vbus_raw,ntc_raw,vref_raw";

pub(crate) fn write_rows(w: &mut impl Write, g: &Seg) -> Result<()> {
    for f in &g.frames {
        writeln!(w, "{},{},{}", g.idx, g.goal, RowFields(f))?;
    }
    Ok(())
}

/// segments.csv: one row a schedule item, the poll before it last.
/// `tick0` is the servo's `sample_tick` of the first row, low 16 bits.
/// `slot_wait_ms`, `slot_late_ms` and `slots_missed` are a goal's that
/// was aimed at a grid point of the period, empty for any other item: how
/// long before its point the host had it staged, the room the period left
/// it (0 for a late one); how long after its point its COMMIT went out (0
/// on time, what the wait itself ran over by, or a late goal's lateness);
/// and the whole periods of that, the grid points it went past.
pub(crate) const SEGMENTS_HEADER: &str = "seg,kind,field,value,ms,tick0,slot_wait_ms,slot_late_ms,slots_missed,rows,rows_dropped,t_start,t_end,vbus_raw,ntc_raw,theta_hat_q16,omega_hat_cps,t_winding_cc,i_lim_counts,limit_flags,fault_flags,pos,i_tick_counts,current_bias_counts,fault_code,omega_hat_src";

pub(crate) fn write_segment(w: &mut impl Write, g: &Seg) -> Result<()> {
    let (field, value, ms) = match &g.item {
        Item::Set { field, value } => (field.as_str(), value.clone(), String::new()),
        Item::Goal { value, ms } => ("", value.to_string(), ms.to_string()),
        Item::Hold { ms } | Item::Rest { ms } => ("", String::new(), ms.to_string()),
    };
    let p = &g.poll;
    let (wait, late, missed) = match &g.held {
        Some(h) => (
            format!("{:.3}", h.wait_ms),
            format!("{:.3}", h.late_ms),
            h.slots_missed.to_string(),
        ),
        None => Default::default(),
    };
    writeln!(
        w,
        "{},{},{field},{value},{ms},{},{wait},{late},{missed},{},{},{:.3},{:.3},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        g.idx,
        g.item.kind(),
        g.stats.stamp0.map(|t| t.to_string()).unwrap_or_default(),
        g.frames.len(),
        g.stats.rows_dropped,
        g.t_start,
        g.t_end,
        p.vbus_raw,
        p.ntc_raw,
        p.theta_hat_q16,
        p.omega_hat_cps,
        p.t_winding_cc,
        p.i_lim_counts,
        p.limit_flags,
        p.fault_flags,
        p.pos,
        p.i_tick_counts,
        p.current_bias_counts,
        p.fault_code,
        p.omega_hat_src,
    )?;
    Ok(())
}

/// The run's meta before it drives: what every recording's opens with,
/// the row, the mode and schedule, the guard, the watchdog, the dead-man
/// and the goal period as asked.
fn meta<S: Servo>(s: &mut S, cfg: &Cfg) -> Result<serde_json::Value> {
    let id = s.id();
    let servo = servo_meta(s.client(), id)?;
    let mut m = serde_json::json!({
        "mode": cfg.mode.as_str(),
        "schedule": cfg.items.iter().map(Item::to_string).collect::<Vec<_>>(),
        "tel_chunks": chunks_json(&cfg.chunks),
        "guard": [cfg.wd.guard.0, cfg.wd.guard.1],
        "watchdog": {
            "current_cap_counts": cfg.wd.cap,
            "current_cap_ma": cfg.cap_ma,
            "current_cap_ms": cfg.cap_ms,
            "current_cap_rows": cfg.wd.cap_rows,
            "current_cap_polls": cfg.wd.cap_polls,
            "ntc_max_c": cfg.wd.ntc_max_cc as f64 / 100.0,
        },
        "deadman": {
            "ms": cfg.deadman_ms,
            "ticks": cfg.deadman_ticks,
            "check": serde_json::Value::Null,
        },
        "center": cfg.center,
        "goal_period": {
            "ms": cfg.pace.as_ref().map(|p| p.period_ms),
            "held": 0,
            "late": 0,
        },
        "rows_dropped": 0,
        "config": serde_json::Value::Null,
        "control": serde_json::Value::Null,
        "restored": [],
        "data_flags_after": serde_json::Value::Null,
        "ended": serde_json::Value::Null,
    });
    if let Some(o) = m.as_object_mut() {
        o.extend(servo);
    }
    Ok(m)
}

/// The meta once the run ended: the rows dropped, the goals held to the
/// period and those of them late, the dead-man check, the set it ran on
/// and how it ended.
fn land(meta: &mut serde_json::Value, cfg: &Cfg, ran: &Ran, r: &Result<()>) {
    meta["rows_dropped"] = ran.rows_dropped.into();
    if let Some(p) = &cfg.pace {
        meta["goal_period"]["held"] = p.held.into();
        meta["goal_period"]["late"] = p.late.into();
    }
    meta["deadman"]["check"] = ran.deadman.map_or(serde_json::Value::Null, Into::into);
    if let Some(set) = &ran.set {
        meta["config"] = set["config"].clone();
        meta["control"] = set["control"].clone();
    }
    meta["restored"] = ran
        .restored
        .iter()
        .map(|k| serde_json::json!({ "field": k.field, "from": k.shown(k.last), "to": k.shown(k.before) }))
        .collect::<Vec<_>>()
        .into();
    meta["data_flags_after"] = ran.data_flags_after.map_or(serde_json::Value::Null, |f| {
        osc_client::data_state::reasons(f)
            .iter()
            .map(|r| r.name())
            .collect::<Vec<_>>()
            .into()
    });
    meta["ended"] = match r {
        Ok(()) => "ok".into(),
        Err(e) => match e.downcast_ref::<Abort>() {
            Some(a) => serde_json::json!({ "reason": a.reason, "segment": a.seg, "at": a.at }),
            None => serde_json::json!({ "reason": format!("{e:#}"), "segment": null }),
        },
    };
}

/// Prepare against the servo, write the files in `out` as the run goes,
/// and land the meta however it ended.
pub(crate) fn capture<S: Servo>(s: &mut S, a: &Args, out: &Path) -> Result<()> {
    let mut cfg = prepare(s, a)?;
    std::fs::create_dir_all(out).with_context(|| format!("mkdir {}", out.display()))?;
    let mut meta = meta(s, &cfg)?;
    let meta_path = out.join("meta.json");
    let write_meta = |m: &serde_json::Value| -> Result<()> {
        std::fs::write(&meta_path, serde_json::to_string_pretty(m)?)
            .with_context(|| format!("write {}", meta_path.display()))
    };
    write_meta(&meta)?;
    let create = |name: &str, header: &str| -> Result<std::io::BufWriter<std::fs::File>> {
        let p = out.join(name);
        let mut w = std::io::BufWriter::new(
            std::fs::File::create(&p).with_context(|| format!("create {}", p.display()))?,
        );
        writeln!(w, "{header}")?;
        Ok(w)
    };
    let mut rows = create("track.csv", CSV_HEADER)?;
    let mut segs = create("segments.csv", SEGMENTS_HEADER)?;

    let mut ran = Ran::default();
    let r = record(s, &mut cfg, &mut ran, |g| {
        let st = &g.stats;
        if g.item.burst_ms().is_some() {
            println!(
                "  seg {} ({}): {} of {} rows, {} dropped, {} seq holes, {} garble bytes",
                g.idx,
                g.item,
                g.frames.len(),
                g.samples,
                st.rows_dropped,
                st.holes,
                st.garble
            );
        }
        write_rows(&mut rows, g)?;
        write_segment(&mut segs, g)
    });
    let flushed = rows.flush().and(segs.flush());
    land(&mut meta, &cfg, &ran, &r);
    write_meta(&meta)?;
    flushed?;
    if ran.rows_dropped > 0 {
        println!(
            "warning: the servo dropped {} TEL rows (both stream buffers were waiting for the \
             wire); the files are saved with those rows missing",
            ran.rows_dropped
        );
    }
    if let Some(p) = cfg.pace.as_ref().filter(|p| p.late > 0) {
        println!(
            "warning: {} of the {} goals held to the {} ms period went out late \
             (slot_late_ms in segments.csv)",
            p.late, p.held, p.period_ms
        );
    }
    r?;
    println!("track: {}", out.join("track.csv").display());
    println!("meta:  {}", meta_path.display());
    Ok(())
}

/// Entry from the top-level `osc-lab track` dispatch.
pub fn run(args: &Args, bus: crate::rig::Bus, id: u8) -> Result<()> {
    pump::install_ctrlc();
    let id = Id::new(id);
    let c = crate::rig::connect(&bus, &[id])?;
    let mut s = Wire::new(c, id);
    capture(&mut s, args, &args.out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::testkit::{Rig, Timed};
    use osc_client::fake::TelSample;

    #[derive(clap::Parser)]
    struct Cli {
        #[command(flatten)]
        args: Args,
    }

    fn parse(argv: &[&str]) -> Args {
        let mut v = vec!["track", "--out", "unused"];
        v.extend_from_slice(argv);
        <Cli as clap::Parser>::try_parse_from(v).unwrap().args
    }

    /// The bench servo, its data state clear, the shaft at `pos` raw with
    /// the board NTC at room temperature and `i` counts held.
    fn rig_at(pos: u16, i: i16) -> Rig {
        let mut r = Rig::bench();
        at(&mut r.c, pos, i, 1913);
        r.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.mode.data_flags = 0;
        });
        r
    }

    fn at(
        c: &mut osc_client::blocking::Client<osc_client::fake::FakePipe>,
        pos: u16,
        i: i16,
        ntc: u16,
    ) {
        let mut s = TelSample {
            vbus_counts: 3204,
            ..TelSample::default()
        };
        (
            s.tick.pos,
            s.tick.vbus_raw,
            s.tick.ntc_raw,
            s.tick.i_tick_counts,
        ) = (pos, 1961, ntc, i);
        c.pipe_mut().set_track(0, vec![s]);
        // the estimate torque comes on at, at the pot's reading
        c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.estimates.theta_hat_q16 = (pos as i32) << 16;
        });
    }

    fn cfg(r: &mut Rig, argv: &[&str]) -> Cfg {
        prepare(r, &parse(argv)).unwrap()
    }

    fn refused(r: &mut Rig, argv: &[&str]) -> String {
        let e = prepare(r, &parse(argv)).err().expect("not refused");
        assert_eq!(exit_code(&e), Some(2), "{e}");
        e.to_string()
    }

    /// What a test keeps of a segment: its index, kind, goal and rows.
    type Seen = (usize, &'static str, i32, usize);

    /// Every segment that lands, and how the run ended.
    fn drive(r: &mut Rig, c: &mut Cfg) -> (Vec<Seen>, Ran, Result<()>) {
        let mut seen = Vec::new();
        let mut ran = Ran::default();
        let res = record(r, c, &mut ran, |g| {
            seen.push((g.idx, g.item.kind(), g.goal, g.frames.len()));
            Ok(())
        });
        (seen, ran, res)
    }

    #[test]
    fn items_parse_and_display_round_trip() {
        for (s, want) in [
            (
                "set:velocity_limit_cps=800",
                Item::Set {
                    field: "velocity_limit_cps".into(),
                    value: "800".into(),
                },
            ),
            (
                "goal:2400@500",
                Item::Goal {
                    value: 2400,
                    ms: 500,
                },
            ),
            (
                "goal:-300@20",
                Item::Goal {
                    value: -300,
                    ms: 20,
                },
            ),
            ("hold@1000", Item::Hold { ms: 1000 }),
            ("rest@250", Item::Rest { ms: 250 }),
            ("rest@60000", Item::Rest { ms: 60_000 }),
            (
                "goal:2400@3276",
                Item::Goal {
                    value: 2400,
                    ms: 3276,
                },
            ),
        ] {
            assert_eq!(parse_item(s).unwrap(), want, "{s}");
            assert_eq!(want.to_string(), s);
        }
        for bad in [
            "goal:2400",
            "goal:x@100",
            "goal:2400@0",
            "goal:2400@x",
            "hold",
            "hold@0",
            "rest@",
            "set:velocity_limit_cps",
            "set:=5",
            "set:x=",
            "2400@100",
            "wait@100",
        ] {
            assert!(parse_item(bad).is_err(), "{bad} parsed");
        }
    }

    /// A capture one burst cannot carry is refused with the limit named; a
    /// rest is no capture and takes any span.
    #[test]
    fn a_capture_over_one_burst_is_refused_at_parse() {
        assert_eq!(MAX_BURST_MS, 3276);
        for s in ["goal:2400@3277", "hold@5000"] {
            let e = parse_item(s).unwrap_err();
            assert!(e.contains("at most 3276 ms"), "{e}");
        }
        let cli = <Cli as clap::Parser>::try_parse_from([
            "track",
            "--out",
            "o",
            "--mode",
            "position",
            "--schedule",
            "hold@100,hold@4000",
        ]);
        assert!(cli.is_err());
        let a = parse(&[
            "--mode",
            "velocity",
            "--schedule",
            "set:v_kp_q88=300,goal:-500@200,rest@9000",
        ]);
        assert_eq!(a.schedule.len(), 3);
        assert_eq!(
            (a.deadman_ms, a.current_cap_ms, a.no_center),
            (1000, 50, false)
        );
    }

    /// Every option as `oscnb.osc.track` passes it.
    #[test]
    fn every_option_parses_as_the_python_wrapper_passes_it() {
        let a = parse(&[
            "--mode",
            "current",
            "--schedule",
            "goal:-50@20",
            "--tel-chunks",
            "0x238+2,0x240+10",
            "--current-cap-ma",
            "300",
            "--current-cap-ms",
            "20",
            "--ntc-max-c",
            "60",
            "--guard-lo",
            "-5",
            "--guard-hi",
            "3000",
            "--deadman-ms",
            "800",
            "--deadman-check",
            "--no-center",
            "--goal-period-ms",
            "25.6",
        ]);
        assert_eq!(a.mode, Mode::Current);
        assert_eq!(a.goal_period_ms, Some(25.6));
        assert_eq!((a.current_cap_ma, a.current_cap_ms), (Some(300.0), 20));
        assert_eq!(
            (a.ntc_max_c, a.guard_lo, a.guard_hi),
            (Some(60.0), Some(-5), Some(3000))
        );
        assert_eq!(
            (a.deadman_ms, a.deadman_check, a.no_center),
            (800, true, true)
        );
    }

    #[test]
    fn set_items_take_config_and_goals_by_name_and_refuse_the_rest() {
        let d = crate::descriptor::builtin();
        assert_eq!(
            resolve_set(&d, "velocity_limit_cps", "800").unwrap(),
            (
                Reg {
                    addr: 0x44,
                    width: 2
                },
                800
            )
        );
        assert_eq!(
            resolve_set(&d, "goal_current", "-120").unwrap(),
            (control::GOAL_CURRENT, -120)
        );
        assert_eq!(resolve_set(&d, "openloop_zero_brake", "true").unwrap().1, 1);
        assert_eq!(resolve_set(&d, "p_kp_q88", "0x100").unwrap().1, 256);
        for (f, v) in [
            ("torque_enable", "1"),
            ("stall_permit", "1"),
            ("mode", "3"),
            ("bus_deadman_ticks", "0"),
            ("tel_count", "100"),
            ("id", "2"),
            ("baud_rate_idx", "0"),
            ("velocity_limit_cps", "-1"),
            ("velocity_limit_cps", "fast"),
            ("no_such_field", "1"),
        ] {
            assert!(resolve_set(&d, f, v).is_err(), "set:{f}={v}");
        }
    }

    /// The bench servo's defaults: the guard is its soft limits, the cap a
    /// quarter over its limit, the NTC cap its derate start, the dead-man
    /// 20000 ticks and the middle of the soft range the end.
    #[test]
    fn a_run_takes_its_defaults_from_the_servo() {
        let mut r = rig_at(2029, 0);
        let c = cfg(
            &mut r,
            &["--mode", "position", "--schedule", "goal:2100@20"],
        );
        assert_eq!(c.wd.guard, (432, 3626));
        assert_eq!(c.wd.cap, 350.0);
        assert_eq!((c.wd.cap_rows, c.wd.cap_polls), (1000, 1));
        assert_eq!(c.deadman_ticks, 20_000);
        assert_eq!(c.center, Some(2029));
        assert_eq!(c.wd.lin(2029), 2029, "the identity table");
        assert!(r.writes.is_empty(), "nothing written before the run");
    }

    /// Each refusal before torque, exit 2, with nothing written: the
    /// experiment image, a data state that refuses the mode, a standing
    /// fault, a shaft outside the guard, a flat pack, and items or a
    /// dead-man the servo cannot carry.
    #[test]
    fn preflight_refuses_before_any_torque() {
        let pos = ["--mode", "position", "--schedule", "goal:2100@20"];
        let mut r = rig_at(2029, 0);
        r.experiment = true;
        assert!(refused(&mut r, &pos).contains("experiment image"));

        // the bench table's covered edits read STAMP_MISMATCH
        let mut r = Rig::bench();
        at(&mut r.c, 2029, 0, 1913);
        let e = refused(&mut r, &pos);
        assert!(
            e.contains("STAMP_MISMATCH") && e.contains("position mode"),
            "{e}"
        );
        assert!(
            prepare(
                &mut r,
                &parse(&["--mode", "current", "--schedule", "goal:50@20"])
            )
            .is_ok()
        );

        let mut r = rig_at(2029, 0);
        r.latch(1, 1);
        assert!(refused(&mut r, &pos).contains("fault standing: flags 0x01 code 1 (over_current)"));

        let mut r = rig_at(300, 0);
        assert!(
            refused(&mut r, &pos)
                .contains("the shaft at 300 counts is outside the guard 432..3626")
        );
        let mut r = rig_at(300, 0);
        assert!(
            prepare(
                &mut r,
                &parse(&[
                    "--guard-lo",
                    "200",
                    "--mode",
                    "position",
                    "--schedule",
                    "goal:2100@20"
                ])
            )
            .is_ok()
        );

        let mut r = rig_at(2029, 0);
        crate::rig::testkit::rail_at(&mut r.c, 1675, 3204);
        assert!(refused(&mut r, &pos).contains("under its floor"));

        let mut r = rig_at(2029, 0);
        assert!(
            refused(
                &mut r,
                &["--mode", "position", "--schedule", "goal:4000@20"]
            )
            .contains("outside the guard")
        );
        assert!(
            refused(&mut r, &["--mode", "current", "--schedule", "goal:400@20"])
                .contains("current limit of 280")
        );
        assert!(
            refused(&mut r, &["--mode", "position", "--schedule", "set:mode=0"])
                .contains("the run owns")
        );
        assert!(
            refused(
                &mut r,
                &[
                    "--deadman-ms",
                    "300",
                    "--mode",
                    "position",
                    "--schedule",
                    "hold@20"
                ]
            )
            .contains("under 500 ms")
        );
        assert!(
            refused(
                &mut r,
                &[
                    "--deadman-ms",
                    "4000",
                    "--mode",
                    "position",
                    "--schedule",
                    "hold@20"
                ]
            )
            .contains("over the field's 65535")
        );
        assert!(
            refused(
                &mut r,
                &[
                    "--deadman-ms",
                    "0",
                    "--deadman-check",
                    "--mode",
                    "position",
                    "--schedule",
                    "hold@20"
                ]
            )
            .contains("needs a dead-man")
        );
        r.cap = Some(crate::rig::pump::BurstCap {
            rows: 1000,
            tick_hz: 20_000.0,
        });
        assert!(
            refused(&mut r, &["--mode", "position", "--schedule", "hold@60"])
                .contains("over the 1000 rows")
        );
        assert!(r.writes.is_empty(), "nothing written by a refusal");
    }

    /// Torque comes on at the servo's estimate, not at the pot sample the
    /// poll read: a goal off the estimate is a move the loop makes.
    #[test]
    fn torque_comes_on_at_the_estimate() {
        let mut r = rig_at(2029, 0);
        r.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.estimates.theta_hat_q16 = (2025 << 16) + 0x6000;
        });
        let mut c = cfg(&mut r, &["--mode", "position", "--schedule", "hold@20"]);
        let (_, _, res) = drive(&mut r, &mut c);
        res.unwrap();
        let goal = r.writes.iter().find(|w| w.0 == control::GOAL_POSITION);
        assert_eq!(goal, Some(&(control::GOAL_POSITION, 2025)));
    }

    /// The dead-man goes out before torque on, the goal holds the present
    /// position, the keepalive feeds it through a rest, and no write ever
    /// asks for the stall permit.
    #[test]
    fn the_deadman_leads_torque_on_and_the_permit_is_never_held() {
        let mut r = rig_at(2029, 0);
        let mut c = cfg(
            &mut r,
            &[
                "--mode",
                "position",
                "--schedule",
                "goal:2100@20,rest@600,hold@20",
            ],
        );
        let (seen, _, res) = drive(&mut r, &mut c);
        res.unwrap();
        assert_eq!(
            seen,
            [
                (0, "goal", 2100, 400),
                (1, "rest", 2100, 0),
                (2, "hold", 2100, 400)
            ]
        );
        let deadman = (control::BUS_DEADMAN_TICKS, 20_000);
        assert_eq!(
            r.writes[..4],
            [
                (control::MODE, 3),
                (control::GOAL_POSITION, 2029),
                deadman,
                (control::TORQUE_ENABLE, 1),
            ]
        );
        let feeds = r.writes.iter().filter(|w| **w == deadman).count();
        assert!(feeds >= 3, "fed through the rest: {feeds}");
        assert!(!r.writes.contains(&(control::STALL_PERMIT, 1)));

        // no dead-man: none written, torque on all the same
        let mut r = rig_at(2029, 0);
        let mut c = cfg(
            &mut r,
            &[
                "--deadman-ms",
                "0",
                "--mode",
                "velocity",
                "--schedule",
                "goal:-500@20",
            ],
        );
        drive(&mut r, &mut c).2.unwrap();
        assert_eq!(
            r.writes[..3],
            [
                (control::MODE, 2),
                (control::GOAL_VELOCITY, 0),
                (control::TORQUE_ENABLE, 1)
            ]
        );
        assert!(!r.writes.iter().any(|w| w.0 == control::BUS_DEADMAN_TICKS));
    }

    /// An image that honors the dead-man latches it in the silence and the
    /// run acks it and goes on; one that ignores it is refused, exit 2,
    /// with torque off and nothing captured.
    #[test]
    fn the_deadman_check_proves_the_image_or_refuses() {
        let argv = [
            "--deadman-check",
            "--mode",
            "position",
            "--schedule",
            "hold@20",
        ];
        let mut r = rig_at(2029, 0);
        r.deadman_honored = true;
        let mut c = cfg(&mut r, &argv);
        let (seen, ran, res) = drive(&mut r, &mut c);
        res.unwrap();
        assert_eq!(ran.deadman, Some("honored"));
        assert_eq!(seen.len(), 1);
        let on = r
            .writes
            .iter()
            .filter(|w| **w == (control::TORQUE_ENABLE, 1))
            .count();
        assert_eq!(on, 2, "the enable, then the ack");

        let mut r = rig_at(2029, 0);
        let mut c = cfg(&mut r, &argv);
        let (seen, ran, res) = drive(&mut r, &mut c);
        let e = res.unwrap_err();
        assert_eq!(exit_code(&e), Some(2));
        assert!(
            e.to_string()
                .contains("this image does not honor the bus dead-man"),
            "{e}"
        );
        assert!(seen.is_empty() && ran.deadman.is_none());
        let on = r
            .writes
            .iter()
            .position(|w| *w == (control::TORQUE_ENABLE, 1))
            .unwrap();
        assert_eq!(
            r.writes[on + 1],
            (control::TORQUE_ENABLE, 0),
            "torque off next"
        );
        assert_eq!(
            r.writes
                .iter()
                .filter(|w| **w == (control::TORQUE_ENABLE, 1))
                .count(),
            1
        );
    }

    /// The abort path for one condition planted after the first burst:
    /// torque off is the next write, the second item never runs, nothing
    /// drives again, and the abort names the item.
    fn aborts_after_first_burst(
        argv: &[&str],
        plant: impl FnMut(&mut osc_client::blocking::Client<osc_client::fake::FakePipe>, u32) + 'static,
    ) -> String {
        let mut r = rig_at(2029, 0);
        let mut c = cfg(&mut r, argv);
        r.after_burst = Some(Box::new(plant));
        let (seen, ran, res) = drive(&mut r, &mut c);
        let e = res.unwrap_err();
        assert_eq!(exit_code(&e), Some(3), "{e}");
        let a = e.downcast_ref::<Abort>().unwrap();
        assert_eq!(a.seg, Some(0));
        assert_eq!(seen.len(), 1, "the aborting item's rows are kept");
        assert!(ran.set.is_some());
        let k = r
            .writes
            .iter()
            .position(|w| w.0 == c.mode.goal() && w.1 == 2100)
            .unwrap();
        assert_eq!(
            r.writes[k + 1],
            (control::TORQUE_ENABLE, 0),
            "torque off next"
        );
        let after = &r.writes[k + 1..];
        assert!(!after.contains(&(control::TORQUE_ENABLE, 1)), "drove again");
        assert!(
            !after
                .iter()
                .any(|w| w.0 == control::MODE || w.0 == control::GOAL_POSITION)
        );
        assert_eq!(after[after.len() - 6..], crate::rig::servo::SAFE);
        e.to_string()
    }

    #[test]
    fn the_watchdog_aborts_on_each_condition() {
        let pos = [
            "--mode",
            "position",
            "--schedule",
            "goal:2100@20,goal:2200@20",
        ];
        let e = aborts_after_first_burst(&pos, |c, k| {
            if k == 0 {
                c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
                    t.telemetry.common.fault_flags = 1 << 2;
                    t.telemetry.mode.fault_code = 3;
                });
            }
        });
        assert!(
            e.contains("segment 0 (goal:2100@20)") && e.contains("code 3 (stall)"),
            "{e}"
        );

        let e = aborts_after_first_burst(&pos, |c, k| {
            if k == 0 {
                at(c, 3700, 0, 1913)
            }
        });
        assert!(
            e.contains("the shaft at 3700 counts is outside the guard"),
            "{e}"
        );

        let e = aborts_after_first_burst(&pos, |c, k| {
            if k == 0 {
                at(c, 2029, 400, 1913)
            }
        });
        assert!(
            e.contains("|current| 400 counts over the 350 count cap"),
            "{e}"
        );

        // the seed curve reads 600 as 64 C
        let hot = [
            "--ntc-max-c",
            "50",
            "--mode",
            "position",
            "--schedule",
            "goal:2100@20,goal:2200@20",
        ];
        let e = aborts_after_first_burst(&hot, |c, k| {
            if k == 0 {
                at(c, 2029, 0, 600)
            }
        });
        assert!(
            e.contains("the board NTC reads") && e.contains("over the"),
            "{e}"
        );

        let cur = [
            "--current-cap-ms",
            "100",
            "--mode",
            "position",
            "--schedule",
            "goal:2100@20,hold@20",
        ];
        let mut r = rig_at(2029, 400);
        let mut c = cfg(&mut r, &cur);
        assert_eq!(c.wd.cap_polls, 2);
        let res = drive(&mut r, &mut c).2;
        assert!(res.unwrap_err().to_string().contains("for 2 polls"));
    }

    /// The rows' own judge: the guard per row, the cap over consecutive
    /// rows that read current, an unread window neither counting nor
    /// ending a run.
    #[test]
    fn rows_are_judged_by_the_guard_and_the_cap() {
        let mut r = rig_at(2029, 0);
        let c = cfg(
            &mut r,
            &[
                "--current-cap-ms",
                "0",
                "--mode",
                "position",
                "--schedule",
                "hold@20",
            ],
        );
        let wd = Watchdog {
            cap_rows: 3,
            ..c.wd
        };
        let row = |tick, pos, raw, valid| TelFrame {
            tick,
            pos: Some(pos),
            current_raw: Some(raw),
            window_valid: valid,
            ..TelFrame::default()
        };
        let over = |t| row(t, 2029, 2400, true);
        let blind = |t| row(t, 2029, 4000, false);
        assert!(
            wd.rows(
                &[
                    over(0),
                    over(1),
                    blind(2),
                    row(3, 2029, 2100, true),
                    over(4)
                ],
                2000
            )
            .is_ok()
        );
        let e = wd
            .rows(&[over(0), blind(1), over(2), over(3)], 2000)
            .unwrap_err();
        assert!(e.contains("for 3 rows, to row tick 3"), "{e}");
        let e = wd
            .rows(&[row(0, 2029, 0, false), row(1, 3700, 0, false)], 0)
            .unwrap_err();
        assert!(
            e.contains("outside the guard 432..3626 at row tick 1"),
            "{e}"
        );
        let held = TelFrame {
            current: Some(-500),
            ..row(0, 2029, 0, false)
        };
        assert!(
            wd.rows(&[held, held, held], 0)
                .unwrap_err()
                .contains("for 3 rows")
        );
        // current flowing back reads under the bias, and counts as much
        let under = |t| row(t, 2029, 1600, true);
        let e = wd.rows(&[under(0), under(1), under(2)], 2000).unwrap_err();
        assert!(e.contains("for 3 rows, to row tick 2"), "{e}");
        assert!(
            wd.rows(&[under(0), under(1), row(2, 2029, 1700, true)], 2000)
                .is_ok()
        );
    }

    /// A burst that comes back short aborts with what it held.
    #[test]
    fn a_short_burst_aborts() {
        let mut r = rig_at(2029, 0);
        let mut c = cfg(
            &mut r,
            &["--mode", "position", "--schedule", "goal:2100@20"],
        );
        r.c.pipe_mut().sim_mut().lose_tel_ticks(0, 32, 64);
        let (seen, _, res) = drive(&mut r, &mut c);
        let e = res.unwrap_err();
        assert!(e.to_string().contains("came back short"), "{e}");
        assert_eq!(seen.len(), 1);
    }

    /// A train of `n` 20 ms goals, a count apart.
    fn train(n: i32) -> String {
        let goals: Vec<_> = (0..n).map(|k| format!("goal:{}@20", 2100 + k)).collect();
        goals.join(",")
    }

    /// What a test of the period keeps of a goal.
    struct Commit {
        /// The servo's tick it landed on, low 16 bits.
        tick: u16,
        staged_ms: f64,
        sent_ms: f64,
        held: Option<Held>,
    }

    /// A run on a rig whose clock its exchanges and bursts move: every
    /// goal's commit, the run's pace and what each carrier took to land.
    fn commits(timed: Timed, argv: &[&str]) -> (Vec<Commit>, Option<Pace>, Vec<f64>) {
        let mut r = rig_at(2029, 0);
        r.timed = Some(timed);
        // the stamps wrap inside the run
        r.t_ms = 3200.0;
        let mut c = cfg(&mut r, argv);
        let mut seen = Vec::new();
        record(&mut r, &mut c, &mut Ran::default(), |g| {
            if let Item::Goal { .. } = g.item {
                seen.push(Commit {
                    tick: g.stats.stamp0.unwrap(),
                    staged_ms: g.stats.staged_ms,
                    sent_ms: g.stats.sent_ms,
                    held: g.held,
                });
            }
            Ok(())
        })
        .unwrap();
        (seen, c.pace, r.landings)
    }

    /// The servo's tick 0.04 % fast on the host's clock, a carrier landing
    /// the instant it is sent.
    const FAST: f64 = 1.0004;

    fn timed(exchange_ms: f64) -> Timed {
        Timed {
            exchange_ms,
            stalls: Vec::new(),
            tick_ratio: FAST,
            landing_ms: (0.0, 0.0),
        }
    }

    /// Ticks from each commit to the next.
    fn spacing(seen: &[Commit]) -> Vec<u16> {
        seen.windows(2)
            .map(|w| w[1].tick.wrapping_sub(w[0].tick))
            .collect()
    }

    /// Ticks each goal of a train landed past its own grid point on the
    /// servo's tick, the first goal's 0, at `period` ticks a point.
    fn past_grid(seen: &[Commit], period: f64) -> Vec<f64> {
        let (mut ticks, mut point) = (0.0, 0.0);
        let mut past = vec![0.0];
        for (w, d) in seen.windows(2).zip(spacing(seen)) {
            ticks += d as f64;
            point += 1.0 + w[1].held.unwrap().slots_missed as f64;
            past.push(ticks - point * period);
        }
        past
    }

    fn mean(x: &[f64]) -> f64 {
        x.iter().sum::<f64>() / x.len() as f64
    }

    fn sd(x: &[f64]) -> f64 {
        let m = mean(x);
        (x.iter().map(|v| (v - m).powi(2)).sum::<f64>() / (x.len() - 1) as f64).sqrt()
    }

    /// With a period every goal of a train commits one period after the
    /// goal before it on the servo's tick, whatever the host's exchanges
    /// take, what a carrier takes to land when that is constant, and
    /// though the host's clock and the servo's disagree; the train's whole
    /// span is the periods' sum. The first goal starts the grid and waits
    /// for nothing.
    #[test]
    fn goals_commit_one_period_apart_on_the_servos_tick() {
        let schedule = train(12);
        let argv = [
            "--goal-period-ms",
            "25.6",
            "--mode",
            "position",
            "--schedule",
            &schedule,
        ];
        for exchange_ms in [0.3, 0.9] {
            let lands = Timed {
                landing_ms: (0.7, 0.0),
                ..timed(exchange_ms)
            };
            let (seen, pace, _) = commits(lands, &argv);
            assert_eq!(seen.len(), 12);
            assert!(seen[0].held.is_none() && seen[0].sent_ms == seen[0].staged_ms);
            for (k, ticks) in spacing(&seen).into_iter().enumerate() {
                assert!(ticks.abs_diff(512) <= 1, "{exchange_ms} ms: {ticks} ticks");
                let held = seen[k + 1].held.unwrap();
                assert!(held.wait_ms > 0.0);
                assert_eq!((held.late_ms, held.slots_missed), (0.0, 0));
            }
            let whole = seen[11].tick.wrapping_sub(seen[0].tick);
            assert!(whole.abs_diff(11 * 512) <= 2, "{whole} ticks");
            let pace = pace.unwrap();
            assert_eq!((pace.held, pace.late), (11, 0));
        }
    }

    /// Without a period a goal goes out as soon as the host is ready: the
    /// commits are the burst and the item's five exchanges apart (the poll
    /// before it, the two staged writes, the drop counter, the poll after
    /// it), so the train runs at the host's pace.
    #[test]
    fn without_a_period_goals_go_out_at_the_hosts_pace() {
        let schedule = train(12);
        let argv = [
            "--deadman-ms",
            "0",
            "--mode",
            "position",
            "--schedule",
            &schedule,
        ];
        for exchange_ms in [0.3, 0.9] {
            let (seen, pace, _) = commits(timed(exchange_ms), &argv);
            assert!(pace.is_none());
            let ticks_per_ms = TICK_HZ as f64 / 1000.0 * FAST;
            let want = (400.0 + 5.0 * exchange_ms * ticks_per_ms).round() as u16;
            for ticks in spacing(&seen) {
                assert!(ticks.abs_diff(want) <= 1, "{ticks} ticks, not {want}");
            }
            assert!(
                seen.iter()
                    .all(|g| g.held.is_none() && g.sent_ms == g.staged_ms)
            );
        }
    }

    /// A carrier's time to land scattered by 1 ms sd goal to goal, the
    /// servo's tick 0.15 % fast on the host's clock, a train of 180 (the
    /// sim plays a minute a run): the commit spacing on the servo's tick
    /// scatters by what two landings' scatter alone gives it and the
    /// trim's 7 % at most, the goals stay on the grid to a landing's own
    /// scatter about the trim's standing 1.2 % of a period, where the
    /// clocks alone would walk them 160 ticks off, and stalls of the host
    /// change none of it: a stalled goal goes out late, the goals after it
    /// make the stall up at the period's room, and the grid they come back
    /// to is the one they left.
    #[test]
    fn the_grid_stands_through_scattered_landings_and_stalls() {
        const PERIOD: f64 = 600.0;
        let schedule = train(180);
        let argv = [
            "--goal-period-ms",
            "30",
            "--mode",
            "position",
            "--schedule",
            &schedule,
        ];
        let scattered = Timed {
            tick_ratio: 1.0015,
            landing_ms: (3.0, 1.0),
            ..timed(0.1)
        };
        let ticks_per_ms = TICK_HZ as f64 / 1000.0 * scattered.tick_ratio;
        let standing = 0.0015 * PERIOD / TRIM_GAIN;
        let on_grid = |past: &[f64]| {
            let settled = &past[60..];
            assert!(
                (mean(settled) - standing).abs() < 5.0,
                "{} ticks",
                mean(settled)
            );
            let worst = past[30..].iter().fold(0.0f64, |m, p| m.max(p.abs()));
            assert!(worst < 100.0, "{worst} ticks");
        };

        let (seen, pace, landings) = commits(scattered.clone(), &argv);
        let pace = pace.unwrap();
        assert_eq!((pace.held, pace.late), (179, 0));
        let spaced: Vec<f64> = spacing(&seen).into_iter().map(f64::from).collect();
        let landed: Vec<f64> = landings
            .windows(2)
            .map(|w| (w[1] - w[0]) * ticks_per_ms)
            .collect();
        assert!((sd(&landings) - 1.0).abs() < 0.1, "{} ms", sd(&landings));
        assert!(
            sd(&spaced) < 1.07 * sd(&landed),
            "{} ticks against the landings' {}",
            sd(&spaced),
            sd(&landed)
        );
        assert!((mean(&spaced) - PERIOD).abs() < 0.5, "{}", mean(&spaced));
        on_grid(&past_grid(&seen, PERIOD));

        // own time 23.4 ms a goal, so 6.6 ms of room: a 20 ms stall is
        // made up within three goals
        let stalled = Timed {
            stalls: (1..6).map(|k| (30 * k, 20.0)).collect(),
            ..scattered
        };
        let (seen, pace, _) = commits(stalled.clone(), &argv);
        for (k, _) in &stalled.stalls {
            let k = *k as usize;
            let late = |g: &Commit| g.held.unwrap().late_ms;
            assert!(
                (late(&seen[k]) - (20.0 - 6.6)).abs() < 4.0,
                "{}",
                late(&seen[k])
            );
            assert!(late(&seen[k + 1]) < late(&seen[k]) - 3.0);
            assert_eq!(late(&seen[k + 3]), 0.0, "goal {k}'s stall made up");
            assert!(
                seen[k..k + 4]
                    .iter()
                    .all(|g| g.held.unwrap().slots_missed == 0)
            );
        }
        let pace = pace.unwrap();
        assert!((10..=15).contains(&pace.late), "{} late", pace.late);
        let past: Vec<f64> = past_grid(&seen, PERIOD)
            .into_iter()
            .zip(&seen)
            .filter(|(_, g)| g.held.is_none_or(|h| h.late_ms == 0.0))
            .map(|(p, _)| p)
            .collect();
        on_grid(&past);
    }

    /// A goal the host has not staged by its grid point goes out at once
    /// and is late by what it was staged after the point; the goals after
    /// it aim at their own points, each late by the period's room less,
    /// and the first one with room to spare is back on the grid the train
    /// started. A goal a whole period late or more has missed the points
    /// it went past, and the goal after it aims at the next one. A stall
    /// longer than the stamp's wrap is counted by the host's clock.
    #[test]
    fn a_stalled_goal_goes_out_late_and_the_grid_stands() {
        let schedule = train(18);
        let argv = [
            "--goal-period-ms",
            "25.6",
            "--deadman-ms",
            "0",
            "--mode",
            "position",
            "--schedule",
            &schedule,
        ];
        // own time 23.4 ms a goal: 2.2 ms of room
        let room = 2.2;
        for (stall_ms, missed) in [(10.0, 0u32), (61.0, 2), (3300.0, 128)] {
            let stalled = Timed {
                stalls: vec![(4, stall_ms)],
                tick_ratio: 1.0,
                ..timed(0.85)
            };
            let (seen, pace, _) = commits(stalled, &argv);
            let first = stall_ms - room - missed as f64 * 25.6;
            let late = (first / room).ceil() as usize;
            for (k, g) in seen.iter().enumerate().skip(1) {
                let held = g.held.unwrap();
                let (want_late, want_missed) = match k {
                    4 => (stall_ms - room, missed),
                    k if k > 4 && k < 4 + late => (first - (k - 4) as f64 * room, 0),
                    _ => (0.0, 0),
                };
                assert!(
                    (held.late_ms - want_late).abs() < 1e-6,
                    "goal {k} of a {stall_ms} ms stall: late {} ms",
                    held.late_ms
                );
                assert_eq!(held.slots_missed, want_missed);
                assert_eq!(held.wait_ms > 0.0, want_late == 0.0);
            }
            let k = 4 + late;
            let points = (k - 3) as u32 + missed;
            assert_eq!(
                seen[k].tick.wrapping_sub(seen[3].tick),
                (512 * points) as u16,
                "back on the grid after a {stall_ms} ms stall"
            );
            assert!(spacing(&seen[k..]).iter().all(|t| *t == 512));
            let pace = pace.unwrap();
            assert_eq!((pace.held, pace.late), (17, late as u32));
        }
    }

    /// A period the host cannot make says so: every goal goes out late, a
    /// little later than the one before, and the train runs at the host's
    /// pace.
    #[test]
    fn a_period_the_host_cannot_hold_counts_every_goal_late() {
        let schedule = train(6);
        let argv = [
            "--goal-period-ms",
            "22",
            "--deadman-ms",
            "0",
            "--mode",
            "position",
            "--schedule",
            &schedule,
        ];
        let slow = Timed {
            tick_ratio: 1.0,
            ..timed(0.9)
        };
        let (seen, pace, _) = commits(slow, &argv);
        for (k, g) in seen.iter().enumerate().skip(1) {
            let held = g.held.unwrap();
            assert!(
                (held.late_ms - 1.6 * k as f64).abs() < 1e-6,
                "{}",
                held.late_ms
            );
            assert_eq!((held.wait_ms, held.slots_missed), (0.0, 0));
        }
        assert!(spacing(&seen).iter().all(|t| *t == 472));
        let pace = pace.unwrap();
        assert_eq!((pace.held, pace.late), (5, 5));
    }

    /// The grid is a train's: a hold, a rest or a set ends it, and a goal
    /// whose burst outlasts the period is followed by a first goal. The
    /// files carry each held goal's room, lateness and missed points, and
    /// the run's count.
    #[test]
    fn the_period_holds_trains_and_the_files_say_how() {
        let out = std::env::temp_dir().join(format!("osc-lab-track-pace-{}", std::process::id()));
        let mut r = rig_at(2029, 0);
        r.timed = Some(Timed {
            stalls: vec![(2, 20.0)],
            ..timed(0.5)
        });
        let a = parse(&[
            "--goal-period-ms",
            "25.6",
            "--mode",
            "position",
            "--schedule",
            "goal:2100@100,goal:2101@20,goal:2102@20,goal:2103@20,hold@20,goal:2104@20,\
             goal:2105@20,rest@50,goal:2106@20,set:velocity_limit_cps=777,goal:2107@20",
        ]);
        capture(&mut r, &a, &out).unwrap();
        let segs = std::fs::read_to_string(out.join("segments.csv")).unwrap();
        let cols: Vec<Vec<&str>> = segs.lines().map(|l| l.split(',').collect()).collect();
        assert_eq!(
            cols[0][6..9],
            ["slot_wait_ms", "slot_late_ms", "slots_missed"]
        );
        let held: Vec<bool> = cols[1..].iter().map(|c| !c[8].is_empty()).collect();
        let t = true;
        assert_eq!(
            held,
            [
                false, false, t, t, false, false, t, false, false, false, false
            ]
        );
        let ms = |row: usize, col: usize| cols[row][col].parse::<f64>().unwrap();
        for c in &cols[1..] {
            assert_eq!(
                (c[6].is_empty(), c[7].is_empty()),
                (c[8].is_empty(), c[8].is_empty())
            );
        }
        // own time 22 ms a goal, 3.6 ms of room: the 20 ms stall is 16.4 ms
        // late, the goal after it 12.8, and a goal on time waits the room
        assert!(
            (ms(3, 7) - 16.4).abs() < 0.1 && ms(3, 6) == 0.0,
            "{}",
            cols[3][7]
        );
        assert!(
            (ms(4, 7) - 12.8).abs() < 0.1 && ms(4, 6) == 0.0,
            "{}",
            cols[4][7]
        );
        assert!(
            (ms(7, 6) - 3.6).abs() < 0.6 && ms(7, 7) == 0.0,
            "{}",
            cols[7][6]
        );
        assert!(cols[1..].iter().all(|c| c[8].is_empty() || c[8] == "0"));
        let meta: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(out.join("meta.json")).unwrap()).unwrap();
        assert_eq!(
            meta["goal_period"],
            serde_json::json!({ "ms": 25.6, "held": 3, "late": 2 })
        );
        std::fs::remove_dir_all(&out).unwrap();
    }

    /// A period is over 0 and at most the keepalive's cadence, since the
    /// wait for a grid point is bus silence: at the longest period and the
    /// shortest dead-man the wait never trips an image that honors it.
    #[test]
    fn the_wait_for_a_grid_point_stays_inside_the_deadman() {
        let mut r = rig_at(2029, 0);
        for bad in ["0", "250.5", "NaN"] {
            let e = refused(
                &mut r,
                &[
                    "--goal-period-ms",
                    bad,
                    "--mode",
                    "position",
                    "--schedule",
                    "goal:2100@20",
                ],
            );
            assert!(e.contains("at most 250 ms"), "{e}");
        }
        r.deadman_honored = true;
        r.timed = Some(timed(0.5));
        let schedule = train(6);
        let mut c = cfg(
            &mut r,
            &[
                "--goal-period-ms",
                "250",
                "--deadman-ms",
                "500",
                "--mode",
                "position",
                "--schedule",
                &schedule,
            ],
        );
        let (seen, _, res) = drive(&mut r, &mut c);
        res.unwrap();
        assert_eq!(seen.len(), 6);
        assert_eq!(c.pace.unwrap().held, 5);
    }

    /// What an item repeats is read once a run: the identity block and the
    /// window floor at preflight, the drop counter ahead of the first
    /// burst and after each, so the run itself reads the counter alone.
    /// Every row the servo dropped is still counted, on the burst it was
    /// dropped from. Without a period every goal is polled before and
    /// after; inside a train held to one there is one poll between two
    /// bursts.
    #[test]
    fn an_item_reads_once_what_it_repeats_and_counts_every_drop() {
        use osc_protocol::table::{BAUD_RATE_IDX, CAPABILITY_FLAGS};
        let identity = (CAPABILITY_FLAGS, BAUD_RATE_IDX + 1 - CAPABILITY_FLAGS);
        let floor = (calib::I_WINDOW_MIN_TICKS.addr, 2);
        let counter = (telemetry::TEL_DROP_COUNT.addr, 2);
        let count = |reads: &[(u16, u16)], span| reads.iter().filter(|r| **r == span).count();
        let schedule = train(5);
        let free = ["--no-center", "--mode", "position", "--schedule", &schedule];
        let held = [&["--goal-period-ms", "25.6"], &free[..]].concat();
        // the poll torque comes on at, then the goals' own
        for (argv, polls) in [(&free[..], 1 + 2 * 5), (&held[..], 1 + 1 + 5)] {
            let mut r = rig_at(2029, 0);
            r.reads();
            let mut c = cfg(&mut r, argv);
            let preflight = r.reads();
            assert_eq!(
                (count(&preflight, identity), count(&preflight, floor)),
                (1, 1)
            );
            r.after_burst = Some(Box::new(|c, k| {
                c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
                    let h = &mut t.telemetry_ext.health;
                    h.tel_drop_count = h.tel_drop_count.wrapping_add(k as u16 % 2 * 3);
                });
            }));
            let mut dropped = Vec::new();
            let mut ran = Ran::default();
            record(&mut r, &mut c, &mut ran, |g| {
                dropped.push(g.stats.rows_dropped);
                Ok(())
            })
            .unwrap();
            assert_eq!(dropped, [0, 3, 0, 3, 0]);
            assert_eq!(ran.rows_dropped, 6);
            let run = r.reads();
            assert_eq!((count(&run, identity), count(&run, floor)), (0, 0));
            assert_eq!(count(&run, counter), 6);
            assert_eq!(count(&run, (POLL_BASE, POLL_LEN)), polls);
        }
    }

    /// Inside a train there is one poll between two bursts, and it sits
    /// behind the next goal's staged writes, ahead of its COMMIT: the first
    /// goal of a train is polled before it as any item is, a goal the next
    /// one follows on the grid leaves the poll after its burst to that
    /// goal, and the last goal of the train is polled after its burst.
    #[test]
    fn a_train_polls_once_between_bursts_behind_the_staged_writes() {
        use crate::rig::testkit::Sent::{Commit, Hold, Read, Write};
        let mut r = rig_at(2029, 0);
        let schedule = format!("{},hold@20", train(3));
        let mut c = cfg(
            &mut r,
            &[
                "--goal-period-ms",
                "25.6",
                "--deadman-ms",
                "0",
                "--no-center",
                "--mode",
                "position",
                "--schedule",
                &schedule,
            ],
        );
        r.sent();
        let (seen, _, res) = drive(&mut r, &mut c);
        res.unwrap();
        assert_eq!(seen.len(), 4);
        let sent = r.sent();
        let poll = Read(POLL_BASE, POLL_LEN);
        let drops = Read(telemetry::TEL_DROP_COUNT.addr, 2);
        let goal = Hold(control::GOAL_POSITION.addr);
        let arm = Hold(control::CHUNK0_ADDR.addr);
        let first = sent.iter().position(|s| *s == goal).unwrap();
        let want = [
            // the first goal: the poll before it, the drop counter's first read
            [poll, drops, goal, arm, Commit, drops].as_slice(),
            &[goal, arm, poll, Commit, drops],
            &[goal, arm, poll, Commit, drops, poll],
            // the hold: its own polls, the arm write its carrier
            &[poll, Write(control::CHUNK0_ADDR.addr), drops, poll],
        ]
        .concat();
        assert_eq!(sent[first - 2..first - 2 + want.len()], want);
    }

    /// Inside a train the watchdog still judges every burst, at the poll
    /// behind the next goal's staged writes: a fault the servo latched in
    /// a goal's burst turns torque off ahead of the next goal's COMMIT, so
    /// that goal lands on a servo that no longer drives and nothing stays
    /// staged; the run aborts at that goal with its rows, the coast, kept,
    /// and the goal after it never goes out.
    #[test]
    fn a_fault_inside_a_train_turns_torque_off_ahead_of_the_next_commit() {
        use crate::rig::testkit::Sent::{Commit, Hold, Read, Write};
        let mut r = rig_at(2029, 0);
        let schedule = train(4);
        let mut c = cfg(
            &mut r,
            &[
                "--goal-period-ms",
                "25.6",
                "--mode",
                "position",
                "--schedule",
                &schedule,
            ],
        );
        r.after_burst = Some(Box::new(|c, k| {
            if k == 1 {
                c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
                    t.telemetry.common.fault_flags = 1 << 2;
                    t.telemetry.mode.fault_code = 3;
                });
            }
        }));
        r.sent();
        let (seen, _, res) = drive(&mut r, &mut c);
        let e = res.unwrap_err();
        assert_eq!(exit_code(&e), Some(3), "{e}");
        assert_eq!(e.downcast_ref::<Abort>().unwrap().seg, Some(2), "{e}");
        assert!(e.to_string().contains("code 3 (stall)"), "{e}");
        assert_eq!(
            seen[..],
            [
                (0, "goal", 2100, 400),
                (1, "goal", 2101, 400),
                (2, "goal", 2102, 400)
            ]
        );
        let sent = r.sent();
        let last = sent.iter().rposition(|s| *s == Commit).unwrap();
        assert_eq!(
            sent[last - 4..=last],
            [
                Hold(control::GOAL_POSITION.addr),
                Hold(control::CHUNK0_ADDR.addr),
                Read(POLL_BASE, POLL_LEN),
                Write(control::TORQUE_ENABLE.addr),
                Commit
            ]
        );
        assert_eq!(sent.iter().filter(|s| **s == Commit).count(), 3);
        let k = r
            .writes
            .iter()
            .position(|w| *w == (control::GOAL_POSITION, 2102))
            .unwrap();
        assert_eq!(r.writes[k + 1], (control::TORQUE_ENABLE, 0));
        assert!(!r.writes[k..].contains(&(control::TORQUE_ENABLE, 1)));
        assert!(!r.writes.contains(&(control::GOAL_POSITION, 2103)));
        // nothing is left staged: a COMMIT changes nothing
        r.c.commit().unwrap();
        assert_eq!(reg_at(&mut r, control::TEL_COUNT), 0);
        assert_eq!(reg_at(&mut r, control::GOAL_POSITION), 2102);
        assert_eq!(reg_at(&mut r, control::TORQUE_ENABLE) & 0xff, 0);
    }

    /// The default end moves to the middle in Position mode, the goal ahead
    /// of the mode change, then torque off; `--no-center` writes no goal.
    #[test]
    fn the_end_centers_unless_asked_not_to() {
        let mut r = rig_at(2029, 0);
        let mut c = cfg(&mut r, &["--mode", "velocity", "--schedule", "goal:300@20"]);
        drive(&mut r, &mut c).2.unwrap();
        let k = r
            .writes
            .iter()
            .position(|w| *w == (control::GOAL_POSITION, 2029))
            .unwrap();
        assert_eq!(r.writes[k + 1], (control::MODE, 3));
        let off = r
            .writes
            .iter()
            .position(|w| *w == (control::TORQUE_ENABLE, 0))
            .unwrap();
        assert!(off > k + 1);

        let mut r = rig_at(2029, 0);
        let mut c = cfg(
            &mut r,
            &[
                "--no-center",
                "--mode",
                "velocity",
                "--schedule",
                "goal:300@20",
            ],
        );
        drive(&mut r, &mut c).2.unwrap();
        assert!(!r.writes.iter().any(|w| w.0 == control::GOAL_POSITION));
        let k = r
            .writes
            .iter()
            .position(|w| *w == (control::GOAL_VELOCITY, 300))
            .unwrap();
        assert_eq!(r.writes[k + 1], (control::TORQUE_ENABLE, 0));
    }

    /// A `set:` item is written on its own ahead of the goal that follows,
    /// and the meta records the set the run ended on.
    #[test]
    fn a_set_item_commits_before_the_step() {
        let mut r = rig_at(2029, 0);
        let mut c = cfg(
            &mut r,
            &[
                "--mode",
                "position",
                "--schedule",
                "set:velocity_limit_cps=777,goal:2100@20",
            ],
        );
        let (_, ran, res) = drive(&mut r, &mut c);
        res.unwrap();
        let k = r.writes.iter().position(|w| w.0.addr == 0x44).unwrap();
        assert_eq!(
            r.writes[k],
            (
                Reg {
                    addr: 0x44,
                    width: 2
                },
                777
            )
        );
        assert!(r.writes[k + 1..].contains(&(control::GOAL_POSITION, 2100)));
        let set = ran.set.unwrap();
        assert_eq!(set["config"]["velocity_limit_cps"], 777);
        assert_eq!(set["control"]["mode"], "Position");
        assert_eq!(set["control"]["torque_enable"], true);
    }

    fn reg_at(r: &mut Rig, reg: Reg) -> i32 {
        let id = r.id();
        read_u16(&mut r.c, id, reg).unwrap() as i32
    }

    /// Every CONFIG field a `set:` wrote goes back to its value before the
    /// run, last-set first, once torque is off and the guard's safe writes
    /// are out, never before; a field set twice goes back once, to the
    /// value before its first set; a `set:` goal is not put back.
    #[test]
    fn set_fields_go_back_after_torque_off() {
        const VEL: Reg = Reg {
            addr: 0x44,
            width: 2,
        };
        let mut r = rig_at(2029, 0);
        let (vel0, band0) = (
            reg_at(&mut r, VEL),
            reg_at(&mut r, config::POS_DEADBAND_COUNTS),
        );
        let mut c = cfg(
            &mut r,
            &[
                "--mode",
                "position",
                "--schedule",
                "set:velocity_limit_cps=777,set:pos_deadband_counts=9,goal:2100@20,\
                 set:velocity_limit_cps=555,set:goal_current=100,hold@20",
            ],
        );
        let (_, ran, res) = drive(&mut r, &mut c);
        res.unwrap();
        let n = r.writes.len();
        let safe = crate::rig::servo::SAFE;
        assert_eq!(r.writes[n - 2 - safe.len()..n - 2], safe);
        assert_eq!(
            r.writes[n - 2..],
            [(VEL, vel0), (config::POS_DEADBAND_COUNTS, band0)]
        );
        assert_eq!(r.writes.iter().filter(|w| **w == (VEL, vel0)).count(), 1);
        assert!(r.writes.contains(&(control::GOAL_CURRENT, 100)));
        assert_ne!((vel0, band0), (555, 9), "the test would prove nothing");
        assert_eq!(
            (
                reg_at(&mut r, VEL),
                reg_at(&mut r, config::POS_DEADBAND_COUNTS)
            ),
            (vel0, band0)
        );
        let restored: Vec<_> = ran
            .restored
            .iter()
            .map(|k| (k.field.as_str(), k.shown(k.last), k.shown(k.before)))
            .collect();
        assert_eq!(
            restored,
            [
                ("velocity_limit_cps", 555, vel0 as i64),
                ("pos_deadband_counts", 9, band0 as i64)
            ]
        );
        assert!(ran.data_flags_after.is_some());
        assert_eq!(
            ran.set.unwrap()["config"]["velocity_limit_cps"],
            555,
            "the set the run ran on"
        );
    }

    /// An abort puts the fields back too, after its torque off, and drives
    /// nothing on the way.
    #[test]
    fn an_abort_puts_the_fields_back_after_torque_off() {
        const VEL: Reg = Reg {
            addr: 0x44,
            width: 2,
        };
        let mut r = rig_at(2029, 0);
        let vel0 = reg_at(&mut r, VEL);
        let mut c = cfg(
            &mut r,
            &[
                "--mode",
                "position",
                "--schedule",
                "set:velocity_limit_cps=777,goal:2100@20,hold@20",
            ],
        );
        r.after_burst = Some(Box::new(|c, _| at(c, 3700, 0, 1913)));
        let (_, ran, res) = drive(&mut r, &mut c);
        assert_eq!(exit_code(&res.unwrap_err()), Some(3));
        let off = r
            .writes
            .iter()
            .position(|w| *w == (control::TORQUE_ENABLE, 0))
            .unwrap();
        let back = r.writes.iter().position(|w| *w == (VEL, vel0)).unwrap();
        assert!(back > off, "put back before torque off");
        assert!(!r.writes[off..].contains(&(control::TORQUE_ENABLE, 1)));
        assert!(!r.writes[off..].iter().any(|w| w.0 == control::MODE));
        assert_eq!(reg_at(&mut r, VEL), vel0);
        assert_eq!(ran.restored.len(), 1);
    }

    /// meta.json keeps the set the run ran on and names what went back.
    #[test]
    fn meta_names_what_went_back() {
        let out = std::env::temp_dir().join(format!("osc-lab-track-back-{}", std::process::id()));
        let mut r = rig_at(2029, 0);
        let vel0 = reg_at(
            &mut r,
            Reg {
                addr: 0x44,
                width: 2,
            },
        );
        let a = parse(&[
            "--mode",
            "position",
            "--schedule",
            "set:velocity_limit_cps=777,hold@10",
        ]);
        capture(&mut r, &a, &out).unwrap();
        let meta: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(out.join("meta.json")).unwrap()).unwrap();
        assert_eq!(meta["config"]["velocity_limit_cps"], 777);
        assert_eq!(
            meta["restored"],
            serde_json::json!([{ "field": "velocity_limit_cps", "from": 777, "to": vel0 }])
        );
        assert!(meta["data_flags_after"].is_array());
        std::fs::remove_dir_all(&out).unwrap();
    }

    /// The files: track.csv a row a tick under sweep.csv's names, the goal
    /// in force on each; segments.csv a row an item; meta.json the run's
    /// constants and how it ended, on a clean run and an aborted one.
    #[test]
    fn the_files_carry_every_row_item_and_ending() {
        let out = std::env::temp_dir().join(format!("osc-lab-track-{}", std::process::id()));
        let mut r = rig_at(2029, 0);
        r.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.mode.omega_hat_src = 1;
            t.telemetry.refs.current_bias_counts = 2047;
        });
        let a = parse(&[
            "--mode",
            "position",
            "--schedule",
            "goal:2100@20,rest@100,hold@10",
        ]);
        capture(&mut r, &a, &out).unwrap();
        let csv = std::fs::read_to_string(out.join("track.csv")).unwrap();
        let mut lines = csv.lines();
        assert_eq!(lines.next(), Some(CSV_HEADER));
        let rows: Vec<Vec<&str>> = lines.map(|l| l.split(',').collect()).collect();
        assert_eq!(rows.len(), 600);
        assert!(rows.iter().all(|c| c.len() == 16 && c[1] == "2100"));
        assert_eq!((rows[0][0], rows[599][0]), ("0", "2"));
        assert!(rows.iter().all(|c| c[4] == "2029" && c[5].is_empty()));
        let segs = std::fs::read_to_string(out.join("segments.csv")).unwrap();
        let segs: Vec<&str> = segs.lines().collect();
        assert_eq!(segs[0], SEGMENTS_HEADER);
        assert_eq!(segs.len(), 4);
        let cols: Vec<&str> = segs[1].split(',').collect();
        assert_eq!(cols.len(), 26);
        assert_eq!(cols[..5], ["0", "goal", "", "2100", "20"]);
        assert_eq!(cols[6..11], ["", "", "", "400", "0"], "no period, no grid");
        assert_eq!(cols[21..], ["2029", "0", "2047", "0", "1"]);
        assert!(segs[2].starts_with("1,rest,,,100,,,,,0,0,"));
        let meta: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(out.join("meta.json")).unwrap()).unwrap();
        for k in [
            "fw",
            "model",
            "plant",
            "sense",
            "tick_hz",
            "pwm_period_ticks",
            "vbus_counts",
            "tel_chunks",
            "rows_dropped",
            "mode",
            "schedule",
            "guard",
            "watchdog",
            "deadman",
            "center",
            "goal_period",
            "config",
            "control",
            "restored",
            "data_flags_after",
            "ended",
        ] {
            assert!(meta.get(k).is_some(), "meta has no {k}");
        }
        assert_eq!(meta["ended"], "ok");
        assert_eq!(meta["restored"], serde_json::json!([]));
        assert_eq!(meta["mode"], "position");
        assert_eq!(
            meta["schedule"],
            serde_json::json!(["goal:2100@20", "rest@100", "hold@10"])
        );
        assert_eq!(
            meta["tel_chunks"],
            serde_json::json!([[0x238, 2], [0x240, 10]])
        );
        assert_eq!(meta["deadman"]["ticks"], 20_000);
        assert_eq!(
            meta["goal_period"],
            serde_json::json!({ "ms": null, "held": 0, "late": 0 })
        );
        assert_eq!(meta["config"]["pos_min_soft_counts"], 432);
        assert_eq!(
            meta["config"].as_object().unwrap().len(),
            crate::descriptor::builtin()
                .fields
                .iter()
                .filter(|f| f.end() <= CONFIG_REGION_SIZE)
                .count()
        );

        let mut r = rig_at(2029, 0);
        r.after_burst = Some(Box::new(|c, _| at(c, 3700, 0, 1913)));
        let a = parse(&["--mode", "position", "--schedule", "goal:2100@20,hold@10"]);
        let e = capture(&mut r, &a, &out).unwrap_err();
        assert_eq!(exit_code(&e), Some(3));
        let meta: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(out.join("meta.json")).unwrap()).unwrap();
        assert_eq!(meta["ended"]["segment"], 0);
        assert!(
            meta["ended"]["reason"]
                .as_str()
                .unwrap()
                .contains("outside the guard")
        );
        let rows = std::fs::read_to_string(out.join("track.csv"))
            .unwrap()
            .lines()
            .count();
        assert_eq!(rows, 1 + 400, "what was captured before the abort");
        std::fs::remove_dir_all(&out).unwrap();
    }
}
