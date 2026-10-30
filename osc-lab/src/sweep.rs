//! `osc-lab sweep` - raw open-loop duty sweep for empirical plant capture:
//! per direction x per duty rung, seek to a start band by polling, brake and
//! settle so the rung opens on a still shaft, then one
//! goal+arm COMMIT captures a TEL burst at full tick rate - no mid-rung
//! polling (the firmware soft-limit duty clamp, current limit, and faults
//! guard the silent window). A torque-off baseline burst runs first. Rows
//! land in sweep.csv, the run's constants in meta.json, both directly in
//! --out (no timestamp subdir).
//!
//! `coast:MS` / `brake:MS` schedule entries capture a zero-duty segment:
//! the preceding drive step feeds it without a settle (momentum carries
//! across the burst gap), and a brake segment runs with openloop_zero_brake
//! set (cleared again right after, so coast segments always see it clear).
//! `then:PCT` is a drive step chained the same way (no seek before it): a
//! reversal pair like `20,then:-20` captures gear play with no reposition
//! between the two drives. `hold:PCT@MS` is a chained drive held
//! unstreamed: the duty stays on with no burst, the lease fed and the faults
//! read through it, so a long drive costs no rows - a heating run streams
//! short windows between holds instead of minutes at the full tick rate.
//!
//! Every segment's host clock lands in segments.csv beside sweep.csv, the
//! unix seconds around its burst or its hold, so a recording's segments sit
//! on one timeline: within a burst `tick` is the clock, across bursts
//! nothing else is.
//!
//! Duty sign is taken as-is (fwd = +duty): the kernel applies
//! `drive_polarity` at the motor output, so +duty moves counts up on a
//! calibrated servo. The seek's distance-growing bail stays as the guard for
//! a servo that was never calibrated.
//!
//! `--stall` seeks the physical end stop instead of a start band and leaves
//! the shaft there, so the rungs push into it: a locked-output ladder for
//! resistance and inductance, with no back-EMF in the onset. It writes the
//! control table's `stall_permit` after every torque enable, rewrites it
//! while it drives (the firmware grants it as a one-second lease), and
//! clears it after. Without that the kernel zeroes the outbound duty at the
//! wall and trips the stall timer, and no soft-limit value avoids it - a stop
//! can sit AT the position rail, which is where both of this servo's are. A
//! rung window longer than the permit can be held without a rewrite is
//! refused.
//!
//! Seeks drive at the duty whose stall the servo's current limit holds
//! ([`DutyPlan`]), by its winding R and the rail - by the class's lowest R
//! before one is stored - and so does every rung `--stall` presses into a
//! stop: a duty over it is refused before anything moves. Free-running
//! rungs are not capped: the firmware limiter governs them.

use std::io::Write;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;
use osc_protocol::tel::Chunks;

use crate::descriptor;
use crate::instrument::frame::TelFrame;
use crate::instrument::limits::{DutyPlan, POT_MAX, ServoLimits, pct_floor};
use crate::instrument::regs::{Reg, calib, control};
use crate::instrument::runway::{BRAKE_DUTY_Q15, BRAKE_POLL_MS, BRAKE_POLLS, BRAKE_REST_EPS};
use crate::instrument::seek::{
    self, SEEK_STEP_Q15, SEEK_TRAVEL_MIN, STALL_EPS, STALL_POLLS, Watch,
};
use crate::rig::Bus;
use crate::rig::limits::Stall;
use crate::rig::park::park;
use crate::rig::plant::Snapshot;
use crate::rig::pump::{self, BurstStats, Lease, STOP, read_snapshot};
use crate::rig::servo::{Servo, Wire, guard};
use crate::rig::snapshot::read_u16;
use crate::rig::{Aborted, blocked};

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub(crate) enum Dirs {
    Both,
    Fwd,
    Rev,
}

impl Dirs {
    pub(crate) fn signs(self) -> &'static [i8] {
        match self {
            Dirs::Both => &[1, -1],
            Dirs::Fwd => &[1],
            Dirs::Rev => &[-1],
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub(crate) enum Decay {
    Slow,
    Fast,
}

impl Decay {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Decay::Slow => "slow",
            Decay::Fast => "fast",
        }
    }
}

/// One schedule entry: a drive rung, a chained drive, or a zero-duty
/// coast/brake segment. Drive and Then carry an optional per-step window in
/// ms, overriding `--window-ms`. A rung is one unpolled burst that nothing can
/// cut short, so its window is the ONLY thing bounding how far the shaft
/// travels: a slow rung needs a long one to cross the same span a fast rung
/// crosses in 150 ms, and a fast rung given the slow one's window drives into
/// the end stop.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Step {
    Drive(u8, Option<u32>),
    Then(i8, Option<u32>),
    Coast(u32),
    Brake(u32),
    Hold(i8, u32),
}

impl std::fmt::Display for Step {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Step::Drive(pct, None) => write!(f, "{pct}"),
            Step::Drive(pct, Some(ms)) => write!(f, "{pct}@{ms}"),
            Step::Then(pct, None) => write!(f, "then:{pct}"),
            Step::Then(pct, Some(ms)) => write!(f, "then:{pct}@{ms}"),
            Step::Coast(ms) => write!(f, "coast:{ms}"),
            Step::Brake(ms) => write!(f, "brake:{ms}"),
            Step::Hold(pct, ms) => write!(f, "hold:{pct}@{ms}"),
        }
    }
}

impl std::str::FromStr for Step {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        parse_step(s)
    }
}

pub(crate) fn parse_step(s: &str) -> Result<Step, String> {
    let ms = |v: &str| v.parse().map_err(|_| format!("bad ms {v:?}"));
    if let Some(v) = s.strip_prefix("coast:") {
        return Ok(Step::Coast(ms(v)?));
    }
    if let Some(v) = s.strip_prefix("brake:") {
        return Ok(Step::Brake(ms(v)?));
    }
    // `PCT@MS` splits the window off first so both drive forms share it.
    let (head, window) = match s.split_once('@') {
        Some((h, w)) => {
            let w: u32 = ms(w)?;
            if w == 0 {
                return Err("step window must be nonzero".into());
            }
            (h, Some(w))
        }
        None => (s, None),
    };
    if let Some(v) = head.strip_prefix("hold:") {
        let pct: i8 = v.parse().map_err(|_| format!("bad pct {v:?}"))?;
        if pct.unsigned_abs() > 100 {
            return Err(format!("hold: pct must be within +/-100, got {pct}"));
        }
        let ms = window.ok_or("hold: needs its span as @MS")?;
        return Ok(Step::Hold(pct, ms));
    }
    if let Some(v) = head.strip_prefix("then:") {
        let pct: i8 = v.parse().map_err(|_| format!("bad pct {v:?}"))?;
        if pct == 0 || pct.unsigned_abs() > 100 {
            return Err(format!(
                "then: pct must be nonzero within +/-100, got {pct}"
            ));
        }
        return Ok(Step::Then(pct, window));
    }
    head.parse()
        .map(|pct| Step::Drive(pct, window))
        .map_err(|_| {
            format!(
                "step is duty pct, then:PCT, coast:MS, brake:MS or hold:PCT@MS, the drives optionally @MS, got {s:?}"
            )
        })
}

/// Whether step k feeds step k+1 with no settle between them: then/coast/
/// brake/hold steps chain to their predecessor so momentum carries across
/// the burst gap.
pub(crate) fn feeds(steps: &[Step], k: usize) -> bool {
    matches!(
        steps.get(k + 1),
        Some(Step::Then(..) | Step::Coast(_) | Step::Brake(_) | Step::Hold(..))
    )
}

/// `osc-lab sweep` args. `--baud`/`--id` come from the top-level globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir; sweep.csv + meta.json land here directly.
    #[arg(long, default_value = "./sweep-out")]
    out: PathBuf,
    /// Duty grid, percent of full scale; `coast:MS` / `brake:MS` entries
    /// capture a zero-duty segment chained to the preceding step, `then:PCT`
    /// a signed drive chained the same way (no seek), `hold:PCT@MS` a signed
    /// drive chained the same way and held unstreamed.
    #[arg(long, value_delimiter = ',', value_parser = parse_step,
          default_values_t = (1..=20u8).map(|k| Step::Drive(k * 5, None)))]
    duty_pct: Vec<Step>,
    #[arg(long, value_enum, default_value_t = Dirs::Both)]
    dirs: Dirs,
    /// openloop_decay for the capture bursts; seeks and brakes stay slow.
    #[arg(long, value_enum, default_value_t = Decay::Slow)]
    decay: Decay,
    /// Capture per rung; samples = window_ms x 20 ticks.
    #[arg(long, default_value_t = 150)]
    window_ms: u32,
    /// Torque-off cool-down between rungs.
    #[arg(long, default_value_t = 500)]
    rest_ms: u32,
    /// Torque-off noise-floor capture before the grid; 0 skips it.
    #[arg(long, default_value_t = 1000)]
    baseline_ms: u32,
    /// Seek drive, percent of full scale [default: the duty whose stall
    /// draws the servo's current limit, by its winding R and the rail]; a
    /// seek whose stall draws more is refused.
    #[arg(long)]
    seek_duty_pct: Option<u8>,
    /// Ceiling for the breakout escalation off a stop, percent of full
    /// scale [default: the duty whose stall draws the servo's current
    /// limit]; a ceiling whose stall draws more is refused. Breakout loads
    /// the teeth like a stall does.
    #[arg(long)]
    seek_cap_pct: Option<u8>,
    /// Seek the physical end stop and ladder against it instead of seeking a
    /// start band. Chain the rungs (`20,then:25,...`) to hold the gear train
    /// wound up across the whole ladder: released, it unwinds and the next
    /// onset measures the re-wind instead of the winding. A rung whose stall
    /// draws more than the servo's current limit is refused.
    #[arg(long)]
    stall: bool,
    /// The load is a resistor, not a motor: no shaft to seek, brake or
    /// settle, so every seek is skipped and the rungs drive directly.
    /// Implies the stall permit - the rungs draw current with no motion,
    /// which is exactly what the stall trip exists to catch.
    #[arg(long)]
    static_load: bool,
    /// Brake-and-hold after the seek, before the rung arms. Without it the
    /// rung opens on a shaft still coasting from the seek, and since a fwd
    /// rung's start band is at the LOW guard the coast is BACKWARD: measured
    /// -0.70 V of entry EMF, which lands in the intercept of any onset fit.
    /// Settled-window analysis does not care; onset R and L do.
    #[arg(long, default_value_t = 300)]
    settle_ms: u32,
    /// Seek envelope only - the captures themselves run unpolled. Low end,
    /// counts [default: the servo's low soft limit, 100 counts in].
    #[arg(long)]
    guard_lo: Option<u16>,
    /// High end, counts [default: the servo's high soft limit, 100 counts
    /// in].
    #[arg(long)]
    guard_hi: Option<u16>,
    /// TEL row: table chunks as addr+len, at most two, comma-joined (hex
    /// ok). Default = the raw six: pos, raw current, vmotor_a, vmotor_b,
    /// applied duty, trough - measurements only, kernel conclusions (vdiff,
    /// vbus, the held current) stay out of raw captures.
    #[arg(long, default_value = "0x238+2,0x240+10")]
    tel_chunks: String,
    /// Attempts per rung before the sweep gives up. A rung is captured as one
    /// burst, so a dropped frame anywhere in it corrupts that rung alone -
    /// retrying just the rung costs seconds where discarding the recording
    /// costs a minute and a half of good rungs with it.
    #[arg(long, default_value_t = 3)]
    rung_tries: u32,
}

/// What one sweep run drives: `Args` minus the output dir, chunks parsed.
pub(crate) struct Cfg {
    pub(crate) steps: Vec<Step>,
    pub(crate) dirs: Dirs,
    pub(crate) decay: Decay,
    pub(crate) window_ms: u32,
    pub(crate) rest_ms: u32,
    pub(crate) baseline_ms: u32,
    pub(crate) seek_duty_pct: u8,
    pub(crate) seek_cap_pct: u8,
    pub(crate) settle_ms: u32,
    pub(crate) stall: bool,
    pub(crate) static_load: bool,
    pub(crate) guard: (u16, u16),
    /// The pot stops, where a stop seek must come to rest.
    pub(crate) stops: Option<(u16, u16)>,
    pub(crate) tel_chunks: Chunks,
    pub(crate) rung_tries: u32,
}

impl Cfg {
    /// The run as asked, the guard filled in from the servo's soft limits
    /// and the seek from `plan`. A resistor has no travel, so a static load
    /// takes any limits and its guard spans the pot. Every duty that can
    /// stall the shaft - the seek, its breakout ceiling, a rung pressed
    /// into a stop - is refused when its stall draws more than the limit.
    fn new(a: &Args, lim: &ServoLimits, plan: &DutyPlan) -> Result<Self> {
        let guard = if a.static_load {
            (
                a.guard_lo.unwrap_or(0),
                a.guard_hi.unwrap_or(POT_MAX as u16),
            )
        } else {
            lim.envelope((a.guard_lo, a.guard_hi))?
        };
        let seek_duty_pct = a.seek_duty_pct.unwrap_or(pct_floor(plan.seek));
        let seek_cap_pct = a.seek_cap_pct.unwrap_or(pct_floor(plan.stop_cap));
        if !a.static_load {
            let stall = |what: &str, pct: u8| lim.check_stall_at(what, pct_q15(pct), plan.r_vpc);
            stall("the seek", seek_duty_pct)?;
            stall("a seek raised to its ceiling", seek_cap_pct)?;
            for s in a.duty_pct.iter().filter(|_| a.stall) {
                let pct = match s {
                    Step::Drive(pct, _) => *pct,
                    Step::Then(pct, _) | Step::Hold(pct, _) => pct.unsigned_abs(),
                    Step::Coast(_) | Step::Brake(_) => continue,
                };
                stall("a rung pressed into the stop", pct)?;
            }
        }
        Ok(Self {
            steps: a.duty_pct.clone(),
            dirs: a.dirs,
            decay: a.decay,
            window_ms: a.window_ms,
            rest_ms: a.rest_ms,
            baseline_ms: a.baseline_ms,
            seek_duty_pct,
            seek_cap_pct,
            settle_ms: a.settle_ms,
            stall: a.stall,
            static_load: a.static_load,
            guard,
            stops: (!a.static_load).then_some(lim.raw),
            tel_chunks: pump::parse_tel_chunks(&a.tel_chunks).context("--tel-chunks")?,
            rung_tries: a.rung_tries,
        })
    }
}

/// One committed segment: the baseline (seg 0) or one schedule step of one
/// direction, after its chain passed clean.
pub(crate) struct Segment {
    pub(crate) seg: u32,
    pub(crate) dir: i8,
    pub(crate) cmd_duty_q15: i16,
    pub(crate) frames: Vec<TelFrame>,
    pub(crate) stats: BurstStats,
    /// Host unix seconds around the burst or the hold.
    pub(crate) t_start: f64,
    pub(crate) t_end: f64,
}

/// One rung's start band: a narrow window CENTRED on the launch guard, so a
/// rung starts at `guard_lo` (fwd) or `guard_hi` (rev) give or take
/// `BAND_HALF`. Centring matters because the seek stops the instant it
/// enters the band and it does not always arrive from mid travel - a rung
/// that ends against a rail leaves the next seek approaching from the far
/// side, and a band offset to one side of the guard then starts the rung a
/// full band-width from where it was asked to.
pub(crate) const BAND_HALF: u16 = 75;

fn start_band(dir: i8, guard_lo: u16, guard_hi: u16) -> (u16, u16) {
    let c = if dir > 0 { guard_lo } else { guard_hi };
    (c.saturating_sub(BAND_HALF), c.saturating_add(BAND_HALF))
}

/// Why a rung is being replayed. Deliberately avoids the words the committed
/// segment line uses: callers scrape sweep stdout to decide a whole recording
/// failed (they grep `[1-9][0-9]* seq holes` and `[1-9][0-9]* garble`), so a
/// note about a rung we RECOVERED must not read as a failed capture. Both
/// halves matter - quoting only the counts still trips the garble half via
/// `holes=2 garble=0`. `retry_note_cannot_read_as_failure` pins it.
fn retry_note(
    seg: u32,
    what: &str,
    holes: impl std::fmt::Display,
    garble: impl std::fmt::Display,
) -> String {
    format!("  seg {seg} ({what}) [h={holes} g={garble}]")
}

pub(crate) fn pct_q15(pct: u8) -> i16 {
    (pct as i32 * 32767 / 100) as i16
}

/// Rows of a `ms` burst on a servo ticking at `tick_hz`.
pub(crate) fn samples_of_ms(ms: u32, tick_hz: f64) -> u32 {
    (ms as f64 * tick_hz / 1000.0).round() as u32
}

/// A seek's poll cadence, and its bound: 500 polls, 10 s.
const SEEK_POLL_MS: u32 = 20;
const SEEK_POLLS: u32 = 500;

/// Zero the duty and wait for the shaft to rest by the seek's stillness
/// rule ([`Watch`]): a seek judges its direction from the band distance,
/// and a shaft still coasting from the drive before reads as its own.
/// Err when it never rests within the seek's bound.
fn rest_before_seek<S: Servo>(s: &mut S, lease: &mut Lease) -> Result<()> {
    s.write(control::GOAL_DUTY, 0)?;
    let mut pos = check_fault(s)?;
    let mut watch = Watch::new(pos, STALL_EPS, STALL_POLLS);
    for _ in 0..SEEK_POLLS {
        check_stop()?;
        lease.keep(s)?;
        s.sleep(SEEK_POLL_MS);
        pos = check_fault(s)?;
        if watch.still(pos) {
            return Ok(());
        }
    }
    bail!(
        "the shaft never came to rest in {} s with the duty at 0 (pos {pos}): a seek cannot \
         tell its own travel from a coast",
        SEEK_POLLS * SEEK_POLL_MS / 1000
    )
}

fn check_stop() -> Result<()> {
    if STOP.load(std::sync::atomic::Ordering::SeqCst) {
        bail!("interrupted");
    }
    Ok(())
}

fn check_fault<S: Servo>(s: &mut S) -> Result<u16> {
    let o = s.snapshot()?;
    if o.fault_flags != 0 {
        bail!(
            "servo faulted: flags {:#04x} code {}",
            o.fault_flags,
            o.fault_code
        );
    }
    Ok(o.pos)
}

/// Open-loop bang-bang seek into [lo, hi], 20 ms cadence, ctrl-c aware,
/// from a shaft at rest ([`rest_before_seek`]). Bails on a fault or on the
/// band distance growing (reversed polarity); aborts on a shaft that comes
/// to rest short of the band; leaves duty 0 and torque ON (the rung drives
/// next).
fn seek_band<S: Servo>(
    s: &mut S,
    lease: &mut Lease,
    (lo, hi): (u16, u16),
    duty_q15: i16,
    cap: i32,
    stops: Option<(u16, u16)>,
) -> Result<()> {
    rest_before_seek(s, lease)?;
    s.write(control::MODE, 0)?;
    lease.torque_on(s)?;
    // Distance to the band, not envelope membership: a seek may legally
    // START at a rail (that is what it is for); reversed polarity shows as
    // the distance GROWING while driving.
    let dist = |pos: u16| (lo.saturating_sub(pos)) as u32 + (pos.saturating_sub(hi)) as u32;
    let mut best = u32::MAX;
    // Leaving a stop costs more than arriving at one, and this seek starts
    // wherever the last rung parked - often hard against a rail. Escalate
    // when nothing moves off a stop: it drives TOWARD the middle, so there is
    // nothing to hit. A shaft at rest anywhere else is blocked.
    let mut mag = duty_q15 as i32;
    let mut watch: Option<Watch> = None;
    for _ in 0..SEEK_POLLS {
        check_stop()?;
        lease.keep(s)?;
        let pos = check_fault(s)?;
        if (lo..=hi).contains(&pos) {
            s.write(control::GOAL_DUTY, 0)?;
            return Ok(());
        }
        let d = dist(pos);
        best = best.min(d);
        if d > best + 200 {
            s.write(control::GOAL_DUTY, 0)?;
            bail!("seek moving away from {lo}..{hi} at pos {pos} (reversed polarity?)");
        }
        let dir: i8 = if pos < lo { 1 } else { -1 };
        let w = watch.get_or_insert(Watch::new(pos, STALL_EPS, STALL_POLLS));
        if w.still(pos) {
            let start = w.start();
            let next = mag + SEEK_STEP_Q15 as i32;
            if pos.abs_diff(start) >= SEEK_TRAVEL_MIN
                || !seek::leaves_stop(start, dir, stops)
                || next > cap
            {
                s.write(control::GOAL_DUTY, 0)?;
                return Err(Aborted {
                    what: "the seek",
                    reason: seek::blocked(start, pos),
                }
                .into());
            }
            mag = next;
        }
        s.write(control::GOAL_DUTY, dir as i32 * mag)?;
        s.sleep(SEEK_POLL_MS);
    }
    s.write(control::GOAL_DUTY, 0)?;
    bail!("seek did not reach {lo}..{hi} in 10 s (gear slipping?)")
}

/// Drive into the physical end stop and leave the shaft pressed against it.
/// The rungs push the same way, so the train's backlash is taken up before
/// the first burst instead of during it.
///
/// A soft position limit is indistinguishable from a mechanical stop by
/// position alone - the kernel zeroes outbound open-loop duty when the
/// endstop band collapses, so the shaft stops either way. DUTY_APPLIED_Q15
/// tells them apart: it holds at a real stop and reads zero at a soft limit.
/// Without that check a run would ladder against a clamped duty and record a
/// grid of zero-current rungs.
///
/// A shaft at rest anywhere but the stop is blocked ([`seek::at_stop`]):
/// standing still is what "arrived", "stuck against the far stop" and
/// "jammed mid travel" all look like, and reading either of the last two
/// as the first runs the whole ladder against the wrong thing.
fn seek_stop<S: Servo>(
    s: &mut S,
    lease: &mut Lease,
    dir: i8,
    duty_q15: i16,
    cap: i32,
    stops: Option<(u16, u16)>,
) -> Result<()> {
    s.write(control::MODE, 0)?;
    lease.torque_on(s)?;
    let mut duty = duty_q15 as i32;
    let start = check_fault(s)?;
    let mut watch = Watch::new(start, STALL_EPS, STALL_POLLS);
    for _ in 0..SEEK_POLLS {
        check_stop()?;
        lease.keep(s)?;
        s.write(control::GOAL_DUTY, dir as i32 * duty)?;
        s.sleep(SEEK_POLL_MS);
        let pos = check_fault(s)?;
        if !watch.still(pos) {
            continue;
        }
        if seek::at_stop(start, pos, dir, stops).is_ok() {
            // Duty stays on: releasing lets the train unwind, and the burst
            // would then re-wind it inside the measurement.
            return Ok(());
        }
        // Leaving a stop costs more than holding against one: 20% of full
        // scale draws current without moving the shaft at all where 40%
        // frees it. Step up rather than guess a constant - off a stop only.
        let next = duty + SEEK_STEP_Q15 as i32;
        if pos.abs_diff(start) >= SEEK_TRAVEL_MIN
            || !seek::leaves_stop(start, dir, stops)
            || next > cap
        {
            s.write(control::GOAL_DUTY, 0)?;
            return Err(Aborted {
                what: "the stop seek",
                reason: seek::blocked(start, pos),
            }
            .into());
        }
        duty = next;
    }
    s.write(control::GOAL_DUTY, 0)?;
    bail!("no end stop in 10 s driving {dir:+} at {duty} q15")
}

/// Dynamic brake after a rung: with openloop_decay Slow, a token duty holds
/// the idle H-bridge leg HIGH for the rest of each PWM period - the winding
/// shorts through the driver and momentum dies in a few hundred counts.
/// A plain duty-0 write is COAST (motor.rs maps zero ticks to both-low);
/// at battery volts the freewheel from a top rung crosses the remaining
/// runway and slams the physical stop. Retreat sign so the firmware
/// soft-limit clamp can never zero the brake near a wall. Leaves duty 0,
/// torque ON (the caller torques off).
pub(crate) fn brake_to_rest<S: Servo>(s: &mut S, lease: &mut Lease, dir: i8) -> Result<()> {
    s.write(control::GOAL_DUTY, -(dir as i32) * BRAKE_DUTY_Q15 as i32)?;
    let mut last = check_fault(s)?;
    for _ in 0..BRAKE_POLLS {
        check_stop()?;
        lease.keep(s)?;
        s.sleep(BRAKE_POLL_MS);
        let pos = check_fault(s)?;
        if pos.abs_diff(last) < BRAKE_REST_EPS {
            break;
        }
        last = pos;
    }
    s.write(control::GOAL_DUTY, 0)?;
    Ok(())
}

fn rest<S: Servo>(s: &mut S, ms: u32) -> Result<()> {
    let mut left = ms;
    while left > 0 {
        check_stop()?;
        let slice = left.min(20);
        s.sleep(slice);
        left -= slice;
    }
    Ok(())
}

/// A held step's slice: the lease is fed and the faults read this often,
/// since no burst is running to carry either.
const HOLD_SLICE_MS: u32 = 50;

/// Hold the live duty `ms` with nothing streamed.
fn hold<S: Servo>(s: &mut S, lease: &mut Lease, ms: u32) -> Result<()> {
    let mut left = ms;
    while left > 0 {
        check_stop()?;
        lease.keep(s)?;
        check_fault(s)?;
        let slice = left.min(HOLD_SLICE_MS);
        s.sleep(slice);
        left -= slice;
    }
    lease.keep(s)
}

/// The host clock, unix seconds: segments.csv's anchor.
fn unix_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// segments.csv: one row a committed segment, `streamed` 0 for a hold.
pub(crate) const SEGMENTS_HEADER: &str = "seg,dir,cmd_duty_q15,streamed,samples,t_start,t_end";

pub(crate) fn write_times(w: &mut impl Write, g: &Segment) -> Result<()> {
    writeln!(
        w,
        "{},{},{},{},{},{:.3},{:.3}",
        g.seg,
        g.dir,
        g.cmd_duty_q15,
        (g.stats.samples > 0) as u8,
        g.stats.samples,
        g.t_start,
        g.t_end
    )?;
    Ok(())
}

/// `pos_lin` stays an empty column: the stream no longer carries it, and
/// every recording keeps one header.
pub(crate) const CSV_HEADER: &str = "seg,cmd_duty_q15,dir,tick,window_valid,pos,current,current_trough,duty_counts,vdiff,vbus,current_raw,vmotor_a,vmotor_b,vbus_raw,ntc_raw,vref_raw,pos_lin";

pub(crate) fn write_rows(w: &mut impl Write, s: &Segment) -> Result<()> {
    let opt = |v: Option<i32>| v.map(|v| v.to_string()).unwrap_or_default();
    let (seg, cmd_duty_q15, dir) = (s.seg, s.cmd_duty_q15, s.dir);
    for f in &s.frames {
        writeln!(
            w,
            "{seg},{cmd_duty_q15},{dir},{},{},{},{},{},{},{},{},{},{},{},{},{},{},",
            f.tick,
            f.window_valid as u8,
            opt(f.pos.map(|v| v as i32)),
            opt(f.current.map(|v| v as i32)),
            opt(f.current_trough.map(|v| v as i32)),
            opt(f.duty_counts.map(|v| v as i32)),
            opt(f.vdiff.map(|v| v as i32)),
            opt(f.vbus.map(|v| v as i32)),
            opt(f.current_raw.map(|v| v as i32)),
            opt(f.vmotor_a.map(|v| v as i32)),
            opt(f.vmotor_b.map(|v| v as i32)),
            opt(f.vbus_raw.map(|v| v as i32)),
            opt(f.ntc_raw.map(|v| v as i32)),
            opt(f.vref_raw.map(|v| v as i32)),
        )?;
    }
    Ok(())
}

/// The run's constants, written as meta.json before the first burst. The
/// `plant` block names the table that linearizes the recording's `pos`,
/// its stamp verdict and its data state at that moment; `drive` is the
/// rule the drive ran under and the servo settings that held it.
/// `tel_chunks` is the row as `[addr, len]` pairs; recordings from before
/// chunked rows carry `tel_mask` instead.
pub(crate) fn meta<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    cfg: &Cfg,
    drive: serde_json::Value,
) -> Result<serde_json::Value> {
    let servo = servo_meta(c, id)?;
    let mut m = serde_json::json!({
        "schedule": cfg.steps.iter().map(Step::to_string).collect::<Vec<_>>(),
        "decay": cfg.decay.as_str(),
        "dirs": cfg.dirs.signs(),
        "window_ms": cfg.window_ms,
        "rung_tries": cfg.rung_tries,
        "rest_ms": cfg.rest_ms,
        "baseline_ms": cfg.baseline_ms,
        "seek_duty_pct": cfg.seek_duty_pct,
        "seek_cap_pct": cfg.seek_cap_pct,
        "settle_ms": cfg.settle_ms,
        "stall": cfg.stall,
        "static_load": cfg.static_load,
        "guard": [cfg.guard.0, cfg.guard.1],
        "tel_chunks": cfg
            .tel_chunks
            .0
            .iter()
            .filter(|c| c.len != 0)
            .map(|c| [c.addr, c.len])
            .collect::<Vec<_>>(),
        "post_rung_brake": true,
        "drive": drive,
    });
    if let Some(o) = m.as_object_mut() {
        o.extend(servo);
    }
    Ok(m)
}

/// What every recording's meta.json opens with: the servo, its tick rate
/// and rail, the PWM period its duty counts are of, the table it ran
/// (`plant`) and its sense constants.
pub(crate) fn servo_meta<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
) -> Result<serde_json::Map<String, serde_json::Value>> {
    let identity = c.identity(id)?;
    let tick_hz = read_u16(c, id, calib::TICK_HZ)?;
    let pwm_period_ticks = read_u16(c, id, calib::PWM_PERIOD_TICKS)?;
    let tel = read_snapshot(c, id)?;
    let d = crate::state::descriptor(c, id)?;
    let plant = Snapshot::read(c, id, &d)?.json();
    let sense = serde_json::json!({
        "shunt_r_mohm": read_u16(c, id, calib::SHUNT_R_MOHM)?,
        "gain_milli": read_u16(c, id, calib::GAIN_MILLI)?,
        "vmotor_div_top": read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        "vmotor_div_bot": read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        "vdd_mv": read_u16(c, id, calib::VDD_MV)?,
    });
    let mut m = serde_json::Map::new();
    m.insert("model".into(), identity.model.into());
    m.insert("fw".into(), identity.fw.into());
    m.insert("tick_hz".into(), tick_hz.into());
    m.insert("pwm_period_ticks".into(), pwm_period_ticks.into());
    m.insert("vbus_counts".into(), tel.vbus_counts.into());
    m.insert("plant".into(), plant);
    m.insert("sense".into(), sense);
    Ok(m)
}

/// How the drive was held: the firmware holds open-loop current to the
/// servo's own `current_limit_counts`.
const RULE: &str = "limit";

/// The `drive` block's servo settings: the limit and the stall settings
/// that held the drive, the winding R, the abort a quarter over the limit,
/// and the seek.
pub(crate) fn drive(
    lim: &ServoLimits,
    stall: &Stall,
    seek_q15: i16,
) -> serde_json::Map<String, serde_json::Value> {
    let mut m = serde_json::Map::new();
    m.insert("rule".into(), RULE.into());
    m.insert("current_limit_counts".into(), lim.i_lim.into());
    m.insert("window_floor_q15".into(), lim.window_floor_q15.into());
    m.insert("i_cont_counts".into(), lim.i_cont.into());
    m.insert("stall_release_counts".into(), stall.release.into());
    m.insert("stall_time_ticks".into(), stall.time_ticks.into());
    m.insert("stall_response".into(), stall.response().into());
    m.insert("stall_tau_trip_counts".into(), lim.tau_trip.into());
    m.insert("r_q12".into(), lim.r_q12.into());
    m.insert("i_abort_counts".into(), lim.abort_default().into());
    m.insert("seek_q15".into(), seek_q15.into());
    m
}

/// The descriptor-placed registers the run toggles.
pub(crate) struct Regs {
    pub(crate) decay: Reg,
    pub(crate) zero_brake: Reg,
}

pub(crate) fn resolve_regs<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Regs> {
    let identity = c.identity(id)?;
    let registry = descriptor::load()?;
    let (d, note) = descriptor::select(&registry, identity.model, identity.fw)?;
    if let Some(note) = note {
        println!("{note}");
    }
    let field_reg = |name: &str| -> Result<Reg> {
        let f = descriptor::field(d, name)?;
        Ok(Reg {
            addr: f.addr,
            width: f.width as u8,
        })
    };
    Ok(Regs {
        decay: field_reg("openloop_decay")?,
        zero_brake: field_reg("openloop_zero_brake")?,
    })
}

/// Baseline, then every chain of every direction. Each committed segment
/// reaches `on_seg` as it lands, so a caller keeps what was captured before
/// a later chain gives up. Leaves the servo guarded and torqued off. Ok
/// carries the TEL rows the servo dropped from the segments' bursts, which
/// the segments are missing.
pub(crate) fn record<S: Servo>(
    s: &mut S,
    cfg: &Cfg,
    mut on_seg: impl FnMut(&Segment) -> Result<()>,
) -> Result<u16> {
    let id = s.id();
    let regs = resolve_regs(s.client(), id)?;
    let r = guard(s, |s| chains(s, cfg, &regs, &mut on_seg));
    // Belt for a run cut mid-burst: brake flag clear, decay back to slow,
    // guards back on. The permit is RAM-only so a power cycle clears it
    // anyway, but a servo left unguarded until someone reboots it is a trap.
    let _ = s.write(regs.zero_brake, 0);
    let _ = s.write(regs.decay, Decay::Slow as i32);
    let _ = s.write(control::STALL_PERMIT, 0);
    r
}

fn chains<S: Servo>(
    s: &mut S,
    cfg: &Cfg,
    regs: &Regs,
    on_seg: &mut impl FnMut(&Segment) -> Result<()>,
) -> Result<u16> {
    let chunks = cfg.tel_chunks;
    // refuses firmware that predates TEL duty counts
    s.pwm_period_ticks()?;
    let tick_hz = s.tick_hz()?;
    let seek_duty = pct_q15(cfg.seek_duty_pct);
    let seek_cap = pct_q15(cfg.seek_cap_pct) as i32;
    let mut rows_dropped = 0u16;
    let mut commit = |g: Segment| -> Result<()> {
        on_seg(&g)?;
        rows_dropped = rows_dropped.saturating_add(g.stats.rows_dropped);
        Ok(())
    };

    s.write(regs.decay, Decay::Slow as i32)?;
    s.write(regs.zero_brake, 0)?;
    let mut lease = Lease::new(cfg.stall || cfg.static_load, s.experiment()?);
    // the servo's clock carries the timeline, anchored once to unix time
    let anchor = unix_s() - s.now_ms() / 1000.0;

    // baseline: mid-travel, torque off, noise floor at full tick rate
    if cfg.baseline_ms > 0 {
        println!("[baseline] {} ms torque-off", cfg.baseline_ms);
        if !cfg.static_load {
            seek_band(s, &mut lease, (1750, 2350), seek_duty, seek_cap, cfg.stops)?;
        }
        lease.write(s, control::TORQUE_ENABLE, 0)?;
        let t_start = anchor + s.now_ms() / 1000.0;
        let (frames, st) = s.stream(samples_of_ms(cfg.baseline_ms, tick_hz), None, chunks)?;
        println!(
            "  seg 0: {} frames, {} samples, {} seq holes, {} garble bytes, {} ring laps",
            st.frames, st.samples, st.holes, st.garble, st.laps
        );
        commit(Segment {
            seg: 0,
            dir: 0,
            cmd_duty_q15: 0,
            frames,
            stats: st,
            t_start,
            t_end: anchor + s.now_ms() / 1000.0,
        })?;
    }

    let steps = &cfg.steps;

    // Retry granularity is the CHAIN, not the step: then/coast/brake steps
    // inherit momentum from the drive they follow, so replaying one alone
    // would capture it from the wrong state. A chain starts wherever the
    // previous step does not feed this one.
    let starts: Vec<usize> = (0..steps.len())
        .filter(|&k| k == 0 || !feeds(steps, k - 1))
        .collect();

    let mut seg = 1u32;
    for &dir in cfg.dirs.signs() {
        for (ci, &chain0) in starts.iter().enumerate() {
            let chain_end = starts.get(ci + 1).copied().unwrap_or(steps.len());
            let seg0 = seg;
            let mut attempt = 0u32;
            let mut pending: Vec<(Segment, String)> = Vec::new();
            loop {
                attempt += 1;
                pending.clear();
                let mut dirty = None;
                let mut live = false;
                seg = seg0;
                for k in chain0..chain_end {
                    let step = steps[k];
                    check_stop()?;
                    // Drive is never chained, so it always arrives with live
                    // false: the seek is the only difference in its prep.
                    if let Step::Drive(..) = step {
                        check_fault(s)?;
                        if cfg.static_load {
                            // nothing to seek
                        } else if cfg.stall {
                            // Already stopped, and still pressed into the stop:
                            // nothing to brake and nothing to let settle.
                            seek_stop(s, &mut lease, dir, seek_duty, seek_cap, cfg.stops)?;
                        } else {
                            seek_band(
                                s,
                                &mut lease,
                                start_band(dir, cfg.guard.0, cfg.guard.1),
                                seek_duty,
                                seek_cap,
                                cfg.stops,
                            )?;
                            // Kill the seek's momentum and let the shaft ring
                            // down. Sign is -dir, the mirror of the post-rung
                            // call: the seek parks NEAR its start-band wall, so
                            // the token brake duty has to point away from that
                            // one instead.
                            brake_to_rest(s, &mut lease, -dir)?;
                            rest(s, cfg.settle_ms)?;
                        }
                    } else if !live {
                        check_fault(s)?;
                    }
                    if !live {
                        s.write(control::MODE, 0)?;
                        lease.torque_on(s)?;
                    }
                    let (ms, duty) = match step {
                        Step::Drive(pct, ms) => (
                            ms.unwrap_or(cfg.window_ms),
                            dir as i32 * pct_q15(pct) as i32,
                        ),
                        Step::Then(pct, ms) => (
                            ms.unwrap_or(cfg.window_ms),
                            dir as i32 * pct.signum() as i32 * pct_q15(pct.unsigned_abs()) as i32,
                        ),
                        Step::Hold(pct, ms) => (
                            ms,
                            dir as i32 * pct.signum() as i32 * pct_q15(pct.unsigned_abs()) as i32,
                        ),
                        Step::Coast(ms) | Step::Brake(ms) => (ms, 0),
                    };
                    if matches!(step, Step::Brake(_)) {
                        s.write(regs.zero_brake, 1)?;
                    }
                    // Fast decay only inside the burst: the seek and the post-step
                    // brake need slow decay to move and to stop.
                    if cfg.decay == Decay::Fast {
                        s.write(regs.decay, Decay::Fast as i32)?;
                    }
                    let t_start = anchor + s.now_ms() / 1000.0;
                    let (frames, st) = if let Step::Hold(..) = step {
                        s.write(control::GOAL_DUTY, duty)?;
                        hold(s, &mut lease, ms)?;
                        (Vec::new(), BurstStats::default())
                    } else {
                        let samples = samples_of_ms(ms, tick_hz);
                        lease.keep(s)?;
                        lease.check_stream(samples)?;
                        s.stream(samples, Some((control::GOAL_DUTY, duty)), chunks)?
                    };
                    let t_end = anchor + s.now_ms() / 1000.0;
                    if cfg.decay == Decay::Fast {
                        s.write(regs.decay, Decay::Slow as i32)?;
                    }
                    if matches!(step, Step::Brake(_)) {
                        s.write(regs.zero_brake, 0)?;
                    }
                    let what = match step {
                        Step::Drive(pct, _) => format!("duty {:+}%", dir as i32 * pct as i32),
                        Step::Then(pct, _) => format!("then {:+}%", dir as i32 * pct as i32),
                        Step::Coast(ms) => format!("coast {ms} ms"),
                        Step::Brake(ms) => format!("brake {ms} ms"),
                        Step::Hold(pct, ms) => {
                            format!("hold {:+}% {ms} ms", dir as i32 * pct as i32)
                        }
                    };
                    if st.holes > 0 || st.garble > 0 || st.laps > 0 {
                        dirty = Some(retry_note(seg, &what, st.holes, st.garble));
                    }
                    let g = Segment {
                        seg,
                        dir,
                        cmd_duty_q15: duty as i16,
                        frames,
                        stats: st,
                        t_start,
                        t_end,
                    };
                    pending.push((g, what));
                    if feeds(steps, k) {
                        live = true;
                    } else {
                        brake_to_rest(s, &mut lease, dir)?;
                        lease.write(s, control::TORQUE_ENABLE, 0)?;
                        rest(s, cfg.rest_ms)?;
                        live = false;
                    }
                    seg += 1;
                }
                match dirty {
                    None => break,
                    Some(why) if attempt < cfg.rung_tries => {
                        println!("  retry rung, attempt {attempt}:{}", why.trim_start());
                    }
                    Some(why) => bail!(
                        "rung failed {} attempts, giving up:{}",
                        cfg.rung_tries,
                        why.trim_start()
                    ),
                }
            }
            for (g, what) in pending.drain(..) {
                let st = &g.stats;
                println!(
                    "  seg {} ({what}): {} frames, {} samples, {} seq holes, {} garble bytes, {} ring laps",
                    g.seg, st.frames, st.samples, st.holes, st.garble, st.laps
                );
                commit(g)?;
            }
        }
    }
    Ok(rows_dropped)
}

/// Rewrite the run's meta with the rows the servo dropped. A sweep is a raw
/// tool and keeps what it captured: dropped rows only warn.
fn land_rows_dropped(
    meta_path: &Path,
    mut meta: serde_json::Value,
    rows_dropped: u16,
) -> Result<Option<String>> {
    meta["rows_dropped"] = rows_dropped.into();
    std::fs::write(meta_path, serde_json::to_string_pretty(&meta)?)
        .with_context(|| format!("write {}", meta_path.display()))?;
    Ok((rows_dropped > 0).then(|| {
        format!(
            "warning: the servo dropped {rows_dropped} TEL rows (both stream buffers were \
             waiting for the wire); the files are saved with those rows missing"
        )
    }))
}

/// Entry from the top-level `osc-lab sweep` dispatch.
pub fn run(args: &Args, bus: Bus, id: u8) -> Result<()> {
    pump::install_ctrlc();
    let id = Id::new(id);
    let mut c = crate::rig::connect(&bus, &[id])?;
    crate::state::check(&mut c, id)?;
    let lim = crate::rig::limits::read(&mut c, id)?;
    let plan = crate::rig::limits::plan(&mut c, id, &lim)?;
    let cfg = Cfg::new(args, &lim, &plan)?;

    std::fs::create_dir_all(&args.out).with_context(|| format!("mkdir {}", args.out.display()))?;

    let stall = Stall::read(&mut c, id)?;
    let drive = drive(&lim, &stall, pct_q15(cfg.seek_duty_pct));
    let meta = meta(&mut c, id, &cfg, serde_json::Value::Object(drive))?;
    let meta_path = args.out.join("meta.json");
    std::fs::write(&meta_path, serde_json::to_string_pretty(&meta)?)
        .with_context(|| format!("write {}", meta_path.display()))?;

    let csv_path = args.out.join("sweep.csv");
    let mut w = std::io::BufWriter::new(
        std::fs::File::create(&csv_path)
            .with_context(|| format!("create {}", csv_path.display()))?,
    );
    writeln!(w, "{CSV_HEADER}")?;
    let seg_path = args.out.join("segments.csv");
    let mut t = std::io::BufWriter::new(
        std::fs::File::create(&seg_path)
            .with_context(|| format!("create {}", seg_path.display()))?,
    );
    writeln!(t, "{SEGMENTS_HEADER}")?;

    // A resistor has no shaft to park.
    let center = if cfg.static_load {
        None
    } else {
        let (lo, hi) = lim.soft;
        Some(((lo + hi) / 2).clamp(0, u16::MAX as i32) as u16)
    };

    let mut s = Wire::new(c, id);
    let r = record(&mut s, &cfg, |g| {
        write_rows(&mut w, g)?;
        write_times(&mut t, g)
    });
    let flushed = w.flush().and(t.flush());
    // a blocked shaft is left where it stopped
    let parked = match (&r, center) {
        (Err(e), _) if blocked(e) => Ok(()),
        (_, Some(at)) => park(&mut s, at, pct_q15(cfg.seek_duty_pct)),
        (_, None) => Ok(()),
    };
    flushed?;
    let rows_dropped = r?;
    parked?;
    if let Some(warning) = land_rows_dropped(&meta_path, meta, rows_dropped)? {
        println!("{warning}");
    }
    println!("sweep: {}", csv_path.display());
    println!("meta:  {}", meta_path.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::instrument::frame::TEL_RAW;
    use crate::instrument::limits::stall_counts;
    use crate::rig::testkit::Rig;

    #[test]
    fn duty_grid_maps_percent_to_q15() {
        assert_eq!(pct_q15(100), 32767);
        assert_eq!(pct_q15(50), 16383);
        assert_eq!(pct_q15(5), 1638);
        assert_eq!(pct_q15(28), 9174);
    }

    #[test]
    fn window_sizes_at_the_servos_tick_rate() {
        assert_eq!(samples_of_ms(150, 20_000.0), 3000);
        assert_eq!(samples_of_ms(1000, 20_000.0), 20000);
        assert_eq!(samples_of_ms(1000, 12_500.0), 12500);
        assert_eq!(samples_of_ms(10_000, 20_000.0), 200_000, "never saturates");
    }

    #[test]
    fn steps_parse_and_display_round_trip() {
        for (s, want) in [
            ("35", Step::Drive(35, None)),
            ("35@800", Step::Drive(35, Some(800))),
            ("then:-20", Step::Then(-20, None)),
            ("then:20", Step::Then(20, None)),
            ("then:20@450", Step::Then(20, Some(450))),
            ("coast:200", Step::Coast(200)),
            ("brake:150", Step::Brake(150)),
            ("hold:100@900", Step::Hold(100, 900)),
            ("hold:-40@50", Step::Hold(-40, 50)),
            ("hold:0@990", Step::Hold(0, 990)),
        ] {
            assert_eq!(parse_step(s).unwrap(), want);
            assert_eq!(want.to_string(), s);
        }
        assert!(parse_step("coast").is_err());
        assert!(parse_step("brake:x").is_err());
        assert!(parse_step("then:0").is_err());
        assert!(parse_step("then:101").is_err());
        assert!(parse_step("then:-101").is_err());
        assert!(parse_step("then:x").is_err());
        assert!(parse_step("slow").is_err());
        assert!(parse_step("35@0").is_err());
        assert!(parse_step("35@x").is_err());
        assert!(parse_step("@800").is_err());
        assert!(parse_step("hold:100").is_err(), "a hold names its span");
        assert!(parse_step("hold:101@900").is_err());
        assert!(parse_step("hold:100@0").is_err());
    }

    #[test]
    fn chained_steps_feed_from_their_predecessor() {
        let steps = [
            Step::Drive(20, None),
            Step::Then(-20, None),
            Step::Then(20, None),
            Step::Brake(200),
        ];
        let fed: Vec<bool> = (0..steps.len()).map(|k| feeds(&steps, k)).collect();
        assert_eq!(fed, [true, true, true, false]);
        assert!(!feeds(&[Step::Then(-20, None), Step::Drive(20, None)], 0));
    }

    #[test]
    fn retry_note_cannot_read_as_failure() {
        // the exact pattern a campaign script scrapes stdout with
        let bad = regex_lite(&retry_note(7, "duty +35%", 4, 9));
        assert!(!bad, "retry note reads as a failed capture");
        // and the committed line for a genuinely dirty rung still must match,
        // otherwise the outer gate would stop catching anything
        assert!(regex_lite(
            "  seg 7 (duty +35%): 200 frames, 4 seq holes, 0 garble bytes"
        ));
    }

    /// `[1-9][0-9]* seq holes` or `[1-9][0-9]* garble`, without a regex dep:
    /// a digit run ending just before the tail, whose first digit is not 0.
    fn regex_lite(line: &str) -> bool {
        [" seq holes", " garble"].iter().any(|tail| {
            line.match_indices(tail).any(|(i, _)| {
                let head = &line[..i];
                let digits = head.len() - head.trim_end_matches(|c: char| c.is_ascii_digit()).len();
                digits > 0 && !head[head.len() - digits..].starts_with('0')
            })
        })
    }

    #[derive(clap::Parser)]
    struct Cli {
        #[command(flatten)]
        args: Args,
    }

    fn parse(argv: &[&str]) -> Args {
        <Cli as clap::Parser>::try_parse_from(argv).unwrap().args
    }

    fn mg90() -> ServoLimits {
        ServoLimits {
            i_lim: 280,
            i_cont: 168,
            tau_trip: 280,
            soft: (432, 3626),
            raw: (209, 3849),
            r_q12: 7270,
            vbus: 3204,
            window_floor_q15: 4356,
            amps_per_count: 3.3 / 4096.0 / (15.0 * 0.060),
        }
    }

    /// The class's lowest winding, 3.0 ohm, on the bench board's sense
    /// chain: 60 mohm at G 15, 6k8/3k3 terminal taps.
    const CLASS_R_VPC: f64 = 3.0 * (3.3 / 4096.0 / 0.9) / (3.3 / 4096.0 * 10_100.0 / 3_300.0);

    fn plan(lim: &ServoLimits) -> DutyPlan {
        lim.stall_plan(CLASS_R_VPC)
    }

    /// The bench MG90, limit 280 at 4.9 ohm: the seek and its ceiling are
    /// the duty whose stall draws the limit, 15% on 2S and 27% on USB.
    /// Before R is stored the class's lowest R plans them: 9% on 2S.
    #[test]
    fn sweep_defaults_come_from_the_servo_limits() {
        for (vbus, want) in [(3204, 15), (1780, 27)] {
            let lim = ServoLimits { vbus, ..mg90() };
            let cfg = Cfg::new(&parse(&["sweep"]), &lim, &plan(&lim)).unwrap();
            assert_eq!(
                (cfg.seek_duty_pct, cfg.seek_cap_pct),
                (want, want),
                "{vbus}"
            );
            let p = plan(&lim);
            let stall = stall_counts(pct_q15(want) as f64 / 32767.0, p.r_vpc, vbus as f64);
            assert!(stall <= 280.0);
        }
        let virgin = ServoLimits { r_q12: 0, ..mg90() };
        let cfg = Cfg::new(&parse(&["sweep"]), &virgin, &plan(&virgin)).unwrap();
        assert_eq!((cfg.seek_duty_pct, cfg.seek_cap_pct), (9, 9));
        // asked for, a gentler seek is taken as it stands
        let a = parse(&["sweep", "--seek-duty-pct", "12", "--seek-cap-pct", "14"]);
        let cfg = Cfg::new(&a, &mg90(), &plan(&mg90())).unwrap();
        assert_eq!((cfg.seek_duty_pct, cfg.seek_cap_pct), (12, 14));
    }

    /// A seek, a breakout ceiling or a rung pressed into a stop whose stall
    /// draws more than the limit is refused in plain words; free-running
    /// rungs and a resistor's rungs are not.
    #[test]
    fn a_stall_sweep_over_the_limit_is_refused() {
        let refused = |argv: &[&str]| {
            let Err(e) = Cfg::new(&parse(argv), &mg90(), &plan(&mg90())) else {
                panic!("{argv:?} was not refused");
            };
            e.to_string()
        };
        assert_eq!(
            refused(&["sweep", "--stall", "--duty-pct", "10,then:15,20"]),
            "a rung pressed into the stop stalls the motor at 20% duty, which draws about 361 \
             counts (323 mA), over this servo's current limit of 280 counts (251 mA); on this \
             supply the limit holds a stall only up to 15% duty"
        );
        assert_eq!(
            refused(&["sweep", "--seek-duty-pct", "28"]),
            "the seek stalls the motor at 28% duty, which draws about 505 counts (452 mA), \
             over this servo's current limit of 280 counts (251 mA); on this supply the limit \
             holds a stall only up to 15% duty"
        );
        assert!(
            refused(&["sweep", "--seek-cap-pct", "45"])
                .starts_with("a seek raised to its ceiling stalls the motor at 45% duty")
        );
        assert!(refused(&["sweep", "--stall", "--duty-pct", "then:-16"]).contains("at 16% duty"));

        let a = parse(&["sweep", "--stall", "--duty-pct", "10,then:15,coast:100"]);
        let cfg = Cfg::new(&a, &mg90(), &plan(&mg90())).unwrap();
        assert!(cfg.stall);
        // free rungs run on the firmware limiter, whatever their duty
        let a = parse(&["sweep", "--duty-pct", "64,100"]);
        assert!(Cfg::new(&a, &mg90(), &plan(&mg90())).is_ok());
        // a resistor has no travel and no stop
        let a = parse(&[
            "sweep",
            "--static-load",
            "--duty-pct",
            "64",
            "--seek-duty-pct",
            "40",
        ]);
        assert!(Cfg::new(&a, &mg90(), &plan(&mg90())).is_ok());
    }

    #[test]
    fn cfg_carries_the_cli_defaults() {
        let cfg = Cfg::new(&parse(&["sweep"]), &mg90(), &plan(&mg90())).unwrap();
        let grid: Vec<Step> = (1..=20u8).map(|k| Step::Drive(k * 5, None)).collect();
        assert_eq!(cfg.steps, grid);
        assert_eq!(cfg.dirs, Dirs::Both);
        assert_eq!(cfg.decay, Decay::Slow);
        assert_eq!(cfg.window_ms, 150);
        assert_eq!(cfg.rest_ms, 500);
        assert_eq!(cfg.baseline_ms, 1000);
        assert_eq!((cfg.seek_duty_pct, cfg.seek_cap_pct), (15, 15));
        assert_eq!(cfg.settle_ms, 300);
        assert!(!cfg.stall && !cfg.static_load);
        assert_eq!(cfg.guard, (532, 3526), "the soft limits, 100 counts in");
        assert_eq!(cfg.stops, Some((209, 3849)));
        assert_eq!(cfg.tel_chunks, TEL_RAW);
        assert_eq!(cfg.rung_tries, 3);
    }

    #[test]
    fn guard_flags_override_the_soft_limits() {
        let a = parse(&["sweep", "--guard-lo", "600"]);
        assert_eq!(
            Cfg::new(&a, &mg90(), &plan(&mg90())).unwrap().guard,
            (600, 3526)
        );
    }

    #[test]
    fn a_servo_without_calibrated_limits_is_refused() {
        let virgin = ServoLimits {
            soft: (0, 4095),
            ..mg90()
        };
        let Err(err) = Cfg::new(&parse(&["sweep"]), &virgin, &plan(&virgin)) else {
            panic!("a virgin servo was not refused");
        };
        assert!(err.to_string().ends_with("run `osc cal` first"), "{err}");
        // a resistor has no travel to calibrate
        let a = parse(&["sweep", "--static-load"]);
        assert_eq!(
            Cfg::new(&a, &virgin, &plan(&virgin)).unwrap().guard,
            (0, 4095)
        );
    }

    #[test]
    fn schedule_strings_parse_through_the_cli_grammar() {
        let steps: Vec<Step> = ["20@450", "then:-20", "brake:150"]
            .iter()
            .map(|s| s.parse().unwrap())
            .collect();
        assert_eq!(
            steps,
            [
                Step::Drive(20, Some(450)),
                Step::Then(-20, None),
                Step::Brake(150)
            ]
        );
    }

    #[test]
    fn start_bands_hug_the_guard_edges() {
        // centred on the guard, so approach direction cannot shift the start
        assert_eq!(start_band(1, 400, 3700), (325, 475));
        assert_eq!(start_band(-1, 400, 3700), (3625, 3775));
    }

    /// A resistor sweep: no seek, the rungs drive directly under the stall
    /// permit, each brakes and torques off, and the run leaves the servo
    /// guarded.
    fn static_cfg(steps: Vec<Step>) -> Cfg {
        Cfg {
            steps,
            dirs: Dirs::Both,
            decay: Decay::Fast,
            window_ms: 20,
            rest_ms: 500,
            baseline_ms: 10,
            seek_duty_pct: 0,
            seek_cap_pct: 0,
            settle_ms: 300,
            stall: false,
            static_load: true,
            guard: (0, 4095),
            stops: None,
            tel_chunks: Chunks::one(0x23c, 12),
            rung_tries: 3,
        }
    }

    /// What a test keeps of a committed segment: its number, sign, duty
    /// and rows.
    type Seen = (u32, i8, i16, usize);

    /// Every committed segment, and the rows the servo dropped from them.
    fn run(rig: &mut Rig, cfg: &Cfg) -> Result<(Vec<Seen>, u16)> {
        let mut seen = Vec::new();
        let dropped = record(rig, cfg, |g| {
            seen.push((g.seg, g.dir, g.cmd_duty_q15, g.frames.len()));
            Ok(())
        })?;
        Ok((seen, dropped))
    }

    /// A resistor sweep through the in-process servo stack: the baseline,
    /// then a chain per direction, every segment the rows its window asked
    /// for at its signed duty, the permit held behind every torque enable,
    /// and the servo left safe.
    #[test]
    fn a_static_load_sweep_records_every_chain_and_leaves_the_servo_safe() {
        let mut rig = Rig::bench();
        let cfg = static_cfg(vec![Step::Drive(40, None), Step::Then(-20, Some(15))]);
        let (seen, dropped) = run(&mut rig, &cfg).unwrap();
        assert_eq!(
            seen,
            [
                (0, 0, 0, 200),
                (1, 1, 13106, 400),
                (2, 1, -6553, 300),
                (3, -1, -13106, 400),
                (4, -1, 6553, 300),
            ]
        );
        assert_eq!(dropped, 0);
        let on: Vec<usize> = (0..rig.writes.len())
            .filter(|&k| rig.writes[k] == (control::TORQUE_ENABLE, 1))
            .collect();
        assert_eq!(on.len(), 2, "one torque enable a chain");
        for k in on {
            assert_eq!(rig.writes[k + 1], (control::STALL_PERMIT, 1));
        }
        // the guard's SAFE writes, then the belt: brake flag, decay, permit
        let n = rig.writes.len();
        assert_eq!(rig.writes[n - 9..n - 3], crate::rig::servo::SAFE);
        assert_eq!(rig.writes[n - 1], (control::STALL_PERMIT, 0));
        let id = rig.id();
        for reg in [control::TORQUE_ENABLE, control::STALL_PERMIT] {
            assert_eq!(rig.c.read(id, reg.addr, 1).unwrap(), [0]);
        }
        assert_eq!(read_u16(&mut rig.c, id, control::TEL_COUNT).unwrap(), 0);
    }

    /// A hold chains like a then step and streams nothing: torque stays on
    /// from the drive through the hold into the next window, the hold's
    /// duty written as the goal, its segment empty, and every segment's
    /// host clock in order.
    #[test]
    fn a_hold_keeps_the_drive_on_unstreamed() {
        let mut rig = Rig::bench();
        let cfg = Cfg {
            dirs: Dirs::Fwd,
            ..static_cfg(vec![
                Step::Drive(40, None),
                Step::Hold(40, 120),
                Step::Then(40, Some(15)),
            ])
        };
        let mut segs = Vec::new();
        record(&mut rig, &cfg, |g| {
            segs.push((g.seg, g.cmd_duty_q15, g.frames.len(), g.t_start, g.t_end));
            Ok(())
        })
        .unwrap();
        let seen: Vec<_> = segs.iter().map(|g| (g.0, g.1, g.2)).collect();
        assert_eq!(
            seen,
            [(0, 0, 200), (1, 13106, 400), (2, 13106, 0), (3, 13106, 300)]
        );
        assert!(segs[2].4 - segs[2].3 > 0.119, "the hold spans its window");
        assert!(segs.windows(2).all(|w| w[0].4 <= w[1].3), "one timeline");
        let on = rig
            .writes
            .iter()
            .filter(|w| **w == (control::TORQUE_ENABLE, 1))
            .count();
        assert_eq!(on, 1, "one torque enable for the chain");
        assert!(rig.writes.contains(&(control::GOAL_DUTY, 13106)));
    }

    /// A sweep whose servo dropped rows saves its rows and a meta that
    /// names the count, and warns; a clean one writes 0 and says nothing.
    #[test]
    fn a_sweep_with_dropped_rows_warns_and_saves() {
        let cfg = Cfg {
            dirs: Dirs::Fwd,
            ..static_cfg(vec![Step::Drive(40, None)])
        };
        for (drops, warns) in [(0, false), (3, true)] {
            let out = std::env::temp_dir().join(format!(
                "osc-lab-sweep-{}-dropped-{drops}",
                std::process::id()
            ));
            std::fs::create_dir_all(&out).unwrap();
            let mut rig = Rig::bench();
            rig.drop = drops;
            let csv_path = out.join("sweep.csv");
            let mut w = std::fs::File::create(&csv_path).unwrap();
            writeln!(w, "{CSV_HEADER}").unwrap();
            let mut frames = 0;
            let dropped = record(&mut rig, &cfg, |g| {
                frames += g.frames.len();
                write_rows(&mut w, g)
            })
            .unwrap();
            assert_eq!(dropped, 2 * drops, "the baseline and the rung");
            let meta_path = out.join("meta.json");
            let meta = serde_json::json!({ "window_ms": 20 });
            let warning = land_rows_dropped(&meta_path, meta, dropped).unwrap();

            let meta: serde_json::Value =
                serde_json::from_str(&std::fs::read_to_string(&meta_path).unwrap()).unwrap();
            assert_eq!(meta["rows_dropped"], dropped);
            assert_eq!(meta["window_ms"], 20, "the rest of the meta is kept");
            let rows = std::fs::read_to_string(&csv_path).unwrap().lines().count();
            assert_eq!(rows, 1 + frames);
            assert_eq!(warning.is_some(), warns);
            if let Some(w) = warning {
                assert!(w.contains(&format!("dropped {dropped} TEL rows")), "{w}");
            }
            std::fs::remove_dir_all(&out).unwrap();
        }
    }

    /// A rung window longer than a held permit can ride without a rewrite
    /// is refused before its burst arms, on the production image; the
    /// experiment image's dead-man rides any burst.
    #[test]
    fn a_rung_longer_than_the_permit_is_refused_on_the_production_image() {
        let cfg = Cfg {
            dirs: Dirs::Fwd,
            baseline_ms: 0,
            ..static_cfg(vec![Step::Drive(40, Some(800))])
        };
        let mut rig = Rig::bench();
        let e = record(&mut rig, &cfg, |_| Ok(())).err().unwrap();
        assert!(e.to_string().starts_with("a 800 ms capture"), "{e}");
        assert!(
            !rig.writes.contains(&(control::GOAL_DUTY, 13106)),
            "the rung never armed"
        );
        let mut rig = Rig::bench();
        rig.experiment = true;
        let (seen, _) = run(&mut rig, &cfg).unwrap();
        assert_eq!(seen, [(1, 1, 13106, 16_000)]);
        let deadman = (
            control::BUS_DEADMAN_TICKS,
            crate::instrument::limits::DEADMAN_TICKS as i32,
        );
        let k = rig
            .writes
            .iter()
            .position(|w| *w == (control::TORQUE_ENABLE, 1))
            .unwrap();
        assert_eq!(rig.writes[k - 1], deadman);
    }

    /// The real capture's sweep.csv, row for row: every line parsed back
    /// into a segment's frames and written again is the same bytes.
    #[test]
    fn sweep_csv_is_byte_compatible_with_a_committed_capture() {
        let text = std::fs::read_to_string(fixture("sweep.csv")).unwrap();
        let mut lines = text.lines();
        assert_eq!(lines.next(), Some(CSV_HEADER));
        let opt = |v: &str| (!v.is_empty()).then(|| v.parse::<i32>().unwrap());
        let mut out = Vec::new();
        writeln!(out, "{CSV_HEADER}").unwrap();
        let mut seg: Option<Segment> = None;
        for line in lines {
            let c: Vec<&str> = line.split(',').collect();
            assert_eq!(c.len(), 18, "{line}");
            let (s, duty, dir) = (
                c[0].parse().unwrap(),
                c[1].parse().unwrap(),
                c[2].parse().unwrap(),
            );
            if seg.as_ref().is_none_or(|g| g.seg != s) {
                if let Some(g) = seg.take() {
                    write_rows(&mut out, &g).unwrap();
                }
                seg = Some(Segment {
                    seg: s,
                    dir,
                    cmd_duty_q15: duty,
                    frames: Vec::new(),
                    stats: BurstStats::default(),
                    t_start: 0.0,
                    t_end: 0.0,
                });
            }
            let u = |k: usize| opt(c[k]).map(|v| v as u16);
            let i = |k: usize| opt(c[k]).map(|v| v as i16);
            seg.as_mut().unwrap().frames.push(TelFrame {
                tick: c[3].parse().unwrap(),
                window_valid: c[4] == "1",
                pos: u(5),
                current: i(6),
                current_trough: u(7),
                duty_counts: i(8),
                vdiff: i(9),
                vbus: u(10),
                current_raw: u(11),
                vmotor_a: u(12),
                vmotor_b: u(13),
                vbus_raw: u(14),
                ntc_raw: u(15),
                vref_raw: u(16),
            });
        }
        write_rows(&mut out, &seg.unwrap()).unwrap();
        assert!(String::from_utf8(out).unwrap() == text, "sweep.csv drifted");
    }

    /// A committed capture of the experiment image on a resistor: its
    /// meta.json and sweep.csv.
    fn fixture(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../telemetry/mg90-b__dev-v006-2A-D/rest-resistance/capture-1/stall/hole-1/3.3V")
            .join(name)
    }

    fn keys(v: &serde_json::Value) -> Vec<String> {
        let mut out = Vec::new();
        for (k, v) in v.as_object().unwrap() {
            out.push(k.clone());
            if let Some(o) = v.as_object() {
                out.extend(o.keys().map(|n| format!("{k}.{n}")));
            }
        }
        out.sort();
        out
    }

    /// The meta a sweep writes on the in-process servo stack carries every
    /// key a committed capture's does, nested blocks included, less the
    /// host build's sha; and nothing else.
    #[test]
    fn meta_keys_match_a_committed_capture() {
        let want: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(fixture("meta.json")).unwrap()).unwrap();
        let mut want = keys(&want);
        want.retain(|k| k != "host_git_sha");

        let mut rig = Rig::bench();
        let id = rig.id();
        let lim = crate::rig::limits::read(&mut rig.c, id).unwrap();
        let p = crate::rig::limits::plan(&mut rig.c, id, &lim).unwrap();
        let cfg = Cfg::new(&parse(&["sweep", "--static-load"]), &lim, &p).unwrap();
        let stall = Stall::read(&mut rig.c, id).unwrap();
        let drive = drive(&lim, &stall, pct_q15(cfg.seek_duty_pct));
        let mut m = meta(&mut rig.c, id, &cfg, serde_json::Value::Object(drive)).unwrap();
        m["rows_dropped"] = 0.into();
        assert_eq!(keys(&m), want);
        assert_eq!(m["drive"]["rule"], "limit");
        assert_eq!(m["plant"]["lut_state"], "IDENTITY");
        assert_eq!(
            m["tel_chunks"],
            serde_json::json!([[0x238, 2], [0x240, 10]])
        );
    }
}
