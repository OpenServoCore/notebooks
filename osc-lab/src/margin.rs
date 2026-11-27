//! `osc-lab margin` - a loop-margin sweep on the `deadman` image: in
//! Velocity mode at a cruise the image sums a triangle into the velocity
//! loop's reference while a TEL burst streams under the bus dead-man
//! (protocol sec 5.6, kernel sec 2.7), and each burst's margin row carries
//! the observer's position and speed and the summed reference. One burst a
//! triangle's half-period N at one cruise: a Position move to the burst's
//! start, the two stimulus fields written, Velocity mode, the cruise
//! committed with the burst's arm, then the cruise stopped and the fields
//! zeroed. The bursts alternate direction so the shaft sweeps back and
//! forth inside the guard. Nothing here reads a margin: the rows and each
//! burst's facts land in --out as `oscnb.margin` reads them.
//!
//! Before each cruise's first burst a source check captures which estimate
//! the speed loop closes on (`omega_hat_src`) at that cruise, recorded per
//! cruise in meta.json with no verdict.
//!
//! The run is `track`'s closed-loop machinery: its preflight, watchdog,
//! dead-man, guards, centering and torque off at the end.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use osc_client::Id;
use osc_client::descriptor::{self as desc, Descriptor, Value};
use osc_protocol::tel::{Chunk, Chunks};
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::regions::control::{INJECT_ACCEL_MAX_Q88, INJECT_HALF_PERIODS_MAX};

use crate::instrument::frame::TelFrame;
use crate::instrument::regs::{control, telemetry};
use crate::rig::pump::{self, BurstStats, Lease};
use crate::rig::servo::{Servo, Wire, guard};
use crate::rig::snapshot::read_u16;
use crate::sweep::{chunks_json, servo_meta, unix_s};
use crate::track::{
    self, Abort, Cfg, Extra, Fault, Guarded, Mode, Poll, Ran, center, check_deadman, go_to, halt,
    meta_extra, parse_extra, poll_extra, preflight, put_back, read_set, refuse, rows_tripped,
    standing, stopped, trip, watch, zero_inject,
};

/// Fast ticks a medium period: the stimulus steps and the row's three
/// fields publish once a period.
const DECIM: u64 = DECIM_MED as u64;

/// The head of a burst the reader drops (`oscnb.margin.HEAD_S`): the
/// run-up to the cruise and the loop settling on the stimulus.
pub(crate) const HEAD_S: f64 = 0.2;

/// What the host takes from a burst's LAST frame to its stop write, an
/// allowance on the far side of every burst.
const REACT_S: f64 = 0.02;

/// The margin row (protocol sec 5.6): `theta_hat_q16` and `omega_hat_cps`,
/// then `omega_ref_cps`.
pub(crate) const ROW: Chunks = Chunks::two(
    Chunk::new(telemetry::THETA_HAT_Q16.addr, 8),
    Chunk::new(telemetry::OMEGA_REF_CPS.addr, 4),
);

/// The source check's row: `omega_hat_src` with the byte beside it, and
/// the observer's position and speed.
pub(crate) const SOURCE_ROW: Chunks = Chunks::two(
    Chunk::new(telemetry::OMEGA_HAT_SRC.addr, 2),
    Chunk::new(telemetry::THETA_HAT_Q16.addr, 8),
);

/// burst-K.csv's columns, `oscnb.margin.COLUMNS`.
pub(crate) const COLUMNS: &str = "tick,theta_hat_q16,omega_hat_cps,omega_ref_cps";

/// `osc-lab margin` args. `--baud`/`--id` come from the top-level globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir, `telemetry/<pair>/margin/capture-N`: burst-K.csv and
    /// burst-K.json a burst, numbered in capture order, and meta.json.
    #[arg(long)]
    out: PathBuf,
    /// The speed loop's band: half-periods N, medium periods, the
    /// triangle at the medium rate over 2N. N = 1 reads its gain margin at
    /// Nyquist.
    #[arg(long, default_value = "40,30,25,20,15,12,10,8,1", value_parser = List::<u16>::parse)]
    speed_n: List<u16>,
    /// The position loop's band, half-periods N. Either band given as
    /// `none` is left out.
    #[arg(long, default_value = "512,400,300,200,150,100,70,50", value_parser = List::<u16>::parse)]
    position_n: List<u16>,
    /// The cruises the speed band runs at, c/s, comma separated: each its
    /// own reading (742,2500 reads it at a slow and a fast cruise).
    #[arg(long, default_value = "742", value_parser = List::<u32>::parse)]
    speed_cruise_cps: List<u32>,
    /// The cruises the position band runs at, c/s, each its own reading.
    #[arg(long, default_value = "742", value_parser = List::<u32>::parse)]
    position_cruise_cps: List<u32>,
    /// The two-cruise check: the position band again at this factor times
    /// its first cruise, read as a check of that reading.
    #[arg(long, default_value_t = 1.25)]
    second_cruise: f64,
    /// Run no two-cruise check.
    #[arg(long)]
    no_second_cruise: bool,
    /// The triangle's slope at most, c/s per medium period.
    #[arg(long, default_value_t = 15.0)]
    slope_cps: f64,
    /// Its peak at most, c/s: a burst's slope is min(slope, 2 x peak / N).
    #[arg(long, default_value_t = 150.0)]
    peak_cps: f64,
    /// Pairs of bursts a point, one up and one down each.
    #[arg(long, default_value_t = 1)]
    repeats: u32,
    /// The fewest whole cycles a burst may carry: a point whose burst
    /// travels the guard's range in fewer is refused before torque.
    #[arg(long, default_value_t = 4)]
    min_cycles: u32,
    /// The bursts' starts sit this share of the guard's range inside its
    /// ends, percent; the far end keeps the same and the stop from the
    /// cruise.
    #[arg(long, default_value_t = 2.5)]
    edge_pct: f64,
    /// The source check before each cruise's first burst, ms after the
    /// burst's head: a capture of `omega_hat_src` at the cruise; 0 = none.
    #[arg(long, default_value_t = 500)]
    source_ms: u32,
    #[command(flatten)]
    guarded: Guarded,
    /// One more read right behind the poll before and the poll after each
    /// step's burst: ADDR:LEN, the address hex (0x optional), 1 to 64
    /// bytes. polls.csv carries a row a poll with its bytes as lowercase
    /// hex, `extra_hex`, empty where the read failed or came back short and
    /// counted in meta.json (`extra_poll_failures`); the run goes on either
    /// way. A poll the watchdog trips on takes no extra read.
    #[arg(long, value_name = "ADDR:LEN", value_parser = parse_extra)]
    poll_extra: Option<Extra>,
}

/// A comma-separated list as a flag takes it; `none` is the empty one.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct List<T>(pub(crate) Vec<T>);

impl<T: std::str::FromStr> List<T> {
    fn parse(s: &str) -> Result<Self, String> {
        if s == "none" {
            return Ok(List(Vec::new()));
        }
        s.split(',')
            .map(str::trim)
            .filter(|v| !v.is_empty())
            .map(|v| {
                v.parse()
                    .map_err(|_| format!("{v:?} is not a whole number"))
            })
            .collect::<Result<_, _>>()
            .map(List)
    }
}

/// The loop a point reads.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Band {
    Speed,
    Position,
}

impl Band {
    fn as_str(self) -> &'static str {
        match self {
            Band::Speed => "speed",
            Band::Position => "position",
        }
    }
}

/// What a step captures.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Kind {
    /// `omega_hat_src` at the cruise, no stimulus.
    Source,
    /// One burst of a point: its band, N, slope (Q8.8 c/s per period) and
    /// the cruise of the reading it belongs to, its own or the one a
    /// two-cruise check checks.
    Burst {
        band: Band,
        n: u16,
        slope_q88: u16,
        reading_cps: u32,
    },
}

/// One capture of the sweep, every number decided before torque.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct Step {
    pub(crate) kind: Kind,
    pub(crate) cruise: u32,
    pub(crate) direction: i32,
    /// Linearized counts.
    pub(crate) start: i32,
    /// Medium periods, the head included.
    pub(crate) periods: u32,
    /// Whole cycles of the triangle after the head; 0 for a source check.
    pub(crate) cycles: u32,
    /// The most the shaft can travel in it, counts.
    pub(crate) travel: f64,
}

impl Step {
    pub(crate) fn rows(&self) -> u32 {
        self.periods * DECIM as u32
    }

    fn label(&self) -> String {
        match self.kind {
            Kind::Source => format!("source check at {} c/s", self.cruise),
            Kind::Burst { band, n, .. } => {
                format!("{} N={n} at {} c/s", band.as_str(), self.cruise)
            }
        }
    }

    fn json(&self, tick_hz: f64) -> serde_json::Value {
        let mut m = serde_json::json!({
            "kind": match self.kind { Kind::Source => "source", Kind::Burst { .. } => "burst" },
            "cruise_cps": self.cruise,
            "direction": self.direction,
            "start": self.start,
            "periods": self.periods,
            "rows": self.rows(),
            "seconds": self.rows() as f64 / tick_hz,
            "travel_counts": self.travel.round(),
        });
        if let Kind::Burst {
            band,
            n,
            slope_q88,
            reading_cps,
        } = self.kind
        {
            m["band"] = band.as_str().into();
            m["inject_half_periods"] = n.into();
            m["inject_accel_q88"] = slope_q88.into();
            m["peak_cps"] = peak_cps(slope_q88, n).into();
            m["reading_cps"] = reading_cps.into();
            m["cycles"] = self.cycles.into();
        }
        m
    }
}

/// The triangle's largest excursion from the cruise, c/s: the kernel's
/// `(N >> 1) - |p - N|` reaches N / 2 either way for an even N and
/// (N + 1) / 2 below the cruise for an odd one.
fn peak_cps(slope_q88: u16, n: u16) -> f64 {
    slope_q88 as f64 / 256.0 * (n - n / 2) as f64
}

/// The plan as asked, before the servo's numbers.
#[derive(Clone, Debug)]
pub(crate) struct Ask {
    pub(crate) speed_n: Vec<u16>,
    pub(crate) position_n: Vec<u16>,
    pub(crate) speed_cruise_cps: Vec<u32>,
    pub(crate) position_cruise_cps: Vec<u32>,
    /// The two-cruise check's factor, None for none.
    pub(crate) second_cruise: Option<f64>,
    pub(crate) slope_cps: f64,
    pub(crate) peak_cps: f64,
    pub(crate) repeats: u32,
    pub(crate) min_cycles: u32,
    pub(crate) edge_pct: f64,
    pub(crate) source_ms: u32,
}

impl Ask {
    fn of(a: &Args) -> Self {
        Self {
            speed_n: a.speed_n.0.clone(),
            position_n: a.position_n.0.clone(),
            speed_cruise_cps: a.speed_cruise_cps.0.clone(),
            position_cruise_cps: a.position_cruise_cps.0.clone(),
            second_cruise: (!a.no_second_cruise).then_some(a.second_cruise),
            slope_cps: a.slope_cps,
            peak_cps: a.peak_cps,
            repeats: a.repeats,
            min_cycles: a.min_cycles,
            edge_pct: a.edge_pct,
            source_ms: a.source_ms,
        }
    }
}

/// The servo's numbers a plan is made of.
#[derive(Copy, Clone, Debug)]
pub(crate) struct Bounds {
    /// Linearized counts the bursts stay inside.
    pub(crate) guard: (i32, i32),
    pub(crate) speed_ceiling_cps: u16,
    /// Q8.8 c/s per medium period: the cruise's run-up and its stop.
    pub(crate) accel_limit_q88: u16,
    pub(crate) tick_hz: f64,
    /// The longest burst the servo arms, rows.
    pub(crate) cap_rows: u16,
    /// The servo lists `omega_hat_src`, so a source check can run.
    pub(crate) source: bool,
}

/// The sweep in capture order and what it was made from.
#[derive(Clone, Debug)]
pub(crate) struct Plan {
    pub(crate) steps: Vec<Step>,
    /// Counts the starts sit inside the guard.
    pub(crate) edge: i32,
    pub(crate) head_periods: u32,
}

#[cfg(test)]
impl Plan {
    pub(crate) fn bursts(&self) -> impl Iterator<Item = &Step> {
        self.steps
            .iter()
            .filter(|s| matches!(s.kind, Kind::Burst { .. }))
    }
}

/// The slope for half-period `n`, Q8.8 c/s per period: the sim's rule,
/// at most `slope_cps` and a peak of at most `peak_cps`.
pub(crate) fn slope_q88(n: u16, slope_cps: f64, peak_cps: f64) -> u16 {
    (slope_cps.min(2.0 * peak_cps / n as f64) * 256.0)
        .floor()
        .clamp(0.0, u16::MAX as f64) as u16
}

/// The sweep: each speed cruise's speed band, each position cruise's
/// position band, then the position band at the two-cruise check's cruise,
/// a source check ahead of each cruise's first burst, every point
/// `repeats` pairs of bursts with the direction alternating over the
/// whole sweep. A burst is the head and the most whole cycles that fit
/// both the travel and one TEL burst; anything that does not is refused
/// here, in words.
pub(crate) fn plan(ask: &Ask, b: &Bounds) -> Result<Plan, String> {
    let fm = b.tick_hz / DECIM as f64;
    if fm <= 0.0 {
        return Err("the servo publishes no tick rate".into());
    }
    if b.accel_limit_q88 == 0 {
        return Err(
            "the servo's accel_limit_q88 is 0: Velocity mode holds its reference at 0".into(),
        );
    }
    if ask.repeats == 0 {
        return Err("--repeats must be at least 1".into());
    }
    if !(ask.edge_pct > 0.0 && ask.edge_pct < 50.0) {
        return Err(format!(
            "--edge-pct {} is not over 0 and under 50",
            ask.edge_pct
        ));
    }
    let accel = b.accel_limit_q88 as f64 / 256.0 * fm;
    let (lo, hi) = b.guard;
    let span = (hi - lo) as f64;
    let edge = (ask.edge_pct / 100.0 * span).ceil() as i32;
    let head = (HEAD_S * fm).round() as u32;
    let tel_periods = b.cap_rows as u32 / DECIM as u32;

    let mut points: Vec<(Band, u32, u32)> = Vec::new();
    for &c in &ask.speed_cruise_cps {
        points.push((Band::Speed, c, c));
    }
    for &c in &ask.position_cruise_cps {
        points.push((Band::Position, c, c));
    }
    if let (Some(f), Some(&first)) = (ask.second_cruise, ask.position_cruise_cps.first()) {
        if f.is_nan() || f <= 0.0 || f == 1.0 {
            return Err(format!(
                "--second-cruise {f} is not a factor over 0 and other than 1"
            ));
        }
        points.push((Band::Position, (first as f64 * f).round() as u32, first));
    }

    let mut steps = Vec::new();
    let mut checked: Vec<u32> = Vec::new();
    let mut dir = 1;
    let mut push = |kind: Kind, cruise: u32, n: u16, slope: u16, steps: &mut Vec<Step>| {
        let c = cruise as f64;
        let swing = peak_cps(slope, n) * n as f64 / (4.0 * fm);
        let stop = c * c / (2.0 * accel) + c * REACT_S;
        let room = span - 2.0 * edge as f64 - stop - swing;
        let fits = ((room / c * fm).floor().max(0.0) as u32).min(tel_periods);
        let (periods, cycles) = match kind {
            Kind::Source => {
                let p = head + (ask.source_ms as f64 / 1000.0 * fm).round() as u32;
                if p > fits {
                    return Err(format!(
                        "the source check at {cruise} c/s takes {p} periods, and {:.0} counts \
                         of travel inside the guard {lo}..{hi} and one TEL burst fit {fits}",
                        room.max(0.0)
                    ));
                }
                (p, 0)
            }
            Kind::Burst { .. } => {
                let cycle = 2 * n as u32;
                let cycles = fits.saturating_sub(head) / cycle;
                if cycles < ask.min_cycles {
                    return Err(format!(
                        "N={n} at {cruise} c/s: {cycles} whole cycles fit one burst ({fits} \
                         periods: {:.0} counts of travel inside the guard {lo}..{hi} less \
                         {edge} each end and {stop:.0} to stop, and one TEL burst's {} rows), \
                         under the {} asked",
                        room.max(0.0),
                        b.cap_rows,
                        ask.min_cycles
                    ));
                }
                (head + cycles * cycle, cycles)
            }
        };
        steps.push(Step {
            kind,
            cruise,
            direction: dir,
            start: if dir > 0 { lo + edge } else { hi - edge },
            periods,
            cycles,
            travel: c * periods as f64 / fm + swing,
        });
        dir = -dir;
        Ok(())
    };
    for (band, cruise, reading_cps) in points {
        let band_n = match band {
            Band::Speed => &ask.speed_n,
            Band::Position => &ask.position_n,
        };
        if band_n.is_empty() {
            continue;
        }
        if cruise == 0 {
            return Err("a cruise of 0 c/s carries no margin row".into());
        }
        let mut kinds = Vec::with_capacity(band_n.len());
        for &n in band_n {
            if n == 0 || n > INJECT_HALF_PERIODS_MAX {
                return Err(format!(
                    "N={n}: the servo reads inject_half_periods from 1 to {INJECT_HALF_PERIODS_MAX}"
                ));
            }
            let slope = slope_q88(n, ask.slope_cps, ask.peak_cps);
            if slope == 0 || slope > INJECT_ACCEL_MAX_Q88 {
                return Err(format!(
                    "N={n}: the slope {slope} (Q8.8 c/s a period) is outside 1..{INJECT_ACCEL_MAX_Q88}"
                ));
            }
            let top = cruise as f64 + peak_cps(slope, n);
            if top > b.speed_ceiling_cps as f64 {
                return Err(format!(
                    "N={n} at {cruise} c/s: the cruise and the triangle's peak reach {top:.0} \
                     c/s, over the servo's speed_ceiling_cps {}, where the clamp would cut it",
                    b.speed_ceiling_cps
                ));
            }
            kinds.push((
                Kind::Burst {
                    band,
                    n,
                    slope_q88: slope,
                    reading_cps,
                },
                n,
                slope,
            ));
        }
        if b.source && ask.source_ms > 0 && !checked.contains(&cruise) {
            push(Kind::Source, cruise, 0, 0, &mut steps)?;
            checked.push(cruise);
        }
        for (kind, n, slope) in kinds {
            for _ in 0..2 * ask.repeats {
                push(kind, cruise, n, slope, &mut steps)?;
            }
        }
    }
    if !steps.iter().any(|s| matches!(s.kind, Kind::Burst { .. })) {
        return Err("the plan holds no burst".into());
    }
    Ok(Plan {
        steps,
        edge,
        head_periods: head,
    })
}

/// The plan as the run prints it.
pub(crate) fn print_plan(plan: &Plan, b: &Bounds) {
    let fm = b.tick_hz / DECIM as f64;
    println!(
        "margin plan: guard {}..{}, starts {} counts inside, speed ceiling {} c/s, accel {} c/s \
         a period, {} Hz medium rate, {} head periods",
        b.guard.0,
        b.guard.1,
        plan.edge,
        b.speed_ceiling_cps,
        b.accel_limit_q88 as f64 / 256.0,
        fm,
        plan.head_periods
    );
    println!(
        "{:>4}  {:<8}  {:>4}  {:>6}  {:>5}  {:>6}  {:>7}  {:>3}  {:>5}  {:>7}  {:>6}  {:>5}  {:>6}",
        "step",
        "band",
        "N",
        "slope",
        "peak",
        "cruise",
        "reading",
        "dir",
        "start",
        "periods",
        "cycles",
        "s",
        "travel"
    );
    for (k, s) in plan.steps.iter().enumerate() {
        let (band, n, slope, peak, reading) = match s.kind {
            Kind::Source => ("source", "-".into(), "-".into(), "-".into(), "-".into()),
            Kind::Burst {
                band,
                n,
                slope_q88,
                reading_cps,
            } => (
                band.as_str(),
                n.to_string(),
                slope_q88.to_string(),
                format!("{:.0}", peak_cps(slope_q88, n)),
                reading_cps.to_string(),
            ),
        };
        println!(
            "{k:>4}  {band:<8}  {n:>4}  {slope:>6}  {peak:>5}  {:>6}  {reading:>7}  {:>3}  {:>5}  \
             {:>7}  {:>6}  {:>5.2}  {:>6.0}",
            s.cruise,
            if s.direction > 0 { "+" } else { "-" },
            s.start,
            s.periods,
            s.cycles,
            s.periods as f64 / fm,
            s.travel
        );
    }
}

/// A point's burst as it came back.
pub(crate) struct Burst {
    /// K in burst-K, capture order.
    pub(crate) index: usize,
    pub(crate) step: Step,
    pub(crate) frames: Vec<TelFrame>,
    pub(crate) stats: BurstStats,
    pub(crate) p_kp_q88: u16,
    pub(crate) t_start: f64,
    pub(crate) t_end: f64,
}

/// A burst's facts, `oscnb.margin.FACTS` first, then the tool's own.
pub(crate) fn facts(b: &Burst) -> serde_json::Value {
    let (band, n, slope, reading) = match b.step.kind {
        Kind::Burst {
            band,
            n,
            slope_q88,
            reading_cps,
        } => (band.as_str(), n, slope_q88, reading_cps),
        Kind::Source => ("source", 0, 0, b.step.cruise),
    };
    serde_json::json!({
        "inject_half_periods": n,
        "inject_accel_q88": slope,
        "cruise_cps": b.step.cruise,
        "direction": b.step.direction,
        "p_kp_q88": b.p_kp_q88,
        "rows_dropped": b.stats.rows_dropped,
        "seq_holes": b.stats.holes,
        "reading_cps": reading,
        "band": band,
        "start": b.step.start,
        "periods": b.step.periods,
        "cycles": b.step.cycles,
        "rows": b.step.rows(),
        "rows_received": b.frames.len(),
        "tick0": b.stats.stamp0,
        "garble_bytes": b.stats.garble,
        "t_start": b.t_start,
        "t_end": b.t_end,
    })
}

/// burst-K.csv: one row a TEL row, `tick` the row's `sample_tick`, low 16
/// bits, then the three fields raw.
pub(crate) fn write_rows(w: &mut impl Write, b: &Burst) -> Result<()> {
    writeln!(w, "{COLUMNS}")?;
    let stamp0 = b.stats.stamp0.unwrap_or(0) as u64;
    let v = |x: Option<i32>| x.map_or(String::new(), |x| x.to_string());
    for f in &b.frames {
        writeln!(
            w,
            "{},{},{},{}",
            (stamp0 + f.tick) & 0xFFFF,
            v(f.theta_hat_q16),
            v(f.omega_hat_cps),
            v(f.omega_ref_cps)
        )?;
    }
    Ok(())
}

/// burst-K.csv and burst-K.json in `dir`.
pub(crate) fn write_burst(dir: &Path, b: &Burst) -> Result<()> {
    let csv = dir.join(format!("burst-{}.csv", b.index));
    let mut w = std::io::BufWriter::new(
        std::fs::File::create(&csv).with_context(|| format!("create {}", csv.display()))?,
    );
    write_rows(&mut w, b)?;
    w.flush()?;
    let json = dir.join(format!("burst-{}.json", b.index));
    std::fs::write(&json, serde_json::to_string_pretty(&facts(b))? + "\n")
        .with_context(|| format!("write {}", json.display()))
}

/// One row a period from the first row, after `head` periods: the rows at
/// the first row's offset in its period.
fn per_period(frames: &[TelFrame], head: u32) -> impl Iterator<Item = &TelFrame> {
    let phase = frames.first().map_or(0, |f| f.tick % DECIM);
    let from = frames.first().map_or(0, |f| f.tick) + head as u64 * DECIM;
    frames
        .iter()
        .filter(move |f| f.tick % DECIM == phase && f.tick >= from)
}

/// Period steps of the reference after the head, and those of them at the
/// triangle's slope either way: a live stimulus moves the reference by
/// the slope every period it injects.
pub(crate) fn stimulus_steps(frames: &[TelFrame], slope_q88: u16, head: u32) -> (u32, u32) {
    let step = (slope_q88 as i64) << 8;
    let rows: Vec<(u64, i64)> = per_period(frames, head)
        .filter_map(|f| Some((f.tick, f.omega_ref_cps? as i64)))
        .collect();
    let (mut pairs, mut at) = (0, 0);
    for w in rows.windows(2) {
        if w[1].0 - w[0].0 == DECIM {
            pairs += 1;
            at += ((w[1].1 - w[0].1).abs() == step) as u32;
        }
    }
    (pairs, at)
}

/// The share of periods after the head at each `omega_hat_src` value.
pub(crate) fn source_share(frames: &[TelFrame], head: u32) -> (u32, BTreeMap<u8, f64>) {
    let mut counts: BTreeMap<u8, u32> = BTreeMap::new();
    for f in per_period(frames, head) {
        if let Some(v) = f.omega_hat_src {
            *counts.entry(v).or_default() += 1;
        }
    }
    let n: u32 = counts.values().sum();
    let share = counts
        .into_iter()
        .map(|(v, k)| (v, k as f64 / n.max(1) as f64))
        .collect();
    (n, share)
}

/// polls.csv's columns: the step, the K of the burst it captures (empty
/// for a source check), `before` or `after` its burst, the host's time of
/// the poll on `t_start`'s clock, and `--poll-extra`'s bytes.
pub(crate) const POLLS_COLUMNS: &str = "step,burst,poll,t,extra_hex";

/// One polls.csv row.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PollRow {
    pub(crate) step: usize,
    pub(crate) burst: Option<usize>,
    pub(crate) at: &'static str,
    pub(crate) t: f64,
    pub(crate) extra_hex: String,
}

/// What the run reports beside its bursts.
#[derive(Default)]
pub(crate) struct Swept {
    pub(crate) ran: Ran,
    /// Each source check, in capture order.
    pub(crate) sources: Vec<serde_json::Value>,
    pub(crate) bursts: usize,
    /// The stimulus fields as read back once the run ended.
    pub(crate) inject_after: Option<[u16; 2]>,
    /// `--poll-extra`'s reads, in capture order; empty without it.
    pub(crate) polls: Vec<PollRow>,
}

impl Swept {
    /// `--poll-extra`'s read behind step `k`'s poll `at` its burst.
    fn poll_extra<S: Servo>(
        &mut self,
        s: &mut S,
        x: Extra,
        k: usize,
        step: &Step,
        at: &'static str,
        anchor: f64,
    ) {
        let t = anchor + s.now_ms() / 1000.0;
        let extra_hex = poll_extra(s, x, &mut self.ran);
        self.polls.push(PollRow {
            step: k,
            burst: matches!(step.kind, Kind::Burst { .. }).then_some(self.bursts),
            at,
            t,
            extra_hex,
        });
    }
}

/// polls.csv in `dir`.
pub(crate) fn write_polls(dir: &Path, polls: &[PollRow]) -> Result<()> {
    let p = dir.join("polls.csv");
    let mut w = std::io::BufWriter::new(
        std::fs::File::create(&p).with_context(|| format!("create {}", p.display()))?,
    );
    writeln!(w, "{POLLS_COLUMNS}")?;
    for r in polls {
        let burst = r.burst.map_or(String::new(), |k| k.to_string());
        writeln!(w, "{},{burst},{},{:.3},{}", r.step, r.at, r.t, r.extra_hex)?;
    }
    w.flush()?;
    Ok(())
}

/// The servo's numbers and the plan, every refusal made.
pub(crate) struct Sweep {
    pub(crate) plan: Plan,
    pub(crate) bounds: Bounds,
    pub(crate) p_kp_q88: u16,
}

/// A field's value by its descriptor name.
fn read_named<S: Servo>(s: &mut S, d: &Descriptor, name: &str) -> Result<i64> {
    let f = crate::descriptor::field(d, name)?;
    let id = s.id();
    let raw = s
        .client()
        .read(id, f.addr, f.width)
        .with_context(|| format!("read {name}"))?;
    Ok(match desc::decode(f, &raw)? {
        Value::Uint(n) => n as i64,
        Value::Int(n) => n,
        Value::Bool(b) => b as i64,
        Value::Enum(n) => n as i64,
        Value::Bytes(_) => anyhow::bail!("{name} is not a scalar"),
    })
}

/// Read the servo and refuse, before any torque, every sweep it cannot
/// take: what any closed-loop run refuses, a sweep with no dead-man, an
/// image whose descriptor does not list the stimulus fields, and a plan
/// that does not fit the servo's limits, travel or TEL burst.
pub(crate) fn prepare<S: Servo>(s: &mut S, a: &Args) -> Result<(Cfg, Sweep)> {
    if a.guarded.deadman_ms == 0 {
        return refuse("the stimulus runs only under a loaded bus dead-man, and --deadman-ms is 0");
    }
    let mut cfg = preflight(s, &a.guarded, Mode::Velocity, ROW)?;
    cfg.extra = a.poll_extra;
    let id = s.id();
    let d = crate::state::descriptor(s.client(), id)?;
    let listed = |r: crate::instrument::regs::Reg| {
        d.fields
            .iter()
            .any(|f| f.addr == r.addr && f.width == r.width as u16)
    };
    if !cfg.inject || !listed(telemetry::OMEGA_REF_CPS) {
        return refuse(
            "the servo does not list the margin stimulus (inject_accel_q88, inject_half_periods, \
             omega_ref_cps; protocol sec 5.6): its firmware predates it",
        );
    }
    let cap = s.burst_cap()?;
    let bounds = Bounds {
        guard: cfg.wd.guard,
        speed_ceiling_cps: read_named(s, &d, "speed_ceiling_cps")? as u16,
        accel_limit_q88: read_named(s, &d, "accel_limit_q88")? as u16,
        tick_hz: cfg.tick_hz,
        cap_rows: cap.rows,
        source: listed(telemetry::OMEGA_HAT_SRC),
    };
    let p_kp_q88 = read_named(s, &d, "p_kp_q88")? as u16;
    let plan = plan(&Ask::of(a), &bounds).or_else(refuse)?;
    standing(s, &cfg.wd)?;
    Ok((
        cfg,
        Sweep {
            plan,
            bounds,
            p_kp_q88,
        },
    ))
}

/// Zero the stimulus fields and read them back, best effort: the last
/// thing a run does to the servo, on every exit.
fn zero_and_read<S: Servo>(s: &mut S, cfg: &Cfg, sw: &mut Swept) -> Result<()> {
    let zeroed = zero_inject(s, cfg);
    let id = s.id();
    let c = s.client();
    let back = (|| -> Result<[u16; 2]> {
        Ok([
            read_u16(c, id, control::INJECT_ACCEL_Q88)?,
            read_u16(c, id, control::INJECT_HALF_PERIODS)?,
        ])
    })();
    match back {
        Ok(v) => {
            sw.inject_after = Some(v);
            if v != [0, 0] {
                eprintln!(
                    "warning: id {}: the stimulus fields read {} and {} after the run, not 0: \
                     zero inject_accel_q88 and inject_half_periods before the next capture",
                    id.as_byte(),
                    v[0],
                    v[1]
                );
            }
            zeroed
        }
        Err(e) => zeroed.and(Err(e)),
    }
}

/// One step: the move to its start, the fields, Velocity mode, the burst
/// at the cruise, then the stop and the fields zeroed whatever the burst
/// returned; the rows and the poll after them judged before the burst is
/// kept, so a fault latched under it keeps it out of the folder. A burst
/// whose rows carry no triangle where the sweep's first burst should is
/// refused: the image ignores the fields.
#[allow(clippy::too_many_arguments)]
fn run_step<S: Servo>(
    s: &mut S,
    cfg: &mut Cfg,
    sweep: &Sweep,
    lease: &mut Lease,
    k: usize,
    anchor: f64,
    sw: &mut Swept,
    on_burst: &mut impl FnMut(&Burst) -> Result<()>,
) -> Result<()> {
    stopped()?;
    lease.keep(s)?;
    let step = sweep.plan.steps[k];
    if let Err(pos) = go_to(s, cfg, lease, step.start)? {
        return trip(format!(
            "the shaft read {pos} counts after {} s, not at the start {}",
            track::CENTER_POLLS * track::POLL_MS / 1000,
            step.start
        ));
    }
    let (inject, chunks) = match step.kind {
        Kind::Burst { n, slope_q88, .. } => (slope_q88 as i32 | (n as i32) << 16, ROW),
        Kind::Source => (0, SOURCE_ROW),
    };
    s.write(control::INJECT, inject)?;
    s.write(control::GOAL_VELOCITY, 0)?;
    s.write(control::MODE, Mode::Velocity.value())?;
    lease.keep(s)?;
    let poll: Poll = watch(s, &mut cfg.wd)?;
    if let Some(x) = cfg.extra {
        sw.poll_extra(s, x, k, &step, "before", anchor);
    }
    let t_start = anchor + s.now_ms() / 1000.0;
    let goal = Some((control::GOAL_VELOCITY, step.direction * step.cruise as i32));
    let burst = s.stream_at(step.rows(), goal, chunks, None, None);
    let stop = s.write(control::GOAL_VELOCITY, 0);
    let zero = s.write(control::INJECT, 0);
    let (frames, stats) = burst?;
    stop?;
    zero?;
    let t_end = anchor + s.now_ms() / 1000.0;
    sw.ran.rows_dropped = sw.ran.rows_dropped.saturating_add(stats.rows_dropped);
    let got = frames.len() as u32 + stats.rows_dropped as u32;
    if got < step.rows() {
        return Err(rows_tripped(
            s,
            format!(
                "the burst came back short: {} of {} rows ({} seq holes, {} garble bytes)",
                frames.len(),
                step.rows(),
                stats.holes,
                stats.garble
            ),
        ));
    }
    if let Err(reason) = cfg.wd.rows(&frames, poll.current_bias_counts) {
        return Err(rows_tripped(s, reason));
    }
    watch(s, &mut cfg.wd)?;
    if let Some(x) = cfg.extra {
        sw.poll_extra(s, x, k, &step, "after", anchor);
    }
    let head = sweep.plan.head_periods;
    match step.kind {
        Kind::Source => {
            let (periods, share) = source_share(&frames, head);
            let text: Vec<String> = share
                .iter()
                .map(|(v, x)| format!("{v} for {:.1} %", 100.0 * x))
                .collect();
            println!(
                "  source check at {} c/s: omega_hat_src {} of {periods} periods",
                step.cruise,
                text.join(", ")
            );
            sw.sources.push(serde_json::json!({
                "cruise_cps": step.cruise,
                "periods": periods,
                "share": share.iter().map(|(v, x)| (v.to_string(), *x)).collect::<BTreeMap<_, _>>(),
            }));
        }
        Kind::Burst { slope_q88, .. } => {
            if sw.bursts == 0 {
                let (pairs, at) = stimulus_steps(&frames, slope_q88, head);
                if pairs == 0 || 2 * at < pairs {
                    return refuse(format!(
                        "the first burst's velocity reference moved by the slope on {at} of its \
                         {pairs} period steps: the image accepts the stimulus fields and ignores \
                         them, so it is not a `deadman` build (scripts/build-image.sh --deadman)"
                    ));
                }
            }
            let b = Burst {
                index: sw.bursts,
                step,
                frames,
                stats,
                p_kp_q88: sweep.p_kp_q88,
                t_start,
                t_end,
            };
            on_burst(&b)?;
            sw.bursts += 1;
        }
    }
    Ok(())
}

/// Torque on at a zero goal in Velocity mode, the dead-man check, every
/// step, the centering, torque off; then the fields zeroed and read back
/// on every exit, and the refused modes read. Each burst reaches `on_burst`
/// as it lands, so an abort keeps what was captured before it.
pub(crate) fn record<S: Servo>(
    s: &mut S,
    cfg: &mut Cfg,
    sweep: &Sweep,
    sw: &mut Swept,
    mut on_burst: impl FnMut(&Burst) -> Result<()>,
) -> Result<()> {
    let mut lease = Lease::deadman(cfg.deadman_ticks);
    let anchor = unix_s() - s.now_ms() / 1000.0;
    let mut no_seg = |_: &track::Seg| Ok(());
    let r = guard(s, |s| {
        zero_inject(s, cfg)?;
        s.write(control::MODE, Mode::Velocity.value())?;
        s.write(control::GOAL_VELOCITY, 0)?;
        lease.torque_on(s)?;
        if cfg.deadman_check {
            let at = "the dead-man check".to_string();
            sw.ran.deadman = Some(
                check_deadman(s, cfg, &mut lease)
                    .map_err(|e| halt(s, &mut lease, None, at, e, &mut no_seg))?,
            );
        }
        for k in 0..sweep.plan.steps.len() {
            if let Err(e) = run_step(s, cfg, sweep, &mut lease, k, anchor, sw, &mut on_burst) {
                let at = format!("step {k} ({})", sweep.plan.steps[k].label());
                let e = halt(s, &mut lease, Some(k), at, e, &mut no_seg);
                sw.ran.set = read_set(s).ok();
                return Err(e);
            }
        }
        sw.ran.set = Some(read_set(s)?);
        if let Some(at) = cfg.center
            && let Err(e) = center(s, cfg, &mut lease, at)
        {
            let where_ = "the centering".to_string();
            return Err(halt(s, &mut lease, None, where_, e, &mut no_seg));
        }
        lease.write(s, control::TORQUE_ENABLE, 0)
    });
    let zeroed = zero_and_read(s, cfg, sw);
    let restored = put_back(s, cfg, &mut sw.ran);
    r.and(zeroed).and(restored)
}

/// The sweep's meta before it drives: the servo, the plan, the limits and
/// guards it runs under.
fn meta<S: Servo>(s: &mut S, cfg: &Cfg, sweep: &Sweep, a: &Args) -> Result<serde_json::Value> {
    let id = s.id();
    let servo = servo_meta(s.client(), id)?;
    let b = &sweep.bounds;
    let mut m = serde_json::json!({
        "experiment": "margin",
        "row": chunks_json(&ROW),
        "source_row": chunks_json(&SOURCE_ROW),
        "plan": {
            "speed_n": a.speed_n.0,
            "position_n": a.position_n.0,
            "speed_cruise_cps": a.speed_cruise_cps.0,
            "position_cruise_cps": a.position_cruise_cps.0,
            "second_cruise": (!a.no_second_cruise).then_some(a.second_cruise),
            "slope_cps": a.slope_cps,
            "peak_cps": a.peak_cps,
            "repeats": a.repeats,
            "min_cycles": a.min_cycles,
            "edge_pct": a.edge_pct,
            "edge_counts": sweep.plan.edge,
            "head_periods": sweep.plan.head_periods,
            "source_ms": a.source_ms,
            "steps": sweep.plan.steps.iter().map(|st| st.json(b.tick_hz)).collect::<Vec<_>>(),
        },
        "limits": {
            "soft": [cfg.soft.0, cfg.soft.1],
            "guard": [cfg.wd.guard.0, cfg.wd.guard.1],
            "speed_ceiling_cps": b.speed_ceiling_cps,
            "accel_limit_q88": b.accel_limit_q88,
            "current_limit_counts": cfg.i_lim,
            "p_kp_q88": sweep.p_kp_q88,
            "tel_rows_max": b.cap_rows,
        },
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
        "source_checks": if b.source {
            serde_json::json!([])
        } else {
            "the servo does not list omega_hat_src".into()
        },
        "bursts": 0,
        "rows_dropped": 0,
        "config": serde_json::Value::Null,
        "control": serde_json::Value::Null,
        "inject_after": serde_json::Value::Null,
        "restored": [],
        "refused_modes_after": serde_json::Value::Null,
        "ended": serde_json::Value::Null,
    });
    meta_extra(&mut m, cfg.extra, 0);
    if let Some(o) = m.as_object_mut() {
        o.extend(servo);
    }
    Ok(m)
}

/// The meta once the sweep ended.
fn land(meta: &mut serde_json::Value, cfg: &Cfg, sweep: &Sweep, sw: &Swept, r: &Result<()>) {
    meta["bursts"] = sw.bursts.into();
    meta["rows_dropped"] = sw.ran.rows_dropped.into();
    meta_extra(meta, cfg.extra, sw.ran.extra_failures);
    meta["deadman"]["check"] = sw.ran.deadman.map_or(serde_json::Value::Null, Into::into);
    if sweep.bounds.source {
        meta["source_checks"] = sw.sources.clone().into();
    }
    if let Some(set) = &sw.ran.set {
        meta["config"] = set["config"].clone();
        meta["control"] = set["control"].clone();
    }
    meta["inject_after"] = sw.inject_after.map_or(
        serde_json::Value::Null,
        |v| serde_json::json!({ "inject_accel_q88": v[0], "inject_half_periods": v[1] }),
    );
    meta["restored"] = sw
        .ran
        .restored
        .iter()
        .map(|k| serde_json::json!({ "field": k.field, "from": k.last, "to": k.before }))
        .collect::<Vec<_>>()
        .into();
    meta["refused_modes_after"] = sw
        .ran
        .refused_modes_after
        .as_ref()
        .map_or(serde_json::Value::Null, |m| m.clone().into());
    meta["ended"] = match r {
        Ok(()) => "ok".into(),
        Err(e) => match e.downcast_ref::<Abort>() {
            Some(a) => serde_json::json!({
                "reason": a.reason,
                "step": a.seg,
                "at": a.at,
                "fault": a.fault.map_or(serde_json::Value::Null, Fault::json),
            }),
            None => serde_json::json!({ "reason": format!("{e:#}"), "step": null, "fault": null }),
        },
    };
}

/// Prepare against the servo, print and write the plan, run it writing
/// each burst as it lands, and land the meta however it ended.
pub(crate) fn capture<S: Servo>(s: &mut S, a: &Args, out: &Path) -> Result<()> {
    let (mut cfg, sweep) = prepare(s, a)?;
    print_plan(&sweep.plan, &sweep.bounds);
    std::fs::create_dir_all(out).with_context(|| format!("mkdir {}", out.display()))?;
    let mut meta = meta(s, &cfg, &sweep, a)?;
    let meta_path = out.join("meta.json");
    let write_meta = |m: &serde_json::Value| -> Result<()> {
        std::fs::write(&meta_path, serde_json::to_string_pretty(m)?)
            .with_context(|| format!("write {}", meta_path.display()))
    };
    write_meta(&meta)?;
    let mut sw = Swept::default();
    let r = record(s, &mut cfg, &sweep, &mut sw, |b| {
        println!(
            "  burst {} ({}): {} of {} rows, {} dropped, {} seq holes, {} garble bytes",
            b.index,
            b.step.label(),
            b.frames.len(),
            b.step.rows(),
            b.stats.rows_dropped,
            b.stats.holes,
            b.stats.garble
        );
        write_burst(out, b)
    });
    let polls = match cfg.extra {
        Some(_) => write_polls(out, &sw.polls),
        None => Ok(()),
    };
    land(&mut meta, &cfg, &sweep, &sw, &r);
    write_meta(&meta)?;
    polls?;
    if let Some(v) = sw.inject_after {
        println!(
            "stimulus fields after the run: inject_accel_q88 {}, inject_half_periods {}",
            v[0], v[1]
        );
    }
    if sw.ran.rows_dropped > 0 {
        println!(
            "warning: the servo dropped {} TEL rows (both stream buffers were waiting for the \
             wire); the bursts are saved with those rows missing",
            sw.ran.rows_dropped
        );
    }
    r?;
    println!("margin: {} bursts in {}", sw.bursts, out.display());
    println!("meta:   {}", meta_path.display());
    Ok(())
}

/// Entry from the top-level `osc-lab margin` dispatch.
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
    use crate::rig::pump::BurstCap;
    use crate::rig::testkit::{Rig, Sent};
    use crate::track::exit_code;

    #[derive(clap::Parser)]
    struct Cli {
        #[command(flatten)]
        args: Args,
    }

    fn parse(argv: &[&str]) -> Args {
        let mut v = vec!["margin", "--out", "unused"];
        v.extend_from_slice(argv);
        <Cli as clap::Parser>::try_parse_from(v).unwrap().args
    }

    /// A servo of soft limits 314..3560, speed ceiling 4025 c/s and the
    /// default accel limit, on a 20 kHz tick with the whole TEL burst.
    fn servo() -> Bounds {
        Bounds {
            guard: (314, 3560),
            speed_ceiling_cps: 4025,
            accel_limit_q88: 3840,
            tick_hz: 20_000.0,
            cap_rows: u16::MAX,
            source: true,
        }
    }

    fn burst_n(s: &Step) -> Option<(Band, u16, u16, u32)> {
        match s.kind {
            Kind::Burst {
                band,
                n,
                slope_q88,
                reading_cps,
            } => Some((band, n, slope_q88, reading_cps)),
            Kind::Source => None,
        }
    }

    /// Every step travels inside the guard, its far end short of the far
    /// edge by the stop, in whole cycles inside one TEL burst.
    fn fits(p: &Plan, b: &Bounds) {
        let fm = b.tick_hz / DECIM as f64;
        let accel = b.accel_limit_q88 as f64 / 256.0 * fm;
        for s in &p.steps {
            let c = s.cruise as f64;
            let stop = c * c / (2.0 * accel) + c * REACT_S;
            let end = s.start as f64 + s.direction as f64 * s.travel;
            let (lo, hi) = (
                (b.guard.0 + p.edge) as f64 + stop,
                (b.guard.1 - p.edge) as f64 - stop,
            );
            assert!(end >= lo && end <= hi, "{s:?} ends at {end}");
            assert!(s.rows() <= b.cap_rows as u32, "{s:?}");
            if let Some((_, n, _, _)) = burst_n(s) {
                assert_eq!(s.periods, p.head_periods + s.cycles * 2 * n as u32);
            }
        }
    }

    /// The default plan: a source check, the speed band at the cruise with
    /// N = 1, the position band at the cruise, then a source check and the
    /// position band at 1.25 x the cruise read as its check, each point a
    /// burst up and a burst down, the direction alternating over the whole
    /// sweep, the starts 2.5 % of the range inside it, the slope the sim's
    /// rule, every burst the head and the whole cycles one TEL burst holds.
    #[test]
    fn the_default_plan_is_the_sims_bands_on_the_servos_travel() {
        let b = servo();
        let p = plan(&Ask::of(&parse(&[])), &b).unwrap();
        print_plan(&p, &b);
        assert_eq!((p.edge, p.head_periods), (82, 400));
        let sources: Vec<(usize, u32)> = p
            .steps
            .iter()
            .enumerate()
            .filter(|(_, s)| s.kind == Kind::Source)
            .map(|(k, s)| (k, s.cruise))
            .collect();
        assert_eq!(sources, [(0, 742), (35, 928)]);
        let bursts: Vec<_> = p.steps.iter().filter_map(burst_n).collect();
        assert_eq!(bursts.len(), 2 * (9 + 8 + 8));
        let ns: Vec<u16> = bursts.iter().step_by(2).map(|b| b.1).collect();
        assert_eq!(
            ns,
            [
                40, 30, 25, 20, 15, 12, 10, 8, 1, 512, 400, 300, 200, 150, 100, 70, 50, 512, 400,
                300, 200, 150, 100, 70, 50
            ]
        );
        assert!(
            bursts[..18]
                .iter()
                .all(|b| b.0 == Band::Speed && b.3 == 742)
        );
        assert!(
            bursts[18..]
                .iter()
                .all(|b| b.0 == Band::Position && b.3 == 742)
        );
        let cruises: Vec<u32> = p.bursts().map(|s| s.cruise).collect();
        assert!(cruises[..34].iter().all(|c| *c == 742) && cruises[34..].iter().all(|c| *c == 928));
        for (k, s) in p.steps.iter().enumerate() {
            assert_eq!(s.direction, if k % 2 == 0 { 1 } else { -1 });
            assert_eq!(s.start, if s.direction > 0 { 396 } else { 3478 });
        }
        for (band, n, slope, _) in &bursts {
            let want = if *n <= 20 {
                3840
            } else {
                2 * 150 * 256 / *n as u32
            };
            assert_eq!(*slope as u32, want, "{band:?} N={n}");
        }
        // one TEL burst, 6553 periods, holds every point at both cruises
        for s in p.bursts() {
            let (_, n, _, _) = burst_n(s).unwrap();
            assert_eq!(s.cycles, (6553 - 400) / (2 * n as u32), "N={n}");
        }
        let src = p.steps[0];
        assert_eq!((src.periods, src.cycles), (1400, 0));
        fits(&p, &b);
        // the sim's check: its longest burst, 6400 periods at 1.25 x the
        // cruise, travels 2968 counts; the longest here is no longer
        let longest = p.bursts().map(|s| s.travel).fold(0.0, f64::max);
        assert!(longest > 2968.0 && longest < 3100.0, "{longest}");
    }

    /// At a fast cruise travel, not the TEL burst, bounds a burst: at 2500
    /// c/s each speed point is the head and about a second of whole cycles.
    #[test]
    fn a_fast_cruise_is_bounded_by_the_travel() {
        let b = servo();
        let a = parse(&["--speed-cruise-cps", "742,2500"]);
        let p = plan(&Ask::of(&a), &b).unwrap();
        print_plan(&p, &b);
        let fast: Vec<&Step> = p.bursts().filter(|s| s.cruise == 2500).collect();
        assert_eq!(fast.len(), 18);
        for s in &fast {
            let (band, n, _, reading) = burst_n(s).unwrap();
            assert_eq!((band, reading), (Band::Speed, 2500), "its own reading");
            let record = (s.periods - p.head_periods) as f64 / 2000.0;
            assert!(record > 0.9 && record <= 1.0, "N={n}: {record} s");
        }
        assert!(
            p.steps
                .iter()
                .any(|s| s.kind == Kind::Source && s.cruise == 2500)
        );
        fits(&p, &b);
    }

    /// What the servo cannot carry is refused at plan time, in words.
    #[test]
    fn the_plan_refuses_what_does_not_fit() {
        let b = servo();
        let refused = |argv: &[&str], b: &Bounds| plan(&Ask::of(&parse(argv)), b).unwrap_err();
        let e = refused(&["--position-cruise-cps", "3950"], &b);
        assert!(e.contains("over the servo's speed_ceiling_cps 4025"), "{e}");
        let e = refused(&["--position-cruise-cps", "3800", "--no-second-cruise"], &b);
        assert!(
            e.contains("N=512 at 3800 c/s: 1 whole cycles") && e.contains("under the 4 asked"),
            "{e}"
        );
        let e = refused(&["--min-cycles", "7"], &b);
        assert!(e.contains("N=512 at 742 c/s: 6 whole cycles"), "{e}");
        let e = refused(&["--position-n", "513"], &b);
        assert!(e.contains("from 1 to 512"), "{e}");
        let e = refused(&["--speed-n", "0"], &b);
        assert!(e.contains("from 1 to 512"), "{e}");
        let e = refused(&["--peak-cps", "0"], &b);
        assert!(e.contains("outside 1..32767"), "{e}");
        let e = refused(&["--speed-n", "none", "--position-n", "none"], &b);
        assert!(e.contains("no burst"), "{e}");
        let e = refused(&["--second-cruise", "1"], &b);
        assert!(e.contains("other than 1"), "{e}");
        let e = refused(&["--edge-pct", "50"], &b);
        assert!(e.contains("under 50"), "{e}");
        let e = refused(&["--repeats", "0"], &b);
        assert!(e.contains("at least 1"), "{e}");
        let still = Bounds {
            accel_limit_q88: 0,
            ..b
        };
        assert!(refused(&[], &still).contains("accel_limit_q88 is 0"));
        let short = Bounds {
            guard: (1000, 1500),
            ..b
        };
        assert!(refused(&[], &short).contains("source check at 742 c/s"));
        // no source field, no source check
        let blind = Bounds { source: false, ..b };
        let p = plan(&Ask::of(&parse(&[])), &blind).unwrap();
        assert!(p.steps.iter().all(|s| s.kind != Kind::Source));
        let p = plan(
            &Ask::of(&parse(&["--source-ms", "0", "--repeats", "2"])),
            &b,
        )
        .unwrap();
        assert!(p.steps.iter().all(|s| s.kind != Kind::Source));
        assert_eq!(p.bursts().count(), 4 * (9 + 8 + 8));
    }

    /// The rig's bursts are short: one TEL burst of 8000 rows, 800 periods.
    fn rig(honors: bool) -> Rig {
        let mut r = Rig::plant_at(2029, honors);
        r.cap = Some(BurstCap {
            rows: 8000,
            tick_hz: 20_000.0,
        });
        r
    }

    /// One speed point, one position point, no source check.
    const SMALL: [&str; 10] = [
        "--speed-n",
        "4",
        "--position-n",
        "50",
        "--no-second-cruise",
        "--source-ms",
        "0",
        "--no-center",
        "--deadman-ms",
        "1000",
    ];

    fn run(r: &mut Rig, argv: &[&str]) -> (Vec<(usize, usize)>, Swept, Result<()>) {
        let a = parse(argv);
        let (mut cfg, sweep) = prepare(r, &a).unwrap();
        let mut seen = Vec::new();
        let mut sw = Swept::default();
        let res = record(r, &mut cfg, &sweep, &mut sw, |b| {
            seen.push((b.index, b.frames.len()));
            Ok(())
        });
        (seen, sw, res)
    }

    /// Around every burst the two fields are written ahead of the goal's
    /// commit, after the move and before Velocity mode; once the burst is
    /// back the cruise is stopped first and the fields zeroed next. They are
    /// zeroed before torque, and once more after the guard's safe writes at
    /// the end, and read back 0.
    #[test]
    fn the_fields_ride_each_burst_and_are_zeroed_around_it() {
        let mut r = rig(true);
        r.sent();
        let (seen, sw, res) = run(&mut r, &SMALL);
        res.unwrap();
        assert_eq!(seen.len(), 4);
        assert!(seen.iter().all(|s| s.1 == 8000), "{seen:?}");
        let on = r
            .writes
            .iter()
            .position(|w| *w == (control::TORQUE_ENABLE, 1))
            .unwrap();
        assert_eq!(r.writes[0], (control::INJECT, 0), "zeroed before torque");
        assert!(on > 0);
        let fields = |n: i32, slope: i32| (control::INJECT, slope | n << 16);
        let mut k = 0;
        for (n, slope, cruise) in [
            (4, 3840, 742),
            (4, 3840, -742),
            (50, 1536, 742),
            (50, 1536, -742),
        ] {
            let at = k + r.writes[k..]
                .iter()
                .position(|w| *w == fields(n, slope))
                .unwrap();
            let start = if cruise > 0 { 512 } else { 3546 };
            let goal = k + r.writes[k..at]
                .iter()
                .rposition(|w| w.0 == control::GOAL_POSITION)
                .unwrap();
            assert_eq!(r.writes[goal], (control::GOAL_POSITION, start));
            assert_eq!(r.writes[goal + 1], (control::MODE, 3));
            assert_eq!(
                r.writes[at..at + 6],
                [
                    fields(n, slope),
                    (control::GOAL_VELOCITY, 0),
                    (control::MODE, 2),
                    (control::GOAL_VELOCITY, cruise),
                    (control::GOAL_VELOCITY, 0),
                    (control::INJECT, 0),
                ]
            );
            k = at + 6;
        }
        let n = r.writes.len();
        let safe = crate::rig::servo::SAFE;
        assert_eq!(r.writes[n - 1 - safe.len()..n - 1], safe);
        assert_eq!(r.writes[n - 1], (control::INJECT, 0));
        assert_eq!(sw.inject_after, Some([0, 0]));
        // on the wire: the field write, then the goal and the arm staged and
        // one COMMIT
        let sent = r.sent();
        let w = sent
            .iter()
            .position(|s| *s == Sent::Write(control::INJECT.addr))
            .unwrap();
        let w = w
            + 1
            + sent[w + 1..]
                .iter()
                .position(|s| *s == Sent::Write(control::INJECT.addr))
                .unwrap();
        let hold = w + sent[w..]
            .iter()
            .position(|s| *s == Sent::Hold(control::GOAL_VELOCITY.addr))
            .unwrap();
        assert_eq!(
            sent[hold..hold + 3],
            [
                Sent::Hold(control::GOAL_VELOCITY.addr),
                Sent::Hold(control::CHUNK0_ADDR.addr),
                Sent::Commit
            ]
        );
    }

    /// A watchdog abort inside the sweep turns torque off first, zeroes the
    /// fields after the guard, reads them back, and keeps the bursts before
    /// it; nothing drives again.
    #[test]
    fn an_abort_zeroes_the_fields_on_its_way_out() {
        let mut r = rig(true);
        r.after_burst = Some(Box::new(|c, k| {
            if k == 1 {
                c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
                    t.telemetry.common.fault_flags = 1 << 2;
                    t.telemetry.mode.fault_code = 3;
                });
            }
        }));
        let (seen, sw, res) = run(&mut r, &SMALL);
        let e = res.unwrap_err();
        assert_eq!(exit_code(&e), Some(3), "{e}");
        assert!(e.to_string().contains("code 3 (stall)"), "{e}");
        assert_eq!(e.downcast_ref::<Abort>().unwrap().seg, Some(1));
        assert_eq!(seen.len(), 1, "the burst before the abort is kept");
        let stop = r
            .writes
            .iter()
            .rposition(|w| *w == (control::INJECT, 0))
            .unwrap();
        let off = r
            .writes
            .iter()
            .position(|w| *w == (control::TORQUE_ENABLE, 0))
            .unwrap();
        assert!(off < stop);
        assert!(!r.writes[off..].contains(&(control::TORQUE_ENABLE, 1)));
        assert_eq!(r.writes.last(), Some(&(control::INJECT, 0)));
        assert_eq!(sw.inject_after, Some([0, 0]));
    }

    /// The rows are guarded on the observer's position: a burst that
    /// carries the shaft past the guard aborts.
    #[test]
    fn a_burst_past_the_guard_aborts() {
        let mut r = rig(true);
        let argv = [&SMALL[..], &["--guard-hi", "2600"]].concat();
        let a = parse(&argv);
        let (mut cfg, mut sweep) = prepare(&mut r, &a).unwrap();
        // a start the plan never makes: the burst runs up past the guard
        sweep.plan.steps[0].start = 2400;
        let mut sw = Swept::default();
        let e = record(&mut r, &mut cfg, &sweep, &mut sw, |_| Ok(())).unwrap_err();
        assert_eq!(exit_code(&e), Some(3), "{e}");
        assert!(
            e.to_string()
                .contains("outside the guard 432..2600 at row tick"),
            "{e}"
        );
        assert_eq!(sw.bursts, 0);
        assert_eq!(r.writes.last(), Some(&(control::INJECT, 0)));
    }

    /// A burst past the guard the servo latched a fault in: the fault is
    /// read behind the burst ahead of the torque off, and the abort and the
    /// meta's `ended` carry both, the firmware's flags and code first.
    #[test]
    fn a_burst_the_rows_and_the_servo_both_trip_names_both() {
        use osc_client::store_state::fault;
        use osc_servo_core::kernel::phases::loops::faults::BIT_POSITION_ERROR;
        let mut r = rig(true);
        let argv = [&SMALL[..], &["--guard-hi", "2600"]].concat();
        let (mut cfg, mut sweep) = prepare(&mut r, &parse(&argv)).unwrap();
        sweep.plan.steps[0].start = 2400;
        r.after_burst = Some(Box::new(|c, k| {
            if k == 0 {
                c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
                    t.telemetry.common.fault_flags = BIT_POSITION_ERROR;
                    t.telemetry.mode.fault_code = fault::POSITION_ERROR;
                });
            }
        }));
        let mut sw = Swept::default();
        r.sent();
        let res = record(&mut r, &mut cfg, &sweep, &mut sw, |_| Ok(()));
        let sent = r.sent();
        let e = res.as_ref().unwrap_err();
        assert_eq!(exit_code(e), Some(3), "{e}");
        let a = e.downcast_ref::<Abort>().unwrap();
        let head = "the servo latched a fault: flags 0x08 code 4 (position_error); the watchdog's \
                    verdict on the burst's rows: the shaft at ";
        assert!(
            a.reason.starts_with(head)
                && a.reason.contains("outside the guard 432..2600 at row tick"),
            "{}",
            a.reason
        );
        assert_eq!(a.fault, Some(Fault { flags: 8, code: 4 }));
        let commit = sent.iter().rposition(|s| *s == Sent::Commit).unwrap();
        let off = commit
            + sent[commit..]
                .iter()
                .position(|s| *s == Sent::Write(control::TORQUE_ENABLE.addr))
                .unwrap();
        let poll = Sent::Read(
            telemetry::FAULT_FLAGS.addr,
            telemetry::WINDOW_FLOOR_Q15.addr + 2 - telemetry::FAULT_FLAGS.addr,
        );
        assert_eq!(sent[off - 1], poll, "{sent:?}");
        let mut meta = serde_json::json!({});
        land(&mut meta, &cfg, &sweep, &sw, &res);
        assert_eq!(meta["ended"]["reason"], a.reason);
        assert_eq!(meta["ended"]["step"], 0);
        assert_eq!(
            meta["ended"]["fault"],
            serde_json::json!({ "flags": 8, "code": 4, "name": "position_error" })
        );
    }

    /// An image that accepts the fields and ignores them streams a flat
    /// reference: the first burst is refused, exit 2, torque off next,
    /// nothing written to the folder, the fields zeroed.
    #[test]
    fn a_stimulus_the_image_ignores_is_refused_on_the_first_burst() {
        let mut r = rig(false);
        let (seen, sw, res) = run(&mut r, &SMALL);
        let e = res.unwrap_err();
        assert_eq!(exit_code(&e), Some(2), "{e}");
        assert!(e.to_string().contains("not a `deadman` build"), "{e}");
        assert!(e.to_string().contains("on 0 of its"), "{e}");
        assert!(seen.is_empty() && sw.bursts == 0);
        let cruise = r
            .writes
            .iter()
            .position(|w| *w == (control::GOAL_VELOCITY, 742))
            .unwrap();
        let off = r
            .writes
            .iter()
            .position(|w| *w == (control::TORQUE_ENABLE, 0))
            .unwrap();
        assert_eq!(
            r.writes[cruise + 1..=off],
            [
                (control::GOAL_VELOCITY, 0),
                (control::INJECT, 0),
                (control::TORQUE_ENABLE, 0)
            ]
        );
        assert_eq!(
            r.writes
                .iter()
                .filter(|w| w.0 == control::GOAL_VELOCITY && w.1 != 0)
                .count(),
            1
        );
        assert_eq!(sw.inject_after, Some([0, 0]));
    }

    /// No dead-man, or a servo whose descriptor lacks the fields, is
    /// refused before anything is written.
    #[test]
    fn a_sweep_without_a_deadman_or_the_fields_is_refused() {
        let mut r = rig(true);
        let e = prepare(&mut r, &parse(&["--deadman-ms", "0"]))
            .err()
            .unwrap();
        assert_eq!(exit_code(&e), Some(2));
        assert!(e.to_string().contains("loaded bus dead-man"), "{e}");
        let e = prepare(&mut r, &parse(&["--speed-cruise-cps", "1400"]))
            .err()
            .unwrap();
        assert!(e.to_string().contains("speed_ceiling_cps 1500"), "{e}");
        assert!(r.writes.is_empty());
    }

    const FACTS: [&str; 7] = [
        "inject_half_periods",
        "inject_accel_q88",
        "cruise_cps",
        "direction",
        "p_kp_q88",
        "rows_dropped",
        "seq_holes",
    ];

    fn read_json(p: &Path) -> serde_json::Value {
        serde_json::from_str(&std::fs::read_to_string(p).unwrap()).unwrap()
    }

    /// The folder: a burst's rows under `oscnb.margin`'s columns, the tick
    /// the row's sample tick and the fields the row's; its facts with
    /// every key the reader needs; meta.json with the plan, the limits, a
    /// source check's share per cruise, the fields' state after the run and
    /// how it ended.
    #[test]
    fn the_folder_carries_each_burst_and_the_sweep() {
        let out = std::env::temp_dir().join(format!("osc-lab-margin-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&out);
        let mut r = rig(true);
        r.c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
            t.telemetry.mode.omega_hat_src = 1;
        });
        let argv = [
            "--speed-n",
            "4",
            "--position-n",
            "none",
            "--source-ms",
            "100",
            "--no-center",
        ];
        capture(&mut r, &parse(&argv), &out).unwrap();
        let csv = std::fs::read_to_string(out.join("burst-0.csv")).unwrap();
        let mut lines = csv.lines();
        assert_eq!(lines.next(), Some(COLUMNS));
        let rows: Vec<Vec<i64>> = lines
            .map(|l| l.split(',').map(|v| v.parse().unwrap()).collect())
            .collect();
        assert_eq!(rows.len(), 8000);
        for w in rows.windows(2) {
            assert_eq!((w[1][0] - w[0][0]).rem_euclid(1 << 16), 1);
        }
        // the source check went up, so burst 0 runs down: the cruise less the
        // triangle's first step
        assert_eq!(rows[0][3], -(742 + 15) << 16);
        let facts = read_json(&out.join("burst-1.json"));
        for k in FACTS {
            assert!(facts.get(k).is_some(), "facts lack {k}");
        }
        assert_eq!(facts["inject_half_periods"], 4);
        assert_eq!(facts["inject_accel_q88"], 3840);
        assert_eq!(
            (facts["cruise_cps"].clone(), facts["direction"].clone()),
            (742.into(), 1.into())
        );
        assert_eq!(facts["reading_cps"], 742);
        assert!(!out.join("burst-2.csv").exists());
        let meta = read_json(&out.join("meta.json"));
        assert_eq!(meta["ended"], "ok");
        assert_eq!(meta["bursts"], 2);
        assert_eq!(
            meta["source_checks"],
            serde_json::json!([{ "cruise_cps": 742, "periods": 200, "share": { "1": 1.0 } }])
        );
        assert_eq!(
            meta["inject_after"],
            serde_json::json!({ "inject_accel_q88": 0, "inject_half_periods": 0 })
        );
        assert_eq!(meta["row"], serde_json::json!([[0x258, 8], [0x3d4, 4]]));
        assert_eq!(meta["plan"]["steps"].as_array().unwrap().len(), 3);
        assert_eq!(meta["limits"]["guard"], serde_json::json!([432, 3626]));
        for k in [
            "fw",
            "model",
            "plant",
            "deadman",
            "watchdog",
            "config",
            "control",
            "refused_modes_after",
        ] {
            assert!(meta.get(k).is_some(), "meta has no {k}");
        }
        std::fs::remove_dir_all(&out).unwrap();
    }

    /// A servo that does not list `omega_hat_src` runs no source check,
    /// and the meta says why.
    #[test]
    fn a_servo_without_the_source_field_says_so_in_meta() {
        let mut r = rig(true);
        let a = parse(&SMALL);
        let (cfg, mut sweep) = prepare(&mut r, &a).unwrap();
        sweep.bounds.source = false;
        let m = meta(&mut r, &cfg, &sweep, &a).unwrap();
        assert_eq!(m["source_checks"], "the servo does not list omega_hat_src");
    }

    /// The live check reads the reference one row a period: steps at the
    /// slope either way are a live triangle, a flat or a clamped reference
    /// is not.
    #[test]
    fn the_stimulus_check_counts_period_steps_at_the_slope() {
        let rows = |f: &dyn Fn(u64) -> i32| -> Vec<TelFrame> {
            (0..4000)
                .map(|t| TelFrame {
                    tick: t,
                    omega_ref_cps: Some(f(t / DECIM)),
                    ..TelFrame::default()
                })
                .collect()
        };
        let tri = |p: u64| (742 + 15 * (2 - (p as i32 % 8 - 4).abs())) << 16;
        assert_eq!(stimulus_steps(&rows(&tri), 15 * 256, 100), (299, 299));
        assert_eq!(
            stimulus_steps(&rows(&|_| 742 << 16), 15 * 256, 100),
            (299, 0)
        );
        assert_eq!(stimulus_steps(&rows(&|_| 0), 15 * 256, 100).1, 0);
    }

    /// The checked-in folder `tests/margin_osc_lab` of the notebooks repo,
    /// which `tests/test_margin.py` reads through `oscnb.margin`: one burst
    /// this writer writes for its rig's plant at N = 2 and the cruise, so
    /// the two sides cannot drift apart. `OSC_LAB_BLESS=1` rewrites it.
    #[test]
    fn the_writer_writes_the_fixture_the_estimator_reads() {
        let mut r = Rig::plant_at(1000, true);
        let (n, slope, cycles) = (2u16, 3840u16, 8u32);
        let periods = 400 + cycles * 2 * n as u32;
        r.write(control::MODE, Mode::Velocity.value()).unwrap();
        r.write(control::INJECT, slope as i32 | (n as i32) << 16)
            .unwrap();
        r.write(control::BUS_DEADMAN_TICKS, 20_000).unwrap();
        r.write(control::TORQUE_ENABLE, 1).unwrap();
        let rows = periods * DECIM as u32;
        let (frames, stats) = r
            .stream_at(rows, Some((control::GOAL_VELOCITY, 742)), ROW, None, None)
            .unwrap();
        assert_eq!(frames.len() as u32, rows);
        let b = Burst {
            index: 0,
            step: Step {
                kind: Kind::Burst {
                    band: Band::Speed,
                    n,
                    slope_q88: slope,
                    reading_cps: 742,
                },
                cruise: 742,
                direction: 1,
                start: 1000,
                periods,
                cycles,
                travel: 742.0 * periods as f64 / 2000.0,
            },
            frames,
            stats,
            p_kp_q88: 170 * 256,
            t_start: 0.0,
            t_end: 0.0,
        };
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("../tests/margin_osc_lab");
        if std::env::var_os("OSC_LAB_BLESS").is_some() {
            std::fs::create_dir_all(&fixture).unwrap();
            write_burst(&fixture, &b).unwrap();
        }
        let out = std::env::temp_dir().join(format!("osc-lab-fixture-{}", std::process::id()));
        std::fs::create_dir_all(&out).unwrap();
        write_burst(&out, &b).unwrap();
        for name in ["burst-0.csv", "burst-0.json"] {
            let want = std::fs::read_to_string(fixture.join(name)).unwrap();
            let got = std::fs::read_to_string(out.join(name)).unwrap();
            assert!(
                got == want,
                "{name} drifted from the fixture: OSC_LAB_BLESS=1 rewrites it"
            );
        }
        std::fs::remove_dir_all(&out).unwrap();
    }

    /// `--poll-extra` parses as `track`'s: hex address, 0x optional, 1 to
    /// 64 bytes.
    #[test]
    fn the_extra_range_parses_on_margin_too() {
        let want = Extra {
            addr: 0x3d4,
            len: 44,
        };
        assert_eq!(parse(&["--poll-extra", "0x3D4:44"]).poll_extra, Some(want));
        assert_eq!(parse(&["--poll-extra", "3d4:44"]).poll_extra, Some(want));
        assert_eq!(parse(&[]).poll_extra, None);
        for bad in ["3d4:0", "3d4:65"] {
            let v = ["margin", "--out", "unused", "--poll-extra", bad];
            assert!(<Cli as clap::Parser>::try_parse_from(v).is_err(), "{bad}");
        }
    }

    /// The folder and the wire of one sweep with `--poll-extra` as given.
    struct Captured {
        files: BTreeMap<String, String>,
        sent: Vec<Sent>,
        r: Result<()>,
    }

    /// A source check and one speed point, a burst up and a burst down.
    const EXTRA_ARGV: [&str; 7] = [
        "--speed-n",
        "4",
        "--position-n",
        "none",
        "--source-ms",
        "100",
        "--no-center",
    ];

    fn captured(extra: Option<&str>, fault_after: Option<u32>) -> Captured {
        static N: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let n = N.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        let out =
            std::env::temp_dir().join(format!("osc-lab-margin-extra-{n}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&out);
        let mut r = rig(true);
        if let Some(at) = fault_after {
            r.after_burst = Some(Box::new(move |c, k| {
                if k == at {
                    c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
                        t.telemetry.common.fault_flags = 1 << 2;
                        t.telemetry.mode.fault_code = 3;
                    });
                }
            }));
        }
        let mut argv = EXTRA_ARGV.to_vec();
        if let Some(x) = extra {
            argv.extend(["--poll-extra", x]);
        }
        r.sent();
        let res = capture(&mut r, &parse(&argv), &out);
        let files = std::fs::read_dir(&out)
            .unwrap()
            .map(|e| {
                let p = e.unwrap().path();
                let name = p.file_name().unwrap().to_string_lossy().into_owned();
                (name, std::fs::read_to_string(&p).unwrap())
            })
            .collect();
        std::fs::remove_dir_all(&out).unwrap();
        Captured {
            files,
            sent: r.sent(),
            r: res,
        }
    }

    /// The folder less polls.csv, the host's clock and `--poll-extra`'s
    /// meta keys: what must not move with the option.
    fn kept(files: &BTreeMap<String, String>) -> BTreeMap<String, String> {
        files
            .iter()
            .filter(|(name, _)| name.as_str() != "polls.csv")
            .map(|(name, text)| {
                if !name.ends_with(".json") {
                    return (name.clone(), text.clone());
                }
                let mut v: serde_json::Value = serde_json::from_str(text).unwrap();
                let o = v.as_object_mut().unwrap();
                for k in ["t_start", "t_end", "poll_extra", "extra_poll_failures"] {
                    o.remove(k);
                }
                (name.clone(), v.to_string())
            })
            .collect()
    }

    /// With `--poll-extra` the range is read right behind the poll before
    /// and the poll after each step's burst, source check included, into
    /// polls.csv: the step, its burst's K, which poll, when, and the bytes
    /// as lowercase hex (before a burst the pot reads the step's start,
    /// 512 `0002` or 3546 `da0d`). Without it there is no polls.csv and no
    /// meta key, the folder is what a run with it writes less those, and
    /// the wire carries the extra reads and nothing else more.
    #[test]
    fn the_extra_range_rides_each_bursts_polls() {
        let plain = captured(None, None);
        let with = captured(Some("0x236:2"), None);
        plain.r.unwrap();
        with.r.unwrap();
        assert!(!plain.files.contains_key("polls.csv"));
        let meta: serde_json::Value = serde_json::from_str(&plain.files["meta.json"]).unwrap();
        assert!(meta.get("poll_extra").is_none());
        assert!(meta.get("extra_poll_failures").is_none());

        let polls: Vec<Vec<&str>> = with.files["polls.csv"]
            .lines()
            .map(|l| l.split(',').collect())
            .collect();
        assert_eq!(polls[0].join(","), POLLS_COLUMNS);
        let head: Vec<[&str; 3]> = polls[1..].iter().map(|c| [c[0], c[1], c[2]]).collect();
        assert_eq!(
            head,
            [
                ["0", "", "before"],
                ["0", "", "after"],
                ["1", "0", "before"],
                ["1", "0", "after"],
                ["2", "1", "before"],
                ["2", "1", "after"],
            ]
        );
        let starts: Vec<&str> = polls[1..].iter().step_by(2).map(|c| c[4]).collect();
        assert_eq!(starts, ["0002", "da0d", "0002"]);
        assert!(polls[1..].iter().all(|c| {
            c.len() == 5
                && c[4].len() == 4
                && c[4]
                    .bytes()
                    .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
        }));
        let meta: serde_json::Value = serde_json::from_str(&with.files["meta.json"]).unwrap();
        assert_eq!(
            meta["poll_extra"],
            serde_json::json!({ "addr": 0x236, "len": 2 })
        );
        assert_eq!(meta["extra_poll_failures"], 0);

        assert_eq!(kept(&with.files), kept(&plain.files));
        let extra = Sent::Read(0x236, 2);
        let at: Vec<usize> = (0..with.sent.len())
            .filter(|&i| with.sent[i] == extra)
            .collect();
        assert_eq!(at.len(), 6, "two a step");
        assert!(
            at.iter().all(|&i| matches!(
                with.sent[i - 1],
                Sent::Read(a, _) if a == telemetry::FAULT_FLAGS.addr
            )),
            "right behind a poll"
        );
        let less: Vec<Sent> = with.sent.into_iter().filter(|s| *s != extra).collect();
        assert_eq!(less, plain.sent);
    }

    /// An extra read that fails leaves the sweep as it was: the same
    /// bursts and the same ending, clean or aborted, each failure an empty
    /// `extra_hex` counted in meta.json.
    #[test]
    fn a_failing_extra_read_leaves_the_sweep_as_it_was() {
        let plain = captured(None, None);
        let with = captured(Some("0xfff0:16"), None);
        plain.r.unwrap();
        with.r.unwrap();
        let rows: Vec<&str> = with.files["polls.csv"].lines().skip(1).collect();
        assert_eq!(rows.len(), 6);
        assert!(rows.iter().all(|l| l.ends_with(",")), "{rows:?}");
        let meta: serde_json::Value = serde_json::from_str(&with.files["meta.json"]).unwrap();
        assert_eq!(
            (meta["extra_poll_failures"].clone(), meta["bursts"].clone()),
            (6.into(), 2.into())
        );
        assert_eq!(kept(&with.files), kept(&plain.files));

        // a fault under the first burst trips the poll after it: no extra read there
        let plain = captured(None, Some(1));
        let with = captured(Some("0xfff0:16"), Some(1));
        let (pe, we) = (plain.r.unwrap_err(), with.r.unwrap_err());
        assert_eq!(exit_code(&we), Some(3));
        assert_eq!(we.to_string(), pe.to_string());
        let rows: Vec<&str> = with.files["polls.csv"].lines().skip(1).collect();
        assert_eq!(rows.len(), 3, "{rows:?}");
        let meta: serde_json::Value = serde_json::from_str(&with.files["meta.json"]).unwrap();
        assert_eq!(meta["extra_poll_failures"], 3);
        assert_eq!(kept(&with.files), kept(&plain.files));
    }
}
