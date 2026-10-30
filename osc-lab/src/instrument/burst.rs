//! Host side of the firmware high-rate shunt burst: arm, poll, walk the
//! eight readback pages, release. One capture is 960 raw ADC codes at one
//! conversion period each - a current step sampled ~46x per PWM period
//! instead of the once per period ordinary telemetry gives.
//!
//! `chans` interleaves voltage channels behind the shunt: every frame is the
//! shunt then the selected extras in bit order, so a channel in slot s is
//! sampled s conversions after its frame's first shunt sample. chans 0 is
//! one shunt code per frame. [`CHAN_INTERLEAVE`] puts another shunt ahead
//! of every extra after the first, so the shunt keeps every second
//! conversion: both terminals ride along as shunt, A, shunt, B.
//!
//! [`BurstIo`] is the four wire moves and the sleep the handshake needs, the
//! driver supplies them. The page walk is the only part with a protocol
//! subtlety - the servo copies a page into the readback window from its main
//! loop with `page_echo` as the guard (0xFF while copying, the page number
//! after a fence), so a reply whose echo does not match the requested page
//! is a plain retry.
//!
//! The burst window is ADC time the scan does not run, so the pre-arm rail,
//! the current-sense bias and the terminal divider bias come from a
//! telemetry read taken just before the arm (see [`Pre`]).

use core::fmt;

use super::limits::BurstAllowance;
use super::regs::{Reg, burst as reg, control};

/// Samples one capture holds; 8 pages of 120.
pub const SAMPLES: usize = 960;
pub const PAGE_SAMPLES: usize = 120;
pub const PAGES: u8 = 8;

/// One READ of the burst section returns the selected page plus the whole
/// header - `MAX_PAYLOAD` on the wire, and exactly the section's live span.
pub const READ_LEN: u16 = 252;

pub const STATE_IDLE: u8 = 0;
pub const STATE_DONE: u8 = 3;
pub const STATE_REJECTED: u8 = 4;

/// `chans` bits: extras sampled behind the shunt, in this order.
pub const CHAN_VMOTOR_A: u8 = 1 << 0;
pub const CHAN_VMOTOR_B: u8 = 1 << 1;
pub const CHAN_VBUS: u8 = 1 << 2;
/// Layout bit: a shunt slot ahead of every extra after the first.
pub const CHAN_INTERLEAVE: u8 = 1 << 3;
pub const CHAN_EXTRAS: u8 = CHAN_VMOTOR_A | CHAN_VMOTOR_B | CHAN_VBUS;
/// Largest mask the servo accepts.
pub const CHANS_MAX: u8 = CHAN_EXTRAS | CHAN_INTERLEAVE;
/// Both terminals at half rate each, the shunt at frame_len 2's rate.
pub const CHANS_DIFF: u8 = CHAN_VMOTOR_A | CHAN_VMOTOR_B | CHAN_INTERLEAVE;

/// Shunt codes per frame for a mask: one, or one per extra when
/// interleaved.
const fn shunts(chans: u8) -> usize {
    let n = (chans & CHAN_EXTRAS).count_ones() as usize;
    if chans & CHAN_INTERLEAVE != 0 && n > 0 {
        n
    } else {
        1
    }
}

/// Codes per frame for a mask: the shunts plus one per selected extra.
pub const fn frame_len(chans: u8) -> usize {
    shunts(chans) + (chans & CHAN_EXTRAS).count_ones() as usize
}

/// The extras a capture asks for. `Driven` samples only the tap of the
/// terminal the step drives (OUT1 -> MOT_A -> vmotor_a, else vmotor_b): the
/// chopping leg, so one channel gives the winding's high side through the
/// ON phase and the OFF phase both, at frame_len 2. Every extra thins the
/// shunt stream. `Diff` samples both terminals interleaved
/// ([`CHANS_DIFF`]): the shunt keeps frame_len 2's rate and each terminal
/// is sampled every 4.3 us, which gives the winding's own voltage, the low
/// side and the divider bias cancelled.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Chans {
    Fixed(u8),
    Driven,
    Diff,
}

impl Chans {
    pub fn for_step(self, step_q15: i16, drive_polarity: bool) -> u8 {
        match self {
            Chans::Fixed(m) => m,
            Chans::Driven if drives_a(step_q15, drive_polarity) => CHAN_VMOTOR_A,
            Chans::Driven => CHAN_VMOTOR_B,
            Chans::Diff => CHANS_DIFF,
        }
    }

    /// `driven`, `diff` or a mask 0..=15.
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "driven" => Some(Chans::Driven),
            "diff" => Some(Chans::Diff),
            _ => s
                .parse::<u8>()
                .ok()
                .filter(|m| *m <= CHANS_MAX)
                .map(Chans::Fixed),
        }
    }
}

/// A step drives terminal A high: the servo negates a step under reversed
/// wiring as it does an OpenLoop duty.
fn drives_a(step_q15: i16, drive_polarity: bool) -> bool {
    (step_q15 >= 0) == drive_polarity
}

/// What the host knew about the servo just before the arm. The burst
/// suspends the scan, so neither number is measurable inside the window.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Pre {
    /// Duty in force over the pre-step half of the capture, q15.
    pub pre_q15: i16,
    pub vbus_raw: u16,
    /// Current-sense zero as the servo's trough tracker reports it.
    pub bias: u16,
    /// Terminal divider bias measured at boot with the bridge Hi-Z, raw.
    pub vmotor_bias: u16,
    pub pos: u16,
    /// The rotor is held against a mechanical stop: no back-EMF.
    pub seated: bool,
    /// `drive_polarity` in force.
    pub drive_polarity: bool,
}

impl Default for Pre {
    fn default() -> Self {
        Self {
            pre_q15: 0,
            vbus_raw: 0,
            bias: 0,
            vmotor_bias: 0,
            pos: 0,
            seated: false,
            drive_polarity: true,
        }
    }
}

/// The burst header as one READ returns it, minus the page payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Meta {
    pub pre_q15: i16,
    pub step_q15: i16,
    /// Raw sample index, not frame index.
    pub step_index: u16,
    pub start_cnt: u16,
    pub pwm_arr: u16,
    pub start_dir: u8,
    pub restore_dir: u8,
    pub vbus_raw: u16,
    pub bias: u16,
    /// The mask the servo echoed and the frame length it sampled with.
    pub chans: u8,
    pub frame_len: u8,
    pub vmotor_bias: u16,
    /// Position read before the arm.
    pub pos: u16,
    pub seated: bool,
    pub drive_polarity: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Capture {
    /// Raw codes in conversion order, frames interleaved.
    pub samples: Vec<u16>,
    pub meta: Meta,
}

/// Poll and retry budgets. Defaults: the capture itself is ~2 ms
/// (BURST_SETTLE_TICKS + 960 conversions), so 200 polls at 2 ms is a 400 ms
/// ceiling - long enough that only a wedged servo hits it.
#[derive(Copy, Clone, Debug)]
pub struct CaptureCfg {
    pub poll_ms: u32,
    pub max_polls: u32,
    /// Attempts per page before the capture is abandoned.
    pub page_tries: u32,
}

impl Default for CaptureCfg {
    fn default() -> Self {
        Self {
            poll_ms: 2,
            max_polls: 200,
            page_tries: 5,
        }
    }
}

/// The wire moves a capture needs. `arm` must apply duty, mask and arm in
/// the same instant (HOLD all three, one COMMIT): a servo that sees arm
/// before the duty captures the old level.
pub trait BurstIo {
    type Error;

    fn arm(&mut self, duty_q15: i16, chans: u8) -> Result<(), Self::Error>;
    fn select_page(&mut self, page: u8) -> Result<(), Self::Error>;
    fn release(&mut self) -> Result<(), Self::Error>;
    /// One READ of the burst section, `len` bytes from `addr`.
    fn read_burst(&mut self, addr: u16, len: u16) -> Result<Vec<u8>, Self::Error>;
    fn pause_ms(&mut self, ms: u32);
}

/// Every rule the firmware arms a burst by, in plain words.
const EVERY_RULE: &str = "it arms only with torque on, in OpenLoop, with no fault latched, with \
                          no telemetry capture running, at most 3.2 V applied (duty times the \
                          rail), 100 ms or more after the burst before, and with the shaft \
                          inside the soft limits unless the stall permit is held";

/// `limit_flags` bit 3: the stall permit's grant is live.
pub const LIMIT_PERMIT_LIVE: u8 = 1 << 3;

/// What the host reads once an arm is refused, to name the rule that
/// refused it.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ArmSeen {
    /// `vbus_counts`: the rail as the firmware judged the arm by.
    pub vbus_counts: u16,
    /// Millivolts a vcount of the terminal taps.
    pub mv_per_count: f64,
    /// Raw `pos`, and the soft limits it must sit strictly inside.
    pub pos: u16,
    pub soft: (i32, i32),
    pub limit_flags: u8,
    pub fault_flags: u8,
}

/// Why the servo refused an arm at `duty_q15`: the rules `seen` shows
/// broken, else every rule, the ones the host cannot see included.
pub fn rejected(duty_q15: i16, seen: &ArmSeen) -> String {
    let mut why = Vec::new();
    if seen.fault_flags != 0 {
        why.push(format!(
            "it has a fault latched (flags {:#04x})",
            seen.fault_flags
        ));
    }
    let cap = (BurstAllowance::MAX_MV / seen.mv_per_count).round() as u32;
    let volts = (duty_q15.unsigned_abs() as u32 * seen.vbus_counts as u32) >> 15;
    if seen.mv_per_count > 0.0 && volts > cap {
        let rail = seen.vbus_counts as f64 * seen.mv_per_count / 1000.0;
        why.push(format!(
            "{:.1}% of its {rail:.2} V rail applies {:.3} V, over the 3.2 V a burst may",
            duty_q15.unsigned_abs() as f64 / 32767.0 * 100.0,
            volts as f64 * seen.mv_per_count / 1000.0
        ));
    }
    let (lo, hi) = seen.soft;
    let inside = (seen.pos as i32) > lo && (seen.pos as i32) < hi;
    if !inside && seen.limit_flags & LIMIT_PERMIT_LIVE == 0 {
        why.push(format!(
            "the shaft at {} is outside the soft limits {lo}..{hi} and the stall permit is not \
             held",
            seen.pos
        ));
    }
    if why.is_empty() {
        return format!("the servo refused the burst: {EVERY_RULE}");
    }
    format!("the servo refused the burst: {}", why.join(", and "))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error<E> {
    Io(E),
    /// The servo refused the arm ([`rejected`] names the rule).
    Rejected,
    /// State never reached Done.
    Timeout {
        state: u8,
        polls: u32,
    },
    /// A page never echoed its own number.
    Page {
        page: u8,
        tries: u32,
    },
    /// A reply too short to hold the header.
    Short {
        got: usize,
        want: usize,
    },
    /// The servo published a sample count this client cannot page.
    Len {
        got: u16,
    },
    /// The servo sampled a different mask than the one armed.
    Chans {
        want: u8,
        got: u8,
    },
    /// The published frame length does not match the echoed mask.
    FrameLen {
        chans: u8,
        got: u8,
    },
}

impl<E: fmt::Display> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Rejected => write!(f, "the servo refused the burst: {EVERY_RULE}"),
            Error::Timeout { state, polls } => {
                write!(f, "burst stuck in state {state} after {polls} polls")
            }
            Error::Page { page, tries } => {
                write!(f, "page {page} never echoed in {tries} tries")
            }
            Error::Short { got, want } => write!(f, "burst read returned {got} B, need {want}"),
            Error::Len { got } => {
                write!(f, "servo published samples_len {got}, expected {SAMPLES}")
            }
            Error::Chans { want, got } => {
                write!(f, "armed chans {want}, servo echoed {got}")
            }
            Error::FrameLen { chans, got } => {
                write!(
                    f,
                    "chans {chans} needs frame_len {}, servo published {got}",
                    frame_len(*chans)
                )
            }
        }
    }
}

/// Byte offsets inside a `READ(PAGE_ECHO, READ_LEN)` reply.
const fn off(r: Reg) -> usize {
    (r.addr - reg::PAGE_ECHO.addr) as usize
}

fn u16_at(raw: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([raw[at], raw[at + 1]])
}

/// Header of one readback reply. Only `page_echo`/`state` are meaningful
/// before the capture is Done.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Header {
    pub page_echo: u8,
    pub state: u8,
    pub samples_len: u16,
    pub step_index: u16,
    pub start_cnt: u16,
    pub pwm_arr: u16,
    pub start_dir: u8,
    pub restore_dir: u8,
}

/// None when the slice is shorter than the header span.
pub fn parse_header(raw: &[u8]) -> Option<Header> {
    if raw.len() < READ_LEN as usize {
        return None;
    }
    Some(Header {
        page_echo: raw[off(reg::PAGE_ECHO)],
        state: raw[off(reg::STATE)],
        samples_len: u16_at(raw, off(reg::SAMPLES_LEN)),
        step_index: u16_at(raw, off(reg::STEP_INDEX)),
        start_cnt: u16_at(raw, off(reg::START_CNT)),
        pwm_arr: u16_at(raw, off(reg::PWM_ARR)),
        start_dir: raw[off(reg::START_DIR)],
        restore_dir: raw[off(reg::RESTORE_DIR)],
    })
}

/// The 120 LE codes of the page a reply carries.
pub fn parse_page(raw: &[u8]) -> impl Iterator<Item = u16> + '_ {
    let base = off(reg::SAMPLES);
    (0..PAGE_SAMPLES).map(move |k| u16_at(raw, base + 2 * k))
}

/// The mask echo and frame length sit one past the page READ.
const TAIL_LEN: u16 = reg::FRAME_LEN.addr + 1 - reg::CHANS_ECHO.addr;

/// One capture end to end: arm at `duty_q15` sampling the `chans` extras,
/// wait for Done, check the echoed mask, walk the pages, release. `pre` is
/// the host's pre-arm telemetry read; it is copied into the capture's meta
/// unchanged.
pub fn capture<IO: BurstIo>(
    io: &mut IO,
    duty_q15: i16,
    chans: u8,
    pre: Pre,
    cfg: &CaptureCfg,
) -> Result<Capture, Error<IO::Error>> {
    io.arm(duty_q15, chans).map_err(Error::Io)?;
    let head = poll_done(io, cfg).and_then(|h| tail(io, chans).map(|t| (h, t)));
    // Release whatever the poll found: a rejected or wedged arm must not be
    // left holding the servo's burst FSM.
    let (head, (echo, flen)) = match head {
        Ok(h) => h,
        Err(e) => {
            let _ = io.release();
            return Err(e);
        }
    };
    let walk = walk_pages(io, cfg);
    io.release().map_err(Error::Io)?;
    let samples = walk?;
    Ok(Capture {
        samples,
        meta: Meta {
            pre_q15: pre.pre_q15,
            step_q15: duty_q15,
            step_index: head.step_index,
            start_cnt: head.start_cnt,
            pwm_arr: head.pwm_arr,
            start_dir: head.start_dir,
            restore_dir: head.restore_dir,
            vbus_raw: pre.vbus_raw,
            bias: pre.bias,
            chans: echo,
            frame_len: flen,
            vmotor_bias: pre.vmotor_bias,
            pos: pre.pos,
            seated: pre.seated,
            drive_polarity: pre.drive_polarity,
        },
    })
}

/// (chans_echo, frame_len), both checked against the armed mask.
fn tail<IO: BurstIo>(io: &mut IO, chans: u8) -> Result<(u8, u8), Error<IO::Error>> {
    let raw = io
        .read_burst(reg::CHANS_ECHO.addr, TAIL_LEN)
        .map_err(Error::Io)?;
    let (Some(&echo), Some(&flen)) = (raw.first(), raw.get(1)) else {
        return Err(Error::Short {
            got: raw.len(),
            want: TAIL_LEN as usize,
        });
    };
    if echo != chans {
        return Err(Error::Chans {
            want: chans,
            got: echo,
        });
    }
    if flen as usize != frame_len(chans) {
        return Err(Error::FrameLen { chans, got: flen });
    }
    Ok((echo, flen))
}

fn poll_done<IO: BurstIo>(io: &mut IO, cfg: &CaptureCfg) -> Result<Header, Error<IO::Error>> {
    let mut last = STATE_IDLE;
    for _ in 0..cfg.max_polls {
        let raw = io
            .read_burst(reg::PAGE_ECHO.addr, READ_LEN)
            .map_err(Error::Io)?;
        let h = parse_header(&raw).ok_or(Error::Short {
            got: raw.len(),
            want: READ_LEN as usize,
        })?;
        last = h.state;
        match h.state {
            STATE_DONE => {
                if h.samples_len as usize != SAMPLES {
                    return Err(Error::Len { got: h.samples_len });
                }
                return Ok(h);
            }
            STATE_REJECTED => return Err(Error::Rejected),
            _ => io.pause_ms(cfg.poll_ms),
        }
    }
    Err(Error::Timeout {
        state: last,
        polls: cfg.max_polls,
    })
}

fn walk_pages<IO: BurstIo>(io: &mut IO, cfg: &CaptureCfg) -> Result<Vec<u16>, Error<IO::Error>> {
    let mut out = Vec::with_capacity(SAMPLES);
    for page in 0..PAGES {
        let mut got = false;
        for _ in 0..cfg.page_tries {
            io.select_page(page).map_err(Error::Io)?;
            let raw = io
                .read_burst(reg::PAGE_ECHO.addr, READ_LEN)
                .map_err(Error::Io)?;
            let h = parse_header(&raw).ok_or(Error::Short {
                got: raw.len(),
                want: READ_LEN as usize,
            })?;
            if h.page_echo == page {
                out.extend(parse_page(&raw));
                got = true;
                break;
            }
        }
        if !got {
            return Err(Error::Page {
                page,
                tries: cfg.page_tries,
            });
        }
    }
    Ok(out)
}

// The CSV a capture lands as.

/// Column header of a `burst-N.csv`. `code` is the raw buffer in
/// conversion order, frames interleaved. The meta columns carry a value on
/// the first data row only - they are one capture's constants, not a series.
pub const CSV_HEADER: &str = "k,code,pre_q15,step_q15,step_index,start_cnt,pwm_arr,\
                              start_dir,restore_dir,vbus_raw,bias,chans,frame_len,vmotor_bias,\
                              pos,seated,drive_polarity";

pub fn to_csv(cap: &Capture) -> String {
    let m = &cap.meta;
    let mut s = String::with_capacity(CSV_HEADER.len() + cap.samples.len() * 10);
    s.push_str(CSV_HEADER);
    s.push('\n');
    for (k, v) in cap.samples.iter().enumerate() {
        if k == 0 {
            let _ = fmt::Write::write_fmt(
                &mut s,
                format_args!(
                    "0,{v},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
                    m.pre_q15,
                    m.step_q15,
                    m.step_index,
                    m.start_cnt,
                    m.pwm_arr,
                    m.start_dir,
                    m.restore_dir,
                    m.vbus_raw,
                    m.bias,
                    m.chans,
                    m.frame_len,
                    m.vmotor_bias,
                    m.pos,
                    m.seated as u8,
                    m.drive_polarity as u8
                ),
            );
        } else {
            let _ = fmt::Write::write_fmt(&mut s, format_args!("{k},{v}\n"));
        }
    }
    s
}

/// The registers a driver writes to arm and page, so a [`BurstIo`]
/// implementation has one import.
pub mod wire {
    use super::super::regs::Reg;

    pub const DUTY_Q15: Reg = super::control::BURST_DUTY_Q15;
    pub const CHANS: Reg = super::control::BURST_CHANS;
    pub const ARM: Reg = super::control::BURST_ARM;
    pub const PAGE: Reg = super::control::BURST_PAGE;
}

#[cfg(test)]
mod tests {
    use super::*;

    const STATE_CAPTURING: u8 = 2;
    /// `page_echo` while the servo is mid-copy.
    const PAGE_BUSY: u8 = 0xFF;

    /// A servo that answers the handshake, with knobs for the failure
    /// modes the client has to survive.
    struct Fake {
        state: u8,
        polls: u32,
        /// Polls to spend in Capturing before Done.
        busy: u32,
        page: u8,
        /// Pages that answer PAGE_BUSY once before echoing.
        stutter: bool,
        stuttered: Vec<u8>,
        armed: Option<i16>,
        chans: u8,
        /// What the servo echoes, when it disagrees with the arm.
        echo: Option<u8>,
        released: bool,
    }

    impl Fake {
        fn new() -> Self {
            Self {
                state: STATE_IDLE,
                polls: 0,
                busy: 2,
                page: 0,
                stutter: false,
                stuttered: Vec::new(),
                armed: None,
                chans: 0,
                echo: None,
                released: false,
            }
        }

        fn reply(&mut self, echo: u8) -> Vec<u8> {
            let mut raw = vec![0u8; READ_LEN as usize];
            raw[off(reg::PAGE_ECHO)] = echo;
            raw[off(reg::STATE)] = self.state;
            if self.state == STATE_DONE {
                for k in 0..PAGE_SAMPLES {
                    let v = (self.page as u16) * PAGE_SAMPLES as u16 + k as u16;
                    let at = off(reg::SAMPLES) + 2 * k;
                    raw[at..at + 2].copy_from_slice(&v.to_le_bytes());
                }
                raw[off(reg::SAMPLES_LEN)..][..2].copy_from_slice(&(SAMPLES as u16).to_le_bytes());
                raw[off(reg::STEP_INDEX)..][..2].copy_from_slice(&485u16.to_le_bytes());
                raw[off(reg::START_CNT)..][..2].copy_from_slice(&1094u16.to_le_bytes());
                raw[off(reg::PWM_ARR)..][..2].copy_from_slice(&1200u16.to_le_bytes());
                raw[off(reg::START_DIR)] = 1;
                raw[off(reg::RESTORE_DIR)] = 0;
            }
            raw
        }
    }

    impl BurstIo for Fake {
        type Error = &'static str;

        fn arm(&mut self, duty_q15: i16, chans: u8) -> Result<(), &'static str> {
            self.armed = Some(duty_q15);
            self.chans = chans;
            self.state = STATE_CAPTURING;
            self.polls = 0;
            Ok(())
        }

        fn select_page(&mut self, page: u8) -> Result<(), &'static str> {
            self.page = page;
            Ok(())
        }

        fn release(&mut self) -> Result<(), &'static str> {
            self.released = true;
            self.state = STATE_IDLE;
            Ok(())
        }

        fn read_burst(&mut self, addr: u16, len: u16) -> Result<Vec<u8>, &'static str> {
            if addr == reg::CHANS_ECHO.addr {
                let echo = self.echo.unwrap_or(self.chans);
                return Ok(vec![echo, frame_len(echo) as u8][..len as usize].to_vec());
            }
            if self.state == STATE_CAPTURING {
                self.polls += 1;
                if self.polls > self.busy {
                    self.state = STATE_DONE;
                }
                return Ok(self.reply(PAGE_BUSY));
            }
            let page = self.page;
            let echo = if self.stutter && !self.stuttered.contains(&page) {
                self.stuttered.push(page);
                PAGE_BUSY
            } else {
                page
            };
            Ok(self.reply(echo))
        }

        fn pause_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn capture_walks_every_page_in_order() {
        let mut f = Fake::new();
        let pre = Pre {
            pre_q15: 0,
            vbus_raw: 2169,
            bias: 118,
            vmotor_bias: 779,
            pos: 122,
            seated: true,
            drive_polarity: false,
        };
        let cap = capture(
            &mut f,
            13107,
            CHAN_VMOTOR_A | CHAN_VBUS,
            pre,
            &CaptureCfg::default(),
        )
        .expect("capture");
        assert_eq!(cap.samples.len(), SAMPLES);
        // the fake numbers every code with its own index
        assert!(
            cap.samples
                .iter()
                .enumerate()
                .all(|(k, v)| *v as usize == k)
        );
        assert_eq!(cap.meta.step_q15, 13107);
        assert_eq!(cap.meta.step_index, 485);
        assert_eq!(cap.meta.vbus_raw, 2169);
        assert_eq!(cap.meta.bias, 118);
        assert_eq!((cap.meta.chans, cap.meta.frame_len), (5, 3));
        assert_eq!((cap.meta.pos, cap.meta.seated), (122, true));
        assert!(!cap.meta.drive_polarity);
        assert_eq!(f.armed, Some(13107));
        assert!(f.released, "arm must be dropped after the walk");
    }

    #[test]
    fn a_mask_the_servo_did_not_sample_fails_the_capture() {
        let mut f = Fake::new();
        f.echo = Some(0);
        let e = capture(
            &mut f,
            8520,
            CHAN_VMOTOR_A,
            Pre::default(),
            &CaptureCfg::default(),
        )
        .unwrap_err();
        assert_eq!(e, Error::Chans { want: 1, got: 0 });
        assert!(f.released, "a refused echo still releases");
    }

    #[test]
    fn a_stuttering_page_is_retried_not_spliced() {
        let mut f = Fake::new();
        f.stutter = true;
        let cap =
            capture(&mut f, 8520, 0, Pre::default(), &CaptureCfg::default()).expect("capture");
        assert!(
            cap.samples
                .iter()
                .enumerate()
                .all(|(k, v)| *v as usize == k)
        );
    }

    #[test]
    fn a_page_that_never_echoes_fails_the_capture() {
        struct Mute(Fake);
        impl BurstIo for Mute {
            type Error = &'static str;
            fn arm(&mut self, d: i16, c: u8) -> Result<(), &'static str> {
                self.0.arm(d, c)
            }
            fn select_page(&mut self, p: u8) -> Result<(), &'static str> {
                self.0.select_page(p)
            }
            fn release(&mut self) -> Result<(), &'static str> {
                self.0.release()
            }
            fn read_burst(&mut self, a: u16, l: u16) -> Result<Vec<u8>, &'static str> {
                let mut raw = self.0.read_burst(a, l)?;
                if self.0.state != STATE_CAPTURING && a == reg::PAGE_ECHO.addr {
                    raw[off(reg::PAGE_ECHO)] = PAGE_BUSY;
                }
                Ok(raw)
            }
            fn pause_ms(&mut self, ms: u32) {
                self.0.pause_ms(ms)
            }
        }
        let mut m = Mute(Fake::new());
        let e = capture(&mut m, 8520, 0, Pre::default(), &CaptureCfg::default()).unwrap_err();
        assert_eq!(
            e,
            Error::Page {
                page: 0,
                tries: CaptureCfg::default().page_tries
            }
        );
        assert!(m.0.released, "a failed walk still releases");
    }

    #[test]
    fn a_rejected_arm_reports_and_releases() {
        struct Reject(Fake);
        impl BurstIo for Reject {
            type Error = &'static str;
            fn arm(&mut self, d: i16, c: u8) -> Result<(), &'static str> {
                self.0.arm(d, c)?;
                self.0.state = STATE_REJECTED;
                Ok(())
            }
            fn select_page(&mut self, p: u8) -> Result<(), &'static str> {
                self.0.select_page(p)
            }
            fn release(&mut self) -> Result<(), &'static str> {
                self.0.release()
            }
            fn read_burst(&mut self, a: u16, l: u16) -> Result<Vec<u8>, &'static str> {
                self.0.read_burst(a, l)
            }
            fn pause_ms(&mut self, ms: u32) {
                self.0.pause_ms(ms)
            }
        }
        let mut r = Reject(Fake::new());
        let e = capture(&mut r, 8520, 0, Pre::default(), &CaptureCfg::default()).unwrap_err();
        assert_eq!(e, Error::Rejected);
        assert!(r.0.released);
    }

    /// The bench servo's taps, 2.466 mV a vcount, where 3.2 V is 1298
    /// vcounts: the refusal names the rule the host can see broken - the
    /// volts from the rail and the duty, the position from the soft limits
    /// and the permit, a latched fault - and every rule when it sees none.
    #[test]
    fn a_rejected_arm_names_its_reason() {
        let seen = ArmSeen {
            vbus_counts: 3204,
            mv_per_count: 3.3 / 4096.0 * 10_100.0 / 3_300.0 * 1000.0,
            pos: 2029,
            soft: (432, 3626),
            limit_flags: 0,
            fault_flags: 0,
        };
        let every = rejected(13106, &seen);
        assert_eq!(every, Error::<&str>::Rejected.to_string());
        assert_eq!(
            every,
            "the servo refused the burst: it arms only with torque on, in OpenLoop, with no \
             fault latched, with no telemetry capture running, at most 3.2 V applied (duty \
             times the rail), 100 ms or more after the burst before, and with the shaft inside \
             the soft limits unless the stall permit is held"
        );
        // 40% on a rail risen to 8.01 V: 1299 vcounts, one over the cap
        let risen = ArmSeen {
            vbus_counts: 3248,
            ..seen
        };
        assert_eq!(
            rejected(13106, &risen),
            "the servo refused the burst: 40.0% of its 8.01 V rail applies 3.203 V, over the 3.2 \
             V a burst may"
        );
        assert_eq!(rejected(-13106, &risen), rejected(13106, &risen));
        assert_eq!(
            rejected(
                13106,
                &ArmSeen {
                    vbus_counts: 3247,
                    ..seen
                }
            ),
            every
        );
        let outside = ArmSeen { pos: 3626, ..seen };
        assert_eq!(
            rejected(8192, &outside),
            "the servo refused the burst: the shaft at 3626 is outside the soft limits \
             432..3626 and the stall permit is not held"
        );
        let permitted = ArmSeen {
            limit_flags: LIMIT_PERMIT_LIVE,
            ..outside
        };
        assert_eq!(rejected(8192, &permitted), every);
        let both = ArmSeen {
            fault_flags: 0x04,
            ..risen
        };
        assert_eq!(
            rejected(13106, &both),
            "the servo refused the burst: it has a fault latched (flags 0x04), and 40.0% of its \
             8.01 V rail applies 3.203 V, over the 3.2 V a burst may"
        );
    }

    #[test]
    fn a_wedged_state_times_out_within_the_budget() {
        struct Wedged;
        impl BurstIo for Wedged {
            type Error = &'static str;
            fn arm(&mut self, _: i16, _: u8) -> Result<(), &'static str> {
                Ok(())
            }
            fn select_page(&mut self, _: u8) -> Result<(), &'static str> {
                Ok(())
            }
            fn release(&mut self) -> Result<(), &'static str> {
                Ok(())
            }
            fn read_burst(&mut self, _: u16, _: u16) -> Result<Vec<u8>, &'static str> {
                let mut raw = vec![0u8; READ_LEN as usize];
                raw[off(reg::STATE)] = STATE_CAPTURING;
                Ok(raw)
            }
            fn pause_ms(&mut self, _: u32) {}
        }
        let cfg = CaptureCfg {
            max_polls: 7,
            ..CaptureCfg::default()
        };
        let e = capture(&mut Wedged, 0, 0, Pre::default(), &cfg).unwrap_err();
        assert_eq!(
            e,
            Error::Timeout {
                state: STATE_CAPTURING,
                polls: 7
            }
        );
    }

    #[test]
    fn header_offsets_match_the_register_map() {
        assert_eq!(off(reg::PAGE_ECHO), 0);
        assert_eq!(off(reg::STATE), 1);
        assert_eq!(off(reg::SAMPLES), 2);
        assert_eq!(off(reg::SAMPLES_LEN), 2 + 2 * PAGE_SAMPLES);
        // the header's last byte is the last byte one READ can carry; the
        // mask echo and frame length are the tail read's
        assert_eq!(off(reg::RESTORE_DIR), READ_LEN as usize - 1);
        assert_eq!(reg::CHANS_ECHO.addr, reg::RESTORE_DIR.addr + 1);
        assert_eq!(reg::SAMPLES.width as usize, 2 * PAGE_SAMPLES);
        assert_eq!(TAIL_LEN, 2);
        assert_eq!(PAGES as usize * PAGE_SAMPLES, SAMPLES);
    }

    /// The meta rides the first row only; every later row is `k,code`.
    #[test]
    fn csv_carries_the_meta_on_the_first_row() {
        let cap = Capture {
            samples: vec![113, 112, 4095],
            meta: Meta {
                pre_q15: 3276,
                step_q15: -8520,
                step_index: 485,
                start_cnt: 1055,
                pwm_arr: 1200,
                start_dir: 1,
                restore_dir: 0,
                vbus_raw: 2154,
                bias: 113,
                chans: CHANS_DIFF,
                frame_len: 4,
                vmotor_bias: 779,
                pos: 122,
                seated: true,
                drive_polarity: false,
            },
        };
        assert_eq!(
            to_csv(&cap),
            format!(
                "{CSV_HEADER}\n0,113,3276,-8520,485,1055,1200,1,0,2154,113,11,4,779,122,1,0\n\
                 1,112\n2,4095\n"
            )
        );
        assert_eq!(CSV_HEADER.split(',').count(), 17);
    }

    /// Under polarity 0 the step's sign flips the terminal it drives.
    #[test]
    fn the_driven_terminal_follows_the_drive_polarity() {
        for (step, polarity, a) in [
            (8520, true, true),
            (-8520, true, false),
            (8520, false, false),
            (-8520, false, true),
        ] {
            let mask = if a { CHAN_VMOTOR_A } else { CHAN_VMOTOR_B };
            assert_eq!(Chans::Driven.for_step(step, polarity), mask);
        }
    }

    /// Interleaved, the shunt holds every even slot and each extra the odd
    /// slot after its own shunt; one extra interleaved is the plain layout.
    #[test]
    fn frame_lengths_follow_the_mask() {
        assert_eq!(frame_len(0), 1);
        assert_eq!(frame_len(CHAN_VMOTOR_A | CHAN_VBUS), 3);
        assert_eq!(frame_len(CHANS_DIFF), 4);
        assert_eq!(frame_len(CHANS_MAX), 6);
        assert_eq!(frame_len(CHAN_VMOTOR_B | CHAN_INTERLEAVE), 2);
        assert_eq!(frame_len(CHAN_INTERLEAVE), 1);
        for m in 0..=CHANS_MAX {
            assert_eq!(SAMPLES % frame_len(m), 0, "chans {m}");
        }
        assert_eq!(Chans::parse("diff"), Some(Chans::Diff));
        assert_eq!(Chans::parse("driven"), Some(Chans::Driven));
        assert_eq!(Chans::Diff.for_step(-8520, true), CHANS_DIFF);
        assert_eq!(Chans::parse("11"), Some(Chans::Fixed(CHANS_DIFF)));
        assert_eq!(Chans::Fixed(5).for_step(-8520, false), 5);
        assert_eq!(Chans::parse("16"), None);
        assert_eq!(Chans::parse("x"), None);
    }
}
