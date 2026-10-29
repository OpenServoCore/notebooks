//! The instrument pieces a capture runs on, copied from the firmware repo's
//! host crates and trimmed to what sweep and burst use: register addresses,
//! TEL frame decode, the burst handshake and its CSV, the sense scales, the
//! seek's stillness watch, the brake, the stall-safe duty sizing and the
//! stall permit lease. No identification or calibration code lives here.

pub mod burst;
pub mod frame;
pub mod limits;
pub mod regs;
pub mod runway;
pub mod seek;
pub mod units;
