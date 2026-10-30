//! `osc-lab` - the raw capture instrument the notebooks record with, driven
//! through the osc-adapter: open-loop duty sweeps of TEL rungs (`sweep`) and
//! high-rate shunt bursts from rest (`burst`). Raw captures only: nothing
//! here fits, identifies or calibrates. Every other verb (get, set, status,
//! baud, cal) is `osc`'s.

mod burst;
mod descriptor;
mod instrument;
mod rig;
mod state;
mod sweep;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use osc_client::BaudRate;

#[derive(Parser, Debug)]
#[command(
    name = "osc-lab",
    about = "Raw captures from an osc-native servo through the osc-adapter."
)]
struct Cli {
    #[command(flatten)]
    bus: rig::Bus,
    /// Servo id for the unicast verbs.
    #[arg(short, long, global = true, default_value_t = 1)]
    id: u8,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Raw open-loop duty sweep: burst-captured TEL rungs across a duty grid
    /// for empirical plant capture (sweep.csv + meta.json).
    Sweep(sweep::Args),
    /// Raw high-rate shunt bursts from rest at a list of step duties, no fit
    /// (burst-K.csv + meta.json).
    Burst(burst::Args),
}

pub(crate) fn parse_u16(s: &str) -> Result<u16> {
    Ok(match s.strip_prefix("0x") {
        Some(h) => u16::from_str_radix(h, 16)?,
        None => s.parse()?,
    })
}

fn parse_rate(s: &str) -> Result<BaudRate> {
    let bps: u32 = s.parse().context("baud is an integer rate or `auto`")?;
    [
        BaudRate::B500000,
        BaudRate::B1000000,
        BaudRate::B2000000,
        BaudRate::B3000000,
    ]
    .into_iter()
    .find(|r| r.as_hz() == bps)
    .ok_or_else(|| anyhow::anyhow!("unsupported baud {bps} (500000/1000000/2000000/3000000)"))
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match &cli.cmd {
        Cmd::Sweep(args) => sweep::run(args, cli.bus.clone(), cli.id),
        Cmd::Burst(args) => burst::run(args, cli.bus.clone(), cli.id),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rates_and_addresses_parse_as_the_user_writes_them() {
        assert_eq!(parse_rate("3000000").unwrap(), BaudRate::B3000000);
        assert!(parse_rate("115200").is_err());
        assert!(parse_rate("fast").is_err());
        assert_eq!(parse_u16("0x23c").unwrap(), 0x23c);
        assert_eq!(parse_u16("572").unwrap(), 572);
        assert!(parse_u16("0xg").is_err());
    }

    /// The globals ride before or after the verb, as `osc` takes them.
    #[test]
    fn the_globals_parse_on_either_side_of_the_verb() {
        let cli = Cli::try_parse_from(["osc-lab", "-i", "3", "burst", "--duty-pct", "25"]).unwrap();
        assert_eq!((cli.id, cli.bus.baud.as_str()), (3, "auto"));
        let cli = Cli::try_parse_from([
            "osc-lab",
            "sweep",
            "-b",
            "3000000",
            "--deadline",
            "800",
            "--static-load",
        ])
        .unwrap();
        assert_eq!((cli.bus.baud.as_str(), cli.bus.deadline), ("3000000", 800));
        assert!(
            Cli::try_parse_from(["osc-lab", "burst"]).is_err(),
            "duties are required"
        );
    }
}
