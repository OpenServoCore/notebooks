//! `osc-lab` - the raw capture instrument the notebooks record with, driven
//! through the osc-adapter: open-loop duty sweeps of TEL rungs (`sweep`),
//! high-rate shunt bursts from rest (`burst`), closed-loop goal schedules
//! (`track`) and loop-margin sweeps (`margin`). Raw captures only: nothing
//! here fits, identifies or calibrates. Every other verb (get, set, status,
//! baud) is `osc-bus`'s.

mod burst;
mod descriptor;
mod instrument;
mod margin;
mod rig;
mod state;
mod sweep;
mod track;

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
    /// Closed-loop schedule on the production image: goals committed with
    /// TEL bursts, raw rows under a host watchdog (track.csv, segments.csv
    /// + meta.json). Exit 2 refused before torque, 3 aborted.
    Track(track::Args),
    /// Loop-margin sweep on the `deadman` image: a triangle on the velocity
    /// loop's reference at a cruise, one TEL burst a point, raw margin rows
    /// (burst-K.csv + burst-K.json a burst, meta.json) as `oscnb.margin`
    /// reads them. Exit 2 refused, 3 aborted.
    Margin(margin::Args),
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
        Cmd::Track(args) => exit("track", track::run(args, cli.bus.clone(), cli.id)),
        Cmd::Margin(args) => exit("margin", margin::run(args, cli.bus.clone(), cli.id)),
    }
}

/// A closed-loop run's refusal and abort as their exit codes.
fn exit(verb: &str, r: Result<()>) -> Result<()> {
    if let Err(e) = &r
        && let Some(code) = track::exit_code(e)
    {
        eprintln!("osc-lab {verb}: {e}");
        std::process::exit(code);
    }
    r
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

    /// The globals ride before or after the verb, as `osc-bus` takes them.
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
