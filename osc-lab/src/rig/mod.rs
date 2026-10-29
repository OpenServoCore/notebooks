//! Rig plumbing for the captures: the bus connection, the servo's limits
//! and the battery gate, the wire moves (TEL bursts and the high-rate
//! burst included), the plant record a meta carries, and the horn park.

pub(crate) mod battery;
pub(crate) mod limits;
pub(crate) mod park;
pub(crate) mod plant;
pub(crate) mod pump;
pub(crate) mod servo;
pub(crate) mod snapshot;
#[cfg(test)]
pub(crate) mod testkit;

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_protocol::wire::DEFAULT_RESPONSE_DEADLINE_US;

use crate::instrument::seek::Blocked;

/// A drive that stopped on a blocked shaft, which nothing may drive again,
/// not even to centre it.
#[derive(Debug)]
pub(crate) struct Aborted {
    pub(crate) what: &'static str,
    pub(crate) reason: Blocked,
}

impl std::fmt::Display for Aborted {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} aborted: {}; a blocked shaft cannot be centred, so torque is off and the shaft \
             stays where it stopped",
            self.what, self.reason
        )
    }
}

impl std::error::Error for Aborted {}

/// The error is a blocked shaft: nothing may drive it again.
pub(crate) fn blocked(e: &anyhow::Error) -> bool {
    e.downcast_ref::<Aborted>().is_some()
}

/// The bus every verb connects to: its rate and the host's
/// RESPONSE_DEADLINE, which the adapter mirror and every addressed servo take.
#[derive(clap::Args, Debug, Clone)]
pub(crate) struct Bus {
    /// Wire baud, or `auto` to find the bus by ENUM probe.
    #[arg(short, long, global = true, default_value = "auto")]
    pub(crate) baud: String,
    /// RESPONSE_DEADLINE in us, set on the adapter and on each addressed
    /// servo that differs (protocol sec 5.4).
    #[arg(long, global = true, default_value_t = DEFAULT_RESPONSE_DEADLINE_US)]
    pub(crate) deadline: u16,
}

/// Connect at the bus rate and bring the adapter and `ids` to the host's
/// RESPONSE_DEADLINE.
pub(crate) fn connect(bus: &Bus, ids: &[Id]) -> Result<Client<NusbPipe>> {
    let mut c = Client::connect(NusbPipe::open()?)?;
    match bus.baud.as_str() {
        "auto" => match c.find_bus_baud()? {
            Some(rate) => eprintln!("bus at {} baud", rate.as_hz()),
            None => bail!("no servo bus found at any supported baud"),
        },
        s => c.host_baud(crate::parse_rate(s)?)?,
    }
    for (id, held) in c.sync_response_deadline(ids, bus.deadline)? {
        eprintln!(
            "id {}: response_deadline_us {held} -> {} (volatile until SAVE)",
            id.as_byte(),
            bus.deadline
        );
    }
    Ok(c)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_blocked_shaft_is_told_from_any_other_failure() {
        let e: anyhow::Error = Aborted {
            what: "the seek",
            reason: Blocked {
                pos: 2048,
                moved: 3,
            },
        }
        .into();
        assert!(blocked(&e));
        assert_eq!(
            e.to_string(),
            "the seek aborted: the shaft is blocked or the position sensor is not reading (pos \
             2048, moved 3 counts); a blocked shaft cannot be centred, so torque is off and the \
             shaft stays where it stopped"
        );
        assert!(!blocked(&anyhow::anyhow!("interrupted")));
    }
}
