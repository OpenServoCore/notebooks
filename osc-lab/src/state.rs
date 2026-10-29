//! Data state before a capture: the one-line warning a capture prints when
//! `data_flags` names a reason. The capture still runs - OpenLoop works on
//! such a servo; the closed loops are what the servo refuses.

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::data_state::DataState;
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;

use crate::descriptor;

/// The servo's descriptor (the built-in plus the operator's overrides,
/// picked by one identity read), owned so callers keep no registry around.
pub(crate) fn descriptor<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Descriptor> {
    let reg = descriptor::load()?;
    let b = c.read(id, 0, 4)?;
    if b.len() < 4 {
        bail!("identity read returned {} B, expected 4", b.len());
    }
    let model = u16::from_le_bytes([b[0], b[1]]);
    let fw = u16::from_le_bytes([b[2], b[3]]);
    let (d, note) = descriptor::select(&reg, model, fw)?;
    if let Some(note) = note {
        println!("{note}");
    }
    Ok(d.clone())
}

/// Read the data state; a set reason prints the one-line warning.
pub(crate) fn check<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<DataState> {
    let d = descriptor(c, id)?;
    let s = c.data_state(id, &d)?;
    if let Some(msg) = s.message() {
        eprintln!("warning: id {}: {} - {msg}", id.as_byte(), s.names());
    }
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::testkit;

    /// A servo off the calibration bench picks the built-in and reads
    /// clean; one whose covered fields were edited since its stamp reads
    /// STAMP_MISMATCH, and the capture still goes on.
    #[test]
    fn the_data_state_is_read_and_never_refused() {
        use osc_client::data_state::STAMP_MISMATCH;
        use osc_client::fake::FakePipe;
        let mut pipe = FakePipe::new(osc_client::BaudRate::B1000000, &[1]);
        pipe.seed_calibrated(0);
        let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
        let d = descriptor(&mut c, id).unwrap();
        assert_eq!(d.model, "osc-servo");
        assert_eq!(check(&mut c, id).unwrap().flags, 0);
        let (mut c, id) = testkit::table(1961);
        assert_eq!(check(&mut c, id).unwrap().flags, STAMP_MISMATCH);
    }
}
