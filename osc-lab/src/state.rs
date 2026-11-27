//! Stored state before a capture: the one-line warning a capture prints
//! when an image is on its defaults or the belt refuses a mode. The capture
//! still runs: it is information, and the servo refuses a mode only on a
//! constant it cannot run on.

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::store_state::{self, StoreState};

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

/// Read the stored state; an image on its defaults or a refused mode
/// prints its one-line warning. Unsaved changes print none: a capture's
/// own volatile writes make them.
pub(crate) fn check<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<StoreState> {
    let d = descriptor(c, id)?;
    let s = c.store_state(id, &d)?;
    for msg in s.messages() {
        if msg != store_state::NOT_SAVED {
            eprintln!("warning: id {}: {msg}", id.as_byte());
        }
    }
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::testkit;

    /// A servo off the calibration bench picks the built-in, loaded and
    /// refusing nothing, and stays so through the bench table's edits.
    #[test]
    fn the_stored_state_is_read_and_never_refused() {
        use osc_client::fake::FakePipe;
        let mut pipe = FakePipe::new(osc_client::BaudRate::B1000000, &[1]);
        pipe.seed_calibrated(0);
        let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
        let d = descriptor(&mut c, id).unwrap();
        assert_eq!(d.model, "osc-servo");
        let s = check(&mut c, id).unwrap();
        assert!(s.loaded() && s.refused == 0, "{s:?}");
        let (mut c, id) = testkit::table(1961);
        let s = check(&mut c, id).unwrap();
        assert!(s.loaded() && s.refused == 0, "{s:?}");
    }
}
