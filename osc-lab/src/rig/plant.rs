//! The plant data the servo holds beside its recordings: the position table
//! as the kernel applies it and the stored state. A capture is only as
//! interpretable as this record: the same raw pot linearizes to different
//! counts under different tables.

use anyhow::Result;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::pos_lut::{INTERVALS, state};
use osc_client::store_state::StoreState;
use osc_protocol::crc::osc_crc;
use osc_servo_core::pos_lut::GRID;
use serde_json::{Value, json};

/// The effective table: the array while LIVE, the identity otherwise.
pub(crate) struct Lut {
    pub(crate) state: u8,
    pub(crate) points: [i16; INTERVALS],
}

impl Lut {
    pub(crate) fn read<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<Self> {
        let s = c.pos_lut_state(id, d)?;
        let points = if s == state::LIVE {
            c.pos_lut(id, d)?.points
        } else {
            [0; INTERVALS]
        };
        Ok(Self { state: s, points })
    }

    pub(crate) fn state_name(&self) -> String {
        match state::name(self.state) {
            Some(n) => n.to_string(),
            None => format!("state {}", self.state),
        }
    }

    /// CRC-16/ARC over the effective points LE; the identity has its own
    /// value.
    pub(crate) fn crc(&self) -> u16 {
        let bytes: Vec<u8> = self.points.iter().flat_map(|k| k.to_le_bytes()).collect();
        osc_crc(&bytes)
    }

    pub(crate) fn nonzero(&self) -> usize {
        self.points.iter().filter(|&&k| k != 0).count()
    }

    /// Raw counts of the first and last nonzero point.
    pub(crate) fn band(&self) -> Option<(u16, u16)> {
        let first = self.points.iter().position(|&k| k != 0)?;
        let last = self.points.iter().rposition(|&k| k != 0)?;
        Some((first as u16 * GRID as u16, last as u16 * GRID as u16))
    }

    /// The compact record a recording's meta carries.
    pub(crate) fn json(&self) -> Value {
        json!({
            "lut_state": self.state_name(),
            "lut_crc": format!("{:#06x}", self.crc()),
            "lut_nonzero_points": self.nonzero(),
            "lut_band": self.band().map(|(lo, hi)| [lo, hi]),
        })
    }
}

/// The table and the stored state as one reading.
pub(crate) struct Snapshot {
    pub(crate) lut: Lut,
    pub(crate) store: StoreState,
}

impl Snapshot {
    pub(crate) fn read<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<Self> {
        let lut = Lut::read(c, id, d)?;
        let store = c.store_state(id, d)?;
        Ok(Self { lut, store })
    }

    /// The `plant` block of a recording's meta: the table, the settings
    /// word (`loaded`, or the images on their defaults), the modes the
    /// servo refuses and whether RAM differs from what a reboot loads.
    pub(crate) fn json(&self) -> Value {
        let mut v = self.lut.json();
        v["settings"] = json!(self.store.settings());
        v["refused_modes"] = json!(self.store.refused_modes);
        v["unsaved"] = json!(self.store.unsaved());
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_client::store_state::{CALIB_DEFAULTS, UNSAVED};

    fn bent() -> Lut {
        let mut points = [0i16; INTERVALS];
        for (k, c) in points.iter_mut().enumerate().take(220).skip(35) {
            *c = (k as i16 - 35) % 7 + 1;
        }
        Lut {
            state: state::LIVE,
            points,
        }
    }

    #[test]
    fn identity_and_a_table_record_themselves() {
        let id = Lut {
            state: state::IDENTITY,
            points: [0; INTERVALS],
        };
        assert_eq!(id.state_name(), "IDENTITY");
        assert_eq!(id.band(), None);
        let b = bent();
        assert_eq!(b.nonzero(), 185);
        assert_eq!(b.band(), Some((560, 3504)));
        assert_ne!(b.crc(), id.crc());
        let zero = Lut {
            state: state::LIVE,
            points: [0; INTERVALS],
        };
        assert_eq!(
            zero.crc(),
            id.crc(),
            "an all-zero LIVE table hashes like the identity"
        );
        assert_eq!(id.json()["lut_crc"], "0x0000");
    }

    #[test]
    fn snapshot_records_table_and_store() {
        let a = Snapshot {
            lut: bent(),
            store: StoreState {
                status: 0,
                refused: 0,
                fault_code: 0,
                refused_modes: Vec::new(),
            },
        };
        let j = a.json();
        assert_eq!(j["lut_state"], "LIVE");
        assert_eq!(j["lut_nonzero_points"], 185);
        assert_eq!(j["lut_band"], json!([560, 3504]));
        assert_eq!(j["lut_crc"], format!("{:#06x}", bent().crc()));
        assert_eq!(j["settings"], "loaded");
        assert_eq!(j["refused_modes"], json!([]));
        assert_eq!(j["unsaved"], false);
        assert!(j.get("stamp_verdict").is_none());

        let b = Snapshot {
            lut: Lut {
                state: state::IDENTITY,
                points: [0; INTERVALS],
            },
            store: StoreState {
                status: CALIB_DEFAULTS | UNSAVED,
                refused: 1 << 3,
                fault_code: 0,
                refused_modes: vec!["Position".into()],
            },
        };
        let j = b.json();
        assert_eq!(j["settings"], "defaults: CALIB");
        assert_eq!(j["refused_modes"], json!(["Position"]));
        assert_eq!(j["unsaved"], true);
        assert_eq!(j["lut_band"], Value::Null);
    }
}
