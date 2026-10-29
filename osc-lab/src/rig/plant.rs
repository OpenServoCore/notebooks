//! The plant data the servo holds beside its recordings: the position table
//! as the kernel applies it, the plant stamp against the live set, and the
//! data state. A capture is only as interpretable as this record: the same
//! raw pot linearizes to different counts under different tables.

use anyhow::Result;
use osc_client::blocking::Client;
use osc_client::data_state::DataState;
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::pos_lut::{INTERVALS, state};
use osc_client::stamp::{UNSTAMPED, Verdict};
use osc_client::{Error, Id};
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

    /// CRC-16/ARC over the effective points LE, the bytes as the stamp
    /// hashes them; the identity has its own value.
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

/// The table, the stamp verdict and the data state as one reading.
pub(crate) struct Snapshot {
    pub(crate) lut: Lut,
    /// None when the descriptor carries no stamp recipe.
    pub(crate) stamp: Option<Verdict>,
    pub(crate) data: DataState,
}

impl Snapshot {
    pub(crate) fn read<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<Self> {
        let lut = Lut::read(c, id, d)?;
        let stamp = match c.stamp_verdict(id, d) {
            Ok(v) => Some(v),
            Err(Error::Descriptor(_)) => None,
            Err(e) => return Err(e.into()),
        };
        let data = c.data_state(id, d)?;
        Ok(Self { lut, stamp, data })
    }

    pub(crate) fn stamp_verdict(&self) -> &'static str {
        match self.stamp {
            None => "none",
            Some(v) if v.stored == UNSTAMPED => "unstamped",
            Some(v) if v.matches() => "match",
            Some(_) => "mismatch",
        }
    }

    pub(crate) fn data_names(&self) -> Vec<&'static str> {
        self.data.reasons().iter().map(|r| r.name()).collect()
    }

    /// The `plant` block of a recording's meta.
    pub(crate) fn json(&self) -> Value {
        let mut v = self.lut.json();
        v["plant_stamp"] = json!(self.stamp.map(|s| format!("{:#06x}", s.stored)));
        v["stamp_computed"] = json!(self.stamp.map(|s| format!("{:#06x}", s.computed)));
        v["stamp_verdict"] = json!(self.stamp_verdict());
        v["data_flags"] = json!(self.data_names());
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_client::data_state::STAMP_MISMATCH;

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
    fn snapshot_records_table_stamp_and_data() {
        let a = Snapshot {
            lut: bent(),
            stamp: Some(Verdict {
                stored: 0x1234,
                computed: 0x1234,
            }),
            data: DataState {
                flags: 0,
                fault_code: 0,
            },
        };
        let j = a.json();
        assert_eq!(j["lut_state"], "LIVE");
        assert_eq!(j["lut_nonzero_points"], 185);
        assert_eq!(j["lut_band"], json!([560, 3504]));
        assert_eq!(j["lut_crc"], format!("{:#06x}", bent().crc()));
        assert_eq!(j["plant_stamp"], "0x1234");
        assert_eq!(j["stamp_verdict"], "match");
        assert_eq!(j["data_flags"], json!([]));

        let b = Snapshot {
            lut: bent(),
            stamp: Some(Verdict {
                stored: 0x1234,
                computed: 0x5678,
            }),
            data: DataState {
                flags: STAMP_MISMATCH,
                fault_code: 0,
            },
        };
        assert_eq!(b.json()["stamp_verdict"], "mismatch");
        assert_eq!(b.json()["data_flags"], json!(["STAMP_MISMATCH"]));

        let unstamped = Snapshot {
            lut: Lut {
                state: state::IDENTITY,
                points: [0; INTERVALS],
            },
            stamp: Some(Verdict {
                stored: UNSTAMPED,
                computed: 7,
            }),
            data: a.data,
        };
        assert_eq!(unstamped.stamp_verdict(), "unstamped");
        assert_eq!(unstamped.json()["lut_band"], Value::Null);
        let no_recipe = Snapshot {
            stamp: None,
            ..unstamped
        };
        assert_eq!(no_recipe.json()["plant_stamp"], Value::Null);
        assert_eq!(no_recipe.stamp_verdict(), "none");
    }
}
