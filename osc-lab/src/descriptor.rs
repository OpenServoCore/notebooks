//! The servo's descriptor: the built-in one is the pinned firmware's own
//! control table (`ControlTable::FIELDS`, the source the firmware repo
//! exports its descriptor JSON from), overlaid by any operator descriptor in
//! the config dir. Types, selection and the value codec live in
//! `osc_client::descriptor`.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use control_table::descriptor::FieldKind;
use osc_client::descriptor::{
    Access, Descriptor, Field, Kind, Registry, Selection, StampSpec, Variant,
};
use osc_protocol::models::MODEL_OSC_SERVO;
use osc_protocol::version::unpack_version;
use osc_servo_core::regions::ControlTable;

/// The descriptor of the firmware this tool was built against.
pub fn builtin() -> Descriptor {
    let fields = ControlTable::FIELDS
        .iter()
        .map(|d| {
            let (kind, variants) = match d.kind {
                FieldKind::UInt => (Kind::Uint, Vec::new()),
                FieldKind::Int => (Kind::Int, Vec::new()),
                FieldKind::Bool => (Kind::Bool, Vec::new()),
                FieldKind::Bytes => (Kind::Bytes, Vec::new()),
                FieldKind::Enum(vs) => (
                    Kind::Enum,
                    vs.iter()
                        .map(|v| Variant {
                            name: v.name.into(),
                            value: v.value,
                        })
                        .collect(),
                ),
            };
            Field {
                name: d.name.into(),
                addr: d.addr,
                width: d.width,
                access: if d.writable { Access::Rw } else { Access::Ro },
                kind,
                min: d.min.map(i64::from),
                max: d.max.map(i64::from),
                variants,
            }
        })
        .collect();
    let (firmware_major, firmware_minor, _) = unpack_version(osc_servo_core::FIRMWARE_VERSION);
    Descriptor {
        format: osc_client::descriptor::FORMAT,
        model: "osc-servo".into(),
        model_number: MODEL_OSC_SERVO,
        firmware_major,
        firmware_minor,
        table_size: core::mem::size_of::<ControlTable>() as u16,
        stamp: Some(StampSpec {
            tag: osc_servo_core::stamp::TAG.into(),
            covered: osc_servo_core::stamp::COVERED_NAMES
                .iter()
                .map(|n| n.to_string())
                .collect(),
            pos_lut_points: osc_servo_core::pos_lut::INTERVALS,
        }),
        fields,
    }
}

/// The built-in overlaid by every external descriptor; an external file
/// sharing the built-in's model and major.minor replaces it.
pub fn load() -> Result<Registry> {
    let mut reg = Registry::new();
    reg.push(builtin());
    for path in external_paths()? {
        let text = std::fs::read_to_string(&path)
            .with_context(|| format!("reading descriptor {}", path.display()))?;
        reg.push(
            Descriptor::parse(&text)
                .with_context(|| format!("parsing descriptor {}", path.display()))?,
        );
    }
    Ok(reg)
}

/// Pick the descriptor for a servo-reported (model, packed fw). Returns the
/// selection plus any advisory note to print at the call site.
pub fn select(reg: &Registry, model: u16, fw: u16) -> Result<(&Descriptor, Option<String>)> {
    let (major, minor, _) = unpack_version(fw);
    match reg.select(model, fw) {
        Selection::Exact(d) => return Ok((d, None)),
        Selection::OlderMinor(d) => {
            let note = format!(
                "note: descriptor {} is fw {}.{}, servo reports fw {major}.{minor}; fields added since are unnamed",
                d.model, d.firmware_major, d.firmware_minor
            );
            return Ok((d, Some(note)));
        }
        Selection::Incompatible { newest: (a, b) } => bail!(
            "no descriptor for model {model:#06x} fw {major}.{minor}; newest known {a}.{b}\ndrop a matching *.json in {}",
            external_dir().display(),
        ),
        Selection::UnknownModel => {}
    }
    let known: Vec<&Descriptor> = reg.iter().collect();
    // model 0 is unassigned (unprovisioned or dev firmware). With exactly
    // one descriptor known there is no ambiguity to resolve.
    if model == 0
        && let [only] = known.as_slice()
    {
        let note = format!(
            "note: servo model unassigned; assuming {} (the only known descriptor)",
            only.model
        );
        return Ok((only, Some(note)));
    }
    let mut known: Vec<String> = known
        .iter()
        .map(|d| {
            format!(
                "{} {}.{} ({:#06x})",
                d.model, d.firmware_major, d.firmware_minor, d.model_number
            )
        })
        .collect();
    known.sort();
    bail!(
        "no descriptor for model {model:#06x}; known: {}\ndrop a matching *.json in {}",
        known.join(", "),
        external_dir().display(),
    );
}

/// External descriptor directory: `$OSC_CONFIG_DIR/descriptors` else
/// `$HOME/.config/osc/descriptors`, the one `osc` reads.
fn external_dir() -> PathBuf {
    if let Some(dir) = std::env::var_os("OSC_CONFIG_DIR") {
        return Path::new(&dir).join("descriptors");
    }
    let home = std::env::var_os("HOME").unwrap_or_default();
    Path::new(&home).join(".config/osc/descriptors")
}

/// Every `*.json` under the external dir, sorted for deterministic overlay.
/// A missing dir is not an error (operators need not have one).
fn external_paths() -> Result<Vec<PathBuf>> {
    let dir = external_dir();
    let entries = match std::fs::read_dir(&dir) {
        Ok(e) => e,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(e) => return Err(e).with_context(|| format!("reading {}", dir.display())),
    };
    let mut paths: Vec<PathBuf> = Vec::new();
    for entry in entries {
        let path = entry?.path();
        if path.extension().is_some_and(|e| e == "json") {
            paths.push(path);
        }
    }
    paths.sort();
    Ok(paths)
}

/// Find a field by exact name; on a miss, suggest near matches so a typo
/// is one correction away.
pub fn field<'a>(d: &'a Descriptor, name: &str) -> Result<&'a Field> {
    if let Some(f) = d.field(name) {
        return Ok(f);
    }
    let near: Vec<&str> = d
        .fields
        .iter()
        .filter(|f| f.name.contains(name) || name.contains(f.name.as_str()))
        .map(|f| f.name.as_str())
        .collect();
    if near.is_empty() {
        bail!("no field named {name:?}");
    }
    bail!("no field named {name:?}; did you mean: {}", near.join(", "));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The built-in names the servo model, its fw and the stamp recipe, and
    /// carries every field the capture reads by name.
    #[test]
    fn the_builtin_is_the_pinned_control_table() {
        let d = builtin();
        assert_eq!(d.model_number, 0x0101);
        assert_eq!(d.fields.len(), ControlTable::FIELDS.len());
        let stamp = d.stamp.as_ref().unwrap();
        assert_eq!(stamp.tag, "osc-plant-1");
        assert_eq!(stamp.pos_lut_points, 256);
        for name in &stamp.covered {
            assert!(d.field(name).is_some(), "{name} not a field");
        }
        for name in ["openloop_decay", "openloop_zero_brake", "tel_count"] {
            assert!(field(&d, name).is_ok(), "{name}");
        }
        let id = d.field("id").unwrap();
        assert_eq!((id.min, id.max), (Some(1), Some(249)));
        assert!(d.stamp().is_ok(), "the client reads the recipe");
        let mut reg = Registry::new();
        reg.push(d.clone());
        let fw = osc_servo_core::FIRMWARE_VERSION;
        assert!(select(&reg, 0x0101, fw).unwrap().1.is_none());
        assert!(select(&reg, 0x0202, fw).is_err());
    }

    #[test]
    fn a_missed_field_name_suggests_near_ones() {
        let d = builtin();
        let e = field(&d, "decay").unwrap_err().to_string();
        assert!(e.contains("openloop_decay"), "{e}");
    }
}
