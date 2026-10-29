//! Typed reads of one table field.

use anyhow::{Context, Result};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;

use crate::instrument::regs::Reg;

pub(crate) fn read_u16<P: Pipe>(c: &mut Client<P>, id: Id, reg: Reg) -> Result<u16> {
    let raw = c.read(id, reg.addr, 2).context("field read")?;
    Ok(u16::from_le_bytes([raw[0], raw[1]]))
}

pub(crate) fn read_u32<P: Pipe>(c: &mut Client<P>, id: Id, reg: Reg) -> Result<u32> {
    let raw = c.read(id, reg.addr, 4).context("field read")?;
    Ok(u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]))
}

pub(crate) fn read_bool<P: Pipe>(c: &mut Client<P>, id: Id, reg: Reg) -> Result<bool> {
    let raw = c.read(id, reg.addr, 1).context("field read")?;
    Ok(*raw.first().context("empty field read")? != 0)
}
