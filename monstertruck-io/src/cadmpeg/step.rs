//! Reading ISO 10303-21 through `cadmpeg-codec-step`, alongside our own reader.
//!
//! This is a **second** STEP reader, deliberately. [`crate::step`] is
//! monstertruck's own, built on `step-p21`, and it stays the default: it is the
//! measurement baseline the boolean kernel is characterised against -- the
//! frozen digests, the corpus conversion census and the failure classes all
//! describe the B-rep *it* produces. Replacing it is not a dependency swap, it
//! invalidates that evidence.
//!
//! So rather than choose blind, both are compiled in when the `cadmpeg` feature
//! is on, and can be run over the same corpus and compared. The swap becomes a
//! decision with a measurement behind it, or it does not happen.
//!
//! # What cadmpeg brings
//!
//! Measured 2026-08-06 against cadmpeg 0.4 over the ten in-repo STEP fixtures:
//! every one decodes, with exact analytic carriers (plane, cylinder, cone,
//! sphere, torus, NURBS) rather than tessellation, and topology that checks out
//! -- `occt-cube` gives 6 faces, 12 edges and 8 vertices, Euler 8 - 12 + 6 = 2.
//! `occt-assy` yields two bodies, so assemblies survive.
//!
//! That is a change from 2026-08-04, when `occt-cylinder.step` dropped its whole
//! `MANIFOLD_SOLID_BREP` because one parameter-space `CIRCLE` failed to decode.
//! That was cadmpeg/cadmpeg#79, and cadmpeg/cadmpeg#83 fixed it.
//!
//! # What is not yet decided
//!
//! Their STEP codec also *writes* (their support ladder calls it L9, which
//! includes source-less generation). That is untested here: "writes back a
//! document it decoded" is a weaker claim than "generates correct STEP from an
//! arbitrary foreign B-rep", which is what replacing our writer would need.
//! Until that is measured, writing STEP stays ours outright.

use crate::{
    Error, Result,
    cadmpeg::{ImportedSolid, to_solids},
};
use cadmpeg_core::ReadSeek;
use cadmpeg_ir::CadIr;
use cadmpeg_ir::codec::{CodecEntry, DecodeOptions};

const FORMAT: &str = "STEP (cadmpeg)";

/// Decode a STEP stream to cadmpeg's intermediate representation.
///
/// Stops one step short of monstertruck's B-rep on purpose. The comparison this
/// module exists for is against the *recovered geometry and topology*, and that
/// is what the intermediate representation carries; going all the way to a
/// [`ImportedSolid`] would fold our conversion's own defects into the reading
/// being measured.
pub fn decode_to_ir(reader: &mut dyn ReadSeek) -> Result<CadIr> {
    let decoded = cadmpeg_codec_step::StepCodec::default()
        .decode(reader, &DecodeOptions::default())
        .map_err(|error| Error::Decode {
            format: FORMAT,
            message: error.to_string(),
        })?;
    Ok(decoded.ir)
}

/// Decode a STEP file to cadmpeg's intermediate representation.
pub fn decode_file_to_ir(path: impl AsRef<std::path::Path>) -> Result<CadIr> {
    let mut file = std::fs::File::open(path)?;
    decode_to_ir(&mut file)
}

/// Read solids from a STEP stream, through cadmpeg rather than [`crate::step`].
///
/// Shares [`to_solids`] with every other format, so it carries that converter's
/// current state: while the conversion is unwritten this returns
/// [`Error::Unimplemented`] rather than an empty model.
pub fn from_reader(reader: &mut dyn ReadSeek) -> Result<Vec<ImportedSolid>> {
    let ir = decode_to_ir(reader)?;
    to_solids(&ir, FORMAT)
}

/// Read solids from a STEP file, through cadmpeg rather than [`crate::step`].
pub fn from_path(path: impl AsRef<std::path::Path>) -> Result<Vec<ImportedSolid>> {
    let mut file = std::fs::File::open(path)?;
    from_reader(&mut file)
}
