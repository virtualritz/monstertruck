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
    Result,
    cadmpeg::{DecoderLoss, Import, ImportedBody, decoded, import_reported, to_bodies},
};
use cadmpeg_core::ReadSeek;
use cadmpeg_core::decode::DecodePolicy;
use cadmpeg_ir::CadIr;

const FORMAT: &str = "STEP (cadmpeg)";

fn decoded_step<T>(
    reader: &mut dyn ReadSeek,
    convert: impl FnOnce(&CadIr, Vec<DecoderLoss>) -> Result<T>,
) -> Result<T> {
    decoded(
        &cadmpeg_codec_step::StepCodec::default(),
        FORMAT,
        reader,
        DecodePolicy::default(),
        convert,
    )
}

/// Decode a STEP stream to cadmpeg's intermediate representation.
///
/// Stops one step short of monstertruck's B-rep on purpose. The comparison this
/// module exists for is against the *recovered geometry and topology*, and that
/// is what the intermediate representation carries; going all the way to a
/// [`ImportedSolid`](crate::cadmpeg::ImportedSolid) would fold our conversion's
/// own defects into the reading being measured.
///
/// Returns the decoder's loss notes beside the document, so that a caller
/// measuring the reader sees what it dropped as well as what it kept.
pub fn decode_to_ir_reported(reader: &mut dyn ReadSeek) -> Result<(CadIr, Vec<DecoderLoss>)> {
    decoded_step(reader, |ir, losses| Ok((ir.clone(), losses)))
}

/// Decode a STEP stream to cadmpeg's intermediate representation, discarding
/// the decoder's loss notes. See [`decode_to_ir_reported`] to keep them.
pub fn decode_to_ir(reader: &mut dyn ReadSeek) -> Result<CadIr> {
    decode_to_ir_reported(reader).map(|(ir, _losses)| ir)
}

/// Decode a STEP file to cadmpeg's intermediate representation.
pub fn decode_file_to_ir(path: impl AsRef<std::path::Path>) -> Result<CadIr> {
    let mut file = std::fs::File::open(path)?;
    decode_to_ir(&mut file)
}

/// Read a STEP stream through cadmpeg, keeping every body that converts.
///
/// The counterpart of [`crate::iges::from_reader`]: bodies that convert,
/// bodies that did not (each named), and the decoder's own loss notes. Prefer
/// this to [`from_reader`] whenever a partial model is better than none.
pub fn import_from_reader(reader: &mut dyn ReadSeek) -> Result<Import> {
    decoded_step(reader, |ir, losses| import_reported(ir, FORMAT, losses))
}

/// Read a STEP file through cadmpeg, keeping every body that converts.
pub fn import_from_path(path: impl AsRef<std::path::Path>) -> Result<Import> {
    let mut file = std::fs::File::open(path)?;
    import_from_reader(&mut file)
}

/// Read bodies from a STEP stream, through cadmpeg rather than [`crate::step`].
///
/// The strict reading: any body that cannot be represented fails the call.
/// Strict about bodies only -- the decoder's loss notes do not fail it, and a
/// `Vec` has nowhere to report them; use [`import_from_reader`] to see them.
pub fn from_reader(reader: &mut dyn ReadSeek) -> Result<Vec<ImportedBody>> {
    decoded_step(reader, |ir, _losses| to_bodies(ir, FORMAT))
}

/// Read bodies from a STEP file, through cadmpeg rather than [`crate::step`].
pub fn from_path(path: impl AsRef<std::path::Path>) -> Result<Vec<ImportedBody>> {
    let mut file = std::fs::File::open(path)?;
    from_reader(&mut file)
}
