//! Reading IGES, via `cadmpeg-codec-iges`.
//!
//! monstertruck has no IGES reader of its own and writing one is not worth the
//! effort: cadmpeg already decodes the format to exact analytic carriers and
//! reports what it could not. This module is only the decoder call; the work is
//! in [`crate::cadmpeg`].
//!
//! # Which versions
//!
//! cadmpeg 0.6 decodes fixed-format ASCII IGES **4.0 through 5.3**. 0.5 read
//! only 5.1 to 5.3 and refused a 4.0 file before this crate saw anything; the
//! envelope widened upstream (cadmpeg/cadmpeg#133). Compressed ASCII and binary
//! IGES are read-only upstream and untested here.
//!
//! # Bounding a decode, and what that does not buy you
//!
//! cadmpeg bounds a decode with [`ResourceLimits`], and its default is
//! `desktop()`: 4 GiB retained, 4 GiB materialised, 64 million entities, 4 billion
//! work units. [`from_path`] uses that default.
//!
//! **These are self-accounting tallies, not a memory bound.** The codec charges
//! figures it computes -- entity counts, work units, retained bytes -- against the
//! ceiling and refuses when they cross. Nothing observes the allocator, so a
//! process can exceed the byte figures without a charge tripping, and a machine
//! with 128 GB is not protected by a 4 GiB number. A hard bound has to come from
//! outside the process: a cgroup (`systemd-run -p MemoryMax=`) or `ulimit -v`.
//!
//! What the ceiling IS good for is refusing pathological input EARLY, with a typed
//! [`crate::Error::Decode`] instead of a long grind. `DecodePolicy::service()`
//! caps retained and materialised bytes at 512 MiB and entities at 8 million;
//! [`from_path_with_policy`] takes any policy you like.
//!
//! For scale: an import over a 28 MB IGES 5.2 assembly peaks at
//! 923 MB RSS, measured in a cgroup with swap disabled.
//!
//! # What comes back
//!
//! [`from_path`] returns an [`Import`], not a bare list. A real IGES assembly
//! carries construction wires beside its surfaces, and those have no monstertruck
//! equivalent -- so the wires are NAMED in [`Import::refused`] while the surfaces
//! come back in [`Import::bodies`]. See [`Import`] for why both halves matter.

use crate::{
    Result,
    cadmpeg::{Import, ImportedBody, decoded, import_reported, to_bodies},
};
use cadmpeg_core::ReadSeek;

/// Re-exported so bounding a decode does not require a direct cadmpeg dependency.
/// See the module note -- these bound the codec's own accounting, not memory.
pub use cadmpeg_core::decode::{DecodeMode, DecodePolicy, ResourceLimits};

const FORMAT: &str = "IGES";

/// Decode with the IGES codec; see [`crate::cadmpeg`]'s `decoded` for why the
/// document is lent to `convert` rather than returned.
fn decoded_iges<T>(
    reader: &mut dyn ReadSeek,
    policy: DecodePolicy,
    convert: impl FnOnce(&cadmpeg_ir::CadIr, Vec<crate::cadmpeg::DecoderLoss>) -> Result<T>,
) -> Result<T> {
    decoded(
        &cadmpeg_codec_iges::IgesCodec,
        FORMAT,
        reader,
        policy,
        convert,
    )
}

/// Read an IGES stream, keeping every body that converts.
///
/// Bodies with no monstertruck equivalent are named in [`Import::refused`]
/// rather than dropped or allowed to fail the whole file.
pub fn from_reader(reader: &mut dyn ReadSeek) -> Result<Import> {
    decoded_iges(reader, DecodePolicy::default(), |ir, losses| {
        import_reported(ir, FORMAT, losses)
    })
}

/// Read an IGES file, keeping every body that converts.
pub fn from_path(path: impl AsRef<std::path::Path>) -> Result<Import> {
    let mut file = std::fs::File::open(path)?;
    from_reader(&mut file)
}

/// Read an IGES stream, failing if any body cannot be represented.
///
/// The strict reading, for a caller to whom a partial model is worse than none.
/// For most IGES input this fails on the construction wires -- see [`from_reader`].
///
/// Strict about BODIES only. The decoder's loss notes do not fail this call --
/// nearly every real file carries informational ones -- and a `Vec` has nowhere
/// to report them, so a caller that needs them uses [`from_reader`].
pub fn all_bodies_from_reader(reader: &mut dyn ReadSeek) -> Result<Vec<ImportedBody>> {
    decoded_iges(reader, DecodePolicy::default(), |ir, _losses| {
        to_bodies(ir, FORMAT)
    })
}

/// Read an IGES file, failing if any body cannot be represented.
pub fn all_bodies_from_path(path: impl AsRef<std::path::Path>) -> Result<Vec<ImportedBody>> {
    let mut file = std::fs::File::open(path)?;
    all_bodies_from_reader(&mut file)
}

/// Read an IGES stream under an explicit resource ceiling.
///
/// Reach for this to refuse pathological input early rather than grind through it:
/// a decode that crosses the ceiling fails as [`crate::Error::Decode`], which is
/// catchable. It is NOT a memory guarantee -- see the module note.
pub fn from_reader_with_policy(reader: &mut dyn ReadSeek, policy: DecodePolicy) -> Result<Import> {
    decoded_iges(reader, policy, |ir, losses| {
        import_reported(ir, FORMAT, losses)
    })
}

/// Read an IGES file under an explicit resource ceiling.
pub fn from_path_with_policy(
    path: impl AsRef<std::path::Path>,
    policy: DecodePolicy,
) -> Result<Import> {
    let mut file = std::fs::File::open(path)?;
    from_reader_with_policy(&mut file, policy)
}
