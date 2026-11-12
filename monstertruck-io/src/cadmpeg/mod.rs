//! The one converter: `cadmpeg`'s intermediate representation to monstertruck.
//!
//! Every cadmpeg codec decodes into [`cadmpeg_ir::CadIr`], so this is the only
//! place that has to understand recovered geometry, and each format module above
//! it is a few lines choosing a decoder. Adding a format should not touch this
//! file except to widen what it already handles.
//!
//! # What the conversion has to get right
//!
//! `CadIr` carries a flat, table-shaped B-rep -- bodies, regions, shells, faces,
//! loops, coedges, edges, vertices, plus separate surface, curve and pcurve
//! tables -- which is the same shape monstertruck's [`CompressedShell`] uses, so
//! the mapping is mostly index translation. Three things are not:
//!
//! * **Analytic carriers must stay analytic.** A cylinder arriving as
//!   [`Surface::Plane`]-adjacent NURBS is a silent loss of exactness that the
//!   boolean kernel pays for later. Verified 2026-08-04 that cadmpeg does keep
//!   them: a real part decoded as 34 planes, 38 cylinders, 10 tori and 4 NURBS
//!   rather than 82 spline patches.
//! * **Units.** cadmpeg normalises to millimetres on decode. Anything that
//!   assumes the source unit will be wrong by a factor.
//! * **Loss.** cadmpeg reports what it dropped, per entity, with a reason. That
//!   report must reach the caller rather than being discarded: a failed decode
//!   carries the decoder's own message in [`Error::Decode`], and a SUCCESSFUL
//!   decode carries the decoder's loss notes in [`Import::losses`]. Until 0.6
//!   only the first half held, and every loss on a decode that succeeded stopped
//!   at the decoder boundary.
//!
//! [`CompressedShell`]: monstertruck_topology::compress::CompressedShell
//! [`Surface::Plane`]: monstertruck_modeling::Surface

mod convert;
pub mod step;

use crate::{Error, Result};
use cadmpeg_core::ReadSeek;
use cadmpeg_core::decode::{DecodeDepth, DecodePolicy};
use cadmpeg_ir::codec::{Codec, DecodeOptions};
use monstertruck_modeling::{Curve, Point3, Surface};
use monstertruck_topology::compress::{CompressedShell, CompressedSolid};

/// Where in the source a decoder loss occurred, re-exported so a caller can
/// read [`DecoderLoss::provenance`] without naming cadmpeg directly.
pub use cadmpeg_ir::SourceProvenance;

/// A solid as monstertruck's kernel wants it, over the canonical curve and
/// surface enums the boolean pipeline consumes.
pub type ImportedSolid = CompressedSolid<Point3, Curve, Surface>;

/// An open shell: faces that do not bound a volume.
pub type ImportedSheet = CompressedShell<Point3, Curve, Surface>;

/// A body recovered from an exchange file.
///
/// Not every format yields solids. Measured 2026-08-06 against cadmpeg 0.4 over
/// its own IGES fixtures: STEP gives `solid` bodies, IGES gives `sheet` and
/// `wire`. IGES in the wild is often a surface collection with trimming curves,
/// not a closed B-rep, so a `Vec<ImportedSolid>` cannot carry most IGES files.
///
/// Wire bodies have no faces and no monstertruck equivalent. They are reported
/// as [`Error::UnsupportedBodyKind`] rather than dropped, so a caller never
/// receives fewer bodies than the file holds without being told.
#[derive(Debug, Clone)]
pub enum ImportedBody {
    /// Shells that bound a volume.
    Solid(ImportedSolid),
    /// Faces that do not bound a volume.
    Sheet(ImportedSheet),
}

/// One body (or one placed instance) the converter could not represent, and why.
///
/// Named, not counted: a caller that wants to warn, log, or re-export needs to
/// know WHICH body and WHAT about it, and "3 bodies were skipped" is not enough
/// to act on. [`Refusal::instances`] says how many uses one entry stands for.
#[derive(Debug)]
pub struct Refusal {
    /// The body's identity in the decoded document.
    pub body: String,
    /// The body kind the decoder reported, when it is the reason.
    pub kind: &'static str,
    /// How many placed instances this one refusal stands for.
    ///
    /// A body that cannot be CONVERTED is refused before its instances are
    /// expanded, so one entry can stand for every use in the assembly. A body
    /// that converts but whose placement cannot be applied is refused per
    /// instance, so each entry stands for one. [`Import::total`] sums this.
    pub instances: usize,
    /// The typed failure.
    pub reason: Error,
}

/// What an import recovered, and what it did not.
///
/// # Why this is not a bare `Vec`
///
/// A `Vec` cannot say that one of its entries was left out, so the first version
/// of this converter failed the WHOLE call when any single body could not be
/// represented. That is safe and useless in equal measure: a real IGES assembly
/// carries construction wires alongside its surfaces, and one wire body would
/// discard the other 665.
///
/// Measured on a 28 MB IGES 5.2 export: 665 sheet bodies and 1 wire body. The
/// strict rule returned an error for a file that is 665/666 convertible.
///
/// So refusals travel WITH the result instead of replacing it. Nothing is
/// silently dropped -- [`Import::refused`] names every body that did not make it
/// -- and nothing usable is thrown away because something else was not.
///
/// # What the decoder dropped before conversion saw it
///
/// A body can only be refused here if the decoder handed it over. What the
/// DECODER could not carry -- an entity it skipped, a placement it could not
/// resolve, a value it normalised -- is in [`Import::losses`], verbatim from its
/// report. An import with no refusals and no losses is the only one that claims
/// nothing was dropped on the way.
///
/// # What the bodies are, and are not
///
/// The bodies are [`CompressedShell`]s, not validated `Shell`s. A converted
/// IGES sheet can carry a closed edge -- its start and end vertex coincide, or
/// two coincident vertices bound one loop -- which `Shell::extract` refuses by
/// design (`Error::SameVertex`). That is a property of the source, not a
/// conversion failure: the healing passes (`monstertruck_healing`) split those
/// edges before a boolean kernel sees the shell. A caller that needs a `Shell`
/// runs healing first.
///
/// The converter also checks that every open edge's converted curve runs from
/// its start vertex to its end vertex, and refuses the body by name when it
/// does not, rather than handing out a shell whose wires fail much later.
///
/// # Assemblies
///
/// [`Import::bodies`] holds one entry per PLACED instance, not per definition: a
/// part an assembly uses three times appears three times, each at its own
/// placement. [`Import::total`] counts instances accordingly, including the
/// instances of a body that could not convert.
#[derive(Debug)]
pub struct Import {
    /// Bodies that converted, one per placed instance.
    pub bodies: Vec<ImportedBody>,
    /// Bodies that did not, each named with its reason.
    pub refused: Vec<Refusal>,
    /// What the decoder reported losing on a decode that succeeded, in its own
    /// words. Empty when the document did not come from a decode this crate
    /// ran, as with [`import`] on a caller's own `CadIr`.
    pub losses: Vec<DecoderLoss>,
}

/// One loss the decoder reported on a decode that SUCCEEDED.
///
/// cadmpeg's own `LossNote`, copied out of its report so that a caller does not
/// need a direct cadmpeg dependency to read it, and so that the report's
/// lifetime does not bind the import's.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecoderLoss {
    /// The decoder's stable, machine-readable loss kind, as it spells it.
    pub code: String,
    /// How serious the decoder judged it.
    pub severity: LossSeverity,
    /// The decoder's human-readable explanation, usually naming the entity.
    pub message: String,
    /// Where in the source the loss occurred, when the decoder could attribute
    /// it: a byte offset and, where the format carries one, a record tag.
    pub provenance: Option<SourceProvenance>,
}

/// How serious a [`DecoderLoss`] is, in the decoder's own four grades.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum LossSeverity {
    /// Informational; no action needed.
    Info,
    /// A non-fatal approximation or normalisation.
    Warning,
    /// A correctness problem in what was decoded.
    Error,
    /// The decode could not complete faithfully.
    Blocking,
}

impl LossSeverity {
    fn of(severity: cadmpeg_ir::report::Severity) -> Self {
        use cadmpeg_ir::report::Severity;
        match severity {
            Severity::Info => Self::Info,
            Severity::Warning => Self::Warning,
            Severity::Error => Self::Error,
            Severity::Blocking => Self::Blocking,
        }
    }
}

impl DecoderLoss {
    fn of(note: &cadmpeg_ir::report::loss::LossNote) -> Self {
        Self {
            code: note.code.to_string(),
            severity: LossSeverity::of(note.severity),
            message: note.message.clone(),
            provenance: note.provenance.clone(),
        }
    }
}

impl Import {
    /// Whether nothing at all converted.
    ///
    /// Distinct from a file that held no geometry, which is
    /// [`Error::NoGeometry`]: this means bodies were present and none of them
    /// could be represented.
    pub fn is_empty(&self) -> bool { self.bodies.is_empty() }

    /// How many bodies the file held, converted or not, counted per instance.
    pub fn total(&self) -> usize {
        self.bodies.len()
            + self
                .refused
                .iter()
                .map(|refusal| refusal.instances)
                .sum::<usize>()
    }

    /// The decoder losses at `severity` or worse.
    pub fn losses_at_least(&self, severity: LossSeverity) -> impl Iterator<Item = &DecoderLoss> {
        self.losses
            .iter()
            .filter(move |loss| loss.severity >= severity)
    }

    /// A one-line summary fit for a log line or a warning.
    pub fn summary(&self) -> String {
        let losses = match self.losses.len() {
            0 => String::new(),
            count => format!("; the decoder reported {count} losses"),
        };
        if self.refused.is_empty() {
            return format!("{} bodies, all converted{losses}", self.bodies.len());
        }
        let mut kinds: std::collections::BTreeMap<&str, usize> = Default::default();
        for refusal in &self.refused {
            *kinds.entry(refusal.kind).or_insert(0) += refusal.instances;
        }
        let detail = kinds
            .iter()
            .map(|(kind, count)| format!("{count} {kind}"))
            .collect::<Vec<_>>()
            .join(", ");
        format!(
            "{} of {} bodies converted; refused: {detail}{losses}",
            self.bodies.len(),
            self.total()
        )
    }
}

/// Decode with `codec`, then hand the BORROWED document and the decoder's
/// losses to `convert`.
///
/// `DecodeResult` lends its document, so the obvious `-> Result<CadIr>` shape
/// forces a `.clone()` -- a second full copy of the decoded model, live for the
/// whole conversion. On a 28 MB IGES assembly that is 666 bodies, 33,202 edges,
/// 32,524 curves and 2,014 surfaces copied for no benefit. So the result stays
/// alive here and the conversion reads through it; taking a closure is what
/// makes that possible without handing a borrow back out of the function.
///
/// Decodes at [`DecodeDepth::Geometry`]: B-rep, product structure and
/// placements, without the design-intent arenas (features, sketches,
/// parameters) that nothing here reads. Shared by every format, so a new format
/// is a codec and a format name.
pub(crate) fn decoded<C: Codec, T>(
    codec: &C,
    format: &'static str,
    reader: &mut dyn ReadSeek,
    policy: DecodePolicy,
    convert: impl FnOnce(&cadmpeg_ir::CadIr, Vec<DecoderLoss>) -> Result<T>,
) -> Result<T> {
    let options = DecodeOptions {
        depth: DecodeDepth::Geometry,
        policy,
    };
    let decoded = codec
        .decode(reader, &options)
        .map_err(|error| Error::Decode {
            format,
            message: error.to_string(),
        })?;
    let losses = decoded
        .report()
        .losses
        .iter()
        .map(DecoderLoss::of)
        .collect();
    convert(decoded.ir(), losses)
}

/// [`import`], with the decoder's losses attached.
pub(crate) fn import_reported(
    ir: &cadmpeg_ir::CadIr,
    format: &'static str,
    losses: Vec<DecoderLoss>,
) -> Result<Import> {
    let mut recovered = import(ir, format)?;
    recovered.losses = losses;
    Ok(recovered)
}

/// Convert every body in a decoded document, keeping what converts.
///
/// Returns [`Error::NoGeometry`] when the document decoded but held no body,
/// which is a fact about the file, not a failure to read it. A body that cannot
/// be represented lands in [`Import::refused`] rather than failing the call --
/// see [`Import`] for why.
///
/// Document-level problems still fail: a dangling reference means the decoded
/// graph is not closed, and converting the closed part of it would hand back a
/// body with a hole in it.
pub fn import(ir: &cadmpeg_ir::CadIr, format: &'static str) -> Result<Import> {
    if ir.model.bodies.is_empty() {
        return Err(Error::NoGeometry { format });
    }
    convert::import(ir, format)
}

/// Convert every body, or fail if any single one cannot be represented.
///
/// Prefer [`import`] unless the caller genuinely needs all-or-nothing. This is
/// the strict reading: useful when a partial model would be worse than none, and
/// wrong for IGES, where construction wires sit beside the geometry as a matter
/// of course.
///
/// Returns [`Error::NoGeometry`] when the document decoded but held no body,
/// which is a fact about the file, not a failure to read it.
///
/// # Every body converts, or the call fails
///
/// A returned `Vec` cannot report that one of its entries was left out, so a body
/// this crate cannot represent fails the whole call and names itself. See the
/// note on `convert` for why silently returning fewer bodies than the file holds
/// is the outcome worth failing to avoid.
pub fn to_bodies(ir: &cadmpeg_ir::CadIr, format: &'static str) -> Result<Vec<ImportedBody>> {
    if ir.model.bodies.is_empty() {
        return Err(Error::NoGeometry { format });
    }
    convert::convert(ir, format)
}

/// Read solids only, discarding sheet bodies.
///
/// # Deprecated
///
/// Prefer [`to_bodies`]. This signature cannot express what IGES actually
/// returns -- mostly sheets -- so it would silently hand back an empty list for
/// files that are full of geometry.
#[deprecated(
    since = "0.4.0",
    note = "cannot represent sheet bodies, which is most IGES content; use `to_bodies`"
)]
pub fn to_solids(ir: &cadmpeg_ir::CadIr, format: &'static str) -> Result<Vec<ImportedSolid>> {
    Ok(to_bodies(ir, format)?
        .into_iter()
        .filter_map(|body| match body {
            ImportedBody::Solid(solid) => Some(solid),
            ImportedBody::Sheet(_) => None,
        })
        .collect())
}
