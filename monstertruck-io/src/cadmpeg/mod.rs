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
//!   report must reach the caller rather than being discarded, which is why
//!   [`Error::Decode`] carries the decoder's own message.
//!
//! [`CompressedShell`]: monstertruck_topology::compress::CompressedShell
//! [`Surface::Plane`]: monstertruck_modeling::Surface

pub mod step;

use crate::{Error, Result};
use monstertruck_modeling::{Curve, Point3, Surface};
use monstertruck_topology::compress::CompressedSolid;

/// A solid as monstertruck's kernel wants it, over the canonical curve and
/// surface enums the boolean pipeline consumes.
pub type ImportedSolid = CompressedSolid<Point3, Curve, Surface>;

/// Convert every body in a decoded document into monstertruck solids.
///
/// Returns [`Error::NoGeometry`] when the document decoded but held no body,
/// which is a fact about the file, not a failure to read it.
pub fn to_solids(ir: &cadmpeg_ir::CadIr, format: &'static str) -> Result<Vec<ImportedSolid>> {
    if ir.model.bodies.is_empty() {
        return Err(Error::NoGeometry { format });
    }
    Err(Error::Unimplemented {
        what: "cadmpeg intermediate representation to monstertruck B-rep",
    })
}
