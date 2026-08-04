//! Boolean operations for solids. Fillets live in `monstertruck-fillet` and shape
//! healing in `monstertruck-healing`; both are post-CSG and kernel-independent.
//!
//! # Examples
//!
//! ```
//! use monstertruck_modeling::*;
//! use monstertruck_solid::or;
//!
//! // Two unit cubes overlapping in a 0.5 cube at one corner.
//! let v = builder::vertex(Point3::origin());
//! let cube_a: Solid = builder::extrude(
//!     &builder::extrude(&builder::extrude(&v, Vector3::unit_x()), Vector3::unit_y()),
//!     Vector3::unit_z(),
//! );
//! let cube_b = builder::translated(&cube_a, Vector3::new(0.5, 0.5, 0.5));
//!
//! // Boolean entry points are Result-shaped: `Ok(Solid)`, or a typed
//! // `ShapeOpsError` -- never a silent `None`.
//! // `monstertruck_modeling::*` brings its own 1-generic `Result` alias into
//! // scope, so leave the binding unannotated rather than shadowing it.
//! let union = or(&cube_a, &cube_b, 0.05);
//! assert!(union.is_ok());
//! ```

#![cfg_attr(not(debug_assertions), deny(warnings))]
#![deny(clippy::all, rust_2018_idioms)]
// Under `--no-default-features` no boolean backend is compiled: the classic
// marcher (`marching-ssi`, the published default) is off and the boolean entry
// points return `ShapeOpsError::NoBackend`. Their generic helpers then go
// unused, so allow dead code in exactly that no-backend configuration.
#![cfg_attr(not(feature = "marching-ssi"), allow(dead_code))]
#![warn(
    missing_docs,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications
)]

mod transversal;
pub use transversal::{
    PlaneCut, ShapeOpsCurve, ShapeOpsError, ShapeOpsSurface, ShellOrientationHints,
    SnapCurveEndpoints, and, and_with_orientation_hints, clip_half_space_z, difference, or,
    plane_cut, symmetric_difference,
};
mod alternative;
