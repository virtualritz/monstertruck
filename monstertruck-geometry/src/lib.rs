//! Geometric primitives: knot vectors, B-splines, NURBS, and T-splines.
//!
//! # Examples
//!
//! ```
//! use monstertruck_geometry::prelude::*;
//!
//! // A quadratic Bezier curve from (0,0,0) through (1,1,0) to (2,0,0)
//! let knot_vec = KnotVector::bezier_knot(2);
//! let ctrl_pts = vec![
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(1.0, 1.0, 0.0),
//!     Point3::new(2.0, 0.0, 0.0),
//! ];
//! let curve = BsplineCurve::new(knot_vec, ctrl_pts);
//!
//! let mid = curve.evaluate(0.5);    // Point3 at parameter t=0.5
//! let tan = curve.derivative(0.5);  // tangent vector
//! let (t0, t1) = curve.range_tuple(); // (0.0, 1.0)
//! ```

#![cfg_attr(not(debug_assertions), deny(warnings))]
#![deny(clippy::all, rust_2018_idioms)]
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

use monstertruck_core::bounding_box::Bounded;
use serde::{Deserialize, Serialize};
use std::{fmt::Debug, ops::Bound};

const INCLUDE_CURVE_TRIALS: usize = 100;
const PRESEARCH_DIVISION: usize = 50;

/// re-export `monstertruck_core`
pub mod base {
    pub use monstertruck_core::{
        assert_near, assert_near2, bounding_box::BoundingBox, cgmath64::*, hash, hash::HashGen,
        prop_assert_near, prop_assert_near2, tolerance::*,
    };
    pub use monstertruck_traits::*;
}
/// NURBS and B-spline curves, surfaces, and knot vectors.
pub mod nurbs;

/// Error types for geometry operations.
pub mod errors;

/// Concrete geometric primitives: [`Plane`](crate::specifieds::Plane), [`Sphere`](crate::specifieds::Sphere), [`Line`](crate::specifieds::Line), etc.
pub mod specifieds;

/// Composite geometry: revolved curves, intersection curves, processor wrappers.
pub mod decorators;

/// T-Spline and T-NURCC surface types.
pub mod t_spline;

mod analytic_surface;
/// [`DeterministicContentHash`](monstertruck_core::DeterministicContentHash) impls for geometry types.
mod content_hash_impls;

/// Trait for extracting an exact polynomial B-spline surface representation.
mod bspline_conversion;
mod parameter_boundary;

/// re-export all modules.
pub mod prelude {
    use crate::*;
    pub use analytic_surface::{
        AnalyticSurfaceKind, HomogeneousExtrusionSurface, SphericalRevolutionSurface,
        SurfaceParameterAxis, TryIntoAnalyticSurfaceKind,
    };
    pub use base::*;
    pub use bspline_conversion::{
        HomogeneousSurfaceConversion, SupportsExactPatchDomains, SurfaceFrameAxes,
        SurfaceParameterRectangle, TryIntoBsplineSurface, TryIntoHomogeneousBsplineCurve,
        TryIntoHomogeneousBsplineSurface,
    };
    pub use decorators::*;
    pub use errors::*;
    pub use nurbs::*;
    pub use parameter_boundary::BoundaryCurve2D;
    pub use specifieds::*;
    pub use t_spline::*;
}
