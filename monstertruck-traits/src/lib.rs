//! Geometric trait definitions: `ParametricCurve`, `ParametricSurface`, `BoundedCurve`, `Invertible`, `Transformed`, and more.
//!
//! # Examples
//!
//! ```
//! use monstertruck_traits::*;
//! use monstertruck_core::cgmath64::*;
//!
//! // `range_tuple` comes from `BoundedCurve`, so the bound needs both traits.
//! fn arc_length<C: ParametricCurve<Point = Point3> + BoundedCurve>(
//!     curve: &C,
//!     steps: usize,
//! ) -> f64 {
//!     let (t0, t1) = curve.range_tuple();
//!     let dt = (t1 - t0) / steps as f64;
//!     (0..steps)
//!         .map(|i| {
//!             let a = curve.evaluate(t0 + dt * i as f64);
//!             let b = curve.evaluate(t0 + dt * (i + 1) as f64);
//!             (b - a).magnitude()
//!         })
//!         .sum()
//! }
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

#[macro_export]
#[doc(hidden)]
macro_rules! nonpositive_tolerance {
    ($tol: expr, $minimum: expr) => {
        assert!(
            $tol >= $minimum,
            "tolerance must be no less than {:e}",
            $minimum
        );
    };
    ($tol: expr) => {
        nonpositive_tolerance!($tol, TOLERANCE)
    };
}

/// Abstract traits: `Curve` and `Surface`.
pub mod traits;
pub use traits::*;
/// Algorithms for curves and surfaces.
pub mod algo;
/// Scalar-generic v2 trait family.
pub mod v2;
#[cfg(feature = "derive")]
pub use monstertruck_derive::{
    BoundedCurve, BoundedSurface, Cut, Invertible, ParameterDivision1D, ParameterDivision2D,
    ParametricCurve, ParametricSurface, ParametricSurface3D, SearchNearestParameterD1,
    SearchNearestParameterD2, SearchParameterD1, SearchParameterD2, SelfSameGeometry,
    TransformedM3, TransformedM4,
};
#[cfg(feature = "polynomial")]
/// Implementation sample using polynomials as an example
pub mod polynomial;
