//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

/// A curve whose evaluation is `NaN` everywhere, as produced by degenerate
/// geometry (e.g. zero-weight rational points).
#[derive(Clone)]
struct NanCurve;

impl ParametricCurve for NanCurve {
    type Point = Point3;
    type Vector = Vector3;
    fn evaluate(&self, _: f64) -> Point3 { Point3::new(f64::NAN, f64::NAN, f64::NAN) }
    fn derivative(&self, _: f64) -> Vector3 { Vector3::new(f64::NAN, f64::NAN, f64::NAN) }
    fn derivative_2(&self, _: f64) -> Vector3 { Vector3::new(f64::NAN, f64::NAN, f64::NAN) }
    fn derivative_n(&self, _: usize, _: f64) -> Vector3 {
        Vector3::new(f64::NAN, f64::NAN, f64::NAN)
    }
}

#[test]
fn parameter_division_terminates_on_nan_evaluation() {
    // The midpoint deviation test `dist2 < tol * tol` is always false for
    // `NaN`, which used to subdivide all the way to the depth cap -- an
    // effectively unbounded amount of work.
    let (params, points) = parameter_division(&NanCurve, (0.0, 1.0), 0.01);
    assert_eq!(params.len(), 2);
    assert_eq!(points.len(), 2);
}

#[test]
fn parameter_division_terminates_on_degenerate_range() {
    // A zero-width range cannot be subdivided; it must return immediately.
    let (params, points) = parameter_division(&NanCurve, (0.0, 0.0), 0.01);
    assert_eq!(params.len(), 2);
    assert_eq!(points.len(), 2);
}
