//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

#[test]
fn arc_length_reparameterization_straight_line() {
    // A straight line should have uniform arc-length parameterization.
    let line = BsplineCurve::new(
        KnotVector::bezier_knot(1),
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 4.0, 0.0)],
    );
    let reparam = reparameterize_arc_length(&line, 20).unwrap();
    // At u=0.5, we should be at the midpoint.
    let mid = reparam.subs(0.5);
    assert!((mid.x - 1.5).abs() < 0.1, "expected x~1.5, got {}", mid.x,);
    assert!((mid.y - 2.0).abs() < 0.1, "expected y~2.0, got {}", mid.y,);
}

#[test]
fn fair_curve_reduces_noise() {
    // Build a noisy curve with 9 control points.
    let knot = KnotVector::uniform_knot(3, 6);
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.5, 0.0),
        Point3::new(2.0, -0.3, 0.0),
        Point3::new(3.0, 0.4, 0.0),
        Point3::new(4.0, -0.2, 0.0),
        Point3::new(5.0, 0.3, 0.0),
        Point3::new(6.0, -0.1, 0.0),
        Point3::new(7.0, 0.2, 0.0),
        Point3::new(8.0, 0.0, 0.0),
    ];
    let noisy = BsplineCurve::new(knot, points);
    let smooth = fair_curve(&noisy, 3, 5, 30).unwrap();

    // The smoothed curve has fewer control points.
    assert_eq!(smooth.control_points().len(), 5);
    // Endpoints should still be approximately preserved.
    let start = smooth.subs(0.0);
    let end = smooth.subs(1.0);
    assert!((start.x - 0.0).abs() < 0.2, "start.x = {}", start.x,);
    assert!((end.x - 8.0).abs() < 0.2, "end.x = {}", end.x,);
}
