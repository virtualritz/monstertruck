//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

#[test]
fn hermite_endpoints_and_tangents() {
    let curve = BsplineCurve::from(HermiteSegment {
        p0: Point3::new(0.0, 0.0, 0.0),
        t0: Vector3::new(3.0, 3.0, 0.0),
        p1: Point3::new(3.0, 0.0, 0.0),
        t1: Vector3::new(3.0, -3.0, 0.0),
    });
    assert_near2!(curve.subs(0.0), Point3::new(0.0, 0.0, 0.0));
    assert_near2!(curve.subs(1.0), Point3::new(3.0, 0.0, 0.0));
    assert_near2!(curve.der(0.0), Vector3::new(3.0, 3.0, 0.0));
    assert_near2!(curve.der(1.0), Vector3::new(3.0, -3.0, 0.0));
}

#[test]
fn catmull_rom_interpolates_interior() {
    let curve = BsplineCurve::from(CatmullRomSpline(vec![
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ]));
    assert_near2!(curve.subs(0.0), Point3::new(0.0, 0.0, 0.0));
    assert_near2!(curve.subs(1.0), Point3::new(2.0, 0.0, 0.0));
}

#[test]
fn power_basis_linear() {
    let curve = BsplineCurve::from(PowerBasisCurve(vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, 4.0, 0.0),
    ]));
    assert_near2!(curve.subs(0.0), Point3::new(0.0, 0.0, 0.0));
    assert_near2!(curve.subs(1.0), Point3::new(3.0, 4.0, 0.0));
    assert_near2!(curve.subs(0.5), Point3::new(1.5, 2.0, 0.0));
}

#[test]
fn power_basis_quadratic() {
    // p(t) = (t², 0, 0).
    let curve = BsplineCurve::from(PowerBasisCurve(vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    ]));
    assert_near2!(curve.subs(0.0), Point3::new(0.0, 0.0, 0.0));
    assert_near2!(curve.subs(1.0), Point3::new(1.0, 0.0, 0.0));
    assert_near2!(curve.subs(0.5), Point3::new(0.25, 0.0, 0.0));
}

#[test]
fn concat_two_cubic_segments() {
    let seg1 = BsplineCurve::new(
        KnotVector::bezier_knot(3),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ],
    );
    let seg2 = BsplineCurve::new(
        KnotVector::bezier_knot(3),
        vec![
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(4.0, -2.0, 0.0),
            Point3::new(5.0, -2.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
        ],
    );
    let combined = BsplineCurve::from(PiecewiseBezier(vec![seg1, seg2]));
    assert_near2!(combined.subs(0.0), Point3::new(0.0, 0.0, 0.0));
    assert_near2!(combined.subs(0.5), Point3::new(3.0, 0.0, 0.0));
    assert_near2!(combined.subs(1.0), Point3::new(6.0, 0.0, 0.0));
}
