//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use monstertruck_core::DeterministicContentHash;
use monstertruck_core::cgmath64::*;

use crate::nurbs::*;
use crate::specifieds::*;

#[test]
fn identical_curves_hash_the_same() {
    let knots = KnotVector::bezier_knot(2);
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    let a = BsplineCurve::new(knots.clone(), pts.clone());
    let b = BsplineCurve::new(knots, pts);
    assert_eq!(a.content_hash64(), b.content_hash64());
}

#[test]
fn changed_curve_parameter_changes_hash() {
    let knots = KnotVector::bezier_knot(2);
    let pts_a = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    let pts_b = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    let a = BsplineCurve::new(knots.clone(), pts_a);
    let b = BsplineCurve::new(knots, pts_b);
    assert_ne!(a.content_hash64(), b.content_hash64());
}

#[test]
fn identical_surfaces_hash_the_same() {
    let knots = (KnotVector::bezier_knot(1), KnotVector::bezier_knot(1));
    let pts = vec![
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        vec![Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
    ];
    let a = BsplineSurface::new(knots.clone(), pts.clone());
    let b = BsplineSurface::new(knots, pts);
    assert_eq!(a.content_hash64(), b.content_hash64());
}

#[test]
fn plane_hashing() {
    let a = Plane::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    let b = Plane::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    let c = Plane::new(
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    assert_eq!(a.content_hash64(), b.content_hash64());
    assert_ne!(a.content_hash64(), c.content_hash64());
}

#[test]
fn sphere_hashing() {
    let a = Sphere::new(Point3::new(0.0, 0.0, 0.0), 1.0);
    let b = Sphere::new(Point3::new(0.0, 0.0, 0.0), 1.0);
    let c = Sphere::new(Point3::new(0.0, 0.0, 0.0), 2.0);
    assert_eq!(a.content_hash64(), b.content_hash64());
    assert_ne!(a.content_hash64(), c.content_hash64());
}

#[test]
fn line_hashing() {
    let a = Line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let b = Line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let c = Line(Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0));
    assert_eq!(a.content_hash64(), b.content_hash64());
    assert_ne!(a.content_hash64(), c.content_hash64());
}
