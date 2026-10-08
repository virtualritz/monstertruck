//! Dividing an intersection curve follows its leader when the leader lies on it.

use monstertruck_geometry::prelude::*;
use std::f64::consts::PI;

fn spheres() -> (Sphere, Sphere) {
    (
        Sphere::new(Point3::new(0.0, 0.0, 1.0), f64::sqrt(2.0)),
        Sphere::new(Point3::new(0.0, 0.0, -1.0), f64::sqrt(2.0)),
    )
}

#[test]
fn division_keeps_an_exact_polyline_leader() {
    let (sphere0, sphere1) = spheres();
    let corners: Vec<Point3> = (0..=32)
        .map(|i| {
            let t = PI * i as f64 / 32.0;
            Point3::new(t.cos(), t.sin(), 0.0)
        })
        .collect();
    let leader = BsplineCurve::new(KnotVector::uniform_knot(1, 32), corners.clone());
    let curve = IntersectionCurve::new(sphere0, sphere1, leader);
    let (params, points) = curve.parameter_division((0.0, 1.0), 1.0e-3);
    assert_eq!(params.len(), points.len());
    assert!(points.len() <= corners.len());
    for point in &points {
        assert!((point.to_vec().magnitude() - 1.0).abs() < 1.0e-9 && point.z.abs() < 1.0e-9);
    }
    assert!(
        points
            .iter()
            .all(|p| corners.iter().any(|c| c.distance(*p) < 1.0e-12))
    );
}

#[test]
fn division_leaves_a_rough_leader() {
    let (sphere0, sphere1) = spheres();
    let leader = BsplineCurve::new(
        KnotVector::bezier_knot(2),
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
        ],
    );
    let curve = IntersectionCurve::new(sphere0, sphere1, leader);
    let (_, points) = curve.parameter_division((0.0, 1.0), 1.0e-3);
    assert!(points.len() > 3);
    for point in &points {
        assert!(
            (point.to_vec().magnitude() - 1.0).abs() < 1.0e-6,
            "{point:?}"
        );
    }
}
