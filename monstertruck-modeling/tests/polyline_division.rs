//! Polyline curves divide into as few points as the tolerance allows.

use monstertruck_modeling::*;

#[test]
fn a_dense_polyline_divides_to_its_corners() {
    let corners: Vec<Point3> = (0..=100)
        .map(|i| {
            let x = f64::from(i) / 100.0;
            Point3::new(x, if x < 0.5 { 0.0 } else { x - 0.5 }, 0.0)
        })
        .collect();
    let curve = Curve::BsplineCurve(BsplineCurve::new(KnotVector::uniform_knot(1, 100), corners));
    let (params, points) = curve.parameter_division((0.0, 1.0), 1.0e-3);
    assert_eq!(params.len(), points.len());
    assert_eq!(
        points,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.0, 0.5, 0.0),
        ]
    );
}

#[test]
fn a_dense_leader_divides_within_tolerance() {
    let corners: Vec<Point3> = (0..=400)
        .map(|i| {
            let t = std::f64::consts::PI * f64::from(i) / 400.0;
            Point3::new(t.cos(), t.sin(), 0.0)
        })
        .collect();
    let leader = BsplineCurve::new(KnotVector::uniform_knot(1, 400), corners.clone());
    let plane = Surface::Plane(Plane::new(
        Point3::origin(),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ));
    let curve = Curve::from(IntersectionCurve::new(plane.clone(), plane, leader));
    let tol = 0.05;
    let (_, points) = curve.parameter_division((0.0, 1.0), tol);
    assert!(points.len() < 20, "{} points", points.len());
    let chord_gap = |p: Point3| {
        points
            .windows(2)
            .map(|w| {
                let (a, b) = (w[0], w[1]);
                let along = ((p - a).dot(b - a) / (b - a).magnitude2()).clamp(0.0, 1.0);
                (a + (b - a) * along).distance(p)
            })
            .fold(f64::INFINITY, f64::min)
    };
    assert!(corners.iter().all(|&c| chord_gap(c) < tol));
}
