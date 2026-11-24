use monstertruck_geometry::prelude::*;

fn below(value: f64) -> f64 { value - value.abs().max(1.0) * f64::EPSILON * 4.0 }

fn highest_multiplicity(knots: &KnotVector) -> usize {
    (0..knots.len())
        .map(|i| knots.multiplicity(i))
        .max()
        .unwrap_or(0)
}

fn uniform_curve() -> BsplineCurve<Point3> {
    let knots = KnotVector::from(vec![0.0, 0.0, 0.0, 0.0, 0.3, 0.6, 0.9, 1.2, 1.2, 1.2, 1.2]);
    let points = (0..7)
        .map(|i| {
            let t = i as f64;
            Point3::new(t.cos(), t.sin(), t * 0.5)
        })
        .collect();
    BsplineCurve::new(knots, points)
}

#[test]
fn curve_cut_just_below_a_knot_keeps_valid_multiplicity() {
    let curve = uniform_curve();
    let mut head = curve.clone();
    let tail = head.cut(below(0.6));
    assert!(highest_multiplicity(head.knot_vector()) <= 4);
    assert!(highest_multiplicity(tail.knot_vector()) <= 4);
    (0..=20).for_each(|i| {
        let t = 0.6 * i as f64 / 20.0;
        assert_near!(head.subs(t), curve.subs(t));
        let t = 0.6 + 0.6 * i as f64 / 20.0;
        assert_near!(tail.subs(t), curve.subs(t));
    });
}

#[test]
fn surface_cut_just_below_a_knot_keeps_valid_multiplicity() {
    let curve = uniform_curve();
    let rows = curve
        .control_points()
        .iter()
        .map(|p| vec![*p, *p + Vector3::new(0.0, 0.0, 1.0)])
        .collect();
    let surface = BsplineSurface::new(
        (curve.knot_vector().clone(), KnotVector::bezier_knot(1)),
        rows,
    );
    let mut head = surface.clone();
    let tail = head.cut_u(below(0.6));
    assert!(highest_multiplicity(head.knot_vector_u()) <= 4);
    assert!(highest_multiplicity(tail.knot_vector_u()) <= 4);
    (0..=20).for_each(|i| {
        let u = 0.6 * i as f64 / 20.0;
        assert_near!(head.subs(u, 0.5), surface.subs(u, 0.5));
        let u = 0.6 + 0.6 * i as f64 / 20.0;
        assert_near!(tail.subs(u, 0.5), surface.subs(u, 0.5));
    });
}
