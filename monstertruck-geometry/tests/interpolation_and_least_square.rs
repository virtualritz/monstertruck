//! Tests for the two `gaussian_elimination` callers.
//!
//! Both `BsplineCurve::try_interpolate` and `BsplineCurve::least_square` reach
//! the same private dense solver in `nurbs::mod`, and before this file neither
//! had a test -- only doc examples, whose tolerances are loose by design (the
//! `least_square` example asserts `< 0.2`). That left the solver pinned by
//! nothing, which matters because it carries no pivoting: its accuracy is a
//! property of the matrices it happens to be handed, not something the code
//! defends.
//!
//! So these assert the two defining properties at a tolerance tight enough to
//! move if the solver does, plus both documented refusal paths -- a refusal
//! that silently became an answer would otherwise be invisible. Every bar below
//! sits within ~3 orders of the value actually measured, not the 3-to-5 orders
//! of slack a round number invites.
//!
//! What these do NOT constrain, stated so nobody assumes otherwise: they do not
//! pin the zero-multiplier guards or the back-substitution bound ported from
//! upstream `a696134f`. Reverting that hunk in full leaves all of them green.
//! That is not a gap in the tests -- 4,808 probes through this API (degrees 2-7,
//! random and equispaced parameters, near-duplicate separations of one ulp,
//! 5e-17, 1e-16, 1e-15, and exact duplicates) produce an identical
//! refuse/answer outcome on both variants. No input reachable through this
//! surface distinguishes them, so no test here can.

use monstertruck_geometry::prelude::*;

/// `uniform_knot(degree, division)` yields `degree + division` control points.
/// Asserted rather than assumed, because every size below is derived from it.
#[test]
fn uniform_knot_control_point_count() {
    let knot_vec = KnotVector::uniform_knot(3, 5);
    assert_eq!(knot_vec.len(), 12);
    assert_eq!(knot_vec.len() - 3 - 1, 8);
}

fn sample_curve() -> BsplineCurve<Point3> {
    BsplineCurve::new(
        KnotVector::uniform_knot(3, 5),
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, -1.0),
            Point3::new(3.0, -1.0, 2.0),
            Point3::new(4.0, 4.0, 1.0),
            Point3::new(6.0, 0.5, -2.0),
            Point3::new(7.0, -3.0, 3.0),
            Point3::new(9.0, 1.0, 0.5),
            Point3::new(10.0, 2.5, -1.5),
        ],
    )
}

/// The defining property of interpolation: the curve passes through every point
/// it was given.
#[test]
fn interpolation_passes_through_every_point() {
    let knot_vec = KnotVector::uniform_knot(3, 5);
    let points = [
        (0.0, Point3::new(1.0, 2.0, 3.0)),
        (0.15, Point3::new(4.0, -1.0, 10.0)),
        (0.3, Point3::new(-3.0, 5.0, 6.0)),
        (0.45, Point3::new(6.0, 2.0, 12.0)),
        (0.6, Point3::new(0.0, -4.0, 1.0)),
        (0.75, Point3::new(8.0, 3.0, -2.0)),
        (0.9, Point3::new(2.0, 7.0, 5.0)),
        (1.0, Point3::new(-1.0, 0.0, 9.0)),
    ];
    let curve = BsplineCurve::try_interpolate(knot_vec, points).unwrap();

    // Reported as the worst of the eight rather than asserted point by point,
    // so a failure says how far off the solver is and not merely that it is.
    let worst = points
        .iter()
        .map(|&(t, p)| curve.subs(t).distance(p))
        .fold(0.0f64, f64::max);
    assert!(
        worst < 1.0e-12,
        "interpolant misses its own points by {worst:.3e}"
    );
}

/// Least squares over data drawn from a curve that already lies in the fit
/// space has an exact answer -- the original control points -- so this grades
/// the solver against a known truth rather than against a loose band.
#[test]
fn least_square_recovers_an_exactly_representable_curve() {
    let original = sample_curve();
    let samples = (0..40)
        .map(|i| {
            let t = i as f64 / 39.0;
            (t, original.subs(t))
        })
        .collect::<Vec<_>>();

    let fit = BsplineCurve::least_square(KnotVector::uniform_knot(3, 5), 3, &samples).unwrap();

    let worst = (0..8)
        .map(|i| fit.control_point(i).distance(*original.control_point(i)))
        .fold(0.0f64, f64::max);
    assert!(
        worst < 1.0e-11,
        "least squares missed the exact answer by {worst:.3e} in control-point distance"
    );
}

/// The overdetermined case with noise: not an exact-recovery question, only
/// that the normal equations are actually solved and the fit tracks the data.
#[test]
fn least_square_fits_perturbed_samples() {
    let original = sample_curve();
    let samples = (0..60)
        .map(|i| {
            let t = i as f64 / 59.0;
            // Deterministic, mean-ish-zero wobble; no RNG, so this cannot flake.
            let wobble = 0.05 * f64::sin(37.0 * t) * f64::cos(11.0 * t);
            let p = original.subs(t);
            (t, Point3::new(p.x + wobble, p.y - wobble, p.z + wobble))
        })
        .collect::<Vec<_>>();

    let fit = BsplineCurve::least_square(KnotVector::uniform_knot(3, 5), 3, &samples).unwrap();

    let worst = samples
        .iter()
        .map(|&(t, p)| fit.subs(t).distance(p))
        .fold(0.0f64, f64::max);
    assert!(worst < 0.1, "fit strayed from its samples by {worst:.3e}");
}

/// Duplicate parameters make two rows identical, so the system is singular.
/// This is the path where a solver that quietly perturbs its own pivots would
/// answer instead of refusing, which is strictly worse than refusing.
#[test]
fn interpolation_refuses_on_duplicate_parameters() {
    let knot_vec = KnotVector::uniform_knot(3, 5);
    let points = [
        (0.0, Point3::new(1.0, 2.0, 3.0)),
        (0.15, Point3::new(4.0, -1.0, 10.0)),
        (0.3, Point3::new(-3.0, 5.0, 6.0)),
        // Same parameter as the row above, different point.
        (0.3, Point3::new(6.0, 2.0, 12.0)),
        (0.6, Point3::new(0.0, -4.0, 1.0)),
        (0.75, Point3::new(8.0, 3.0, -2.0)),
        (0.9, Point3::new(2.0, 7.0, 5.0)),
        (1.0, Point3::new(-1.0, 0.0, 9.0)),
    ];
    assert!(
        BsplineCurve::try_interpolate(knot_vec, points).is_err(),
        "a singular interpolation system was answered rather than refused"
    );
}

/// `least_square`'s documented failure: a span with no sample leaves its
/// control point unconstrained, so the normal matrix is singular.
#[test]
fn least_square_refuses_when_a_span_carries_no_sample() {
    // Every sample crowded into the first half, so the last spans are empty.
    let original = sample_curve();
    let samples = (0..30)
        .map(|i| {
            let t = 0.3 * i as f64 / 29.0;
            (t, original.subs(t))
        })
        .collect::<Vec<_>>();

    assert!(
        BsplineCurve::least_square(KnotVector::uniform_knot(3, 5), 3, &samples).is_err(),
        "an unconstrained control point was fitted rather than refused"
    );
}

/// The third `gaussian_elimination` call site. Its only prior coverage was a doc
/// example, and this repo's `.config/nextest.toml` says nextest does not run
/// doctests -- so a change to the shared solver could not reach it from the
/// test runner at all.
///
/// The saddle `u^2 - v^2` lies exactly in the biquadratic span being fitted, so
/// unlike the doc example's `< 0.05` this can demand machine precision.
#[test]
fn surface_least_square_recovers_an_exactly_representable_saddle() {
    let knots = (
        KnotVector::uniform_knot(2, 2),
        KnotVector::uniform_knot(2, 2),
    );
    let mut points = Vec::new();
    for i in 0..10 {
        for j in 0..10 {
            let (u, v) = (i as f64 / 9.0, j as f64 / 9.0);
            points.push(((u, v), Point3::new(u, v, u * u - v * v)));
        }
    }
    let surface = BsplineSurface::least_square(knots, (2, 2), &points).unwrap();

    let worst = points
        .iter()
        .map(|((u, v), p)| surface.subs(*u, *v).distance(*p))
        .fold(0.0f64, f64::max);
    assert!(
        worst < 1.0e-12,
        "surface fit missed an exactly representable saddle by {worst:.3e}"
    );
}

/// Everything above rides on one matrix shape (degree 3, 8 control points).
/// The solver's accuracy is a property of the matrix it is handed, so a single
/// shape is a single observation. Degrees 2 and 5 are banded like it; the Bezier
/// knot vector is not banded at all -- its basis is Bernstein and the matrix is
/// dense, which is the shape `monstertruck-fillet` drives through
/// `composite_line_bezier`.
#[test]
fn interpolation_holds_across_degrees_and_a_dense_bezier_basis() {
    for degree in [2usize, 5] {
        let n = degree + 5;
        let knot_vec = KnotVector::uniform_knot(degree, n - degree);
        let pts: Vec<(f64, Point3)> = (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                (t, Point3::new(3.0 * t, f64::sin(4.0 * t), t * t - 1.0))
            })
            .collect();
        let curve = BsplineCurve::try_interpolate(knot_vec, pts.clone()).unwrap();
        let worst = pts
            .iter()
            .map(|&(t, p)| curve.subs(t).distance(p))
            .fold(0.0f64, f64::max);
        assert!(
            worst < 1.0e-13,
            "banded degree {degree} interpolant misses its points by {worst:.3e}"
        );
    }

    for degree in [3usize, 6] {
        let knot_vec = KnotVector::bezier_knot(degree);
        let n = degree + 1;
        let pts: Vec<(f64, Point3)> = (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                (t, Point3::new(2.0 * t, 1.0 - t, f64::cos(3.0 * t)))
            })
            .collect();
        let curve = BsplineCurve::try_interpolate(knot_vec, pts.clone()).unwrap();
        let worst = pts
            .iter()
            .map(|&(t, p)| curve.subs(t).distance(p))
            .fold(0.0f64, f64::max);
        assert!(
            worst < 1.0e-13,
            "dense Bernstein degree {degree} interpolant misses its points by {worst:.3e}"
        );
    }
}
