//! A trim that is closed in space but open in parameter space -- a
//! cylinder's circle, running `u` from 0 to 2π -- matches a point at its
//! start with both of its ends. Projecting that point without a hint must
//! still land on the start: a boundary is walked from there, and landing on
//! the far end sends every later projection off the curve and folds the
//! face.

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI, TAU};

use monstertruck_geometry::prelude::*;
use monstertruck_meshing::prelude::*;

/// A rational quadratic cylinder of radius 1 and height 1 around `z`, `u`
/// over `[0, 2π]` in four quarter spans, `v` over `[0, 1]`.
fn cylinder() -> NurbsSurface<Vector4> {
    let ring = [
        (1.0, 0.0, 1.0),
        (1.0, 1.0, FRAC_1_SQRT_2),
        (0.0, 1.0, 1.0),
        (-1.0, 1.0, FRAC_1_SQRT_2),
        (-1.0, 0.0, 1.0),
        (-1.0, -1.0, FRAC_1_SQRT_2),
        (0.0, -1.0, 1.0),
        (1.0, -1.0, FRAC_1_SQRT_2),
        (1.0, 0.0, 1.0),
    ];
    let control_points = ring
        .iter()
        .map(|&(x, y, w)| {
            [0.0, 1.0]
                .iter()
                .map(|&z| Vector4::new(x * w, y * w, z * w, w))
                .collect()
        })
        .collect();
    let u_knots = KnotVector::from(vec![
        0.0,
        0.0,
        0.0,
        FRAC_PI_2,
        FRAC_PI_2,
        PI,
        PI,
        3.0 * FRAC_PI_2,
        3.0 * FRAC_PI_2,
        TAU,
        TAU,
        TAU,
    ]);
    let v_knots = KnotVector::bezier_knot(1);
    NurbsSurface::new(BsplineSurface::new((u_knots, v_knots), control_points))
}

/// Projects both evaluations of `trim`'s meeting point -- a neighbouring
/// face's sample of it can be either, a rounding apart -- and requires each
/// to land at the trim's start.
fn assert_projects_to_start(trim: &ParameterCurve<BsplineCurve<Point2>, NurbsSurface<Vector4>>) {
    let (start, end) = trim.range_tuple();
    assert!(
        trim.subs(start).near(&trim.subs(end)),
        "the fixture must be closed in space"
    );
    [start, end].into_iter().for_each(|at| {
        let (parameter, uv) = trim
            .project_boundary_point(trim.subs(at), None)
            .expect("the start of the trim projects onto it");
        assert!(
            (parameter - start).abs() < (parameter - end).abs(),
            "the point at {at} projected to {parameter} (uv {uv:?}), \
             the far end of [{start}, {end}]"
        );
    });
}

/// A circle's trim, from `(from, v)` to `(to, v)`.
fn circle_trim(
    from: f64,
    to: f64,
    v: f64,
) -> ParameterCurve<BsplineCurve<Point2>, NurbsSurface<Vector4>> {
    let line = BsplineCurve::new(
        KnotVector::bezier_knot(1),
        vec![Point2::new(from, v), Point2::new(to, v)],
    );
    ParameterCurve::new(line, cylinder())
}

#[test]
fn a_trim_running_up_the_seam_projects_its_start_to_its_start() {
    assert_projects_to_start(&circle_trim(0.0, TAU, 0.0));
}

/// The case a real part hit: the top circle of a hole, taken in its trim
/// loop's direction, runs `u` down from 2π.
#[test]
fn a_trim_running_down_the_seam_projects_its_start_to_its_start() {
    assert_projects_to_start(&circle_trim(TAU, 0.0, 1.0));
}
