//! Unit tests for the parent module (`scalar_generic_tests`).
//!
//! Split out of the module file so the source stays readable. The module
//! name is unchanged, so every test keeps its path and its identity.

use super::*;
use monstertruck_traits::v2;

type Point2F32 = cgmath::Point2<f32>;
type Point3F32 = cgmath::Point3<f32>;
type Vector3F32 = cgmath::Vector3<f32>;

// -- Compile-time trait satisfaction ------------------------------------

const fn _assert_v2_curve<C: v2::ParametricCurve>() {}
const fn _assert_v2_bounded<C: v2::BoundedCurve>() {}
const fn _assert_v2_cut<C: v2::Cut>() {}
const fn _assert_v2_search<C: v2::SearchParameter<v2::D1<f32>>>() {}
const fn _assert_v2_nearest<C: v2::SearchNearestParameter<v2::D1<f32>>>() {}

#[allow(dead_code)]
const _: () = {
    _assert_v2_curve::<Line<Point2F32>>();
    _assert_v2_curve::<Line<Point3F32>>();
    _assert_v2_bounded::<Line<Point2F32>>();
    _assert_v2_bounded::<Line<Point3F32>>();
    _assert_v2_cut::<Line<Point2F32>>();
    _assert_v2_cut::<Line<Point3F32>>();
    _assert_v2_search::<Line<Point2F32>>();
    _assert_v2_search::<Line<Point3F32>>();
    _assert_v2_nearest::<Line<Point2F32>>();
    _assert_v2_nearest::<Line<Point3F32>>();
};

// -- Runtime correctness (f32) -----------------------------------------

#[test]
fn f32_evaluate_and_derivative() {
    let line: Line<Point3F32> = Line(Point3F32::new(1.0, 0.0, 0.0), Point3F32::new(0.0, 1.0, 0.0));

    let mid = v2::ParametricCurve::evaluate(&line, 0.5f32);
    assert!((mid.x - 0.5).abs() < 1e-6);
    assert!((mid.y - 0.5).abs() < 1e-6);
    assert!((mid.z - 0.0).abs() < 1e-6);

    let tangent: Vector3F32 = v2::ParametricCurve::derivative(&line, 0.5f32);
    assert!((tangent.x - (-1.0)).abs() < 1e-6);
    assert!((tangent.y - 1.0).abs() < 1e-6);
    assert!((tangent.z - 0.0).abs() < 1e-6);

    let accel: Vector3F32 = v2::ParametricCurve::derivative_2(&line, 0.5f32);
    assert!((accel.x).abs() < 1e-6);
    assert!((accel.y).abs() < 1e-6);
    assert!((accel.z).abs() < 1e-6);
}

#[test]
fn f32_bounded_curve() {
    let line: Line<Point3F32> = Line(Point3F32::new(0.0, 0.0, 0.0), Point3F32::new(1.0, 1.0, 1.0));
    let (t0, t1) = v2::BoundedCurve::range_tuple(&line);
    assert_eq!(t0, 0.0f32);
    assert_eq!(t1, 1.0f32);
}

#[test]
fn f32_cut() {
    let line: Line<Point3F32> = Line(Point3F32::new(1.0, 0.0, 0.0), Point3F32::new(0.0, 1.0, 0.0));
    let mut left = line;
    let right = v2::Cut::cut(&mut left, 0.4f32);

    // left covers [0, 0.4]
    assert_eq!(left.0, line.0);
    let expected_mid = v2::ParametricCurve::evaluate(&line, 0.4f32);
    assert!((left.1.x - expected_mid.x).abs() < 1e-6);
    assert!((left.1.y - expected_mid.y).abs() < 1e-6);

    // right covers [0.4, 1]
    assert!((right.0.x - expected_mid.x).abs() < 1e-6);
    assert_eq!(right.1, line.1);
}

#[test]
fn f32_presearch() {
    let line: Line<Point3F32> = Line(Point3F32::new(0.0, 0.0, 0.0), Point3F32::new(1.0, 0.0, 0.0));
    let query = Point3F32::new(0.7, 0.0, 0.0);
    let t = v2::algo::curve::presearch(&line, query, (0.0f32, 1.0f32), 100);
    assert!((t - 0.7).abs() < 0.02); // within one division step
}

#[test]
fn f32_search_parameter_and_nearest() {
    let line: Line<Point3F32> = Line(Point3F32::new(0.0, 0.0, 0.0), Point3F32::new(1.0, 0.0, 0.0));
    let on_curve = Point3F32::new(0.25, 0.0, 0.0);
    let off_curve = Point3F32::new(0.25, 1.0, 0.0);

    let found = v2::SearchParameter::<v2::D1<f32>>::search_parameter(
        &line,
        on_curve,
        v2::SearchParameterHint1D::None,
        0,
    )
    .unwrap();
    assert!((found - 0.25).abs() < 1e-6);

    assert!(
        v2::SearchParameter::<v2::D1<f32>>::search_parameter(
            &line,
            off_curve,
            v2::SearchParameterHint1D::None,
            0,
        )
        .is_none()
    );

    let nearest = v2::SearchNearestParameter::<v2::D1<f32>>::search_nearest_parameter(
        &line,
        off_curve,
        v2::SearchParameterHint1D::None,
        0,
    )
    .unwrap();
    assert!((nearest - 0.25).abs() < 1e-6);
}

// -- f32 vs f64 equivalence --------------------------------------------

#[test]
fn f32_f64_equivalence() {
    let line_f32: Line<Point3F32> =
        Line(Point3F32::new(1.0, 2.0, 3.0), Point3F32::new(4.0, 5.0, 6.0));
    let line_f64: Line<Point3> = Line(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0));

    for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let p32 = v2::ParametricCurve::evaluate(&line_f32, t as f32);
        let p64 = v2::ParametricCurve::evaluate(&line_f64, t);
        assert!((p32.x as f64 - p64.x).abs() < 1e-6);
        assert!((p32.y as f64 - p64.y).abs() < 1e-6);
        assert!((p32.z as f64 - p64.z).abs() < 1e-6);
    }
}
