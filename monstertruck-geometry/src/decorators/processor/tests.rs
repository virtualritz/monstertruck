//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;
use std::f64::consts::FRAC_PI_2;

fn assert_division_samples_match_parameters<C, P>(
    curve: &C,
    (parameters, points): (Vec<f64>, Vec<P>),
) where
    C: BoundedCurve + ParameterDivision1D<Point = P> + ParametricCurve<Point = P>,
    P: Tolerance,
{
    assert_eq!(parameters.len(), points.len());
    assert!(parameters.windows(2).all(|window| window[0] <= window[1]));
    assert!(
        parameters
            .iter()
            .zip(points)
            .all(|(parameter, point)| curve.evaluate(*parameter).near(&point))
    );
}

fn assert_division_points_match_parameters<C, P>(curve: &C)
where
    C: BoundedCurve + ParameterDivision1D<Point = P> + ParametricCurve<Point = P>,
    P: Tolerance, {
    let range = curve.range_tuple();
    assert_division_samples_match_parameters(curve, curve.parameter_division(range, 0.01));
    let Some(samples) = curve.try_parameter_division(range, 0.01) else {
        panic!("fallible division should accept positive tolerance");
    };
    assert_division_samples_match_parameters(curve, samples);
}

#[test]
fn inverted_matrix3_curve_division_points_match_parameters() {
    let mut curve = Processor::new(TrimmedCurve::new(
        UnitCircle::<Point2>::new(),
        (0.0, FRAC_PI_2),
    ));
    curve.invert();

    assert_division_points_match_parameters(&curve);
}

#[test]
fn inverted_matrix4_curve_division_points_match_parameters() {
    let mut curve = Processor::new(TrimmedCurve::new(
        UnitCircle::<Point3>::new(),
        (0.0, FRAC_PI_2),
    ));
    curve.invert();

    assert_division_points_match_parameters(&curve);
}
