//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

#[test]
fn surface_curve_rejects_invalid_pcurve_on_identical_surface() {
    let surface = Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy()));
    let leader = Curve3D::Line(Line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)));
    let invalid_trim = ParameterCurve::new(
        Box::new(Curve2D::Line(Line(
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 1.0),
        ))),
        Box::new(surface.clone()),
    );
    let surface_curve = SurfaceCurve3D::new(
        SurfaceCurveKind::SurfaceCurve,
        Box::new(leader),
        vec![SurfaceCurveAssociatedGeometry::ParameterCurve(invalid_trim)],
        SurfaceCurveRepresentation::Curve3D,
    );

    assert!(
        surface_curve.parameter_curve_on(&surface).is_none(),
        "Invalid face-local pcurves must not be accepted only because their surface entity matches."
    );
}

#[test]
fn public_parameter_curve_api_uses_descriptive_names() {
    let surface = Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy()));
    let trim: StepParameterCurve = ParameterCurve::new(
        Box::new(Curve2D::Line(Line(
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
        ))),
        Box::new(surface),
    );
    let curve = Curve3D::ParameterCurve(trim);

    assert!(matches!(curve, Curve3D::ParameterCurve(_)));
}
