//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

fn plane_surface() -> Surface {
    Surface::Plane(Plane::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ))
}

fn exact_quadratic_leader() -> NurbsCurve<Vector4> {
    NurbsCurve::new(BsplineCurve::new(
        KnotVector::bezier_knot(2),
        vec![
            Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector4::new(0.5, 1.0, 0.0, 1.0),
            Vector4::new(1.0, 0.0, 0.0, 1.0),
        ],
    ))
}

fn line_pcurve_on_plane() -> Curve {
    Curve::ParameterCurve(ParameterCurve::new(
        Curve2D::Line(Line(Point2::new(0.0, 0.0), Point2::new(1.0, 0.0))),
        Box::new(plane_surface()),
    ))
}

fn intersection_curve_with_leader(leader: Curve) -> Curve {
    Curve::IntersectionCurve(SurfaceCurve::with_boundaries(
        Box::new(plane_surface()),
        Box::new(plane_surface()),
        Box::new(leader),
        None,
        None,
    ))
}

/// Boolean seam edges are `IntersectionCurve`s with exact leaders. The
/// fillet conversion must return THAT curve, not a resampled polyline.
#[test]
fn intersection_curve_with_exact_leader_converts_exactly() {
    let leader = exact_quadratic_leader();
    let curve = intersection_curve_with_leader(Curve::NurbsCurve(leader.clone()));
    let converted = NurbsCurve::<Vector4>::try_from(curve).expect("exact leader must convert");
    assert_eq!(converted.degree(), leader.degree());
    assert_eq!(converted.knot_vector(), leader.knot_vector());
    assert_eq!(converted.control_points(), leader.control_points());
}

/// A parameter curve has no exact NURBS representation: refuse, never
/// silently sample.
#[test]
fn parameter_curve_refuses_instead_of_sampling() {
    assert!(NurbsCurve::<Vector4>::try_from(line_pcurve_on_plane()).is_err());
}

/// A seam whose leader chain bottoms out non-exact must refuse as well.
#[test]
fn intersection_curve_with_non_exact_leader_refuses() {
    let curve = intersection_curve_with_leader(line_pcurve_on_plane());
    assert!(NurbsCurve::<Vector4>::try_from(curve).is_err());
}
