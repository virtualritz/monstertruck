use monstertruck_core::cgmath64::{Point3, Vector4};

use super::*;

fn polynomial_surface(degree_u: usize, degree_v: usize) -> BsplineSurface<Point3> {
    let control_points = (0..=degree_u)
        .map(|u| {
            (0..=degree_v)
                .map(|v| Point3::new(u as f64, v as f64, 0.0))
                .collect()
        })
        .collect();

    BsplineSurface::new(
        (
            KnotVector::bezier_knot(degree_u),
            KnotVector::bezier_knot(degree_v),
        ),
        control_points,
    )
}

#[test]
fn boundary_side_selects_the_cross_boundary_axis() {
    let surface = polynomial_surface(1, 3);

    assert_eq!(
        surface
            .continuity_capability(BoundarySide::MinV, ContinuityOrder::G3)
            .unsupported_reason(),
        None
    );
    assert_eq!(
        surface
            .continuity_capability(BoundarySide::MinU, ContinuityOrder::G2)
            .unsupported_reason(),
        Some(
            UnsupportedContinuityCapability::InsufficientDegreeAndControlRows {
                available_degree: 1,
                required_degree: 2,
                available_rows: 2,
                required_rows: 3,
            }
        )
    );
}

#[test]
fn unclamped_cross_boundary_axis_is_unsupported() {
    let surface = BsplineSurface::new(
        (
            KnotVector::from(vec![0.0, 1.0, 2.0, 3.0]),
            KnotVector::bezier_knot(1),
        ),
        vec![
            vec![Point3::new(0.0, 0.0, 0.0); 2],
            vec![Point3::new(1.0, 0.0, 0.0); 2],
        ],
    );

    assert_eq!(
        surface
            .continuity_capability(BoundarySide::MaxU, ContinuityOrder::G1)
            .unsupported_reason(),
        Some(UnsupportedContinuityCapability::UnclampedBoundary)
    );
    assert_eq!(
        surface
            .continuity_capability(BoundarySide::MaxV, ContinuityOrder::G1)
            .unsupported_reason(),
        None
    );
}

#[test]
fn nurbs_capability_requires_positive_finite_weights() {
    let positive = NurbsSurface::new(BsplineSurface::new(
        (KnotVector::bezier_knot(1), KnotVector::bezier_knot(1)),
        vec![vec![Vector4::new(0.0, 0.0, 0.0, 1.0); 2]; 2],
    ));
    let zero_weight = NurbsSurface::new(BsplineSurface::new(
        (KnotVector::bezier_knot(1), KnotVector::bezier_knot(1)),
        vec![vec![Vector4::new(0.0, 0.0, 0.0, 0.0); 2]; 2],
    ));

    assert_eq!(
        positive
            .continuity_capability(BoundarySide::MinU, ContinuityOrder::G1)
            .unsupported_reason(),
        None
    );
    assert_eq!(
        zero_weight
            .continuity_capability(BoundarySide::MinU, ContinuityOrder::G0)
            .unsupported_reason(),
        Some(UnsupportedContinuityCapability::NonPositiveWeight)
    );
}

/// Insufficient degree ALONE, with the control rows adequate.
///
/// This is the branch that reports `InsufficientDegree` while still carrying a
/// known maximum order, so it is the one that would panic if
/// `try_unsupported`'s precondition -- a known maximum strictly below the
/// request -- were ever violated. A degree-1 axis with four control rows asked
/// for `G2` hits it: the rows suffice for `G2`, the degree does not.
#[test]
fn insufficient_degree_alone_reports_the_achievable_order() {
    let surface = BsplineSurface::new(
        (KnotVector::uniform_knot(1, 3), KnotVector::bezier_knot(2)),
        vec![vec![Point3::new(0.0, 0.0, 0.0); 3]; 4],
    );

    let capability = surface.continuity_capability(BoundarySide::MinU, ContinuityOrder::G2);

    assert_eq!(
        capability.unsupported_reason(),
        Some(UnsupportedContinuityCapability::InsufficientDegree {
            available: 1,
            required: 2,
        }),
    );
    // Strictly below the request, which is what makes the report coherent.
    assert_eq!(
        capability.maximum_supported_order(),
        Some(ContinuityOrder::G1)
    );
}
