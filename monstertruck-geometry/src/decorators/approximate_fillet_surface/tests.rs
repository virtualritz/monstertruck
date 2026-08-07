//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;
use proptest::{prelude::*, property_test};

#[property_test]
fn plane_cylinder(#[strategy = 0.0..=1.0] u: f64, #[strategy = 0.0..=1.0] v: f64) {
    let surface = ApproximateFilletSurface {
        knot_vec: KnotVector::bezier_knot(1),
        surface0: Plane::xy(),
        side_control_points0: vec![(-1.0, 0.0).into(), (-1.0, 1.0).into()],
        tangent_vecs0: vec![(f64::sqrt(2.0), 0.0).into(); 2],
        surface1: Plane::yz(),
        side_control_points1: vec![(0.0, -1.0).into(), (1.0, -1.0).into()],
        tangent_vecs1: vec![(-f64::sqrt(2.0), 0.0).into(); 2],
        weights: vec![(1.0 + f64::sqrt(2.0)) / 3.0; 2],
    };
    let w = 1.0 / f64::sqrt(2.0);
    let nurbs_surface = NurbsSurface::new(BsplineSurface::<Vector4>::new(
        (KnotVector::bezier_knot(2), KnotVector::bezier_knot(1)),
        vec![
            vec![(-1.0, 0.0, 0.0, 1.0).into(), (-1.0, 1.0, 0.0, 1.0).into()],
            vec![(0.0, 0.0, 0.0, w).into(), (0.0, w, 0.0, w).into()],
            vec![(0.0, 0.0, -1.0, 1.0).into(), (0.0, 1.0, -1.0, 1.0).into()],
        ],
    ));

    prop_assert_near!(surface.evaluate(u, v), nurbs_surface.evaluate(u, v));
    prop_assert_near!(
        surface.derivatives(3, u, v),
        nurbs_surface.derivatives(3, u, v)
    );
}

#[property_test]
fn test_ders(#[strategy = 0.0..=1.0] u: f64, #[strategy = 0.0..=1.0] v: f64) {
    #[rustfmt::skip]
    let surface0 = &BsplineSurface::<Point3>::new(
        (KnotVector::bezier_knot(2), KnotVector::bezier_knot(2)),
        vec![
            vec![(-1.0, 0.0, 0.0).into(), (-1.0, 0.5, 0.0).into(), (-1.0, 1.0, 0.0).into()],
            vec![(-0.5, 0.0, 0.0).into(), (-0.5, 0.5, 1.0).into(), (-0.5, 1.0, 0.0).into()],
            vec![(0.0, 0.0, 0.0).into(), (0.0, 0.5, 0.0).into(), (0.0, 1.0, 0.0).into()],
        ]
    );
    #[rustfmt::skip]
    let surface1 = &BsplineSurface::<Point3>::new(
        (KnotVector::bezier_knot(2), KnotVector::bezier_knot(2)),
        vec![
            vec![(0.0, 0.0, -1.0).into(), (0.0, 0.0, -0.5).into(), (0.0, 0.0, 0.0).into()],
            vec![(0.0, 0.5, -1.0).into(), (1.0, 0.5, -0.5).into(), (0.0, 0.5, 0.0).into()],
            vec![(0.0, 1.0, -1.0).into(), (0.0, 1.0, -0.5).into(), (0.0, 1.0, 0.0).into()],
        ]
    );

    let surface = ApproximateFilletSurface {
        knot_vec: KnotVector::bezier_knot(2),
        surface0,
        side_control_points0: vec![(0.8, 0.0).into(), (0.5, 0.5).into(), (0.8, 1.0).into()],
        tangent_vecs0: vec![(0.2, -0.1).into(), (0.4, 0.0).into(), (0.2, 0.1).into()],
        surface1,
        side_control_points1: vec![(0.0, 0.8).into(), (0.5, 0.5).into(), (1.0, 0.8).into()],
        tangent_vecs1: vec![(-0.2, -0.1).into(), (-0.4, 0.0).into(), (-0.2, 0.1).into()],
        weights: vec![1.0, 2.0, 1.0],
    };

    let pt = surface.evaluate(u, v);
    let ders = surface.derivatives(3, u, v);
    assert_near!(pt.to_vec(), ders[0][0]);

    const EPS: f64 = 1.0e-4;

    let upders = surface.derivatives(2, u + EPS, v);
    let umders = surface.derivatives(2, u - EPS, v);
    let calc_uders = upders.element_wise_derivatives(&umders, |x, y| x - y) / (2.0 * EPS);
    let res_uders = ders.derivative_u();

    let iter = res_uders
        .slice_iter()
        .flatten()
        .zip(calc_uders.slice_iter().flatten());
    for (a, b) in iter {
        prop_assert!((a - b).magnitude() < 10.0 * EPS);
    }

    let vpders = surface.derivatives(2, u, v + EPS);
    let vmders = surface.derivatives(2, u, v - EPS);
    let calc_vders = vpders.element_wise_derivatives(&vmders, |x, y| x - y) / (2.0 * EPS);
    let res_vders = ders.derivative_v();

    let iter = res_vders
        .slice_iter()
        .flatten()
        .zip(calc_vders.slice_iter().flatten());
    for (a, b) in iter {
        prop_assert!((a - b).magnitude() < 10.0 * EPS);
    }

    let pt0 = surface.evaluate(0.0, v);
    let (u0, v0) = surface0.search_parameter(pt0, (0.5, 0.5), 100).unwrap();
    assert_near!(surface.normal(0.0, v), surface0.normal(u0, v0));

    let pt1 = surface.evaluate(1.0, v);
    let (u1, v1) = surface1.search_parameter(pt1, (0.5, 0.5), 100).unwrap();
    assert_near!(surface.normal(1.0, v), surface1.normal(u1, v1));
}
