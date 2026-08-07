//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

#[test]
fn empty_curves_error() {
    let mut curves: Vec<BsplineCurve<Vector2>> = Vec::new();
    assert!(matches!(
        make_curves_compatible(&mut curves),
        Err(Error::EmptyControlPoints)
    ));
}

#[test]
fn single_curve_normalizes_knots() {
    let mut curves = vec![BsplineCurve::new(
        KnotVector::from(vec![0.0, 0.0, 2.0, 2.0]),
        vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0)],
    )];
    make_curves_compatible(&mut curves).unwrap();
    assert_eq!(curves[0].knot_vector().as_slice(), &[0.0, 0.0, 1.0, 1.0]);
}

#[test]
fn two_curves_same_degree_different_knots() {
    let c0 = BsplineCurve::new(
        KnotVector::from(vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]),
        vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(3.0, 3.0),
        ],
    );
    let c1 = BsplineCurve::new(
        KnotVector::from(vec![0.0, 0.0, 0.0, 0.75, 1.0, 1.0, 1.0]),
        vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(2.0, 1.0),
            Vector2::new(3.0, 1.0),
        ],
    );
    let org0 = c0.clone();
    let org1 = c1.clone();

    let mut curves = vec![c0, c1];
    make_curves_compatible(&mut curves).unwrap();

    assert_eq!(curves[0].knot_vector(), curves[1].knot_vector());
    assert_eq!(curves[0].degree(), curves[1].degree());
    // Shape is preserved.
    assert!(curves[0].near2_as_curve(&org0));
    assert!(curves[1].near2_as_curve(&org1));
}

#[test]
fn two_curves_different_degrees() {
    let c0 = BsplineCurve::new(
        KnotVector::bezier_knot(1),
        vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0)],
    );
    let c1 = BsplineCurve::new(
        KnotVector::bezier_knot(3),
        vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(0.33, 1.0),
            Vector2::new(0.66, 1.0),
            Vector2::new(1.0, 0.0),
        ],
    );
    let org0 = c0.clone();
    let org1 = c1.clone();

    let mut curves = vec![c0, c1];
    make_curves_compatible(&mut curves).unwrap();

    assert_eq!(curves[0].degree(), curves[1].degree());
    assert_eq!(curves[0].degree(), 3);
    assert_eq!(curves[0].knot_vector(), curves[1].knot_vector());
    assert!(curves[0].near2_as_curve(&org0));
    assert!(curves[1].near2_as_curve(&org1));
}

#[test]
fn three_curves_mixed() {
    let c0 = BsplineCurve::new(
        KnotVector::bezier_knot(1),
        vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0)],
    );
    let c1 = BsplineCurve::new(
        KnotVector::bezier_knot(2),
        vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(0.5, 1.0),
            Vector2::new(1.0, 0.0),
        ],
    );
    let c2 = BsplineCurve::new(
        KnotVector::from(vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]),
        vec![
            Vector2::new(0.0, 1.0),
            Vector2::new(0.5, 0.0),
            Vector2::new(0.5, 1.0),
            Vector2::new(1.0, 0.0),
        ],
    );
    let org0 = c0.clone();
    let org1 = c1.clone();
    let org2 = c2.clone();

    let mut curves = vec![c0, c1, c2];
    make_curves_compatible(&mut curves).unwrap();

    // All must share the same degree and knot vector.
    assert_eq!(curves[0].degree(), curves[1].degree());
    assert_eq!(curves[1].degree(), curves[2].degree());
    assert_eq!(curves[0].knot_vector(), curves[1].knot_vector());
    assert_eq!(curves[1].knot_vector(), curves[2].knot_vector());
    // Shapes preserved.
    assert!(curves[0].near2_as_curve(&org0));
    assert!(curves[1].near2_as_curve(&org1));
    assert!(curves[2].near2_as_curve(&org2));
}

#[test]
fn empty_surfaces_error() {
    let mut surfaces: Vec<BsplineSurface<Vector2>> = Vec::new();
    assert!(matches!(
        make_surfaces_compatible(&mut surfaces),
        Err(Error::EmptyControlPoints)
    ));
}

#[test]
fn two_surfaces_different_degrees() {
    let s0 = BsplineSurface::new(
        (KnotVector::bezier_knot(1), KnotVector::bezier_knot(1)),
        vec![
            vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)],
            vec![Vector2::new(0.0, 1.0), Vector2::new(1.0, 1.0)],
        ],
    );
    let s1 = BsplineSurface::new(
        (KnotVector::bezier_knot(2), KnotVector::bezier_knot(2)),
        vec![
            vec![
                Vector2::new(0.0, 0.0),
                Vector2::new(0.5, 0.0),
                Vector2::new(1.0, 0.0),
            ],
            vec![
                Vector2::new(0.0, 0.5),
                Vector2::new(0.5, 0.5),
                Vector2::new(1.0, 0.5),
            ],
            vec![
                Vector2::new(0.0, 1.0),
                Vector2::new(0.5, 1.0),
                Vector2::new(1.0, 1.0),
            ],
        ],
    );
    let org0 = s0.clone();
    let org1 = s1.clone();

    let mut surfaces = vec![s0, s1];
    make_surfaces_compatible(&mut surfaces).unwrap();

    assert_eq!(surfaces[0].udegree(), surfaces[1].udegree());
    assert_eq!(surfaces[0].vdegree(), surfaces[1].vdegree());
    assert_eq!(surfaces[0].knot_vectors(), surfaces[1].knot_vectors());
    assert!(surfaces[0].near2_as_surface(&org0));
    assert!(surfaces[1].near2_as_surface(&org1));
}
