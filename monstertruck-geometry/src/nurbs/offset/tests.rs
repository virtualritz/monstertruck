//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

#[test]
fn offset_2d_straight_line() {
    let line = BsplineCurve::new(
        KnotVector::bezier_knot(1),
        vec![Point2::new(0.0, 0.0), Point2::new(4.0, 0.0)],
    );
    let offset = curve_offset_2d(&line, 1.0, 20).unwrap();
    // Check several points along the offset.
    for i in 0..=10 {
        let u = i as f64 / 10.0;
        let pt = offset.subs(u);
        assert!(
            (pt.y - 1.0).abs() < 0.05,
            "at u={u}: expected y~1.0, got y={}",
            pt.y,
        );
        let expected_x = 4.0 * u;
        assert!(
            (pt.x - expected_x).abs() < 0.1,
            "at u={u}: expected x~{expected_x}, got x={}",
            pt.x,
        );
    }
}

#[test]
fn offset_3d_straight_line_z_normal() {
    let line = BsplineCurve::new(
        KnotVector::bezier_knot(1),
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0)],
    );
    let offset = curve_offset_3d(&line, 2.0, Vector3::unit_z(), 20).unwrap();
    for i in 0..=10 {
        let u = i as f64 / 10.0;
        let pt = offset.subs(u);
        assert!(
            (pt.z - 2.0).abs() < 0.05,
            "at u={u}: expected z~2.0, got z={}",
            pt.z,
        );
    }
}

#[test]
fn surface_offset_flat_plane() {
    let surface = BsplineSurface::new(
        (KnotVector::bezier_knot(1), KnotVector::bezier_knot(1)),
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 3.0, 0.0)],
            vec![Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 3.0, 0.0)],
        ],
    );
    let offset_surf = surface_offset(&surface, 1.5, (10, 10)).unwrap();
    // Check grid of points.
    for i in 0..=5 {
        for j in 0..=5 {
            let u = i as f64 / 5.0;
            let v = j as f64 / 5.0;
            let pt = offset_surf.subs(u, v);
            assert!(
                (pt.z - 1.5).abs() < 0.15,
                "at ({u},{v}): expected z~1.5, got z={}",
                pt.z,
            );
        }
    }
}
