//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;
use crate::{Curve, Surface, builder};

type Wire = monstertruck_topology::Wire<Point3, Curve>;
type Face = monstertruck_topology::Face<Point3, Curve, Surface>;

/// Helper: builds a rectangular wire in the XY plane.
fn rect_wire(x0: f64, y0: f64, x1: f64, y1: f64) -> Wire {
    let v0 = builder::vertex(Point3::new(x0, y0, 0.0));
    let v1 = builder::vertex(Point3::new(x1, y0, 0.0));
    let v2 = builder::vertex(Point3::new(x1, y1, 0.0));
    let v3 = builder::vertex(Point3::new(x0, y1, 0.0));
    vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v3),
        builder::line(&v3, &v0),
    ]
    .into()
}

/// Helper: builds a CW (clockwise) rectangular wire in the XY plane.
fn rect_wire_cw(x0: f64, y0: f64, x1: f64, y1: f64) -> Wire { rect_wire(x0, y0, x1, y1).inverse() }

#[test]
fn single_wire_ccw() {
    let wire = rect_wire(-1.0, -1.0, 1.0, 1.0);
    let face: Face = attach_plane_normalized(vec![wire]).unwrap();
    assert_eq!(face.boundaries().len(), 1);
}

#[test]
fn single_wire_cw_gets_normalized() {
    // CW wire should be automatically flipped to CCW.
    let wire = rect_wire_cw(-1.0, -1.0, 1.0, 1.0);
    let face: Face = attach_plane_normalized(vec![wire]).unwrap();
    assert_eq!(face.boundaries().len(), 1);
}

#[test]
fn outer_with_hole() {
    let outer = rect_wire(-2.0, -2.0, 2.0, 2.0);
    let hole = rect_wire(-0.5, -0.5, 0.5, 0.5);
    let face: Face = attach_plane_normalized(vec![outer, hole]).unwrap();
    assert_eq!(face.boundaries().len(), 2);
}

#[test]
fn outer_with_hole_both_ccw_gets_normalized() {
    // Both wires are CCW; the hole must be auto-flipped to CW.
    let outer = rect_wire(-2.0, -2.0, 2.0, 2.0);
    let hole = rect_wire(-0.5, -0.5, 0.5, 0.5);
    let face: Face = attach_plane_normalized(vec![outer, hole]).unwrap();
    assert_eq!(face.boundaries().len(), 2);
}

#[test]
fn outer_with_hole_reversed_order() {
    // Hole given first, outer second - should still work.
    let outer = rect_wire(-2.0, -2.0, 2.0, 2.0);
    let hole = rect_wire_cw(-0.5, -0.5, 0.5, 0.5);
    let face: Face = attach_plane_normalized(vec![hole, outer]).unwrap();
    assert_eq!(face.boundaries().len(), 2);
}

#[test]
fn multiple_holes() {
    let outer = rect_wire(-5.0, -5.0, 5.0, 5.0);
    let hole1 = rect_wire(-4.0, -4.0, -2.0, -2.0);
    let hole2 = rect_wire(1.0, 1.0, 3.0, 3.0);
    let face: Face = attach_plane_normalized(vec![outer, hole1, hole2]).unwrap();
    assert_eq!(face.boundaries().len(), 3);
}

#[test]
fn mixed_winding_multiple_holes() {
    // All wires given as CCW; normalization should flip the holes.
    let outer = rect_wire(-5.0, -5.0, 5.0, 5.0);
    let hole1 = rect_wire(-4.0, -4.0, -2.0, -2.0);
    let hole2 = rect_wire(1.0, 1.0, 3.0, 3.0);
    let face: Face = attach_plane_normalized(vec![hole1, outer, hole2]).unwrap();
    assert_eq!(face.boundaries().len(), 3);
}

#[test]
fn open_wire_rejected() {
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
    let v2 = builder::vertex(Point3::new(1.0, 1.0, 0.0));
    let wire: Wire = vec![builder::line(&v0, &v1), builder::line(&v1, &v2)].into();
    let result = attach_plane_normalized::<Curve, Surface>(vec![wire]);
    assert!(matches!(result, Err(Error::OpenWire)));
}

#[test]
fn solid_from_profile_simple() {
    let outer = rect_wire(-1.0, -1.0, 1.0, 1.0);
    let solid =
        solid_from_planar_profile::<Curve, Surface>(vec![outer], Vector3::new(0.0, 0.0, 1.0))
            .unwrap();
    // A box: 6 faces.
    assert_eq!(solid.boundaries()[0].len(), 6);
}

#[test]
fn solid_from_profile_with_hole() {
    let outer = rect_wire(-2.0, -2.0, 2.0, 2.0);
    let hole = rect_wire(-0.5, -0.5, 0.5, 0.5);
    let solid =
        solid_from_planar_profile::<Curve, Surface>(vec![outer, hole], Vector3::new(0.0, 0.0, 1.0))
            .unwrap();
    let shell = &solid.boundaries()[0];
    // Bottom face + top face + 4 outer sides + 4 inner sides = 10 faces.
    assert_eq!(shell.len(), 10);
}

#[test]
fn near_degenerate_tiny_hole() {
    let outer = rect_wire(-10.0, -10.0, 10.0, 10.0);
    // Very tiny hole.
    let hole = rect_wire(-0.001, -0.001, 0.001, 0.001);
    let face: Face = attach_plane_normalized(vec![outer, hole]).unwrap();
    assert_eq!(face.boundaries().len(), 2);
}
