//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

/// Minimal synthetic font test: builds contours manually via the collector.
#[test]
fn contour_collector_line_segments() {
    use ttf_parser::OutlineBuilder;
    let mut c = ContourCollector::new();
    // Square contour.
    c.move_to(0.0, 0.0);
    c.line_to(100.0, 0.0);
    c.line_to(100.0, 100.0);
    c.line_to(0.0, 100.0);
    c.close();
    assert_eq!(c.contours.len(), 1);
    // 3 explicit line_to segments; the close back to start is handled
    // implicitly by `contour_to_wire` when assembling the wire.
    assert_eq!(c.contours[0].2.len(), 3);
}

#[test]
fn contour_to_wire_square() {
    use ttf_parser::OutlineBuilder;
    let mut c = ContourCollector::new();
    c.move_to(0.0, 0.0);
    c.line_to(1.0, 0.0);
    c.line_to(1.0, 1.0);
    c.line_to(0.0, 1.0);
    c.close();

    let (sx, sy, segs) = &c.contours[0];
    let wire = contour_to_wire(*sx, *sy, segs, 1.0, false, 0.0, 1e-7).unwrap();
    assert!(wire.is_closed());
    assert_eq!(wire.len(), 4);
}

#[test]
fn contour_to_wire_with_bezier() {
    use ttf_parser::OutlineBuilder;
    let mut c = ContourCollector::new();
    c.move_to(0.0, 0.0);
    c.quad_to(0.5, 1.0, 1.0, 0.0);
    c.line_to(0.0, 0.0);
    c.close();

    let (sx, sy, segs) = &c.contours[0];
    let wire = contour_to_wire(*sx, *sy, segs, 1.0, false, 0.0, 1e-7).unwrap();
    assert!(wire.is_closed());
    assert_eq!(wire.len(), 2);
}

#[test]
fn contour_to_wire_cubic() {
    use ttf_parser::OutlineBuilder;
    let mut c = ContourCollector::new();
    c.move_to(0.0, 0.0);
    c.curve_to(0.3, 1.0, 0.7, 1.0, 1.0, 0.0);
    c.line_to(0.0, 0.0);
    c.close();

    let (sx, sy, segs) = &c.contours[0];
    let wire = contour_to_wire(*sx, *sy, segs, 1.0, false, 0.0, 1e-7).unwrap();
    assert!(wire.is_closed());
    assert_eq!(wire.len(), 2);
}

#[test]
fn contour_y_flip() {
    let pt = transform_point(10.0, 20.0, 0.01, true, 5.0);
    assert!((pt.x - 0.1).abs() < 1e-10);
    assert!((pt.y - (-0.2)).abs() < 1e-10);
    assert!((pt.z - 5.0).abs() < 1e-10);
}

#[test]
fn contour_no_y_flip() {
    let pt = transform_point(10.0, 20.0, 0.01, false, 0.0);
    assert!((pt.x - 0.1).abs() < 1e-10);
    assert!((pt.y - 0.2).abs() < 1e-10);
}

#[test]
fn multiple_contours() {
    use ttf_parser::OutlineBuilder;
    let mut c = ContourCollector::new();
    // Outer square.
    c.move_to(0.0, 0.0);
    c.line_to(10.0, 0.0);
    c.line_to(10.0, 10.0);
    c.line_to(0.0, 10.0);
    c.close();
    // Inner square (hole).
    c.move_to(2.0, 2.0);
    c.line_to(8.0, 2.0);
    c.line_to(8.0, 8.0);
    c.line_to(2.0, 8.0);
    c.close();

    assert_eq!(c.contours.len(), 2);

    let wires: Vec<Wire> = c
        .contours
        .iter()
        .map(|(sx, sy, segs)| contour_to_wire(*sx, *sy, segs, 0.1, false, 0.0, 1e-7).unwrap())
        .collect();

    assert_eq!(wires.len(), 2);
    assert!(wires.iter().all(|w| w.is_closed()));
}

#[test]
fn degenerate_contour_filtered() {
    assert!(is_degenerate_contour(
        5.0,
        5.0,
        &[Segment::Line(5.0, 5.0), Segment::Line(5.0, 5.0)]
    ));
    assert!(!is_degenerate_contour(0.0, 0.0, &[Segment::Line(1.0, 0.0)]));
}
