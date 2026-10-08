//! Tests that a glyph after the first lands where its advance puts it.

use super::*;

#[test]
fn placed_contour_moves_every_point() {
    let contour = OffsetContour::placed(
        0.0,
        0.0,
        vec![
            Segment::Line(10.0, 0.0),
            Segment::Quad(15.0, 5.0, 10.0, 10.0),
            Segment::Line(0.0, 10.0),
        ],
        100.0,
    );
    let wire = contour_to_wire(
        contour.start_x,
        contour.start_y,
        &contour.segments,
        1.0,
        false,
        0.0,
        1e-7,
    )
    .unwrap();
    let xs: Vec<f64> = wire.vertex_iter().map(|v| v.point().x).collect();
    assert_eq!(xs, vec![100.0, 110.0, 110.0, 100.0]);
}
