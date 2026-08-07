//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use crate::*;

/// RED witness (010 stage P-D'): every face of a cuboid SHOULD carry the
/// canonical rectangular frame -- in the face's own surface parameters its
/// trim loop is the unit square, so the loop's parameter bounds are
/// `((0, 1), (0, 1))`.
///
/// Today the side-face plane frame takes `v[i + 1]` unreduced, so at `i == 3`
/// the u-axis is a face DIAGONAL rather than an edge. That face's trim loop
/// is the sheared parallelogram `(0,0) (1,-1) (1,0) (0,1)`, whose bounds are
/// `((0, 1), (-1, 1))` -- a rectangle of twice the face's area. The
/// exact-clip domain the boolean kernel derives for a face is exactly these
/// bounds (`trimmed_face_param_range_from_loops_exact`), so the shear makes
/// the domain rectangle disagree with the true trim window.
///
/// The one-line repair (`v[(i + 1) % 4]`) turns this green, but it is NOT
/// yet landable: the loops_store seam cut on that face currently depends on
/// the enlarged rectangle. With the canonical frame, the finding-006 guard
/// row `SW-B3-PLANE-SPHERE-DIFFERENCE-Iab-T00-S1-D07-Ga` oversplits face 4
/// (3 loops `[4,9,9]` -> 4 loops `[4,4,10,9]`, an extra kept `And`
/// fragment) and regresses from `pi/12` to `3/2 * pi/12` -- a SILENT-WRONG
/// solid, plus a 1 s -> 14 s cost. Un-ignore together with the loops_store
/// fix that makes the seam cut independent of the domain rectangle.
#[test]
#[ignore = "RED witness: the canonical-frame repair regresses the 006 guard \
            row SW-B3-PLANE-SPHERE-DIFFERENCE-Iab-T00-S1-D07-Ga to a \
            silent-wrong 3/2 * pi/12; needs the loops_store seam-cut fix first"]
fn cuboid_faces_carry_the_canonical_unit_square_frame() {
    // Deliberately non-cubic and off-origin, with the corners given in an
    // order the bounding box must normalise.
    let p = Point3::new(-1.0, 2.0, -3.0);
    let q = Point3::new(10.0, -5.0, 4.0);
    let solid: Solid = primitive::cuboid(BoundingBox::from_iter([p, q]));
    let shell = &solid.boundaries()[0];
    assert_eq!(shell.len(), 6, "a cuboid has six faces.");

    shell.iter().enumerate().for_each(|(index, face)| {
        let surface = face.surface();
        let ((u_min, u_max), (v_min, v_max)) = face
            .boundaries()
            .iter()
            .flatten()
            .map(|edge| {
                surface
                    .search_parameter(edge.front().point(), None, 100)
                    .unwrap_or_else(|| {
                        panic!("face {index}: a boundary vertex is not on its own surface")
                    })
            })
            .fold(
                (
                    (f64::INFINITY, f64::NEG_INFINITY),
                    (f64::INFINITY, f64::NEG_INFINITY),
                ),
                |((u_min, u_max), (v_min, v_max)), (u, v)| {
                    ((u_min.min(u), u_max.max(u)), (v_min.min(v), v_max.max(v)))
                },
            );
        assert_near!(u_min, 0.0, "face {index} u_min");
        assert_near!(u_max, 1.0, "face {index} u_max");
        assert_near!(v_min, 0.0, "face {index} v_min");
        assert_near!(v_max, 1.0, "face {index} v_max");
    });
}
