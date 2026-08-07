//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

/// Creates a T-NURCC cube with sides of lenth `1`, lower front left point at `(0, 0, 0)`,
/// and all verticies in the first octant.
fn make_cube() -> Result<Tnurcc<Point3>> {
    use crate::prelude::Point3;
    let points = vec![
        Point3::from((0.0, 0.0, 0.0)), // 0
        Point3::from((0.0, 0.0, 1.0)), // 1
        Point3::from((1.0, 0.0, 1.0)), // 2
        Point3::from((1.0, 0.0, 0.0)), // 3
        Point3::from((0.0, 1.0, 0.0)), // 4
        Point3::from((0.0, 1.0, 1.0)), // 5
        Point3::from((1.0, 1.0, 1.0)), // 6
        Point3::from((1.0, 1.0, 0.0)), // 7
    ];

    let faces = vec![
        [
            // Front
            (0, vec![(3, 1.0)]),
            (3, vec![(2, 1.0)]),
            (2, vec![(1, 1.0)]),
            (1, vec![(0, 1.0)]),
        ],
        [
            // Left
            (0, vec![(1, 1.0)]),
            (1, vec![(5, 1.0)]),
            (5, vec![(4, 1.0)]),
            (4, vec![(0, 1.0)]),
        ],
        [
            // Top
            (1, vec![(2, 1.0)]),
            (2, vec![(6, 1.0)]),
            (6, vec![(5, 1.0)]),
            (5, vec![(1, 1.0)]),
        ],
        [
            // Back
            (4, vec![(5, 1.0)]),
            (5, vec![(6, 1.0)]),
            (6, vec![(7, 1.0)]),
            (7, vec![(4, 1.0)]),
        ],
        [
            // Right
            (2, vec![(3, 1.0)]),
            (3, vec![(7, 1.0)]),
            (7, vec![(6, 1.0)]),
            (6, vec![(2, 1.0)]),
        ],
        [
            // Bottom
            (0, vec![(4, 1.0)]),
            (4, vec![(7, 1.0)]),
            (7, vec![(3, 1.0)]),
            (3, vec![(0, 1.0)]),
        ],
    ];

    Tnurcc::try_new(points, faces)
}

fn verify_tnurcc_control_points(t: &Tnurcc<Point3>) {
    for (i, p) in t.control_points.iter().enumerate() {
        // Incoming edge of the point
        let point_edge =
            Arc::clone(p.read().incoming_edge.as_ref().unwrap_or_else(|| {
                panic!("Point {} should have an incoming edge", p.read().index,)
            }));

        // Point-based iter will rotate around the current control point
        // Incedentally verifies that the control point is referenced by the edge
        let iter = TnurccAcwPointIter::from_edge(
            Arc::clone(&point_edge),
            point_edge
                .read()
                .point_end(Arc::clone(p))
                .unwrap_or_else(|| {
                    panic!(
                        "Point {} should be a side of its incoming edge",
                        p.read().index,
                    )
                }),
        );
        let next = iter.last().unwrap_or_else(|| {
            panic!(
                "Point {} edge-rotation iterator should wrap around and end.",
                p.read().index,
            )
        });

        // Assert the next acw edge (from the last one returned by the iter)
        // is the same edge as the one it started at
        let next_point_end = next.read().point_end(Arc::clone(p)).unwrap_or_else(|| {
            panic!(
                "Edges reached through point {} iter should be connected to that point",
                p.read().index,
            )
        });
        let final_edge = next.read().acw_edge_from_end(next_point_end);
        assert!(
            std::ptr::eq(final_edge.as_ref(), point_edge.as_ref()),
            "Iter does not rotate around point {} correctly. Reached {}, expected {}",
            p.read().index,
            final_edge.read().index,
            point_edge.read().index,
        );

        // Calculate the anti-clockwise valence of the point and verify it matches the
        // recorded valence of the point.
        let iter = TnurccAcwPointIter::from_edge(
            Arc::clone(&point_edge),
            point_edge
                .read()
                .point_end(Arc::clone(p))
                .unwrap_or_else(|| {
                    panic!(
                        "Point {} should be a side of its incoming edge",
                        p.read().index,
                    )
                }),
        );
        let acw_calc_valence = iter.count();
        assert!(
            acw_calc_valence == p.read().valence,
            "Point {} anti-clockwise valence {} does not match recorded valence {}",
            p.read().index,
            acw_calc_valence,
            p.read().valence,
        );

        // Check that the index field matches the index of the point
        assert!(
            p.read().index == i,
            "Point {} index field must match index in mesh points array",
            p.read().index,
        );
    }
}

fn verify_tnurcc_edges(t: &Tnurcc<Point3>) {
    for (i, e) in t.edges.iter().enumerate() {
        // Check index field
        assert!(
            i == e.read().index,
            "Tnurcc edge index field must be equal to edge index in edge array"
        );

        let common_faces = [
            TnurccFaceSide::Left,
            TnurccFaceSide::Left,
            TnurccFaceSide::Right,
            TnurccFaceSide::Right,
        ];

        let common_points = [
            [TnurccVertexEnd::Dest, TnurccVertexEnd::Origin],
            [TnurccVertexEnd::Origin, TnurccVertexEnd::Dest],
            [TnurccVertexEnd::Dest, TnurccVertexEnd::Origin],
            [TnurccVertexEnd::Origin, TnurccVertexEnd::Dest],
        ];

        // Check connected edges
        for (dir_index, &dir) in [
            TnurccConnection::LeftAcw,
            TnurccConnection::LeftCw,
            TnurccConnection::RightAcw,
            TnurccConnection::RightCw,
        ]
        .iter()
        .enumerate()
        {
            // Get edge in the direction under investigation
            let con = e.read().connection(dir);

            // Check the face between the two is the same and correct
            let common_face = e
                .read()
                .common_face(Arc::clone(&con))
                .expect("Connected edges must have a common face between them");
            assert!(std::ptr::eq(
                common_face.as_ref(),
                e.read()
                    .face_from_side(common_faces[dir_index])
                    .expect("Tnurcc must be closed on all edges")
                    .as_ref()
            ));

            // Check that the point between them is the same and correct
            let common_point = e
                .read()
                .common_point(Arc::clone(&con))
                .expect("Connected edges must have a common point between them");

            // In order to check to make sure that the common points is the correct one,
            // both the connection and orientation of the connected edge relative to the
            // common face needs to be computed in order to know what the relative
            // orientation of the two edges is to each other.
            let other_common_point = con.read().point_at_end(
                common_points[dir as usize][con
                    .read()
                    .face_side(Arc::clone(&common_face))
                    .expect("Common face must be a side on con")
                    as usize],
            );

            assert!(
                std::ptr::eq(common_point.as_ref(), other_common_point.as_ref()),
                "Connected edges {} and {} do not share the correct point.",
                e.read().index,
                con.read().index
            );
        }
    }
}

fn verify_tnurcc_faces(t: &Tnurcc<Point3>) {
    for face in t.faces.iter() {
        // Get reference edge for face
        let face_edge = Arc::clone(
            face.read()
                .edge
                .as_ref()
                .expect("All faces should have a reference edge in T-NURCC"),
        );

        // Assert the next acw edge (from the last one returned by the iter)
        // is the same edge as the one it started at
        let last_edge = TnurccAcwFaceIter::try_from_edge(
            Arc::clone(&face_edge),
            face_edge.read().face_side(Arc::clone(face)).unwrap(),
        )
        .expect("Prevously tested assertion")
        .last()
        .expect("Iter of size greater than 0 should have a last element");

        // Assert that the face is closed (The next edge around the face after exhausting the iterator
        // should be the original reference edge)
        let next_face_side = last_edge
            .read()
            .face_side(Arc::clone(face))
            .expect("Edges reached through a face iter should be connected to that face");
        let final_edge = last_edge.read().acw_edge_from_side(next_face_side);
        assert!(
            std::ptr::eq(final_edge.as_ref(), face_edge.as_ref()),
            "Iter does not rotate around face correctly. Reached {}, expected {}",
            final_edge.read().index,
            face_edge.read().index,
        );
    }
}

#[test]
fn t_nurcc_test_make_cube_euclidiean_geometry() {
    // Sanity check that the cube is (probably) actually a cube

    let surface = make_cube();
    assert!(
        surface.is_ok(),
        "Surface was unsuccessfuly created with error: {}.",
        surface.err().unwrap()
    );
    let surface = make_cube().unwrap();

    assert_eq!(surface.faces.len(), 6, "Cube does not contain 6 faces.");
    assert_eq!(
        surface.control_points.len(),
        8,
        "Cube does not contain 8 verticies."
    );
    assert_eq!(surface.edges.len(), 12, "Cube does not contain 12 edges.");
}

#[test]
fn t_nurcc_test_cube_control_point_properties() {
    let t = make_cube().expect("Cube should be successfuly created");

    verify_tnurcc_control_points(&t);

    for p in t.control_points.iter() {
        // Check valencies
        assert_eq!(
            p.read().valence,
            3,
            "Point {} does not have a valence of 3.",
            p.read().index,
        );
    }
}

#[test]
fn t_nurcc_test_cube_edge_properties() {
    let t = make_cube().expect("Cube should be successfuly created");

    verify_tnurcc_edges(&t);
}

#[test]
fn t_nurcc_test_cube_face_properties() {
    let surface = make_cube().unwrap();

    verify_tnurcc_faces(&surface);

    for face in surface.faces.iter() {
        let face_edge = Arc::clone(
            face.read()
                .edge
                .as_ref()
                .expect("All faces should have a reference edge in T-NURCC"),
        );

        // Assert that each face has four edges
        let edge_count = TnurccAcwFaceIter::try_from_edge(
            Arc::clone(&face_edge),
            face_edge.read().face_side(Arc::clone(face)).unwrap(),
        )
        .expect("face_edge should have Some(face) because it was cloned from face")
        .count();

        assert!(
            edge_count == 4,
            "Rectangular faces should have 4 faces to rotate around"
        );
    }
}

fn t_nurcc_subdivded_cube() -> Tnurcc<Point3> {
    let mut surface = make_cube().unwrap();
    surface
        .global_subdivide()
        .expect("Subdivision of cube is possible");
    surface
}

#[test]
fn t_nurcc_test_subdivide_euclidean_geometry() {
    let surface = t_nurcc_subdivded_cube();

    // Check basic geometric properties
    assert_eq!(
        surface.faces.len(),
        6 * 4,
        "Number of faces after subdivide should be 4 times the original quantity"
    );
    assert_eq!(
        surface.control_points.len(),
        (8 + 12 + 6),
        "Number of points after subdivide should be the sum of points, edges, and faces prior to subdividing"
    );
    assert_eq!(
        surface.edges.len(),
        (12 * 2 + 4 * 6),
        "Number of edges after subdivide should be the sum of twice the count of edges prior subdividing and the sum of the number of edges on each face for each face"
    );
}

#[test]
fn t_nurcc_test_subdivide_edges() {
    let surface = t_nurcc_subdivded_cube();

    verify_tnurcc_edges(&surface);
}

#[test]
fn t_nurcc_test_subdivide_faces() {
    let surface = t_nurcc_subdivded_cube();

    verify_tnurcc_faces(&surface);

    // Make sure the faces are well formed (a little redundant but better be thorough)
    surface.faces.iter().for_each(|f| {
        let start_edge = Arc::clone(
            f.read()
                .edge
                .as_ref()
                .expect("All faces should have an edge"),
        );

        // Anticlockwise traversal
        let mut acw_traverse_edge = Arc::clone(&start_edge);
        // Each face is 4 sided
        for i in 0..4 {
            acw_traverse_edge = {
                let side = acw_traverse_edge
                    .read()
                    .face_side(Arc::clone(f))
                    .unwrap_or_else(|| panic!("Face should be connected to reference edge, error on ACW traversal {} face {}", i, f.read().index));
                match side {
                    TnurccFaceSide::Left => acw_traverse_edge
                        .read()
                        .connection(TnurccConnection::LeftAcw),
                    TnurccFaceSide::Right => acw_traverse_edge
                        .read()
                        .connection(TnurccConnection::RightAcw),
                }
            };
        }

        // Clockwise traversal
        let mut cw_traverse_edge = Arc::clone(&start_edge);
        // Each face is 4 sided
        for i in 0..4 {
            cw_traverse_edge = {
                let side = cw_traverse_edge
                    .read()
                    .face_side(Arc::clone(f))
                    .unwrap_or_else(|| panic!("Face should be connected to reference edge, error on CW traversal {} face {}", i, f.read().index));
                match side {
                    TnurccFaceSide::Left => cw_traverse_edge
                        .read()
                        .connection(TnurccConnection::LeftCw),
                    TnurccFaceSide::Right => cw_traverse_edge
                        .read()
                        .connection(TnurccConnection::RightCw),
                }
            };
        }

        assert!(
            std::ptr::eq(start_edge.as_ref(), acw_traverse_edge.as_ref()),
            "Anticlockwise traversal around face index {} did not return to the start edge.",
            f.read().index
        );

        assert!(
            std::ptr::eq(start_edge.as_ref(), cw_traverse_edge.as_ref()),
            "Clockwise traversal around face index {} did not return to the start edge.",
            f.read().index
        );
    });
}

#[test]
fn t_nurcc_test_subdivide_points() {
    let surface = t_nurcc_subdivded_cube();

    verify_tnurcc_control_points(&surface);
}

#[test]
fn t_nurcc_test_double_subdivide() {
    let mut surface = t_nurcc_subdivded_cube();
    surface
        .global_subdivide()
        .expect("Double subdivide should succeed");
    verify_tnurcc_control_points(&surface);
    verify_tnurcc_edges(&surface);
    verify_tnurcc_faces(&surface);
}

#[test]
fn t_nurcc_test_clone() {
    use std::mem::drop;
    let mut surface;
    {
        let clone = make_cube().unwrap();
        surface = clone.clone();
        drop(clone);
    }

    surface
        .global_subdivide()
        .expect("Cloned subdivide should succeed");

    surface
        .global_subdivide()
        .expect("Cloned double subdivide should succeed");

    verify_tnurcc_control_points(&surface);
    verify_tnurcc_edges(&surface);
    verify_tnurcc_faces(&surface);
}

#[test]
fn t_nurcc_test_from_quad_mesh_cube() {
    let points = vec![
        Point3::from((0.0, 0.0, 0.0)),
        Point3::from((0.0, 0.0, 1.0)),
        Point3::from((1.0, 0.0, 1.0)),
        Point3::from((1.0, 0.0, 0.0)),
        Point3::from((0.0, 1.0, 0.0)),
        Point3::from((0.0, 1.0, 1.0)),
        Point3::from((1.0, 1.0, 1.0)),
        Point3::from((1.0, 1.0, 0.0)),
    ];
    // Same winding as make_cube().
    let faces = [
        [0, 3, 2, 1],
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [4, 5, 6, 7],
        [2, 3, 7, 6],
        [0, 4, 7, 3],
    ];
    let tnurcc = Tnurcc::from_quad_mesh(points, &faces);
    assert!(
        tnurcc.is_ok(),
        "from_quad_mesh should succeed for a cube: {}",
        tnurcc.err().unwrap()
    );
    let tnurcc = tnurcc.unwrap();
    assert_eq!(tnurcc.control_points.len(), 8);
    assert_eq!(tnurcc.faces.len(), 6);
    assert_eq!(tnurcc.edges.len(), 12);

    verify_tnurcc_control_points(&tnurcc);
    verify_tnurcc_edges(&tnurcc);
    verify_tnurcc_faces(&tnurcc);
}

#[test]
fn t_nurcc_test_from_quad_mesh_open_mesh_rejected() {
    // An open mesh (not all edges shared by 2 faces) should be rejected.
    let points = vec![
        Point3::from((0.0, 0.0, 0.0)),
        Point3::from((1.0, 0.0, 0.0)),
        Point3::from((1.0, 1.0, 0.0)),
        Point3::from((0.0, 1.0, 0.0)),
    ];
    // Single quad face -- edges have only 1 face each.
    let faces = [[0, 1, 2, 3]];
    let result = Tnurcc::from_quad_mesh(points, &faces);
    assert!(
        result.is_err(),
        "Open mesh (single face) should be rejected"
    );
}

#[test]
fn t_nurcc_test_subdivision_point_count() {
    let mut tnurcc = make_cube().unwrap();
    // Before: V=8, E=12, F=6.
    tnurcc
        .global_subdivide()
        .expect("Subdivision should succeed");
    // After 1 CC level: V' = V + E + F = 8 + 12 + 6 = 26.
    assert_eq!(tnurcc.control_points.len(), 26);
    assert_eq!(tnurcc.edges.len(), 48);
    assert_eq!(tnurcc.faces.len(), 24);
}

#[test]
fn t_nurcc_test_to_tmesh_cube() {
    let tnurcc = make_cube().unwrap();
    let tmesh = tnurcc.to_tmesh(2);
    assert!(
        tmesh.is_ok(),
        "to_tmesh should succeed: {}",
        tmesh.err().unwrap()
    );
    let tmesh = tmesh.unwrap();

    // After 2 subdivisions: V = 98 total, all should be included.
    assert!(
        tmesh.control_points().len() > 20,
        "T-mesh should have many control points, got {}",
        tmesh.control_points().len()
    );

    // Verify subs() produces valid (non-NaN) points at several locations.
    for &(u, v) in &[(0.25, 0.25), (0.5, 0.5), (0.75, 0.75), (0.1, 0.9)] {
        let p: Point3 = tmesh
            .subs(u, v)
            .unwrap_or_else(|e| panic!("subs({}, {}) failed: {}", u, v, e));
        assert!(
            !p.x.is_nan() && !p.y.is_nan() && !p.z.is_nan(),
            "subs({}, {}) returned NaN: {:?}",
            u,
            v,
            p
        );
    }
}
