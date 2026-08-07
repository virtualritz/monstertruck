use itertools::Itertools;
use monstertruck_geometry::prelude::*;
use monstertruck_meshing::prelude::*;

use crate::types::*;

// `FilletError` is deliberately NOT imported here: the tests below name it as
// `super::FilletError`, and importing it would make that qualification
// redundant.
use super::{
    FilletOptions, RadiusSpec, build_5face_box, build_6face_box, build_box_shell, dump_shell_step,
    fillet_along_wire, fillet_edges, fillet_edges_generic, fillet_with_side,
};

#[test]
fn fillet_semi_cube() {
    let p = [
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(0.0, -0.1, 0.0),
        Point3::new(1.1, -0.1, 0.0),
        Point3::new(1.1, 1.1, 0.0),
        Point3::new(0.0, 1.1, 0.0),
    ];
    let v = Vertex::from_points(p);

    let line = |i: usize, j: usize| {
        let bsp = BsplineCurve::new(KnotVector::bezier_knot(1), vec![p[i], p[j]]);
        Edge::new(&v[i], &v[j], NurbsCurve::from(bsp).into())
    };
    let edge = [
        line(0, 1),
        line(1, 2),
        line(2, 3),
        line(3, 0),
        line(0, 4),
        line(1, 5),
        line(2, 6),
        line(3, 7),
        line(4, 5),
        line(5, 6),
        line(6, 7),
        line(7, 4),
    ];

    let plane = |i: usize, j: usize, k: usize, l: usize| {
        let control_points = vec![vec![p[i], p[l]], vec![p[j], p[k]]];
        let knot_vec = KnotVector::bezier_knot(1);
        let knot_vecs = (knot_vec.clone(), knot_vec);
        let bsp = BsplineSurface::new(knot_vecs, control_points);

        let wire: Wire = [i, j, k, l]
            .into_iter()
            .circular_tuple_windows()
            .map(|(i, j)| {
                edge.iter()
                    .find_map(|edge| {
                        if edge.front() == &v[i] && edge.back() == &v[j] {
                            Some(edge.clone())
                        } else if edge.back() == &v[i] && edge.front() == &v[j] {
                            Some(edge.inverse())
                        } else {
                            None
                        }
                    })
                    .unwrap()
            })
            .collect();
        Face::new(vec![wire], bsp.into())
    };
    let mut shell: Shell = [
        plane(0, 1, 2, 3),
        plane(1, 0, 4, 5),
        plane(2, 1, 5, 6),
        plane(3, 2, 6, 7),
    ]
    .into();

    let _poly = shell.robust_triangulation(0.001).to_polygon();
    dump_shell_step("semi-cube", &shell);

    let opts = FilletOptions {
        radius: RadiusSpec::Constant(0.4),
        ..Default::default()
    };
    let (face0, face1, face2, _, side1) = fillet_with_side(
        &shell[1],
        &shell[2],
        edge[5].id(),
        None,
        Some(&shell[0]),
        &opts,
    )
    .unwrap();
    (shell[1], shell[2], shell[0]) = (face0, face1, side1.unwrap());
    shell.push(face2);

    let (face0, face1, face2, _, side1) = fillet_with_side(
        &shell[2],
        &shell[3],
        edge[6].id(),
        None,
        Some(&shell[0]),
        &opts,
    )
    .unwrap();
    (shell[2], shell[3], shell[0]) = (face0, face1, side1.unwrap());
    shell.push(face2);

    let mut boundary = shell[0].boundaries().pop().unwrap();
    boundary.pop_back();
    assert_eq!(boundary.front_vertex().unwrap(), &v[0]);

    let _poly = shell.robust_triangulation(0.001).to_polygon();
    dump_shell_step("pre-fillet-cube", &shell);

    fillet_along_wire(
        &mut shell,
        &boundary,
        &FilletOptions {
            radius: RadiusSpec::Constant(0.2),
            ..Default::default()
        },
    )
    .unwrap();

    let _poly = shell.robust_triangulation(0.001).to_polygon();
    dump_shell_step("fillet-cube", &shell);
}

#[test]
fn fillet_closed_wire_box_top() {
    // Build a 5-face partial box (top + 4 sides), then fillet all 4 top edges
    // which form a closed square wire on the top face.
    let p = [
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let v = Vertex::from_points(p);

    let line = |i: usize, j: usize| {
        let bsp = BsplineCurve::new(KnotVector::bezier_knot(1), vec![p[i], p[j]]);
        Edge::new(&v[i], &v[j], NurbsCurve::from(bsp).into())
    };
    let edge = [
        line(0, 1), // 0: top front
        line(1, 2), // 1: top right
        line(2, 3), // 2: top back
        line(3, 0), // 3: top left
        line(0, 4), // 4
        line(1, 5), // 5
        line(2, 6), // 6
        line(3, 7), // 7
        line(4, 5), // 8
        line(5, 6), // 9
        line(6, 7), // 10
        line(7, 4), // 11
    ];

    let plane = |i: usize, j: usize, k: usize, l: usize| {
        let control_points = vec![vec![p[i], p[l]], vec![p[j], p[k]]];
        let knot_vec = KnotVector::bezier_knot(1);
        let knot_vecs = (knot_vec.clone(), knot_vec);
        let bsp = BsplineSurface::new(knot_vecs, control_points);

        let wire: Wire = [i, j, k, l]
            .into_iter()
            .circular_tuple_windows()
            .map(|(i, j)| {
                edge.iter()
                    .find_map(|edge| {
                        if edge.front() == &v[i] && edge.back() == &v[j] {
                            Some(edge.clone())
                        } else if edge.back() == &v[i] && edge.front() == &v[j] {
                            Some(edge.inverse())
                        } else {
                            None
                        }
                    })
                    .unwrap()
            })
            .collect();
        Face::new(vec![wire], bsp.into())
    };

    let mut shell: Shell = [
        plane(0, 1, 2, 3), // face 0: top
        plane(1, 0, 4, 5), // face 1: front
        plane(2, 1, 5, 6), // face 2: right
        plane(3, 2, 6, 7), // face 3: back
        plane(0, 3, 7, 4), // face 4: left
    ]
    .into();

    let initial_face_count = shell.len();

    // All 4 top edges form a closed wire on the top face.
    let closed_wire: Wire = [
        edge[0].clone(),
        edge[1].clone(),
        edge[2].clone(),
        edge[3].clone(),
    ]
    .into();
    assert!(closed_wire.is_closed());

    fillet_along_wire(
        &mut shell,
        &closed_wire,
        &FilletOptions {
            radius: RadiusSpec::Constant(0.2),
            ..Default::default()
        },
    )
    .unwrap();

    // 4 fillet faces should be added.
    assert_eq!(shell.len(), initial_face_count + 4);

    // The shell should still triangulate cleanly.
    let _poly = shell.robust_triangulation(0.001).to_polygon();
    dump_shell_step("fillet-closed-box-top", &shell);
}

#[test]
fn fillet_edges_single_edge() {
    let (mut shell, edge, _) = build_box_shell();
    let initial_face_count = shell.len();

    // Fillet edge[5] (shared by face 1: front and face 2: right),
    // same as the first fillet in fillet_semi_cube.
    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.4),
        ..Default::default()
    };
    fillet_edges(&mut shell, &[edge[5].id()], Some(&params)).unwrap();

    // fillet_with_side adds 1 fillet face.
    assert!(shell.len() > initial_face_count);

    // Verify the shell can still be triangulated.
    let _poly = shell.robust_triangulation(0.001).to_polygon();
    dump_shell_step("fillet-edges-single", &shell);
}

#[test]
fn fillet_edges_rejects_missing() {
    let (mut shell, _, v) = build_box_shell();

    // Create a bogus edge not in the shell.
    let bogus = {
        let bsp = BsplineCurve::new(
            KnotVector::bezier_knot(1),
            vec![
                Point3::new(99.0, 99.0, 99.0),
                Point3::new(100.0, 100.0, 100.0),
            ],
        );
        Edge::new(&v[0], &v[1], NurbsCurve::from(bsp).into())
    };

    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.3),
        ..Default::default()
    };
    let result = fillet_edges(&mut shell, &[bogus.id()], Some(&params));
    assert!(matches!(result, Err(super::FilletError::EdgeNotFound)));
}

#[test]
fn fillet_edges_rejects_boundary() {
    // Build a simple 2-face open shell where one edge is on the boundary.
    let p = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let v = Vertex::from_points(p);
    let line = |i: usize, j: usize| {
        let bsp = BsplineCurve::new(KnotVector::bezier_knot(1), vec![p[i], p[j]]);
        Edge::new(&v[i], &v[j], NurbsCurve::from(bsp).into())
    };

    let edge = [line(0, 1), line(1, 2), line(2, 3), line(3, 0)];

    let knot_vec = KnotVector::bezier_knot(1);
    let surface: NurbsSurface<_> = BsplineSurface::new(
        (knot_vec.clone(), knot_vec),
        vec![vec![p[0], p[3]], vec![p[1], p[2]]],
    )
    .into();

    let wire: Wire = [
        edge[0].clone(),
        edge[1].clone(),
        edge[2].clone(),
        edge[3].clone(),
    ]
    .into();
    let face = Face::new(vec![wire], surface);
    let mut shell: Shell = vec![face].into();

    // edge[0] is a boundary edge (shared by only 1 face).
    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.3),
        ..Default::default()
    };
    let result = fillet_edges(&mut shell, &[edge[0].id()], Some(&params));
    assert!(matches!(
        result,
        Err(super::FilletError::NonManifoldEdge(1))
    ));
}

// ---------------------------------------------------------------------------
// Generic fillet tests
// ---------------------------------------------------------------------------

/// Generic fillet with identity (internal) types -- verifies the pipeline works as passthrough.
#[test]
fn generic_fillet_identity() {
    let (mut shell, edge, _) = build_box_shell();
    let initial_face_count = shell.len();

    let target_edge = shell.edge_iter().find(|e| e.id() == edge[5].id()).unwrap();

    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.4),
        ..Default::default()
    };
    fillet_edges_generic(&mut shell, &[target_edge], Some(&params)).unwrap();

    assert!(shell.len() > initial_face_count);
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

/// Fillet two independent edges (different face pairs) in a single `fillet_edges` call.
#[test]
fn fillet_edges_multi_chain() {
    // 5-face box: top + 4 sides
    let p = [
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let v = Vertex::from_points(p);

    let line = |i: usize, j: usize| {
        let bsp = BsplineCurve::new(KnotVector::bezier_knot(1), vec![p[i], p[j]]);
        Edge::new(&v[i], &v[j], NurbsCurve::from(bsp).into())
    };
    let edge = [
        line(0, 1), // 0: top front
        line(1, 2), // 1: top right
        line(2, 3), // 2: top back
        line(3, 0), // 3: top left
        line(0, 4), // 4
        line(1, 5), // 5
        line(2, 6), // 6
        line(3, 7), // 7
        line(4, 5), // 8
        line(5, 6), // 9
        line(6, 7), // 10
        line(7, 4), // 11
    ];

    let plane = |i: usize, j: usize, k: usize, l: usize| {
        let control_points = vec![vec![p[i], p[l]], vec![p[j], p[k]]];
        let knot_vec = KnotVector::bezier_knot(1);
        let knot_vecs = (knot_vec.clone(), knot_vec);
        let bsp = BsplineSurface::new(knot_vecs, control_points);

        let wire: Wire = [i, j, k, l]
            .into_iter()
            .circular_tuple_windows()
            .map(|(i, j)| {
                edge.iter()
                    .find_map(|edge| {
                        if edge.front() == &v[i] && edge.back() == &v[j] {
                            Some(edge.clone())
                        } else if edge.back() == &v[i] && edge.front() == &v[j] {
                            Some(edge.inverse())
                        } else {
                            None
                        }
                    })
                    .unwrap()
            })
            .collect();
        Face::new(vec![wire], bsp.into())
    };

    let mut shell: Shell = [
        plane(0, 1, 2, 3), // face 0: top
        plane(1, 0, 4, 5), // face 1: front
        plane(2, 1, 5, 6), // face 2: right
        plane(3, 2, 6, 7), // face 3: back
        plane(0, 3, 7, 4), // face 4: left
    ]
    .into();

    let initial_face_count = shell.len();

    // Fillet two independent edges belonging to different face pairs:
    // edge[5] (front-right) and edge[7] (top-left / back-left).
    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.3),
        ..Default::default()
    };
    fillet_edges(&mut shell, &[edge[5].id(), edge[7].id()], Some(&params)).unwrap();

    // Both fillets should add faces.
    assert!(
        shell.len() >= initial_face_count + 2,
        "expected at least 2 new fillet faces, got {} total (was {})",
        shell.len(),
        initial_face_count
    );

    // The shell should triangulate cleanly.
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

/// Edge too short for requested fillet radius → DegenerateEdge error.
#[test]
fn fillet_rejects_degenerate_edge() {
    let (mut shell, edge, _) = build_box_shell();

    // The box edges are length 1.0. Request a radius of 0.6 → 2*0.6 = 1.2 > 1.0.
    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.6),
        ..Default::default()
    };
    let result = fillet_edges(&mut shell, &[edge[5].id()], Some(&params));
    assert!(
        matches!(result, Err(super::FilletError::DegenerateEdge)),
        "expected DegenerateEdge, got: {result:?}"
    );
}

// ---------------------------------------------------------------------------
// Phase 6a: Identity-based edge replacement
// ---------------------------------------------------------------------------

/// Verify `cut_face_by_bezier` works on a 5-edge boundary (pentagon).
#[test]
fn cut_face_five_edge_boundary() {
    use super::topology::cut_face_by_bezier;

    // Build a planar pentagon: vertices at unit-circle positions.
    let pts: Vec<Point3> = (0..5)
        .map(|i| {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / 5.0;
            Point3::new(angle.cos(), angle.sin(), 0.0)
        })
        .collect();
    let v = Vertex::from_points(&pts);

    let line_edge = |i: usize, j: usize| {
        let bsp = BsplineCurve::new(KnotVector::bezier_knot(1), vec![pts[i], pts[j]]);
        Edge::new(&v[i], &v[j], NurbsCurve::from(bsp).into())
    };

    // 5 edges: e0(0→1), e1(1→2), e2(2→3), e3(3→4), e4(4→0)
    let edges = [
        line_edge(0, 1),
        line_edge(1, 2),
        line_edge(2, 3),
        line_edge(3, 4),
        line_edge(4, 0),
    ];

    let wire: Wire = edges.iter().cloned().collect();

    // Simple planar surface covering the pentagon area.
    let surface: NurbsSurface<_> = BsplineSurface::new(
        (KnotVector::bezier_knot(1), KnotVector::bezier_knot(1)),
        vec![
            vec![Point3::new(-1.5, -1.5, 0.0), Point3::new(-1.5, 1.5, 0.0)],
            vec![Point3::new(1.5, -1.5, 0.0), Point3::new(1.5, 1.5, 0.0)],
        ],
    )
    .into();

    let face = Face::new(vec![wire], surface);

    // Pick edge[2] (2→3) as the filleted edge.
    // Adjacent edges: front=edge[1] (1→2), back=edge[3] (3→4).
    // Build a bezier that starts near the midpoint of edge[1] and ends near
    // the midpoint of edge[3], crossing through the filleted edge region.
    let mid1 = (pts[1] + pts[2].to_vec()) / 2.0;
    let mid3 = (pts[3] + pts[4].to_vec()) / 2.0;
    let mid_control = (mid1 + mid3.to_vec()) / 2.0;
    let bezier: NurbsCurve<Vector4> = NurbsCurve::from(BsplineCurve::new(
        KnotVector::bezier_knot(2),
        vec![mid1, mid_control, mid3],
    ));

    let result = cut_face_by_bezier(&face, bezier, edges[2].id());
    assert!(result.is_some(), "cut_face_by_bezier returned None");

    let (new_face, fillet_edge) = result.unwrap();
    // Should still have 5 edges (3 original untouched + new_front + fillet + new_back,
    // replacing front + filleted + back = same count).
    let boundary = &new_face.absolute_boundaries()[0];
    assert_eq!(
        boundary.len(),
        5,
        "expected 5 edges after cut, got {}",
        boundary.len()
    );
    // The fillet edge should appear in the boundary.
    assert!(
        boundary.iter().any(|e| e.id() == fillet_edge.id()),
        "fillet edge not found in new boundary"
    );
}

// ---------------------------------------------------------------------------
// Phase 7: Boundary-run chain grouping tests
// ---------------------------------------------------------------------------

/// Fillet all 4 top edges of a cuboid in a single `fillet_edges` call.
///
/// Previously this produced 4 singleton chains (different face pairs) processed
/// sequentially, causing `EdgeNotFound` as earlier fillets invalidated adjacent
/// edge IDs. With boundary-run grouping, the 4 top edges form one closed chain
/// on the top face, processed in a single `fillet_along_wire_closed` call.
#[test]
fn fillet_edges_cuboid_top_4() {
    let (mut shell, edge, _v) = build_5face_box();
    let top_ids: Vec<EdgeId> = (0..4).map(|i| edge[i].id()).collect();
    let opts = FilletOptions {
        radius: RadiusSpec::Constant(0.2),
        ..Default::default()
    };
    fillet_edges(&mut shell, &top_ids, Some(&opts)).unwrap();
    // 4 fillet faces added (one per edge in the closed wire).
    assert!(shell.len() >= 9, "expected >= 9 faces, got {}", shell.len());
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

/// Fillet top 4 + bottom 4 edges (two independent closed chains).
#[test]
fn fillet_edges_cuboid_top_and_bottom() {
    let (mut shell, edge, _v) = build_6face_box();
    let ids: Vec<EdgeId> = [0, 1, 2, 3, 8, 9, 10, 11]
        .iter()
        .map(|&i| edge[i].id())
        .collect();
    let opts = FilletOptions {
        radius: RadiusSpec::Constant(0.15),
        ..Default::default()
    };
    fillet_edges(&mut shell, &ids, Some(&opts)).unwrap();
    // 8 fillet faces added (4 per closed chain).
    assert!(
        shell.len() >= 14,
        "expected >= 14 faces, got {}",
        shell.len()
    );
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}
