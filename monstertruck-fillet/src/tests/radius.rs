use itertools::Itertools;
use monstertruck_geometry::prelude::*;
use monstertruck_meshing::prelude::*;

use crate::types::*;

// `FilletError` is deliberately NOT imported here: the tests below name it as
// `super::FilletError`, and importing it would make that qualification
// redundant.
use super::{
    FilletOptions, RadiusSpec, build_box_shell, fillet_along_wire, fillet_edges, fillet_with_side,
};

/// Variable-radius fillet along a closed wire (radius varies 0.15..0.20, f(0) ≈ f(1)).
#[test]
fn variable_radius_closed_wire() {
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
        let bsp = BsplineSurface::new((knot_vec.clone(), knot_vec), control_points);
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
        plane(0, 3, 7, 4),
    ]
    .into();

    let initial_face_count = shell.len();

    let closed_wire: Wire = [
        edge[0].clone(),
        edge[1].clone(),
        edge[2].clone(),
        edge[3].clone(),
    ]
    .into();
    assert!(closed_wire.is_closed());

    // Variable radius: 0.15 at endpoints, peaks at ~0.20 at t=0.5.
    // f(0) ≈ f(1) ≈ 0.15, satisfying the closed-wire constraint.
    let opts = FilletOptions {
        radius: RadiusSpec::Variable(Box::new(|t| 0.15 + 0.05 * (std::f64::consts::PI * t).sin())),
        ..Default::default()
    };
    fillet_along_wire(&mut shell, &closed_wire, &opts).unwrap();

    assert_eq!(shell.len(), initial_face_count + 4);
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

// ---------------------------------------------------------------------------
// Phase 6c: Variable radius on open wires
// ---------------------------------------------------------------------------

/// Variable radius on an open wire should succeed (no f(0)≈f(1) constraint).
#[test]
fn variable_radius_open_wire() {
    // Reuse fillet_semi_cube topology: 4-face open box, 2-edge open wire.
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
        let bsp = BsplineSurface::new((knot_vec.clone(), knot_vec), control_points);
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
    assert!(!boundary.is_closed());

    // Variable radius where f(0)=0.1, f(1)=0.3 -- NOT equal, would fail on closed wire.
    let var_opts = FilletOptions {
        radius: RadiusSpec::Variable(Box::new(|t| 0.1 + 0.2 * t)),
        ..Default::default()
    };
    fillet_along_wire(&mut shell, &boundary, &var_opts).unwrap();

    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

// ---------------------------------------------------------------------------
// Phase 6b: Per-edge radius
// ---------------------------------------------------------------------------

/// Per-edge radius with two edges having different radii.
#[test]
fn per_edge_radius_two_edges() {
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
        let bsp = BsplineSurface::new((knot_vec.clone(), knot_vec), control_points);
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
        plane(0, 3, 7, 4),
    ]
    .into();

    let initial_face_count = shell.len();

    // Two independent edges with different radii.
    let params = FilletOptions {
        radius: RadiusSpec::PerEdge(vec![0.3, 0.15]),
        ..Default::default()
    };
    fillet_edges(&mut shell, &[edge[5].id(), edge[7].id()], Some(&params)).unwrap();

    assert!(
        shell.len() >= initial_face_count + 2,
        "expected at least 2 new fillet faces, got {} total (was {})",
        shell.len(),
        initial_face_count
    );
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

/// Per-edge radius count mismatch → PerEdgeRadiusMismatch error.
#[test]
fn per_edge_radius_mismatch() {
    let (mut shell, edge, _) = build_box_shell();

    // Provide 1 radius for 2 edges → mismatch.
    let params = FilletOptions {
        radius: RadiusSpec::PerEdge(vec![0.3]),
        ..Default::default()
    };
    let result = fillet_edges(&mut shell, &[edge[5].id(), edge[6].id()], Some(&params));
    assert!(
        matches!(
            result,
            Err(super::FilletError::PerEdgeRadiusMismatch {
                given: 1,
                expected: 2
            })
        ),
        "expected PerEdgeRadiusMismatch, got: {result:?}"
    );
}

/// Per-edge radius where one edge is too short → DegenerateEdge.
#[test]
fn per_edge_radius_degenerate() {
    let (mut shell, edge, _) = build_box_shell();

    // edge[5] length ~1.0, radius 0.15 → ok (2*0.15=0.3 < 1.0).
    // edge[6] length ~1.0, radius 0.6 → too big (2*0.6=1.2 > 1.0).
    let params = FilletOptions {
        radius: RadiusSpec::PerEdge(vec![0.15, 0.6]),
        ..Default::default()
    };
    let result = fillet_edges(&mut shell, &[edge[5].id(), edge[6].id()], Some(&params));
    assert!(
        matches!(result, Err(super::FilletError::DegenerateEdge)),
        "expected DegenerateEdge, got: {result:?}"
    );
}
