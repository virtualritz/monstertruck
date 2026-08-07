use super::*;

// ---------------------------------------------------------------------------
// Chamfer tests
// ---------------------------------------------------------------------------

/// Single-edge chamfer on a 2-face shell.
#[test]
fn chamfer_single_edge() {
    let (mut shell, edge, _) = build_box_shell();
    let initial_face_count = shell.len();

    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.4),
        profile: FilletProfile::Chamfer,
        ..Default::default()
    };
    fillet_edges(&mut shell, &[edge[5].id()], Some(&params)).unwrap();

    assert!(shell.len() > initial_face_count);
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

/// Chamfer along an open wire (same topology as fillet_semi_cube).
#[test]
fn chamfer_semi_cube() {
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

    let chamfer_opts = FilletOptions {
        radius: RadiusSpec::Constant(0.4),
        profile: FilletProfile::Chamfer,
        ..Default::default()
    };
    let (face0, face1, face2, _, side1) = fillet_with_side(
        &shell[1],
        &shell[2],
        edge[5].id(),
        None,
        Some(&shell[0]),
        &chamfer_opts,
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
        &chamfer_opts,
    )
    .unwrap();
    (shell[2], shell[3], shell[0]) = (face0, face1, side1.unwrap());
    shell.push(face2);

    let mut boundary = shell[0].boundaries().pop().unwrap();
    boundary.pop_back();

    fillet_along_wire(
        &mut shell,
        &boundary,
        &FilletOptions {
            radius: RadiusSpec::Constant(0.2),
            profile: FilletProfile::Chamfer,
            ..Default::default()
        },
    )
    .unwrap();

    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

/// Chamfer along a closed wire (same topology as fillet_closed_wire_box_top).
#[test]
fn chamfer_closed_wire() {
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

    fillet_along_wire(
        &mut shell,
        &closed_wire,
        &FilletOptions {
            radius: RadiusSpec::Constant(0.2),
            profile: FilletProfile::Chamfer,
            ..Default::default()
        },
    )
    .unwrap();

    assert_eq!(shell.len(), initial_face_count + 4);
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

// ---------------------------------------------------------------------------
// Ridge tests
// ---------------------------------------------------------------------------

/// Single-edge ridge on a 2-face shell.
#[test]
fn ridge_single_edge() {
    let (mut shell, edge, _) = build_box_shell();
    let initial_face_count = shell.len();

    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.4),
        profile: FilletProfile::Ridge,
        ..Default::default()
    };
    fillet_edges(&mut shell, &[edge[5].id()], Some(&params)).unwrap();

    assert!(shell.len() > initial_face_count);
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

/// Ridge along an open wire (same topology as chamfer_semi_cube).
#[test]
fn ridge_semi_cube() {
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

    let ridge_opts = FilletOptions {
        radius: RadiusSpec::Constant(0.4),
        profile: FilletProfile::Ridge,
        ..Default::default()
    };
    let (face0, face1, face2, _, side1) = fillet_with_side(
        &shell[1],
        &shell[2],
        edge[5].id(),
        None,
        Some(&shell[0]),
        &ridge_opts,
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
        &ridge_opts,
    )
    .unwrap();
    (shell[2], shell[3], shell[0]) = (face0, face1, side1.unwrap());
    shell.push(face2);

    let mut boundary = shell[0].boundaries().pop().unwrap();
    boundary.pop_back();

    fillet_along_wire(
        &mut shell,
        &boundary,
        &FilletOptions {
            radius: RadiusSpec::Constant(0.2),
            profile: FilletProfile::Ridge,
            ..Default::default()
        },
    )
    .unwrap();

    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

/// Ridge along a closed wire (same topology as chamfer_closed_wire).
#[test]
fn ridge_closed_wire() {
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

    fillet_along_wire(
        &mut shell,
        &closed_wire,
        &FilletOptions {
            radius: RadiusSpec::Constant(0.2),
            profile: FilletProfile::Ridge,
            ..Default::default()
        },
    )
    .unwrap();

    assert_eq!(shell.len(), initial_face_count + 4);
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

// ---------------------------------------------------------------------------
// Custom profile tests
// ---------------------------------------------------------------------------

/// Custom with linear profile (0,0)→(1,0) -- should behave like chamfer.
#[test]
fn custom_profile_linear() {
    let (mut shell, edge, _) = build_box_shell();
    let initial_face_count = shell.len();

    let profile = BsplineCurve::new(
        KnotVector::bezier_knot(1),
        vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
    );
    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.4),
        profile: FilletProfile::Custom(Box::new(profile)),
        ..Default::default()
    };
    fillet_edges(&mut shell, &[edge[5].id()], Some(&params)).unwrap();

    assert!(shell.len() > initial_face_count);
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}

/// Custom with degree-2 bump (0,0)→(0.5,1.0)→(1,0) -- non-trivial shape.
#[test]
fn custom_profile_bump() {
    let (mut shell, edge, _) = build_box_shell();
    let initial_face_count = shell.len();

    let profile = BsplineCurve::new(
        KnotVector::bezier_knot(2),
        vec![
            Point2::new(0.0, 0.0),
            Point2::new(0.5, 1.0),
            Point2::new(1.0, 0.0),
        ],
    );
    let params = FilletOptions {
        radius: RadiusSpec::Constant(0.4),
        profile: FilletProfile::Custom(Box::new(profile)),
        ..Default::default()
    };
    fillet_edges(&mut shell, &[edge[5].id()], Some(&params)).unwrap();

    assert!(shell.len() > initial_face_count);
    let _poly = shell.robust_triangulation(0.001).to_polygon();
}
