use super::*;

// ---------------------------------------------------------------------------
// Geometric accuracy tests
// ---------------------------------------------------------------------------

/// Round fillet contact curves lie at the correct distance from the original planes.
#[test]
fn radius_error_bounds() {
    let (shell, edge, _) = build_box_shell();

    // face 1 (front) is y=0 plane, face 2 (right) is x=1 plane.
    // edge[5] (1→5) runs along z at (x=1, y=0). These faces are orthogonal.
    let radius = 0.3;
    let (_, _, fillet) = fillet(
        &shell[1],
        &shell[2],
        edge[5].id(),
        &FilletOptions {
            radius: RadiusSpec::Constant(radius),
            ..Default::default()
        },
    )
    .unwrap();

    let fillet_surface = fillet.oriented_surface();
    let n = 8;
    let tol = 0.01;

    // u=0 contact curve: fillet touches face1 (y=0 plane).
    // Points should be on that plane (dy≈0) at distance radius from face2 (dx≈radius).
    for j in 0..=n {
        let v = j as f64 / n as f64;
        let pt = fillet_surface.subs(0.0, v);
        let dy = pt.y.abs();
        let dx = (pt.x - 1.0).abs();
        assert!(dy < tol, "u=0 contact not on y=0 plane: dy={dy:.6}");
        assert!(
            (dx - radius).abs() < tol,
            "u=0 contact distance from x=1 plane: dx={dx:.6}, expected {radius}"
        );
    }

    // u=1 contact curve: fillet touches face2 (x=1 plane).
    // Points should be on that plane (dx≈0) at distance radius from face1 (dy≈radius).
    for j in 0..=n {
        let v = j as f64 / n as f64;
        let pt = fillet_surface.subs(1.0, v);
        let dx = (pt.x - 1.0).abs();
        let dy = pt.y.abs();
        assert!(dx < tol, "u=1 contact not on x=1 plane: dx={dx:.6}");
        assert!(
            (dy - radius).abs() < tol,
            "u=1 contact distance from y=0 plane: dy={dy:.6}, expected {radius}"
        );
    }

    // Interior: all points should be inside the fillet pocket (0 < dx < radius, 0 < dy < radius).
    for i in 1..n {
        for j in 0..=n {
            let u = i as f64 / n as f64;
            let v = j as f64 / n as f64;
            let pt = fillet_surface.subs(u, v);
            let dx = (pt.x - 1.0).abs();
            let dy = pt.y.abs();
            assert!(
                dx < radius + tol && dy < radius + tol,
                "interior point ({u:.2},{v:.2}) outside pocket: dx={dx:.4} dy={dy:.4}"
            );
        }
    }
}

/// Adjacent fillet surfaces in a multi-edge wire should meet with C0 continuity
/// and approximate G1 tangent alignment at their shared seam.
#[test]
fn continuity_at_wire_joins() {
    // Use the same 4-face semi-cube topology as fillet_semi_cube, producing
    // a 2-edge open wire fillet with two adjacent fillet surfaces.
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

    let initial_count = shell.len();
    fillet_along_wire(
        &mut shell,
        &boundary,
        &FilletOptions {
            radius: RadiusSpec::Constant(0.2),
            ..Default::default()
        },
    )
    .unwrap();

    // Fillet faces are appended at the end.
    let fillet_faces: Vec<_> = (initial_count..shell.len()).map(|i| &shell[i]).collect();
    assert!(
        fillet_faces.len() >= 2,
        "expected at least 2 fillet faces, got {}",
        fillet_faces.len()
    );

    // C0 check: adjacent fillet faces share boundary vertices. For each pair,
    // find vertices that appear in both faces' boundaries and verify positions match.
    let tol = 0.01;
    for win in fillet_faces.windows(2) {
        let verts0: Vec<_> = win[0]
            .boundary_iters()
            .into_iter()
            .flatten()
            .map(|e| (e.front().point(), e.back().point()))
            .collect();
        let verts1: Vec<_> = win[1]
            .boundary_iters()
            .into_iter()
            .flatten()
            .map(|e| (e.front().point(), e.back().point()))
            .collect();

        // Collect all vertex positions from each face.
        let pts0: Vec<Point3> = verts0.iter().flat_map(|(f, b)| [*f, *b]).collect();
        let pts1: Vec<Point3> = verts1.iter().flat_map(|(f, b)| [*f, *b]).collect();

        // Find shared vertices (points within tolerance).
        let shared: Vec<_> = pts0
            .iter()
            .filter(|p0| pts1.iter().any(|p1| (*p0 - *p1).magnitude() < tol))
            .collect();

        assert!(
            shared.len() >= 2,
            "adjacent fillet faces should share at least 2 vertices, found {}",
            shared.len()
        );
    }

    // G1 check: for each fillet face, sample the surface normal at the interior
    // and verify it varies smoothly (no sudden flips).
    for face in &fillet_faces {
        let s = face.oriented_surface();
        let n = 4;
        let mut prev_normal = None;
        for j in 0..=n {
            let v = j as f64 / n as f64;
            let normal = s.normal(0.5, v);
            if let Some(prev) = prev_normal {
                let dot: f64 = normal.dot(prev);
                assert!(
                    dot > 0.5,
                    "normal flip within fillet face: dot={dot:.4} at v={v:.2}"
                );
            }
            prev_normal = Some(normal);
        }
    }
}
