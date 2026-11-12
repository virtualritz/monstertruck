use super::*;
use monstertruck_modeling::{Curve, Surface, builder};
use monstertruck_topology::{Shell, StableId, shell::ShellCondition};

type ModelShell = CompressedShell<Point3, Curve, Surface>;

/// Clamped degree-1 B-spline from `p` to `q`, the common form of a straight
/// edge in an imported patch network.
fn segment(p: Point3, q: Point3) -> Curve {
    Curve::BsplineCurve(BsplineCurve::new(KnotVector::bezier_knot(1), vec![p, q]))
}

/// Clamped cubic B-spline from `p` to `q` whose interior control points bulge
/// by `bulge` in `+x`. It stays in the `z = 0` plane.
fn bulged(p: Point3, q: Point3, bulge: f64) -> Curve {
    let at = |t: f64| p + (q - p) * t + Vector3::new(bulge, 0.0, 0.0);
    Curve::BsplineCurve(BsplineCurve::new(
        KnotVector::bezier_knot(3),
        vec![p, at(1.0 / 3.0), at(2.0 / 3.0), q],
    ))
}

fn plane() -> Surface {
    Surface::Plane(Plane::new(
        Point3::origin(),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ))
}

fn p(x: f64, y: f64) -> Point3 { Point3::new(x, y, 0.0) }

/// A shell of planar faces from vertices, edges as `(front, back, curve)` and
/// one boundary wire per face as `(edge, orientation)` uses.
fn shell_of(
    vertices: Vec<Point3>,
    edges: Vec<(usize, usize, Curve)>,
    wires: &[&[(usize, bool)]],
) -> ModelShell {
    CompressedShell {
        vertices,
        edges: edges
            .into_iter()
            .map(|(front, back, curve)| CompressedEdge {
                vertices: (front, back),
                curve,
            })
            .collect(),
        faces: wires
            .iter()
            .map(|wire| CompressedFace {
                boundaries: vec![
                    wire.iter()
                        .copied()
                        .map(CompressedEdgeIndex::from)
                        .collect(),
                ],
                orientation: true,
                surface: plane(),
            })
            .collect(),
        vertex_stable_ids: None,
        edge_stable_ids: None,
        face_stable_ids: None,
    }
}

fn segments(vertices: &[Point3], pairs: &[(usize, usize)]) -> Vec<(usize, usize, Curve)> {
    pairs
        .iter()
        .map(|&(a, b)| (a, b, segment(vertices[a], vertices[b])))
        .collect()
}

/// Two unit quads side by side. Quad A spans `x` in `[0, 1]`, quad B spans
/// `[1 + gap, 2]`. A's right edge (edge 1) and B's left edge (edge 7) are the
/// seam: both bulge in `+x`, by `bulge_a` and `bulge_b`. B's vertices come
/// after A's, so B's seam corners are the ones that move.
fn two_quads(gap: f64, bulge_a: f64, bulge_b: f64) -> ModelShell {
    let x = 1.0 + gap;
    let v = vec![
        p(0.0, 0.0),
        p(1.0, 0.0),
        p(1.0, 1.0),
        p(0.0, 1.0),
        p(x, 0.0),
        p(2.0, 0.0),
        p(2.0, 1.0),
        p(x, 1.0),
    ];
    let mut edges = segments(&v, &[(0, 1)]);
    edges.push((1, 2, bulged(v[1], v[2], bulge_a)));
    edges.extend(segments(&v, &[(2, 3), (3, 0), (4, 5), (5, 6), (6, 7)]));
    edges.push((7, 4, bulged(v[7], v[4], bulge_b)));
    shell_of(
        v,
        edges,
        &[
            &[(0, true), (1, true), (2, true), (3, true)],
            &[(4, true), (5, true), (6, true), (7, true)],
        ],
    )
}

fn seam(gap: f64) -> ModelShell { two_quads(gap, 0.1, 0.1) }

fn naked_count(shell: &ModelShell) -> usize {
    let mut counts = vec![0usize; shell.edges.len()];
    shell
        .faces
        .iter()
        .flat_map(|face| face.boundaries.iter().flatten())
        .for_each(|edge_use| counts[edge_use.index] += 1);
    counts.into_iter().filter(|count| *count == 1).count()
}

fn control_point_bits(shell: &ModelShell) -> Vec<[u64; 3]> {
    shell
        .edges
        .iter()
        .flat_map(|edge| match &edge.curve {
            Curve::BsplineCurve(curve) => curve.control_points().clone(),
            other => panic!("fixture edges are B-splines, got {other:?}"),
        })
        .map(|point| [point.x.to_bits(), point.y.to_bits(), point.z.to_bits()])
        .collect()
}

/// The largest distance from a sample of a shared edge's curve to the plane of
/// each face using it. Every fixture face lies in `z = 0`, so the residual is
/// `|z|`.
fn max_curve_to_surface_residual(shell: &ModelShell) -> f64 {
    let mut faces_of = vec![Vec::new(); shell.edges.len()];
    shell.faces.iter().enumerate().for_each(|(face, f)| {
        f.boundaries
            .iter()
            .flatten()
            .for_each(|edge_use| faces_of[edge_use.index].push(face));
    });
    shell
        .edges
        .iter()
        .zip(&faces_of)
        .filter(|(_, faces)| faces.len() == 2)
        .flat_map(|(edge, _)| {
            let (t0, t1) = edge.curve.range_tuple();
            (0..=64).map(move |i| edge.curve.subs(t0 + (t1 - t0) * i as f64 / 64.0).z.abs())
        })
        .fold(0.0, f64::max)
}

fn measure(label: &str, before: &ModelShell, after: &ModelShell, report: &SnapReport) {
    println!(
        "{label}: vertices {} -> {}, edges {} -> {}, faces {} -> {}, naked {} -> {} {:?}, \
         max vertex displacement {:e}, max curve deviation {:e}, max curve endpoint gap {:e}, \
         max curve-to-surface residual {:e}",
        before.vertices.len(),
        after.vertices.len(),
        before.edges.len(),
        after.edges.len(),
        before.faces.len(),
        after.faces.len(),
        report.naked_edges_before,
        report.naked_edges_after,
        report.naked_edges,
        report.max_vertex_displacement,
        report.max_curve_deviation,
        report.max_curve_endpoint_gap,
        max_curve_to_surface_residual(after),
    );
}

/// Bitwise equality: `Surface` has no `PartialEq`, and `Debug` prints every
/// `f64` in its shortest round-trip form, so equal text means equal bits.
fn assert_unchanged(shell: &ModelShell, before: &ModelShell) {
    assert_eq!(format!("{shell:?}"), format!("{before:?}"));
}

#[test]
fn seam_4e3_apart_welds_at_1e2() {
    let gap = 4.0e-3;
    let before = seam(gap);
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("compatible seam welds");
    measure("seam 4e-3, tol 1e-2", &before, &shell, &report);

    assert!(report.changed());
    assert_eq!(report.merged_vertices, 2);
    assert_eq!(report.merged_edges, 1);
    assert_eq!(
        (report.naked_edges_before, report.naked_edges_after),
        (8, 6)
    );
    assert_eq!(naked_count(&shell), 6);
    assert_eq!(
        (shell.vertices.len(), shell.edges.len(), shell.faces.len()),
        (6, 7, 2)
    );
    assert!((report.max_vertex_displacement - gap).abs() < 1.0e-12);
    assert!(report.max_curve_deviation > 0.0 && report.max_curve_deviation <= 0.01);
    assert!(report.max_curve_endpoint_gap <= 0.01);
    assert_eq!(max_curve_to_surface_residual(&shell), 0.0);

    // A's vertices stay; B's seam corners 4 and 7 merged into 1 and 2.
    assert_eq!(&shell.vertices[..4], &before.vertices[..4]);
    assert_eq!(report.vertex_map, vec![0, 1, 2, 3, 1, 4, 5, 2]);
    // B's seam edge 7 now rides on A's edge 1, in the opposite direction.
    assert_eq!(report.edge_map[7], CompressedEdgeIndex::from((1, false)));
    assert_eq!(report.edge_map[1], CompressedEdgeIndex::from((1, true)));

    let extracted = Shell::extract(shell).expect("welded shell extracts");
    assert_eq!(extracted.shell_condition(), ShellCondition::Oriented);
    assert_eq!(extracted.len(), 2);
}

#[test]
fn seam_above_tolerance_stays_naked() {
    let before = seam(4.0e-3);
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 3.0e-3).expect("nothing to refuse");
    measure("seam 4e-3, tol 3e-3", &before, &shell, &report);
    assert!(!report.changed());
    assert_eq!(report.naked_edges, (0..8).collect::<Vec<_>>());
    assert_unchanged(&shell, &before);
}

#[test]
fn zero_tolerance_is_an_exact_noop() {
    // Even coincident vertices and identical curves do not weld at zero.
    let before = two_quads(0.0, 0.1, 0.1);
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.0).expect("zero is valid");
    assert!(!report.changed());
    assert_eq!(report.naked_edges_after, 8);
    assert_unchanged(&shell, &before);
}

#[test]
fn invalid_tolerance_is_refused() {
    let before = seam(4.0e-3);
    [-1.0e-3, f64::NAN, f64::INFINITY, f64::NEG_INFINITY]
        .into_iter()
        .for_each(|tolerance| {
            let mut shell = before.clone();
            let result = snap_naked_edges(&mut shell, tolerance);
            assert!(
                matches!(result, Err(SnapError::InvalidTolerance(_))),
                "tolerance {tolerance}: {result:?}"
            );
            assert_unchanged(&shell, &before);
        });
}

#[test]
fn closed_shell_is_unchanged() {
    let v = builder::vertex(Point3::origin());
    let e = builder::extrude(&v, Vector3::unit_x());
    let f = builder::extrude(&e, Vector3::unit_y());
    let cube = builder::extrude(&f, Vector3::unit_z());
    let before = cube.boundaries()[0].compress();
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("closed cube");
    assert!(!report.changed());
    assert_eq!(
        (report.naked_edges_before, report.naked_edges_after),
        (0, 0)
    );
    assert_unchanged(&shell, &before);
}

#[test]
fn weld_is_deterministic_and_idempotent() {
    let runs: Vec<_> = (0..3)
        .map(|_| {
            let mut shell = seam(4.0e-3);
            let report = snap_naked_edges(&mut shell, 0.01).unwrap();
            (
                control_point_bits(&shell),
                format!("{shell:?}"),
                report,
                shell,
            )
        })
        .collect();
    runs.windows(2).for_each(|pair| {
        assert_eq!(pair[0].0, pair[1].0, "control points differ between runs");
        assert_eq!(pair[0].1, pair[1].1, "shells differ between runs");
        assert_eq!(pair[0].2, pair[1].2, "reports differ between runs");
    });

    let welded = runs[0].3.clone();
    let mut again = welded.clone();
    let report = snap_naked_edges(&mut again, 0.01).expect("second pass");
    assert!(!report.changed());
    assert_eq!(report.naked_edges_after, 6);
    assert_unchanged(&again, &welded);
}

#[test]
fn stable_ids_surfaces_and_orientation_survive() {
    // Quad B is flagged reversed with its wire listed backwards, so it still
    // traverses the seam opposite to A.
    let mut before = seam(4.0e-3);
    before.faces[1].orientation = false;
    before.faces[1].boundaries = vec![
        [(7, false), (6, false), (5, false), (4, false)]
            .map(CompressedEdgeIndex::from)
            .to_vec(),
    ];
    before.vertex_stable_ids = Some((1..=8).map(StableId::new).collect());
    before.edge_stable_ids = Some((101..=108).map(StableId::new).collect());
    before.face_stable_ids = Some(vec![StableId::new(201), StableId::new(202)]);
    let extracted_before = Shell::extract(before.clone()).expect("fixture extracts");

    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("compatible seam welds");
    measure("attributes, tol 1e-2", &before, &shell, &report);
    assert_eq!((report.merged_vertices, report.merged_edges), (2, 1));

    shell
        .faces
        .iter()
        .zip(&before.faces)
        .for_each(|(after, before)| {
            assert_eq!(after.orientation, before.orientation);
            assert_eq!(
                format!("{:?}", after.surface),
                format!("{:?}", before.surface)
            );
        });
    assert_eq!(shell.face_stable_ids, before.face_stable_ids);
    let raw = |ids: &Option<Vec<StableId>>| -> Vec<u64> {
        ids.as_ref().unwrap().iter().map(|id| id.raw()).collect()
    };
    // Survivors keep their own IDs; B's merged corners (5, 8) and seam edge
    // (108) drop theirs.
    assert_eq!(raw(&shell.vertex_stable_ids), vec![1, 2, 3, 4, 6, 7]);
    assert_eq!(
        raw(&shell.edge_stable_ids),
        vec![101, 102, 103, 104, 105, 106, 107]
    );

    let extracted = Shell::extract(shell).expect("welded shell extracts");
    assert_eq!(extracted.shell_condition(), ShellCondition::Oriented);
    extracted
        .iter()
        .zip(extracted_before.iter())
        .for_each(|(after, before)| assert_eq!(after.orientation(), before.orientation()));
}

/// Three faces in a fan: T1-T2 and T2-T3 each match, with the shared corners
/// `c1`, `c2`, `c3` 0.006 apart in a row. Welding both would put `c1` and `c3`
/// in one vertex 0.012 apart, so the second weld stays naked.
#[test]
fn no_transitive_tolerance_creep() {
    let v = vec![
        p(0.0, 0.0),
        p(0.0, 1.0),
        p(-1.0, 0.5),
        p(0.006, 0.0),
        p(0.006, -1.0),
        p(1.0, 0.0),
        p(0.006, 1.0),
        p(0.012, 0.0),
        p(0.012, -1.0),
        p(-1.0, -0.5),
    ];
    let edges = segments(
        &v,
        &[
            (0, 1),
            (1, 2),
            (2, 0),
            (3, 4),
            (4, 5),
            (5, 6),
            (6, 3),
            (8, 7),
            (7, 9),
            (9, 8),
        ],
    );
    let before = shell_of(
        v,
        edges,
        &[
            &[(0, true), (1, true), (2, true)],
            &[(3, true), (4, true), (5, true), (6, true)],
            &[(7, true), (8, true), (9, true)],
        ],
    );
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("no refusal");
    measure("fan, tol 1e-2", &before, &shell, &report);
    assert_eq!((report.merged_vertices, report.merged_edges), (2, 1));
    assert_eq!(
        (report.naked_edges_before, report.naked_edges_after),
        (10, 8)
    );
    assert!(report.max_vertex_displacement <= 0.01);
    assert!(shell.vertices.contains(&p(0.012, 0.0)));
    Shell::extract(shell).expect("welded shell extracts");
}

#[test]
fn refuses_equal_endpoints_with_different_interiors() {
    // Corners coincide, but the seam curves bulge by 0.1 and 0.05.
    let before = two_quads(0.0, 0.1, 0.05);
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("no topology refusal");
    assert!(!report.changed());
    assert_eq!(report.naked_edges_after, 8);
    assert!(
        matches!(report.skipped.as_slice(), [(1, 7, SkipReason::CurvesDiffer { at_least })] if *at_least > 0.01),
        "{:?}",
        report.skipped
    );
    assert_unchanged(&shell, &before);
}

#[test]
fn refuses_a_three_face_join() {
    // A third face C stands on the seam, its edge matching both A's and B's.
    let mut before = seam(4.0e-3);
    let x = 1.003;
    let base = before.vertices.len();
    before.vertices.extend([
        p(x, 1.0),
        p(x, 0.0),
        Point3::new(x, 0.0, 1.0),
        Point3::new(x, 1.0, 1.0),
    ]);
    let e = before.edges.len();
    let v = before.vertices.clone();
    before.edges.push(CompressedEdge {
        vertices: (base, base + 1),
        curve: bulged(v[base], v[base + 1], 0.1),
    });
    [(1, 2), (2, 3), (3, 0)].into_iter().for_each(|(a, b)| {
        before.edges.push(CompressedEdge {
            vertices: (base + a, base + b),
            curve: segment(v[base + a], v[base + b]),
        });
    });
    before.faces.push(CompressedFace {
        boundaries: vec![
            [(e, true), (e + 1, true), (e + 2, true), (e + 3, true)]
                .map(CompressedEdgeIndex::from)
                .to_vec(),
        ],
        orientation: true,
        surface: plane(),
    });
    let mut shell = before.clone();
    let result = snap_naked_edges(&mut shell, 0.01);
    assert_eq!(result, Err(SnapError::ThreeEdgeUses { edge: 1 }));
    assert_unchanged(&shell, &before);
}

#[test]
fn incompatible_orientation_is_left_naked_and_reported() {
    // B's flag flipped without reversing its wire: both faces now run the seam
    // the same way. Welding it would be wrong, so it stays naked, reported.
    let mut before = seam(4.0e-3);
    before.faces[1].orientation = false;
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("a conflict is not an error");
    assert!(!report.changed());
    assert_eq!(report.naked_edges_after, 8);
    assert_eq!(
        report.skipped,
        vec![(1, 7, SkipReason::IncompatibleOrientation)]
    );
    assert_unchanged(&shell, &before);
}

/// Unit square A (vertices 0..=3) and a face B whose right and top seams match
/// A's edges 1 and 2. The two B corners matching A's corner 2 are `q` and `q2`;
/// `wire` lists B's boundary between them.
fn corner_fixture(detour: bool) -> ModelShell {
    let mut v = vec![p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0), p(0.0, 1.0)];
    // B: p 4, q 5, q2 6, r 7, s 8 (and t 9 for the detour).
    v.extend([
        p(1.003, 0.0),
        p(1.003, 1.0),
        p(1.0, 1.003),
        p(0.0, 1.003),
        p(2.0, 0.5),
    ]);
    let mut pairs = vec![
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (5, 4),
        (4, 8),
        (8, 7),
        (7, 6),
    ];
    if detour {
        v.push(p(2.0, 1.5));
        pairs.extend([(6, 9), (9, 5)]);
    } else {
        pairs.push((6, 5));
    }
    let b_wire: Vec<(usize, bool)> = (4..pairs.len()).map(|edge| (edge, true)).collect();
    let edges = segments(&v, &pairs);
    shell_of(
        v,
        edges,
        &[&[(0, true), (1, true), (2, true), (3, true)], &b_wire],
    )
}

#[test]
fn refuses_a_collapsed_edge() {
    // B's short edge q2 -> q (edge 8) would have both ends merged into A's 2.
    let before = corner_fixture(false);
    let mut shell = before.clone();
    let result = snap_naked_edges(&mut shell, 0.01);
    assert_eq!(result, Err(SnapError::CollapsedEdge { edge: 8 }));
    assert_unchanged(&shell, &before);
}

#[test]
fn refuses_a_self_intersecting_repair() {
    // B detours from q2 out to t and back to q: merging q and q2 into A's 2
    // makes B's boundary pass through one vertex twice.
    let before = corner_fixture(true);
    let mut shell = before.clone();
    let result = snap_naked_edges(&mut shell, 0.01);
    assert_eq!(result, Err(SnapError::NonSimpleWire { face: 1 }));
    assert_unchanged(&shell, &before);
}

/// Hangs a triangle on `vertex` by the vertex alone: the triangle shares no
/// edge with anything, so it pinches the faces already at `vertex`.
fn hang_triangle(shell: &mut ModelShell, vertex: usize, far: [Point3; 2]) {
    let base = shell.vertices.len();
    shell.vertices.extend(far);
    let v = shell.vertices.clone();
    let e = shell.edges.len();
    [(vertex, base), (base, base + 1), (base + 1, vertex)]
        .into_iter()
        .for_each(|(a, b)| {
            shell.edges.push(CompressedEdge {
                vertices: (a, b),
                curve: segment(v[a], v[b]),
            });
        });
    shell.faces.push(CompressedFace {
        boundaries: vec![
            [(e, true), (e + 1, true), (e + 2, true)]
                .map(CompressedEdgeIndex::from)
                .to_vec(),
        ],
        orientation: true,
        surface: plane(),
    });
}

#[test]
fn a_pinch_that_exists_before_the_weld_does_not_block_it() {
    // Triangle C hangs on A's seam corner 2 before the weld. Welding the seam
    // leaves vertex 2 exactly as pinched as it was, so the seam welds.
    let mut before = seam(4.0e-3);
    hang_triangle(&mut before, 2, [p(1.5, 2.5), p(1.2, 3.0)]);
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("pinch is not the weld's");
    measure("pre-existing pinch, tol 1e-2", &before, &shell, &report);
    assert_eq!((report.merged_vertices, report.merged_edges), (2, 1));
    assert_eq!(
        (report.naked_edges_before, report.naked_edges_after),
        (11, 9)
    );
    Shell::extract(shell).expect("welded shell extracts");
}

#[test]
fn refuses_a_weld_that_worsens_a_pinch() {
    // Triangles hang on A's corner 2 and on B's corner 7. Each vertex has two
    // fans; welding merges 7 into 2 and leaves three fans at one vertex.
    let mut before = seam(4.0e-3);
    hang_triangle(&mut before, 2, [p(0.5, 2.5), p(0.2, 3.0)]);
    hang_triangle(&mut before, 7, [p(2.5, 2.5), p(2.2, 3.0)]);
    let mut shell = before.clone();
    let result = snap_naked_edges(&mut shell, 0.01);
    assert_eq!(result, Err(SnapError::SingularVertex { vertex: 2 }));
    assert_unchanged(&shell, &before);
}

/// Reviewer's finding 1: B's seam is a 256-span cubic whose bump sits between
/// any fixed set of samples. A real bound on the curve distance must see it.
#[test]
fn a_bump_between_samples_is_not_welded() {
    [0.05, 0.5, 5.0].into_iter().for_each(|height| {
        let mut before = two_quads(0.0, 0.0, 0.0);
        let (spans, degree) = (256usize, 3usize);
        let count = spans + degree;
        let mut control: Vec<Point3> = (0..count)
            .map(|i| p(1.0, 1.0 - i as f64 / (count - 1) as f64))
            .collect();
        control[4].x += height;
        control[5].x += height;
        before.edges[7].curve = Curve::BsplineCurve(BsplineCurve::new(
            KnotVector::uniform_knot(degree, spans),
            control,
        ));
        before.edges[1].curve = segment(p(1.0, 0.0), p(1.0, 1.0));
        let mut shell = before.clone();
        let report = snap_naked_edges(&mut shell, 0.01).expect("no refusal");
        assert!(!report.changed(), "height {height}");
        assert!(
            matches!(report.skipped.as_slice(), [(1, 7, SkipReason::CurvesDiffer { at_least })] if *at_least >= 0.9 * height),
            "height {height}: {:?}",
            report.skipped
        );
        assert_unchanged(&shell, &before);
    });
}

/// The bound is a bound: on the welded seam it is at least the true distance
/// (0.004 between parallel seams) and within the tolerance.
#[test]
fn the_reported_curve_deviation_bounds_the_true_distance() {
    let mut shell = seam(4.0e-3);
    let report = snap_naked_edges(&mut shell, 0.01).unwrap();
    assert!(report.max_curve_deviation >= 4.0e-3 - 1.0e-12);
    assert!(report.max_curve_deviation <= 0.01);
    assert!(report.skipped.is_empty());
}

#[test]
fn a_closed_edge_that_exists_before_the_weld_does_not_block_it() {
    let mut before = seam(4.0e-3);
    before.vertices.push(p(5.0, 5.0));
    before.edges.push(CompressedEdge {
        vertices: (8, 8),
        curve: Curve::BsplineCurve(BsplineCurve::new(
            KnotVector::bezier_knot(3),
            vec![p(5.0, 5.0), p(6.0, 6.0), p(4.0, 6.0), p(5.0, 5.0)],
        )),
    });
    before.faces.push(CompressedFace {
        boundaries: vec![vec![CompressedEdgeIndex::from((8, true))]],
        orientation: true,
        surface: plane(),
    });
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("closed edge is not the weld's");
    assert_eq!((report.merged_vertices, report.merged_edges), (2, 1));
}

/// Reviewer's finding 5: bucket indices used to overflow (a debug panic, a
/// silent wrap in release) for a tolerance this small.
#[test]
fn tiny_tolerances_neither_panic_nor_weld_wrongly() {
    [1.0e-300, 1.0e-320, 5.0e-324]
        .into_iter()
        .for_each(|tolerance| {
            [seam(4.0e-3), two_quads(0.0, 0.1, 0.1)]
                .into_iter()
                .for_each(|before| {
                    let mut shell = before.clone();
                    let report = snap_naked_edges(&mut shell, tolerance).expect("valid tolerance");
                    assert!(!report.changed(), "tolerance {tolerance:e}");
                    assert_unchanged(&shell, &before);
                });
        });
}

/// Reviewer's finding 7: a pair that tolerance creep rules out must not count
/// toward a three-face join. A's seam matches B (0.006 to one side) and C
/// (0.006 to the other); B and C are 0.012 apart, so only A-B welds.
#[test]
fn a_pair_ruled_out_by_creep_is_not_a_three_face_join() {
    let mut before = two_quads(6.0e-3, 0.0, 0.0);
    let x = 1.0 - 6.0e-3;
    let base = before.vertices.len();
    before.vertices.extend([
        p(x, 1.0),
        p(x, 0.0),
        Point3::new(x, 0.0, 1.0),
        Point3::new(x, 1.0, 1.0),
    ]);
    let v = before.vertices.clone();
    let e = before.edges.len();
    [(0, 1), (1, 2), (2, 3), (3, 0)]
        .into_iter()
        .for_each(|(a, b)| {
            before.edges.push(CompressedEdge {
                vertices: (base + a, base + b),
                curve: segment(v[base + a], v[base + b]),
            });
        });
    before.faces.push(CompressedFace {
        boundaries: vec![
            [(e, true), (e + 1, true), (e + 2, true), (e + 3, true)]
                .map(CompressedEdgeIndex::from)
                .to_vec(),
        ],
        orientation: true,
        surface: plane(),
    });
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("not a join");
    assert_eq!(report.merged_edges, 1);
    assert_eq!(report.skipped, vec![(1, e, SkipReason::ToleranceCreep)]);
}

/// Reviewer's finding 7: a curve end that was off its vertex before the weld
/// is not blamed on the weld when that vertex does not move.
#[test]
fn a_curve_end_gap_that_exists_before_the_weld_is_not_blamed_on_it() {
    let mut before = seam(4.0e-3);
    // Both seam curves end 0.015 below their bottom vertices.
    before.edges[1].curve = bulged(p(1.0, -0.015), p(1.0, 1.0), 0.1);
    before.edges[7].curve = bulged(p(1.004, 1.0), p(1.004, -0.015), 0.1);
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("gap is not the weld's");
    assert_eq!(report.merged_edges, 1);
    assert!((report.max_curve_endpoint_gap - 4.0e-3).abs() < 1.0e-12);
}

#[test]
fn a_closed_non_manifold_shell_is_a_noop() {
    let v = builder::vertex(Point3::origin());
    let e = builder::extrude(&v, Vector3::unit_x());
    let f = builder::extrude(&e, Vector3::unit_y());
    let cube = builder::extrude(&f, Vector3::unit_z());
    let mut before = cube.boundaries()[0].compress();
    let duplicate = before.faces[0].clone();
    before.faces.push(duplicate);
    let mut shell = before.clone();
    let report = snap_naked_edges(&mut shell, 0.01).expect("no naked edges");
    assert!(!report.changed());
    assert_unchanged(&shell, &before);
}

#[test]
fn an_edge_used_three_times_with_naked_edges_present_is_refused() {
    // Face A three times over: its edges are used three times each, while
    // B's edges are naked.
    let mut before = seam(4.0e-3);
    let duplicate = before.faces[0].clone();
    before.faces.push(duplicate.clone());
    before.faces.push(duplicate);
    let mut shell = before.clone();
    let result = snap_naked_edges(&mut shell, 0.01);
    assert!(
        matches!(result, Err(SnapError::ThreeEdgeUses { .. })),
        "{result:?}"
    );
    assert_unchanged(&shell, &before);
}
