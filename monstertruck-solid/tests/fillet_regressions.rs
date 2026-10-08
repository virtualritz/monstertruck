//! Fillets and chamfers on simple solids, checked against volumes worked out by hand.

use monstertruck_meshing::prelude::*;
use monstertruck_modeling::*;
use std::f64::consts::PI;

const MESH_TOL: f64 = 0.005;

fn cuboid(min: Point3, max: Point3) -> Solid {
    primitive::cuboid(BoundingBox::from_iter([min, max]))
}

fn cylinder(center: Point3, radius: f64, height: f64) -> Solid {
    let seed = builder::vertex(center + Vector3::unit_x() * radius);
    let rim = builder::revolve(
        &seed,
        center,
        Vector3::unit_z(),
        builder::SweepAngle::Closed,
        4,
    );
    let base = builder::try_attach_plane(&[rim]).unwrap();
    builder::extrude(&base, Vector3::unit_z() * height)
}

fn unique_edges(shell: &Shell, pick: impl Fn(Point3, Point3) -> bool) -> Vec<Edge> {
    shell
        .edge_iter()
        .filter(|edge| pick(edge.front().point(), edge.back().point()))
        .fold(Vec::new(), |mut edges, edge| {
            if !edges.iter().any(|known: &Edge| known.id() == edge.id()) {
                edges.push(edge);
            }
            edges
        })
}

fn blend(
    solid: &Solid,
    pick: impl Fn(Point3, Point3) -> bool,
    options: FilletOptions,
) -> (Solid, usize) {
    let mut shell = solid.boundaries()[0].clone();
    let edges = unique_edges(&shell, pick);
    assert!(!edges.is_empty(), "edge selection matched nothing");
    fillet_edges(&mut shell, &edges, Some(&options))
        .unwrap_or_else(|error| panic!("fillet_edges: {error:?}"));
    let condition = shell.shell_condition();
    let solid = Solid::try_new(vec![shell])
        .unwrap_or_else(|error| panic!("blended shell is not a solid ({condition:?}): {error}"));
    (solid, edges.len())
}

fn volume(solid: &Solid) -> f64 { solid.triangulation(MESH_TOL).to_polygon().volume() }

fn assert_volume(label: &str, solid: &Solid, expected: f64, relative: f64) {
    let actual = volume(solid);
    assert!(
        (actual - expected).abs() <= expected.abs() * relative,
        "{label}: volume {actual:.4}, expected {expected:.4} within {:.3}%",
        relative * 100.0
    );
}

fn round(radius: f64) -> FilletOptions { FilletOptions::constant(radius) }

fn chamfer(distance: f64) -> FilletOptions {
    FilletOptions::constant(distance).with_profile(FilletProfile::Chamfer)
}

const SPANDREL: f64 = 1.0 - PI / 4.0;

const SPANDREL_CENTROID: f64 = (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);

fn plate() -> Solid { cuboid(Point3::new(-20.0, -15.0, 0.0), Point3::new(20.0, 15.0, 3.0)) }

fn on_top(z: f64) -> impl Fn(Point3, Point3) -> bool {
    move |a: Point3, b: Point3| (a.z - z).abs() < 1e-9 && (b.z - z).abs() < 1e-9
}

#[test]
fn round_one_top_edge() {
    let (solid, count) = blend(
        &plate(),
        |a, b| on_top(3.0)(a, b) && a.y > 14.9 && b.y > 14.9,
        round(1.0),
    );
    assert_eq!(count, 1);
    assert_volume("round one edge", &solid, 3600.0 - 40.0 * SPANDREL, 0.001);
}

#[test]
fn round_four_vertical_edges() {
    let vertical = |a: Point3, b: Point3| (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9;
    let (solid, count) = blend(&plate(), vertical, round(1.0));
    assert_eq!(count, 4);
    assert_volume(
        "round vertical edges",
        &solid,
        3600.0 - 4.0 * 3.0 * SPANDREL,
        0.001,
    );
}

const ROUND_CORNER_OVERLAP: f64 = 0.095_870_338;

#[test]
fn round_top_perimeter() {
    let (solid, count) = blend(&plate(), on_top(3.0), round(1.0));
    assert_eq!(count, 4);
    let expected = 3600.0 - 140.0 * SPANDREL + 4.0 * ROUND_CORNER_OVERLAP;
    assert_volume("round top perimeter", &solid, expected, 0.0005);
}

#[test]
fn chamfer_top_perimeter() {
    let (solid, count) = blend(&plate(), on_top(3.0), chamfer(1.0));
    assert_eq!(count, 4);
    assert_volume(
        "chamfer top perimeter",
        &solid,
        3600.0 - 70.0 + 4.0 / 3.0,
        0.0005,
    );
}

#[test]
fn rounding_every_cube_edge_reports_missing_vertex_blend() {
    let cube = cuboid(Point3::origin(), Point3::new(20.0, 20.0, 20.0));
    let mut shell = cube.boundaries()[0].clone();
    let edges = unique_edges(&shell, |_, _| true);
    let result = fillet_edges(&mut shell, &edges, Some(&round(2.0)));
    assert!(
        matches!(result, Err(FilletError::VertexBlendUnsupported)),
        "{result:?}"
    );
}

fn post(height: f64) -> Solid { cylinder(Point3::origin(), 5.0, height) }

#[test]
fn chamfer_cylinder_top() {
    let (solid, _) = blend(&post(10.0), on_top(10.0), chamfer(1.0));
    let removed = 2.0 * PI * (5.0 - 1.0 / 3.0) * 0.5;
    assert_volume(
        "chamfer cylinder top",
        &solid,
        PI * 25.0 * 10.0 - removed,
        0.003,
    );
}

#[test]
fn round_cylinder_top() {
    let (solid, _) = blend(&post(10.0), on_top(10.0), round(1.0));
    let removed = 2.0 * PI * (5.0 - SPANDREL_CENTROID) * SPANDREL;
    assert_volume(
        "round cylinder top",
        &solid,
        PI * 25.0 * 10.0 - removed,
        0.003,
    );
}

fn drilled_block() -> Solid {
    let corners = builder::vertices([
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(20.0, 20.0, 0.0),
        Point3::new(0.0, 20.0, 0.0),
    ]);
    let outline: Wire = (0..4)
        .map(|i| builder::line(&corners[i], &corners[(i + 1) % 4]))
        .collect();
    let seed = builder::vertex(Point3::new(12.0, 10.0, 0.0));
    let hole = builder::revolve(
        &seed,
        Point3::new(10.0, 10.0, 0.0),
        Vector3::unit_z(),
        builder::SweepAngle::Closed,
        4,
    );
    let base = builder::try_attach_plane(&[outline, hole.inverse()]).unwrap();
    builder::extrude(&base, Vector3::unit_z() * 20.0)
}

#[test]
fn chamfer_hole_rim() {
    let rim = |a: Point3, b: Point3| {
        let center = Point3::new(10.0, 10.0, 20.0);
        on_top(20.0)(a, b) && (a - center).magnitude() < 2.1 && (b - center).magnitude() < 2.1
    };
    let (solid, _) = blend(&drilled_block(), rim, chamfer(0.5));
    let removed = 2.0 * PI * (2.0 + 0.5 / 3.0) * 0.125;
    assert_volume(
        "chamfer hole rim",
        &solid,
        8000.0 - PI * 4.0 * 20.0 - removed,
        0.003,
    );
}

#[test]
fn round_outer_edge_after_drilling() {
    let outer = |a: Point3, b: Point3| on_top(20.0)(a, b) && a.y > 19.99 && b.y > 19.99;
    let (solid, count) = blend(&drilled_block(), outer, round(1.0));
    assert_eq!(count, 1);
    assert_volume(
        "round outer edge after drilling",
        &solid,
        8000.0 - PI * 4.0 * 20.0 - 20.0 * SPANDREL,
        0.003,
    );
}

fn back_and_right(a: Point3, b: Point3) -> bool {
    on_top(3.0)(a, b) && ((a.y > 14.9 && b.y > 14.9) || (a.x > 19.9 && b.x > 19.9))
}

#[test]
fn round_open_top_chain() {
    let (solid, count) = blend(&plate(), back_and_right, round(1.0));
    assert_eq!(count, 2);
    let expected = 3600.0 - 70.0 * SPANDREL + ROUND_CORNER_OVERLAP;
    assert_volume("round open top chain", &solid, expected, 0.0005);
}

#[test]
fn chamfer_open_top_chain() {
    let (solid, count) = blend(&plate(), back_and_right, chamfer(1.0));
    assert_eq!(count, 2);
    assert_volume(
        "chamfer open top chain",
        &solid,
        3600.0 - 35.0 + 1.0 / 3.0,
        0.0005,
    );
}

fn interior_multiplicity_within_degree(knots: &KnotVector, degree: usize) -> bool {
    let (_, multiplicities) = knots.to_single_multi();
    multiplicities.len() < 3
        || multiplicities[1..multiplicities.len() - 1]
            .iter()
            .all(|&m| m <= degree)
}

#[test]
fn filleted_geometry_is_exportable() {
    let (solid, _) = blend(&plate(), on_top(3.0), round(1.0));
    let shell = &solid.boundaries()[0];
    let planes = shell
        .face_iter()
        .filter(|face| matches!(face.surface(), Surface::Plane(_)))
        .count();
    assert_eq!(
        planes, 6,
        "planar faces should stay planes through a fillet"
    );
    shell.face_iter().for_each(|face| {
        if let Surface::NurbsSurface(surface) = face.surface() {
            assert!(interior_multiplicity_within_degree(
                surface.knot_vector_u(),
                surface.udegree()
            ));
            assert!(interior_multiplicity_within_degree(
                surface.knot_vector_v(),
                surface.vdegree()
            ));
        }
    });
    shell.edge_iter().for_each(|edge| {
        if let Curve::NurbsCurve(curve) = edge.curve() {
            assert!(interior_multiplicity_within_degree(
                curve.knot_vector(),
                curve.degree()
            ));
        }
    });
}
