//! Booleans that used to fail or lose material, checked against volumes worked out by hand.

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

fn drill(solid: &Solid, center: Point3, radius: f64, height: f64) -> Solid {
    let tool = cylinder(center, radius, height);
    monstertruck_solid::difference_normalized(solid, &tool)
        .unwrap_or_else(|error| panic!("drill r={radius} at {center:?}: {error}"))
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

fn plate() -> Solid { cuboid(Point3::new(-20.0, -15.0, 0.0), Point3::new(20.0, 15.0, 3.0)) }

fn plate_hole_volume(radius: f64) -> f64 { 40.0 * 30.0 * 3.0 - PI * radius * radius * 3.0 }

#[test]
fn plate_hole_at_any_scale() {
    for scale in [0.01, 1.0, 100.0] {
        let plate = cuboid(
            Point3::new(-20.0, -15.0, 0.0) * scale,
            Point3::new(20.0, 15.0, 3.0) * scale,
        );
        let solid = drill(
            &plate,
            Point3::new(10.0, 5.0, -1.0) * scale,
            1.6 * scale,
            5.0 * scale,
        );
        let expected = plate_hole_volume(1.6) * scale * scale * scale;
        assert_volume(&format!("plate hole x{scale}"), &solid, expected, 0.002);
    }
}

#[test]
fn plate_hole_near_corner() {
    let solid = drill(&plate(), Point3::new(18.0, 13.0, -1.0), 0.5, 5.0);
    assert_volume(
        "plate hole near corner",
        &solid,
        plate_hole_volume(0.5),
        0.002,
    );
}

#[test]
fn plate_with_boss() {
    let boss = cylinder(Point3::new(0.0, 0.0, 2.0), 4.0, 6.0);
    let solid = monstertruck_solid::or_normalized(&plate(), &boss).expect("union with boss");
    assert_volume("plate with boss", &solid, 3600.0 + PI * 16.0 * 5.0, 0.002);
}

#[test]
fn plate_four_holes() {
    let solid = [(-10.0, 5.0), (10.0, 5.0), (10.0, -5.0), (-10.0, -5.0)]
        .into_iter()
        .fold(plate(), |solid, (x, y)| {
            drill(&solid, Point3::new(x, y, -1.0), 1.6, 5.0)
        });
    assert_volume(
        "plate four holes",
        &solid,
        40.0 * 30.0 * 3.0 - 4.0 * PI * 1.6 * 1.6 * 3.0,
        0.002,
    );
}

fn square_wire(half: f64, z: f64) -> Wire {
    let v = builder::vertices([
        Point3::new(-half, -half, z),
        Point3::new(half, -half, z),
        Point3::new(half, half, z),
        Point3::new(-half, half, z),
    ]);
    (0..4)
        .map(|i| builder::line(&v[i], &v[(i + 1) % 4]))
        .collect()
}

#[test]
fn ring_boss_keeps_the_plate_inside_it() {
    let plate = cuboid(Point3::new(-30.0, -15.0, 0.0), Point3::new(30.0, 15.0, 2.0));
    let ring: Solid = profile::solid_from_planar_profile(
        vec![square_wire(5.0, 1.5), square_wire(3.0, 1.5)],
        Vector3::unit_z() * 1.5,
    )
    .unwrap();
    let solid = monstertruck_solid::or_normalized(&plate, &ring).expect("union with a ring");
    assert_volume("ring boss", &solid, 3600.0 + (100.0 - 36.0) * 1.0, 1.0e-6);
}

#[test]
fn ring_crossing_a_face_at_a_mesh_row() {
    let block = cuboid(
        Point3::new(-20.0, -20.0, 0.0),
        Point3::new(20.0, 20.0, 10.0),
    );
    let outer = cylinder(Point3::new(0.0, 0.0, 8.0), 12.0, 4.0);
    let mut inner = cylinder(Point3::new(0.0, 0.0, 7.0), 8.0, 6.0);
    inner.not();
    let ring = monstertruck_solid::and(&outer, &inner, 1.0e-3).expect("ring");
    let solid = monstertruck_solid::or_normalized(&block, &ring).expect("ring across the top face");
    assert_volume(
        "ring across a face",
        &solid,
        16000.0 + PI * (144.0 - 64.0) * 2.0,
        0.002,
    );
}

#[test]
fn box_across_an_edge_of_a_union() {
    let base = cuboid(Point3::new(0.0, -10.0, 0.0), Point3::new(40.0, 10.0, 2.0));
    let wall = cuboid(Point3::new(2.0, -8.0, 1.0), Point3::new(4.0, 8.0, 30.0));
    let bracket = monstertruck_solid::or_normalized(&base, &wall).expect("bracket");
    assert_volume("bracket", &bracket, 1600.0 + 928.0 - 32.0, 0.001);
    let gusset = cuboid(Point3::new(3.0, -1.0, 1.5), Point3::new(8.0, 1.0, 8.0));
    let solid = monstertruck_solid::or_normalized(&bracket, &gusset)
        .expect("block across the inside corner");
    assert_volume(
        "block across a corner",
        &solid,
        2496.0 + 4.0 * 6.0 * 2.0,
        0.001,
    );
}

#[test]
fn small_hole_survives_a_later_cut() {
    let plate = cuboid(Point3::new(-40.0, -25.0, 0.0), Point3::new(40.0, 25.0, 2.0));
    let first = drill(&plate, Point3::new(-20.0, 0.0, -1.0), 0.8, 4.0);
    let mut tool = cylinder(Point3::new(20.0, 0.0, -1.0), 0.8, 4.0);
    tool.not();
    let second = monstertruck_solid::and(&first, &tool, 0.05).expect("second hole");
    let one = volume(&plate) - volume(&first);
    let two = volume(&plate) - volume(&second);
    assert!(two > one * 1.5, "one hole {one}, two holes {two}");
}
