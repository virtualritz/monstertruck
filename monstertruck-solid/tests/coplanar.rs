//! Unions, differences and intersections of solids whose faces lie on one plane.

use monstertruck_meshing::prelude::*;
use monstertruck_modeling::*;

fn cuboid(min: [f64; 3], max: [f64; 3]) -> Solid {
    primitive::cuboid(BoundingBox::from_iter([
        Point3::new(min[0], min[1], min[2]),
        Point3::new(max[0], max[1], max[2]),
    ]))
}

fn volume(solid: &Solid) -> f64 { solid.triangulation(0.005).to_polygon().volume() }

fn check<E: std::fmt::Display>(label: &str, result: std::result::Result<Solid, E>, expected: f64) {
    let solid = result.unwrap_or_else(|error| panic!("{label}: {error}"));
    let got = volume(&solid);
    assert!(
        (got - expected).abs() < expected * 1.0e-6,
        "{label}: volume {got}, expected {expected}"
    );
    for shell in solid.boundaries() {
        assert_eq!(
            shell.shell_condition(),
            monstertruck_topology::shell::ShellCondition::Closed,
            "{label}"
        );
    }
}

fn base() -> Solid { cuboid([-10.0, -10.0, 0.0], [10.0, 10.0, 10.0]) }

#[test]
fn stack_the_same_footprint() {
    let top = cuboid([-10.0, -10.0, 10.0], [10.0, 10.0, 15.0]);
    check(
        "stack",
        monstertruck_solid::or_normalized(&base(), &top),
        6000.0,
    );
}

#[test]
fn stack_a_smaller_block() {
    let top = cuboid([-5.0, -5.0, 10.0], [5.0, 5.0, 15.0]);
    check(
        "smaller",
        monstertruck_solid::or_normalized(&base(), &top),
        4500.0,
    );
}

#[test]
fn stack_flush_with_three_sides() {
    let top = cuboid([-10.0, 0.0, 10.0], [10.0, 10.0, 15.0]);
    check(
        "flush",
        monstertruck_solid::or_normalized(&base(), &top),
        5000.0,
    );
}

#[test]
fn overlap_flush_with_three_sides() {
    let top = cuboid([-10.0, 0.0, 9.0], [10.0, 10.0, 15.0]);
    check(
        "overlap",
        monstertruck_solid::or_normalized(&base(), &top),
        5000.0,
    );
}

#[test]
fn side_by_side() {
    let other = cuboid([10.0, -5.0, 0.0], [20.0, 5.0, 10.0]);
    check(
        "side",
        monstertruck_solid::or_normalized(&base(), &other),
        5000.0,
    );
}

#[test]
fn pocket_from_the_top() {
    let cutter = cuboid([-5.0, -5.0, 5.0], [5.0, 5.0, 10.0]);
    check(
        "pocket",
        monstertruck_solid::difference_normalized(&base(), &cutter),
        3500.0,
    );
}

#[test]
fn notch_flush_with_a_side() {
    let cutter = cuboid([0.0, -10.0, 5.0], [10.0, 0.0, 10.0]);
    check(
        "notch",
        monstertruck_solid::difference_normalized(&base(), &cutter),
        3500.0,
    );
}

#[test]
fn common_of_flush_blocks() {
    let other = cuboid([0.0, -10.0, 0.0], [20.0, 10.0, 10.0]);
    check(
        "common",
        monstertruck_solid::and_normalized(&base(), &other),
        2000.0,
    );
}

#[test]
fn overlap_inside_without_flush_sides() {
    let top = cuboid([-9.5, 0.0, 9.0], [9.5, 9.5, 15.0]);
    check(
        "inside",
        monstertruck_solid::or_normalized(&base(), &top),
        4000.0 + 19.0 * 9.5 * 5.0,
    );
}

#[test]
fn overlap_flush_with_one_side() {
    let top = cuboid([-9.5, 0.0, 9.0], [10.0, 9.5, 15.0]);
    check(
        "one side",
        monstertruck_solid::or_normalized(&base(), &top),
        4000.0 + 19.5 * 9.5 * 5.0,
    );
}

fn cylinder(center: [f64; 3], radius: f64, height: f64) -> Solid {
    let center = Point3::new(center[0], center[1], center[2]);
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

#[test]
fn boss_on_a_plate() {
    let boss = cylinder([3.0, 2.0, 10.0], 2.0, 5.0);
    let expected = 4000.0 + std::f64::consts::PI * 4.0 * 5.0;
    let clock = std::time::Instant::now();
    let solid = monstertruck_solid::or_normalized(&base(), &boss).expect("union");
    let got = volume(&solid);
    assert!(
        (got - expected).abs() < expected * 1.0e-4,
        "{got} vs {expected}"
    );
    assert!(clock.elapsed().as_secs_f64() < 2.0, "{:?}", clock.elapsed());
}
