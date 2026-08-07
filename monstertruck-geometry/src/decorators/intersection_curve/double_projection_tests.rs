//! Unit tests for the parent module (`double_projection_tests`).
//!
//! Split out of the module file so the source stays readable. The module
//! name is unchanged, so every test keeps its path and its identity.

use super::*;
use proptest::prelude::*;
use std::f64::consts::PI;
type PResult = std::result::Result<(), TestCaseError>;

fn unit_normal_candidate(u: [f64; 2]) -> Vector3 {
    let angle = 2.0 * PI * u[0];
    let w = f64::sqrt(1.0 - u[1] * u[1]);
    Vector3::new(w * f64::cos(angle), w * f64::sin(angle), u[1])
}

fn create_axis(n: Vector3) -> (Vector3, Vector3) {
    let idx = if n[0].abs() < n[1].abs() { 0 } else { 1 };
    let idx = if n[idx].abs() < n[2].abs() { idx } else { 2 };
    let mut e = Vector3::zero();
    e[idx] = 1.0;
    let x = n.cross(e).normalize();
    (x, n.cross(x))
}

fn exec_plane_case(c0: [f64; 3], n0: [f64; 2], c1: [f64; 3], n1: [f64; 2]) -> PResult {
    let c0 = Point3::from(c0);
    let n0 = unit_normal_candidate(n0);
    let (x, y) = create_axis(n0);
    let plane0 = Plane::new(c0, c0 + x, c0 + y);
    let c1 = Point3::from(c1);
    let n1 = unit_normal_candidate(n1);
    let (x, y) = create_axis(n1);
    let plane1 = Plane::new(c1, c1 + x, c1 + y);
    let n = n0.cross(n1).normalize();
    let mut o = None;
    for i in 0..10 {
        let t = i as f64;
        let p = Point3::origin() + t * n;
        let (q, p0, p1) = double_projection(&plane0, None, &plane1, None, p, n, 100)
            .unwrap_or_else(|| panic!("plane0: {plane0:?}\nplane1: {plane1:?}\n p: {p:?}"));
        prop_assert_near!(q, plane0.evaluate(p0.x, p0.y));
        prop_assert_near!(q, plane1.evaluate(p1.x, p1.y));
        if let Some(o) = o {
            prop_assert_near!(q.distance2(o), t * t);
        } else {
            o = Some(q);
        }
    }
    Ok(())
}

proptest! {
    #[test]
    fn plane_case(
        c0 in prop::array::uniform3(-1f64..=1f64),
        n0 in prop::array::uniform2(0f64..=1f64),
        c1 in prop::array::uniform3(-1f64..=1f64),
        n1 in prop::array::uniform2(0f64..=1f64),
    ) {
        exec_plane_case(c0, n0, c1, n1)?;
    }
}

fn exec_sphere_case(t: f64, r: f64) -> PResult {
    let sphere0 = Sphere::new(Point3::new(0.0, 0.0, 1.0), f64::sqrt(2.0));
    let sphere1 = Sphere::new(Point3::new(0.0, 0.0, -1.0), f64::sqrt(2.0));
    let p = Point3::new(r * f64::cos(t), r * f64::sin(t), 0.0);
    let n = Vector3::new(-f64::sin(t), f64::cos(t), 0.0);
    let (q, p0, p1) = double_projection(&sphere0, None, &sphere1, None, p, n, 100)
        .unwrap_or_else(|| panic!("p: {p:?}"));
    prop_assert_near!(q, sphere0.evaluate(p0.x, p0.y));
    prop_assert_near!(q, sphere1.evaluate(p1.x, p1.y));
    prop_assert_near!(q, Point3::new(f64::cos(t), f64::sin(t), 0.0));
    Ok(())
}

proptest! {
    #[test]
    fn sphere_case(t in 0f64..=(2.0 * PI), r in 0.5f64..=1.5f64) {
        exec_sphere_case(t, r)?;
    }
}
