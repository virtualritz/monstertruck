//! Unit tests for the parent module (`partial_torus`).
//!
//! Split out of the module file so the source stays readable. The module
//! name is unchanged, so every test keeps its path and its identity.

use crate::*;
fn test_surface_orientation(surface: &Surface, sign: f64) {
    let rev = match surface {
        Surface::Plane(_) => return,
        Surface::RevolutionSurface(rev) => rev,
        _ => panic!(),
    };
    let (Some((u0, u1)), Some((v0, v1))) = rev.try_range_tuple() else {
        panic!();
    };
    let (u, v) = ((u0 + u1) / 2.0, (v0 + v1) / 2.0);
    let p = surface.subs(u, v);
    let q = Point3::from_vec(Vector3::new(p.x, p.y, 0.0).normalize() * 0.75);
    let n0 = sign * (p - q).normalize();
    let n1 = surface.normal(u, v);
    assert_near!(n0, n1)
}

fn test_boundary_orientation(face: &Face) {
    let surface = face.oriented_surface();
    let boundary = face.boundaries().pop().unwrap();
    let vec = boundary
        .iter()
        .flat_map(|edge| {
            let curve = edge.oriented_curve();
            let (t0, t1) = curve.range_tuple();
            [curve.subs(t0), curve.subs((t0 + t1) / 2.0), curve.subs(t1)]
        })
        .map(|p| surface.search_parameter(p, None, 100).unwrap())
        .collect::<Vec<_>>();
    let area = vec.windows(2).fold(0.0, |sum, v| {
        let ((u0, v0), (u1, v1)) = (v[0], v[1]);
        sum + (u0 + u1) * (v1 - v0)
    });
    assert!(area > 0.0)
}

fn test_shell(shell: &Shell, sign: f64) {
    shell.iter().for_each(|face| {
        test_boundary_orientation(face);
        test_surface_orientation(&face.oriented_surface(), sign);
    })
}

#[test]
fn partial_torus() {
    let v = builder::vertex(Point3::new(0.5, 0.0, 0.0));
    let w = builder::revolve(
        &v,
        Point3::new(0.75, 0.0, 0.0),
        Vector3::unit_y(),
        builder::SweepAngle::Closed,
        2,
    );
    let face = builder::try_attach_plane(&[w]).unwrap();
    test_shell(&shell![face.clone()], 1.0);
    let torus = builder::revolve(
        &face,
        Point3::origin(),
        Vector3::unit_z(),
        builder::SweepAngle::Partial(Rad(2.0)),
        1,
    );
    test_shell(&torus.boundaries()[0], 1.0);
    assert!(torus.is_geometric_consistent());
    let torus = builder::revolve(
        &face,
        Point3::origin(),
        Vector3::unit_z(),
        builder::SweepAngle::Partial(Rad(5.0)),
        2,
    );
    test_shell(&torus.boundaries()[0], 1.0);
    assert!(torus.is_geometric_consistent());
    let torus = builder::revolve(
        &face,
        Point3::origin(),
        Vector3::unit_z(),
        builder::SweepAngle::Partial(Rad(-2.0)),
        1,
    );
    test_shell(&torus.boundaries()[0], -1.0);
    assert!(torus.is_geometric_consistent());
    let torus = builder::revolve(
        &face,
        Point3::origin(),
        Vector3::unit_z(),
        builder::SweepAngle::Partial(Rad(-5.0)),
        2,
    );
    test_shell(&torus.boundaries()[0], -1.0);
    assert!(torus.is_geometric_consistent());
}
