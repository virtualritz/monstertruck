//! Unit tests for the parent module (`test_geom_impl`).
//!
//! Split out of the module file so the source stays readable. The module
//! name is unchanged, so every test keeps its path and its identity.

use super::*;
use proptest::*;

fn pole_to_normal(pole: [f64; 2]) -> Vector3 {
    let theta = PI * pole[0];
    let z = pole[1];
    let zi = f64::sqrt(f64::max(1.0 - z * z, 0.0));
    Vector3::new(f64::cos(theta) * zi, f64::sin(theta) * zi, z)
}

fn complex_boundary(angles: [f64; 10]) -> Vec<Point3> {
    let mut angle_store = 0.0;
    angles
        .into_iter()
        .enumerate()
        .flat_map(move |(i, angle)| {
            let prev_angle = angle_store;
            angle_store = angle;
            let r = 10.0 - i as f64;
            let min_theta = f64::acos(1.0 - 0.01 / r);
            let divs = 1 + (f64::abs(angle - prev_angle) / min_theta) as usize;
            (0..=divs).map(move |i| {
                let t = i as f64 / divs as f64;
                let theta = (1.0 - t) * prev_angle + t * angle;
                Point3::new(r * f64::cos(theta), r * f64::sin(theta), 0.0)
            })
        })
        .chain([Point3::origin(), Point3::new(10.0, 0.0, 0.0)])
        .collect()
}

fn dist_square(p: Point3, a: f64, b: f64) -> f64 {
    let absp = p.map(f64::abs);
    f64::min(a - absp.x, b - absp.y)
}

fn multiple_boundary(points: [Point3; 4], radius_ratios: [f64; 4]) -> Vec<Vec<Point3>> {
    let mut res = vec![vec![
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(-10.0, 10.0, 0.0),
        Point3::new(-10.0, -10.0, 0.0),
        Point3::new(10.0, -10.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
    ]];
    let mut radii = Vec::<f64>::new();
    res.extend((0..4).map(|i| {
        let mut dist = dist_square(points[i], 10.0, 10.0);
        (0..i).for_each(|j| {
            dist = f64::min(dist, points[i].distance(points[j]) - radii[j]);
        });
        (i + 1..4).for_each(|j| {
            dist = f64::min(dist, points[i].distance(points[j]));
        });
        radii.push(radius_ratios[i] * dist);
        (0..=10)
            .map(|j| {
                let theta = j as f64 / 10.0 * 2.0 * PI;
                Point3::new(f64::sin(theta), f64::cos(theta), 0.0)
            })
            .collect()
    }));
    res
}

proptest! {
    #[test]
    fn test_circum_center(
        p0 in array::uniform3(-10.0f64..10.0),
        p1 in array::uniform3(-10.0f64..10.0),
        p2 in array::uniform3(-10.0f64..10.0),
    ) {
        let p0 = Point3::from(p0);
        let p1 = Point3::from(p1);
        let p2 = Point3::from(p2);
        let c = circum_center(p0, p1, p2);

        // The point `c` exists at the same distance from the three points.
        let d0 = c.distance2(p0);
        let d1 = c.distance2(p1);
        let d2 = c.distance2(p2);
        prop_assert!(d0.near(&d1) && d1.near(&d2) && d2.near(&d0));
    }

    #[test]
    fn test_circle_arc_three_point(
        p0 in array::uniform3(-10.0f64..10.0),
        p1 in array::uniform3(-10.0f64..10.0),
        p2 in array::uniform3(-10.0f64..10.0),
        t in TOLERANCE..(1.0 - TOLERANCE),
    ) {
        let p0 = Point3::from(p0);
        let p1 = Point3::from(p1);
        let p2 = Point3::from(p2);
        let curve = circle_arc_by_three_points(p0, p1, p2);

        // The curve `curve` is from `p0` to `p1`.
        prop_assert_near!(curve.front(), p0);
        prop_assert_near!(curve.back(), p1);

        // Any point on the curve is on the same side as point `p2`.
        // Check by the circular angle theorem.
        let (t0, t1) = curve.range_tuple();
        let p3 = curve.subs((1.0 - t) * t0 + t * t1);
        let angle2 = (p2 - p1).angle(p2 - p0);
        let angle3 = (p3 - p1).angle(p3 - p0);
        prop_assert_near!(angle2, angle3);
    }

    #[test]
    fn test_circle_arc_by_start_tangent(
        p0 in array::uniform3(-10.0f64..10.0),
        p1 in array::uniform3(-10.0f64..10.0),
        tangent in array::uniform3(-10.0f64..10.0),
        t in TOLERANCE..(1.0 - TOLERANCE),
    ) {
        let p0 = Point3::from(p0);
        let p1 = Point3::from(p1);
        let tangent = Vector3::from(tangent);
        let chord = p1 - p0;
        prop_assume!(!chord.so_small());
        prop_assume!(!tangent.so_small());
        // Exclude the near-degenerate neighbourhood where the tangent is
        // (anti-)parallel to the chord. The old guard tested the *raw* cross
        // product, whose magnitude scales with |tangent|*|chord|, so it
        // admitted vanishingly small tangent/chord angles -- and the radius
        // R = |chord|/(2 sin(theta)) then explodes toward infinity, which no
        // finite-precision co-circularity check can certify. Bound the
        // *angle* directly (scale-invariant); this mirrors the constructor's
        // own `CircularArcTangentParallelToChord` degeneracy and keeps the
        // radius bounded so the reconstruction below stays well-conditioned.
        let sin_angle = tangent.normalize().cross(chord.normalize()).magnitude();
        prop_assume!(sin_angle > 1.0e-3);

        let curve = try_circle_arc_by_start_tangent(p0, p1, tangent)
            .expect("non-degenerate inputs must yield a valid arc.");
        let (t0, t1) = curve.range_tuple();

        // Endpoints land on the requested points.
        prop_assert_near!(curve.front(), p0);
        prop_assert_near!(curve.back(), p1);
        // The start tangent direction matches the requested one.
        prop_assert_near!(curve.derivative(t0).normalize(), tangent.normalize());

        // Any sample is co-circular with the endpoints. Reconstruct the
        // reference circle from a *well-separated* triple -- the two
        // endpoints and the arc midpoint -- because `circum_center` is
        // catastrophically ill-conditioned once its three inputs are nearly
        // coincident. Sampling the arc at a `t` arbitrarily close to 0 or 1
        // lands the sample right on top of an endpoint, so `t` must never
        // feed the reconstruction basis; it only picks the point under test.
        let mid = curve.evaluate(0.5 * (t0 + t1));
        let origin = circum_center(p0, p1, mid);
        let radius = origin.distance(p0);
        let p2 = curve.evaluate((1.0 - t) * t0 + t * t1);
        // `p2` lies on that circle to within a radius-relative tolerance:
        // co-circularity is a relative-radius property, so comparing squared
        // distances under an absolute tolerance (the previous form) is the
        // wrong metric -- a large radius inflates the absolute position error
        // even when the arc itself is exact.
        prop_assert!((origin.distance(p2) - radius).abs() <= TOLERANCE * (1.0 + radius));
    }

    #[test]
    fn test_circle_arc(
        origin in array::uniform3(-10.0f64..10.0),
        axis_pole in array::uniform2(-1.0f64..1.0),
        angle in TOLERANCE..(1.5 * PI),
        pt0 in array::uniform3(-10.0f64..10.0),
        t in TOLERANCE..(1.0 - TOLERANCE),
    ) {
        let origin = Point3::from(origin);
        let axis = pole_to_normal(axis_pole);
        let angle = Rad(angle);
        let pt0 = Point3::from(pt0);
        let curve = circle_arc(pt0, origin, axis, angle);

        // front point and back point
        let trans = Matrix4::from_translation(origin.to_vec())
            * Matrix4::from_axis_angle(axis, angle)
            * Matrix4::from_translation(-origin.to_vec());
        let pt1 = trans.transform_point(pt0);
        prop_assert_near!(curve.front(), pt0);
        prop_assert_near!(curve.back(), pt1);

        // Any point on the curve lies in the same plane perpendicular to the axis.
        let (t0, t1) = curve.range_tuple();
        let pt2 = curve.subs((1.0 - t) * t0 + t * t1);
        let vec0 = pt0 - origin;
        let vec2 = pt2 - origin;
        prop_assert_near!(vec0.dot(axis), vec2.dot(axis));

        // Any point on the curve lies in the circle arc from `p0` to `p1`.
        // Check by the circular angle theorem.
        let angle0 = (pt2 - pt1).angle(pt2 - pt0);
        prop_assert_near!(angle0 * 2.0, Rad(2.0 * PI) - angle);
    }

    #[test]
    fn test_take_one_axis_by_normal(normal in array::uniform3(-100.0f64..100.0)) {
        let normal = Vector3::from(normal);
        let axis = take_one_axis_by_normal(normal);
        prop_assert!(normal.so_small() || (!axis.so_small() && axis.dot(normal).so_small()));
    }

    #[test]
    fn test_attach_plane_with_single_boundary(
        axis_pole in array::uniform2(-1.0f64..1.0),
        origin in array::uniform3(-10.0f64..10.0),
        angles in array::uniform10(0.01f64..(2.0 * PI - 0.01)),
    ) {
        let axis = pole_to_normal(axis_pole);
        let origin = Point3::from(origin);
        let diag = take_one_axis_by_normal(axis);
        let trsf = Matrix4::from_cols(
            diag.extend(0.0),
            axis.cross(diag).extend(0.0),
            axis.extend(0.0),
            origin.to_homogeneous(),
        );
        let boundary: Vec<_> = complex_boundary(angles)
            .into_iter()
            .map(|p| trsf.transform_point(p))
            .collect();
        let plane = attach_plane(vec![boundary]).unwrap();
        prop_assert_near!(plane.normal(), axis);
    }

    #[test]
    fn test_attach_plane_with_multiple_boundary(
        axis_pole in array::uniform2(-1.0f64..1.0),
        origin in array::uniform3(-10.0f64..10.0),
        points in array::uniform8(1.0f64..9.0),
        radius_ratios in array::uniform4(0.1f64..0.9),
    ) {
        let axis = pole_to_normal(axis_pole);
        let origin = Point3::from(origin);
        let diag = take_one_axis_by_normal(axis);
        let trsf = Matrix4::from_cols(
            diag.extend(0.0),
            axis.cross(diag).extend(0.0),
            axis.extend(0.0),
            origin.to_homogeneous(),
        );
        let points = [
            Point3::new(points[0], points[1], 0.0),
            Point3::new(points[2] - 10.0, points[3], 0.0),
            Point3::new(points[4] - 10.0, points[5] - 10.0, 0.0),
            Point3::new(points[6], points[7] - 10.0, 0.0),
        ];
        let mut multiple_boundary = multiple_boundary(points, radius_ratios);
        multiple_boundary
            .iter_mut()
            .flatten()
            .for_each(|p| *p = trsf.transform_point(*p));
        let plane = attach_plane(multiple_boundary).unwrap();
        prop_assert_near!(plane.normal(), axis);
    }
}
