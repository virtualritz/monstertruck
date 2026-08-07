//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;
use std::f64::consts::{FRAC_1_SQRT_2, PI};

const GRID: usize = 17;

fn as_nurbs(surface: BsplineSurface<Vector4>) -> NurbsSurface<Vector4> {
    NurbsSurface::new(surface)
}

fn grid_params() -> impl Iterator<Item = f64> { (0..GRID).map(|i| i as f64 / (GRID - 1) as f64) }

/// Exact `param -> angle` map of the full rational-quadratic unit circle the
/// conversion uses. The map is non-linear inside each 90-degree arc, so it
/// must be *evaluated*, not assumed to be `2*pi*t`.
fn full_circle_angle(t: f64) -> f64 {
    let p = full_unit_circle_curve().subs(t);
    let angle = p.y.atan2(p.x);
    if angle < 0.0 { angle + 2.0 * PI } else { angle }
}

/// Exact `param -> colatitude` map of the sphere meridian semicircle.
fn meridian_colatitude(s: f64) -> f64 {
    let w = FRAC_1_SQRT_2;
    let curve = BsplineCurve::new_unchecked(
        KnotVector::from(vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0]),
        vec![
            Vector4::new(0.0, 1.0, 0.0, 1.0),
            Vector4::new(w, w, 0.0, w),
            Vector4::new(1.0, 0.0, 0.0, 1.0),
            Vector4::new(w, -w, 0.0, w),
            Vector4::new(0.0, -1.0, 0.0, 1.0),
        ],
    );
    let p = curve.subs(s);
    // sin(colatitude) = radial = x/w, cos(colatitude) = axis = y/w.
    p.x.atan2(p.y)
}

fn torus_off_surface(torus: &Torus, point: Point3) -> f64 {
    let c = torus.center();
    let radial = ((point.x - c.x).powi(2) + (point.y - c.y).powi(2)).sqrt();
    ((radial - torus.large_radius()).powi(2) + (point.z - c.z).powi(2)).sqrt()
        - torus.small_radius()
}

fn assert_torus_grid(torus: Torus) {
    let hom = torus
        .try_into_homogeneous_bspline_surface()
        .expect("torus converts to homogeneous NURBS");
    let conv = as_nurbs(hom);
    let tol = 1.0e-9 * (torus.large_radius() + torus.small_radius());
    for s in grid_params() {
        let u_ang = full_circle_angle(s);
        for t in grid_params() {
            let v_ang = full_circle_angle(t);
            let got = conv.subs(s, t);
            let want = torus.subs(u_ang, v_ang);
            assert!(
                got.distance(want) <= tol,
                "torus subs mismatch at ({s},{t}) -> ({u_ang},{v_ang}): \
                 got {got:?} want {want:?} d={}",
                got.distance(want)
            );
            let off = torus_off_surface(&torus, got).abs();
            assert!(off <= tol, "torus off-surface at ({s},{t}): {off}");
        }
    }
}

fn assert_sphere_grid(sphere: Sphere) {
    let hom = sphere
        .try_into_homogeneous_bspline_surface()
        .expect("sphere converts to homogeneous NURBS");
    let conv = as_nurbs(hom);
    let tol = 1.0e-9 * sphere.radius();
    for s in grid_params() {
        let u_ang = meridian_colatitude(s);
        for t in grid_params() {
            let v_ang = full_circle_angle(t);
            let got = conv.subs(s, t);
            let want = sphere.subs(u_ang, v_ang);
            assert!(
                got.distance(want) <= tol,
                "sphere subs mismatch at ({s},{t}) -> ({u_ang},{v_ang}): \
                 got {got:?} want {want:?} d={}",
                got.distance(want)
            );
            let off = (got.distance(sphere.center()) - sphere.radius()).abs();
            assert!(off <= tol, "sphere off-surface at ({s},{t}): {off}");
        }
    }
}

#[test]
fn torus_ring_matches_analytic_on_grid() {
    assert_torus_grid(Torus::new(Point3::new(1.0, -2.0, 0.5), 3.0, 1.0));
}

#[test]
fn torus_horn_matches_analytic_on_grid() {
    assert_torus_grid(Torus::new(Point3::origin(), 2.0, 2.0));
}

#[test]
fn torus_tiny_pi_scale_matches_analytic() {
    // Pi horn-fillet scale (0.1 mm).
    assert_torus_grid(Torus::new(Point3::new(0.03, -0.01, 0.02), 0.1, 0.1));
}

#[test]
fn torus_fp_near_horn_is_representable() {
    // Pi torus #1: large radius a few ulps below small radius. Must NOT be
    // rejected as a spindle and must convert geometrically-exactly.
    let torus = Torus::new(
        Point3::origin(),
        0.099_999_999_992_725,
        0.099_999_999_999_987_88,
    );
    assert!(torus.try_into_homogeneous_bspline_surface().is_some());
    assert_torus_grid(torus);
}

#[test]
fn torus_horn_inner_equator_collapses_to_center() {
    let torus = Torus::new(Point3::new(0.5, 0.5, 0.5), 1.5, 1.5);
    let conv = as_nurbs(torus.try_into_homogeneous_bspline_surface().unwrap());
    // Tube angle pi (v-knot 0.5) is the inner equator -> a single point at
    // the center for a horn torus.
    for s in grid_params() {
        let p = conv.subs(s, 0.5);
        assert!(
            p.distance(torus.center()) <= 1.0e-9 * torus.large_radius(),
            "horn inner equator not at center at s={s}: {p:?}"
        );
    }
}

#[test]
fn torus_seams_are_periodic() {
    let torus = Torus::new(Point3::origin(), 4.0, 1.0);
    let conv = as_nurbs(torus.try_into_homogeneous_bspline_surface().unwrap());
    let tol = 1.0e-9 * (torus.large_radius() + torus.small_radius());
    for g in grid_params() {
        assert!(
            conv.subs(0.0, g).distance(conv.subs(1.0, g)) <= tol,
            "ring seam not periodic at v={g}"
        );
        assert!(
            conv.subs(g, 0.0).distance(conv.subs(g, 1.0)) <= tol,
            "tube seam not periodic at u={g}"
        );
    }
}

#[test]
fn torus_spindle_is_rejected() {
    // Small radius exceeds large radius: a self-intersecting spindle torus,
    // not a valid B-rep face.
    let torus = Torus::new(Point3::origin(), 1.0, 2.0);
    assert!(torus.try_into_homogeneous_bspline_surface().is_none());
    assert!(torus.try_into_bspline_surface().is_none());
}

#[test]
fn torus_is_rational_not_polynomial() {
    let torus = Torus::new(Point3::origin(), 3.0, 1.0);
    assert!(torus.try_into_bspline_surface().is_none());
}

#[test]
fn sphere_unit_matches_analytic_on_grid() {
    assert_sphere_grid(Sphere::new(Point3::origin(), 1.0));
}

#[test]
fn sphere_offset_matches_analytic_on_grid() {
    assert_sphere_grid(Sphere::new(Point3::new(1.0, 2.0, 3.0), 4.56));
}

#[test]
fn sphere_tiny_matches_analytic_on_grid() {
    assert_sphere_grid(Sphere::new(Point3::new(-0.02, 0.05, 0.01), 0.1));
}

#[test]
fn sphere_poles_collapse() {
    let sphere = Sphere::new(Point3::new(1.0, -1.0, 2.0), 2.5);
    let conv = as_nurbs(sphere.try_into_homogeneous_bspline_surface().unwrap());
    let north = sphere.center() + Vector3::new(0.0, 0.0, sphere.radius());
    let south = sphere.center() + Vector3::new(0.0, 0.0, -sphere.radius());
    let tol = 1.0e-9 * sphere.radius();
    for t in grid_params() {
        assert!(
            conv.subs(0.0, t).distance(north) <= tol,
            "north pole at t={t}"
        );
        assert!(
            conv.subs(1.0, t).distance(south) <= tol,
            "south pole at t={t}"
        );
    }
}

#[test]
fn sphere_longitude_seam_is_periodic() {
    let sphere = Sphere::new(Point3::origin(), 3.0);
    let conv = as_nurbs(sphere.try_into_homogeneous_bspline_surface().unwrap());
    let tol = 1.0e-9 * sphere.radius();
    for s in grid_params() {
        assert!(
            conv.subs(s, 0.0).distance(conv.subs(s, 1.0)) <= tol,
            "longitude seam not periodic at u={s}"
        );
    }
}

#[test]
fn sphere_is_rational_not_polynomial() {
    assert!(
        Sphere::new(Point3::origin(), 1.0)
            .try_into_bspline_surface()
            .is_none()
    );
}

// -----------------------------------------------------------------------
// Trim-driven span of the swept analytic surfaces (stage P-CONV).
//
// A STEP `CYLINDRICAL_SURFACE` loads as
// `Processor<RevolutionSurface<Line<Point3>>, Matrix4>` with a UNIT-LENGTH
// profile line and `orientation = false`, so the untrimmed conversion emits
// one axial unit of a surface the analytic form treats as unbounded. Nothing
// in the workspace pinned that before, which is why it survived.
// -----------------------------------------------------------------------

/// The STEP loader's own construction (`step_types.rs`, `CylindricalSurface`
/// -> `step_geometry::CylindricalSurface`): a unit-length profile line at
/// `center + x * radius`, revolved about `axis`, wrapped in an INVERTED
/// processor (which swaps the `(u, v)` axes, so `v` is the profile axis).
fn step_cylinder(
    center: Point3,
    axis: Vector3,
    radius: f64,
) -> Processor<RevolutionSurface<Line<Point3>>, Matrix4> {
    let radial = Vector3::unit_x();
    let start = center + radial * radius;
    let mut cylinder = Processor::new(RevolutionSurface::by_revolution(
        Line(start, start + axis),
        center,
        axis,
    ));
    cylinder.invert();
    cylinder
}

fn control_net_bbox(surface: &BsplineSurface<Vector4>) -> (Point3, Point3) {
    surface
        .control_points()
        .iter()
        .flat_map(|row| row.iter())
        .map(|point| point.to_point())
        .fold(
            (
                Point3::new(f64::MAX, f64::MAX, f64::MAX),
                Point3::new(f64::MIN, f64::MIN, f64::MIN),
            ),
            |(min, max), point| {
                (
                    Point3::new(min.x.min(point.x), min.y.min(point.y), min.z.min(point.z)),
                    Point3::new(max.x.max(point.x), max.y.max(point.y), max.z.max(point.z)),
                )
            },
        )
}

/// THE REGRESSION PIN. A cylinder whose face is trimmed to `v` in `[0, 80]`
/// must convert to a control net that spans all 80 units, not the profile
/// line's incidental 1.
#[test]
fn trimmed_cylinder_control_net_spans_the_face_not_the_profile() {
    let cylinder = step_cylinder(Point3::origin(), Vector3::unit_z(), 8.0);

    // Untrimmed: the one-unit stub, unchanged.
    let stub = cylinder
        .try_into_homogeneous_bspline_surface()
        .expect("cylinder converts");
    let (stub_min, stub_max) = control_net_bbox(&stub);
    assert!(
        (stub_max.z - stub_min.z - 1.0).abs() < 1.0e-12,
        "{stub_min:?} {stub_max:?}"
    );

    // `v` is the profile axis (the processor is inverted), so the face's
    // trim rectangle is (angle, height).
    let converted = cylinder
        .try_into_homogeneous_bspline_surface_over(Some(((0.0, 1.0), (0.0, 80.0))))
        .expect("trimmed cylinder converts");
    assert_eq!(
        converted.surface_frame_axes,
        (false, true),
        "the re-spanned axis must be reported to the consumer"
    );
    let trimmed = converted.surface;
    let (min, max) = control_net_bbox(&trimmed);
    assert!(
        min.z <= 0.0 + 1.0e-12 && max.z >= 80.0 - 1.0e-12,
        "converted control net must cover the whole 80-unit face, got z in [{}, {}]",
        min.z,
        max.z
    );
    // Radially it is still exactly the r = 8 cylinder's control polygon: the
    // rational-quadratic circle's control points project onto the square
    // circumscribing radius 8, so the net's x/y extent is exactly +-8.
    assert!((max.x - 8.0).abs() < 1.0e-9 && (min.x + 8.0).abs() < 1.0e-9);
    assert!((max.y - 8.0).abs() < 1.0e-9 && (min.y + 8.0).abs() < 1.0e-9);

    // Knot span: the emitted `v` parameter IS the surface's own profile
    // parameter over the requested interval, not a renormalized copy.
    let (_, v_knots) = trimmed.knot_vectors();
    assert!((v_knots[0] - 0.0).abs() < 1.0e-12);
    assert!((v_knots[v_knots.len() - 1] - 80.0).abs() < 1.0e-12);

    // And it is still EXACTLY the same cylinder: every evaluated point sits
    // on radius 8, and `subs` agrees with the analytic surface.
    let conv = as_nurbs(trimmed);
    for s in grid_params() {
        for t in grid_params() {
            let v = 80.0 * t;
            let got = conv.subs(s, v);
            let want = ParametricSurface::evaluate(&cylinder, s * 2.0 * PI, v);
            let radius = (got.x * got.x + got.y * got.y).sqrt();
            assert!(
                (radius - 8.0).abs() <= 1.0e-9 * 80.0,
                "off-cylinder at ({s},{v}): r={radius}"
            );
            assert!(
                (got.z - v).abs() <= 1.0e-9 * 80.0,
                "height mismatch at ({s},{v}): {got:?}"
            );
            // The angular map is the exact rational-circle map, so compare
            // against the analytic surface at the SAME evaluated angle.
            let angle = full_circle_angle(s);
            let want_at_angle = ParametricSurface::evaluate(&cylinder, angle, v);
            assert!(
                got.distance(want_at_angle) <= 1.0e-9 * 80.0,
                "subs mismatch at ({s},{v}): got {got:?} want {want_at_angle:?} \
                 (untrimmed reference {want:?})"
            );
        }
    }
}

/// The widening is DISJOINTNESS-gated: a trim that merely overlaps the
/// profile's own range leaves the conversion byte-identical, which is what
/// keeps the unit-height fillet faces (and every frozen fixture) unmoved.
#[test]
fn overlapping_trim_leaves_the_cylinder_conversion_byte_identical() {
    let cylinder = step_cylinder(Point3::new(1.0, -2.0, 3.0), Vector3::unit_z(), 2.5);
    let plain = cylinder
        .try_into_homogeneous_bspline_surface()
        .expect("cylinder converts");
    for trim in [
        // Exactly the profile's own range.
        ((0.0, 1.0), (0.0, 1.0)),
        // The upstream trim pad (`expand_param_axis_range`, at least
        // TOLERANCE) -- the same range, reported padded.
        ((0.0, 1.0), (-1.0e-6, 1.0 + 1.0e-6)),
        // Strictly inside: already covered, nothing to re-span.
        ((0.0, 1.0), (0.25, 0.75)),
    ] {
        let over = cylinder
            .try_into_homogeneous_bspline_surface_over(Some(trim))
            .expect("cylinder converts");
        assert_eq!(over.surface_frame_axes, (false, false));
        let over = over.surface;
        assert_eq!(
            over.control_points(),
            plain.control_points(),
            "overlapping trim {trim:?} must not move the control net"
        );
        assert_eq!(over.knot_vectors(), plain.knot_vectors());
    }
    // ...but a trim that genuinely reaches BELOW the profile start does
    // re-span: the face really is somewhere the naive sweep does not go.
    let below = cylinder
        .try_into_homogeneous_bspline_surface_over(Some(((0.0, 1.0), (-0.6, 1.0))))
        .expect("cylinder converts");
    assert_eq!(below.surface_frame_axes, (false, true));
    let (_, v_knots) = below.surface.knot_vectors();
    assert!((v_knots[0] + 0.6).abs() < 1.0e-12);
}

/// `None` -- and every non-swept surface -- is exactly the plain conversion.
#[test]
fn untrimmed_and_non_swept_conversions_are_unchanged() {
    let cylinder = step_cylinder(Point3::origin(), Vector3::unit_z(), 3.0);
    assert_eq!(
        cylinder
            .try_into_homogeneous_bspline_surface_over(None)
            .unwrap()
            .surface
            .control_points(),
        cylinder
            .try_into_homogeneous_bspline_surface()
            .unwrap()
            .control_points()
    );
    let trim = Some(((0.0, 1.0), (-500.0, -400.0)));
    for (over, plain) in [
        (
            Sphere::new(Point3::new(1.0, 2.0, 3.0), 4.0)
                .try_into_homogeneous_bspline_surface_over(trim),
            Sphere::new(Point3::new(1.0, 2.0, 3.0), 4.0).try_into_homogeneous_bspline_surface(),
        ),
        (
            Torus::new(Point3::origin(), 3.0, 1.0).try_into_homogeneous_bspline_surface_over(trim),
            Torus::new(Point3::origin(), 3.0, 1.0).try_into_homogeneous_bspline_surface(),
        ),
        (
            Plane::new(
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            )
            .try_into_homogeneous_bspline_surface_over(trim),
            Plane::new(
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            )
            .try_into_homogeneous_bspline_surface(),
        ),
    ] {
        assert_eq!(
            over.map(|converted| converted.surface.control_points().clone()),
            plain.map(|surface| surface.control_points().clone())
        );
    }
}

/// A cone is the same `RevolutionSurface<Line>` with a slanted profile, so
/// the widened sweep must stay the exact cone -- the radius has to keep
/// growing linearly along the axis, not freeze at the stub's.
#[test]
fn trimmed_cone_stays_an_exact_cone() {
    // Half-angle 45 degrees: profile from (1, 0, 0) towards (2, 0, 1).
    let start = Point3::new(1.0, 0.0, 0.0);
    let mut cone = Processor::new(RevolutionSurface::by_revolution(
        Line(start, start + Vector3::new(1.0, 0.0, 1.0)),
        Point3::origin(),
        Vector3::unit_z(),
    ));
    cone.invert();
    let conv = as_nurbs(
        cone.try_into_homogeneous_bspline_surface_over(Some(((0.0, 1.0), (10.0, 40.0))))
            .expect("cone converts")
            .surface,
    );
    for s in grid_params() {
        for t in grid_params() {
            let v = 10.0 + 30.0 * t;
            let point = conv.subs(s, v);
            let radius = (point.x * point.x + point.y * point.y).sqrt();
            // On this cone radius == 1 + height and height == v.
            assert!((point.z - v).abs() <= 1.0e-9 * 40.0, "height at ({s},{v})");
            assert!(
                (radius - (1.0 + v)).abs() <= 1.0e-9 * 40.0,
                "off-cone at ({s},{v}): r={radius} expected {}",
                1.0 + v
            );
        }
    }
}

#[test]
fn transformed_torus_via_processor_lies_on_surface() {
    // General placement rides the pre-existing `Processor<_, Matrix4>` arm;
    // confirm the composed surface is the correctly reoriented torus.
    let torus = Torus::new(Point3::origin(), 3.0, 1.0);
    // Rotate about x by 0.6 rad (tilts the torus axis off +z), then translate.
    let (sn, cs) = 0.6f64.sin_cos();
    let rot_x = Matrix4::from_cols(
        Vector4::new(1.0, 0.0, 0.0, 0.0),
        Vector4::new(0.0, cs, sn, 0.0),
        Vector4::new(0.0, -sn, cs, 0.0),
        Vector4::new(0.0, 0.0, 0.0, 1.0),
    );
    let transform = Matrix4::from_translation(Vector3::new(5.0, -3.0, 2.0)) * rot_x;
    let placed = Processor::with_transform(torus, transform);
    let conv = as_nurbs(
        placed
            .try_into_homogeneous_bspline_surface()
            .expect("placed torus converts"),
    );
    let inv = transform.invert().expect("rigid transform is invertible");
    let tol = 1.0e-9 * (torus.large_radius() + torus.small_radius());
    for s in grid_params() {
        for t in grid_params() {
            let got = conv.subs(s, t);
            let local_h = inv * got.to_homogeneous();
            let local = Point3::new(
                local_h.x / local_h.w,
                local_h.y / local_h.w,
                local_h.z / local_h.w,
            );
            let off = torus_off_surface(&torus, local).abs();
            assert!(off <= tol, "placed torus off-surface at ({s},{t}): {off}");
        }
    }
}
