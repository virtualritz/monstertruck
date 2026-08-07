//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;
use monstertruck_geometry::prelude::{
    Line, Matrix4, ParameterCurve, Plane, Point2, Point3, Processor, RevolutionSurface, Vector3,
};
use std::f64::consts::{FRAC_PI_2, TAU};

fn trim_line(surface: Plane, front: Point2, back: Point2) -> ParameterCurve<Line<Point2>, Plane> {
    ParameterCurve::new(Line(front, back), surface)
}

#[test]
fn normalize_axis_preserves_far_nonperiodic_values() {
    assert_eq!(
        PolyBoundaryPiece::normalize_axis(68.0, None, None, Some((0.0, 1.0))),
        Some(68.0),
    );
    assert_eq!(
        PolyBoundaryPiece::normalize_axis(1.0 + TOLERANCE * 0.5, None, None, Some((0.0, 1.0)),),
        Some(1.0),
    );
}

#[test]
fn face_local_trim_orientation_is_not_reversed_by_edge_orientation() {
    let surface = Plane::xy();
    let trims = [
        trim_line(surface, Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
        trim_line(surface, Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)),
        trim_line(surface, Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)),
        trim_line(surface, Point2::new(0.0, 1.0), Point2::new(0.0, 0.0)),
    ];
    let edges = [
        Line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
        Line(Point3::new(1.0, 1.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
        Line(Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)),
        Line(Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)),
    ];
    let wire = [
        (true, Some(&trims[0]), &edges[0]),
        (false, Some(&trims[1]), &edges[1]),
        (true, Some(&trims[2]), &edges[2]),
        (false, Some(&trims[3]), &edges[3]),
    ];

    let piece = PolyBoundaryPiece::try_new_from_trimmed(&surface, wire.into_iter(), TOLERANCE)
        .expect("face-local trims should build a boundary");
    let uvs = piece
        .0
        .iter()
        .map(|point| (point.x, point.y))
        .collect::<Vec<_>>();

    assert_eq!(
        uvs,
        vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    );
}

#[test]
fn exact_trim_orientation_is_aligned_to_oriented_edge_endpoints() {
    let surface = Plane::xy();
    let trim = trim_line(surface, Point2::new(0.0, 0.0), Point2::new(1.0, 0.0));
    let edge = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    let wire = [(true, Some(&trim), &edge)];

    let piece = PolyBoundaryPiece::try_new_from_trimmed(&surface, wire.into_iter(), TOLERANCE)
        .expect("exact trim should build a boundary");
    let uvs = piece
        .0
        .iter()
        .map(|point| (point.x, point.y))
        .collect::<Vec<_>>();

    assert_eq!(uvs[0], (1.0, 0.0));
    assert_eq!(uvs[1], (0.0, 0.0));
}

#[test]
fn closed_exact_trim_orientation_is_not_endpoint_aligned() {
    let surface = Plane::xy();
    let trim = trim_line(surface, Point2::new(0.0, 0.0), Point2::new(1.0, 0.0));
    let edge = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let wire = [(true, Some(&trim), &edge)];

    let piece = PolyBoundaryPiece::try_new_from_trimmed(&surface, wire.into_iter(), TOLERANCE)
        .expect("closed exact trim should build a boundary");
    let uvs = piece
        .0
        .iter()
        .map(|point| (point.x, point.y))
        .collect::<Vec<_>>();

    assert_eq!(uvs[0], (0.0, 0.0));
    assert_eq!(uvs[1], (1.0, 0.0));
}

#[test]
fn aligned_exact_projects_periodic_edge_samples_before_resampling() {
    let profile = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0));
    let surface = RevolutionSurface::by_revolution(profile, Point3::origin(), Vector3::unit_z());
    let half_span = FRAC_PI_2 / 3.0;
    let boundary = Some(vec![
        Point2::new(0.0, TAU - half_span),
        Point2::new(0.0, half_span),
    ]);
    let polyline = (0..=4)
        .map(|index| {
            let angle = -half_span + half_span * index as f64 / 2.0;
            surface.evaluate(0.0, angle)
        })
        .collect::<PolylineCurve>();
    let piece = PolyBoundaryPiece::try_new_from_aligned_exact(
        &surface,
        [(boundary, polyline)].into_iter(),
        &|_: &RevolutionSurface<Line<Point3>>, point: Point3, _| {
            let angle = point.y.atan2(point.x);
            Some((point.z, angle))
        },
    )
    .expect("periodic edge samples should project onto the surface");
    let span = piece
        .0
        .iter()
        .map(|point| point.y)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), value| {
            (min.min(value), max.max(value))
        });

    assert!(span.1 - span.0 <= 2.0 * half_span + TOLERANCE);
}

#[test]
fn full_period_cone_loop_closes_to_singular_side() {
    let profile = Line(Point3::new(2.5, 0.0, 0.0), Point3::new(3.5, 0.0, 1.0));
    let mut surface: Processor<_, Matrix4> = Processor::new(RevolutionSurface::by_revolution(
        profile,
        Point3::origin(),
        Vector3::unit_z(),
    ));
    surface.invert();
    let ring = PolyBoundaryPiece(
        (0..=16)
            .map(|index| Point2::new(TAU * index as f64 / 16.0, 0.0))
            .map(|uv| SurfacePoint::from((uv, surface.evaluate(uv.x, uv.y))))
            .collect(),
    );
    let boundary = PolyBoundary::new(vec![ring], &surface, 0.01);

    assert!(
        boundary.uv_min.y <= -2.5 + TOLERANCE,
        "expected cone cap to include the apex side, got {:?}..{:?}",
        boundary.uv_min,
        boundary.uv_max,
    );
    assert!(
        boundary.uv_max.y <= TOLERANCE,
        "expected cone cap to end at the trim ring, got {:?}..{:?}",
        boundary.uv_min,
        boundary.uv_max,
    );
}

#[test]
fn full_period_ring_pair_uses_shared_periodic_seam() {
    let profile = Line(Point3::new(2.5, 0.0, 0.0), Point3::new(2.5, 0.0, 1.0));
    let mut surface: Processor<_, Matrix4> = Processor::new(RevolutionSurface::by_revolution(
        profile,
        Point3::origin(),
        Vector3::unit_z(),
    ));
    surface.invert();
    let ring = |v: f64, samples: &[f64]| {
        PolyBoundaryPiece(
            samples
                .iter()
                .map(|u| Point2::new(*u, v))
                .map(|uv| SurfacePoint::from((uv, surface.evaluate(uv.x, uv.y))))
                .collect(),
        )
    };
    let ring0_samples = [
        3.0 * FRAC_PI_2,
        4.0,
        3.0,
        2.0,
        1.0,
        0.0,
        -0.8,
        -1.4,
        3.0 * FRAC_PI_2 - TAU,
    ];
    let ring1_samples = [0.0, -0.3, -0.9, -1.8, -2.7, -4.1, -5.4, -TAU];
    let original_points = ring0_samples
        .iter()
        .map(|u| surface.evaluate(*u, 0.25))
        .chain(ring1_samples.iter().map(|u| surface.evaluate(*u, 0.75)))
        .collect::<Vec<_>>();
    let boundary = PolyBoundary::new(
        vec![ring(0.25, &ring0_samples), ring(0.75, &ring1_samples)],
        &surface,
        0.01,
    );
    let span = boundary.uv_max.x - boundary.uv_min.x;

    assert!(
        span <= TAU + TOLERANCE,
        "expected full-period rings to share a seam, got {:?}..{:?}",
        boundary.uv_min,
        boundary.uv_max,
    );
    assert!(original_points.iter().all(|point| {
        boundary
            .loops
            .iter()
            .flatten()
            .any(|boundary_point| boundary_point.point.distance(*point) <= TOLERANCE)
    }));
}

#[test]
fn cdt_insertion_splits_duplicate_seam_chords_through_existing_vertices() {
    const LOOP_POINT_BITS: &[(u64, u64)] = &[
        (4614256656552045848, 4585802930648964187),
        (4614138752589512040, 4585802930648964187),
        (4614020848626978228, 4585802930648964187),
        (4613785040701910618, 4585802930648964187),
        (4613549232776843003, 4585802930648964188),
        (4613313424851775388, 4585802930648964188),
        (4613077616926707775, 4585802930648964188),
        (4612841809001640160, 4585802930648964188),
        (4612606001076572546, 4585802930648964188),
        (4612370193151504928, 4585802930648964188),
        (4612134385226437316, 4585802930648964188),
        (4611898577301369703, 4585802930648964189),
        (4611639520325216272, 4585802930648964189),
        (4611167904475081038, 4585802930648964189),
        (4610696288624945807, 4585802930648964189),
        (4610224672774810583, 4585802930648964189),
        (4609753056924675350, 4585802930648964190),
        (4609517248999607734, 4585802930648964190),
        (4609281441074540120, 4585802930648964190),
        (4608809825224404895, 4585802930648964190),
        (4608338209374269663, 4585802930648964190),
        (4607866593524134430, 4585802930648964190),
        (4607394977673999207, 4585802930648964190),
        (4606664304847710542, 4585802930648964191),
        (4605721073147440095, 4585802930648964191),
        (4604777841447169628, 4585802930648964191),
        (4603834609746899166, 4585802930648964191),
        (4602891378046628704, 4585802930648964191),
        (4601217473520069596, 4585802930648964191),
        (4599331010119528668, 4585802930648964192),
        (4596713873892699142, 4585802930648964192),
        (4592210274265328630, 4585802930648964192),
        (0, 4585802930648964182),
        (4614256656552045848, 4591506709037279728),
        (4592210274265328640, 4591506709037279728),
        (4596713873892699136, 4591506709037279728),
        (4599331010119528672, 4591506709037279728),
        (4601217473520069600, 4591506709037279728),
        (4602891378046628704, 4591506709037279728),
        (4603834609746899168, 4591506709037279728),
        (4604777841447169628, 4591506709037279728),
        (4605721073147440096, 4591506709037279729),
        (4606664304847710544, 4591506709037279729),
        (4607394977673999208, 4591506709037279729),
        (4607866593524134430, 4591506709037279729),
        (4608338209374269663, 4591506709037279729),
        (4608809825224404895, 4591506709037279729),
        (4609281441074540120, 4591506709037279729),
        (4609517248999607735, 4591506709037279729),
        (4609753056924675350, 4591506709037279729),
        (4610224672774810584, 4591506709037279729),
        (4610696288624945807, 4591506709037279729),
        (4611167904475081040, 4591506709037279729),
        (4611639520325216270, 4591506709037279729),
        (4611898577301369703, 4591506709037279729),
        (4612134385226437316, 4591506709037279729),
        (4612370193151504928, 4591506709037279729),
        (4612606001076572547, 4591506709037279730),
        (4612841809001640160, 4591506709037279730),
        (4613077616926707775, 4591506709037279730),
        (4613313424851775388, 4591506709037279730),
        (4613549232776843003, 4591506709037279730),
        (4613785040701910618, 4591506709037279730),
        (4614020848626978228, 4591506709037279730),
        (4614138752589512039, 4591506709037279730),
        (4614256656552045848, 4591506709037279730),
        (0, 4585802930648964192),
    ];
    let loop_points = LOOP_POINT_BITS
        .iter()
        .map(|(u, v)| Point2::new(f64::from_bits(*u), f64::from_bits(*v)))
        .collect::<Vec<_>>();
    let boundary = PolyBoundary {
        loops: vec![
            loop_points
                .into_iter()
                .map(|uv| SurfacePoint::from((uv, Point3::new(uv.x, uv.y, 0.0))))
                .collect(),
        ],
        uv_min: Point2::new(0.0, f64::from_bits(4585802930648964182)),
        uv_max: Point2::new(TAU * 0.5, f64::from_bits(4591506709037279730)),
    };
    let mut triangulation = Cdt::new();
    let mut boundary_map = HashMap::<FixedVertexHandle, Point3>::default();

    let (_, added_constraints, _) = boundary.insert_to(&mut triangulation, &mut boundary_map);

    assert!(added_constraints > 0);
}
