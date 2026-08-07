//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use super::{
    boundary::{PolyBoundary, PolyBoundaryPiece, SurfacePoint},
    mesh::trimming_tessellation,
    *,
};
use monstertruck_geometry::prelude::{Line, Matrix4, Plane, Processor, RevolutionSurface};

#[derive(Clone, Debug)]
struct SampledTrim {
    samples: Vec<Point2>,
}

impl ExactTrimBoundary2D for SampledTrim {
    fn exact_trim_boundary_2d(&self, _: f64) -> Vec<Point2> { self.samples.clone() }
}

fn sampled_trim(samples: impl IntoIterator<Item = Point2>) -> SampledTrim {
    SampledTrim {
        samples: samples.into_iter().collect(),
    }
}

fn square_piece(surface: &Plane, min: f64, max: f64, ccw: bool) -> PolyBoundaryPiece {
    let mut boundary = vec![
        Point2::new(min, min),
        Point2::new(max, min),
        Point2::new(max, max),
        Point2::new(min, max),
    ];
    if !ccw {
        boundary.reverse();
    }
    boundary.push(boundary[0]);
    PolyBoundaryPiece(
        boundary
            .into_iter()
            .map(|uv| SurfacePoint::from((uv, surface.subs(uv.x, uv.y))))
            .collect(),
    )
}

fn cylinder_patch_piece(surface: &RevolutionSurface<Line<Point3>>) -> PolyBoundaryPiece {
    let lower = (0..=16).map(|index| Point2::new(0.0, TAU * index as f64 / 16.0));
    let right = [Point2::new(0.5, TAU), Point2::new(1.0, TAU)];
    let upper = (0..=16)
        .rev()
        .map(|index| Point2::new(1.0, TAU * index as f64 / 16.0));
    let left = [Point2::new(0.5, 0.0), Point2::new(0.0, 0.0)];
    PolyBoundaryPiece(
        lower
            .chain(right)
            .chain(upper)
            .chain(left)
            .map(|uv| SurfacePoint::from((uv, surface.subs(uv.x, uv.y))))
            .collect(),
    )
}

fn cylinder_ring_piece(
    surface: &RevolutionSurface<Line<Point3>>,
    profile_parameter: f64,
    reversed: bool,
) -> PolyBoundaryPiece {
    let mut boundary = (0..=16)
        .map(|index| Point2::new(profile_parameter, TAU * index as f64 / 16.0))
        .chain([Point2::new(profile_parameter, 0.0)])
        .collect::<Vec<_>>();
    if reversed {
        boundary.reverse();
    }
    PolyBoundaryPiece(
        boundary
            .into_iter()
            .map(|uv| SurfacePoint::from((uv, surface.subs(uv.x, uv.y))))
            .collect(),
    )
}

fn shortened_cylinder_ring_piece(
    surface: &RevolutionSurface<Line<Point3>>,
    profile_parameter: f64,
    reversed: bool,
) -> PolyBoundaryPiece {
    let mut boundary = (0..=16)
        .map(|index| Point2::new(profile_parameter, TAU * 0.95 * index as f64 / 16.0))
        .chain([Point2::new(profile_parameter, 0.0)])
        .collect::<Vec<_>>();
    if reversed {
        boundary.reverse();
    }
    PolyBoundaryPiece(
        boundary
            .into_iter()
            .map(|uv| SurfacePoint::from((uv, surface.subs(uv.x, uv.y))))
            .collect(),
    )
}

fn step_cylinder_surface() -> Processor<RevolutionSurface<Line<Point3>>, Matrix4> {
    let profile = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0));
    let mut surface: Processor<_, Matrix4> = Processor::new(RevolutionSurface::by_revolution(
        profile,
        Point3::origin(),
        Vector3::unit_z(),
    ));
    surface.invert();
    surface
}

fn double_lap_step_cylinder_piece(
    surface: &Processor<RevolutionSurface<Line<Point3>>, Matrix4>,
) -> PolyBoundaryPiece {
    let lower = (0..=16).map(|index| Point2::new(-TAU + TAU * index as f64 / 16.0, 0.0));
    let right = [Point2::new(0.0, 1.0), Point2::new(TAU, 1.0)];
    let upper = (0..=16)
        .rev()
        .map(|index| Point2::new(TAU * index as f64 / 16.0, 1.0));
    let left = [Point2::new(0.0, 0.0), Point2::new(-TAU, 0.0)];
    PolyBoundaryPiece(
        lower
            .chain(right)
            .chain(upper)
            .chain(left)
            .map(|uv| SurfacePoint::from((uv, surface.subs(uv.x, uv.y))))
            .collect(),
    )
}

fn shortened_step_cylinder_ring_piece(
    surface: &Processor<RevolutionSurface<Line<Point3>>, Matrix4>,
    height_parameter: f64,
    reversed: bool,
) -> PolyBoundaryPiece {
    let mut boundary = (0..=16)
        .map(|index| Point2::new(TAU * 0.95 * index as f64 / 16.0, height_parameter))
        .chain([Point2::new(0.0, height_parameter)])
        .collect::<Vec<_>>();
    if reversed {
        boundary.reverse();
    }
    PolyBoundaryPiece(
        boundary
            .into_iter()
            .map(|uv| SurfacePoint::from((uv, surface.subs(uv.x, uv.y))))
            .collect(),
    )
}

fn shifted_step_cylinder_ring_piece(
    surface: &Processor<RevolutionSurface<Line<Point3>>, Matrix4>,
    height_parameter: f64,
    offset: f64,
    reversed: bool,
) -> PolyBoundaryPiece {
    let mut boundary = (0..=16)
        .map(|index| Point2::new(offset + TAU * 0.95 * index as f64 / 16.0, height_parameter))
        .chain([Point2::new(offset, height_parameter)])
        .collect::<Vec<_>>();
    if reversed {
        boundary.reverse();
    }
    PolyBoundaryPiece(
        boundary
            .into_iter()
            .map(|uv| SurfacePoint::from((uv, surface.subs(uv.x, uv.y))))
            .collect(),
    )
}

fn seam_crossing_step_cylinder_ring_piece(
    surface: &Processor<RevolutionSurface<Line<Point3>>, Matrix4>,
    height_parameter: f64,
    reversed: bool,
) -> PolyBoundaryPiece {
    seam_crossing_step_cylinder_ring_piece_from(surface, height_parameter, PI, reversed)
}

fn seam_crossing_step_cylinder_ring_piece_from(
    surface: &Processor<RevolutionSurface<Line<Point3>>, Matrix4>,
    height_parameter: f64,
    start_angle: f64,
    reversed: bool,
) -> PolyBoundaryPiece {
    let mut boundary = (0..10)
        .map(|index| {
            Point2::new(
                (start_angle - TAU * index as f64 / 10.0).rem_euclid(TAU),
                height_parameter,
            )
        })
        .chain([Point2::new(start_angle.rem_euclid(TAU), height_parameter)])
        .collect::<Vec<_>>();
    if reversed {
        boundary.reverse();
    }
    PolyBoundaryPiece(
        boundary
            .into_iter()
            .map(|uv| SurfacePoint::from((uv, surface.subs(uv.x, uv.y))))
            .collect(),
    )
}

fn seam_crossing_step_cylinder_patch_piece(
    surface: &Processor<RevolutionSurface<Line<Point3>>, Matrix4>,
) -> PolyBoundaryPiece {
    let upper = (0..=4)
        .map(|index| Point2::new((TAU - FRAC_PI_2 * index as f64 / 4.0).rem_euclid(TAU), 1.0));
    let left = [Point2::new(FRAC_PI_2 * 3.0, 0.0)];
    let lower = (0..=4)
        .rev()
        .map(|index| Point2::new((TAU - FRAC_PI_2 * index as f64 / 4.0).rem_euclid(TAU), 0.0));
    let close = [Point2::new(0.0, 1.0)];
    PolyBoundaryPiece(
        upper
            .chain(left)
            .chain(lower)
            .chain(close)
            .map(|uv| SurfacePoint::from((uv, surface.subs(uv.x, uv.y))))
            .collect(),
    )
}

#[test]
fn parameter_curve_trim_samples_curved_surface_lift() {
    let profile = Line(Point3::new(0.25, 0.0, 0.0), Point3::new(0.25, 0.0, 1.0));
    let surface = RevolutionSurface::by_revolution(profile, Point3::origin(), Vector3::unit_z());
    let trim = ParameterCurve::new(
        Line(Point2::new(0.5, 0.0), Point2::new(0.5, FRAC_PI_2)),
        surface,
    );

    let uv_points = trim.exact_trim_boundary_2d(0.01);
    let lifted = uv_points
        .iter()
        .map(|uv| trim.surface().subs(uv.x, uv.y))
        .collect::<Vec<_>>();
    let length = lifted
        .windows(2)
        .map(|window| window[0].distance(window[1]))
        .sum::<f64>();
    let direct = lifted
        .first()
        .zip(lifted.last())
        .map(|(front, back)| front.distance(*back))
        .unwrap_or(0.0);

    assert!(
        uv_points.len() > 4,
        "curved lifted trims need interior UV samples",
    );
    assert!(length / direct > 1.05);
}

#[test]
fn shared_polyline_preserves_exact_trim_samples() {
    let profile = Line(Point3::new(2.5, 0.0, 0.0), Point3::new(2.5, 0.0, 1.0));
    let mut surface: Processor<_, Matrix4> = Processor::new(RevolutionSurface::by_revolution(
        profile,
        Point3::origin(),
        Vector3::unit_z(),
    ));
    surface.invert();
    let trim = ParameterCurve::new(
        Line(Point2::new(TAU * 0.75, 8.0), Point2::new(-TAU * 0.25, 8.0)),
        surface,
    );
    let trim_points = trim.exact_trim_boundary_2d(0.138564065);
    let polyline = polyline_from_trim_curve(
        trim.surface(),
        &trim,
        (0, 1),
        &[
            trim.surface().subs(TAU * 0.75, 8.0),
            trim.surface().subs(-TAU * 0.25, 8.0),
        ],
        0.138564065,
    )
    .expect("exact trim should produce a shared edge polyline");

    assert_eq!(polyline.len(), trim_points.len());
}

#[test]
fn shared_polyline_densifies_topological_edge_curve() {
    let surface = Plane::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    let shell = CompressedTrimmedShell {
        vertices: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        edges: vec![CompressedEdge {
            vertices: (0, 1),
            curve: Line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
        }],
        faces: vec![CompressedTrimmedFace {
            boundaries: vec![vec![CompressedEdgeUse::from((
                0,
                true,
                Some(sampled_trim([
                    Point2::new(0.0, 0.0),
                    Point2::new(0.5, 1.0),
                    Point2::new(1.0, 0.0),
                ])),
            ))]],
            orientation: true,
            surface,
        }],
    };

    let meshed = trimmed_cshell_tessellation(
        &shell,
        0.01,
        |surface: &Plane, point: Point3, _| surface.search_parameter(point, None, 100),
        TessellationPrimitiveOptions::default(),
    );
    let middle = meshed.edges[0].curve[1];

    assert_eq!(meshed.edges[0].curve.len(), 3);
    assert!((middle.x - 0.5).abs() <= TOLERANCE);
    assert!(middle.y.abs() <= TOLERANCE);
}

#[test]
fn trimmed_cshell_reuses_shared_edge_samples_for_exact_trims() {
    let profile = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0));
    let surface = RevolutionSurface::by_revolution(profile, Point3::origin(), Vector3::unit_z());
    let curve_on_surface = |front, back| ParameterCurve::new(Line(front, back), surface);
    let vertices = vec![
        surface.subs(0.0, 0.0),
        surface.subs(0.0, FRAC_PI_2),
        surface.subs(1.0, FRAC_PI_2),
        surface.subs(1.0, 0.0),
    ];
    let edges = vec![
        CompressedEdge {
            vertices: (0, 1),
            curve: curve_on_surface(Point2::new(0.0, 0.0), Point2::new(0.0, FRAC_PI_2)),
        },
        CompressedEdge {
            vertices: (1, 2),
            curve: curve_on_surface(Point2::new(0.0, FRAC_PI_2), Point2::new(1.0, FRAC_PI_2)),
        },
        CompressedEdge {
            vertices: (2, 3),
            curve: curve_on_surface(Point2::new(1.0, FRAC_PI_2), Point2::new(1.0, 0.0)),
        },
        CompressedEdge {
            vertices: (3, 0),
            curve: curve_on_surface(Point2::new(1.0, 0.0), Point2::new(0.0, 0.0)),
        },
    ];
    let right = sampled_trim([Point2::new(0.0, FRAC_PI_2), Point2::new(1.0, FRAC_PI_2)]);
    let top = sampled_trim([Point2::new(1.0, FRAC_PI_2), Point2::new(1.0, 0.0)]);
    let left = sampled_trim([Point2::new(1.0, 0.0), Point2::new(0.0, 0.0)]);
    let face_with_bottom = |bottom| CompressedTrimmedFace {
        boundaries: vec![vec![
            CompressedEdgeUse::from((0, true, Some(bottom))),
            CompressedEdgeUse::from((1, true, Some(right.clone()))),
            CompressedEdgeUse::from((2, true, Some(top.clone()))),
            CompressedEdgeUse::from((3, true, Some(left.clone()))),
        ]],
        orientation: true,
        surface,
    };
    let shell = CompressedTrimmedShell {
        vertices,
        edges,
        faces: vec![
            face_with_bottom(sampled_trim([
                Point2::new(0.0, 0.0),
                Point2::new(0.0, FRAC_PI_2 * 0.5),
                Point2::new(0.0, FRAC_PI_2),
            ])),
            face_with_bottom(sampled_trim([
                Point2::new(0.0, 0.0),
                Point2::new(0.0, FRAC_PI_2 * 0.25),
                Point2::new(0.0, FRAC_PI_2 * 0.5),
                Point2::new(0.0, FRAC_PI_2 * 0.75),
                Point2::new(0.0, FRAC_PI_2),
            ])),
        ],
    };

    let meshed = trimmed_cshell_tessellation(
        &shell,
        0.01,
        |_: &RevolutionSurface<Line<Point3>>, point: Point3, _| {
            Some((point.z, point.y.atan2(point.x)))
        },
        TessellationPrimitiveOptions::default(),
    );
    let shared_edge_count = meshed.edges[0].curve.len();
    let second_face = meshed.faces[1]
        .surface
        .as_ref()
        .expect("trimmed face should mesh");
    let bottom_angles = second_face
        .positions()
        .iter()
        .filter(|point| point.z.abs() <= 1.0e-8)
        .map(|point| point.y.atan2(point.x))
        .fold(Vec::<f64>::new(), |mut acc, x| {
            if !acc.iter().any(|known| (known - x).abs() <= 1.0e-8) {
                acc.push(x);
            }
            acc
        });

    assert!(shared_edge_count >= 5);
    assert_eq!(bottom_angles.len(), shared_edge_count);
}

#[test]
fn trimmed_shell_isoparams_stay_inside_trim_domain() {
    let surface = Plane::new(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    let mut vertices = Vec::<Point3>::new();
    let mut edges = Vec::<CompressedEdge<Line<Point3>>>::new();
    let mut edge_use = |front: Point2, back: Point2| {
        let front_point = surface.evaluate(front.x, front.y);
        let back_point = surface.evaluate(back.x, back.y);
        let vertex_index = vertices.len();
        let edge_index = edges.len();
        vertices.extend([front_point, back_point]);
        edges.push(CompressedEdge {
            vertices: (vertex_index, vertex_index + 1),
            curve: Line(front_point, back_point),
        });
        CompressedEdgeUse::from((edge_index, true, Some(sampled_trim([front, back]))))
    };
    let outer = vec![
        edge_use(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0)),
        edge_use(Point2::new(2.0, 0.0), Point2::new(2.0, 2.0)),
        edge_use(Point2::new(2.0, 2.0), Point2::new(0.0, 2.0)),
        edge_use(Point2::new(0.0, 2.0), Point2::new(0.0, 0.0)),
    ];
    let hole = vec![
        edge_use(Point2::new(0.75, 0.75), Point2::new(0.75, 1.25)),
        edge_use(Point2::new(0.75, 1.25), Point2::new(1.25, 1.25)),
        edge_use(Point2::new(1.25, 1.25), Point2::new(1.25, 0.75)),
        edge_use(Point2::new(1.25, 0.75), Point2::new(0.75, 0.75)),
    ];
    let shell = CompressedTrimmedShell {
        vertices,
        edges,
        faces: vec![CompressedTrimmedFace {
            boundaries: vec![outer, hole],
            orientation: true,
            surface,
        }],
    };

    let output = compressed_trimmed_shell_tessellation_with_isoparams(
        &shell,
        0.01,
        |surface: &Plane, point: Point3, _| surface.search_parameter(point, None, 100),
        TessellationPrimitiveOptions::default(),
        Some(IsoparametricCurveOptions {
            samples_per_direction: 3,
            segments_per_curve: 32,
        }),
    );
    let curves = &output.face_isoparams[0];

    assert!(!curves.is_empty());
    assert!(output.shell.faces[0].surface.is_some());
    assert!(curves.iter().flatten().all(|point| {
        let inside_outer = point.x >= -1.0e-6
            && point.x <= 2.0 + 1.0e-6
            && point.y >= -1.0e-6
            && point.y <= 2.0 + 1.0e-6;
        let inside_hole = point.x > 0.75 + 1.0e-5
            && point.x < 1.25 - 1.0e-5
            && point.y > 0.75 + 1.0e-5
            && point.y < 1.25 - 1.0e-5;
        inside_outer && !inside_hole
    }));
}

#[test]
fn trimmed_cylinder_patch_inserts_surface_rows_for_ruled_axis() {
    let profile = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 10.0));
    let surface = RevolutionSurface::by_revolution(profile, Point3::origin(), Vector3::unit_z());
    let boundary = PolyBoundary::new(vec![cylinder_patch_piece(&surface)], &surface, 0.01);

    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert!(
        mesh.uv_coords()
            .iter()
            .any(|uv| uv.x > 0.01 && uv.x < 0.99 && uv.y > 0.01 && uv.y < TAU - 0.01),
        "trimmed cylinder mesh should include interior surface vertices",
    );
}

#[test]
fn periodic_full_rings_pair_into_cylinder_strip() {
    let profile = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 10.0));
    let surface = RevolutionSurface::by_revolution(profile, Point3::origin(), Vector3::unit_z());
    let boundary = PolyBoundary::new(
        vec![
            cylinder_ring_piece(&surface, 0.0, false),
            cylinder_ring_piece(&surface, 1.0, true),
        ],
        &surface,
        0.01,
    );

    assert_eq!(boundary.loops.len(), 1);
    assert!(boundary.include(Point2::new(0.5, TAU * 0.5)));

    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert!(!mesh.tri_faces().is_empty());
}

#[test]
fn periodic_ring_closed_by_wrap_completes_to_full_period() {
    let profile = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 10.0));
    let surface = RevolutionSurface::by_revolution(profile, Point3::origin(), Vector3::unit_z());
    let boundary = PolyBoundary::new(
        vec![
            shortened_cylinder_ring_piece(&surface, 0.0, false),
            shortened_cylinder_ring_piece(&surface, 1.0, true),
        ],
        &surface,
        0.01,
    );

    assert!(
        boundary.uv_max.y - boundary.uv_min.y >= TAU - 0.01,
        "wrapped full rings should open to one complete period",
    );
    assert!(
        boundary.uv_max.y - boundary.uv_min.y <= TAU + 0.01,
        "paired wrapped rings should not span multiple periods",
    );
    let middle_v = (boundary.uv_min.y + boundary.uv_max.y) * 0.5;
    assert!(boundary.include(Point2::new(0.5, middle_v)));

    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert!(mesh.tri_faces().len() > 16);
}

#[test]
fn step_cylinder_ring_pair_compacts_to_one_period() {
    let surface = step_cylinder_surface();
    let boundary = PolyBoundary::new(
        vec![
            shortened_step_cylinder_ring_piece(&surface, 0.0, false),
            shortened_step_cylinder_ring_piece(&surface, 1.0, true),
        ],
        &surface,
        0.01,
    );

    assert!(
        boundary.uv_max.x - boundary.uv_min.x <= TAU + 0.01,
        "STEP-style paired rings should not span multiple angular periods",
    );
    let middle_u = (boundary.uv_min.x + boundary.uv_max.x) * 0.5;
    assert!(boundary.include(Point2::new(middle_u, 0.5)));

    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert!(mesh.tri_faces().len() > 16);
}

#[test]
fn shifted_step_cylinder_ring_pair_compacts_to_one_period() {
    let surface = step_cylinder_surface();
    let boundary = PolyBoundary::new(
        vec![
            shifted_step_cylinder_ring_piece(&surface, 0.0, 0.0, false),
            shifted_step_cylinder_ring_piece(&surface, 1.0, TAU, true),
        ],
        &surface,
        0.01,
    );

    assert!(
        boundary.uv_max.x - boundary.uv_min.x <= TAU + 0.01,
        "shifted STEP-style paired rings should not span multiple angular periods",
    );
    let middle_u = (boundary.uv_min.x + boundary.uv_max.x) * 0.5;
    assert!(boundary.include(Point2::new(middle_u, 0.5)));

    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert!(mesh.tri_faces().len() > 16);
}

#[test]
fn seam_crossing_step_cylinder_ring_pair_compacts_to_one_period() {
    let surface = step_cylinder_surface();
    let boundary = PolyBoundary::new(
        vec![
            seam_crossing_step_cylinder_ring_piece(&surface, 0.0, false),
            seam_crossing_step_cylinder_ring_piece(&surface, 1.0, true),
        ],
        &surface,
        0.01,
    );

    assert!(
        boundary.uv_max.x - boundary.uv_min.x <= TAU + 0.01,
        "seam-crossing STEP-style paired rings should not span multiple angular periods",
    );
    let middle_u = (boundary.uv_min.x + boundary.uv_max.x) * 0.5;
    assert!(boundary.include(Point2::new(middle_u, 0.5)));

    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert!(mesh.tri_faces().len() > 16);
}

#[test]
fn mismatched_seam_step_cylinder_ring_pair_compacts_to_one_period() {
    let surface = step_cylinder_surface();
    let boundary = PolyBoundary::new(
        vec![
            seam_crossing_step_cylinder_ring_piece_from(&surface, 0.0, FRAC_PI_2 * 3.0, false),
            seam_crossing_step_cylinder_ring_piece_from(&surface, 1.0, 0.0, true),
        ],
        &surface,
        0.01,
    );

    assert!(
        boundary.uv_max.x - boundary.uv_min.x <= TAU + 0.01,
        "paired rings with mismatched seam starts should not span multiple angular periods",
    );
    let middle_u = (boundary.uv_min.x + boundary.uv_max.x) * 0.5;
    assert!(boundary.include(Point2::new(middle_u, 0.5)));

    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert!(mesh.tri_faces().len() > 16);
}

#[test]
fn partial_periodic_loop_crossing_seam_compacts_to_short_arc() {
    let surface = step_cylinder_surface();
    let boundary = PolyBoundary::new(
        vec![seam_crossing_step_cylinder_patch_piece(&surface)],
        &surface,
        0.01,
    );

    assert!(
        boundary.uv_max.x - boundary.uv_min.x <= FRAC_PI_2 + 0.01,
        "partial periodic loops crossing the seam should keep the short angular arc",
    );
    let middle_u = (boundary.uv_min.x + boundary.uv_max.x) * 0.5;
    assert!(boundary.include(Point2::new(middle_u, 0.5)));

    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert!(mesh.tri_faces().len() > 16);
}

#[test]
fn periodic_cylinder_loop_spanning_two_laps_collapses_to_one_lap() {
    let surface = step_cylinder_surface();
    let boundary = PolyBoundary::new(
        vec![double_lap_step_cylinder_piece(&surface)],
        &surface,
        0.01,
    );

    assert!(
        boundary.uv_max.x - boundary.uv_min.x <= TAU + 0.01,
        "periodic cylinder trim should not span two angular laps",
    );
    assert!(boundary.include(Point2::new(0.5 * TAU, 0.5)));

    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert!(!mesh.tri_faces().is_empty());
}

#[test]
fn trimming_tessellation_is_invariant_to_annulus_loop_winding() {
    let surface = Plane::xy();
    let canonical = PolyBoundary::new(
        vec![
            square_piece(&surface, 0.0, 1.0, true),
            square_piece(&surface, 0.25, 0.75, false),
        ],
        &surface,
        0.01,
    );
    let reversed = PolyBoundary::new(
        vec![
            square_piece(&surface, 0.0, 1.0, false),
            square_piece(&surface, 0.25, 0.75, true),
        ],
        &surface,
        0.01,
    );

    assert!(canonical.include(Point2::new(0.1, 0.1)));
    assert!(!canonical.include(Point2::new(0.5, 0.5)));
    assert!(reversed.include(Point2::new(0.1, 0.1)));
    assert!(!reversed.include(Point2::new(0.5, 0.5)));

    let canonical_mesh = trimming_tessellation(
        &surface,
        &canonical,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );
    let reversed_mesh = trimming_tessellation(
        &surface,
        &reversed,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert_eq!(
        canonical_mesh.tri_faces().len(),
        reversed_mesh.tri_faces().len(),
    );
    assert!(!canonical_mesh.tri_faces().is_empty());
}

#[test]
fn trimming_tessellation_is_invariant_to_single_loop_winding() {
    let surface = Plane::xy();
    let forward = PolyBoundary::new(vec![square_piece(&surface, 0.0, 1.0, true)], &surface, 0.01);
    let reversed = PolyBoundary::new(
        vec![square_piece(&surface, 0.0, 1.0, false)],
        &surface,
        0.01,
    );

    assert!(forward.include(Point2::new(0.5, 0.5)));
    assert!(reversed.include(Point2::new(0.5, 0.5)));

    let forward_mesh = trimming_tessellation(
        &surface,
        &forward,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );
    let reversed_mesh = trimming_tessellation(
        &surface,
        &reversed,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );

    assert_eq!(
        forward_mesh.tri_faces().len(),
        reversed_mesh.tri_faces().len()
    );
    assert!(!forward_mesh.tri_faces().is_empty());
}

// --- spec 007 D1: loud tessellation face-drop diagnostic ---------------

#[test]
fn classify_face_drop_maps_each_silent_drop() {
    use super::diagnostics::classify_face_drop;

    // `None` mesh on an untrimmed face -> the surface had no bounded domain.
    assert_eq!(
        classify_face_drop(None, true),
        Some(FaceDropReason::UnboundedDomain),
    );
    // `None` mesh on a trimmed face -> a boundary loop would not project
    // (the revolve-pole / periodic-seam / degenerate-trim family).
    assert_eq!(
        classify_face_drop(None, false),
        Some(FaceDropReason::BoundaryProjectionFailed),
    );
    // `Some` but empty -> a silently empty tessellation, on either path.
    let empty = PolygonMesh::default();
    assert_eq!(
        classify_face_drop(Some(&empty), false),
        Some(FaceDropReason::EmptyTessellation),
    );
    assert_eq!(
        classify_face_drop(Some(&empty), true),
        Some(FaceDropReason::EmptyTessellation),
    );

    // A real, non-empty tessellation is not a drop.
    let profile = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 10.0));
    let surface = RevolutionSurface::by_revolution(profile, Point3::origin(), Vector3::unit_z());
    let boundary = PolyBoundary::new(vec![cylinder_patch_piece(&surface)], &surface, 0.01);
    let mesh = trimming_tessellation(
        &surface,
        &boundary,
        0.01,
        TessellationPrimitiveOptions::default(),
        usize::MAX,
    );
    assert!(!mesh.faces().is_empty());
    assert_eq!(classify_face_drop(Some(&mesh), false), None);
}

#[test]
fn observe_face_drop_advances_the_loud_metric() {
    use super::diagnostics::observe_face_drop;

    let profile = Line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 10.0));
    let surface = RevolutionSurface::by_revolution(profile, Point3::origin(), Vector3::unit_z());

    // The metric is process-global and monotonic within a run, so a strict
    // increase across our own dropping call proves the signal fired even if
    // other parallel tests advance it too.
    let before = face_drop_count();
    observe_face_drop(&surface, Some(7), None, false, 1, 4);
    assert!(
        face_drop_count() > before,
        "a boundary-projection drop must advance the face-drop metric",
    );
}
