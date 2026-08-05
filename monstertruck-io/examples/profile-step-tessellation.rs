//! Profiles STEP tessellation at an explicit tolerance factor.

use clap::Parser;
use monstertruck_io::step::load::{step_geometry::*, *};
use monstertruck_meshing::prelude::*;
use monstertruck_topology::compress::*;
use std::path::PathBuf;
use std::time::Instant;

type CShell = CompressedTrimmedShell<Point3, Curve3D, Surface, StepParameterCurve>;

#[derive(Parser, Debug)]
struct Args {
    /// Input STEP file.
    input_step_file: PathBuf,
    /// Relative tolerance factor multiplied by shell bbox diameter.
    #[arg(long, default_value_t = 0.001)]
    tolerance_factor: f64,
    /// Optional shell index filter.
    #[arg(long)]
    shell: Option<usize>,
    /// Optional face indices to inspect before meshing.
    #[arg(long, value_delimiter = ',')]
    inspect_faces: Vec<usize>,
    /// Tessellation mode.
    #[arg(long, default_value = "robust")]
    mode: TessellationMode,
    /// Report shared-edge vs face-boundary drift after meshing.
    #[arg(long)]
    check_boundaries: bool,
}

#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum TessellationMode {
    Plain,
    Robust,
}

fn curve_kind(curve: &Curve3D) -> &'static str {
    match curve {
        Curve3D::Line(_) => "Line",
        Curve3D::Polyline(_) => "Polyline",
        Curve3D::Conic(_) => "Conic",
        Curve3D::BsplineCurve(_) => "BsplineCurve",
        Curve3D::ParameterCurve(_) => "ParameterCurve",
        Curve3D::SurfaceCurve(_) => "SurfaceCurve",
        Curve3D::IntersectionCurve(_) => "IntersectionCurve",
        Curve3D::NurbsCurve(_) => "NurbsCurve",
    }
}

fn surface_kind(surface: &Surface) -> &'static str {
    match surface {
        Surface::ElementarySurface(ElementarySurface::Plane(_)) => "Plane",
        Surface::ElementarySurface(ElementarySurface::Sphere(_)) => "Sphere",
        Surface::ElementarySurface(ElementarySurface::CylindricalSurface(_)) => {
            "CylindricalSurface"
        }
        Surface::ElementarySurface(ElementarySurface::ToroidalSurface(_)) => "ToroidalSurface",
        Surface::ElementarySurface(ElementarySurface::ConicalSurface(_)) => "ConicalSurface",
        Surface::SweepSurface(SweepSurface::ExtrusionSurface(_)) => "ExtrusionSurface",
        Surface::SweepSurface(SweepSurface::RevolutionSurface(_)) => "RevolutionSurface",
        Surface::BsplineSurface(_) => "BsplineSurface",
        Surface::NurbsSurface(_) => "NurbsSurface",
    }
}

fn shell_bounding_box(shell: &CShell) -> BoundingBox<Point3> {
    let mut bdd: BoundingBox<Point3> = shell.vertices.iter().copied().collect();
    shell.edges.iter().for_each(|edge| {
        let (t0, t1) = edge.curve.range_tuple();
        (0..=4).for_each(|i| {
            let t = t0 + (t1 - t0) * i as f64 / 4.0;
            bdd.push(edge.curve.subs(t));
        });
    });
    bdd
}

fn point_segment_distance(point: Point3, front: Point3, back: Point3) -> f64 {
    let segment = back - front;
    let denom = segment.dot(segment);
    if denom.so_small() {
        point.distance(front)
    } else {
        let t = ((point - front).dot(segment) / denom).clamp(0.0, 1.0);
        let nearest = front + segment * t;
        point.distance(nearest)
    }
}

fn mesh_boundary_segments(mesh: &PolygonMesh) -> Vec<(Point3, Point3)> {
    let positions = mesh.positions();
    let mut edge_counts = std::collections::BTreeMap::<(usize, usize), Vec<(usize, usize)>>::new();
    mesh.tri_faces().iter().for_each(|tri| {
        let indices = [tri[0].pos, tri[1].pos, tri[2].pos];
        (0..3).for_each(|i| {
            let front = indices[i];
            let back = indices[(i + 1) % 3];
            let key = if front < back {
                (front, back)
            } else {
                (back, front)
            };
            edge_counts.entry(key).or_default().push((front, back));
        });
    });
    edge_counts
        .into_iter()
        .filter(|(_, occurrences)| occurrences.len() == 1)
        .map(|(_, occurrences)| occurrences[0])
        .map(|(front, back)| (positions[front], positions[back]))
        .collect()
}

fn report_boundary_drift(
    shell_idx: usize,
    shell: &CompressedShell<Point3, PolylineCurve<Point3>, Option<PolygonMesh>>,
) {
    let face_segments = shell
        .faces
        .iter()
        .map(|face| {
            face.surface
                .as_ref()
                .map(mesh_boundary_segments)
                .unwrap_or_default()
        })
        .collect::<Vec<_>>();
    shell.edges.iter().enumerate().for_each(|(edge_idx, edge)| {
        let incident_faces = shell
            .faces
            .iter()
            .enumerate()
            .filter(|(_, face)| {
                face
                    .boundaries
                    .iter()
                    .flatten()
                    .any(|boundary_edge| boundary_edge.index == edge_idx)
            })
            .map(|(face_idx, _)| face_idx)
            .collect::<Vec<_>>();
        let relevant_segments = shell
            .faces
            .iter()
            .enumerate()
            .filter(|(face_idx, _)| incident_faces.contains(face_idx))
            .flat_map(|(face_idx, _)| face_segments[face_idx].iter().copied())
            .collect::<Vec<_>>();
        if relevant_segments.is_empty() {
            return;
        }
        let max_drift = edge
            .curve
            .iter()
            .copied()
            .map(|point| {
                relevant_segments
                    .iter()
                    .map(|(front, back)| point_segment_distance(point, *front, *back))
                    .min_by(|lhs, rhs| lhs.total_cmp(rhs))
                    .unwrap_or(f64::INFINITY)
            })
            .max_by(|lhs, rhs| lhs.total_cmp(rhs))
            .unwrap_or(0.0);
        if max_drift > 1.0e-6 {
            eprintln!(
                "boundary-drift shell={shell_idx} edge={edge_idx} edge_points={} incident_faces={incident_faces:?} segment_count={} max_drift={max_drift:.9}",
                edge.curve.len(),
                relevant_segments.len(),
            );
        }
    });
}

fn main() {
    let Args {
        input_step_file,
        tolerance_factor,
        shell,
        inspect_faces,
        mode,
        check_boundaries,
    } = Args::parse();
    let step_file = std::fs::read_to_string(&input_step_file).unwrap();
    let table = Table::from_step(&step_file).unwrap();
    let assy = table.step_assy().unwrap();
    let mut shells = assy
        .top_nodes()
        .flat_map(|top| assy.paths_iter(top.index()))
        .flat_map(|path| {
            path.terminal_node()
                .shape()
                .iter()
                .filter_map(|idx| {
                    if let Some(step_solid) = table.manifold_solid_brep.get(idx) {
                        table
                            .to_compressed_trimmed_solid(step_solid)
                            .ok()
                            .map(|solid| solid.boundaries)
                    } else if let Some(step_shells) = table.shell_based_surface_model.get(idx) {
                        table.to_compressed_trimmed_shells(step_shells).ok()
                    } else {
                        None
                    }
                })
                .flatten()
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    if shells.is_empty() {
        shells = table
            .shell
            .values()
            .filter_map(|shell| table.to_compressed_trimmed_shell(shell).ok())
            .collect();
    }
    shells.sort_by_key(|shell| shell.faces.len());
    shells
        .into_iter()
        .enumerate()
        .filter(|(shell_idx, _)| shell.is_none_or(|target| *shell_idx == target))
        .for_each(|(shell_idx, shell)| {
            let diameter = shell_bounding_box(&shell).diameter();
            let tolerance = f64::max(diameter * tolerance_factor, TOLERANCE);
            eprintln!(
                "profile shell={shell_idx} faces={} diameter={diameter:.6} tolerance={tolerance:.9}",
                shell.faces.len(),
            );
            inspect_faces.iter().for_each(|face_idx| {
                if let Some(face) = shell.faces.get(*face_idx) {
                    eprintln!(
                        "inspect shell={shell_idx} face={face_idx} surface={} loops={}",
                        surface_kind(&face.surface),
                        face.boundaries.len(),
                    );
                    face.boundaries.iter().enumerate().for_each(|(wire_idx, wire)| {
                        let kinds = wire
                            .iter()
                            .filter_map(|edge_idx| shell.edges.get(edge_idx.index))
                            .map(|edge| curve_kind(&edge.curve))
                            .collect::<Vec<_>>();
                        eprintln!(
                            "inspect shell={shell_idx} face={face_idx} wire={wire_idx} edge_kinds={kinds:?}",
                        );
                        wire.iter()
                            .enumerate()
                            .for_each(|(edge_pos, edge_idx)| {
                                let Some(edge) = shell.edges.get(edge_idx.index) else {
                                    return;
                                };
                                let boundary =
                                    edge.curve.parameter_boundary_2d(&face.surface, tolerance);
                                let boundary_points = boundary
                                    .as_ref()
                                    .map_or(0, std::vec::Vec::len);
                                let boundary_front = boundary
                                    .as_ref()
                                    .and_then(|boundary| boundary.first().copied())
                                    .map(|uv| format!("({:.6},{:.6})", uv.x, uv.y))
                                    .unwrap_or_else(|| "-".into());
                                let boundary_back = boundary
                                    .as_ref()
                                    .and_then(|boundary| boundary.last().copied())
                                    .map(|uv| format!("({:.6},{:.6})", uv.x, uv.y))
                                    .unwrap_or_else(|| "-".into());
                                match &edge.curve {
                                    Curve3D::SurfaceCurve(curve) => eprintln!(
                                        "inspect shell={shell_idx} face={face_idx} wire={wire_idx} edge={edge_pos} edge_index={} exact={} boundary_points={} edge_curve_points={} boundary_front={} boundary_back={} assoc={} face_surface={}",
                                        edge_idx.index,
                                        boundary.is_some(),
                                        boundary_points,
                                        edge.curve.parameter_division(edge.curve.range_tuple(), tolerance).1.len(),
                                        boundary_front,
                                        boundary_back,
                                        curve.associated_geometry().len(),
                                        surface_kind(&face.surface),
                                    ),
                                    Curve3D::IntersectionCurve(curve) => eprintln!(
                                        "inspect shell={shell_idx} face={face_idx} wire={wire_idx} edge={edge_pos} edge_index={} exact={} boundary_points={} edge_curve_points={} boundary_front={} boundary_back={} s0={} s1={} face_surface={}",
                                        edge_idx.index,
                                        boundary.is_some(),
                                        boundary_points,
                                        edge.curve.parameter_division(edge.curve.range_tuple(), tolerance).1.len(),
                                        boundary_front,
                                        boundary_back,
                                        surface_kind(curve.surface0().as_ref()),
                                        surface_kind(curve.surface1().as_ref()),
                                        surface_kind(&face.surface),
                                    ),
                                    _ => eprintln!(
                                        "inspect shell={shell_idx} face={face_idx} wire={wire_idx} edge={edge_pos} edge_index={} orientation={} exact={} boundary_points={boundary_points} edge_curve_points={} boundary_front={} boundary_back={}",
                                        edge_idx.index,
                                        edge_idx.orientation,
                                        boundary.is_some(),
                                        edge.curve.parameter_division(edge.curve.range_tuple(), tolerance).1.len(),
                                        boundary_front,
                                        boundary_back,
                                    ),
                                }
                                if boundary.is_none() {
                                    edge.curve
                                        .parameter_division(edge.curve.range_tuple(), tolerance)
                                        .1
                                        .into_iter()
                                        .enumerate()
                                        .for_each(|(sample_idx, point)| {
                                            let direct = face
                                                .surface
                                                .search_parameter(point, None, 100)
                                                .map(|(u, v)| format!("({u:.6},{v:.6})"))
                                                .unwrap_or_else(|| "-".into());
                                            let nearest = face
                                                .surface
                                                .search_nearest_parameter(point, None, 100)
                                                .map(|(u, v)| format!("({u:.6},{v:.6})"))
                                                .unwrap_or_else(|| "-".into());
                                            eprintln!(
                                                "inspect shell={shell_idx} face={face_idx} wire={wire_idx} edge={edge_pos} sample={sample_idx} point=({:.6},{:.6},{:.6}) direct={} nearest={}",
                                                point.x,
                                                point.y,
                                                point.z,
                                                direct,
                                                nearest,
                                            );
                                        });
                                }
                            });
                    });
                }
            });
            let start = Instant::now();
            let meshed = match mode {
                TessellationMode::Plain => shell.triangulation(tolerance),
                TessellationMode::Robust => shell.robust_triangulation(tolerance),
            };
            let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
            let meshed_faces = meshed
                .faces
                .iter()
                .filter(|face| face.surface.is_some())
                .count();
            eprintln!(
                "profile shell={shell_idx} mode={mode:?} elapsed_ms={elapsed_ms:.3} meshed_faces={meshed_faces}/{}",
                meshed.faces.len(),
            );
            if check_boundaries {
                report_boundary_drift(shell_idx, &meshed);
            }
        });
}
