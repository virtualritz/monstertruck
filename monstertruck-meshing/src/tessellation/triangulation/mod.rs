#![allow(clippy::many_single_char_names)]

use super::*;
use crate::Point2;
use crate::filters::StructuringFilter;
use array_macro::array;
use handles::FixedVertexHandle;
use itertools::{Either, Itertools};
use monstertruck_geometry::prelude::ParameterCurve;
use rustc_hash::FxHashMap as HashMap;
use std::{collections::hash_map::Entry, env, iter};
// `std::time::Instant::now()` panics on `wasm32-unknown-unknown`.
// `web_time::Instant` is std-compatible on native and uses
// `performance.now()` in the browser.
use web_time::Instant;

mod boundary;
mod diagnostics;
mod mesh;

use boundary::{PolyBoundary, PolyBoundaryPiece};
pub(crate) use diagnostics::classify_face_drop;
use diagnostics::observe_face_drop;
pub use diagnostics::{FaceDropReason, face_drop_count, reset_face_drop_count};
use mesh::{trimming_tessellation, untrimmed_tessellation};

type SPoint2 = spade::Point2<f64>;
type Cdt = ConstrainedDelaunayTriangulation<SPoint2>;
type MeshedShell = Shell<Point3, PolylineCurve, Option<PolygonMesh>>;
type MeshedCompressedShell = CompressedShell<Point3, PolylineCurve, Option<PolygonMesh>>;
type TrimmedShell<C, S, T> = CompressedTrimmedShell<Point3, C, S, T>;

const MAX_LIFTED_TRIM_SUBDIVISION_DEPTH: usize = 16;

/// Provides exact trim boundaries in surface parameter space.
pub trait ExactTrimBoundary2D {
    /// Samples the trim boundary in surface parameter space.
    fn exact_trim_boundary_2d(&self, tolerance: f64) -> Vec<Point2>;

    /// Projects a point on the spatial boundary back onto this trim boundary.
    fn project_boundary_point(&self, _point: Point3, _hint: Option<f64>) -> Option<(f64, Point2)> {
        None
    }
}

impl<C, S> ExactTrimBoundary2D for ParameterCurve<C, S>
where
    C: ParametricCurve2D + BoundedCurve + ParameterDivision1D<Point = Point2>,
    S: ParametricSurface3D,
    ParameterCurve<C, S>: SearchParameter<CurveParameter, Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>,
{
    fn exact_trim_boundary_2d(&self, tolerance: f64) -> Vec<Point2> {
        lifted_trim_boundary_2d(self, tolerance)
    }

    fn project_boundary_point(&self, point: Point3, hint: Option<f64>) -> Option<(f64, Point2)> {
        self.search_parameter(point, hint, 100)
            .or_else(|| self.search_nearest_parameter(point, hint, 100))
            .or_else(|| self.search_parameter(point, None, 100))
            .or_else(|| self.search_nearest_parameter(point, None, 100))
            .map(|t| (t, self.curve().subs(t)))
    }
}

fn lifted_trim_boundary_2d<C, S>(trim_curve: &ParameterCurve<C, S>, tolerance: f64) -> Vec<Point2>
where
    C: ParametricCurve2D + BoundedCurve + ParameterDivision1D<Point = Point2>,
    S: ParametricSurface3D, {
    let (parameters, boundary) = trim_curve
        .curve()
        .parameter_division(trim_curve.curve().range_tuple(), tolerance);
    if parameters.len() != boundary.len() || boundary.len() <= 1 {
        boundary
    } else {
        let mut refined = Vec::with_capacity(boundary.len());
        refined.push(boundary[0]);
        parameters
            .windows(2)
            .zip(boundary.windows(2))
            .for_each(|(parameters, boundary)| {
                append_lifted_trim_segment(
                    trim_curve,
                    (parameters[0], boundary[0]),
                    (parameters[1], boundary[1]),
                    tolerance * tolerance,
                    0,
                    &mut refined,
                );
            });
        refined
    }
}

fn append_lifted_trim_segment<C, S>(
    trim_curve: &ParameterCurve<C, S>,
    front: (f64, Point2),
    back: (f64, Point2),
    tolerance2: f64,
    depth: usize,
    refined: &mut Vec<Point2>,
) where
    C: ParametricCurve2D,
    S: ParametricSurface3D,
{
    let (front_parameter, front_uv) = front;
    let (back_parameter, back_uv) = back;
    if depth >= MAX_LIFTED_TRIM_SUBDIVISION_DEPTH || (back_parameter - front_parameter).so_small() {
        refined.push(back_uv);
    } else {
        let middle_parameter = (front_parameter + back_parameter) * 0.5;
        let middle_uv = trim_curve.curve().subs(middle_parameter);
        let front_point = trim_curve.surface().subs(front_uv.x, front_uv.y);
        let middle_point = trim_curve.surface().subs(middle_uv.x, middle_uv.y);
        let back_point = trim_curve.surface().subs(back_uv.x, back_uv.y);
        if point_segment_distance2(middle_point, front_point, back_point) > tolerance2 {
            append_lifted_trim_segment(
                trim_curve,
                front,
                (middle_parameter, middle_uv),
                tolerance2,
                depth + 1,
                refined,
            );
            append_lifted_trim_segment(
                trim_curve,
                (middle_parameter, middle_uv),
                back,
                tolerance2,
                depth + 1,
                refined,
            );
        } else {
            refined.push(back_uv);
        }
    }
}

fn mesh_trace_enabled() -> bool { env::var_os("MT_MESH_TRACE").is_some() }

fn boundary_tolerance_candidates(tolerance: f64) -> Vec<f64> {
    iter::successors(Some(tolerance), |current| {
        let next = *current * 0.5;
        (next > TOLERANCE).then_some(next)
    })
    .take(5)
    .collect()
}

fn fallback_polyline_curve<C: PolylineableCurve>(
    edge: &CompressedEdge<C>,
    orientation: bool,
    tolerance: f64,
) -> PolylineCurve {
    let curve = PolylineCurve::from_curve(&edge.curve, edge.curve.range_tuple(), tolerance);
    if orientation { curve } else { curve.inverse() }
}

fn densify_curve_parameters(mut parameters: Vec<f64>, min_points: usize) -> Vec<f64> {
    if parameters.len() >= min_points || parameters.len() < 2 || min_points < 2 {
        parameters
    } else {
        let intervals = parameters.len() - 1;
        let extra = min_points - parameters.len();
        parameters = parameters
            .windows(2)
            .enumerate()
            .flat_map(|(interval, window)| {
                let extra_before = interval * extra / intervals;
                let extra_after = (interval + 1) * extra / intervals;
                let splits = 1 + extra_after - extra_before;
                (0..splits).map(move |index| {
                    let weight = index as f64 / splits as f64;
                    window[0] + (window[1] - window[0]) * weight
                })
            })
            .chain(parameters.last().copied())
            .collect();
        parameters
    }
}

fn fallback_polyline_curve_with_min_points<C: PolylineableCurve>(
    edge: &CompressedEdge<C>,
    orientation: bool,
    tolerance: f64,
    min_points: usize,
) -> PolylineCurve {
    let parameters = densify_curve_parameters(
        edge.curve
            .parameter_division(edge.curve.range_tuple(), tolerance)
            .0,
        min_points,
    );
    let curve = parameters
        .into_iter()
        .map(|parameter| edge.curve.subs(parameter))
        .collect::<PolylineCurve>();
    if orientation { curve } else { curve.inverse() }
}

fn untrimmed_isoparametric_curves<S: PreMeshableSurface>(
    surface: &S,
    uv_range: ((f64, f64), (f64, f64)),
    options: IsoparametricCurveOptions,
) -> Vec<Vec<Point3>> {
    if options.samples_per_direction == 0 || options.segments_per_curve == 0 {
        Vec::new()
    } else {
        let ((u_min, u_max), (v_min, v_max)) = uv_range;
        let axis_values = |min: f64, max: f64| {
            (0..options.samples_per_direction)
                .map(move |index| {
                    let parameter = (index + 1) as f64 / (options.samples_per_direction + 1) as f64;
                    min + (max - min) * parameter
                })
                .collect::<Vec<_>>()
        };
        let sample_axis = |constant_axis: usize, constant_value: f64| {
            (0..=options.segments_per_curve)
                .map(|index| {
                    let parameter = index as f64 / options.segments_per_curve as f64;
                    let variable_value = match constant_axis {
                        0 => v_min + (v_max - v_min) * parameter,
                        _ => u_min + (u_max - u_min) * parameter,
                    };
                    match constant_axis {
                        0 => surface.evaluate(constant_value, variable_value),
                        _ => surface.evaluate(variable_value, constant_value),
                    }
                })
                .collect::<Vec<_>>()
        };
        axis_values(u_min, u_max)
            .into_iter()
            .map(|u| sample_axis(0, u))
            .chain(
                axis_values(v_min, v_max)
                    .into_iter()
                    .map(|v| sample_axis(1, v)),
            )
            .collect()
    }
}

fn oriented_edge_indices<'a>(
    wire: &'a [CompressedEdgeIndex],
    face_orientation: bool,
) -> impl Iterator<Item = (usize, bool)> + 'a {
    match face_orientation {
        true => Either::Left(
            wire.iter()
                .map(|edge_idx| (edge_idx.index, edge_idx.orientation)),
        ),
        false => Either::Right(
            wire.iter()
                .rev()
                .map(|edge_idx| (edge_idx.index, !edge_idx.orientation)),
        ),
    }
}

fn oriented_trimmed_edge_uses<'a, T>(
    wire: &'a [CompressedEdgeUse<T>],
    face_orientation: bool,
) -> impl Iterator<Item = (usize, bool, Option<&'a T>)> + 'a {
    match face_orientation {
        true => Either::Left(wire.iter().map(|edge_use| {
            (
                edge_use.index,
                edge_use.orientation,
                edge_use.trim_curve.as_ref(),
            )
        })),
        false => Either::Right(wire.iter().rev().map(|edge_use| {
            (
                edge_use.index,
                !edge_use.orientation,
                edge_use.trim_curve.as_ref(),
            )
        })),
    }
}

fn polyline_from_trim_curve<S, T>(
    surface: &S,
    trim_curve: &T,
    edge_vertices: (usize, usize),
    vertices: &[Point3],
    tolerance: f64,
) -> Option<PolylineCurve>
where
    S: ParametricSurface3D,
    T: ExactTrimBoundary2D,
{
    let mut points = simplify_parameter_boundary(
        surface,
        trim_curve.exact_trim_boundary_2d(tolerance),
        tolerance,
    )
    .into_iter()
    .map(|uv| surface.subs(uv.x, uv.y))
    .collect::<Vec<_>>();
    if points.len() < 2 {
        return None;
    }
    let front = *vertices.get(edge_vertices.0)?;
    let back = *vertices.get(edge_vertices.1)?;
    let start = *points.first()?;
    let end = *points.last()?;
    let direct = start.distance2(front) + end.distance2(back);
    let reversed = start.distance2(back) + end.distance2(front);
    if reversed < direct {
        points.reverse();
    }
    Some(PolylineCurve(points))
}

fn point_segment_distance2(point: Point3, front: Point3, back: Point3) -> f64 {
    let segment = back - front;
    let denom = segment.dot(segment);
    if denom.so_small() {
        point.distance2(front)
    } else {
        let t = ((point - front).dot(segment) / denom).clamp(0.0, 1.0);
        let nearest = front + segment * t;
        point.distance2(nearest)
    }
}

fn simplify_parameter_boundary<S>(
    surface: &S,
    boundary: Vec<Point2>,
    tolerance: f64,
) -> Vec<Point2>
where
    S: ParametricSurface3D,
{
    let filtered = boundary
        .into_iter()
        .fold(Vec::<Point2>::new(), |mut acc, uv| {
            if acc.last().is_none_or(|last| !last.near(&uv)) {
                acc.push(uv);
            }
            acc
        });
    if filtered.len() <= 2
        || filtered
            .first()
            .zip(filtered.last())
            .is_some_and(|(front, back)| front.near(back))
    {
        filtered
    } else {
        let points = filtered
            .iter()
            .map(|uv| surface.subs(uv.x, uv.y))
            .collect::<Vec<_>>();
        let mut keep = vec![false; filtered.len()];
        keep[0] = true;
        keep[filtered.len() - 1] = true;
        let mut stack = vec![(0usize, filtered.len() - 1)];
        let tolerance2 = tolerance * tolerance;
        while let Some((front, back)) = stack.pop() {
            if back <= front + 1 {
                continue;
            }
            let (index, max_distance2) = ((front + 1)..back)
                .map(|index| {
                    (
                        index,
                        point_segment_distance2(points[index], points[front], points[back]),
                    )
                })
                .max_by(|lhs, rhs| lhs.1.total_cmp(&rhs.1))
                .unwrap_or((front, 0.0));
            if max_distance2 > tolerance2 {
                keep[index] = true;
                stack.push((front, index));
                stack.push((index, back));
            }
        }
        filtered
            .into_iter()
            .enumerate()
            .filter_map(|(index, uv)| keep[index].then_some(uv))
            .collect()
    }
}

fn resample_boundary(boundary: &[Point2], target_len: usize) -> Option<Vec<Point2>> {
    if target_len == 0 || boundary.is_empty() {
        None
    } else if target_len == 1 || boundary.len() == 1 {
        Some(vec![*boundary.first()?; target_len])
    } else {
        let cumulative = iter::once(0.0)
            .chain(boundary.windows(2).scan(0.0, |length, window| {
                *length += window[0].distance(window[1]);
                Some(*length)
            }))
            .collect::<Vec<_>>();
        let total = *cumulative.last()?;
        if total.so_small() {
            Some(vec![*boundary.first()?; target_len])
        } else {
            let last = *boundary.last()?;
            Some(
                (0..target_len)
                    .map(|index| {
                        if index + 1 == target_len {
                            last
                        } else {
                            let distance = total * index as f64 / (target_len - 1) as f64;
                            let upper = cumulative.partition_point(|value| *value < distance);
                            let segment = upper.saturating_sub(1).min(boundary.len() - 2);
                            let front = boundary[segment];
                            let back = boundary[segment + 1];
                            let front_distance = cumulative[segment];
                            let back_distance = cumulative[segment + 1];
                            let weight = if (back_distance - front_distance).so_small() {
                                0.0
                            } else {
                                (distance - front_distance) / (back_distance - front_distance)
                            };
                            Point2::new(
                                front.x + (back.x - front.x) * weight,
                                front.y + (back.y - front.y) * weight,
                            )
                        }
                    })
                    .collect(),
            )
        }
    }
}

fn log_mesh_trace(face_idx: usize, stage: &str, extra: impl AsRef<str>, elapsed: Instant) {
    if mesh_trace_enabled() {
        eprintln!(
            "mesh_trace face={face_idx} stage={stage} elapsed_ms={:.3} {}",
            elapsed.elapsed().as_secs_f64() * 1000.0,
            extra.as_ref(),
        );
    }
}

pub(super) trait SP<S>:
    Fn(&S, Point3, Option<(f64, f64)>) -> Option<(f64, f64)> + Parallelizable {
}
impl<S, F> SP<S> for F where F: Fn(&S, Point3, Option<(f64, f64)>) -> Option<(f64, f64)> + Parallelizable {}

pub(super) fn search_parameter_sp<S: MeshableSurface>(trials: usize) -> impl SP<S> {
    move |surface: &S, point: Point3, hint: Option<(f64, f64)>| {
        surface
            .search_parameter(point, hint, trials)
            .or_else(|| surface.search_parameter(point, None, trials))
    }
}

pub(super) fn search_nearest_parameter_sp<S: RobustMeshableSurface>(trials: usize) -> impl SP<S> {
    move |surface: &S, point: Point3, hint: Option<(f64, f64)>| {
        surface
            .search_parameter(point, hint, trials)
            .or_else(|| surface.search_parameter(point, None, trials))
            .or_else(|| surface.search_nearest_parameter(point, hint, trials))
            .or_else(|| surface.search_nearest_parameter(point, None, trials))
    }
}

/// Compatibility wrapper: searches parameter with 100 trials.
#[cfg(test)]
pub(super) fn by_search_parameter<S: MeshableSurface>(
    surface: &S,
    point: Point3,
    hint: Option<(f64, f64)>,
) -> Option<(f64, f64)> {
    search_parameter_sp::<S>(100)(surface, point, hint)
}

/// Tessellates faces.
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn shell_tessellation<'a, C, S>(
    shell: &Shell<Point3, C, S>,
    tolerance: f64,
    sp: impl SP<S>,
    primitive_config: TessellationPrimitiveOptions,
) -> MeshedShell
where
    C: PolylineableCurve + 'a,
    S: PreMeshableSurface + 'a,
{
    let vmap: HashMap<_, _> = shell
        .vertex_par_iter()
        .map(|v| (v.id(), v.mapped(Point3::clone)))
        .collect();
    let eset: HashMap<_, _> = shell.edge_par_iter().map(move |e| (e.id(), e)).collect();
    let edge_map: HashMap<_, _> = eset
        .into_par_iter()
        .map(move |(id, edge)| {
            // SAFETY: vmap was built from all vertices in the shell.
            let v0 = vmap.get(&edge.absolute_front().id()).unwrap();
            let v1 = vmap.get(&edge.absolute_back().id()).unwrap();
            let curve = edge.curve();
            let poly = PolylineCurve::from_curve(&curve, curve.range_tuple(), tolerance);
            // Spec 012 U4: `Edge::debug_new` is fallible now and this
            // `par_iter().map()` has no channel; `new_unchecked` is what the
            // release arm of `debug_new` already did.
            (id, Edge::new_unchecked(v0, v1, poly))
        })
        .collect();
    let create_edge = |edge: &Edge<Point3, C>| -> Edge<_, _> {
        // SAFETY: edge_map was built from all edges in the shell.
        let new_edge = edge_map.get(&edge.id()).unwrap();
        match edge.orientation() {
            true => new_edge.clone(),
            false => new_edge.inverse(),
        }
    };
    let create_boundary =
        |wire: &Wire<Point3, C>| -> Wire<_, _> { wire.edge_iter().map(create_edge).collect() };
    let create_face = move |(face_idx, face): (usize, &Face<Point3, C, S>)| -> Face<_, _, _> {
        let wires: Vec<_> = face
            .absolute_boundaries()
            .iter()
            .map(create_boundary)
            .collect();
        shell_create_polygon(
            &face.surface(),
            wires,
            face.orientation(),
            tolerance,
            &sp,
            primitive_config,
            Some(face_idx),
        )
    };
    shell.face_par_iter().enumerate().map(create_face).collect()
}

/// Tessellates faces.
#[cfg(any(target_arch = "wasm32", test))]
pub(super) fn shell_tessellation_single_thread<'a, C, S>(
    shell: &'a Shell<Point3, C, S>,
    tolerance: f64,
    sp: impl SP<S>,
    primitive_config: TessellationPrimitiveOptions,
) -> MeshedShell
where
    C: PolylineableCurve + 'a,
    S: PreMeshableSurface + 'a,
{
    use monstertruck_core::entry_map::FxEntryMap as EntryMap;
    use monstertruck_topology::Vertex as TVertex;
    let mut vmap = EntryMap::new(
        move |v: &TVertex<Point3>| v.id(),
        move |v| v.mapped(Point3::clone),
    );
    let mut edge_map = EntryMap::new(
        move |edge: &'a Edge<Point3, C>| edge.id(),
        move |edge| {
            let vf = edge.absolute_front();
            let v0 = vmap.entry_or_insert(vf).clone();
            let vb = edge.absolute_back();
            let v1 = vmap.entry_or_insert(vb).clone();
            let curve = edge.curve();
            let poly = PolylineCurve::from_curve(&curve, curve.range_tuple(), tolerance);
            // Spec 012 U4, as the parallel sibling above.
            Edge::new_unchecked(&v0, &v1, poly)
        },
    );
    let mut create_edge = move |edge: &'a Edge<Point3, C>| -> Edge<_, _> {
        let new_edge = edge_map.entry_or_insert(edge);
        match edge.orientation() {
            true => new_edge.clone(),
            false => new_edge.inverse(),
        }
    };
    let mut create_boundary = move |wire: &'a Wire<Point3, C>| -> Wire<_, _> {
        wire.edge_iter().map(&mut create_edge).collect()
    };
    let create_face = move |(face_idx, face): (usize, &'a Face<Point3, C, S>)| -> Face<_, _, _> {
        let wires: Vec<_> = face
            .absolute_boundaries()
            .iter()
            .map(&mut create_boundary)
            .collect();
        shell_create_polygon(
            &face.surface(),
            wires,
            face.orientation(),
            tolerance,
            &sp,
            primitive_config,
            Some(face_idx),
        )
    };
    shell.face_iter().enumerate().map(create_face).collect()
}

/// Tessellates faces.
pub(super) fn cshell_tessellation<'a, C, S>(
    shell: &CompressedShell<Point3, C, S>,
    tolerance: f64,
    sp: impl SP<S>,
    primitive_config: TessellationPrimitiveOptions,
) -> MeshedCompressedShell
where
    C: PolylineableCurve + ParameterBoundary2D<S> + 'a,
    S: PreMeshableSurface + 'a,
{
    let vertices = shell.vertices.clone();
    let tessellate_edge = |edge: &CompressedEdge<C>| {
        let curve = &edge.curve;
        CompressedEdge {
            vertices: edge.vertices,
            curve: PolylineCurve::from_curve(curve, curve.range_tuple(), tolerance),
        }
    };
    #[cfg(not(target_arch = "wasm32"))]
    let edges: Vec<_> = shell.edges.par_iter().map(tessellate_edge).collect();
    #[cfg(target_arch = "wasm32")]
    let edges: Vec<_> = shell.edges.iter().map(tessellate_edge).collect();
    let tessellate_face = |(face_idx, face): (usize, &CompressedFace<S>)| {
        let face_start = Instant::now();
        let boundaries = face.boundaries.clone();
        let surface = &face.surface;
        let face_orientation = face.orientation;

        // Fast path: untrimmed face with bounded surface domain.
        let is_untrimmed = boundaries.iter().all(|wire| wire.is_empty());
        if is_untrimmed && let (Some(urange), Some(vrange)) = surface.try_range_tuple() {
            let polygon =
                untrimmed_tessellation(surface, (urange, vrange), tolerance, primitive_config.mode);
            log_mesh_trace(face_idx, "untrimmed", "ok", face_start);
            observe_face_drop(
                surface,
                Some(face_idx),
                Some(&polygon),
                true,
                boundaries.len(),
                0,
            );
            return CompressedFace {
                boundaries,
                orientation: face.orientation,
                surface: Some(polygon),
            };
        }

        let create_edge = |(edge_index, edge_orientation): (usize, bool)| match edge_orientation {
            true => Some(edges.get(edge_index)?.curve.clone()),
            false => Some(edges.get(edge_index)?.curve.inverse()),
        };
        let create_boundary = |wire: &Vec<CompressedEdgeIndex>| {
            let exact_edges = |boundary_tolerance| {
                oriented_edge_indices(wire, face_orientation)
                    .filter_map(|(edge_index, edge_orientation)| {
                        let edge = shell.edges.get(edge_index)?;
                        let mut boundary = edge
                            .curve
                            .parameter_boundary_2d(surface, boundary_tolerance);
                        if !edge_orientation {
                            boundary = boundary.map(|mut boundary| {
                                boundary.reverse();
                                boundary
                            });
                        }
                        let polyline = if boundary_tolerance.near(&tolerance) {
                            match edge_orientation {
                                true => edges.get(edge_index)?.curve.clone(),
                                false => edges.get(edge_index)?.curve.inverse(),
                            }
                        } else {
                            fallback_polyline_curve(edge, edge_orientation, boundary_tolerance)
                        };
                        Some((edge_orientation, &edge.curve, boundary, polyline))
                    })
                    .collect::<Vec<_>>()
            };
            let create_exact_piece = |boundary_tolerance| {
                let exact_edges = exact_edges(boundary_tolerance);
                let direct_piece = PolyBoundaryPiece::try_new_from_exact(
                    surface,
                    exact_edges
                        .iter()
                        .map(|(orientation, curve, _, _)| (*orientation, *curve)),
                    boundary_tolerance,
                );
                let aligned_piece = PolyBoundaryPiece::try_new_from_aligned_exact(
                    surface,
                    exact_edges
                        .into_iter()
                        .map(|(_, _, boundary, polyline)| (boundary, polyline)),
                    &sp,
                );
                aligned_piece.or(direct_piece)
            };
            let exact_piece = create_exact_piece(tolerance);
            if mesh_trace_enabled() {
                let exact_count = wire
                    .iter()
                    .filter(|edge_idx| {
                        shell
                            .edges
                            .get(edge_idx.index)
                            .and_then(|edge| edge.curve.parameter_boundary_2d(surface, tolerance))
                            .is_some()
                    })
                    .count();
                eprintln!(
                    "mesh_trace face={face_idx} stage=exact-boundary edges={}/{} success={}",
                    exact_count,
                    wire.len(),
                    exact_piece.is_some(),
                );
            }
            exact_piece
                .or_else(|| {
                    boundary_tolerance_candidates(tolerance)
                        .into_iter()
                        .skip(1)
                        .find_map(create_exact_piece)
                })
                .or_else(|| {
                    let wire_iter =
                        oriented_edge_indices(wire, face_orientation).filter_map(create_edge);
                    PolyBoundaryPiece::try_new(surface, wire_iter, &sp, tolerance)
                })
        };
        let boundary_start = Instant::now();
        let preboundary: Option<Vec<_>> = boundaries.iter().map(create_boundary).collect();
        if let Some(preboundary) = &preboundary {
            let point_count = preboundary.iter().map(|piece| piece.0.len()).sum::<usize>();
            log_mesh_trace(
                face_idx,
                "preboundary",
                format!("loops={} points={point_count}", preboundary.len()),
                boundary_start,
            );
        } else {
            log_mesh_trace(face_idx, "preboundary", "failed", boundary_start);
        }
        let polygon: Option<PolygonMesh> = preboundary.map(|preboundary| {
            let close_start = Instant::now();
            let boundary = PolyBoundary::new(preboundary, &surface, tolerance);
            log_mesh_trace(
                face_idx,
                "polyboundary",
                format!(
                    "closed_loops={} uv_min=({:.6},{:.6}) uv_max=({:.6},{:.6})",
                    boundary.loops.len(),
                    boundary.uv_min.x,
                    boundary.uv_min.y,
                    boundary.uv_max.x,
                    boundary.uv_max.y,
                ),
                close_start,
            );
            let mesh_start = Instant::now();
            let polygon =
                trimming_tessellation(&surface, &boundary, tolerance, primitive_config, face_idx);
            log_mesh_trace(
                face_idx,
                "trimmed",
                format!(
                    "tris={} quads={}",
                    polygon.tri_faces().len(),
                    polygon.quad_faces().len()
                ),
                mesh_start,
            );
            polygon
        });
        log_mesh_trace(
            face_idx,
            "face",
            format!("surface={}", polygon.is_some()),
            face_start,
        );
        observe_face_drop(
            surface,
            Some(face_idx),
            polygon.as_ref(),
            is_untrimmed,
            boundaries.len(),
            boundaries.iter().map(|wire| wire.len()).sum(),
        );
        CompressedFace {
            boundaries,
            orientation: face.orientation,
            surface: polygon,
        }
    };
    #[cfg(not(target_arch = "wasm32"))]
    let faces = shell
        .faces
        .par_iter()
        .enumerate()
        .map(tessellate_face)
        .collect();
    #[cfg(target_arch = "wasm32")]
    let faces = shell
        .faces
        .iter()
        .enumerate()
        .map(tessellate_face)
        .collect();
    MeshedCompressedShell {
        vertices,
        edges,
        faces,
        vertex_stable_ids: None,
        edge_stable_ids: None,
        face_stable_ids: None,
    }
}

pub(super) fn trimmed_cshell_tessellation<'a, C, S, T>(
    shell: &TrimmedShell<C, S, T>,
    tolerance: f64,
    sp: impl SP<S>,
    primitive_config: TessellationPrimitiveOptions,
) -> MeshedCompressedShell
where
    C: PolylineableCurve + ParameterBoundary2D<S> + ExactParameterBoundary2D<S> + 'a,
    S: PreMeshableSurface + 'a,
    T: ExactTrimBoundary2D + Parallelizable + 'a,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: ExactTrimBoundary2D,
{
    compressed_trimmed_shell_tessellation_with_isoparams(
        shell,
        tolerance,
        sp,
        primitive_config,
        None,
    )
    .shell
}

pub(super) fn compressed_trimmed_shell_tessellation_with_isoparams<'a, C, S, T>(
    shell: &TrimmedShell<C, S, T>,
    tolerance: f64,
    sp: impl SP<S>,
    primitive_config: TessellationPrimitiveOptions,
    isoparametric_options: Option<IsoparametricCurveOptions>,
) -> CompressedShellTessellation
where
    C: PolylineableCurve + ParameterBoundary2D<S> + ExactParameterBoundary2D<S> + 'a,
    S: PreMeshableSurface + 'a,
    T: ExactTrimBoundary2D + Parallelizable + 'a,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: ExactTrimBoundary2D,
{
    let vertices = shell.vertices.clone();
    let exact_edge_polylines = shell
        .faces
        .iter()
        .flat_map(|face| {
            face.boundaries.iter().flat_map(move |wire| {
                wire.iter().filter_map(move |edge_use| {
                    edge_use
                        .trim_curve
                        .as_ref()
                        .map(|trim_curve| (edge_use.index, &face.surface, trim_curve))
                })
            })
        })
        .fold(
            HashMap::<usize, PolylineCurve>::default(),
            |mut acc, (edge_idx, surface, trim_curve)| {
                let edge = &shell.edges[edge_idx];
                let candidate = polyline_from_trim_curve(
                    surface,
                    trim_curve,
                    edge.vertices,
                    &vertices,
                    tolerance,
                )
                .map(|curve| {
                    fallback_polyline_curve_with_min_points(edge, true, tolerance, curve.len())
                })
                .unwrap_or_else(|| fallback_polyline_curve(edge, true, tolerance));
                match acc.entry(edge_idx) {
                    Entry::Occupied(mut entry) => {
                        if candidate.len() > entry.get().len() {
                            entry.insert(candidate);
                        }
                    }
                    Entry::Vacant(entry) => {
                        entry.insert(candidate);
                    }
                }
                acc
            },
        );
    let tessellate_edge = |(edge_idx, edge): (usize, &CompressedEdge<C>)| {
        let curve = &edge.curve;
        CompressedEdge {
            vertices: edge.vertices,
            curve: exact_edge_polylines
                .get(&edge_idx)
                .cloned()
                .unwrap_or_else(|| {
                    PolylineCurve::from_curve(curve, curve.range_tuple(), tolerance)
                }),
        }
    };
    #[cfg(not(target_arch = "wasm32"))]
    let edges: Vec<_> = shell
        .edges
        .par_iter()
        .enumerate()
        .map(tessellate_edge)
        .collect();
    #[cfg(target_arch = "wasm32")]
    let edges: Vec<_> = shell
        .edges
        .iter()
        .enumerate()
        .map(tessellate_edge)
        .collect();
    let tessellate_face = |(face_idx, face): (usize, &CompressedTrimmedFace<S, T>)| {
        let face_start = Instant::now();
        log_mesh_trace(face_idx, "face-start", "", face_start);
        let boundaries = face
            .boundaries
            .iter()
            .map(|wire| {
                wire.iter()
                    .map(|edge_use| CompressedEdgeIndex {
                        index: edge_use.index,
                        orientation: edge_use.orientation,
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let surface = &face.surface;
        let face_orientation = face.orientation;

        let is_untrimmed = boundaries.iter().all(|wire| wire.is_empty());
        if is_untrimmed && let (Some(urange), Some(vrange)) = surface.try_range_tuple() {
            let polygon =
                untrimmed_tessellation(surface, (urange, vrange), tolerance, primitive_config.mode);
            let isoparams = isoparametric_options
                .map(|options| untrimmed_isoparametric_curves(surface, (urange, vrange), options))
                .unwrap_or_default();
            log_mesh_trace(face_idx, "untrimmed", "ok", face_start);
            observe_face_drop(
                surface,
                Some(face_idx),
                Some(&polygon),
                true,
                boundaries.len(),
                0,
            );
            return (
                CompressedFace {
                    boundaries,
                    orientation: face.orientation,
                    surface: Some(polygon),
                },
                isoparams,
            );
        }

        let create_boundary = |wire: &Vec<CompressedEdgeUse<T>>| {
            let wire_start = Instant::now();
            let all_exact = wire.iter().all(|edge_use| edge_use.trim_curve.is_some());
            let has_shared_edge_polyline = wire
                .iter()
                .any(|edge_use| exact_edge_polylines.contains_key(&edge_use.index));
            let direct_trim_piece = || {
                PolyBoundaryPiece::try_new_from_trimmed(
                    surface,
                    oriented_trimmed_edge_uses(wire, face_orientation).filter_map(
                        |(edge_index, edge_orientation, trim_curve)| {
                            shell
                                .edges
                                .get(edge_index)
                                .map(|edge| (edge_orientation, trim_curve, &edge.curve))
                        },
                    ),
                    tolerance,
                )
            };
            let aligned_trim_piece = || {
                PolyBoundaryPiece::try_new_from_aligned_trimmed(
                    surface,
                    oriented_trimmed_edge_uses(wire, face_orientation).filter_map(
                        |(edge_index, edge_orientation, trim_curve)| {
                            shell.edges.get(edge_index).map(|edge| {
                                let mut polyline = exact_edge_polylines
                                    .get(&edge_index)
                                    .cloned()
                                    .unwrap_or_else(|| {
                                        fallback_polyline_curve(edge, true, tolerance)
                                    });
                                if !edge_orientation {
                                    polyline.invert();
                                }
                                (trim_curve, &edge.curve, polyline)
                            })
                        },
                    ),
                    &sp,
                    tolerance,
                )
            };
            let exact_piece = if has_shared_edge_polyline {
                aligned_trim_piece()
                    .filter(PolyBoundaryPiece::is_cdt_compatible)
                    .or_else(direct_trim_piece)
            } else {
                direct_trim_piece().or_else(aligned_trim_piece)
            }
            .or_else(|| {
                if all_exact {
                    None
                } else {
                    boundary_tolerance_candidates(tolerance)
                        .into_iter()
                        .skip(1)
                        .find_map(|boundary_tolerance| {
                            PolyBoundaryPiece::try_new_from_trimmed(
                                surface,
                                oriented_trimmed_edge_uses(wire, face_orientation).filter_map(
                                    |(edge_index, edge_orientation, trim_curve)| {
                                        shell
                                            .edges
                                            .get(edge_index)
                                            .map(|edge| (edge_orientation, trim_curve, &edge.curve))
                                    },
                                ),
                                boundary_tolerance,
                            )
                            .or_else(|| {
                                let wire_iter = oriented_trimmed_edge_uses(wire, face_orientation)
                                    .filter_map(|(edge_index, edge_orientation, _)| {
                                        shell.edges.get(edge_index).map(|edge| {
                                            fallback_polyline_curve(
                                                edge,
                                                edge_orientation,
                                                boundary_tolerance,
                                            )
                                        })
                                    });
                                PolyBoundaryPiece::try_new(surface, wire_iter, &sp, tolerance)
                            })
                        })
                }
            });
            if mesh_trace_enabled() {
                let exact_count = wire
                    .iter()
                    .filter(|edge_use| {
                        edge_use.trim_curve.as_ref().is_some_and(|trim_curve| {
                            !trim_curve.exact_trim_boundary_2d(tolerance).is_empty()
                        }) || shell.edges.get(edge_use.index).is_some_and(|edge| {
                            edge.curve
                                .parameter_boundary_2d(surface, tolerance)
                                .is_some()
                        })
                    })
                    .count();
                eprintln!(
                    "mesh_trace face={face_idx} stage=exact-boundary edges={}/{} success={}",
                    exact_count,
                    wire.len(),
                    exact_piece.is_some(),
                );
            }
            log_mesh_trace(
                face_idx,
                "wire",
                format!("edges={} success={}", wire.len(), exact_piece.is_some()),
                wire_start,
            );
            exact_piece
        };
        let boundary_start = Instant::now();
        let preboundary: Option<Vec<_>> = face.boundaries.iter().map(create_boundary).collect();
        if let Some(preboundary) = &preboundary {
            let point_count = preboundary.iter().map(|piece| piece.0.len()).sum::<usize>();
            log_mesh_trace(
                face_idx,
                "preboundary",
                format!("loops={} points={point_count}", preboundary.len()),
                boundary_start,
            );
        } else {
            log_mesh_trace(face_idx, "preboundary", "failed", boundary_start);
        }
        let (polygon, isoparams) = preboundary
            .map(|preboundary| {
                let close_start = Instant::now();
                let boundary = PolyBoundary::new(preboundary, surface, tolerance);
                log_mesh_trace(
                    face_idx,
                    "polyboundary",
                    format!(
                        "closed_loops={} uv_min=({:.6},{:.6}) uv_max=({:.6},{:.6})",
                        boundary.loops.len(),
                        boundary.uv_min.x,
                        boundary.uv_min.y,
                        boundary.uv_max.x,
                        boundary.uv_max.y,
                    ),
                    close_start,
                );
                let isoparams = isoparametric_options
                    .map(|options| boundary.isoparametric_curves(surface, options))
                    .unwrap_or_default();
                let mesh_start = Instant::now();
                let polygon = trimming_tessellation(
                    surface,
                    &boundary,
                    tolerance,
                    primitive_config,
                    face_idx,
                );
                log_mesh_trace(
                    face_idx,
                    "trimmed",
                    format!(
                        "tris={} quads={}",
                        polygon.tri_faces().len(),
                        polygon.quad_faces().len()
                    ),
                    mesh_start,
                );
                (Some(polygon), isoparams)
            })
            .unwrap_or_default();
        log_mesh_trace(
            face_idx,
            "face",
            format!("surface={}", polygon.is_some()),
            face_start,
        );
        observe_face_drop(
            surface,
            Some(face_idx),
            polygon.as_ref(),
            is_untrimmed,
            boundaries.len(),
            boundaries.iter().map(|wire| wire.len()).sum(),
        );
        (
            CompressedFace {
                boundaries,
                orientation: face.orientation,
                surface: polygon,
            },
            isoparams,
        )
    };
    #[cfg(not(target_arch = "wasm32"))]
    let face_outputs = shell
        .faces
        .par_iter()
        .enumerate()
        .map(tessellate_face)
        .collect::<Vec<_>>();
    #[cfg(target_arch = "wasm32")]
    let face_outputs = shell
        .faces
        .iter()
        .enumerate()
        .map(tessellate_face)
        .collect::<Vec<_>>();
    let (faces, face_isoparams) = face_outputs.into_iter().unzip();
    CompressedShellTessellation {
        shell: MeshedCompressedShell {
            vertices,
            edges,
            faces,
            vertex_stable_ids: None,
            edge_stable_ids: None,
            face_stable_ids: None,
        },
        face_isoparams,
    }
}

fn shell_create_polygon<S: PreMeshableSurface>(
    surface: &S,
    wires: Vec<Wire<Point3, PolylineCurve>>,
    orientation: bool,
    tolerance: f64,
    sp: impl SP<S>,
    primitive_config: TessellationPrimitiveOptions,
    face_idx: Option<usize>,
) -> Face<Point3, PolylineCurve, Option<PolygonMesh>> {
    // Fast path: untrimmed face with bounded surface domain.
    let is_untrimmed = wires.iter().all(|w| w.is_empty());
    let polygon = if is_untrimmed {
        if let (Some(urange), Some(vrange)) = surface.try_range_tuple() {
            Some(untrimmed_tessellation(
                surface,
                (urange, vrange),
                tolerance,
                primitive_config.mode,
            ))
        } else {
            None
        }
    } else {
        boundary::projection_debug_set_face(face_idx);
        let preboundary = wires
            .iter()
            .map(|wire: &Wire<_, _>| {
                let wire_iter = wire.iter().map(Edge::oriented_curve);
                PolyBoundaryPiece::try_new(surface, wire_iter, &sp, tolerance)
            })
            .collect::<Option<Vec<_>>>();
        preboundary.map(|preboundary| {
            let boundary = PolyBoundary::new(preboundary, &surface, tolerance);
            trimming_tessellation(surface, &boundary, tolerance, primitive_config, usize::MAX)
        })
    };
    let loops = wires.len();
    let edges = wires.iter().map(|wire| wire.len()).sum();
    observe_face_drop(
        surface,
        face_idx,
        polygon.as_ref(),
        is_untrimmed,
        loops,
        edges,
    );
    let mut new_face = Face::new_unchecked(wires, polygon);
    if !orientation {
        new_face.invert();
    }
    new_face
}

#[test]
#[ignore]
#[cfg(not(target_arch = "wasm32"))]
fn par_bench() {
    use monstertruck_modeling::*;
    use std::time::Instant;
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../resources/shape/bottle.json"
    );
    let json = std::fs::read_to_string(path).expect("bottle.json resource missing");
    let solid: Solid = serde_json::from_str(&json).unwrap();
    let shell = solid.into_boundaries().pop().unwrap();

    let instant = Instant::now();
    (0..100).for_each(|_| {
        let _shell = shell_tessellation(
            &shell,
            0.01,
            by_search_parameter,
            TessellationPrimitiveOptions::default(),
        );
    });
    println!("{}ms", instant.elapsed().as_millis());

    let instant = Instant::now();
    (0..100).for_each(|_| {
        let _shell = shell_tessellation_single_thread(
            &shell,
            0.01,
            by_search_parameter,
            TessellationPrimitiveOptions::default(),
        );
    });
    println!("{}ms", instant.elapsed().as_millis());
}

#[cfg(test)]
mod tests;
