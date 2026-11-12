//! Shape healing for solids imported from other CAD systems.
//!
//! The compressed-shell repair passes that turn a loaded B-rep into something a
//! boolean kernel can work with: closed-edge and closed-face splitting, seam
//! stripping, pass-through-edge dedup, shell orientation normalization and
//! conservative naked-edge welding.
//!
//! These passes are POST-CSG and kernel-independent -- nothing here references a
//! boolean backend, which is why the crate sits below both the published
//! `monstertruck-solid` marching kernel and any external SSI boolean backend
//! rather than inside either.

#![cfg_attr(not(debug_assertions), deny(warnings))]
#![deny(clippy::all, rust_2018_idioms)]
#![warn(
    missing_docs,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications
)]

pub(crate) use ahash::AHashSet as HashSet;

use monstertruck_geometry::prelude::*;
use monstertruck_meshing::rexport_polymesh::*;
use monstertruck_topology::compress::*;
use monstertruck_topology::trimmed::{TrimmedShell, TrimmedSolid};
use monstertruck_traits::SnapCurveEndpoints;
use rustc_hash::FxHashMap as HashMap;
use std::env;
// `web_time::Instant` is `std::time::Instant` on native and falls back to the
// browser performance clock on wasm32, where `std::time::Instant::now()` panics.
use web_time::Instant;

type Edge<C> = CompressedEdge<C>;
type EdgeIndex = CompressedEdgeIndex;
type Wire = Vec<EdgeIndex>;
type Face<S> = CompressedFace<S>;
type Shell<P, C, S> = CompressedShell<P, C, S>;

trait SP<S>: Fn(&S, Point3, Option<(f64, f64)>) -> Option<(f64, f64)> {}
impl<S, F> SP<S> for F where F: Fn(&S, Point3, Option<(f64, f64)>) -> Option<(f64, f64)> {}

mod naked_edge_weld;
pub use naked_edge_weld::{SkipReason, SnapError, SnapReport, snap_naked_edges};

mod split_closed_edges;
use split_closed_edges::{split_closed_edges, split_closed_edges_trimmed};

mod split_pass_through_edges;
use split_pass_through_edges::{
    dedup_coincident_pass_through_edges, remap_trimmed_edge_uses_for_dedup,
    split_pass_through_edges, split_pass_through_edges_trimmed,
};
// Promoted (doc-hidden) so an external SSI boolean backend can reuse these
// compressed-shell repair passes; unused inside this crate
// outside `#[cfg(test)]`.
#[doc(hidden)]
pub use split_pass_through_edges::{
    split_non_simple_compressed_wires, split_pinched_compressed_faces,
};

mod split_closed_faces;
use split_closed_faces::split_closed_faces;

mod split_seam_faces;
use split_seam_faces::split_seam_faces_trimmed;

fn same_surface<S: ParametricSurface3D>(lhs: &S, rhs: &S) -> bool {
    let sample_params = |surface: &S| {
        if let (Some((u0, u1)), Some((v0, v1))) = surface.try_range_tuple() {
            [
                (u0, v0),
                (u1, v0),
                (u0, v1),
                (u1, v1),
                ((u0 + u1) * 0.5, (v0 + v1) * 0.5),
            ]
        } else {
            [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (0.5, 0.5)]
        }
    };
    sample_params(lhs)
        .into_iter()
        .zip(sample_params(rhs))
        .all(|((lu, lv), (ru, rv))| lhs.subs(lu, lv).near(&rhs.subs(ru, rv)))
}

fn preserved_trim_for_same_edge<C, S, T>(
    original: &CompressedTrimmedShell<Point3, C, S, T>,
    surface: &S,
    edge_index: usize,
    orientation: bool,
) -> Option<T>
where
    C: Clone,
    S: ParametricSurface3D,
    T: Clone + Invertible,
{
    original
        .faces
        .iter()
        .filter(|face| same_surface(&face.surface, surface))
        .find_map(|face| {
            face.boundaries.iter().flatten().find_map(|edge_use| {
                (edge_use.index == edge_index).then(|| {
                    edge_use.trim_curve.clone().map(|mut trim_curve| {
                        if edge_use.orientation != orientation {
                            trim_curve.invert();
                        }
                        trim_curve
                    })
                })
            })?
        })
}

fn oriented_trim_segment<C, T>(
    edge_use: &CompressedEdgeUse<T>,
    segment_orientation: bool,
    segment_edge: &CompressedEdge<C>,
    vertices: &[Point3],
    tol: f64,
) -> Option<T>
where
    C: ParametricCurve3D<Point = Point3>,
    T: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + Cut
        + Clone
        + Invertible,
{
    let mut trim_curve = edge_use.trim_curve.clone()?;
    if edge_use.orientation != segment_orientation {
        trim_curve.invert();
    }
    let segment_front = vertices[segment_edge.vertices.0];
    let segment_back = vertices[segment_edge.vertices.1];
    let t_front = trim_curve.search_nearest_parameter(segment_front, None, 100)?;
    let t_back = trim_curve.search_nearest_parameter(segment_back, Some(t_front), 100)?;
    let distance_ok = |lhs: Point3, rhs: Point3| lhs.distance2(rhs) <= tol * tol;
    if !distance_ok(trim_curve.subs(t_front), segment_front)
        || !distance_ok(trim_curve.subs(t_back), segment_back)
    {
        return None;
    }
    let (trim_t0, trim_t1) = trim_curve.range_tuple();
    let trim_len = trim_t1 - trim_t0;
    if trim_len.so_small() {
        return None;
    }
    let map_alpha = |t| {
        let alpha = (t - trim_t0) / trim_len;
        if segment_orientation {
            alpha
        } else {
            1.0 - alpha
        }
    };
    let mut alphas = [map_alpha(t_front), map_alpha(t_back)];
    alphas.sort_by(|lhs, rhs| lhs.total_cmp(rhs));
    let [alpha0, alpha1] = alphas;
    if alpha0.so_small() && (1.0 - alpha1).so_small() {
        return Some(trim_curve);
    }
    let trim_start = trim_t0 + trim_len * alpha0;
    let trim_end = trim_t0 + trim_len * alpha1;
    let param_tol = (trim_len.abs() * 1.0e-6).max(TOLERANCE);
    if !trim_start.is_finite() || !trim_end.is_finite() || trim_end <= trim_start + param_tol {
        return None;
    }
    let cut_if_interior = |curve: &mut T, t: f64| {
        let (range_start, range_end) = curve.range_tuple();
        let range_len = range_end - range_start;
        if range_len.so_small() {
            None
        } else {
            let param_tol = (range_len.abs() * 1.0e-6).max(TOLERANCE);
            let t = t.clamp(range_start, range_end);
            if t <= range_start + param_tol || range_end <= t + param_tol {
                None
            } else {
                Some(curve.cut(t))
            }
        }
    };
    if trim_start <= trim_t0 + param_tol {
        if trim_end >= trim_t1 - param_tol {
            Some(trim_curve)
        } else {
            let _tail = cut_if_interior(&mut trim_curve, trim_end)?;
            Some(trim_curve)
        }
    } else {
        let mut middle = cut_if_interior(&mut trim_curve, trim_start)?;
        let (middle_t0, middle_t1) = middle.range_tuple();
        let middle_tol = ((middle_t1 - middle_t0).abs() * 1.0e-6).max(TOLERANCE);
        if trim_end <= middle_t0 + middle_tol {
            return None;
        }
        if trim_end < middle_t1 - middle_tol {
            let _tail = cut_if_interior(&mut middle, trim_end)?;
        }
        Some(middle)
    }
}

fn derive_preserved_trim_segment<C, S, T>(
    original: &CompressedTrimmedShell<Point3, C, S, T>,
    surface: &S,
    orientation: bool,
    segment_edge: &CompressedEdge<C>,
    vertices: &[Point3],
    tol: f64,
) -> Option<T>
where
    C: ParametricCurve3D<Point = Point3>,
    S: ParametricSurface3D + Clone,
    T: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + Cut
        + Clone
        + Invertible,
{
    original
        .faces
        .iter()
        .filter(|face| same_surface(&face.surface, surface))
        .flat_map(|face| face.boundaries.iter().flatten())
        .find_map(|edge_use| {
            oriented_trim_segment(edge_use, orientation, segment_edge, vertices, tol)
        })
}

fn reattach_preserved_face_trims_with<C, S, T, F>(
    original: &CompressedTrimmedShell<Point3, C, S, T>,
    healed: CompressedShell<Point3, C, S>,
    tol: f64,
    mut regenerate_trim: F,
) -> CompressedTrimmedShell<Point3, C, S, T>
where
    C: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + Clone,
    S: ParametricSurface3D + Clone,
    T: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + Cut
        + Clone
        + Invertible,
    F: FnMut(&CompressedEdge<C>, &S) -> Option<T>,
{
    let debug_trace = env::var("MT_BOOL_TRACE").is_ok();
    let debug_reattach = env::var("MT_BOOL_DEBUG_REATTACH").is_ok();
    let vertices = healed.vertices.clone();
    let edges = healed.edges.clone();
    let mut preserved_count = 0usize;
    let mut derived_count = 0usize;
    let mut regenerated_count = 0usize;
    let mut missing_count = 0usize;
    let mut missing_logged = 0usize;
    let faces = healed
        .faces
        .into_iter()
        .map(|face| {
            let surface = face.surface.clone();
            let boundaries = face
                .boundaries
                .into_iter()
                .map(|wire| {
                    wire.into_iter()
                        .map(|CompressedEdgeIndex { index, orientation }| {
                            let preserved_trim = preserved_trim_for_same_edge(
                                original,
                                &surface,
                                index,
                                orientation,
                            );
                            let derived_trim = preserved_trim.clone().or_else(|| {
                                derive_preserved_trim_segment(
                                    original,
                                    &surface,
                                    orientation,
                                    &edges[index],
                                    &vertices,
                                    tol,
                                )
                            });
                            let regenerated_trim = regenerate_trim(&edges[index], &surface);
                            let trim_curve = if preserved_trim.is_some() {
                                preserved_count += 1;
                                preserved_trim
                            } else if derived_trim.is_some() {
                                derived_count += 1;
                                derived_trim
                            } else if regenerated_trim.is_some() {
                                regenerated_count += 1;
                                regenerated_trim
                            } else {
                                missing_count += 1;
                                if debug_reattach && missing_logged < 32 {
                                    let edge = &edges[index];
                                    eprintln!(
                                        "debug reattach missing surface_face edge={} orientation={} front={:?} back={:?}",
                                        index,
                                        orientation,
                                        vertices[edge.vertices.0],
                                        vertices[edge.vertices.1],
                                    );
                                    missing_logged += 1;
                                }
                                None
                            };
                            CompressedEdgeUse {
                                index,
                                orientation,
                                trim_curve,
                            }
                        })
                        .collect()
                })
                .collect();
            CompressedTrimmedFace {
                boundaries,
                orientation: face.orientation,
                surface: face.surface,
            }
        })
        .collect();
    if debug_trace {
        eprintln!(
            "trace bool reattach_trims preserved={} derived={} regenerated={} missing={}",
            preserved_count, derived_count, regenerated_count, missing_count,
        );
    }
    CompressedTrimmedShell {
        vertices: healed.vertices,
        edges: healed.edges,
        faces,
    }
}

fn reattach_preserved_face_trims<C, S, T>(
    original: &CompressedTrimmedShell<Point3, C, S, T>,
    healed: CompressedShell<Point3, C, S>,
    tol: f64,
) -> CompressedTrimmedShell<Point3, C, S, T>
where
    C: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + ParameterDivision1D<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + ParameterBoundary2D<S>
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + ExactParameterBoundary2D<S, BoundaryCurve = T>
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + Clone,
    T: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible,
{
    fn normalize_axis(
        value: f64,
        previous: Option<f64>,
        period: Option<f64>,
        range: Option<(f64, f64)>,
    ) -> Option<f64> {
        if !value.is_finite() {
            None
        } else if let Some(previous) = previous {
            if let Some(period) = period {
                (-2..=2)
                    .map(|index| value + index as f64 * period)
                    .min_by(|lhs, rhs| {
                        let lhs_distance = f64::abs(*lhs - previous);
                        let rhs_distance = f64::abs(*rhs - previous);
                        lhs_distance.total_cmp(&rhs_distance)
                    })
            } else if let Some((min, max)) = range {
                Some(value.clamp(min, max))
            } else {
                Some(value)
            }
        } else if let Some((min, max)) = range {
            if let Some(period) = period {
                let span = max - min;
                if span.so_small() {
                    Some(min)
                } else {
                    let mut normalized = value - f64::floor((value - min) / period) * period;
                    if normalized < min {
                        normalized += period;
                    }
                    if normalized > max {
                        normalized -= period;
                    }
                    Some(normalized.clamp(min, max))
                }
            } else {
                Some(value.clamp(min, max))
            }
        } else {
            Some(value)
        }
    }

    fn sampled_trim_segment<C, S, T>(edge: &CompressedEdge<C>, surface: &S, tol: f64) -> Option<T>
    where
        C: ParametricCurve3D<Point = Point3>
            + BoundedCurve<Point = Point3>
            + ParameterDivision1D<Point = Point3>,
        S: ParametricSurface3D
            + SearchParameter<SurfaceParameter, Point = Point3>
            + SearchNearestParameter<SurfaceParameter, Point = Point3>,
        T: BoundaryCurveFromSamples<S>, {
        let normalize_uv = |uv: Point2, previous: Option<Point2>| {
            let (u_range, v_range) = surface.try_range_tuple();
            Some(Point2::new(
                normalize_axis(uv.x, previous.map(|uv| uv.x), surface.period_u(), u_range)?,
                normalize_axis(uv.y, previous.map(|uv| uv.y), surface.period_v(), v_range)?,
            ))
        };
        let project = |point: Point3, hint: Option<(f64, f64)>| {
            surface
                .search_parameter(point, hint, 100)
                .or_else(|| surface.search_parameter(point, None, 100))
                .or_else(|| surface.search_nearest_parameter(point, hint, 100))
                .or_else(|| surface.search_nearest_parameter(point, None, 100))
                .map(Point2::from)
                .and_then(|uv| normalize_uv(uv, hint.map(Point2::from)))
        };
        let points = edge
            .curve
            .parameter_division(edge.curve.range_tuple(), tol)
            .1;
        points
            .iter()
            .copied()
            .scan(None, |hint, point| {
                let uv = project(point, *hint);
                *hint = uv.map(|uv| (uv.x, uv.y));
                Some(uv)
            })
            .collect::<Option<Vec<_>>>()
            .or_else(|| {
                points
                    .into_iter()
                    .map(|point| project(point, None))
                    .collect()
            })
            .and_then(|points| T::boundary_curve_from_samples(surface, points))
    }

    let debug_reattach = env::var("MT_BOOL_DEBUG_REATTACH").is_ok();
    reattach_preserved_face_trims_with(original, healed, tol, |edge, surface| {
        edge.curve
            .exact_parameter_boundary_2d(surface)
            .or_else(|| regenerate_linear_trim_segment(edge, surface, debug_reattach))
            .or_else(|| {
                edge.curve
                    .parameter_boundary_2d(surface, tol)
                    .and_then(|points| T::boundary_curve_from_samples(surface, points))
            })
            .or_else(|| sampled_trim_segment(edge, surface, tol))
    })
}

fn regenerate_linear_trim_segment<C, S, T>(
    edge: &CompressedEdge<C>,
    surface: &S,
    debug_reattach: bool,
) -> Option<T>
where
    C: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + ExactParameterBoundary2D<S, BoundaryCurve = T>
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + Clone,
    T: BoundedCurve + Cut + Clone + Invertible,
{
    fn normalize_axis(
        value: f64,
        previous: Option<f64>,
        period: Option<f64>,
        range: Option<(f64, f64)>,
    ) -> Option<f64> {
        if !value.is_finite() {
            None
        } else if let Some(previous) = previous {
            if let Some(period) = period {
                (-2..=2)
                    .map(|index| value + index as f64 * period)
                    .min_by(|lhs, rhs| {
                        let lhs_distance = f64::abs(*lhs - previous);
                        let rhs_distance = f64::abs(*rhs - previous);
                        lhs_distance.total_cmp(&rhs_distance)
                    })
            } else if let Some((min, max)) = range {
                Some(value.clamp(min, max))
            } else {
                Some(value)
            }
        } else if let Some((min, max)) = range {
            if let Some(period) = period {
                let span = max - min;
                if span.so_small() {
                    Some(min)
                } else {
                    let mut normalized = value - f64::floor((value - min) / period) * period;
                    if normalized < min {
                        normalized += period;
                    }
                    if normalized > max {
                        normalized -= period;
                    }
                    Some(normalized.clamp(min, max))
                }
            } else {
                Some(value.clamp(min, max))
            }
        } else {
            Some(value)
        }
    }

    let normalize_uv = |uv: Point2, previous: Option<Point2>| {
        let (u_range, v_range) = surface.try_range_tuple();
        Some(Point2::new(
            normalize_axis(uv.x, previous.map(|uv| uv.x), surface.period_u(), u_range)?,
            normalize_axis(uv.y, previous.map(|uv| uv.y), surface.period_v(), v_range)?,
        ))
    };
    let point_to_uv = |point, hint: Option<(f64, f64)>| {
        surface
            .search_parameter(point, hint, 100)
            .or_else(|| surface.search_parameter(point, None, 100))
            .or_else(|| surface.search_nearest_parameter(point, hint, 100))
            .or_else(|| surface.search_nearest_parameter(point, None, 100))
            .map(Point2::from)
            .and_then(|uv| normalize_uv(uv, hint.map(Point2::from)))
    };
    let Some(uv0) = point_to_uv(edge.curve.front(), None) else {
        if debug_reattach {
            eprintln!(
                "debug reattach linear no_uv front={:?} back={:?}",
                edge.curve.front(),
                edge.curve.back(),
            );
        }
        return None;
    };
    let Some(uv1) = point_to_uv(edge.curve.back(), Some((uv0.x, uv0.y))) else {
        if debug_reattach {
            eprintln!(
                "debug reattach linear no_uv back front={:?} back={:?} uv0={:?}",
                edge.curve.front(),
                edge.curve.back(),
                uv0,
            );
        }
        return None;
    };
    let linear_trim = ParameterCurve::new(Line(uv0, uv1), surface.clone());
    let (t0, t1) = edge.curve.range_tuple();
    let quarters_ok = [0.25, 0.5, 0.75].into_iter().all(|alpha| {
        let uv = linear_trim.subs(alpha);
        surface
            .subs(uv.x, uv.y)
            .near(&edge.curve.subs(t0 + (t1 - t0) * alpha))
    });
    if !quarters_ok {
        if debug_reattach {
            eprintln!(
                "debug reattach linear mismatch front={:?} back={:?} uv0={:?} uv1={:?}",
                edge.curve.front(),
                edge.curve.back(),
                uv0,
                uv1,
            );
        }
        return None;
    }
    let Some(curve) = C::try_from(linear_trim.clone()).ok() else {
        if debug_reattach {
            eprintln!(
                "debug reattach linear try_from failed front={:?} back={:?} uv0={:?} uv1={:?}",
                edge.curve.front(),
                edge.curve.back(),
                uv0,
                uv1,
            );
        }
        return None;
    };
    let exact = curve.exact_parameter_boundary_2d(surface);
    if debug_reattach && exact.is_none() {
        eprintln!(
            "debug reattach linear exact_none front={:?} back={:?} uv0={:?} uv1={:?}",
            edge.curve.front(),
            edge.curve.back(),
            uv0,
            uv1,
        );
    }
    exact
}

/// Splits closed edges and faces.
///
/// # Details
/// The topology of the shapes handled by monstertruck has the following rules
/// - The endpoints of the edges must be different.
/// - The boundaries of the faces must be a simple wire.
///
/// Shapes created in other CAD systems do not necessarily follow these rules.
/// When such shapes are handled by monstertruck, this method is applied at the
/// stage of `CompressedShell` and `CompressedSolid`, which are intermediate
/// forms.
///
/// Parameter inversion falls back through [`SearchParameter`] and then
/// [`SearchNearestParameter`], with and without a hint. Imported CAD routinely
/// carries points a hair off their surface, and an exact-only inversion
/// silently declines to split those faces -- the caller gets an unhealed shell
/// with no error, and it breaks somewhere else entirely.
///
/// # Remarks
/// Boundary simplification is still only implemented for cylinders.
/// It has not yet been implemented for cases involving singularities, such as spherical surfaces.
pub trait SplitClosedEdgesAndFaces {
    /// Splits closed edges and faces
    fn split_closed_edges_and_faces(&mut self, tol: f64);
}

impl<C, S> SplitClosedEdgesAndFaces for CompressedShell<Point3, C, S>
where
    C: ParametricCurve3D
        + BoundedCurve
        + Cut
        + ParameterDivision1D<Point = Point3>
        + ParameterBoundary2D<S>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + SnapCurveEndpoints
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    fn split_closed_edges_and_faces(&mut self, tol: f64) {
        let debug_trace = env::var("MT_BOOL_TRACE").is_ok();
        let enable_pass_through = env::var("MT_BOOL_DISABLE_PASS_THROUGH_SPLIT").is_err();
        let force_single_face_pass_through =
            env::var("MT_BOOL_FORCE_SINGLE_FACE_PASS_THROUGH").is_ok();
        let skip_single_face_pass_through =
            self.faces.len() <= 1 && !force_single_face_pass_through;
        let started = Instant::now();
        fn sp<S>(surface: &S, point: Point3, hint: Option<(f64, f64)>) -> Option<(f64, f64)>
        where S: SearchParameter<SurfaceParameter, Point = Point3>
                + SearchNearestParameter<SurfaceParameter, Point = Point3> {
            surface
                .search_parameter(point, hint, 100)
                .or_else(|| surface.search_parameter(point, None, 100))
                .or_else(|| surface.search_nearest_parameter(point, hint, 100))
                .or_else(|| surface.search_nearest_parameter(point, None, 100))
        }
        if debug_trace {
            eprintln!(
                "trace bool robust_split start vertices={} edges={} faces={}",
                self.vertices.len(),
                self.edges.len(),
                self.faces.len(),
            );
        }
        split_closed_edges(self);
        if debug_trace {
            eprintln!(
                "trace bool robust_split after_edges elapsed_ms={:.3}",
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
        if enable_pass_through && !skip_single_face_pass_through {
            split_pass_through_edges(self, tol);
        }
        if debug_trace {
            eprintln!(
                "trace bool robust_split after_pass_through elapsed_ms={:.3} skipped={}",
                started.elapsed().as_secs_f64() * 1000.0,
                skip_single_face_pass_through || !enable_pass_through,
            );
        }
        let faces_changed = split_closed_faces(self, tol, sp);
        if debug_trace {
            eprintln!(
                "trace bool robust_split after_faces elapsed_ms={:.3} changed={faces_changed}",
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
    }
}

impl<C, S> SplitClosedEdgesAndFaces for CompressedSolid<Point3, C, S>
where
    C: ParametricCurve3D
        + BoundedCurve
        + Cut
        + ParameterDivision1D<Point = Point3>
        + ParameterBoundary2D<S>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + SnapCurveEndpoints
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    fn split_closed_edges_and_faces(&mut self, tol: f64) {
        let fs = SplitClosedEdgesAndFaces::split_closed_edges_and_faces;
        self.boundaries.iter_mut().for_each(|shell| fs(shell, tol))
    }
}

impl<C, S, T> SplitClosedEdgesAndFaces for CompressedTrimmedShell<Point3, C, S, T>
where
    C: ParametricCurve3D
        + BoundedCurve
        + Cut
        + ParameterDivision1D<Point = Point3>
        + ParameterBoundary2D<S>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + SnapCurveEndpoints
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + ExactParameterBoundary2D<S, BoundaryCurve = T>
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + Clone,
    T: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible,
{
    fn split_closed_edges_and_faces(&mut self, tol: f64) {
        let debug_trace = env::var("MT_BOOL_TRACE").is_ok();
        let debug_heal_split = env::var("MT_BOOL_DEBUG_HEAL_SPLIT").is_ok();
        let enable_pass_through = env::var("MT_BOOL_DISABLE_PASS_THROUGH_SPLIT").is_err();
        let force_single_face_pass_through =
            env::var("MT_BOOL_FORCE_SINGLE_FACE_PASS_THROUGH").is_ok();
        let skip_single_face_pass_through =
            self.faces.len() <= 1 && !force_single_face_pass_through;
        let started = Instant::now();
        fn sp<S>(surface: &S, point: Point3, hint: Option<(f64, f64)>) -> Option<(f64, f64)>
        where S: SearchParameter<SurfaceParameter, Point = Point3>
                + SearchNearestParameter<SurfaceParameter, Point = Point3> {
            surface
                .search_parameter(point, hint, 100)
                .or_else(|| surface.search_parameter(point, None, 100))
                .or_else(|| surface.search_nearest_parameter(point, hint, 100))
                .or_else(|| surface.search_nearest_parameter(point, None, 100))
        }
        split_closed_edges_trimmed(self);
        if debug_trace {
            eprintln!(
                "trace bool robust_split_trimmed after_edges elapsed_ms={:.3}",
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
        if enable_pass_through && !skip_single_face_pass_through {
            split_pass_through_edges_trimmed(self, tol);
            if env::var("MT_BOOL_DISABLE_PASS_THROUGH_DEDUP").is_err() {
                let mut use_orientations: Vec<Vec<(usize, bool)>> =
                    vec![Vec::new(); self.edges.len()];
                self.faces
                    .iter()
                    .enumerate()
                    .for_each(|(face_index, face)| {
                        face.boundaries
                            .iter()
                            .flat_map(|wire| wire.iter())
                            .for_each(|edge_use| {
                                if let Some(uses) = use_orientations.get_mut(edge_use.index) {
                                    uses.push((face_index, edge_use.orientation));
                                }
                            });
                    });
                let remap =
                    dedup_coincident_pass_through_edges(&self.edges, &use_orientations, tol);
                if !remap.is_empty() {
                    self.faces.iter_mut().for_each(|face| {
                        face.boundaries
                            .iter_mut()
                            .for_each(|wire| remap_trimmed_edge_uses_for_dedup(wire, &remap));
                    });
                    // Compact: orphaned duplicates would fail the compressed
                    // shell's reference validation downstream.
                    let used: std::collections::BTreeSet<usize> = self
                        .faces
                        .iter()
                        .flat_map(|face| face.boundaries.iter())
                        .flat_map(|wire| wire.iter().map(|edge_use| edge_use.index))
                        .collect();
                    let mut new_indices = vec![usize::MAX; self.edges.len()];
                    let mut compacted = Vec::with_capacity(used.len());
                    for (index, edge) in std::mem::take(&mut self.edges).into_iter().enumerate() {
                        if used.contains(&index) {
                            new_indices[index] = compacted.len();
                            compacted.push(edge);
                        }
                    }
                    self.edges = compacted;
                    self.faces.iter_mut().for_each(|face| {
                        face.boundaries.iter_mut().for_each(|wire| {
                            wire.iter_mut()
                                .for_each(|edge_use| edge_use.index = new_indices[edge_use.index]);
                        });
                    });
                }
            }
        }
        if debug_trace {
            eprintln!(
                "trace bool robust_split_trimmed after_pass_through elapsed_ms={:.3} skipped={}",
                started.elapsed().as_secs_f64() * 1000.0,
                skip_single_face_pass_through || !enable_pass_through,
            );
        }
        let mut plain = self.cloned_without_trims();
        let first_faces_changed = split_closed_faces(&mut plain, tol, sp);
        if debug_trace || debug_heal_split {
            eprintln!(
                "trace bool robust_split_trimmed after_faces elapsed_ms={:.3} changed={first_faces_changed}",
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
        if first_faces_changed {
            let original = self.clone();
            if enable_pass_through && !skip_single_face_pass_through {
                split_closed_edges(&mut plain);
                split_pass_through_edges(&mut plain, tol);
                let second_faces_changed = split_closed_faces(&mut plain, tol, sp);
                if debug_trace || debug_heal_split {
                    eprintln!(
                        "trace bool robust_split_trimmed after_faces_propagate elapsed_ms={:.3} changed={second_faces_changed}",
                        started.elapsed().as_secs_f64() * 1000.0,
                    );
                }
            }
            *self = reattach_preserved_face_trims(&original, plain, tol);
        }
        // Resolve any residual periodic-surface seam face that `split_closed_faces`
        // could not divide into simple sub-faces (inward-normal / hole cylinders
        // whose param-space loops carry no positive outer, or seam-crossing halves
        // that wrap the u-period) into its two vertex-disjoint cap loops, so the
        // trimmed solid extracts instead of refusing `NotSimpleWire` at solidify.
        let seam_splits = split_seam_faces_trimmed(self);
        if debug_trace || debug_heal_split {
            eprintln!(
                "trace bool robust_split_trimmed after_seam elapsed_ms={:.3} seam_splits={seam_splits}",
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
    }
}

impl<C, S, T> SplitClosedEdgesAndFaces for CompressedTrimmedSolid<Point3, C, S, T>
where
    C: ParametricCurve3D
        + BoundedCurve
        + Cut
        + ParameterDivision1D<Point = Point3>
        + ParameterBoundary2D<S>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + SnapCurveEndpoints
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + ExactParameterBoundary2D<S, BoundaryCurve = T>
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + Clone,
    T: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible,
{
    fn split_closed_edges_and_faces(&mut self, tol: f64) {
        let fs = SplitClosedEdgesAndFaces::split_closed_edges_and_faces;
        self.boundaries.iter_mut().for_each(|shell| fs(shell, tol))
    }
}

/// Convenience function: heal a compressed shell and extract it.
///
/// Applies [`SplitClosedEdgesAndFaces`] before calling
/// [`Shell::extract`](monstertruck_topology::Shell::extract), which avoids
/// `NotSimpleWire` errors from STEP files with repeated vertices.
pub fn extract_healed<C, S>(
    mut cshell: CompressedShell<Point3, C, S>,
    tol: f64,
) -> monstertruck_topology::Result<monstertruck_topology::Shell<Point3, C, S>>
where
    C: ParametricCurve3D
        + BoundedCurve
        + Cut
        + ParameterDivision1D<Point = Point3>
        + ParameterBoundary2D<S>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + SnapCurveEndpoints
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + Clone,
{
    cshell.split_closed_edges_and_faces(tol);
    monstertruck_topology::Shell::extract(cshell)
}

/// Convenience function: heal a compressed trimmed shell and extract it while
/// preserving exact face-local trim curves when the edge curves can supply them.
pub fn extract_healed_trimmed<C, S, T>(
    mut cshell: CompressedTrimmedShell<Point3, C, S, T>,
    tol: f64,
) -> monstertruck_topology::Result<TrimmedShell<Point3, C, S, T>>
where
    C: ParametricCurve3D
        + BoundedCurve
        + Cut
        + ParameterDivision1D<Point = Point3>
        + ParameterBoundary2D<S>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + SnapCurveEndpoints
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + ExactParameterBoundary2D<S, BoundaryCurve = T>
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + Clone,
    T: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible,
{
    if let Ok(shell) = TrimmedShell::try_from(cshell.clone()) {
        Ok(shell)
    } else {
        cshell.split_closed_edges_and_faces(tol);
        TrimmedShell::try_from(cshell)
    }
}

/// Convenience function: heal a compressed trimmed solid and extract it while
/// preserving exact face-local trim curves when the edge curves can supply them.
///
/// # Refuses a PARTIAL input instead of returning one (ledger class C11)
///
/// A `CompressedTrimmedSolid` can arrive short of faces: the STEP loader
/// converts a shell face by face, so a typed surface refusal -- spec 011 T1's
/// degenerate-torus refusal, for instance -- drops that face and the shell is
/// still returned as `Ok`. Healing cannot repair that; the shell it is handed
/// simply has a hole.
///
/// This function therefore ends in
/// [`TrimmedSolid::try_new`](monstertruck_topology::trimmed::TrimmedSolid::try_new)
/// and refuses such an input with the same typed error the plain path has
/// always used -- `NotClosedShell`, `NotConnected`, `NotManifold`,
/// `EmptyShell`. Measured over every in-repo fixture (15 files, 185 solids,
/// spec 011 C11 work): zero change, all 185 already satisfied the invariant.
/// Measured over `ROTOR-201NAL-Z7.STEP`: 3 of 33 solids move from `Ok`-and-
/// abort-downstream to a typed refusal here.
///
/// **What the error does NOT tell you** is WHY the shell is short -- that
/// knowledge belongs to the loader, which is a layer above this crate. A
/// caller who needs the reason should convert through the STEP loader's
/// `*_reported` conversions (spec 011 T7) and read the `ShellLoadReport`
/// alongside this refusal.
pub fn extract_healed_trimmed_solid<C, S, T>(
    mut csolid: CompressedTrimmedSolid<Point3, C, S, T>,
    tol: f64,
) -> monstertruck_topology::Result<TrimmedSolid<Point3, C, S, T>>
where
    C: ParametricCurve3D
        + BoundedCurve
        + Cut
        + ParameterDivision1D<Point = Point3>
        + ParameterBoundary2D<S>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + SnapCurveEndpoints
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + ExactParameterBoundary2D<S, BoundaryCurve = T>
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + Clone,
    T: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible,
{
    let solid = if let Ok(solid) = TrimmedSolid::try_from(csolid.clone()) {
        solid
    } else {
        csolid.split_closed_edges_and_faces(tol);
        TrimmedSolid::try_from(csolid)?
    };
    // Real-world STEP files routinely ship shells whose faces disagree on
    // edge direction (shell condition `Regular`); heal them to `Closed`.
    //
    // `try_new` and not `new`: orientation healing is the LAST repair this
    // function has, so whatever is still wrong after it is not repairable
    // here and must leave as a typed refusal rather than as an `Ok` that
    // violates `Solid`'s precondition. See the C11 note on the item docs.
    TrimmedSolid::try_new(
        solid
            .into_boundaries()
            .into_iter()
            .map(|shell| normalize_trimmed_shell_orientation(shell).0)
            .collect(),
    )
}

/// Outcome of [`normalize_shell_orientation`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OrientationNormalization {
    /// Faces flipped to restore pairwise-opposite edge use.
    pub flipped_faces: usize,
    /// Orientation contradictions (non-orientable configurations); the
    /// affected faces are left as they were.
    pub conflicts: usize,
    /// Edge uses not shared by exactly two face boundaries; such edges carry
    /// no orientation constraint and are skipped.
    pub irregular_edges: usize,
}

/// Which faces must flip (relative to the shell's current state) so that
/// every edge shared by two faces is traversed in opposite directions.
fn shell_orientation_flips<P, C, S>(
    shell: &monstertruck_topology::Shell<P, C, S>,
) -> (Vec<bool>, OrientationNormalization) {
    let mut uses: HashMap<_, Vec<(usize, bool)>> = HashMap::default();
    shell.iter().enumerate().for_each(|(face_index, face)| {
        face.boundary_iters().into_iter().for_each(|boundary| {
            boundary.for_each(|edge| {
                uses.entry(edge.id())
                    .or_default()
                    .push((face_index, edge.orientation()));
            });
        });
    });

    let mut conflicts = 0usize;
    let mut irregular_edges = 0usize;
    let mut adjacency: Vec<Vec<(usize, bool)>> = vec![Vec::new(); shell.len()];
    uses.values().for_each(|edge_uses| {
        let [(face0, direction0), (face1, direction1)] = edge_uses[..] else {
            irregular_edges += 1;
            return;
        };
        if face0 == face1 {
            // A closed face traverses its seam edge twice; a whole-face flip
            // cannot repair a same-direction seam.
            if direction0 == direction1 {
                conflicts += 1;
            }
            return;
        }
        let same_direction = direction0 == direction1;
        adjacency[face0].push((face1, same_direction));
        adjacency[face1].push((face0, same_direction));
    });

    let mut flip = vec![false; shell.len()];
    let mut assigned = vec![false; shell.len()];
    let mut queue = std::collections::VecDeque::new();
    for seed in 0..shell.len() {
        if assigned[seed] {
            continue;
        }
        assigned[seed] = true;
        queue.push_back(seed);
        while let Some(face) = queue.pop_front() {
            for &(other, same_direction) in &adjacency[face] {
                // Opposite effective directions require
                // flip[other] = flip[face] XOR same_direction.
                let required = flip[face] ^ same_direction;
                if !assigned[other] {
                    assigned[other] = true;
                    flip[other] = required;
                    queue.push_back(other);
                } else if flip[other] != required {
                    conflicts += 1;
                }
            }
        }
    }

    let flipped_faces = flip.iter().filter(|flip| **flip).count();
    (
        flip,
        OrientationNormalization {
            flipped_faces,
            conflicts,
            irregular_edges,
        },
    )
}

/// Restores consistent face orientation on a shell whose faces disagree on
/// edge direction (shell condition `Regular`): flood-fills face flips so
/// every shared edge is traversed in opposite directions by its two faces.
///
/// Purely topological -- it never inspects geometry, and it does NOT decide
/// global outwardness (an all-flipped shell is equally consistent; boolean
/// entry points already detect inverted shells).
pub fn normalize_shell_orientation<P, C, S>(
    shell: &mut monstertruck_topology::Shell<P, C, S>,
) -> OrientationNormalization {
    let (flips, outcome) = shell_orientation_flips(shell);
    flips.into_iter().enumerate().for_each(|(index, flip)| {
        if flip {
            shell[index].invert();
        }
    });
    outcome
}

/// Trimmed variant of [`normalize_shell_orientation`]. Flipping is a face
/// orientation flag; the absolute-boundary trim layout is unaffected, so the
/// stored trims stay valid.
pub fn normalize_trimmed_shell_orientation<P: Clone, C: Clone, S: Clone, T: Clone>(
    shell: TrimmedShell<P, C, S, T>,
) -> (TrimmedShell<P, C, S, T>, OrientationNormalization) {
    let plain: monstertruck_topology::Shell<P, C, S> = shell
        .faces()
        .iter()
        .map(|face| face.face().clone())
        .collect();
    let (flips, outcome) = shell_orientation_flips(&plain);
    if outcome.flipped_faces == 0 {
        return (shell, outcome);
    }
    let rebuilt = shell
        .faces()
        .iter()
        .zip(flips)
        .map(|(trimmed_face, flip)| {
            if flip {
                let mut face = trimmed_face.face().clone();
                face.invert();
                monstertruck_topology::trimmed::TrimmedFace::new(face, trimmed_face.trims().clone())
            } else {
                trimmed_face.clone()
            }
        })
        .collect();
    (rebuilt, outcome)
}

#[cfg(test)]
mod tests;
