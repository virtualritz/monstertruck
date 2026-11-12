//! Conservative welding of naked edges in a compressed shell.
//!
//! A naked edge is an edge used by exactly one face boundary. Two faces built
//! separately that should meet along a seam leave two naked edges a small gap
//! apart, each with its own end vertices. [`snap_naked_edges`] finds the pairs
//! of naked edges that provably match within a tolerance, makes the two faces
//! reference one shared edge, and merges the matching end vertices. A pair it
//! cannot weld safely stays naked and is reported with its reason; a weld that
//! would break the topology, or input it cannot reason about, is refused with
//! a [`SnapError`] and the shell is left untouched.

use super::*;
use monstertruck_topology::StableId;
use std::{collections::BTreeMap, fmt};

/// Each curve is flattened into a polyline no farther than this fraction of
/// the tolerance from the curve.
const FLATNESS_FRACTION: f64 = 1.0 / 16.0;
/// Polyline segments allowed per curve. A curve that needs more (a long or
/// wiggly curve against a tiny tolerance) is not bounded, so not welded.
const MAX_POLYLINE_SEGMENTS: usize = 1 << 14;
/// Recursion depth allowed when flattening one Bezier span.
const MAX_FLATTEN_DEPTH: u32 = 40;

/// Why a pair of naked edges whose ends matched within the tolerance was left
/// naked. Edges are indices into the input shell.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum SkipReason {
    /// The curves are provably farther apart than the tolerance somewhere:
    /// their distance is at least `at_least`.
    CurvesDiffer {
        /// A lower bound on the largest distance between the two curves.
        at_least: f64,
    },
    /// The curves could not be proven to lie within the tolerance of each
    /// other: a curve has no exact B-spline form, the proof would need more
    /// work than the limit allows, or the best upper bound stays above the
    /// tolerance although the curves may agree.
    DeviationNotBounded,
    /// Both correspondences of the ends hold (both edges are shorter than the
    /// tolerance), so which end matches which is a guess.
    AmbiguousEnds,
    /// The curves match, but their faces traverse them in the same direction.
    /// Welding them would make the shell non-orientable.
    IncompatibleOrientation,
    /// Welding would put two merged vertices farther than the tolerance apart,
    /// through a vertex an earlier weld already merged.
    ToleranceCreep,
}

/// Outcome of a successful [`snap_naked_edges`].
///
/// The mappings let a caller carry its own per-vertex and per-edge attributes
/// across the weld. Stable IDs on the shell are already remapped: see
/// [`snap_naked_edges`] for the survivor policy.
///
/// The residual between a reused curve and the surface of the face that now
/// uses it is NOT measured: the surface type is unconstrained, and the weld
/// never reads a surface. It is bounded only indirectly, by the curve
/// deviation plus however far the dropped curve was from that surface.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct SnapReport {
    /// Vertices merged into another vertex and removed from the shell.
    pub merged_vertices: usize,
    /// Pairs of naked edges welded into one shared edge. Each pair removed one
    /// edge from the shell.
    pub merged_edges: usize,
    /// Naked edges (edges used by exactly one face boundary) before the weld.
    pub naked_edges_before: usize,
    /// Naked edges after the weld.
    pub naked_edges_after: usize,
    /// The edges still naked after the weld, as ascending indices into the
    /// welded shell's `edges`. Empty means every edge is shared.
    pub naked_edges: Vec<usize>,
    /// Pairs of naked edges whose ends matched within the tolerance but that
    /// were left naked, as `(lower edge, higher edge, reason)` with indices
    /// into the input shell, in ascending order.
    pub skipped: Vec<(usize, usize, SkipReason)>,
    /// The largest distance from a merged vertex's original position to the
    /// position of the vertex it merged into. Never more than the tolerance.
    pub max_vertex_displacement: f64,
    /// The largest upper bound, over the welded pairs, on the distance between
    /// the two curves of a pair. It is a bound, not a sample: each curve is
    /// flattened through its exact B-spline form with a certified flatness,
    /// and the polylines are compared with a certified segment bound. Never
    /// more than the tolerance.
    pub max_curve_deviation: f64,
    /// The largest distance between a curve end whose vertex moved and that
    /// vertex, after the curve's endpoint hints were re-aligned. Lines,
    /// B-splines and NURBS keep their geometry, so this is the gap the weld
    /// leaves at moved corners. Ends whose vertex did not move are not
    /// counted, so a gap that was there before the weld is not reported as the
    /// weld's.
    pub max_curve_endpoint_gap: f64,
    /// For every original vertex, its index in the welded shell.
    pub vertex_map: Vec<usize>,
    /// For every original edge, the edge of the welded shell that carries it,
    /// with `orientation == true` when the two run in the same direction.
    pub edge_map: Vec<CompressedEdgeIndex>,
}

impl SnapReport {
    /// Whether the weld changed the shell.
    pub fn changed(&self) -> bool { self.merged_vertices > 0 || self.merged_edges > 0 }

    fn unchanged<P, C, S>(
        shell: &CompressedShell<P, C, S>,
        naked_edges: Vec<usize>,
        skipped: Vec<(usize, usize, SkipReason)>,
    ) -> Self {
        SnapReport {
            merged_vertices: 0,
            merged_edges: 0,
            naked_edges_before: naked_edges.len(),
            naked_edges_after: naked_edges.len(),
            naked_edges,
            skipped,
            max_vertex_displacement: 0.0,
            max_curve_deviation: 0.0,
            max_curve_endpoint_gap: 0.0,
            vertex_map: (0..shell.vertices.len()).collect(),
            edge_map: (0..shell.edges.len())
                .map(|index| CompressedEdgeIndex {
                    index,
                    orientation: true,
                })
                .collect(),
        }
    }
}

/// Why [`snap_naked_edges`] refused to weld. The shell is unchanged.
///
/// Indices refer to the input shell.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum SnapError {
    /// The tolerance is negative, infinite or NaN.
    InvalidTolerance(f64),
    /// An edge refers to a missing vertex, or a face to a missing edge.
    IndexOutOfRange,
    /// A stable-ID array does not have one entry per element.
    StableIdLengthMismatch,
    /// The edge is already used by three or more face boundaries in a shell
    /// that has naked edges, or welding would make it shared by three or more
    /// faces: a non-manifold join.
    ThreeEdgeUses {
        /// The edge.
        edge: usize,
    },
    /// The weld would merge both end vertices of this edge, which were
    /// distinct before.
    CollapsedEdge {
        /// The edge.
        edge: usize,
    },
    /// The weld would make a boundary wire of this face, simple before, visit
    /// a vertex twice.
    NonSimpleWire {
        /// The face.
        face: usize,
    },
    /// The faces around the merged vertex would form more fans (groups joined
    /// through shared edges) than any of the merged vertices had before: the
    /// weld creates or worsens a pinch.
    SingularVertex {
        /// The lowest original index among the merged vertices.
        vertex: usize,
    },
    /// After the weld, an end of this edge's curve would lie farther than the
    /// tolerance from its moved vertex, farther than it was from its original
    /// vertex, and the curve cannot be moved there.
    CurveEndpointOutOfTolerance {
        /// The edge.
        edge: usize,
    },
}

impl fmt::Display for SnapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SnapError::InvalidTolerance(tolerance) => {
                write!(
                    f,
                    "weld tolerance {tolerance} is not finite and non-negative"
                )
            }
            SnapError::IndexOutOfRange => write!(f, "the shell has an out-of-range index"),
            SnapError::StableIdLengthMismatch => {
                write!(f, "a stable-ID array does not match its element count")
            }
            SnapError::ThreeEdgeUses { edge } => {
                write!(f, "edge {edge} joins or would join three or more faces")
            }
            SnapError::CollapsedEdge { edge } => write!(f, "the weld would collapse edge {edge}"),
            SnapError::NonSimpleWire { face } => {
                write!(f, "the weld would make a wire of face {face} non-simple")
            }
            SnapError::SingularVertex { vertex } => {
                write!(f, "the weld would pinch the faces around vertex {vertex}")
            }
            SnapError::CurveEndpointOutOfTolerance { edge } => write!(
                f,
                "the weld would leave the curve of edge {edge} off its vertex"
            ),
        }
    }
}

impl std::error::Error for SnapError {}

/// Welds the naked edges of `shell` that provably match within `tolerance`.
///
/// Two naked edges match when they belong to different faces, each end of one
/// lies within `tolerance` of an end of the other, the curves are proven to
/// stay within `tolerance` of each other along their whole length, and their
/// faces traverse them in opposite directions. A matched pair becomes one
/// shared edge: the faces reference the lower-indexed edge and its curve, the
/// matching end vertices merge into the lower-indexed vertex, and every
/// oriented edge use is remapped so each face traverses its boundary in the
/// direction it did before.
///
/// The curve proof needs each curve's exact B-spline form
/// ([`TryIntoHomogeneousBsplineCurve`]); lines, B-splines and NURBS have one.
/// The conversion is checked against the curve at every knot and span
/// midpoint. A curve without one is never welded.
///
/// Guarantees:
///
/// - Nothing moves farther than `tolerance` from its original position.
///   Merging is not transitive: a pair whose weld would put two merged
///   vertices farther than `tolerance` apart stays naked.
/// - Face order, surfaces and orientation flags are unchanged. No face is
///   added or removed and no edge is collapsed.
/// - The shell is not forced closed. Unmatched edges stay naked and are listed
///   in [`SnapReport::naked_edges`]; pairs whose ends matched but that were
///   not welded are listed with their reason in [`SnapReport::skipped`].
/// - The result depends only on the input and its indices, never on hashing.
/// - Curves keep their geometry. Endpoint hints are re-aligned through
///   [`SnapCurveEndpoints`], and every curve end at a moved vertex is checked
///   to lie within `tolerance` of it; the remaining gap is reported in
///   [`SnapReport::max_curve_endpoint_gap`].
///
/// Stable IDs: a merged vertex or edge keeps the ID of its lowest original
/// index; the IDs of the elements merged into it are dropped. Face IDs are
/// unchanged. Use [`SnapReport::vertex_map`] and [`SnapReport::edge_map`] for
/// any other per-element data.
///
/// `tolerance == 0.0` returns at once and changes nothing, as does a shell
/// without naked edges (even a non-manifold one).
///
/// # Errors
///
/// Refusals are transactional: on any error the shell is unchanged.
/// [`SnapError`] lists the cases: an invalid tolerance, index or stable-ID
/// array; an edge already used three or more times; a weld that would join
/// three faces on one edge; and a weld that would itself collapse an edge,
/// make a simple wire non-simple, create or worsen a pinched vertex, or leave
/// a curve end off its moved vertex. Defects the input already had do not
/// count against the weld.
pub fn snap_naked_edges<C, S>(
    shell: &mut CompressedShell<Point3, C, S>,
    tolerance: f64,
) -> std::result::Result<SnapReport, SnapError>
where
    C: SnapCurveEndpoints
        + ParametricCurve<Point = Point3, Vector = Vector3>
        + TryIntoHomogeneousBsplineCurve,
{
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(SnapError::InvalidTolerance(tolerance));
    }
    if tolerance == 0.0 {
        return Ok(SnapReport::unchanged(
            shell,
            naked_edge_indices(shell),
            Vec::new(),
        ));
    }
    validate_indices(shell)?;
    let uses = edge_uses(shell);
    let naked: Vec<usize> = (0..uses.len())
        .filter(|&edge| uses[edge].len() == 1)
        .collect();
    if naked.is_empty() {
        return Ok(SnapReport::unchanged(shell, naked, Vec::new()));
    }
    if let Some(edge) = uses.iter().position(|uses| uses.len() > 2) {
        return Err(SnapError::ThreeEdgeUses { edge });
    }
    let (candidates, mut skipped) = matching_pairs(shell, &uses, &naked, tolerance);
    let (target, accepted) = accept_without_creep(shell, &candidates, tolerance, &mut skipped);
    skipped.sort_by_key(|&(keep, drop, _)| (keep, drop));
    let mut matches = vec![0usize; shell.edges.len()];
    accepted.iter().for_each(|pair| {
        matches[pair.keep] += 1;
        matches[pair.drop] += 1;
    });
    if let Some(edge) = matches.iter().position(|&count| count > 1) {
        return Err(SnapError::ThreeEdgeUses { edge });
    }
    if accepted.is_empty() {
        return Ok(SnapReport::unchanged(shell, naked, skipped));
    }
    let welded = build_welded(shell, &target, &accepted, tolerance)?;
    Ok(welded.commit(shell, naked.len(), skipped))
}

fn validate_indices<P, C, S>(
    shell: &CompressedShell<P, C, S>,
) -> std::result::Result<(), SnapError> {
    let vertex_count = shell.vertices.len();
    let edge_count = shell.edges.len();
    let edges_ok = shell
        .edges
        .iter()
        .all(|edge| edge.vertices.0 < vertex_count && edge.vertices.1 < vertex_count);
    let uses_ok = shell
        .faces
        .iter()
        .flat_map(|face| face.boundaries.iter().flatten())
        .all(|edge_use| edge_use.index < edge_count);
    if !edges_ok || !uses_ok {
        return Err(SnapError::IndexOutOfRange);
    }
    let length_ok = |ids: &Option<Vec<StableId>>, count: usize| {
        ids.as_ref().is_none_or(|ids| ids.len() == count)
    };
    let ids_ok = length_ok(&shell.vertex_stable_ids, vertex_count)
        && length_ok(&shell.edge_stable_ids, edge_count)
        && length_ok(&shell.face_stable_ids, shell.faces.len());
    match ids_ok {
        true => Ok(()),
        false => Err(SnapError::StableIdLengthMismatch),
    }
}

/// Indices of the edges used by exactly one face boundary. Uses whose index is
/// out of range are ignored.
fn naked_edge_indices<P, C, S>(shell: &CompressedShell<P, C, S>) -> Vec<usize> {
    let mut counts = vec![0usize; shell.edges.len()];
    shell
        .faces
        .iter()
        .flat_map(|face| face.boundaries.iter().flatten())
        .for_each(|edge_use| {
            if let Some(count) = counts.get_mut(edge_use.index) {
                *count += 1;
            }
        });
    counts
        .into_iter()
        .enumerate()
        .filter_map(|(index, count)| (count == 1).then_some(index))
        .collect()
}

/// For every edge, its uses: the face, and whether that face traverses the edge
/// from its front to its back once the face orientation is applied.
fn edge_uses<P, C, S>(shell: &CompressedShell<P, C, S>) -> Vec<Vec<(usize, bool)>> {
    let mut uses = vec![Vec::new(); shell.edges.len()];
    shell
        .faces
        .iter()
        .enumerate()
        .for_each(|(face_index, face)| {
            face.boundaries.iter().flatten().for_each(|edge_use| {
                uses[edge_use.index].push((face_index, edge_use.orientation == face.orientation));
            });
        });
    uses
}

/// A matched pair of naked edges, `keep < drop`.
#[derive(Clone, Copy, Debug)]
struct Pair {
    keep: usize,
    drop: usize,
    /// `true` when `keep`'s front matches `drop`'s back (and back matches
    /// front); `false` when front matches front.
    reversed: bool,
    deviation: f64,
}

impl Pair {
    /// The two vertex identifications this weld makes, `(keep's, drop's)`.
    fn identifications<C>(&self, edges: &[CompressedEdge<C>]) -> [(usize, usize); 2] {
        let (k0, k1) = edges[self.keep].vertices;
        let (d0, d1) = edges[self.drop].vertices;
        match self.reversed {
            true => [(k0, d1), (k1, d0)],
            false => [(k0, d0), (k1, d1)],
        }
    }
}

type BucketKey = [i64; 3];

/// The grid cell of `point`. A coordinate whose quotient overflows `i64`
/// saturates; the neighbourhood below saturates too, so two points within one
/// cell size still land in neighbouring cells.
fn bucket_key(point: Point3, bucket_size: f64) -> BucketKey {
    [
        (point.x / bucket_size).floor() as i64,
        (point.y / bucket_size).floor() as i64,
        (point.z / bucket_size).floor() as i64,
    ]
}

fn neighboring_bucket_keys(key: BucketKey) -> impl Iterator<Item = BucketKey> {
    let shifted = |value: i64, delta: i64| value.saturating_add(delta);
    let mut keys: Vec<BucketKey> = [-1i64, 0, 1]
        .into_iter()
        .flat_map(move |dx| {
            [-1i64, 0, 1].into_iter().flat_map(move |dy| {
                [-1i64, 0, 1].into_iter().map(move |dz| {
                    [
                        shifted(key[0], dx),
                        shifted(key[1], dy),
                        shifted(key[2], dz),
                    ]
                })
            })
        })
        .collect();
    keys.sort_unstable();
    keys.dedup();
    keys.into_iter()
}

/// Finds the pairs of naked edges whose ends correspond, proves their curves
/// agree and checks their orientation. Returns the weldable pairs in
/// `(keep, drop)` order and the pairs left naked with their reason.
#[allow(clippy::type_complexity)]
fn matching_pairs<C, S>(
    shell: &CompressedShell<Point3, C, S>,
    uses: &[Vec<(usize, bool)>],
    naked: &[usize],
    tolerance: f64,
) -> (Vec<Pair>, Vec<(usize, usize, SkipReason)>)
where
    C: ParametricCurve<Point = Point3, Vector = Vector3> + TryIntoHomogeneousBsplineCurve,
{
    let tol2 = tolerance * tolerance;
    let near = |a: usize, b: usize| shell.vertices[a].distance2(shell.vertices[b]) <= tol2;

    // Naked edges by the bucket of each of their end vertices.
    let mut buckets = HashMap::<BucketKey, Vec<usize>>::default();
    naked.iter().for_each(|&edge| {
        let (front, back) = shell.edges[edge].vertices;
        [front, back].into_iter().for_each(|vertex| {
            buckets
                .entry(bucket_key(shell.vertices[vertex], tolerance))
                .or_default()
                .push(edge);
        });
    });

    let flatness = tolerance * FLATNESS_FRACTION;
    let mut polylines: Vec<Option<Option<BoundedPolyline>>> = vec![None; shell.edges.len()];
    let mut polyline_of = |edge: usize| {
        polylines[edge]
            .get_or_insert_with(|| bounded_polyline(&shell.edges[edge].curve, flatness))
            .clone()
    };

    let mut pairs = Vec::new();
    let mut skipped = Vec::new();
    naked.iter().for_each(|&keep| {
        let (k0, k1) = shell.edges[keep].vertices;
        let mut candidates: Vec<usize> =
            neighboring_bucket_keys(bucket_key(shell.vertices[k0], tolerance))
                .filter_map(|key| buckets.get(&key))
                .flatten()
                .copied()
                .filter(|&other| other > keep && uses[other][0].0 != uses[keep][0].0)
                .collect();
        candidates.sort_unstable();
        candidates.dedup();
        candidates.into_iter().for_each(|drop| {
            let (d0, d1) = shell.edges[drop].vertices;
            let reversed = near(k0, d1) && near(k1, d0);
            let aligned = near(k0, d0) && near(k1, d1);
            if !reversed && !aligned {
                return;
            }
            if reversed && aligned {
                skipped.push((keep, drop, SkipReason::AmbiguousEnds));
                return;
            }
            let deviation = match (polyline_of(keep), polyline_of(drop)) {
                (Some(lhs), Some(rhs)) => certified_deviation(&lhs, &rhs, tolerance),
                _ => Deviation::NotBounded,
            };
            let deviation = match deviation {
                Deviation::Within(bound) => bound,
                Deviation::AtLeast(at_least) => {
                    skipped.push((keep, drop, SkipReason::CurvesDiffer { at_least }));
                    return;
                }
                Deviation::NotBounded => {
                    skipped.push((keep, drop, SkipReason::DeviationNotBounded));
                    return;
                }
            };
            // Opposite traversal means: with fronts matched to backs the faces
            // run the edges the same way relative to their own fronts; with
            // fronts matched to fronts they run them oppositely.
            let forward_keep = uses[keep][0].1;
            let forward_drop = uses[drop][0].1;
            if (forward_keep == forward_drop) != reversed {
                skipped.push((keep, drop, SkipReason::IncompatibleOrientation));
                return;
            }
            pairs.push(Pair {
                keep,
                drop,
                reversed,
                deviation,
            });
        });
    });
    (pairs, skipped)
}

/// A polyline with its vertices on a curve, and an upper bound on the
/// Hausdorff distance between the two.
#[derive(Clone, Debug)]
struct BoundedPolyline {
    points: Vec<Point3>,
    error: f64,
}

/// Flattens `curve` through its exact homogeneous B-spline form into a
/// polyline within `flatness` of it, or `None` when that cannot be certified.
///
/// Each Bezier span with positive weights lies in the convex hull of its
/// projected control points and interpolates its first and last one, so the
/// span is within `d` of its chord (and the chord within `d` of the span),
/// where `d` is the largest distance from a control point to the chord. Spans
/// are halved by de Casteljau until `d <= flatness`.
fn bounded_polyline<C>(curve: &C, flatness: f64) -> Option<BoundedPolyline>
where C: ParametricCurve<Point = Point3, Vector = Vector3> + TryIntoHomogeneousBsplineCurve {
    let homogeneous = curve.try_into_homogeneous_bspline_curve()?;
    if !conversion_matches(curve, &homogeneous) {
        return None;
    }
    let mut polyline = BoundedPolyline {
        points: Vec::new(),
        error: 0.0,
    };
    let mut budget = MAX_POLYLINE_SEGMENTS;
    homogeneous
        .bezier_decomposition()
        .iter()
        .try_for_each(|span| {
            flatten_span(
                span.control_points(),
                flatness,
                &mut polyline,
                &mut budget,
                MAX_FLATTEN_DEPTH,
            )
        })?;
    (!polyline.points.is_empty()).then_some(polyline)
}

/// Whether the conversion spans the curve's own parameter range and agrees
/// with the curve at every distinct knot and between each pair of them. This
/// guards against a conversion that is only an approximation.
fn conversion_matches<C>(curve: &C, homogeneous: &BsplineCurve<Vector4>) -> bool
where C: ParametricCurve<Point = Point3, Vector = Vector3> {
    let rational = NurbsCurve::new(homogeneous.clone());
    let (Some((t0, t1)), Some((s0, s1))) = (curve.try_range_tuple(), rational.try_range_tuple())
    else {
        return false;
    };
    let same = |a: f64, b: f64| (a - b).abs() <= TOLERANCE * (1.0 + a.abs().max(b.abs()));
    if !same(t0, s0) || !same(t1, s1) {
        return false;
    }
    let (knots, _) = homogeneous.knot_vector().to_single_multi();
    let mut parameters: Vec<f64> = knots.clone();
    parameters.extend(knots.windows(2).map(|pair| 0.5 * (pair[0] + pair[1])));
    parameters
        .into_iter()
        .filter(|t| (t0..=t1).contains(t))
        .all(|t| curve.evaluate(t).distance(rational.evaluate(t)) <= TOLERANCE)
}

fn flatten_span(
    control: &[Vector4],
    flatness: f64,
    polyline: &mut BoundedPolyline,
    budget: &mut usize,
    depth: u32,
) -> Option<()> {
    let weights_ok = control
        .iter()
        .all(|point| point.w > 0.0 && point.w.is_finite());
    if control.len() < 2 || !weights_ok {
        return None;
    }
    let projected: Vec<Point3> = control
        .iter()
        .map(|point| Point3::new(point.x / point.w, point.y / point.w, point.z / point.w))
        .collect();
    let (start, end) = (projected[0], projected[projected.len() - 1]);
    let spread = projected
        .iter()
        .map(|&point| distance_to_segment(point, start, end))
        .fold(0.0, f64::max);
    if !spread.is_finite() {
        return None;
    }
    if spread <= flatness {
        *budget = budget.checked_sub(1)?;
        if polyline.points.is_empty() {
            polyline.points.push(start);
        }
        polyline.points.push(end);
        polyline.error = polyline.error.max(spread);
        return Some(());
    }
    if depth == 0 {
        return None;
    }
    let (left, right) = halve_bezier(control);
    flatten_span(&left, flatness, polyline, budget, depth - 1)?;
    flatten_span(&right, flatness, polyline, budget, depth - 1)
}

/// De Casteljau at the parameter midpoint, in homogeneous coordinates.
fn halve_bezier(control: &[Vector4]) -> (Vec<Vector4>, Vec<Vector4>) {
    let mut level = control.to_vec();
    let mut left = vec![level[0]];
    let mut right = vec![level[level.len() - 1]];
    while level.len() > 1 {
        level = level
            .windows(2)
            .map(|pair| (pair[0] + pair[1]) * 0.5)
            .collect();
        left.push(level[0]);
        right.push(level[level.len() - 1]);
    }
    right.reverse();
    (left, right)
}

enum Deviation {
    /// The curves are within this upper bound of each other, itself within
    /// the tolerance.
    Within(f64),
    /// The curves are at least this far apart somewhere, more than the
    /// tolerance.
    AtLeast(f64),
    /// Neither could be shown.
    NotBounded,
}

/// Decides whether two curves are within `tolerance` of each other, from their
/// certified polylines.
///
/// For curves `A`, `B` with polylines `PA`, `PB` within `eA`, `eB` of them,
/// `H(A, B) <= eA + eB + H(PA, PB)`, and `H(A, B) >= H(PA, PB) - eA - eB`.
/// The polyline distance is bounded above segment by segment (see
/// [`segment_bound`]) and below by the distance from each vertex.
fn certified_deviation(lhs: &BoundedPolyline, rhs: &BoundedPolyline, tolerance: f64) -> Deviation {
    let slack = lhs.error + rhs.error;
    let lower = f64::max(
        max_vertex_distance(&lhs.points, &rhs.points),
        max_vertex_distance(&rhs.points, &lhs.points),
    ) - slack;
    if lower > tolerance {
        return Deviation::AtLeast(lower);
    }
    let target = (tolerance - slack).max(0.0);
    let upper = slack
        + f64::max(
            one_sided_bound(&lhs.points, &rhs.points, target),
            one_sided_bound(&rhs.points, &lhs.points, target),
        );
    match upper <= tolerance {
        true => Deviation::Within(upper),
        false => Deviation::NotBounded,
    }
}

fn max_vertex_distance(from: &[Point3], to: &[Point3]) -> f64 {
    from.iter()
        .map(|&point| distance_to_polyline(point, to))
        .fold(0.0, f64::max)
}

/// An upper bound on the largest distance from a point of polyline `from` to
/// polyline `to`.
fn one_sided_bound(from: &[Point3], to: &[Point3], target: f64) -> f64 {
    match from {
        [only] => distance_to_polyline(*only, to),
        _ => from
            .windows(2)
            .map(|segment| segment_bound(segment[0], segment[1], to, target))
            .fold(0.0, f64::max),
    }
}

/// An upper bound on `sup` over `x` in segment `[p, q]` of the distance from
/// `x` to polyline `to`.
///
/// The distance from `x` to one segment of `to` is convex along `[p, q]`, so
/// its largest value is at `p` or `q`; the distance to the polyline is at most
/// the distance to any one of its segments. Halving `[p, q]` tightens the
/// bound; it stops once the bound meets `target` or the piece is shorter than
/// a quarter of it.
fn segment_bound(p: Point3, q: Point3, to: &[Point3], target: f64) -> f64 {
    let bound = match to {
        [only] => f64::max(p.distance(*only), q.distance(*only)),
        _ => to
            .windows(2)
            .map(|segment| {
                f64::max(
                    distance_to_segment(p, segment[0], segment[1]),
                    distance_to_segment(q, segment[0], segment[1]),
                )
            })
            .fold(f64::INFINITY, f64::min),
    };
    if bound <= target || p.distance(q) <= 0.25 * target {
        return bound;
    }
    let middle = p.midpoint(q);
    bound.min(f64::max(
        segment_bound(p, middle, to, target),
        segment_bound(middle, q, to, target),
    ))
}

fn distance_to_segment(point: Point3, start: Point3, end: Point3) -> f64 {
    let direction = end - start;
    let length2 = direction.magnitude2();
    let t = match length2 > 0.0 {
        true => ((point - start).dot(direction) / length2).clamp(0.0, 1.0),
        false => 0.0,
    };
    point.distance(start + direction * t)
}

fn distance_to_polyline(point: Point3, polyline: &[Point3]) -> f64 {
    match polyline {
        [only] => point.distance(*only),
        _ => polyline
            .windows(2)
            .map(|segment| distance_to_segment(point, segment[0], segment[1]))
            .fold(f64::INFINITY, f64::min),
    }
}

/// Accepts the pairs in order, skipping any whose vertex identifications would
/// put two merged vertices farther than `tolerance` apart. Returns, for every
/// vertex, the vertex it merges into (itself when it stays), and the accepted
/// pairs. An edge may appear in two accepted pairs; the caller refuses that.
fn accept_without_creep<C, S>(
    shell: &CompressedShell<Point3, C, S>,
    pairs: &[Pair],
    tolerance: f64,
    skipped: &mut Vec<(usize, usize, SkipReason)>,
) -> (Vec<usize>, Vec<Pair>) {
    let tol2 = tolerance * tolerance;
    let mut group_of: Vec<usize> = (0..shell.vertices.len()).collect();
    let mut members: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let members_of = |members: &BTreeMap<usize, Vec<usize>>, group: usize| {
        members.get(&group).cloned().unwrap_or_else(|| vec![group])
    };
    let mut accepted = Vec::new();
    pairs.iter().for_each(|pair| {
        let identifications = pair.identifications(&shell.edges);
        // The groups each identification joins; two identifications that
        // share a group join all of theirs into one.
        let mut components: Vec<Vec<usize>> = identifications
            .iter()
            .map(|&(a, b)| {
                let mut groups = vec![group_of[a], group_of[b]];
                groups.sort_unstable();
                groups.dedup();
                groups
            })
            .collect();
        if components[0]
            .iter()
            .any(|group| components[1].contains(group))
        {
            let second = components.pop().unwrap_or_default();
            components[0].extend(second);
            components[0].sort_unstable();
            components[0].dedup();
        }
        let within_tolerance = components.iter().all(|groups| {
            let points: Vec<Point3> = groups
                .iter()
                .flat_map(|&group| members_of(&members, group))
                .map(|vertex| shell.vertices[vertex])
                .collect();
            points.iter().enumerate().all(|(i, lhs)| {
                points[i + 1..]
                    .iter()
                    .all(|rhs| lhs.distance2(*rhs) <= tol2)
            })
        });
        if !within_tolerance {
            skipped.push((pair.keep, pair.drop, SkipReason::ToleranceCreep));
            return;
        }
        components.into_iter().for_each(|groups| {
            let survivor = groups[0];
            let joined: Vec<usize> = groups
                .iter()
                .flat_map(|&group| {
                    let group_members = members_of(&members, group);
                    members.remove(&group);
                    group_members
                })
                .collect();
            joined
                .iter()
                .for_each(|&vertex| group_of[vertex] = survivor);
            members.insert(survivor, joined);
        });
        accepted.push(*pair);
    });
    // Each group is named by its lowest member, so `group_of` is the target.
    (group_of, accepted)
}

/// The welded shell, built aside so a refusal leaves the input untouched.
struct Welded<C> {
    vertices: Vec<Point3>,
    edges: Vec<CompressedEdge<C>>,
    boundaries: Vec<Vec<Vec<CompressedEdgeIndex>>>,
    vertex_stable_ids: Option<Vec<StableId>>,
    edge_stable_ids: Option<Vec<StableId>>,
    report: SnapReport,
}

fn build_welded<C, S>(
    shell: &CompressedShell<Point3, C, S>,
    target: &[usize],
    accepted: &[Pair],
    tolerance: f64,
) -> std::result::Result<Welded<C>, SnapError>
where
    C: SnapCurveEndpoints + ParametricCurve<Point = Point3, Vector = Vector3>,
{
    let mut dropped: Vec<Option<usize>> = vec![None; shell.edges.len()];
    accepted
        .iter()
        .for_each(|pair| dropped[pair.drop] = Some(pair.keep));

    // Topology on the original indices, with vertices remapped to targets.
    let remapped = |edge: usize| {
        let (front, back) = shell.edges[edge].vertices;
        (target[front], target[back])
    };
    if let Some(edge) = (0..shell.edges.len()).find(|&edge| {
        let (front, back) = remapped(edge);
        let (original_front, original_back) = shell.edges[edge].vertices;
        dropped[edge].is_none() && front == back && original_front != original_back
    }) {
        return Err(SnapError::CollapsedEdge { edge });
    }

    // Original edge -> (surviving original edge, same direction).
    let carrier = |edge: usize| match dropped[edge] {
        Some(keep) => (keep, remapped(edge) == remapped(keep)),
        None => (edge, true),
    };
    let use_front = |edge_use: &CompressedEdgeIndex, vertex_of: &dyn Fn(usize) -> usize| {
        let (front, back) = shell.edges[edge_use.index].vertices;
        match edge_use.orientation {
            true => vertex_of(front),
            false => vertex_of(back),
        }
    };
    let has_repeat = |mut vertices: Vec<usize>| {
        vertices.sort_unstable();
        vertices.windows(2).any(|pair| pair[0] == pair[1])
    };
    for (face_index, face) in shell.faces.iter().enumerate() {
        for wire in &face.boundaries {
            let before: Vec<usize> = wire
                .iter()
                .map(|edge_use| use_front(edge_use, &|v| v))
                .collect();
            let after: Vec<usize> = wire
                .iter()
                .map(|edge_use| use_front(edge_use, &|v| target[v]))
                .collect();
            if has_repeat(after) && !has_repeat(before) {
                return Err(SnapError::NonSimpleWire { face: face_index });
            }
        }
    }

    check_fans(shell, target, &carrier, &remapped)?;

    // Geometry: the surviving curves, endpoint hints re-aligned, moved ends
    // checked against the tolerance and against their gap before the weld.
    let mut max_curve_endpoint_gap = 0.0f64;
    let mut curves = Vec::with_capacity(shell.edges.len());
    for (edge_index, edge) in shell.edges.iter().enumerate() {
        if dropped[edge_index].is_some() {
            continue;
        }
        let (front, back) = edge.vertices;
        let (front_moved, back_moved) = (target[front] != front, target[back] != back);
        if !front_moved && !back_moved {
            curves.push(edge.curve.clone());
            continue;
        }
        let range = edge.curve.try_range_tuple();
        let ends_of = |curve: &C| range.map(|(t0, t1)| (curve.evaluate(t0), curve.evaluate(t1)));
        let before_ends = ends_of(&edge.curve);
        let (front_point, back_point) =
            (shell.vertices[target[front]], shell.vertices[target[back]]);
        let mut curve = edge.curve.clone();
        curve.snap_endpoints(front_point, back_point);
        let (Some((old_front, old_back)), Some((new_front, new_back))) =
            (before_ends, ends_of(&curve))
        else {
            return Err(SnapError::CurveEndpointOutOfTolerance { edge: edge_index });
        };
        let ends = [
            (
                front_moved,
                new_front.distance(front_point),
                old_front.distance(shell.vertices[front]),
            ),
            (
                back_moved,
                new_back.distance(back_point),
                old_back.distance(shell.vertices[back]),
            ),
        ];
        for (moved, gap, gap_before) in ends {
            if !moved {
                continue;
            }
            if gap.is_nan() || (gap > tolerance && gap > gap_before) {
                return Err(SnapError::CurveEndpointOutOfTolerance { edge: edge_index });
            }
            max_curve_endpoint_gap = max_curve_endpoint_gap.max(gap);
        }
        curves.push(curve);
    }

    // Compaction and the old-to-new mappings.
    let mut vertex_map = vec![0usize; target.len()];
    let mut next = 0;
    (0..target.len()).for_each(|vertex| {
        if target[vertex] == vertex {
            vertex_map[vertex] = next;
            next += 1;
        }
    });
    (0..target.len()).for_each(|vertex| vertex_map[vertex] = vertex_map[target[vertex]]);
    let mut surviving_index = vec![0usize; shell.edges.len()];
    let mut next = 0;
    (0..shell.edges.len()).for_each(|edge| {
        if dropped[edge].is_none() {
            surviving_index[edge] = next;
            next += 1;
        }
    });
    let edge_map: Vec<CompressedEdgeIndex> = (0..shell.edges.len())
        .map(|edge| {
            let (surviving, orientation) = carrier(edge);
            CompressedEdgeIndex {
                index: surviving_index[surviving],
                orientation,
            }
        })
        .collect();

    let vertices: Vec<Point3> = (0..target.len())
        .filter(|&vertex| target[vertex] == vertex)
        .map(|vertex| shell.vertices[vertex])
        .collect();
    let edges: Vec<CompressedEdge<C>> = (0..shell.edges.len())
        .filter(|&edge| dropped[edge].is_none())
        .zip(curves)
        .map(|(edge, curve)| {
            let (front, back) = shell.edges[edge].vertices;
            CompressedEdge {
                vertices: (vertex_map[front], vertex_map[back]),
                curve,
            }
        })
        .collect();
    let boundaries: Vec<Vec<Vec<CompressedEdgeIndex>>> = shell
        .faces
        .iter()
        .map(|face| {
            face.boundaries
                .iter()
                .map(|wire| {
                    wire.iter()
                        .map(|edge_use| {
                            let mapped = edge_map[edge_use.index];
                            CompressedEdgeIndex {
                                index: mapped.index,
                                orientation: edge_use.orientation == mapped.orientation,
                            }
                        })
                        .collect()
                })
                .collect()
        })
        .collect();
    let vertex_stable_ids = shell.vertex_stable_ids.as_ref().map(|ids| {
        (0..target.len())
            .filter(|&vertex| target[vertex] == vertex)
            .map(|vertex| ids[vertex])
            .collect()
    });
    let edge_stable_ids = shell.edge_stable_ids.as_ref().map(|ids| {
        (0..shell.edges.len())
            .filter(|&edge| dropped[edge].is_none())
            .map(|edge| ids[edge])
            .collect()
    });

    let max_vertex_displacement = (0..target.len())
        .filter(|&vertex| target[vertex] != vertex)
        .map(|vertex| shell.vertices[vertex].distance(shell.vertices[target[vertex]]))
        .fold(0.0, f64::max);
    let max_curve_deviation = accepted
        .iter()
        .map(|pair| pair.deviation)
        .fold(0.0, f64::max);
    let report = SnapReport {
        merged_vertices: target.len() - vertices.len(),
        merged_edges: accepted.len(),
        naked_edges_before: 0,
        naked_edges_after: 0,
        naked_edges: Vec::new(),
        skipped: Vec::new(),
        max_vertex_displacement,
        max_curve_deviation,
        max_curve_endpoint_gap,
        vertex_map,
        edge_map,
    };
    Ok(Welded {
        vertices,
        edges,
        boundaries,
        vertex_stable_ids,
        edge_stable_ids,
        report,
    })
}

/// Refuses a weld that leaves a merged vertex with more fans than any of the
/// vertices merged into it had before. A fan is a group of faces around the
/// vertex joined through edges incident to it that two of them use.
fn check_fans<C, S>(
    shell: &CompressedShell<Point3, C, S>,
    target: &[usize],
    carrier: &dyn Fn(usize) -> (usize, bool),
    remapped: &dyn Fn(usize) -> (usize, usize),
) -> std::result::Result<(), SnapError> {
    // Merged vertex -> its members, original indices.
    let mut members = BTreeMap::<usize, Vec<usize>>::new();
    (0..target.len())
        .filter(|&vertex| target[vertex] != vertex)
        .for_each(|vertex| {
            members
                .entry(target[vertex])
                .or_insert_with(|| vec![target[vertex]])
                .push(vertex)
        });
    if members.is_empty() {
        return Ok(());
    }
    let is_member = |vertex: usize| members.contains_key(&target[vertex]);

    // `(face, edge)` for every use of an edge at a vertex of interest, before
    // (original edges and vertices) and after (surviving edges, merged vertex).
    let mut before = BTreeMap::<usize, Vec<(usize, usize)>>::new();
    let mut after = BTreeMap::<usize, Vec<(usize, usize)>>::new();
    shell
        .faces
        .iter()
        .enumerate()
        .for_each(|(face_index, face)| {
            face.boundaries.iter().flatten().for_each(|edge_use| {
                let (front, back) = shell.edges[edge_use.index].vertices;
                [front, back]
                    .into_iter()
                    .filter(|&v| is_member(v))
                    .for_each(|vertex| {
                        before
                            .entry(vertex)
                            .or_default()
                            .push((face_index, edge_use.index));
                    });
                let surviving = carrier(edge_use.index).0;
                let (front, back) = remapped(surviving);
                [front, back]
                    .into_iter()
                    .filter(|v| members.contains_key(v))
                    .for_each(|vertex| {
                        after
                            .entry(vertex)
                            .or_default()
                            .push((face_index, surviving));
                    });
            });
        });
    for (&vertex, group) in &members {
        let worst_before = group
            .iter()
            .map(|member| before.get(member).map_or(0, |list| fan_components(list)))
            .max()
            .unwrap_or(0)
            .max(1);
        let now = after.get(&vertex).map_or(0, |list| fan_components(list));
        if now > worst_before {
            return Err(SnapError::SingularVertex { vertex });
        }
    }
    Ok(())
}

/// The number of groups the faces in `incident` form when two faces are joined
/// by an edge both use. `incident` lists `(face, edge)` for every use of an
/// edge at one vertex.
fn fan_components(incident: &[(usize, usize)]) -> usize {
    let mut faces: Vec<usize> = incident.iter().map(|&(face, _)| face).collect();
    faces.sort_unstable();
    faces.dedup();
    let mut parent: Vec<usize> = (0..faces.len()).collect();
    fn root(parent: &mut [usize], mut node: usize) -> usize {
        while parent[node] != node {
            parent[node] = parent[parent[node]];
            node = parent[node];
        }
        node
    }
    let slot = |face: usize| faces.binary_search(&face).unwrap_or_default();
    let mut by_edge = BTreeMap::<usize, Vec<usize>>::new();
    incident
        .iter()
        .for_each(|&(face, edge)| by_edge.entry(edge).or_default().push(slot(face)));
    by_edge.values().for_each(|slots| {
        slots.windows(2).for_each(|pair| {
            let (a, b) = (root(&mut parent, pair[0]), root(&mut parent, pair[1]));
            parent[a.max(b)] = a.min(b);
        });
    });
    (0..faces.len())
        .filter(|&slot| root(&mut parent, slot) == slot)
        .count()
}

impl<C> Welded<C> {
    fn commit<S>(
        self,
        shell: &mut CompressedShell<Point3, C, S>,
        naked_before: usize,
        skipped: Vec<(usize, usize, SkipReason)>,
    ) -> SnapReport {
        shell.vertices = self.vertices;
        shell.edges = self.edges;
        shell
            .faces
            .iter_mut()
            .zip(self.boundaries)
            .for_each(|(face, boundaries)| face.boundaries = boundaries);
        shell.vertex_stable_ids = self.vertex_stable_ids;
        shell.edge_stable_ids = self.edge_stable_ids;
        let naked_edges = naked_edge_indices(shell);
        SnapReport {
            naked_edges_before: naked_before,
            naked_edges_after: naked_edges.len(),
            naked_edges,
            skipped,
            ..self.report
        }
    }
}
