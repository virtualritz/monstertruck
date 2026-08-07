use super::*;
use mesh::spade_round;
use std::{f64::consts::TAU, iter, mem};

const PERIODIC_LOOP_OTHER_AXIS_TOLERANCE: f64 = 1.0e-6;
const SINGULAR_RADIUS_RELATIVE_TOLERANCE: f64 = 1.0e-6;
const DEFAULT_SINGULAR_PARAMETER_PROBE: f64 = 1.0;
const ISOPARAM_BOUNDARY_SEARCH_ITERATIONS: usize = 32;

// ---------------------------------------------------------------------------
// Boundary-projection failure lens (`MT_DEBUG_BOUNDARY_PROJECTION=1`)
//
// MEASUREMENT ONLY. Everything below is print-only and reached exclusively
// through `projection_debug_enabled()`, which reads the env var; with the var
// unset nothing here runs and no value it computes is fed back into the
// projection. It answers one question, the one spec 012 U1c left open: when
// `shell_create_polygon` drops a whole face because `PolyBoundaryPiece::try_new`
// returned `None`, WHICH boundary vertex refused to project and why?
//
// Per failing vertex it reports: the face index (set by the caller through a
// thread-local, since `try_new` is not given one), which restart attempt was
// running, the vertex's position in the loop, its 3D point and magnitude, the
// previous vertex's uv, whether the solver returned nothing at all or returned
// a uv that `normalize_uv` then rejected, the surface's declared range and
// periods, the residual `|surface(prev_uv) - point|`, how far the previous uv
// sits from each range endpoint (the seam / pole signature), the brute-force
// nearest point on the DECLARED domain and on a domain widened by one span
// either way (which tells an off-domain footpoint apart from a genuinely
// off-surface vertex), and what hinted vs. mid-domain-seeded
// `search_nearest_parameter` would have returned.
//
// `BPROJ_OUTCOME` prints one line per `try_new`, so the restart ladder
// (`head` -> `edge-seed` -> `brute-seed`) is visible beside the failures.
// ---------------------------------------------------------------------------

fn projection_debug_enabled() -> bool { env::var_os("MT_DEBUG_BOUNDARY_PROJECTION").is_some() }

thread_local! {
    /// Face index of the face currently being projected, set by
    /// `shell_create_polygon` (and its compressed twins) before calling
    /// `PolyBoundaryPiece::try_new`. `usize::MAX` means "not recorded".
    static PROJECTION_DEBUG_FACE: std::cell::Cell<usize> = const { std::cell::Cell::new(usize::MAX) };
}

/// Records the face index the projection lens should attribute failures to.
/// No-op unless the lens is enabled, so the non-debug path is untouched.
pub(super) fn projection_debug_set_face(face_idx: Option<usize>) {
    if projection_debug_enabled() {
        PROJECTION_DEBUG_FACE.with(|cell| cell.set(face_idx.unwrap_or(usize::MAX)));
    }
}

fn projection_debug_face() -> String {
    let index = PROJECTION_DEBUG_FACE.with(std::cell::Cell::get);
    if index == usize::MAX {
        "?".to_string()
    } else {
        index.to_string()
    }
}

fn fmt_range(range: Option<(f64, f64)>) -> String {
    range.map_or_else(
        || "none".to_string(),
        |(lo, hi)| format!("[{lo:.9},{hi:.9}]"),
    )
}

fn fmt_option_f64(value: Option<f64>) -> String {
    value.map_or_else(|| "none".to_string(), |v| format!("{v:.9}"))
}

/// Brute-force nearest point on the surface's DECLARED domain, by a coarse grid
/// plus a shrinking local sweep. Uses only `subs`, so it needs no solver trait
/// and answers the one question the checked solver's `None` hides: is the point
/// on the surface at all, and if so where?
///
/// Measurement only -- called exclusively from the lens.
fn sampled_nearest<S: PreMeshableSurface>(
    surface: &S,
    point: Point3,
    urange: (f64, f64),
    vrange: (f64, f64),
) -> ((f64, f64), f64) {
    const GRID: usize = 96;
    let (u0, u1) = urange;
    let (v0, v1) = vrange;
    let mut best = ((u0, v0), f64::INFINITY);
    for i in 0..=GRID {
        let u = u0 + (u1 - u0) * i as f64 / GRID as f64;
        for j in 0..=GRID {
            let v = v0 + (v1 - v0) * j as f64 / GRID as f64;
            let d = surface.subs(u, v).distance2(point);
            if d < best.1 {
                best = ((u, v), d);
            }
        }
    }
    let mut du = (u1 - u0) / GRID as f64;
    let mut dv = (v1 - v0) / GRID as f64;
    for _ in 0..60 {
        let ((bu, bv), _) = best;
        for i in -2i32..=2 {
            for j in -2i32..=2 {
                let u = (bu + du * i as f64).clamp(u0, u1);
                let v = (bv + dv * j as f64).clamp(v0, v1);
                let d = surface.subs(u, v).distance2(point);
                if d < best.1 {
                    best = ((u, v), d);
                }
            }
        }
        du *= 0.5;
        dv *= 0.5;
    }
    (best.0, best.1.sqrt())
}

/// One line per unprojectable boundary vertex. Print-only.
#[allow(clippy::too_many_arguments)]
fn report_projection_failure<S: PreMeshableSurface>(
    surface: &S,
    attempt: &str,
    start: usize,
    offset: usize,
    total: usize,
    point: Point3,
    previous: Option<(f64, f64)>,
    raw: Option<(f64, f64)>,
) {
    let (urange, vrange) = surface.try_range_tuple();
    let vertex = if total == 0 {
        0
    } else {
        (start + offset) % total
    };
    let stage = match raw {
        None => "solver-none",
        Some(_) => "normalize-rejected",
    };
    let raw_text = raw.map_or_else(|| "none".to_string(), |(u, v)| format!("({u:.9},{v:.9})"));
    let previous_text =
        previous.map_or_else(|| "none".to_string(), |(u, v)| format!("({u:.9},{v:.9})"));
    // Residual of the last vertex that DID project, plus how singular the
    // surface is there -- a pole shows a vanishing derivative.
    let (residual, du, dv) = previous.map_or((f64::NAN, f64::NAN, f64::NAN), |(u, v)| {
        (
            surface.subs(u, v).distance(point),
            surface.derivative_u(u, v).magnitude(),
            surface.derivative_v(u, v).magnitude(),
        )
    });
    // Seam signature: distance from the previous uv to each declared endpoint.
    let seam = previous.map_or_else(
        || "prev=none".to_string(),
        |(u, v)| {
            let du0 = urange.map(|(lo, _)| (u - lo).abs());
            let du1 = urange.map(|(_, hi)| (u - hi).abs());
            let dv0 = vrange.map(|(lo, _)| (v - lo).abs());
            let dv1 = vrange.map(|(_, hi)| (v - hi).abs());
            format!(
                "d_u_lo={} d_u_hi={} d_v_lo={} d_v_hi={}",
                fmt_option_f64(du0),
                fmt_option_f64(du1),
                fmt_option_f64(dv0),
                fmt_option_f64(dv1),
            )
        },
    );
    // Is the point on the surface at all? Brute force over the declared domain,
    // and again over a domain widened by one full span on each side, so an
    // off-domain (extrapolating) footpoint is told apart from a genuinely
    // off-surface vertex.
    let nearest = match (urange, vrange) {
        (Some(ur), Some(vr)) => {
            let ((u, v), d) = sampled_nearest(surface, point, ur, vr);
            let at_u_edge = (u - ur.0).abs() < 1.0e-9 || (u - ur.1).abs() < 1.0e-9;
            let at_v_edge = (v - vr.0).abs() < 1.0e-9 || (v - vr.1).abs() < 1.0e-9;
            let wide_u = (ur.0 - (ur.1 - ur.0), ur.1 + (ur.1 - ur.0));
            let wide_v = (vr.0 - (vr.1 - vr.0), vr.1 + (vr.1 - vr.0));
            let ((wu, wv), wd) = sampled_nearest(surface, point, wide_u, wide_v);
            format!(
                "near_uv=({u:.9},{v:.9}) near_dist={d:.9} near_at_u_edge={at_u_edge} \
                 near_at_v_edge={at_v_edge} wide_uv=({wu:.9},{wv:.9}) wide_dist={wd:.9}"
            )
        }
        _ => "near=unbounded-domain".to_string(),
    };
    // WOULD THE MISSING RUNG HAVE ANSWERED? `search_nearest_parameter`'s free
    // function needs only `ParametricSurface`, so it IS callable here -- the
    // reason it is absent from this path is the `SearchNearestParameter` TRAIT
    // bound on `search_nearest_parameter_sp`, not the algorithm.
    let snp = |hint: Option<(f64, f64)>| -> String {
        let seed = hint.unwrap_or_else(|| {
            let u = urange.map_or(0.0, |(lo, hi)| 0.5 * (lo + hi));
            let v = vrange.map_or(0.0, |(lo, hi)| 0.5 * (lo + hi));
            (u, v)
        });
        match algo::surface::search_nearest_parameter(surface, point, seed, 100) {
            Some((u, v)) => {
                let d = surface.subs(u, v).distance(point);
                format!("uv=({u:.9},{v:.9}) dist={d:.9}")
            }
            None => "diverged".to_string(),
        }
    };
    let snp_text = format!("snp_hint[{}] snp_mid[{}]", snp(previous), snp(None));
    let point_norm = point.to_vec().magnitude();
    eprintln!(
        "BPROJ_FAIL face={} attempt={attempt} start={start} vertex={vertex}/{total} \
         stage={stage} point=({:.9},{:.9},{:.9}) prev_uv={previous_text} raw_uv={raw_text} \
         urange={} vrange={} period_u={} period_v={} residual={residual:.9} \
         |du|={du:.9} |dv|={dv:.9} |point|={point_norm:.6} {seam} {nearest} {snp_text} \
         surface={}",
        projection_debug_face(),
        point.x,
        point.y,
        point.z,
        fmt_range(urange),
        fmt_range(vrange),
        fmt_option_f64(surface.period_u()),
        fmt_option_f64(surface.period_v()),
        std::any::type_name::<S>(),
    );
}

/// One line per `try_new` outcome, so the restart ladder is visible.
fn report_projection_outcome(attempt: &str, ok: bool) {
    if projection_debug_enabled() {
        eprintln!(
            "BPROJ_OUTCOME face={} attempt={attempt} ok={ok}",
            projection_debug_face(),
        );
    }
}

#[derive(Clone, Copy, Debug, derive_more::Deref, derive_more::DerefMut)]
pub(super) struct SurfacePoint {
    pub(super) point: Point3,
    #[deref]
    #[deref_mut]
    pub(super) uv: Point2,
}

impl From<(Point2, Point3)> for SurfacePoint {
    fn from((uv, point): (Point2, Point3)) -> Self { Self { point, uv } }
}

/// Trials for the verified nearest-parameter rung, matching the 100 the
/// robust path's `search_nearest_parameter_sp` uses by default.
const VERIFIED_FOOTPOINT_TRIALS: usize = 100;

/// The missing solver rung on the `Shell::triangulation` path, with a residual
/// check that ties acceptance to the caller's own tessellation tolerance.
///
/// WHY THIS EXISTS -- measured, spec 012 (`MT_DEBUG_BOUNDARY_PROJECTION=1`).
/// `search_parameter_sp` is CHECKED-ONLY: `search_parameter(hint)` then
/// `search_parameter(None)`. `search_parameter`'s Newton minimises the
/// TANGENTIAL residual, so it lands on the footpoint, and then accepts it only
/// if `surface.evaluate(u, v).near(&point)` -- an ABSOLUTE test at
/// `TOLERANCE = 1e-6`. Boundary polylines are sampled from the EDGE CURVE,
/// which in interchange data does not ride exactly on the face's surface.
///
/// Every boundary vertex that drops a face on the in-repo fixtures fails by
/// that margin and by nothing else. Censused at 1e-1/1e-2/1e-3, hinted
/// nearest-parameter from the previous vertex converges for ALL of them, with
/// residual `1.03e-6 .. 2.51e-6` -- between 1.03x and 2.51x the fixed 1e-6, and
/// three orders of magnitude INSIDE the finest chord ever requested (1e-3):
///
/// * coffy #219 faces 11/21/26/33 (`rational-bspline`, non-periodic, model
///   extent ~3.8e3): residual 1.03e-6 .. 2.51e-6.
/// * ap224 #1727 faces 28-39 and 44-47 (periodic-u revolutions, model extent
///   ~1.2): residual 2.38e-6 and 1.13e-6.
///
/// So the two "families" the drop census suggested are ONE cause. The
/// hint is load-bearing and the unhinted seed is not a substitute: seeded at
/// mid-domain the same solve returns residuals of 0.0625, 1.12 and 64.0, or
/// diverges. This rung is therefore hinted-only.
///
/// WHY IT CANNOT PRODUCE A SILENT PARTIAL. The result is accepted only when
/// `|surface(u, v) - point| <= tolerance`, the very chord the caller asked the
/// tessellator to honour. A boundary vertex admitted here is inside the
/// accuracy the output already advertises; anything worse is refused and the
/// face still drops, loudly, through `observe_face_drop`.
///
/// The free function is used deliberately: it needs only `ParametricSurface`,
/// so the rung is reachable under `PreMeshableSurface` with no bound change and
/// no API break. The `SearchNearestParameter` TRAIT bound -- not the algorithm
/// -- is what kept this path checked-only.
fn verified_footpoint<S: PreMeshableSurface>(
    surface: &S,
    point: Point3,
    hint: (f64, f64),
    tolerance: f64,
) -> Option<(f64, f64)> {
    let (u, v) =
        algo::surface::search_nearest_parameter(surface, point, hint, VERIFIED_FOOTPOINT_TRIALS)?;
    if !u.is_finite() || !v.is_finite() {
        return None;
    }
    (surface.subs(u, v).distance(point) <= tolerance).then_some((u, v))
}

fn orient_boundary_to_edge<C, S>(
    surface: &S,
    boundary: &mut [Point2],
    edge_curve: &C,
    orientation: bool,
) where
    C: BoundedCurve<Point = Point3> + ParametricCurve3D,
    S: ParametricSurface3D,
{
    let (edge_front, edge_back) = match orientation {
        true => (edge_curve.front(), edge_curve.back()),
        false => (edge_curve.back(), edge_curve.front()),
    };
    if edge_front.near(&edge_back) {
        return;
    }
    if let (Some(front_uv), Some(back_uv)) = (boundary.first(), boundary.last()) {
        let front_point = surface.evaluate(front_uv.x, front_uv.y);
        let back_point = surface.evaluate(back_uv.x, back_uv.y);
        let direct = front_point.distance2(edge_front) + back_point.distance2(edge_back);
        let reversed = front_point.distance2(edge_back) + back_point.distance2(edge_front);
        if reversed < direct {
            boundary.reverse();
        }
    }
}

#[derive(Debug, Default, Clone)]
pub(super) struct PolyBoundaryPiece(pub(super) Vec<SurfacePoint>);

impl PolyBoundaryPiece {
    fn rounded_uv_key(point: SurfacePoint) -> (u64, u64) {
        (
            spade_round(point.x).to_bits(),
            spade_round(point.y).to_bits(),
        )
    }

    pub(super) fn is_cdt_compatible(&self) -> bool {
        let len = self.0.len();
        let mut seen = HashMap::<(u64, u64), usize>::default();
        self.0.iter().copied().enumerate().all(|(index, point)| {
            let key = Self::rounded_uv_key(point);
            if let Some(previous) = seen.insert(key, index) {
                index == previous + 1 || (previous == 0 && index + 1 == len)
            } else {
                true
            }
        })
    }

    fn clamp_near_range(value: f64, (min, max): (f64, f64)) -> f64 {
        if value < min && min - value < TOLERANCE {
            min
        } else if value > max && value - max < TOLERANCE {
            max
        } else {
            value
        }
    }

    fn from_surface_points(mut vec: Vec<SurfacePoint>) -> Option<Self> {
        vec = vec
            .into_iter()
            .fold(Vec::<SurfacePoint>::new(), |mut acc, point| {
                if acc.last().is_none_or(|last| !last.uv.near(&point.uv)) {
                    acc.push(point);
                }
                acc
            });
        if vec.is_empty() {
            None
        } else {
            if vec
                .first()
                .is_some_and(|first| vec.last().is_some_and(|last| !first.uv.near(&last.uv)))
            {
                vec.push(vec[0]);
            }
            Some(Self(vec))
        }
    }

    fn from_parameter_boundary<S: PreMeshableSurface>(
        surface: &S,
        boundary: Vec<Point2>,
    ) -> Option<Self> {
        let mut previous = None;
        let vec = boundary
            .into_iter()
            .map(|uv| Self::normalize_uv(surface, (uv.x, uv.y), previous))
            .collect::<Option<Vec<_>>>()?
            .into_iter()
            .map(|(u, v)| {
                previous = Some((u, v));
                SurfacePoint::from((Point2::new(u, v), surface.evaluate(u, v)))
            })
            .collect();
        Self::from_surface_points(vec)
    }

    pub(super) fn try_new_from_trimmed<'a, S, T, C>(
        surface: &S,
        wire: impl Iterator<Item = (bool, Option<&'a T>, &'a C)>,
        tolerance: f64,
    ) -> Option<Self>
    where
        S: PreMeshableSurface,
        T: ExactTrimBoundary2D + 'a,
        C: ParametricCurve3D
            + BoundedCurve<Point = Point3>
            + ParameterBoundary2D<S>
            + ExactParameterBoundary2D<S>
            + 'a,
        <C as ExactParameterBoundary2D<S>>::BoundaryCurve: ExactTrimBoundary2D,
    {
        wire.map(|(orientation, trim_curve, edge_curve)| {
            let boundary = if let Some(trim_curve) = trim_curve {
                let mut boundary = trim_curve.exact_trim_boundary_2d(tolerance);
                orient_boundary_to_edge(surface, &mut boundary, edge_curve, orientation);
                boundary
            } else {
                let mut boundary = edge_curve
                    .exact_parameter_boundary_2d(surface)
                    .map(|trim_curve| trim_curve.exact_trim_boundary_2d(tolerance))
                    .or_else(|| edge_curve.parameter_boundary_2d(surface, tolerance))?;
                if !orientation {
                    boundary.reverse();
                }
                boundary
            };
            let boundary = simplify_parameter_boundary(surface, boundary, tolerance);
            Some(boundary)
        })
        .collect::<Option<Vec<Vec<Point2>>>>()
        .and_then(|boundaries| {
            let concatenated =
                boundaries
                    .into_iter()
                    .fold(Vec::<Point2>::new(), |mut acc, mut boundary| {
                        if !acc.is_empty() && !boundary.is_empty() {
                            boundary.remove(0);
                        }
                        acc.extend(boundary);
                        acc
                    });
            Self::from_parameter_boundary(surface, concatenated)
        })
    }

    pub(super) fn try_new_from_aligned_trimmed<'a, S, T, C>(
        surface: &S,
        wire: impl Iterator<Item = (Option<&'a T>, &'a C, PolylineCurve)>,
        sp: &impl SP<S>,
        tolerance: f64,
    ) -> Option<Self>
    where
        S: PreMeshableSurface,
        T: ExactTrimBoundary2D + 'a,
        C: ParameterBoundary2D<S> + ExactParameterBoundary2D<S> + 'a,
        <C as ExactParameterBoundary2D<S>>::BoundaryCurve: ExactTrimBoundary2D,
    {
        wire.map(|(trim_curve, edge_curve, polyline)| {
            trim_curve
                .and_then(|trim_curve| {
                    let mut previous_t = None;
                    let mut previous_uv = None;
                    polyline
                        .iter()
                        .copied()
                        .map(|point| {
                            let (t, uv) = trim_curve.project_boundary_point(point, previous_t)?;
                            let (u, v) = Self::normalize_uv(surface, (uv.x, uv.y), previous_uv)?;
                            previous_t = Some(t);
                            previous_uv = Some((u, v));
                            Some(SurfacePoint::from((Point2::new(u, v), point)))
                        })
                        .collect::<Option<Vec<_>>>()
                })
                .or_else(|| {
                    let mut previous = None;
                    polyline
                        .iter()
                        .copied()
                        .map(|point| {
                            let (u, v) = sp(surface, point, previous)
                                .and_then(|uv| Self::normalize_uv(surface, uv, previous))?;
                            previous = Some((u, v));
                            Some(SurfacePoint::from((Point2::new(u, v), point)))
                        })
                        .collect::<Option<Vec<_>>>()
                })
                .or_else(|| {
                    edge_curve
                        .parameter_boundary_2d(surface, tolerance)
                        .map(|boundary| simplify_parameter_boundary(surface, boundary, tolerance))
                        .and_then(|boundary| Self::from_parameter_boundary(surface, boundary))
                        .map(|piece| piece.0)
                })
        })
        .collect::<Option<Vec<Vec<SurfacePoint>>>>()
        .and_then(|pieces| {
            let concatenated =
                pieces
                    .into_iter()
                    .fold(Vec::<SurfacePoint>::new(), |mut acc, mut piece| {
                        if !acc.is_empty() && !piece.is_empty() {
                            piece.remove(0);
                        }
                        acc.extend(piece);
                        acc
                    });
            Self::from_surface_points(concatenated)
        })
    }

    pub(super) fn try_new_from_exact<'a, S: PreMeshableSurface, C: ParameterBoundary2D<S> + 'a>(
        surface: &S,
        wire: impl Iterator<Item = (bool, &'a C)>,
        tolerance: f64,
    ) -> Option<Self> {
        wire.map(|(orientation, curve)| {
            let mut boundary = curve.parameter_boundary_2d(surface, tolerance)?;
            if !orientation {
                boundary.reverse();
            }
            Some(boundary)
        })
        .collect::<Option<Vec<Vec<Point2>>>>()
        .and_then(|boundaries| {
            let concatenated =
                boundaries
                    .into_iter()
                    .fold(Vec::<Point2>::new(), |mut acc, mut boundary| {
                        if !acc.is_empty() && !boundary.is_empty() {
                            boundary.remove(0);
                        }
                        acc.extend(boundary);
                        acc
                    });
            Self::from_parameter_boundary(surface, concatenated)
        })
    }

    fn aligned_boundary_piece<S: PreMeshableSurface>(
        surface: &S,
        boundary: &[Point2],
        polyline: &PolylineCurve,
        previous: Option<SurfacePoint>,
    ) -> Option<(Vec<SurfacePoint>, Option<SurfacePoint>)> {
        let mut next_previous = previous;
        let piece = resample_boundary(boundary, polyline.len())?
            .into_iter()
            .zip(polyline.iter().copied())
            .map(|(uv, point)| {
                let (u, v) = Self::normalize_uv(
                    surface,
                    (uv.x, uv.y),
                    next_previous.as_ref().map(|point| (point.x, point.y)),
                )?;
                let surface_point = SurfacePoint::from((Point2::new(u, v), point));
                next_previous = Some(surface_point);
                Some(surface_point)
            })
            .collect::<Option<Vec<_>>>()?;
        Some((piece, next_previous))
    }

    fn projected_polyline_piece<S: PreMeshableSurface>(
        surface: &S,
        polyline: &PolylineCurve,
        sp: &impl SP<S>,
        previous: Option<SurfacePoint>,
    ) -> Option<(Vec<SurfacePoint>, Option<SurfacePoint>)> {
        let mut next_previous = previous;
        let seed = next_previous.as_ref().and_then(|previous_point| {
            polyline
                .first()
                .copied()
                .filter(|point| point.near(&previous_point.point))
                .map(|point| {
                    SurfacePoint::from((Point2::new(previous_point.x, previous_point.y), point))
                })
        });
        let prefix = seed.into_iter().collect::<Vec<_>>();
        let suffix = polyline
            .iter()
            .copied()
            .skip(prefix.len())
            .map(|point| {
                let surface_point = if next_previous
                    .as_ref()
                    .is_some_and(|previous_point| point.near(&previous_point.point))
                {
                    next_previous.as_ref().map(|previous_point| {
                        SurfacePoint::from((Point2::new(previous_point.x, previous_point.y), point))
                    })?
                } else {
                    let (u, v) = sp(
                        surface,
                        point,
                        next_previous.as_ref().map(|point| (point.x, point.y)),
                    )
                    .and_then(|uv| {
                        Self::normalize_uv(
                            surface,
                            uv,
                            next_previous.as_ref().map(|point| (point.x, point.y)),
                        )
                    })?;
                    SurfacePoint::from((Point2::new(u, v), point))
                };
                next_previous = Some(surface_point);
                Some(surface_point)
            })
            .collect::<Option<Vec<_>>>()?;
        Some((prefix.into_iter().chain(suffix).collect(), next_previous))
    }

    pub(super) fn try_new_from_aligned_exact<S: PreMeshableSurface>(
        surface: &S,
        wire: impl Iterator<Item = (Option<Vec<Point2>>, PolylineCurve)>,
        sp: &impl SP<S>,
    ) -> Option<Self> {
        let pieces = wire.collect::<Vec<_>>();
        if pieces.is_empty() {
            None
        } else {
            let piece_count = pieces.len();
            let start = pieces
                .iter()
                .position(|(boundary, _)| boundary.is_some())
                .unwrap_or(0);
            let mut previous = None::<SurfacePoint>;
            let periodic_surface = surface.period_u().is_some() || surface.period_v().is_some();
            pieces
                .into_iter()
                .cycle()
                .skip(start)
                .take(piece_count)
                .map(|(boundary, polyline)| {
                    let boundary_piece = || {
                        boundary.as_ref().and_then(|boundary| {
                            Self::aligned_boundary_piece(surface, boundary, &polyline, previous)
                        })
                    };
                    let projected_piece =
                        || Self::projected_polyline_piece(surface, &polyline, sp, previous);
                    let piece = if periodic_surface {
                        projected_piece().or_else(boundary_piece)
                    } else {
                        boundary_piece().or_else(projected_piece)
                    };
                    piece
                        .map(|(piece, next_previous)| {
                            previous = next_previous;
                            piece
                        })
                        .or_else(|| {
                            boundary.and_then(|boundary| {
                                Self::from_parameter_boundary(surface, boundary).map(|piece| {
                                    previous = piece.0.last().copied();
                                    piece.0
                                })
                            })
                        })
                })
                .collect::<Option<Vec<Vec<SurfacePoint>>>>()
                .and_then(|pieces| {
                    let concatenated = pieces.into_iter().fold(
                        Vec::<SurfacePoint>::new(),
                        |mut acc, mut piece| {
                            if !acc.is_empty() && !piece.is_empty() {
                                piece.remove(0);
                            }
                            acc.extend(piece);
                            acc
                        },
                    );
                    Self::from_surface_points(concatenated)
                })
        }
    }

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
                Some(periodic_min_difference(value, previous, period))
            } else if let Some(range) = range {
                Some(Self::clamp_near_range(value, range))
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
                Some(Self::clamp_near_range(value, (min, max)))
            }
        } else {
            Some(value)
        }
    }

    fn normalize_uv<S: PreMeshableSurface>(
        surface: &S,
        uv: (f64, f64),
        previous: Option<(f64, f64)>,
    ) -> Option<(f64, f64)> {
        let (urange, vrange) = surface.try_range_tuple();
        let u = Self::normalize_axis(uv.0, previous.map(|(u, _)| u), surface.period_u(), urange)?;
        let v = Self::normalize_axis(uv.1, previous.map(|(_, v)| v), surface.period_v(), vrange)?;
        Some((u, v))
    }

    fn parameter_seed_score<S: PreMeshableSurface>(surface: &S, (u, v): (f64, f64)) -> f64 {
        let uder = surface.derivative_u(u, v);
        let vder = surface.derivative_v(u, v);
        uder.dot(uder) + vder.dot(vder)
    }

    fn project_loop<S: PreMeshableSurface>(
        surface: &S,
        boundary: &[Point3],
        sp: &impl SP<S>,
        start: usize,
        initial_uv: Option<(f64, f64)>,
        tolerance: f64,
        attempt: &str,
    ) -> Option<Vec<SurfacePoint>> {
        let mut initial_uv = initial_uv;
        let total = boundary.len();
        boundary
            .iter()
            .copied()
            .cycle()
            .skip(start)
            .take(boundary.len() + 1)
            .enumerate()
            .scan(None, |previous, (offset, pt)| {
                let previous_before = *previous;
                let raw = initial_uv
                    .take()
                    .or_else(|| sp(surface, pt, *previous))
                    .or_else(|| {
                        // Revolve-pole fallback. At the revolution axis the surface
                        // is singular -- every v maps to the same 3D point -- so the
                        // parameter search (both exact and nearest) can fail to
                        // converge there and return `None`. The pole nonetheless
                        // coincides with a u-endpoint for EVERY v, so reuse the
                        // adjacent vertex's v and let the singular-bridge below
                        // carry the loop across the pole, instead of dropping the
                        // entire face's mesh (the corner-100 in-cube polar cap).
                        //
                        // Gate on v-invariance of the u-endpoint (the exact,
                        // scale-free signature of a revolution pole) so this fires
                        // only at a genuine axis point, never masking a real
                        // projection failure; the vertex only has to sit within a
                        // coarse mesh chord of that pole (a boundary polyline sample
                        // near the pole is a fraction off the exact axis point).
                        let (_, v_prev) = (*previous)?;
                        let (u0, u1) = surface.try_range_tuple().0?;
                        [u0, u1].into_iter().find_map(|u_end| {
                            let pole = surface.subs(u_end, v_prev);
                            let is_pole = pole.near(&surface.subs(u_end, v_prev + 1.0));
                            let at_pole = (pole - pt).magnitude2() < 1.0e-6;
                            (is_pole && at_pole).then_some((u_end, v_prev))
                        })
                    })
                    .or_else(|| {
                        let uv = verified_footpoint(surface, pt, (*previous)?, tolerance)?;
                        if projection_debug_enabled() {
                            eprintln!(
                                "BPROJ_RESCUE face={} attempt={attempt} vertex={}/{total} \
                                 uv=({:.9},{:.9}) residual={:.9} tolerance={tolerance:.9}",
                                projection_debug_face(),
                                if total == 0 {
                                    0
                                } else {
                                    (start + offset) % total
                                },
                                uv.0,
                                uv.1,
                                surface.subs(uv.0, uv.1).distance(pt),
                            );
                        }
                        Some(uv)
                    });
                let uv = raw
                    .and_then(|uv| Self::normalize_uv(surface, uv, *previous))
                    .map(|(u, v)| {
                        let points = if let Some((u0, v0)) = *previous {
                            if !u0.near(&u) && surface.derivative_u(u0, v0).so_small() {
                                vec![
                                    SurfacePoint::from((Point2::new(u, v0), pt)),
                                    SurfacePoint::from((Point2::new(u, v), pt)),
                                ]
                            } else if !v0.near(&v) && surface.derivative_v(u0, v0).so_small() {
                                vec![
                                    SurfacePoint::from((Point2::new(u0, v), pt)),
                                    SurfacePoint::from((Point2::new(u, v), pt)),
                                ]
                            } else {
                                vec![SurfacePoint::from((Point2::new(u, v), pt))]
                            }
                        } else {
                            vec![SurfacePoint::from((Point2::new(u, v), pt))]
                        };
                        *previous = Some((u, v));
                        points
                    });
                if uv.is_none() && projection_debug_enabled() {
                    report_projection_failure(
                        surface,
                        attempt,
                        start,
                        offset,
                        total,
                        pt,
                        previous_before,
                        raw,
                    );
                }
                Some(uv)
            })
            .collect::<Option<Vec<Vec<SurfacePoint>>>>()
            .map(|chunks| chunks.into_iter().flatten().collect())
    }

    pub(super) fn try_new<S: PreMeshableSurface>(
        surface: &S,
        wire: impl Iterator<Item = PolylineCurve>,
        sp: impl SP<S>,
        tolerance: f64,
    ) -> Option<Self> {
        let (urange, vrange) = surface.try_range_tuple();
        let (bdry3d, candidate_starts) = wire.fold(
            (Vec::<Point3>::new(), Vec::<usize>::new()),
            |(mut boundary, mut starts), poly_edge| {
                let edge_len = poly_edge.len().saturating_sub(1);
                if edge_len != 0 {
                    starts.push(boundary.len());
                    boundary.extend(poly_edge.into_iter().take(edge_len));
                }
                (boundary, starts)
            },
        );
        if bdry3d.is_empty() {
            return None;
        }
        let projected = Self::project_loop(surface, &bdry3d, &sp, 0, None, tolerance, "head")
            .or_else(|| {
                candidate_starts
                    .into_iter()
                    .filter(|start| *start != 0)
                    .filter_map(|start| {
                        let point = bdry3d[start];
                        let uv = sp(surface, point, None)
                            .and_then(|uv| Self::normalize_uv(surface, uv, None))?;
                        Some((Self::parameter_seed_score(surface, uv), start, uv))
                    })
                    .max_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0))
                    .and_then(|(_, start, uv)| {
                        Self::project_loop(
                            surface,
                            &bdry3d,
                            &sp,
                            start,
                            Some(uv),
                            tolerance,
                            "edge-seed",
                        )
                    })
                    .or_else(|| {
                        bdry3d
                            .iter()
                            .copied()
                            .enumerate()
                            .filter(|(start, _)| *start != 0)
                            .filter_map(|(start, point)| {
                                let uv = sp(surface, point, None)
                                    .and_then(|uv| Self::normalize_uv(surface, uv, None))?;
                                Some((Self::parameter_seed_score(surface, uv), start, uv))
                            })
                            .max_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0))
                            .and_then(|(_, start, uv)| {
                                Self::project_loop(
                                    surface,
                                    &bdry3d,
                                    &sp,
                                    start,
                                    Some(uv),
                                    tolerance,
                                    "brute-seed",
                                )
                            })
                    })
            });
        report_projection_outcome("ladder", projected.is_some());
        let mut vec = projected?;
        let grav = vec.iter().fold(Point2::origin(), |g, p| g + p.uv.to_vec()) / vec.len() as f64;
        if let (Some(up), Some((u0, _))) = (surface.period_u(), urange) {
            let quot = f64::floor((grav.x - u0) / up);
            vec.iter_mut().for_each(|p| p.x -= quot * up);
        }
        if let (Some(vp), Some((v0, _))) = (surface.period_v(), vrange) {
            let quot = f64::floor((grav.y - v0) / vp);
            vec.iter_mut().for_each(|p| p.y -= quot * vp);
        }
        // SAFETY: vec is non-empty because it was built from a non-empty boundary.
        let last = *vec.last().unwrap();
        if !vec[0].near(&last) {
            let Point2 { x: u0, y: v0 } = last.uv;
            if surface.derivative_u(u0, v0).so_small() || surface.derivative_v(u0, v0).so_small() {
                vec.push(vec[0]);
            }
        }
        Some(Self(vec))
    }
}

fn abs_diff(previous: f64) -> impl Fn(&f64, &f64) -> std::cmp::Ordering {
    let f = move |x: &f64| f64::abs(x - previous);
    // SAFETY: UV parameters from surface evaluation are finite, so comparison succeeds.
    move |x: &f64, y: &f64| f(x).partial_cmp(&f(y)).unwrap()
}
fn periodic_min_difference(u: f64, u0: f64, up: f64) -> f64 {
    let closure = |i| u + i as f64 * up;
    // SAFETY: the iterator (-2..=2) is non-empty, containing five elements.
    (-2..=2).map(closure).min_by(abs_diff(u0)).unwrap()
}

#[derive(Debug, Clone)]
pub(super) struct PolyBoundary {
    pub(super) loops: Vec<Vec<SurfacePoint>>,
    /// UV-space axis-aligned bounding box for cheap rejection in `include()`.
    pub(super) uv_min: Point2,
    pub(super) uv_max: Point2,
}

impl Default for PolyBoundary {
    fn default() -> Self {
        Self {
            loops: Vec::new(),
            uv_min: Point2::new(f64::INFINITY, f64::INFINITY),
            uv_max: Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }
}

fn normalize_range(curve: &mut Vec<SurfacePoint>, compidx: usize, (u0, u1): (f64, f64)) {
    let p = curve[0];
    let q = curve[curve.len() - 1];
    let tmp = f64::min(p[compidx], q[compidx]) + TOLERANCE;
    let del = f64::floor((tmp - u0) / (u1 - u0)) * (u1 - u0);
    curve.iter_mut().for_each(|p| p[compidx] -= del);
    let Some(i) = curve
        .iter()
        .position(|p| (curve[0][compidx] - u1) * (p[compidx] - u1) < 0.0)
    else {
        return;
    };
    let mut curve1 = curve.split_off(i + 1);
    curve1.pop();
    curve1.insert(0, curve[i]);
    match curve[0][compidx] < curve[curve.len() - 1][compidx] {
        true => curve1.iter_mut(),
        false => curve.iter_mut(),
    }
    .for_each(|p| p[compidx] -= u1 - u0);
    curve1.append(curve);
    *curve = curve1;
}

fn loop_signed_area(curve: &[SurfacePoint]) -> f64 {
    curve
        .iter()
        .circular_tuple_windows()
        .fold(0.0, |sum, (p, q)| sum + (q.x + p.x) * (q.y - p.y))
}

fn loop_orientation(curve: &[SurfacePoint]) -> bool { loop_signed_area(curve) > 0.0 }

fn normalize_closed_loop_winding(closed: &mut [Vec<SurfacePoint>]) {
    let outer_index = closed
        .iter()
        .enumerate()
        .max_by(|(_, lhs), (_, rhs)| {
            loop_signed_area(lhs)
                .abs()
                .total_cmp(&loop_signed_area(rhs).abs())
        })
        .map(|(index, _)| index);
    if let Some(outer_index) = outer_index {
        closed.iter_mut().enumerate().for_each(|(index, curve)| {
            let should_be_positive = index == outer_index;
            if loop_orientation(curve) != should_be_positive {
                curve.reverse();
            }
        });
    }
}

fn periodic_axis_full_span(
    curve: &[SurfacePoint],
    surface: &impl PreMeshableSurface,
) -> Option<(usize, f64)> {
    let closed = curve
        .first()
        .zip(curve.last())
        .is_some_and(|(front, back)| front.uv.near(&back.uv));
    if curve.len() < 4 || !closed {
        None
    } else {
        [(0usize, surface.period_u()), (1usize, surface.period_v())]
            .into_iter()
            .filter_map(|(axis, period)| Some((axis, period?)))
            .find(|(axis, period)| {
                let other_axis = 1 - *axis;
                match (
                    curve_axis_span(curve, *axis),
                    curve_axis_span(curve, other_axis),
                ) {
                    (Some((min, max)), Some((other_min, other_max))) => {
                        max - min + TOLERANCE >= period * 0.75
                            && other_max - other_min <= PERIODIC_LOOP_OTHER_AXIS_TOLERANCE
                    }
                    _ => false,
                }
            })
    }
}

fn curve_axis_span(curve: &[SurfacePoint], axis: usize) -> Option<(f64, f64)> {
    curve
        .iter()
        .map(|point| point[axis])
        .fold(None, |span, value| {
            Some(match span {
                Some((min, max)) => (f64::min(min, value), f64::max(max, value)),
                None => (value, value),
            })
        })
}

fn surface_axis_range(
    surface: &impl PreMeshableSurface,
    axis: usize,
) -> (Option<f64>, Option<(f64, f64)>) {
    let (urange, vrange) = surface.try_range_tuple();
    match axis {
        0 => (surface.period_u(), urange),
        _ => (surface.period_v(), vrange),
    }
}

fn normalized_periodic_candidate(
    curve: &[SurfacePoint],
    surface: &impl PreMeshableSurface,
    axis: usize,
    period: f64,
    range: Option<(f64, f64)>,
    start: usize,
    closed_len: usize,
) -> Option<(f64, Vec<SurfacePoint>)> {
    let mut previous = None;
    let candidate = curve
        .iter()
        .take(closed_len)
        .cycle()
        .skip(start)
        .take(closed_len)
        .copied()
        .map(|mut point| {
            point[axis] =
                PolyBoundaryPiece::normalize_axis(point[axis], previous, Some(period), range)?;
            previous = Some(point[axis]);
            point.point = surface.evaluate(point.x, point.y);
            Some(point)
        })
        .collect::<Option<Vec<_>>>()?;
    let span = curve_axis_span(&candidate, axis).map(|(min, max)| max - min)?;
    Some((span, candidate))
}

fn compact_periodic_closed_loop(curve: &mut Vec<SurfacePoint>, surface: &impl PreMeshableSurface) {
    let closed = curve
        .first()
        .zip(curve.last())
        .is_some_and(|(front, back)| front.uv.near(&back.uv));
    if curve.len() < 4 || !closed {
        return;
    }
    if periodic_axis_full_span(curve, surface).is_some() {
        return;
    }
    for axis in 0..=1 {
        let (Some(period), range) = surface_axis_range(surface, axis) else {
            continue;
        };
        let Some((min, max)) = curve_axis_span(curve, axis) else {
            continue;
        };
        let has_periodic_jump = curve
            .windows(2)
            .any(|points| (points[1][axis] - points[0][axis]).abs() > period * 0.5);
        if max - min <= period + TOLERANCE && !has_periodic_jump {
            continue;
        }
        let closed_len = curve.len() - 1;
        let best = (0..closed_len)
            .filter_map(|start| {
                normalized_periodic_candidate(
                    curve, surface, axis, period, range, start, closed_len,
                )
            })
            .min_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0));
        if let Some((span, mut candidate)) = best
            && span <= period + TOLERANCE
        {
            candidate.push(candidate[0]);
            *curve = candidate;
        }
    }
}

fn unwrap_periodic_open_curve(curve: &mut [SurfacePoint], axis: usize, period: f64) {
    let mut previous = None;
    curve.iter_mut().for_each(|point| {
        if let Some(value) =
            PolyBoundaryPiece::normalize_axis(point[axis], previous, Some(period), None)
        {
            point[axis] = value;
        }
        previous = Some(point[axis]);
    });
}

fn shift_curve_axis(curve: &mut [SurfacePoint], axis: usize, shift: f64) {
    curve.iter_mut().for_each(|point| {
        point[axis] += shift;
    });
}

fn paired_curve_axis_span(
    curve0: &[SurfacePoint],
    curve1: &[SurfacePoint],
    axis: usize,
    curve1_shift: f64,
) -> Option<f64> {
    curve0
        .iter()
        .map(|point| point[axis])
        .chain(curve1.iter().map(|point| point[axis] + curve1_shift))
        .fold(None, |span, value| {
            Some(match span {
                Some((min, max)) => (f64::min(min, value), f64::max(max, value)),
                None => (value, value),
            })
        })
        .map(|(min, max)| max - min)
}

fn align_periodic_open_pair(
    curve0: &[SurfacePoint],
    curve1: &mut [SurfacePoint],
    axis: usize,
    period: f64,
) {
    let best_shift = (-4..=4)
        .filter_map(|lap| {
            let shift = lap as f64 * period;
            paired_curve_axis_span(curve0, curve1, axis, shift).map(|span| (span, shift))
        })
        .min_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0))
        .map(|(_, shift)| shift);
    if let Some(shift) = best_shift {
        shift_curve_axis(curve1, axis, shift);
    }
}

fn periodic_seam_bounds(
    lower: f64,
    upper: f64,
    period: f64,
    range: Option<(f64, f64)>,
) -> (f64, f64) {
    let origin = range.map(|(min, _)| min).unwrap_or(0.0);
    let mut seam_lower = origin + ((lower - origin) / period).floor() * period;
    while upper > seam_lower + period + TOLERANCE {
        seam_lower += period;
    }
    (seam_lower, seam_lower + period)
}

fn seam_surface_point(
    surface: &impl PreMeshableSurface,
    mut template: SurfacePoint,
    axis: usize,
    value: f64,
) -> SurfacePoint {
    template[axis] = value;
    template.point = surface.evaluate(template.x, template.y);
    template
}

fn open_periodic_closed_loop(
    mut curve: Vec<SurfacePoint>,
    surface: &impl PreMeshableSurface,
    axis: usize,
    period: f64,
) -> Vec<SurfacePoint> {
    curve.pop();
    let jump = curve
        .windows(2)
        .enumerate()
        .map(|(index, points)| (index, points[1][axis] - points[0][axis]))
        .filter(|(_, delta)| delta.abs() > period * 0.5)
        .max_by(|lhs, rhs| lhs.1.abs().total_cmp(&rhs.1.abs()));
    if let Some((index, delta)) = jump {
        let lower = f64::min(curve[index][axis], curve[index + 1][axis]);
        let upper = f64::max(curve[index][axis], curve[index + 1][axis]);
        let (_, range) = surface_axis_range(surface, axis);
        let (seam_lower, seam_upper) = periodic_seam_bounds(lower, upper, period, range);
        let mut opened = Vec::with_capacity(curve.len() + 2);
        if delta > 0.0 {
            opened.push(seam_surface_point(
                surface,
                curve[index + 1],
                axis,
                seam_upper,
            ));
            opened.extend_from_slice(&curve[index + 1..]);
            opened.extend_from_slice(&curve[..=index]);
            opened.push(seam_surface_point(surface, curve[index], axis, seam_lower));
        } else {
            opened.push(seam_surface_point(
                surface,
                curve[index + 1],
                axis,
                seam_lower,
            ));
            opened.extend_from_slice(&curve[index + 1..]);
            opened.extend_from_slice(&curve[..=index]);
            opened.push(seam_surface_point(surface, curve[index], axis, seam_upper));
        }
        opened
    } else {
        unwrap_periodic_open_curve(&mut curve, axis, period);
        if let (Some(front), Some(back)) = (curve.first().copied(), curve.last().copied()) {
            let delta = back[axis] - front[axis];
            if delta.abs() + TOLERANCE < period {
                let mut seam = front;
                seam[axis] += if delta < 0.0 { -period } else { period };
                seam.point = surface.evaluate(seam.x, seam.y);
                curve.push(seam);
            }
        }
        curve
    }
}

fn periodic_open_axis_full_span(
    curve: &[SurfacePoint],
    surface: &impl PreMeshableSurface,
) -> Option<(usize, f64)> {
    if curve.len() < 2 {
        None
    } else {
        [(0usize, surface.period_u()), (1usize, surface.period_v())]
            .into_iter()
            .filter_map(|(axis, period)| Some((axis, period?)))
            .find(|(axis, period)| {
                let other_axis = 1 - *axis;
                match (
                    curve_axis_span(curve, *axis),
                    curve_axis_span(curve, other_axis),
                ) {
                    (Some((min, max)), Some((other_min, other_max))) => {
                        max - min + TOLERANCE >= period * 0.75 && other_max - other_min <= 1.0e-6
                    }
                    _ => false,
                }
            })
    }
}

fn full_period_curve_direction(curve: &[SurfacePoint], axis: usize) -> Option<f64> {
    curve
        .first()
        .zip(curve.last())
        .map(|(front, back)| if back[axis] < front[axis] { -1.0 } else { 1.0 })
}

fn periodic_axis_value(value: f64, target_start: f64, period: f64, direction: f64) -> f64 {
    if direction < 0.0 {
        target_start - (target_start - value).rem_euclid(period)
    } else {
        target_start + (value - target_start).rem_euclid(period)
    }
}

fn periodic_seam_point(
    surface: &impl PreMeshableSurface,
    template: SurfacePoint,
    axis: usize,
    axis_value: f64,
) -> SurfacePoint {
    let mut point = template;
    point[axis] = axis_value;
    point.point = surface.evaluate(point.x, point.y);
    point
}

fn align_full_period_curve_to_axis(
    curve: &mut Vec<SurfacePoint>,
    surface: &impl PreMeshableSurface,
    axis: usize,
    period: f64,
    target_start: f64,
    direction: f64,
) -> bool {
    if curve.len() < 2 {
        false
    } else {
        if full_period_curve_direction(curve, axis).is_some_and(|current| current != direction) {
            curve.reverse();
        }
        let Some(template) = curve.first().copied() else {
            return false;
        };
        let target_end = target_start + direction * period;
        let open_len = curve.len().saturating_sub(1);
        let mut aligned = curve
            .iter()
            .take(open_len)
            .copied()
            .map(|mut point| {
                point[axis] = periodic_axis_value(point[axis], target_start, period, direction);
                point
            })
            .collect::<Vec<_>>();
        aligned.sort_by(|lhs, rhs| {
            if direction < 0.0 {
                rhs[axis].total_cmp(&lhs[axis])
            } else {
                lhs[axis].total_cmp(&rhs[axis])
            }
        });
        let mut with_seams = Vec::with_capacity(aligned.len() + 2);
        with_seams.push(periodic_seam_point(surface, template, axis, target_start));
        aligned
            .into_iter()
            .filter(|point| {
                (point[axis] - target_start).abs() > TOLERANCE
                    && (point[axis] - target_end).abs() > TOLERANCE
            })
            .for_each(|point| with_seams.push(point));
        with_seams.push(periodic_seam_point(surface, template, axis, target_end));
        *curve = with_seams;
        true
    }
}

fn align_full_period_open_pair(
    curve0: &mut Vec<SurfacePoint>,
    curve1: &mut Vec<SurfacePoint>,
    surface: &impl PreMeshableSurface,
) -> bool {
    let Some((axis0, period0)) = periodic_open_axis_full_span(curve0, surface) else {
        return false;
    };
    let Some((axis1, period1)) = periodic_open_axis_full_span(curve1, surface) else {
        return false;
    };
    if axis0 != axis1 || (period0 - period1).abs() > TOLERANCE {
        false
    } else {
        let Some(direction) = full_period_curve_direction(curve0, axis0) else {
            return false;
        };
        let Some(start0) = curve0.first().map(|point| point[axis0]) else {
            return false;
        };
        let aligned0 =
            align_full_period_curve_to_axis(curve0, surface, axis0, period0, start0, direction);
        let Some(start1) = curve0.last().map(|point| point[axis0]) else {
            return false;
        };
        let aligned1 =
            align_full_period_curve_to_axis(curve1, surface, axis0, period0, start1, -direction);
        aligned0 && aligned1
    }
}

fn axis_derivative_norm(surface: &impl PreMeshableSurface, axis: usize, uv: Point2) -> Option<f64> {
    let norm = match axis {
        0 => surface.derivative_u(uv.x, uv.y).magnitude(),
        _ => surface.derivative_v(uv.x, uv.y).magnitude(),
    };
    norm.is_finite().then_some(norm)
}

fn uv_with_axis_values(
    template: SurfacePoint,
    axis: usize,
    axis_value: f64,
    other_value: f64,
) -> Point2 {
    let mut uv = template.uv;
    uv[axis] = axis_value;
    uv[1 - axis] = other_value;
    uv
}

fn singular_periodic_other_parameter(
    curve: &[SurfacePoint],
    surface: &impl PreMeshableSurface,
    periodic_axis: usize,
) -> Option<f64> {
    let other_axis = 1 - periodic_axis;
    let other = curve.iter().map(|point| point[other_axis]).sum::<f64>() / curve.len() as f64;
    let periodic = curve[0][periodic_axis];
    let uv = uv_with_axis_values(curve[0], periodic_axis, periodic, other);
    let radius = axis_derivative_norm(surface, periodic_axis, uv)?;
    if radius <= TOLERANCE {
        Some(other)
    } else {
        let (_, range) = surface_axis_range(surface, other_axis);
        let step = range
            .map(|(min, max)| (max - min).abs())
            .filter(|span| span.is_finite() && *span > TOLERANCE)
            .unwrap_or(DEFAULT_SINGULAR_PARAMETER_PROBE);
        [-step, step]
            .into_iter()
            .filter_map(|delta| {
                let sample_uv =
                    uv_with_axis_values(curve[0], periodic_axis, periodic, other + delta);
                let sample_radius = axis_derivative_norm(surface, periodic_axis, sample_uv)?;
                let slope = (sample_radius - radius) / delta;
                if slope.abs() <= TOLERANCE {
                    None
                } else {
                    let candidate = other - radius / slope;
                    let candidate_uv =
                        uv_with_axis_values(curve[0], periodic_axis, periodic, candidate);
                    let candidate_radius =
                        axis_derivative_norm(surface, periodic_axis, candidate_uv)?;
                    (candidate.is_finite()
                        && candidate_radius
                            <= TOLERANCE.max(radius * SINGULAR_RADIUS_RELATIVE_TOLERANCE))
                    .then_some((candidate_radius, (candidate - other).abs(), candidate))
                }
            })
            .min_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0).then(lhs.1.total_cmp(&rhs.1)))
            .map(|(_, _, candidate)| candidate)
    }
}

fn singular_surface_point(
    surface: &impl PreMeshableSurface,
    mut template: SurfacePoint,
    axis: usize,
    other_value: f64,
) -> SurfacePoint {
    let other_axis = 1 - axis;
    template[other_axis] = other_value;
    template.point = surface.evaluate(template.x, template.y);
    template
}

fn connect_edges<P>(vecs: impl IntoIterator<Item = Vec<P>>) -> Vec<P> {
    let closure = |vec: Vec<P>| {
        let len = vec.len();
        vec.into_iter().take(len - 1)
    };
    vecs.into_iter().flat_map(closure).collect()
}

fn close_open_periodic_curve_to_singular(
    curve: Vec<SurfacePoint>,
    surface: &impl PreMeshableSurface,
    axis: usize,
    tolerance: f64,
    point_cache: &mut HashMap<UvKey, Point3>,
) -> Option<Vec<SurfacePoint>> {
    let singular_other = singular_periodic_other_parameter(&curve, surface, axis)?;
    let (p, q) = (*curve.first()?, *curve.last()?);
    let singular_p = singular_surface_point(surface, p, axis, singular_other);
    let singular_q = singular_surface_point(surface, q, axis, singular_other);
    let vec0 = polyline_on_surface(surface, q, singular_q, tolerance, point_cache);
    let vec1 = polyline_on_surface(surface, singular_q, singular_p, tolerance, point_cache);
    let vec2 = polyline_on_surface(surface, singular_p, p, tolerance, point_cache);
    Some(connect_edges([curve, vec0, vec1, vec2]))
}

fn open_pair_connector_score(
    p0: SurfacePoint,
    p1: SurfacePoint,
    q0: SurfacePoint,
    q1: SurfacePoint,
) -> f64 {
    let p1q0 = p1.uv - q0.uv;
    let q1p0 = q1.uv - p0.uv;
    p1q0.x * p1q0.x + p1q0.y * p1q0.y + q1p0.x * q1p0.x + q1p0.y * q1p0.y
}

pub(super) type UvKey = (u64, u64);

pub(super) fn uv_key(uv: Point2) -> UvKey { (uv.x.to_bits(), uv.y.to_bits()) }

pub(super) fn boundary_segment_parameter(
    point: Point2,
    front: Point2,
    back: Point2,
) -> Option<f64> {
    let segment = back - front;
    let denom = segment.dot(segment);
    if denom.so_small() {
        None
    } else {
        let offset = point - front;
        let parameter = offset.dot(segment) / denom;
        let projected = front + segment * parameter;
        (parameter > 0.0 && parameter < 1.0 && projected.distance(point) <= 1.0e-9)
            .then_some(parameter)
    }
}

pub(super) fn surface_point_with_cache(
    surface: &impl PreMeshableSurface,
    uv: Point2,
    point_cache: &mut HashMap<UvKey, Point3>,
) -> SurfacePoint {
    let point = *point_cache
        .entry(uv_key(uv))
        .or_insert_with(|| surface.evaluate(uv.x, uv.y));
    (uv, point).into()
}

fn push_isoparam_uv(curve: &mut Vec<Point2>, uv: Point2) {
    if curve.last().is_none_or(|previous| !previous.near(&uv)) {
        curve.push(uv);
    }
}

fn push_finished_isoparam_curve(curves: &mut Vec<Vec<Point2>>, curve: &mut Vec<Point2>) {
    if curve.len() >= 2 {
        curves.push(mem::take(curve));
    } else {
        curve.clear();
    }
}

fn add_direct_cdt_constraint(
    triangulation: &mut Cdt,
    added_constraints: &mut usize,
    skipped_constraints: &mut usize,
    front: FixedVertexHandle,
    back: FixedVertexHandle,
) -> bool {
    if front == back || !triangulation.can_add_constraint(front, back) {
        *skipped_constraints += 1;
        false
    } else {
        let constraints = triangulation.add_constraint_and_split(front, back, |point| point);
        if constraints.is_empty() {
            *skipped_constraints += 1;
            false
        } else {
            *added_constraints += constraints.len();
            true
        }
    }
}

impl PolyBoundary {
    pub(super) fn new(
        pieces: Vec<PolyBoundaryPiece>,
        surface: &impl PreMeshableSurface,
        tolerance: f64,
    ) -> Self {
        let (mut closed, mut open) = (Vec::new(), Vec::new());
        pieces.into_iter().for_each(|PolyBoundaryPiece(mut vec)| {
            compact_periodic_closed_loop(&mut vec, surface);
            if let Some((axis, period)) = periodic_axis_full_span(&vec, surface) {
                open.push(open_periodic_closed_loop(vec, surface, axis, period));
            } else {
                match vec[0].uv.distance(vec[vec.len() - 1].uv) < 1.0e-3 {
                    true => {
                        vec.pop();
                        closed.push(vec)
                    }
                    false => open.push(vec),
                }
            }
        });
        open.retain(|curve| curve.len() >= 2);
        closed.retain(|curve| curve.len() >= 2);
        let mut point_cache = HashMap::<UvKey, Point3>::default();
        match open.len() {
            1 => {
                // SAFETY: open.len() == 1 was matched above.
                let mut curve = open.pop().unwrap();
                if let Some((axis, _)) = periodic_open_axis_full_span(&curve, surface)
                    && let Some(singular_curve) = close_open_periodic_curve_to_singular(
                        curve.clone(),
                        surface,
                        axis,
                        tolerance,
                        &mut point_cache,
                    )
                {
                    closed.push(singular_curve);
                } else if let (Some((u0, u1)), Some((v0, v1))) = surface.try_range_tuple() {
                    let p = curve[0];
                    let q = curve[curve.len() - 1];
                    if p.x < q.x - TOLERANCE {
                        normalize_range(&mut curve, 0, (u0, u1));
                        let p = curve[0];
                        let q = curve[curve.len() - 1];
                        let x = surface_point_with_cache(
                            surface,
                            Point2::new(u0, v1),
                            &mut point_cache,
                        );
                        let y = surface_point_with_cache(
                            surface,
                            Point2::new(u1, v1),
                            &mut point_cache,
                        );
                        let vec0 = polyline_on_surface(surface, q, y, tolerance, &mut point_cache);
                        let vec1 = polyline_on_surface(surface, y, x, tolerance, &mut point_cache);
                        let vec2 = polyline_on_surface(surface, x, p, tolerance, &mut point_cache);
                        closed.push(connect_edges([vec0, vec1, vec2, curve]));
                    } else if q.x < p.x - TOLERANCE {
                        normalize_range(&mut curve, 0, (u0, u1));
                        let p = curve[0];
                        let q = curve[curve.len() - 1];
                        let x = surface_point_with_cache(
                            surface,
                            Point2::new(u1, v0),
                            &mut point_cache,
                        );
                        let y = surface_point_with_cache(
                            surface,
                            Point2::new(u0, v0),
                            &mut point_cache,
                        );
                        let vec0 = polyline_on_surface(surface, q, y, tolerance, &mut point_cache);
                        let vec1 = polyline_on_surface(surface, y, x, tolerance, &mut point_cache);
                        let vec2 = polyline_on_surface(surface, x, p, tolerance, &mut point_cache);
                        closed.push(connect_edges([vec0, vec1, vec2, curve]));
                    } else if p.y < q.y - TOLERANCE {
                        normalize_range(&mut curve, 1, (v0, v1));
                        let p = curve[0];
                        let q = curve[curve.len() - 1];
                        let x = surface_point_with_cache(
                            surface,
                            Point2::new(u0, v0),
                            &mut point_cache,
                        );
                        let y = surface_point_with_cache(
                            surface,
                            Point2::new(u0, v1),
                            &mut point_cache,
                        );
                        let vec0 = polyline_on_surface(surface, q, y, tolerance, &mut point_cache);
                        let vec1 = polyline_on_surface(surface, y, x, tolerance, &mut point_cache);
                        let vec2 = polyline_on_surface(surface, x, p, tolerance, &mut point_cache);
                        closed.push(connect_edges([vec0, vec1, vec2, curve]));
                    } else if q.y < p.y - TOLERANCE {
                        normalize_range(&mut curve, 1, (v0, v1));
                        let p = curve[0];
                        let q = curve[curve.len() - 1];
                        let x = surface_point_with_cache(
                            surface,
                            Point2::new(u1, v1),
                            &mut point_cache,
                        );
                        let y = surface_point_with_cache(
                            surface,
                            Point2::new(u1, v0),
                            &mut point_cache,
                        );
                        let vec0 = polyline_on_surface(surface, q, y, tolerance, &mut point_cache);
                        let vec1 = polyline_on_surface(surface, y, x, tolerance, &mut point_cache);
                        let vec2 = polyline_on_surface(surface, x, p, tolerance, &mut point_cache);
                        closed.push(connect_edges([vec0, vec1, vec2, curve]));
                    }
                }
            }
            2 => {
                // SAFETY: open.len() == 2 was matched above.
                let mut curve1 = open.pop().unwrap();
                let mut curve0 = open.pop().unwrap();
                fn end_pts<T: Copy>(vec: &[T]) -> (T, T) { (vec[0], vec[vec.len() - 1]) }
                let full_period_pair_aligned =
                    align_full_period_open_pair(&mut curve0, &mut curve1, surface);
                let ((p0, p1), (q0, q1)) = (end_pts(&curve0), end_pts(&curve1));
                if !full_period_pair_aligned && !p0.x.near(&p1.x) && !q0.x.near(&q1.x) {
                    if let Some(period) = surface.period_u() {
                        align_periodic_open_pair(&curve0, &mut curve1, 0, period);
                    } else if let (Some(urange), _) = surface.try_range_tuple() {
                        normalize_range(&mut curve0, 0, urange);
                        normalize_range(&mut curve1, 0, urange);
                    }
                } else if !full_period_pair_aligned && !p0.y.near(&p1.y) && !q0.y.near(&q1.y) {
                    if let Some(period) = surface.period_v() {
                        align_periodic_open_pair(&curve0, &mut curve1, 1, period);
                    } else if let (_, Some(vrange)) = surface.try_range_tuple() {
                        normalize_range(&mut curve0, 1, vrange);
                        normalize_range(&mut curve1, 1, vrange);
                    }
                }
                let ((p0, p1), (q0, q1)) = (end_pts(&curve0), end_pts(&curve1));
                let current_score = open_pair_connector_score(p0, p1, q0, q1);
                let reversed_score = open_pair_connector_score(p0, p1, q1, q0);
                if reversed_score + TOLERANCE < current_score {
                    curve1.reverse();
                }
                let ((p0, p1), (q0, q1)) = (end_pts(&curve0), end_pts(&curve1));
                let vec0 = polyline_on_surface(surface, p1, q0, tolerance, &mut point_cache);
                let vec1 = polyline_on_surface(surface, q1, p0, tolerance, &mut point_cache);
                closed.push(connect_edges([curve0, vec0, curve1, vec1]));
            }
            _ => {}
        }
        if closed.len() == 1 && !loop_orientation(&closed[0]) {
            closed[0].reverse();
        }
        if !closed.iter().any(|curve| loop_orientation(curve))
            && let (Some((u0, u1)), Some((v0, v1))) = surface.try_range_tuple()
        {
            let p = [
                surface_point_with_cache(surface, Point2::new(u0, v0), &mut point_cache),
                surface_point_with_cache(surface, Point2::new(u1, v0), &mut point_cache),
                surface_point_with_cache(surface, Point2::new(u1, v1), &mut point_cache),
                surface_point_with_cache(surface, Point2::new(u0, v1), &mut point_cache),
            ];
            let vec0 = polyline_on_surface(surface, p[0], p[1], tolerance, &mut point_cache);
            let vec1 = polyline_on_surface(surface, p[1], p[2], tolerance, &mut point_cache);
            let vec2 = polyline_on_surface(surface, p[2], p[3], tolerance, &mut point_cache);
            let vec3 = polyline_on_surface(surface, p[3], p[0], tolerance, &mut point_cache);
            closed.push(connect_edges([vec0, vec1, vec2, vec3]));
        }
        normalize_closed_loop_winding(&mut closed);
        let (mut uv_min, mut uv_max) = (
            Point2::new(f64::INFINITY, f64::INFINITY),
            Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
        );
        for pt in closed.iter().flatten() {
            uv_min.x = f64::min(uv_min.x, pt.x);
            uv_min.y = f64::min(uv_min.y, pt.y);
            uv_max.x = f64::max(uv_max.x, pt.x);
            uv_max.y = f64::max(uv_max.y, pt.y);
        }
        Self {
            loops: closed,
            uv_min,
            uv_max,
        }
    }

    /// Whether `c` is included in the domain with boundary = `self`.
    pub(super) fn include(&self, c: Point2) -> bool {
        // AABB early reject.
        if c.x < self.uv_min.x || c.x > self.uv_max.x || c.y < self.uv_min.y || c.y > self.uv_max.y
        {
            return false;
        }
        let t = TAU * HashGen::hash1(c);
        let r = Vector2::new(f64::cos(t), f64::sin(t));
        self.loops
            .iter()
            .flat_map(|vec| vec.iter().circular_tuple_windows())
            .try_fold(0_i32, move |counter, (p0, p1)| {
                let a = **p0 - c;
                let b = **p1 - c;
                let s0 = r.x * a.y - r.y * a.x; // v times a.
                let s1 = r.x * b.y - r.y * b.x; // v times b.
                let s2 = a.x * b.y - a.y * b.x; // a times b.
                let x = s2 / (s1 - s0);
                if x.so_small() && s0 * s1 < 0.0 {
                    None
                } else if x > 0.0 && s0 <= 0.0 && s1 > 0.0 {
                    Some(counter + 1)
                } else if x > 0.0 && s0 >= 0.0 && s1 < 0.0 {
                    Some(counter - 1)
                } else {
                    Some(counter)
                }
            })
            .map(|counter| counter > 0)
            .unwrap_or(false)
    }

    pub(super) fn isoparametric_curves<S: PreMeshableSurface>(
        &self,
        surface: &S,
        options: IsoparametricCurveOptions,
    ) -> Vec<Vec<Point3>> {
        if self.loops.is_empty()
            || options.samples_per_direction == 0
            || options.segments_per_curve == 0
            || self.uv_min.x >= self.uv_max.x
            || self.uv_min.y >= self.uv_max.y
        {
            Vec::new()
        } else {
            (0..=1)
                .flat_map(|axis| {
                    self.isoparametric_axis_values(axis, options.samples_per_direction)
                        .into_iter()
                        .flat_map(move |axis_value| {
                            self.isoparametric_uv_curves(
                                axis,
                                axis_value,
                                options.segments_per_curve,
                            )
                            .into_iter()
                            .map(|curve| {
                                curve
                                    .into_iter()
                                    .map(|uv| surface.evaluate(uv.x, uv.y))
                                    .collect::<Vec<_>>()
                            })
                        })
                })
                .collect()
        }
    }

    fn isoparametric_axis_values(&self, axis: usize, count: usize) -> Vec<f64> {
        let (min, max) = match axis {
            0 => (self.uv_min.x, self.uv_max.x),
            _ => (self.uv_min.y, self.uv_max.y),
        };
        (0..count)
            .map(|index| {
                let parameter = (index + 1) as f64 / (count + 1) as f64;
                min + (max - min) * parameter
            })
            .collect()
    }

    fn isoparametric_uv_curves(
        &self,
        constant_axis: usize,
        constant_value: f64,
        segments: usize,
    ) -> Vec<Vec<Point2>> {
        let variable_axis = 1 - constant_axis;
        let (min, max) = match variable_axis {
            0 => (self.uv_min.x, self.uv_max.x),
            _ => (self.uv_min.y, self.uv_max.y),
        };
        let samples = (0..=segments)
            .map(|index| {
                let parameter = index as f64 / segments as f64;
                let variable_value = min + (max - min) * parameter;
                match constant_axis {
                    0 => Point2::new(constant_value, variable_value),
                    _ => Point2::new(variable_value, constant_value),
                }
            })
            .collect::<Vec<_>>();
        self.trim_uv_samples(samples)
    }

    fn trim_uv_samples(&self, samples: Vec<Point2>) -> Vec<Vec<Point2>> {
        let (_, mut curves, mut current) = samples.into_iter().fold(
            (
                None::<(Point2, bool)>,
                Vec::<Vec<Point2>>::new(),
                Vec::<Point2>::new(),
            ),
            |(previous, mut curves, mut current), uv| {
                let included = self.include(uv);
                if let Some((previous_uv, previous_included)) = previous
                    && previous_included != included
                {
                    let crossing =
                        self.isoparametric_boundary_crossing(previous_uv, uv, previous_included);
                    push_isoparam_uv(&mut current, crossing);
                    if previous_included {
                        push_finished_isoparam_curve(&mut curves, &mut current);
                    }
                    if included {
                        push_isoparam_uv(&mut current, crossing);
                    }
                }
                if included {
                    push_isoparam_uv(&mut current, uv);
                }
                (Some((uv, included)), curves, current)
            },
        );
        push_finished_isoparam_curve(&mut curves, &mut current);
        curves
    }

    fn isoparametric_boundary_crossing(
        &self,
        front: Point2,
        back: Point2,
        front_included: bool,
    ) -> Point2 {
        let (mut included, mut excluded) = match front_included {
            true => (front, back),
            false => (back, front),
        };
        (0..ISOPARAM_BOUNDARY_SEARCH_ITERATIONS).for_each(|_| {
            let middle = Point2::new(
                (included.x + excluded.x) * 0.5,
                (included.y + excluded.y) * 0.5,
            );
            if self.include(middle) {
                included = middle;
            } else {
                excluded = middle;
            }
        });
        Point2::new(
            (included.x + excluded.x) * 0.5,
            (included.y + excluded.y) * 0.5,
        )
    }

    /// Inserts points and adds constraint into triangulation.
    pub(super) fn insert_to(
        &self,
        triangulation: &mut Cdt,
        boundary_map: &mut HashMap<FixedVertexHandle, Point3>,
    ) -> (usize, usize, usize) {
        let poly2tri: Vec<_> = self
            .loops
            .iter()
            .flatten()
            .map(|pt| {
                let p = [spade_round(pt.x), spade_round(pt.y)];
                match triangulation.insert(SPoint2::from(p)) {
                    Err(_) => None,
                    Ok(idx) => {
                        boundary_map.insert(idx, pt.point);
                        Some(idx)
                    }
                }
            })
            .collect();
        let split_vertices = triangulation
            .vertices()
            .map(|vertex| {
                let point = vertex.position();
                (vertex.fix(), Point2::new(point.x, point.y))
            })
            .collect::<Vec<_>>();
        let mut counter = 0;
        let mut added_constraints = 0usize;
        let mut skipped_constraints = 0usize;
        let mut add_constraint = |front: FixedVertexHandle, back: FixedVertexHandle| {
            if front == back {
                false
            } else {
                let front_point = triangulation.vertex(front).position();
                let back_point = triangulation.vertex(back).position();
                let front_point = Point2::new(front_point.x, front_point.y);
                let back_point = Point2::new(back_point.x, back_point.y);
                let mut chain = split_vertices
                    .iter()
                    .filter_map(|(handle, point)| {
                        if *handle == front || *handle == back {
                            None
                        } else {
                            boundary_segment_parameter(*point, front_point, back_point)
                                .map(|parameter| (parameter, *handle))
                        }
                    })
                    .collect::<Vec<_>>();
                chain.sort_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0));
                let handles = iter::once(front)
                    .chain(chain.into_iter().map(|(_, handle)| handle))
                    .chain(iter::once(back))
                    .collect::<Vec<_>>();
                let mut handled = false;
                handles.windows(2).for_each(|window| {
                    if window[0] != window[1] {
                        handled = true;
                        add_direct_cdt_constraint(
                            triangulation,
                            &mut added_constraints,
                            &mut skipped_constraints,
                            window[0],
                            window[1],
                        );
                    }
                });
                handled
            }
        };
        self.loops.iter().map(Vec::len).for_each(|len| {
            let range = counter..counter + len;
            counter += len;
            let mut prev: Option<usize> = None;
            range.circular_tuple_windows().for_each(|(i, j)| {
                let Some(vj) = poly2tri[j] else { return };
                if let Some(p) = prev {
                    let Some(v) = poly2tri[p] else { return };
                    if add_constraint(v, vj) {
                        prev = None;
                    }
                } else {
                    let Some(vi) = poly2tri[i] else { return };
                    if !add_constraint(vi, vj) {
                        prev = Some(i);
                    }
                }
            });
        });
        (boundary_map.len(), added_constraints, skipped_constraints)
    }
}

fn polyline_on_surface(
    surface: impl PreMeshableSurface,
    p: SurfacePoint,
    q: SurfacePoint,
    tolerance: f64,
    point_cache: &mut HashMap<UvKey, Point3>,
) -> Vec<SurfacePoint> {
    use monstertruck_geometry::prelude::*;
    let line = Line(p.uv, q.uv);
    let pcurve = ParameterCurve::new(line, &surface);
    let (vec, _) = pcurve.parameter_division(pcurve.range_tuple(), tolerance);
    vec.into_iter()
        .map(|t| {
            let uv = line.evaluate(t);
            surface_point_with_cache(&surface, uv, point_cache)
        })
        .collect()
}

#[cfg(test)]
mod tests;
