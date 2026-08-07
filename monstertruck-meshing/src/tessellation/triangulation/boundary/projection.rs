//! `PolyBoundaryPiece`: projecting a face's 3D wire into the surface's
//! parameter space, including the restart ladder and the verified-footpoint
//! solver rung.
//!
//! Split out of the module file so the source stays readable; the module path
//! is unchanged.

use super::*;
use mesh::spade_round;
use std::sync::OnceLock;

/// Trials for the verified nearest-parameter rung, matching the 100 the
/// robust path's `search_nearest_parameter_sp` uses by default.
const VERIFIED_FOOTPOINT_TRIALS: usize = 100;

/// `MT_MESH_VERIFIED_FOOTPOINT`: the spec 012 V1 treatment arm, **DEFAULT ON**.
/// See [`verified_footpoint`] for what it does and what it moves. Set the
/// variable to `0` for the pre-`882f9b6f` control arm -- both arms stay in one
/// binary (ledger M13). Read once, so a mid-run change is not observable.
fn verified_footpoint_enabled() -> bool {
    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| !matches!(env::var("MT_MESH_VERIFIED_FOOTPOINT").as_deref(), Ok("0")))
}

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
///
/// # `MT_MESH_VERIFIED_FOOTPOINT=0` -- the control arm (spec 019 R1c, task #80)
///
/// **DEFAULT ON.** Setting the variable to `0` selects the pre-`882f9b6f`
/// behaviour -- the rung returns `None` and the face drops as it used to -- so
/// both arms live in ONE binary (ledger M13). This fix shipped without an A/B
/// arm, and its absence is what kept the regression below unattributable for
/// three rounds of analysis.
///
/// **WHY THE ARM IS NOT COSMETIC: this rung changes BOOLEAN OUTPUT, not just
/// mesh output.** The boolean meshes shells internally at the operation's own
/// tolerance (`max(tol, 10*TOLERANCE)` in
/// `integrate::healing::{shell_signed_volume, solid_signed_volume}` and
/// `pipeline::orientation::resolve_material_side`), and carries a per-face
/// `PolygonMesh` into the SSI. A face that this rung rescues is a face those
/// consumers previously did not see, so the rung can move the assembled B-rep.
///
/// Measured on `SW-B3-SLIVER-ODD-ANGLE-Uab-T00-S1-D106-Ga`
/// (`sweep::runner::tests::smoke_sliver_union_near_contact_pass`), one binary,
/// oracle `1.8060271164836628`:
///
/// | arm | result B-rep | hole-free volume | vs oracle |
/// |---|---|---|---|
/// | `MT_MESH_VERIFIED_FOOTPOINT=0` | 24 faces, 88 edges | 1.8058267975602282 | -0.0111% |
/// | default (ON) | 27 faces, 94 edges | 1.7588726252461468 | **-2.611%** |
///
/// The rescued vertex is `(1, 0.995969663, 1)` at residual `3.927e-3` against
/// the operation's `tolerance = 0.01`: far outside the old fixed `1e-6`, well
/// inside what the caller asked for. The rung is doing exactly what it says.
///
/// **The defect it exposes is elsewhere, and reverting the rung is the wrong
/// fix.** `or()` arms the P-D106 certified sub-tolerance face repair ONLY on the
/// ERROR path of a completed first pass (`integrate::mod.rs`, "no currently-green
/// union can move"). With the rung off, the first pass refuses and the repair
/// runs; with it on the first pass CLOSES A MANIFOLD, so the repair never runs
/// and its un-repaired answer is returned. Counted under `MT_BOOL_TRACE`: the
/// control arm executes the pipeline twice (`divide_face` 46, `classify` 4,
/// `build_shells` 2), the default arm once (24, 2, 1). A failure-path-only
/// repair is silently bypassed the moment an unrelated improvement stops the
/// failure -- ledger class C18.
fn verified_footpoint<S: PreMeshableSurface>(
    surface: &S,
    point: Point3,
    hint: (f64, f64),
    tolerance: f64,
) -> Option<(f64, f64)> {
    if !verified_footpoint_enabled() {
        return None;
    }
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
pub(in crate::tessellation::triangulation) struct PolyBoundaryPiece(
    pub(in crate::tessellation::triangulation) Vec<SurfacePoint>,
);

impl PolyBoundaryPiece {
    fn rounded_uv_key(point: SurfacePoint) -> (u64, u64) {
        (
            spade_round(point.x).to_bits(),
            spade_round(point.y).to_bits(),
        )
    }

    pub(in crate::tessellation::triangulation) fn is_cdt_compatible(&self) -> bool {
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

    pub(in crate::tessellation::triangulation) fn try_new_from_trimmed<'a, S, T, C>(
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

    pub(in crate::tessellation::triangulation) fn try_new_from_aligned_trimmed<'a, S, T, C>(
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

    pub(in crate::tessellation::triangulation) fn try_new_from_exact<
        'a,
        S: PreMeshableSurface,
        C: ParameterBoundary2D<S> + 'a,
    >(
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

    pub(in crate::tessellation::triangulation) fn try_new_from_aligned_exact<
        S: PreMeshableSurface,
    >(
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

    pub(super) fn normalize_axis(
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

    pub(in crate::tessellation::triangulation) fn try_new<S: PreMeshableSurface>(
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
