//! The boundary-projection failure lens. Split out of the module file so the
//! source stays readable; the module path is unchanged. What it measures and
//! why is the banner below.

use super::*;

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

pub(super) fn projection_debug_enabled() -> bool {
    env::var_os("MT_DEBUG_BOUNDARY_PROJECTION").is_some()
}

thread_local! {
    /// Face index of the face currently being projected, set by
    /// `shell_create_polygon` (and its compressed twins) before calling
    /// `PolyBoundaryPiece::try_new`. `usize::MAX` means "not recorded".
    static PROJECTION_DEBUG_FACE: std::cell::Cell<usize> = const { std::cell::Cell::new(usize::MAX) };
}

/// Records the face index the projection lens should attribute failures to.
/// No-op unless the lens is enabled, so the non-debug path is untouched.
pub(in crate::tessellation::triangulation) fn projection_debug_set_face(face_idx: Option<usize>) {
    if projection_debug_enabled() {
        PROJECTION_DEBUG_FACE.with(|cell| cell.set(face_idx.unwrap_or(usize::MAX)));
    }
}

pub(super) fn projection_debug_face() -> String {
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
pub(super) fn report_projection_failure<S: PreMeshableSurface>(
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
pub(super) fn report_projection_outcome(attempt: &str, ok: bool) {
    if projection_debug_enabled() {
        eprintln!(
            "BPROJ_OUTCOME face={} attempt={attempt} ok={ok}",
            projection_debug_face(),
        );
    }
}
