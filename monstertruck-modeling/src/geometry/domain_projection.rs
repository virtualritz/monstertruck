//! Projecting a 3-D point onto a surface INSIDE that surface's own parameter
//! domain: what the reported domain is worth per surface kind, the four-attempt
//! solver plan, and the `MT_MODELING_DEBUG_PROJECTION` census lens.

#[cfg(test)]
mod project_domain_tests;

use super::*;

/// The amount by which `value` lies OUTSIDE `range` on one surface parameter
/// axis, or `0.0` when it lies inside (or the axis has no reported bound).
///
/// A periodic axis identifies `value` with every `value + k * period`, so the
/// question there is never whether THIS representative is inside the range but
/// whether ANY whole-period offset of it is -- mirroring the period branch of
/// the STEP loader's `normalize_axis`
/// (`monstertruck-io/src/step/load/step_geometry/geom_impls/`), except that this
/// one only MEASURES; it never rewrites the value.
///
/// A non-finite value is infinitely far outside every bounded domain.
fn parameter_axis_excess(value: f64, range: Option<(f64, f64)>, period: Option<f64>) -> f64 {
    let Some((min, max)) = range else {
        return 0.0;
    };
    if !value.is_finite() {
        return f64::INFINITY;
    }
    if let Some(period) = period.filter(|period| *period > 0.0) {
        let shifted = value - f64::floor((value - min) / period) * period;
        // `shifted` is in `[min, min + period)`; a range spanning a full period
        // therefore always contains it.
        return if shifted <= max {
            0.0
        } else {
            f64::min(shifted - max, min + period - shifted)
        };
    }
    if value < min {
        min - value
    } else if value > max {
        value - max
    } else {
        0.0
    }
}

/// A surface's own parameter domain, resolved once per trim curve so the
/// per-sample domain test costs nothing.
#[derive(Clone, Copy, Debug)]
struct SurfaceParameterDomain {
    u_range: Option<(f64, f64)>,
    v_range: Option<(f64, f64)>,
    period_u: Option<f64>,
    period_v: Option<f64>,
}

/// Whether a curve's reported `parameter_range` BOUNDS it, or is a nominal
/// segment stamped onto an analytically unbounded curve.
///
/// `Line::parameter_range` is the hardcoded `[0, 1]` of
/// `monstertruck-geometry/src/specifieds/line/mod.rs` ("Return `0.0..=1.0` i.e. we
/// regard it as a segment"), and `Line::evaluate` is `p0 + t * (p1 - p0)`, so
/// the parameter is a multiple of the CHORD, not a fraction of any real extent.
/// The STEP loader builds a cylinder's profile as `Line(p, p + z)` with a UNIT
/// axis direction (`monstertruck-io/src/step/load/step_types/`,
/// `From<&CylindricalSurface>`), so `t` there is a world-scale axial distance
/// on an axially UNBOUNDED surface and `[0, 1]` is one arbitrary metre of it.
///
/// Everything else answers `true`, which is the SAFE default: it leaves the
/// domain test live and the behaviour unchanged. Only a range that is provably
/// a placeholder may be dropped, because dropping a real one would cost a
/// genuine C3 rescue.
fn reported_range_bounds_the_curve(curve: &Curve) -> bool {
    match curve {
        Curve::Line(_) => false,
        // Knot vectors, and the delegating decorators, all report the extent
        // they actually carry data for.
        Curve::BsplineCurve(_)
        | Curve::NurbsCurve(_)
        | Curve::ParameterCurve(_)
        | Curve::IntersectionCurve(_) => true,
    }
}

/// Whether each axis of `surface`'s reported parameter range is a real BOUND,
/// as `(u, v)`.
///
/// **This is the comparand question, and it is not rhetorical.**
/// [`SurfaceParameterDomain::contains`] compares a solved `(u, v)` against
/// `try_range_tuple()`. The solved value is always in the surface's own
/// intrinsic frame. The reported range is in that frame too -- but for two
/// variants it is not a MEASUREMENT of the surface, it is a placeholder:
///
/// - `Plane::parameter_range` (`monstertruck-geometry/src/specifieds/plane.rs`)
///   is a hardcoded `[0, 1] x [0, 1]` whose own doc comment explains that a
///   plane is infinite and that the square exists only so `range_tuple()` has
///   something to return instead of panicking. The STEP loader builds planes
///   from an axis placement with unit direction vectors, so `u` and `v` are
///   world-scale distances and a trim at `u = 12.5` is perfectly ordinary.
/// - a `RevolutionSurface`'s PROFILE axis inherits the profile curve's range
///   (`monstertruck-geometry/src/decorators/revolved_curve.rs`,
///   `parameter_range`), which for the cylinders and cones the STEP loader
///   emits is the nominal unit segment described on
///   [`reported_range_bounds_the_curve`]. Its TURN axis, `[0, 2pi)` with a
///   matching period, is real. `Processor` swaps the two axes when its
///   orientation is reversed (`decorators/processor.rs`, `parameter_range`),
///   and `From<&CylindricalSurface>` inverts, so on a loaded cylinder the
///   PERIODIC axis is `u` and the placeholder is `v`.
///
/// Comparing a real parameter against a placeholder is C2's signature exactly:
/// both sides are `Option<(f64, f64)>`, the compiler is happy, and the answer
/// is meaningless. It is also why widening the guard would be the wrong repair
/// -- the guard is not too tight, it is pointed at the wrong quantity, and on
/// the surfaces where the quantity IS right (knot-bounded nets, periodic axes)
/// it is doing real work.
///
/// MEASURED, spec 011 T5 baseline, before this function existed: of 18,453
/// out-of-domain points across ap224 and boxy (74.7% of 24,699 -- not the
/// 5,019 / 20.3% the T5 write-up reports, which is ap224's figure alone),
/// 18,428 were on a plane axis or a revolution profile axis and NOT ONE of
/// those was ever rescued. Only 25 were on `NurbsSurface`, where the range
/// comes from a knot vector -- and 23 of those 25 are every rescue the guard
/// has ever made. Per surface kind, `fallback` (out-of-domain, nothing rescued
/// it) before this function:
///
/// | | plane | revolution | nurbs |
/// |---|---|---|---|
/// | ap224, points | 5,691 | 2,018 | 3,556 |
/// | ap224, fallback | 3,266 | 1,728 | **2** |
/// | ap224, rescues | 0 | 0 | **23** |
/// | boxy, points | 11,354 | 2,080 | -- |
/// | boxy, fallback | 11,354 | 2,080 | -- |
/// | boxy, rescues | 0 | 0 | -- |
///
/// The 2,425 ap224 plane points and 290 revolution points that DID test
/// in-domain are not a counter-example, they are the tell: the planes are the
/// small ones whose world-scale trim happens to fall under 1.0, and the
/// revolution hits are samples landing on `v = -0.0`, which `-0.0 < 0.0 ==
/// false` admits into `[0, 1]`. Nothing about either is geometric.
///
/// AND THE EFFECT OF DROPPING THEM, measured the same way: every projected
/// `(u, v)` on both fixtures is BYTE-IDENTICAL, all 977 per-chain digests and
/// per-chain `u`/`v` extents unchanged. Only the cost moves --
/// **ap224 25,637 -> 11,317 solves (-56%), boxy 53,436 -> 13,434 (-75%)** --
/// and `rejected` holds at 23. That identity is not luck: a placeholder axis
/// never rescued anything, so the chain always ran to exhaustion and returned
/// attempt 0's answer, which is exactly the answer attempt 0 is now accepted
/// on. Pinned by
/// `project_domain_tests::a_placeholder_domain_costs_one_solve_per_point_not_four`
/// and `::a_knot_bounded_surface_keeps_its_domain_and_its_rejection`.
fn reported_range_bounds_the_surface(surface: &Surface) -> (bool, bool) {
    match surface {
        Surface::Plane(_) => (false, false),
        // Knot vectors: the net genuinely carries no data outside them, which
        // is the whole premise of C3's extrapolation trap.
        Surface::BsplineSurface(_) | Surface::NurbsSurface(_) => (true, true),
        // Hardcoded `[0, 1]^2` like the plane's, but a T-mesh IS conventionally
        // parameterised over a unit cell, and no fixture exercises it. Left
        // authoritative on the safe default: this keeps today's behaviour.
        Surface::TsplineSurface(_) => (true, true),
        // Analytic and REAL on both axes: the sphere's `u` is the meridian
        // `[0, pi]` and its `v` a full turn; both torus axes are full turns.
        // Neither is the `[0, 1]` placeholder U2 found on the cone/cylinder
        // axial axis, and both agree with what these faces reported while they
        // were still rational nets (`(true, true)`), so this arm is
        // behaviour-preserving.
        Surface::SphericalSurface(_) | Surface::ToroidalSurface(_) => (true, true),
        Surface::RevolutionSurface(processor) => {
            let profile = reported_range_bounds_the_curve(processor.entity().entity_curve());
            // A full turn is always a real bound, and it carries a period.
            let turn = true;
            if processor.orientation() {
                (profile, turn)
            } else {
                (turn, profile)
            }
        }
    }
}

impl SurfaceParameterDomain {
    /// Resolves the domain the containment test may legitimately use.
    ///
    /// An axis whose reported range is a placeholder rather than a bound (see
    /// [`reported_range_bounds_the_surface`]) is recorded as `None`, which
    /// [`parameter_axis_excess`] already treats as "imposes nothing". So the
    /// guard is not widened and not disabled: it is simply not asked a question
    /// the surface cannot answer.
    fn of(surface: &Surface) -> Self {
        let (u_range, v_range) = surface.try_range_tuple();
        let (u_bounds, v_bounds) = reported_range_bounds_the_surface(surface);
        Self {
            u_range: u_range.filter(|_| u_bounds),
            v_range: v_range.filter(|_| v_bounds),
            period_u: surface.period_u(),
            period_v: surface.period_v(),
        }
    }

    /// `true` when `uv` is inside the domain up to `slack`. An axis with no
    /// reported bound imposes nothing.
    fn contains(&self, uv: Point2, slack: f64) -> bool {
        parameter_axis_excess(uv.x, self.u_range, self.period_u) <= slack
            && parameter_axis_excess(uv.y, self.v_range, self.period_v) <= slack
    }
}

/// Per-chain tally of [`project_onto_surface_domain`]'s outcomes, surfaced by
/// the `MT_MODELING_DEBUG_PROJECTION` lens. Print-only: nothing reads these
/// counters back into the geometry.
#[derive(Clone, Copy, Default)]
struct ProjectionCensus {
    /// Answers accepted because they landed inside the domain, indexed by
    /// attempt. Which SOLVER each index means depends on [`EXACT_FIRST`]: see
    /// [`attempt_plan`], and read the `order=` field of the `[proj-chain]` line
    /// before reading `a0..a3`. At `EXACT_FIRST = false` (the shipping value)
    /// index 0 is nearest-hinted, not exact-hinted.
    in_domain: [usize; 4],
    /// Points where an earlier attempt answered OUTSIDE the domain and a later
    /// one rescued it -- the out-of-domain rejection firing usefully.
    rejected: usize,
    /// Points where every attempt answered outside the domain, so the
    /// historical first answer was handed back unchanged.
    fallback: usize,
    /// Points no attempt answered for at all.
    none: usize,
    /// Solver invocations -- the Newton count. One per attempt actually run,
    /// so a point that is accepted on attempt 0 costs 1 and a point that
    /// exhausts the plan costs 4. This is what the domain test buys or spends:
    /// an out-of-domain answer does not end the chain, so every rejected point
    /// pays for the remaining attempts.
    solves: usize,
    /// Solver answers discarded because they were NOT FINITE (see
    /// [`project_onto_surface_domain`]). Counted per attempt, not per point.
    non_finite: usize,
}

thread_local! {
    static PROJECTION_CENSUS: RefCell<ProjectionCensus> =
        const { RefCell::new(ProjectionCensus {
            in_domain: [0; 4],
            rejected: 0,
            fallback: 0,
            none: 0,
            solves: 0,
            non_finite: 0,
        }) };
}

/// Whether [`project_onto_surface_domain`] asks the EXACT solver first.
///
/// `false` is the pre-spec-010 ordering (nearest-hinted first) and is kept
/// switchable ON PURPOSE: the swap is NOT inert. It does NOT match the STEP
/// loader's twin either way -- see [`attempt_plan`], which spells out why
/// `true` gets only attempt 0 in common with it.
///
/// **Measured twice, held at `false` both times.** Spec 010 (7ff.3) measured it
/// before the C4 seam fix; spec 011 T5 re-measured it after, because 7ff.3's
/// stated blocker was that face 17's trim was still known-wrong and so every
/// curve count on that face was unreliable. C4 is fixed (faces 17 and 19 now
/// read `u = [0.5, 1.0]` under BOTH orderings) and the answer did not change.
///
/// On `ap224_main_solid_union_refuses_typed`, 11,265 points over 827 chains:
///
/// | | nearest-first | exact-first |
/// |---|---|---|
/// | in-domain, attempt 0 / 1 | 6,246 / 0 | 6,198 / 48 |
/// | in-domain, attempt 2 / 3 | 23 / 0 | 23 / 0 |
/// | `rejected` (out-of-domain, rescued) | **23** | **23** |
/// | `fallback` (out-of-domain, nothing rescued it) | **4,996** | **4,996** |
/// | chains moved | -- | 315 of 827, last-bit only |
///
/// So the swap does not remove one excursion. It relabels 48 of 6,246 hinted
/// acceptances from one solver to the other and leaves both the rejection count
/// and the out-of-domain count bit-identical. **The prediction that exact-first
/// reduces C3's rejection firing is FALSIFIED, and the falsification survives
/// the C4 fix**: ordering and the domain test catch disjoint problems.
///
/// `boxy_main_solid_union_refuses_typed` does not move at all -- 0 of 150
/// chains, all 13,434 points identical, census 129 pairs / 6 ERR / 59-42-22
/// under both orders.
///
/// **The `fallback` row above is HISTORICAL and will not reproduce.** It was
/// taken before [`reported_range_bounds_the_surface`] existed, when the domain
/// test was being asked about plane and revolution axes whose reported range is
/// a placeholder. Re-running the same lens today gives `fallback = 2` on ap224
/// and `0` on boxy. `rejected` is the row that matters here and it is
/// unchanged at 23 -- the two measurements are independent, and neither the
/// ordering nor the comparand fix moved a single projected value.
///
/// AIDEV-NOTE: the swap's whole remaining effect is that the ULP movement
/// changes FOUR ap224 face pairs' SSI curve counts -- (13,3) 2 -> 1,
/// (13,4) 3 -> 2, (17,1) 2 -> 1, (17,2) 3 -> 2, emptying the 3-curve class
/// (2 pairs -> 0). Post-C4 that is four pairs, not the six 7ff.3 recorded
/// pre-C4, and (17,3) / (17,4) do not move. Which curve count is CORRECT is
/// STILL unresolved, but the reason has changed: it is no longer that the trim
/// is wrong, it is that there is no curve-count oracle for those pairs. Do not
/// flip this flag until one exists -- flipping it picks a number by flag.
const EXACT_FIRST: bool = false;

const PROJECTION_ORDER: &str = if EXACT_FIRST {
    "exact-first"
} else {
    "nearest-first"
};

/// Read once: the lens must not cost an environment lookup per sample.
fn projection_census_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| env::var_os("MT_MODELING_DEBUG_PROJECTION").is_some())
}

fn record_projection(update: impl FnOnce(&mut ProjectionCensus)) {
    if !projection_census_enabled() {
        return;
    }
    PROJECTION_CENSUS.with(|census| update(&mut census.borrow_mut()));
}

/// What attempt `attempt` of [`project_onto_surface_domain`] does, as
/// `(unhinted, exact)`.
///
/// The plan is INTERLEAVED: attempts 0 and 1 are the hinted pair, 2 and 3 the
/// unhinted pair, and the two solvers alternate within each pair.
/// [`EXACT_FIRST`] chooses only which solver opens.
///
/// **The STEP twin's plan is different, and no value of [`EXACT_FIRST`] turns
/// this one into it.** `geom_impls.rs`'s `project` groups by SOLVER --
/// exact-hinted, exact-unhinted, nearest-hinted, nearest-unhinted -- so the two
/// plans can agree on attempt 0 at most. A third variant,
/// `monstertruck-geometry/src/parameter_boundary/mod.rs`, runs the exact solver
/// hinted then unhinted and never falls back to nearest at all (and at 30
/// trials, not 100).
///
/// Recorded as code rather than prose because C3's recurrence guard is exactly
/// this: when two crates hold twins of the same routine, the asymmetry IS the
/// bug report. Pinned by
/// `project_domain_tests::the_attempt_plan_interleaves_the_solvers_and_is_not_the_step_twins`.
const fn attempt_plan(attempt: usize) -> (bool, bool) {
    (attempt >= 2, attempt.is_multiple_of(2) == EXACT_FIRST)
}

/// Projects `point` onto `surface`, preferring an answer inside the surface's
/// own parameter domain.
///
/// The four solver attempts follow [`attempt_plan`]: the two HINTED attempts
/// first, then the two unhinted ones, alternating solvers within each pair, with
/// [`EXACT_FIRST`] choosing only which solver opens. This is NOT the plan of the
/// STEP loader's twin (`monstertruck-io/src/step/load/step_geometry/geom_impls/`,
/// `sampled_parameter_boundary`), which groups by solver instead; see
/// [`attempt_plan`] for the full asymmetry. Both solvers are unbounded Newtons
/// (`monstertruck-traits/src/algo/surface/mod.rs`), so either can walk off the end
/// of the knot vector when the hint sits at a domain edge; the difference is
/// that `search_parameter` at least verifies its answer reproduces the sample
/// point, while `search_nearest_parameter` returns whatever the iteration
/// settled on. Asking the checked solver first is therefore strictly the better
/// opening move, and the chain feeds each answer forward as the next hint, so
/// the opening move is what a whole excursion hangs on.
///
/// The domain test is the complementary belt: an answer OUTSIDE the domain no
/// longer ends the chain. On a rational net that EXTRAPOLATES the same surface
/// an off-domain answer reproduces the sample to ~1e-16, so the on-surface
/// check alone cannot catch it and the range test must.
///
/// The out-of-domain answer is still kept as a FALLBACK: if no attempt lands in
/// the domain the first answer is returned unchanged, so the domain test only
/// ever replaces an off-domain answer with an in-domain one and never turns a
/// previously-successful projection into `None`.
///
/// # A non-finite answer is discarded, not returned
///
/// The one case where this routine WILL answer `None` where the bare solver
/// chain answered `Some` is a NON-FINITE `(u, v)`. Such an answer is dropped as
/// if the solver had returned `None`, so the remaining attempts still run and a
/// finite answer can win; if none is finite the projection refuses, and the
/// caller's chain refuses typed with it.
///
/// This is deliberately NOT left to the domain test, which cannot carry it:
/// [`parameter_axis_excess`] returns `0.0` for an axis with no reported bound
/// BEFORE it looks at finiteness, so on a surface reporting no range a
/// non-finite pair would test as INSIDE the domain and be returned as a
/// first-class in-domain answer. On a surface that does report a range the
/// excess is `INFINITY`, which keeps the pair out of the in-domain branch but
/// still parks it in `fallback`, from where it is returned once no attempt
/// lands. Both routes hand a `NaN` or `inf` parameter to downstream geometry.
///
/// The STEP loader's twin has never had either route: its `normalize_axis`
/// (`monstertruck-io/src/step/load/step_geometry/geom_impls/`) returns `None` for
/// a non-finite value, so the whole chain fails typed. Rejecting rather than
/// clamping is C3's resolution style, and rejecting rather than substituting a
/// value keeps this routine's promise that every answer it returns came from a
/// solver.
///
/// **Measured, spec 011: this fires ZERO times.** `nonfinite=0` on every one of
/// the 977 `[proj-chain]` lines across `ap224_main_solid_union_refuses_typed`
/// (827 chains, 11,265 points, 25,637 solves) and
/// `boxy_main_solid_union_refuses_typed` (150 chains, 13,434 points, 53,436
/// solves). So this guard repairs NO live fixture and no measurement moves
/// under it -- it is kept because the alternative on the day it does fire is
/// handing a `NaN` parameter to the SSI, and because the STEP twin already
/// refuses there, which makes the modeling side the odd one out. Do not cite it
/// as a fix for an observed defect. Pinned by
/// `project_domain_tests::a_non_finite_solver_answer_is_never_returned`.
fn project_onto_surface_domain(
    surface: &Surface,
    point: Point3,
    hint: Option<(f64, f64)>,
    domain: SurfaceParameterDomain,
    slack: f64,
) -> Option<Point2> {
    let mut fallback: Option<Point2> = None;
    for attempt in 0..4 {
        let (unhinted, exact) = attempt_plan(attempt);
        if unhinted && hint.is_none() {
            // The first two attempts already WERE the unhinted ones.
            break;
        }
        let attempt_hint = if unhinted { None } else { hint };
        record_projection(|census| census.solves += 1);
        let found = if exact {
            surface.search_parameter(point, attempt_hint, 100)
        } else {
            surface.search_nearest_parameter(point, attempt_hint, 100)
        };
        let Some(uv) = found.map(|(u, v)| Point2::new(u, v)) else {
            continue;
        };
        // A NON-FINITE answer is not an answer. Discard it exactly as if the
        // solver had returned `None`: try the remaining attempts, and refuse
        // typed if none of them produces a finite pair. See the module note on
        // `project_onto_surface_domain` for why this cannot be left to the
        // domain test.
        if !uv.x.is_finite() || !uv.y.is_finite() {
            record_projection(|census| census.non_finite += 1);
            continue;
        }
        if domain.contains(uv, slack) {
            let rescued = fallback.is_some();
            record_projection(|census| {
                census.in_domain[attempt] += 1;
                census.rejected += usize::from(rescued);
            });
            return Some(uv);
        }
        fallback.get_or_insert(uv);
    }
    let answered = fallback.is_some();
    record_projection(|census| {
        if answered {
            census.fallback += 1;
        } else {
            census.none += 1;
        }
    });
    fallback
}

pub(super) fn sampled_parameter_boundary(
    curve: &(
         impl ParametricCurve3D<Point = Point3> + BoundedCurve + ParameterDivision1D<Point = Point3>
     ),
    surface: &Surface,
    tolerance: f64,
) -> Option<Vec<Point2>> {
    let points = curve.parameter_division(curve.range_tuple(), tolerance).1;
    let sample_count = points.len();
    let domain = SurfaceParameterDomain::of(surface);
    record_projection(|census| *census = ProjectionCensus::default());
    let project = |point: Point3, hint: Option<(f64, f64)>| {
        project_onto_surface_domain(surface, point, hint, domain, TOLERANCE)
    };
    let boundary = points
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
        });
    debug_projection_chain(sample_count, boundary.as_deref(), surface, domain);
    boundary
}

/// Which [`Surface`] variant this is, for the `[proj-chain]` lens only.
fn surface_kind(surface: &Surface) -> &'static str {
    match surface {
        Surface::Plane(_) => "plane",
        Surface::BsplineSurface(_) => "bspline",
        Surface::NurbsSurface(_) => "nurbs",
        Surface::RevolutionSurface(_) => "revolution",
        Surface::TsplineSurface(_) => "tspline",
        Surface::SphericalSurface(_) => "sphere",
        Surface::ToroidalSurface(_) => "torus",
    }
}

/// One `[proj-chain]` line per sampled trim curve (`MT_MODELING_DEBUG_PROJECTION`).
///
/// Print-only. `a0..a3` are the per-attempt in-domain acceptances (see
/// [`ProjectionCensus`]), `rejected` counts the out-of-domain answers a later
/// attempt rescued, `fallback` the ones nothing could rescue. `digest` is an
/// FNV-1a fold over the produced `(u, v)` bit patterns, so two runs that
/// produce identical chains produce identical digests and a reordering's blast
/// radius is a line-by-line diff of these.
///
/// `kind` and `dom=` report the surface variant and the domain the containment
/// test compares against, next to the `u=`/`v=` the projector actually answered
/// in. That juxtaposition is the point: C2's recurrence guard says a frame must
/// be readable off the value, and without `dom=` the reader cannot tell an
/// out-of-domain excursion from a domain that is not a bound at all.
fn debug_projection_chain(
    sample_count: usize,
    boundary: Option<&[Point2]>,
    surface: &Surface,
    domain: SurfaceParameterDomain,
) {
    if !projection_census_enabled() {
        return;
    }
    let census = PROJECTION_CENSUS.with(|census| *census.borrow());
    let bounds = boundary.and_then(|uvs| {
        uvs.iter().fold(None, |bounds: Option<[f64; 4]>, uv| {
            Some(match bounds {
                None => [uv.x, uv.x, uv.y, uv.y],
                Some([u0, u1, v0, v1]) => [u0.min(uv.x), u1.max(uv.x), v0.min(uv.y), v1.max(uv.y)],
            })
        })
    });
    let [u0, u1, v0, v1] = bounds.unwrap_or([f64::NAN; 4]);
    let digest =
        boundary
            .unwrap_or_default()
            .iter()
            .fold(0xcbf2_9ce4_8422_2325_u64, |digest, uv| {
                [uv.x, uv.y].iter().fold(digest, |digest, value| {
                    (digest ^ value.to_bits()).wrapping_mul(0x0000_0100_0000_01b3)
                })
            });
    let axis = |range: Option<(f64, f64)>, period: Option<f64>| match (range, period) {
        (None, _) => "none".to_owned(),
        (Some((lo, hi)), None) => format!("[{lo:.9},{hi:.9}]"),
        (Some((lo, hi)), Some(period)) => format!("[{lo:.9},{hi:.9}]p{period:.9}"),
    };
    eprintln!(
        "[proj-chain] order={PROJECTION_ORDER} kind={} samples={sample_count} ok={} points={} \
         a0={} a1={} a2={} a3={} rejected={} fallback={} none={} solves={} nonfinite={} \
         dom_u={} dom_v={} u=[{u0:.9},{u1:.9}] v=[{v0:.9},{v1:.9}] digest={digest:#018x}",
        surface_kind(surface),
        boundary.is_some(),
        boundary.map_or(0, <[Point2]>::len),
        census.in_domain[0],
        census.in_domain[1],
        census.in_domain[2],
        census.in_domain[3],
        census.rejected,
        census.fallback,
        census.none,
        census.solves,
        census.non_finite,
        axis(domain.u_range, domain.period_u),
        axis(domain.v_range, domain.period_v),
    );
}
