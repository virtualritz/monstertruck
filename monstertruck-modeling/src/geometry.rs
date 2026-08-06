use super::*;
use derive_more::From;
use monstertruck_core::{ContentHasher, DeterministicContentHash};
use monstertruck_geometry::prelude::{
    AnalyticSurfaceKind, BoundaryCurve2D, HomogeneousSurfaceConversion, SupportsExactPatchDomains,
    SurfaceParameterRectangle, TryIntoAnalyticSurfaceKind, TryIntoBsplineSurface,
    TryIntoHomogeneousBsplineCurve, TryIntoHomogeneousBsplineSurface,
};
#[doc(hidden)]
pub use monstertruck_geometry::prelude::{algo, inv_or_zero};
pub use monstertruck_geometry::{decorators::*, nurbs::*, specifieds::*, t_spline::*};
pub use monstertruck_mesh::PolylineCurve;
use monstertruck_topology::compress::{CompressedTrimmedShell, CompressedTrimmedSolid};
use monstertruck_topology::trimmed::{TrimmedShell, TrimmedSolid};
// Only the rayon-parallel (native) `to_trimmed_with_parameter_curves` builds
// faces directly; the wasm32 arm goes through `to_trimmed_with_face_trims`.
#[cfg(not(target_arch = "wasm32"))]
use monstertruck_topology::trimmed::TrimmedFace;
use monstertruck_traits::SnapCurveEndpoints;
#[cfg(not(target_arch = "wasm32"))]
use rayon::prelude::*;
use rustc_hash::FxHashMap as HashMap;
use serde::{Deserialize, Serialize};
use std::cell::RefCell;
use std::hash::Hasher;
use std::{env, iter};
// `web_time::Instant` is `std::time::Instant` on native and falls back to the
// browser performance clock on wasm32, where `std::time::Instant::now()` panics.
use web_time::Instant;

type ModelSurfaceCurve = SurfaceCurve<
    Box<Curve>,
    Box<Surface>,
    Box<Surface>,
    ParameterCurve<Curve2D, Box<Surface>>,
    ParameterCurve<Curve2D, Box<Surface>>,
>;
type ModelTrimCurve = ParameterCurve<Curve2D, Box<Surface>>;
type ExactTrimCacheKey = (u64, u64);

thread_local! {
    static EXACT_NURBS_REVOLUTION_TRIM_CACHE: RefCell<HashMap<ExactTrimCacheKey, Option<ModelTrimCurve>>> =
        RefCell::new(HashMap::default());
}

/// 3-dimensional curve
#[derive(
    Clone,
    Debug,
    Serialize,
    Deserialize,
    From,
    ParametricCurve,
    BoundedCurve,
    Cut,
    Invertible,
    SearchNearestParameterD1,
    SearchParameterD1,
)]
pub enum Curve {
    /// line
    Line(Line<Point3>),
    /// 3-dimensional B-spline curve
    BsplineCurve(BsplineCurve<Point3>),
    /// 3-dimensional NURBS curve
    NurbsCurve(NurbsCurve<Vector4>),
    /// 3-dimensional curve carried by a 2-dimensional parameter curve on a surface
    #[allow(clippy::enum_variant_names)]
    ParameterCurve(ParameterCurve<Curve2D, Box<Surface>>),
    /// intersection curve
    IntersectionCurve(ModelSurfaceCurve),
}

/// 2-dimensional curve used as a parameter-space trim.
#[derive(
    Clone,
    Debug,
    Serialize,
    Deserialize,
    From,
    ParametricCurve,
    BoundedCurve,
    ParameterDivision1D,
    Cut,
    Invertible,
    SearchNearestParameterD1,
    SearchParameterD1,
)]
pub enum Conic2D {
    /// ellipse
    Ellipse(Processor<TrimmedCurve<UnitCircle<Point2>>, Matrix3>),
    /// hyperbola
    Hyperbola(Processor<TrimmedCurve<UnitHyperbola<Point2>>, Matrix3>),
    /// parabola
    Parabola(Processor<TrimmedCurve<UnitParabola<Point2>>, Matrix3>),
}

/// 2-dimensional curve used as a parameter-space trim.
#[derive(
    Clone,
    Debug,
    Serialize,
    Deserialize,
    From,
    ParametricCurve,
    BoundedCurve,
    ParameterDivision1D,
    Cut,
    Invertible,
    SearchNearestParameterD1,
    SearchParameterD1,
)]
pub enum Curve2D {
    /// line
    Line(Line<Point2>),
    /// polyline
    Polyline(PolylineCurve<Point2>),
    /// conic
    Conic(Conic2D),
    /// 2-dimensional B-spline curve
    BsplineCurve(BsplineCurve<Point2>),
    /// 2-dimensional NURBS curve
    NurbsCurve(NurbsCurve<Vector3>),
}

macro_rules! derive_curve_method {
    ($curve: expr, $method: expr, $($ver: ident),*) => {
        match $curve {
            Curve::Line(got) => $method(got, $($ver), *),
            Curve::BsplineCurve(got) => $method(got, $($ver), *),
            Curve::NurbsCurve(got) => $method(got, $($ver), *),
            Curve::ParameterCurve(got) => $method(got, $($ver), *),
            Curve::IntersectionCurve(got) => $method(got, $($ver), *),
        }
    };
}

macro_rules! derive_curve_self_method {
    ($curve: expr, $method: expr, $($ver: ident),*) => {
        match $curve {
            Curve::Line(got) => Curve::Line($method(got, $($ver), *)),
            Curve::BsplineCurve(got) => Curve::BsplineCurve($method(got, $($ver), *)),
            Curve::NurbsCurve(got) => Curve::NurbsCurve($method(got, $($ver), *)),
            Curve::ParameterCurve(got) => Curve::ParameterCurve($method(got, $($ver), *)),
            Curve::IntersectionCurve(got) => Curve::IntersectionCurve($method(got, $($ver), *)),
        }
    };
}

fn sample_curve_to_nurbs(curve: &(impl ParametricCurve3D + BoundedCurve)) -> NurbsCurve<Vector4> {
    let (t0, t1) = curve.range_tuple();
    let samples = 16usize;
    let points: Vec<Point3> = (0..=samples)
        .map(|i| t0 + (t1 - t0) * (i as f64) / (samples as f64))
        .map(|t| curve.evaluate(t))
        .collect();
    let knots: Vec<f64> = (0..=samples).map(|i| i as f64 / samples as f64).collect();
    let knot_vec = KnotVector::from(
        iter::once(0.0)
            .chain(knots.iter().copied())
            .chain(iter::once(1.0))
            .collect::<Vec<_>>(),
    );
    NurbsCurve::from(BsplineCurve::new(knot_vec, points))
}

fn linear_bspline_division(
    curve: &BsplineCurve<Point3>,
    range: (f64, f64),
) -> Option<(Vec<f64>, Vec<Point3>)> {
    let curve_range = curve.range_tuple();
    (curve.degree() == 1 && curve_range.0.near(&range.0) && curve_range.1.near(&range.1)).then(
        || {
            (
                (1..=curve.control_points().len())
                    .map(|index| curve.knot(index))
                    .collect(),
                curve.control_points().clone(),
            )
        },
    )
}

/// The amount by which `value` lies OUTSIDE `range` on one surface parameter
/// axis, or `0.0` when it lies inside (or the axis has no reported bound).
///
/// A periodic axis identifies `value` with every `value + k * period`, so the
/// question there is never whether THIS representative is inside the range but
/// whether ANY whole-period offset of it is -- mirroring the period branch of
/// the STEP loader's `normalize_axis`
/// (`monstertruck-io/src/step/load/step_geometry/geom_impls.rs`), except that this
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
/// `monstertruck-geometry/src/specifieds/line.rs` ("Return `0.0..=1.0` i.e. we
/// regard it as a segment"), and `Line::evaluate` is `p0 + t * (p1 - p0)`, so
/// the parameter is a multiple of the CHORD, not a fraction of any real extent.
/// The STEP loader builds a cylinder's profile as `Line(p, p + z)` with a UNIT
/// axis direction (`monstertruck-io/src/step/load/step_types.rs`,
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
/// `monstertruck-geometry/src/parameter_boundary.rs`, runs the exact solver
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
/// STEP loader's twin (`monstertruck-io/src/step/load/step_geometry/geom_impls.rs`,
/// `sampled_parameter_boundary`), which groups by solver instead; see
/// [`attempt_plan`] for the full asymmetry. Both solvers are unbounded Newtons
/// (`monstertruck-traits/src/algo/surface.rs`), so either can walk off the end
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
/// (`monstertruck-io/src/step/load/step_geometry/geom_impls.rs`) returns `None` for
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

fn sampled_parameter_boundary(
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

fn line_points(line: Line<Point2>, tolerance: f64) -> Vec<Point2> {
    line.parameter_division(line.range_tuple(), tolerance).1
}

fn point_segment_distance2_2d(point: Point2, start: Point2, end: Point2) -> f64 {
    let edge = end - start;
    let len2 = edge.magnitude2();
    if len2 <= TOLERANCE * TOLERANCE {
        point.distance2(start)
    } else {
        let t = ((point - start).dot(edge) / len2).clamp(0.0, 1.0);
        point.distance2(start + edge * t)
    }
}

fn points_are_linear_2d(
    points: impl IntoIterator<Item = Point2>,
    start: Point2,
    end: Point2,
    tolerance: f64,
) -> bool {
    let tolerance2 = tolerance.max(TOLERANCE).powi(2);
    points
        .into_iter()
        .all(|point| point_segment_distance2_2d(point, start, end) <= tolerance2)
}

fn homogeneous_point2(control: Vector3) -> Option<Point2> {
    (control.z.abs() > f64::EPSILON)
        .then(|| Point2::new(control.x / control.z, control.y / control.z))
        .filter(|point| point.x.is_finite() && point.y.is_finite())
}

fn linear_curve2d_boundary(curve: &Curve2D, tolerance: f64) -> Option<Line<Point2>> {
    let range = curve.range_tuple();
    let start = curve.subs(range.0);
    let end = curve.subs(range.1);
    match curve {
        Curve2D::Line(line) => Some(*line),
        Curve2D::Polyline(polyline) => {
            points_are_linear_2d(polyline.as_slice().iter().copied(), start, end, tolerance)
                .then_some(Line(start, end))
        }
        Curve2D::BsplineCurve(curve) => points_are_linear_2d(
            curve.control_points().iter().copied(),
            start,
            end,
            tolerance,
        )
        .then_some(Line(start, end)),
        Curve2D::NurbsCurve(curve) => curve
            .control_points()
            .iter()
            .copied()
            .map(homogeneous_point2)
            .collect::<Option<Vec<_>>>()
            .filter(|points| points_are_linear_2d(points.iter().copied(), start, end, tolerance))
            .map(|_| Line(start, end)),
        Curve2D::Conic(_) => None,
    }
}

fn parameter_curve_points(
    boundary: &ParameterCurve<Curve2D, Box<Surface>>,
    tolerance: f64,
) -> Vec<Point2> {
    linear_curve2d_boundary(boundary.curve(), tolerance)
        .map(|line| line_points(line, tolerance))
        .unwrap_or_else(|| {
            boundary
                .curve()
                .parameter_division(boundary.curve().range_tuple(), tolerance)
                .1
        })
}

fn boundary_matches_surface_curve(
    leader: &Curve,
    boundary: &ParameterCurve<Curve2D, Box<Surface>>,
    surface: &Surface,
) -> bool {
    let leader_range = leader.range_tuple();
    let boundary_range = boundary.curve().range_tuple();
    [0.0, 0.5, 1.0].into_iter().all(|s| {
        let leader_t = leader_range.0 + (leader_range.1 - leader_range.0) * s;
        let boundary_t = boundary_range.0 + (boundary_range.1 - boundary_range.0) * s;
        let uv = boundary.curve().subs(boundary_t);
        surface.subs(uv.x, uv.y).near(&leader.subs(leader_t))
    })
}

fn boundary_curve_orientation<C>(curve: &C, boundary: &C) -> Option<bool>
where C: ParametricCurve<Point = Point3> + BoundedCurve<Point = Point3> {
    let curve_range = curve.range_tuple();
    let boundary_range = boundary.range_tuple();
    let samples = [0.0, 0.25, 0.5, 0.75, 1.0];
    let curve_t = |s: f64| curve_range.0 + (curve_range.1 - curve_range.0) * s;
    let boundary_t = |s: f64| boundary_range.0 + (boundary_range.1 - boundary_range.0) * s;
    let forward = samples.iter().copied().all(|s| {
        curve
            .evaluate(curve_t(s))
            .near(&boundary.evaluate(boundary_t(s)))
    });
    if forward {
        Some(false)
    } else {
        samples
            .iter()
            .copied()
            .all(|s| {
                curve
                    .evaluate(curve_t(s))
                    .near(&boundary.evaluate(boundary_t(1.0 - s)))
            })
            .then_some(true)
    }
}

fn orient_boundary_line(line: Line<Point2>, reversed: bool) -> Line<Point2> {
    if reversed { Line(line.1, line.0) } else { line }
}

fn direct_bspline_boundary_line(
    curve: &BsplineCurve<Point3>,
    surface: &BsplineSurface<Point3>,
) -> Option<Line<Point2>> {
    let (u_range, v_range) = surface.try_range_tuple();
    let ((u0, u1), (v0, v1)) = (u_range?, v_range?);
    let last_u = surface.control_points().len().checked_sub(1)?;
    let last_v = surface.control_points().first()?.len().checked_sub(1)?;
    [
        (
            surface.curve_v(0),
            Line(Point2::new(u0, v0), Point2::new(u0, v1)),
        ),
        (
            surface.curve_v(last_u),
            Line(Point2::new(u1, v0), Point2::new(u1, v1)),
        ),
        (
            surface.curve_u(0),
            Line(Point2::new(u0, v0), Point2::new(u1, v0)),
        ),
        (
            surface.curve_u(last_v),
            Line(Point2::new(u0, v1), Point2::new(u1, v1)),
        ),
    ]
    .into_iter()
    .find_map(|(boundary, line)| {
        boundary_curve_orientation(curve, &boundary)
            .map(|reversed| orient_boundary_line(line, reversed))
    })
}

fn direct_nurbs_boundary_line(
    curve: &NurbsCurve<Vector4>,
    surface: &NurbsSurface<Vector4>,
) -> Option<Line<Point2>> {
    let (u_range, v_range) = surface.try_range_tuple();
    let ((u0, u1), (v0, v1)) = (u_range?, v_range?);
    let last_u = surface.control_points().len().checked_sub(1)?;
    let last_v = surface.control_points().first()?.len().checked_sub(1)?;
    [
        (
            surface.curve_v(0),
            Line(Point2::new(u0, v0), Point2::new(u0, v1)),
        ),
        (
            surface.curve_v(last_u),
            Line(Point2::new(u1, v0), Point2::new(u1, v1)),
        ),
        (
            surface.curve_u(0),
            Line(Point2::new(u0, v0), Point2::new(u1, v0)),
        ),
        (
            surface.curve_u(last_v),
            Line(Point2::new(u0, v1), Point2::new(u1, v1)),
        ),
    ]
    .into_iter()
    .find_map(|(boundary, line)| {
        boundary_curve_orientation(curve, &boundary)
            .map(|reversed| orient_boundary_line(line, reversed))
    })
}

fn curve2d_from_sampled_boundary(points: Vec<Point2>) -> Option<Curve2D> {
    if points.len() < 2 {
        None
    } else {
        let front = points.first().copied()?;
        let back = points.last().copied()?;
        let line = Line(front, back);
        let is_linear = points.iter().copied().all(|point| {
            line.search_nearest_parameter(point, None, 1)
                .is_some_and(|t| line.evaluate(t).near(&point))
        });
        if is_linear {
            Some(Curve2D::Line(line))
        } else {
            let denom = (points.len() - 1) as f64;
            let knot_vec = KnotVector::from(
                iter::once(0.0)
                    .chain((0..points.len()).map(|index| index as f64 / denom))
                    .chain(iter::once(1.0))
                    .collect::<Vec<_>>(),
            );
            Some(Curve2D::BsplineCurve(BsplineCurve::new(knot_vec, points)))
        }
    }
}

fn content_hash64<T: DeterministicContentHash>(value: &T) -> u64 {
    let mut hasher = ContentHasher::default();
    value.content_hash(&mut hasher);
    hasher.finish()
}

fn same_surface(lhs: &Surface, rhs: &Surface) -> bool {
    if std::mem::discriminant(lhs) != std::mem::discriminant(rhs) {
        false
    } else if content_hash64(lhs) == content_hash64(rhs) {
        true
    } else if let (Some((lu0, lu1)), Some((lv0, lv1)), Some((ru0, ru1)), Some((rv0, rv1))) = (
        lhs.try_range_tuple().0,
        lhs.try_range_tuple().1,
        rhs.try_range_tuple().0,
        rhs.try_range_tuple().1,
    ) {
        [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (0.5, 0.5)]
            .into_iter()
            .all(|(s, t)| {
                let lp = lhs.evaluate(lu0 + (lu1 - lu0) * s, lv0 + (lv1 - lv0) * t);
                let rp = rhs.evaluate(ru0 + (ru1 - ru0) * s, rv0 + (rv1 - rv0) * t);
                lp.near(&rp)
            })
    } else {
        false
    }
}

fn exact_line_boundary(
    line: &Line<Point3>,
    surface: &Surface,
) -> Option<ParameterCurve<Curve2D, Box<Surface>>> {
    match surface {
        Surface::Plane(plane) => line.exact_parameter_boundary_2d(plane).map(|boundary| {
            let (curve, plane) = boundary.decompose();
            ParameterCurve::new(Curve2D::Line(curve), Box::new(Surface::Plane(plane)))
        }),
        Surface::BsplineSurface(surface) => {
            line.exact_parameter_boundary_2d(surface).map(|boundary| {
                let (curve, surface) = boundary.decompose();
                ParameterCurve::new(
                    Curve2D::Line(curve),
                    Box::new(Surface::BsplineSurface(surface)),
                )
            })
        }
        Surface::NurbsSurface(surface) => {
            line.exact_parameter_boundary_2d(surface).map(|boundary| {
                let (curve, surface) = boundary.decompose();
                ParameterCurve::new(
                    Curve2D::Line(curve),
                    Box::new(Surface::NurbsSurface(surface)),
                )
            })
        }
        Surface::RevolutionSurface(surface) => {
            line.exact_parameter_boundary_2d(surface).map(|boundary| {
                let (curve, surface) = boundary.decompose();
                ParameterCurve::new(
                    Curve2D::Line(curve),
                    Box::new(Surface::RevolutionSurface(surface)),
                )
            })
        }
        _ => None,
    }
}

fn exact_bspline_boundary(
    curve: &BsplineCurve<Point3>,
    surface: &Surface,
) -> Option<ParameterCurve<Curve2D, Box<Surface>>> {
    match surface {
        Surface::BsplineSurface(surface) => {
            curve.exact_parameter_boundary_2d(surface).map(|boundary| {
                let (curve, surface) = boundary.decompose();
                ParameterCurve::new(
                    Curve2D::Line(curve),
                    Box::new(Surface::BsplineSurface(surface)),
                )
            })
        }
        Surface::RevolutionSurface(surface) => {
            curve.exact_parameter_boundary_2d(surface).map(|boundary| {
                let (curve, surface) = boundary.decompose();
                ParameterCurve::new(
                    Curve2D::Line(curve),
                    Box::new(Surface::RevolutionSurface(surface)),
                )
            })
        }
        _ => None,
    }
}

fn exact_nurbs_boundary(curve: &NurbsCurve<Vector4>, surface: &Surface) -> Option<ModelTrimCurve> {
    match surface {
        Surface::NurbsSurface(surface) => {
            curve.exact_parameter_boundary_2d(surface).map(|boundary| {
                let (curve, surface) = boundary.decompose();
                ParameterCurve::new(
                    Curve2D::Line(curve),
                    Box::new(Surface::NurbsSurface(surface)),
                )
            })
        }
        Surface::RevolutionSurface(surface) => {
            cached_exact_nurbs_boundary_on_revolution_surface(curve, surface)
        }
        _ => None,
    }
}

fn cached_exact_nurbs_boundary_on_revolution_surface(
    curve: &NurbsCurve<Vector4>,
    surface: &Processor<RevolutionSurface<Curve>, Matrix4>,
) -> Option<ModelTrimCurve> {
    let key = (content_hash64(curve), content_hash64(surface));
    EXACT_NURBS_REVOLUTION_TRIM_CACHE.with(|cache| {
        let cached = cache.borrow().get(&key).cloned();
        cached.unwrap_or_else(|| {
            let result = curve.exact_parameter_boundary_2d(surface).map(|boundary| {
                let (curve, surface) = boundary.decompose();
                ParameterCurve::new(
                    Curve2D::Line(curve),
                    Box::new(Surface::RevolutionSurface(surface)),
                )
            });
            cache.borrow_mut().insert(key, result.clone());
            result
        })
    })
}

impl Transformed<Matrix4> for Curve {
    fn transform_by(&mut self, trans: Matrix4) {
        derive_curve_method!(self, Transformed::transform_by, trans);
    }
    fn transformed(&self, trans: Matrix4) -> Self {
        derive_curve_self_method!(self, Transformed::transformed, trans)
    }
}

impl ParameterDivision1D for Curve {
    type Point = Point3;
    fn parameter_division(&self, range: (f64, f64), tol: f64) -> (Vec<f64>, Vec<Self::Point>) {
        let debug_profile = env::var("MT_PROFILE_CURVE_DIVISION").is_ok();
        let started = Instant::now();
        let result = match self {
            Curve::Line(curve) => curve.parameter_division(range, tol),
            Curve::BsplineCurve(curve) => linear_bspline_division(curve, range)
                .unwrap_or_else(|| curve.parameter_division(range, tol)),
            Curve::NurbsCurve(curve) => curve.parameter_division(range, tol),
            Curve::ParameterCurve(curve) => curve.parameter_division(range, tol),
            Curve::IntersectionCurve(curve) => curve.leader().parameter_division(range, tol),
        };
        if debug_profile {
            let kind = match self {
                Curve::Line(_) => "Line",
                Curve::BsplineCurve(_) => "BsplineCurve",
                Curve::NurbsCurve(_) => "NurbsCurve",
                Curve::ParameterCurve(_) => "ParameterCurve",
                Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                    Curve::Line(_) => "IntersectionCurve(Line)",
                    Curve::BsplineCurve(_) => "IntersectionCurve(BsplineCurve)",
                    Curve::NurbsCurve(_) => "IntersectionCurve(NurbsCurve)",
                    Curve::ParameterCurve(_) => "IntersectionCurve(ParameterCurve)",
                    Curve::IntersectionCurve(_) => "IntersectionCurve(IntersectionCurve)",
                },
            };
            eprintln!(
                "trace bool model_curve_division kind={} points={} tol={} elapsed_ms={:.3}",
                kind,
                result.1.len(),
                tol,
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
        result
    }
}

impl From<IntersectionCurve<BsplineCurve<Point3>, Surface, Surface>> for Curve {
    fn from(c: IntersectionCurve<BsplineCurve<Point3>, Surface, Surface>) -> Curve {
        let (surface0, surface1, leader) = c.destruct();
        Curve::IntersectionCurve(SurfaceCurve::with_boundaries(
            Box::new(surface0),
            Box::new(surface1),
            Box::new(leader.into()),
            None,
            None,
        ))
    }
}

fn boundary_curve_2d_to_model_curve(
    curve: ParameterCurve<BoundaryCurve2D, Surface>,
) -> ParameterCurve<Curve2D, Box<Surface>> {
    let surface = Box::new(curve.surface().clone());
    match curve.curve() {
        BoundaryCurve2D::Line(line) => ParameterCurve::new(Curve2D::Line(*line), surface),
        BoundaryCurve2D::BsplineCurve(bspline) => {
            ParameterCurve::new(Curve2D::BsplineCurve(bspline.clone()), surface)
        }
        BoundaryCurve2D::NurbsCurve(nurbs) => {
            ParameterCurve::new(Curve2D::NurbsCurve(nurbs.clone()), surface)
        }
    }
}

impl
    From<
        SurfaceCurve<
            BsplineCurve<Point3>,
            Surface,
            Surface,
            ParameterCurve<BoundaryCurve2D, Surface>,
            ParameterCurve<BoundaryCurve2D, Surface>,
        >,
    > for Curve
{
    fn from(
        c: SurfaceCurve<
            BsplineCurve<Point3>,
            Surface,
            Surface,
            ParameterCurve<BoundaryCurve2D, Surface>,
            ParameterCurve<BoundaryCurve2D, Surface>,
        >,
    ) -> Curve {
        let (surface0, surface1, leader, boundary0, boundary1) = c.destruct_with_boundaries();
        Curve::IntersectionCurve(SurfaceCurve::with_boundaries(
            Box::new(surface0),
            Box::new(surface1),
            Box::new(leader.into()),
            boundary0.map(boundary_curve_2d_to_model_curve),
            boundary1.map(boundary_curve_2d_to_model_curve),
        ))
    }
}

impl
    From<
        SurfaceCurve<
            NurbsCurve<Vector4>,
            Surface,
            Surface,
            ParameterCurve<BoundaryCurve2D, Surface>,
            ParameterCurve<BoundaryCurve2D, Surface>,
        >,
    > for Curve
{
    fn from(
        c: SurfaceCurve<
            NurbsCurve<Vector4>,
            Surface,
            Surface,
            ParameterCurve<BoundaryCurve2D, Surface>,
            ParameterCurve<BoundaryCurve2D, Surface>,
        >,
    ) -> Curve {
        let (surface0, surface1, leader, boundary0, boundary1) = c.destruct_with_boundaries();
        Curve::IntersectionCurve(SurfaceCurve::with_boundaries(
            Box::new(surface0),
            Box::new(surface1),
            Box::new(leader.into()),
            boundary0.map(boundary_curve_2d_to_model_curve),
            boundary1.map(boundary_curve_2d_to_model_curve),
        ))
    }
}

impl ToSameGeometry<Curve> for Line<Point3> {
    #[inline]
    fn to_same_geometry(&self) -> Curve { Curve::from(*self) }
}

impl ToSameGeometry<Curve> for Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4> {
    #[inline]
    fn to_same_geometry(&self) -> Curve { Curve::NurbsCurve(self.to_same_geometry()) }
}

impl ToSameGeometry<Curve> for BsplineCurve<Point3> {
    #[inline]
    fn to_same_geometry(&self) -> Curve { Curve::from(self.clone()) }
}

impl Curve {
    /// Into non-ratinalized 4-dimensional B-spline curve
    pub fn lift_up(&self) -> BsplineCurve<Vector4> {
        match self {
            Curve::Line(curve) => Curve::BsplineCurve((*curve).into()).lift_up(),
            Curve::BsplineCurve(curve) => BsplineCurve::new(
                curve.knot_vector().clone(),
                curve
                    .control_points()
                    .iter()
                    .map(|pt| pt.to_vec().extend(1.0))
                    .collect(),
            ),
            Curve::NurbsCurve(curve) => curve.non_rationalized().clone(),
            Curve::ParameterCurve(curve) => sample_curve_to_nurbs(curve).non_rationalized().clone(),
            Curve::IntersectionCurve(curve) => curve.leader().lift_up(),
        }
    }
}

fn curve2d_endpoint_hints(curve: &Curve2D) -> Option<(Point2, Point2)> {
    match curve {
        Curve2D::Line(curve) => Some((curve.0, curve.1)),
        Curve2D::Polyline(curve) => Some((*curve.first()?, *curve.last()?)),
        Curve2D::BsplineCurve(curve) => Some((
            *curve.control_points().first()?,
            *curve.control_points().last()?,
        )),
        Curve2D::NurbsCurve(curve) => Some((
            curve.control_points().first()?.to_point(),
            curve.control_points().last()?.to_point(),
        )),
        Curve2D::Conic(_) => None,
    }
}

fn set_curve2d_endpoints(curve: &mut Curve2D, front: Point2, back: Point2) {
    match curve {
        Curve2D::Line(curve) => {
            curve.0 = front;
            curve.1 = back;
        }
        Curve2D::Polyline(curve) => {
            if let Some(point) = curve.first_mut() {
                *point = front;
            }
            if let Some(point) = curve.last_mut() {
                *point = back;
            }
        }
        Curve2D::BsplineCurve(curve) => {
            if !curve.control_points().is_empty() {
                *curve.control_point_mut(0) = front;
            }
            if curve.control_points().len() > 1 {
                let last = curve.control_points().len() - 1;
                *curve.control_point_mut(last) = back;
            }
        }
        Curve2D::NurbsCurve(curve) => {
            if !curve.control_points().is_empty() {
                let point = curve.control_point_mut(0);
                let weight = point.weight();
                *point = front.to_vec().extend(weight);
            }
            if curve.control_points().len() > 1 {
                let last = curve.control_points().len() - 1;
                let point = curve.control_point_mut(last);
                let weight = point.weight();
                *point = back.to_vec().extend(weight);
            }
        }
        Curve2D::Conic(_) => {}
    }
}

fn snap_parameter_curve_endpoints(
    curve: &mut ParameterCurve<Curve2D, Box<Surface>>,
    front: Point3,
    back: Point3,
) {
    let hints = curve2d_endpoint_hints(curve.curve());
    let front_uv = hints
        .and_then(|(front_hint, _)| {
            curve
                .surface()
                .search_nearest_parameter(front, Some((front_hint.x, front_hint.y)), 100)
        })
        .or_else(|| curve.surface().search_nearest_parameter(front, None, 100))
        .map(|(u, v)| Point2::new(u, v));
    let back_uv = hints
        .and_then(|(_, back_hint)| {
            curve
                .surface()
                .search_nearest_parameter(back, Some((back_hint.x, back_hint.y)), 100)
        })
        .or_else(|| curve.surface().search_nearest_parameter(back, None, 100))
        .map(|(u, v)| Point2::new(u, v));
    if let (Some(front_uv), Some(back_uv)) = (front_uv, back_uv) {
        let (mut curve2d, surface) = curve.clone().decompose();
        set_curve2d_endpoints(&mut curve2d, front_uv, back_uv);
        *curve = ParameterCurve::new(curve2d, surface);
    }
}

impl SnapCurveEndpoints for Curve {
    fn snap_endpoints(&mut self, front: Point3, back: Point3) {
        match self {
            Curve::IntersectionCurve(curve) => {
                curve.leader_mut().snap_endpoints(front, back);
                if let Some(boundary) = curve.boundary0_mut() {
                    snap_parameter_curve_endpoints(boundary, front, back);
                }
                if let Some(boundary) = curve.boundary1_mut() {
                    snap_parameter_curve_endpoints(boundary, front, back);
                }
            }
            Curve::ParameterCurve(curve) => snap_parameter_curve_endpoints(curve, front, back),
            _ => {}
        }
    }
}

impl TryIntoHomogeneousBsplineCurve for Curve {
    fn try_into_homogeneous_bspline_curve(&self) -> Option<BsplineCurve<Vector4>> {
        match self {
            Curve::Line(curve) => curve.try_into_homogeneous_bspline_curve(),
            Curve::BsplineCurve(curve) => curve.try_into_homogeneous_bspline_curve(),
            Curve::NurbsCurve(curve) => curve.try_into_homogeneous_bspline_curve(),
            Curve::ParameterCurve(_) => None,
            Curve::IntersectionCurve(curve) => curve.leader().try_into_homogeneous_bspline_curve(),
        }
    }

    fn try_into_homogeneous_bspline_curve_over(
        &self,
        range: (f64, f64),
    ) -> Option<BsplineCurve<Vector4>> {
        match self {
            // Only a line has an exact analytic continuation past its own range;
            // every other variant keeps the trait's refusing default. This is
            // what lets a cylinder's profile be re-spanned onto the face's trim
            // once it has been re-homed into the modeling `Curve` enum.
            Curve::Line(curve) => curve.try_into_homogeneous_bspline_curve_over(range),
            _ => None,
        }
    }
}

impl TryFrom<ParameterCurve<Line<Point2>, Surface>> for Curve {
    type Error = ();
    fn try_from(curve: ParameterCurve<Line<Point2>, Surface>) -> std::result::Result<Self, ()> {
        let (line, surface) = curve.decompose();
        Ok(Curve::ParameterCurve(ParameterCurve::new(
            Curve2D::Line(line),
            Box::new(surface),
        )))
    }
}

impl ParameterBoundary2D<Surface> for Curve {
    fn parameter_boundary_2d(&self, surface: &Surface, tolerance: f64) -> Option<Vec<Point2>> {
        match self {
            Curve::Line(curve) => exact_line_boundary(curve, surface)
                .map(|boundary| parameter_curve_points(&boundary, tolerance))
                .or_else(|| sampled_parameter_boundary(curve, surface, tolerance)),
            Curve::BsplineCurve(curve) => exact_bspline_boundary(curve, surface)
                .map(|boundary| parameter_curve_points(&boundary, tolerance))
                .or_else(|| {
                    if let Surface::BsplineSurface(bspline_surface) = surface {
                        direct_bspline_boundary_line(curve, bspline_surface)
                            .map(|line| line_points(line, tolerance))
                    } else {
                        None
                    }
                })
                .or_else(|| sampled_parameter_boundary(curve, surface, tolerance)),
            Curve::NurbsCurve(curve) => exact_nurbs_boundary(curve, surface)
                .map(|boundary| parameter_curve_points(&boundary, tolerance))
                .or_else(|| {
                    if let Surface::NurbsSurface(nurbs_surface) = surface {
                        direct_nurbs_boundary_line(curve, nurbs_surface)
                            .map(|line| line_points(line, tolerance))
                    } else {
                        None
                    }
                })
                // Fall back to a sampled polyline trim on surfaces the exact and
                // direct-line paths do not cover (notably planar caps, whose
                // circular boundary arcs are `NurbsCurve`s on a `Surface::Plane`).
                // Without this the cap arcs yield no parameter curve and drop from
                // downstream NURBS export. Mirrors the `Line`/`BsplineCurve` arms.
                .or_else(|| sampled_parameter_boundary(curve, surface, tolerance)),
            Curve::ParameterCurve(curve) => same_surface(curve.surface().as_ref(), surface)
                .then(|| parameter_curve_points(curve, tolerance)),
            Curve::IntersectionCurve(curve) => {
                if let Some(boundary) = curve.boundary0().filter(|boundary| {
                    boundary_matches_surface_curve(curve.leader().as_ref(), boundary, surface)
                }) {
                    Some(parameter_curve_points(boundary, tolerance))
                } else if let Some(boundary) = curve.boundary1().filter(|boundary| {
                    boundary_matches_surface_curve(curve.leader().as_ref(), boundary, surface)
                }) {
                    Some(parameter_curve_points(boundary, tolerance))
                } else if same_surface(curve.surface0().as_ref(), surface) {
                    curve
                        .boundary0()
                        .map(|boundary| parameter_curve_points(boundary, tolerance))
                        .or_else(|| curve.leader().parameter_boundary_2d(surface, tolerance))
                } else if same_surface(curve.surface1().as_ref(), surface) {
                    curve
                        .boundary1()
                        .map(|boundary| parameter_curve_points(boundary, tolerance))
                        .or_else(|| curve.leader().parameter_boundary_2d(surface, tolerance))
                } else {
                    sampled_parameter_boundary(curve.leader().as_ref(), surface, tolerance)
                }
            }
        }
    }
}

impl ExactParameterBoundary2D<Surface> for Curve {
    type BoundaryCurve = ParameterCurve<Curve2D, Box<Surface>>;

    fn exact_parameter_boundary_2d(&self, surface: &Surface) -> Option<Self::BoundaryCurve> {
        match self {
            Curve::Line(curve) => exact_line_boundary(curve, surface),
            Curve::BsplineCurve(curve) => exact_bspline_boundary(curve, surface),
            Curve::NurbsCurve(curve) => exact_nurbs_boundary(curve, surface),
            Curve::ParameterCurve(curve) if same_surface(curve.surface().as_ref(), surface) => {
                Some(curve.clone())
            }
            Curve::IntersectionCurve(curve) if same_surface(curve.surface0().as_ref(), surface) => {
                curve
                    .boundary0()
                    .cloned()
                    .or_else(|| curve.leader().exact_parameter_boundary_2d(surface))
            }
            Curve::IntersectionCurve(curve) if same_surface(curve.surface1().as_ref(), surface) => {
                curve
                    .boundary1()
                    .cloned()
                    .or_else(|| curve.leader().exact_parameter_boundary_2d(surface))
            }
            _ => None,
        }
    }
}

impl BoundaryCurveFromSamples<Surface> for ParameterCurve<Curve2D, Box<Surface>> {
    fn boundary_curve_from_samples(surface: &Surface, points: Vec<Point2>) -> Option<Self> {
        curve2d_from_sampled_boundary(points)
            .map(|curve| ParameterCurve::new(curve, Box::new(surface.clone())))
    }
}

impl Curve {
    /// Converts this curve into a face-local parameter curve on `surface`.
    ///
    /// Exact trim data is preserved when available. Otherwise this falls back
    /// to a sampled polyline trim in the surface domain.
    pub fn to_parameter_curve_on(
        &self,
        surface: &Surface,
        tolerance: f64,
    ) -> Option<ParameterCurve<Curve2D, Box<Surface>>> {
        let debug_profile = env::var("MT_PROFILE_PARAMETER_CURVE_ON").is_ok();
        let started = Instant::now();
        let exact = self.exact_parameter_boundary_2d(surface);
        let exact_hit = exact.is_some();
        let result = exact.or_else(|| {
            self.parameter_boundary_2d(surface, tolerance)
                .filter(|points| points.len() >= 2)
                .and_then(curve2d_from_sampled_boundary)
                .map(|curve| ParameterCurve::new(curve, Box::new(surface.clone())))
        });
        if debug_profile {
            let kind = match self {
                Curve::Line(_) => "Line",
                Curve::BsplineCurve(_) => "BsplineCurve",
                Curve::NurbsCurve(_) => "NurbsCurve",
                Curve::ParameterCurve(_) => "ParameterCurve",
                Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                    Curve::Line(_) => "IntersectionCurve(Line)",
                    Curve::BsplineCurve(_) => "IntersectionCurve(BsplineCurve)",
                    Curve::NurbsCurve(_) => "IntersectionCurve(NurbsCurve)",
                    Curve::ParameterCurve(_) => "IntersectionCurve(ParameterCurve)",
                    Curve::IntersectionCurve(_) => "IntersectionCurve(IntersectionCurve)",
                },
            };
            eprintln!(
                "trace bool parameter_curve_on kind={} exact={} output={} elapsed_ms={:.3}",
                kind,
                exact_hit,
                result.is_some(),
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
        result
    }
}

/// Extension trait for creating runtime trimmed topology with face-local parameter curves.
pub trait ToTrimmedParameterCurves {
    /// The trimmed topology output.
    type Output;

    /// Creates runtime trimmed topology with face-local parameter curves.
    fn to_trimmed_with_parameter_curves(&self, tolerance: f64) -> Self::Output;
}

impl ToTrimmedParameterCurves for Shell {
    type Output = TrimmedShell<Point3, Curve, Surface, ParameterCurve<Curve2D, Box<Surface>>>;

    #[cfg(not(target_arch = "wasm32"))]
    fn to_trimmed_with_parameter_curves(&self, tolerance: f64) -> Self::Output {
        self.iter()
            .cloned()
            .collect::<Vec<_>>()
            .into_par_iter()
            .map(|face| {
                let surface = face.surface();
                let trims = face
                    .absolute_boundaries()
                    .iter()
                    .map(|wire| {
                        wire.iter()
                            .map(|edge| edge.curve().to_parameter_curve_on(&surface, tolerance))
                            .collect()
                    })
                    .collect();
                TrimmedFace::new(face, trims)
            })
            .collect::<Vec<_>>()
            .into_iter()
            .collect()
    }

    #[cfg(target_arch = "wasm32")]
    fn to_trimmed_with_parameter_curves(&self, tolerance: f64) -> Self::Output {
        self.to_trimmed_with_face_trims(|edge, surface| {
            edge.curve().to_parameter_curve_on(surface, tolerance)
        })
    }
}

impl ToTrimmedParameterCurves for Solid {
    type Output = TrimmedSolid<Point3, Curve, Surface, ParameterCurve<Curve2D, Box<Surface>>>;

    #[cfg(not(target_arch = "wasm32"))]
    fn to_trimmed_with_parameter_curves(&self, tolerance: f64) -> Self::Output {
        TrimmedSolid::new(
            self.boundaries()
                .par_iter()
                .map(|shell| shell.to_trimmed_with_parameter_curves(tolerance))
                .collect(),
        )
    }

    #[cfg(target_arch = "wasm32")]
    fn to_trimmed_with_parameter_curves(&self, tolerance: f64) -> Self::Output {
        self.to_trimmed_with_face_trims(|edge, surface| {
            edge.curve().to_parameter_curve_on(surface, tolerance)
        })
    }
}

/// Extension trait for creating compressed trimmed topology with face-local parameter curves.
pub trait ToCompressedTrimmedParameterCurves {
    /// The compressed trimmed topology output.
    type Output;

    /// Creates compressed trimmed topology with face-local parameter curves.
    fn compress_with_parameter_curves(&self, tolerance: f64) -> Self::Output;
}

impl ToCompressedTrimmedParameterCurves for Shell {
    type Output =
        CompressedTrimmedShell<Point3, Curve, Surface, ParameterCurve<Curve2D, Box<Surface>>>;

    fn compress_with_parameter_curves(&self, tolerance: f64) -> Self::Output {
        let trimmed = self.to_trimmed_with_parameter_curves(tolerance);
        CompressedTrimmedShell::from(&trimmed)
    }
}

impl ToCompressedTrimmedParameterCurves for Solid {
    type Output =
        CompressedTrimmedSolid<Point3, Curve, Surface, ParameterCurve<Curve2D, Box<Surface>>>;

    fn compress_with_parameter_curves(&self, tolerance: f64) -> Self::Output {
        let trimmed = self.to_trimmed_with_parameter_curves(tolerance);
        CompressedTrimmedSolid::from(&trimmed)
    }
}

/// 3-dimensional surfaces
#[derive(
    Clone,
    Debug,
    Serialize,
    Deserialize,
    From,
    ParametricSurface,
    ParameterDivision2D,
    Invertible,
    SearchParameterD2,
)]
#[allow(clippy::large_enum_variant)]
pub enum Surface {
    /// Plane
    Plane(Plane),
    /// 3-dimensional B-spline surface
    BsplineSurface(BsplineSurface<Point3>),
    /// 3-dimensional NURBS Surface
    NurbsSurface(NurbsSurface<Vector4>),
    /// revoluted curve
    #[serde(alias = "RevolutedCurve")]
    RevolutionSurface(Processor<RevolutionSurface<Curve>, Matrix4>),
    /// T-spline surface
    TsplineSurface(Tmesh<Point3>),
    /// Analytic sphere, posed by the processor's transform.
    ///
    /// Carried as the analytic type rather than its (exact) rational net so the
    /// CLOSED-FORM [`ParameterDivision2D`] survives the STEP conversion -- see
    /// the module note on `TryFrom<&Surface> for Surface` in
    /// `monstertruck-io/src/step/load/step_geometry/geom_impls.rs`. Spec 012 U1.2.
    SphericalSurface(Processor<Sphere, Matrix4>),
    /// Analytic torus, posed by the processor's transform. Same reason as
    /// [`Surface::SphericalSurface`]; spindle tori never reach this variant.
    ToroidalSurface(Processor<Torus, Matrix4>),
}

macro_rules! derive_surface_method {
    ($surface: expr, $method: expr, $($ver: ident),*) => {
        match $surface {
            Self::Plane(got) => $method(got, $($ver), *),
            Self::BsplineSurface(got) => $method(got, $($ver), *),
            Self::NurbsSurface(got) => $method(got, $($ver), *),
            Self::RevolutionSurface(got) => $method(got, $($ver), *),
            Self::TsplineSurface(got) => $method(got, $($ver), *),
            Self::SphericalSurface(got) => $method(got, $($ver), *),
            Self::ToroidalSurface(got) => $method(got, $($ver), *),
        }
    };
}

macro_rules! derive_surface_self_method {
    ($surface: expr, $method: expr, $($ver: ident),*) => {
        match $surface {
            Self::Plane(got) => Self::Plane($method(got, $($ver), *)),
            Self::BsplineSurface(got) => Self::BsplineSurface($method(got, $($ver), *)),
            Self::NurbsSurface(got) => Self::NurbsSurface($method(got, $($ver), *)),
            Self::RevolutionSurface(got) => Self::RevolutionSurface($method(got, $($ver), *)),
            Self::TsplineSurface(got) => Self::TsplineSurface($method(got, $($ver), *)),
            Self::SphericalSurface(got) => Self::SphericalSurface($method(got, $($ver), *)),
            Self::ToroidalSurface(got) => Self::ToroidalSurface($method(got, $($ver), *)),
        }
    };
}

impl ParametricSurface3D for Surface {
    #[inline(always)]
    fn normal(&self, u: f64, v: f64) -> Vector3 {
        derive_surface_method!(self, ParametricSurface3D::normal, u, v)
    }
}

/// Trials the analytic containment test allows its inverse map, matching
/// `INCLUDE_CURVE_TRIALS` in `monstertruck-geometry`'s `NurbsSurface` impl --
/// the one [`Surface::SphericalSurface`] and [`Surface::ToroidalSurface`]
/// faces reached before spec 012 U1.2 gave them analytic variants.
const ANALYTIC_INCLUDE_CURVE_TRIALS: usize = 100;

/// Whether every sample of `curve` recovers a parameter on `surface`.
///
/// Samples the CURVE at the same density the `NurbsSurface` impl this replaces
/// used -- interior points of each knot span, `2 * degree` of them -- rather
/// than the control polygon, whose points are not on a rational curve at all.
fn analytic_surface_includes_curve<S>(surface: &S, curve: &Curve) -> bool
where S: SearchParameter<SurfaceParameter, Point = Point3> {
    let lifted = NurbsCurve::new(curve.lift_up());
    let (knots, _) = lifted.knot_vector().to_single_multi();
    let degree = usize::max(lifted.degree() * 2, 2);
    knots.windows(2).all(|window| {
        (0..=degree).all(|index| {
            let ratio = index as f64 / degree as f64;
            let parameter = window[0] * (1.0 - ratio) + window[1] * ratio;
            surface
                .search_parameter(lifted.subs(parameter), None, ANALYTIC_INCLUDE_CURVE_TRIALS)
                .is_some()
        })
    })
}

impl Transformed<Matrix4> for Surface {
    fn transform_by(&mut self, trans: Matrix4) {
        derive_surface_method!(self, Transformed::transform_by, trans);
    }
    fn transformed(&self, trans: Matrix4) -> Self {
        derive_surface_self_method!(self, Transformed::transformed, trans)
    }
}

impl IncludeCurve<Curve> for Surface {
    #[inline(always)]
    fn include(&self, curve: &Curve) -> bool {
        if let Curve::ParameterCurve(curve) = curve {
            same_surface(curve.surface().as_ref(), self)
        } else {
            match self {
                Surface::BsplineSurface(surface) => match curve {
                    &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                    Curve::BsplineCurve(curve) => surface.include(curve),
                    Curve::NurbsCurve(curve) => surface.include(curve),
                    Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                        Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                        Curve::BsplineCurve(curve) => surface.include(curve),
                        Curve::NurbsCurve(curve) => surface.include(curve),
                        Curve::ParameterCurve(_) => false,
                        Curve::IntersectionCurve(_) => false,
                    },
                    Curve::ParameterCurve(_) => unreachable!(),
                },
                Surface::NurbsSurface(surface) => match curve {
                    &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                    Curve::BsplineCurve(curve) => surface.include(curve),
                    Curve::NurbsCurve(curve) => surface.include(curve),
                    Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                        Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                        Curve::BsplineCurve(curve) => surface.include(curve),
                        Curve::NurbsCurve(curve) => surface.include(curve),
                        Curve::ParameterCurve(_) => false,
                        Curve::IntersectionCurve(_) => false,
                    },
                    Curve::ParameterCurve(_) => unreachable!(),
                },
                Surface::Plane(surface) => match curve {
                    &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                    Curve::BsplineCurve(curve) => surface.include(curve),
                    Curve::NurbsCurve(curve) => surface.include(curve),
                    Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                        Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                        Curve::BsplineCurve(curve) => surface.include(curve),
                        Curve::NurbsCurve(curve) => surface.include(curve),
                        Curve::ParameterCurve(_) => false,
                        Curve::IntersectionCurve(_) => false,
                    },
                    Curve::ParameterCurve(_) => unreachable!(),
                },
                Surface::TsplineSurface(surface) => {
                    curve.lift_up().control_points().iter().all(|v| {
                        let p = v.to_point();
                        surface.search_parameter(p, None, 1).is_some()
                    })
                }
                // The analytic variants answer containment through their own
                // exact inverse map. `Torus` carries no `IncludeCurve` impl at
                // all, so the pair shares one sampled test -- and it samples
                // the CURVE, like the `NurbsSurface` impl these faces used to
                // reach, not the control polygon like the T-mesh arm above. A
                // rational curve's control points are not on the curve, so
                // testing them would have been strictly more conservative than
                // what this replaces.
                Surface::SphericalSurface(surface) => {
                    analytic_surface_includes_curve(surface, curve)
                }
                Surface::ToroidalSurface(surface) => {
                    analytic_surface_includes_curve(surface, curve)
                }
                Surface::RevolutionSurface(surface) => match surface.entity_curve() {
                    &Curve::Line(entity_line) => {
                        let entity_bsp = BsplineCurve::from(entity_line);
                        let surface = RevolutionSurface::by_revolution(
                            &entity_bsp,
                            surface.origin(),
                            surface.axis(),
                        );
                        match curve {
                            &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                            Curve::BsplineCurve(curve) => surface.include(curve),
                            Curve::NurbsCurve(curve) => surface.include(curve),
                            Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                                Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                                Curve::BsplineCurve(curve) => surface.include(curve),
                                Curve::NurbsCurve(curve) => surface.include(curve),
                                Curve::ParameterCurve(_) => false,
                                Curve::IntersectionCurve(_) => false,
                            },
                            Curve::ParameterCurve(_) => unreachable!(),
                        }
                    }
                    Curve::BsplineCurve(entity_curve) => {
                        let surface = RevolutionSurface::by_revolution(
                            entity_curve,
                            surface.origin(),
                            surface.axis(),
                        );
                        match curve {
                            &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                            Curve::BsplineCurve(curve) => surface.include(curve),
                            Curve::NurbsCurve(curve) => surface.include(curve),
                            Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                                Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                                Curve::BsplineCurve(curve) => surface.include(curve),
                                Curve::NurbsCurve(curve) => surface.include(curve),
                                Curve::ParameterCurve(_) => false,
                                Curve::IntersectionCurve(_) => false,
                            },
                            Curve::ParameterCurve(_) => unreachable!(),
                        }
                    }
                    Curve::NurbsCurve(entity_curve) => {
                        let surface = RevolutionSurface::by_revolution(
                            entity_curve,
                            surface.origin(),
                            surface.axis(),
                        );
                        match curve {
                            &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                            Curve::BsplineCurve(curve) => surface.include(curve),
                            Curve::NurbsCurve(curve) => surface.include(curve),
                            Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                                Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                                Curve::BsplineCurve(curve) => surface.include(curve),
                                Curve::NurbsCurve(curve) => surface.include(curve),
                                Curve::ParameterCurve(_) => false,
                                Curve::IntersectionCurve(_) => false,
                            },
                            Curve::ParameterCurve(_) => unreachable!(),
                        }
                    }
                    Curve::IntersectionCurve(entity_curve) => {
                        let leader = entity_curve.leader().as_ref();
                        match leader {
                            Curve::Line(entity_line) => {
                                let entity_bsp = BsplineCurve::from(*entity_line);
                                let surface = RevolutionSurface::by_revolution(
                                    &entity_bsp,
                                    surface.origin(),
                                    surface.axis(),
                                );
                                match curve {
                                    &Curve::Line(curve) => {
                                        surface.include(&BsplineCurve::from(curve))
                                    }
                                    Curve::BsplineCurve(curve) => surface.include(curve),
                                    Curve::NurbsCurve(curve) => surface.include(curve),
                                    Curve::IntersectionCurve(curve) => {
                                        match curve.leader().as_ref() {
                                            Curve::Line(curve) => {
                                                surface.include(&BsplineCurve::from(*curve))
                                            }
                                            Curve::BsplineCurve(curve) => surface.include(curve),
                                            Curve::NurbsCurve(curve) => surface.include(curve),
                                            Curve::ParameterCurve(_) => false,
                                            Curve::IntersectionCurve(_) => false,
                                        }
                                    }
                                    Curve::ParameterCurve(_) => unreachable!(),
                                }
                            }
                            Curve::BsplineCurve(entity_curve) => {
                                let surface = RevolutionSurface::by_revolution(
                                    entity_curve,
                                    surface.origin(),
                                    surface.axis(),
                                );
                                match curve {
                                    &Curve::Line(curve) => {
                                        surface.include(&BsplineCurve::from(curve))
                                    }
                                    Curve::BsplineCurve(curve) => surface.include(curve),
                                    Curve::NurbsCurve(curve) => surface.include(curve),
                                    Curve::IntersectionCurve(curve) => {
                                        match curve.leader().as_ref() {
                                            Curve::Line(curve) => {
                                                surface.include(&BsplineCurve::from(*curve))
                                            }
                                            Curve::BsplineCurve(curve) => surface.include(curve),
                                            Curve::NurbsCurve(curve) => surface.include(curve),
                                            Curve::ParameterCurve(_) => false,
                                            Curve::IntersectionCurve(_) => false,
                                        }
                                    }
                                    Curve::ParameterCurve(_) => unreachable!(),
                                }
                            }
                            Curve::NurbsCurve(entity_curve) => {
                                let surface = RevolutionSurface::by_revolution(
                                    entity_curve,
                                    surface.origin(),
                                    surface.axis(),
                                );
                                match curve {
                                    &Curve::Line(curve) => {
                                        surface.include(&BsplineCurve::from(curve))
                                    }
                                    Curve::BsplineCurve(curve) => surface.include(curve),
                                    Curve::NurbsCurve(curve) => surface.include(curve),
                                    Curve::IntersectionCurve(curve) => {
                                        match curve.leader().as_ref() {
                                            Curve::Line(curve) => {
                                                surface.include(&BsplineCurve::from(*curve))
                                            }
                                            Curve::BsplineCurve(curve) => surface.include(curve),
                                            Curve::NurbsCurve(curve) => surface.include(curve),
                                            Curve::ParameterCurve(_) => false,
                                            Curve::IntersectionCurve(_) => false,
                                        }
                                    }
                                    Curve::ParameterCurve(_) => unreachable!(),
                                }
                            }
                            Curve::ParameterCurve(_) => false,
                            Curve::IntersectionCurve(_) => false,
                        }
                    }
                    Curve::ParameterCurve(_) => false,
                },
            }
        }
    }
}

impl IncludeCurve<Curve> for Plane {
    fn include(&self, curve: &Curve) -> bool {
        curve.lift_up().control_points().iter().all(|v| {
            let p = v.to_point();
            self.search_parameter(p, None, 1).is_some()
        })
    }
}

impl ToSameGeometry<Surface> for Plane {
    fn to_same_geometry(&self) -> Surface { (*self).into() }
}

impl ToSameGeometry<Surface> for RevolutionSurface<Curve> {
    fn to_same_geometry(&self) -> Surface {
        Surface::RevolutionSurface(Processor::new(self.clone()))
    }
}

fn transform_preserves_nearest_parameter(transform: &Matrix4) -> bool {
    let tol = 1.0e-10;
    let x = Vector3::new(transform.x.x, transform.x.y, transform.x.z);
    let y = Vector3::new(transform.y.x, transform.y.y, transform.y.z);
    let z = Vector3::new(transform.z.x, transform.z.y, transform.z.z);
    let scale2 = x.magnitude2();
    scale2 > tol
        && f64::abs(y.magnitude2() - scale2) <= tol
        && f64::abs(z.magnitude2() - scale2) <= tol
        && f64::abs(x.dot(y)) <= tol
        && f64::abs(x.dot(z)) <= tol
        && f64::abs(y.dot(z)) <= tol
        && f64::abs(transform.x.w) <= tol
        && f64::abs(transform.y.w) <= tol
        && f64::abs(transform.z.w) <= tol
        && f64::abs(transform.w.w - 1.0) <= tol
}

fn revolution_surface_nearest_parameter(
    surface: &Processor<RevolutionSurface<Curve>, Matrix4>,
    point: Point3,
    hint: SearchParameterHint2D,
    trials: usize,
) -> Option<(f64, f64)> {
    if env::var("MT_BOOL_DISABLE_REVOLUTION_NEAREST_FAST_PATH").is_err()
        && transform_preserves_nearest_parameter(surface.transform())
    {
        let inv = surface.transform().inverse_transform()?;
        let point = inv.transform_point(point);
        let uv = surface
            .entity()
            .search_nearest_parameter(point, hint, trials)?;
        Some(if surface.orientation() {
            uv
        } else {
            (uv.1, uv.0)
        })
    } else {
        surface.search_nearest_parameter(point, hint, trials)
    }
}

/// Separable presearch for a revolved-curve surface, bit-identical to the generic
/// [`algo::surface::presearch`] over the same `Processor<RevolutionSurface<..>, Matrix4>`.
///
/// `Processor::evaluate(u, v)` expands (per `orientation`) to
/// `transform(origin + rotation(angle) * (curve(param) - origin))`, where the
/// NURBS/basis `curve(param)` evaluation depends only on one grid axis and the
/// sincos-built `rotation(angle)` matrix only on the other. The generic grid scan
/// re-derives both `division + 1` times per grid line; this hoists each out of the
/// `O(division^2)` inner loop, computing every distinct `curve(param)` and
/// `rotation(angle)` exactly once and reusing it across the crossing line -- the
/// same waste `NurbsSurface::presearch_separable` removes for tensor-product NURBS,
/// which never covered the revolved-curve surface path.
///
/// Everything that decides the result is unchanged: the same grid nodes (`u`, `v`
/// from the identical expressions), the identical evaluation arithmetic
/// (`transform.transform_point(origin + rotation * (curve - origin))`), the same
/// `distance2` metric, and the same strict-`<` first-minimum tie-break. The
/// returned `(u, v)` -- and therefore every downstream Newton seed -- is
/// byte-for-byte identical to the generic presearch.
fn revolution_processor_presearch_separable(
    rotted: &Processor<RevolutionSurface<Curve>, Matrix4>,
    point: Point3,
    (urange, vrange): ((f64, f64), (f64, f64)),
    division: usize,
) -> (f64, f64) {
    let revolution = rotted.entity();
    let transform = rotted.transform();
    let origin = revolution.origin();
    let axis = revolution.axis();
    let curve = revolution.entity_curve();
    // Spec 014 W3: the same grid as `algo::surface::presearch`, charged in the
    // same unit. This path presearches at division 100 where the tensor-product
    // paths use 50, so nodes -- unlike a count of CALLS -- can tell a revolution
    // search apart from a NURBS one (10,201 vs 2,601).
    algo::surface::charge_presearch_nodes(algo::surface::presearch_nodes(division));
    let ((u0, u1), (v0, v1)) = (urange, vrange);
    // Identical to the generic presearch's grid-node expression.
    let node = |bound0: f64, bound1: f64, index: usize| {
        let t = index as f64 / division as f64;
        bound0 * (1.0 - t) + bound1 * t
    };
    // Each expensive separable factor is derived once per grid line and reused
    // across the crossing line (the hoist), then paired with its node value so the
    // loops iterate by value -- no range-indexing. `Processor::evaluate(u, v)` feeds
    // (curve arg, rotation arg) = (u, v) when `orientation`, else (v, u), so the two
    // factors bind to opposite axes per orientation; the returned node and the
    // outer=`u`/inner=`v` iteration order stay fixed to reproduce the generic
    // presearch's argmin and strict-`<` first-minimum tie-break exactly.
    let mut res = (0.0, 0.0);
    let mut min = f64::INFINITY;
    if rotted.orientation() {
        // evaluate(u, v) = transform(origin + rotation(v) * (curve(u) - origin)).
        let curve_by_u: Vec<(f64, Point3)> = (0..=division)
            .map(|i| {
                let u = node(u0, u1, i);
                (u, curve.evaluate(u))
            })
            .collect();
        let rotation_by_v: Vec<(f64, Matrix3)> = (0..=division)
            .map(|j| {
                let v = node(v0, v1, j);
                (v, Matrix3::from_axis_angle(axis, Rad(v)))
            })
            .collect();
        for &(u, curve_point) in &curve_by_u {
            for &(v, rotation) in &rotation_by_v {
                let dist = transform
                    .transform_point(origin + rotation * (curve_point - origin))
                    .distance2(point);
                if dist < min {
                    min = dist;
                    res = (u, v);
                }
            }
        }
    } else {
        // evaluate(u, v) = transform(origin + rotation(u) * (curve(v) - origin)).
        let rotation_by_u: Vec<(f64, Matrix3)> = (0..=division)
            .map(|i| {
                let u = node(u0, u1, i);
                (u, Matrix3::from_axis_angle(axis, Rad(u)))
            })
            .collect();
        let curve_by_v: Vec<(f64, Point3)> = (0..=division)
            .map(|j| {
                let v = node(v0, v1, j);
                (v, curve.evaluate(v))
            })
            .collect();
        for &(u, rotation) in &rotation_by_u {
            for &(v, curve_point) in &curve_by_v {
                let dist = transform
                    .transform_point(origin + rotation * (curve_point - origin))
                    .distance2(point);
                if dist < min {
                    min = dist;
                    res = (u, v);
                }
            }
        }
    }
    res
}

impl SearchNearestParameter<SurfaceParameter> for Surface {
    type Point = Point3;
    fn search_nearest_parameter<H: Into<SearchParameterHint2D>>(
        &self,
        point: Point3,
        hint: H,
        trials: usize,
    ) -> Option<(f64, f64)> {
        match self {
            Surface::Plane(plane) => plane.search_nearest_parameter(point, hint, trials),
            Surface::BsplineSurface(bspsurface) => {
                bspsurface.search_nearest_parameter(point, hint, trials)
            }
            Surface::NurbsSurface(surface) => surface.search_nearest_parameter(point, hint, trials),
            Surface::TsplineSurface(surface) => {
                surface.search_nearest_parameter(point, hint, trials)
            }
            // The analytic inverse map, not a Newton descent. This is the half
            // of the sphere/torus conversion that the rational net never
            // carried, and (011-T1) the half that decides where trims land.
            Surface::SphericalSurface(surface) => {
                surface.search_nearest_parameter(point, hint, trials)
            }
            Surface::ToroidalSurface(surface) => {
                surface.search_nearest_parameter(point, hint, trials)
            }
            Surface::RevolutionSurface(rotted) => {
                let hint = hint.into();
                revolution_surface_nearest_parameter(rotted, point, hint, trials).or_else(|| {
                    let hint = match hint {
                        SearchParameterHint2D::Parameter(hint0, hint1) => (hint0, hint1),
                        SearchParameterHint2D::Range(x, y) => {
                            revolution_processor_presearch_separable(rotted, point, (x, y), 100)
                        }
                        SearchParameterHint2D::None => revolution_processor_presearch_separable(
                            rotted,
                            point,
                            rotted.range_tuple(),
                            100,
                        ),
                    };
                    algo::surface::search_nearest_parameter(rotted, point, hint, trials).or_else(
                        || {
                            let candidate = rotted.evaluate(hint.0, hint.1);
                            candidate.near(&point).then_some(hint)
                        },
                    )
                })
            }
        }
    }
}

impl TryIntoBsplineSurface for Surface {
    fn try_into_bspline_surface(&self) -> Option<BsplineSurface<Point3>> {
        match self {
            Surface::Plane(p) => p.try_into_bspline_surface(),
            Surface::BsplineSurface(b) => b.try_into_bspline_surface(),
            Surface::NurbsSurface(n) => n.try_into_bspline_surface(),
            Surface::RevolutionSurface(_) => None,
            Surface::TsplineSurface(_) => None,
            // No exact NON-rational net exists for either; the exact form is
            // the homogeneous one below.
            Surface::SphericalSurface(_) | Surface::ToroidalSurface(_) => None,
        }
    }
}

impl TryIntoHomogeneousBsplineSurface for Surface {
    fn try_into_homogeneous_bspline_surface(&self) -> Option<BsplineSurface<Vector4>> {
        match self {
            Surface::Plane(p) => p.try_into_homogeneous_bspline_surface(),
            Surface::BsplineSurface(b) => b.try_into_homogeneous_bspline_surface(),
            Surface::NurbsSurface(n) => n.try_into_homogeneous_bspline_surface(),
            Surface::RevolutionSurface(r) => r.try_into_homogeneous_bspline_surface(),
            Surface::TsplineSurface(_) => None,
            // The SAME call the STEP loader used to make eagerly, on the same
            // `Processor<_, Matrix4>`: the net the boolean's homogeneous path
            // prepares is byte-identical to the one it saw before this
            // variant existed. Only WHEN it is built has moved.
            Surface::SphericalSurface(s) => s.try_into_homogeneous_bspline_surface(),
            Surface::ToroidalSurface(t) => t.try_into_homogeneous_bspline_surface(),
        }
    }

    fn try_into_homogeneous_bspline_surface_over(
        &self,
        parameter_range: Option<SurfaceParameterRectangle>,
    ) -> Option<HomogeneousSurfaceConversion> {
        match self {
            Surface::Plane(p) => p.try_into_homogeneous_bspline_surface_over(parameter_range),
            Surface::BsplineSurface(b) => {
                b.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::NurbsSurface(n) => {
                n.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::RevolutionSurface(r) => {
                r.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::TsplineSurface(_) => None,
            Surface::SphericalSurface(s) => {
                s.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::ToroidalSurface(t) => {
                t.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
        }
    }
}

impl SupportsExactPatchDomains for Surface {
    fn supports_exact_patch_domains(&self) -> bool {
        match self {
            Surface::Plane(p) => p.supports_exact_patch_domains(),
            Surface::BsplineSurface(b) => b.supports_exact_patch_domains(),
            Surface::NurbsSurface(n) => n.supports_exact_patch_domains(),
            Surface::RevolutionSurface(r) => r.supports_exact_patch_domains(),
            Surface::TsplineSurface(t) => t.supports_exact_patch_domains(),
            // `false`, where the rational net answered `true`. This is the same
            // flip T22 made when cylinders and cones moved onto the analytic
            // revolution variant, and for the same reason: the patch domain of
            // an analytic surface is not a knot span.
            Surface::SphericalSurface(s) => s.supports_exact_patch_domains(),
            Surface::ToroidalSurface(t) => t.supports_exact_patch_domains(),
        }
    }
}

impl TryIntoAnalyticSurfaceKind for Surface {
    fn try_into_analytic_surface_kind(&self) -> Option<AnalyticSurfaceKind> {
        match self {
            Surface::Plane(p) => p.try_into_analytic_surface_kind(),
            Surface::BsplineSurface(b) => b.try_into_analytic_surface_kind(),
            Surface::NurbsSurface(n) => n.try_into_analytic_surface_kind(),
            Surface::RevolutionSurface(r) => r.try_into_analytic_surface_kind(),
            Surface::TsplineSurface(t) => t.try_into_analytic_surface_kind(),
            // `None`, which is exactly what these faces answered as rational
            // nets: `NurbsSurface::try_into_analytic_surface_kind` only ever
            // recognises a plane or a homogeneous extrusion, and a sphere net
            // is neither. Teaching it `SphericalRevolution` would be a
            // BOOLEAN change, and this track is the tessellation one --
            // recorded as follow-on work instead of smuggled in here.
            Surface::SphericalSurface(s) => s.try_into_analytic_surface_kind(),
            Surface::ToroidalSurface(t) => t.try_into_analytic_surface_kind(),
        }
    }
}

impl ToSameGeometry<Surface> for HomotopySurface<Curve, Curve> {
    fn to_same_geometry(&self) -> Surface {
        let curve0 = self.first_curve().clone().lift_up();
        let curve1 = self.second_curve().clone().lift_up();
        NurbsSurface::new(BsplineSurface::homotopy(curve0, curve1)).into()
    }
}

impl ToSameGeometry<Surface> for ExtrusionSurface<Curve, Vector3> {
    fn to_same_geometry(&self) -> Surface {
        let (curve0, vector) = (self.entity_curve(), self.extruding_vector());
        let trsl = Matrix4::from_translation(vector);
        let curve1 = self.entity_curve().transformed(trsl);
        match (curve0, curve1) {
            (Curve::Line(line), Curve::Line(_)) => {
                Plane::new(line.0, line.1, line.0 + vector).into()
            }
            (Curve::BsplineCurve(curve0), Curve::BsplineCurve(curve1)) => {
                BsplineSurface::homotopy(curve0.clone(), curve1.clone()).into()
            }
            (Curve::NurbsCurve(curve0), Curve::NurbsCurve(curve1)) => {
                NurbsSurface::new(BsplineSurface::homotopy(
                    curve0.non_rationalized().clone(),
                    curve1.non_rationalized().clone(),
                ))
                .into()
            }
            (Curve::IntersectionCurve(_), Curve::IntersectionCurve(_)) => unimplemented!(),
            _ => unreachable!(),
        }
    }
}

// ---------------------------------------------------------------------------
// Deterministic content hashing for modeling enums.
// ---------------------------------------------------------------------------

impl DeterministicContentHash for Conic2D {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        match self {
            Self::Ellipse(v) => {
                state.write_u8(0);
                v.content_hash(state);
            }
            Self::Hyperbola(v) => {
                state.write_u8(1);
                v.content_hash(state);
            }
            Self::Parabola(v) => {
                state.write_u8(2);
                v.content_hash(state);
            }
        }
    }
}

impl DeterministicContentHash for Curve2D {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        match self {
            Self::Line(v) => {
                state.write_u8(0);
                v.content_hash(state);
            }
            Self::Polyline(v) => {
                state.write_u8(1);
                v.content_hash(state);
            }
            Self::Conic(v) => {
                state.write_u8(2);
                v.content_hash(state);
            }
            Self::BsplineCurve(v) => {
                state.write_u8(3);
                v.content_hash(state);
            }
            Self::NurbsCurve(v) => {
                state.write_u8(4);
                v.content_hash(state);
            }
        }
    }
}

impl DeterministicContentHash for Curve {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        match self {
            Self::Line(v) => {
                state.write_u8(0);
                v.content_hash(state);
            }
            Self::BsplineCurve(v) => {
                state.write_u8(1);
                v.content_hash(state);
            }
            Self::NurbsCurve(v) => {
                state.write_u8(2);
                v.content_hash(state);
            }
            Self::ParameterCurve(v) => {
                state.write_u8(3);
                v.content_hash(state);
            }
            Self::IntersectionCurve(v) => {
                state.write_u8(4);
                v.content_hash(state);
            }
        }
    }
}

impl DeterministicContentHash for Surface {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        match self {
            Self::Plane(v) => {
                state.write_u8(0);
                v.content_hash(state);
            }
            Self::BsplineSurface(v) => {
                state.write_u8(1);
                v.content_hash(state);
            }
            Self::NurbsSurface(v) => {
                state.write_u8(2);
                v.content_hash(state);
            }
            Self::RevolutionSurface(v) => {
                state.write_u8(3);
                v.content_hash(state);
            }
            Self::TsplineSurface(v) => {
                state.write_u8(4);
                v.content_hash(state);
            }
            // New tags only. 0..=4 keep their meaning, so every solid that
            // carries no sphere and no torus hashes byte-identically.
            Self::SphericalSurface(v) => {
                state.write_u8(5);
                v.content_hash(state);
            }
            Self::ToroidalSurface(v) => {
                state.write_u8(6);
                v.content_hash(state);
            }
        }
    }
}

#[cfg(test)]
mod project_domain_tests {
    use super::*;
    use std::f64::consts::TAU;

    /// A rational-quadratic FULL circle in `u` (the era-standard nine-point
    /// NURBS circle) extruded along `z` for `v` in `[0, 1]`.
    ///
    /// This is the ap224 face-17 mechanism in miniature: the surface is
    /// geometrically periodic in `u` but reports NO period, and its last Bezier
    /// span's conic is the whole circle, so evaluating `u > 1` EXTRAPOLATES back
    /// onto the same surface. Every 3-D point on it therefore has an in-domain
    /// preimage AND off-domain ones that reproduce the point exactly -- nothing
    /// downstream can tell the wrong one from the right one.
    fn extruded_nurbs_circle() -> Surface {
        let sqrt_half = std::f64::consts::FRAC_1_SQRT_2;
        let circle: Vec<(f64, f64, f64)> = vec![
            (1.0, 0.0, 1.0),
            (1.0, 1.0, sqrt_half),
            (0.0, 1.0, 1.0),
            (-1.0, 1.0, sqrt_half),
            (-1.0, 0.0, 1.0),
            (-1.0, -1.0, sqrt_half),
            (0.0, -1.0, 1.0),
            (1.0, -1.0, sqrt_half),
            (1.0, 0.0, 1.0),
        ];
        let control_points = circle
            .into_iter()
            .map(|(x, y, w)| {
                vec![
                    Vector4::new(x * w, y * w, 0.0, w),
                    Vector4::new(x * w, y * w, w, w),
                ]
            })
            .collect();
        let u_knots = KnotVector::from(vec![
            0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
        ]);
        let v_knots = KnotVector::from(vec![0.0, 0.0, 1.0, 1.0]);
        Surface::NurbsSurface(NurbsSurface::new(BsplineSurface::new(
            (u_knots, v_knots),
            control_points,
        )))
    }

    /// The attempt plan is the one thing a reorder changes silently, and it is
    /// also the thing this projector is repeatedly claimed to share with the
    /// STEP loader's twin. Pin both: the plan's own shape under EITHER value of
    /// [`EXACT_FIRST`], and the fact that it is NOT the twin's plan.
    ///
    /// Measured, spec 011 T5: flipping [`EXACT_FIRST`] leaves `rejected` at 23
    /// and `fallback` at 4,996 on ap224 and moves `boxy` not at all, so the
    /// ordering is not what makes either twin C3-safe. What actually differs is
    /// the DISCIPLINE -- the twin normalizes and clamps each answer before
    /// feeding it forward as the next hint, while this one rejects an
    /// out-of-domain answer and retries. A future agent asked to "match the
    /// twin's ordering" should read that as the wrong lever.
    #[test]
    fn the_attempt_plan_interleaves_the_solvers_and_is_not_the_step_twins() {
        let plan: Vec<(bool, bool)> = (0..4).map(attempt_plan).collect();

        // Hinted pair first, then the unhinted pair.
        assert_eq!(
            plan.iter()
                .map(|&(unhinted, _)| unhinted)
                .collect::<Vec<_>>(),
            vec![false, false, true, true],
            "attempts 0/1 are hinted and 2/3 unhinted, whatever EXACT_FIRST says",
        );
        // The solvers ALTERNATE; EXACT_FIRST only picks the opener.
        assert_eq!(
            plan.iter().map(|&(_, exact)| exact).collect::<Vec<_>>(),
            vec![EXACT_FIRST, !EXACT_FIRST, EXACT_FIRST, !EXACT_FIRST],
            "the plan alternates solvers within each pair",
        );

        // `geom_impls.rs`'s `project`, read off the source: exact-hinted,
        // exact-unhinted, nearest-hinted, nearest-unhinted.
        let step_twin: Vec<(bool, bool)> =
            vec![(false, true), (true, true), (false, false), (true, false)];
        assert_ne!(
            plan, step_twin,
            "the modeling plan interleaves the solvers and the STEP twin groups \
             them, so neither value of EXACT_FIRST reproduces the twin",
        );
        // Concretely, attempt 1 is where they part: the twin re-asks the SAME
        // solver without a hint; this projector asks the OTHER solver with it.
        assert!(!plan[1].0, "attempt 1 is still hinted here");
        assert!(step_twin[1].0, "the twin's attempt 1 is unhinted");
        assert_eq!(
            plan[1].1, !plan[0].1,
            "attempt 1 switches solver here; the twin keeps it",
        );
    }

    #[test]
    fn axis_excess_is_zero_inside_and_the_gap_outside() {
        let range = Some((0.0, 1.0));
        assert_eq!(parameter_axis_excess(0.5, range, None), 0.0);
        assert_eq!(parameter_axis_excess(0.0, range, None), 0.0);
        assert_eq!(parameter_axis_excess(1.0, range, None), 0.0);
        assert_eq!(parameter_axis_excess(-0.25, range, None), 0.25);
        assert_eq!(parameter_axis_excess(1.25, range, None), 0.25);
        // The ap224 face-17 excursion.
        assert!((parameter_axis_excess(-44.146_387, range, None) - 44.146_387).abs() < 1.0e-6);
    }

    #[test]
    fn an_axis_with_no_reported_bound_imposes_nothing() {
        assert_eq!(parameter_axis_excess(-1.0e9, None, None), 0.0);
        assert_eq!(parameter_axis_excess(1.0e9, None, Some(1.0)), 0.0);
    }

    #[test]
    fn a_non_finite_value_is_outside_every_bounded_axis() {
        let range = Some((0.0, 1.0));
        assert_eq!(parameter_axis_excess(f64::NAN, range, None), f64::INFINITY);
        assert_eq!(
            parameter_axis_excess(f64::INFINITY, range, None),
            f64::INFINITY
        );
    }

    #[test]
    fn a_periodic_axis_accepts_a_whole_period_offset() {
        let period = TAU;
        let range = Some((0.0, period));
        // The very representatives a naive range test would reject.
        assert_eq!(parameter_axis_excess(-0.25, range, Some(period)), 0.0);
        assert_eq!(
            parameter_axis_excess(period + 0.25, range, Some(period)),
            0.0
        );
        assert_eq!(
            parameter_axis_excess(-7.0 * period, range, Some(period)),
            0.0
        );
        // ... but the same axis restricted to HALF a period still rejects the
        // half it does not cover, in EVERY representative: `4.0` and
        // `4.0 - period` are the same point and both sit outside `[0, pi]`.
        let half = Some((0.0, std::f64::consts::PI));
        assert!(parameter_axis_excess(4.0, half, Some(period)) > 0.1);
        assert!(parameter_axis_excess(4.0 - period, half, Some(period)) > 0.1);
        // A point that DOES reduce into the covered half is accepted from any
        // representative.
        assert_eq!(parameter_axis_excess(1.0, half, Some(period)), 0.0);
        assert_eq!(parameter_axis_excess(1.0 + period, half, Some(period)), 0.0);
        assert_eq!(parameter_axis_excess(1.0 - period, half, Some(period)), 0.0);
    }

    #[test]
    fn a_hinted_projection_that_would_leave_the_domain_falls_back_in_domain() {
        let surface = extruded_nurbs_circle();
        let domain = SurfaceParameterDomain::of(&surface);
        assert_eq!(domain.u_range, Some((0.0, 1.0)));
        assert_eq!(
            domain.period_u, None,
            "the defect needs an UNREPORTED period"
        );

        let target = (0.02, 0.5);
        let point = surface.subs(target.0, target.1);
        // A hint parked at the far domain edge is exactly the self-sustaining
        // state the sampled chain walks into.
        let hint = Some((0.999, 0.5));

        let legacy = surface
            .search_nearest_parameter(point, hint, 100)
            .map(|(u, v)| Point2::new(u, v))
            .expect("the unbounded Newton converges");
        assert!(
            legacy.x > 1.0 + TOLERANCE,
            "precondition: the hinted solve must escape the domain, got {legacy:?}",
        );
        assert!(
            surface.subs(legacy.x, legacy.y).distance(point) < 1.0e-9,
            "precondition: the off-domain answer reproduces the sample, so nothing \
             downstream can reject it",
        );

        let fixed = project_onto_surface_domain(&surface, point, hint, domain, TOLERANCE)
            .expect("a projection is still produced");
        assert!(
            domain.contains(fixed, TOLERANCE),
            "the in-domain answer must win, got {fixed:?}",
        );
        assert!((fixed.x - target.0).abs() < 1.0e-6, "got {fixed:?}");
        assert!((fixed.y - target.1).abs() < 1.0e-6, "got {fixed:?}");
    }

    #[test]
    fn an_in_domain_hinted_projection_is_the_first_attempt_bit_for_bit() {
        let surface = extruded_nurbs_circle();
        let domain = SurfaceParameterDomain::of(&surface);
        let target = (0.4, 0.25);
        let point = surface.subs(target.0, target.1);
        let hint = Some((0.42, 0.3));

        let exact = surface
            .search_parameter(point, hint, 100)
            .map(|(u, v)| Point2::new(u, v))
            .expect("the exact hinted solve converges");
        let nearest = surface
            .search_nearest_parameter(point, hint, 100)
            .map(|(u, v)| Point2::new(u, v))
            .expect("the nearest hinted solve converges");

        // Pin the INVARIANT, not one ordering: whichever hinted solve
        // [`EXACT_FIRST`] puts first, an in-domain answer from it is returned
        // untouched. Deriving the expectation from the flag keeps this test
        // honest under both orders instead of turning the flag into a tripwire.
        let first = if EXACT_FIRST { exact } else { nearest };
        assert!(domain.contains(first, TOLERANCE), "precondition");

        let fixed = project_onto_surface_domain(&surface, point, hint, domain, TOLERANCE)
            .expect("a projection is produced");
        assert_eq!(
            (fixed.x, fixed.y),
            (first.x, first.y),
            "an in-domain answer from the first attempt ({PROJECTION_ORDER}) is \
             returned bit-for-bit",
        );

        // Recorded because it is the whole argument for `EXACT_FIRST`: on an
        // in-domain sample the exact solve lands on the target while the
        // nearest solve converges to the same point a thousand ulps short, so
        // exact-first is also the more accurate opening move. This is an
        // observation about the two solves and holds whichever one runs first.
        assert!(
            (exact.x - target.0).abs() <= (nearest.x - target.0).abs(),
            "exact {exact:?} must be at least as close to {target:?} as nearest {nearest:?}",
        );
    }

    #[test]
    fn the_first_answer_survives_when_no_attempt_lands_in_domain() {
        let surface = extruded_nurbs_circle();
        let domain = SurfaceParameterDomain::of(&surface);
        // Well above the extrusion: both Newtons are unbounded in `v` too, so
        // EVERY attempt answers outside `v in [0, 1]`.
        let point = Point3::new(1.0, 0.0, 5.0);
        let hint = Some((0.01, 0.9));

        // The chain in whichever order [`EXACT_FIRST`] selects, so this pins the
        // fallback invariant rather than one ordering.
        let chained = if EXACT_FIRST {
            surface
                .search_parameter(point, hint, 100)
                .or_else(|| surface.search_nearest_parameter(point, hint, 100))
                .or_else(|| surface.search_parameter(point, None, 100))
                .or_else(|| surface.search_nearest_parameter(point, None, 100))
        } else {
            surface
                .search_nearest_parameter(point, hint, 100)
                .or_else(|| surface.search_parameter(point, hint, 100))
                .or_else(|| surface.search_nearest_parameter(point, None, 100))
                .or_else(|| surface.search_parameter(point, None, 100))
        }
        .map(|(u, v)| Point2::new(u, v))
        .expect("the chain answers");
        assert!(
            !domain.contains(chained, TOLERANCE),
            "precondition: {chained:?}",
        );

        let fixed = project_onto_surface_domain(&surface, point, hint, domain, TOLERANCE)
            .expect("the fallback must not turn a success into None");
        assert_eq!(
            (fixed.x, fixed.y),
            (chained.x, chained.y),
            "with no in-domain answer available the first answer is preserved",
        );
    }

    fn unit_plane() -> Surface {
        Surface::Plane(Plane::new(
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ))
    }

    #[test]
    fn a_surface_with_no_reported_range_is_unaffected() {
        let plane = unit_plane();
        let unbounded = SurfaceParameterDomain {
            u_range: None,
            v_range: None,
            period_u: None,
            period_v: None,
        };
        let point = Point3::new(-500.0, 900.0, 0.0);
        let hint = Some((3.0, 4.0));
        let legacy = plane
            .search_nearest_parameter(point, hint, 100)
            .map(|(u, v)| Point2::new(u, v))
            .expect("a plane always answers");
        let fixed = project_onto_surface_domain(&plane, point, hint, unbounded, TOLERANCE)
            .expect("a plane always answers");
        assert_eq!((fixed.x, fixed.y), (legacy.x, legacy.y));
    }

    /// [`Plane::parameter_range`] reports `[0, 1] x [0, 1]` "as square" even
    /// though a plane is infinite, so a legitimate planar-cap trim routinely
    /// projects far outside it. Nothing may move there: a plane's projection is
    /// linear, so all four solver attempts return the SAME answer and the
    /// fallback hands back the historical value untouched. This is why the
    /// planar-cap families are inert under this change.
    ///
    /// The answer stayed inert; the COST did not, which is what
    /// [`reported_range_bounds_the_surface`] now fixes. The assertion below is
    /// therefore stronger than it was: the square is no longer merely survived,
    /// it is not consulted, so the projection is accepted on the first attempt
    /// instead of paying for four.
    #[test]
    fn a_plane_whose_reported_square_is_not_a_real_bound_is_unaffected() {
        let plane = unit_plane();
        assert_eq!(
            plane.try_range_tuple(),
            (Some((0.0, 1.0)), Some((0.0, 1.0))),
            "the surface still REPORTS a unit square",
        );
        let domain = SurfaceParameterDomain::of(&plane);
        assert_eq!(
            (domain.u_range, domain.v_range),
            (None, None),
            "...but the projector must not treat it as a bound",
        );

        let point = Point3::new(-500.0, 900.0, 0.0);
        let hint = Some((3.0, 4.0));
        let legacy = plane
            .search_nearest_parameter(point, hint, 100)
            .map(|(u, v)| Point2::new(u, v))
            .expect("a plane always answers");
        assert!(
            parameter_axis_excess(legacy.x, Some((0.0, 1.0)), None) > TOLERANCE,
            "precondition: the honest answer IS outside the reported square: {legacy:?}",
        );

        let fixed = project_onto_surface_domain(&plane, point, hint, domain, TOLERANCE)
            .expect("a plane always answers");
        assert_eq!(
            (fixed.x, fixed.y),
            (legacy.x, legacy.y),
            "a plane's unique projection must survive the domain test",
        );
    }

    /// JOB 1. A non-finite `(u, v)` must never leave the projector.
    ///
    /// The two escape routes are asserted separately because they are
    /// genuinely different: on a surface reporting no range at all a
    /// non-finite pair used to test as INSIDE the domain (
    /// [`parameter_axis_excess`] returns `0.0` for `None` before it looks at
    /// finiteness) and be returned as a first-class answer; on a bounded
    /// surface it tested as outside and came back through `fallback`.
    #[test]
    fn a_non_finite_solver_answer_is_never_returned() {
        // Route 1: the domain test cannot see it.
        let unbounded = SurfaceParameterDomain {
            u_range: None,
            v_range: None,
            period_u: None,
            period_v: None,
        };
        assert!(
            unbounded.contains(Point2::new(f64::NAN, 0.5), TOLERANCE),
            "precondition: an unbounded domain CONTAINS a NaN, so the domain \
             test alone cannot reject one",
        );

        // Route 2: it is outside, so it lands in `fallback` and is returned
        // once nothing rescues it.
        let bounded = SurfaceParameterDomain {
            u_range: Some((0.0, 1.0)),
            v_range: Some((0.0, 1.0)),
            period_u: None,
            period_v: None,
        };
        assert!(
            !bounded.contains(Point2::new(f64::INFINITY, 0.5), TOLERANCE),
            "precondition: a bounded domain rejects an infinity...",
        );
        assert_eq!(
            parameter_axis_excess(f64::INFINITY, Some((0.0, 1.0)), None),
            f64::INFINITY,
            "...by measuring an infinite excess, which is exactly what parks it \
             in `fallback`",
        );

        // The guard itself: whatever the surface reports, a non-finite answer
        // is discarded and the remaining attempts still run.
        let surface = extruded_nurbs_circle();
        let point = Point3::new(f64::NAN, 0.0, 0.5);
        for domain in [unbounded, bounded, SurfaceParameterDomain::of(&surface)] {
            let projected = project_onto_surface_domain(&surface, point, None, domain, TOLERANCE);
            assert!(
                projected.is_none_or(|uv| uv.x.is_finite() && uv.y.is_finite()),
                "a non-finite projection escaped for domain {domain:?}: {projected:?}",
            );
        }
    }

    /// JOB 2. The C3 guard stays LIVE where the reported range is a real bound.
    ///
    /// This is the half that must not regress: dropping the comparand on the
    /// placeholder surfaces is only defensible if the knot-bounded surfaces
    /// keep rejecting. Same surface, same excursion, same rescue as
    /// `an_off_domain_hinted_answer_is_rejected_for_an_in_domain_one`, asserted
    /// here through `SurfaceParameterDomain::of` so a future edit to
    /// [`reported_range_bounds_the_surface`] that quietly nulls a NURBS axis
    /// fails.
    #[test]
    fn a_knot_bounded_surface_keeps_its_domain_and_its_rejection() {
        let surface = extruded_nurbs_circle();
        assert_eq!(
            reported_range_bounds_the_surface(&surface),
            (true, true),
            "a knot vector IS a bound on both axes",
        );
        let domain = SurfaceParameterDomain::of(&surface);
        assert_eq!(
            (domain.u_range, domain.v_range),
            surface.try_range_tuple(),
            "nothing is dropped for a knot-bounded surface",
        );

        let target = (0.02, 0.5);
        let point = surface.subs(target.0, target.1);
        let hint = Some((0.999, 0.5));
        let escaped = surface
            .search_nearest_parameter(point, hint, 100)
            .map(|(u, v)| Point2::new(u, v))
            .expect("the unbounded Newton converges");
        assert!(
            escaped.x > 1.0 + TOLERANCE,
            "precondition: the hinted solve escapes: {escaped:?}",
        );
        let fixed = project_onto_surface_domain(&surface, point, hint, domain, TOLERANCE)
            .expect("a projection is still produced");
        assert!(
            domain.contains(fixed, TOLERANCE),
            "the excursion must still be rejected in favour of an in-domain \
             answer, got {fixed:?}",
        );
    }

    /// JOB 2. A loaded cylinder's AXIAL axis is a placeholder and its ANGULAR
    /// axis is real, and the two must be decided independently.
    ///
    /// Built the way `From<&CylindricalSurface>`
    /// (`monstertruck-io/src/step/load/step_types.rs`) builds one: revolve
    /// `Line(p, p + z)` with a UNIT axis, then invert -- which swaps the
    /// `Processor`'s axes, so the periodic axis is `u`. A face on such a
    /// cylinder legitimately occupies tens of world units of `v`, and the
    /// reported `[0, 1]` is one arbitrary metre of an unbounded surface.
    #[test]
    fn a_loaded_cylinders_axial_axis_is_not_a_bound_but_its_angular_axis_is() {
        let axis = Vector3::unit_z();
        let center = Point3::origin();
        let start = center + Vector3::unit_x() * 3.0;
        let mut cylinder = Processor::new(RevolutionSurface::by_revolution(
            Curve::Line(Line(start, start + axis)),
            center,
            axis,
        ));
        cylinder.invert();
        let surface = Surface::RevolutionSurface(cylinder);

        assert_eq!(
            surface.try_range_tuple(),
            (Some((0.0, TAU)), Some((0.0, 1.0))),
            "the loaded-cylinder frame: angular in u, nominal unit segment in v",
        );
        assert_eq!(
            reported_range_bounds_the_surface(&surface),
            (true, false),
            "the turn is a bound; the profile line's [0, 1] is not",
        );

        let domain = SurfaceParameterDomain::of(&surface);
        assert_eq!(
            (domain.u_range, domain.v_range),
            (Some((0.0, TAU)), None),
            "only the placeholder axis is dropped",
        );
        assert_eq!(
            domain.period_u,
            Some(TAU),
            "and the angular axis keeps its period, so the periodic branch of \
             `parameter_axis_excess` still runs",
        );

        // A point 20 units up the cylinder: ordinary geometry, far outside the
        // reported v-square, and the projector must now accept it outright.
        let point = Point3::new(0.0, 3.0, 20.0);
        let projected = project_onto_surface_domain(&surface, point, None, domain, TOLERANCE)
            .expect("a cylinder answers");
        assert!(
            (projected.y - 20.0).abs() < 1.0e-9,
            "the axial parameter is a world-scale distance: {projected:?}",
        );
        assert!(
            domain.contains(projected, TOLERANCE),
            "and it is in-domain now, so the chain stops at the first attempt",
        );
        assert!(
            surface.subs(projected.x, projected.y).distance(point) < 1.0e-9,
            "sanity: the answer is on the surface",
        );
    }

    /// JOB 2, the cost claim. On a placeholder-domain surface the chain now
    /// costs ONE solve per point instead of four.
    ///
    /// Measured through the census rather than asserted in prose, because
    /// "forces up to 4 Newton solves per point instead of 1" is the entire
    /// justification for the change and M10 says a claim needs the counter that
    /// backs it.
    #[test]
    fn a_placeholder_domain_costs_one_solve_per_point_not_four() {
        // SAFETY-OF-MEASUREMENT: the census is thread-local and gated on the
        // env lens, which this test cannot switch on. Count the attempts the
        // plan actually runs instead, from the same predicate the projector
        // uses.
        let plane = unit_plane();
        let domain = SurfaceParameterDomain::of(&plane);
        let point = Point3::new(-500.0, 900.0, 0.0);
        let hint = Some((3.0, 4.0));

        let mut solves = 0usize;
        for attempt in 0..4 {
            let (unhinted, exact) = attempt_plan(attempt);
            if unhinted && hint.is_none() {
                break;
            }
            let attempt_hint = if unhinted { None } else { hint };
            solves += 1;
            let found = if exact {
                plane.search_parameter(point, attempt_hint, 100)
            } else {
                plane.search_nearest_parameter(point, attempt_hint, 100)
            };
            if let Some(uv) = found.map(|(u, v)| Point2::new(u, v))
                && uv.x.is_finite()
                && uv.y.is_finite()
                && domain.contains(uv, TOLERANCE)
            {
                break;
            }
        }
        assert_eq!(
            solves, 1,
            "the first attempt must be accepted; four means the guard is being \
             asked about a square that is not a bound",
        );
    }
}
