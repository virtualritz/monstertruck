use newton::Jacobian;

use super::*;

/// Maximum recursion depth used by [`parameter_division`].
///
/// Bounds runtime when a surface fails to converge -- for example an
/// ill-conditioned offset or a singular evaluation -- so a tessellation
/// caller falls back to the partial division it has so far instead of
/// looping forever.
///
/// **This bounds DEPTH, not WORK, and the two are not the same quantity here.**
/// [`sub_parameter_division`] refines a TENSOR-PRODUCT grid: one stubborn cell
/// sets the flag for its whole `u` row and `v` column, so a level can double
/// both axes and the examined cell count is `O(4^depth)`. At depth 100 the
/// implied cell count is `4^100`; the machine grinds long before the counter
/// runs out (ledger M10 -- a guard that does not cover what is claimed for it).
/// [`MAX_PARAMETER_DIVISION_CELLS`] is the bound that actually binds.
const MAX_PARAMETER_DIVISION_RECURSION: usize = 100;

/// Maximum number of grid CELLS [`parameter_division`] may examine, summed over
/// every refinement level of one top-level call.
///
/// This is the quantity that bounds the work: each examined cell costs five
/// surface evaluations plus one hash, and the per-level cell count is
/// `(udiv.len() - 1) * (vdiv.len() - 1)`, which can quadruple from one level to
/// the next. A depth cap does not bound it (see
/// [`MAX_PARAMETER_DIVISION_RECURSION`]).
///
/// A level is admitted only if it fits ENTIRELY inside the remaining budget, so
/// the division returned is always a level-complete grid and never a half-refined
/// one -- the result stays a deterministic function of the surface, the range and
/// the tolerance, independent of machine, thread count and load.
///
/// **The headroom this value is set from (spec 012 U1.1, measured 2026-07-31 at
/// `tol = 1e-3`, the chord the certified-empty guard and every fixture boolean
/// row use).** Cells spent by the largest SINGLE division, measured by
/// `user_fixture_boolean_tests::division_budget_sweep_over_the_in_repo_fixtures`
/// and `corpus_boolean_rows.rs::u1_divergent_face_probe`:
///
/// | geometry | worst single division | converges? | of this cap |
/// |---|---|---|---|
/// | boxy `#26`, 80 faces | 0 | -- (all analytic) | 0% |
/// | io1 `#10`, 22 faces | 0 | -- (all analytic) | 0% |
/// | ap224 `#1727`, 48 faces | 5,621 | yes | 0.07% |
/// | **coffy `#219` face 24** | **5,193,917** (852x2715 grid) | **yes** | **61.9%** |
/// | ROTOR `#25387` face 4 | 1,847,121 | yes | 22.0% |
/// | ROTOR `#19264` face 4 | >4,194,304 | **NO** | clipped |
///
/// The cap sits above every division measured that CONVERGES -- coffy face 24 is
/// the binding one, and an earlier `1 << 22` clipped it, which is why the value
/// is not that -- and below the point where a non-converging division has stopped
/// making progress. ROTOR #19264 face 4 is the non-converging case: its spend
/// moves only 7% between `tol = 1e-2` and `tol = 1e-3` because it is not
/// approaching the tolerance at all, so no affordable cap would let it finish and
/// bounding it is the whole point.
///
/// Do NOT raise this without re-running that sweep. `parameter_division_with_budget`
/// exists so the sweep needs no environment variable and no configuration knob on
/// the production path.
///
/// **What a caller who cannot handle a refusal sees.** [`parameter_division`] and
/// the `ParameterDivision2D` trait method keep their signatures and return the
/// level-complete, coarser-than-`tol` division -- the same shape of result the
/// depth cap already produced, except that it is now reachable in bounded time
/// and readable afterwards through [`division_work`] and [`division_totals`].
/// That is deliberate: this trait is implemented by roughly twenty geometry types
/// and consumed by viewers, area and volume estimates and mesh export, none of
/// which has a refusal to return, and a panic there would be a worse failure than
/// a coarse mesh. A caller whose SOUNDNESS depends on the chord bound must use
/// [`try_parameter_division`] and get the typed refusal instead.
pub const MAX_PARAMETER_DIVISION_CELLS: u64 = 1 << 23;

/// What one [`parameter_division`] call cost, and whether it was cut short.
///
/// `cells` is the load-independent work unit: cells examined, five surface
/// evaluations each. `truncated` is set when the call hit
/// [`MAX_PARAMETER_DIVISION_CELLS`] and returned a COARSER division than the
/// requested tolerance asks for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DivisionWork {
    /// Grid cells examined, summed over every refinement level.
    pub cells: u64,
    /// Whether the cell budget was exhausted, so the returned division is
    /// coarser than `tol` requires.
    pub truncated: bool,
}

std::thread_local! {
    /// Work charged on this thread since the last [`take_division_work`].
    ///
    /// Accumulating (rather than resetting per call) is what makes nested
    /// divisions -- `Processor` delegating to its entity, a `Shell` triangulation
    /// walking many faces -- add up instead of overwriting each other. Each cell
    /// pays one thread-local read/write against five surface evaluations, so the
    /// meter is not measurable on the hot path.
    static DIVISION_WORK: std::cell::Cell<DivisionWork> =
        const { std::cell::Cell::new(DivisionWork { cells: 0, truncated: false }) };
}

/// Reads the work charged on this thread since the last [`take_division_work`],
/// without clearing it.
#[must_use]
pub fn division_work() -> DivisionWork { DIVISION_WORK.with(std::cell::Cell::get) }

/// Reads and clears the work charged on this thread.
///
/// Call it immediately BEFORE a division to zero the meter and immediately
/// AFTER to read that division's cost, including any nested ones.
pub fn take_division_work() -> DivisionWork { DIVISION_WORK.replace(DivisionWork::default()) }

/// Process-wide cells examined, across every thread.
///
/// The thread-local [`division_work`] cannot see a division that ran on a rayon
/// worker, which is where shell tessellation actually does its work; this
/// counter can. It is charged once per REFINEMENT LEVEL (~20 relaxed adds per
/// top-level division at the very most), not once per cell, so it is free on the
/// hot path.
static DIVISION_CELLS_TOTAL: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Process-wide count of divisions cut short by [`MAX_PARAMETER_DIVISION_CELLS`].
static DIVISION_TRUNCATIONS_TOTAL: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(0);

/// Process-wide HIGH-WATER MARK of cells spent by a SINGLE top-level division.
///
/// This is the quantity a headroom table needs: the cap applies per call, so the
/// question "does this cap clip anything that terminates today" is answered by
/// the largest single call, never by a total.
static DIVISION_CELLS_MAX: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Reads the process-wide `(cells, truncations)` counters.
#[must_use]
pub fn division_totals() -> (u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        DIVISION_CELLS_TOTAL.load(Relaxed),
        DIVISION_TRUNCATIONS_TOTAL.load(Relaxed),
    )
}

/// Zeroes the process-wide counters and returns what they held.
pub fn take_division_totals() -> (u64, u64) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        DIVISION_CELLS_TOTAL.swap(0, Relaxed),
        DIVISION_TRUNCATIONS_TOTAL.swap(0, Relaxed),
    )
}

/// Process-wide PRESEARCH GRID NODES scanned, across every thread -- spec 014
/// W3's second candidate work unit.
///
/// A `search_nearest_parameter` with no hint spends nearly all of its time in
/// [`presearch`], scanning a `(division + 1)^2` grid of surface evaluations
/// before Newton ever starts. A COUNT OF CALLS therefore hides a factor of ~4:
/// tensor-product surfaces presearch at `PRESEARCH_DIVISION` (51x51 = 2,601
/// nodes) while the revolution processor presearches at 100 (101x101 = 10,201).
/// This counter charges the nodes instead, so two calls over different surface
/// families are not counted as equal work.
///
/// Charged ONCE PER PRESEARCH -- one relaxed add per thousands of evaluations --
/// never inside the grid loop, which is the loop being measured.
static PRESEARCH_NODES_TOTAL: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Charges `nodes` presearch grid nodes. Public because the hand-rolled
/// separable presearches (`NurbsSurface::presearch_separable`, the revolution
/// processor's) bypass [`presearch`] and must charge the same unit, or the
/// counter would read differently depending on which fast path a surface takes.
#[inline]
pub fn charge_presearch_nodes(nodes: u64) {
    PRESEARCH_NODES_TOTAL.fetch_add(nodes, std::sync::atomic::Ordering::Relaxed);
}

/// Grid nodes for a `(division + 1) x (division + 1)` presearch scan.
#[inline]
#[must_use]
pub fn presearch_nodes(division: usize) -> u64 {
    let side = division as u64 + 1;
    side.saturating_mul(side)
}

/// Reads the process-wide presearch-node counter.
#[must_use]
pub fn presearch_nodes_total() -> u64 {
    PRESEARCH_NODES_TOTAL.load(std::sync::atomic::Ordering::Relaxed)
}

/// Reads the process-wide high-water mark of cells spent by one division.
#[must_use]
pub fn division_max_cells() -> u64 { DIVISION_CELLS_MAX.load(std::sync::atomic::Ordering::Relaxed) }

/// Zeroes the high-water mark and returns what it held.
pub fn take_division_max_cells() -> u64 {
    DIVISION_CELLS_MAX.swap(0, std::sync::atomic::Ordering::Relaxed)
}

/// A [`parameter_division`] that ran out of cell budget before it met `tol`.
///
/// Carries the division actually produced, so a caller that can use a coarser
/// grid may still take it -- deliberately, and knowing that it is coarser --
/// rather than being handed one silently.
#[derive(Clone, Debug, PartialEq)]
pub struct DivisionTruncated {
    /// The level-complete but too-coarse division reached before the budget ran
    /// out.
    pub division: (Vec<f64>, Vec<f64>),
    /// Cells examined before the budget ran out.
    pub cells: u64,
    /// The budget that was exhausted.
    pub budget: u64,
}

impl std::fmt::Display for DivisionTruncated {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "surface parameter division exhausted its cell budget: {} of {} cells examined, \
             division stopped at {}x{} and is coarser than the requested tolerance",
            self.cells,
            self.budget,
            self.division.0.len(),
            self.division.1.len(),
        )
    }
}

impl std::error::Error for DivisionTruncated {}

/// Divides the domain into equal parts, examines all the values, and returns `(u, v)` such that
/// `surface.evaluate(u, v)` is closest to `point`.
/// This method is useful to get an efficient hint of `search_nearest_parameter`.
pub fn presearch<S>(
    surface: &S,
    point: S::Point,
    (urange, vrange): SurfaceParameterRange,
    division: usize,
) -> (f64, f64)
where
    S: ParametricSurface,
    S::Point: MetricSpace<Metric = f64> + Copy,
{
    // Spec 014 W3: charged once, before the scan, so the unit is a deterministic
    // function of the grid rather than of how far the scan got.
    charge_presearch_nodes(presearch_nodes(division));
    let mut res = (0.0, 0.0);
    let mut min = f64::INFINITY;
    let ((u0, u1), (v0, v1)) = (urange, vrange);
    for i in 0..=division {
        for j in 0..=division {
            let p = i as f64 / division as f64;
            let q = j as f64 / division as f64;
            let u = u0 * (1.0 - p) + u1 * p;
            let v = v0 * (1.0 - q) + v1 * q;
            let dist = surface.evaluate(u, v).distance2(point);
            if dist < min {
                min = dist;
                res = (u, v);
            }
        }
    }
    res
}

/// Vectors whose points returned by the surface that can be the target of [`search_nearest_parameter`].
pub trait SearchNearestParameterVector: InnerSpace<Scalar = f64> + Tolerance {
    #[doc(hidden)]
    type Point;
    #[doc(hidden)]
    type Matrix: Jacobian<Self>;
    #[doc(hidden)]
    fn subs<S>(surface: &S, point: Self::Point, param: Self) -> CalcOutput<Self, Self::Matrix>
    where S: ParametricSurface<Point = Self::Point, Vector = Self>;
    #[doc(hidden)]
    fn into_param(self) -> (f64, f64);
    #[doc(hidden)]
    fn from_param(param: (f64, f64)) -> Self;
}

impl SearchNearestParameterVector for Vector2 {
    type Point = Point2;
    type Matrix = Matrix2;
    fn subs<S>(
        surface: &S,
        point: Point2,
        Vector2 { x: u, y: v }: Vector2,
    ) -> CalcOutput<Self, Matrix2>
    where
        S: ParametricSurface<Point = Point2, Vector = Vector2>,
    {
        CalcOutput {
            value: surface.evaluate(u, v) - point,
            derivation: Matrix2::from_cols(surface.derivative_u(u, v), surface.derivative_v(u, v)),
        }
    }
    fn into_param(self) -> (f64, f64) { self.into() }
    fn from_param(param: (f64, f64)) -> Self { param.into() }
}

impl SearchNearestParameterVector for Vector3 {
    type Point = Point3;
    type Matrix = Matrix3;
    fn subs<S>(
        surface: &S,
        point: Self::Point,
        Vector3 { x: u, y: v, z: w }: Vector3,
    ) -> CalcOutput<Self, Self::Matrix>
    where
        S: ParametricSurface<Point = Self::Point, Vector = Self>,
    {
        let diff = surface.evaluate(u, v) - point;
        let uder = surface.derivative_u(u, v);
        let vder = surface.derivative_v(u, v);
        let uuder = surface.derivative_uu(u, v);
        let uvder = surface.derivative_uv(u, v);
        let vvder = surface.derivative_vv(u, v);
        let uv_cross = uder.cross(vder);
        CalcOutput {
            value: diff + uv_cross * w,
            derivation: Matrix3::from_cols(
                uder + (uuder.cross(vder) + uder.cross(uvder)) * w,
                vder + (uvder.cross(vder) + uder.cross(vvder)) * w,
                uv_cross,
            ),
        }
    }
    fn into_param(self) -> (f64, f64) { self.truncate().into() }
    fn from_param((u, v): (f64, f64)) -> Self { Self::new(u, v, 0.0) }
}

/// Searches the parameter by Newton's method.
#[inline(always)]
pub fn search_nearest_parameter<P, S>(
    surface: &S,
    point: P,
    hint: (f64, f64),
    trials: usize,
) -> Option<(f64, f64)>
where
    P: EuclideanSpace<Scalar = f64> + MetricSpace<Metric = f64>,
    P::Diff: SearchNearestParameterVector<Point = P>,
    S: ParametricSurface<Point = P, Vector = P::Diff>,
{
    let function = move |param: P::Diff| SearchNearestParameterVector::subs(surface, point, param);
    let res = newton::solve(function, P::Diff::from_param(hint), trials);
    res.ok().map(P::Diff::into_param)
}

/// Vectors whose points returned by the surface that can be the target of [`search_parameter`].
pub trait SearchParameterVector: InnerSpace<Scalar = f64> + Tolerance {
    #[doc(hidden)]
    type Point;
    #[doc(hidden)]
    fn subs<S>(surface: &S, point: Self::Point, param: Vector2) -> CalcOutput<Vector2, Matrix2>
    where S: ParametricSurface<Point = Self::Point, Vector = Self>;
}

impl SearchParameterVector for Vector2 {
    type Point = Point2;
    fn subs<S>(
        surface: &S,
        point: Point2,
        Vector2 { x: u, y: v }: Vector2,
    ) -> CalcOutput<Vector2, Matrix2>
    where
        S: ParametricSurface<Point = Point2, Vector = Vector2>,
    {
        CalcOutput {
            value: surface.evaluate(u, v) - point,
            derivation: Matrix2::from_cols(surface.derivative_u(u, v), surface.derivative_v(u, v)),
        }
    }
}

impl SearchParameterVector for Vector3 {
    type Point = Point3;
    fn subs<S>(
        surface: &S,
        point: Self::Point,
        Vector2 { x: u, y: v }: Vector2,
    ) -> CalcOutput<Vector2, Matrix2>
    where
        S: ParametricSurface<Point = Self::Point, Vector = Self>,
    {
        let diff = surface.evaluate(u, v) - point;
        let uder = surface.derivative_u(u, v);
        let vder = surface.derivative_v(u, v);
        CalcOutput {
            value: Vector2::new(uder.dot(diff), vder.dot(diff)),
            derivation: Matrix2::new(
                uder.dot(uder),
                uder.dot(vder),
                uder.dot(vder),
                vder.dot(vder),
            ),
        }
    }
}

/// Searches the parameter by Newton's method.
#[inline(always)]
pub fn search_parameter<P, S>(
    surface: &S,
    point: P,
    hint: (f64, f64),
    trials: usize,
) -> Option<(f64, f64)>
where
    P: EuclideanSpace<Scalar = f64> + MetricSpace<Metric = f64> + Tolerance,
    P::Diff: SearchParameterVector<Point = P>,
    S: ParametricSurface<Point = P, Vector = P::Diff>,
{
    let function = move |param: Vector2| SearchParameterVector::subs(surface, point, param);
    let res = newton::solve(function, hint.into(), trials);
    res.ok().and_then(
        |Vector2 { x: u, y: v }| match surface.evaluate(u, v).near(&point) {
            true => Some((u, v)),
            false => None,
        },
    )
}

/// Searches the parameters of the intersection point of `surface` and `curve`.
pub fn search_intersection_parameter<C, S>(
    surface: &S,
    hint0: (f64, f64),
    curve: &C,
    hint1: f64,
    trials: usize,
) -> Option<((f64, f64), f64)>
where
    C: ParametricCurve3D,
    S: ParametricSurface3D,
{
    let function = move |Vector3 { x, y, z }| CalcOutput {
        value: surface.evaluate(x, y) - curve.evaluate(z),
        derivation: Matrix3::from_cols(
            surface.derivative_u(x, y),
            surface.derivative_v(x, y),
            -curve.derivative(z),
        ),
    };
    let hint = Vector3::new(hint0.0, hint0.1, hint1);
    let Vector3 { x, y, z } = newton::solve(function, hint, trials).ok()?;
    match surface.evaluate(x, y).near(&curve.evaluate(z)) {
        true => Some(((x, y), z)),
        false => None,
    }
}

/// Creates the surface division
///
/// # Panics
///
/// `tol` must be more than `TOLERANCE`.
#[inline(always)]
pub fn parameter_division<S>(
    surface: &S,
    (urange, vrange): SurfaceParameterRange,
    tol: f64,
) -> (Vec<f64>, Vec<f64>)
where
    S: ParametricSurface,
    S::Point: EuclideanSpace<Scalar = f64> + MetricSpace<Metric = f64> + HashGen<f64>,
{
    parameter_division_with_budget(surface, (urange, vrange), tol, MAX_PARAMETER_DIVISION_CELLS)
        .division
}

/// The remaining cell budget of one top-level division, and whether it ran out.
struct CellBudget {
    remaining: u64,
    exhausted: bool,
}

/// One division and what it cost, as returned by [`parameter_division_with_budget`].
#[derive(Clone, Debug, PartialEq)]
pub struct BudgetedDivision {
    /// The `(udiv, vdiv)` division. Level-complete either way, and coarser than
    /// `tol` asks for exactly when `truncated`.
    pub division: (Vec<f64>, Vec<f64>),
    /// Cells examined, summed over every refinement level.
    pub cells: u64,
    /// Whether the budget ran out before the tolerance was met.
    pub truncated: bool,
}

/// [`parameter_division`] against an explicit cell budget.
///
/// The two production entry points fix the budget at
/// [`MAX_PARAMETER_DIVISION_CELLS`]; this one is what a HEADROOM STUDY calls, so
/// the ceiling can be swept without an environment variable or any other hidden
/// configuration knob on the production path.
///
/// # Panics
///
/// `tol` must be more than `TOLERANCE`.
pub fn parameter_division_with_budget<S>(
    surface: &S,
    (urange, vrange): SurfaceParameterRange,
    tol: f64,
    cell_budget: u64,
) -> BudgetedDivision
where
    S: ParametricSurface,
    S::Point: EuclideanSpace<Scalar = f64> + MetricSpace<Metric = f64> + HashGen<f64>,
{
    nonpositive_tolerance!(tol);
    let (mut udiv, mut vdiv) = (vec![urange.0, urange.1], vec![vrange.0, vrange.1]);
    let mut budget = CellBudget {
        remaining: cell_budget,
        exhausted: false,
    };
    sub_parameter_division(
        surface,
        (&mut udiv, &mut vdiv),
        tol,
        MAX_PARAMETER_DIVISION_RECURSION,
        &mut budget,
    );
    let cells = cell_budget - budget.remaining;
    DIVISION_CELLS_MAX.fetch_max(cells, std::sync::atomic::Ordering::Relaxed);
    BudgetedDivision {
        division: (udiv, vdiv),
        cells,
        truncated: budget.exhausted,
    }
}

/// [`parameter_division`], but a division that could not be refined to `tol`
/// within [`MAX_PARAMETER_DIVISION_CELLS`] is an `Err` naming the stage instead
/// of a silently-coarse `Ok`.
///
/// Use this wherever a coarse division would be UNSOUND rather than merely ugly
/// -- a certified-empty guard sampling a curved operand, say, whose soundness
/// argument rests on the mesh staying within the chord of the true surface. The
/// infallible [`parameter_division`] keeps its signature and its behaviour for
/// the many callers -- viewers, area and volume estimates -- that have no
/// refusal to return and are better served by a coarse grid than by a panic; it
/// leaves the truncation readable through [`division_work`].
///
/// # Panics
///
/// `tol` must be more than `TOLERANCE`.
pub fn try_parameter_division<S>(
    surface: &S,
    (urange, vrange): SurfaceParameterRange,
    tol: f64,
) -> Result<(Vec<f64>, Vec<f64>), DivisionTruncated>
where
    S: ParametricSurface,
    S::Point: EuclideanSpace<Scalar = f64> + MetricSpace<Metric = f64> + HashGen<f64>,
{
    let budgeted = parameter_division_with_budget(
        surface,
        (urange, vrange),
        tol,
        MAX_PARAMETER_DIVISION_CELLS,
    );
    match budgeted.truncated {
        false => Ok(budgeted.division),
        true => Err(DivisionTruncated {
            division: budgeted.division,
            cells: budgeted.cells,
            budget: MAX_PARAMETER_DIVISION_CELLS,
        }),
    }
}

fn sub_parameter_division<S>(
    surface: &S,
    (udiv, vdiv): (&mut Vec<f64>, &mut Vec<f64>),
    tol: f64,
    remaining_trials: usize,
    budget: &mut CellBudget,
) where
    S: ParametricSurface,
    S::Point: EuclideanSpace<Scalar = f64> + MetricSpace<Metric = f64> + HashGen<f64>,
{
    if remaining_trials == 0 {
        // Reaching the DEPTH cap is a truncation too: the caller is handed a
        // division coarser than `tol` asks for. It used to return silently, so a
        // caller could not tell an accurate division from an abandoned one. It is
        // hard to reach now -- the cell budget binds first for any grid that
        // grows -- but a grid that adds a single interval per level walks all the
        // way here on very little work, and that case has to be honest as well.
        budget.exhausted = true;
        DIVISION_WORK.with(|work| {
            let mut current = work.get();
            current.truncated = true;
            work.set(current);
        });
        DIVISION_TRUNCATIONS_TOTAL.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        return;
    }
    // Admit a refinement level only if it fits ENTIRELY in the remaining budget.
    // All-or-nothing per level keeps the returned grid level-complete, so the
    // result is a deterministic function of surface, range and tolerance rather
    // than of where a mid-level abort happened to land.
    let level_cells = (udiv.len() - 1) as u64 * (vdiv.len() - 1) as u64;
    if level_cells > budget.remaining {
        budget.exhausted = true;
        DIVISION_WORK.with(|work| {
            let mut current = work.get();
            current.truncated = true;
            work.set(current);
        });
        DIVISION_TRUNCATIONS_TOTAL.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        return;
    }
    budget.remaining -= level_cells;
    DIVISION_WORK.with(|work| {
        let mut current = work.get();
        current.cells += level_cells;
        work.set(current);
    });
    DIVISION_CELLS_TOTAL.fetch_add(level_cells, std::sync::atomic::Ordering::Relaxed);
    let mut divide_flag0 = vec![false; udiv.len() - 1];
    let mut divide_flag1 = vec![false; vdiv.len() - 1];

    for (u, ub) in udiv.windows(2).zip(&mut divide_flag0) {
        for (v, vb) in vdiv.windows(2).zip(&mut divide_flag1) {
            if *ub && *vb {
                continue;
            }
            let (u_gen, v_gen) = ((u[0] + u[1]) / 2.0, (v[0] + v[1]) / 2.0);
            // Independent hash channels for `p` and `q`: `hash1` twice would
            // return the same value, biasing the in-cell sample along the
            // diagonal `p == q`.
            let [p_jitter, q_jitter] = HashGen::hash2(surface.evaluate(u_gen, v_gen));
            let p = 0.5 + (0.2 * p_jitter - 0.1);
            let q = 0.5 + (0.2 * q_jitter - 0.1);
            let u0 = u[0] * (1.0 - p) + u[1] * p;
            let v0 = v[0] * (1.0 - q) + v[1] * q;
            let p0 = surface.evaluate(u0, v0);
            let pt00 = surface.evaluate(u[0], v[0]);
            let pt01 = surface.evaluate(u[0], v[1]);
            let pt10 = surface.evaluate(u[1], v[0]);
            let pt11 = surface.evaluate(u[1], v[1]);
            let pt = S::Point::from_vec(
                pt00.to_vec() * (1.0 - p) * (1.0 - q)
                    + pt01.to_vec() * (1.0 - p) * q
                    + pt10.to_vec() * p * (1.0 - q)
                    + pt11.to_vec() * p * q,
            );
            if p0.distance2(pt) > tol * tol {
                let delu = pt00.midpoint(pt01).distance(p0) + pt10.midpoint(pt11).distance(p0);
                let delv = pt00.midpoint(pt10).distance(p0) + pt01.midpoint(pt11).distance(p0);
                if delu > delv * 2.0 {
                    *ub = true;
                } else if delv > delu * 2.0 {
                    *vb = true;
                } else {
                    (*ub, *vb) = (true, true);
                }
            }
        }
    }

    let mut new_udiv = vec![udiv[0]];
    for (u, ub) in udiv.windows(2).zip(divide_flag0) {
        if ub {
            new_udiv.push((u[0] + u[1]) / 2.0);
        }
        new_udiv.push(u[1]);
    }

    let mut new_vdiv = vec![vdiv[0]];
    for (v, vb) in vdiv.windows(2).zip(divide_flag1) {
        if vb {
            new_vdiv.push((v[0] + v[1]) / 2.0);
        }
        new_vdiv.push(v[1]);
    }

    if udiv.len() != new_udiv.len() || vdiv.len() != new_vdiv.len() {
        *udiv = new_udiv;
        *vdiv = new_vdiv;
        sub_parameter_division(surface, (udiv, vdiv), tol, remaining_trials - 1, budget);
    }
}

#[cfg(test)]
mod division_budget_tests;
