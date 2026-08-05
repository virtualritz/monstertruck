//! T3 -- the C1 EXTENT oracle for `SURFACE_OF_LINEAR_EXTRUSION` and
//! `SURFACE_OF_REVOLUTION`, plus the modeling-conversion fate the Phase 0
//! census could not see.
//!
//! Spec 011 Phase 0 measured that all 3,341 extrusions and all 232 revolutions
//! in `Ai-14R.stp` "convert". Two things hide behind that word and this file
//! measures both:
//!
//! 1. **Which conversion.** Phase 0 measured `step_geometry::Surface::try_from(
//!    &SurfaceAny)` -- the STEP-side enum, which for a swept surface is a
//!    near-tautology. The conversion C1 is about is the NEXT one,
//!    `monstertruck_modeling::Surface::try_from(&step_geometry::Surface)`, and
//!    nobody had run it on these faces.
//! 2. **Extent.** Even a successful conversion may be a wrongly-BOUNDED net --
//!    C1's whole point -- and no success/failure count can see that.
//!
//! ## How the required extent is derived -- no trim, no projector
//!
//! Per C1's recurrence guard and the 7cc/7y technique: the required extent comes
//! from the face's boundary EDGE CURVES, sampled in model space. STEP's
//! `EDGE_CURVE` is already trimmed to its two vertices by
//! `EdgeCurve::parse_curve3d`, so sampling it over its own parameter range gives
//! exactly the 3D point set the surface must contain. Nothing here calls
//! `search_parameter`, `to_parameter_curve_on`, or any pcurve: the oracle must
//! not be able to fail for a projector reason.
//!
//! Each boundary sample is then placed in the ANALYTIC surface's own parameter
//! frame by a bracketed 1D search over the profile curve (closed form in the
//! other axis), and the resulting required rectangle is compared with the
//! rectangle the emitted net spans:
//!
//! | class | emitted PROFILE axis | emitted SWEEP axis |
//! |---|---|---|
//! | `SURFACE_OF_LINEAR_EXTRUSION` | profile curve's own knot span | `[0, 1]`, i.e. ONE `extrusion_axis` vector |
//! | `SURFACE_OF_REVOLUTION` | profile curve's own knot span | a FULL turn |
//!
//! Note the deliberate naming: PROFILE and SWEEP, never `u` and `v`. The two
//! classes disagree about which surface parameter is which -- a STEP revolution
//! is an INVERTED `Processor`, so its `u` is the angle and its `v` is the profile
//! -- and calling both axes `u`/`v` in one measurement is exactly C2.
//!
//! Both spans are read off the conversion rather than assumed, and
//! `emitted_extrusion_net_matches_the_analytic_surface` pins the reading by
//! measuring one against the other.
//!
//! ## Reading the numbers
//!
//! `residual` is the distance from a boundary sample to the analytic surface
//! EXTENDED past its emitted bounds. A small residual means the sample really is
//! on the analytic surface and the (u, v) recovered for it is meaningful; a large
//! one means the reconstruction did not find the point, and those faces are
//! reported separately rather than folded into the truncation rate.
//!
//! `shortfall` is in MODEL UNITS, not parameter units, so it is comparable
//! across faces: for an extrusion a sweep excess of `e` is `e * |extrusion_axis|`
//! millimetres of surface the emitted net does not have.
//!
//! Corpus-gated and `#[ignore]`d. Run with:
//!
//! ```text
//! flock -o /tmp/mt-cargo.lock cargo nextest run -p monstertruck-step \
//!     --run-ignored all -E 'test(swept_surface_extent)' --no-capture
//! ```

use monstertruck_io::step::load::{
    EdgeCurveHolder, Table,
    step_geometry::{
        Curve3D, StepExtrusionSurface, StepRevolutionSurface, Surface, SweepSurface,
        re_exports::{
            EuclideanSpace, InnerSpace, ParametricCurve, ParametricSurface, ParametricSurface3D,
            Point3, Vector3,
        },
    },
};
use monstertruck_modeling::Surface as ModelingSurface;
use std::cell::{Cell, RefCell};
use std::collections::{BTreeMap, HashMap};
use std::panic::AssertUnwindSafe;
use std::path::{Path, PathBuf};
use step_p21::{
    ast::Name,
    tables::{EntityTable, IntoOwned, PlaceHolder},
};

// ------------------------------------------------------- panic containment
//
// Same containment the Phase 0 census uses, and for the same measured reason:
// conversion can UNWIND rather than refuse, and a harness that aborts on the
// first unwind can only ever report the first one.

thread_local! {
    static CATCHING: Cell<bool> = const { Cell::new(false) };
    static LAST_PANIC: RefCell<Option<String>> = const { RefCell::new(None) };
}

fn install_oracle_panic_hook() {
    std::panic::set_hook(Box::new(|info| {
        if CATCHING.get() {
            let payload = info
                .payload()
                .downcast_ref::<&str>()
                .map(|s| (*s).to_owned())
                .or_else(|| info.payload().downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "<non-string panic payload>".to_owned());
            let at = info
                .location()
                .map(|l| format!(" at {}:{}", l.file(), l.line()))
                .unwrap_or_default();
            LAST_PANIC.with(|slot| *slot.borrow_mut() = Some(format!("{payload}{at}")));
        } else {
            eprintln!("{info}");
        }
    }));
}

fn catching<T>(f: impl FnOnce() -> T) -> Result<T, String> {
    CATCHING.set(true);
    let caught = std::panic::catch_unwind(AssertUnwindSafe(f));
    CATCHING.set(false);
    caught.map_err(|_| {
        LAST_PANIC
            .with(|slot| slot.borrow_mut().take())
            .unwrap_or_else(|| "<panic>".to_owned())
    })
}

/// Collapse `#123` to `#<id>` so N identical refusals group into one row.
fn without_ids(message: &str) -> String {
    let mut out = String::with_capacity(message.len());
    let mut chars = message.chars().peekable();
    while let Some(c) = chars.next() {
        out.push(c);
        if c == '#' && chars.peek().is_some_and(char::is_ascii_digit) {
            while chars.peek().is_some_and(char::is_ascii_digit) {
                chars.next();
            }
            out.push_str("<id>");
        }
    }
    out
}

// ---------------------------------------------------------------- discovery

/// Same discovery contract as `corpus_load.rs` / `corpus_conversion_census.rs`.
fn corpus_root() -> Option<PathBuf> {
    std::env::var_os("MONSTERTRUCK_STEP_CORPUS")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|h| Path::new(&h).join("code/step-corpus/bigassy"))
        })
        .filter(|p| p.is_dir())
}

/// Every corpus file with a swept surface in it. Measured 2026-07-30: the swept
/// classes live ONLY in `Ai-14R.stp` (Phase 0 per-class table), so the list is
/// one entry -- but the loop is over a list so a later file joins it by name.
const SWEPT_CORPUS_FILES: [&str; 1] = ["Ai-14R.stp"];

// ------------------------------------------------------- boundary sampling

/// Samples per boundary edge, endpoints included. The required extent is a
/// min/max over samples, so this only has to be dense enough not to miss the
/// extreme of a curved edge.
const SAMPLES_PER_EDGE: usize = 33;

/// Model-space points on this face's boundary, taken from the edge curves alone.
///
/// Walks `FACE_SURFACE.bounds -> FACE_BOUND.bound -> EDGE_LOOP.edge_list ->
/// ORIENTED_EDGE.edge_element -> EDGE_CURVE`, the same chain `shell_edges` walks,
/// but reading the public table maps directly so no shell has to be assembled.
fn boundary_samples(
    table: &Table,
    face: &monstertruck_io::step::load::FaceSurfaceHolder,
) -> Vec<Point3> {
    let mut points = Vec::new();
    for bound in &face.bounds {
        let Some(bound) = resolve(&table.face_bound, bound) else {
            continue;
        };
        let Some(edge_loop) = resolve(&table.edge_loop, &bound.bound) else {
            continue;
        };
        for edge in &edge_loop.edge_list {
            let PlaceHolder::Ref(Name::Entity(id)) = edge else {
                continue;
            };
            let edge_curve_id = table
                .oriented_edge
                .get(id)
                .and_then(|oriented| match oriented.edge_element {
                    PlaceHolder::Ref(Name::Entity(inner)) => Some(inner),
                    _ => None,
                })
                .unwrap_or(*id);
            let Ok(edge_curve) = EntityTable::<EdgeCurveHolder>::get_owned(table, edge_curve_id)
            else {
                continue;
            };
            let Ok(curve) = edge_curve.parse_curve3d() else {
                continue;
            };
            sample_curve(&curve, &mut points);
        }
    }
    points
}

/// `PlaceHolder` -> holder, for the `Owned` and `Ref` cases the loader accepts.
fn resolve<H: Clone>(map: &HashMap<u64, H>, place: &PlaceHolder<H>) -> Option<H> {
    match place {
        PlaceHolder::Owned(holder) => Some(holder.clone()),
        PlaceHolder::Ref(Name::Entity(id)) => map.get(id).cloned(),
        _ => None,
    }
}

fn sample_curve(curve: &Curve3D, out: &mut Vec<Point3>) {
    let Some((t0, t1)) = curve.try_range_tuple() else {
        return;
    };
    if !t0.is_finite() || !t1.is_finite() {
        return;
    }
    let n = SAMPLES_PER_EDGE - 1;
    for index in 0..=n {
        let t = t0 + (t1 - t0) * index as f64 / n as f64;
        let p = curve.evaluate(t);
        if finite(p.to_vec()) {
            out.push(p);
        }
    }
}

fn finite(v: Vector3) -> bool { v.x.is_finite() && v.y.is_finite() && v.z.is_finite() }

// --------------------------------------------------------- the measurement

/// One face's required rectangle in the analytic surface's own parameter frame,
/// alongside the rectangle the emitted net spans.
#[derive(Clone, Copy, Debug)]
struct Extent {
    required_profile: (f64, f64),
    required_sweep: (f64, f64),
    emitted_profile: (f64, f64),
    emitted_sweep: (f64, f64),
    /// Worst distance from a boundary sample to the EXTENDED analytic surface.
    residual: f64,
    /// Model units per unit of `v`. `|extrusion_axis|` for an extrusion; for a
    /// revolution `v` is an angle and the full turn is always emitted, so 0.
    sweep_scale: f64,
    /// Model units per unit of `u` -- the profile's chord length over its span.
    profile_scale: f64,
    samples: usize,
}

impl Extent {
    /// How much surface, in model units, the boundary needs beyond the emitted
    /// net -- summed over the four sides it can be short on.
    fn shortfall(&self) -> f64 {
        let below_u = (self.emitted_profile.0 - self.required_profile.0).max(0.0);
        let above_u = (self.required_profile.1 - self.emitted_profile.1).max(0.0);
        let below_v = (self.emitted_sweep.0 - self.required_sweep.0).max(0.0);
        let above_v = (self.required_sweep.1 - self.emitted_sweep.1).max(0.0);
        (below_u + above_u) * self.profile_scale + (below_v + above_v) * self.sweep_scale
    }

    /// The multiple of the emitted sweep span the boundary actually needs. 1.0
    /// means exactly big enough; 2.7 means the net holds 37% of what is needed.
    fn required_sweep_span_ratio(&self) -> f64 {
        let emitted = self.emitted_sweep.1 - self.emitted_sweep.0;
        if emitted.abs() < f64::EPSILON {
            f64::INFINITY
        } else {
            (self.required_sweep.1 - self.required_sweep.0) / emitted
        }
    }
}

/// Grid resolution for the bracketing pass over the profile parameter, then this
/// many ternary-search rounds around the winning bracket.
const PROFILE_GRID: usize = 129;
const REFINE_ROUNDS: usize = 40;

/// Required extent of an extrusion, measured against the analytic
/// `S(u, v) = C(u) + extrusion_axis * v`.
///
/// For a fixed `u` the closest point on the sweep line is closed form --
/// `v = (P - C(u)) . w / |w|^2` -- so only `u` needs searching, over a bracketed
/// 1D residual with no derivative and no solver state.
fn extrusion_extent(surface: &StepExtrusionSurface, samples: &[Point3]) -> Option<Extent> {
    let profile = surface.entity_curve();
    let axis = surface.extruding_vector();
    let axis_len2 = axis.magnitude2();
    if axis_len2 <= 0.0 || !axis_len2.is_finite() {
        return None;
    }
    let (u0, u1) = profile.try_range_tuple()?;
    if !(u0.is_finite() && u1.is_finite()) || u1 <= u0 {
        return None;
    }

    let residual_at = |u: f64, p: Point3| {
        let c = profile.evaluate(u);
        let v = (p - c).dot(axis) / axis_len2;
        ((p - c) - axis * v).magnitude()
    };
    let v_at = |u: f64, p: Point3| (p - profile.evaluate(u)).dot(axis) / axis_len2;

    let mut ext = Extent {
        required_profile: (f64::INFINITY, f64::NEG_INFINITY),
        required_sweep: (f64::INFINITY, f64::NEG_INFINITY),
        emitted_profile: (u0, u1),
        emitted_sweep: (0.0, 1.0),
        residual: 0.0,
        sweep_scale: axis_len2.sqrt(),
        profile_scale: profile_profile_scale(profile, u0, u1),
        samples: samples.len(),
    };
    for &p in samples {
        let u = argmin_on_span(u0, u1, |u| residual_at(u, p));
        ext.residual = ext.residual.max(residual_at(u, p));
        let v = v_at(u, p);
        ext.required_profile.0 = ext.required_profile.0.min(u);
        ext.required_profile.1 = ext.required_profile.1.max(u);
        ext.required_sweep.0 = ext.required_sweep.0.min(v);
        ext.required_sweep.1 = ext.required_sweep.1.max(v);
    }
    (ext.samples > 0).then_some(ext)
}

/// Required extent of a revolution, measured against the analytic
/// `S(u, v) = rotate(C(u), v)` about `(origin, axis)`.
///
/// The revolution angle drops out: a point's `(axial, radial)` coordinates are
/// rotation invariant, so the profile match is a 1D search in that half-plane and
/// `v` is whatever angle the point sits at -- always inside the emitted full
/// turn. So the only axis that can truncate is `u`.
fn revolution_extent(surface: &StepRevolutionSurface, samples: &[Point3]) -> Option<Extent> {
    let revolution = surface.entity();
    let inverse = matrix::invert(surface.transform())?;
    let profile = revolution.entity_curve();
    let origin = revolution.origin();
    let axis = revolution.axis().normalize();
    let (u0, u1) = profile.try_range_tuple()?;
    if !(u0.is_finite() && u1.is_finite()) || u1 <= u0 {
        return None;
    }

    let cylindrical = |p: Point3| {
        let local = matrix::transform_point(&inverse, p);
        let r = local - origin;
        let z = r.dot(axis);
        (z, (r - axis * z).magnitude())
    };
    let residual_at = |u: f64, target: (f64, f64)| {
        let (z, r) = cylindrical_of_profile(profile, u, origin, axis);
        ((z - target.0).powi(2) + (r - target.1).powi(2)).sqrt()
    };

    let full_turn = std::f64::consts::TAU;
    let mut ext = Extent {
        required_profile: (f64::INFINITY, f64::NEG_INFINITY),
        required_sweep: (0.0, full_turn),
        emitted_profile: (u0, u1),
        emitted_sweep: (0.0, full_turn),
        residual: 0.0,
        sweep_scale: 0.0,
        profile_scale: profile_profile_scale(profile, u0, u1),
        samples: samples.len(),
    };
    for &p in samples {
        let target = cylindrical(p);
        let u = argmin_on_span(u0, u1, |u| residual_at(u, target));
        ext.residual = ext.residual.max(residual_at(u, target));
        ext.required_profile.0 = ext.required_profile.0.min(u);
        ext.required_profile.1 = ext.required_profile.1.max(u);
    }
    (ext.samples > 0).then_some(ext)
}

fn cylindrical_of_profile(profile: &Curve3D, u: f64, origin: Point3, axis: Vector3) -> (f64, f64) {
    let r = profile.evaluate(u) - origin;
    let z = r.dot(axis);
    (z, (r - axis * z).magnitude())
}

/// Model units per unit of profile parameter, as a chord-length average. Only
/// used to express a `u` shortfall as a length, so an average is enough.
fn profile_profile_scale(profile: &Curve3D, u0: f64, u1: f64) -> f64 {
    let n = 64;
    let mut length = 0.0;
    let mut previous = profile.evaluate(u0);
    for index in 1..=n {
        let u = u0 + (u1 - u0) * index as f64 / n as f64;
        let p = profile.evaluate(u);
        length += (p - previous).magnitude();
        previous = p;
    }
    if u1 > u0 { length / (u1 - u0) } else { 0.0 }
}

/// Bracket the minimum of `f` on `[a, b]` with a uniform grid, then ternary-search
/// the winning bracket. Deliberately dumb: no Newton step, no tolerance to tune,
/// and it cannot wander outside `[a, b]` -- the oracle must not be able to report
/// a truncation that is really a solver excursion.
fn argmin_on_span(a: f64, b: f64, f: impl Fn(f64) -> f64) -> f64 {
    let mut best = a;
    let mut best_value = f(a);
    for index in 1..PROFILE_GRID {
        let u = a + (b - a) * index as f64 / (PROFILE_GRID - 1) as f64;
        let value = f(u);
        if value < best_value {
            best_value = value;
            best = u;
        }
    }
    let step = (b - a) / (PROFILE_GRID - 1) as f64;
    let (mut lo, mut hi) = ((best - step).max(a), (best + step).min(b));
    for _ in 0..REFINE_ROUNDS {
        let m0 = lo + (hi - lo) / 3.0;
        let m1 = hi - (hi - lo) / 3.0;
        if f(m0) < f(m1) {
            hi = m1;
        } else {
            lo = m0;
        }
    }
    let mid = 0.5 * (lo + hi);
    if f(mid) < best_value { mid } else { best }
}

/// Everything needed from `cgmath` comes through the STEP geometry prelude, so
/// this test does not have to name `cgmath` (not a dev-dependency) directly.
mod matrix {
    use monstertruck_io::step::load::step_geometry::re_exports::{
        Matrix4, Point3, SquareMatrix, Transform,
    };

    pub fn invert(matrix: &Matrix4) -> Option<Matrix4> { matrix.invert() }

    pub fn transform_point(matrix: &Matrix4, p: Point3) -> Point3 { matrix.transform_point(p) }
}

// ----------------------------------------- emitted-net fidelity (not extent)
//
// The extent question presupposes that the emitted net IS the surface, merely
// bounded. That presupposition has to be measured too -- and on the extrusion
// arm it was FALSE before this track, which is why it is its own column rather
// than an assumption.

#[derive(Clone, Copy, Debug, Default)]
struct NetFidelity {
    /// Worst |emitted(u, v) - analytic(u, v)| over a grid inside the emitted
    /// rectangle, ignoring non-finite samples (counted separately).
    worst_deviation: f64,
    nonfinite: usize,
    compared: usize,
}

/// One `(analytic (u, v), emitted (u, v))` comparison point.
type FramePair = ((f64, f64), (f64, f64));

/// Compare the emitted `ModelingSurface` against the analytic STEP surface at a
/// list of `(analytic (u, v), emitted (u, v))` pairs.
///
/// The pairs are explicit because THE TWO FRAMES DIFFER by class, and comparing
/// them as if they did not is C2 in miniature -- see `fidelity_pairs`.
fn net_fidelity(emitted: &ModelingSurface, analytic: &Surface, pairs: &[FramePair]) -> NetFidelity {
    let mut out = NetFidelity::default();
    for &((au, av), (eu, ev)) in pairs {
        let want = ParametricSurface::evaluate(analytic, au, av);
        let got = ParametricSurface::evaluate(emitted, eu, ev);
        if !finite(want.to_vec()) {
            continue;
        }
        if !finite(got.to_vec()) {
            out.nonfinite += 1;
            continue;
        }
        out.compared += 1;
        out.worst_deviation = out.worst_deviation.max((want - got).magnitude());
    }
    out
}

/// Where to compare the two frames, per class. Both mappings were read off the
/// conversion code and are pinned by the in-gate tests below.
///
/// - **Extrusion.** The analytic value is a bare `ExtrusionSurface`: `u` is the
///   profile parameter, `v` in `[0, 1]` is the sweep. The emitted net uses the
///   same two numbers, so the pairs are the identity.
/// - **Revolution.** The analytic value is an INVERTED `Processor`
///   (`SurfaceOfRevolution::try_from` calls `invert()`), and an inverted
///   processor SWAPS `(u, v)` -- so analytically `u` is the revolution ANGLE IN
///   RADIANS and `v` is the profile parameter, which is ISO 10303-42's own
///   convention. `Processor::try_into_homogeneous_bspline_surface` mirrors the
///   swap with `BsplineSurface::invert` (a transpose), so the emitted net has the
///   same two axes -- but its angular axis is the rational circle's NORMALIZED
///   `[0, 1]` knot span, and inside each quadratic arc the knot-to-angle map is
///   non-linear. The only angles where the two frames are comparable at all are
///   therefore the arc knots: the quarter turns.
fn fidelity_pairs(surface: &Surface, profile_span: (f64, f64)) -> Vec<FramePair> {
    let (p0, p1) = profile_span;
    let profile = |i: usize, n: usize| p0 + (p1 - p0) * i as f64 / n as f64;
    match surface {
        Surface::SweepSurface(SweepSurface::RevolutionSurface(_)) => (0..=4)
            .flat_map(|q| {
                let t = q as f64 / 4.0;
                (0..=6).map(move |i| {
                    let p = profile(i, 6);
                    ((t * std::f64::consts::TAU, p), (t, p))
                })
            })
            .collect(),
        _ => (0..=6)
            .flat_map(|i| {
                let u = profile(i, 6);
                (0..=6).map(move |j| {
                    let v = j as f64 / 6.0;
                    ((u, v), (u, v))
                })
            })
            .collect(),
    }
}

// --------------------------------------------------------------- reporting

#[derive(Default)]
struct ClassReport {
    faces: usize,

    // --- modeling-conversion fate (the column Phase 0 never measured) ---
    modeling_converted: usize,
    modeling_refused: BTreeMap<String, usize>,
    modeling_panicked: BTreeMap<String, usize>,

    // --- emitted-net fidelity, among the converted ---
    net_faithful: usize,
    net_wrong: usize,
    net_nonfinite: usize,
    worst_net_deviation: f64,

    // --- extent, measured on the analytic surface ---
    unmeasurable: usize,
    off_surface: usize,
    covered: usize,
    truncated_sweep: usize,
    truncated_profile: usize,
    worst_shortfall: f64,
    total_shortfall: f64,
    worst_ratio: f64,
    ratio_histogram: BTreeMap<u64, usize>,
    worst_residual: f64,
}

/// A boundary sample further than this from the extended analytic surface means
/// the reconstruction did not find the point; the face is reported as
/// `off_surface` and excluded from the truncation rate. Deliberately coarse, so
/// the oracle errs toward NOT claiming a truncation.
const RESIDUAL_LIMIT: f64 = 1.0e-3;

/// A `u`/`v` excess below this fraction of the emitted span is a boundary that
/// touches the emitted edge exactly, not a truncation.
const PARAMETER_SLACK: f64 = 1.0e-9;

/// Emitted-vs-analytic agreement above this is a WRONG net, not round-off.
const FIDELITY_LIMIT: f64 = 1.0e-9;

impl ClassReport {
    fn record_modeling(&mut self, fate: &Result<Result<ModelingSurface, String>, String>) {
        match fate {
            Err(panic) => {
                *self
                    .modeling_panicked
                    .entry(without_ids(panic))
                    .or_default() += 1
            }
            Ok(Err(e)) => *self.modeling_refused.entry(without_ids(e)).or_default() += 1,
            Ok(Ok(_)) => self.modeling_converted += 1,
        }
    }

    fn record_fidelity(&mut self, fidelity: NetFidelity) {
        self.worst_net_deviation = self.worst_net_deviation.max(fidelity.worst_deviation);
        if fidelity.nonfinite > 0 {
            self.net_nonfinite += 1;
        }
        if fidelity.worst_deviation > FIDELITY_LIMIT {
            self.net_wrong += 1;
        } else if fidelity.nonfinite == 0 && fidelity.compared > 0 {
            self.net_faithful += 1;
        }
    }

    fn record_extent(&mut self, face_id: u64, surface_id: u64, extent: Option<Extent>) {
        let Some(extent) = extent else {
            self.unmeasurable += 1;
            println!("      [unmeasurable] face #{face_id} surface #{surface_id}");
            return;
        };
        self.worst_residual = self.worst_residual.max(extent.residual);
        // NaN counts as off-surface, hence the explicit finiteness test rather
        // than a negated comparison.
        if !extent.residual.is_finite() || extent.residual > RESIDUAL_LIMIT {
            self.off_surface += 1;
            println!(
                "      [off-surface] face #{face_id} surface #{surface_id} residual={:.4e} \
                 samples={} required profile {:?} sweep {:?}",
                extent.residual, extent.samples, extent.required_profile, extent.required_sweep
            );
            return;
        }
        let u_span = extent.emitted_profile.1 - extent.emitted_profile.0;
        let v_span = extent.emitted_sweep.1 - extent.emitted_sweep.0;
        let over_u = (extent.emitted_profile.0 - extent.required_profile.0).max(0.0)
            + (extent.required_profile.1 - extent.emitted_profile.1).max(0.0)
            > PARAMETER_SLACK * u_span.abs();
        let over_v = (extent.emitted_sweep.0 - extent.required_sweep.0).max(0.0)
            + (extent.required_sweep.1 - extent.emitted_sweep.1).max(0.0)
            > PARAMETER_SLACK * v_span.abs();
        if over_u {
            self.truncated_profile += 1;
        }
        if over_v {
            self.truncated_sweep += 1;
        }
        if !over_u && !over_v {
            self.covered += 1;
            return;
        }
        let shortfall = extent.shortfall();
        println!(
            "      [TRUNCATED] face #{face_id} surface #{surface_id} shortfall={shortfall:.4} \
             residual={:.2e} required profile {:?} vs emitted {:?}; required sweep {:?} vs \
             emitted {:?}; sweep vector length {:.4}",
            extent.residual,
            extent.required_profile,
            extent.emitted_profile,
            extent.required_sweep,
            extent.emitted_sweep,
            extent.sweep_scale
        );
        self.total_shortfall += shortfall;
        self.worst_shortfall = self.worst_shortfall.max(shortfall);
        let ratio = extent.required_sweep_span_ratio();
        if ratio.is_finite() {
            self.worst_ratio = self.worst_ratio.max(ratio);
            *self
                .ratio_histogram
                .entry(ratio.ceil().max(0.0) as u64)
                .or_default() += 1;
        }
    }

    fn print(&self, label: &str) {
        println!("\n  {label} -- {} faces", self.faces);
        println!(
            "    modeling conversion:  converted={:5}  refused={:5}  PANICKED={:5}",
            self.modeling_converted,
            self.modeling_refused.values().sum::<usize>(),
            self.modeling_panicked.values().sum::<usize>()
        );
        for (message, count) in &self.modeling_panicked {
            println!("      PANIC x{count}: {message}");
        }
        for (message, count) in &self.modeling_refused {
            println!("      refused x{count}: {message}");
        }
        println!(
            "    emitted net vs analytic (of the converted):  faithful={:5}  WRONG={:5}  \
             non-finite={:5}  worst deviation={:.3e}",
            self.net_faithful, self.net_wrong, self.net_nonfinite, self.worst_net_deviation
        );
        let measured = self.faces - self.off_surface - self.unmeasurable;
        let truncated = measured - self.covered;
        let rate = if measured == 0 {
            0.0
        } else {
            100.0 * truncated as f64 / measured as f64
        };
        println!(
            "    EXTENT: measured={measured:5}  covered={:5}  TRUNCATED={truncated:5} \
             ({rate:5.1}%)   off-surface={:4}  unmeasurable={:3}",
            self.covered, self.off_surface, self.unmeasurable
        );
        println!(
            "      truncated on sweep axis={:5}  on profile axis={:5}  \
             worst shortfall={:.4} model units  total={:.1}  worst residual={:.3e}",
            self.truncated_sweep,
            self.truncated_profile,
            self.worst_shortfall,
            self.total_shortfall,
            self.worst_residual
        );
        if !self.ratio_histogram.is_empty() {
            let rows = self
                .ratio_histogram
                .iter()
                .map(|(bucket, count)| format!("<={bucket}x: {count}"))
                .collect::<Vec<_>>()
                .join("  ");
            println!(
                "      required sweep span / emitted sweep span -- worst {:.2}x -- {rows}",
                self.worst_ratio
            );
        }
    }
}

// -------------------------------------------------------------- the oracle

#[test]
#[ignore = "needs the ~1 GB corpus; run with --run-ignored all"]
fn swept_surface_extent_oracle() {
    install_oracle_panic_hook();
    let Some(root) = corpus_root() else {
        println!("SKIP: no corpus (set MONSTERTRUCK_STEP_CORPUS)");
        return;
    };
    let only = std::env::var("MONSTERTRUCK_CENSUS_ONLY").ok();
    for name in SWEPT_CORPUS_FILES {
        if only.as_deref().is_some_and(|filter| !name.contains(filter)) {
            continue;
        }
        let path = root.join(name);
        if !path.is_file() {
            println!("SKIP {name}: absent");
            continue;
        }
        let bytes = std::fs::read(&path).expect("corpus file readable");
        let table = Table::from_step_bytes(&bytes).expect("corpus file table-parses");
        println!("\n== {name} ==");
        report_file(&table);
    }
}

fn report_file(table: &Table) {
    let mut extrusion = ClassReport::default();
    let mut revolution = ClassReport::default();
    for (&face_id, holder) in &table.face_surface {
        let PlaceHolder::Ref(Name::Entity(id)) = holder.face_geometry else {
            continue;
        };
        let is_extrusion = table.surface_of_linear_extrusion.contains_key(&id);
        let is_revolution = table.surface_of_revolution.contains_key(&id);
        if !is_extrusion && !is_revolution {
            continue;
        }
        let Ok(surface_any) = holder.face_geometry.clone().into_owned(table) else {
            continue;
        };
        let Ok(surface) = Surface::try_from(&surface_any) else {
            continue;
        };
        let samples = boundary_samples(table, holder);
        let report = if is_extrusion {
            &mut extrusion
        } else {
            &mut revolution
        };
        report.faces += 1;

        let fate = catching(|| ModelingSurface::try_from(&surface).map_err(|e| e.to_string()));
        report.record_modeling(&fate);
        let emitted = match fate {
            Ok(Ok(emitted)) => Some(emitted),
            _ => None,
        };

        let extent = match &surface {
            Surface::SweepSurface(SweepSurface::ExtrusionSurface(surface)) => {
                extrusion_extent(surface, &samples)
            }
            Surface::SweepSurface(SweepSurface::RevolutionSurface(surface)) => {
                revolution_extent(surface, &samples)
            }
            _ => None,
        };
        if let (Some(emitted), Some(extent)) = (emitted, extent) {
            let pairs = fidelity_pairs(&surface, extent.emitted_profile);
            let fidelity =
                catching(|| net_fidelity(&emitted, &surface, &pairs)).unwrap_or_default();
            report.record_fidelity(fidelity);
        }
        report.record_extent(face_id, id, extent);
    }
    extrusion.print("SURFACE_OF_LINEAR_EXTRUSION");
    revolution.print("SURFACE_OF_REVOLUTION");
}

// ---------------------------------------------- in-gate mechanism pinning

/// A profile, an extrusion of it, and the STEP surface wrapper -- the shape every
/// `SURFACE_OF_LINEAR_EXTRUSION` in the corpus has (all 3,341 carry a
/// `B_SPLINE_CURVE_WITH_KNOTS` profile; measured 2026-07-30).
fn extrusion_fixture(control_points: Vec<Point3>, degree: usize, axis: Vector3) -> Surface {
    use monstertruck_io::step::load::step_geometry::re_exports::{BsplineCurve, KnotVector};
    let divisions = control_points.len() - degree;
    let profile = Curve3D::BsplineCurve(BsplineCurve::new(
        KnotVector::uniform_knot(degree, divisions),
        control_points,
    ));
    Surface::SweepSurface(SweepSurface::ExtrusionSurface(
        StepExtrusionSurface::by_extrusion(profile, axis),
    ))
}

fn as_extrusion(surface: &Surface) -> &StepExtrusionSurface {
    match surface {
        Surface::SweepSurface(SweepSurface::ExtrusionSurface(extrusion)) => extrusion,
        _ => panic!("fixture is not an extrusion"),
    }
}

/// The oracle's central premise: the emitted net IS the analytic surface over the
/// span the oracle claims for it -- `u` = the profile's own knot span, `v` in
/// `[0, 1]` = one `extrusion_axis`.
#[test]
fn emitted_extrusion_net_matches_the_analytic_surface() {
    let surface = extrusion_fixture(
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.4, 0.7, 0.0),
            Point3::new(0.6, 1.6, 0.0),
            Point3::new(1.2, 2.4, 0.0),
        ],
        2,
        Vector3::new(0.0, 0.0, 7.5),
    );
    let analytic = as_extrusion(&surface);
    let (u0, u1) = analytic.entity_curve().try_range_tuple().unwrap();
    let emitted = ModelingSurface::try_from(&surface).expect("extrusion converts");
    let fidelity = net_fidelity(&emitted, &surface, &fidelity_pairs(&surface, (u0, u1)));
    assert_eq!(
        fidelity.nonfinite, 0,
        "the emitted net evaluated to NaN inside its own rectangle"
    );
    assert!(
        fidelity.worst_deviation < FIDELITY_LIMIT,
        "emitted net departs from the analytic extrusion by {:.3e} -- the extent \
         numbers would be about a different surface",
        fidelity.worst_deviation
    );

    // ... and the analytic surface keeps going past v = 1, which is what makes a
    // v > 1 requirement a truncation rather than an off-surface sample.
    let beyond = ParametricSurface::evaluate(analytic, u0, 3.0);
    let at_one = ParametricSurface::evaluate(analytic, u0, 1.0);
    assert!(
        (beyond - at_one).magnitude() > 1.0,
        "the analytic extrusion did not extend past its emitted span"
    );
    assert!(ParametricSurface3D::normal(analytic, u0, 0.5).magnitude() > 0.5);
}

/// The transposition regression guard. Before this track, the emitted extrusion
/// net indexed its control points `[sweep][profile]` while its knot vectors were
/// `(profile, sweep)`, so `BsplineSurface::try_new`'s knot-vs-control-point rule
/// PANICKED on every profile with 4 or more control points -- essentially every
/// real B-spline profile -- and silently produced a transposed surface for the
/// rest. This pins both halves, one fixture each.
#[test]
fn extrusion_nets_are_indexed_profile_by_sweep() {
    use monstertruck_io::step::load::step_geometry::re_exports::TryIntoHomogeneousBsplineSurface;

    // Four control points: the shape that used to panic.
    let wide = extrusion_fixture(
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.4, 0.7, 0.0),
            Point3::new(0.6, 1.6, 0.0),
            Point3::new(1.2, 2.4, 0.0),
        ],
        2,
        Vector3::new(0.0, 0.0, 7.5),
    );
    let net = wide
        .try_into_homogeneous_bspline_surface()
        .expect("extrusion has a homogeneous net");
    assert_eq!(
        (net.control_points().len(), net.control_points()[0].len()),
        (4, 2),
        "control points must be indexed [profile][sweep]"
    );
    assert_eq!(net.vdegree(), 1, "the sweep axis is linear");
    assert_eq!(
        net.udegree(),
        2,
        "the profile axis keeps the profile's degree"
    );
    let profile_span = as_extrusion(&wide)
        .entity_curve()
        .try_range_tuple()
        .unwrap();
    let (u0, u1) = (
        *net.knot_vectors().0.first().unwrap(),
        *net.knot_vectors().0.last().unwrap(),
    );
    let (v0, v1) = (
        *net.knot_vectors().1.first().unwrap(),
        *net.knot_vectors().1.last().unwrap(),
    );
    assert!((u0 - profile_span.0).abs() < 1.0e-12 && (u1 - profile_span.1).abs() < 1.0e-12);
    assert!(v0.abs() < 1.0e-12 && (v1 - 1.0).abs() < 1.0e-12);

    // Three control points: the shape that used to convert to a TRANSPOSED
    // surface instead of panicking -- silent-wrong, the worse half.
    let narrow = extrusion_fixture(
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        1,
        Vector3::new(0.0, 0.0, 4.0),
    );
    let analytic = as_extrusion(&narrow);
    let emitted = ModelingSurface::try_from(&narrow).expect("extrusion converts");
    let (u0, u1) = analytic.entity_curve().try_range_tuple().unwrap();
    // (0.25, 0.75) is the point that used to read [1, 1, 0.667] instead of
    // [1, 0.5, 3].
    let want = ParametricSurface::evaluate(analytic, 0.25, 0.75);
    let got = ParametricSurface::evaluate(&emitted, 0.25, 0.75);
    assert!(
        (want - got).magnitude() < 1.0e-12,
        "transposed evaluation: analytic {want:?} vs emitted {got:?}"
    );
    let fidelity = net_fidelity(&emitted, &narrow, &fidelity_pairs(&narrow, (u0, u1)));
    assert_eq!(fidelity.nonfinite, 0);
    assert!(fidelity.worst_deviation < FIDELITY_LIMIT);
}

/// The oracle itself, on geometry whose answer is known by construction: a face
/// whose boundary spans 3.0 sweep units of a surface emitted over 1.0 must read
/// as truncated by 2 x |axis| model units, and one that stays inside must read as
/// covered. Without this the corpus numbers are unfalsifiable.
#[test]
fn oracle_separates_a_truncating_boundary_from_a_covered_one() {
    let surface = extrusion_fixture(
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
        ],
        1,
        Vector3::new(0.0, 0.0, 4.0),
    );
    let extrusion = as_extrusion(&surface);
    let (u0, u1) = extrusion.entity_curve().try_range_tuple().unwrap();
    let on_surface = |u: f64, v: f64| ParametricSurface::evaluate(extrusion, u, v);

    let inside: Vec<Point3> = (0..=8)
        .map(|i| on_surface(u0 + (u1 - u0) * i as f64 / 8.0, i as f64 / 8.0))
        .collect();
    let inside = extrusion_extent(extrusion, &inside).expect("measurable");
    assert!(inside.residual < 1.0e-10, "residual {}", inside.residual);
    assert!(
        inside.shortfall() == 0.0,
        "a boundary inside the emitted span must not read as truncated: {inside:?}"
    );

    let outside: Vec<Point3> = (0..=8)
        .map(|i| on_surface(u0 + (u1 - u0) * i as f64 / 8.0, 3.0 * i as f64 / 8.0))
        .collect();
    let outside = extrusion_extent(extrusion, &outside).expect("measurable");
    assert!(outside.residual < 1.0e-8, "residual {}", outside.residual);
    assert!(
        (outside.required_sweep.1 - 3.0).abs() < 1.0e-6,
        "required v top should be 3.0, got {}",
        outside.required_sweep.1
    );
    // Two extra sweep units of a 4-unit axis = 8 model units missing.
    assert!(
        (outside.shortfall() - 8.0).abs() < 1.0e-4,
        "shortfall should be 8 model units, got {}",
        outside.shortfall()
    );
    assert!((outside.required_sweep_span_ratio() - 3.0).abs() < 1.0e-6);
}
