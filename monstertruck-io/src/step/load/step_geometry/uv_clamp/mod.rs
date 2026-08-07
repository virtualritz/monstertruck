//! Spec 012 U2 -- the `normalize_uv` clamp lens, and the provenance question it
//! exists to answer.
//!
//! [`sampled_parameter_boundary`](super::geom_impls) normalizes every projected
//! `(u, v)` against `surface.try_range_tuple()`. Two of its arms WRITE:
//! `clamp_near_range` snaps a value that overshoots the reported range by less
//! than `TOLERANCE` onto the boundary, and the periodic no-previous arm ends in
//! an UNCONDITIONAL `normalized.clamp(min, max)`.
//!
//! The question is not "does it clamp" but **what is it clamping against**. For
//! several surface classes the reported range is not a measurement of the
//! surface at all -- it is a placeholder that exists so `range_tuple()` has
//! something to return (see
//! `monstertruck-geometry/src/specifieds/plane.rs::parameter_range`, whose own
//! doc forbids treating the unit square as a domain). Spec 011 fixed the
//! modeling twin by not asking those axes the question
//! (`monstertruck-modeling/src/geometry.rs::reported_range_bounds_the_surface`);
//! [`reported_range_bounds_the_surface`] here is the same idea against the STEP
//! surface enum.
//!
//! Everything below the provenance function is MEASUREMENT ONLY: the census is
//! a thread-local behind `MT_STEP_DEBUG_UV_CLAMP`, nothing it computes is fed
//! back into the geometry, and with the variable unset every recording call is
//! a single `OnceLock` read followed by a return.
//!
//! # Two findings this lens produced that were not the point of it
//!
//! **The unconditional periodic clamp has never fired.** `normalize_axis`'s
//! no-previous periodic arm ends in `normalized.clamp(min, max)` with no
//! near-range test, which reads as the most dangerous write in the routine.
//! Measured: `clamped = 0` on every row of both populations -- 8,339 in-repo and
//! 14,908 corpus invocations of that arm, 0 moves, and 0 `span.so_small()`
//! degenerate short-circuits. It cannot fire on today's surfaces because every
//! periodic axis reaching it reports `max - min == period` exactly (a full turn),
//! so the wrap already lands inside `[min, max]` and the clamp is a no-op. It is
//! a LATENT defect, guarded rather than repaired: nothing observed needed it
//! fixed, and this paragraph exists so it is not later cited as a fix (M10).
//!
//! **`clamp_near_range` is NOT a C3 guard, and the ledger says it is.**
//! `KERNEL_FAILURE_CLASSES.md` C3 records that "the STEP crate's twin
//! (`geom_impls.rs:21-124`) has had this discipline all along via
//! `normalize_axis` / `clamp_near_range`". It has not. `clamp_near_range` acts
//! only on an excess BELOW `TOLERANCE`; a C3 excursion is by definition orders
//! of magnitude outside the domain (the class's own example is `-44.146` on a
//! `[0, 1]` domain) and passes through this function untouched. Measured on
//! KNOT-bounded axes, where the range is a real bound and the excursion is a
//! genuine domain violation: 2,441 in-repo and 16,347 corpus raw answers land
//! more than `TOLERANCE` outside the knot vector, up to 21.8 past it on a
//! `RATIONAL_B_SPLINE_SURFACE`. Not one is repaired here. The STEP twin's real
//! protection is its solver ORDER (checked `search_parameter` first) and the
//! periodic nearest-multiple arm -- not the clamp. Fixing that is a separate
//! job with its own oracle; recorded, not attempted.

use super::{Conic3D, Curve3D, ElementarySurface, Surface, SweepSurface, re_exports::TOLERANCE};
use std::cell::RefCell;

/// Which axis a recording belongs to.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Axis {
    U,
    V,
}

/// The surface classes the census splits on, named as the STEP entity is named.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub(crate) enum SurfaceClass {
    Plane,
    Sphere,
    Cylinder,
    Torus,
    Cone,
    Extrusion,
    Revolution,
    Bspline,
    Nurbs,
}

impl SurfaceClass {
    /// Presentation order for [`ClampCensus::report`]; the census itself is
    /// indexed by [`SurfaceClass::index`].
    #[cfg(test)]
    pub(crate) const ALL: [Self; 9] = [
        Self::Plane,
        Self::Sphere,
        Self::Cylinder,
        Self::Torus,
        Self::Cone,
        Self::Extrusion,
        Self::Revolution,
        Self::Bspline,
        Self::Nurbs,
    ];

    const fn index(self) -> usize {
        match self {
            Self::Plane => 0,
            Self::Sphere => 1,
            Self::Cylinder => 2,
            Self::Torus => 3,
            Self::Cone => 4,
            Self::Extrusion => 5,
            Self::Revolution => 6,
            Self::Bspline => 7,
            Self::Nurbs => 8,
        }
    }

    #[cfg(test)]
    pub(crate) const fn label(self) -> &'static str {
        match self {
            Self::Plane => "PLANE",
            Self::Sphere => "SPHERICAL_SURFACE",
            Self::Cylinder => "CYLINDRICAL_SURFACE",
            Self::Torus => "TOROIDAL_SURFACE",
            Self::Cone => "CONICAL_SURFACE",
            Self::Extrusion => "SURFACE_OF_LINEAR_EXTRUSION",
            Self::Revolution => "SURFACE_OF_REVOLUTION",
            Self::Bspline => "B_SPLINE_SURFACE",
            Self::Nurbs => "RATIONAL_B_SPLINE_SURFACE",
        }
    }

    pub(crate) fn of(surface: &Surface) -> Self {
        match surface {
            Surface::ElementarySurface(ElementarySurface::Plane(_)) => Self::Plane,
            Surface::ElementarySurface(ElementarySurface::Sphere(_)) => Self::Sphere,
            Surface::ElementarySurface(ElementarySurface::CylindricalSurface(_)) => Self::Cylinder,
            Surface::ElementarySurface(ElementarySurface::ToroidalSurface(_)) => Self::Torus,
            Surface::ElementarySurface(ElementarySurface::ConicalSurface(_)) => Self::Cone,
            Surface::SweepSurface(SweepSurface::ExtrusionSurface(_)) => Self::Extrusion,
            Surface::SweepSurface(SweepSurface::RevolutionSurface(_)) => Self::Revolution,
            Surface::BsplineSurface(_) => Self::Bspline,
            Surface::NurbsSurface(_) => Self::Nurbs,
        }
    }
}

/// Whether the curve's reported range is a real BOUND rather than a placeholder.
///
/// Mirrors `monstertruck-modeling`'s `reported_range_bounds_the_curve`, over the
/// STEP curve enum. `Line::parameter_range` is a hardcoded `[0, 1]` over
/// `p0 + t * (p1 - p0)`, and the STEP loader builds a cylinder's and a cone's
/// profile with a UNIT direction vector (`load/step_types.rs`), so `t` is a
/// world-scale distance along an axially UNBOUNDED surface and `[0, 1]` is one
/// arbitrary unit of it.
///
/// Everything else answers `true`, which is the safe default: it leaves the
/// clamp exactly as it is.
fn reported_range_bounds_the_curve(curve: &Curve3D) -> bool {
    match curve {
        Curve3D::Line(_) => false,
        // `UnitParabola` / `UnitHyperbola` report no range at all
        // (`try_range_tuple() -> None`), so the trimmed processor's range is the
        // trim -- a real one. Knot vectors and the delegating decorators all
        // report the extent they carry data for.
        Curve3D::Conic(Conic3D::Ellipse(_) | Conic3D::Hyperbola(_) | Conic3D::Parabola(_))
        | Curve3D::Polyline(_)
        | Curve3D::BsplineCurve(_)
        | Curve3D::NurbsCurve(_)
        | Curve3D::IntersectionCurve(_) => true,
        Curve3D::ParameterCurve(_) => true,
        Curve3D::SurfaceCurve(curve) => reported_range_bounds_the_curve(curve.leader()),
    }
}

/// Whether each axis of `surface`'s reported parameter range is a real BOUND, as
/// `(u, v)`.
///
/// This is the comparand question spec 011 answered on the modeling side. A
/// `false` axis means `try_range_tuple()` returned a number that is NOT a
/// measurement of the surface:
///
/// - `Plane::parameter_range` is a hardcoded `[0, 1] x [0, 1]` on a surface that
///   is infinite in both directions, and the STEP loader builds planes from an
///   axis placement with unit direction vectors, so `u` and `v` are world-scale
///   distances. A planar trim at `u = 12.5` is perfectly ordinary.
/// - a `RevolutionSurface`'s PROFILE axis inherits the profile curve's range,
///   which for the cylinders and cones the loader emits is the nominal unit
///   segment above. Its TURN axis, `[0, 2pi)` with a matching period, is real.
///   `Processor` swaps the two axes when its orientation is reversed, and
///   `From<&CylindricalSurface>` inverts, so on a loaded cylinder the PERIODIC
///   axis is `u` and the placeholder is `v`.
/// - an `ExtrusionSurface`'s `v` range is a hardcoded `[0, 1]` over the
///   extruding VECTOR (`decorators/extruded_curve.rs::parameter_range`), and a
///   STEP `SURFACE_OF_LINEAR_EXTRUSION` is unbounded along it.
///
/// A sphere's `[0, PI] x [0, 2pi)` and a torus's `[0, 2pi)^2` ARE measurements:
/// they close the surface. Knot vectors are measurements: the net carries no
/// data outside them.
pub(crate) fn reported_range_bounds_the_surface(surface: &Surface) -> (bool, bool) {
    match surface {
        Surface::ElementarySurface(ElementarySurface::Plane(_)) => (false, false),
        Surface::ElementarySurface(
            ElementarySurface::Sphere(_) | ElementarySurface::ToroidalSurface(_),
        ) => (true, true),
        Surface::ElementarySurface(
            ElementarySurface::CylindricalSurface(processor)
            | ElementarySurface::ConicalSurface(processor),
        ) => {
            let profile =
                reported_range_bounds_the_curve(&Curve3D::Line(*processor.entity().entity_curve()));
            axes_by_orientation(processor.orientation(), profile)
        }
        Surface::SweepSurface(SweepSurface::RevolutionSurface(processor)) => {
            let profile = reported_range_bounds_the_curve(processor.entity().entity_curve());
            axes_by_orientation(processor.orientation(), profile)
        }
        // `u` is the swept curve's own range; `v` is the hardcoded unit segment
        // along the extruding vector. There is no `Processor` here, so no swap.
        Surface::SweepSurface(SweepSurface::ExtrusionSurface(surface)) => (
            reported_range_bounds_the_curve(surface.entity_curve()),
            false,
        ),
        Surface::BsplineSurface(_) | Surface::NurbsSurface(_) => (true, true),
    }
}

/// `RevolutionSurface` reports `(profile, turn)`; a reversed `Processor` swaps
/// the pair. A full turn is always a real bound.
const fn axes_by_orientation(orientation: bool, profile: bool) -> (bool, bool) {
    if orientation {
        (profile, true)
    } else {
        (true, profile)
    }
}

// ---------------------------------------------------------------------------
// The lens. MEASUREMENT ONLY -- nothing below is read back into the geometry.
// ---------------------------------------------------------------------------

/// One `(class, axis, provenance)` row of the census.
#[derive(Clone, Copy, Default, Debug)]
pub(crate) struct ClampRow {
    /// `clamp_near_range` invocations on this row.
    pub(crate) calls: usize,
    /// Invocations that returned a DIFFERENT value than they were given.
    pub(crate) moved: usize,
    /// Moves onto `min`. On a placeholder axis `min` is the surface's own
    /// parameter ORIGIN (a plane's placement point, a revolution profile's
    /// start), so a move here snaps to something the surface does mean --
    /// just not to a domain edge.
    pub(crate) moved_min: usize,
    /// Moves onto `max`. On a placeholder axis `max` is one arbitrary unit
    /// along an unbounded direction and means nothing at all. This is the
    /// population U2 exists to find.
    pub(crate) moved_max: usize,
    /// Largest `|clamped - original|` observed.
    pub(crate) max_move: f64,
    /// Invocations of the periodic no-previous arm (wrap plus the unconditional
    /// `clamp`).
    pub(crate) periodic_calls: usize,
    /// Periodic-arm invocations where the WRAP moved the value.
    pub(crate) periodic_wrapped: usize,
    /// Periodic-arm invocations where the unconditional `clamp` moved the
    /// already-wrapped value. This is the arm the spec flags as strongest.
    pub(crate) periodic_clamped: usize,
    /// Largest `|clamped - wrapped|` on the unconditional clamp.
    pub(crate) periodic_max_clamp: f64,
    /// Periodic-arm invocations that took the degenerate `span.so_small()`
    /// short-circuit and returned `min` outright.
    pub(crate) periodic_degenerate: usize,
    /// Values that reached `normalize_axis` and were passed through with no
    /// range to test against at all.
    pub(crate) unranged: usize,
    /// Raw solver answers (before any normalization) that sat INSIDE the range
    /// the surface reports.
    pub(crate) reported_in: usize,
    /// Raw answers outside the reported range by less than `TOLERANCE`. This is
    /// exactly the population `clamp_near_range` can act on.
    pub(crate) reported_out_near: usize,
    /// Raw answers outside the reported range by `TOLERANCE` OR MORE.
    /// `clamp_near_range` passes every one of these through untouched, so this
    /// column is the honest answer to "does the STEP twin guard C3?".
    pub(crate) reported_out_far: usize,
    /// Largest excess beyond the reported range, in the axis's own units.
    pub(crate) max_excess: f64,
}

impl ClampRow {
    #[cfg(test)]
    fn is_empty(&self) -> bool {
        self.calls == 0
            && self.periodic_calls == 0
            && self.unranged == 0
            && self.reported_in == 0
            && self.reported_out_near == 0
            && self.reported_out_far == 0
    }
}

/// The whole census: `[class][axis][provenance]`, provenance index 0 = the
/// reported range is a PLACEHOLDER, 1 = it is a real bound.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ClampCensus {
    pub(crate) rows: [[[ClampRow; 2]; 2]; 9],
    /// Chains (`sampled_parameter_boundary` calls) whose points were normalized.
    pub(crate) chains: usize,
    /// Chains in which at least one value was MOVED by either clamp.
    pub(crate) chains_moved: usize,
    /// Points normalized.
    pub(crate) points: usize,
}

impl Default for ClampCensus {
    fn default() -> Self {
        Self {
            rows: [[[ClampRow {
                calls: 0,
                moved: 0,
                moved_min: 0,
                moved_max: 0,
                max_move: 0.0,
                periodic_calls: 0,
                periodic_wrapped: 0,
                periodic_clamped: 0,
                periodic_max_clamp: 0.0,
                periodic_degenerate: 0,
                unranged: 0,
                reported_in: 0,
                reported_out_near: 0,
                reported_out_far: 0,
                max_excess: 0.0,
            }; 2]; 2]; 9],
            chains: 0,
            chains_moved: 0,
            points: 0,
        }
    }
}

impl ClampCensus {
    #[cfg(test)]
    pub(crate) fn row(&self, class: SurfaceClass, axis: Axis, real_range: bool) -> ClampRow {
        self.rows[class.index()][usize::from(axis == Axis::V)][usize::from(real_range)]
    }

    fn row_mut(&mut self, class: SurfaceClass, axis: Axis, real_range: bool) -> &mut ClampRow {
        &mut self.rows[class.index()][usize::from(axis == Axis::V)][usize::from(real_range)]
    }

    /// Total moves across every row, by provenance: `(placeholder, real)`.
    #[cfg(test)]
    pub(crate) fn moved_by_provenance(&self) -> (usize, usize) {
        let mut totals = (0, 0);
        for class in SurfaceClass::ALL {
            for axis in [Axis::U, Axis::V] {
                for real in [false, true] {
                    let row = self.row(class, axis, real);
                    let moves = row.moved + row.periodic_clamped;
                    if real {
                        totals.1 += moves;
                    } else {
                        totals.0 += moves;
                    }
                }
            }
        }
        totals
    }

    #[cfg(test)]
    pub(crate) fn report(&self, tag: &str) {
        eprintln!(
            "[uv-clamp] {tag} chains={} chains_moved={} points={}",
            self.chains, self.chains_moved, self.points,
        );
        for class in SurfaceClass::ALL {
            for axis in [Axis::U, Axis::V] {
                for real in [false, true] {
                    let row = self.row(class, axis, real);
                    if row.is_empty() {
                        continue;
                    }
                    eprintln!(
                        "[uv-clamp] {tag} {:<27} {:?} range={:<11} calls={:<8} moved={:<6} \
                         to_min={:<6} to_max={:<6} max_move={:<12.3e} periodic_calls={:<8} \
                         wrapped={:<6} clamped={:<6} max_clamp={:<12.3e} degenerate={:<4} \
                         unranged={:<6} in={:<8} out_near={:<6} out_far={:<8} \
                         max_excess={:.3e}",
                        class.label(),
                        axis,
                        if real { "measured" } else { "PLACEHOLDER" },
                        row.calls,
                        row.moved,
                        row.moved_min,
                        row.moved_max,
                        row.max_move,
                        row.periodic_calls,
                        row.periodic_wrapped,
                        row.periodic_clamped,
                        row.periodic_max_clamp,
                        row.periodic_degenerate,
                        row.unranged,
                        row.reported_in,
                        row.reported_out_near,
                        row.reported_out_far,
                        row.max_excess,
                    );
                }
            }
        }
        let (placeholder, real) = self.moved_by_provenance();
        eprintln!("[uv-clamp] {tag} TOTAL moved: placeholder={placeholder} measured={real}");
    }
}

thread_local! {
    static CENSUS: RefCell<ClampCensus> = RefCell::new(ClampCensus::default());
    /// Set while a chain is being normalized; `true` once anything moved.
    static CHAIN_MOVED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Read once: the lens must not cost an environment lookup per sample.
pub(crate) fn enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var_os("MT_STEP_DEBUG_UV_CLAMP").is_some())
}

fn record(update: impl FnOnce(&mut ClampCensus)) {
    if !enabled() {
        return;
    }
    CENSUS.with(|census| update(&mut census.borrow_mut()));
}

/// What one `normalize_axis` call did with its range, as reported by the caller.
pub(crate) enum Event {
    /// `clamp_near_range` ran. `moved` is `clamped - value`; `to_max` says
    /// which end it snapped to when it moved.
    Clamp { moved: f64, to_max: bool },
    /// The periodic no-previous arm ran.
    Periodic {
        wrapped: bool,
        clamped: f64,
        degenerate: bool,
    },
    /// No range was reported for this axis; the value passed through.
    Unranged,
}

/// Records where a RAW solver answer sat relative to the range the surface
/// REPORTS -- taken before any normalization, and independently of whether the
/// clamp is allowed to act on that range. This is the column that separates
/// "the clamp fires" from "the domain is violated": `clamp_near_range` only ever
/// touches an excess below `TOLERANCE`, so everything in `reported_out_far` is a
/// domain excursion the STEP twin does NOT repair.
pub(crate) fn record_reported_excess(
    surface: &Surface,
    axis: Axis,
    real_range: bool,
    value: f64,
    range: Option<(f64, f64)>,
) {
    let Some((min, max)) = range else { return };
    record(|census| {
        let row = census.row_mut(SurfaceClass::of(surface), axis, real_range);
        let excess = f64::max(min - value, value - max);
        if excess <= 0.0 {
            row.reported_in += 1;
        } else if excess < TOLERANCE {
            row.reported_out_near += 1;
        } else {
            row.reported_out_far += 1;
        }
        row.max_excess = row.max_excess.max(excess);
    });
}

pub(crate) fn record_axis(surface: &Surface, axis: Axis, real_range: bool, event: Event) {
    record(|census| {
        let row = census.row_mut(SurfaceClass::of(surface), axis, real_range);
        let mut moved = false;
        match event {
            Event::Clamp {
                moved: delta,
                to_max,
            } => {
                row.calls += 1;
                if delta != 0.0 {
                    row.moved += 1;
                    if to_max {
                        row.moved_max += 1;
                    } else {
                        row.moved_min += 1;
                    }
                    row.max_move = row.max_move.max(delta.abs());
                    moved = true;
                }
            }
            Event::Periodic {
                wrapped,
                clamped,
                degenerate,
            } => {
                row.periodic_calls += 1;
                if degenerate {
                    row.periodic_degenerate += 1;
                }
                if wrapped {
                    row.periodic_wrapped += 1;
                }
                if clamped != 0.0 {
                    row.periodic_clamped += 1;
                    row.periodic_max_clamp = row.periodic_max_clamp.max(clamped.abs());
                    moved = true;
                }
            }
            Event::Unranged => row.unranged += 1,
        }
        if moved {
            CHAIN_MOVED.with(|flag| flag.set(true));
        }
    });
}

pub(crate) fn record_point() { record(|census| census.points += 1); }

pub(crate) fn begin_chain() {
    record(|census| {
        census.chains += 1;
        CHAIN_MOVED.with(|flag| flag.set(false));
    });
}

pub(crate) fn end_chain() {
    record(|census| {
        if CHAIN_MOVED.with(std::cell::Cell::get) {
            census.chains_moved += 1;
        }
    });
}

/// Snapshot of the census on the calling thread.
#[cfg(test)]
pub(crate) fn snapshot() -> ClampCensus { CENSUS.with(|census| *census.borrow()) }

/// Clears the census on the calling thread.
#[cfg(test)]
pub(crate) fn reset() { CENSUS.with(|census| *census.borrow_mut() = ClampCensus::default()); }

#[cfg(test)]
mod tests;
