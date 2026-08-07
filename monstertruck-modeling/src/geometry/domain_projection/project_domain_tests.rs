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
/// (`monstertruck-io/src/step/load/step_types/`) builds one: revolve
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
