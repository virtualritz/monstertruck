use super::*;
use monstertruck_geometry::prelude::{
    HomogeneousSurfaceConversion, SupportsExactPatchDomains, SurfaceParameterRectangle,
    TryIntoBsplineSurface, TryIntoHomogeneousBsplineCurve, TryIntoHomogeneousBsplineSurface,
};
use monstertruck_modeling::{
    Conic2D as ModelingConic2D, Curve as ModelingCurve, Curve2D as ModelingCurve2D,
    Surface as ModelingSurface,
};
use monstertruck_traits::SnapCurveEndpoints;
use std::{cmp::Ordering, env};
// `std::time::Instant::now()` panics on `wasm32-unknown-unknown`. The
// `web_time` crate is std-compatible on native and falls back to
// `performance.now()` in the browser, so all the `Instant::now()` /
// `.elapsed()` call sites below stay unchanged.
use web_time::Instant;

#[cfg(test)]
use std::f64::consts::TAU;

fn sampled_parameter_boundary<C>(
    curve: &C,
    surface: &Surface,
    tolerance: f64,
) -> Option<Vec<Point2>>
where
    C: ParametricCurve3D + BoundedCurve + ParameterDivision1D<Point = Point3>,
{
    fn abs_diff(previous: f64) -> impl Fn(&f64, &f64) -> Ordering {
        let distance = move |value: &f64| f64::abs(*value - previous);
        // SAFETY: All compared values are finite after the finiteness check in
        // `normalize_axis`.
        move |lhs: &f64, rhs: &f64| distance(lhs).partial_cmp(&distance(rhs)).unwrap()
    }

    /// `surface` and `axis` are carried for the `MT_STEP_DEBUG_UV_CLAMP` lens
    /// only (`uv_clamp.rs`); with the variable unset every `record_axis` call is
    /// a `OnceLock` read and a return, and no value the lens computes is read
    /// back here.
    fn normalize_axis(
        value: f64,
        previous: Option<f64>,
        period: Option<f64>,
        range: Option<(f64, f64)>,
        surface: &Surface,
        axis: uv_clamp::Axis,
        real_range: bool,
    ) -> Option<f64> {
        if !value.is_finite() {
            None
        } else if let Some(previous) = previous {
            if let Some(period) = period {
                (-2..=2)
                    .map(|index| value + index as f64 * period)
                    .min_by(abs_diff(previous))
            } else if let Some(range) = range {
                let clamped = clamp_near_range(value, range);
                uv_clamp::record_axis(
                    surface,
                    axis,
                    real_range,
                    uv_clamp::Event::Clamp {
                        moved: clamped - value,
                        to_max: clamped == range.1,
                    },
                );
                Some(clamped)
            } else {
                uv_clamp::record_axis(surface, axis, real_range, uv_clamp::Event::Unranged);
                Some(value)
            }
        } else if let Some((min, max)) = range {
            if let Some(period) = period {
                let span = max - min;
                if span.so_small() {
                    uv_clamp::record_axis(
                        surface,
                        axis,
                        real_range,
                        uv_clamp::Event::Periodic {
                            wrapped: min != value,
                            clamped: 0.0,
                            degenerate: true,
                        },
                    );
                    Some(min)
                } else {
                    let mut normalized = value - f64::floor((value - min) / period) * period;
                    if normalized < min {
                        normalized += period;
                    }
                    if normalized > max {
                        normalized -= period;
                    }
                    let clamped = normalized.clamp(min, max);
                    uv_clamp::record_axis(
                        surface,
                        axis,
                        real_range,
                        uv_clamp::Event::Periodic {
                            wrapped: normalized != value,
                            clamped: clamped - normalized,
                            degenerate: false,
                        },
                    );
                    Some(clamped)
                }
            } else {
                let clamped = clamp_near_range(value, (min, max));
                uv_clamp::record_axis(
                    surface,
                    axis,
                    real_range,
                    uv_clamp::Event::Clamp {
                        moved: clamped - value,
                        to_max: clamped == max,
                    },
                );
                Some(clamped)
            }
        } else {
            uv_clamp::record_axis(surface, axis, real_range, uv_clamp::Event::Unranged);
            Some(value)
        }
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

    // THE CLAMP IS NOT ASKED ABOUT AN AXIS THAT REPORTS A PLACEHOLDER.
    //
    // `clamp_near_range` snaps a value that overshoots the reported range by
    // less than `TOLERANCE` onto the boundary. On a knot-bounded axis that is
    // exactly right and it is doing real work: the net carries no data past the
    // knot vector, so an unbounded Newton that walked one ULP-cloud past the end
    // must come back. On a PLACEHOLDER axis there is no boundary to come back
    // to -- a plane's `[0, 1]^2` and a loaded cylinder's or cone's axial `[0, 1]`
    // are the values `parameter_range` returns so `range_tuple()` has something
    // to return, over an unbounded direction whose parameter is a world-scale
    // distance. Clamping there writes a number the solver did not produce, onto
    // a line in parameter space that means nothing (see
    // `uv_clamp::reported_range_bounds_the_surface`, and the 40-line doc on
    // `monstertruck-geometry/src/specifieds/plane.rs::parameter_range` that
    // forbids treating the square as a domain). Same repair as spec 011's
    // `a4604cef` on the modeling twin, and the same reason: do not ask a
    // placeholder a question it cannot answer.
    //
    // MEASURED before the change with `MT_STEP_DEBUG_UV_CLAMP=1` (the lens in
    // `uv_clamp.rs`, driven by `uv_clamp_probe.rs`), over two populations: the
    // 15 in-repo fixtures at full depth (58,244 chains, 939,416 points) and 7 of
    // the 8 big-assembly corpus files sampled by lowest solid id (81 solids,
    // 35,447 chains, 862,318 points; Scania-Engine not sampled -- its parse
    // alone exceeds nextest's 20-minute kill). **The clamp is LIVE**, and the
    // split is what decides the shape of the fix:
    //
    // | axis                     | calls   | MOVED  | -> min | -> max | max |move| |
    // |--------------------------|--------:|-------:|-------:|-------:|-----------:|
    // | IN-REPO                  |         |        |        |        |            |
    // | plane u (placeholder)    | 490,007 |  1,324 |  1,302 |     22 |    7.43e-7 |
    // | plane v (placeholder)    | 490,007 |  1,582 |  1,524 |     58 |   2.78e-10 |
    // | cylinder v (placeholder) |  67,472 |  3,655 |  3,569 |     86 |   5.63e-11 |
    // | cone v (placeholder)     |   3,926 |    704 |    704 |      0 |   9.61e-11 |
    // | bspline u+v (knots)      | 128,664 | 24,144 | 12,105 | 12,039 |    8.60e-7 |
    // | nurbs u+v (knots)        | 624,366 | 64,708 | 36,896 | 27,812 |    9.30e-7 |
    // | CORPUS                   |         |        |        |        |            |
    // | plane u (placeholder)    | 291,687 |    842 |    828 |     14 |    7.06e-7 |
    // | plane v (placeholder)    | 291,687 |    768 |    748 |     20 |    9.58e-7 |
    // | cylinder v (placeholder) | 311,100 | 17,492 | 14,717 |  2,775 |    2.50e-7 |
    // | cone v (placeholder)     | 103,061 | 26,338 | 23,057 |  3,281 |   1.21e-13 |
    // | bspline u+v (knots)      |  86,630 |  5,670 |  3,335 |  2,335 |    9.99e-7 |
    // | nurbs u+v (knots)        |  84,588 |  7,050 |  4,170 |  2,880 |    9.99e-7 |
    //
    // In-repo, 92.4% of all movement (88,852 of 96,117) is against a REAL knot
    // vector and is untouched by this change. **On the corpus the ratio
    // INVERTS**: 45,440 of 58,160 moves -- 78% -- are against a placeholder.
    // The in-repo fixtures are B-spline-heavy scan data; real assemblies are
    // analytic-heavy, so anyone measuring only in-repo would have concluded this
    // was a 7% edge case.
    //
    // The `-> min` column is why the placeholder half looked harmless: most
    // moves snap onto `min = 0`, which is the surface's own parameter ORIGIN --
    // a plane's placement point, a revolution profile's start -- so the write is
    // ~1e-10 and reads as de-noising. It is NOT principled de-noising: the same
    // face's sample at u = 5.0 gets none, and the noise floor is only small
    // because the placement happens to sit on the trim. The other **6,090
    // corpus moves snap onto `max = 1`**, one arbitrary unit along an unbounded
    // direction, which is the same write with none of the excuse.
    //
    // AND THE SQUARE IS NOT A DOMAIN, MEASURED AS SUCH: of the raw solver
    // answers on a placeholder axis, the fraction sitting OUTSIDE the reported
    // range by more than `TOLERANCE` -- where `clamp_near_range` does nothing at
    // all -- is 437,258 of 490,007 planar u in-repo (89.2%) and 274,196 of
    // 291,687 on the corpus (94.0%), reaching `u = 5.0e5` outside `[0, 1]`. A
    // range that four out of five samples violate by five orders of magnitude
    // is not a bound, and clamping the one-in-a-thousand that lands within
    // 1e-6 of it is an accident of where the placement was written.
    //
    // Only NON-PERIODIC axes are dropped. No placeholder axis carries a period
    // today (a revolution's turn and a sphere's/torus's angles are all real
    // bounds, and a `Line` profile has no period), so the guard is inert --
    // it is written this way so that the periodic wrap arm below, which is
    // selected by `range.is_some()`, provably cannot lose its range. Pinned by
    // `no_placeholder_axis_carries_a_period`.
    //
    // WHAT THIS CHANGE COSTS, measured by per-chain BIT digests of the produced
    // parameter loops, before and after, over both populations: 3,147 of 58,268
    // in-repo chains change (5.4%). NOT ONE changes its point count, and not one
    // flips between `Some` and `None` -- the chains that refused still refuse
    // and the chains that answered still answer, with the same number of
    // samples. The largest movement of any chain's u/v extent is 7.43e-7, and
    // only 19 chains move by more than 1e-9; the mode is ~1e-16, the ULP cloud
    // around the placement origin that used to be snapped flat to `0.0`.
    // Placeholder moves go 7,265 -> 0 and knot moves hold at exactly 88,852.
    let (u_real, v_real) = uv_clamp::reported_range_bounds_the_surface(surface);
    let normalize_uv = |uv: Point2, previous: Option<(f64, f64)>| {
        let (period_u, period_v) = (surface.period_u(), surface.period_v());
        let (urange, vrange) = surface.try_range_tuple();
        uv_clamp::record_point();
        uv_clamp::record_reported_excess(surface, uv_clamp::Axis::U, u_real, uv.x, urange);
        uv_clamp::record_reported_excess(surface, uv_clamp::Axis::V, v_real, uv.y, vrange);
        let urange = urange.filter(|_| u_real || period_u.is_some());
        let vrange = vrange.filter(|_| v_real || period_v.is_some());
        Some(Point2::new(
            normalize_axis(
                uv.x,
                previous.map(|(u, _)| u),
                period_u,
                urange,
                surface,
                uv_clamp::Axis::U,
                u_real,
            )?,
            normalize_axis(
                uv.y,
                previous.map(|(_, v)| v),
                period_v,
                vrange,
                surface,
                uv_clamp::Axis::V,
                v_real,
            )?,
        ))
    };
    let points = curve
        .try_parameter_division(curve.range_tuple(), tolerance)?
        .1;
    let project = |point: Point3, hint: Option<(f64, f64)>| {
        surface
            .search_parameter(point, hint, 100)
            .or_else(|| surface.search_parameter(point, None, 100))
            .or_else(|| surface.search_nearest_parameter(point, hint, 100))
            .or_else(|| surface.search_nearest_parameter(point, None, 100))
            .map(|(u, v)| Point2::new(u, v))
    };
    // ONE PASS, NO RETRY -- and the absence is measured, not an oversight.
    //
    // This chain used to end in `.or_else(|| <the identical hinted scan>)`.
    // `project` and `normalize_uv` are deterministic functions of
    // `(point, hint)`, neither solver keeps state across calls, and the retry
    // re-seeded the hint from the same `None` over the same `points` -- so it
    // recomputed the same sequence and returned the same `None`. Every time.
    //
    // The twin in `monstertruck-modeling/src/geometry.rs`
    // (`sampled_parameter_boundary`) retries differently: it re-projects every
    // point UNHINTED, abandoning the chain. That IS a different computation, and
    // copying it here was the other candidate. Both were measured before either
    // was chosen (spec 011 open item 6), over the 15 in-repo fixtures at full
    // depth plus 7 of the 8 corpus files sampled by solid:
    //
    // | population                       | chains | reach the retry | same-hinted rescues | UNHINTED rescues |
    // |----------------------------------|-------:|----------------:|--------------------:|-----------------:|
    // | 15 in-repo fixtures, all solids  | 58,244 |              14 |                   0 |                0 |
    // | ROTOR-201NAL-Z7, all 33 solids   | 11,810 |               2 |                   0 |                0 |
    // | Rocky_House, 12 of 156           |  8,938 |              11 |                   0 |                0 |
    // | Cruise_Assembly, 12              |    995 |               0 |                   0 |                0 |
    // | UMC-500, 12 of 217               |  3,094 |              52 |                   0 |                0 |
    // | Ai-14R, 4                        |  1,978 |               0 |                   0 |                0 |
    // | NissanGT-R, 4                    |  6,108 |              16 |                   0 |                0 |
    // | Scania-8x4, 4 of 832             |  3,256 |              32 |                   0 |                0 |
    // | **total**                        | 94,423 |         **127** |               **0** |            **0** |
    //
    // So the site was LIVE (127 chains, 0.13%, reached it) and the retry rescued
    // none of them -- as the determinism argument requires. The finding that
    // decided the shape of the fix is the last column: the modeling twin's
    // unhinted retry would have rescued **none of the same 127 either**. The
    // asymmetry between the twins was real, but "the other one is different, so
    // copy it" is not the conclusion -- it would have replaced a retry that
    // cannot rescue with one that measurably does not. Both are dead here; only
    // one of them looks alive.
    //
    // The 127 are not a loss the retry was hiding: the chain refuses typed, and
    // its caller (`monstertruck-solid`'s `reattach_preserved_face_trims`) falls
    // through to its own `sampled_trim_segment`. If a rescue is ever wanted at
    // THIS level, the measurement says it must be a genuinely different solve --
    // not a different hint into the same one.
    uv_clamp::begin_chain();
    let boundary = points
        .iter()
        .copied()
        .scan(None, |hint, point| {
            let uv = project(point, *hint).and_then(|uv| normalize_uv(uv, *hint));
            *hint = uv.map(|uv| (uv.x, uv.y));
            Some(uv)
        })
        .collect();
    uv_clamp::end_chain();
    boundary
}

fn exact_parameter_curve_on(curve: &Curve3D, surface: &Surface) -> Option<StepParameterCurve> {
    match (curve, surface) {
        (Curve3D::ParameterCurve(curve), surface)
            if SurfaceCurve3D::same_surface(curve.surface().as_ref(), surface) =>
        {
            Some(curve.clone())
        }
        (Curve3D::SurfaceCurve(curve), surface) => curve
            .parameter_curve_on(surface)
            .cloned()
            .or_else(|| match surface {
                Surface::ElementarySurface(ElementarySurface::Plane(_)) => {
                    exact_parameter_curve_on(curve.leader(), surface)
                }
                _ => None,
            }),
        (Curve3D::Line(curve), Surface::ElementarySurface(ElementarySurface::Plane(plane))) => {
            exact_line_parameter_curve_on_plane(curve, plane, surface)
        }
        (Curve3D::Conic(curve), surface) => exact_conic_parameter_curve_on(curve, surface),
        (Curve3D::IntersectionCurve(curve), surface)
            if SurfaceCurve3D::same_surface(curve.surface0().as_ref(), surface)
                || SurfaceCurve3D::same_surface(curve.surface1().as_ref(), surface) =>
        {
            exact_parameter_curve_on(curve.leader().as_ref(), surface)
        }
        _ => None,
    }
}

fn projected_conic_transform_on_plane(transform: &Matrix4, plane: &Plane) -> Matrix3 {
    let project = |point| {
        let parameter = plane.parameter(transform.transform_point(point));
        Point2::new(parameter.x, parameter.y)
    };
    let origin = project(Point3::origin());
    let u_axis = project(Point3::new(1.0, 0.0, 0.0)) - origin;
    let v_axis = project(Point3::new(0.0, 1.0, 0.0)) - origin;
    Matrix3::from_cols(
        Vector3::new(u_axis.x, u_axis.y, 0.0),
        Vector3::new(v_axis.x, v_axis.y, 0.0),
        Vector3::new(origin.x, origin.y, 1.0),
    )
}

fn pcurve_matches_surface_curve<C>(curve: &C, trim: &Curve2D, surface: &Surface) -> bool
where C: ParametricCurve3D<Point = Point3> + BoundedCurve {
    let (t0, t1) = curve.range_tuple();
    [
        t0,
        (3.0 * t0 + t1) * 0.25,
        (t0 + t1) * 0.5,
        (t0 + 3.0 * t1) * 0.25,
        t1,
    ]
    .into_iter()
    .all(|parameter| {
        let uv = trim.evaluate(parameter);
        surface
            .evaluate(uv.x, uv.y)
            .near(&curve.evaluate(parameter))
    })
}

fn exact_ellipse_parameter_curve_on_plane(
    curve: &Ellipse<Point3, Matrix4>,
    plane: &Plane,
    surface: &Surface,
) -> Option<StepParameterCurve> {
    let trimmed = TrimmedCurve::new(UnitCircle::new(), curve.entity().range());
    let mut projected = Processor::with_transform(
        trimmed,
        projected_conic_transform_on_plane(curve.transform(), plane),
    );
    if !curve.orientation() {
        projected.invert();
    }
    let trim = Curve2D::Conic(Conic2D::Ellipse(projected));
    pcurve_matches_surface_curve(curve, &trim, surface)
        .then(|| StepParameterCurve::new(Box::new(trim), Box::new(surface.clone())))
}

fn exact_hyperbola_parameter_curve_on_plane(
    curve: &Hyperbola<Point3, Matrix4>,
    plane: &Plane,
    surface: &Surface,
) -> Option<StepParameterCurve> {
    let trimmed = TrimmedCurve::new(UnitHyperbola::new(), curve.entity().range());
    let mut projected = Processor::with_transform(
        trimmed,
        projected_conic_transform_on_plane(curve.transform(), plane),
    );
    if !curve.orientation() {
        projected.invert();
    }
    let trim = Curve2D::Conic(Conic2D::Hyperbola(projected));
    pcurve_matches_surface_curve(curve, &trim, surface)
        .then(|| StepParameterCurve::new(Box::new(trim), Box::new(surface.clone())))
}

fn exact_parabola_parameter_curve_on_plane(
    curve: &Parabola<Point3, Matrix4>,
    plane: &Plane,
    surface: &Surface,
) -> Option<StepParameterCurve> {
    let trimmed = TrimmedCurve::new(UnitParabola::new(), curve.entity().range());
    let mut projected = Processor::with_transform(
        trimmed,
        projected_conic_transform_on_plane(curve.transform(), plane),
    );
    if !curve.orientation() {
        projected.invert();
    }
    let trim = Curve2D::Conic(Conic2D::Parabola(projected));
    pcurve_matches_surface_curve(curve, &trim, surface)
        .then(|| StepParameterCurve::new(Box::new(trim), Box::new(surface.clone())))
}

fn line_parameter_curve(line: Line<Point2>, surface: &Surface) -> StepParameterCurve {
    StepParameterCurve::new(Box::new(Curve2D::Line(line)), Box::new(surface.clone()))
}

fn exact_line_parameter_curve_on_plane(
    curve: &Line<Point3>,
    plane: &Plane,
    surface: &Surface,
) -> Option<StepParameterCurve> {
    let front = plane.parameter(curve.front());
    let back = plane.parameter(curve.back());
    let trim = Curve2D::Line(Line(
        Point2::new(front.x, front.y),
        Point2::new(back.x, back.y),
    ));
    pcurve_matches_surface_curve(curve, &trim, surface)
        .then(|| StepParameterCurve::new(Box::new(trim), Box::new(surface.clone())))
}

fn nearest_periodic_component(value: f64, reference: f64, period: Option<f64>) -> f64 {
    period.map_or(value, |period| {
        value + ((reference - value) / period).round() * period
    })
}

fn nearest_periodic_surface_parameter(
    point: Point2,
    reference: Point2,
    periods: (Option<f64>, Option<f64>),
) -> Point2 {
    Point2::new(
        nearest_periodic_component(point.x, reference.x, periods.0),
        nearest_periodic_component(point.y, reference.y, periods.1),
    )
}

fn exact_line_parameter_curve_by_surface_search<C>(
    curve: &C,
    surface: &Surface,
) -> Option<StepParameterCurve>
where
    C: ParametricCurve3D<Point = Point3> + BoundedCurve,
{
    let (t0, t1) = curve.range_tuple();
    let periods = (surface.period_u(), surface.period_v());
    let samples = [
        t0,
        (3.0 * t0 + t1) * 0.25,
        (t0 + t1) * 0.5,
        (t0 + 3.0 * t1) * 0.25,
        t1,
    ]
    .into_iter()
    .try_fold(Vec::with_capacity(5), |mut samples, parameter| {
        let point = curve.evaluate(parameter);
        let hint = samples
            .last()
            .map(|(_, uv): &(Point3, Point2)| (*uv).into());
        let uv = surface.search_parameter(point, hint, 30)?;
        let uv = Point2::from(uv);
        let uv = samples
            .last()
            .map(|(_, reference): &(Point3, Point2)| {
                nearest_periodic_surface_parameter(uv, *reference, periods)
            })
            .unwrap_or(uv);
        samples.push((point, uv));
        Some(samples)
    })?;
    let line = Line(samples.first()?.1, samples.last()?.1);
    (!line.0.near(&line.1)).then_some(())?;
    samples
        .iter()
        .all(|(point, uv)| {
            line.search_nearest_parameter(*uv, None, 1)
                .filter(|parameter| *parameter >= -TOLERANCE && *parameter <= 1.0 + TOLERANCE)
                .map(|parameter| line.evaluate(parameter.clamp(0.0, 1.0)))
                .is_some_and(|projected| {
                    projected.distance2(*uv) <= TOLERANCE * TOLERANCE
                        && surface.evaluate(projected.x, projected.y).near(point)
                })
        })
        .then(|| line_parameter_curve(line, surface))
}

fn exact_conic_parameter_curve_on(
    curve: &Conic3D,
    surface: &Surface,
) -> Option<StepParameterCurve> {
    match (curve, surface) {
        (Conic3D::Ellipse(curve), Surface::ElementarySurface(ElementarySurface::Plane(plane))) => {
            exact_ellipse_parameter_curve_on_plane(curve, plane, surface)
        }
        (
            Conic3D::Hyperbola(curve),
            Surface::ElementarySurface(ElementarySurface::Plane(plane)),
        ) => exact_hyperbola_parameter_curve_on_plane(curve, plane, surface),
        (Conic3D::Parabola(curve), Surface::ElementarySurface(ElementarySurface::Plane(plane))) => {
            exact_parabola_parameter_curve_on_plane(curve, plane, surface)
        }
        (Conic3D::Ellipse(curve), _) => {
            exact_line_parameter_curve_by_surface_search(curve, surface)
        }
        _ => None,
    }
}

fn to_modeling_trim(
    curve: &StepParameterCurve,
) -> std::result::Result<ParameterCurve<ModelingCurve2D, Box<ModelingSurface>>, StepConvertingError>
{
    Ok(ParameterCurve::new(
        curve.curve().as_ref().try_into()?,
        Box::new(curve.surface().as_ref().try_into()?),
    ))
}

impl SurfaceCurveAssociatedGeometry {
    fn surface(&self) -> &Surface {
        match self {
            SurfaceCurveAssociatedGeometry::ParameterCurve(curve) => curve.surface().as_ref(),
            SurfaceCurveAssociatedGeometry::Surface(surface) => surface,
        }
    }
}

impl ParametricCurve for SurfaceCurve3D {
    type Point = Point3;
    type Vector = Vector3;

    fn evaluate(&self, t: f64) -> Self::Point { self.leader().evaluate(t) }

    fn derivative(&self, t: f64) -> Self::Vector { self.leader().derivative(t) }

    fn derivative_2(&self, t: f64) -> Self::Vector { self.leader().derivative_2(t) }

    fn derivative_n(&self, n: usize, t: f64) -> Self::Vector { self.leader().derivative_n(n, t) }

    fn parameter_range(&self) -> ParameterRange { self.leader().parameter_range() }

    fn period(&self) -> Option<f64> { self.leader().period() }
}

impl BoundedCurve for SurfaceCurve3D {}

impl ParameterDivision1D for SurfaceCurve3D {
    type Point = Point3;

    fn try_parameter_division(
        &self,
        range: (f64, f64),
        tol: f64,
    ) -> Option<(Vec<f64>, Vec<Self::Point>)> {
        self.leader().try_parameter_division(range, tol)
    }

    fn parameter_division(&self, range: (f64, f64), tol: f64) -> (Vec<f64>, Vec<Self::Point>) {
        self.leader().parameter_division(range, tol)
    }
}

impl SurfaceCurveAssociatedGeometry {
    fn split_at(&mut self, t: f64) -> Self {
        match self {
            SurfaceCurveAssociatedGeometry::ParameterCurve(curve) => {
                SurfaceCurveAssociatedGeometry::ParameterCurve(curve.cut(t))
            }
            SurfaceCurveAssociatedGeometry::Surface(surface) => {
                SurfaceCurveAssociatedGeometry::Surface(surface.clone())
            }
        }
    }
}

impl Cut for SurfaceCurve3D {
    fn cut(&mut self, t: f64) -> Self {
        let leader = Box::new(self.leader_mut().cut(t));
        let associated_geometry = self
            .associated_geometry
            .iter_mut()
            .map(|entry| entry.split_at(t))
            .collect();
        Self::new(
            self.kind(),
            leader,
            associated_geometry,
            self.master_representation(),
        )
    }
}

impl SnapCurveEndpoints for SurfaceCurve3D {
    fn snap_endpoints(&mut self, front: Point3, back: Point3) {
        self.leader_mut().snap_endpoints(front, back);
    }
}

impl SnapCurveEndpoints for Curve3D {
    fn snap_endpoints(&mut self, front: Point3, back: Point3) {
        match self {
            Curve3D::Polyline(curve) => curve.snap_endpoints(front, back),
            Curve3D::SurfaceCurve(curve) => curve.snap_endpoints(front, back),
            Curve3D::IntersectionCurve(curve) => curve.snap_endpoints(front, back),
            Curve3D::Line(_)
            | Curve3D::Conic(_)
            | Curve3D::BsplineCurve(_)
            | Curve3D::ParameterCurve(_)
            | Curve3D::NurbsCurve(_) => {}
        }
    }
}

impl Invertible for SurfaceCurveAssociatedGeometry {
    fn invert(&mut self) {
        if let SurfaceCurveAssociatedGeometry::ParameterCurve(curve) = self {
            curve.invert();
        }
    }
}

impl Invertible for SurfaceCurve3D {
    fn invert(&mut self) {
        self.leader_mut().invert();
        self.associated_geometry
            .iter_mut()
            .for_each(Invertible::invert);
    }
}

impl SearchParameter<CurveParameter> for SurfaceCurve3D {
    type Point = Point3;

    fn search_parameter<H: Into<SearchParameterHint1D>>(
        &self,
        point: Self::Point,
        hint: H,
        trials: usize,
    ) -> Option<f64> {
        self.leader().search_parameter(point, hint, trials)
    }
}

impl SearchNearestParameter<CurveParameter> for SurfaceCurve3D {
    type Point = Point3;

    fn search_nearest_parameter<H: Into<SearchParameterHint1D>>(
        &self,
        point: Self::Point,
        hint: H,
        trials: usize,
    ) -> Option<f64> {
        self.leader().search_nearest_parameter(point, hint, trials)
    }
}

impl Transformed<Matrix4> for SurfaceCurveAssociatedGeometry {
    fn transform_by(&mut self, trans: Matrix4) {
        match self {
            SurfaceCurveAssociatedGeometry::ParameterCurve(curve) => curve.transform_by(trans),
            SurfaceCurveAssociatedGeometry::Surface(surface) => surface.transform_by(trans),
        }
    }
}

impl Transformed<Matrix4> for SurfaceCurve3D {
    fn transform_by(&mut self, trans: Matrix4) {
        self.leader_mut().transform_by(trans);
        self.associated_geometry
            .iter_mut()
            .for_each(|entry| entry.transform_by(trans));
    }
}

impl ParameterDivision1D for Curve3D {
    type Point = Point3;

    fn try_parameter_division(
        &self,
        range: (f64, f64),
        tol: f64,
    ) -> Option<(Vec<f64>, Vec<Self::Point>)> {
        let debug_profile = env::var("MT_PROFILE_CURVE_DIVISION").is_ok();
        // Only consult the clock when actually profiling -- `Instant::now()`
        // panics on `wasm32-unknown-unknown` ("time not implemented"), so
        // an unconditional call here breaks browser STEP loading.
        let started = debug_profile.then(Instant::now);
        let result = match self {
            Curve3D::Line(curve) => curve.try_parameter_division(range, tol),
            Curve3D::Polyline(curve) => curve.try_parameter_division(range, tol),
            Curve3D::Conic(curve) => curve.try_parameter_division(range, tol),
            Curve3D::BsplineCurve(curve) => curve.try_parameter_division(range, tol),
            Curve3D::ParameterCurve(curve) => curve.try_parameter_division(range, tol),
            Curve3D::SurfaceCurve(curve) => curve.try_parameter_division(range, tol),
            Curve3D::IntersectionCurve(curve) => curve.leader().try_parameter_division(range, tol),
            Curve3D::NurbsCurve(curve) => curve.try_parameter_division(range, tol),
        };
        if let Some(started) = started {
            let kind = match self {
                Curve3D::Line(_) => "Line",
                Curve3D::Polyline(_) => "Polyline",
                Curve3D::Conic(_) => "Conic",
                Curve3D::BsplineCurve(_) => "BsplineCurve",
                Curve3D::ParameterCurve(_) => "StepParameterCurve",
                Curve3D::SurfaceCurve(_) => "SurfaceCurve",
                Curve3D::IntersectionCurve(_) => "IntersectionCurve",
                Curve3D::NurbsCurve(_) => "NurbsCurve",
            };
            eprintln!(
                "trace bool curve_division kind={} points={} tol={} elapsed_ms={:.3}",
                kind,
                result.as_ref().map_or(0, |(_, points)| points.len()),
                tol,
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
        result
    }

    fn parameter_division(&self, range: (f64, f64), tol: f64) -> (Vec<f64>, Vec<Self::Point>) {
        let debug_profile = env::var("MT_PROFILE_CURVE_DIVISION").is_ok();
        // Same wasm-safety guard as `try_parameter_division` above.
        let started = debug_profile.then(Instant::now);
        let result = match self {
            Curve3D::Line(curve) => curve.parameter_division(range, tol),
            Curve3D::Polyline(curve) => curve.parameter_division(range, tol),
            Curve3D::Conic(curve) => curve.parameter_division(range, tol),
            Curve3D::BsplineCurve(curve) => curve.parameter_division(range, tol),
            Curve3D::ParameterCurve(curve) => curve.parameter_division(range, tol),
            Curve3D::SurfaceCurve(curve) => curve.parameter_division(range, tol),
            Curve3D::IntersectionCurve(curve) => curve.leader().parameter_division(range, tol),
            Curve3D::NurbsCurve(curve) => curve.parameter_division(range, tol),
        };
        if let Some(started) = started {
            let kind = match self {
                Curve3D::Line(_) => "Line",
                Curve3D::Polyline(_) => "Polyline",
                Curve3D::Conic(_) => "Conic",
                Curve3D::BsplineCurve(_) => "BsplineCurve",
                Curve3D::ParameterCurve(_) => "StepParameterCurve",
                Curve3D::SurfaceCurve(_) => "SurfaceCurve",
                Curve3D::IntersectionCurve(_) => "IntersectionCurve",
                Curve3D::NurbsCurve(_) => "NurbsCurve",
            };
            eprintln!(
                "trace bool curve_division kind={} points={} tol={} elapsed_ms={:.3}",
                kind,
                result.1.len(),
                tol,
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
        result
    }
}

impl ToSameGeometry<Curve3D> for SurfaceCurve3D {
    fn to_same_geometry(&self) -> Curve3D { Curve3D::SurfaceCurve(self.clone()) }
}

impl From<IntersectionCurve<BsplineCurve<Point3>, Surface, Surface>> for Curve3D {
    fn from(ic: IntersectionCurve<BsplineCurve<Point3>, Surface, Surface>) -> Self {
        let (surface0, surface1, leader) = ic.destruct();
        Curve3D::IntersectionCurve(IntersectionCurve::new(
            Box::new(surface0),
            Box::new(surface1),
            Box::new(Curve3D::BsplineCurve(leader)),
        ))
    }
}

impl TryIntoHomogeneousBsplineSurface for Sphere {
    fn try_into_homogeneous_bspline_surface(&self) -> Option<BsplineSurface<Vector4>> {
        self.0.try_into_homogeneous_bspline_surface()
    }

    fn try_into_homogeneous_bspline_surface_over(
        &self,
        parameter_range: Option<SurfaceParameterRectangle>,
    ) -> Option<HomogeneousSurfaceConversion> {
        self.0
            .try_into_homogeneous_bspline_surface_over(parameter_range)
    }
}

impl TryIntoBsplineSurface for Sphere {
    fn try_into_bspline_surface(&self) -> Option<BsplineSurface<Point3>> {
        self.0.try_into_bspline_surface()
    }
}

impl TryIntoHomogeneousBsplineCurve for Curve3D {
    fn try_into_homogeneous_bspline_curve(&self) -> Option<BsplineCurve<Vector4>> {
        match self {
            Curve3D::Line(line) => line.try_into_homogeneous_bspline_curve(),
            Curve3D::Conic(Conic3D::Ellipse(curve)) => curve.try_into_homogeneous_bspline_curve(),
            Curve3D::Conic(_) => None,
            Curve3D::BsplineCurve(curve) => curve.try_into_homogeneous_bspline_curve(),
            Curve3D::ParameterCurve(_) => None,
            Curve3D::Polyline(_) => None,
            Curve3D::SurfaceCurve(curve) => curve.leader().try_into_homogeneous_bspline_curve(),
            Curve3D::IntersectionCurve(curve) => {
                curve.leader().try_into_homogeneous_bspline_curve()
            }
            Curve3D::NurbsCurve(curve) => curve.try_into_homogeneous_bspline_curve(),
        }
    }

    fn try_into_homogeneous_bspline_curve_over(
        &self,
        range: (f64, f64),
    ) -> Option<BsplineCurve<Vector4>> {
        match self {
            // Only a line has an exact analytic continuation past its own range;
            // every other variant keeps the trait's refusing default.
            Curve3D::Line(line) => line.try_into_homogeneous_bspline_curve_over(range),
            _ => None,
        }
    }
}

impl ParameterBoundary2D<Surface> for Curve3D {
    fn parameter_boundary_2d(&self, surface: &Surface, tolerance: f64) -> Option<Vec<Point2>> {
        match self {
            Curve3D::ParameterCurve(curve) => {
                if curve.surface().as_ref() == surface {
                    curve
                        .curve()
                        .try_parameter_division(curve.curve().range_tuple(), tolerance)
                        .map(|(_, points)| points)
                } else {
                    sampled_parameter_boundary(curve, surface, tolerance)
                }
            }
            Curve3D::SurfaceCurve(curve) => curve
                .parameter_curve_on(surface)
                .and_then(|parameter_curve| {
                    parameter_curve
                        .curve()
                        .try_parameter_division(parameter_curve.curve().range_tuple(), tolerance)
                        .map(|(_, points)| points)
                })
                .or_else(|| sampled_parameter_boundary(curve.leader(), surface, tolerance)),
            Curve3D::IntersectionCurve(curve) => {
                exact_parameter_curve_on(curve.leader().as_ref(), surface)
                    .and_then(|parameter_curve| {
                        parameter_curve
                            .curve()
                            .try_parameter_division(
                                parameter_curve.curve().range_tuple(),
                                tolerance,
                            )
                            .map(|(_, points)| points)
                    })
                    .or_else(|| {
                        sampled_parameter_boundary(curve.leader().as_ref(), surface, tolerance)
                    })
                    .or_else(|| {
                        curve
                            .leader()
                            .try_parameter_division(curve.range_tuple(), tolerance)?
                            .0
                            .into_iter()
                            .map(|t| {
                                let (_, uv0, uv1) = curve.search_triple(t, 100)?;
                                if curve.surface0().as_ref() == surface {
                                    Some(uv0)
                                } else if curve.surface1().as_ref() == surface {
                                    Some(uv1)
                                } else {
                                    None
                                }
                            })
                            .collect::<Option<Vec<_>>>()
                    })
            }
            Curve3D::Line(_)
            | Curve3D::Polyline(_)
            | Curve3D::Conic(_)
            | Curve3D::BsplineCurve(_)
            | Curve3D::NurbsCurve(_) => sampled_parameter_boundary(self, surface, tolerance),
        }
    }
}

impl ExactParameterBoundary2D<Surface> for Curve3D {
    type BoundaryCurve = StepParameterCurve;

    fn exact_parameter_boundary_2d(&self, surface: &Surface) -> Option<Self::BoundaryCurve> {
        CurveTrimRef::new(self, surface).try_into().ok()
    }
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
                .is_some_and(|t| line.subs(t).near(&point))
        });
        if is_linear {
            Some(Curve2D::Line(line))
        } else {
            let denom = (points.len() - 1) as f64;
            let knot_vec = KnotVector::from(
                std::iter::once(0.0)
                    .chain((0..points.len()).map(|index| index as f64 / denom))
                    .chain(std::iter::once(1.0))
                    .collect::<Vec<_>>(),
            );
            Some(Curve2D::BsplineCurve(BsplineCurve::new(knot_vec, points)))
        }
    }
}

impl BoundaryCurveFromSamples<Surface> for StepParameterCurve {
    fn boundary_curve_from_samples(surface: &Surface, points: Vec<Point2>) -> Option<Self> {
        curve2d_from_sampled_boundary(points)
            .map(|curve| ParameterCurve::new(Box::new(curve), Box::new(surface.clone())))
    }
}

impl TryIntoBsplineSurface for Surface {
    fn try_into_bspline_surface(&self) -> Option<BsplineSurface<Point3>> {
        match self {
            Surface::ElementarySurface(ElementarySurface::Plane(surface)) => {
                surface.try_into_bspline_surface()
            }
            Surface::ElementarySurface(ElementarySurface::Sphere(surface)) => {
                surface.try_into_bspline_surface()
            }
            Surface::ElementarySurface(ElementarySurface::CylindricalSurface(surface)) => {
                surface.try_into_bspline_surface()
            }
            Surface::ElementarySurface(ElementarySurface::ToroidalSurface(surface)) => {
                surface.try_into_bspline_surface()
            }
            Surface::ElementarySurface(ElementarySurface::ConicalSurface(surface)) => {
                surface.try_into_bspline_surface()
            }
            Surface::SweepSurface(SweepSurface::ExtrusionSurface(surface)) => {
                surface.try_into_bspline_surface()
            }
            Surface::SweepSurface(SweepSurface::RevolutionSurface(surface)) => {
                surface.try_into_bspline_surface()
            }
            Surface::BsplineSurface(surface) => surface.try_into_bspline_surface(),
            Surface::NurbsSurface(surface) => surface.try_into_bspline_surface(),
        }
    }
}

impl TryIntoHomogeneousBsplineSurface for Surface {
    fn try_into_homogeneous_bspline_surface(&self) -> Option<BsplineSurface<Vector4>> {
        match self {
            Surface::ElementarySurface(ElementarySurface::Plane(surface)) => {
                surface.try_into_homogeneous_bspline_surface()
            }
            Surface::ElementarySurface(ElementarySurface::Sphere(surface)) => {
                surface.try_into_homogeneous_bspline_surface()
            }
            Surface::ElementarySurface(ElementarySurface::CylindricalSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface()
            }
            Surface::ElementarySurface(ElementarySurface::ToroidalSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface()
            }
            Surface::ElementarySurface(ElementarySurface::ConicalSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface()
            }
            Surface::SweepSurface(SweepSurface::ExtrusionSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface()
            }
            Surface::SweepSurface(SweepSurface::RevolutionSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface()
            }
            Surface::BsplineSurface(surface) => surface.try_into_homogeneous_bspline_surface(),
            Surface::NurbsSurface(surface) => surface.try_into_homogeneous_bspline_surface(),
        }
    }

    fn try_into_homogeneous_bspline_surface_over(
        &self,
        parameter_range: Option<SurfaceParameterRectangle>,
    ) -> Option<HomogeneousSurfaceConversion> {
        match self {
            Surface::ElementarySurface(ElementarySurface::Plane(surface)) => {
                surface.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::ElementarySurface(ElementarySurface::Sphere(surface)) => {
                surface.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::ElementarySurface(ElementarySurface::CylindricalSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::ElementarySurface(ElementarySurface::ToroidalSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::ElementarySurface(ElementarySurface::ConicalSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::SweepSurface(SweepSurface::ExtrusionSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::SweepSurface(SweepSurface::RevolutionSurface(surface)) => {
                surface.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::BsplineSurface(surface) => {
                surface.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::NurbsSurface(surface) => {
                surface.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
        }
    }
}

impl SupportsExactPatchDomains for Surface {
    fn supports_exact_patch_domains(&self) -> bool {
        matches!(self, Surface::BsplineSurface(_) | Surface::NurbsSurface(_))
    }
}

impl TryFrom<&Curve3D> for ModelingCurve {
    type Error = StepConvertingError;
    fn try_from(value: &Curve3D) -> std::result::Result<Self, Self::Error> {
        match value {
            Curve3D::Line(line) => Ok((*line).into()),
            Curve3D::BsplineCurve(curve) => Ok(curve.clone().into()),
            Curve3D::NurbsCurve(curve) => Ok(curve.clone().into()),
            Curve3D::ParameterCurve(curve) => {
                Ok(ModelingCurve::ParameterCurve(ParameterCurve::new(
                    curve.curve().as_ref().try_into()?,
                    Box::new(curve.surface().as_ref().try_into()?),
                )))
            }
            Curve3D::SurfaceCurve(curve) => {
                let surfaces = curve
                    .associated_geometry()
                    .iter()
                    .map(SurfaceCurveAssociatedGeometry::surface)
                    .collect::<Vec<_>>();
                if surfaces.len() >= 2 {
                    let surface0 = surfaces[0].try_into()?;
                    let surface1 = surfaces[1].try_into()?;
                    let boundary0 = curve
                        .parameter_curve_on(surfaces[0])
                        .cloned()
                        .map(|trim| to_modeling_trim(&trim))
                        .transpose()?;
                    let boundary1 = curve
                        .parameter_curve_on(surfaces[1])
                        .cloned()
                        .map(|trim| to_modeling_trim(&trim))
                        .transpose()?;
                    Ok(ModelingCurve::IntersectionCurve(
                        SurfaceCurve::with_boundaries(
                            Box::new(surface0),
                            Box::new(surface1),
                            Box::new(curve.leader().try_into()?),
                            boundary0,
                            boundary1,
                        ),
                    ))
                } else {
                    curve.leader().try_into()
                }
            }
            Curve3D::IntersectionCurve(curve) => Ok(ModelingCurve::IntersectionCurve(
                SurfaceCurve::with_boundaries(
                    Box::new(curve.surface0().as_ref().try_into()?),
                    Box::new(curve.surface1().as_ref().try_into()?),
                    Box::new(curve.leader().as_ref().try_into()?),
                    None,
                    None,
                ),
            )),
            _ => value
                .try_into_homogeneous_bspline_curve()
                .map(|curve| ModelingCurve::NurbsCurve(NurbsCurve::new(curve)))
                .ok_or_else(|| "STEP curve cannot be represented in modeling geometry.".into()),
        }
    }
}

impl TryFrom<&Conic2D> for ModelingConic2D {
    type Error = StepConvertingError;
    fn try_from(value: &Conic2D) -> std::result::Result<Self, Self::Error> {
        match value {
            Conic2D::Ellipse(curve) => Ok((*curve).into()),
            Conic2D::Hyperbola(curve) => Ok((*curve).into()),
            Conic2D::Parabola(curve) => Ok((*curve).into()),
        }
    }
}

impl TryFrom<&Curve2D> for ModelingCurve2D {
    type Error = StepConvertingError;
    fn try_from(value: &Curve2D) -> std::result::Result<Self, Self::Error> {
        match value {
            Curve2D::Line(curve) => Ok((*curve).into()),
            Curve2D::Polyline(curve) => Ok(curve.clone().into()),
            Curve2D::Conic(curve) => Ok(ModelingCurve2D::Conic(curve.try_into()?)),
            Curve2D::BsplineCurve(curve) => Ok(curve.clone().into()),
            Curve2D::NurbsCurve(curve) => Ok(curve.clone().into()),
        }
    }
}

impl<'a> TryFrom<SurfaceCurveTrimRef<'a>> for StepParameterCurve {
    type Error = StepConvertingError;

    fn try_from(value: SurfaceCurveTrimRef<'a>) -> std::result::Result<Self, Self::Error> {
        let curve = value.curve();
        let surface = value.surface();
        exact_parameter_curve_on(&Curve3D::SurfaceCurve(curve.clone()), surface)
            .ok_or_else(|| "STEP surface curve has no exact trim on the requested surface.".into())
    }
}

impl<'a> TryFrom<CurveTrimRef<'a>> for StepParameterCurve {
    type Error = StepConvertingError;

    fn try_from(value: CurveTrimRef<'a>) -> std::result::Result<Self, Self::Error> {
        exact_parameter_curve_on(value.curve(), value.surface())
            .ok_or_else(|| "STEP curve has no exact trim on the requested surface.".into())
    }
}

/// Whether the analytic sphere route is admissible, restating the guard in
/// `TryIntoHomogeneousBsplineSurface for Sphere` (`bspline_conversion.rs`).
///
/// The predicate has to be the BUILDER's and no wider: a sphere that fails it
/// must reach the generic arm, where the builder returns `None` and the
/// conversion refuses -- which is what it does today.
/// Whether STEP spheres route onto the analytic [`ModelingSurface::SphericalSurface`].
///
/// **`false` -- STILL HELD, but NOT for the reason the previous revision gave.**
/// Ledger class C14.
///
/// # The previous diagnosis, and its falsification
///
/// This arm was reverted on 2026-07-31 with the finding "it changed the
/// GEOMETRY, not merely its tessellation cost", bisected on
/// `rotor_sphere_pin_a_union_refuses_ambiguous_topology_sign`, which reads ROTOR
/// #19264's extracted mesh volume against a pinned `551.5116` and sees
/// `338.6367` once the arm is on.
///
/// **That reading was FALSIFIED by measurement on 2026-07-31.** The geometry did
/// change -- and it changed toward being CORRECT. `551.5116` is not the solid's
/// volume and is not a converged quantity.
///
/// # What the solids actually are, and their closed forms
///
/// Both sphere pins are the same shape, read off their own loaded boundary
/// geometry rather than assumed: a sphere of radius `R = 12.5` centred at the
/// origin, cut by planes at `x = +-h`, with a bore of radius `r` along the
/// x-axis (#19264: `h = 8`, `r = 7.5`; #25387: `h = 9`, `r = 6`). Two
/// half-sphere faces split at `z = 0`, two half-bore faces, two end annuli.
/// The trim circles land at `sqrt(R^2 - h^2)` = `9.604686` / `8.674676`, which
/// is what the loaded edges report to 15 digits.
///
/// By Archimedes' hat-box theorem ONE half-sphere face's contribution to the
/// divergence-theorem x-flux is exactly `pi * 2 h^3 / 3` -- `1072.3303` and
/// `1526.8140` -- and its area is exactly `2 pi R h` = `628.3185` / `706.8583`.
/// Neither depends on the mesh. Measured per face:
///
/// | route | face | x-flux | vs closed form | area |
/// |---|---|---|---|---|
/// | net (`false`) | #19264 z<0 | 1075.6349 | +0.31% | 627.7081 |
/// | net (`false`) | #19264 z>0 | 1272.1296 | **+18.63%** | **265.9786** |
/// | analytic (`true`) | #19264 both | 1067.4449 | -0.46% | 627.4176 |
/// | net (`false`) | #25387 z<0 | 1528.8421 | +0.13% | -- |
/// | net (`false`) | #25387 z>0 | 2000.9617 | **+31.06%** | -- |
/// | analytic (`true`) | #25387 both | 1519.4105 | -0.49% | 705.6795 |
///
/// Refining the ANALYTIC arm walks each face monotonically up to its closed form
/// from below, as an inscribed mesh must (#19264: `1067.4449 -> 1069.6015 ->
/// 1070.5880` against `1072.3303`; areas `627.4176 -> 627.9058 -> 628.0886`
/// against `628.3185`). Refining the NET arm does not converge at all: its bad
/// face goes `1272.1296 -> 1018.5026`, from 18.6% above the closed form to 5.0%
/// below it, and the whole-shell sum goes `551.5116 -> 281.4251`.
///
/// # The mechanism: TRIM INTERPRETATION, on the rational net
///
/// Under the net route one of the two half-sphere faces -- the one straddling
/// the net's periodic seam -- is triangulated over the wrong sub-region of
/// parameter space and covers **42% of its own area** (`265.98` of `628.32`),
/// which refinement lifts only to 45%. Face ORIENTATION is not the mechanism
/// (both routes give the same sign on both faces), and nothing is dropped or
/// duplicated (`face_drop_count() == 0`, six faces, closed shell, on both
/// routes). The analytic route does not have the defect because it never leaves
/// the closed form.
///
/// # The arm is no longer held -- it was re-landed at `d212e597`
///
/// It had been held only because switching it on moved `551.5116` and
/// `1323.4471`, `CorpusSolid::volume` IDENTITY pins in `corpus_boolean_rows.rs`,
/// and re-pinning them was an owner decision rather than a code fix. That
/// decision was taken; the constant below is `true`. This section is kept
/// because the reasoning is what the re-landing rested on.
///
/// # Two further, INDEPENDENT defects this work uncovered (not C14)
///
/// The instrument both sides of C14 were argued with -- a divergence-theorem sum
/// over the whole shell -- was not a volume on these solids. `occt-sphere.step`
/// measured `-523.58` on a `+523.5988` ball and `occt-cube.step` `-1000` on a
/// 1000-volume cube, while `primitive::cuboid` measured `+24` exactly on a
/// 2x3x4 box and `occt-cylinder` / `occt-cone` / `occt-torus` were all correct
/// and positive. Bit-identical with this switch on and off, so it predated and
/// survived the routing question.
///
/// **Spec 013 V1 found TWO mechanisms behind that, not one, and the assumption
/// that the cube and the sphere shared a defect was wrong.**
///
/// 1. **C15 proper, the cube and ROTOR #19264's annuli/bore faces.** A STEP
///    `ADVANCED_FACE`'s loops are oriented about the FACE normal, but
///    `CompressedFace` stores boundaries in the SURFACE sense; the loader passed
///    them through, so every `same_sense = .F.` face was traversed backwards and
///    the shell loaded `Regular`. Fixed in `Table::absolute_bound_orientation`
///    (`load/convert.rs`), with the symmetric `FACE_BOUND` flag on the save side.
/// 2. **A meshing defect, the sphere.** `occt-sphere` is a ONE-face shell, so it
///    cannot have an inconsistent orientation and never did. Its winding was
///    wrong because `ensure_winding_matches_normals` normalizes each face
///    normal, a sphere's pole strip is degenerate, and one `0/0 = NaN` term made
///    `vote < 0.0` false. Fixed in `monstertruck-meshing`.
///
/// The oracle for both is `occt_sphere_extracts_to_the_analytic_ball` (now the
/// SIGNED closed form) plus `monstertruck-healing/tests/step_shell_orientation.rs`.
///
/// # The oracle, and the proof it discriminates
///
/// `rotor_sphere_faces_carry_their_closed_form_x_flux` (`corpus_boolean_rows.rs`)
/// asserts each half-sphere face against `pi * 2 h^3 / 3` in a 1% band.
/// Measured both ways: it FAILS on the net route (+18.632% / +31.055%) and
/// PASSES on the analytic route (-0.456% / -0.485%). While this arm is held the
/// row additionally admits the ONE named net-route value per solid, so the tree
/// stays green; `MT_C14_FORCE_ANALYTIC_BAND=1` drops that escape and reproduces
/// the failure on demand.
///
/// The TORUS sibling below stays ON for the same kind of reason it always did:
/// it carries an analytic oracle
/// (`occt_torus_intersection_with_an_enclosing_box_is_the_torus`, volume
/// 789.5072 against the closed form `2*PI^2*R*r^2 = 789.5684`) and passes it.
///
/// TO RE-LAND: re-pin `ROTOR_SPHERE_PIN_A::volume` and `ROTOR_SPHERE_PIN_B::volume`
/// to the analytic arm's measurements, flip this constant, and confirm the
/// closed-form row above. The display win is real (ROTOR #19264 UNBOUNDED at
/// chord 1e-3 -> 5.1 s).
///
/// Public so a test in another crate can assert WHICH surface a loaded sphere
/// face reached the kernel as, in BOTH states of this switch -- an oracle that
/// does not know which route it measured cannot certify either one.
pub const ROUTE_ANALYTIC_SPHERE: bool = true;

fn analytic_sphere_is_representable(sphere: &monstertruck_geometry::prelude::Sphere) -> bool {
    let radius = sphere.radius();
    radius.is_finite() && radius > TOLERANCE
}

/// Whether the analytic torus route is admissible.
///
/// Restates `TryIntoHomogeneousBsplineSurface for Torus`
/// (`bspline_conversion.rs`) verbatim, INCLUDING its spindle rejection. Spec
/// 011 T1: on a spindle torus (`small - large` above the relative tolerance)
/// the surface passes through itself and `search_parameter` is silently wrong
/// on roughly a third of the domain, so the class refuses typed. Horn tori
/// (`large == small`, including the near-horn fillets real STEP files carry a
/// few ulps below) ARE representable and must keep converting.
fn analytic_torus_is_representable(torus: &Torus) -> bool {
    let (large_radius, small_radius) = (torus.large_radius(), torus.small_radius());
    large_radius.is_finite()
        && small_radius.is_finite()
        && small_radius > TOLERANCE
        && large_radius > TOLERANCE
        && small_radius - large_radius <= TOLERANCE * (large_radius + small_radius)
}

/// STEP surface -> modeling surface.
///
/// Cylinders and cones map onto the ANALYTIC
/// [`ModelingSurface::RevolutionSurface`] variant rather than being flattened.
/// The flattening path is lossy in a way nothing downstream can undo: a STEP
/// `CYLINDRICAL_SURFACE` is a revolution of a UNIT-LENGTH profile line, so the
/// untrimmed homogeneous conversion emits a control net spanning ONE AXIAL UNIT
/// of an unbounded surface, and no consumer can widen a 4x2 rational net back
/// out to the extent the face actually occupies. The kernel then reports a
/// CONFIDENT empty for face pairs that demonstrably intersect -- 92 of boxy's
/// 126 pairs before this mapping (spec 010, T22).
///
/// Measured on the boxy union: `OK curves=0` falls 100 -> 59 over the 126-pair
/// census with **no pair regressing** (all 26 already-tracing pairs
/// byte-identical), and six pairs move from a silent `Ok(vec![])` to an honest
/// `SsiFailed`. The alternative of keeping the NURBS representation and
/// re-spanning it over each face's trims was measured and REJECTED: it emits
/// one surface carrying two parameter conventions -- the angular axis
/// renormalized to `[0, 1]` while the axial axis keeps model-space knots -- so
/// the angular origin is unrecoverable, and it regressed six pairs from two
/// traced curves to zero. See `FIX_PLAN_010_PRODUCER_TRACK.md` sections 7m/7n.
///
/// Flipping to the analytic variant moves `supports_exact_patch_domains` to
/// `false` and `parameter_range` to `((0, 1), (0, 2pi))` for these surfaces,
/// which is what lets the broad phase see their true extent. The save side
/// already round-trips this variant back to a STEP `CYLINDRICAL_SURFACE`
/// (`save/geometry.rs`), so it improves save fidelity rather than costing it.
///
/// # Spheres and tori (spec 012 U1.2), same shape, different axis
///
/// T22 above was about the emitted net's EXTENT. Spheres and tori never had
/// that defect -- their dedicated rational builders are machine-exact over the
/// whole domain (ledger C1, "not this class", 7y) -- but routing them through
/// [`TryIntoHomogeneousBsplineSurface`] threw away something else the analytic
/// form carries: their CLOSED-FORM [`ParameterDivision2D`]
/// (`specifieds/sphere.rs`, `specifieds/torus.rs`). The generic net divider
/// then has to discover a sphere's curvature by adaptive bisection.
///
/// Measured over ROTOR's five T4 solids, 169 faces, at the guard's `1e-3`:
/// **35,053 refinement cells on the STEP side against 8,469,082 on the modeling
/// side, and 8,434,029 of that gap -- 99.6% -- is these two classes**, which
/// spend ZERO on the STEP side. A six-face solid (#19264: two spheres, two
/// cylinders, two planes) took 116.3 s to mesh for DISPLAY.
///
/// So they map onto analytic variants too. What that costs, enumerated:
/// `try_into_homogeneous_bspline_surface` on the new variants is the SAME call
/// on the SAME `Processor<_, Matrix4>` this arm used to make eagerly, so the
/// net the boolean prepares is byte-identical and only its construction moved
/// from load time to use time. `supports_exact_patch_domains` flips `true` ->
/// `false`, exactly as T22's flip did. `search_parameter` becomes the analytic
/// inverse instead of a Newton descent on a net. Nothing in the boolean engine,
/// the topology crate or the mesher matches on `Surface`'s variants, so the
/// dispatch cost is confined to `monstertruck-modeling`'s own `geometry.rs`,
/// this crate's save side, and `fillet_impl.rs`.
///
/// # The degenerate torus stays refused (spec 011 T1)
///
/// The refusal for `|large| < small` lives in the BUILDER
/// (`bspline_conversion.rs`), so a routing change that stops calling the
/// builder would silently reopen it -- and it must not: on a spindle the
/// FORWARD map is exact to 8e-16 while `search_parameter` is wrong on ~29% of
/// the domain, which is what places trims. [`analytic_torus_is_representable`]
/// therefore restates the builder's predicate verbatim, and a torus that fails
/// it falls through to the generic arm, where the builder returns `None` and
/// the conversion refuses exactly as it does today. Pinned by
/// `spindle_torus_parameter_recovery_is_unsound_while_ring_and_horn_are_exact`
/// (`monstertruck-geometry/tests/torus.rs`) and by
/// `a_spindle_torus_is_still_refused_by_the_analytic_route` below.
impl TryFrom<&Surface> for ModelingSurface {
    type Error = StepConvertingError;
    fn try_from(value: &Surface) -> std::result::Result<Self, Self::Error> {
        match value {
            Surface::ElementarySurface(ElementarySurface::Plane(surface)) => Ok((*surface).into()),
            // Both arrive as `Processor<RevolutionSurface<Line<Point3>>, Matrix4>`,
            // so one or-pattern binds them. `map_ref` carries the `Matrix4` and
            // the processor's orientation across untouched; only the profile is
            // lifted into the modeling curve enum.
            Surface::ElementarySurface(
                ElementarySurface::CylindricalSurface(surface)
                | ElementarySurface::ConicalSurface(surface),
            ) => Ok(ModelingSurface::RevolutionSurface(surface.map_ref(
                |revolution| {
                    RevolutionSurface::by_revolution(
                        ModelingCurve::Line(*revolution.entity_curve()),
                        revolution.origin(),
                        revolution.axis(),
                    )
                },
            ))),
            // `map_ref` again: the `Matrix4` and the orientation flag ride
            // across untouched, only the STEP newtype's `(u, v)` relabeling is
            // dropped. That relabeling has Jacobian determinant +1, so the
            // composite orientation -- and therefore the surface normal -- is
            // unchanged either way. Face trims cannot be affected: they are
            // ERASED (`TrimmedSolid::erase_trims`) before the geometry is
            // mapped, and every consumer re-derives `(u, v)` by projecting the
            // 3D boundary onto the modeling surface.
            Surface::ElementarySurface(ElementarySurface::Sphere(surface))
                if ROUTE_ANALYTIC_SPHERE
                    && analytic_sphere_is_representable(&surface.entity().0) =>
            {
                Ok(ModelingSurface::SphericalSurface(
                    surface.map_ref(|sphere| sphere.0),
                ))
            }
            // TORUS ROUTING, spec 012 W1: the sibling of the sphere arm above,
            // and it was HELD BACK behind an `if false &&` for one round.
            //
            // The stated reason to hold was that switching it on MOVED ap224's
            // pinned refusal from `UnknownClassificationFailed` to
            // `CreateLoopsStoreFailed{IntersectionCurvesFailed{(15,4),
            // SsiFailed}}`, read as "SSI cannot intersect the analytic torus
            // where it could intersect the NURBS form". **That reading was
            // FALSIFIED by measurement.** With
            // `MT_SSI_DEBUG_EXCLUSIONS`/`MT_SSI_DEBUG_TRIM_FILTER` on the
            // failing pair, the SSI backend tested 16 patch pairs, passed 2,
            // and traced 2 core curves -- the RIGHT ones. The SSI was fine.
            // What failed was the FACE: `trim_rejected=2`, `side0=0` on all 8
            // segments, i.e. every traced point tested outside face 15's own
            // parameter loop, because `Torus::search_parameter` discarded the
            // caller's hint and spelled the seam vertex a whole period away
            // from its neighbours. Ledger class C4, fixed at the source
            // (`monstertruck-geometry/src/specifieds/torus.rs`,
            // `nearest_periodic_angle`), and with it:
            //
            //   * ap224 face 15's `u` trim range: `(0.0645, 6.2832)` -- the
            //     whole ring -- becomes `(0, PI)`, and face 19's becomes
            //     `(PI, 2 PI)`. Both now agree with what the same faces report
            //     over the rational net, to the padding.
            //   * ZERO SSI face-pair errors on the ap224 union, and
            //     `ap224_main_solid_union_refuses_typed` passes on its
            //     ORIGINAL pin, `UnknownClassificationFailed{shell_index: 1}`.
            //     Nothing had to be re-pinned.
            //
            // Coverage, the other reason it was held: `occt_torus_*` in
            // `user_fixture_boolean_tests` now carries a torus-bearing boolean
            // that SUCCEEDS against a closed-form volume, so the capability is
            // covered rather than only its refusal.
            //
            // The guard below is the T1 spindle predicate restated verbatim, so
            // spec 011's degenerate-torus refusal survives this unchanged.
            Surface::ElementarySurface(ElementarySurface::ToroidalSurface(surface))
                if analytic_torus_is_representable(surface.entity()) =>
            {
                // `*`, not `.clone()`: `Processor<Torus, Matrix4>` is `Copy`.
                // The dead arm carried the clone unnoticed -- clippy does not
                // lint through an `if false` guard, which is one more reason a
                // held-back arm is not a free thing to leave lying around.
                Ok(ModelingSurface::ToroidalSurface(*surface))
            }
            _ => value
                .try_into_homogeneous_bspline_surface()
                .map(|surface| ModelingSurface::NurbsSurface(NurbsSurface::new(surface)))
                .or_else(|| {
                    value
                        .try_into_bspline_surface()
                        .map(ModelingSurface::BsplineSurface)
                })
                .ok_or_else(|| "STEP surface cannot be represented in modeling geometry.".into()),
        }
    }
}

impl ToSameGeometry<Curve2D> for Line<Point2> {
    #[inline]
    fn to_same_geometry(&self) -> Curve2D { Curve2D::Line(*self) }
}

impl ToSameGeometry<Curve2D> for Processor<TrimmedCurve<UnitCircle<Point2>>, Matrix3> {
    #[inline]
    fn to_same_geometry(&self) -> Curve2D { Curve2D::Conic(Conic2D::Ellipse(*self)) }
}

impl ToSameGeometry<Curve2D> for BsplineCurve<Point2> {
    #[inline]
    fn to_same_geometry(&self) -> Curve2D { Curve2D::BsplineCurve(self.clone()) }
}

impl ToSameGeometry<Curve3D> for Line<Point3> {
    #[inline]
    fn to_same_geometry(&self) -> Curve3D { Curve3D::Line(*self) }
}

impl ToSameGeometry<Curve3D> for Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4> {
    #[inline]
    fn to_same_geometry(&self) -> Curve3D { Curve3D::Conic(Conic3D::Ellipse(*self)) }
}

impl ToSameGeometry<Curve3D> for BsplineCurve<Point3> {
    #[inline]
    fn to_same_geometry(&self) -> Curve3D { Curve3D::BsplineCurve(self.clone()) }
}

impl Conic3D {
    pub fn posture(&self) -> Matrix4 {
        match self {
            Conic3D::Ellipse(processor) => *processor.transform(),
            Conic3D::Hyperbola(processor) => *processor.transform(),
            Conic3D::Parabola(processor) => *processor.transform(),
        }
    }
}

impl IncludeCurve<Curve3D> for Plane {
    fn include(&self, curve: &Curve3D) -> bool {
        match curve {
            Curve3D::Line(line) => self.include(line),
            Curve3D::BsplineCurve(bsp) => self.include(bsp),
            Curve3D::NurbsCurve(bsp) => self.include(bsp),
            Curve3D::Conic(conic) => {
                let mat = conic.posture();
                let axis = mat.z.truncate();
                axis.cross(self.normal()).so_small()
            }
            Curve3D::Polyline(poly) => poly
                .iter()
                .all(|p| self.search_parameter(*p, None, 1).is_some()),
            Curve3D::ParameterCurve(curve) => matches!(
                curve.surface().as_ref(),
                Surface::ElementarySurface(ElementarySurface::Plane(surface)) if self == surface
            ),
            Curve3D::SurfaceCurve(curve) => self.include(curve.leader()),
            Curve3D::IntersectionCurve(curve) => self.include(curve.leader().as_ref()),
        }
    }
}

impl ToSameGeometry<Surface> for Plane {
    #[inline]
    fn to_same_geometry(&self) -> Surface {
        Surface::ElementarySurface(ElementarySurface::Plane(*self))
    }
}

impl ToSameGeometry<Surface> for ExtrusionSurface<Curve3D, Vector3> {
    #[inline]
    fn to_same_geometry(&self) -> Surface {
        Surface::SweepSurface(SweepSurface::ExtrusionSurface(self.clone()))
    }
}

impl ToSameGeometry<Surface> for RevolutionSurface<Curve3D> {
    #[inline]
    fn to_same_geometry(&self) -> Surface {
        let default = || {
            let (curve, origin, axis) = (self.entity_curve().inverse(), self.origin(), self.axis());
            let processor = Processor::new(RevolutionSurface::by_revolution(curve, origin, axis));
            Surface::SweepSurface(SweepSurface::RevolutionSurface(processor))
        };
        match self.entity_curve() {
            Curve3D::Line(line) => {
                let &Line(p, q) = line;
                let v = q - p;
                let axis = self.axis();
                if v.cross(axis).so_small() {
                    let o = self.origin();
                    let origin = o + (p - o).dot(axis) * axis;
                    let revo = RevolutionSurface::by_revolution(*line, origin, axis);
                    let processor = Processor::new(revo);
                    Surface::ElementarySurface(ElementarySurface::CylindricalSurface(processor))
                } else {
                    default()
                }
            }
            Curve3D::SurfaceCurve(_) => default(),
            Curve3D::IntersectionCurve(_) => default(),
            _ => default(),
        }
    }
}

#[test]
fn to_same_geometry_revolution_of_axis_parallel_line_is_uninverted_cylinder() {
    let axis = Vector3::unit_z();
    let center = Point3::new(0.5, -0.5, 0.0);
    let radius = 2.0;
    let p = center + radius * Vector3::unit_x();
    let line = Line(p, p + axis);
    let revolution = RevolutionSurface::by_revolution(Curve3D::Line(line), center, axis);

    let surface = revolution.to_same_geometry();

    match surface {
        Surface::ElementarySurface(ElementarySurface::CylindricalSurface(processor)) => {
            assert!(
                processor.orientation(),
                "cylinder from axis-parallel line must not be inverted.",
            );

            let entity = processor.entity();
            assert_eq!(entity.axis(), axis, "cylinder axis must match.");

            let origin = entity.origin();
            assert_near!(origin.z, 0.0);
            let in_plane = origin - center;
            assert_near!(in_plane.dot(axis), 0.0);

            let profile_distance = (line.0 - origin).magnitude();
            assert_near!(profile_distance, radius);
        }
        other => panic!("expected cylindrical surface, got {other:?}"),
    }
}

#[test]
fn to_same_geometry_2d_line_round_trip() {
    let line = Line(Point2::new(0.0, 0.0), Point2::new(2.0, 1.0));
    let curve: Curve2D = line.to_same_geometry();
    match curve {
        Curve2D::Line(rebuilt) => assert_eq!(rebuilt, line),
        other => panic!("expected Curve2D::Line, got {other:?}"),
    }
}

#[test]
fn to_same_geometry_2d_ellipse_wraps_in_conic() {
    let scale = Matrix3::from_nonuniform_scale(2.0, 3.0);
    let arc = Processor::with_transform(
        TrimmedCurve::new(UnitCircle::<Point2>::new(), (0.0, TAU)),
        scale,
    );
    let curve: Curve2D = arc.to_same_geometry();
    match curve {
        Curve2D::Conic(Conic2D::Ellipse(rebuilt)) => assert_eq!(rebuilt, arc),
        other => panic!("expected Curve2D::Conic(Conic2D::Ellipse), got {other:?}"),
    }
}

#[test]
fn to_same_geometry_2d_bspline_curve_round_trip() {
    let knots = KnotVector::uniform_knot(2, 2);
    let control = vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 0.0),
        Point2::new(3.0, 1.0),
    ];
    let spline = BsplineCurve::new(knots, control);
    let curve: Curve2D = spline.to_same_geometry();
    match curve {
        Curve2D::BsplineCurve(rebuilt) => assert_eq!(rebuilt, spline),
        other => panic!("expected Curve2D::BsplineCurve, got {other:?}"),
    }
}

#[test]
fn sampled_parameter_boundary_preserves_unbounded_cylinder_axis_parameter() {
    let center = Point3::new(0.0, 0.0, 68.0);
    let axis = Vector3::unit_z();
    let radius = 0.3;
    let p = center + radius * Vector3::unit_x();
    let mut cylinder = Processor::new(RevolutionSurface::by_revolution(
        Line(p, p + axis),
        center,
        axis,
    ));
    cylinder.invert();
    let surface = Surface::ElementarySurface(ElementarySurface::CylindricalSurface(cylinder));
    let curve = Line(
        Point3::new(radius, 0.0, -6.25),
        Point3::new(radius, 0.0, 6.25),
    );

    let boundary = sampled_parameter_boundary(&curve, &surface, 0.001).unwrap();
    let max_abs = boundary
        .iter()
        .flat_map(|uv| [uv.x.abs(), uv.y.abs()])
        .fold(0.0, f64::max);

    assert!(max_abs > 10.0);
}

/// Spec 012 U2. A planar trim sample that overshoots the plane's fictional
/// `[0, 1]` square keeps the value the solver produced.
///
/// `Plane::xy()`'s `u` and `v` ARE world `x` and `y`, so this line's far
/// endpoint sits at `u = 1 + 5e-7` -- inside `TOLERANCE` of the reported `max`,
/// which is what `clamp_near_range` used to snap to exactly `1.0`. There is no
/// domain edge at `u = 1` on a plane; the number is a placeholder.
#[test]
fn a_planar_trim_sample_past_the_fictional_unit_square_is_not_snapped_to_it() {
    let surface = Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy()));
    let overshoot = 5.0e-7;
    let curve = Line(
        Point3::new(1.0 - overshoot, 0.5, 0.0),
        Point3::new(1.0 + overshoot, 0.5, 0.0),
    );

    let boundary = sampled_parameter_boundary(&curve, &surface, 1.0e-3).unwrap();
    let last = *boundary.last().unwrap();

    assert!(
        last.x > 1.0,
        "the sample sits past u = 1 and must stay there; got {}",
        last.x,
    );
    assert!(
        (last.x - (1.0 + overshoot)).abs() < 1.0e-12,
        "the projected u must be the solver's answer, not the placeholder \
         boundary; got {}",
        last.x,
    );
}

/// The other half of U2: on a KNOT-bounded surface the clamp is load-bearing and
/// is kept. The net carries no data past the knot vector, so a Newton answer one
/// ULP-cloud past the end has a real boundary to come back to.
///
/// This bilinear patch spans world `[0, 1]^2` in the `z = 0` plane, so a sample
/// at world `x = 1 + 5e-7` extrapolates to `u = 1 + 5e-7` -- and must come back
/// to exactly `1.0`.
#[test]
fn a_knot_bounded_surface_still_snaps_a_sample_that_overshoots_its_knots() {
    let knots = KnotVector::bezier_knot(1);
    let surface = Surface::BsplineSurface(BsplineSurface::new(
        (knots.clone(), knots),
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ],
    ));
    let overshoot = 5.0e-7;
    let curve = Line(
        Point3::new(1.0 - overshoot, 0.5, 0.0),
        Point3::new(1.0 + overshoot, 0.5, 0.0),
    );

    let boundary = sampled_parameter_boundary(&curve, &surface, 1.0e-3).unwrap();

    assert_eq!(
        boundary.last().unwrap().x,
        1.0,
        "a knot vector IS a bound and the clamp must still snap onto it.",
    );
}

/// The guard on the drop is inert, and this is why: no axis whose reported range
/// is a placeholder carries a period. If that ever stops being true the drop
/// would take the range away from the periodic wrap arm, which is selected by
/// `range.is_some()` -- so the invariant is pinned rather than assumed.
#[test]
fn no_placeholder_axis_carries_a_period() {
    let center = Point3::new(0.0, 0.0, 0.0);
    let axis = Vector3::unit_z();
    let profile = Point3::new(0.3, 0.0, 0.0);
    let mut cylinder = Processor::new(RevolutionSurface::by_revolution(
        Line(profile, profile + axis),
        center,
        axis,
    ));
    cylinder.invert();
    let cone = Processor::new(RevolutionSurface::by_revolution(
        Line(profile, profile + axis + Vector3::unit_x()),
        center,
        axis,
    ));

    let surfaces = [
        Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy())),
        Surface::ElementarySurface(ElementarySurface::CylindricalSurface(cylinder)),
        Surface::ElementarySurface(ElementarySurface::ConicalSurface(cone)),
        Surface::ElementarySurface(ElementarySurface::Sphere(Processor::new(Sphere(
            monstertruck_geometry::prelude::Sphere::new(center, 2.0),
        )))),
        Surface::ElementarySurface(ElementarySurface::ToroidalSurface(Processor::new(
            Torus::new(center, 3.0, 1.0),
        ))),
    ];

    for surface in surfaces {
        let (u_real, v_real) = uv_clamp::reported_range_bounds_the_surface(&surface);
        assert!(
            u_real || surface.period_u().is_none(),
            "placeholder u axis with a period: {surface:?}",
        );
        assert!(
            v_real || surface.period_v().is_none(),
            "placeholder v axis with a period: {surface:?}",
        );
    }
}

#[test]
fn raw_conic_boundary_without_pcurve_uses_sampled_projection_at_safe_tolerance() {
    let curve = Curve3D::Conic(Conic3D::Ellipse(
        Processor::new(TrimmedCurve::new(UnitCircle::<Point3>::new(), (0.0, TAU)))
            .transformed(Matrix4::from_nonuniform_scale(100.0, 100.0, 100.0)),
    ));
    let surface = Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy()));

    let boundary = curve
        .parameter_boundary_2d(&surface, 1.0e-3)
        .expect("safe raw conic projection should produce a parameter boundary.");

    assert!(boundary.len() > 4);
    assert!(
        boundary
            .iter()
            .any(|point| point.distance2(Point2::new(100.0, 0.0)) < 1.0e-6)
    );
}

#[test]
fn raw_line_boundary_without_pcurve_uses_sampled_projection() {
    let curve = Curve3D::Line(Line(
        Point3::new(0.25, 0.5, 0.0),
        Point3::new(0.75, 0.5, 0.0),
    ));
    let surface = Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy()));
    let boundary = curve
        .parameter_boundary_2d(&surface, 1.0e-5)
        .expect("safe raw line projection should produce a parameter boundary.");

    assert!(boundary.len() >= 2);
    assert!(
        boundary
            .first()
            .is_some_and(|point| point.near(&Point2::new(0.25, 0.5)))
    );
    assert!(
        boundary
            .last()
            .is_some_and(|point| point.near(&Point2::new(0.75, 0.5)))
    );
}

#[test]
fn surface_curve_line_without_pcurve_converts_to_exact_pcurve() {
    let leader = Curve3D::Line(Line(
        Point3::new(0.25, 0.5, 0.0),
        Point3::new(0.75, 0.5, 0.0),
    ));
    let curve = Curve3D::SurfaceCurve(SurfaceCurve3D::new(
        SurfaceCurveKind::SurfaceCurve,
        Box::new(leader),
        Vec::new(),
        SurfaceCurveRepresentation::Curve3D,
    ));
    let surface = Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy()));

    let boundary = StepParameterCurve::try_from(CurveTrimRef::new(&curve, &surface))
        .expect("surface curve leader should produce an exact parameter boundary.");

    match boundary.curve().as_ref() {
        Curve2D::Line(line) => {
            assert!(line.front().near(&Point2::new(0.25, 0.5)));
            assert!(line.back().near(&Point2::new(0.75, 0.5)));
        }
        curve => panic!("expected line boundary, got {curve:?}"),
    }
}

#[test]
fn surface_curve_line_without_pcurve_on_cylinder_stays_fallback_only() {
    let axis = Vector3::unit_z();
    let center = Point3::origin();
    let point = Point3::new(1.0, 0.0, 0.0);
    let profile = Line(point, point + axis);
    let surface = Surface::ElementarySurface(ElementarySurface::CylindricalSurface(
        Processor::new(RevolutionSurface::by_revolution(profile, center, axis)),
    ));
    let leader = Curve3D::Line(Line(surface.subs(0.0, 0.0), surface.subs(0.0, 1.0)));
    let curve = Curve3D::SurfaceCurve(SurfaceCurve3D::new(
        SurfaceCurveKind::SurfaceCurve,
        Box::new(leader),
        Vec::new(),
        SurfaceCurveRepresentation::Curve3D,
    ));

    assert!(StepParameterCurve::try_from(CurveTrimRef::new(&curve, &surface)).is_err());
    assert!(curve.exact_parameter_boundary_2d(&surface).is_none());
}

#[test]
fn raw_line_without_pcurve_on_cylinder_stays_fallback_only() {
    let axis = Vector3::unit_z();
    let center = Point3::origin();
    let point = Point3::new(1.0, 0.0, 0.0);
    let profile = Line(point, point + axis);
    let surface = Surface::ElementarySurface(ElementarySurface::CylindricalSurface(
        Processor::new(RevolutionSurface::by_revolution(profile, center, axis)),
    ));
    let curve = Curve3D::Line(Line(surface.subs(0.0, 0.0), surface.subs(0.0, 1.0)));

    assert!(StepParameterCurve::try_from(CurveTrimRef::new(&curve, &surface)).is_err());
    assert!(curve.exact_parameter_boundary_2d(&surface).is_none());
}

#[test]
fn builder() {
    use monstertruck_meshing::prelude::*;
    use monstertruck_modeling::builder;
    monstertruck_topology::prelude!(Point3, Curve3D, Surface);

    // cube
    let v = builder::vertices([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]);
    let e = builder::line(&v[0], &v[1]);
    let f = builder::extrude(&e, Vector3::unit_y());
    let cube: Solid = builder::extrude(&f, Vector3::unit_z());
    let mut poly = cube.triangulation(0.1).to_polygon();
    poly.put_together_same_attrs(1.0e-3).remove_unused_attrs();
    assert_eq!(poly.shell_condition(), ShellCondition::Closed);

    // cylinder
    let v = builder::vertices([(1.0, 0.0, 1.0), (1.0, 0.0, 0.0)]);
    let e = builder::line(&v[0], &v[1]);
    let mut shell = builder::revolve(
        &e,
        Point3::origin(),
        Vector3::unit_z(),
        builder::SweepAngle::Closed,
        2,
    );
    let boundaries = shell.extract_boundaries();
    assert_eq!(boundaries.len(), 2);
    shell.push(builder::try_attach_plane([boundaries[0].inverse()]).unwrap());
    shell.push(builder::try_attach_plane([boundaries[1].inverse()]).unwrap());
    let cylinder = Solid::new(vec![shell]);
    let mut poly = cylinder.triangulation(0.1).to_polygon();
    poly.put_together_same_attrs(1.0e-3).remove_unused_attrs();
    assert_eq!(poly.shell_condition(), ShellCondition::Closed);

    // torus
    let v = builder::vertex((1.5, 0.0, 0.0));
    let w = builder::revolve(
        &v,
        Point3::new(1.0, 0.0, 0.0),
        Vector3::unit_y(),
        builder::SweepAngle::Closed,
        2,
    );
    let f = builder::try_attach_plane([w]).unwrap();
    let torus: Solid = builder::revolve(
        &f,
        Point3::origin(),
        Vector3::unit_z(),
        builder::SweepAngle::Closed,
        2,
    );
    let mut poly = torus.triangulation(0.1).to_polygon();
    poly.put_together_same_attrs(1.0e-3).remove_unused_attrs();
    assert_eq!(poly.shell_condition(), ShellCondition::Closed);

    // cylinder hole
    let v = builder::vertex((-1.0, -1.0, -1.0));
    let e = builder::extrude(&v, 2.0 * Vector3::unit_x());
    let f = builder::extrude(&e, 2.0 * Vector3::unit_y());
    let s: Solid = builder::extrude(&f, 2.0 * Vector3::unit_z());
    let mut shell = s.into_boundaries().pop().unwrap();
    let line = builder::line(
        &builder::vertex((0.5, 0.0, 1.0)),
        &builder::vertex((0.5, 0.0, -1.0)),
    );
    let hole = builder::revolve(
        &line,
        Point3::origin(),
        -Vector3::unit_z(),
        builder::SweepAngle::Closed,
        2,
    );
    let boundary = hole.extract_boundaries();
    assert_eq!(boundary.len(), 2);
    if boundary[0][0].front().point().z < 0.0 {
        let _ = shell[0].add_boundary(boundary[0].inverse());
        let _ = shell[5].add_boundary(boundary[1].inverse());
    } else {
        let _ = shell[0].add_boundary(boundary[1].inverse());
        let _ = shell[5].add_boundary(boundary[0].inverse());
    }
    shell.extend(hole);
    let solid = Solid::new(vec![shell]);
    let mut poly = solid.triangulation(0.1).to_polygon();
    poly.put_together_same_attrs(1.0e-3).remove_unused_attrs();
    assert_eq!(poly.shell_condition(), ShellCondition::Closed);
}

// ---------------------------------------------------------------------------
// Spec 012 U1.2: the analytic sphere/torus route.
// ---------------------------------------------------------------------------

#[cfg(test)]
fn u1_step_sphere(radius: f64) -> Surface {
    Surface::ElementarySurface(ElementarySurface::Sphere(Processor::new(Sphere(
        monstertruck_geometry::prelude::Sphere::new(Point3::new(1.0, -2.0, 3.0), radius),
    ))))
}

#[cfg(test)]
fn u1_step_torus(large_radius: f64, small_radius: f64) -> Surface {
    Surface::ElementarySurface(ElementarySurface::ToroidalSurface(Processor::new(
        Torus::new(Point3::new(1.0, -2.0, 3.0), large_radius, small_radius),
    )))
}

/// The routing change is a TESSELLATION change and nothing else: the rational
/// net the boolean's homogeneous path prepares must be the very same one it
/// prepared while the modeling surface WAS that net.
///
/// Byte-identity, not a tolerance -- both sides call the same builder on the
/// same `Processor<_, Matrix4>`, so anything short of equality would mean the
/// route picked up an extra arithmetic step (the C1 re-spanning trap).
#[test]
fn the_analytic_route_emits_the_same_rational_net_the_flattened_route_did() {
    for step in [u1_step_sphere(53.0), u1_step_torus(7.0, 2.0)] {
        let flattened = step
            .try_into_homogeneous_bspline_surface()
            .expect("the builder accepts these radii");
        let modeling = ModelingSurface::try_from(&step).expect("must convert");
        assert!(
            matches!(
                modeling,
                ModelingSurface::SphericalSurface(_) | ModelingSurface::ToroidalSurface(_)
            ),
            "spheres and tori must reach the analytic variants, got {modeling:?}",
        );
        let analytic = modeling
            .try_into_homogeneous_bspline_surface()
            .expect("the analytic variant must still yield its net");
        assert_eq!(
            flattened.knot_vectors(),
            analytic.knot_vectors(),
            "the emitted knot vectors moved",
        );
        assert_eq!(
            flattened.control_points(),
            analytic.control_points(),
            "the emitted control net moved",
        );
    }
}

/// The closed form is what the whole track is for: the analytic variants must
/// spend ZERO adaptive refinement cells where the net spent them by the
/// million.
///
/// A count, not a wall clock (ledger M13/C8): `division_totals` is the
/// process-wide cell counter the U1.1 budget already maintains.
#[test]
fn the_analytic_route_spends_no_adaptive_refinement_cells() {
    use monstertruck_traits::algo::surface::take_division_totals;
    for (step, range) in [
        (
            u1_step_sphere(53.0),
            ((0.0, std::f64::consts::PI), (0.0, TAU)),
        ),
        (u1_step_torus(7.0, 2.0), ((0.0, TAU), (0.0, TAU))),
    ] {
        let modeling = ModelingSurface::try_from(&step).expect("must convert");
        let net = monstertruck_geometry::prelude::NurbsSurface::new(
            step.try_into_homogeneous_bspline_surface().unwrap(),
        );
        // Each side over ITS OWN declared range -- the analytic one in radians,
        // the net over its knot span. Comparing them over one frame would be
        // the C2 trap, and would also stack the extrapolation defect (b) onto
        // a measurement that is about the closed form (a).
        let net_range = net.range_tuple();

        let _ = take_division_totals();
        let _ = modeling.parameter_division(range, 1.0e-3);
        let (analytic_cells, _) = take_division_totals();

        let _ = net.parameter_division(net_range, 1.0e-3);
        let (net_cells, _) = take_division_totals();

        assert_eq!(
            analytic_cells, 0,
            "the analytic variant must divide in closed form",
        );
        assert!(
            net_cells > 0,
            "the net must still cost what it always cost, else this test proves nothing",
        );
    }
}

/// Spec 011 T1 must survive the routing change.
///
/// The spindle refusal lives in the BUILDER, so an analytic route that stopped
/// calling the builder would reopen it silently. On a spindle the FORWARD map
/// stays exact while `search_parameter` is wrong on ~29% of the domain, which
/// is what places trims -- so the class must keep refusing typed in every
/// encoding, and horn tori (the fillet form) must keep converting.
#[test]
fn a_spindle_torus_is_still_refused_by_the_analytic_route() {
    // |large| < small in each of the spellings the corpus carries.
    for (large_radius, small_radius) in [(1.0, 3.0), (0.5, 20.0), (1.0e-3, 1.0)] {
        let step = u1_step_torus(large_radius, small_radius);
        assert!(
            ModelingSurface::try_from(&step).is_err(),
            "spindle torus (R = {large_radius}, r = {small_radius}) must refuse typed",
        );
    }
    // Horn (R == r) and ring (R > r) are unaffected and must still convert.
    for (large_radius, small_radius) in [(2.0, 2.0), (7.0, 2.0)] {
        let step = u1_step_torus(large_radius, small_radius);
        assert!(
            matches!(
                ModelingSurface::try_from(&step),
                Ok(ModelingSurface::ToroidalSurface(_))
            ),
            "torus (R = {large_radius}, r = {small_radius}) must convert typed",
        );
    }
}

/// The guard is the BUILDER's and no wider: a surface the analytic route
/// declines must land on exactly the answer it lands on today, not on a
/// different refusal and not on a silent success.
#[test]
fn the_analytic_guard_matches_the_builders_own_predicate() {
    for step in [
        u1_step_sphere(f64::INFINITY),
        u1_step_torus(1.0, 3.0),
        u1_step_torus(f64::INFINITY, 1.0),
    ] {
        assert_eq!(
            step.try_into_homogeneous_bspline_surface().is_none(),
            ModelingSurface::try_from(&step).is_err(),
            "the routing guard and the builder must agree on {step:?}",
        );
    }
}
