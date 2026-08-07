//! Parameter-space trims: the 2D boundary a 3D STEP curve has on a STEP
//! surface, found exactly where the pair admits a closed form and by
//! projected sampling otherwise.

use super::*;

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
    // The twin in `monstertruck-modeling/src/geometry/`
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
