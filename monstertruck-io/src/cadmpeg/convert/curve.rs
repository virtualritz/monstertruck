//! Curve carriers: intermediate representation to [`Curve`], trimmed to an edge.
//!
//! # A curve is converted FOR an edge, never on its own
//!
//! monstertruck's [`CompressedEdge`] holds a curve whose own bounded range runs
//! from the edge's start vertex to its end vertex: `Edge::try_new` checks the
//! curve's endpoints against the vertices it is given. The intermediate
//! representation instead stores an UNBOUNDED carrier plus the edge's parameter
//! interval on it -- a full circle and `[t0, t1]`, an infinite line and two
//! signed distances.
//!
//! So the conversion needs both halves, and the trim is not an afterthought. This
//! module takes the edge's interval with the carrier, and every arm below returns
//! a curve already restricted and already oriented start-to-end. Converting the
//! carrier first and trimming afterwards would need a second parameterization
//! argument at every call site, and would be wrong for exactly the carriers whose
//! parameterization is not preserved by the conversion.
//!
//! # What the interval means
//!
//! Per the representation's own contract, conic parameters are ANGLES from the
//! reference direction and line parameters are SIGNED DISTANCES along the unit
//! direction. They are not normalised to `[0, 1]`, and the start vertex lies at
//! `t0`. A converter that assumed a normalised interval would produce arcs of the
//! right shape in the wrong place.
//!
//! [`CompressedEdge`]: monstertruck_topology::compress::CompressedEdge
//!
//! Since cadmpeg 0.6 the representation's edge interval is ORDERED: the edge's
//! start vertex lies at the lower parameter, and the codecs reverse the edge's
//! vertices rather than its interval when the edge runs against its carrier.
//! The backwards-interval arm below is kept anyway -- it costs one branch and
//! keeps this module correct for a caller that hands it one.

use cadmpeg_ir::geometry::nurbs::NurbsCurve as IrNurbsCurve;
use cadmpeg_ir::geometry::{CurveGeometry, SolvedCurveGeometry};
use monstertruck_core::cgmath64::control_point::ControlPoint;
use monstertruck_geometry::prelude::{
    BoundedCurve, BsplineCurve, Concat, Cut, Invertible, KnotVector, NurbsCurve,
    ParameterTransform, Processor, Tolerance, TrimmedCurve, UnitCircle,
};
use monstertruck_modeling::{
    Curve, Line, MetricSpace, Point3, SquareMatrix, Transform as _, Transformed, Vector4,
};

use super::surface::{built, homogeneous};
use super::{Context, frame};
use crate::{Error, Result};

/// The parameter interval an edge occupies on its carrier, when the file says.
pub(super) type Interval = Option<[f64; 2]>;

/// Convert one curve carrier, restricted to `interval` and oriented so that the
/// result runs from the edge's start vertex to its end vertex.
///
/// `endpoints` are the edge's own vertex positions. They are the fallback for
/// carriers whose restriction is determined by its endpoints -- a line -- and they
/// are never used to REPLACE a stated interval.
///
/// A `Procedural` carrier converts through its solved cache when the
/// construction states no cache fit tolerance, and is refused by name
/// otherwise, for the reasons given on [`super::surface::convert`]. The edge
/// interval is on the cache's parameterization, which is the one the
/// representation evaluates the carrier by. As on the surface side, this is a
/// policy for exact caches rather than a measured rescue: the STEP and IGES
/// decoders shipped at 0.6.11 did not exercise it in the reviewed fixtures.
pub(super) fn convert(
    geometry: &CurveGeometry,
    interval: Interval,
    endpoints: (Point3, Point3),
    context: &Context<'_>,
) -> Result<Curve> {
    match geometry {
        CurveGeometry::Solved(solved) => solved_curve(solved, interval, endpoints, context),
        CurveGeometry::Procedural {
            construction,
            cache: Some(cache),
        } => {
            let construction = context.procedural_curve(construction.as_str())?;
            if construction.cache_fit_tolerance().is_some() {
                return Err(unsupported("procedural (fitted cache)", context));
            }
            solved_curve(cache, interval, endpoints, context)
        }
        CurveGeometry::Procedural { cache: None, .. } => {
            Err(unsupported("procedural (no solved cache)", context))
        }
    }
}

fn solved_curve(
    geometry: &SolvedCurveGeometry,
    interval: Interval,
    endpoints: (Point3, Point3),
    context: &Context<'_>,
) -> Result<Curve> {
    match geometry {
        SolvedCurveGeometry::Line(line) => {
            let origin = frame::finite_point(line.origin());
            // Unit length by type (`UnitVector3`).
            let direction = frame::vector(line.direction().as_raw());
            match interval {
                // The stated interval wins: a line's parameter is a signed
                // distance, so this reproduces the source's own endpoints
                // exactly, including the case where they sit a tolerance away
                // from the vertex points.
                Some([start, end]) => Ok(Curve::Line(Line(
                    origin + direction * start,
                    origin + direction * end,
                ))),
                // With no interval the vertices ARE the trim, exactly: the
                // segment between two points on a line is that line's
                // restriction, with nothing inferred.
                None => Ok(Curve::Line(Line(endpoints.0, endpoints.1))),
            }
        }

        SolvedCurveGeometry::Circle(circle) => {
            let frame = frame::frame(circle.center(), circle.frame());
            // Positive by type (`PositiveLength`).
            let radius = circle.radius().get();
            // An arc needs its angles. Without them the carrier is the whole
            // circle, and which of the two arcs between the vertices the edge
            // meant is not recoverable -- unless the edge's ends COINCIDE, in
            // which case the edge is one closed traversal and the IR supplies
            // its range. This is the mappable case the evidence note lists:
            // refusing it costs whole bodies of the in-repo STEP corpus.
            let interval = interval.or_else(|| {
                (endpoints.0.distance(endpoints.1) <= monstertruck_modeling::TOLERANCE)
                    .then(|| geometry.closed_traversal_range())
                    .flatten()
                    .map(|range| range.endpoints())
            });
            let Some([start, end]) = interval else {
                return Err(context.malformed(
                    "a circular edge carries no parameter range and distinct ends, so which \
                     arc between its vertices it spans is not recoverable"
                        .to_owned(),
                ));
            };
            // A FULL circle is not one rational quadratic: its middle weight
            // collapses (the start and end points coincide) and evaluation
            // returns NaN. The four-arc NURBS below is exact, including its
            // angle parameterization, and is what a closed circular edge needs.
            if ((end - start).abs() - std::f64::consts::TAU).abs() <= slack(end - start) {
                return Ok(full_circle(&frame, radius, start));
            }
            // The radius rides in the placement and the entity stays a UNIT
            // circle, which is the construction `monstertruck-modeling`'s own
            // `circle_arc` uses. `TrimmedCurve`'s range is the angle sweep, so an
            // interval past `2 pi` (an arc across the zero azimuth, which the
            // representation writes unwrapped) and a reversed interval both give
            // the right arc.
            let arc = Processor::with_transform(
                TrimmedCurve::new(UnitCircle::<Point3>::new(), (start, end)),
                frame.placement(radius),
            );
            // Exact: a circular arc IS a rational quadratic, so this is a change
            // of representation and not an approximation.
            rational(&arc, context)
        }

        SolvedCurveGeometry::Nurbs(curve) => nurbs(curve, interval, context),

        SolvedCurveGeometry::Degenerate(_) => Err(unsupported("degenerate", context)),

        SolvedCurveGeometry::Transformed(placed) => {
            let matrix = frame::matrix(placed.transform());
            // The endpoints are in MODEL space and the basis carrier is not, so
            // the fallback endpoints have to be pulled back before the basis is
            // converted, or a line with no stated interval would be built from
            // points that are not on it. An affine map is invertible or it is not
            // a placement.
            let inverse = matrix.invert().ok_or_else(|| {
                context.malformed("a transformed curve carries a singular placement".to_owned())
            })?;
            let pulled = (
                inverse.transform_point(endpoints.0),
                inverse.transform_point(endpoints.1),
            );
            let mut curve = solved_curve(placed.basis(), interval, pulled, context)?;
            curve.transform_by(matrix);
            Ok(curve)
        }

        // Ellipses, parabolas and hyperbolas are exactly representable as
        // rational curves in principle, but this crate's geometry has no builder
        // for them -- only the circle has one -- so admitting them would mean
        // writing that builder here, unverified, and calling the result exact.
        // Refused by name until the builder exists next to the circle's.
        SolvedCurveGeometry::Ellipse(_) => Err(unsupported("elliptical", context)),
        SolvedCurveGeometry::Parabola(_) => Err(unsupported("parabolic", context)),
        SolvedCurveGeometry::Hyperbola(_) => Err(unsupported("hyperbolic", context)),
        // A composite carrier is several curves in one edge; monstertruck's
        // `CompressedEdge` holds exactly one, so admitting it means
        // concatenating the segments into one curve. Not yet written.
        SolvedCurveGeometry::Composite { .. } => Err(unsupported("composite", context)),
        // An approximation with a chordal bound. Admitting it as a spline would
        // present a polyline as the exact edge.
        SolvedCurveGeometry::Polyline(_) => Err(unsupported("source-native polyline", context)),
        SolvedCurveGeometry::Unknown { .. } => Err(unsupported("unclassified native", context)),
        // Exhaustive on purpose, with no catch-all: see the note in
        // [`super::surface::convert`].
    }
}

/// The whole circle of `radius` about `frame`, traversed once counterclockwise
/// from angle `start`, as an exact four-arc rational quadratic.
///
/// Four quarter arcs keep every weight positive and every control point finite.
/// The knots are the angles themselves, so the curve's own range is
/// `[start, start + 2 pi]` and its midpoint is the antipode of the seam.
fn full_circle(frame: &frame::Frame, radius: f64, start: f64) -> Curve {
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, SQRT_2};
    let control: Vec<Vector4> = (0..=8)
        .map(|index| {
            let angle = start + index as f64 * FRAC_PI_2 / 2.0;
            // Odd control points are the tangent intersections, sqrt(2) radii
            // out, carrying the quarter arc's cos(pi/4) weight.
            let (distance, weight) = match index % 2 {
                0 => (radius, 1.0),
                _ => (radius * SQRT_2, FRAC_1_SQRT_2),
            };
            let point = frame.origin
                + frame.reference * (angle.cos() * distance)
                + frame.co_reference() * (angle.sin() * distance);
            Vector4::new(point.x * weight, point.y * weight, point.z * weight, weight)
        })
        .collect();
    let mut knots = vec![start; 3];
    (1..4).for_each(|segment| {
        let knot = start + segment as f64 * FRAC_PI_2;
        knots.push(knot);
        knots.push(knot);
    });
    knots.extend([start + 2.0 * std::f64::consts::PI; 3]);
    Curve::NurbsCurve(NurbsCurve::new(BsplineCurve::new(
        KnotVector::from(knots),
        control,
    )))
}

fn unsupported(kind: &'static str, context: &Context<'_>) -> Error {
    Error::UnsupportedCurveKind {
        format: context.format,
        kind,
    }
}

/// A carrier whose exact form is rational, as a [`Curve::NurbsCurve`].
fn rational<T>(curve: &T, context: &Context<'_>) -> Result<Curve>
where T: monstertruck_geometry::prelude::TryIntoHomogeneousBsplineCurve {
    curve
        .try_into_homogeneous_bspline_curve()
        .map(|net| Curve::NurbsCurve(NurbsCurve::new(net)))
        .ok_or_else(|| {
            context.malformed(
                "an exactly-representable curve could not be built, which means its own numbers \
                 are degenerate"
                    .to_owned(),
            )
        })
}

/// A free-form curve, restricted to the edge's knot interval.
///
/// Periodic and unclamped knot vectors map exactly; see [`super::surface`]'s
/// note on `nurbs` for why. A periodic edge's interval may additionally lie
/// outside the domain or straddle its seam, because the representation writes
/// a periodic interval UNWRAPPED; [`periodic_span`] handles both.
fn nurbs(curve: &IrNurbsCurve, interval: Interval, context: &Context<'_>) -> Result<Curve> {
    let knots = KnotVector::from(curve.knots().to_vec());
    let degree = curve.degree() as usize;
    let periodic = curve.periodic();
    let points = curve.control_points();
    match curve.weights() {
        None => {
            let net = built(
                BsplineCurve::try_new(
                    knots,
                    points.iter().map(|p| frame::finite_point(*p)).collect(),
                ),
                context,
            )?;
            Ok(Curve::BsplineCurve(trimmed(
                net, degree, periodic, interval, context,
            )?))
        }
        Some(weights) => {
            let net = built(
                BsplineCurve::try_new(
                    knots,
                    points
                        .iter()
                        .zip(&weights)
                        .map(|(point, weight)| homogeneous(&point.get(), weight.get()))
                        .collect(),
                ),
                context,
            )?;
            Ok(Curve::NurbsCurve(NurbsCurve::new(trimmed(
                net, degree, periodic, interval, context,
            )?)))
        }
    }
}

/// Clamp a free-form net onto its effective domain, then restrict it to the
/// edge's interval.
///
/// A free-form carrier's parameterization IS preserved by the conversion -- the
/// knot vector carries the source's own knot values -- so the edge's interval is
/// a knot-span restriction and `Cut` performs it exactly.
fn trimmed<P: ControlPoint<f64> + Tolerance>(
    mut net: BsplineCurve<P>,
    degree: usize,
    periodic: bool,
    interval: Interval,
    context: &Context<'_>,
) -> Result<BsplineCurve<P>> {
    let (low, high) = clamp_onto_domain(&mut net, degree, context)?;
    let Some([start, end]) = interval else {
        return Ok(net);
    };
    let (from, to) = (start.min(end), start.max(end));
    let mut span = if periodic {
        periodic_span(net, (low, high), (from, to), context)?
    } else {
        restrict(net, from, to, context)?
    };
    if end < start {
        // The edge runs against the carrier's parameter direction. Inverting here
        // -- rather than leaving it to the coedge's sense -- keeps the invariant
        // this module promises: the returned curve runs start vertex to end
        // vertex, so `Edge::try_new` can check it.
        span.invert();
    }
    Ok(span)
}

/// Clamp a net onto `[knot[degree], knot[count]]` by knot insertion, which
/// leaves the spline unchanged, and return that domain. A net already clamped
/// is returned as it was.
fn clamp_onto_domain<P: ControlPoint<f64> + Tolerance>(
    net: &mut BsplineCurve<P>,
    degree: usize,
    context: &Context<'_>,
) -> Result<(f64, f64)> {
    let count = net.control_points().len();
    let knots = net.knot_vector();
    let (low, high) = (knots[degree], knots[count]);
    let front = knots[..=degree].iter().all(|knot| *knot == low);
    let back = knots[count..].iter().all(|knot| *knot == high);
    if !back {
        let _beyond = net.cut(high);
    }
    if !front {
        *net = net.cut(low);
    }
    if !net.is_clamped() || net.range_tuple() != (low, high) {
        return Err(context.malformed(format!(
            "a NURBS curve's knot vector could not be clamped onto its effective domain \
             [{low}, {high}]"
        )));
    }
    Ok((low, high))
}

/// Restrict a periodic net, already clamped onto its domain `[low, high]`, to
/// the increasing interval `[from, to]`, which the representation may state
/// unwrapped -- shifted by whole periods, or running past the seam.
///
/// The representation evaluates a periodic net modulo the period, so the
/// interval is first shifted by whole periods to start inside the domain. If it
/// then ends inside the domain it is an ordinary restriction. If it runs past
/// the seam, the edge is the tail `[from, high]` followed by the head
/// `[low, to - period]`, concatenated with the head's knots moved up one
/// period. The concatenation is exact because a closed periodic net meets
/// itself at the seam; when it does not, the source's own evaluation is
/// discontinuous there, and that is refused by name rather than bridged.
fn periodic_span<P: ControlPoint<f64> + Tolerance>(
    net: BsplineCurve<P>,
    (low, high): (f64, f64),
    (from, to): (f64, f64),
    context: &Context<'_>,
) -> Result<BsplineCurve<P>> {
    let period = high - low;
    let slack = slack(period);
    if to - from > period + slack {
        return Err(context.malformed(format!(
            "an edge spans [{from}, {to}] on a periodic curve whose period is only {period}"
        )));
    }
    let turns = ((from - low) / period).floor();
    let (mut from, mut to) = (from - turns * period, to - turns * period);
    if high - from <= slack {
        // Starts at the seam from below: the same point as the domain's start.
        from -= period;
        to -= period;
    }
    let from = from.max(low);
    if to <= high + slack {
        return restrict(net, from, to.min(high), context);
    }
    let tail = restrict(net.clone(), from, high, context)?;
    let mut head = restrict(net, low, to - period, context)?;
    head.parameter_transform(1.0, period);
    tail.try_concat(&head)
        .map_err(|_| Error::UnsupportedCurveKind {
            format: context.format,
            kind: "periodic NURBS edge across a seam the curve does not close at",
        })
}

/// The parameter slack a range check allows on a span of `width`.
fn slack(width: f64) -> f64 { monstertruck_modeling::TOLERANCE.max(width.abs() * 1.0e-9) }

/// Restrict a clamped net to the increasing interval `[low, high]` on its own
/// parameterization.
///
/// `Cut` splits in place: after `cut(t)` the receiver covers `[a, t]` and the
/// returned curve covers `[t, b]`. Two cuts therefore isolate the middle span,
/// and the order matters -- cutting at the far end first would discard the span
/// the second cut needs.
fn restrict<P: ControlPoint<f64> + Tolerance>(
    mut curve: BsplineCurve<P>,
    low: f64,
    high: f64,
    context: &Context<'_>,
) -> Result<BsplineCurve<P>> {
    let (own_low, own_high) = curve.range_tuple();
    let slack = slack(own_high - own_low);
    if !low.is_finite() || !high.is_finite() {
        return Err(context.malformed(format!(
            "an edge carries the parameter range [{low}, {high}]"
        )));
    }
    if low < own_low - slack || high > own_high + slack {
        // Extrapolating a spline past its knot span is not the same curve, so a
        // range that leaves the carrier is a defect in the file, not a rounding
        // question.
        return Err(context.malformed(format!(
            "an edge spans [{low}, {high}] on a curve whose own range is [{own_low}, {own_high}]"
        )));
    }
    // Clamp INTO the carrier's range before cutting: a range that is inside it to
    // within `slack` is the whole carrier, and asking `Cut` for a parameter a few
    // ulps outside is not defined.
    let (low, high) = (low.max(own_low), high.min(own_high));
    if high - own_low < slack || own_high - low < slack {
        return Err(context.malformed(format!(
            "an edge spans the empty range [{low}, {high}] on its curve"
        )));
    }
    if own_high - high > slack {
        let _discarded = curve.cut(high);
    }
    if low - own_low > slack {
        curve = curve.cut(low);
    }
    Ok(curve)
}
