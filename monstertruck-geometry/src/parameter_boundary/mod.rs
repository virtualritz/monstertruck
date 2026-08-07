use crate::prelude::*;
use smallvec::SmallVec;

/// Face-local 2D trim curve used to preserve better boundary structure than a
/// raw UV polyline.
#[derive(
    Clone,
    Debug,
    ParametricCurve,
    BoundedCurve,
    ParameterDivision1D,
    Cut,
    Invertible,
    SearchNearestParameterD1,
    SearchParameterD1,
)]
pub enum BoundaryCurve2D {
    /// Linear trim segment in UV.
    Line(Line<Point2>),
    /// Degree-1 B-spline trim through sampled UV points.
    BsplineCurve(BsplineCurve<Point2>),
    /// Rational trim curve in UV.
    NurbsCurve(NurbsCurve<Vector3>),
}

impl<S> ParameterBoundary2D<S> for Line<Point3> {}

impl<S> ParameterBoundary2D<S> for BsplineCurve<Point3> {}

impl<S> ParameterBoundary2D<S> for NurbsCurve<Vector4> {}

impl<C, S> ExactParameterBoundary2D<S> for ParameterCurve<C, S>
where
    C: ParametricCurve2D<Point = Point2>
        + BoundedCurve<Point = Point2>
        + SearchParameter<CurveParameter, Point = Point2>
        + SearchNearestParameter<CurveParameter, Point = Point2>
        + Cut
        + Invertible
        + Clone,
    S: ParametricSurface3D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + Clone
        + PartialEq,
{
    type BoundaryCurve = Self;

    fn exact_parameter_boundary_2d(&self, surface: &S) -> Option<Self::BoundaryCurve> {
        (self.surface() == surface).then(|| self.clone())
    }
}

impl<S: Clone> BoundaryCurveFromSamples<S> for ParameterCurve<Line<Point2>, S> {
    fn boundary_curve_from_samples(surface: &S, points: Vec<Point2>) -> Option<Self> {
        (points.len() >= 2).then(|| {
            let front = points.first().copied().unwrap();
            let back = points.last().copied().unwrap();
            ParameterCurve::new(Line(front, back), surface.clone())
        })
    }
}

impl<S: Clone> BoundaryCurveFromSamples<S> for ParameterCurve<BoundaryCurve2D, S> {
    fn boundary_curve_from_samples(surface: &S, points: Vec<Point2>) -> Option<Self> {
        if points.len() < 2 {
            None
        } else {
            let front = points.first().copied().unwrap();
            let back = points.last().copied().unwrap();
            let line = Line(front, back);
            let is_linear = points.iter().copied().all(|point| {
                line.search_nearest_parameter(point, None, 1)
                    .is_some_and(|t| line.subs(t).near(&point))
            });
            Some(ParameterCurve::new(
                if is_linear {
                    BoundaryCurve2D::Line(line)
                } else {
                    let denom = (points.len() - 1) as f64;
                    let knot_vec = KnotVector::from(
                        std::iter::once(0.0)
                            .chain((0..points.len()).map(|index| index as f64 / denom))
                            .chain(std::iter::once(1.0))
                            .collect::<Vec<_>>(),
                    );
                    BoundaryCurve2D::BsplineCurve(BsplineCurve::new(knot_vec, points))
                },
                surface.clone(),
            ))
        }
    }
}

impl TryFrom<ParameterCurve<Line<Point2>, Plane>> for BsplineCurve<Point3> {
    type Error = ();

    fn try_from(
        value: ParameterCurve<Line<Point2>, Plane>,
    ) -> std::result::Result<Self, Self::Error> {
        let (line, plane) = value.decompose();
        Ok(BsplineCurve::from(Line(
            plane.subs(line.0.x, line.0.y),
            plane.subs(line.1.x, line.1.y),
        )))
    }
}

impl<C, S> ParameterBoundary2D<S> for ParameterCurve<C, S>
where
    C: ParametricCurve2D + BoundedCurve + ParameterDivision1D<Point = Point2>,
    S: PartialEq,
{
    fn parameter_boundary_2d(&self, surface: &S, tolerance: f64) -> Option<Vec<Point2>> {
        if self.surface() == surface {
            self.curve()
                .try_parameter_division(self.curve().range_tuple(), tolerance)
                .map(|(_, points)| points)
        } else {
            None
        }
    }
}

impl<C, S> ParameterBoundary2D<S> for TrimmedCurve<C>
where C: ParameterBoundary2D<S>
{
    fn parameter_boundary_2d(&self, surface: &S, tolerance: f64) -> Option<Vec<Point2>> {
        self.curve().parameter_boundary_2d(surface, tolerance)
    }
}

impl<C, T, S> ParameterBoundary2D<S> for Processor<C, T> where Processor<C, T>: ParametricCurve3D {}

fn exact_line_boundary_on_plane(
    line: &Line<Point3>,
    plane: &Plane,
) -> Option<ParameterCurve<Line<Point2>, Plane>> {
    let (u0, v0) = plane.search_parameter(line.front(), None, 1)?;
    let (u1, v1) = plane.search_parameter(line.back(), None, 1)?;
    let boundary = ParameterCurve::new(Line(Point2::new(u0, v0), Point2::new(u1, v1)), *plane);
    boundary.subs(0.5).near(&line.subs(0.5)).then_some(boundary)
}

fn exact_line_boundary_on_affine_surface<S>(
    line: &Line<Point3>,
    surface: &S,
) -> Option<ParameterCurve<Line<Point2>, S>>
where
    S: Clone + ParametricSurface3D,
{
    let (u_range, v_range) = surface.try_range_tuple();
    let ((u0, u1), (v0, v1)) = (u_range?, v_range?);
    let p00 = surface.subs(u0, v0);
    let p10 = surface.subs(u1, v0);
    let p01 = surface.subs(u0, v1);
    let p11 = surface.subs(u1, v1);
    let midpoint = surface.subs((u0 + u1) * 0.5, (v0 + v1) * 0.5);
    (p11.near(&(p10 + (p01 - p00))) && midpoint.near(&(p00 + ((p10 - p00) + (p01 - p00)) * 0.5)))
        .then_some(())?;
    let u_axis = p10 - p00;
    let v_axis = p01 - p00;
    let uu = u_axis.dot(u_axis);
    let uv = u_axis.dot(v_axis);
    let vv = v_axis.dot(v_axis);
    let denominator = uu * vv - uv * uv;
    (!denominator.so_small()).then_some(())?;
    let project = |point: Point3| {
        let delta = point - p00;
        let du = delta.dot(u_axis);
        let dv = delta.dot(v_axis);
        let su = (du * vv - dv * uv) / denominator;
        let sv = (dv * uu - du * uv) / denominator;
        let parameter = Point2::new(u0 + (u1 - u0) * su, v0 + (v1 - v0) * sv);
        surface
            .subs(parameter.x, parameter.y)
            .near(&point)
            .then_some(parameter)
    };
    let front = project(line.front())?;
    let middle = project(line.subs(0.5))?;
    let back = project(line.back())?;
    let boundary = Line(front, back);
    boundary
        .subs(0.5)
        .near(&middle)
        .then(|| ParameterCurve::new(boundary, surface.clone()))
}

fn exact_line_boundary_on_homogeneous_extrusion_surface(
    line: &Line<Point3>,
    surface: &NurbsSurface<Vector4>,
) -> Option<ParameterCurve<Line<Point2>, NurbsSurface<Vector4>>> {
    let extrusion = match surface.try_into_analytic_surface_kind()? {
        AnalyticSurfaceKind::HomogeneousExtrusion(extrusion) => extrusion,
        _ => None?,
    };
    let vector = extrusion.vector;
    let line_vector = line.back() - line.front();
    let vector2 = vector.magnitude2();
    let line_vector2 = line_vector.magnitude2();
    (!vector2.so_small() && !line_vector2.so_small()).then_some(())?;
    (line_vector.cross(vector).magnitude2() <= TOLERANCE * TOLERANCE * line_vector2 * vector2)
        .then_some(())?;
    let base_curve = NurbsCurve::new(extrusion.curve.clone());
    let curve_tolerance = TOLERANCE
        * (extrusion.curve_range.1 - extrusion.curve_range.0)
            .abs()
            .max(1.0);
    let curve_parameter = base_curve.search_nearest_parameter(line.front(), None, 30)?;
    let back_curve_parameter = base_curve
        .search_nearest_parameter(line.back(), Some(curve_parameter), 30)
        .or_else(|| base_curve.search_nearest_parameter(line.back(), None, 30))?;
    ((back_curve_parameter - curve_parameter).abs() <= curve_tolerance).then_some(())?;
    let base_point = base_curve.subs(curve_parameter);
    let extrusion_parameter = |point: Point3| {
        let factor = (point - base_point).dot(vector) / vector2;
        let parameter = extrusion.extrusion_range.0
            + (extrusion.extrusion_range.1 - extrusion.extrusion_range.0) * factor;
        let uv = match (extrusion.curve_axis, extrusion.extrusion_axis) {
            (SurfaceParameterAxis::U, SurfaceParameterAxis::V) => {
                Point2::new(curve_parameter, parameter)
            }
            (SurfaceParameterAxis::V, SurfaceParameterAxis::U) => {
                Point2::new(parameter, curve_parameter)
            }
            _ => None?,
        };
        surface.subs(uv.x, uv.y).near(&point).then_some(uv)
    };
    let front = extrusion_parameter(line.front())?;
    let middle = extrusion_parameter(line.subs(0.5))?;
    let back = extrusion_parameter(line.back())?;
    let boundary = Line(front, back);
    boundary
        .subs(0.5)
        .near(&middle)
        .then(|| ParameterCurve::new(boundary, surface.clone()))
}

fn exact_boundary_segment<C, B, P>(curve: &C, boundary: &B) -> Option<(f64, f64)>
where
    C: ParametricCurve<Point = P> + BoundedCurve<Point = P>,
    B: ParametricCurve<Point = P>
        + BoundedCurve<Point = P>
        + SearchParameter<CurveParameter, Point = P>,
    P: Copy, {
    let (t0, t1) = curve.range_tuple();
    let samples = [
        t0,
        (3.0 * t0 + t1) * 0.25,
        (t0 + t1) * 0.5,
        (t0 + 3.0 * t1) * 0.25,
        t1,
    ];
    samples
        .into_iter()
        .all(|t| {
            boundary
                .search_parameter(curve.subs(t), None, 100)
                .is_some()
        })
        .then(|| {
            let front = boundary.search_parameter(curve.front(), None, 100)?;
            let back = boundary
                .search_parameter(curve.back(), Some(front), 100)
                .or_else(|| boundary.search_parameter(curve.back(), None, 100))?;
            Some((front, back))
        })
        .flatten()
}

fn exact_boundary_line_on_surface<C, B, P>(
    curve: &C,
    last_u: usize,
    last_v: usize,
    column_curve: impl Fn(usize) -> B,
    row_curve: impl Fn(usize) -> B,
    ((u0, u1), (v0, v1)): ((f64, f64), (f64, f64)),
) -> Option<Line<Point2>>
where
    C: ParametricCurve<Point = P>
        + BoundedCurve<Point = P>
        + SearchParameter<CurveParameter, Point = P>,
    P: Copy,
    B: ParametricCurve<Point = P>
        + BoundedCurve<Point = P>
        + SearchParameter<CurveParameter, Point = P>,
{
    exact_boundary_segment(curve, &column_curve(0))
        .map(|(s0, s1)| Line(Point2::new(u0, s0), Point2::new(u0, s1)))
        .or_else(|| {
            exact_boundary_segment(curve, &column_curve(last_u))
                .map(|(s0, s1)| Line(Point2::new(u1, s0), Point2::new(u1, s1)))
        })
        .or_else(|| {
            exact_boundary_segment(curve, &row_curve(0))
                .map(|(s0, s1)| Line(Point2::new(s0, v0), Point2::new(s1, v0)))
        })
        .or_else(|| {
            exact_boundary_segment(curve, &row_curve(last_v))
                .map(|(s0, s1)| Line(Point2::new(s0, v1), Point2::new(s1, v1)))
        })
}

fn exact_bspline_boundary_on_surface(
    curve: &BsplineCurve<Point3>,
    surface: &BsplineSurface<Point3>,
) -> Option<ParameterCurve<Line<Point2>, BsplineSurface<Point3>>> {
    let (u_range, v_range) = surface.try_range_tuple();
    let ((u0, u1), (v0, v1)) = (u_range?, v_range?);
    let last_u = surface.control_points().len().checked_sub(1)?;
    let last_v = surface.control_points().first()?.len().checked_sub(1)?;
    let boundary = exact_boundary_line_on_surface(
        curve,
        last_u,
        last_v,
        |index| surface.curve_v(index),
        |index| surface.curve_u(index),
        ((u0, u1), (v0, v1)),
    )?;
    Some(ParameterCurve::new(boundary, surface.clone()))
}

fn exact_nurbs_boundary_on_surface(
    curve: &NurbsCurve<Vector4>,
    surface: &NurbsSurface<Vector4>,
) -> Option<ParameterCurve<Line<Point2>, NurbsSurface<Vector4>>> {
    let (u_range, v_range) = surface.try_range_tuple();
    let ((u0, u1), (v0, v1)) = (u_range?, v_range?);
    let last_u = surface.control_points().len().checked_sub(1)?;
    let last_v = surface.control_points().first()?.len().checked_sub(1)?;
    let boundary = exact_boundary_line_on_surface(
        curve,
        last_u,
        last_v,
        |index| surface.curve_v(index),
        |index| surface.curve_u(index),
        ((u0, u1), (v0, v1)),
    )?;
    Some(ParameterCurve::new(boundary, surface.clone()))
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

fn exact_linear_boundary_by_surface_search<C, S>(
    curve: &C,
    surface: &S,
) -> Option<ParameterCurve<Line<Point2>, S>>
where
    C: ParametricCurve3D + BoundedCurve<Point = Point3>,
    S: Clone + ParametricSurface3D + SearchParameter<D2, Point = Point3>,
{
    let (t0, t1) = curve.range_tuple();
    exact_linear_boundary_by_surface_search_with_parameters(
        curve,
        surface,
        [
            t0,
            (3.0 * t0 + t1) * 0.25,
            (t0 + t1) * 0.5,
            (t0 + 3.0 * t1) * 0.25,
            t1,
        ],
    )
}

fn exact_line_boundary_by_surface_search<C, S>(
    curve: &C,
    surface: &S,
) -> Option<ParameterCurve<Line<Point2>, S>>
where
    C: ParametricCurve3D + BoundedCurve<Point = Point3>,
    S: Clone + ParametricSurface3D + SearchParameter<D2, Point = Point3>,
{
    let (t0, t1) = curve.range_tuple();
    exact_linear_boundary_by_surface_search_with_parameters(
        curve,
        surface,
        [t0, (t0 + t1) * 0.5, t1],
    )
}

fn project_sample_to_parameter_line<S>(
    point: Point3,
    uv: Point2,
    line: Line<Point2>,
    surface: &S,
) -> Option<Point2>
where
    S: ParametricSurface3D,
{
    let direction = line.1 - line.0;
    let len2 = direction.magnitude2();
    (len2 > TOLERANCE * TOLERANCE)
        .then(|| {
            let parameter = (uv - line.0).dot(direction) / len2;
            let projected = line.0 + direction * parameter;
            surface
                .subs(projected.x, projected.y)
                .near(&point)
                .then_some(projected)
        })
        .flatten()
}

fn exact_parameter_line_from_samples<S>(
    samples: &[(Point3, Point2)],
    surface: &S,
    candidate: Line<Point2>,
) -> Option<Line<Point2>>
where
    S: ParametricSurface3D,
{
    let projected = samples
        .iter()
        .copied()
        .map(|(point, uv)| project_sample_to_parameter_line(point, uv, candidate, surface))
        .collect::<Option<SmallVec<[Point2; 5]>>>()?;
    let line = Line(*projected.first()?, *projected.last()?);
    (!line.0.near(&line.1)).then_some(line)
}

fn exact_linear_boundary_by_surface_search_with_parameters<C, S, I>(
    curve: &C,
    surface: &S,
    parameters: I,
) -> Option<ParameterCurve<Line<Point2>, S>>
where
    C: ParametricCurve3D + BoundedCurve<Point = Point3>,
    S: Clone + ParametricSurface3D + SearchParameter<D2, Point = Point3>,
    I: IntoIterator<Item = f64>,
{
    let mut parameters = parameters.into_iter();
    let periods = (surface.period_u(), surface.period_v());
    let samples: SmallVec<[(Point3, Point2); 5]> =
        parameters.try_fold(SmallVec::new(), |mut samples, parameter| {
            let point = curve.subs(parameter);
            let hint = samples
                .last()
                .map(|(_, uv): &(Point3, Point2)| (*uv).into());
            let uv = surface
                .search_parameter(point, hint, 30)
                .or_else(|| surface.search_parameter(point, None, 30))?;
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
    let sample_count = samples.len();
    let pairs = (0..sample_count)
        .flat_map(|start| ((start + 1)..sample_count).map(move |end| (start, end)));
    pairs
        .filter_map(|(start, end)| {
            let candidate = Line(samples[start].1, samples[end].1);
            exact_parameter_line_from_samples(&samples, surface, candidate)
        })
        .next()
        .map(|line| ParameterCurve::new(line, surface.clone()))
}

fn exact_boundary_on_homogeneous_surface_only<C, S>(
    curve: &C,
    surface: &S,
) -> Option<ParameterCurve<Line<Point2>, S>>
where
    C: ParametricCurve3D
        + BoundedCurve<Point = Point3>
        + SearchParameter<CurveParameter, Point = Point3>
        + TryIntoHomogeneousBsplineCurve,
    S: Clone
        + ParametricSurface3D
        + SearchParameter<D2, Point = Point3>
        + TryIntoHomogeneousBsplineSurface,
{
    let hom_surface = surface.try_into_homogeneous_bspline_surface()?;
    let (u_range, v_range) = surface.try_range_tuple();
    let ranges = (u_range?, v_range?);
    let last_u = hom_surface.control_points().len().checked_sub(1)?;
    let last_v = hom_surface.control_points().first()?.len().checked_sub(1)?;
    let boundary = exact_boundary_line_on_surface(
        curve,
        last_u,
        last_v,
        |index| NurbsCurve::new(hom_surface.curve_v(index)),
        |index| NurbsCurve::new(hom_surface.curve_u(index)),
        ranges,
    )?;
    // The row/column curves parameterize the border in the homogeneous
    // B-spline's knot space, which need not agree with the original
    // surface's parameterization along that border (a revolution surface
    // measures its angle in radians while the homogeneous form inherits the
    // normalized full-circle knots, so a quarter turn reads 0.25 instead of
    // pi/2). Accept the matched border line only when its image on the
    // original surface reproduces the curve; otherwise report no exact
    // boundary so the surface-search fallback derives true parameters.
    let mid = boundary.subs(0.5);
    (surface
        .subs(boundary.0.x, boundary.0.y)
        .near(&curve.front())
        && surface.subs(boundary.1.x, boundary.1.y).near(&curve.back())
        && curve
            .search_parameter(surface.subs(mid.x, mid.y), None, 100)
            .is_some())
    .then(|| ParameterCurve::new(boundary, surface.clone()))
}

fn exact_boundary_on_homogeneous_surface<C, S>(
    curve: &C,
    surface: &S,
) -> Option<ParameterCurve<Line<Point2>, S>>
where
    C: ParametricCurve3D
        + BoundedCurve<Point = Point3>
        + SearchParameter<D1, Point = Point3>
        + TryIntoHomogeneousBsplineCurve,
    S: Clone
        + ParametricSurface3D
        + SearchParameter<D2, Point = Point3>
        + TryIntoHomogeneousBsplineSurface,
{
    exact_boundary_on_homogeneous_surface_only(curve, surface)
        .or_else(|| exact_linear_boundary_by_surface_search(curve, surface))
}

fn exact_line_boundary_on_homogeneous_surface<C, S>(
    curve: &C,
    surface: &S,
) -> Option<ParameterCurve<Line<Point2>, S>>
where
    C: ParametricCurve3D
        + BoundedCurve<Point = Point3>
        + SearchParameter<D1, Point = Point3>
        + TryIntoHomogeneousBsplineCurve,
    S: Clone
        + ParametricSurface3D
        + SearchParameter<D2, Point = Point3>
        + TryIntoHomogeneousBsplineSurface,
{
    exact_line_boundary_by_surface_search(curve, surface)
        .or_else(|| exact_boundary_on_homogeneous_surface_only(curve, surface))
}

impl ExactParameterBoundary2D<Plane> for Line<Point3> {
    type BoundaryCurve = ParameterCurve<Line<Point2>, Plane>;

    fn exact_parameter_boundary_2d(&self, surface: &Plane) -> Option<Self::BoundaryCurve> {
        exact_line_boundary_on_plane(self, surface)
    }
}

impl ExactParameterBoundary2D<BsplineSurface<Point3>> for Line<Point3> {
    type BoundaryCurve = ParameterCurve<Line<Point2>, BsplineSurface<Point3>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &BsplineSurface<Point3>,
    ) -> Option<Self::BoundaryCurve> {
        (surface.degrees() == (1, 1))
            .then(|| exact_line_boundary_on_affine_surface(self, surface))
            .flatten()
    }
}

impl ExactParameterBoundary2D<BsplineSurface<Point3>> for BsplineCurve<Point3> {
    type BoundaryCurve = ParameterCurve<Line<Point2>, BsplineSurface<Point3>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &BsplineSurface<Point3>,
    ) -> Option<Self::BoundaryCurve> {
        exact_bspline_boundary_on_surface(self, surface)
    }
}

impl ExactParameterBoundary2D<NurbsSurface<Vector4>> for Line<Point3> {
    type BoundaryCurve = ParameterCurve<Line<Point2>, NurbsSurface<Vector4>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &NurbsSurface<Vector4>,
    ) -> Option<Self::BoundaryCurve> {
        exact_line_boundary_on_homogeneous_extrusion_surface(self, surface).or_else(|| {
            (surface.degrees() == (1, 1))
                .then(|| exact_line_boundary_on_affine_surface(self, surface))
                .flatten()
        })
    }
}

impl ExactParameterBoundary2D<NurbsSurface<Vector4>> for NurbsCurve<Vector4> {
    type BoundaryCurve = ParameterCurve<Line<Point2>, NurbsSurface<Vector4>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &NurbsSurface<Vector4>,
    ) -> Option<Self::BoundaryCurve> {
        exact_nurbs_boundary_on_surface(self, surface)
    }
}

impl<C> ExactParameterBoundary2D<RevolutionSurface<C>> for Line<Point3>
where C: Clone + ParametricCurve3D + BoundedCurve + TryIntoHomogeneousBsplineCurve
{
    type BoundaryCurve = ParameterCurve<Line<Point2>, RevolutionSurface<C>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &RevolutionSurface<C>,
    ) -> Option<Self::BoundaryCurve> {
        exact_line_boundary_on_homogeneous_surface(self, surface)
    }
}

impl<C> ExactParameterBoundary2D<RevolutionSurface<C>> for BsplineCurve<Point3>
where C: Clone + ParametricCurve3D + BoundedCurve + TryIntoHomogeneousBsplineCurve
{
    type BoundaryCurve = ParameterCurve<Line<Point2>, RevolutionSurface<C>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &RevolutionSurface<C>,
    ) -> Option<Self::BoundaryCurve> {
        exact_boundary_on_homogeneous_surface(self, surface)
    }
}

impl<C> ExactParameterBoundary2D<RevolutionSurface<C>> for NurbsCurve<Vector4>
where C: Clone + ParametricCurve3D + BoundedCurve + TryIntoHomogeneousBsplineCurve
{
    type BoundaryCurve = ParameterCurve<Line<Point2>, RevolutionSurface<C>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &RevolutionSurface<C>,
    ) -> Option<Self::BoundaryCurve> {
        exact_boundary_on_homogeneous_surface(self, surface)
    }
}

impl<C> ExactParameterBoundary2D<Processor<RevolutionSurface<C>, Matrix4>> for Line<Point3>
where C: Clone + ParametricCurve3D + BoundedCurve + TryIntoHomogeneousBsplineCurve
{
    type BoundaryCurve = ParameterCurve<Line<Point2>, Processor<RevolutionSurface<C>, Matrix4>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &Processor<RevolutionSurface<C>, Matrix4>,
    ) -> Option<Self::BoundaryCurve> {
        exact_line_boundary_on_homogeneous_surface(self, surface)
    }
}

impl<C> ExactParameterBoundary2D<Processor<RevolutionSurface<C>, Matrix4>> for BsplineCurve<Point3>
where C: Clone + ParametricCurve3D + BoundedCurve + TryIntoHomogeneousBsplineCurve
{
    type BoundaryCurve = ParameterCurve<Line<Point2>, Processor<RevolutionSurface<C>, Matrix4>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &Processor<RevolutionSurface<C>, Matrix4>,
    ) -> Option<Self::BoundaryCurve> {
        exact_boundary_on_homogeneous_surface(self, surface)
    }
}

impl<C> ExactParameterBoundary2D<Processor<RevolutionSurface<C>, Matrix4>> for NurbsCurve<Vector4>
where C: Clone + ParametricCurve3D + BoundedCurve + TryIntoHomogeneousBsplineCurve
{
    type BoundaryCurve = ParameterCurve<Line<Point2>, Processor<RevolutionSurface<C>, Matrix4>>;

    fn exact_parameter_boundary_2d(
        &self,
        surface: &Processor<RevolutionSurface<C>, Matrix4>,
    ) -> Option<Self::BoundaryCurve> {
        exact_boundary_on_homogeneous_surface(self, surface)
    }
}

impl<C, S0, S1, T, S> ExactParameterBoundary2D<S> for SurfaceCurve<C, S0, S1, T, T>
where
    T: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + Cut
        + Invertible
        + Clone,
    S0: PartialEq<S>,
    S1: PartialEq<S>,
{
    type BoundaryCurve = T;

    fn exact_parameter_boundary_2d(&self, surface: &S) -> Option<Self::BoundaryCurve> {
        if self.surface0() == surface {
            self.boundary0().cloned()
        } else if self.surface1() == surface {
            self.boundary1().cloned()
        } else {
            None
        }
    }
}

impl<C, S0, S1, T0, T1, S> ParameterBoundary2D<S> for SurfaceCurve<C, S0, S1, T0, T1>
where
    C: ParametricCurve3D + BoundedCurve,
    S0: ParametricSurface3D
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + PartialEq<S>,
    S1: ParametricSurface3D
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + PartialEq<S>,
    T0: ParameterBoundary2D<S>,
    T1: ParameterBoundary2D<S>,
    T0: Clone,
    T1: Clone,
{
    fn parameter_boundary_2d(&self, surface: &S, tolerance: f64) -> Option<Vec<Point2>> {
        if self.surface0() == surface {
            self.boundary0()
                .and_then(|boundary| boundary.parameter_boundary_2d(surface, tolerance))
                .or_else(|| {
                    self.try_parameter_division(self.range_tuple(), tolerance)?
                        .0
                        .into_iter()
                        .map(|t| self.search_triple(t, 100).map(|(_, uv0, _)| uv0))
                        .collect()
                })
        } else if self.surface1() == surface {
            self.boundary1()
                .and_then(|boundary| boundary.parameter_boundary_2d(surface, tolerance))
                .or_else(|| {
                    self.try_parameter_division(self.range_tuple(), tolerance)?
                        .0
                        .into_iter()
                        .map(|t| self.search_triple(t, 100).map(|(_, _, uv1)| uv1))
                        .collect()
                })
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests;
