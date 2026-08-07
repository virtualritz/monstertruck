//! Face-local parameter curves: extracting a 2-D trim for a 3-D curve on a
//! surface (exactly where possible, by sampling otherwise), and the public
//! traits that build trimmed topology out of them.

use super::content_hash::content_hash64;
use super::*;

type ModelTrimCurve = ParameterCurve<Curve2D, Box<Surface>>;
type ExactTrimCacheKey = (u64, u64);

thread_local! {
    static EXACT_NURBS_REVOLUTION_TRIM_CACHE: RefCell<HashMap<ExactTrimCacheKey, Option<ModelTrimCurve>>> =
        RefCell::new(HashMap::default());
}

pub(super) fn line_points(line: Line<Point2>, tolerance: f64) -> Vec<Point2> {
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

pub(super) fn parameter_curve_points(
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

pub(super) fn boundary_matches_surface_curve(
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

pub(super) fn direct_bspline_boundary_line(
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

pub(super) fn direct_nurbs_boundary_line(
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

pub(super) fn curve2d_from_sampled_boundary(points: Vec<Point2>) -> Option<Curve2D> {
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

pub(super) fn same_surface(lhs: &Surface, rhs: &Surface) -> bool {
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

pub(super) fn exact_line_boundary(
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

pub(super) fn exact_bspline_boundary(
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

pub(super) fn exact_nurbs_boundary(
    curve: &NurbsCurve<Vector4>,
    surface: &Surface,
) -> Option<ModelTrimCurve> {
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
