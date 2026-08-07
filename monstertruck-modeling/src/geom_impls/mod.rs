use crate::errors::Error;
use crate::*;
use itertools::Itertools;
use std::f64::consts::PI;

pub(super) fn circle_arc_by_three_points(
    point0: Point3,
    point1: Point3,
    transit: Point3,
) -> Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4> {
    let origin = circum_center(point0, point1, transit);
    let (vec0, vec1) = (point0 - transit, point1 - transit);
    let axis = vec1.cross(vec0).normalize();
    let angle = Rad(PI) - vec0.angle(vec1);
    circle_arc(point0, origin, axis, angle * 2.0)
}

/// Constructs the unique circular arc that starts at `point0` with the
/// given start `tangent`, ends at `point1`, and turns less than a full
/// revolution.
///
/// Returns:
/// - [`Error::DegenerateCircularArcTangent`] if `tangent` is zero
///   or near-zero.
/// - [`Error::CircularArcTangentParallelToChord`] if `tangent` is
///   parallel (or anti-parallel) to `point1 - point0` -- the plane of
///   the arc would be under-determined and the radius infinite.
pub(super) fn try_circle_arc_by_start_tangent(
    point0: Point3,
    point1: Point3,
    tangent: Vector3,
) -> std::result::Result<Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4>, Error> {
    let chord = point1 - point0;
    if tangent.magnitude2().so_small() {
        return Err(Error::DegenerateCircularArcTangent);
    }
    let tangent = tangent.normalize();
    let axis_raw = tangent.cross(chord);
    if axis_raw.magnitude2().so_small() {
        return Err(Error::CircularArcTangentParallelToChord);
    }
    let axis = axis_raw.normalize();
    let to_origin = axis.cross(tangent);
    let radius = chord.dot(chord) / (2.0 * chord.dot(to_origin));
    let origin = point0 + radius * to_origin;
    let (vec0, vec1) = (point0 - origin, point1 - origin);
    let mut angle = f64::atan2(axis.dot(vec0.cross(vec1)), vec0.dot(vec1));
    if angle <= 0.0 {
        angle += 2.0 * PI;
    }
    Ok(circle_arc(point0, origin, axis, Rad(angle)))
}

fn circum_center(pt0: Point3, pt1: Point3, pt2: Point3) -> Point3 {
    let (vec0, vec1) = (pt1 - pt0, pt2 - pt0);
    let (a2, ab, b2) = (vec0.dot(vec0), vec0.dot(vec1), vec1.dot(vec1));
    let (det, u, v) = (a2 * b2 - ab * ab, a2 * b2 - ab * b2, a2 * b2 - ab * a2);
    pt0 + u / (2.0 * det) * vec0 + v / (2.0 * det) * vec1
}

pub(super) fn circle_arc(
    point: Point3,
    origin: Point3,
    axis: Vector3,
    angle: Rad<f64>,
) -> Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4> {
    let origin = origin + (axis.dot(point - origin)) * axis;
    let diag = point - origin;
    let axis_trsf = Matrix4::from_cols(
        diag.extend(0.0),
        axis.cross(diag).extend(0.0),
        axis.extend(0.0),
        origin.to_homogeneous(),
    );
    let unit_arc = TrimmedCurve::new(UnitCircle::new(), (0.0, angle.0));
    Processor::with_transform(unit_arc, axis_trsf)
}

fn closed_polyline_orientation<'a>(pts: impl IntoIterator<Item = &'a Vec<Point3>>) -> bool {
    pts.into_iter()
        .flat_map(|vec| vec.iter().circular_tuple_windows())
        .map(|(p0, p1)| (p1[0] + p0[0]) * (p1[1] - p0[1]))
        .sum::<f64>()
        >= 0.0
}

fn take_one_axis_by_normal(n: Vector3) -> Vector3 {
    let a = n.map(f64::abs);
    if a.x > a.z || a.y > a.z {
        Vector3::new(-n.y, n.x, 0.0).normalize()
    } else {
        Vector3::new(-n.z, 0.0, n.x).normalize()
    }
}

pub(super) fn attach_plane(mut pts: Vec<Vec<Point3>>) -> Option<Plane> {
    let center = pts
        .iter()
        .flatten()
        .fold(Point3::origin(), |sum, pt| sum + pt.to_vec())
        / pts.len() as f64;
    let normal = pts
        .iter()
        .flat_map(|vec| vec.iter().circular_tuple_windows())
        .fold(Vector3::zero(), |sum, (p0, p1)| {
            sum + (p0 - center).cross(p1 - center)
        });
    let n = match normal.so_small() {
        true => return None,
        false => normal.normalize(),
    };
    let a = take_one_axis_by_normal(n);
    let mat: Matrix4 = Matrix3::from_cols(a, n.cross(a), n).into();
    pts.iter_mut()
        .flatten()
        .for_each(|pt| *pt = mat.invert().unwrap().transform_point(*pt));
    let bnd_box: BoundingBox<Point3> = pts.iter().flatten().collect();
    let diag = bnd_box.diagonal();
    if !diag[2].so_small() {
        return None;
    }
    let (max, min) = match closed_polyline_orientation(&pts) {
        true => (bnd_box.max(), bnd_box.min()),
        false => (bnd_box.min(), bnd_box.max()),
    };
    let plane = Plane::new(
        Point3::new(min[0], min[1], min[2]),
        Point3::new(max[0], min[1], min[2]),
        Point3::new(min[0], max[1], min[2]),
    )
    .transformed(mat);
    Some(plane)
}

#[cfg(test)]
mod test_geom_impl;

impl<T: Clone> GeometricMapping<T> for () {
    #[inline]
    fn mapping(self) -> impl Fn(&T) -> T { Clone::clone }
}
impl<T: Transformed<Matrix4>> GeometricMapping<T> for Matrix4 {
    #[inline]
    fn mapping(self) -> impl Fn(&T) -> T { move |t| t.transformed(self) }
}
impl<T> GeometricMapping<T> for fn(&T) -> T {
    #[inline]
    fn mapping(self) -> impl Fn(&T) -> T { self }
}

impl<T, H> Connector<T, H> for fn(&T, &T) -> H {
    #[inline]
    fn connector(self) -> impl Fn(&T, &T) -> H { self }
}

#[derive(Debug, Clone, Copy)]
pub struct LineConnector;

impl<C> Connector<Point3, C> for LineConnector
where Line<Point3>: ToSameGeometry<C>
{
    fn connector(self) -> impl Fn(&Point3, &Point3) -> C { |p, q| Line(*p, *q).to_same_geometry() }
}

#[derive(Debug, Clone, Copy)]
pub struct ExtrudeConnector {
    pub vector: Vector3,
}

impl<C, S> Connector<C, S> for ExtrudeConnector
where
    C: Clone,
    ExtrusionSurface<C, Vector3>: ToSameGeometry<S>,
{
    fn connector(self) -> impl Fn(&C, &C) -> S {
        move |curve0, _| {
            ExtrusionSurface::by_extrusion(curve0.clone(), self.vector).to_same_geometry()
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ArcConnector {
    pub origin: Point3,
    pub axis: Vector3,
    pub angle: Rad<f64>,
}

impl<C> Connector<Point3, C> for ArcConnector
where Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4>: ToSameGeometry<C>
{
    fn connector(self) -> impl Fn(&Point3, &Point3) -> C {
        let Self {
            origin,
            axis,
            angle,
        } = self;
        move |p, _| circle_arc(*p, origin, axis, angle).to_same_geometry()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RevoluteConnector {
    pub origin: Point3,
    pub axis: Vector3,
}

impl<C, S> Connector<C, S> for RevoluteConnector
where
    C: Clone,
    RevolutionSurface<C>: ToSameGeometry<S>,
{
    fn connector(self) -> impl Fn(&C, &C) -> S {
        let Self { origin, axis } = self;
        move |curve, _| {
            RevolutionSurface::by_revolution(curve.clone(), origin, axis).to_same_geometry()
        }
    }
}
