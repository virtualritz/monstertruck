use monstertruck_geometry::prelude as geom;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::result::Result;
use step_p21::{Holder, ast::Name, primitive::Logical, tables::PlaceHolder};

use super::Table;
use super::step_geometry::{self, *};

/// Undefined structures are parsed into this.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = dummy)]
#[holder(generate_deserialize)]
pub struct Dummy {
    pub record: String,
    pub is_simple: bool,
}

/// Many geometric and topological elements are contained within this entity's child classes.
/// Since it is essentially an `Any` type, one must manually map the reference according to the context.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = representation_item)]
#[holder(generate_deserialize)]
pub struct RepresentationItem {
    pub name: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = representation_context)]
#[holder(generate_deserialize)]
pub struct RepresentationContext {
    pub context_identifier: String,
    pub context_type: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = representation)]
#[holder(generate_deserialize)]
pub struct Representation {
    pub name: String,
    #[holder(use_place_holder)]
    pub items: Vec<RepresentationItem>,
    #[holder(use_place_holder)]
    pub context_of_items: Vec<RepresentationContext>,
}

/// `cartesian_point`
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = cartesian_point)]
#[holder(generate_deserialize)]
pub struct CartesianPoint {
    pub label: String,
    pub coordinates: Vec<f64>,
}
impl From<&CartesianPoint> for Point2 {
    #[inline(always)]
    fn from(pt: &CartesianPoint) -> Self {
        let pt = &pt.coordinates;
        match pt.len() {
            0 => Point2::origin(),
            1 => Point2::new(pt[0], 0.0),
            _ => Point2::new(pt[0], pt[1]),
        }
    }
}
impl From<&CartesianPoint> for Point3 {
    #[inline(always)]
    fn from(pt: &CartesianPoint) -> Self {
        let pt = &pt.coordinates;
        match pt.len() {
            0 => Point3::origin(),
            1 => Point3::new(pt[0], 0.0, 0.0),
            2 => Point3::new(pt[0], pt[1], 0.0),
            _ => Point3::new(pt[0], pt[1], pt[2]),
        }
    }
}

/// `direction`
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = direction)]
#[holder(generate_deserialize)]
pub struct Direction {
    pub label: String,
    pub direction_ratios: Vec<f64>,
}
impl From<&Direction> for Vector2 {
    #[inline(always)]
    fn from(dir: &Direction) -> Self {
        let dir = &dir.direction_ratios;
        match dir.len() {
            0 => Vector2::zero(),
            1 => Vector2::new(dir[0], 0.0),
            _ => Vector2::new(dir[0], dir[1]),
        }
    }
}
impl From<&Direction> for Vector3 {
    #[inline(always)]
    fn from(dir: &Direction) -> Self {
        let dir = &dir.direction_ratios;
        match dir.len() {
            0 => Vector3::zero(),
            1 => Vector3::new(dir[0], 0.0, 0.0),
            2 => Vector3::new(dir[0], dir[1], 0.0),
            _ => Vector3::new(dir[0], dir[1], dir[2]),
        }
    }
}

/// `vector`
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = vector)]
#[holder(generate_deserialize)]
pub struct Vector {
    pub label: String,
    #[holder(use_place_holder)]
    pub orientation: Direction,
    pub magnitude: f64,
}
impl From<&Vector> for Vector2 {
    #[inline(always)]
    fn from(vec: &Vector) -> Self { Self::from(&vec.orientation) * vec.magnitude }
}
impl From<&Vector> for Vector3 {
    #[inline(always)]
    fn from(vec: &Vector) -> Self { Self::from(&vec.orientation) * vec.magnitude }
}

/// `placement`
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = placement)]
#[holder(generate_deserialize)]
pub struct Placement {
    pub label: String,
    #[holder(use_place_holder)]
    pub location: CartesianPoint,
}
impl From<&Placement> for Point2 {
    #[inline(always)]
    fn from(p: &Placement) -> Self { Self::from(&p.location) }
}
impl From<&Placement> for Point3 {
    #[inline(always)]
    fn from(p: &Placement) -> Self { Self::from(&p.location) }
}

/// `axis1_placement`
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = axis1_placement)]
#[holder(generate_deserialize)]
pub struct Axis1Placement {
    pub label: String,
    #[holder(use_place_holder)]
    pub location: CartesianPoint,
    #[holder(use_place_holder)]
    pub direction: Option<Direction>,
}

impl Axis1Placement {
    pub fn direction(&self) -> Vector3 {
        self.direction
            .as_ref()
            .map(Vector3::from)
            .unwrap_or_else(Vector3::unit_z)
    }
}

/// `axis2_placement`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum Axis2Placement {
    #[holder(use_place_holder)]
    Axis2Placement2d(Axis2Placement2d),
    #[holder(use_place_holder)]
    Axis2Placement3d(Axis2Placement3d),
}

impl TryFrom<&Axis2Placement> for Matrix3 {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(axis: &Axis2Placement) -> Result<Self, StepConvertingError> {
        use Axis2Placement::*;
        match axis {
            Axis2Placement2d(axis) => Ok(Matrix3::from(axis)),
            Axis2Placement3d(_) => Err("This is not a 2D axis placement.".into()),
        }
    }
}
impl TryFrom<&Axis2Placement> for Matrix4 {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(axis: &Axis2Placement) -> Result<Self, StepConvertingError> {
        use Axis2Placement::*;
        match axis {
            Axis2Placement2d(_) => Err("This is not a 3D axis placement.".into()),
            Axis2Placement3d(axis) => Ok(Matrix4::from(axis)),
        }
    }
}

/// `axis2_placement_2d`
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = axis2_placement_2d)]
#[holder(generate_deserialize)]
pub struct Axis2Placement2d {
    pub label: String,
    #[holder(use_place_holder)]
    pub location: CartesianPoint,
    #[holder(use_place_holder)]
    pub ref_direction: Option<Direction>,
}

impl From<&Axis2Placement2d> for Matrix3 {
    #[inline(always)]
    fn from(axis: &Axis2Placement2d) -> Self {
        let z = Point2::from(&axis.location);
        let x = match &axis.ref_direction {
            Some(axis) => Vector2::from(axis),
            None => Vector2::unit_x(),
        };
        let y = Vector2::new(-x.y, x.x);
        Matrix3::from_cols(x.extend(0.0), y.extend(0.0), z.to_vec().extend(1.0))
    }
}

/// `axis2_placement_3d`
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = axis2_placement_3d)]
#[holder(generate_deserialize)]
pub struct Axis2Placement3d {
    pub label: String,
    #[holder(use_place_holder)]
    pub location: CartesianPoint,
    #[holder(use_place_holder)]
    pub axis: Option<Direction>,
    #[holder(use_place_holder)]
    pub ref_direction: Option<Direction>,
}

impl From<&Axis2Placement3d> for Matrix4 {
    #[inline(always)]
    fn from(axis: &Axis2Placement3d) -> Matrix4 {
        let w = Point3::from(&axis.location);
        let z = match &axis.axis {
            Some(axis) => Vector3::from(axis),
            None => Vector3::unit_z(),
        };
        // Pick a fallback reference direction that is not parallel to `z`.
        let fallback = match z.near(&Vector3::unit_x()) {
            true => Vector3::unit_y(),
            false => Vector3::unit_x(),
        };
        let x = match &axis.ref_direction {
            Some(axis) => Vector3::from(axis),
            None => fallback,
        };
        // Gram-Schmidt: remove the `z` component of `x`. ISO 10303 permits
        // `ref_direction` to be parallel to `axis`; in that case the projected
        // vector is zero and `normalize()` would produce `NaN`, which later
        // panics in `nonpositive_tolerance!` during meshing. Fall back to an
        // arbitrary direction orthogonal to `z` when the projection degenerates.
        let projected = x - x.dot(z) * z;
        let x = match projected.magnitude2().so_small() {
            true => (fallback - fallback.dot(z) * z).normalize(),
            false => projected.normalize(),
        };
        let y = z.cross(x);
        Matrix4::from_cols(
            x.extend(0.0),
            y.extend(0.0),
            z.extend(0.0),
            w.to_vec().extend(1.0),
        )
    }
}

/// `curve`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum CurveAny {
    #[holder(use_place_holder)]
    Line(Box<Line>),
    #[holder(use_place_holder)]
    BoundedCurve(Box<BoundedCurveAny>),
    #[holder(use_place_holder)]
    Conic(Box<Conic>),
    #[holder(use_place_holder)]
    Pcurve(Box<Pcurve>),
    #[holder(use_place_holder)]
    SurfaceCurve(Box<SurfaceCurve>),
}

impl TryFrom<&CurveAny> for Curve2D {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(curve: &CurveAny) -> Result<Self, Self::Error> {
        use CurveAny::*;
        Ok(match curve {
            Line(line) => Self::Line(line.as_ref().into()),
            BoundedCurve(b) => b.as_ref().try_into()?,
            Conic(curve) => Self::Conic(curve.as_ref().try_into()?),
            Pcurve(_) => return Err("Pcurves cannot be parsed to 2D curves.".into()),
            SurfaceCurve(_) => return Err("Surface curves cannot be parsed to 2D curves.".into()),
        })
    }
}

impl TryFrom<&CurveAny> for Curve3D {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(curve: &CurveAny) -> Result<Self, Self::Error> {
        use CurveAny::*;
        Ok(match curve {
            Line(line) => Self::Line(line.as_ref().into()),
            BoundedCurve(b) => b.as_ref().try_into()?,
            Conic(curve) => Self::Conic(curve.as_ref().try_into()?),
            Pcurve(c) => Self::ParameterCurve(c.as_ref().try_into()?),
            SurfaceCurve(c) => c.as_ref().try_into()?,
        })
    }
}

/// `line`
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = line)]
#[holder(generate_deserialize)]
pub struct Line {
    pub label: String,
    #[holder(use_place_holder)]
    pub pnt: CartesianPoint,
    #[holder(use_place_holder)]
    pub dir: Vector,
}
impl<'a, P> From<&'a Line> for geom::Line<P>
where
    P: EuclideanSpace + From<&'a CartesianPoint>,
    P::Diff: From<&'a Vector>,
{
    #[inline(always)]
    fn from(line: &'a Line) -> Self {
        let p = P::from(&line.pnt);
        let q = p + P::Diff::from(&line.dir);
        Self(p, q)
    }
}

/// `bounded_curve`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum BoundedCurveAny {
    #[holder(use_place_holder)]
    Polyline(Box<Polyline>),
    #[holder(use_place_holder)]
    BsplineCurve(Box<BsplineCurveAny>),
}

impl TryFrom<&BoundedCurveAny> for Curve2D {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &BoundedCurveAny) -> Result<Self, Self::Error> {
        use BoundedCurveAny::*;
        Ok(match value {
            Polyline(x) => Self::Polyline(x.as_ref().into()),
            BsplineCurve(x) => x.as_ref().try_into()?,
        })
    }
}

impl TryFrom<&BoundedCurveAny> for Curve3D {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &BoundedCurveAny) -> Result<Self, Self::Error> {
        use BoundedCurveAny::*;
        Ok(match value {
            Polyline(x) => Self::Polyline(x.as_ref().into()),
            BsplineCurve(x) => x.as_ref().try_into()?,
        })
    }
}

/// `polyline`
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = polyline)]
#[holder(generate_deserialize)]
pub struct Polyline {
    pub label: String,
    #[holder(use_place_holder)]
    pub points: Vec<CartesianPoint>,
}
impl<'a, P: From<&'a CartesianPoint>> From<&'a Polyline> for PolylineCurve<P> {
    #[inline(always)]
    fn from(poly: &'a Polyline) -> Self { Self(poly.points.iter().map(|pt| P::from(pt)).collect()) }
}

/// `b_spline_curve_form`
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum BsplineCurveForm {
    PolylineForm,
    CircularArc,
    EllipticArc,
    ParabolicArc,
    HyperbolicArc,
    Unspecified,
}

/// `knot_type`
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum KnotType {
    UniformKnots,
    Unspecified,
    QuasiUniformKnots,
    PiecewiseBezierKnots,
}

/// `b_spline_curve_with_knots`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = b_spline_curve_with_knots)]
#[holder(generate_deserialize)]
pub struct BsplineCurveWithKnots {
    pub label: String,
    pub degree: i64,
    #[holder(use_place_holder)]
    pub control_points_list: Vec<CartesianPoint>,
    pub curve_form: BsplineCurveForm,
    pub closed_curve: Logical,
    pub self_intersect: Logical,
    pub knot_multiplicities: Vec<i64>,
    pub knots: Vec<f64>,
    pub knot_spec: KnotType,
}
impl<P: for<'a> From<&'a CartesianPoint>> TryFrom<&BsplineCurveWithKnots> for BsplineCurve<P> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(curve: &BsplineCurveWithKnots) -> Result<Self, StepConvertingError> {
        let knots = curve.knots.clone();
        let multi = curve
            .knot_multiplicities
            .iter()
            .map(|n| *n as usize)
            .collect();
        let knots = KnotVector::from_single_multi(knots, multi)?;
        let ctrpts = curve.control_points_list.iter().map(Into::into).collect();
        Ok(Self::try_new(knots, ctrpts)?)
    }
}

/// `bezier_curve`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = bezier_curve)]
#[holder(generate_deserialize)]
pub struct BezierCurve {
    pub label: String,
    pub degree: i64,
    #[holder(use_place_holder)]
    pub control_points_list: Vec<CartesianPoint>,
    pub curve_form: BsplineCurveForm,
    pub closed_curve: Logical,
    pub self_intersect: Logical,
}
impl<P: for<'a> From<&'a CartesianPoint>> TryFrom<&BezierCurve> for BsplineCurve<P> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(curve: &BezierCurve) -> Result<Self, StepConvertingError> {
        let degree = curve.degree as usize;
        let knots = KnotVector::bezier_knot(degree);
        let ctrpts = curve.control_points_list.iter().map(Into::into).collect();
        Ok(Self::try_new(knots, ctrpts)?)
    }
}

/// `quasi_uniform_curve`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = quasi_uniform_curve)]
#[holder(generate_deserialize)]
pub struct QuasiUniformCurve {
    pub label: String,
    pub degree: i64,
    #[holder(use_place_holder)]
    pub control_points_list: Vec<CartesianPoint>,
    pub curve_form: BsplineCurveForm,
    pub closed_curve: Logical,
    pub self_intersect: Logical,
}
impl<P: for<'a> From<&'a CartesianPoint>> TryFrom<&QuasiUniformCurve> for BsplineCurve<P> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(curve: &QuasiUniformCurve) -> Result<Self, StepConvertingError> {
        let knots = quasi_uniform_knots(curve.control_points_list.len(), curve.degree as usize);
        let ctrpts = curve.control_points_list.iter().map(Into::into).collect();
        Ok(Self::try_new(knots, ctrpts)?)
    }
}

fn quasi_uniform_knots(num_ctrl: usize, degree: usize) -> KnotVector {
    let division = num_ctrl - degree;
    let mut knots = KnotVector::uniform_knot(degree, division);
    knots.transform(division as f64, 0.0);
    knots
}

/// `uniform_curve`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = uniform_curve)]
#[holder(generate_deserialize)]
pub struct UniformCurve {
    pub label: String,
    pub degree: i64,
    #[holder(use_place_holder)]
    pub control_points_list: Vec<CartesianPoint>,
    pub curve_form: BsplineCurveForm,
    pub closed_curve: Logical,
    pub self_intersect: Logical,
}
impl<P: for<'a> From<&'a CartesianPoint>> TryFrom<&UniformCurve> for BsplineCurve<P> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(curve: &UniformCurve) -> Result<Self, StepConvertingError> {
        let knots = uniform_knots(curve.control_points_list.len(), curve.degree as usize)?;
        let ctrpts = curve.control_points_list.iter().map(Into::into).collect();
        Ok(Self::try_new(knots, ctrpts)?)
    }
}

fn uniform_knots(num_ctrl: usize, degree: usize) -> geom::Result<KnotVector> {
    KnotVector::try_from(
        (0..degree + num_ctrl + 1)
            .map(|i| i as f64 - degree as f64)
            .collect::<Vec<_>>(),
    )
}

/// Entity that does not exist in AP042.
/// Curve before rationalization of [`RationalBsplineCurve`] defined by a complex entity
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum NonRationalBsplineCurve {
    #[holder(use_place_holder)]
    BsplineCurveWithKnots(BsplineCurveWithKnots),
    #[holder(use_place_holder)]
    BezierCurve(BezierCurve),
    #[holder(use_place_holder)]
    QuasiUniformCurve(QuasiUniformCurve),
    #[holder(use_place_holder)]
    UniformCurve(UniformCurve),
}

impl<P: for<'a> From<&'a CartesianPoint>> TryFrom<&NonRationalBsplineCurve> for BsplineCurve<P> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(curve: &NonRationalBsplineCurve) -> Result<Self, StepConvertingError> {
        use NonRationalBsplineCurve::*;
        match curve {
            BsplineCurveWithKnots(x) => x.try_into(),
            BezierCurve(x) => x.try_into(),
            QuasiUniformCurve(x) => x.try_into(),
            UniformCurve(x) => x.try_into(),
        }
    }
}

/// `rational_b_spline_curve` as complex entity
///
/// This struct is an ad hoc implementation that differs from the definition by EXPRESS:
/// in AP042, rationalized curves are defined as complex entities,
/// but here the curves before rationalization are held as internal variables.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = rational_b_spline_curve)]
#[holder(generate_deserialize)]
pub struct RationalBsplineCurve {
    #[holder(use_place_holder)]
    pub non_rational_b_spline_curve: NonRationalBsplineCurve,
    pub weights_data: Vec<f64>,
}
impl<V> TryFrom<&RationalBsplineCurve> for NurbsCurve<V>
where
    V: Homogeneous<Scalar = f64>,
    V::Point: for<'a> From<&'a CartesianPoint>,
{
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(curve: &RationalBsplineCurve) -> Result<Self, StepConvertingError> {
        Ok(Self::try_from_bspline_and_weights(
            BsplineCurve::try_from(&curve.non_rational_b_spline_curve)?,
            curve.weights_data.clone(),
        )?)
    }
}

/// b_spline_curve
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum BsplineCurveAny {
    #[holder(use_place_holder)]
    NonRationalBsplineCurve(Box<NonRationalBsplineCurve>),
    #[holder(use_place_holder)]
    RationalBsplineCurve(Box<RationalBsplineCurve>),
}

impl TryFrom<&BsplineCurveAny> for Curve2D {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &BsplineCurveAny) -> Result<Self, Self::Error> {
        use BsplineCurveAny::*;
        Ok(match value {
            NonRationalBsplineCurve(bsp) => Self::BsplineCurve(bsp.as_ref().try_into()?),
            RationalBsplineCurve(bsp) => Self::NurbsCurve(bsp.as_ref().try_into()?),
        })
    }
}

impl TryFrom<&BsplineCurveAny> for Curve3D {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &BsplineCurveAny) -> Result<Self, Self::Error> {
        use BsplineCurveAny::*;
        Ok(match value {
            NonRationalBsplineCurve(bsp) => Self::BsplineCurve(bsp.as_ref().try_into()?),
            RationalBsplineCurve(bsp) => Self::NurbsCurve(bsp.as_ref().try_into()?),
        })
    }
}

/// `conic`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum Conic {
    #[holder(use_place_holder)]
    Circle(Circle),
    #[holder(use_place_holder)]
    Ellipse(Ellipse),
    #[holder(use_place_holder)]
    Hyperbola(Hyperbola),
    #[holder(use_place_holder)]
    Parabola(Parabola),
}

impl TryFrom<&Conic> for Conic2D {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &Conic) -> Result<Self, Self::Error> {
        Ok(match value {
            Conic::Circle(value) => Conic2D::Ellipse(value.try_into()?),
            Conic::Ellipse(value) => Conic2D::Ellipse(value.try_into()?),
            Conic::Hyperbola(value) => Conic2D::Hyperbola(value.try_into()?),
            Conic::Parabola(value) => Conic2D::Parabola(value.try_into()?),
        })
    }
}

impl TryFrom<&Conic> for Conic3D {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &Conic) -> Result<Self, Self::Error> {
        Ok(match value {
            Conic::Circle(value) => Conic3D::Ellipse(value.try_into()?),
            Conic::Ellipse(value) => Conic3D::Ellipse(value.try_into()?),
            Conic::Hyperbola(value) => Conic3D::Hyperbola(value.try_into()?),
            Conic::Parabola(value) => Conic3D::Parabola(value.try_into()?),
        })
    }
}

/// `circle`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = circle)]
#[holder(generate_deserialize)]
pub struct Circle {
    pub label: String,
    #[holder(use_place_holder)]
    pub position: Axis2Placement,
    pub radius: f64,
}

impl TryFrom<&Circle> for step_geometry::Ellipse<Point2, Matrix3> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(circle: &Circle) -> Result<Self, Self::Error> {
        let transform = Matrix3::try_from(&circle.position)? * Matrix3::from_scale(circle.radius);
        Ok(
            Processor::new(geom::TrimmedCurve::new(UnitCircle::new(), (0.0, 2.0 * PI)))
                .transformed(transform),
        )
    }
}

impl TryFrom<&Circle> for step_geometry::Ellipse<Point3, Matrix4> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(circle: &Circle) -> Result<Self, Self::Error> {
        let transform = Matrix4::try_from(&circle.position)? * Matrix4::from_scale(circle.radius);
        Ok(
            Processor::new(geom::TrimmedCurve::new(UnitCircle::new(), (0.0, 2.0 * PI)))
                .transformed(transform),
        )
    }
}

/// `ellipse`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = ellipse)]
#[holder(generate_deserialize)]
pub struct Ellipse {
    pub label: String,
    #[holder(use_place_holder)]
    pub position: Axis2Placement,
    pub semi_axis_1: f64,
    pub semi_axis_2: f64,
}

impl TryFrom<&Ellipse> for step_geometry::Ellipse<Point2, Matrix3> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(ellipse: &Ellipse) -> Result<Self, Self::Error> {
        let (r0, r1) = (ellipse.semi_axis_1, ellipse.semi_axis_2);
        let transform =
            Matrix3::try_from(&ellipse.position)? * Matrix3::from_nonuniform_scale(r0, r1);
        Ok(
            Processor::new(geom::TrimmedCurve::new(UnitCircle::new(), (0.0, 2.0 * PI)))
                .transformed(transform),
        )
    }
}

impl TryFrom<&Ellipse> for step_geometry::Ellipse<Point3, Matrix4> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(ellipse: &Ellipse) -> Result<Self, Self::Error> {
        let (r0, r1) = (ellipse.semi_axis_1, ellipse.semi_axis_2);
        let transform = Matrix4::try_from(&ellipse.position)?
            * Matrix4::from_nonuniform_scale(r0, r1, f64::min(r0, r1));
        Ok(
            Processor::new(geom::TrimmedCurve::new(UnitCircle::new(), (0.0, 2.0 * PI)))
                .transformed(transform),
        )
    }
}

/// `hyperbola`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = hyperbola)]
#[holder(generate_deserialize)]
pub struct Hyperbola {
    pub label: String,
    #[holder(use_place_holder)]
    pub position: Axis2Placement,
    pub semi_axis: f64,
    pub semi_imag_axis: f64,
}

impl TryFrom<&Hyperbola> for step_geometry::Hyperbola<Point2, Matrix3> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(hyperbola: &Hyperbola) -> Result<Self, Self::Error> {
        let (r0, r1) = (hyperbola.semi_axis, hyperbola.semi_imag_axis);
        let transform =
            Matrix3::try_from(&hyperbola.position)? * Matrix3::from_nonuniform_scale(r0, r1);
        Ok(
            Processor::new(geom::TrimmedCurve::new(UnitHyperbola::new(), (-1.0, 1.0)))
                .transformed(transform),
        )
    }
}

impl TryFrom<&Hyperbola> for step_geometry::Hyperbola<Point3, Matrix4> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(hyperbola: &Hyperbola) -> Result<Self, Self::Error> {
        let (r0, r1) = (hyperbola.semi_axis, hyperbola.semi_imag_axis);
        let transform = Matrix4::try_from(&hyperbola.position)?
            * Matrix4::from_nonuniform_scale(r0, r1, f64::min(r0, r1));
        Ok(
            Processor::new(geom::TrimmedCurve::new(UnitHyperbola::new(), (-1.0, 1.0)))
                .transformed(transform),
        )
    }
}

/// `parabola`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = parabola)]
#[holder(generate_deserialize)]
pub struct Parabola {
    pub label: String,
    #[holder(use_place_holder)]
    pub position: Axis2Placement,
    pub focal_dist: f64,
}

impl TryFrom<&Parabola> for step_geometry::Parabola<Point2, Matrix3> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(parabola: &Parabola) -> Result<Self, Self::Error> {
        let transform =
            Matrix3::try_from(&parabola.position)? * Matrix3::from_scale(parabola.focal_dist);
        Ok(
            Processor::new(geom::TrimmedCurve::new(UnitParabola::new(), (-1.0, 1.0)))
                .transformed(transform),
        )
    }
}

impl TryFrom<&Parabola> for step_geometry::Parabola<Point3, Matrix4> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(parabola: &Parabola) -> Result<Self, Self::Error> {
        let transform =
            Matrix4::try_from(&parabola.position)? * Matrix4::from_scale(parabola.focal_dist);
        Ok(
            Processor::new(geom::TrimmedCurve::new(UnitParabola::new(), (-1.0, 1.0)))
                .transformed(transform),
        )
    }
}

/// Element of a [`GeometricCurveSet`] -- a point or curve reference.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum GeometricSetSelect {
    #[holder(use_place_holder)]
    Curve(Box<CurveAny>),
    #[holder(use_place_holder)]
    Point(Box<CartesianPoint>),
}

/// `geometric_curve_set` -- a set of standalone 3D curves (and optionally points).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = geometric_curve_set)]
#[holder(generate_deserialize)]
pub struct GeometricCurveSet {
    /// Label from the STEP entity.
    pub label: String,
    /// Elements of the set (curves and/or points).
    #[holder(use_place_holder)]
    pub elements: Vec<GeometricSetSelect>,
}

/// `definitional_representation`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = definitional_representation)]
#[holder(generate_deserialize)]
pub struct DefinitionalRepresentation {
    label: String,
    #[holder(use_place_holder)]
    representation_item: Vec<CurveAny>,
    #[holder(use_place_holder)]
    context_of_items: Dummy,
}

/// `pcurve`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = pcurve)]
#[holder(generate_deserialize)]
pub struct Pcurve {
    label: String,
    #[holder(use_place_holder)]
    basis_surface: SurfaceAny,
    #[holder(use_place_holder)]
    reference_to_curve: DefinitionalRepresentation,
}

impl TryFrom<&Pcurve> for step_geometry::StepParameterCurve {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &Pcurve) -> Result<Self, Self::Error> {
        let surface: Surface = (&value.basis_surface).try_into()?;
        let curve: Curve2D = value
            .reference_to_curve
            .representation_item
            .first()
            .ok_or("no representation item")?
            .try_into()?;
        Ok(step_geometry::StepParameterCurve::new(
            Box::new(curve),
            Box::new(surface),
        ))
    }
}

/// `pcurve_or_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum PcurveOrSurface {
    #[holder(use_place_holder)]
    Pcurve(Box<Pcurve>),
    #[holder(use_place_holder)]
    Surface(Box<SurfaceAny>),
}

/// `preferred_surface_representation`
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum PreferredSurfaceCurveRepresentation {
    Curve3D,
    PcurveS1,
    PcurveS2,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SurfaceCurveEntityKind {
    #[default]
    Surface,
    Seam,
    Intersection,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
pub(crate) struct SurfaceCurveParams(
    String,
    PlaceHolder<CurveAnyHolder>,
    Vec<PlaceHolder<PcurveOrSurfaceHolder>>,
    PreferredSurfaceCurveRepresentation,
);

impl SurfaceCurveParams {
    pub(crate) fn into_holder(self, kind: SurfaceCurveEntityKind) -> SurfaceCurveHolder {
        let SurfaceCurveParams(label, curve_3d, associated_geometry, master_representation) = self;
        SurfaceCurveHolder {
            label,
            curve_3d,
            associated_geometry,
            master_representation,
            kind,
        }
    }
}

#[test]
fn deserialize_pscr() {
    let (_, p) = step_p21::parser::exchange::parameter(".PCURVE_S1.").unwrap();
    let x = PreferredSurfaceCurveRepresentation::deserialize(&p).unwrap();
    assert!(matches!(x, PreferredSurfaceCurveRepresentation::PcurveS1));
    let (_, p) = step_p21::parser::exchange::parameter(".PCURVE_S2.").unwrap();
    let x = PreferredSurfaceCurveRepresentation::deserialize(&p).unwrap();
    assert!(matches!(x, PreferredSurfaceCurveRepresentation::PcurveS2));
}

/// `surface_curve`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = surface_curve)]
#[holder(generate_deserialize)]
pub struct SurfaceCurve {
    label: String,
    #[holder(use_place_holder)]
    curve_3d: CurveAny,
    #[holder(use_place_holder)]
    associated_geometry: Vec<PcurveOrSurface>,
    master_representation: PreferredSurfaceCurveRepresentation,
    #[serde(skip)]
    kind: SurfaceCurveEntityKind,
}

impl TryFrom<&SurfaceCurve> for Curve3D {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &SurfaceCurve) -> Result<Self, Self::Error> {
        let associated_surface =
            |entry: &PcurveOrSurface| -> Result<step_geometry::Surface, StepConvertingError> {
                match entry {
                    PcurveOrSurface::Pcurve(x) => (&x.basis_surface).try_into(),
                    PcurveOrSurface::Surface(x) => x.as_ref().try_into(),
                }
            };
        let associated_geometry = value
            .associated_geometry
            .iter()
            .map(|entry| match entry {
                PcurveOrSurface::Pcurve(curve) => curve
                    .as_ref()
                    .try_into()
                    .map(step_geometry::SurfaceCurveAssociatedGeometry::ParameterCurve),
                PcurveOrSurface::Surface(surface) => surface
                    .as_ref()
                    .try_into()
                    .map(Box::new)
                    .map(step_geometry::SurfaceCurveAssociatedGeometry::Surface),
            })
            .collect::<Result<Vec<_>, _>>()?;
        let kind = match value.kind {
            SurfaceCurveEntityKind::Surface => step_geometry::SurfaceCurveKind::SurfaceCurve,
            SurfaceCurveEntityKind::Seam => step_geometry::SurfaceCurveKind::SeamCurve,
            SurfaceCurveEntityKind::Intersection => {
                step_geometry::SurfaceCurveKind::IntersectionCurve
            }
        };
        let master_representation = match value.master_representation {
            PreferredSurfaceCurveRepresentation::Curve3D => {
                step_geometry::SurfaceCurveRepresentation::Curve3D
            }
            PreferredSurfaceCurveRepresentation::PcurveS1 => {
                step_geometry::SurfaceCurveRepresentation::ParameterCurve0
            }
            PreferredSurfaceCurveRepresentation::PcurveS2 => {
                step_geometry::SurfaceCurveRepresentation::ParameterCurve1
            }
        };
        let leader = (&value.curve_3d).try_into()?;
        let surface_curve = step_geometry::SurfaceCurve3D::new(
            kind,
            Box::new(leader),
            associated_geometry,
            master_representation,
        );
        if surface_curve.associated_geometry().len() >= 2
            && surface_curve.associated_geometry().iter().all(|entry| {
                matches!(
                    entry,
                    step_geometry::SurfaceCurveAssociatedGeometry::Surface(_)
                )
            })
        {
            let surface0 = value
                .associated_geometry
                .first()
                .ok_or("The 0-indexed associated geometry is missing.")?;
            let surface1 = value
                .associated_geometry
                .get(1)
                .ok_or("The 1-indexed associated geometry is missing.")?;
            Ok(Curve3D::IntersectionCurve(IntersectionCurve::new(
                Box::new(associated_surface(surface0)?),
                Box::new(associated_surface(surface1)?),
                Box::new(surface_curve.leader().clone()),
            )))
        } else {
            Ok(Curve3D::SurfaceCurve(surface_curve))
        }
    }
}

/// `surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum SurfaceAny {
    #[holder(use_place_holder)]
    ElementarySurface(Box<ElementarySurfaceAny>),
    #[holder(use_place_holder)]
    BsplineSurface(Box<BsplineSurfaceAny>),
    #[holder(use_place_holder)]
    SweptSurface(Box<SweptSurfaceAny>),
}

impl TryFrom<&SurfaceAny> for Surface {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(x: &SurfaceAny) -> Result<Self, Self::Error> {
        use SurfaceAny::*;
        Ok(match x {
            ElementarySurface(x) => Self::ElementarySurface(x.as_ref().try_into()?),
            BsplineSurface(x) => x.as_ref().try_into()?,
            SweptSurface(x) => Self::SweepSurface(x.as_ref().try_into()?),
        })
    }
}

/// `elementary_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum ElementarySurfaceAny {
    #[holder(use_place_holder)]
    Plane(Box<Plane>),
    #[holder(use_place_holder)]
    SphericalSurface(Box<SphericalSurface>),
    #[holder(use_place_holder)]
    CylindricalSurface(Box<CylindricalSurface>),
    #[holder(use_place_holder)]
    ToroidalSurface(Box<ToroidalSurface>),
    /// The self-intersecting torus subtype. Present so the record deserializes
    /// into a NAMED class instead of `Table::dummy`; its geometry is refused with
    /// the class named (see [`refuse_degenerate_torus`]).
    #[holder(use_place_holder)]
    DegenerateToroidalSurface(Box<DegenerateToroidalSurface>),
    #[holder(use_place_holder)]
    ConicalSurface(Box<ConicalSurface>),
}

/// Fallible since spec 011 T1: two of the five elementary surfaces carry radii a
/// STEP file can put outside the representable domain, and the torus arm used to
/// hand them to a constructor that PANICS (`Torus::new`, on
/// `major_radius <= 0.0`). 207 corpus faces reached that panic and took 11 solids
/// -- 15,191 faces -- down with them. The refusal now happens here, typed.
impl TryFrom<&ElementarySurfaceAny> for ElementarySurface {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &ElementarySurfaceAny) -> Result<Self, Self::Error> {
        use ElementarySurfaceAny::*;
        Ok(match value {
            Plane(x) => Self::Plane(x.as_ref().into()),
            SphericalSurface(x) => Self::Sphere(x.as_ref().into()),
            CylindricalSurface(x) => Self::CylindricalSurface(x.as_ref().into()),
            ToroidalSurface(x) => Self::ToroidalSurface(x.as_ref().try_into()?),
            DegenerateToroidalSurface(x) => Self::ToroidalSurface(x.as_ref().try_into()?),
            ConicalSurface(x) => Self::ConicalSurface(x.as_ref().into()),
        })
    }
}

/// `plane`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = plane)]
#[holder(generate_deserialize)]
pub struct Plane {
    label: String,
    #[holder(use_place_holder)]
    position: Axis2Placement3d,
}

impl From<&Plane> for geom::Plane {
    #[inline(always)]
    fn from(plane: &Plane) -> Self {
        let mat = Matrix4::from(&plane.position);
        let o = Point3::from_homogeneous(mat[3]);
        let p = o + mat[0].truncate();
        let q = o + mat[1].truncate();
        Self::new(o, p, q)
    }
}

/// `spherical_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = spherical_surface)]
#[holder(generate_deserialize)]
pub struct SphericalSurface {
    label: String,
    #[holder(use_place_holder)]
    position: Axis2Placement3d,
    radius: f64,
}

impl From<&SphericalSurface> for step_geometry::SphericalSurface {
    #[inline(always)]
    fn from(ss: &SphericalSurface) -> Self {
        let mat = Matrix4::from(&ss.position);
        let sphere = Sphere(geom::Sphere::new(Point3::origin(), ss.radius));
        Processor::new(sphere).transformed(mat)
    }
}

/// `cylindrical_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = cylindrical_surface)]
#[holder(generate_deserialize)]
pub struct CylindricalSurface {
    label: String,
    #[holder(use_place_holder)]
    position: Axis2Placement3d,
    radius: f64,
}

impl From<&CylindricalSurface> for step_geometry::CylindricalSurface {
    #[inline(always)]
    fn from(cs: &CylindricalSurface) -> Self {
        let mat = Matrix4::from(&cs.position);
        let x = mat[0].truncate();
        let z = mat[2].truncate();
        let center = Point3::from_homogeneous(mat[3]);
        let radius = cs.radius;
        let p = center + x * radius;
        let mut res = Processor::new(RevolutionSurface::by_revolution(Line(p, p + z), center, z));
        res.invert();
        res
    }
}

/// `toroidal_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = toroidal_surface)]
#[holder(generate_deserialize)]
pub struct ToroidalSurface {
    label: String,
    #[holder(use_place_holder)]
    position: Axis2Placement3d,
    major_radius: f64,
    minor_radius: f64,
}

/// The DEGENERATE (self-intersecting) toroidal regime, refused with the class
/// named -- for BOTH spellings real exporters use for it.
///
/// # The regime, and why it is one regime with two spellings
///
/// A torus whose `minor_radius` exceeds its `major_radius` self-intersects: the
/// tube circle crosses the axis of revolution, so the swept surface passes
/// through itself and closes into an apex (the "spindle" or "lemon" torus).
/// ISO 10303-42 splits `degenerate_toroidal_surface` out of `toroidal_surface`
/// precisely for it (`WHERE major_radius < minor_radius`, plus `select_outer` to
/// pick a sheet). Measured over the 8-file corpus (spec 011 T1), it arrives
/// THREE ways:
///
/// | spelling | records | files |
/// |---|---|---|
/// | `DEGENERATE_TOROIDAL_SURFACE` | 253 | Rocky_House 156, Cruise_Assembly 63, Ai-14R 34 |
/// | `TOROIDAL_SURFACE` with NEGATIVE `major_radius` | 207 | NissanGT-R 201, ROTOR 6 |
/// | `TOROIDAL_SURFACE` with `0 < major_radius < minor_radius` | 136 | NissanGT-R 126, UMC-500 10 |
///
/// A negative major radius is out-of-schema (`positive_length_measure`), and it
/// is the same surface: `(-R, r)` at `(u, v)` equals `(R, r)` at
/// `(u + pi, pi - v)`. That reparameterisation is NOT a placement change, so it
/// cannot be normalised away without invalidating the face's trims (C2).
/// In every one of the 207 measured records `|major| < minor`, i.e. all three
/// spellings are the SAME degenerate regime.
///
/// # Why refuse rather than represent (measured, not assumed)
///
/// The rational-NURBS control net IS exact here -- with the builder's spindle
/// guard bypassed, the emitted net matches the analytic torus to a relative
/// 8e-16..9e-16 over the whole domain, with the control hull a superset of the
/// analytic bbox (the 7y standard, at the witness radii `0.633974596215563/1.0`
/// and at both negative-major samples). Exactness is not what fails.
///
/// What fails is everything that has to invert the parameterisation. On a
/// spindle, `Torus::search_parameter` returns `None` for 168 of 576 on-surface
/// grid points (29%; 144/576 = 25% at the NissanGT-R radii), and
/// `search_nearest_parameter` answers those same points with parameters that
/// evaluate to a DIFFERENT point -- silently, because the implicit form
/// `(sqrt(x^2+y^2) - R)^2 + z^2 = r^2` does not describe the sheet swept through
/// the axis. The ring control scores 576/576 on both. Converting would therefore
/// trade a typed refusal for a face whose trims can be placed wrongly on a
/// quarter of its domain -- worse than no fix, by the correct-or-typed standard.
/// `monstertruck_geometry`'s builder already encodes the same verdict by
/// returning `None` (`bspline_conversion.rs`, `torus_spindle_is_rejected`).
///
/// # What is NOT refused
///
/// HORN tori (`major_radius == minor_radius`, and the fp-near-horn fillets where
/// the two differ by a few ulps) are representable and stay so: the inner
/// equator pinches to a point on the axis, nothing self-intersects, and
/// parameter recovery measures 576/576. The rule below is deliberately the same
/// predicate the geometry builder uses, so the STEP layer refuses exactly what
/// the geometry layer cannot build -- no wider. `Ai-14R.stp` writes an exact
/// horn torus (`3., 3.`) as a `DEGENERATE_TOROIDAL_SURFACE`, so the spelling
/// cannot be trusted to imply the regime; the RADII decide.
fn refuse_degenerate_torus(
    entity: &str,
    major_radius: f64,
    minor_radius: f64,
) -> Result<(), StepConvertingError> {
    if !major_radius.is_finite() || !minor_radius.is_finite() {
        return Err(format!(
            "{entity} refused: non-finite radii (major_radius {major_radius}, \
             minor_radius {minor_radius})."
        )
        .into());
    }
    if minor_radius <= 0.0 {
        return Err(format!(
            "{entity} refused: non-positive minor_radius {minor_radius} \
             (major_radius {major_radius}); ISO 10303-42 declares it a \
             positive_length_measure."
        )
        .into());
    }
    if major_radius <= 0.0 {
        return Err(format!(
            "{entity} refused: non-positive major_radius {major_radius} with minor_radius \
             {minor_radius} -- ISO 10303-42 declares major_radius a positive_length_measure. \
             Negating it names the SAME surface as ({}, {minor_radius}) under a \
             (u + pi, pi - v) reparameterisation that no placement can absorb, and that \
             surface is the degenerate self-intersecting (spindle/lemon) torus monstertruck \
             cannot represent.",
            -major_radius
        )
        .into());
    }
    // Exactly the geometry builder's predicate (`TryIntoHomogeneousBsplineSurface
    // for Torus`): horn and fp-near-horn stay representable, true spindles do not.
    if minor_radius - major_radius > TOLERANCE * (major_radius + minor_radius) {
        return Err(format!(
            "{entity} refused: degenerate self-intersecting torus -- major_radius \
             {major_radius} < minor_radius {minor_radius} (the spindle/lemon regime \
             ISO 10303-42 splits degenerate_toroidal_surface out for). monstertruck has no \
             representation for it: the rational-NURBS builder rejects it, and analytic \
             parameter recovery is silently wrong on ~29% of its domain."
        )
        .into());
    }
    Ok(())
}

impl TryFrom<&ToroidalSurface> for step_geometry::ToroidalSurface {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(
        ToroidalSurface {
            position,
            major_radius,
            minor_radius,
            ..
        }: &ToroidalSurface,
    ) -> Result<Self, Self::Error> {
        refuse_degenerate_torus("TOROIDAL_SURFACE", *major_radius, *minor_radius)?;
        let mat = Matrix4::from(position);
        let torus = Torus::new(Point3::origin(), *major_radius, *minor_radius);
        Ok(Processor::new(torus).transformed(mat))
    }
}

/// `degenerate_toroidal_surface`
///
/// ISO 10303-42 subtype of `toroidal_surface`:
/// `(name, position, major_radius, minor_radius, select_outer)`. The fifth
/// attribute is a BOOLEAN -- which SHEET of the self-intersecting torus the face
/// lies on -- and is nothing like `toroidal_surface`'s attribute list, which
/// simply stops at `minor_radius`. The subtype's WHERE rule is
/// `major_radius < minor_radius`.
///
/// The entity exists in the schema, so it deserializes here rather than falling
/// into `Table::dummy`: a record that lands in `dummy` can only ever be refused
/// as a lookup miss (`Lookup failed for #145`), which names neither the class nor
/// the reason. See [`refuse_degenerate_torus`] for the fate of the geometry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = degenerate_toroidal_surface)]
#[holder(generate_deserialize)]
pub struct DegenerateToroidalSurface {
    label: String,
    #[holder(use_place_holder)]
    position: Axis2Placement3d,
    major_radius: f64,
    minor_radius: f64,
    select_outer: bool,
}

impl TryFrom<&DegenerateToroidalSurface> for step_geometry::ToroidalSurface {
    type Error = StepConvertingError;
    fn try_from(
        DegenerateToroidalSurface {
            position,
            major_radius,
            minor_radius,
            select_outer,
            ..
        }: &DegenerateToroidalSurface,
    ) -> Result<Self, Self::Error> {
        refuse_degenerate_torus("DEGENERATE_TOROIDAL_SURFACE", *major_radius, *minor_radius)?;
        // Past the radii check the surface is a horn torus (or a ring one, if an
        // exporter mis-spells one as degenerate): `select_outer = .T.` is then the
        // whole surface and converts, while `.F.` selects the sheet that has
        // pinched to a single point on the axis -- which is not a surface.
        if !select_outer {
            return Err(format!(
                "DEGENERATE_TOROIDAL_SURFACE refused: select_outer = .F. selects the inner \
                 (apex) sheet, which at major_radius {major_radius} / minor_radius \
                 {minor_radius} has collapsed onto the axis of revolution and is not a \
                 representable surface."
            )
            .into());
        }
        let mat = Matrix4::from(position);
        let torus = Torus::new(Point3::origin(), *major_radius, *minor_radius);
        Ok(Processor::new(torus).transformed(mat))
    }
}

/// `conical_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = conical_surface)]
#[holder(generate_deserialize)]
pub struct ConicalSurface {
    label: String,
    #[holder(use_place_holder)]
    position: Axis2Placement3d,
    radius: f64,
    semi_angle: f64,
}

fn normalize_conical_semi_angle(semi_angle: f64) -> f64 {
    if semi_angle.abs() > PI {
        semi_angle.to_radians()
    } else {
        semi_angle
    }
}

impl From<&ConicalSurface> for step_geometry::ConicalSurface {
    fn from(
        ConicalSurface {
            position,
            radius,
            semi_angle,
            ..
        }: &ConicalSurface,
    ) -> Self {
        let mat = Matrix4::from(position);
        let p = Point3::new(*radius, 0.0, 0.0);
        let semi_angle = normalize_conical_semi_angle(*semi_angle);
        let v = Vector3::new(f64::tan(semi_angle), 0.0, 1.0);
        let rev =
            RevolutionSurface::by_revolution(Line(p, p + v), Point3::origin(), Vector3::unit_z());
        let mut processor = Processor::new(rev);
        processor.transform_by(mat);
        processor.invert();
        processor
    }
}

/// `b_spline_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum BsplineSurfaceAny {
    #[holder(use_place_holder)]
    NonRationalBsplineSurface(NonRationalBsplineSurface),
    #[holder(use_place_holder)]
    RationalBsplineSurface(RationalBsplineSurface),
}

impl TryFrom<&BsplineSurfaceAny> for Surface {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &BsplineSurfaceAny) -> Result<Self, Self::Error> {
        use BsplineSurfaceAny::*;
        Ok(match value {
            NonRationalBsplineSurface(bsp) => Surface::BsplineSurface(bsp.try_into()?),
            RationalBsplineSurface(bsp) => Surface::NurbsSurface(bsp.try_into()?),
        })
    }
}

/// `b_spline_surface_form`
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum BsplineSurfaceForm {
    PlaneSurf,
    CylindricalSurf,
    ConicalSurf,
    SphericalSurf,
    ToroidalSurf,
    SurfOfRevolution,
    RuledSurf,
    GeneralisedCone,
    QuadricSurf,
    SurfOfLinearExtrusion,
    Unspecified,
}

/// `b_spline_surface_with_knots`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = b_spline_surface_with_knots)]
#[holder(generate_deserialize)]
pub struct BsplineSurfaceWithKnots {
    label: String,
    u_degree: i64,
    v_degree: i64,
    #[holder(use_place_holder)]
    control_points_list: Vec<Vec<CartesianPoint>>,
    surface_form: BsplineSurfaceForm,
    u_closed: Logical,
    v_closed: Logical,
    self_intersect: Logical,
    u_multiplicities: Vec<i64>,
    v_multiplicities: Vec<i64>,
    u_knots: Vec<f64>,
    v_knots: Vec<f64>,
    knot_spec: KnotType,
}

impl TryFrom<&BsplineSurfaceWithKnots> for BsplineSurface<Point3> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(surface: &BsplineSurfaceWithKnots) -> Result<Self, StepConvertingError> {
        let uknots = surface.u_knots.to_vec();
        let umulti = surface
            .u_multiplicities
            .iter()
            .map(|n| *n as usize)
            .collect();
        let uknots = KnotVector::from_single_multi(uknots, umulti)?;
        let vknots = surface.v_knots.to_vec();
        let vmulti = surface
            .v_multiplicities
            .iter()
            .map(|n| *n as usize)
            .collect();
        let vknots = KnotVector::from_single_multi(vknots, vmulti)?;
        let ctrls = surface
            .control_points_list
            .iter()
            .map(|vec| vec.iter().map(Point3::from).collect())
            .collect();
        Ok(Self::try_new((uknots, vknots), ctrls)?)
    }
}

/// `uniform_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = uniform_surface)]
#[holder(generate_deserialize)]
pub struct UniformSurface {
    label: String,
    u_degree: i64,
    v_degree: i64,
    #[holder(use_place_holder)]
    control_points_list: Vec<Vec<CartesianPoint>>,
    surface_form: BsplineSurfaceForm,
    u_closed: Logical,
    v_closed: Logical,
    self_intersect: Logical,
}

impl TryFrom<&UniformSurface> for BsplineSurface<Point3> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(surface: &UniformSurface) -> Result<Self, StepConvertingError> {
        let uknots = uniform_knots(surface.control_points_list.len(), surface.u_degree as usize)?;
        let first = surface
            .control_points_list
            .first()
            .ok_or("control points list is empty.")?;
        let vknots = uniform_knots(first.len(), surface.v_degree as usize)?;
        let ctrls = surface
            .control_points_list
            .iter()
            .map(|vec| vec.iter().map(Point3::from).collect())
            .collect();
        Ok(Self::try_new((uknots, vknots), ctrls)?)
    }
}

/// `quasi_uniform_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = quasi_uniform_surface)]
#[holder(generate_deserialize)]
pub struct QuasiUniformSurface {
    label: String,
    u_degree: i64,
    v_degree: i64,
    #[holder(use_place_holder)]
    control_points_list: Vec<Vec<CartesianPoint>>,
    surface_form: BsplineSurfaceForm,
    u_closed: Logical,
    v_closed: Logical,
    self_intersect: Logical,
}

impl TryFrom<&QuasiUniformSurface> for BsplineSurface<Point3> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(surface: &QuasiUniformSurface) -> Result<Self, StepConvertingError> {
        let uknots =
            quasi_uniform_knots(surface.control_points_list.len(), surface.u_degree as usize);
        let first = surface
            .control_points_list
            .first()
            .ok_or("control points list is empty.")?;
        let vknots = quasi_uniform_knots(first.len(), surface.v_degree as usize);
        let ctrls = surface
            .control_points_list
            .iter()
            .map(|vec| vec.iter().map(Point3::from).collect())
            .collect();
        Ok(Self::try_new((uknots, vknots), ctrls)?)
    }
}

/// `bezier_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = bezier_surface)]
#[holder(generate_deserialize)]
pub struct BezierSurface {
    label: String,
    u_degree: i64,
    v_degree: i64,
    #[holder(use_place_holder)]
    control_points_list: Vec<Vec<CartesianPoint>>,
    surface_form: BsplineSurfaceForm,
    u_closed: Logical,
    v_closed: Logical,
    self_intersect: Logical,
}

impl From<&BezierSurface> for BsplineSurface<Point3> {
    #[inline(always)]
    fn from(value: &BezierSurface) -> Self {
        let uknots = KnotVector::bezier_knot(value.u_degree as usize);
        let vknots = KnotVector::bezier_knot(value.v_degree as usize);
        let ctrls = value
            .control_points_list
            .iter()
            .map(|vec| vec.iter().map(Point3::from).collect())
            .collect();
        Self::new((uknots, vknots), ctrls)
    }
}

/// Entity that does not exist in AP042.
/// Surface before rationalization of [`RationalBsplineSurface`] defined by a complex entity
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum NonRationalBsplineSurface {
    #[holder(use_place_holder)]
    BsplineSurfaceWithKnots(Box<BsplineSurfaceWithKnots>),
    #[holder(use_place_holder)]
    UniformSurface(Box<UniformSurface>),
    #[holder(use_place_holder)]
    QuasiUniformSurface(Box<QuasiUniformSurface>),
    #[holder(use_place_holder)]
    BezierSurface(Box<BezierSurface>),
}

impl TryFrom<&NonRationalBsplineSurface> for BsplineSurface<Point3> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &NonRationalBsplineSurface) -> Result<Self, Self::Error> {
        use NonRationalBsplineSurface::*;
        match value {
            BsplineSurfaceWithKnots(x) => x.as_ref().try_into(),
            UniformSurface(x) => x.as_ref().try_into(),
            QuasiUniformSurface(x) => x.as_ref().try_into(),
            BezierSurface(x) => Ok(x.as_ref().into()),
        }
    }
}

/// `rational_b_spline_surface` as complex entity
///
/// This struct is an ad hoc implementation that differs from the definition by EXPRESS:
/// in AP042, rationalized curves are defined as complex entities,
/// but here the surfaces before rationalization are held as internal variables.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = rational_b_spline_surface)]
#[holder(generate_deserialize)]
pub struct RationalBsplineSurface {
    #[holder(use_place_holder)]
    non_rational_b_spline_surface: NonRationalBsplineSurface,
    weights_data: Vec<Vec<f64>>,
}

impl TryFrom<&RationalBsplineSurface> for NurbsSurface<Vector4> {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(
        RationalBsplineSurface {
            non_rational_b_spline_surface,
            weights_data,
        }: &RationalBsplineSurface,
    ) -> Result<Self, Self::Error> {
        let surface: BsplineSurface<Point3> = non_rational_b_spline_surface.try_into()?;
        Ok(Self::try_from_bspline_and_weights(
            surface,
            weights_data.clone(),
        )?)
    }
}

/// `swept_surface`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum SweptSurfaceAny {
    #[holder(use_place_holder)]
    SurfaceOfLinearExtrusion(Box<SurfaceOfLinearExtrusion>),
    #[holder(use_place_holder)]
    SurfaceOfRevolution(Box<SurfaceOfRevolution>),
}

impl TryFrom<&SweptSurfaceAny> for SweepSurface {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(value: &SweptSurfaceAny) -> Result<Self, Self::Error> {
        use SweptSurfaceAny::*;
        Ok(match value {
            SurfaceOfLinearExtrusion(x) => SweepSurface::ExtrusionSurface(x.as_ref().try_into()?),
            SurfaceOfRevolution(x) => SweepSurface::RevolutionSurface(x.as_ref().try_into()?),
        })
    }
}

/// `surface_of_linear_extrusion`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = surface_of_linear_extrusion)]
#[holder(generate_deserialize)]
pub struct SurfaceOfLinearExtrusion {
    label: String,
    #[holder(use_place_holder)]
    swept_curve: CurveAny,
    #[holder(use_place_holder)]
    extrusion_axis: Vector,
}

impl TryFrom<&SurfaceOfLinearExtrusion> for StepExtrusionSurface {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(sr: &SurfaceOfLinearExtrusion) -> Result<Self, Self::Error> {
        let curve = Curve3D::try_from(&sr.swept_curve)?;
        let vector = Vector3::from(&sr.extrusion_axis);
        Ok(ExtrusionSurface::by_extrusion(curve, vector))
    }
}

/// `surface_of_revolution`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = surface_of_revolution)]
#[holder(generate_deserialize)]
pub struct SurfaceOfRevolution {
    label: String,
    #[holder(use_place_holder)]
    swept_curve: CurveAny,
    #[holder(use_place_holder)]
    axis_position: Axis1Placement,
}

impl TryFrom<&SurfaceOfRevolution> for StepRevolutionSurface {
    type Error = StepConvertingError;
    #[inline(always)]
    fn try_from(sr: &SurfaceOfRevolution) -> Result<Self, Self::Error> {
        let curve = Curve3D::try_from(&sr.swept_curve)?;
        let origin = Point3::from(&sr.axis_position.location);
        let axis = sr.axis_position.direction().normalize();
        let mut rev = Processor::new(RevolutionSurface::by_revolution(curve, origin, axis));
        rev.invert();
        Ok(rev)
    }
}

/// `vertex_point`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = vertex_point)]
#[holder(generate_deserialize)]
pub struct VertexPoint {
    pub label: String,
    #[holder(use_place_holder)]
    pub vertex_geometry: CartesianPoint,
}

/// `edge`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum EdgeAny {
    #[holder(use_place_holder)]
    EdgeCurve(EdgeCurve),
    #[holder(use_place_holder)]
    OrientedEdge(OrientedEdge),
}

/// `edge_curve`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = edge_curve)]
#[holder(generate_deserialize)]
pub struct EdgeCurve {
    pub label: String,
    #[holder(use_place_holder)]
    pub edge_start: VertexPoint,
    #[holder(use_place_holder)]
    pub edge_end: VertexPoint,
    #[holder(use_place_holder)]
    pub edge_geometry: CurveAny,
    pub same_sense: bool,
}

impl EdgeCurve {
    pub fn parse_curve2d(&self) -> Result<Curve2D, StepConvertingError> {
        let p = Point2::from(&self.edge_start.vertex_geometry);
        let q = Point2::from(&self.edge_end.vertex_geometry);
        let (p, q) = match self.same_sense {
            true => (p, q),
            false => (q, p),
        };
        Self::sub_parse_2d(&self.edge_geometry, p, q, self.same_sense)
    }
    fn sub_parse_2d(
        curve: &CurveAny,
        p: Point2,
        q: Point2,
        same_sense: bool,
    ) -> Result<Curve2D, StepConvertingError> {
        let mut curve = match curve {
            CurveAny::Line(line) => {
                let line = geom::Line::<Point2>::from(line.as_ref());
                let p = line.projection(p);
                let q = line.projection(q);
                Curve2D::Line(Line(p, q))
            }
            CurveAny::BoundedCurve(b) => b.as_ref().try_into()?,
            CurveAny::Conic(curve) => match curve.as_ref() {
                Conic::Circle(circle) => {
                    let mat =
                        Matrix3::try_from(&circle.position)? * Matrix3::from_scale(circle.radius);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitCircle::<Point2>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                        UnitCircle::<Point2>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let circle = TrimmedCurve::new(UnitCircle::<Point2>::new(), (u, v));
                    let mut ellipse = Processor::new(circle);
                    ellipse.transform_by(mat);
                    Curve2D::Conic(Conic2D::Ellipse(ellipse))
                }
                Conic::Ellipse(ellipse) => {
                    let mat = Matrix3::try_from(&ellipse.position)?
                        * Matrix3::from_nonuniform_scale(ellipse.semi_axis_1, ellipse.semi_axis_2);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitCircle::<Point2>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                        UnitCircle::<Point2>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let circle = TrimmedCurve::new(UnitCircle::<Point2>::new(), (u, v));
                    let mut ellipse = Processor::new(circle);
                    ellipse.transform_by(mat);
                    Curve2D::Conic(Conic2D::Ellipse(ellipse))
                }
                Conic::Hyperbola(hyperbola) => {
                    let mat = Matrix3::try_from(&hyperbola.position)?
                        * Matrix3::from_nonuniform_scale(
                            hyperbola.semi_axis,
                            hyperbola.semi_imag_axis,
                        );
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Hyperbola".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, v) = (
                        UnitHyperbola::<Point2>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on hyperbola".to_string())?,
                        UnitHyperbola::<Point2>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on hyparbola".to_string())?,
                    );
                    let unit = TrimmedCurve::new(UnitHyperbola::<Point2>::new(), (u, v));
                    let mut hyperbola = Processor::new(unit);
                    hyperbola.transform_by(mat);
                    Curve2D::Conic(Conic2D::Hyperbola(hyperbola))
                }
                Conic::Parabola(parabola) => {
                    let mat = Matrix3::try_from(&parabola.position)?
                        * Matrix3::from_scale(parabola.focal_dist);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Parabola".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, v) = (
                        UnitHyperbola::<Point2>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on parabola".to_string())?,
                        UnitHyperbola::<Point2>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on parabola".to_string())?,
                    );
                    let unit = TrimmedCurve::new(UnitHyperbola::<Point2>::new(), (u, v));
                    let mut parabola = Processor::new(unit);
                    parabola.transform_by(mat);
                    Curve2D::Conic(Conic2D::Hyperbola(parabola))
                }
            },
            CurveAny::Pcurve(_) => return Err("Pcurves cannot be parsed to 2D curves.".into()),
            CurveAny::SurfaceCurve(_) => {
                return Err("Surface curves cannot be parsed to 2D curves.".into());
            }
        };
        if !same_sense {
            curve.invert();
        }
        Ok(curve)
    }
    pub fn parse_curve3d(&self) -> Result<Curve3D, StepConvertingError> {
        let p = Point3::from(&self.edge_start.vertex_geometry);
        let q = Point3::from(&self.edge_end.vertex_geometry);
        let (p, q) = match self.same_sense {
            true => (p, q),
            false => (q, p),
        };
        Self::sub_parse_curve3d(&self.edge_geometry, p, q, self.same_sense)
    }
    fn sub_parse_curve3d(
        curve: &CurveAny,
        p: Point3,
        q: Point3,
        same_sense: bool,
    ) -> Result<Curve3D, StepConvertingError> {
        let mut curve = match curve {
            CurveAny::Line(_) => Curve3D::Line(Line(p, q)),
            CurveAny::BoundedCurve(b) => b.as_ref().try_into()?,
            CurveAny::Conic(curve) => match curve.as_ref() {
                Conic::Circle(circle) => {
                    let mat =
                        Matrix4::try_from(&circle.position)? * Matrix4::from_scale(circle.radius);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitCircle::<Point3>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| format!("the point is not on circle: {p:?}"))?,
                        UnitCircle::<Point3>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| format!("the point is not on circle: {q:?}"))?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let circle = TrimmedCurve::new(UnitCircle::<Point3>::new(), (u, v));
                    let mut ellipse = Processor::new(circle);
                    ellipse.transform_by(mat);
                    Curve3D::Conic(Conic3D::Ellipse(ellipse))
                }
                Conic::Ellipse(ellipse) => {
                    let mat = Matrix4::try_from(&ellipse.position)?
                        * Matrix4::from_nonuniform_scale(
                            ellipse.semi_axis_1,
                            ellipse.semi_axis_2,
                            f64::min(ellipse.semi_axis_1, ellipse.semi_axis_2),
                        );
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitCircle::<Point3>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| format!("the point is not on circle: {p:?}"))?,
                        UnitCircle::<Point3>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| format!("the point is not on circle: {q:?}"))?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let circle = TrimmedCurve::new(UnitCircle::<Point3>::new(), (u, v));
                    let mut ellipse = Processor::new(circle);
                    ellipse.transform_by(mat);
                    Curve3D::Conic(Conic3D::Ellipse(ellipse))
                }
                Conic::Hyperbola(hyperbola) => {
                    let mat = Matrix4::try_from(&hyperbola.position)?
                        * Matrix4::from_nonuniform_scale(
                            hyperbola.semi_axis,
                            hyperbola.semi_imag_axis,
                            f64::min(hyperbola.semi_axis, hyperbola.semi_imag_axis),
                        );
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitHyperbola::<Point3>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                        UnitHyperbola::<Point3>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let unit = TrimmedCurve::new(UnitHyperbola::<Point3>::new(), (u, v));
                    let mut hyperbola = Processor::new(unit);
                    hyperbola.transform_by(mat);
                    Curve3D::Conic(Conic3D::Hyperbola(hyperbola))
                }
                Conic::Parabola(parabola) => {
                    let mat = Matrix4::try_from(&parabola.position)?
                        * Matrix4::from_scale(parabola.focal_dist);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Parabola".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, v) = (
                        UnitHyperbola::<Point3>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on parabola".to_string())?,
                        UnitHyperbola::<Point3>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on parabola".to_string())?,
                    );
                    let unit = TrimmedCurve::new(UnitHyperbola::<Point3>::new(), (u, v));
                    let mut parabola = Processor::new(unit);
                    parabola.transform_by(mat);
                    Curve3D::Conic(Conic3D::Hyperbola(parabola))
                }
            },
            CurveAny::Pcurve(c) => {
                let surface: Surface = (&c.basis_surface).try_into()?;
                let u = surface
                    .search_nearest_parameter(p, None, 100)
                    .ok_or_else(|| "the point is not on surface".to_string())?;
                let v = surface
                    .search_nearest_parameter(q, None, 100)
                    .ok_or_else(|| "the point is not on surface".to_string())?;
                let curve2d = c
                    .reference_to_curve
                    .representation_item
                    .first()
                    .ok_or("no representation item")?;
                let curve2d = Self::sub_parse_2d(
                    curve2d,
                    Point2::new(u.0, u.1),
                    Point2::new(v.0, v.1),
                    true,
                )?;
                Curve3D::ParameterCurve(geom::ParameterCurve::new(
                    Box::new(curve2d),
                    Box::new(surface),
                ))
            }
            CurveAny::SurfaceCurve(c) => {
                if p.near(&q) {
                    return Self::sub_parse_curve3d(&c.curve_3d, p, q, same_sense);
                }
                let associated_surface = |entry: &PcurveOrSurface| -> Result<
                    step_geometry::Surface,
                    StepConvertingError,
                > {
                    match entry {
                        PcurveOrSurface::Pcurve(x) => (&x.basis_surface).try_into(),
                        PcurveOrSurface::Surface(x) => x.as_ref().try_into(),
                    }
                };
                let associated_geometry = c
                    .associated_geometry
                    .iter()
                    .map(|entry| match entry {
                        PcurveOrSurface::Pcurve(curve) => curve
                            .as_ref()
                            .try_into()
                            .map(step_geometry::SurfaceCurveAssociatedGeometry::ParameterCurve),
                        PcurveOrSurface::Surface(surface) => surface
                            .as_ref()
                            .try_into()
                            .map(Box::new)
                            .map(step_geometry::SurfaceCurveAssociatedGeometry::Surface),
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                let kind = match c.kind {
                    SurfaceCurveEntityKind::Surface => {
                        step_geometry::SurfaceCurveKind::SurfaceCurve
                    }
                    SurfaceCurveEntityKind::Seam => step_geometry::SurfaceCurveKind::SeamCurve,
                    SurfaceCurveEntityKind::Intersection => {
                        step_geometry::SurfaceCurveKind::IntersectionCurve
                    }
                };
                let master_representation = match c.master_representation {
                    PreferredSurfaceCurveRepresentation::Curve3D => {
                        step_geometry::SurfaceCurveRepresentation::Curve3D
                    }
                    PreferredSurfaceCurveRepresentation::PcurveS1 => {
                        step_geometry::SurfaceCurveRepresentation::ParameterCurve0
                    }
                    PreferredSurfaceCurveRepresentation::PcurveS2 => {
                        step_geometry::SurfaceCurveRepresentation::ParameterCurve1
                    }
                };
                let leader = Self::sub_parse_curve3d(&c.curve_3d, p, q, true)?;
                if associated_geometry.len() >= 2
                    && associated_geometry.iter().all(|entry| {
                        matches!(
                            entry,
                            step_geometry::SurfaceCurveAssociatedGeometry::Surface(_)
                        )
                    })
                {
                    let surface0 = c
                        .associated_geometry
                        .first()
                        .ok_or("The 0-indexed associated geometry is missing.")?;
                    let surface1 = c
                        .associated_geometry
                        .get(1)
                        .ok_or("The 1-indexed associated geometry is missing.")?;
                    Curve3D::IntersectionCurve(geom::IntersectionCurve::new(
                        Box::new(associated_surface(surface0)?),
                        Box::new(associated_surface(surface1)?),
                        Box::new(leader),
                    ))
                } else {
                    Curve3D::SurfaceCurve(step_geometry::SurfaceCurve3D::new(
                        kind,
                        Box::new(leader),
                        associated_geometry,
                        master_representation,
                    ))
                }
            }
        };
        if !same_sense {
            curve.invert();
        }
        Ok(curve)
    }
}

/// `oriented_edge`
///
/// `oriented_edge` has duplicated information.
/// These are not included here because they are essentially omitted.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = oriented_edge)]
#[holder(generate_deserialize)]
pub struct OrientedEdge {
    pub label: String,
    #[holder(use_place_holder)]
    pub edge_element: EdgeCurve,
    pub orientation: bool,
}

impl OrientedEdgeHolder {
    pub(crate) fn edge_element_holder(&self, table: &Table) -> Option<EdgeCurveHolder> {
        match &self.edge_element {
            PlaceHolder::Owned(holder) => Some(holder.clone()),
            PlaceHolder::Ref(Name::Entity(idx)) => table.edge_curve.get(idx).cloned(),
            _ => None,
        }
    }
    pub(crate) fn edge_element_idx(&self) -> Option<u64> {
        if let PlaceHolder::Ref(Name::Entity(idx)) = self.edge_element {
            Some(idx)
        } else {
            None
        }
    }
}

/// `edge_loop`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = edge_loop)]
#[holder(generate_deserialize)]
pub struct EdgeLoop {
    pub label: String,
    #[holder(use_place_holder)]
    pub edge_list: Vec<EdgeAny>,
}

/// `vertex_loop`
///
/// A `loop` whose entire extent is ONE vertex -- the degenerate boundary
/// exporters emit at a parameterisation singularity: a cone apex, a sphere pole,
/// or a stand-in for the (boundary-less) closed torus.
///
/// It has an arm here purely so the record is ATTRIBUTABLE. Before spec 011 T7
/// it fell into [`Table::dummy`](crate::load::Table::dummy), so
/// `FaceBoundHolder::bound_holder` looked the id up in `Table::edge_loop`, got
/// `None`, and the wire was discarded by a `filter_map` **with no error and no
/// message anywhere** -- the only truly silent drop the spec 011 Phase 0 census
/// found. It bit four in-repo fixtures, `boxy-with-surfacetex.step` losing 10 of
/// its 160 wires.
///
/// It is still not REPRESENTED in a `CompressedShell`: a compressed wire is a
/// sequence of edge uses and a point boundary has none, so the loop is reported
/// as [`LossReason::DegenerateVertexLoop`](crate::load::report::LossReason::DegenerateVertexLoop)
/// and dropped. Read that variant's docs for the measurement behind the choice.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = vertex_loop)]
#[holder(generate_deserialize)]
pub struct VertexLoop {
    pub label: String,
    #[holder(use_place_holder)]
    pub loop_vertex: VertexPoint,
}

/// `face_bound`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = face_bound)]
#[holder(generate_deserialize)]
/// `FACE_OUTER_BOUNDS` is also parsed to this struct.
pub struct FaceBound {
    pub label: String,
    // For now, we are going with the policy of accepting nothing but edgeloop.
    #[holder(use_place_holder)]
    pub bound: EdgeLoop,
    pub orientation: bool,
}

/// What a `FACE_BOUND`'s `bound` reference turns out to point at.
///
/// The declared type of `FaceBound::bound` is `EdgeLoop`, but STEP's `loop` has
/// three subtypes and the reference is untyped until it is resolved. Naming the
/// three outcomes is what lets the loader distinguish a LEGITIMATE degeneracy
/// (a vertex loop) from a genuine loss (a bound that is simply not there).
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BoundResolution {
    /// An `EDGE_LOOP`, inline or resolved from the table.
    EdgeLoop(EdgeLoopHolder),
    /// A `VERTEX_LOOP`, with its entity id.
    VertexLoop(u64),
    /// Neither: no record with that id, or a `loop` subtype with no arm. Carries
    /// the id when the reference had one.
    Unresolved(Option<u64>),
}

impl FaceBoundHolder {
    pub(crate) fn resolve_bound(&self, table: &Table) -> BoundResolution {
        match &self.bound {
            PlaceHolder::Owned(holder) => BoundResolution::EdgeLoop(holder.clone()),
            PlaceHolder::Ref(Name::Entity(idx)) => table
                .edge_loop
                .get(idx)
                .cloned()
                .map(BoundResolution::EdgeLoop)
                .unwrap_or_else(|| {
                    if table.vertex_loop.contains_key(idx) {
                        BoundResolution::VertexLoop(*idx)
                    } else {
                        BoundResolution::Unresolved(Some(*idx))
                    }
                }),
            _ => BoundResolution::Unresolved(None),
        }
    }

    pub(crate) fn bound_holder(&self, table: &Table) -> Option<EdgeLoopHolder> {
        match self.resolve_bound(table) {
            BoundResolution::EdgeLoop(holder) => Some(holder),
            _ => None,
        }
    }
}

/// `face`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum FaceAny {
    #[holder(use_place_holder)]
    FaceSurface(FaceSurface),
    #[holder(use_place_holder)]
    OrientedFace(OrientedFace),
}

/// `face_surface`
///
/// `advanced_face` is also parsed to this struct.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = face_surface)]
#[holder(generate_deserialize)]
pub struct FaceSurface {
    pub label: String,
    #[holder(use_place_holder)]
    pub bounds: Vec<FaceBound>,
    #[holder(use_place_holder)]
    pub face_geometry: SurfaceAny,
    pub same_sense: bool,
}

impl FaceSurfaceHolder {
    /// Every bound the face LISTS, paired with the entity id the face referenced.
    ///
    /// The id is carried alongside so a bound that fails to resolve can still be
    /// reported by name -- without it, a lost wire is anonymous and a human has
    /// nothing to go and look at.
    pub(crate) fn bounds_holder<'a>(
        &'a self,
        table: &'a Table,
    ) -> Vec<(Option<u64>, Option<FaceBoundHolder>)> {
        self.bounds
            .iter()
            .map(|bound| match bound {
                PlaceHolder::Owned(bound) => (None, Some(bound.clone())),
                PlaceHolder::Ref(Name::Entity(idx)) => {
                    (Some(*idx), table.face_bound.get(idx).cloned())
                }
                _ => (None, None),
            })
            .collect()
    }
}

/// `oriented_face`
///
/// `oriented_face` has duplicated information.
/// These are not included here because they are essentially omitted.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = oriented_face)]
#[holder(generate_deserialize)]
pub struct OrientedFace {
    pub label: String,
    #[holder(use_place_holder)]
    pub face_element: FaceSurface,
    pub orientation: bool,
}

impl OrientedFaceHolder {
    pub(crate) fn face_element_holder(&self, table: &Table) -> Option<FaceSurfaceHolder> {
        match &self.face_element {
            PlaceHolder::Ref(Name::Entity(idx)) => table.face_surface.get(idx).cloned(),
            PlaceHolder::Owned(x) => Some(x.clone()),
            _ => None,
        }
    }
}

/// `shell`
///
/// Includes `open_shell` and `closed_shell`.
/// Since these differences are only informal propositions, the data structure does not distinguish between the two.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shell)]
#[holder(generate_deserialize)]
pub struct Shell {
    pub label: String,
    #[holder(use_place_holder)]
    pub cfs_faces: Vec<FaceAny>,
}

impl ShellHolder {
    pub(crate) fn cfs_faces_holder<'a>(
        &'a self,
        table: &'a Table,
    ) -> impl Iterator<Item = Option<FaceAnyHolder>> + 'a {
        self.cfs_faces.iter().map(|face| match face {
            PlaceHolder::Owned(holder) => Some(holder.clone()),
            PlaceHolder::Ref(Name::Entity(idx)) => table
                .oriented_face
                .get(idx)
                .cloned()
                .map(FaceAnyHolder::OrientedFace)
                .or_else(|| {
                    table
                        .face_surface
                        .get(idx)
                        .cloned()
                        .map(FaceAnyHolder::FaceSurface)
                }),
            _ => None,
        })
    }
}

/// `oriented_shell`
///
/// Includes `oriented_open_shell` and `oriented_closed_shell`.
/// Since these differences are only informal propositions, the data structure does not distinguish between the two.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = oriented_shell)]
#[holder(generate_deserialize)]
pub struct OrientedShell {
    pub label: String,
    #[holder(use_place_holder)]
    pub shell_element: Shell,
    pub orientation: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum ShellAny {
    #[holder(use_place_holder)]
    Shell(Shell),
    #[holder(use_place_holder)]
    OrientedShell(OrientedShell),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shell_based_surface_model)]
#[holder(generate_deserialize)]
pub struct ShellBasedSurfaceModel {
    pub label: String,
    #[holder(use_place_holder)]
    pub sbsm_boundary: Vec<ShellAny>,
}

/// Also serves as `brep_with_voids`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = manifold_solid_brep)]
#[holder(generate_deserialize)]
pub struct ManifoldSolidBrep {
    pub label: String,
    #[holder(use_place_holder)]
    pub outer: ShellAny,
    #[holder(use_place_holder)]
    pub voids: Vec<OrientedShell>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = application_context)]
#[holder(generate_deserialize)]
pub struct ApplicationContext {
    pub application: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_context)]
#[holder(generate_deserialize)]
pub struct ProductContext {
    pub name: String,
    #[holder(use_place_holder)]
    pub frame_of_reference: ApplicationContext,
    pub discipline_type: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product)]
#[holder(generate_deserialize)]
pub struct Product {
    pub id: String,
    pub name: String,
    /// `OPTIONAL text` in ISO 10303-41 `product`, so `$` is CONFORMANT here.
    /// Measured: 115 of 225 `PRODUCT` records in `Scania-8x4.stp` and 27 of 180
    /// in `Scania-Engine-V8-XT-Turbo.step` write `$`; while this was `String`
    /// every one of them was refused and dropped, taking the product out of
    /// `Table::product` and the part out of the assembly graph.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub frame_of_reference: Vec<ProductContext>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_definition_formation)]
#[holder(generate_deserialize)]
pub struct ProductDefinitionFormation {
    pub id: String,
    /// `OPTIONAL text` in ISO 10303-41 `product_definition_formation`, so `$` is
    /// CONFORMANT. Measured: **100%** of the 225 + 180 records in the two Scania
    /// files write `$` here, which is why `Table::product_definition_formation`
    /// was completely empty for both and `Table::step_assy` had nothing to walk.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub of_product: Product,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_definition_context)]
#[holder(generate_deserialize)]
pub struct ProductDefinitionContext {
    pub name: String,
    #[holder(use_place_holder)]
    pub frame_of_reference: ApplicationContext,
    pub life_cycle_stage: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_definition)]
#[holder(generate_deserialize)]
pub struct ProductDefinition {
    pub id: String,
    pub description: String,
    #[holder(use_place_holder)]
    pub formation: ProductDefinitionFormation,
    #[holder(use_place_holder)]
    pub frame_of_reference: ProductDefinitionContext,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum CharacterizedDefinition {
    #[holder(use_place_holder)]
    ProductDefinition(Box<ProductDefinition>),
    #[holder(use_place_holder)]
    ProductDefinitionShape(Box<ProductDefinitionShape>),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_definition_shape)]
#[holder(generate_deserialize)]
pub struct ProductDefinitionShape {
    /// `label`, and NOT optional in ISO 10303-41 `property_definition` -- so a
    /// `$` here is the EXPORTER being non-conformant, not the schema allowing it.
    /// Accepted anyway, and the reason is measured: 470 of 695 records in
    /// `Scania-8x4.stp` and 726 of 906 in `Scania-Engine-V8-XT-Turbo.step` are
    /// spelled `PRODUCT_DEFINITION_SHAPE($,$,#..)`. Refusing them is refusing the
    /// file's entire assembly over two display strings, which is the same trade
    /// [`Table::from_step_bytes`](crate::load::Table::from_step_bytes) already
    /// declines to make for its encoding fallback. `Option` rather than an empty
    /// `String` so the distinction between "unset" and "set to empty" survives --
    /// 225 records in the same file really do write `''`.
    pub name: Option<String>,
    /// `OPTIONAL text` in ISO 10303-41 `property_definition`: `$` is CONFORMANT.
    /// Measured at 100% of both files' records.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub definition: CharacterizedDefinition,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shape_representation)]
#[holder(generate_deserialize)]
pub struct ShapeRepresentation {
    pub name: String,
    #[holder(use_place_holder)]
    pub items: Vec<RepresentationItem>,
    #[holder(use_place_holder)]
    pub context_of_items: RepresentationContext,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = context_dependent_shape_representation)]
#[holder(generate_deserialize)]
pub struct ContextDependentShapeRepresentation {
    #[holder(use_place_holder)]
    pub representation_relation: ShapeRepresentationRelationshipWithTransformation,
    #[holder(use_place_holder)]
    pub represented_product_relation: ProductDefinitionShape,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shape_definition_representation)]
#[holder(generate_deserialize)]
pub struct ShapeDefinitionRepresentation {
    #[holder(use_place_holder)]
    pub definition: ProductDefinitionShape,
    #[holder(use_place_holder)]
    pub used_representation: ShapeRepresentation,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shape_representation_relationship)]
#[holder(generate_deserialize)]
pub struct ShapeRepresentationRelationship {
    /// `label`, mandatory in ISO 10303-43 `representation_relationship`; accepted
    /// as unset for the same measured reason as
    /// [`ProductDefinitionShape::name`].
    pub name: Option<String>,
    /// `OPTIONAL text` in ISO 10303-43 `representation_relationship`: `$` is
    /// CONFORMANT.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub rep_1: ShapeRepresentation,
    #[holder(use_place_holder)]
    pub rep_2: ShapeRepresentation,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shape_representation_relationship_with_transformation)]
#[holder(generate_deserialize)]
pub struct ShapeRepresentationRelationshipWithTransformation {
    /// `label`, mandatory in ISO 10303-43; accepted as unset. Measured: **100%**
    /// of the 470 + 726 `REPRESENTATION_RELATIONSHIP` sub-records of the complex
    /// `SHAPE_REPRESENTATION_RELATIONSHIP` instances in the two Scania files are
    /// spelled `REPRESENTATION_RELATIONSHIP($,$,#..,#..)`. These are the records
    /// that ATTACH the placement matrices to the parts, so all 1,086 solids lost
    /// their position to this one refusal.
    pub name: Option<String>,
    /// `OPTIONAL text` in ISO 10303-43: `$` is CONFORMANT.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub rep_1: ShapeRepresentation,
    #[holder(use_place_holder)]
    pub rep_2: ShapeRepresentation,
    #[holder(use_place_holder)]
    pub transformation_operator: ItemDefinedTransformation,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = next_assembly_usage_occurrence)]
#[holder(generate_deserialize)]
pub struct NextAssemblyUsageOccurrence {
    pub id: String,
    pub name: String,
    pub description: String,
    #[holder(use_place_holder)]
    pub relating_product_definition: ProductDefinition,
    #[holder(use_place_holder)]
    pub related_product_definition: ProductDefinition,
    pub reference_designator: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = item_defined_transformation)]
#[holder(generate_deserialize)]
pub struct ItemDefinedTransformation {
    /// `label`, mandatory in ISO 10303-43 `item_defined_transformation`; accepted
    /// as unset. Measured: **100%** of the 470 + 726 records in the two Scania
    /// files are spelled `ITEM_DEFINED_TRANSFORMATION($,$,#..,#..)`. This entity
    /// carries the assembly's placement matrices.
    name: Option<String>,
    /// `OPTIONAL text` in ISO 10303-43: `$` is CONFORMANT.
    description: Option<String>,
    #[holder(use_place_holder)]
    transform_item_1: Axis2Placement,
    #[holder(use_place_holder)]
    transform_item_2: Axis2Placement,
}

impl TryFrom<&ItemDefinedTransformation> for Matrix3 {
    type Error = StepConvertingError;
    fn try_from(value: &ItemDefinedTransformation) -> Result<Self, Self::Error> {
        let mat1: Self = (&value.transform_item_1).try_into()?;
        let mat2: Self = (&value.transform_item_2).try_into()?;
        let inv = mat1
            .invert()
            .ok_or("failed to invert transform_item_1 Matrix3")?;
        Ok(mat2 * inv)
    }
}

impl TryFrom<&ItemDefinedTransformation> for Matrix4 {
    type Error = StepConvertingError;
    fn try_from(value: &ItemDefinedTransformation) -> Result<Self, Self::Error> {
        let mat1: Self = (&value.transform_item_1).try_into()?;
        let mat2: Self = (&value.transform_item_2).try_into()?;
        let inv = mat1
            .invert()
            .ok_or("failed to invert transform_item_1 Matrix4")?;
        Ok(mat2 * inv)
    }
}

// Deprecated aliases for types renamed per RFC 430 (C-CASE).
#[deprecated(note = "renamed to BsplineCurveForm per RFC 430 (C-CASE)")]
pub type BSplineCurveForm = BsplineCurveForm;
#[deprecated(note = "renamed to BsplineCurveWithKnots per RFC 430 (C-CASE)")]
pub type BSplineCurveWithKnots = BsplineCurveWithKnots;
#[deprecated(note = "renamed to NonRationalBsplineCurve per RFC 430 (C-CASE)")]
pub type NonRationalBSplineCurve = NonRationalBsplineCurve;
#[deprecated(note = "renamed to RationalBsplineCurve per RFC 430 (C-CASE)")]
pub type RationalBSplineCurve = RationalBsplineCurve;
#[deprecated(note = "renamed to BsplineCurveAny per RFC 430 (C-CASE)")]
pub type BSplineCurveAny = BsplineCurveAny;
#[deprecated(note = "renamed to BsplineSurfaceAny per RFC 430 (C-CASE)")]
pub type BSplineSurfaceAny = BsplineSurfaceAny;
#[deprecated(note = "renamed to BsplineSurfaceForm per RFC 430 (C-CASE)")]
pub type BSplineSurfaceForm = BsplineSurfaceForm;
#[deprecated(note = "renamed to BsplineSurfaceWithKnots per RFC 430 (C-CASE)")]
pub type BSplineSurfaceWithKnots = BsplineSurfaceWithKnots;
#[deprecated(note = "renamed to NonRationalBsplineSurface per RFC 430 (C-CASE)")]
pub type NonRationalBSplineSurface = NonRationalBsplineSurface;
#[deprecated(note = "renamed to RationalBsplineSurface per RFC 430 (C-CASE)")]
pub type RationalBSplineSurface = RationalBsplineSurface;

#[cfg(test)]
mod tests {
    use super::*;

    fn direction(ratios: [f64; 3]) -> Direction {
        Direction {
            label: String::new(),
            direction_ratios: ratios.to_vec(),
        }
    }

    /// Regression for `axis ∥ ref_direction`: ISO 10303 allows the placement's
    /// `axis` and `ref_direction` to be parallel, but the Gram-Schmidt step
    /// then normalized a zero vector and produced `NaN`, which later panicked
    /// during meshing (`tolerance must be no less than 1e-6`). The conversion
    /// must yield a finite, orthonormal basis instead.
    #[test]
    fn axis2_placement3d_parallel_axis_and_ref_direction_is_finite() {
        let placement = Axis2Placement3d {
            label: String::new(),
            location: CartesianPoint {
                label: String::new(),
                coordinates: vec![1.0, 2.0, 3.0],
            },
            axis: Some(direction([0.0, 0.0, 1.0])),
            // Parallel to `axis` -- the degenerate case.
            ref_direction: Some(direction([0.0, 0.0, 1.0])),
        };

        let matrix = Matrix4::from(&placement);
        let (x, y, z) = (
            matrix.x.truncate(),
            matrix.y.truncate(),
            matrix.z.truncate(),
        );
        for component in [x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z] {
            assert!(component.is_finite(), "placement basis must be finite");
        }
        // The recovered basis is orthonormal.
        assert!((x.magnitude() - 1.0).abs() < 1.0e-9);
        assert!((y.magnitude() - 1.0).abs() < 1.0e-9);
        assert!(x.dot(z).abs() < 1.0e-9);
        assert!(y.dot(z).abs() < 1.0e-9);
    }
}
