//! Curve entities: lines, conics, polylines, B-spline and NURBS curves, and
//! the standalone curve sets built from them.
//!
//! The `pcurve`/`surface_curve` cluster is the exception: it lives in the
//! parent module because [`topology`](super::topology) reads its fields.

use super::*;

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
