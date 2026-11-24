use crate::{prelude::*, *};
use std::ops::{Deref, DerefMut, Mul};

/// revolution
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
struct Revolution {
    origin: Point3,
    axis: Vector3,
}

/// Surface constructed by revolving a curve around an axis.
/// # Examples
/// Revolved sphere
/// ```
/// use monstertruck_geometry::prelude::*;
/// use std::f64::consts::PI;
/// let knot_vec = KnotVector::bezier_knot(2);
/// let control_points = vec![
///     Vector4::new(1.0, 0.0, 0.0, 1.0),
///     Vector4::new(0.0, 1.0, 0.0, 0.0),
///     Vector4::new(-1.0, 0.0, 0.0, 1.0),
/// ];
/// // upper half circle on xy-plane
/// let uhcircle = NurbsCurve::new(BsplineCurve::new(knot_vec, control_points));
/// // sphere constructed by revolving circle
/// let sphere = RevolutionSurface::by_revolution(
///     uhcircle, Point3::origin(), Vector3::unit_x(),
/// );
/// const N: usize = 30;
/// for i in 0..=N {
///     for j in 0..=N {
///         let u = i as f64 / N as f64;
///         let v = 2.0 * PI * j as f64 / N as f64;
///         let pt: Vector3 = sphere.evaluate(u, v).to_vec();
///         assert_near2!(pt.magnitude2(), 1.0);
///         assert_near!(pt, sphere.normal(u, v));
///     }
/// }
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct RevolutionSurface<C> {
    curve: C,
    revolution: Revolution,
}

/// Surface constructed by linearly extruding a curve along a vector.
///
/// # Examples
/// ```
/// use monstertruck_geometry::prelude::*;
///
/// // entity curve
/// let cpts = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(0.0, 1.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
/// ];
/// let spts = vec![
///     vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0)],
///     vec![Point3::new(0.0, 1.0, 0.0), Point3::new(0.0, 1.0, 1.0)],
///     vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0)],
/// ];
/// let curve = BsplineCurve::new(KnotVector::bezier_knot(2), cpts);
///
/// // create extruded surface
/// let surface0 = ExtrusionSurface::by_extrusion(curve, Vector3::unit_z());
///
/// // same surface defined by B-spline description
/// let surface1 = BsplineSurface::new((KnotVector::bezier_knot(2), KnotVector::bezier_knot(1)), spts);
///
/// assert_eq!(surface0.range_tuple(), surface1.range_tuple());
///
/// const N: usize = 10;
/// for i in 0..=N {
///     for j in 0..=N {
///         let u = i as f64 / N as f64;
///         let v = j as f64 / N as f64;
///         assert_near!(
///             surface0.evaluate(u, v),
///             ParametricSurface::evaluate(&surface1, u, v)
///         );
///         assert_near!(surface0.derivative_u(u, v), surface1.derivative_u(u, v));
///         assert_near!(surface0.derivative_v(u, v), surface1.derivative_v(u, v));
///         assert_near!(surface0.derivative_uu(u, v), surface1.derivative_uu(u, v));
///         assert_near!(surface0.derivative_uv(u, v), surface1.derivative_uv(u, v));
///         assert_near!(surface0.derivative_vv(u, v), surface1.derivative_vv(u, v));
///         assert_near!(surface0.normal(u, v), surface1.normal(u, v));
///     }
/// }
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct ExtrusionSurface<C, V> {
    curve: C,
    vector: V,
}

/// invertible and transformable geometric element
/// # Examples
/// Curve processing example
/// ```
/// use monstertruck_geometry::prelude::*;
///
/// let curve: BsplineCurve<Point3> = BsplineCurve::new(
///     KnotVector::bezier_knot(2),
///     vec![
///         Point3::new(0.0, 0.0, 0.0),
///         Point3::new(0.0, 0.0, 1.0),
///         Point3::new(1.0, 0.0, 0.0),
///     ],
/// );
/// let mut processed = Processor::<_, Matrix4>::new(curve.clone());
///
/// // both curves are the same curve
/// const N: usize = 100;
/// for i in 0..=N {
///     let t = i as f64 / N as f64;
///     assert_eq!(curve.evaluate(t), processed.evaluate(t));
/// }
///
/// // Processed curve can inverted!
/// processed.invert();
/// for i in 0..=N {
///     let t = i as f64 / N as f64;
///     assert_eq!(curve.evaluate(1.0 - t), processed.evaluate(t));
/// }
/// ```
/// Surface processing example
/// ```
/// use monstertruck_geometry::prelude::*;
/// use std::f64::consts::PI;
///
/// let sphere = Sphere::new(Point3::new(1.0, 2.0, 3.0), 2.45);
/// let mut processed = Processor::<_, Matrix4>::new(sphere);
///
/// // both surfaces are the same surface
/// const N: usize = 100;
/// for i in 0..=N {
///     for j in 0..=N {
///         let u = PI * i as f64 / N as f64;
///         let v = 2.0 * PI * j as f64 / N as f64;
///         assert_eq!(sphere.evaluate(u, v), processed.evaluate(u, v));
///     }
/// }
///
/// // Processed surface can be inverted!
/// // Here, "invert surface" means swap (u, v)-axes.
/// processed.invert();
/// for i in 0..=N {
///     for j in 0..=N {
///         let u = PI * i as f64 / N as f64;
///         let v = 2.0 * PI * j as f64 / N as f64;
///         assert_eq!(sphere.evaluate(u, v), processed.evaluate(v, u));
///     }
/// }
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Processor<E, T> {
    entity: E,
    transform: T,
    orientation: bool,
}

/// The composited maps
///
/// # Examples
/// ```
/// use monstertruck_geometry::prelude::*;
///
/// // parameter curve
/// let curve = BsplineCurve::new(
///     KnotVector::bezier_knot(2),
///     vec![
///         Point2::new(1.0, 1.0),
///         Point2::new(1.0, 0.0),
///         Point2::new(0.0, 0.0),
///     ],
/// );
/// // surface
/// let surface = BsplineSurface::new(
///     (KnotVector::bezier_knot(2), KnotVector::bezier_knot(1)),
///     vec![
///         vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
///         vec![Point3::new(0.0, 0.0, 1.0), Point3::new(0.0, 1.0, 1.0)],
///         vec![Point3::new(1.0, 0.0, 1.0), Point3::new(1.0, 1.0, 1.0)],
///     ],
/// );
/// // the composite of parameter curve and surface
/// let pcurve = ParameterCurve::new(curve, surface);
/// assert_eq!(pcurve.range_tuple(), (0.0, 1.0));
///
/// const N: usize = 100;
/// for i in 0..=N {
///     let t = i as f64 / N as f64;
///     assert_near!(
///         pcurve.evaluate(t),
///         Point3::new(
///             (1.0 - t * t) * (1.0 - t * t),
///             (1.0 - t) * (1.0 - t),
///             1.0 - t * t * t * t,
///         ),
///     );
///     assert_near!(
///         pcurve.derivative(t),
///         Vector3::new(4.0 * t * (t * t - 1.0), 2.0 * (t - 1.0), -4.0 * t * t * t,),
///     );
///     assert_near!(
///         pcurve.derivative_2(t),
///         Vector3::new(4.0 * (3.0 * t * t - 1.0), 2.0, -12.0 * t * t,),
///     );
/// }
///
/// let t = 0.675;
/// let pt = pcurve.evaluate(t);
/// assert_near!(pcurve.search_parameter(pt, None, 100).unwrap(), t);
///
/// let pt = pt + Vector3::new(0.01, 0.06, -0.03);
/// assert!(pcurve.search_parameter(pt, None, 100).is_none());
/// let t = pcurve.search_nearest_parameter(pt, None, 100).unwrap();
/// assert!(pcurve.derivative(t).dot(pcurve.evaluate(t) - pt).so_small());
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct ParameterCurve<C, S> {
    curve: C,
    surface: S,
}

/// A G1 edge-blend surface smoothly connecting the boundaries of two surfaces.
///
/// The blend is spanned by a cubic Bezier in the `v`-direction whose end
/// control points ride the two boundary parameter curves `pcurve0`/`pcurve1`
/// and whose inner control points are offset along each boundary's surface
/// tangent (perpendicular to the boundary, in the tangent plane) by the
/// corresponding `magnitude` scalar function. This makes the blend positionally
/// and tangentially continuous with both supporting surfaces along their shared
/// boundaries -- a fillet-like additive decorator.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct EdgeBlendSurface<C0, S0, F0, C1, S1, F1> {
    pcurve0: ParameterCurve<C0, S0>,
    magnitude0: F0,
    pcurve1: ParameterCurve<C1, S1>,
    magnitude1: F1,
}

/// Intersection curve between two surfaces.
///
/// # Examples
/// ```
/// use std::f64::consts::PI;
/// use monstertruck_geometry::prelude::*;
///
/// // The intersection curve of the two spheres is the unit circle.
/// let sphere0 = Sphere::new(Point3::new(0.0, 0.0, 1.0), f64::sqrt(2.0));
/// let sphere1 = Sphere::new(Point3::new(0.0, 0.0, -1.0), f64::sqrt(2.0));
///
/// // Approximating a semicircle with a parabola
/// let bspcurve = BsplineCurve::new(
///     KnotVector::bezier_knot(2),
///     vec![
///         Point3::new(1.0, 0.0, 0.0),
///         Point3::new(0.0, 2.0, 0.0),
///         Point3::new(-1.0, 0.0, 0.0)
///     ],
/// );
///
/// // Declare an intersection curve
/// let intersection_curve = IntersectionCurve::new(sphere0, sphere1, bspcurve);
///
/// // All points of curve is on the upper half unit circle.
/// for i in 0..=100 {
///     let t = i as f64 / 100.0;
///     let p = intersection_curve.evaluate(t);
///     assert_near!(p.distance2(Point3::origin()), 1.0);
/// }
///
/// // Get the length of the half unit circle by Simpson's rule.
/// let coef = |i: usize| if matches!(i, 0 | 100) { 1.0 } else { 2.0 };
/// let sum = (0..=100).fold(0.0, |sum, i| {
///     let t = i as f64 / 100.0;
///     sum + intersection_curve.derivative(t).magnitude() * coef(i)
/// });
/// let length = sum / 100.0 / 2.0;
/// assert!(f64::abs(length - PI) < 1.0e-4 * PI);
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct IntersectionCurve<C, S0, S1> {
    surface0: S0,
    surface1: S1,
    leader: C,
}

/// Surface curve with exact face-local boundaries on both supporting surfaces.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct SurfaceCurve<C, S0, S1, T0, T1> {
    surface0: S0,
    surface1: S1,
    leader: C,
    boundary0: Option<T0>,
    boundary1: Option<T1>,
}

/// trimmed curve for parametric curve
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct TrimmedCurve<C> {
    curve: C,
    range: (f64, f64),
}

/// homotopy surface connecting two curves.
///
/// # Examples
/// ```
/// use monstertruck_geometry::prelude::*;
///
/// // create homotopy between two lines
/// let line0 = Line(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
/// let line1 = Line(Point3::new(0.0, -1.0, 1.0), Point3::new(0.0, 1.0, 1.0));
/// let homotopy = HomotopySurface::new(line0, line1);
///
/// // explicit definition
/// let surface = |u: f64, v: f64| {
///     Point3::new((2.0 * u - 1.0) * (1.0 - v), (2.0 * u - 1.0) * v, v)
/// };
/// let uder = |v: f64| Vector3::new(2.0 * (1.0 - v), 2.0 * v, 0.0);
/// let vder = |u: f64| Vector3::new(1.0 - 2.0 * u, 2.0 * u - 1.0, 1.0);
/// let uvder = Vector3::new(-2.0, 2.0, 0.0);
///
/// // test
/// for i in 0..=10 {
///     for j in 0..=10 {
///         let (u, v) = (i as f64 / 10.0, j as f64 / 10.0);
///         assert_near!(homotopy.evaluate(u, v), surface(u, v));
///         assert_near!(homotopy.derivative_u(u, v), uder(v));
///         assert_near!(homotopy.derivative_v(u, v), vder(u));
///         assert!(homotopy.derivative_uu(u, v).so_small());
///         assert_near!(homotopy.derivative_uv(u, v), uvder);
///         assert!(homotopy.derivative_vv(u, v).so_small());
///     }
/// }
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct HomotopySurface<C0, C1> {
    curve0: C0,
    curve1: C1,
}

/// Rolling ball fillet surface along one edge between two surfaces.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct RollingBallFilletSurface<C, S0, S1, R> {
    edge_curve: C,
    surface0: S0,
    surface1: S1,
    radius: R,
}

/// Orbit curve of a rolling ball fillet contact point.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct RollingBallFilletContactCurve<C, S0, S1, R> {
    surface: RollingBallFilletSurface<C, S0, S1, R>,
    index: usize,
}

/// Approximate surface for fillets.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct ApproximateFilletSurface<S0, S1> {
    knot_vec: KnotVector,
    surface0: S0,
    side_control_points0: Vec<Point2>,
    tangent_vecs0: Vec<Vector2>,
    surface1: S1,
    side_control_points1: Vec<Point2>,
    tangent_vecs1: Vec<Vector2>,
    weights: Vec<f64>,
}

/// Curve `entity` offset by another parametric curve `offset` added pointwise.
///
/// `OffsetCurve<C, N>::subs(t)` returns `entity.subs(t) + offset.subs(t)`, and
/// every derivative is the sum of the corresponding `entity` / `offset`
/// derivatives. The parameter range and period come from `entity`.
///
/// Pair with [`NormalOffsetField`] when you want a normal-direction offset
/// (i.e. an offset of magnitude `scalar(t)` along the curve normal): build
/// `OffsetCurve::new(entity, NormalOffsetField::new(entity.clone(), scalar))`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct OffsetCurve<C, N> {
    entity: C,
    offset: N,
}

/// Surface `entity` offset by another parametric surface `offset` added pointwise.
///
/// Analogous to [`OffsetCurve`] but for two-parameter surfaces.
/// `OffsetSurface<S, N>::subs(u, v)` returns
/// `entity.subs(u, v) + offset.subs(u, v)`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct OffsetSurface<S, N> {
    entity: S,
    offset: N,
}

/// Unit-normal field of a curve in the plane or a surface in space,
/// scaled by a per-parameter [`CurveScalarFunction`] or
/// [`SurfaceScalarFunction`] respectively.
///
/// Implements [`ParametricCurve`] when `T: ParametricCurve2D` (with the
/// normal taken as the in-plane rotation of the tangent by 90 degrees)
/// and [`ParametricSurface`] when `T: ParametricSurface3D` (with the
/// normal taken as the normalised cross product of the parametric
/// derivatives). The scalar function supplies the offset magnitude.
///
/// Typically used as the `N` parameter of [`OffsetCurve`] / [`OffsetSurface`]
/// to express the classical offset-curve / offset-surface constructions.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, SelfSameGeometry)]
pub struct NormalOffsetField<T, F> {
    entity: T,
    scalar: F,
}

/// Scalar function parameterised over a curve's `t` parameter.
///
/// Implementors include `f64` (interpreted as a constant function) and
/// per-coordinate projections of the small fixed-dimension spline types
/// (e.g. `BsplineCurve<Vector1>`). Typically supplied as the magnitude
/// component of a [`NormalOffsetField`] on a 2D curve.
pub trait CurveScalarFunction: Clone {
    /// Returns the `n`th-order derivative at parameter `t`.
    fn derivative_n(&self, n: usize, t: f64) -> f64;
    /// Substitutes the parameter `t` and returns the function value.
    #[inline]
    fn evaluate(&self, t: f64) -> f64 { self.derivative_n(0, t) }
    /// Returns the first derivative.
    #[inline]
    fn derivative(&self, t: f64) -> f64 { self.derivative_n(1, t) }
    /// Returns the second derivative.
    #[inline]
    fn derivative_2(&self, t: f64) -> f64 { self.derivative_n(2, t) }
    /// Returns derivatives `0..=max_order` as a [`CurveDerivatives<f64>`].
    #[inline]
    fn derivatives(&self, max_order: usize, t: f64) -> CurveDerivatives<f64> {
        (0..=max_order).map(|n| self.derivative_n(n, t)).collect()
    }
}

/// Scalar function parameterised over a surface's `(u, v)` parameters.
///
/// Implementors include `f64` (interpreted as a constant function) and
/// per-coordinate projections of fixed-dimension spline surface types
/// (e.g. `BsplineSurface<Vector1>`). Typically supplied as the magnitude
/// component of a [`NormalOffsetField`] on a 3D surface.
pub trait SurfaceScalarFunction: Clone {
    /// Returns the mixed partial derivative
    /// $\partial^{m+n} f / \partial u^m \partial v^n$ at `(u, v)`.
    fn derivative_mn(&self, m: usize, n: usize, u: f64, v: f64) -> f64;
    /// Substitutes the parameter `(u, v)` and returns the function value.
    #[inline]
    fn evaluate(&self, u: f64, v: f64) -> f64 { self.derivative_mn(0, 0, u, v) }
    /// First partial derivative with respect to `u`.
    #[inline]
    fn derivative_u(&self, u: f64, v: f64) -> f64 { self.derivative_mn(1, 0, u, v) }
    /// First partial derivative with respect to `v`.
    #[inline]
    fn derivative_v(&self, u: f64, v: f64) -> f64 { self.derivative_mn(0, 1, u, v) }
    /// Second partial derivative with respect to `u`.
    #[inline]
    fn derivative_uu(&self, u: f64, v: f64) -> f64 { self.derivative_mn(2, 0, u, v) }
    /// Mixed second partial derivative with respect to `u` and `v`.
    #[inline]
    fn derivative_uv(&self, u: f64, v: f64) -> f64 { self.derivative_mn(1, 1, u, v) }
    /// Second partial derivative with respect to `v`.
    #[inline]
    fn derivative_vv(&self, u: f64, v: f64) -> f64 { self.derivative_mn(0, 2, u, v) }
    /// Returns the lower-triangular block of partial derivatives up to total order `max_order`.
    #[inline]
    fn derivatives(&self, max_order: usize, u: f64, v: f64) -> SurfaceDerivatives<f64> {
        let mut derivatives = SurfaceDerivatives::new(max_order);
        (0..=max_order).for_each(|m| {
            (0..=max_order - m).for_each(|n| derivatives[m][n] = self.derivative_mn(m, n, u, v))
        });
        derivatives
    }
}

// The canonical names are `CurveScalarFunction` and `SurfaceScalarFunction`:
// they say what the function is parameterised *over* and use only the
// concrete words "curve" and "surface" that appear all over the kernel.
// The upstream `truck-geometry` spellings `ScalarFunctionD1` /
// `ScalarFunctionD2` -- whose trailing `D1`/`D2` reads like a generic
// argument over our existing parameter-space markers, which it is not --
// are kept as `#[deprecated]` re-exports so code ported from upstream
// keeps compiling with a warning.
#[deprecated(since = "0.3.1", note = "renamed to `CurveScalarFunction`.")]
pub use self::CurveScalarFunction as ScalarFunctionD1;
#[deprecated(since = "0.3.1", note = "renamed to `SurfaceScalarFunction`.")]
pub use self::SurfaceScalarFunction as ScalarFunctionD2;

// `NormalField` in upstream conflates "normal field" with "offset along
// the normal." `NormalOffsetField` makes the intent explicit. The alias
// keeps porting from `truck-geometry` compiling.
#[deprecated(since = "0.3.1", note = "renamed to `NormalOffsetField`.")]
pub use self::NormalOffsetField as NormalField;

pub use self::ApproximateFilletSurface as ApproxFilletSurface;
pub use self::RollingBallFilletSurface as RbfSurface;

mod approximate_fillet_surface;
mod edge_blend;
mod extruded_curve;
mod homotopy;
mod intersection_curve;
pub use intersection_curve::thin_division;
mod offset;
mod pcurve;
mod processor;
mod revolved_curve;
/// Structures and traits associated with rolling ball fillet surfaces.
pub mod rolling_ball_fillet;
/// Compatibility exports for historical rolling ball fillet names.
pub mod rbf_surface {
    pub use super::rolling_ball_fillet::RadiusFunction;
}
mod scalar_function;
mod trimmed_curve;

fn bound2opt<T>(x: Bound<T>) -> Option<T> {
    match x {
        Bound::Included(x) => Some(x),
        Bound::Excluded(x) => Some(x),
        Bound::Unbounded => None,
    }
}

// Intersection of two parameter ranges, kept module-private in `decorators` so
// both `HomotopySurface` and `EdgeBlendSurface` can restrict their `u`-domain to
// the overlap of their two boundary curves.
fn range_common_part<R0, R1>(range0: &R0, range1: &R1) -> ParameterRange
where
    R0: std::ops::RangeBounds<f64>,
    R1: std::ops::RangeBounds<f64>, {
    use std::cmp::Ordering;
    let (t00, t01) = (range0.start_bound(), range0.end_bound());
    let (t10, t11) = (range1.start_bound(), range1.end_bound());
    let t0 = match (bound2opt(t00), bound2opt(t10)) {
        // SAFETY: parameter range bounds are finite `f64` values, so `partial_cmp` always returns `Some`.
        (Some(x), Some(y)) => match x.partial_cmp(y).unwrap() {
            Ordering::Greater => t00,
            Ordering::Less => t10,
            Ordering::Equal => match matches!(t00, Bound::Excluded(_)) {
                true => t00,
                false => t10,
            },
        },
        (_, None) => t00,
        (None, _) => t10,
    };
    let t1 = match (bound2opt(t01), bound2opt(t11)) {
        // SAFETY: parameter range bounds are finite `f64` values, so `partial_cmp` always returns `Some`.
        (Some(x), Some(y)) => match x.partial_cmp(y).unwrap() {
            Ordering::Less => t01,
            Ordering::Greater => t11,
            Ordering::Equal => match matches!(t01, Bound::Excluded(_)) {
                true => t01,
                false => t11,
            },
        },
        (_, None) => t01,
        (None, _) => t11,
    };
    (t0.cloned(), t1.cloned())
}

#[test]
fn test_range_common_part() {
    use std::ops::RangeBounds;
    fn to_parameter_range<R: RangeBounds<f64>>(x: &R) -> ParameterRange {
        (x.start_bound().cloned(), x.end_bound().cloned())
    }
    fn compare<R0, R1, R2>(range0: R0, range1: R1, range2: R2)
    where
        R0: RangeBounds<f64>,
        R1: RangeBounds<f64>,
        R2: RangeBounds<f64>, {
        assert_eq!(
            range_common_part(&range0, &range1),
            to_parameter_range(&range2),
        );
        assert_eq!(
            range_common_part(&range1, &range0),
            to_parameter_range(&range2),
        );
    }
    compare(0.0..2.0, -1.0..1.0, 0.0..1.0);
    compare(0.0..=2.0, -1.0..2.0, 0.0..2.0);
    compare(..=2.0, 0.0.., 0.0..=2.0);
    compare(
        (Bound::Excluded(0.0), Bound::Included(1.0)),
        0.0..1.0,
        (Bound::Excluded(0.0), Bound::Excluded(1.0)),
    );
    compare(0.0..1.0, 2.0..3.0, 2.0..1.0)
}
