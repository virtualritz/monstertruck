use super::*;
use monstertruck_core::newton::{self, CalcOutput};
use monstertruck_traits::{ParametricCurve as ParametricCurveTrait, SnapCurveEndpoints};
use std::ops::Bound;

impl<P> SnapCurveEndpoints for Line<P> {}

impl<P> SnapCurveEndpoints for BsplineCurve<P> {}

impl<V> SnapCurveEndpoints for NurbsCurve<V> {}

impl<C: SnapCurveEndpoints, T> SnapCurveEndpoints for Processor<C, T> {
    fn snap_endpoints(&mut self, front: Point3, back: Point3) {
        self.entity.snap_endpoints(front, back);
    }
}

impl<C: SnapCurveEndpoints> SnapCurveEndpoints for TrimmedCurve<C> {
    fn snap_endpoints(&mut self, front: Point3, back: Point3) {
        self.curve_mut().snap_endpoints(front, back);
    }
}

impl<C, S> SnapCurveEndpoints for ParameterCurve<C, S> {}

impl<C: SnapCurveEndpoints, S0, S1> SnapCurveEndpoints for IntersectionCurve<C, S0, S1> {
    fn snap_endpoints(&mut self, front: Point3, back: Point3) {
        self.leader.snap_endpoints(front, back);
    }
}

fn double_projection<S0, S1>(
    surface0: &S0,
    hint0: Option<(f64, f64)>,
    surface1: &S1,
    hint1: Option<(f64, f64)>,
    plane_point: Point3,
    plane_normal: Vector3,
    trials: usize,
) -> Option<(Point3, Point2, Point2)>
where
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    let function = move |Vector4 { x, y, z, w }| {
        let ders0 = surface0.derivatives(1, x, y);
        let (pt0, uder0, vder0) = (ders0[0][0], ders0[1][0], ders0[0][1]);
        let ders1 = surface1.derivatives(1, z, w);
        let (pt1, uder1, vder1) = (ders1[0][0], ders1[1][0], ders1[0][1]);
        CalcOutput {
            value: (pt0 - pt1).extend(plane_normal.dot((pt0 + pt1) / 2.0 - plane_point.to_vec())),
            derivation: Matrix4::from_cols(
                uder0.extend(plane_normal.dot(uder0) / 2.0),
                vder0.extend(plane_normal.dot(vder0) / 2.0),
                (-uder1).extend(plane_normal.dot(uder1) / 2.0),
                (-vder1).extend(plane_normal.dot(vder1) / 2.0),
            ),
        }
    };
    let (x, y) = hint0.or_else(|| surface0.search_nearest_parameter(plane_point, hint0, trials))?;
    let (z, w) = hint1.or_else(|| surface1.search_nearest_parameter(plane_point, hint1, trials))?;
    let res = newton::solve(function, Vector4 { x, y, z, w }, trials);
    let Vector4 { x, y, z, w } = match res {
        Ok(res) => res,
        Err(_) => {
            let pt0 = surface0.evaluate(x, y);
            let pt1 = surface1.evaluate(z, w);
            let n0 = surface0.normal(x, y);
            let n1 = surface1.normal(z, w);
            // Newton's method may fail when the Jacobian is singular, which happens
            // when surfaces are coplanar or tangent.
            // If the points are close enough and normals are parallel (indicating
            // coplanarity), accept the initial guess as a valid intersection point.
            if pt0.near(&pt1) && n0.cross(n1).magnitude() < TOLERANCE {
                let point = pt0.midpoint(pt1);
                return Some((point, Point2::new(x, y), Point2::new(z, w)));
            } else {
                return None;
            }
        }
    };
    let point = surface0.evaluate(x, y).midpoint(surface1.evaluate(z, w));
    Some((point, Point2::new(x, y), Point2::new(z, w)))
}

impl<C, S0, S1> IntersectionCurve<C, S0, S1> {
    /// Constructor
    #[inline(always)]
    pub fn new(surface0: S0, surface1: S1, leader: C) -> Self {
        Self {
            surface0,
            surface1,
            leader,
        }
    }
    /// This curve is a part of intersection of `self.surface0()` and `self.surface1()`.
    #[inline(always)]
    pub fn surface0(&self) -> &S0 { &self.surface0 }
    /// This curve is a part of intersection of `self.surface0()` and `self.surface1()`.
    #[inline(always)]
    pub fn surface1(&self) -> &S1 { &self.surface1 }
    /// Returns the polyline leading this curve.
    #[inline(always)]
    pub fn leader(&self) -> &C { &self.leader }
    /// This curve is a part of intersection of `self.surface0()` and `self.surface1()`.
    #[inline(always)]
    pub fn surface0_mut(&mut self) -> &mut S0 { &mut self.surface0 }
    /// This curve is a part of intersection of `self.surface0()` and `self.surface1()`.
    #[inline(always)]
    pub fn surface1_mut(&mut self) -> &mut S1 { &mut self.surface1 }
    /// Returns the curve leading this curve.
    #[inline(always)]
    pub fn leader_mut(&mut self) -> &mut C { &mut self.leader }
    /// destruct `self`.
    #[inline(always)]
    pub fn destruct(self) -> (S0, S1, C) { (self.surface0, self.surface1, self.leader) }
}

impl<C, S0, S1, T0, T1> SurfaceCurve<C, S0, S1, T0, T1> {
    /// Constructor with face-local boundary curves.
    #[inline(always)]
    pub fn with_boundaries(
        surface0: S0,
        surface1: S1,
        leader: C,
        boundary0: Option<T0>,
        boundary1: Option<T1>,
    ) -> Self {
        Self {
            surface0,
            surface1,
            leader,
            boundary0,
            boundary1,
        }
    }
    /// This curve is carried by `self.surface0()` and `self.surface1()`.
    #[inline(always)]
    pub fn surface0(&self) -> &S0 { &self.surface0 }
    /// This curve is carried by `self.surface0()` and `self.surface1()`.
    #[inline(always)]
    pub fn surface1(&self) -> &S1 { &self.surface1 }
    /// Returns the 3D leader curve.
    #[inline(always)]
    pub fn leader(&self) -> &C { &self.leader }
    /// This curve is carried by `self.surface0()` and `self.surface1()`.
    #[inline(always)]
    pub fn surface0_mut(&mut self) -> &mut S0 { &mut self.surface0 }
    /// This curve is carried by `self.surface0()` and `self.surface1()`.
    #[inline(always)]
    pub fn surface1_mut(&mut self) -> &mut S1 { &mut self.surface1 }
    /// Returns the 3D leader curve.
    #[inline(always)]
    pub fn leader_mut(&mut self) -> &mut C { &mut self.leader }
    /// Returns the exact face-local boundary on `surface0`, if available.
    #[inline(always)]
    pub fn boundary0(&self) -> Option<&T0> { self.boundary0.as_ref() }
    /// Returns the exact face-local boundary on `surface1`, if available.
    #[inline(always)]
    pub fn boundary1(&self) -> Option<&T1> { self.boundary1.as_ref() }
    /// Returns the mutable exact face-local boundary on `surface0`, if available.
    #[inline(always)]
    pub fn boundary0_mut(&mut self) -> Option<&mut T0> { self.boundary0.as_mut() }
    /// Returns the mutable exact face-local boundary on `surface1`, if available.
    #[inline(always)]
    pub fn boundary1_mut(&mut self) -> Option<&mut T1> { self.boundary1.as_mut() }
    /// Destructs `self`, preserving face-local boundaries.
    #[inline(always)]
    pub fn destruct_with_boundaries(self) -> (S0, S1, C, Option<T0>, Option<T1>) {
        (
            self.surface0,
            self.surface1,
            self.leader,
            self.boundary0,
            self.boundary1,
        )
    }
}

impl<C, S0, S1> IntersectionCurve<C, S0, S1>
where
    C: ParametricCurve3D,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    /// Search triple value of the point corresponding to the parameter `t`.
    /// - the coordinate on 3D space
    /// - the uv coordinate on `self.surface0()`
    /// - the uv coordinate on `self.surface1()`
    #[inline(always)]
    pub fn search_triple(&self, t: f64, trials: usize) -> Option<(Point3, Point2, Point2)> {
        let point = self.leader.evaluate(t);
        double_projection(
            self.surface0(),
            None,
            self.surface1(),
            None,
            point,
            self.leader.derivative(t),
            trials,
        )
        .or_else(|| self.search_nearest_point(point, None, None, trials))
    }
    /// Search triple value of the point nearest to `point`.
    /// - the coordinate on 3D space
    /// - the uv coordinate on `self.surface0()`
    /// - the uv coordinate on `self.surface1()`
    pub fn search_nearest_point(
        &self,
        point: Point3,
        hint0: Option<(f64, f64)>,
        hint1: Option<(f64, f64)>,
        trials: usize,
    ) -> Option<(Point3, Point2, Point2)> {
        let (surface0, surface1) = (self.surface0(), self.surface1());
        let function = |Vector4 { x, y, z, w }| {
            let ders0 = surface0.derivatives(2, x, y);
            let (pt0, uder0, vder0, uuder0, uvder0, vvder0) = (
                ders0[0][0],
                ders0[1][0],
                ders0[0][1],
                ders0[2][0],
                ders0[1][1],
                ders0[0][2],
            );
            let ders1 = surface1.derivatives(2, z, w);
            let (pt1, uder1, vder1, uuder1, uvder1, vvder1) = (
                ders1[0][0],
                ders1[1][0],
                ders1[0][1],
                ders1[2][0],
                ders1[1][1],
                ders1[0][2],
            );
            let diff = (pt0 + pt1) / 2.0 - point.to_vec();
            let (n0, n1) = (uder0.cross(vder0), uder1.cross(vder1));
            let n = n0.cross(n1);
            let n_xder = (uuder0.cross(vder0) + uder0.cross(uvder0)).cross(n1);
            let n_yder = (uvder0.cross(vder0) + uder0.cross(vvder0)).cross(n1);
            let n_zder = n0.cross(uuder1.cross(vder1) + uder1.cross(uvder1));
            let n_wder = n0.cross(uvder1.cross(vder1) + uder1.cross(vvder1));
            CalcOutput {
                value: (pt0 - pt1).extend(n.dot(diff)),
                derivation: Matrix4::from_cols(
                    uder0.extend(n_xder.dot(diff) + n.dot(uder0) / 2.0),
                    vder0.extend(n_yder.dot(diff) + n.dot(vder0) / 2.0),
                    (-uder1).extend(n_zder.dot(diff) + n.dot(uder1) / 2.0),
                    (-vder1).extend(n_wder.dot(diff) + n.dot(vder1) / 2.0),
                ),
            }
        };
        let (x, y) = hint0.or_else(|| surface0.search_nearest_parameter(point, hint0, trials))?;
        let (z, w) = hint1.or_else(|| surface1.search_nearest_parameter(point, hint1, trials))?;
        let Vector4 { x, y, z, w } =
            newton::solve(function, Vector4 { x, y, z, w }, trials).ok()?;
        let point = surface0.evaluate(x, y).midpoint(surface1.evaluate(z, w));
        Some((point, Point2::new(x, y), Point2::new(z, w)))
    }
}

impl<C, S0, S1, T0, T1> SurfaceCurve<C, S0, S1, T0, T1>
where
    C: ParametricCurve3D,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    /// Search triple value of the point corresponding to the parameter `t`.
    /// - the coordinate on 3D space
    /// - the uv coordinate on `self.surface0()`
    /// - the uv coordinate on `self.surface1()`
    #[inline(always)]
    pub fn search_triple(&self, t: f64, trials: usize) -> Option<(Point3, Point2, Point2)> {
        let point = self.leader.evaluate(t);
        double_projection(
            self.surface0(),
            None,
            self.surface1(),
            None,
            point,
            self.leader.derivative(t),
            trials,
        )
        .or_else(|| self.search_nearest_point(point, None, None, trials))
    }
    /// Search triple value of the point nearest to `point`.
    /// - the coordinate on 3D space
    /// - the uv coordinate on `self.surface0()`
    /// - the uv coordinate on `self.surface1()`
    pub fn search_nearest_point(
        &self,
        point: Point3,
        hint0: Option<(f64, f64)>,
        hint1: Option<(f64, f64)>,
        trials: usize,
    ) -> Option<(Point3, Point2, Point2)> {
        let intersection = IntersectionCurve::new(
            self.surface0.clone(),
            self.surface1.clone(),
            self.leader.clone(),
        );
        intersection.search_nearest_point(point, hint0, hint1, trials)
    }
}

#[derive(Clone, Copy, Debug)]
struct DerRoutineImmutableArgs {
    s0ders: SurfaceDerivatives<Vector3>,
    s0normal: Vector3,
    s1ders: SurfaceDerivatives<Vector3>,
    s1normal: Vector3,
    leaders: CurveDerivatives<Vector3>,
}

fn curve_der_n(
    sum0: Vector3,
    s0normal: Vector3,
    sum1: Vector3,
    s1normal: Vector3,
    leaders: &CurveDerivatives<Vector3>,
    cders: &CurveDerivatives<Vector3>,
    n: usize,
) -> Vector3 {
    let mat = Matrix3::from_cols(s0normal, s1normal, leaders[1]).transpose();
    let sub = leaders.element_wise_derivatives(cders, |x, y| x - y);
    let suml = leaders
        .derivative()
        .combinatorial_derivative(&sub, Vector3::dot, n);
    let b = Vector3::new(s0normal.dot(sum0), s1normal.dot(sum1), suml);
    // SAFETY: the matrix columns are two surface normals and the leader curve tangent,
    // which are linearly independent at a transversal intersection point.
    mat.invert().unwrap() * b
}

fn uv_der_n(
    uder: Vector3,
    vder: Vector3,
    sum: Vector3,
    normal: Vector3,
    cder_n: Vector3,
) -> Vector2 {
    let mat = Matrix3::from_cols(uder, vder, normal);
    let b = cder_n - sum;
    // SAFETY: the matrix columns are the surface partial derivatives and normal,
    // which are linearly independent at a regular surface point.
    let uv_der_n = mat.invert().unwrap() * b;
    debug_assert!(uv_der_n.z.abs() < 1.0e-4, "{}", uv_der_n.z.abs());
    Vector2::new(uv_der_n.x, uv_der_n.y)
}

fn der_routine(
    DerRoutineImmutableArgs {
        s0ders,
        s0normal,
        s1ders,
        s1normal,
        leaders,
    }: &DerRoutineImmutableArgs,
    uv0ders: &mut CurveDerivatives<Vector2>,
    uv1ders: &mut CurveDerivatives<Vector2>,
    cders: &mut CurveDerivatives<Vector3>,
    n: usize,
) {
    let sum0 = s0ders.composite_der(uv0ders, n);
    let sum1 = s1ders.composite_der(uv1ders, n);
    cders[n] = curve_der_n(sum0, *s0normal, sum1, *s1normal, leaders, cders, n);
    let (uder0, vder0) = (s0ders[1][0], s0ders[0][1]);
    uv0ders[n] = uv_der_n(uder0, vder0, sum0, *s0normal, cders[n]);
    let (uder1, vder1) = (s1ders[1][0], s1ders[0][1]);
    uv1ders[n] = uv_der_n(uder1, vder1, sum1, *s1normal, cders[n]);
}

impl<C, S0, S1> ParametricCurveTrait for IntersectionCurve<C, S0, S1>
where
    C: ParametricCurve3D,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    type Point = Point3;
    type Vector = Vector3;
    fn evaluate(&self, t: f64) -> Point3 {
        if let (Bound::Included(t0), Bound::Included(_)) = self.leader.parameter_range()
            && t.near(&t0)
        {
            self.leader.evaluate(t0)
        } else if let (Bound::Included(_), Bound::Included(t1)) = self.leader.parameter_range()
            && t.near(&t1)
        {
            self.leader.evaluate(t1)
        } else {
            self.search_triple(t, 100)
                .map(|triple| triple.0)
                .unwrap_or_else(|| self.leader.evaluate(t))
        }
    }
    fn derivative(&self, t: f64) -> Vector3 {
        let IntersectionCurve {
            surface0,
            surface1,
            leader,
        } = self;
        let [l, l_der, l_der2] = leader.derivatives(2, t).to_array::<3>();
        let (c, uv0, uv1) = match self.search_triple(t, 100) {
            Some(triple) => triple,
            None => return leader.derivative(t),
        };
        let (n0, n1) = (surface0.normal(uv0.x, uv0.y), surface1.normal(uv1.x, uv1.y));
        let n = n0.cross(n1);
        let k = (l_der.magnitude2() - (c - l).dot(l_der2)) / n.dot(l_der);
        n * k
    }
    #[inline(always)]
    fn derivative_2(&self, t: f64) -> Vector3 { self.derivative_n(2, t) }
    #[inline(always)]
    fn derivative_n(&self, n: usize, t: f64) -> Vector3 {
        match n {
            0 => return self.evaluate(t).to_vec(),
            1 => return self.derivative(t),
            _ => {}
        }
        self.derivatives(n, t)[n]
    }
    fn derivatives(&self, n: usize, t: f64) -> CurveDerivatives<Vector3> {
        let (c, uv0, uv1) = match self.search_triple(t, 100) {
            Some(triple) => triple,
            None => {
                let leader_ders = self.leader.derivatives(n, t);
                let mut cders = CurveDerivatives::new(n);
                (0..=n).for_each(|i| cders[i] = leader_ders[i]);
                return cders;
            }
        };
        let mut uv0ders = CurveDerivatives::new(n);
        uv0ders[0] = uv0.to_vec();
        let mut uv1ders = CurveDerivatives::new(n);
        uv1ders[0] = uv1.to_vec();
        let mut cders = CurveDerivatives::new(n);
        cders[0] = c.to_vec();

        let IntersectionCurve {
            surface0,
            surface1,
            leader,
        } = self;
        let info = DerRoutineImmutableArgs {
            s0ders: surface0.derivatives(n, uv0.x, uv0.y),
            s0normal: surface0.normal(uv0.x, uv0.y),
            s1ders: surface1.derivatives(n, uv1.x, uv1.y),
            s1normal: surface1.normal(uv1.x, uv1.y),
            leaders: leader.derivatives(n + 1, t),
        };
        (1..=n).for_each(|i| der_routine(&info, &mut uv0ders, &mut uv1ders, &mut cders, i));
        cders
    }
    #[inline(always)]
    fn parameter_range(&self) -> ParameterRange { self.leader.parameter_range() }
}

impl<C, S0, S1> BoundedCurve for IntersectionCurve<C, S0, S1>
where
    C: ParametricCurve3D + BoundedCurve,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
}

impl<C, S0, S1> ParameterDivision1D for IntersectionCurve<C, S0, S1>
where
    C: ParametricCurve3D + BoundedCurve,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    type Point = Point3;
    #[inline(always)]
    fn parameter_division(&self, range: (f64, f64), tol: f64) -> (Vec<f64>, Vec<Point3>) {
        algo::curve::parameter_division(self, range, tol)
    }
}

impl<C, S0, S1> Cut for IntersectionCurve<C, S0, S1>
where
    C: Cut<Point = Point3, Vector = Vector3> + SnapCurveEndpoints,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    fn cut(&mut self, t: f64) -> Self {
        let split = self.search_triple(t, 100).map(|(point, _, _)| point);
        let front = self.front();
        let mut leader = self.leader.cut(t);
        let back = leader.back();
        if let Some(point) = split {
            self.leader.snap_endpoints(front, point);
            leader.snap_endpoints(point, back);
        }
        Self {
            surface0: self.surface0.clone(),
            surface1: self.surface1.clone(),
            leader,
        }
    }
}

impl<C: Invertible, S0: Clone, S1: Clone> Invertible for IntersectionCurve<C, S0, S1> {
    fn invert(&mut self) { self.leader.invert(); }
}

impl<C, S0, S1> SearchParameter<CurveParameter> for IntersectionCurve<C, S0, S1>
where
    C: ParametricCurve3D + SearchNearestParameter<CurveParameter, Point = Point3>,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    type Point = Point3;
    fn search_parameter<H: Into<SearchParameterHint1D>>(
        &self,
        point: Point3,
        hint: H,
        trials: usize,
    ) -> Option<f64> {
        let t = self
            .leader()
            .search_nearest_parameter(point, hint, trials)?;
        let pt = self.evaluate(t);
        match pt.near(&point) {
            true => Some(t),
            false => None,
        }
    }
}

impl<C, S0, S1> SearchNearestParameter<CurveParameter> for IntersectionCurve<C, S0, S1>
where
    C: ParametricCurve3D + SearchNearestParameter<CurveParameter, Point = Point3>,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    type Point = Point3;
    fn search_nearest_parameter<H: Into<SearchParameterHint1D>>(
        &self,
        point: Point3,
        hint: H,
        trials: usize,
    ) -> Option<f64> {
        let (near_point, _, _) = self.search_nearest_point(point, None, None, trials)?;
        self.leader()
            .search_nearest_parameter(near_point, hint, trials)
    }
}

impl<C, S0, S1> Transformed<Matrix4> for IntersectionCurve<C, S0, S1>
where
    C: Transformed<Matrix4>,
    S0: Transformed<Matrix4>,
    S1: Transformed<Matrix4>,
{
    fn transform_by(&mut self, trans: Matrix4) {
        self.surface0.transform_by(trans);
        self.surface1.transform_by(trans);
        self.leader.transform_by(trans);
    }
}

impl<C: BoundedCurve> IntersectionCurve<C, Plane, Plane> {
    /// Optimizes intersection curve of [`Plane`] into [`Line`].
    #[inline]
    pub fn optimize(&self) -> Line<C::Point> {
        let (s, t) = self.leader.range_tuple();
        Line(self.leader.evaluate(s), self.leader.evaluate(t))
    }
}

impl<C, S0, S1, T0, T1> ParametricCurveTrait for SurfaceCurve<C, S0, S1, T0, T1>
where
    C: ParametricCurve3D,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3>,
    T0: Clone,
    T1: Clone,
{
    type Point = Point3;
    type Vector = Vector3;
    fn evaluate(&self, t: f64) -> Point3 {
        IntersectionCurve::new(
            self.surface0.clone(),
            self.surface1.clone(),
            self.leader.clone(),
        )
        .evaluate(t)
    }
    fn derivative(&self, t: f64) -> Vector3 {
        IntersectionCurve::new(
            self.surface0.clone(),
            self.surface1.clone(),
            self.leader.clone(),
        )
        .derivative(t)
    }
    fn derivative_2(&self, t: f64) -> Vector3 {
        IntersectionCurve::new(
            self.surface0.clone(),
            self.surface1.clone(),
            self.leader.clone(),
        )
        .derivative_2(t)
    }
    fn derivative_n(&self, n: usize, t: f64) -> Vector3 {
        IntersectionCurve::new(
            self.surface0.clone(),
            self.surface1.clone(),
            self.leader.clone(),
        )
        .derivative_n(n, t)
    }
    fn derivatives(&self, n: usize, t: f64) -> CurveDerivatives<Vector3> {
        IntersectionCurve::new(
            self.surface0.clone(),
            self.surface1.clone(),
            self.leader.clone(),
        )
        .derivatives(n, t)
    }
    #[inline(always)]
    fn parameter_range(&self) -> ParameterRange { self.leader.parameter_range() }
}

impl<C, S0, S1, T0, T1> BoundedCurve for SurfaceCurve<C, S0, S1, T0, T1>
where
    C: ParametricCurve3D + BoundedCurve,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
    T0: Clone,
    T1: Clone,
{
}

impl<C, S0, S1, T0, T1> ParameterDivision1D for SurfaceCurve<C, S0, S1, T0, T1>
where
    C: ParametricCurve3D + BoundedCurve,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
    T0: Clone,
    T1: Clone,
{
    type Point = Point3;
    #[inline(always)]
    fn parameter_division(&self, range: (f64, f64), tol: f64) -> (Vec<f64>, Vec<Point3>) {
        algo::curve::parameter_division(self, range, tol)
    }
}

impl<C, S0, S1, T0, T1> Cut for SurfaceCurve<C, S0, S1, T0, T1>
where
    C: Cut<Point = Point3, Vector = Vector3>,
    T0: Cut,
    T1: Cut,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
{
    #[inline(always)]
    fn cut(&mut self, t: f64) -> Self {
        Self {
            surface0: self.surface0.clone(),
            surface1: self.surface1.clone(),
            leader: self.leader.cut(t),
            boundary0: self.boundary0.as_mut().map(|boundary| boundary.cut(t)),
            boundary1: self.boundary1.as_mut().map(|boundary| boundary.cut(t)),
        }
    }
}

impl<C, S0: Clone, S1: Clone, T0, T1> Invertible for SurfaceCurve<C, S0, S1, T0, T1>
where
    C: Invertible,
    T0: Invertible,
    T1: Invertible,
{
    fn invert(&mut self) {
        self.leader.invert();
        if let Some(boundary) = self.boundary0_mut() {
            boundary.invert();
        }
        if let Some(boundary) = self.boundary1_mut() {
            boundary.invert();
        }
    }
}

impl<C, S0, S1, T0, T1> SearchParameter<CurveParameter> for SurfaceCurve<C, S0, S1, T0, T1>
where
    C: ParametricCurve3D + SearchNearestParameter<CurveParameter, Point = Point3>,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
    T0: Clone,
    T1: Clone,
{
    type Point = Point3;
    fn search_parameter<H: Into<SearchParameterHint1D>>(
        &self,
        point: Point3,
        hint: H,
        trials: usize,
    ) -> Option<f64> {
        let intersection = IntersectionCurve::new(
            self.surface0.clone(),
            self.surface1.clone(),
            self.leader.clone(),
        );
        intersection.search_parameter(point, hint, trials)
    }
}

impl<C, S0, S1, T0, T1> SearchNearestParameter<CurveParameter> for SurfaceCurve<C, S0, S1, T0, T1>
where
    C: ParametricCurve3D + SearchNearestParameter<CurveParameter, Point = Point3>,
    S0: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
    S1: ParametricSurface3D + SearchNearestParameter<SurfaceParameter, Point = Point3> + Clone,
    T0: Clone,
    T1: Clone,
{
    type Point = Point3;
    fn search_nearest_parameter<H: Into<SearchParameterHint1D>>(
        &self,
        point: Point3,
        hint: H,
        trials: usize,
    ) -> Option<f64> {
        let intersection = IntersectionCurve::new(
            self.surface0.clone(),
            self.surface1.clone(),
            self.leader.clone(),
        );
        intersection.search_nearest_parameter(point, hint, trials)
    }
}

impl<C, S0, S1, T0, T1> Transformed<Matrix4> for SurfaceCurve<C, S0, S1, T0, T1>
where
    C: Transformed<Matrix4>,
    S0: Transformed<Matrix4>,
    S1: Transformed<Matrix4>,
    T0: Transformed<Matrix4>,
    T1: Transformed<Matrix4>,
{
    fn transform_by(&mut self, trans: Matrix4) {
        self.surface0.transform_by(trans);
        self.surface1.transform_by(trans);
        self.leader.transform_by(trans);
        if let Some(boundary) = self.boundary0_mut() {
            boundary.transform_by(trans);
        }
        if let Some(boundary) = self.boundary1_mut() {
            boundary.transform_by(trans);
        }
    }
}

#[cfg(test)]
mod double_projection_tests;
