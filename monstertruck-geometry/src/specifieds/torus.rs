use super::*;
use std::f64::consts::PI;

impl Torus {
    /// constructor
    #[inline(always)]
    pub fn new(center: Point3, large_radius: f64, small_radius: f64) -> Self {
        if large_radius <= 0.0 || small_radius <= 0.0 {
            panic!("radius must be larger than 0");
        }
        Self {
            center,
            large_radius,
            small_radius,
        }
    }

    /// get center
    #[inline(always)]
    pub const fn center(&self) -> Point3 { self.center }

    /// get large radius
    #[inline(always)]
    pub const fn large_radius(&self) -> f64 { self.large_radius }

    /// get small radius
    #[inline(always)]
    pub const fn small_radius(&self) -> f64 { self.small_radius }
}

impl ParametricSurface for Torus {
    type Point = Point3;
    type Vector = Vector3;
    #[inline(always)]
    fn derivative_mn(&self, m: usize, n: usize, u: f64, v: f64) -> Self::Vector {
        let ((su, cu), (sv, cv)) = (u.sin_cos(), v.sin_cos());
        let center = match (m, n) {
            (0, 0) => self.center.to_vec(),
            _ => Vector3::zero(),
        };
        let u_z = if m == 0 { 1.0 } else { 0.0 };
        let u_part = match m % 4 {
            0 => Vector3::new(cu, su, u_z),
            1 => Vector3::new(-su, cu, 0.0),
            2 => Vector3::new(-cu, -su, 0.0),
            _ => Vector3::new(su, -cu, 0.0),
        };
        let r0 = if n == 0 { self.large_radius } else { 0.0 };
        let r1 = self.small_radius;
        let v_part_d2 = match n % 4 {
            0 => Vector2::new(r0 + r1 * cv, r1 * sv),
            1 => Vector2::new(-r1 * sv, r1 * cv),
            2 => Vector2::new(-r1 * cv, -r1 * sv),
            _ => Vector2::new(r1 * sv, -r1 * cv),
        };
        let v_part = Vector3::new(v_part_d2.x, v_part_d2.x, v_part_d2.y);
        center + u_part.mul_element_wise(v_part)
    }
    #[inline(always)]
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        let sr = self.small_radius() * Vector2::new(f64::cos(v), f64::sin(v));
        let lr = (self.large_radius() + sr.x) * Vector2::new(f64::cos(u), f64::sin(u));
        self.center() + Vector3::new(lr.x, lr.y, sr.y)
    }
    #[inline(always)]
    fn derivative_u(&self, u: f64, v: f64) -> Vector3 {
        let sr = self.small_radius() * f64::cos(v);
        let lr = (self.large_radius() + sr) * Vector2::new(f64::cos(u), f64::sin(u));
        Vector3::new(-lr.y, lr.x, 0.0)
    }
    #[inline(always)]
    fn derivative_v(&self, u: f64, v: f64) -> Vector3 {
        let sv = self.small_radius() * Vector2::new(-f64::sin(v), f64::cos(v));
        Vector3::new(sv.x * f64::cos(u), sv.x * f64::sin(u), sv.y)
    }
    #[inline(always)]
    fn derivative_uu(&self, u: f64, v: f64) -> Vector3 {
        let sr = self.small_radius() * f64::cos(v);
        let lr = (self.large_radius() + sr) * Vector2::new(f64::cos(u), f64::sin(u));
        Vector3::new(-lr.x, -lr.y, 0.0)
    }
    #[inline(always)]
    fn derivative_uv(&self, u: f64, v: f64) -> Vector3 {
        let sr = -self.small_radius() * f64::sin(v);
        let lr = sr * Vector2::new(f64::cos(u), f64::sin(u));
        Vector3::new(-lr.y, lr.x, 0.0)
    }
    #[inline(always)]
    fn derivative_vv(&self, u: f64, v: f64) -> Vector3 {
        let sv = -self.small_radius() * Vector2::new(f64::cos(v), f64::sin(v));
        Vector3::new(sv.x * f64::cos(u), sv.x * f64::sin(u), sv.y)
    }
    #[inline(always)]
    fn parameter_range(&self) -> (ParameterRange, ParameterRange) {
        const RANGE: (Bound<f64>, Bound<f64>) = (Bound::Included(0.0), Bound::Excluded(2.0 * PI));
        (RANGE, RANGE)
    }
    #[inline(always)]
    fn period_u(&self) -> Option<f64> { Some(2.0 * PI) }
    #[inline(always)]
    fn period_v(&self) -> Option<f64> { Some(2.0 * PI) }
}

impl ParametricSurface3D for Torus {
    #[inline(always)]
    fn normal(&self, u: f64, v: f64) -> Vector3 {
        let sv = Vector2::new(f64::cos(v), f64::sin(v));
        Vector3::new(sv.x * f64::cos(u), sv.x * f64::sin(u), sv.y)
    }
    #[inline(always)]
    fn normal_uder(&self, u: f64, v: f64) -> Vector3 {
        let sv = Vector2::new(f64::cos(v), f64::sin(v));
        Vector3::new(-sv.x * f64::sin(u), sv.x * f64::cos(u), sv.y)
    }
    #[inline(always)]
    fn normal_vder(&self, u: f64, v: f64) -> Vector3 {
        let sv = Vector2::new(-f64::sin(v), f64::cos(v));
        Vector3::new(sv.x * f64::cos(u), sv.x * f64::sin(u), sv.y)
    }
}

impl BoundedSurface for Torus {}

// -- v2 scalar-generic impls ------------------------------------------------

use monstertruck_traits::v2;

impl v2::ParametricSurface for Torus {
    type Scalar = f64;
    type Point = Point3;
    type Vector = Vector3;

    #[inline(always)]
    fn evaluate(&self, u: f64, v: f64) -> Point3 { ParametricSurface::evaluate(self, u, v) }
    #[inline(always)]
    fn derivative_u(&self, u: f64, v: f64) -> Vector3 {
        ParametricSurface::derivative_u(self, u, v)
    }
    #[inline(always)]
    fn derivative_v(&self, u: f64, v: f64) -> Vector3 {
        ParametricSurface::derivative_v(self, u, v)
    }
    #[inline(always)]
    fn derivative_uu(&self, u: f64, v: f64) -> Vector3 {
        ParametricSurface::derivative_uu(self, u, v)
    }
    #[inline(always)]
    fn derivative_uv(&self, u: f64, v: f64) -> Vector3 {
        ParametricSurface::derivative_uv(self, u, v)
    }
    #[inline(always)]
    fn derivative_vv(&self, u: f64, v: f64) -> Vector3 {
        ParametricSurface::derivative_vv(self, u, v)
    }
    #[inline(always)]
    fn period_u(&self) -> Option<f64> { ParametricSurface::period_u(self) }
    #[inline(always)]
    fn period_v(&self) -> Option<f64> { ParametricSurface::period_v(self) }
}

impl v2::BoundedSurface for Torus {
    #[inline(always)]
    fn range_tuple(&self) -> ((f64, f64), (f64, f64)) { BoundedSurface::range_tuple(self) }
}

impl v2::ParametricSurface3D for Torus {}

impl v2::SearchNearestParameter<v2::D2<f64>> for Torus {
    type Point = Point3;
    #[inline(always)]
    fn search_nearest_parameter<H: Into<v2::SearchParameterHint2D<f64>>>(
        &self,
        pt: Point3,
        _: H,
        trials: usize,
    ) -> Option<(f64, f64)> {
        SearchNearestParameter::<D2>::search_nearest_parameter(self, pt, None, trials)
    }
}

impl v2::SearchParameter<v2::D2<f64>> for Torus {
    type Point = Point3;
    #[inline(always)]
    fn search_parameter<H: Into<v2::SearchParameterHint2D<f64>>>(
        &self,
        pt: Point3,
        _: H,
        trials: usize,
    ) -> Option<(f64, f64)> {
        SearchParameter::<D2>::search_parameter(self, pt, None, trials)
    }
}

/// The `(u, v)` a [`SearchParameterHint2D`] carries, or `None`.
///
/// `Range` answers `None`: it names a rectangle, not a point, and the seam
/// disambiguation below wants an anchor. Every caller that needs the
/// disambiguation (the boundary projection chain, the SSI's parameter refiner)
/// passes `Parameter`.
#[inline]
fn hint_parameter(hint: SearchParameterHint2D) -> Option<(f64, f64)> {
    match hint {
        SearchParameterHint2D::Parameter(u, v) => Some((u, v)),
        SearchParameterHint2D::Range(..) | SearchParameterHint2D::None => None,
    }
}

/// The co-periodic representative of `angle` nearest `hint` -- `angle + 2 k PI`
/// for the integer `k` that minimises the distance -- or `angle` unchanged when
/// no hint is available.
///
/// # The defect this closes, measured (spec 012 W1)
///
/// A torus is exactly `2 * PI`-periodic in both parameters, so every angle has
/// infinitely many correct spellings and this routine's branch arithmetic picks
/// one by a rule that reads only the CURRENT point: `2 * PI - acos(x)` is
/// exactly `2 * PI` when the point sits on the `u = 0` seam with a negative-zero
/// `y`, and exactly `0` when the same point carries a positive-zero `y`.
/// Ordinary for a seam vertex of a real STEP face, and the caller was never
/// given a way to say which spelling it wanted -- the `hint` argument was
/// literally discarded (`_: H`).
///
/// That is invisible until a torus reaches a caller that PROJECTS a boundary.
/// `sampled_parameter_boundary` (`monstertruck-modeling/src/geometry/`) walks
/// a face's boundary polyline through this routine, feeding each answer forward
/// as the next hint, so one seam vertex spelled a whole period away from its
/// neighbours leaves the face's parameter LOOP with a spike across the entire
/// domain. Measured on ap224 with spec 012's analytic-torus routing on -- two
/// `R = 0.9107, r = 0.0312` fillet tori, seam-touching from OPPOSITE sides:
///
/// | face | true `u` extent (from the rational net) | analytic answer, before |
/// |---|---|---|
/// | 15 | `(0, PI)` | `(0.0645, 6.2832)` -- the whole ring |
/// | 19 | `(PI, 2 PI)` | `(0, 6.2187)` -- the whole ring |
///
/// The SSI then traced the RIGHT curves against the box cutter's planes and the
/// trim filter discarded every one of them (`side0 = 0` on all 8 segments),
/// which surfaced as `CreateLoopsStoreFailed{IntersectionCurvesFailed{(15,4),
/// SsiFailed}}`. **So this is ledger class C4, not a missing analytic SSI path:
/// the kernel intersected the analytic torus perfectly well and was handed a
/// face whose own trim loop excluded its interior.**
///
/// **A fixed rule cannot work here, and that is a measurement, not a guess.**
/// The first attempt at this reduced an exact `2 * PI` to `0` -- correct for
/// face 15, and it moved the refusal straight onto face 19, whose seam vertex
/// needs the `2 * PI` spelling. The two faces are the same shape on opposite
/// sides of one seam, so only the CALLER's context distinguishes them, and the
/// hint is that context.
///
/// **Unhinted callers are byte-identical.** `SearchParameterHint2D::None`
/// returns `angle` untouched, so every call site that passes no hint -- which is
/// every call site that existed before spec 012 routed tori here -- sees exactly
/// the arithmetic it saw before.
#[inline]
fn nearest_periodic_angle(angle: f64, hint: Option<f64>) -> f64 {
    match hint {
        None => angle,
        Some(hint) => angle + 2.0 * PI * ((hint - angle) / (2.0 * PI)).round(),
    }
}

impl SearchParameter<SurfaceParameter> for Torus {
    type Point = Point3;
    fn search_parameter<H: Into<SearchParameterHint2D>>(
        &self,
        point: Point3,
        hint: H,
        _: usize,
    ) -> Option<(f64, f64)> {
        let hint = hint_parameter(hint.into());
        let r = point - self.center();
        let rxy = Vector2::new(r.x, r.y);
        let v = f64::asin(f64::clamp(r.z / self.small_radius(), -1.0, 1.0));
        let minus = rxy.magnitude2() < self.large_radius() * self.large_radius();
        let v = match (minus, v < 0.0) {
            (true, _) => PI - v,
            (false, false) => v,
            (false, true) => 2.0 * PI + v,
        };
        let v = nearest_periodic_angle(v, hint.map(|hint| hint.1));
        let rxy_n = rxy.normalize();
        let u = f64::acos(f64::clamp(rxy_n.x, -1.0, 1.0));
        let u = match rxy_n.y < 0.0 {
            true => 2.0 * PI - u,
            false => u,
        };
        let u = nearest_periodic_angle(u, hint.map(|hint| hint.0));
        // Still the SAME acceptance test on the SAME surface: `subs` is
        // `2 * PI`-periodic, so a re-spelled parameter evaluates to the very
        // same point and this check is unweakened by the re-spelling.
        match self.subs(u, v).near(&point) {
            true => Some((u, v)),
            false => None,
        }
    }
}

impl SearchNearestParameter<SurfaceParameter> for Torus {
    type Point = Point3;
    fn search_nearest_parameter<H: Into<SearchParameterHint2D>>(
        &self,
        point: Point3,
        hint: H,
        _: usize,
    ) -> Option<(f64, f64)> {
        let hint = hint_parameter(hint.into());
        let r = point - self.center();
        let rxy = Vector2::new(r.x, r.y);
        if rxy.so_small() {
            return None;
        }
        let rxy_n = rxy.normalize();
        let large_r = self.large_radius() * rxy_n.extend(0.0);
        let diff = r - large_r;
        if diff.so_small() {
            return None;
        }
        let small_r = diff.normalize();

        let u = f64::acos(f64::clamp(rxy_n.x, -1.0, 1.0));
        let u = match rxy_n.y < 0.0 {
            true => 2.0 * PI - u,
            false => u,
        };
        // The same seam disambiguation as the exact solver above -- see
        // [`nearest_periodic_angle`]. The two are twins over the same branch
        // arithmetic and `project_onto_surface_domain` ALTERNATES between them
        // (attempts 0/2 exact, 1/3 nearest), so teaching only one would leave
        // the defect reachable on every other attempt.
        let u = nearest_periodic_angle(u, hint.map(|hint| hint.0));
        let v = f64::asin(f64::clamp(small_r.z, -1.0, 1.0));
        let v = match (small_r.dot(large_r) < 0.0, v < 0.0) {
            (true, _) => PI - v,
            (false, false) => v,
            (false, true) => 2.0 * PI + v,
        };
        let v = nearest_periodic_angle(v, hint.map(|hint| hint.1));
        Some((u, v))
    }
}

impl ParameterDivision2D for Torus {
    fn parameter_division(
        &self,
        (urange, vrange): ((f64, f64), (f64, f64)),
        tol: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let circle = UnitCircle::<Point2>::new();
        let utol = tol / (self.small_radius() + self.large_radius());
        let (udiv, _) = circle.parameter_division(urange, utol);
        let vtol = tol / self.small_radius();
        let (vdiv, _) = circle.parameter_division(vrange, vtol);
        (udiv, vdiv)
    }
}
