//! Curve behaviour on the STEP curve enums: evaluation, division, cutting,
//! inversion, endpoint snapping, parameter search and transformation.

use super::*;

impl ParametricCurve for SurfaceCurve3D {
    type Point = Point3;
    type Vector = Vector3;

    fn evaluate(&self, t: f64) -> Self::Point { self.leader().evaluate(t) }

    fn derivative(&self, t: f64) -> Self::Vector { self.leader().derivative(t) }

    fn derivative_2(&self, t: f64) -> Self::Vector { self.leader().derivative_2(t) }

    fn derivative_n(&self, n: usize, t: f64) -> Self::Vector { self.leader().derivative_n(n, t) }

    fn parameter_range(&self) -> ParameterRange { self.leader().parameter_range() }

    fn period(&self) -> Option<f64> { self.leader().period() }
}

impl BoundedCurve for SurfaceCurve3D {}

impl ParameterDivision1D for SurfaceCurve3D {
    type Point = Point3;

    fn try_parameter_division(
        &self,
        range: (f64, f64),
        tol: f64,
    ) -> Option<(Vec<f64>, Vec<Self::Point>)> {
        self.leader().try_parameter_division(range, tol)
    }

    fn parameter_division(&self, range: (f64, f64), tol: f64) -> (Vec<f64>, Vec<Self::Point>) {
        self.leader().parameter_division(range, tol)
    }
}

impl Cut for SurfaceCurve3D {
    fn cut(&mut self, t: f64) -> Self {
        let leader = Box::new(self.leader_mut().cut(t));
        let associated_geometry = self
            .associated_geometry
            .iter_mut()
            .map(|entry| entry.split_at(t))
            .collect();
        Self::new(
            self.kind(),
            leader,
            associated_geometry,
            self.master_representation(),
        )
    }
}

impl SnapCurveEndpoints for SurfaceCurve3D {
    fn snap_endpoints(&mut self, front: Point3, back: Point3) {
        self.leader_mut().snap_endpoints(front, back);
    }
}

impl SnapCurveEndpoints for Curve3D {
    fn snap_endpoints(&mut self, front: Point3, back: Point3) {
        match self {
            Curve3D::Polyline(curve) => curve.snap_endpoints(front, back),
            Curve3D::SurfaceCurve(curve) => curve.snap_endpoints(front, back),
            Curve3D::IntersectionCurve(curve) => curve.snap_endpoints(front, back),
            Curve3D::Line(_)
            | Curve3D::Conic(_)
            | Curve3D::BsplineCurve(_)
            | Curve3D::ParameterCurve(_)
            | Curve3D::NurbsCurve(_) => {}
        }
    }
}

impl Invertible for SurfaceCurveAssociatedGeometry {
    fn invert(&mut self) {
        if let SurfaceCurveAssociatedGeometry::ParameterCurve(curve) = self {
            curve.invert();
        }
    }
}

impl Invertible for SurfaceCurve3D {
    fn invert(&mut self) {
        self.leader_mut().invert();
        self.associated_geometry
            .iter_mut()
            .for_each(Invertible::invert);
    }
}

impl SearchParameter<CurveParameter> for SurfaceCurve3D {
    type Point = Point3;

    fn search_parameter<H: Into<SearchParameterHint1D>>(
        &self,
        point: Self::Point,
        hint: H,
        trials: usize,
    ) -> Option<f64> {
        self.leader().search_parameter(point, hint, trials)
    }
}

impl SearchNearestParameter<CurveParameter> for SurfaceCurve3D {
    type Point = Point3;

    fn search_nearest_parameter<H: Into<SearchParameterHint1D>>(
        &self,
        point: Self::Point,
        hint: H,
        trials: usize,
    ) -> Option<f64> {
        self.leader().search_nearest_parameter(point, hint, trials)
    }
}

impl Transformed<Matrix4> for SurfaceCurveAssociatedGeometry {
    fn transform_by(&mut self, trans: Matrix4) {
        match self {
            SurfaceCurveAssociatedGeometry::ParameterCurve(curve) => curve.transform_by(trans),
            SurfaceCurveAssociatedGeometry::Surface(surface) => surface.transform_by(trans),
        }
    }
}

impl Transformed<Matrix4> for SurfaceCurve3D {
    fn transform_by(&mut self, trans: Matrix4) {
        self.leader_mut().transform_by(trans);
        self.associated_geometry
            .iter_mut()
            .for_each(|entry| entry.transform_by(trans));
    }
}

impl ParameterDivision1D for Curve3D {
    type Point = Point3;

    fn try_parameter_division(
        &self,
        range: (f64, f64),
        tol: f64,
    ) -> Option<(Vec<f64>, Vec<Self::Point>)> {
        let debug_profile = env::var("MT_PROFILE_CURVE_DIVISION").is_ok();
        // Only consult the clock when actually profiling -- `Instant::now()`
        // panics on `wasm32-unknown-unknown` ("time not implemented"), so
        // an unconditional call here breaks browser STEP loading.
        let started = debug_profile.then(Instant::now);
        let result = match self {
            Curve3D::Line(curve) => curve.try_parameter_division(range, tol),
            Curve3D::Polyline(curve) => curve.try_parameter_division(range, tol),
            Curve3D::Conic(curve) => curve.try_parameter_division(range, tol),
            Curve3D::BsplineCurve(curve) => curve.try_parameter_division(range, tol),
            Curve3D::ParameterCurve(curve) => curve.try_parameter_division(range, tol),
            Curve3D::SurfaceCurve(curve) => curve.try_parameter_division(range, tol),
            Curve3D::IntersectionCurve(curve) => curve.leader().try_parameter_division(range, tol),
            Curve3D::NurbsCurve(curve) => curve.try_parameter_division(range, tol),
        };
        if let Some(started) = started {
            let kind = match self {
                Curve3D::Line(_) => "Line",
                Curve3D::Polyline(_) => "Polyline",
                Curve3D::Conic(_) => "Conic",
                Curve3D::BsplineCurve(_) => "BsplineCurve",
                Curve3D::ParameterCurve(_) => "StepParameterCurve",
                Curve3D::SurfaceCurve(_) => "SurfaceCurve",
                Curve3D::IntersectionCurve(_) => "IntersectionCurve",
                Curve3D::NurbsCurve(_) => "NurbsCurve",
            };
            eprintln!(
                "trace bool curve_division kind={} points={} tol={} elapsed_ms={:.3}",
                kind,
                result.as_ref().map_or(0, |(_, points)| points.len()),
                tol,
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
        result
    }

    fn parameter_division(&self, range: (f64, f64), tol: f64) -> (Vec<f64>, Vec<Self::Point>) {
        let debug_profile = env::var("MT_PROFILE_CURVE_DIVISION").is_ok();
        // Same wasm-safety guard as `try_parameter_division` above.
        let started = debug_profile.then(Instant::now);
        let result = match self {
            Curve3D::Line(curve) => curve.parameter_division(range, tol),
            Curve3D::Polyline(curve) => curve.parameter_division(range, tol),
            Curve3D::Conic(curve) => curve.parameter_division(range, tol),
            Curve3D::BsplineCurve(curve) => curve.parameter_division(range, tol),
            Curve3D::ParameterCurve(curve) => curve.parameter_division(range, tol),
            Curve3D::SurfaceCurve(curve) => curve.parameter_division(range, tol),
            Curve3D::IntersectionCurve(curve) => curve.leader().parameter_division(range, tol),
            Curve3D::NurbsCurve(curve) => curve.parameter_division(range, tol),
        };
        if let Some(started) = started {
            let kind = match self {
                Curve3D::Line(_) => "Line",
                Curve3D::Polyline(_) => "Polyline",
                Curve3D::Conic(_) => "Conic",
                Curve3D::BsplineCurve(_) => "BsplineCurve",
                Curve3D::ParameterCurve(_) => "StepParameterCurve",
                Curve3D::SurfaceCurve(_) => "SurfaceCurve",
                Curve3D::IntersectionCurve(_) => "IntersectionCurve",
                Curve3D::NurbsCurve(_) => "NurbsCurve",
            };
            eprintln!(
                "trace bool curve_division kind={} points={} tol={} elapsed_ms={:.3}",
                kind,
                result.1.len(),
                tol,
                started.elapsed().as_secs_f64() * 1000.0,
            );
        }
        result
    }
}

impl ToSameGeometry<Curve3D> for SurfaceCurve3D {
    fn to_same_geometry(&self) -> Curve3D { Curve3D::SurfaceCurve(self.clone()) }
}

impl From<IntersectionCurve<BsplineCurve<Point3>, Surface, Surface>> for Curve3D {
    fn from(ic: IntersectionCurve<BsplineCurve<Point3>, Surface, Surface>) -> Self {
        let (surface0, surface1, leader) = ic.destruct();
        Curve3D::IntersectionCurve(IntersectionCurve::new(
            Box::new(surface0),
            Box::new(surface1),
            Box::new(Curve3D::BsplineCurve(leader)),
        ))
    }
}
