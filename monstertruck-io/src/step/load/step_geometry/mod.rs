use crate::step::save;
use derive_more::From;
use monstertruck_derive::{StepCurve, StepFormat, StepLength, StepSurface};
use serde::{Deserialize, Serialize};

/// re-export structs in `monstertruck-geometry` and `monstertruck-mesh`.
pub mod re_exports {
    pub use monstertruck_geometry::prelude::*;
    pub use monstertruck_mesh::*;
}
pub use re_exports::*;

/// Errors that occur when converting STEP format
pub type StepConvertingError = Box<dyn std::error::Error + Send + Sync + 'static>;

/// `ellipse`, realized in `monstertruck`
pub type Ellipse<P, M> = Processor<TrimmedCurve<UnitCircle<P>>, M>;
/// `hyperbola`, realized in `monstertruck`
pub type Hyperbola<P, M> = Processor<TrimmedCurve<UnitHyperbola<P>>, M>;
/// `parabola`, realized in `monstertruck`
pub type Parabola<P, M> = Processor<TrimmedCurve<UnitParabola<P>>, M>;
/// `spherical_surface`, realized in `monstertruck`
pub type SphericalSurface = Processor<Sphere, Matrix4>;
/// `cylindrical_surface`, realized in `monstertruck`
pub type CylindricalSurface = Processor<RevolutionSurface<Line<Point3>>, Matrix4>;
/// `toroidal_surface`, realized in `monstertruck`
pub type ToroidalSurface = Processor<Torus, Matrix4>;
/// `conical_surface`, realized in `monstertruck`
pub type ConicalSurface = Processor<RevolutionSurface<Line<Point3>>, Matrix4>;
/// `surface_of_linear_extrusion`, realized in `monstertruck`
pub type StepExtrusionSurface = ExtrusionSurface<Curve3D, Vector3>;
/// `surface_of_revolution`, realized in `monstertruck`
pub type StepRevolutionSurface = Processor<RevolutionSurface<Curve3D>, Matrix4>;
/// STEP parameter curve on a surface, realized in `monstertruck`.
pub type StepParameterCurve =
    monstertruck_geometry::prelude::ParameterCurve<Box<Curve2D>, Box<Surface>>;

/// STEP `surface_curve` trim lookup on a specific surface.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceCurveTrimRef<'a> {
    curve: &'a SurfaceCurve3D,
    surface: &'a Surface,
}

impl<'a> SurfaceCurveTrimRef<'a> {
    /// Creates a new trim lookup reference.
    pub const fn new(curve: &'a SurfaceCurve3D, surface: &'a Surface) -> Self {
        Self { curve, surface }
    }

    /// Returns the referenced STEP `surface_curve`.
    pub const fn curve(self) -> &'a SurfaceCurve3D { self.curve }

    /// Returns the target surface.
    pub const fn surface(self) -> &'a Surface { self.surface }
}

/// STEP curve trim lookup on a specific surface.
#[derive(Clone, Copy, Debug)]
pub struct CurveTrimRef<'a> {
    curve: &'a Curve3D,
    surface: &'a Surface,
}

impl<'a> CurveTrimRef<'a> {
    /// Creates a new trim lookup reference.
    pub const fn new(curve: &'a Curve3D, surface: &'a Surface) -> Self { Self { curve, surface } }

    /// Returns the referenced curve.
    pub const fn curve(self) -> &'a Curve3D { self.curve }

    /// Returns the target surface.
    pub const fn surface(self) -> &'a Surface { self.surface }
}

/// Preferred master representation for a STEP `surface_curve`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SurfaceCurveRepresentation {
    Curve3D,
    ParameterCurve0,
    ParameterCurve1,
}

/// STEP `surface_curve` flavor.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SurfaceCurveKind {
    SurfaceCurve,
    SeamCurve,
    IntersectionCurve,
}

/// Associated geometry entry of a STEP `surface_curve`.
#[derive(Clone, Debug, PartialEq, From, Serialize, Deserialize)]
pub enum SurfaceCurveAssociatedGeometry {
    ParameterCurve(StepParameterCurve),
    Surface(Box<Surface>),
}

/// STEP `surface_curve` with preserved associated trim geometry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SurfaceCurve3D {
    kind: SurfaceCurveKind,
    leader: Box<Curve3D>,
    associated_geometry: Vec<SurfaceCurveAssociatedGeometry>,
    master_representation: SurfaceCurveRepresentation,
}

impl SurfaceCurve3D {
    fn parameter_curve_matches_leader_on_surface(
        &self,
        curve: &StepParameterCurve,
        surface: &Surface,
    ) -> bool {
        let (curve_min, curve_max) = curve.curve().range_tuple();
        let (leader_min, leader_max) = self.leader().range_tuple();
        let tolerance2 = (100.0 * TOLERANCE).powi(2).max(1.0e-12);
        [0.0, 0.5, 1.0].into_iter().all(|fraction| {
            let curve_t = curve_min + (curve_max - curve_min) * fraction;
            let leader_t = leader_min + (leader_max - leader_min) * fraction;
            let uv = curve.curve().subs(curve_t);
            let surface_point = surface.subs(uv.x, uv.y);
            let leader_point = self.leader().subs(leader_t);
            surface_point.distance2(leader_point) <= tolerance2
        })
    }

    pub(crate) fn same_surface(lhs: &Surface, rhs: &Surface) -> bool {
        if lhs == rhs {
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
                    let lp = lhs.subs(lu0 + (lu1 - lu0) * s, lv0 + (lv1 - lv0) * t);
                    let rp = rhs.subs(ru0 + (ru1 - ru0) * s, rv0 + (rv1 - rv0) * t);
                    lp.near(&rp)
                })
        } else {
            false
        }
    }

    /// Creates a new STEP `surface_curve`.
    pub fn new(
        kind: SurfaceCurveKind,
        leader: Box<Curve3D>,
        associated_geometry: Vec<SurfaceCurveAssociatedGeometry>,
        master_representation: SurfaceCurveRepresentation,
    ) -> Self {
        Self {
            kind,
            leader,
            associated_geometry,
            master_representation,
        }
    }

    /// Returns the STEP `surface_curve` flavor.
    pub fn kind(&self) -> SurfaceCurveKind { self.kind }

    /// Returns the master representation.
    pub fn master_representation(&self) -> SurfaceCurveRepresentation { self.master_representation }

    /// Returns the 3D leader curve.
    pub fn leader(&self) -> &Curve3D { self.leader.as_ref() }

    /// Returns the mutable 3D leader curve.
    pub fn leader_mut(&mut self) -> &mut Curve3D { self.leader.as_mut() }

    /// Returns the associated geometries.
    pub fn associated_geometry(&self) -> &[SurfaceCurveAssociatedGeometry] {
        &self.associated_geometry
    }

    /// Returns the first associated `ParameterCurve` whose basis surface matches `surface`.
    pub fn parameter_curve_on(&self, surface: &Surface) -> Option<&StepParameterCurve> {
        self.associated_geometry
            .iter()
            .filter_map(|entry| match entry {
                SurfaceCurveAssociatedGeometry::ParameterCurve(curve) => Some(curve),
                _ => None,
            })
            .find(|curve| {
                (curve.surface().as_ref() == surface
                    || Self::same_surface(curve.surface().as_ref(), surface))
                    && self.parameter_curve_matches_leader_on_surface(curve, surface)
            })
    }
}

/// Renamed to [`StepExtrusionSurface`].
#[deprecated(note = "renamed to StepExtrusionSurface")]
pub type StepExtrudedCurve = StepExtrusionSurface;
/// Renamed to [`StepRevolutionSurface`].
#[deprecated(note = "renamed to StepRevolutionSurface")]
pub type StepRevolutedCurve = StepRevolutionSurface;

/// `conic` in 2D, realized in `monstertruck`
#[derive(
    Clone,
    Copy,
    Debug,
    PartialEq,
    From,
    Serialize,
    Deserialize,
    ParametricCurve,
    BoundedCurve,
    Cut,
    Invertible,
    ParameterDivision1D,
    SearchParameterD1,
    SearchNearestParameterD1,
    TransformedM3,
    SelfSameGeometry,
    StepLength,
    StepFormat,
    StepCurve,
)]
pub enum Conic2D {
    Ellipse(Ellipse<Point2, Matrix3>),
    Hyperbola(Hyperbola<Point2, Matrix3>),
    Parabola(Parabola<Point2, Matrix3>),
}

/// `curve` in 2D, realized in `monstertruck`
#[derive(
    Clone,
    Debug,
    PartialEq,
    From,
    Serialize,
    Deserialize,
    ParametricCurve,
    BoundedCurve,
    Cut,
    Invertible,
    ParameterDivision1D,
    SearchParameterD1,
    SearchNearestParameterD1,
    TransformedM3,
    SelfSameGeometry,
    StepLength,
    StepFormat,
    StepCurve,
)]

pub enum Curve2D {
    Line(Line<Point2>),
    Polyline(PolylineCurve<Point2>),
    Conic(Conic2D),
    BsplineCurve(BsplineCurve<Point2>),
    NurbsCurve(NurbsCurve<Vector3>),
}

/// `conic` in 3D, realized in `monstertruck`
#[derive(
    Clone,
    Copy,
    Debug,
    PartialEq,
    From,
    Serialize,
    Deserialize,
    ParametricCurve,
    BoundedCurve,
    Cut,
    Invertible,
    ParameterDivision1D,
    SearchParameterD1,
    SearchNearestParameterD1,
    TransformedM4,
    SelfSameGeometry,
    StepLength,
    StepFormat,
    StepCurve,
)]
pub enum Conic3D {
    Ellipse(Ellipse<Point3, Matrix4>),
    Hyperbola(Hyperbola<Point3, Matrix4>),
    Parabola(Parabola<Point3, Matrix4>),
}

/// `curve` in 3D, realized in `monstertruck`
#[derive(
    Clone,
    Debug,
    PartialEq,
    From,
    Serialize,
    Deserialize,
    ParametricCurve,
    BoundedCurve,
    Cut,
    Invertible,
    SearchParameterD1,
    SearchNearestParameterD1,
    TransformedM4,
    SelfSameGeometry,
    StepLength,
    StepFormat,
    StepCurve,
)]
pub enum Curve3D {
    Line(Line<Point3>),
    Polyline(PolylineCurve<Point3>),
    Conic(Conic3D),
    BsplineCurve(BsplineCurve<Point3>),
    ParameterCurve(StepParameterCurve),
    SurfaceCurve(SurfaceCurve3D),
    IntersectionCurve(IntersectionCurve<Box<Curve3D>, Box<Surface>, Box<Surface>>),
    NurbsCurve(NurbsCurve<Vector4>),
}

/// `elementary_surface`, realized in `monstertruck`
#[derive(
    Clone,
    Copy,
    Debug,
    PartialEq,
    Serialize,
    Deserialize,
    ParametricSurface3D,
    ParameterDivision2D,
    SearchParameterD2,
    SearchNearestParameterD2,
    Invertible,
    TransformedM4,
    SelfSameGeometry,
    StepLength,
    StepSurface,
)]
pub enum ElementarySurface {
    Plane(Plane),
    Sphere(SphericalSurface),
    CylindricalSurface(CylindricalSurface),
    ToroidalSurface(ToroidalSurface),
    ConicalSurface(ConicalSurface),
}

/// `swept_surface`, realized in `monstertruck`
#[derive(
    Clone,
    Debug,
    From,
    PartialEq,
    Serialize,
    Deserialize,
    ParametricSurface3D,
    ParameterDivision2D,
    SearchParameterD2,
    SearchNearestParameterD2,
    Invertible,
    TransformedM4,
    SelfSameGeometry,
    StepLength,
    StepFormat,
    StepSurface,
)]
pub enum SweepSurface {
    ExtrusionSurface(StepExtrusionSurface),
    RevolutionSurface(StepRevolutionSurface),
}

/// `surface`, realized in `monstertruck`
#[derive(
    Clone,
    Debug,
    From,
    PartialEq,
    Serialize,
    Deserialize,
    ParametricSurface3D,
    ParameterDivision2D,
    SearchParameterD2,
    SearchNearestParameterD2,
    Invertible,
    TransformedM4,
    SelfSameGeometry,
    StepLength,
    StepSurface,
)]
pub enum Surface {
    ElementarySurface(ElementarySurface),
    SweepSurface(SweepSurface),
    BsplineSurface(BsplineSurface<Point3>),
    NurbsSurface(NurbsSurface<Vector4>),
}

impl save::StepFormat for Surface {
    fn fmt(&self, idx: usize, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use Surface::*;
        match self {
            ElementarySurface(x) => x.fmt(idx, f),
            SweepSurface(x) => x.fmt(idx, f),
            BsplineSurface(x) => x.fmt(idx, f),
            NurbsSurface(x) => x.fmt(idx, f),
        }
    }
}

/// `spherical_surface`, realized in `monstertruck`
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, StepSurface)]
pub struct Sphere(pub monstertruck_geometry::prelude::Sphere);

impl save::StepSurface for Processor<Sphere, Matrix4> {
    #[inline(always)]
    fn same_sense(&self) -> bool { self.orientation() }
}

mod sphere;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn surface_curve_rejects_invalid_pcurve_on_identical_surface() {
        let surface = Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy()));
        let leader = Curve3D::Line(Line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)));
        let invalid_trim = ParameterCurve::new(
            Box::new(Curve2D::Line(Line(
                Point2::new(0.0, 1.0),
                Point2::new(1.0, 1.0),
            ))),
            Box::new(surface.clone()),
        );
        let surface_curve = SurfaceCurve3D::new(
            SurfaceCurveKind::SurfaceCurve,
            Box::new(leader),
            vec![SurfaceCurveAssociatedGeometry::ParameterCurve(invalid_trim)],
            SurfaceCurveRepresentation::Curve3D,
        );

        assert!(
            surface_curve.parameter_curve_on(&surface).is_none(),
            "Invalid face-local pcurves must not be accepted only because their surface entity matches."
        );
    }

    #[test]
    fn public_parameter_curve_api_uses_descriptive_names() {
        let surface = Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy()));
        let trim: StepParameterCurve = ParameterCurve::new(
            Box::new(Curve2D::Line(Line(
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
            ))),
            Box::new(surface),
        );
        let curve = Curve3D::ParameterCurve(trim);

        assert!(matches!(curve, Curve3D::ParameterCurve(_)));
    }
}

/// Implementation required to apply a closed surface division to a shape parsed from a STEP file.
mod from_pcurve {
    use super::{Curve2D, Curve3D, Surface};
    use monstertruck_geometry::prelude::*;

    impl From<ParameterCurve<Line<Point2>, Surface>> for Curve3D {
        fn from(value: ParameterCurve<Line<Point2>, Surface>) -> Self {
            let (line, surface) = value.decompose();
            Curve3D::ParameterCurve(ParameterCurve::new(
                Curve2D::Line(line).into(),
                surface.into(),
            ))
        }
    }
}

/// implementation for trait `monstertruck_modeling::builder`.
mod geom_impls;
pub use geom_impls::ROUTE_ANALYTIC_SPHERE;
/// implementation for output STEP format.
mod stepout_impls;
/// Provenance of a surface's reported parameter range, and the
/// `MT_STEP_DEBUG_UV_CLAMP` lens over `normalize_uv`'s two writing arms.
mod uv_clamp;
/// Spec 012 U2 measurement harness for [`uv_clamp`]. Test-only.
#[cfg(test)]
mod uv_clamp_probe;
