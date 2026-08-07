//! Surface entities: elementary surfaces, B-spline/NURBS surfaces and
//! swept surfaces.

use super::*;

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
