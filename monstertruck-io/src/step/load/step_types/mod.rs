//! STEP entity holder types, grouped by entity family.
//!
//! Every type is re-exported from here, so `step_types::Thing` -- and the
//! `pub use step_types::*` that the parent module applies to it -- resolves
//! exactly as it did when this was a single file. The parts are:
//!
//! - [`placement`] -- points, directions, vectors, axis placements.
//! - [`curve`] -- lines, conics, polylines, B-spline and NURBS curves.
//! - [`surface`] -- elementary, B-spline/NURBS and swept surfaces.
//! - [`topology`] -- vertices, edges, loops, faces, shells, solids.
//! - [`product`] -- contexts, products, shape representations, transformations.
//!
//! What stays in this root file is what more than one family needs: the generic
//! `representation` wrappers, the two knot-vector constructors shared by the
//! B-spline curve and B-spline surface families, and the `pcurve`/`surface_curve`
//! cluster, which is a curve expressed through surfaces and read field by field
//! by `edge_curve` over in [`topology`].

use monstertruck_geometry::prelude as geom;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::result::Result;
use step_p21::{Holder, ast::Name, primitive::Logical, tables::PlaceHolder};

use super::Table;
use super::step_geometry::{self, *};

mod curve;
mod placement;
mod product;
mod surface;
mod topology;

pub use curve::*;
pub use placement::*;
pub use product::*;
pub use surface::*;
pub use topology::*;

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

fn quasi_uniform_knots(num_ctrl: usize, degree: usize) -> KnotVector {
    let division = num_ctrl - degree;
    let mut knots = KnotVector::uniform_knot(degree, division);
    knots.transform(division as f64, 0.0);
    knots
}

fn uniform_knots(num_ctrl: usize, degree: usize) -> geom::Result<KnotVector> {
    KnotVector::try_from(
        (0..degree + num_ctrl + 1)
            .map(|i| i as f64 - degree as f64)
            .collect::<Vec<_>>(),
    )
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
mod tests;
