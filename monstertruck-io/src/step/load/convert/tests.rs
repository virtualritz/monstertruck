//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;
use crate::step::load::step_geometry::SurfaceCurveRepresentation as StepSurfaceCurveRepresentation;
use std::f64::consts::TAU;

/// Loads `occt-cylinder.step` and builds its trimmed shell. The
/// cylinder fixture carries `PCURVE` entities on both planar and
/// cylindrical surfaces, so this exercises the `ToSameGeometry<Curve2D>`
/// load path end-to-end -- the unit tests in `step_geometry/geom_impls`
/// only verify the impls in isolation. At least one of the resulting
/// trim curves must be present (`Some(_)`) and at least one of them
/// must contain a 2D curve variant that comes through the conversion.
#[test]
fn pcurve_load_path_populates_trim_curves() -> anyhow::Result<()> {
    let step_string = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../resources/step/occt-cylinder.step",
    ));
    let table = crate::step::load::Table::from_step(step_string)?;
    let step_shell = table
        .shell
        .values()
        .next()
        .ok_or_else(|| anyhow::anyhow!("the cylinder fixture must contain a STEP shell."))?;
    let trimmed = table.to_compressed_trimmed_shell(step_shell)?;
    let total_edge_uses: usize = trimmed
        .faces
        .iter()
        .flat_map(|face| &face.boundaries)
        .map(|wire| wire.len())
        .sum();
    let trim_curves_present: usize = trimmed
        .faces
        .iter()
        .flat_map(|face| &face.boundaries)
        .flat_map(|wire| wire.iter())
        .filter(|edge_use| edge_use.trim_curve.is_some())
        .count();
    assert!(
        total_edge_uses > 0,
        "the cylinder fixture should have at least one edge-use after trimmed loading.",
    );
    assert!(
        trim_curves_present > 0,
        "at least one edge-use should carry a trim curve. \
         total edge-uses: {total_edge_uses}.",
    );
    Ok(())
}

fn cylinder_surface() -> Surface {
    let axis = Vector3::unit_z();
    let center = Point3::origin();
    let point = Point3::new(1.0, 0.0, 0.0);
    let line = Line(point, point + axis);
    Surface::ElementarySurface(ElementarySurface::CylindricalSurface(Processor::new(
        RevolutionSurface::by_revolution(line, center, axis),
    )))
}

fn line_pcurve(surface: &Surface, u: f64) -> step_geometry::StepParameterCurve {
    step_geometry::StepParameterCurve::new(
        Box::new(Curve2D::Line(Line(
            Point2::new(u, 0.0),
            Point2::new(u, 1.0),
        ))),
        Box::new(surface.clone()),
    )
}

fn seam_curve(surface: &Surface) -> Curve3D {
    let leader = Curve3D::Line(Line(surface.subs(0.0, 0.0), surface.subs(0.0, 1.0)));
    Curve3D::SurfaceCurve(SurfaceCurve3D::new(
        StepSurfaceCurveKind::SeamCurve,
        Box::new(leader),
        vec![
            SurfaceCurveAssociatedGeometry::ParameterCurve(line_pcurve(surface, 0.0)),
            SurfaceCurveAssociatedGeometry::ParameterCurve(line_pcurve(surface, TAU)),
        ],
        StepSurfaceCurveRepresentation::ParameterCurve1,
    ))
}

#[test]
fn seam_curve_opposite_orientations_use_opposite_parameter_curves() {
    let surface = cylinder_surface();
    let curve = seam_curve(&surface);

    let forward = Table::exact_trim_curve_on(&curve, &surface, true)
        .expect("forward trim curve should exist");
    let backward = Table::exact_trim_curve_on(&curve, &surface, false)
        .expect("backward trim curve should exist");

    let forward_start = forward.curve().subs(forward.curve().range_tuple().0);
    let backward_start = backward.curve().subs(backward.curve().range_tuple().0);

    assert!(forward_start.x.near(&0.0));
    assert!(backward_start.x.near(&TAU));
}
