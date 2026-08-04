use monstertruck_geometry::prelude::*;
use monstertruck_modeling::{Curve as ModelingCurve, Surface as ModelingSurface};
use monstertruck_step::load::{CylindricalSurfaceHolder, Table, step_geometry, step_p21};
use monstertruck_step::save::*;
use std::f64::consts::TAU;
use step_p21::tables::EntityTable;

/// Builds a modeling-layer right circular cylinder as a surface of revolution
/// of a straight profile line parallel to the revolution axis.
fn modeling_cylinder(
    origin: Point3,
    axis: Vector3,
    radius: f64,
    half_height: f64,
) -> ModelingSurface {
    let profile = ModelingCurve::Line(Line(
        Point3::new(radius, 0.0, -half_height),
        Point3::new(radius, 0.0, half_height),
    ));
    ModelingSurface::RevolutionSurface(Processor::new(RevolutionSurface::by_revolution(
        profile, origin, axis,
    )))
}

#[test]
fn modeling_cylinder_saves_as_cylindrical_surface() {
    let origin = Point3::origin();
    let axis = Vector3::unit_z();
    let radius = 3.0;
    let surface = modeling_cylinder(origin, axis, radius, 2.0);

    let text =
        CompleteStepDisplay::new(StepDisplay::new(&surface, 1), Default::default()).to_string();

    // The analytic entity must appear verbatim, not a rational B-spline degrade.
    assert!(
        text.contains("CYLINDRICAL_SURFACE"),
        "expected CYLINDRICAL_SURFACE in emitted STEP, got:\n{text}"
    );
    assert!(
        !text.contains("B_SPLINE_SURFACE"),
        "cylinder must stay analytic, not degrade to a B-spline surface:\n{text}"
    );

    // Reload and confirm the analytic cylinder survives.
    let table = Table::from_step(&text).unwrap();
    let owned = EntityTable::<CylindricalSurfaceHolder>::get_owned(&table, 1).unwrap();
    let reloaded: step_geometry::CylindricalSurface = (&owned).into();

    // Every surface point lies on the revolution axis at the recovered radius.
    for i in 0..8 {
        let v = TAU * i as f64 / 8.0;
        for &u in &[0.0, 0.5, 1.0] {
            let point = reloaded.subs(u, v);
            let offset = point - origin;
            let radial = offset - axis * offset.dot(axis);
            assert!(
                (radial.magnitude() - radius).abs() < 1.0e-6,
                "reloaded cylinder point off-radius at (u = {u}, v = {v}): {}",
                radial.magnitude()
            );
        }
    }
}
