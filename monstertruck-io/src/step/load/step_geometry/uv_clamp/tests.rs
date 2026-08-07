//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;
use crate::step::load::step_geometry::{CylindricalSurface, Plane, re_exports::*};

/// The three placeholder axes the STEP loader actually emits, named.
#[test]
fn the_placeholder_axes_are_the_plane_and_a_loaded_cylinders_profile() {
    let plane = Surface::ElementarySurface(ElementarySurface::Plane(Plane::xy()));
    assert_eq!(
        reported_range_bounds_the_surface(&plane),
        (false, false),
        "a plane's [0,1]^2 is a placeholder on BOTH axes.",
    );

    // Exactly the shape `From<&CylindricalSurface>` builds: a revolution of
    // a UNIT-direction line, then inverted.
    let center = Point3::new(0.0, 0.0, 68.0);
    let axis = Vector3::unit_z();
    let p = center + 0.3 * Vector3::unit_x();
    let mut cylinder: CylindricalSurface = Processor::new(RevolutionSurface::by_revolution(
        Line(p, p + axis),
        center,
        axis,
    ));
    cylinder.invert();
    let surface = Surface::ElementarySurface(ElementarySurface::CylindricalSurface(cylinder));
    assert_eq!(
        reported_range_bounds_the_surface(&surface),
        (true, false),
        "on a loaded (inverted) cylinder u is the real turn and v the \
         placeholder axial metre.",
    );

    // The un-inverted orientation swaps the pair -- pinned so a change in
    // `Processor::parameter_range` cannot silently invert the meaning.
    let upright: CylindricalSurface = Processor::new(RevolutionSurface::by_revolution(
        Line(p, p + axis),
        center,
        axis,
    ));
    assert_eq!(
        reported_range_bounds_the_surface(&Surface::ElementarySurface(
            ElementarySurface::CylindricalSurface(upright)
        )),
        (false, true),
    );
}

/// The closed analytic surfaces report REAL ranges, and must keep them:
/// dropping one would disable a clamp that has a boundary to clamp to.
#[test]
fn sphere_and_torus_ranges_are_measurements_not_placeholders() {
    let sphere = Surface::ElementarySurface(ElementarySurface::Sphere(Processor::new(
        crate::step::load::step_geometry::Sphere(monstertruck_geometry::prelude::Sphere::new(
            Point3::origin(),
            2.0,
        )),
    )));
    assert_eq!(reported_range_bounds_the_surface(&sphere), (true, true));

    let torus = Surface::ElementarySurface(ElementarySurface::ToroidalSurface(Processor::new(
        Torus::new(Point3::origin(), 3.0, 1.0),
    )));
    assert_eq!(reported_range_bounds_the_surface(&torus), (true, true));
}
