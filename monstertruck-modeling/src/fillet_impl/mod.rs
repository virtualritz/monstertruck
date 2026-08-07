use monstertruck_fillet::{FilletIntersectionCurve, ParameterCurveLinear};
use monstertruck_geometry::prelude::*;

use crate::{Curve, Curve2D, Surface};

impl TryFrom<Surface> for NurbsSurface<Vector4> {
    type Error = ();
    fn try_from(surface: Surface) -> std::result::Result<Self, ()> {
        match surface {
            Surface::Plane(plane) => Ok(NurbsSurface::from(BsplineSurface::from(plane))),
            Surface::BsplineSurface(bsp) => Ok(NurbsSurface::from(bsp)),
            Surface::NurbsSurface(ns) => Ok(ns),
            // The exact rational net -- the SAME one these faces WERE before
            // spec 012 U1.2 moved them onto the analytic variants, so the
            // fillet path sees no change.
            Surface::SphericalSurface(_) | Surface::ToroidalSurface(_) => surface
                .try_into_homogeneous_bspline_surface()
                .map(NurbsSurface::new)
                .ok_or(()),
            Surface::RevolutionSurface(_) | Surface::TsplineSurface(_) => Err(()),
        }
    }
}
// From<NurbsSurface<Vector4>> for Surface -- provided by derive_more::From

impl TryFrom<Curve> for NurbsCurve<Vector4> {
    type Error = ();
    // Exact or refuse: boolean seam edges are `IntersectionCurve`s whose
    // exact leader must survive conversion; a sampled polyline stand-in would
    // silently degrade downstream fillet/chamfer geometry. Parameter curves
    // and non-exact leader chains have no exact NURBS form, so they refuse
    // here and surface as `FilletError::UnsupportedGeometry`.
    fn try_from(curve: Curve) -> std::result::Result<Self, ()> {
        curve
            .try_into_homogeneous_bspline_curve()
            .map(NurbsCurve::new)
            .ok_or(())
    }
}
// From<NurbsCurve<Vector4>> for Curve -- provided by derive_more::From

impl From<ParameterCurveLinear> for Curve {
    fn from(c: ParameterCurveLinear) -> Self {
        let (line, surface) = c.decompose();
        Curve::ParameterCurve(ParameterCurve::new(
            Curve2D::Line(line),
            Box::new(Surface::NurbsSurface(surface)),
        ))
    }
}

impl From<FilletIntersectionCurve> for Curve {
    // Convert-back path only: these are intersection curves the fillet
    // algorithm itself creates between its generated NURBS strips (never
    // boolean seam input), and they have no exact closed form.
    fn from(c: FilletIntersectionCurve) -> Self {
        let range = c.range_tuple();
        Curve::NurbsCurve(sample_to_nurbs(range, |t| c.subs(t), 16))
    }
}

/// Sample a parametric curve into a degree-1 NURBS polyline approximation.
fn sample_to_nurbs(
    range: (f64, f64),
    subs: impl Fn(f64) -> Point3,
    n: usize,
) -> NurbsCurve<Vector4> {
    let (t0, t1) = range;
    let pts: Vec<Point3> = (0..=n)
        .map(|i| subs(t0 + (t1 - t0) * (i as f64) / (n as f64)))
        .collect();
    let knots: Vec<f64> = (0..=n).map(|i| i as f64 / n as f64).collect();
    let knot_vec = KnotVector::from(
        std::iter::once(0.0)
            .chain(knots.iter().copied())
            .chain(std::iter::once(1.0))
            .collect::<Vec<_>>(),
    );
    let bsp = BsplineCurve::new(knot_vec, pts);
    NurbsCurve::from(bsp)
}

#[cfg(test)]
mod tests;
