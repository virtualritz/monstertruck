use monstertruck_core::tolerance::Tolerance;
use monstertruck_geometry::prelude::*;
use monstertruck_traits::{BoundedCurve, ParametricCurve, ParametricSurface};
use std::cell::RefCell;

use super::error::FilletError;
use super::types::{self, Curve, ParameterCurveLinear};

type InternalShell = types::Shell;
const SURFACE_SAMPLE_COUNT: usize = 64;

/// Intersection curve type used internally by fillet operations.
pub type FilletIntersectionCurve =
    IntersectionCurve<ParameterCurveLinear, Box<NurbsSurface<Vector4>>, Box<NurbsSurface<Vector4>>>;

/// Surface types that can participate in fillet operations.
///
/// Automatically implemented for any type satisfying the bounds.
pub trait FilletableSurface:
    Clone
    + ParametricSurface<Point = Point3>
    + TryInto<NurbsSurface<Vector4>>
    + From<NurbsSurface<Vector4>> {
    /// Converts this surface to a NURBS surface used by internal fillet logic.
    fn to_nurbs_surface(&self) -> Option<NurbsSurface<Vector4>> {
        self.clone()
            .try_into()
            .ok()
            .or_else(|| sample_surface_to_nurbs(self, SURFACE_SAMPLE_COUNT))
    }
}

impl<T> FilletableSurface for T where T: Clone
        + ParametricSurface<Point = Point3>
        + TryInto<NurbsSurface<Vector4>>
        + From<NurbsSurface<Vector4>>
{
}

/// Curve types that can participate in fillet operations.
///
/// Automatically implemented for any type satisfying the bounds.
pub trait FilletableCurve:
    Clone
    + ParametricCurve<Point = Point3>
    + BoundedCurve
    + TryInto<NurbsCurve<Vector4>>
    + From<NurbsCurve<Vector4>>
    + From<ParameterCurveLinear>
    + From<FilletIntersectionCurve> {
    /// Converts this curve to the exact NURBS curve used by internal fillet
    /// logic, or `None` when the curve has no exact NURBS representation.
    ///
    /// Refusal surfaces as [`FilletError::UnsupportedGeometry`] from
    /// [`fillet_edges_generic`](super::fillet_edges_generic); it must never
    /// be papered over with a sampled approximation, or exact boolean seam
    /// leaders would silently degrade into polylines.
    fn to_nurbs_curve(&self) -> Option<NurbsCurve<Vector4>> { self.clone().try_into().ok() }
}

impl<T> FilletableCurve for T where T: Clone
        + ParametricCurve<Point = Point3>
        + BoundedCurve
        + TryInto<NurbsCurve<Vector4>>
        + From<NurbsCurve<Vector4>>
        + From<ParameterCurveLinear>
        + From<FilletIntersectionCurve>
{
}

fn sample_surface_to_nurbs<S: ParametricSurface<Point = Point3>>(
    surface: &S,
    sample_count: usize,
) -> Option<NurbsSurface<Vector4>> {
    let (u_range, v_range) = surface.try_range_tuple();
    let ((u0, u1), (v0, v1)) = u_range.zip(v_range)?;
    let control_points: Vec<Vec<Point3>> = (0..=sample_count)
        .map(|iu| {
            let u = u0 + (u1 - u0) * (iu as f64) / (sample_count as f64);
            (0..=sample_count)
                .map(|iv| {
                    let v = v0 + (v1 - v0) * (iv as f64) / (sample_count as f64);
                    surface.evaluate(u, v)
                })
                .collect()
        })
        .collect();
    let u_knot = KnotVector::uniform_knot(1, sample_count);
    let v_knot = KnotVector::uniform_knot(1, sample_count);
    Some(NurbsSurface::from(BsplineSurface::new(
        (u_knot, v_knot),
        control_points,
    )))
}

// TryFrom for extracting NurbsCurve from internal Curve type.
impl TryFrom<Curve> for NurbsCurve<Vector4> {
    type Error = ();
    fn try_from(curve: Curve) -> std::result::Result<Self, ()> {
        match curve {
            Curve::NurbsCurve(c) => Ok(c),
            _ => Err(()),
        }
    }
}

/// The curves and surfaces of a shell before conversion, each beside its NURBS form, so
/// [`convert_shell_out`] can give back the original wherever the fillet left it untouched.
pub(super) struct Originals<C, S> {
    curves: Vec<(NurbsCurve<Vector4>, C)>,
    surfaces: Vec<(NurbsSurface<Vector4>, S)>,
}

/// The internal shell, the internal ids of the selected edges, and the originals.
pub(super) type ConvertedShell<C, S> = (InternalShell, Vec<types::EdgeId>, Originals<C, S>);

/// Convert an external shell to internal fillet types.
///
/// Returns the internal shell, the internal `EdgeId`s corresponding to
/// the selected external edges (matched by endpoint positions), and the
/// [`Originals`] to restore on the way out.
pub(super) fn convert_shell_in<C: FilletableCurve, S: FilletableSurface>(
    shell: &monstertruck_topology::Shell<Point3, C, S>,
    edges: &[monstertruck_topology::Edge<Point3, C>],
) -> std::result::Result<ConvertedShell<C, S>, FilletError> {
    // Collect endpoint pairs for requested edges (front, back).
    let edge_endpoints: Vec<(Point3, Point3)> = edges
        .iter()
        .map(|e| (e.absolute_front().point(), e.absolute_back().point()))
        .collect();

    let originals = RefCell::new(Originals {
        curves: Vec::new(),
        surfaces: Vec::new(),
    });
    let internal_shell: InternalShell = shell
        .try_mapped(
            |p| Some(*p),
            |c| {
                let nurbs = c.to_nurbs_curve()?;
                originals
                    .borrow_mut()
                    .curves
                    .push((nurbs.clone(), c.clone()));
                Some(Curve::NurbsCurve(nurbs))
            },
            |s| {
                let nurbs = s.to_nurbs_surface()?;
                originals
                    .borrow_mut()
                    .surfaces
                    .push((nurbs.clone(), s.clone()));
                Some(nurbs)
            },
        )
        .ok_or(FilletError::UnsupportedGeometry {
            context: "failed to convert shell curves or surfaces to NURBS",
        })?;

    // Match external edges to internal edges by endpoint positions.
    let internal_edge_ids: Vec<types::EdgeId> = edge_endpoints
        .iter()
        .map(|(ext_front, ext_back)| {
            internal_shell
                .edge_iter()
                .find(|ie| {
                    let f = ie.absolute_front().point();
                    let b = ie.absolute_back().point();
                    (f.near(ext_front) && b.near(ext_back))
                        || (f.near(ext_back) && b.near(ext_front))
                })
                .map(|ie| ie.id())
                .ok_or(FilletError::EdgeNotFound)
        })
        .collect::<std::result::Result<Vec<_>, _>>()?;

    Ok((internal_shell, internal_edge_ids, originals.into_inner()))
}

/// Convert an internal fillet shell back to external types.
pub(super) fn convert_shell_out<C: FilletableCurve, S: FilletableSurface>(
    shell: &InternalShell,
    originals: &Originals<C, S>,
) -> std::result::Result<monstertruck_topology::Shell<Point3, C, S>, FilletError> {
    shell
        .try_mapped(
            |p| Some(*p),
            |c| {
                Some(match c {
                    Curve::NurbsCurve(nc) => originals
                        .curves
                        .iter()
                        .find(|(nurbs, _)| nurbs == nc)
                        .map(|(_, original)| original.clone())
                        .unwrap_or_else(|| C::from(nc.clone())),
                    Curve::ParameterCurve(pc) => C::from(pc.clone()),
                    Curve::IntersectionCurve(ic) => C::from(ic.clone()),
                })
            },
            |s| {
                Some(
                    originals
                        .surfaces
                        .iter()
                        .find(|(nurbs, _)| nurbs == s)
                        .map(|(_, original)| original.clone())
                        .unwrap_or_else(|| S::from(s.clone())),
                )
            },
        )
        .ok_or(FilletError::UnsupportedGeometry {
            context: "failed to convert internal shell back to external types",
        })
}
