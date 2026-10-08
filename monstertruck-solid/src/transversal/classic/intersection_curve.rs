//! Classic (0.3.2) marching intersection-curve backend.
//!
//! Extracts the mesh-vs-mesh interference segments for a surface pair, chains
//! them into polylines, and re-samples each polyline onto both surfaces to
//! produce a cleaned [`IntersectionCurve`] leader. Ported verbatim from the
//! published 0.3.2 crate's `transversal::intersection_curve`, except that the
//! parameter-space polylines the 0.3.2 wrapper carried are dropped here: the
//! classic loops-store consumes only the 3D leader.

use monstertruck_core::cgmath64::*;
use monstertruck_geometry::prelude::*;
use monstertruck_meshing::prelude::*;

use crate::transversal::polyline_construction::construct_polylines;

type Polyline = PolylineCurve<Point3>;

/// Re-sample a raw interference polyline onto both surfaces, returning a
/// cleaned polyline intersection curve. Mirrors 0.3.2
/// `IntersectionCurveWithParameters::try_new`, keeping only the 3D leader.
fn build_intersection_curve<S>(
    surface0: S,
    surface1: S,
    poly: Polyline,
) -> Option<IntersectionCurve<Polyline, S, S>>
where
    S: ParametricSurface3D
        + Clone
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    let ic = IntersectionCurve::new(surface0.clone(), surface1.clone(), poly);
    let raw = ic.leader().clone();
    let len = raw.len();
    if len < 2 {
        return None;
    }
    let recover_triple = |point: Point3| {
        ic.search_nearest_point(point, None, None, 100).or_else(|| {
            let p0 = ic
                .surface0()
                .search_parameter(point, None, 100)
                .or_else(|| ic.surface0().search_nearest_parameter(point, None, 100))?;
            let p1 = ic
                .surface1()
                .search_parameter(point, None, 100)
                .or_else(|| ic.surface1().search_nearest_parameter(point, None, 100))?;
            let q0 = ic.surface0().evaluate(p0.0, p0.1);
            let q1 = ic.surface1().evaluate(p1.0, p1.1);
            Some((q0.midpoint(q1), p0.into(), p1.into()))
        })
    };
    let mut polyline = PolylineCurve(Vec::new());
    for i in 0..len - 1 {
        let (q, _, _) = ic
            .search_triple(i as f64, 100)
            .or_else(|| recover_triple(raw[i]))?;
        polyline.push(q);
    }
    let q = if raw[0].near(&raw[len - 1]) {
        polyline[0]
    } else {
        ic.search_triple((len - 1) as f64, 100)
            .or_else(|| recover_triple(raw[len - 1]))?
            .0
    };
    polyline.push(q);
    Some(IntersectionCurve::new(surface0, surface1, polyline))
}

fn without_backtracks(mut polyline: Polyline) -> Polyline {
    let mut i = 1;
    while i + 1 < polyline.len() {
        if polyline[i - 1].near(&polyline[i + 1]) {
            polyline.remove(i);
            polyline.remove(i);
            i = i.saturating_sub(1).max(1);
        } else {
            i += 1;
        }
    }
    polyline
}

type IntersectionTuple<S> = (Polyline, IntersectionCurve<Polyline, S, S>);

/// Marching intersection curves between two trimmed surfaces, keyed by their
/// tessellated polygon meshes. Returns, per intersection branch, the raw
/// interference polyline and the cleaned polyline intersection curve.
pub(super) fn intersection_curves<S>(
    surface0: S,
    polygon0: &PolygonMesh,
    surface1: S,
    polygon1: &PolygonMesh,
) -> Option<Vec<IntersectionTuple<S>>>
where
    S: ParametricSurface3D
        + Clone
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>,
{
    let interferences = polygon0.extract_interference(polygon1);
    construct_polylines(&interferences)
        .into_iter()
        .map(without_backtracks)
        .filter(|polyline| polyline.len() >= 2)
        .map(|polyline| {
            let curve =
                build_intersection_curve(surface0.clone(), surface1.clone(), polyline.clone())?;
            Some((polyline, curve))
        })
        .collect()
}
