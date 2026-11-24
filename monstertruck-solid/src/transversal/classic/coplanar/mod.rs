//! Booleans between faces that lie on one plane.
//!
//! Marching intersections finds nothing between coplanar faces, so before it runs,
//! each such face is divided along the outline of the faces it overlaps, and the
//! overlapping pieces are marked to be kept once or dropped.

use super::super::integrate::{ShapeOpsCurve, ShapeOpsSurface};
use monstertruck_geometry::prelude::*;
use monstertruck_topology::*;
use rustc_hash::FxHashMap as HashMap;

mod arrangement;
mod plane2d;
mod sew;
mod split;

use arrangement::*;
use plane2d::*;
pub(super) use sew::sew;
use split::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct PlaneOf {
    origin: Point3,
    normal: Vector3,
}

pub(super) fn plane_of<C, S: TryIntoAnalyticSurfaceKind + Clone>(
    face: &Face<Point3, C, S>,
) -> Option<PlaneOf> {
    match face.surface().try_into_analytic_surface_kind()? {
        AnalyticSurfaceKind::Plane(plane) => {
            let normal = plane.normal();
            Some(PlaneOf {
                origin: plane.origin(),
                normal: if face.orientation() { normal } else { -normal },
            })
        }
        _ => None,
    }
}

pub(super) fn coplanar(a: PlaneOf, b: PlaneOf, tol: f64) -> bool {
    a.normal.cross(b.normal).magnitude() < 1.0e-7 && (b.origin - a.origin).dot(a.normal).abs() < tol
}

pub(super) fn inner_point<C, S: ShapeOpsSurface>(
    face: &Face<Point3, C, S>,
    tol: f64,
) -> Option<Point3>
where
    C: Clone + BoundedCurve + ParameterDivision1D<Point = Point3>,
{
    let frame = Frame::new(plane_of(face)?);
    interior_point(&outline(face, &frame, tol), tol).map(|p| frame.lift(p))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Overlap {
    Same,
    Opposite,
}

pub(super) struct Imprinted<C, S> {
    pub(super) shell0: Shell<Point3, C, S>,
    pub(super) shell1: Shell<Point3, C, S>,
    pub(super) overlap0: HashMap<FaceId<S>, Overlap>,
    pub(super) overlap1: HashMap<FaceId<S>, Overlap>,
}

fn overlapping<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    a: &Face<Point3, C, S>,
    b: &Face<Point3, C, S>,
    frame: &Frame,
    tol: f64,
) -> bool {
    let (pa, pb) = (outline(a, frame, tol), outline(b, frame, tol));
    let strictly =
        |polygons: &[Vec<Point2>], p: Point2| inside(polygons, p) && !on_outline(polygons, p, tol);
    let probes = |polygons: &[Vec<Point2>]| -> Vec<Point2> {
        let mut points: Vec<Point2> = polygons.iter().flatten().copied().collect();
        for polygon in polygons {
            let n = polygon.len();
            for i in 0..n {
                let (a, b) = (polygon[i], polygon[(i + 1) % n]);
                let along = b - a;
                let length = along.magnitude();
                if length < tol {
                    continue;
                }
                let left = Vector2::new(-along.y, along.x) / length;
                for k in [0.25, 0.5, 0.75] {
                    for side in [1.0, -1.0] {
                        points.push(a + along * k + left * side * (length * 1.0e-2).max(tol * 4.0));
                    }
                }
            }
        }
        let corners: Vec<Point2> = polygons.iter().flatten().copied().collect();
        if let Some(first) = corners.first() {
            let (lo, hi) = corners.iter().fold((*first, *first), |(lo, hi), p| {
                (
                    Point2::new(lo.x.min(p.x), lo.y.min(p.y)),
                    Point2::new(hi.x.max(p.x), hi.y.max(p.y)),
                )
            });
            for i in 1..8 {
                for j in 1..8 {
                    points.push(Point2::new(
                        lo.x + (hi.x - lo.x) * i as f64 / 8.0,
                        lo.y + (hi.y - lo.y) * j as f64 / 8.0,
                    ));
                }
            }
        }
        points
            .into_iter()
            .filter(|p| inside(polygons, *p) || on_outline(polygons, *p, tol))
            .collect()
    };
    if probes(&pa)
        .into_iter()
        .any(|p| strictly(&pa, p) && strictly(&pb, p))
        || probes(&pb)
            .into_iter()
            .any(|p| strictly(&pb, p) && strictly(&pa, p))
    {
        return true;
    }
    let segments = |polygons: &[Vec<Point2>]| -> Vec<(Point2, Point2)> {
        polygons
            .iter()
            .flat_map(|poly| (0..poly.len()).map(move |i| (poly[i], poly[(i + 1) % poly.len()])))
            .collect()
    };
    let (sa, sb) = (segments(&pa), segments(&pb));
    sa.iter().any(|(a0, a1)| {
        sb.iter().any(|(b0, b1)| {
            crossing(*a0, *a1, *b0, *b1).is_some_and(|(s, t)| {
                s > 1.0e-6 && s < 1.0 - 1.0e-6 && t > 1.0e-6 && t < 1.0 - 1.0e-6
            })
        })
    })
}

pub(super) fn imprint<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    shell0: &Shell<Point3, C, S>,
    shell1: &Shell<Point3, C, S>,
    tol: f64,
) -> Option<Imprinted<C, S>> {
    let planes0: Vec<Option<PlaneOf>> = shell0.face_iter().map(plane_of).collect();
    let planes1: Vec<Option<PlaneOf>> = shell1.face_iter().map(plane_of).collect();
    let faces0: Vec<Face<Point3, C, S>> = shell0.face_iter().cloned().collect();
    let faces1: Vec<Face<Point3, C, S>> = shell1.face_iter().cloned().collect();
    let pairs: Vec<(usize, usize)> = (0..faces0.len())
        .flat_map(|i| (0..faces1.len()).map(move |j| (i, j)))
        .filter(|&(i, j)| match (planes0[i], planes1[j]) {
            (Some(a), Some(b)) if coplanar(a, b, tol) => {
                overlapping(&faces0[i], &faces1[j], &Frame::new(a), tol)
            }
            _ => false,
        })
        .collect();
    if pairs.is_empty() {
        return None;
    }
    let (edges0, edges1) = (unique_edges(shell0), unique_edges(shell1));
    let (splits0, splits1) = split_points(&edges0, &edges1, &pairs, &faces0, &faces1, tol);
    let split0 = apply_splits(shell0, &edges0, &splits0);
    let split1 = apply_splits(shell1, &edges1, &splits1);
    let faces0: Vec<Face<Point3, C, S>> = split0.face_iter().cloned().collect();
    let faces1: Vec<Face<Point3, C, S>> = split1.face_iter().cloned().collect();
    let mut overlap0 = HashMap::default();
    let mut overlap1 = HashMap::default();
    let rebuild = |faces: &[Face<Point3, C, S>],
                   others: &[Face<Point3, C, S>],
                   partners: &dyn Fn(usize) -> Vec<usize>,
                   planes: &[Option<PlaneOf>],
                   other_planes: &[Option<PlaneOf>],
                   overlaps: &mut HashMap<FaceId<S>, Overlap>|
     -> Option<Shell<Point3, C, S>> {
        let mut shell = Shell::new();
        for (i, face) in faces.iter().enumerate() {
            let mine = partners(i);
            if mine.is_empty() {
                shell.push(face.clone());
                continue;
            }
            let theirs: Vec<&Face<Point3, C, S>> = mine.iter().map(|&j| &others[j]).collect();
            for (piece, over) in divide(face, &theirs, tol)? {
                if let Some(k) = over {
                    let same = match (planes[i], other_planes[mine[k]]) {
                        (Some(a), Some(b)) => a.normal.dot(b.normal) > 0.0,
                        _ => return None,
                    };
                    overlaps.insert(
                        piece.id(),
                        if same {
                            Overlap::Same
                        } else {
                            Overlap::Opposite
                        },
                    );
                }
                shell.push(piece);
            }
        }
        Some(shell)
    };
    let partners0 = |i: usize| pairs.iter().filter(|p| p.0 == i).map(|p| p.1).collect();
    let partners1 = |j: usize| pairs.iter().filter(|p| p.1 == j).map(|p| p.0).collect();
    let shell0 = rebuild(
        &faces0,
        &faces1,
        &partners0,
        &planes0,
        &planes1,
        &mut overlap0,
    )?;
    let shell1 = rebuild(
        &faces1,
        &faces0,
        &partners1,
        &planes1,
        &planes0,
        &mut overlap1,
    )?;
    Some(Imprinted {
        shell0,
        shell1,
        overlap0,
        overlap1,
    })
}
