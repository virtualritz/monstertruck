//! Splitting edges where the outlines of two coplanar faces cross.

use super::super::super::integrate::{ShapeOpsCurve, ShapeOpsSurface};
use monstertruck_geometry::prelude::*;
use monstertruck_topology::*;
use rustc_hash::FxHashMap as HashMap;
use std::collections::HashSet;

use super::plane_of;
use super::plane2d::*;

/// An edge index, its samples with their parameters, and the samples on the plane.
pub(super) type Sampled = (usize, Vec<(f64, Point3)>, Vec<Point2>);

pub(super) type Splits = HashMap<usize, Vec<(f64, Point3)>>;

pub(super) fn unique_edges<C, S>(shell: &Shell<Point3, C, S>) -> Vec<Edge<Point3, C>> {
    let mut seen = HashSet::new();
    shell
        .face_iter()
        .flat_map(|face| {
            face.absolute_boundaries()
                .iter()
                .flat_map(|w| w.iter().cloned())
                .collect::<Vec<_>>()
        })
        .filter(|edge| seen.insert(edge.id()))
        .map(|edge| edge.absolute_clone())
        .collect()
}

pub(super) fn note_split<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    splits: &mut Splits,
    index: usize,
    edge: &Edge<Point3, C>,
    point: Point3,
    tol: f64,
) {
    let curve = edge.curve();
    let (t0, t1) = curve.range_tuple();
    let (front, back) = (edge.absolute_front().point(), edge.absolute_back().point());
    if point.distance(front) < tol || point.distance(back) < tol {
        return;
    }
    let Some(t) = curve.search_nearest_parameter(point, None, 100) else {
        return;
    };
    if t <= t0 || t >= t1 || curve.subs(t).distance(point) > tol {
        return;
    }
    let list = splits.entry(index).or_default();
    if !list.iter().any(|(_, p)| p.distance(point) < tol) {
        list.push((t, curve.subs(t)));
    }
}

pub(super) fn split_points<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    edges0: &[Edge<Point3, C>],
    edges1: &[Edge<Point3, C>],
    pairs: &[(usize, usize)],
    faces0: &[Face<Point3, C, S>],
    faces1: &[Face<Point3, C, S>],
    tol: f64,
) -> (Splits, Splits) {
    let index_of = |edges: &[Edge<Point3, C>], edge: &Edge<Point3, C>| {
        edges.iter().position(|e| e.id() == edge.id())
    };
    let (mut splits0, mut splits1) = (Splits::default(), Splits::default());
    for &(i, j) in pairs {
        let plane = match plane_of(&faces0[i]) {
            Some(plane) => plane,
            None => continue,
        };
        let frame = Frame::new(plane);
        let own = |face: &Face<Point3, C, S>, edges: &[Edge<Point3, C>]| -> Vec<Sampled> {
            face.edge_iter()
                .filter_map(|edge| {
                    let index = index_of(edges, &edge)?;
                    let absolute = &edges[index];
                    let points = samples(&absolute.curve(), tol);
                    let flat = points.iter().map(|(_, p)| frame.flat(*p)).collect();
                    Some((index, points, flat))
                })
                .collect()
        };
        let a = own(&faces0[i], edges0);
        let b = own(&faces1[j], edges1);
        for (ia, _, fa) in &a {
            for (ib, _, fb) in &b {
                for sa in fa.windows(2) {
                    for sb in fb.windows(2) {
                        if let Some((s, _)) = crossing(sa[0], sa[1], sb[0], sb[1]) {
                            let at = frame.lift(sa[0] + (sa[1] - sa[0]) * s);
                            note_split(&mut splits0, *ia, &edges0[*ia], at, tol);
                            note_split(&mut splits1, *ib, &edges1[*ib], at, tol);
                        }
                    }
                }
            }
        }
        let corners = |edges: &[Edge<Point3, C>], list: &[Sampled]| -> Vec<Point3> {
            list.iter()
                .flat_map(|(index, _, _)| {
                    [
                        edges[*index].absolute_front().point(),
                        edges[*index].absolute_back().point(),
                    ]
                })
                .collect()
        };
        for corner in corners(edges1, &b) {
            let flat = frame.flat(corner);
            for (ia, _, fa) in &a {
                if fa
                    .windows(2)
                    .any(|s| segment_distance(flat, s[0], s[1]) < tol)
                {
                    note_split(&mut splits0, *ia, &edges0[*ia], corner, tol);
                }
            }
        }
        for corner in corners(edges0, &a) {
            let flat = frame.flat(corner);
            for (ib, _, fb) in &b {
                if fb
                    .windows(2)
                    .any(|s| segment_distance(flat, s[0], s[1]) < tol)
                {
                    note_split(&mut splits1, *ib, &edges1[*ib], corner, tol);
                }
            }
        }
    }
    (splits0, splits1)
}

pub(super) fn apply_splits<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    shell: &Shell<Point3, C, S>,
    edges: &[Edge<Point3, C>],
    splits: &Splits,
) -> Shell<Point3, C, S> {
    let mut pieces: HashMap<EdgeId<C>, Vec<Edge<Point3, C>>> = HashMap::default();
    for (index, edge) in edges.iter().enumerate() {
        let Some(list) = splits.get(&index) else {
            continue;
        };
        let mut list = list.clone();
        list.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut rest = edge.curve();
        let mut from = edge.absolute_front().clone();
        let mut made = Vec::new();
        for (_, point) in list {
            let Some(t) = rest.search_nearest_parameter(point, None, 100) else {
                continue;
            };
            let (t0, t1) = rest.range_tuple();
            if t <= t0 || t >= t1 {
                continue;
            }
            let tail = rest.cut(t);
            let vertex = Vertex::new(point);
            made.push(Edge::new(&from, &vertex, rest));
            rest = tail;
            from = vertex;
        }
        made.push(Edge::new(&from, edge.absolute_back(), rest));
        pieces.insert(edge.id(), made);
    }
    if pieces.is_empty() {
        return shell.clone();
    }
    shell
        .face_iter()
        .map(|face| {
            let wires: Vec<Wire<Point3, C>> = face
                .absolute_boundaries()
                .iter()
                .map(|wire| {
                    wire.iter()
                        .flat_map(|edge| match pieces.get(&edge.id()) {
                            None => vec![edge.clone()],
                            Some(made) if edge.orientation() => made.clone(),
                            Some(made) => made.iter().rev().map(Edge::inverse).collect(),
                        })
                        .collect()
                })
                .collect();
            let mut rebuilt = Face::new_unchecked(wires, face.surface());
            if !face.orientation() {
                rebuilt.invert();
            }
            rebuilt
        })
        .collect()
}
