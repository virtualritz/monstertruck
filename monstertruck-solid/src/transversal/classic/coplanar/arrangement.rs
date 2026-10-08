//! Dividing a face along the edges of the faces that overlap it on its plane.

use super::super::super::integrate::{ShapeOpsCurve, ShapeOpsSurface};
use monstertruck_geometry::prelude::*;
use monstertruck_topology::*;
use std::collections::HashSet;
use std::iter::once;

use super::plane_of;
use super::plane2d::*;

pub(super) struct HalfEdge<C> {
    edge: Edge<Point3, C>,
    from: usize,
    to: usize,
    leave: f64,
    arrive: f64,
    flat: Vec<Point2>,
}

pub(super) fn same_place(a: &[Point3], b: &[Point3], tol: f64) -> bool {
    let (a0, a1) = (a[0], a[a.len() - 1]);
    let (b0, b1) = (b[0], b[b.len() - 1]);
    let ends = (a0.distance(b0) < tol && a1.distance(b1) < tol)
        || (a0.distance(b1) < tol && a1.distance(b0) < tol);
    ends && a.iter().all(|p| {
        b.windows(2).any(|s| {
            let ab = s[1] - s[0];
            let t = ((*p - s[0]).dot(ab) / ab.magnitude2().max(1.0e-300)).clamp(0.0, 1.0);
            (s[0] + ab * t).distance(*p) < tol * 10.0
        })
    })
}

/// The pieces of a face cut by the faces overlapping it, each with the index of the
/// overlapping face it lies under, if any.
pub(super) type Pieces<C, S> = Vec<(Face<Point3, C, S>, Option<usize>)>;

pub(super) fn divide<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    face: &Face<Point3, C, S>,
    others: &[&Face<Point3, C, S>],
    tol: f64,
) -> Option<Pieces<C, S>> {
    let plane = plane_of(face)?;
    let frame = Frame::new(plane);
    let own_outline = outline(face, &frame, tol);
    let mut vertices: Vec<Vertex<Point3>> = Vec::new();
    fn vertex_of(
        vertices: &mut Vec<Vertex<Point3>>,
        point: Point3,
        existing: Option<&Vertex<Point3>>,
        tol: f64,
    ) -> usize {
        if let Some(found) = vertices
            .iter()
            .position(|v| v.point().distance(point) < tol)
        {
            return found;
        }
        vertices.push(existing.cloned().unwrap_or_else(|| Vertex::new(point)));
        vertices.len() - 1
    }
    let direction = |a: Point2, b: Point2| (b.y - a.y).atan2(b.x - a.x);
    let mut halves: Vec<HalfEdge<C>> = Vec::new();
    let mut boundary_points: Vec<Vec<Point3>> = Vec::new();
    for wire in face.boundaries() {
        for edge in wire.edge_iter() {
            let points = oriented_points(edge, tol);
            let from = vertex_of(&mut vertices, edge.front().point(), Some(edge.front()), tol);
            let to = vertex_of(&mut vertices, edge.back().point(), Some(edge.back()), tol);
            let flat: Vec<Point2> = points.iter().map(|p| frame.flat(*p)).collect();
            let n = flat.len();
            halves.push(HalfEdge {
                edge: edge.clone(),
                from,
                to,
                leave: direction(flat[0], flat[1]),
                arrive: direction(flat[n - 1], flat[n - 2]),
                flat,
            });
            boundary_points.push(points);
        }
    }
    let mut seen = HashSet::new();
    for other in others {
        for edge in other.edge_iter() {
            if !seen.insert(edge.id()) {
                continue;
            }
            let curve = edge.curve();
            let points: Vec<Point3> = samples(&curve, tol).into_iter().map(|(_, p)| p).collect();
            if boundary_points.iter().any(|b| same_place(b, &points, tol)) {
                continue;
            }
            let flat: Vec<Point2> = points.iter().map(|p| frame.flat(*p)).collect();
            let n = flat.len();
            let middle = if n > 2 {
                flat[n / 2]
            } else {
                flat[0] + (flat[1] - flat[0]) * 0.5
            };
            if on_outline(&own_outline, middle, tol) || !inside(&own_outline, middle) {
                continue;
            }
            let (front, back) = (edge.absolute_front().point(), edge.absolute_back().point());
            let from = vertex_of(&mut vertices, front, None, tol);
            let to = vertex_of(&mut vertices, back, None, tol);
            let made = Edge::new(&vertices[from], &vertices[to], curve);
            halves.push(HalfEdge {
                edge: made.clone(),
                from,
                to,
                leave: direction(flat[0], flat[1]),
                arrive: direction(flat[n - 1], flat[n - 2]),
                flat: flat.clone(),
            });
            let reversed: Vec<Point2> = flat.iter().rev().copied().collect();
            halves.push(HalfEdge {
                edge: made.inverse(),
                from: to,
                to: from,
                leave: direction(reversed[0], reversed[1]),
                arrive: direction(reversed[n - 1], reversed[n - 2]),
                flat: reversed,
            });
        }
    }
    let count = halves.len();
    let mut used = vec![false; count];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for start in 0..count {
        if used[start] {
            continue;
        }
        let mut cycle = vec![start];
        used[start] = true;
        let mut here = start;
        loop {
            let back = halves[here].arrive;
            let next = (0..count)
                .filter(|&k| halves[k].from == halves[here].to)
                .min_by(|&a, &b| {
                    let turn = |k: usize| {
                        let twin = halves[k].edge.id() == halves[here].edge.id();
                        if twin {
                            return std::f64::consts::TAU;
                        }
                        let mut t = back - halves[k].leave;
                        while t <= 1.0e-12 {
                            t += std::f64::consts::TAU;
                        }
                        while t > std::f64::consts::TAU {
                            t -= std::f64::consts::TAU;
                        }
                        t
                    };
                    turn(a).total_cmp(&turn(b))
                })?;
            if next == start {
                break;
            }
            if used[next] || cycle.len() > count {
                return None;
            }
            used[next] = true;
            cycle.push(next);
            here = next;
        }
        cycles.push(cycle);
    }
    let polygon = |cycle: &[usize]| -> Vec<Point2> {
        cycle
            .iter()
            .flat_map(|&k| {
                let flat = &halves[k].flat;
                flat[..flat.len() - 1].to_vec()
            })
            .collect()
    };
    let shapes: Vec<(Vec<usize>, Vec<Point2>, f64)> = cycles
        .into_iter()
        .map(|cycle| {
            let points = polygon(&cycle);
            let area = polygon_area(&points);
            (cycle, points, area)
        })
        .collect();
    let outers: Vec<usize> = (0..shapes.len())
        .filter(|&k| shapes[k].2 > tol * tol)
        .collect();
    let mut holes_of: Vec<Vec<usize>> = vec![Vec::new(); shapes.len()];
    for hole in (0..shapes.len()).filter(|&k| shapes[k].2 < -tol * tol) {
        let hole_points = &shapes[hole].1;
        let n = hole_points.len();
        let Some((a, b)) = (0..n)
            .map(|i| (hole_points[i], hole_points[(i + 1) % n]))
            .max_by(|x, y| (x.1 - x.0).magnitude().total_cmp(&(y.1 - y.0).magnitude()))
        else {
            continue;
        };
        let along = b - a;
        let left = Vector2::new(-along.y, along.x).normalize();
        let point = a + along * 0.5 + left * (along.magnitude() * 1.0e-3).max(tol * 4.0);
        let owner = outers
            .iter()
            .copied()
            .filter(|&o| inside(&[shapes[o].1.clone()], point))
            .min_by(|&a, &b| shapes[a].2.total_cmp(&shapes[b].2))?;
        holes_of[owner].push(hole);
    }
    let surface = face.surface();
    let mut out = Vec::new();
    for &o in &outers {
        let wires: Vec<Wire<Point3, C>> = once(o)
            .chain(holes_of[o].iter().copied())
            .map(|k| {
                shapes[k]
                    .0
                    .iter()
                    .map(|&h| halves[h].edge.clone())
                    .collect()
            })
            .collect();
        let polygons: Vec<Vec<Point2>> = once(o)
            .chain(holes_of[o].iter().copied())
            .map(|k| shapes[k].1.clone())
            .collect();
        let point = interior_point(&polygons, tol)?;
        let overlap = others.iter().position(|other| {
            let theirs = outline(other, &frame, tol);
            inside(&theirs, point) && !on_outline(&theirs, point, tol)
        });
        let absolute: Vec<Wire<Point3, C>> = match face.orientation() {
            true => wires,
            false => wires.iter().map(Wire::inverse).collect(),
        };
        let mut piece = Face::new_unchecked(absolute, surface.clone());
        if !face.orientation() {
            piece.invert();
        }
        out.push((piece, overlap));
    }
    Some(out)
}
