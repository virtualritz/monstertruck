//! Merging the coincident vertices and edges two boolean operands leave behind.

use super::super::super::integrate::{ShapeOpsCurve, ShapeOpsSurface};
use monstertruck_geometry::prelude::*;
use monstertruck_topology::compress::{CompressedEdge, CompressedEdgeIndex, CompressedShell};
use monstertruck_topology::*;

pub(in super::super) fn sew<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    shell: &Shell<Point3, C, S>,
    tol: f64,
) -> Option<Shell<Point3, C, S>> {
    let compressed = shell.compress();
    let mut vertices: Vec<Point3> = Vec::new();
    let vertex_map: Vec<usize> = compressed
        .vertices
        .iter()
        .map(|p| {
            vertices
                .iter()
                .position(|q| q.distance(*p) < tol)
                .unwrap_or_else(|| {
                    vertices.push(*p);
                    vertices.len() - 1
                })
        })
        .collect();
    let middle = |curve: &C| {
        let (t0, t1) = curve.range_tuple();
        curve.subs((t0 + t1) / 2.0)
    };
    let mut edges: Vec<CompressedEdge<C>> = Vec::new();
    let edge_map: Vec<(usize, bool)> = compressed
        .edges
        .iter()
        .map(|edge| {
            let (a, b) = (vertex_map[edge.vertices.0], vertex_map[edge.vertices.1]);
            let m = middle(&edge.curve);
            edges
                .iter()
                .position(|known| {
                    (known.vertices == (a, b) || known.vertices == (b, a))
                        && middle(&known.curve).distance(m) < tol * 10.0
                })
                .map(|k| (k, edges[k].vertices == (a, b)))
                .unwrap_or_else(|| {
                    edges.push(CompressedEdge {
                        vertices: (a, b),
                        curve: edge.curve.clone(),
                    });
                    (edges.len() - 1, true)
                })
        })
        .collect();
    let faces = compressed
        .faces
        .into_iter()
        .map(|mut face| {
            face.boundaries.iter_mut().flatten().for_each(|use_| {
                let (index, same) = edge_map[use_.index];
                *use_ = CompressedEdgeIndex {
                    index,
                    orientation: use_.orientation == same,
                };
            });
            face
        })
        .collect();
    Shell::extract(CompressedShell {
        vertices,
        edges,
        faces,
        vertex_stable_ids: None,
        edge_stable_ids: None,
        face_stable_ids: None,
    })
    .ok()
}
