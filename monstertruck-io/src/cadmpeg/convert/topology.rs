//! Topology: the intermediate representation's entity tables to
//! [`CompressedShell`].
//!
//! Both sides are half-edge B-reps with the same nesting -- body, region, shell,
//! face, loop, coedge, edge, vertex -- so this is translation from string ids to
//! array indices, not reconstruction. What it has to get right is orientation and
//! loop order, because both produce shells that look correct and fail much later.
//!
//! # Orientation composes, and each flip is applied exactly once
//!
//! Four things flip a boundary in the source: a face's sense against its
//! carrier's oriented normal, the carrier's oriented normal against its natural
//! one, a coedge's sense against its edge, and the direction the edge's own
//! parameter interval runs. They do NOT all belong here:
//!
//! * the edge's interval direction is handled where the curve is built
//!   ([`super::curve`]), so a converted edge curve always runs start vertex to end
//!   vertex;
//! * the coedge's sense becomes [`CompressedEdgeIndex::orientation`];
//! * the face's sense, composed with whether the converted surface's natural
//!   normal opposes the carrier's oriented one ([`super::surface::Oriented`]),
//!   becomes [`CompressedFace::orientation`].
//!
//! Applying any of them twice, or in the wrong layer, is the failure mode. The
//! curve is already oriented by the time this module sees it, so nothing here
//! re-reads the edge's interval.
//!
//! # Loop order
//!
//! `Face::try_new` takes the FIRST wire as the outer boundary. Since cadmpeg 0.6
//! the face itself carries the classification ([`FaceLoops`]): a classified face
//! names its outer loop and iterates it first, and an unclassified one keeps
//! source order. Both are taken as the face states them.
//!
//! The RING itself is stated in the FACE sense: counterclockwise about the
//! face's oriented normal. [`CompressedFace`] stores its boundaries in the
//! converted surface's NATURAL sense, and [`CompressedFace::create_face`]
//! reverses them back through the orientation flag. So when the flag is false
//! the ring is reversed and every edge use flipped here; storing it unchanged
//! would reverse it twice and leave the face wound against its own normal while
//! every volume oracle still reads plausible numbers.
//!
//! # Pole occurrences inside a ring
//!
//! A ring may carry `vertex_uses`: pole vertices anchored between two of its
//! coedges, typically a sphere face's seam meeting itself at the pole. They are
//! not converted, as they were not before 0.6. Each anchors at a vertex its
//! neighbouring coedges already end and start at, so the ring of edges stays
//! closed without it; what is lost is only the parameter-space image of the
//! pole, which `CompressedFace` has nowhere to store and every consumer
//! re-derives. A loop that is ONLY a vertex ([`LoopBoundary::Vertex`]) is a
//! different case and is refused by name below.
//!
//! [`CompressedShell`]: monstertruck_topology::compress::CompressedShell
//! [`CompressedEdgeIndex::orientation`]: monstertruck_topology::compress::CompressedEdgeIndex
//! [`CompressedFace::orientation`]: monstertruck_topology::compress::CompressedFace
//! [`FaceLoops`]: cadmpeg_ir::topology::FaceLoops

use std::collections::HashMap;

use cadmpeg_ir::topology::{Face as IrFace, LoopBoundary, Sense, Shell as IrShell};
use monstertruck_modeling::{BoundedCurve, Curve, MetricSpace, ParametricCurve, Point3, Surface};
use monstertruck_topology::compress::{
    CompressedEdge, CompressedEdgeIndex, CompressedFace, CompressedShell,
};

use super::{Context, curve, frame, surface};
use crate::Result;

/// A shell under construction, interning vertices and edges as faces reach them.
///
/// Interning is what makes the output a SHARED-edge shell rather than a pile of
/// independent faces: two faces that use the same source edge get the same index,
/// which is the property the boolean kernel's topology depends on.
pub(super) struct ShellBuilder<'a, 'd> {
    context: &'a Context<'d>,
    vertices: Vec<Point3>,
    vertex_index: HashMap<String, usize>,
    edges: Vec<CompressedEdge<Curve>>,
    edge_index: HashMap<String, usize>,
    faces: Vec<CompressedFace<Surface>>,
}

impl<'a, 'd> ShellBuilder<'a, 'd> {
    pub(super) fn new(context: &'a Context<'d>) -> Self {
        Self {
            context,
            vertices: Vec::new(),
            vertex_index: HashMap::new(),
            edges: Vec::new(),
            edge_index: HashMap::new(),
            faces: Vec::new(),
        }
    }

    /// Add every face of one source shell.
    ///
    /// Several source shells may be added to one builder. That is how a sheet body
    /// with more than one shell is carried: [`super::ImportedSheet`] is a single
    /// compressed shell, whose `faces` array is free to hold disconnected
    /// components, and for a body that bounds no volume the shell partition
    /// carries no meaning to lose. A SOLID's shells are never merged this way --
    /// there each one is a separate boundary and the partition is the difference
    /// between a void and an outer surface.
    ///
    /// A shell that also owns wire edges or free vertices -- members a 0.6 shell
    /// can carry beside its faces -- is refused by name: `CompressedShell` has
    /// nowhere to put them, and converting only the faces would hand back fewer
    /// entities than the shell holds without saying so.
    pub(super) fn add_shell(&mut self, shell: &IrShell) -> Result<()> {
        if !shell.wire_edges().is_empty() || !shell.free_vertices().is_empty() {
            return Err(crate::Error::UnsupportedBodyKind {
                format: self.context.format,
                kind: "shell with wire edges or free vertices",
            });
        }
        for face in shell.faces() {
            let face = self.context.face(face.as_str())?;
            let face = self.build_face(face)?;
            self.faces.push(face);
        }
        Ok(())
    }

    pub(super) fn finish(self) -> CompressedShell<Point3, Curve, Surface> {
        CompressedShell {
            vertices: self.vertices,
            edges: self.edges,
            faces: self.faces,
            vertex_stable_ids: None,
            edge_stable_ids: None,
            face_stable_ids: None,
        }
    }

    /// Whether anything was built at all.
    pub(super) fn is_empty(&self) -> bool { self.faces.is_empty() }

    fn build_face(&mut self, face: &IrFace) -> Result<CompressedFace<Surface>> {
        let carrier = self.context.surface(face.surface.as_str())?;
        let oriented = surface::convert(&carrier.geometry, self.context)?;
        // The face's sense is relative to the carrier's ORIENTED normal, and
        // `reversed` says the converted surface's natural normal opposes that
        // one. The flag is therefore `sense != reversed`, and it also decides
        // which sense the STORED boundaries are in (below).
        let orientation = (face.sense == Sense::Forward) != oriented.reversed;
        // `FaceLoops::iter` yields a classified face's outer loop FIRST, which
        // is where `Face::try_new` reads the outer boundary; an unclassified
        // face keeps source order. So the order is taken as given.
        let mut boundaries = Vec::with_capacity(face.loops.len());
        for identifier in face.loops.iter() {
            let ring = self.context.boundary_loop(identifier.as_str())?;
            let LoopBoundary::Ring(ring) = &ring.boundary else {
                // A loop that is one vertex is a surface singularity -- a pole,
                // where a whole boundary collapses to one point. monstertruck's
                // compressed boundary is a ring of EDGES with nowhere to put it.
                return Err(crate::Error::UnsupportedSurfaceKind {
                    format: self.context.format,
                    kind: "singular (pole) face boundary",
                });
            };
            let mut edges = Vec::with_capacity(ring.coedges().len());
            for identifier in ring.coedges() {
                let coedge = self.context.coedge(identifier.as_str())?;
                if coedge.use_curve.is_some() {
                    // A coedge-local 3D carrier means the two faces meeting at
                    // this edge disagree about where it is. `CompressedEdge` holds
                    // ONE curve, so admitting this would silently keep one face's
                    // version and hand the other a boundary it does not lie on.
                    return Err(crate::Error::UnsupportedCurveKind {
                        format: self.context.format,
                        kind: "coedge-local (tolerant edge)",
                    });
                }
                let index = self.intern_edge(coedge.edge.as_str())?;
                edges.push(CompressedEdgeIndex {
                    index,
                    orientation: coedge.sense == Sense::Forward,
                });
            }
            // The source states the ring in the FACE sense (counterclockwise
            // about the face's oriented normal). `CompressedFace` stores it in
            // the converted surface's NATURAL sense, and `create_face` reverses
            // it back through the orientation flag. Storing the face-sense ring
            // unchanged would make that reversal happen twice and hand the
            // kernel a boundary wound against its own face.
            if !orientation {
                edges.reverse();
                edges
                    .iter_mut()
                    .for_each(|edge_use| edge_use.orientation = !edge_use.orientation);
            }
            boundaries.push(edges);
        }
        if boundaries.is_empty() {
            return Err(self
                .context
                .malformed(format!("face {} carries no boundary loop", face.id)));
        }
        Ok(CompressedFace {
            boundaries,
            // Applied exactly once; the stored rings were reversed above when
            // this is false, so the two flips compose to the source's loop.
            orientation,
            surface: oriented.surface,
        })
    }

    /// The index of a source edge in this shell, converting it on first use.
    fn intern_edge(&mut self, identifier: &str) -> Result<usize> {
        if let Some(index) = self.edge_index.get(identifier) {
            return Ok(*index);
        }
        let edge = self.context.edge(identifier)?;
        let front = self.intern_vertex(edge.start.as_str())?;
        let back = self.intern_vertex(edge.end.as_str())?;
        let Some(carrier) = edge.curve() else {
            // An edge with no attributed curve is a tolerant or degenerate edge.
            // There is no geometry to convert and inventing a line between the
            // vertices would put a straight boundary where the file declined to
            // say there was one.
            return Err(crate::Error::UnsupportedCurveKind {
                format: self.context.format,
                kind: "absent (tolerant or degenerate edge)",
            });
        };
        let carrier = self.context.curve(carrier.as_str())?;
        let curve = curve::convert(
            &carrier.geometry,
            edge.param_range().map(|range| range.get()),
            (self.vertices[front], self.vertices[back]),
            self.context,
        )?;
        // A curve that does not run from the edge's start vertex to its end
        // vertex is not this edge, and handing the shell out means the failure
        // surfaces later as an unclosed or non-simple wire far from its cause.
        // Closed edges are exempt: they are a legitimate IGES sheet property
        // (start == end, or two coincident vertices) that the healing passes
        // split, and a full-circle carrier's zero parameter need not lie at the
        // vertex.
        let closed = front == back
            || self.vertices[front].distance(self.vertices[back])
                <= monstertruck_modeling::TOLERANCE;
        if !closed {
            let (start, end) = curve.range_tuple();
            let scale = self.vertices[front].distance(self.vertices[back]);
            let tolerance = monstertruck_modeling::TOLERANCE.max(scale * 1.0e-9);
            let (curve_start, curve_end) = (curve.subs(start), curve.subs(end));
            if curve_start.distance(self.vertices[front]) > tolerance
                || curve_end.distance(self.vertices[back]) > tolerance
            {
                return Err(self.context.malformed(format!(
                    "edge {identifier} states vertices from {:?} to {:?}, but its converted curve \
                     runs from {curve_start:?} to {curve_end:?}",
                    self.vertices[front], self.vertices[back]
                )));
            }
        }
        let index = self.edges.len();
        self.edges.push(CompressedEdge {
            vertices: (front, back),
            curve,
        });
        self.edge_index.insert(identifier.to_owned(), index);
        Ok(index)
    }

    /// The index of a source vertex in this shell, resolving its point on first
    /// use.
    fn intern_vertex(&mut self, identifier: &str) -> Result<usize> {
        if let Some(index) = self.vertex_index.get(identifier) {
            return Ok(*index);
        }
        let vertex = self.context.vertex(identifier)?;
        let point = self.context.point(vertex.point.as_str())?;
        let index = self.vertices.len();
        self.vertices.push(frame::finite_point(point.position()));
        self.vertex_index.insert(identifier.to_owned(), index);
        Ok(index)
    }
}
