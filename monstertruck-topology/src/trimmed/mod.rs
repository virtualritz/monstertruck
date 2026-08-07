use crate::compress::{
    CompressedEdge, CompressedEdgeIndex, CompressedEdgeUse, CompressedTrimmedFace,
    CompressedTrimmedShell, CompressedTrimmedSolid,
};
use crate::{errors::Error, *};
use rustc_hash::FxHashMap as HashMap;

/// Destructured parts of a [`TrimmedFace`].
pub type TrimmedFaceParts<P, C, S, T> = (Face<P, C, S>, Vec<Vec<Option<T>>>);

/// Runtime face with face-local trim storage per edge-use.
#[derive(Clone, Debug)]
pub struct TrimmedFace<P, C, S, T> {
    face: Face<P, C, S>,
    trims: Vec<Vec<Option<T>>>,
}

/// Runtime shell with face-local trim storage per edge-use.
#[derive(Clone, Debug)]
pub struct TrimmedShell<P, C, S, T> {
    face_list: Vec<TrimmedFace<P, C, S, T>>,
}

/// Runtime solid with face-local trim storage per edge-use.
#[derive(Clone, Debug)]
pub struct TrimmedSolid<P, C, S, T> {
    boundaries: Vec<TrimmedShell<P, C, S, T>>,
}

impl<P, C, S, T> TrimmedFace<P, C, S, T> {
    /// Creates a trimmed face from a face and per-edge-use trims.
    ///
    /// The `trims` layout must match [`Face::absolute_boundaries`].
    pub fn try_new(face: Face<P, C, S>, trims: Vec<Vec<Option<T>>>) -> Result<Self> {
        let same_wire_count = face.absolute_boundaries().len() == trims.len();
        let same_edge_counts = same_wire_count
            && face
                .absolute_boundaries()
                .iter()
                .zip(trims.iter())
                .all(|(wire, trim_wire)| wire.len() == trim_wire.len());
        if same_edge_counts {
            Ok(Self { face, trims })
        } else {
            Err(Error::NotSimpleWire)
        }
    }

    /// Creates a trimmed face from a face and per-edge-use trims.
    /// # Panics
    /// Panics if the trim layout does not match [`Face::absolute_boundaries`].
    pub fn new(face: Face<P, C, S>, trims: Vec<Vec<Option<T>>>) -> Self {
        Self::try_new(face, trims).expect("TrimmedFace::new: trim layout mismatch")
    }

    /// Returns the wrapped face.
    #[inline(always)]
    pub const fn face(&self) -> &Face<P, C, S> { &self.face }

    /// Returns the face-local trims matching [`Face::absolute_boundaries`].
    #[inline(always)]
    pub const fn trims(&self) -> &Vec<Vec<Option<T>>> { &self.trims }

    /// Consumes `self` and drops trim storage.
    #[inline(always)]
    pub fn erase_trims(self) -> Face<P, C, S> { self.face }

    /// Destructures into the face and trim storage.
    #[inline(always)]
    pub fn into_parts(self) -> TrimmedFaceParts<P, C, S, T> { (self.face, self.trims) }
}

impl<P, C, S, T> From<Face<P, C, S>> for TrimmedFace<P, C, S, T> {
    fn from(face: Face<P, C, S>) -> Self {
        let trims = face
            .absolute_boundaries()
            .iter()
            .map(|wire| wire.iter().map(|_| None).collect())
            .collect();
        Self { face, trims }
    }
}

impl<P, C, S, T> From<TrimmedFace<P, C, S, T>> for Face<P, C, S> {
    fn from(face: TrimmedFace<P, C, S, T>) -> Self { face.erase_trims() }
}

impl<P, C, S, T> FromIterator<TrimmedFace<P, C, S, T>> for TrimmedShell<P, C, S, T> {
    fn from_iter<I: IntoIterator<Item = TrimmedFace<P, C, S, T>>>(iter: I) -> Self {
        Self {
            face_list: iter.into_iter().collect(),
        }
    }
}

impl<P, C, S, T> TrimmedShell<P, C, S, T> {
    /// Returns the faces.
    #[inline(always)]
    pub const fn faces(&self) -> &Vec<TrimmedFace<P, C, S, T>> { &self.face_list }

    /// Consumes `self` and drops trim storage.
    pub fn erase_trims(self) -> Shell<P, C, S> {
        self.face_list
            .into_iter()
            .map(TrimmedFace::erase_trims)
            .collect()
    }

    /// A plain [`Shell`] view of the faces, trims dropped, without consuming
    /// `self`. Face clones are handle clones; no geometry is copied.
    fn to_plain_shell(&self) -> Shell<P, C, S> {
        self.face_list
            .iter()
            .map(|trimmed_face| trimmed_face.face.clone())
            .collect()
    }

    /// The precondition this shell must satisfy to be a boundary of a
    /// [`TrimmedSolid`]; see [`Shell::check_solid_boundary`].
    pub fn check_solid_boundary(&self) -> Result<()> {
        self.to_plain_shell().check_solid_boundary()
    }
}

impl<P, C, S, T> From<Shell<P, C, S>> for TrimmedShell<P, C, S, T> {
    fn from(shell: Shell<P, C, S>) -> Self { shell.into_iter().map(TrimmedFace::from).collect() }
}

impl<P, C, S, T> From<TrimmedShell<P, C, S, T>> for Shell<P, C, S> {
    fn from(shell: TrimmedShell<P, C, S, T>) -> Self { shell.erase_trims() }
}

impl<P, C, S, T> TrimmedSolid<P, C, S, T> {
    /// Creates a trimmed solid from its boundary shells, WITHOUT validating
    /// them.
    ///
    /// # This is `Solid::new_unchecked`, not `Solid::new`
    ///
    /// Despite the shared name, this constructor is the trimmed counterpart of
    /// [`Solid::new_unchecked`] and not of [`Solid::new`]: it checks nothing.
    /// Spec 011's ledger entry for class C11, and the corpus row that pinned
    /// it, both recorded this call as "the infallible `Solid::new`" -- it is
    /// not, it never validated at all, and the abort they were chasing came
    /// from a `Solid::debug_new` two hops downstream in
    /// [`Solid::try_mapped`]. That confusion is the reason this doc comment is
    /// this long.
    ///
    /// Use [`Self::try_new`] whenever the boundaries come from data rather
    /// than from a construction that already established the invariant.
    #[inline(always)]
    pub const fn new(boundaries: Vec<TrimmedShell<P, C, S, T>>) -> Self { Self { boundaries } }

    /// Creates a trimmed solid whose boundaries must be non-empty, connected,
    /// and closed manifold -- the SAME precondition as [`Solid::try_new`],
    /// via the single definition in [`Shell::check_solid_boundary`].
    ///
    /// The plain path has enforced this since forever
    /// (`Solid::extract` ends in `Solid::try_new(shells?)?`); the trimmed path
    /// did not, so a `CompressedTrimmedSolid` that was short a face -- the
    /// shape a typed surface refusal upstream produces -- extracted to `Ok`
    /// and only failed later, in a `debug_assertions`-only construction that
    /// panicked in debug and silently returned an invalid solid in release.
    /// Ledger class C11.
    #[inline(always)]
    pub fn try_new(boundaries: Vec<TrimmedShell<P, C, S, T>>) -> Result<Self> {
        for shell in &boundaries {
            shell.check_solid_boundary()?;
        }
        Ok(Self { boundaries })
    }

    /// Returns the boundary shells.
    #[inline(always)]
    pub const fn boundaries(&self) -> &Vec<TrimmedShell<P, C, S, T>> { &self.boundaries }

    /// Consumes `self` and returns the boundary shells.
    #[inline(always)]
    pub fn into_boundaries(self) -> Vec<TrimmedShell<P, C, S, T>> { self.boundaries }

    /// Consumes `self` and drops trim storage.
    pub fn erase_trims(self) -> Solid<P, C, S> {
        Solid::new_unchecked(
            self.boundaries
                .into_iter()
                .map(TrimmedShell::erase_trims)
                .collect(),
        )
    }
}

impl<P, C, S, T> From<Solid<P, C, S>> for TrimmedSolid<P, C, S, T> {
    fn from(solid: Solid<P, C, S>) -> Self {
        Self {
            boundaries: solid
                .into_boundaries()
                .into_iter()
                .map(TrimmedShell::from)
                .collect(),
        }
    }
}

impl<P, C, S, T> From<TrimmedSolid<P, C, S, T>> for Solid<P, C, S> {
    fn from(solid: TrimmedSolid<P, C, S, T>) -> Self { solid.erase_trims() }
}

impl<P, C, S, T> FromIterator<TrimmedShell<P, C, S, T>> for TrimmedSolid<P, C, S, T> {
    fn from_iter<I: IntoIterator<Item = TrimmedShell<P, C, S, T>>>(iter: I) -> Self {
        Self {
            boundaries: iter.into_iter().collect(),
        }
    }
}

impl<P: Clone, C: Clone, S: Clone> Shell<P, C, S> {
    /// Creates a runtime trimmed shell by evaluating `trim_curve` for each edge-use.
    pub fn to_trimmed_with_face_trims<T, F>(&self, mut trim_curve: F) -> TrimmedShell<P, C, S, T>
    where F: FnMut(&Edge<P, C>, &S) -> Option<T> {
        self.iter()
            .map(|face| {
                let surface = face.surface();
                let trims = face
                    .absolute_boundaries()
                    .iter()
                    .map(|wire| wire.iter().map(|edge| trim_curve(edge, &surface)).collect())
                    .collect();
                TrimmedFace::new(face.clone(), trims)
            })
            .collect()
    }

    /// Creates a runtime trimmed shell from exact trims carried by the edge curves.
    pub fn to_trimmed_with_exact_face_trims(
        &self,
    ) -> TrimmedShell<P, C, S, <C as ExactParameterBoundary2D<S>>::BoundaryCurve>
    where C: ExactParameterBoundary2D<S> {
        self.to_trimmed_with_face_trims(|edge, surface| {
            edge.curve().exact_parameter_boundary_2d(surface)
        })
    }
}

impl<P: Clone, C: Clone, S: Clone> Solid<P, C, S> {
    /// Creates a runtime trimmed solid by evaluating `trim_curve` for each edge-use.
    pub fn to_trimmed_with_face_trims<T, F>(&self, mut trim_curve: F) -> TrimmedSolid<P, C, S, T>
    where F: FnMut(&Edge<P, C>, &S) -> Option<T> {
        TrimmedSolid {
            boundaries: self
                .boundaries()
                .iter()
                .map(|shell| shell.to_trimmed_with_face_trims(&mut trim_curve))
                .collect(),
        }
    }

    /// Creates a runtime trimmed solid from exact trims carried by the edge curves.
    pub fn to_trimmed_with_exact_face_trims(
        &self,
    ) -> TrimmedSolid<P, C, S, <C as ExactParameterBoundary2D<S>>::BoundaryCurve>
    where C: ExactParameterBoundary2D<S> {
        self.to_trimmed_with_face_trims(|edge, surface| {
            edge.curve().exact_parameter_boundary_2d(surface)
        })
    }
}

struct TrimmedCompressDirector<P, C> {
    vmap: HashMap<VertexId<P>, (usize, P)>,
    emap: HashMap<EdgeId<C>, (usize, CompressedEdge<C>)>,
}

impl<P: Clone, C: Clone> TrimmedCompressDirector<P, C> {
    fn new() -> Self {
        Self {
            vmap: HashMap::default(),
            emap: HashMap::default(),
        }
    }

    fn get_vid(&mut self, vertex: &Vertex<P>) -> usize {
        let id = self.vmap.len();
        self.vmap
            .entry(vertex.id())
            .or_insert_with(|| (id, vertex.point()))
            .0
    }

    fn get_eid(&mut self, edge: &Edge<P, C>) -> CompressedEdgeIndex {
        if let Some((index, _)) = self.emap.get(&edge.id()) {
            (*index, edge.orientation()).into()
        } else {
            let index = self.emap.len();
            let front_id = self.get_vid(edge.absolute_front());
            let back_id = self.get_vid(edge.absolute_back());
            let cedge = CompressedEdge {
                vertices: (front_id, back_id),
                curve: edge.curve(),
            };
            self.emap.insert(edge.id(), (index, cedge));
            (index, edge.orientation()).into()
        }
    }

    fn into_vertices_edges(self) -> (Vec<P>, Vec<CompressedEdge<C>>) {
        let mut vertices: Vec<_> = self.vmap.into_values().collect();
        vertices.sort_by_key(|(index, _)| *index);
        let mut edges: Vec<_> = self.emap.into_values().collect();
        edges.sort_by_key(|(index, _)| *index);
        (
            vertices.into_iter().map(|(_, point)| point).collect(),
            edges.into_iter().map(|(_, edge)| edge).collect(),
        )
    }
}

impl<P: Clone, C: Clone, S: Clone, T: Clone> From<&TrimmedShell<P, C, S, T>>
    for CompressedTrimmedShell<P, C, S, T>
{
    fn from(shell: &TrimmedShell<P, C, S, T>) -> Self {
        let mut director = TrimmedCompressDirector::new();
        let faces = shell
            .face_list
            .iter()
            .map(|face| CompressedTrimmedFace {
                boundaries: face
                    .face
                    .absolute_boundaries()
                    .iter()
                    .zip(face.trims.iter())
                    .map(|(wire, trim_wire)| {
                        wire.iter()
                            .zip(trim_wire.iter())
                            .map(|(edge, trim_curve)| {
                                let CompressedEdgeIndex { index, orientation } =
                                    director.get_eid(edge);
                                CompressedEdgeUse {
                                    index,
                                    orientation,
                                    trim_curve: trim_curve.clone(),
                                }
                            })
                            .collect()
                    })
                    .collect(),
                orientation: face.face.orientation(),
                surface: face.face.surface(),
            })
            .collect();
        let (vertices, edges) = director.into_vertices_edges();
        Self {
            vertices,
            edges,
            faces,
        }
    }
}

impl<P: Clone, C: Clone, S: Clone, T: Clone> From<&TrimmedSolid<P, C, S, T>>
    for CompressedTrimmedSolid<P, C, S, T>
{
    fn from(solid: &TrimmedSolid<P, C, S, T>) -> Self {
        Self {
            boundaries: solid
                .boundaries
                .iter()
                .map(CompressedTrimmedShell::from)
                .collect(),
        }
    }
}

impl<P, C, S, T> TryFrom<CompressedTrimmedShell<P, C, S, T>> for TrimmedShell<P, C, S, T> {
    type Error = Error;

    fn try_from(shell: CompressedTrimmedShell<P, C, S, T>) -> Result<Self> {
        let CompressedTrimmedShell {
            vertices,
            edges,
            faces,
        } = shell;
        let vertices: Vec<_> = vertices.into_iter().map(Vertex::new).collect();
        let edges = edges
            .into_iter()
            .map(|edge| {
                let front = &vertices[edge.vertices.0];
                let back = &vertices[edge.vertices.1];
                Edge::try_new(front, back, edge.curve)
            })
            .collect::<Result<Vec<_>>>()?;
        faces
            .into_iter()
            .map(|face| {
                let boundaries = face
                    .boundaries
                    .iter()
                    .map(|wire| {
                        wire.iter()
                            .map(|edge_use| {
                                if edge_use.orientation {
                                    edges[edge_use.index].clone()
                                } else {
                                    edges[edge_use.index].inverse()
                                }
                            })
                            .collect()
                    })
                    .collect::<Vec<Wire<P, C>>>();
                let trims = face
                    .boundaries
                    .into_iter()
                    .map(|wire| {
                        wire.into_iter()
                            .map(|edge_use| edge_use.trim_curve)
                            .collect()
                    })
                    .collect();
                let mut runtime_face = Face::try_new(boundaries, face.surface)?;
                if !face.orientation {
                    runtime_face.invert();
                }
                TrimmedFace::try_new(runtime_face, trims)
            })
            .collect()
    }
}

impl<P, C, S, T> TryFrom<CompressedTrimmedSolid<P, C, S, T>> for TrimmedSolid<P, C, S, T> {
    type Error = Error;

    fn try_from(solid: CompressedTrimmedSolid<P, C, S, T>) -> Result<Self> {
        Ok(Self {
            boundaries: solid
                .boundaries
                .into_iter()
                .map(TrimmedShell::try_from)
                .collect::<Result<Vec<_>>>()?,
        })
    }
}

#[cfg(test)]
mod tests;
