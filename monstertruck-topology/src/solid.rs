use crate::errors::Error;
use crate::shell::ShellCondition;
use crate::*;
use std::vec::Vec;

impl<P, C, S> Solid<P, C, S> {
    /// Create a solid whose boundaries must be non-empty, connected,
    /// and closed manifold.
    #[inline(always)]
    pub fn try_new(boundaries: Vec<Shell<P, C, S>>) -> Result<Solid<P, C, S>> {
        for shell in &boundaries {
            if shell.is_empty() {
                return Err(Error::EmptyShell);
            } else if !shell.is_connected() {
                return Err(Error::NotConnected);
            } else if shell.shell_condition() != ShellCondition::Closed {
                return Err(Error::NotClosedShell);
            } else if !shell.singular_vertices().is_empty() {
                return Err(Error::NotManifold);
            }
        }
        Ok(Solid::new_unchecked(boundaries))
    }

    /// Create a solid whose boundaries must be non-empty, connected,
    /// and closed manifold.
    #[inline(always)]
    pub fn new(boundaries: Vec<Shell<P, C, S>>) -> Solid<P, C, S> {
        Self::try_new(boundaries).unwrap_or_else(|error| panic!("Solid::new: {error}"))
    }

    /// Create without validation (for performance-critical code).
    #[inline(always)]
    pub fn new_unchecked(boundaries: Vec<Shell<P, C, S>>) -> Solid<P, C, S> {
        Solid {
            boundaries,
            id_allocator: StableIdAllocator::new(),
            attributes: SolidAttributes::new(),
        }
    }

    /// Allocate a fresh [`StableId`] from this solid's allocator.
    #[inline(always)]
    pub fn alloc_id(&mut self) -> StableId { self.id_allocator.allocate() }

    /// Returns a reference to this solid's [`StableIdAllocator`].
    #[inline(always)]
    pub fn id_allocator(&self) -> &StableIdAllocator { &self.id_allocator }

    /// Returns a mutable reference to this solid's [`StableIdAllocator`].
    #[inline(always)]
    pub fn id_allocator_mut(&mut self) -> &mut StableIdAllocator { &mut self.id_allocator }

    /// Returns a reference to this solid's [`SolidAttributes`].
    #[inline(always)]
    pub fn attributes(&self) -> &SolidAttributes { &self.attributes }

    /// Returns a mutable reference to this solid's [`SolidAttributes`].
    #[inline(always)]
    pub fn attributes_mut(&mut self) -> &mut SolidAttributes { &mut self.attributes }

    /// Returns a reference to the face [`ElementAttributes`].
    #[inline(always)]
    pub fn face_attributes(&self) -> &ElementAttributes { &self.attributes.faces }

    /// Returns a mutable reference to the face [`ElementAttributes`].
    #[inline(always)]
    pub fn face_attributes_mut(&mut self) -> &mut ElementAttributes { &mut self.attributes.faces }

    /// Returns a reference to the edge [`ElementAttributes`].
    #[inline(always)]
    pub fn edge_attributes(&self) -> &ElementAttributes { &self.attributes.edges }

    /// Returns a mutable reference to the edge [`ElementAttributes`].
    #[inline(always)]
    pub fn edge_attributes_mut(&mut self) -> &mut ElementAttributes { &mut self.attributes.edges }

    /// Returns a reference to the vertex [`ElementAttributes`].
    #[inline(always)]
    pub fn vertex_attributes(&self) -> &ElementAttributes { &self.attributes.vertices }

    /// Returns a mutable reference to the vertex [`ElementAttributes`].
    #[inline(always)]
    pub fn vertex_attributes_mut(&mut self) -> &mut ElementAttributes {
        &mut self.attributes.vertices
    }

    /// Create with validation only in debug mode.
    #[inline(always)]
    pub fn debug_new(boundaries: Vec<Shell<P, C, S>>) -> Solid<P, C, S> {
        match cfg!(debug_assertions) {
            true => Solid::new(boundaries),
            false => Solid::new_unchecked(boundaries),
        }
    }

    /// Returns the reference of boundary shells
    #[inline(always)]
    pub const fn boundaries(&self) -> &Vec<Shell<P, C, S>> { &self.boundaries }
    /// Returns the boundary shells
    #[inline(always)]
    pub fn into_boundaries(self) -> Vec<Shell<P, C, S>> { self.boundaries }

    /// Returns an iterator over the faces.
    #[inline(always)]
    pub fn face_iter(&self) -> impl Iterator<Item = &Face<P, C, S>> {
        self.boundaries.iter().flatten()
    }

    /// Returns an iterator over the edges.
    #[inline(always)]
    pub fn edge_iter(&self) -> impl Iterator<Item = Edge<P, C>> + '_ {
        self.face_iter().flat_map(Face::boundaries).flatten()
    }

    /// Returns an iterator over the vertices.
    #[inline(always)]
    pub fn vertex_iter(&self) -> impl Iterator<Item = Vertex<P>> + '_ {
        self.edge_iter().map(|edge| edge.front().clone())
    }

    /// invert all faces
    #[inline(always)]
    pub fn not(&mut self) {
        self.boundaries
            .iter_mut()
            .flat_map(|shell| shell.face_iter_mut())
            .for_each(|face| {
                face.invert();
            })
    }

    /// Assigns a fresh [`StableId`] to every face that does not have one yet,
    /// leaving already-assigned faces untouched.
    ///
    /// Faces of a freshly built solid are [`StableId::UNASSIGNED`] until ids
    /// are explicitly assigned, so per-face attributes (e.g. a colour) cannot
    /// be keyed on them. Calling this first gives every face a stable id to key
    /// attributes on.
    pub fn ensure_face_stable_ids(&mut self) {
        let id_allocator = &mut self.id_allocator;
        for shell in &mut self.boundaries {
            for face in shell.face_iter_mut() {
                if !face.stable_id().is_assigned() {
                    face.set_stable_id(id_allocator.allocate());
                }
            }
        }
    }

    /// Returns a new solid whose surfaces are mapped by `surface_mapping`,
    /// curves are mapped by `curve_mapping` and points are mapped by `point_mapping`.
    /// # Remarks
    /// Accessing geometry elements directly in the closure will result in a deadlock.
    /// So, this method does not appear to the document.
    #[doc(hidden)]
    #[inline(always)]
    pub fn try_mapped<Q, D, T>(
        &self,
        mut point_mapping: impl FnMut(&P) -> Option<Q>,
        mut curve_mapping: impl FnMut(&C) -> Option<D>,
        mut surface_mapping: impl FnMut(&S) -> Option<T>,
    ) -> Option<Solid<Q, D, T>> {
        Some(Solid::debug_new(
            self.boundaries()
                .iter()
                .map(move |shell| {
                    shell.try_mapped(&mut point_mapping, &mut curve_mapping, &mut surface_mapping)
                })
                .collect::<Option<Vec<_>>>()?,
        ))
    }

    /// Returns a new solid whose surfaces are mapped by `surface_mapping`,
    /// curves are mapped by `curve_mapping` and points are mapped by `point_mapping`.
    /// # Remarks
    /// Accessing geometry elements directly in the closure will result in a deadlock.
    /// So, this method does not appear to the document.
    #[doc(hidden)]
    #[inline(always)]
    pub fn mapped<Q, D, T>(
        &self,
        mut point_mapping: impl FnMut(&P) -> Q,
        mut curve_mapping: impl FnMut(&C) -> D,
        mut surface_mapping: impl FnMut(&S) -> T,
    ) -> Solid<Q, D, T> {
        Solid::debug_new(
            self.boundaries()
                .iter()
                .map(move |shell| {
                    shell.mapped(&mut point_mapping, &mut curve_mapping, &mut surface_mapping)
                })
                .collect(),
        )
    }

    /// Returns the consistence of the geometry of end vertices
    /// and the geometry of edge.
    #[inline(always)]
    pub fn is_geometric_consistent(&self) -> bool
    where
        P: Tolerance,
        C: BoundedCurve<Point = P>,
        S: IncludeCurve<C>, {
        self.boundaries()
            .iter()
            .all(|shell| shell.is_geometric_consistent())
    }

    /// Cuts one edge into two edges at vertex.
    #[inline(always)]
    pub fn cut_edge(
        &mut self,
        edge_id: EdgeId<C>,
        vertex: &Vertex<P>,
    ) -> Option<(Edge<P, C>, Edge<P, C>)>
    where
        P: Clone,
        C: Cut<Point = P> + SearchParameter<CurveParameter, Point = P>,
    {
        let res = self
            .boundaries
            .iter_mut()
            .find_map(|shell| shell.cut_edge(edge_id, vertex));
        #[cfg(debug_assertions)]
        let _ = Solid::new(self.boundaries.clone());
        res
    }
    /// Removes `vertex` from `self` by concat two edges on both sides.
    #[inline(always)]
    pub fn remove_vertex_by_concat_edges(&mut self, vertex_id: VertexId<P>) -> Option<Edge<P, C>>
    where
        P: Debug,
        C: Concat<C, Point = P, Output = C> + Invertible + ParameterTransform, {
        let res = self
            .boundaries
            .iter_mut()
            .find_map(|shell| shell.remove_vertex_by_concat_edges(vertex_id));
        #[cfg(debug_assertions)]
        let _ = Solid::new(self.boundaries.clone());
        res
    }

    /// Cut a face with `face_id` by edge.
    #[inline(always)]
    pub fn cut_face_by_edge(&mut self, face_id: FaceId<S>, edge: Edge<P, C>) -> bool
    where S: Clone {
        let tuple = self.boundaries.iter_mut().find_map(|shell| {
            let find_res = shell
                .face_iter_mut()
                .enumerate()
                .find(move |(_, face)| face.id() == face_id)
                .map(move |(i, _)| i);
            find_res.map(move |i| (shell, i))
        });
        if let Some((shell, i)) = tuple
            && let Some((face0, face1)) = shell[i].cut_by_edge(edge)
        {
            shell[i] = face0;
            shell.push(face1);
            return true;
        }
        false
    }

    /// Creates display struct for debugging the solid.
    #[inline(always)]
    pub fn display(
        &self,
        format: SolidDisplayFormat,
    ) -> DebugDisplay<'_, Self, SolidDisplayFormat> {
        DebugDisplay {
            entity: self,
            format,
        }
    }
}

impl<P: Clone, C: Clone, S: Clone> Solid<P, C, Option<S>> {
    /// Returns the value with the Option removed if there is no `None` in the surfaces of the faces.
    #[inline(always)]
    pub fn collect_option(&self) -> Option<Solid<P, C, S>> {
        self.try_mapped(|x| Some(x.clone()), |x| Some(x.clone()), Clone::clone)
    }
}

impl<P, C, S> PartialEq for Solid<P, C, S> {
    fn eq(&self, other: &Self) -> bool { self.boundaries == other.boundaries }
}

impl<P, C, S> Eq for Solid<P, C, S> {}

impl<P: Debug, C: Debug, S: Debug> Debug for DebugDisplay<'_, Solid<P, C, S>, SolidDisplayFormat> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self.format {
            SolidDisplayFormat::ShellsList { shell_format } => f
                .debug_list()
                .entries(
                    self.entity
                        .boundaries
                        .iter()
                        .map(|shell| shell.display(shell_format)),
                )
                .finish(),
            SolidDisplayFormat::ShellsListTuple { shell_format } => f
                .debug_tuple("Solid")
                .field(&DebugDisplay {
                    entity: self.entity,
                    format: SolidDisplayFormat::ShellsList { shell_format },
                })
                .finish(),
            SolidDisplayFormat::Struct { shell_format } => f
                .debug_struct("Solid")
                .field(
                    "boundaries",
                    &DebugDisplay {
                        entity: self.entity,
                        format: SolidDisplayFormat::ShellsList { shell_format },
                    },
                )
                .finish(),
        }
    }
}

#[cfg(test)]
pub(super) fn cube() -> Solid<(), (), ()> {
    use crate::*;
    let v = Vertex::from_points([(); 8]);
    let edge = [
        Edge::new(&v[0], &v[1], ()), // 0
        Edge::new(&v[1], &v[2], ()), // 1
        Edge::new(&v[2], &v[3], ()), // 2
        Edge::new(&v[3], &v[0], ()), // 3
        Edge::new(&v[0], &v[4], ()), // 4
        Edge::new(&v[1], &v[5], ()), // 5
        Edge::new(&v[2], &v[6], ()), // 6
        Edge::new(&v[3], &v[7], ()), // 7
        Edge::new(&v[4], &v[5], ()), // 8
        Edge::new(&v[5], &v[6], ()), // 9
        Edge::new(&v[6], &v[7], ()), // 10
        Edge::new(&v[7], &v[4], ()), // 11
    ];

    let wire0 = wire![&edge[0], &edge[1], &edge[2], &edge[3]];
    let face0 = Face::new(vec![wire0], ());

    let wire1 = wire![&edge[4], &edge[8], &edge[5].inverse(), &edge[0].inverse(),];
    let face1 = Face::new(vec![wire1], ());

    let wire2 = wire![&edge[5], &edge[9], &edge[6].inverse(), &edge[1].inverse(),];
    let face2 = Face::new(vec![wire2], ());

    let wire3 = wire![&edge[6], &edge[10], &edge[7].inverse(), &edge[2].inverse(),];
    let face3 = Face::new(vec![wire3], ());
    let wire4 = wire![&edge[7], &edge[11], &edge[4].inverse(), &edge[3].inverse(),];
    let face4 = Face::new(vec![wire4], ());
    let wire5 = wire![
        &edge[11].inverse(),
        &edge[10].inverse(),
        &edge[9].inverse(),
        &edge[8].inverse(),
    ];
    let face5 = Face::new(vec![wire5], ());

    let mut shell = Shell::new();
    shell.push(face0);
    shell.push(face5);
    assert!(!shell.is_connected());
    shell.push(face1);
    assert_eq!(shell.shell_condition(), ShellCondition::Oriented);
    assert!(shell.is_connected());
    shell.push(face2);
    shell.push(face3);
    shell.push(face4);

    Solid::new(vec![shell])
}

#[test]
fn cube_test() { cube(); }

#[test]
fn ensure_face_stable_ids_assigns_unassigned_faces() {
    use monstertruck_core::StableId;
    let mut solid = cube();
    // A freshly built cube has unassigned face ids.
    assert!(solid.face_iter().all(|f| !f.stable_id().is_assigned()));

    solid.ensure_face_stable_ids();
    let ids: Vec<StableId> = solid.face_iter().map(|f| f.stable_id()).collect();
    assert_eq!(ids.len(), 6);
    assert!(ids.iter().all(|id| id.is_assigned()), "all faces assigned");
    let mut unique = ids.clone();
    unique.sort();
    unique.dedup();
    assert_eq!(unique.len(), 6, "face ids are unique");

    // Idempotent: a second call leaves already-assigned faces untouched.
    solid.ensure_face_stable_ids();
    let ids_again: Vec<StableId> = solid.face_iter().map(|f| f.stable_id()).collect();
    assert_eq!(ids, ids_again, "second call is a no-op for assigned faces");
}
