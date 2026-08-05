use super::report::{LossCategory as Cat, LossReason as Why, LossSink};
use super::step_geometry::{
    SurfaceCurve3D as StepSurfaceCurve3D,
    SurfaceCurveAssociatedGeometry as StepSurfaceCurveAssociatedGeometry,
    SurfaceCurveKind as StepSurfaceCurveKind,
};
use super::*;
use monstertruck_topology::compress::*;
use std::collections::HashSet;

/// A compressed shell in the STEP loader's concrete geometry types.
///
/// The `*_reported` conversions (spec 011 T7) return the value paired with a
/// [`ShellLoadReport`], and spelling the four generic arguments inline at every
/// such signature is what tripped `clippy::type_complexity`. Naming the three
/// concrete instantiations once keeps the reported and unreported variants
/// visibly the same type.
pub type StepCompressedShell = CompressedShell<Point3, Curve3D, Surface>;

/// A compressed trimmed shell (exact trim curves preserved) in the STEP
/// loader's concrete geometry types. See [`StepCompressedShell`].
pub type StepCompressedTrimmedShell =
    CompressedTrimmedShell<Point3, Curve3D, Surface, step_geometry::StepParameterCurve>;

/// A compressed solid in the STEP loader's concrete geometry types.
/// See [`StepCompressedShell`].
pub type StepCompressedSolid = CompressedSolid<Point3, Curve3D, Surface>;

impl Table {
    fn place_holder_edge_any_to_index_and_edge_curve(
        &self,
        edge: &PlaceHolder<EdgeAnyHolder>,
    ) -> Option<(u64, EdgeCurveHolder)> {
        use PlaceHolder::Ref;
        let Ref(Name::Entity(idx)) = edge else {
            return None;
        };
        self.oriented_edge
            .get(idx)
            .and_then(|oriented_edge| {
                Some((
                    oriented_edge.edge_element_idx()?,
                    oriented_edge.edge_element_holder(self)?,
                ))
            })
            .or_else(|| {
                self.edge_curve
                    .get(idx)
                    .map(|edge_curve| (*idx, edge_curve.clone()))
            })
    }
    fn face_any_to_orientation_and_face(
        &self,
        face: Option<FaceAnyHolder>,
    ) -> Option<(bool, FaceSurfaceHolder)> {
        match face? {
            FaceAnyHolder::FaceSurface(face) => Some((true, face)),
            FaceAnyHolder::OrientedFace(oriented_face) => {
                let face_element = oriented_face.face_element_holder(self)?;
                Some((oriented_face.orientation, face_element))
            }
        }
    }

    /// The entity id behind a reference place-holder, when it has one.
    fn ref_id<T>(place_holder: &PlaceHolder<T>) -> Option<u64> {
        if let PlaceHolder::Ref(Name::Entity(idx)) = place_holder {
            Some(*idx)
        } else {
            None
        }
    }

    /// Resolve a face bound to the `EDGE_LOOP` the loader can represent, and
    /// REPORT the ones it cannot.
    ///
    /// This is the site of the census's only genuinely silent drop: the three
    /// non-`EDGE_LOOP` outcomes used to be one `?` on an `Option`, so a wire
    /// vanished with no error, no message and no count.
    fn resolved_edge_loop(
        &self,
        bound: &FaceBoundHolder,
        sink: &LossSink,
    ) -> Option<EdgeLoopHolder> {
        match bound.resolve_bound(self) {
            BoundResolution::EdgeLoop(edge_loop) => Some(edge_loop),
            BoundResolution::VertexLoop(id) => {
                sink.lost(Cat::Wire, Why::DegenerateVertexLoop, Some(id), None);
                None
            }
            BoundResolution::Unresolved(id) => {
                // A record that IS in the file but has no arm lands in
                // `Table::dummy` with its name intact, which is a different
                // defect from an id that resolves to nothing at all.
                let unrecognised = id.and_then(|id| self.dummy.get(&id));
                let (why, detail) = match unrecognised {
                    Some(dummy) => (
                        Why::BoundNotALoopWeImplement,
                        Some(dummy.record.chars().take(80).collect::<String>()),
                    ),
                    None => (Why::BoundUnresolved, None),
                };
                sink.lost(Cat::Wire, why, id, detail);
                None
            }
        }
    }

    fn shell_vertices(
        &self,
        shell: &ShellHolder,
        sink: &LossSink,
    ) -> (Vec<Point3>, HashMap<u64, usize>) {
        use PlaceHolder::Ref;
        let mut vidx_map = HashMap::<u64, usize>::new();
        // Distinct ids we have already TRIED, as opposed to ids we succeeded on.
        //
        // The dedup guard used to be `!vidx_map.contains_key(&idx)` with the
        // insert done BEFORE the fallible resolve, so a vertex that failed still
        // consumed an index and every later vertex's `vidx_map` entry pointed one
        // slot past its own point -- silently wrong topology, not merely lossy.
        // Keeping "attempted" separate from "kept" makes the index dense over
        // successes; when nothing fails the numbering is identical to before.
        let mut attempted = HashSet::<u64>::new();
        let vertex_to_point = |v: PlaceHolder<VertexPointHolder>| {
            let Ref(Name::Entity(idx)) = v else {
                sink.listed(Cat::Vertex, 1);
                sink.lost(
                    Cat::Vertex,
                    Why::VertexUnresolved,
                    None,
                    Some("the vertex is not an entity reference".to_owned()),
                );
                return None;
            };
            if !attempted.insert(idx) {
                return None;
            }
            sink.listed(Cat::Vertex, 1);
            match EntityTable::<VertexPointHolder>::get_owned(self, idx) {
                Ok(vertex) => {
                    let index = vidx_map.len();
                    vidx_map.insert(idx, index);
                    sink.kept(Cat::Vertex, 1);
                    Some(Point3::from(&vertex.vertex_geometry))
                }
                Err(e) => {
                    eprintln!("{e}");
                    sink.lost(
                        Cat::Vertex,
                        Why::VertexUnresolved,
                        Some(idx),
                        Some(e.to_string()),
                    );
                    None
                }
            }
        };
        let vertices: Vec<Point3> = shell
            .cfs_faces_holder(self)
            .filter_map(move |face| self.face_any_to_orientation_and_face(face))
            .flat_map(move |(_, face)| face.bounds_holder(self))
            // Wires are counted ONCE, in `shell_faces`/`shell_trimmed_faces`.
            // This walk only needs the edges, so it uses the non-reporting
            // resolver and cannot double-count a lost vertex loop.
            .filter_map(move |(_, bound)| bound?.bound_holder(self))
            .flat_map(move |bound| bound.edge_list)
            .filter_map(move |edge| self.place_holder_edge_any_to_index_and_edge_curve(&edge))
            .flat_map(move |(_, edge)| [edge.edge_start, edge.edge_end])
            .filter_map(vertex_to_point)
            .collect();
        (vertices, vidx_map)
    }

    fn shell_edges(
        &self,
        shell: &ShellHolder,
        vidx_map: &HashMap<u64, usize>,
        sink: &LossSink,
    ) -> (Vec<CompressedEdge<Curve3D>>, HashMap<u64, usize>) {
        use PlaceHolder::Ref;
        let mut eidx_map = HashMap::<u64, usize>::new();
        // Same split as `shell_vertices`, for the same reason: a failed edge used
        // to consume an index and shift every later edge's `eidx_map` entry.
        let mut attempted = HashSet::<u64>::new();
        let edge_curve_to_compressed_edge = |(idx, edge): (u64, EdgeCurveHolder)| {
            if !attempted.insert(idx) {
                return None;
            }
            sink.listed(Cat::Edge, 1);
            let edge_curve = match edge.clone().into_owned(self) {
                Ok(edge_curve) => edge_curve,
                Err(e) => {
                    eprintln!("{e}");
                    sink.lost(
                        Cat::Edge,
                        Why::EdgeUnresolved,
                        Some(idx),
                        Some(e.to_string()),
                    );
                    return None;
                }
            };
            let curve = match edge_curve.parse_curve3d() {
                Ok(curve) => curve,
                Err(e) => {
                    eprintln!("{e}");
                    sink.lost(
                        Cat::Edge,
                        Why::EdgeCurveRefused,
                        Some(idx),
                        Some(e.to_string()),
                    );
                    return None;
                }
            };
            let (Ref(Name::Entity(front_idx)), Ref(Name::Entity(back_idx))) =
                (&edge.edge_start, &edge.edge_end)
            else {
                sink.lost(
                    Cat::Edge,
                    Why::EdgeEndpointUnresolved,
                    Some(idx),
                    Some("edge_start or edge_end is not an entity reference".to_owned()),
                );
                return None;
            };
            let (Some(front), Some(back)) = (vidx_map.get(front_idx), vidx_map.get(back_idx))
            else {
                sink.lost(
                    Cat::Edge,
                    Why::EdgeEndpointUnresolved,
                    Some(idx),
                    Some(format!("no kept vertex for #{front_idx} or #{back_idx}",)),
                );
                return None;
            };
            let index = eidx_map.len();
            eidx_map.insert(idx, index);
            sink.kept(Cat::Edge, 1);
            Some(CompressedEdge {
                vertices: (*front, *back),
                curve,
            })
        };
        let edges: Vec<CompressedEdge<Curve3D>> = shell
            .cfs_faces_holder(self)
            .filter_map(move |face| self.face_any_to_orientation_and_face(face))
            .flat_map(move |(_, face)| face.bounds_holder(self))
            // Wires are counted ONCE, in `shell_faces`/`shell_trimmed_faces`.
            // This walk only needs the edges, so it uses the non-reporting
            // resolver and cannot double-count a lost vertex loop.
            .filter_map(move |(_, bound)| bound?.bound_holder(self))
            .flat_map(move |bound| bound.edge_list)
            .filter_map(move |edge| self.place_holder_edge_any_to_index_and_edge_curve(&edge))
            .filter_map(edge_curve_to_compressed_edge)
            .collect();
        (edges, eidx_map)
    }
    /// Composes a `FACE_BOUND`'s own orientation with the face's `same_sense`,
    /// so a boundary loop is emitted in the SURFACE's sense and not the FACE's.
    ///
    /// # Why this composition exists (ledger C15, spec 013 V1)
    ///
    /// ISO 10303-42 orients an `ADVANCED_FACE`'s loops about the FACE normal,
    /// and the face normal is the surface normal only when `same_sense` is
    /// `.T.`. `CompressedFace`/`CompressedTrimmedFace` want the other
    /// convention: `CompressDirector::create_cface` stores `Face::boundaries`
    /// -- the ABSOLUTE boundaries, in the surface's own sense -- alongside the
    /// orientation flag, and `CompressedFace::create_face` rebuilds with
    /// `Face::try_new(stored, surface)` followed by `invert()`, which flips the
    /// EFFECTIVE traversal a second time.
    ///
    /// Passing STEP's loops through verbatim therefore traverses every
    /// `same_sense = .F.` face backwards relative to its neighbours. Measured
    /// on `occt-cube.step` (2026-08-01): `#17`, `#237` and `#331` are `.F.`
    /// against `#137`, `#284`, `#338` `.T.`, and the loaded shell came out
    /// `ShellCondition::Regular` -- all twelve edges traversed the SAME way by
    /// both their faces, none of them opposite.
    ///
    /// `normalize_trimmed_shell_orientation` then repaired that topologically,
    /// by 2-colouring from face 0. That is correct as far as it goes and its
    /// doc is explicit that it "does NOT decide global outwardness" -- but face
    /// 0 of the cube is `#17`, a `.F.` face, so the whole shell landed INWARD:
    /// six `false` flags and a divergence-theorem volume of **-1000** on a
    /// 1000-volume cube. That number is what `OperandVolume::trusted` (`> 0`,
    /// `<=` a CERTIFIED bounding box since spec 014 W2 -- the vertex hull it
    /// used before was not one) and `verify_volume_conservation` bound booleans
    /// with.
    ///
    /// The reversal machinery this reuses is the one already here for
    /// `FACE_BOUND.orientation`: flip each use, reverse the loop, invert the
    /// exact trim, and pick the other seam curve. Composing is all that was
    /// missing.
    #[inline]
    fn absolute_bound_orientation(bound: &FaceBoundHolder, face_sense: bool) -> bool {
        bound.orientation == face_sense
    }

    /// A `FACE_BOUND`'s loop, in the SURFACE's own sense.
    ///
    /// `face_sense` is the value that will be stored as the compressed face's
    /// `orientation`, and it belongs here as well as on the face: see
    /// [`Self::absolute_bound_orientation`].
    fn face_bound_to_edges(
        &self,
        bound: FaceBoundHolder,
        face_sense: bool,
        eidx_map: &HashMap<u64, usize>,
        sink: &LossSink,
    ) -> Option<Vec<CompressedEdgeIndex>> {
        use PlaceHolder::Ref;
        let ori = Self::absolute_bound_orientation(&bound, face_sense);
        let bound = self.resolved_edge_loop(&bound, sink)?;
        sink.listed(Cat::EdgeUse, bound.edge_list.len());
        let mut edges: Vec<CompressedEdgeIndex> = bound
            .edge_list
            .into_iter()
            .filter_map(|edge| {
                let Ref(Name::Entity(idx)) = edge else {
                    sink.lost(
                        Cat::EdgeUse,
                        Why::EdgeUseUnresolved,
                        None,
                        Some("the edge use is not an entity reference".to_owned()),
                    );
                    return None;
                };
                let edge_idx = if let Some(oriented_edge) = self.oriented_edge.get(&idx) {
                    let element = oriented_edge
                        .edge_element_idx()
                        .and_then(|element| eidx_map.get(&element).copied());
                    let Some(index) = element else {
                        sink.lost(Cat::EdgeUse, Why::EdgeUseUnresolved, Some(idx), None);
                        return None;
                    };
                    CompressedEdgeIndex {
                        index,
                        orientation: oriented_edge.orientation == ori,
                    }
                } else {
                    let Some(index) = eidx_map.get(&idx).copied() else {
                        sink.lost(Cat::EdgeUse, Why::EdgeUseUnresolved, Some(idx), None);
                        return None;
                    };
                    CompressedEdgeIndex {
                        index,
                        orientation: ori,
                    }
                };
                Some(edge_idx)
            })
            .collect();
        sink.kept(Cat::EdgeUse, edges.len());
        if !ori {
            edges.reverse();
        }
        Some(edges)
    }

    fn exact_trim_curve_on(
        curve: &Curve3D,
        surface: &Surface,
        orientation: bool,
    ) -> Option<step_geometry::StepParameterCurve> {
        let mut trim_curve = match curve {
            Curve3D::SurfaceCurve(surface_curve) => {
                Self::seam_trim_curve_on(surface_curve, surface, orientation).or_else(|| {
                    step_geometry::StepParameterCurve::try_from(step_geometry::CurveTrimRef::new(
                        curve, surface,
                    ))
                    .ok()
                })?
            }
            _ => step_geometry::StepParameterCurve::try_from(step_geometry::CurveTrimRef::new(
                curve, surface,
            ))
            .ok()?,
        };
        if !orientation {
            trim_curve.invert();
        }
        Some(trim_curve)
    }

    fn seam_trim_curve_on(
        curve: &StepSurfaceCurve3D,
        surface: &Surface,
        orientation: bool,
    ) -> Option<step_geometry::StepParameterCurve> {
        (curve.kind() == StepSurfaceCurveKind::SeamCurve)
            .then(|| {
                let curves = curve
                    .associated_geometry()
                    .iter()
                    .filter_map(|geometry| match geometry {
                        StepSurfaceCurveAssociatedGeometry::ParameterCurve(trim_curve)
                            if trim_curve.surface().as_ref() == surface
                                || StepSurfaceCurve3D::same_surface(
                                    trim_curve.surface().as_ref(),
                                    surface,
                                ) =>
                        {
                            Some(trim_curve)
                        }
                        _ => None,
                    })
                    .cloned()
                    .collect::<Vec<_>>();
                let index = usize::from(!orientation);
                curves.get(index).or_else(|| curves.first()).cloned()
            })
            .flatten()
    }

    /// The trimmed sibling of [`Self::face_bound_to_edges`], likewise in the
    /// SURFACE's own sense. Reversing a use also inverts its exact trim, which
    /// `exact_trim_curve_on` already does off the composed orientation.
    fn face_bound_to_edge_uses(
        &self,
        bound: FaceBoundHolder,
        face_sense: bool,
        face_surface: &Surface,
        eidx_map: &HashMap<u64, usize>,
        sink: &LossSink,
    ) -> Option<Vec<CompressedEdgeUse<step_geometry::StepParameterCurve>>> {
        use PlaceHolder::Ref;
        let ori = Self::absolute_bound_orientation(&bound, face_sense);
        let bound = self.resolved_edge_loop(&bound, sink)?;
        sink.listed(Cat::EdgeUse, bound.edge_list.len());
        let mut edges = bound
            .edge_list
            .into_iter()
            .filter_map(|edge| {
                let Ref(Name::Entity(idx)) = edge else {
                    sink.lost(
                        Cat::EdgeUse,
                        Why::EdgeUseUnresolved,
                        None,
                        Some("the edge use is not an entity reference".to_owned()),
                    );
                    return None;
                };
                let Some((edge_entity_idx, edge_curve_holder)) =
                    self.place_holder_edge_any_to_index_and_edge_curve(&Ref(Name::Entity(idx)))
                else {
                    sink.lost(Cat::EdgeUse, Why::EdgeUseUnresolved, Some(idx), None);
                    return None;
                };
                let orientation = self
                    .oriented_edge
                    .get(&idx)
                    .map(|oriented_edge| oriented_edge.orientation == ori)
                    .unwrap_or(ori);
                let edge_curve = match edge_curve_holder.into_owned(self) {
                    Ok(edge_curve) => edge_curve,
                    Err(e) => {
                        eprintln!("{e}");
                        sink.lost(
                            Cat::EdgeUse,
                            Why::EdgeUnresolved,
                            Some(idx),
                            Some(e.to_string()),
                        );
                        return None;
                    }
                };
                let curve = match edge_curve.parse_curve3d() {
                    Ok(curve) => curve,
                    Err(e) => {
                        eprintln!("{e}");
                        sink.lost(
                            Cat::EdgeUse,
                            Why::EdgeCurveRefused,
                            Some(idx),
                            Some(e.to_string()),
                        );
                        return None;
                    }
                };
                let Some(index) = eidx_map.get(&edge_entity_idx).copied() else {
                    sink.lost(Cat::EdgeUse, Why::EdgeUseUnresolved, Some(idx), None);
                    return None;
                };
                Some(CompressedEdgeUse {
                    index,
                    orientation,
                    trim_curve: Self::exact_trim_curve_on(&curve, face_surface, orientation),
                })
            })
            .collect::<Vec<_>>();
        sink.kept(Cat::EdgeUse, edges.len());
        if !ori {
            edges.reverse();
        }
        Some(edges)
    }

    /// Resolve and convert one face's surface, REPORTING both refusal arms.
    ///
    /// Both arms used to be `.map_err(|e| eprintln!("{e}")).ok()?`, which is how a
    /// typed per-record refusal still reached the caller as `Ok(shell)` minus a
    /// face -- 253 corpus faces, plus every face of a shell whose surface class
    /// the loader refuses.
    fn face_surface_or_report(&self, face: &FaceSurfaceHolder, sink: &LossSink) -> Option<Surface> {
        let id = Self::ref_id(&face.face_geometry);
        let step_surface: SurfaceAny = match face.face_geometry.clone().into_owned(self) {
            Ok(step_surface) => step_surface,
            Err(e) => {
                eprintln!("{e}");
                sink.lost(Cat::Face, Why::SurfaceUnresolved, id, Some(e.to_string()));
                return None;
            }
        };
        match Surface::try_from(&step_surface) {
            Ok(surface) => Some(surface),
            Err(e) => {
                eprintln!("{e}");
                sink.lost(Cat::Face, Why::SurfaceRefused, id, Some(e.to_string()));
                None
            }
        }
    }

    /// The face's boundary loops, with every loop the loader cannot represent
    /// counted against [`Cat::Wire`] rather than dropped in silence.
    fn face_boundaries_or_report<T>(
        &self,
        face: &FaceSurfaceHolder,
        sink: &LossSink,
        mut to_wire: impl FnMut(FaceBoundHolder) -> Option<T>,
    ) -> Vec<T> {
        let bounds = face.bounds_holder(self);
        sink.listed(Cat::Wire, bounds.len());
        bounds
            .into_iter()
            .filter_map(|(id, bound)| {
                let Some(bound) = bound else {
                    sink.lost(Cat::Wire, Why::BoundUnresolved, id, None);
                    return None;
                };
                let wire = to_wire(bound)?;
                sink.kept(Cat::Wire, 1);
                Some(wire)
            })
            .collect()
    }

    /// Every face the shell lists, paired with the orientation, reporting the
    /// ones whose reference resolves to no face record at all.
    fn shell_listed_faces<'a>(
        &'a self,
        shell: &'a ShellHolder,
        sink: &'a LossSink,
    ) -> impl Iterator<Item = (bool, FaceSurfaceHolder)> + 'a {
        shell.cfs_faces_holder(self).filter_map(move |face| {
            sink.listed(Cat::Face, 1);
            let resolved = self.face_any_to_orientation_and_face(face);
            if resolved.is_none() {
                sink.lost(Cat::Face, Why::FaceUnresolved, None, None);
            }
            resolved
        })
    }

    fn shell_faces(
        &self,
        shell: &ShellHolder,
        eidx_map: &HashMap<u64, usize>,
        sink: &LossSink,
    ) -> Vec<CompressedFace<Surface>> {
        self.shell_listed_faces(shell, sink)
            .filter_map(|(orientation, face)| {
                let surface = self.face_surface_or_report(&face, sink)?;
                // The stored boundaries and the stored flag are ONE convention,
                // not two: see `absolute_bound_orientation`.
                let face_sense = orientation == face.same_sense;
                let boundaries = self.face_boundaries_or_report(&face, sink, |bound| {
                    self.face_bound_to_edges(bound, face_sense, eidx_map, sink)
                });
                sink.kept(Cat::Face, 1);
                Some(CompressedFace {
                    surface,
                    boundaries,
                    orientation: face_sense,
                })
            })
            .collect()
    }

    fn shell_trimmed_faces(
        &self,
        shell: &ShellHolder,
        eidx_map: &HashMap<u64, usize>,
        sink: &LossSink,
    ) -> Vec<CompressedTrimmedFace<Surface, step_geometry::StepParameterCurve>> {
        self.shell_listed_faces(shell, sink)
            .filter_map(|(orientation, face)| {
                let surface = self.face_surface_or_report(&face, sink)?;
                // The stored boundaries and the stored flag are ONE convention,
                // not two: see `absolute_bound_orientation`.
                let face_sense = orientation == face.same_sense;
                let boundaries = self.face_boundaries_or_report(&face, sink, |bound| {
                    self.face_bound_to_edge_uses(bound, face_sense, &surface, eidx_map, sink)
                });
                sink.kept(Cat::Face, 1);
                Some(CompressedTrimmedFace {
                    surface,
                    boundaries,
                    orientation: face_sense,
                })
            })
            .collect()
    }

    /// Constructs [`CompressedShell`] from a STEP `Shell`.
    ///
    /// # Example
    /// ```
    /// # fn main() -> anyhow::Result<()> {
    /// # use anyhow::anyhow;
    /// use monstertruck_io::step::load::{*, step_geometry::*};
    ///
    /// let step_string = include_str!(concat!(
    ///     env!("CARGO_MANIFEST_DIR"),
    ///     "/../resources/step/occt-cube.step",
    /// ));
    /// let table = Table::from_step(&step_string)?;
    /// let step_shell = table
    ///     .shell
    ///     .values()
    ///     .next()
    ///     .ok_or_else(|| anyhow!("STEP file contains no shell."))?;
    /// let cshell = table.to_compressed_shell(step_shell)?;
    /// assert_eq!(cshell.faces.len(), 6);
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_compressed_shell(
        &self,
        shell: &impl StepShell,
    ) -> Result<CompressedShell<Point3, Curve3D, Surface>, LoadError> {
        Ok(shell.to_compressed_shell(self)?)
    }

    /// [`Self::to_compressed_shell`] plus a truthful answer to "did I get
    /// everything?".
    ///
    /// # Why the un-suffixed method is not enough
    ///
    /// `to_compressed_shell` returns `Ok` for a shell that is short of faces,
    /// wires, edges or vertices, and always has. Measured over the corpus and the
    /// in-repo fixtures (spec 011 Phase 0): 253 faces and 15,444 shell faces lost
    /// corpus-wide, plus wires lost on four in-repo fixtures including the
    /// reference boolean fixture. The evidence was one line on stderr, or -- for a
    /// `VERTEX_LOOP` -- nothing at all.
    ///
    /// The report is ADDITIVE: this method does exactly the same conversion work
    /// and returns exactly the same shell.
    ///
    /// # Example
    /// ```
    /// # fn main() -> anyhow::Result<()> {
    /// # use anyhow::anyhow;
    /// use monstertruck_io::step::load::{*, report::*};
    ///
    /// let step_string = include_str!(concat!(
    ///     env!("CARGO_MANIFEST_DIR"),
    ///     "/../resources/step/occt-cube.step",
    /// ));
    /// let table = Table::from_step(step_string)?;
    /// let step_shell = table
    ///     .shell
    ///     .values()
    ///     .next()
    ///     .ok_or_else(|| anyhow!("STEP file contains no shell."))?;
    /// let (cshell, report) = table.to_compressed_shell_reported(step_shell)?;
    /// assert_eq!(cshell.faces.len(), 6);
    /// // The cube has no degenerate loops and no refused surfaces.
    /// assert!(report.is_lossless(), "{report}");
    /// assert_eq!(report.count(LossCategory::Face).listed, 6);
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_compressed_shell_reported(
        &self,
        shell: &impl StepShell,
    ) -> Result<(CompressedShell<Point3, Curve3D, Surface>, ShellLoadReport), LoadError> {
        Ok(shell.to_compressed_shell_reported(self)?)
    }

    /// Constructs `CompressedTrimmedShell` from `Shell` in STEP file while preserving
    /// exact face-local `ParameterCurve`s when they are present in the STEP data.
    pub fn to_compressed_trimmed_shell(
        &self,
        shell: &impl StepShell,
    ) -> Result<
        CompressedTrimmedShell<Point3, Curve3D, Surface, step_geometry::StepParameterCurve>,
        LoadError,
    > {
        Ok(shell.to_compressed_trimmed_shell(self)?)
    }

    /// [`Self::to_compressed_trimmed_shell`] plus the loss report. See
    /// [`Self::to_compressed_shell_reported`].
    pub fn to_compressed_trimmed_shell_reported(
        &self,
        shell: &impl StepShell,
    ) -> Result<(StepCompressedTrimmedShell, ShellLoadReport), LoadError> {
        Ok(shell.to_compressed_trimmed_shell_reported(self)?)
    }

    /// Constructs `CompressedShell`s from `ShellBasedSurfaceModel` in STEP file
    pub fn to_compressed_shells(
        &self,
        shells: &ShellBasedSurfaceModelHolder,
    ) -> Result<Vec<CompressedShell<Point3, Curve3D, Surface>>, LoadError> {
        let mut res = Vec::new();
        for place_holder in &shells.sbsm_boundary {
            let PlaceHolder::Ref(Name::Entity(idx)) = place_holder else {
                return Err("failed to reference an element of `sbsm_boundary`".into());
            };
            if let Some(shell) = self.shell.get(idx) {
                res.push(self.to_compressed_shell(shell)?);
            } else if let Some(oriented_shell) = self.oriented_shell.get(idx) {
                res.push(self.to_compressed_shell(oriented_shell)?);
            } else {
                return Err("failed to reference an element of `sbsm_boundary`".into());
            }
        }
        Ok(res)
    }

    /// Constructs `CompressedTrimmedShell`s from `ShellBasedSurfaceModel` in STEP file.
    pub fn to_compressed_trimmed_shells(
        &self,
        shells: &ShellBasedSurfaceModelHolder,
    ) -> Result<
        Vec<CompressedTrimmedShell<Point3, Curve3D, Surface, step_geometry::StepParameterCurve>>,
        LoadError,
    > {
        let mut res = Vec::new();
        for place_holder in &shells.sbsm_boundary {
            let PlaceHolder::Ref(Name::Entity(idx)) = place_holder else {
                return Err("failed to reference an element of `sbsm_boundary`".into());
            };
            if let Some(shell) = self.shell.get(idx) {
                res.push(self.to_compressed_trimmed_shell(shell)?);
            } else if let Some(oriented_shell) = self.oriented_shell.get(idx) {
                res.push(self.to_compressed_trimmed_shell(oriented_shell)?);
            } else {
                return Err("failed to reference an element of `sbsm_boundary`".into());
            }
        }
        Ok(res)
    }

    /// Constructs [`CompressedSolid`] from [`ManifoldSolidBrep`] in a STEP file.
    ///
    /// The result keeps the STEP `outer` shell as the first entry of
    /// `boundaries` and appends one entry per `void` (inner cavity).
    ///
    /// `Solid::extract` on the returned [`CompressedSolid`] requires the
    /// outer shell to satisfy [`ShellCondition::Closed`](monstertruck_topology::shell::ShellCondition::Closed). Many real-world
    /// STEP exports produce shells that are topologically *regular* but
    /// not strictly *oriented*; for those, prefer
    /// `to_compressed_trimmed_solid` and downstream meshing/healing.
    ///
    /// # Example
    /// ```
    /// # fn main() -> anyhow::Result<()> {
    /// # use anyhow::anyhow;
    /// use monstertruck_io::step::load::{*, step_geometry::*};
    ///
    /// let step_string = include_str!(concat!(
    ///     env!("CARGO_MANIFEST_DIR"),
    ///     "/../resources/step/occt-cube.step",
    /// ));
    /// let table = Table::from_step(&step_string)?;
    /// let step_solid = table
    ///     .manifold_solid_brep
    ///     .values()
    ///     .next()
    ///     .ok_or_else(|| anyhow!("STEP file contains no manifold solid B-rep."))?;
    /// let csolid = table.to_compressed_solid(step_solid)?;
    /// assert_eq!(csolid.boundaries.len(), 1);
    /// assert_eq!(csolid.boundaries[0].faces.len(), 6);
    /// # Ok(())
    /// # }
    /// ```
    pub fn to_compressed_solid(
        &self,
        solid: &ManifoldSolidBrepHolder,
    ) -> Result<CompressedSolid<Point3, Curve3D, Surface>, LoadError> {
        self.to_compressed_solid_reported(solid)
            .map(|(solid, _)| solid)
    }

    /// [`Self::to_compressed_solid`] plus ONE [`ShellLoadReport`] per entry of
    /// `boundaries`, positionally aligned with it -- so a caller can say which
    /// shell lost what, not merely that the solid lost something.
    ///
    /// See [`Self::to_compressed_shell_reported`] for why this exists.
    pub fn to_compressed_solid_reported(
        &self,
        solid: &ManifoldSolidBrepHolder,
    ) -> Result<(StepCompressedSolid, Vec<ShellLoadReport>), LoadError> {
        let PlaceHolder::Ref(Name::Entity(outer_idx)) = &solid.outer else {
            return Err("failed to reference `solid.outer`".into());
        };
        let outer_shell = if let Some(step_shell) = self.shell.get(outer_idx) {
            self.to_compressed_shell_reported(step_shell)
        } else if let Some(step_shell) = self.oriented_shell.get(outer_idx) {
            self.to_compressed_shell_reported(step_shell)
        } else {
            Err("failed to reference `solid.outer`".into())
        }?;
        let mut boundaries = vec![outer_shell.0];
        let mut reports = vec![outer_shell.1];
        for shell in &solid.voids {
            let PlaceHolder::Ref(Name::Entity(outer_idx)) = shell else {
                return Err("failed to reference an element of `solid.voids`".into());
            };
            let Some(oriented_shell) = self.oriented_shell.get(outer_idx) else {
                return Err("failed to reference an element of `solid.voids`".into());
            };
            let (shell, report) = self.to_compressed_shell_reported(oriented_shell)?;
            boundaries.push(shell);
            reports.push(report);
        }
        Ok((
            CompressedSolid {
                boundaries,
                id_allocator: None,
                attributes: None,
            },
            reports,
        ))
    }

    /// Constructs `CompressedTrimmedSolid` from `ManifoldSolidBrep` in STEP file.
    pub fn to_compressed_trimmed_solid(
        &self,
        solid: &ManifoldSolidBrepHolder,
    ) -> Result<
        CompressedTrimmedSolid<Point3, Curve3D, Surface, step_geometry::StepParameterCurve>,
        LoadError,
    > {
        let PlaceHolder::Ref(Name::Entity(outer_idx)) = &solid.outer else {
            return Err("failed to reference `solid.outer`".into());
        };
        let outer_shell = if let Some(step_shell) = self.shell.get(outer_idx) {
            self.to_compressed_trimmed_shell(step_shell)
        } else if let Some(step_shell) = self.oriented_shell.get(outer_idx) {
            self.to_compressed_trimmed_shell(step_shell)
        } else {
            Err("failed to reference `solid.outer`".into())
        }?;
        let mut boundaries = vec![outer_shell];
        for inner in &solid.voids {
            let PlaceHolder::Ref(Name::Entity(inner_idx)) = inner else {
                return Err("failed to reference a member of `solid.voids`".into());
            };
            if let Some(shell) = self.shell.get(inner_idx) {
                boundaries.push(self.to_compressed_trimmed_shell(shell)?);
            } else if let Some(oriented_shell) = self.oriented_shell.get(inner_idx) {
                boundaries.push(self.to_compressed_trimmed_shell(oriented_shell)?);
            } else {
                return Err("failed to reference a member of `solid.voids`".into());
            }
        }
        Ok(CompressedTrimmedSolid { boundaries })
    }

    /// Extracts [`Curve3D`] curves from a [`GeometricCurveSet`].
    ///
    /// Returns `None` if the holder cannot be resolved or yields no curves.
    /// Silently skips points and curves that fail conversion.
    pub fn to_curve3d_set(&self, gcs: &GeometricCurveSetHolder) -> Option<Vec<Curve3D>> {
        let owned = gcs.clone().into_owned(self).ok()?;
        let curves: Vec<Curve3D> = owned
            .elements
            .iter()
            .filter_map(|elem| match elem {
                GeometricSetSelect::Curve(curve) => Curve3D::try_from(curve.as_ref())
                    .map_err(|e| eprintln!("skipping curve in geometric_curve_set: {e}"))
                    .ok(),
                GeometricSetSelect::Point(_) => None,
            })
            .collect();
        if curves.is_empty() {
            None
        } else {
            Some(curves)
        }
    }
}

#[derive(Clone, Debug, derive_more::From)]
pub enum NodeMatrix {
    Identity,
    Transform(Box<ItemDefinedTransformation>),
}

pub use crate::step::common::PartAttributes;

pub type ProductEntity = NodeEntity<Vec<u64>, PartAttributes>;
pub type AssembleEntity = EdgeEntity<NodeMatrix, PartAttributes>;
pub type StepAssembly = Assembly<Vec<u64>, PartAttributes, NodeMatrix, PartAttributes>;

impl TryFrom<&NodeMatrix> for Matrix3 {
    type Error = StepConvertingError;
    fn try_from(value: &NodeMatrix) -> Result<Self, Self::Error> {
        match value {
            NodeMatrix::Identity => Ok(Self::identity()),
            NodeMatrix::Transform(trans) => (&**trans).try_into(),
        }
    }
}

impl TryFrom<&NodeMatrix> for Matrix4 {
    type Error = StepConvertingError;
    fn try_from(value: &NodeMatrix) -> Result<Self, Self::Error> {
        match value {
            NodeMatrix::Identity => Ok(Self::identity()),
            NodeMatrix::Transform(trans) => (&**trans).try_into(),
        }
    }
}

impl Table {
    fn product_node_entity(
        &self,
        pds_idx: u64,
        pd: &ProductDefinitionHolder,
    ) -> Result<ProductEntity, StepConvertingError> {
        let PlaceHolder::Ref(Name::Entity(pdf_idx)) = &pd.formation else {
            return Err("failed to reference `product_definition.formation`".into());
        };
        let Some(pdf) = self.product_definition_formation.get(pdf_idx) else {
            return Err("failed to reference `prouct_definition_formation`".into());
        };
        let PlaceHolder::Ref(Name::Entity(p_idx)) = &pdf.of_product else {
            return Err("failed to reference `product_definition_formation.of_product`".into());
        };
        let Some(product) = self.product.get(p_idx) else {
            return Err("failed to reference `product`".into());
        };
        let attrs = PartAttributes {
            id: product.id.clone(),
            name: product.name.clone(),
            // `PartAttributes::description` is a display field shared with the
            // save side and is a plain `String`, so an UNSET STEP description
            // flattens to `""` here -- the one place the `Option` added for
            // spec 011 T6 loses the unset/empty distinction. Deliberate: keeping
            // it would change what `save` emits for these parts, which is a
            // separate decision from making the load stop dropping the record.
            description: product.description.clone().unwrap_or_default(),
            // Filled in below, once the representation has been RESOLVED. Set
            // here it would be a promise; set there it is a fact.
            shape_representation: None,
        };

        let Some(sdr) = self.shape_definition_representation.values().find(|sdr| {
            let &PlaceHolder::Ref(Name::Entity(idx)) = &sdr.definition else {
                return false;
            };
            pds_idx == idx
        }) else {
            return Err("failed to find `shape_definition_representation` corresp. to `product_definition_shape`".into());
        };
        let PlaceHolder::Ref(Name::Entity(sr_idx)) = &sdr.used_representation else {
            return Err(
                "failed to reference `shape_definition_representation.used_representation`".into(),
            );
        };
        let Some(sr) = self.shape_representation.get(sr_idx) else {
            return Err("failed to reference `shape_representation`".into());
        };
        // The id this whole function already had and used to discard. It is
        // recorded only on the `get` success path above, so
        // `PartAttributes::shape_representation` is `Some(id)` ONLY for an id
        // the table demonstrably holds -- there is no "carried but dangling"
        // state for a caller to have to guard against.
        let attrs = PartAttributes {
            shape_representation: Some(*sr_idx),
            ..attrs
        };
        let Some(shape) = sr
            .items
            .iter()
            .map(|place_holder| {
                if let &PlaceHolder::Ref(Name::Entity(item_idx)) = place_holder {
                    Some(item_idx)
                } else {
                    None
                }
            })
            .collect::<Option<Vec<_>>>()
        else {
            return Err("failed to reference an element of `shape_representation.items`".into());
        };

        Ok(ProductEntity { shape, attrs })
    }

    fn assy_node_entity(
        &self,
        pds_idx: u64,
        next_assy: &NextAssemblyUsageOccurrenceHolder,
    ) -> Result<(AssembleEntity, (u64, u64)), StepConvertingError> {
        let &PlaceHolder::Ref(Name::Entity(parent_idx)) = &next_assy.relating_product_definition
        else {
            return Err("failed to reference the parent node".into());
        };
        let &PlaceHolder::Ref(Name::Entity(child_idx)) = &next_assy.related_product_definition
        else {
            return Err("failed to reference the child node".into());
        };

        let attrs = PartAttributes {
            id: next_assy.id.clone(),
            name: next_assy.name.clone(),
            description: next_assy.description.clone(),
            // An EDGE is a `NEXT_ASSEMBLY_USAGE_OCCURRENCE`, not a product with
            // a shape representation of its own. This is one of the two
            // populations that legitimately carry no id; see
            // `PartAttributes::shape_representation`.
            shape_representation: None,
        };

        let Some(cdsr) = self
            .context_dependent_shape_representation
            .values()
            .find(|cdsr| {
                let &PlaceHolder::Ref(Name::Entity(idx)) = &cdsr.represented_product_relation
                else {
                    return false;
                };
                pds_idx == idx
            })
        else {
            return Err("".into());
        };

        let PlaceHolder::Ref(Name::Entity(srrwt_idx)) = &cdsr.representation_relation else {
            return Err("failed to reference `context_dependent_shape_representation.representation_relation`".into());
        };

        let Some(srrwt) = self
            .shape_representation_relationship_with_transformation
            .get(srrwt_idx)
        else {
            return Err("failed to reference `shape_representation_relationship`".into());
        };
        let idtf = srrwt.transformation_operator.clone().into_owned(self)?;

        let entity = AssembleEntity {
            matrix: NodeMatrix::Transform(idtf.into()),
            attrs,
        };

        Ok((entity, (parent_idx, child_idx)))
    }

    pub fn step_assy(&self) -> Result<StepAssembly, LoadError> {
        let mut product_entities = Vec::<ProductEntity>::new();
        let mut indices_map = HashMap::<u64, usize>::new();
        let mut assy_nodes = Vec::<(AssembleEntity, (u64, u64))>::new();
        for (&pds_idx, pds) in &self.product_definition_shape {
            let &PlaceHolder::Ref(Name::Entity(idx)) = &pds.definition else {
                return Err("failed to reference `product_definition_shape.definition`".into());
            };
            if let Some(pd) = self.product_definition.get(&idx) {
                product_entities.push(self.product_node_entity(pds_idx, pd)?);
                indices_map.insert(idx, product_entities.len() - 1);
            } else if let Some(next_assy) = self.next_assembly_usage_occurrence.get(&idx) {
                assy_nodes.push(self.assy_node_entity(pds_idx, next_assy)?);
            }
        }

        let adjacency = assy_nodes
            .into_iter()
            .map(|(entity, (from, to))| {
                let from = *indices_map.get(&from)?;
                let to = *indices_map.get(&to)?;
                Some((from, to, entity))
            })
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| LoadError::from("failed to reference `product_definition_shape`."))?;

        StepAssembly::try_from_adjacency(product_entities, adjacency)
            .ok_or_else(|| LoadError::from("maybe the assembly graph has a cycle."))
    }

    /// The representation a loaded assembly node came from -- the id it CARRIES
    /// if it has one, and the reverse match otherwise.
    ///
    /// This is what a caller holding a [`ProductEntity`] should ask. Since spec
    /// 012 the node carries the id directly
    /// ([`PartAttributes::shape_representation`]), which is exact by
    /// construction: [`Table::step_assy`] records it on the same success path
    /// that resolved the record. [`Table::shape_representation_of_items`] stays
    /// as the fallback for the nodes that legitimately carry none -- an
    /// assembly this crate did not load, i.e. one an application or the save
    /// side built, where the field is `Default`'s `None`.
    ///
    /// **Measured on the two Scania files (`scania_assembly_graphs_now_load_and_walk`):
    /// the two routes agree node for node, and the id route reaches the same
    /// 832 of 832 and 254 of 254 solids.** A disagreement would be a finding,
    /// not a rounding error, and that pin is where it would surface.
    pub fn shape_representation_of_node(&self, node: &ProductEntity) -> Option<u64> {
        node.attrs
            .shape_representation
            .or_else(|| self.shape_representation_of_items(&node.shape))
    }

    /// The representation whose `items` are EXACTLY `items` -- the bridge from a
    /// [`ProductEntity::shape`] back to the `SHAPE_REPRESENTATION` it came from.
    ///
    /// **Since spec 012 this is the FALLBACK, not the primary route.** A node
    /// loaded by [`Table::step_assy`] carries its representation id in
    /// [`PartAttributes::shape_representation`]; prefer
    /// [`Table::shape_representation_of_node`], which reads the field and only
    /// falls back to this search when there is none. This search remains public
    /// because it answers a question the field cannot: which representation, if
    /// any, holds an item list a caller constructed itself.
    ///
    /// [`Table::step_assy`] fills a node's `shape` from
    /// `shape_definition_representation.used_representation.items`. Before spec
    /// 012 it discarded the representation's own id, so a caller holding an
    /// assembly node could not ask anything else about that representation, and
    /// this inverse was the only bridge.
    ///
    /// Matching is on the whole id list, not on containment, and that is not
    /// caution for its own sake: **measured, `coffy.step` lists `#15756` in BOTH
    /// `#15521` and `#261`**, so a containment rule would answer that item's
    /// owner ambiguously. Whole-list matching is unique on every file measured --
    /// including the two Scania ones, where no representation item is shared at
    /// all (0 of 1,571 and 0 of 1,160). `None` when nothing matches; the FIRST
    /// match in ascending id order if several somehow do.
    ///
    /// `items` that are not all entity references cannot match anything, because
    /// `ProductEntity::shape` only exists when every item was one.
    pub fn shape_representation_of_items(&self, items: &[u64]) -> Option<u64> {
        let mut best: Option<u64> = None;
        for (&id, representation) in &self.shape_representation {
            if representation.items.len() != items.len() {
                continue;
            }
            let same = representation.items.iter().zip(items).all(
                |(held, want)| matches!(held, PlaceHolder::Ref(Name::Entity(held)) if held == want),
            );
            if same && best.is_none_or(|previous| id < previous) {
                best = Some(id);
            }
        }
        best
    }

    /// The solids a part's representation reaches through the **non-transforming**
    /// `SHAPE_REPRESENTATION_RELATIONSHIP` hop, plus what the hop lost.
    ///
    /// # Why this is not [`ProductEntity::shape`]
    ///
    /// `shape` means "the items directly in this node's representation", and on
    /// real assemblies that is honestly what it holds: **measured on
    /// `Scania-8x4.stp` it is `{AXIS2_PLACEMENT_3D: 695, SHELL_BASED_SURFACE_MODEL:
    /// 44}` and on `Scania-Engine-V8-XT-Turbo.step` `{AXIS2_PLACEMENT_3D: 906}` --
    /// zero solids.** The solids live in a second representation, an
    /// `ADVANCED_BREP_SHAPE_REPRESENTATION`, and this hop is what reaches them:
    /// **832 of 832 and 254 of 254**, exactly the 1,086 solids spec 011 T6 found
    /// with no placement.
    ///
    /// Making `shape` chase the relationship silently would make an existing
    /// method's answer depend on a hop the caller cannot see -- and `shape` is
    /// also what the save side round-trips, where the direct meaning is the
    /// right one. So the hop is a separate, visible call. Promoting it to the
    /// default later is easy; un-conflating two meanings after the fact is not.
    ///
    /// # Non-transforming only
    ///
    /// A transforming relationship is a complex record carrying
    /// `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`, and it lands in
    /// [`Table::shape_representation_relationship_with_transformation`], a
    /// different map that [`Table::step_assy`] already walks to build the
    /// assembly's edge matrices. Following it here too would apply the same
    /// placement twice. This method reads ONLY
    /// [`Table::shape_representation_relationship`], which by construction holds
    /// simple records exclusively -- a transformation cannot be attached to one.
    ///
    /// Only the `rep_1 == shape_representation -> rep_2` direction is followed.
    /// **Measured: the reverse direction carries zero solids on both Scania
    /// files** (all 188 and 158 relationships have the part's own representation
    /// as `rep_1`), so following it would add no reach and would let one part
    /// claim another's geometry.
    ///
    /// # What the report says
    ///
    /// [`LossCategory::Representation`] counts relationships found versus
    /// relationships whose `rep_2` the table actually holds;
    /// [`LossCategory::Solid`] counts the items of those representations versus
    /// the ones that are solids. Read
    /// [`LossReason::RepresentationItemNotASolid`] before treating a lossy report
    /// as a defect: a representation listing its own `AXIS2_PLACEMENT_3D` beside
    /// its solid is normal, and is the usual reason a file reports items lost
    /// while losing no solid at all.
    ///
    /// Ids come back in ascending relationship id, then representation item
    /// order, and are NOT deduplicated -- `kept` and the returned length always
    /// agree.
    ///
    /// # Example -- how a viewer opts in
    ///
    /// The hop only has work to do when a part's own representation RELATES to
    /// the representation holding the solid; on a file whose parts carry their
    /// solids directly it correctly reaches nothing and reports no loss, which
    /// is what the fixture below shows.
    ///
    /// ```
    /// # fn main() -> anyhow::Result<()> {
    /// use monstertruck_io::step::load::{report::*, Table};
    ///
    /// let bytes = include_bytes!(concat!(
    ///     env!("CARGO_MANIFEST_DIR"),
    ///     "/../resources/step/occt-assy.step",
    /// ));
    /// let table = Table::from_step_bytes(bytes)?;
    /// let assembly = table.step_assy()?;
    ///
    /// let mut solids = Vec::new();
    /// for node in assembly.all_nodes() {
    ///     // `node.shape()` is the items DIRECTLY in this node's representation
    ///     // -- on a real assembly, placements. The hop is the extra call.
    ///     let Some(representation) = table.shape_representation_of_items(node.shape()) else {
    ///         continue;
    ///     };
    ///     let (reached, report) = table.solids_via_shape_relationship(representation);
    ///     // The report says what the hop could NOT deliver, and why.
    ///     assert_eq!(report.lost_for(LossReason::RelatedRepresentationUnresolved), 0);
    ///     solids.extend(reached);
    /// }
    /// // Every relationship this file does have resolved, so nothing was lost.
    /// assert!(solids.iter().all(|id| *id > 0));
    /// # Ok(())
    /// # }
    /// ```
    pub fn solids_via_shape_relationship(
        &self,
        shape_representation: u64,
    ) -> (Vec<u64>, ShellLoadReport) {
        let mut report = ShellLoadReport::default();
        let mut solids = Vec::new();

        let mut relationships: Vec<_> = self
            .shape_representation_relationship
            .iter()
            .filter(|(_, srr)| {
                matches!(&srr.rep_1, PlaceHolder::Ref(Name::Entity(rep_1)) if *rep_1 == shape_representation)
            })
            .collect();
        relationships.sort_unstable_by_key(|(id, _)| **id);

        for (&srr_id, srr) in relationships {
            report.note_listed(Cat::Representation, 1);
            let PlaceHolder::Ref(Name::Entity(rep_2)) = &srr.rep_2 else {
                report.note_lost(
                    Cat::Representation,
                    Why::RelatedRepresentationUnresolved,
                    Some(srr_id),
                    Some("`rep_2` is not an entity reference".to_owned()),
                );
                continue;
            };
            let Some(representation) = self.shape_representation.get(rep_2) else {
                report.note_lost(
                    Cat::Representation,
                    Why::RelatedRepresentationUnresolved,
                    Some(*rep_2),
                    Some(self.describe_unheld(*rep_2)),
                );
                continue;
            };
            report.note_kept(Cat::Representation, 1);

            for item in &representation.items {
                report.note_listed(Cat::Solid, 1);
                let PlaceHolder::Ref(Name::Entity(item_id)) = item else {
                    report.note_lost(
                        Cat::Solid,
                        Why::RepresentationItemNotASolid,
                        Some(*rep_2),
                        Some("the item is not an entity reference".to_owned()),
                    );
                    continue;
                };
                if self.manifold_solid_brep.contains_key(item_id) {
                    report.note_kept(Cat::Solid, 1);
                    solids.push(*item_id);
                } else {
                    report.note_lost(
                        Cat::Solid,
                        Why::RepresentationItemNotASolid,
                        Some(*item_id),
                        Some(self.describe_unheld(*item_id)),
                    );
                }
            }
        }

        (solids, report)
    }

    /// One line naming what `id` actually is, for a loss detail. The entity NAME
    /// is the whole point -- "not a solid" without it sends a reader back to the
    /// file with nothing but an id.
    fn describe_unheld(&self, id: u64) -> String {
        if let Some(dummy) = self.dummy.get(&id) {
            // `Table::dummy` stores `format!("{record:?}")`, whose derived
            // `Debug` puts the entity name in the first quoted field.
            // Diagnostic convenience, not a contract: an unparseable shape
            // degrades to the record text itself.
            let name = dummy.record.split('"').nth(1).unwrap_or(&dummy.record);
            format!("#{id} is `{name}`, an entity type the loader has no arm for")
        } else if self.axis2_placement_3d.contains_key(&id) {
            format!("#{id} is an AXIS2_PLACEMENT_3D")
        } else if self.shell_based_surface_model.contains_key(&id) {
            format!("#{id} is a SHELL_BASED_SURFACE_MODEL")
        } else if self.geometric_curve_set.contains_key(&id) {
            format!("#{id} is a GEOMETRIC_CURVE_SET")
        } else {
            format!("#{id} is in no table map")
        }
    }
}

/// A STEP entity a compressed shell can be built from.
///
/// The `*_reported` methods are the PRIMARY ones: they return the report of what
/// the conversion kept and lost. The un-suffixed methods are provided defaults
/// that discard it, so every existing caller keeps working unchanged.
pub trait StepShell {
    /// Convert, and report what was kept and lost.
    fn to_compressed_shell_reported(
        &self,
        table: &Table,
    ) -> Result<(CompressedShell<Point3, Curve3D, Surface>, ShellLoadReport), StepConvertingError>;

    /// Convert preserving exact trim curves, and report what was kept and lost.
    fn to_compressed_trimmed_shell_reported(
        &self,
        table: &Table,
    ) -> Result<(StepCompressedTrimmedShell, ShellLoadReport), StepConvertingError>;

    /// Convert, DISCARDING the loss report.
    ///
    /// A lossy conversion is indistinguishable from a clean one through this
    /// method -- that is exactly the defect spec 011 T7 exists to end. Prefer
    /// [`Self::to_compressed_shell_reported`] in new code.
    fn to_compressed_shell(
        &self,
        table: &Table,
    ) -> Result<CompressedShell<Point3, Curve3D, Surface>, StepConvertingError> {
        self.to_compressed_shell_reported(table)
            .map(|(shell, _)| shell)
    }

    /// Convert preserving exact trim curves, DISCARDING the loss report.
    /// Prefer [`Self::to_compressed_trimmed_shell_reported`] in new code.
    fn to_compressed_trimmed_shell(
        &self,
        table: &Table,
    ) -> Result<
        CompressedTrimmedShell<Point3, Curve3D, Surface, step_geometry::StepParameterCurve>,
        StepConvertingError,
    > {
        self.to_compressed_trimmed_shell_reported(table)
            .map(|(shell, _)| shell)
    }
}

impl StepShell for ShellHolder {
    fn to_compressed_shell_reported(
        &self,
        table: &Table,
    ) -> Result<(CompressedShell<Point3, Curve3D, Surface>, ShellLoadReport), StepConvertingError>
    {
        let sink = LossSink::default();
        let (vertices, vidx_map) = table.shell_vertices(self, &sink);
        let (edges, eidx_map) = table.shell_edges(self, &vidx_map, &sink);
        let faces = table.shell_faces(self, &eidx_map, &sink);
        Ok((
            CompressedShell {
                vertices,
                edges,
                faces,
                vertex_stable_ids: None,
                edge_stable_ids: None,
                face_stable_ids: None,
            },
            sink.into_report(),
        ))
    }

    fn to_compressed_trimmed_shell_reported(
        &self,
        table: &Table,
    ) -> Result<
        (
            CompressedTrimmedShell<Point3, Curve3D, Surface, step_geometry::StepParameterCurve>,
            ShellLoadReport,
        ),
        StepConvertingError,
    > {
        let sink = LossSink::default();
        let (vertices, vidx_map) = table.shell_vertices(self, &sink);
        let (edges, eidx_map) = table.shell_edges(self, &vidx_map, &sink);
        let faces = table.shell_trimmed_faces(self, &eidx_map, &sink);
        Ok((
            CompressedTrimmedShell {
                vertices,
                edges,
                faces,
            },
            sink.into_report(),
        ))
    }
}

impl StepShell for OrientedShellHolder {
    fn to_compressed_shell_reported(
        &self,
        table: &Table,
    ) -> Result<(CompressedShell<Point3, Curve3D, Surface>, ShellLoadReport), StepConvertingError>
    {
        let PlaceHolder::Ref(Name::Entity(idx)) = &self.shell_element else {
            return Err("failed to reference shell".into());
        };
        let Some(shell) = table.shell.get(idx) else {
            return Err("failed to reference shell".into());
        };
        let (mut res, report) = shell.to_compressed_shell_reported(table)?;
        if !self.orientation {
            for face in &mut res.faces {
                face.orientation = !face.orientation;
            }
        }
        Ok((res, report))
    }

    fn to_compressed_trimmed_shell_reported(
        &self,
        table: &Table,
    ) -> Result<
        (
            CompressedTrimmedShell<Point3, Curve3D, Surface, step_geometry::StepParameterCurve>,
            ShellLoadReport,
        ),
        StepConvertingError,
    > {
        let PlaceHolder::Ref(Name::Entity(idx)) = &self.shell_element else {
            return Err("failed to reference shell".into());
        };
        let Some(shell) = table.shell.get(idx) else {
            return Err("failed to reference shell".into());
        };
        let (mut res, report) = shell.to_compressed_trimmed_shell_reported(table)?;
        if !self.orientation {
            res.faces.iter_mut().for_each(|face| {
                face.orientation = !face.orientation;
            });
        }
        Ok((res, report))
    }
}

impl StepShell for ShellAnyHolder {
    fn to_compressed_shell_reported(
        &self,
        table: &Table,
    ) -> Result<(CompressedShell<Point3, Curve3D, Surface>, ShellLoadReport), StepConvertingError>
    {
        match self {
            ShellAnyHolder::OrientedShell(shell) => shell.to_compressed_shell_reported(table),
            ShellAnyHolder::Shell(shell) => shell.to_compressed_shell_reported(table),
        }
    }

    fn to_compressed_trimmed_shell_reported(
        &self,
        table: &Table,
    ) -> Result<
        (
            CompressedTrimmedShell<Point3, Curve3D, Surface, step_geometry::StepParameterCurve>,
            ShellLoadReport,
        ),
        StepConvertingError,
    > {
        match self {
            ShellAnyHolder::OrientedShell(shell) => {
                shell.to_compressed_trimmed_shell_reported(table)
            }
            ShellAnyHolder::Shell(shell) => shell.to_compressed_trimmed_shell_reported(table),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::step::load::step_geometry::SurfaceCurveRepresentation as StepSurfaceCurveRepresentation;
    use std::f64::consts::TAU;

    /// Loads `occt-cylinder.step` and builds its trimmed shell. The
    /// cylinder fixture carries `PCURVE` entities on both planar and
    /// cylindrical surfaces, so this exercises the `ToSameGeometry<Curve2D>`
    /// load path end-to-end -- the unit tests in `step_geometry/geom_impls`
    /// only verify the impls in isolation. At least one of the resulting
    /// trim curves must be present (`Some(_)`) and at least one of them
    /// must contain a 2D curve variant that comes through the conversion.
    #[test]
    fn pcurve_load_path_populates_trim_curves() -> anyhow::Result<()> {
        let step_string = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../resources/step/occt-cylinder.step",
        ));
        let table = crate::step::load::Table::from_step(step_string)?;
        let step_shell =
            table.shell.values().next().ok_or_else(|| {
                anyhow::anyhow!("the cylinder fixture must contain a STEP shell.")
            })?;
        let trimmed = table.to_compressed_trimmed_shell(step_shell)?;
        let total_edge_uses: usize = trimmed
            .faces
            .iter()
            .flat_map(|face| &face.boundaries)
            .map(|wire| wire.len())
            .sum();
        let trim_curves_present: usize = trimmed
            .faces
            .iter()
            .flat_map(|face| &face.boundaries)
            .flat_map(|wire| wire.iter())
            .filter(|edge_use| edge_use.trim_curve.is_some())
            .count();
        assert!(
            total_edge_uses > 0,
            "the cylinder fixture should have at least one edge-use after trimmed loading.",
        );
        assert!(
            trim_curves_present > 0,
            "at least one edge-use should carry a trim curve. \
             total edge-uses: {total_edge_uses}.",
        );
        Ok(())
    }

    fn cylinder_surface() -> Surface {
        let axis = Vector3::unit_z();
        let center = Point3::origin();
        let point = Point3::new(1.0, 0.0, 0.0);
        let line = Line(point, point + axis);
        Surface::ElementarySurface(ElementarySurface::CylindricalSurface(Processor::new(
            RevolutionSurface::by_revolution(line, center, axis),
        )))
    }

    fn line_pcurve(surface: &Surface, u: f64) -> step_geometry::StepParameterCurve {
        step_geometry::StepParameterCurve::new(
            Box::new(Curve2D::Line(Line(
                Point2::new(u, 0.0),
                Point2::new(u, 1.0),
            ))),
            Box::new(surface.clone()),
        )
    }

    fn seam_curve(surface: &Surface) -> Curve3D {
        let leader = Curve3D::Line(Line(surface.subs(0.0, 0.0), surface.subs(0.0, 1.0)));
        Curve3D::SurfaceCurve(SurfaceCurve3D::new(
            StepSurfaceCurveKind::SeamCurve,
            Box::new(leader),
            vec![
                SurfaceCurveAssociatedGeometry::ParameterCurve(line_pcurve(surface, 0.0)),
                SurfaceCurveAssociatedGeometry::ParameterCurve(line_pcurve(surface, TAU)),
            ],
            StepSurfaceCurveRepresentation::ParameterCurve1,
        ))
    }

    #[test]
    fn seam_curve_opposite_orientations_use_opposite_parameter_curves() {
        let surface = cylinder_surface();
        let curve = seam_curve(&surface);

        let forward = Table::exact_trim_curve_on(&curve, &surface, true)
            .expect("forward trim curve should exist");
        let backward = Table::exact_trim_curve_on(&curve, &surface, false)
            .expect("backward trim curve should exist");

        let forward_start = forward.curve().subs(forward.curve().range_tuple().0);
        let backward_start = backward.curve().subs(backward.curve().range_tuple().0);

        assert!(forward_start.x.near(&0.0));
        assert!(backward_start.x.near(&TAU));
    }
}
