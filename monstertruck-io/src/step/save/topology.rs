use super::{Result, *};
trait StepAssociatedEntity {
    fn fmt(&self, idx: usize, formatter: &mut Formatter<'_>) -> Result;
    fn step_length(&self) -> usize;
}

impl<T> StepAssociatedEntity for T
where T: StepFormat + StepLength
{
    fn fmt(&self, idx: usize, formatter: &mut Formatter<'_>) -> Result {
        StepFormat::fmt(self, idx, formatter)
    }

    fn step_length(&self) -> usize { StepLength::step_length(self) }
}

enum StepAssociatedGeometry<'a> {
    ExactParameterCurve(&'a dyn StepAssociatedEntity),
    /// Reference to an already-emitted surface entity (the adjacent face's
    /// surface). It emits nothing and claims no entity index; the enclosing
    /// `SURFACE_CURVE` argument list points at the stored index directly,
    /// instead of re-emitting the whole surface once per bounding edge.
    SurfaceRef(usize),
}

impl StepFormat for StepAssociatedGeometry<'_> {
    fn fmt(&self, idx: usize, formatter: &mut Formatter<'_>) -> Result {
        match self {
            Self::ExactParameterCurve(curve) => curve.fmt(idx, formatter),
            Self::SurfaceRef(_) => Ok(()),
        }
    }
}

impl StepLength for StepAssociatedGeometry<'_> {
    fn step_length(&self) -> usize {
        match self {
            Self::ExactParameterCurve(curve) => curve.step_length(),
            Self::SurfaceRef(_) => 0,
        }
    }
}

struct StepFace<'a, S> {
    boundaries: Vec<Vec<CompressedEdgeIndex>>,
    orientation: bool,
    surface: &'a S,
}

/// An edge-to-geometry association captured before surface entity indices are
/// assigned. [`StepShell::from_parts`] resolves each [`Self::FaceSurface`] to a
/// [`StepAssociatedGeometry::SurfaceRef`] once the face surface indices exist,
/// so a surface is emitted once (as the face geometry) and referenced, not
/// duplicated, per bounding edge.
enum EdgeAssociationSource<'a> {
    ExactParameterCurve(&'a dyn StepAssociatedEntity),
    /// Position of the adjacent face in the shell's face list.
    FaceSurface(usize),
}

struct StepSurfaceCurve<'a, C> {
    leader: &'a C,
    associated_geometry: Vec<StepAssociatedGeometry<'a>>,
}

impl<C> StepFormat for StepSurfaceCurve<'_, C>
where C: StepFormat + StepLength
{
    fn fmt(&self, idx: usize, formatter: &mut Formatter<'_>) -> Result {
        let leader_idx = idx + 1;
        let (associated_indices, _) = self.associated_geometry.iter().fold(
            (
                Vec::<usize>::with_capacity(self.associated_geometry.len()),
                leader_idx + self.leader.step_length(),
            ),
            |(mut indices, cursor), entry| match entry {
                // A referenced surface contributes the referenced index and no
                // cursor advance -- nothing new is emitted for it.
                StepAssociatedGeometry::SurfaceRef(surface_idx) => {
                    indices.push(*surface_idx);
                    (indices, cursor)
                }
                _ => {
                    indices.push(cursor);
                    (indices, cursor + StepLength::step_length(entry))
                }
            },
        );
        formatter.write_fmt(format_args!(
            "#{idx} = SURFACE_CURVE('', #{leader_idx}, {associated_geometry}, .CURVE_3D.);\n",
            associated_geometry = IndexSliceDisplay(associated_indices.iter().copied()),
        ))?;
        StepFormat::fmt(self.leader, leader_idx, formatter)?;
        self.associated_geometry
            .iter()
            .zip(associated_indices)
            .try_for_each(|(entry, entry_idx)| StepFormat::fmt(entry, entry_idx, formatter))
    }
}

impl<C> StepLength for StepSurfaceCurve<'_, C>
where C: StepLength
{
    fn step_length(&self) -> usize {
        1 + self.leader.step_length()
            + self
                .associated_geometry
                .iter()
                .map(StepLength::step_length)
                .sum::<usize>()
    }
}

impl<C> StepCurve for StepSurfaceCurve<'_, C>
where C: StepCurve
{
    fn same_sense(&self) -> bool { self.leader.same_sense() }
}

enum StepEdgeGeometry<'a, C> {
    Curve(&'a C),
    SurfaceCurve(StepSurfaceCurve<'a, C>),
}

impl<C> StepFormat for StepEdgeGeometry<'_, C>
where C: StepFormat + StepLength
{
    fn fmt(&self, idx: usize, formatter: &mut Formatter<'_>) -> Result {
        match self {
            Self::Curve(curve) => StepFormat::fmt(curve, idx, formatter),
            Self::SurfaceCurve(curve) => StepFormat::fmt(curve, idx, formatter),
        }
    }
}

impl<C> StepLength for StepEdgeGeometry<'_, C>
where C: StepLength
{
    fn step_length(&self) -> usize {
        match self {
            Self::Curve(curve) => curve.step_length(),
            Self::SurfaceCurve(curve) => curve.step_length(),
        }
    }
}

impl<C> StepCurve for StepEdgeGeometry<'_, C>
where C: StepCurve
{
    fn same_sense(&self) -> bool {
        match self {
            Self::Curve(curve) => curve.same_sense(),
            Self::SurfaceCurve(curve) => curve.same_sense(),
        }
    }
}

pub(super) struct StepShell<'a, P, C, S> {
    vertices: &'a [P],
    edges: &'a [CompressedEdge<C>],
    faces: Vec<StepFace<'a, S>>,
    idx: usize,
    face_indices: Vec<usize>,
    ep_edges: usize,
    ep_vertices: usize,
    surface_indices: Vec<usize>,
    edge_geometries: Vec<StepEdgeGeometry<'a, C>>,
    curve_indices: Vec<usize>,
    ep_points: usize,
    is_open: bool,
}

impl<'a, P, C, S> StepShell<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn new_curve3d_only(shell: &'a CompressedShell<P, C, S>, idx: usize, is_open: bool) -> Self {
        let faces = shell
            .faces
            .iter()
            .map(|face| StepFace {
                boundaries: face.boundaries.clone(),
                orientation: face.orientation,
                surface: &face.surface,
            })
            .collect::<Vec<_>>();
        let edge_associations = std::iter::repeat_with(Vec::<EdgeAssociationSource<'a>>::new)
            .take(shell.edges.len())
            .collect::<Vec<_>>();
        Self::from_parts(
            &shell.vertices,
            &shell.edges,
            faces,
            edge_associations,
            idx,
            is_open,
        )
    }

    fn new(shell: &'a CompressedShell<P, C, S>, idx: usize, is_open: bool) -> Self {
        let faces = shell
            .faces
            .iter()
            .map(|face| StepFace {
                boundaries: face.boundaries.clone(),
                orientation: face.orientation,
                surface: &face.surface,
            })
            .collect::<Vec<_>>();
        let mut edge_associations = std::iter::repeat_with(Vec::<EdgeAssociationSource<'a>>::new)
            .take(shell.edges.len())
            .collect::<Vec<_>>();
        faces.iter().enumerate().for_each(|(face_pos, face)| {
            face.boundaries.iter().for_each(|wire| {
                wire.iter().for_each(|ce| {
                    if let Some(associations) = edge_associations.get_mut(ce.index) {
                        associations.push(EdgeAssociationSource::FaceSurface(face_pos));
                    }
                });
            });
        });
        Self::from_parts(
            &shell.vertices,
            &shell.edges,
            faces,
            edge_associations,
            idx,
            is_open,
        )
    }
}

impl<'a, P, C, S> StepShell<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn new_trimmed<T>(
        shell: &'a CompressedTrimmedShell<P, C, S, T>,
        idx: usize,
        is_open: bool,
    ) -> Self
    where
        T: StepFormat + StepLength,
    {
        let faces = shell
            .faces
            .iter()
            .map(|face| StepFace {
                boundaries: face
                    .boundaries
                    .iter()
                    .map(|wire| {
                        wire.iter()
                            .map(
                                |CompressedEdgeUse {
                                     index, orientation, ..
                                 }| {
                                    CompressedEdgeIndex {
                                        index: *index,
                                        orientation: *orientation,
                                    }
                                },
                            )
                            .collect()
                    })
                    .collect(),
                orientation: face.orientation,
                surface: &face.surface,
            })
            .collect::<Vec<_>>();
        let mut edge_associations = std::iter::repeat_with(Vec::<EdgeAssociationSource<'a>>::new)
            .take(shell.edges.len())
            .collect::<Vec<_>>();
        shell.faces.iter().enumerate().for_each(|(face_pos, face)| {
            face.boundaries.iter().for_each(|wire| {
                wire.iter().for_each(|edge_use| {
                    let association = edge_use
                        .trim_curve
                        .as_ref()
                        .map(|trim_curve| EdgeAssociationSource::ExactParameterCurve(trim_curve))
                        .unwrap_or(EdgeAssociationSource::FaceSurface(face_pos));
                    edge_associations[edge_use.index].push(association);
                });
            });
        });
        Self::from_parts(
            &shell.vertices,
            &shell.edges,
            faces,
            edge_associations,
            idx,
            is_open,
        )
    }

    fn from_parts(
        vertices: &'a [P],
        edges: &'a [CompressedEdge<C>],
        faces: Vec<StepFace<'a, S>>,
        mut edge_associations: Vec<Vec<EdgeAssociationSource<'a>>>,
        idx: usize,
        is_open: bool,
    ) -> Self {
        let mut cursor = idx + 1;
        let face_indices = faces
            .iter()
            .map(|face| {
                let res = cursor;
                cursor += match face.boundaries.is_empty() {
                    // `advanced_face`, `face_bound`, `vertex_loop`,
                    // `vertex_point`, `point_on_surface`.
                    true => 5,
                    false => {
                        // One `advanced_face` plus, per boundary, a `face_bound`,
                        // an `edge_loop`, and one `oriented_edge` per edge.
                        1 + face
                            .boundaries
                            .iter()
                            .map(|boundary| 2 + boundary.len())
                            .sum::<usize>()
                    }
                };
                res
            })
            .collect::<Vec<_>>();
        let ep_edges = cursor;
        let ep_vertices = ep_edges + edges.len();
        cursor = ep_vertices + vertices.len();
        let surface_indices = faces
            .iter()
            .map(|face| {
                let res = cursor;
                cursor += face.surface.step_length();
                res
            })
            .collect::<Vec<_>>();
        let edge_geometries = edges
            .iter()
            .enumerate()
            .map(|(i, edge)| {
                let sources = std::mem::take(&mut edge_associations[i]);
                if sources.is_empty() {
                    StepEdgeGeometry::Curve(&edge.curve)
                } else {
                    // Resolve each face-surface association to a reference to
                    // that face's already-counted surface entity.
                    let associated_geometry = sources
                        .into_iter()
                        .map(|source| match source {
                            EdgeAssociationSource::ExactParameterCurve(curve) => {
                                StepAssociatedGeometry::ExactParameterCurve(curve)
                            }
                            EdgeAssociationSource::FaceSurface(face_pos) => {
                                StepAssociatedGeometry::SurfaceRef(surface_indices[face_pos])
                            }
                        })
                        .collect();
                    StepEdgeGeometry::SurfaceCurve(StepSurfaceCurve {
                        leader: &edge.curve,
                        associated_geometry,
                    })
                }
            })
            .collect::<Vec<_>>();
        let curve_indices = edge_geometries
            .iter()
            .map(|geometry| {
                let res = cursor;
                cursor += geometry.step_length();
                res
            })
            .collect::<Vec<_>>();
        let ep_points = cursor;
        StepShell {
            vertices,
            edges,
            faces,
            idx,
            face_indices,
            ep_edges,
            ep_vertices,
            surface_indices,
            edge_geometries,
            curve_indices,
            ep_points,
            is_open,
        }
    }
}

impl<P, C, S> Display for StepShell<'_, P, C, S>
where
    P: StepFormat + Copy,
    C: StepFormat + StepLength + StepCurve,
    S: StepFormat + StepLength + StepSurface,
{
    fn fmt(&self, formatter: &mut Formatter<'_>) -> Result {
        let StepShell {
            vertices,
            edges,
            faces,
            idx,
            face_indices,
            ep_edges,
            ep_vertices,
            surface_indices,
            edge_geometries,
            curve_indices,
            ep_points,
            is_open,
        } = self;
        let shell_kind = match is_open {
            true => "OPEN_SHELL",
            false => "CLOSED_SHELL",
        };
        formatter.write_fmt(format_args!(
            "#{idx} = {shell_kind}('', {face_indices});\n",
            face_indices = IndexSliceDisplay(self.face_indices.clone()),
        ))?;
        faces.iter().enumerate().try_for_each(|(i, f)| {
            let idx = face_indices[i];
            let mut cursor = idx + 1;
            let face_geometry = surface_indices[i];
            let face_bounds = match f.boundaries.is_empty() {
                true => vec![cursor],
                false => {
                    let closure = |b: &Vec<CompressedEdgeIndex>| {
                        let res = cursor;
                        cursor += 2 + b.len();
                        res
                    };
                    f.boundaries.iter().map(closure).collect()
                }
            };
            // `advanced_face` is the AP203/AP214/AP242 canonical shell face: a
            // `face_surface` subtype placed directly into the shell instead of
            // an `oriented_face`/`face_surface` pair. The former pair's face
            // orientation is folded into `same_sense` as `orientation ==
            // surface same_sense`, so a reload rebuilds the same
            // `CompressedFace` orientation.
            formatter.write_fmt(format_args!(
                "#{idx} = ADVANCED_FACE('', {face_bound}, #{face_geometry}, {same_sense});\n",
                same_sense = BooleanDisplay(f.orientation == f.surface.same_sense()),
                face_bound = IndexSliceDisplay(face_bounds.clone()),
            ))?;
            cursor = idx + 1;
            if f.boundaries.is_empty() {
                let face_bound_idx = cursor;
                let vertex_loop_idx = cursor + 1;
                let vertex_idx = cursor + 2;
                let vertex_geometry = cursor + 3;
                formatter.write_fmt(format_args!(
                    "#{face_bound_idx} = FACE_BOUND('', #{vertex_loop_idx}, .T.);
#{vertex_loop_idx} = VERTEX_LOOP('', #{vertex_idx});
#{vertex_idx} = VERTEX_POINT('', #{vertex_geometry});
#{vertex_geometry} = POINT_ON_SURFACE('', #{face_geometry}, 0.0, 0.0);\n"
                ))?;
            }
            f.boundaries.iter().try_for_each(|b| {
                let face_bound_idx = cursor;
                let edge_loop_idx = cursor + 1;
                let ep_oriented_edges = cursor + 2;
                cursor += 2 + b.len();
                formatter.write_fmt(format_args!(
                    "#{face_bound_idx} = FACE_BOUND('', #{edge_loop_idx}, {orientation});
#{edge_loop_idx} = EDGE_LOOP('', {oriented_edge_indices});\n",
                    // `CompressedFace::boundaries` are ABSOLUTE -- oriented
                    // about the surface's own normal -- while ISO 10303-42
                    // orients a face's loops about the FACE normal, which is
                    // the reverse when `orientation` is false. `FACE_BOUND`'s
                    // own flag is exactly the standard's way to say "reverse
                    // this loop", so emitting `f.orientation` here writes a
                    // conforming file AND round-trips through
                    // `Table::absolute_bound_orientation`, which composes the
                    // same two flags on the way back in (ledger C15).
                    //
                    // This used to be an unconditional `.T.`, which round-tripped
                    // only because the LOADER dropped the same composition.
                    orientation = BooleanDisplay(f.orientation),
                    oriented_edge_indices =
                        IndexSliceDisplay(ep_oriented_edges..ep_oriented_edges + b.len()),
                ))?;
                b.iter().enumerate().try_for_each(|(j, ce)| {
                    formatter.write_fmt(format_args!(
                        "#{idx} = ORIENTED_EDGE('', *, *, #{edge_element}, {orientation});\n",
                        idx = ep_oriented_edges + j,
                        edge_element = ep_edges + ce.index,
                        orientation = if ce.orientation { ".T." } else { ".F." },
                    ))
                })
            })
        })?;
        edge_geometries
            .iter()
            .enumerate()
            .try_for_each(|(i, geometry)| {
                let same_sense = if geometry.same_sense() { ".T." } else { ".F." };
                formatter.write_fmt(format_args!(
                    "#{idx} = EDGE_CURVE('', #{edge_start}, #{edge_end}, #{edge_geometry}, {same_sense});\n",
                    idx = ep_edges + i,
                    edge_start = ep_vertices + edges[i].vertices.0,
                    edge_end = ep_vertices + edges[i].vertices.1,
                    edge_geometry = curve_indices[i],
                ))
            })?;
        (0..vertices.len()).try_for_each(|i| {
            formatter.write_fmt(format_args!(
                "#{idx} = VERTEX_POINT('', #{vertex_geometry});\n",
                idx = ep_vertices + i,
                vertex_geometry = ep_points + i,
            ))
        })?;
        faces.iter().zip(surface_indices).try_for_each(|(f, idx)| {
            Display::fmt(&StepDisplay::new(&f.surface, *idx), formatter)
        })?;
        edge_geometries
            .iter()
            .zip(curve_indices)
            .try_for_each(|(geometry, idx)| StepFormat::fmt(geometry, *idx, formatter))?;
        vertices
            .iter()
            .enumerate()
            .try_for_each(|(i, v)| Display::fmt(&StepDisplay::new(*v, ep_points + i), formatter))
    }
}

impl<P, C, S> StepLength for StepShell<'_, P, C, S> {
    fn step_length(&self) -> usize {
        1 + self.ep_points + self.vertices.len() - self.face_indices[0]
    }
}

pub(super) struct StepSolid<'a, P, C, S> {
    idx: usize,
    boundaries: Vec<StepShell<'a, P, C, S>>,
}

impl<'a, P, C, S> StepSolid<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn new_curve3d_only(solid: &'a CompressedSolid<P, C, S>, idx: usize) -> Self {
        let mut cursor = idx + 1;
        let boundaries = solid
            .boundaries
            .iter()
            .map(|shell| {
                let res = StepShell::new_curve3d_only(shell, cursor, false);
                cursor += 1 + res.step_length();
                res
            })
            .collect::<Vec<_>>();
        StepSolid { idx, boundaries }
    }

    fn new(solid: &'a CompressedSolid<P, C, S>, idx: usize) -> Self {
        let mut cursor = idx + 1;
        let boundaries = solid
            .boundaries
            .iter()
            .map(|shell| {
                let res = StepShell::new(shell, cursor, false);
                cursor += 1 + res.step_length();
                res
            })
            .collect::<Vec<_>>();
        StepSolid { idx, boundaries }
    }
}

impl<'a, P, C, S> StepSolid<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn new_trimmed<T>(solid: &'a CompressedTrimmedSolid<P, C, S, T>, idx: usize) -> Self
    where T: StepFormat + StepLength {
        let mut cursor = idx + 1;
        let boundaries = solid
            .boundaries
            .iter()
            .map(|shell| {
                let res = StepShell::new_trimmed(shell, cursor, false);
                cursor += 1 + res.step_length();
                res
            })
            .collect::<Vec<_>>();
        StepSolid { idx, boundaries }
    }
}

impl<P, C, S> Display for StepSolid<'_, P, C, S>
where
    P: StepFormat + Copy,
    C: StepFormat + StepLength + StepCurve,
    S: StepFormat + StepLength + StepSurface,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let StepSolid { idx, boundaries } = self;
        match boundaries.len() {
            0 => {
                f.pad("empty solid!")?;
                Err(std::fmt::Error)
            }
            1 => {
                let shell_idx = idx + 1;
                let step_shell = &boundaries[0];
                f.write_fmt(format_args!(
                    "#{idx} = MANIFOLD_SOLID_BREP('', #{shell_idx});\n"
                ))?;
                Display::fmt(step_shell, f)
            }
            _ => {
                let first_shell_idx = boundaries[0].face_indices[0] - 1;
                f.write_fmt(format_args!(
                    "#{idx} = BREP_WITH_VOIDS('', #{first_shell_idx}, {other_shells});\n",
                    other_shells = IndexSliceDisplay(
                        boundaries[1..]
                            .iter()
                            .map(|step_shell| step_shell.face_indices[0] - 2)
                    ),
                ))?;
                Display::fmt(&boundaries[0], f)?;
                boundaries[1..].iter().try_for_each(|step_shell| {
                    let oriented_shell_idx = step_shell.face_indices[0] - 2;
                    let shell_idx = step_shell.face_indices[0] - 1;
                    f.write_fmt(format_args!(
                    "#{oriented_shell_idx} = ORIENTED_CLOSED_SHELL('', *, #{shell_idx}, .T.);\n",
                ))?;
                    Display::fmt(step_shell, f)
                })
            }
        }
    }
}

impl<P, C, S> StepLength for StepSolid<'_, P, C, S> {
    fn step_length(&self) -> usize {
        let b = &self.boundaries;
        match b.len() {
            0 => 0,
            1 => 1 + b[0].step_length(),
            _ => b.len() + b.iter().map(StepLength::step_length).sum::<usize>(),
        }
    }
}

pub(super) enum PreStepModel<'a, P, C, S> {
    /// shell based surface model
    Shell(StepShell<'a, P, C, S>),
    /// solid model
    Solid(StepSolid<'a, P, C, S>),
}

impl<'a, P, C, S> From<&'a CompressedShell<P, C, S>> for PreStepModel<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn from(shell: &'a CompressedShell<P, C, S>) -> Self {
        Self::Shell(StepShell::new(shell, 17, true))
    }
}

impl<'a, P, C, S> From<&'a CompressedSolid<P, C, S>> for PreStepModel<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn from(solid: &'a CompressedSolid<P, C, S>) -> Self { Self::Solid(StepSolid::new(solid, 16)) }
}

impl<'a, P, C, S, T> From<&'a CompressedTrimmedShell<P, C, S, T>> for PreStepModel<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
    T: StepFormat + StepLength,
{
    fn from(shell: &'a CompressedTrimmedShell<P, C, S, T>) -> Self {
        Self::Shell(StepShell::new_trimmed(shell, 17, true))
    }
}

impl<'a, P, C, S, T> From<&'a CompressedTrimmedSolid<P, C, S, T>> for PreStepModel<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
    T: StepFormat + StepLength,
{
    fn from(solid: &'a CompressedTrimmedSolid<P, C, S, T>) -> Self {
        Self::Solid(StepSolid::new_trimmed(solid, 16))
    }
}

impl<P, C, S> Display for PreStepModel<'_, P, C, S>
where
    P: StepFormat + Copy,
    C: StepFormat + StepLength + StepCurve,
    S: StepFormat + StepLength + StepSurface,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        match self {
            Self::Shell(x) => {
                f.write_fmt(format_args!(
                    "#{idx} = SHELL_BASED_SURFACE_MODEL('', (#{shell_idx}));\n",
                    idx = x.idx - 1,
                    shell_idx = x.idx
                ))?;
                Display::fmt(&x, f)
            }
            Self::Solid(x) => Display::fmt(x, f),
        }
    }
}

impl<P, C, S> StepLength for PreStepModel<'_, P, C, S> {
    fn step_length(&self) -> usize {
        match self {
            Self::Shell(x) => 1 + x.step_length(),
            Self::Solid(x) => x.step_length(),
        }
    }
}

impl<'a, P, C, S> From<&'a CompressedShell<P, C, S>> for StepModel<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn from(shell: &'a CompressedShell<P, C, S>) -> Self {
        Self(shell.into(), StepMeasurementContext::default())
    }
}

impl<'a, P, C, S> From<&'a CompressedSolid<P, C, S>> for StepModel<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn from(solid: &'a CompressedSolid<P, C, S>) -> Self {
        Self(solid.into(), StepMeasurementContext::default())
    }
}

impl<'a, P, C, S> StepModel<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    /// Creates a STEP model that exports only shared 3-dimensional edge curves.
    pub fn from_curve3d_only_shell(shell: &'a CompressedShell<P, C, S>) -> Self {
        Self(
            PreStepModel::Shell(StepShell::new_curve3d_only(shell, 17, true)),
            StepMeasurementContext::default(),
        )
    }

    /// Creates a STEP model that exports only shared 3-dimensional edge curves.
    pub fn from_curve3d_only_solid(solid: &'a CompressedSolid<P, C, S>) -> Self {
        Self(
            PreStepModel::Solid(StepSolid::new_curve3d_only(solid, 16)),
            StepMeasurementContext::default(),
        )
    }

    /// Overrides the length unit and distance accuracy written into the
    /// representation-context preamble. The default preserves millimetre
    /// lengths and a `1.0E-6` `distance_accuracy_value`.
    pub fn with_measurement_context(mut self, context: StepMeasurementContext) -> Self {
        self.1 = context;
        self
    }

    /// Returns the length unit and distance accuracy written into the preamble.
    pub fn measurement_context(&self) -> StepMeasurementContext { self.1 }
}

impl<'a, P, C, S, T> From<&'a CompressedTrimmedShell<P, C, S, T>> for StepModel<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
    T: StepFormat + StepLength,
{
    fn from(shell: &'a CompressedTrimmedShell<P, C, S, T>) -> Self {
        Self(shell.into(), StepMeasurementContext::default())
    }
}

impl<'a, P, C, S, T> From<&'a CompressedTrimmedSolid<P, C, S, T>> for StepModel<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
    T: StepFormat + StepLength,
{
    fn from(solid: &'a CompressedTrimmedSolid<P, C, S, T>) -> Self {
        Self(solid.into(), StepMeasurementContext::default())
    }
}

impl<P, C, S> Display for StepModel<'_, P, C, S>
where
    P: StepFormat + Copy,
    C: StepFormat + StepLength + StepCurve,
    S: StepFormat + StepLength + StepSurface,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let length_prefix = self.1.length_prefix;
        let accuracy = self.1.accuracy();
        f.write_fmt(format_args!(
"#1 = APPLICATION_PROTOCOL_DEFINITION('international standard', 'automotive_design', 2000, #2);
#2 = APPLICATION_CONTEXT('core data for automotive mechanical design processes');
#3 = SHAPE_DEFINITION_REPRESENTATION(#4, #10);
#4 = PRODUCT_DEFINITION_SHAPE('','', #5);
#5 = PRODUCT_DEFINITION('design','', #6, #9);
#6 = PRODUCT_DEFINITION_FORMATION('','', #7);
#7 = PRODUCT('','','', (#8));
#8 = PRODUCT_CONTEXT('', #2, 'mechanical');
#9 = PRODUCT_DEFINITION_CONTEXT('part definition', #2, 'design');
#10 = SHAPE_REPRESENTATION('', (#16), #11);
#11 = (
    GEOMETRIC_REPRESENTATION_CONTEXT(3)
    GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#15))
    GLOBAL_UNIT_ASSIGNED_CONTEXT((#12, #13, #14))
    REPRESENTATION_CONTEXT('Context #1', '3D Context with UNIT and UNCERTAINTY')
);
#12 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT({length_prefix},.METRE.) );
#13 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );
#14 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );
#15 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({accuracy}), #12, 'distance_accuracy_value','confusion accuracy');\n"
        ))?;
        Display::fmt(&self.0, f)
    }
}

impl<P, C, S> Default for StepModels<'_, P, C, S> {
    fn default() -> Self {
        Self {
            models: Vec::new(),
            next_idx: 16,
            measurement_context: StepMeasurementContext::default(),
        }
    }
}

impl<'a, P, C, S> StepModels<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    /// The next available entity index after all pushed models.
    pub fn next_idx(&self) -> usize { self.next_idx }

    /// Overrides the length unit and distance accuracy written into the
    /// representation-context preamble. The default preserves millimetre
    /// lengths and a `1.0E-6` `distance_accuracy_value`.
    pub fn with_measurement_context(mut self, context: StepMeasurementContext) -> Self {
        self.measurement_context = context;
        self
    }

    /// Returns the length unit and distance accuracy written into the preamble.
    pub fn measurement_context(&self) -> StepMeasurementContext { self.measurement_context }
    /// push a shell to step models
    pub fn push_shell(&mut self, shell: &'a CompressedShell<P, C, S>) {
        let model = PreStepModel::Shell(StepShell::new(shell, self.next_idx + 1, true));
        self.next_idx += model.step_length();
        self.models.push(model)
    }
    /// push a solid to step models
    pub fn push_solid(&mut self, solid: &'a CompressedSolid<P, C, S>) {
        let model = PreStepModel::Solid(StepSolid::new(solid, self.next_idx));
        self.next_idx += model.step_length();
        self.models.push(model)
    }

    /// Pushes a shell while exporting only shared 3-dimensional edge curves.
    pub fn push_curve3d_only_shell(&mut self, shell: &'a CompressedShell<P, C, S>) {
        let model =
            PreStepModel::Shell(StepShell::new_curve3d_only(shell, self.next_idx + 1, true));
        self.next_idx += model.step_length();
        self.models.push(model)
    }

    /// Pushes a solid while exporting only shared 3-dimensional edge curves.
    pub fn push_curve3d_only_solid(&mut self, solid: &'a CompressedSolid<P, C, S>) {
        let model = PreStepModel::Solid(StepSolid::new_curve3d_only(solid, self.next_idx));
        self.next_idx += model.step_length();
        self.models.push(model)
    }
}

impl<'a, P, C, S> StepModels<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    /// Pushes a trimmed shell to step models.
    pub fn push_trimmed_shell<T>(&mut self, shell: &'a CompressedTrimmedShell<P, C, S, T>)
    where T: StepFormat + StepLength {
        let model = PreStepModel::Shell(StepShell::new_trimmed(shell, self.next_idx + 1, true));
        self.next_idx += model.step_length();
        self.models.push(model)
    }

    /// Pushes a trimmed solid to step models.
    pub fn push_trimmed_solid<T>(&mut self, solid: &'a CompressedTrimmedSolid<P, C, S, T>)
    where T: StepFormat + StepLength {
        let model = PreStepModel::Solid(StepSolid::new_trimmed(solid, self.next_idx));
        self.next_idx += model.step_length();
        self.models.push(model)
    }
}

impl<'a, P, C, S> FromIterator<&'a CompressedShell<P, C, S>> for StepModels<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn from_iter<T: IntoIterator<Item = &'a CompressedShell<P, C, S>>>(iter: T) -> Self {
        let mut next_idx = 16;
        let models = iter
            .into_iter()
            .map(|shell| {
                let model = PreStepModel::Shell(StepShell::new(shell, next_idx + 1, true));
                next_idx += model.step_length();
                model
            })
            .collect();
        Self {
            models,
            next_idx,
            measurement_context: StepMeasurementContext::default(),
        }
    }
}

impl<'a, P, C, S> FromIterator<&'a CompressedSolid<P, C, S>> for StepModels<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
{
    fn from_iter<T: IntoIterator<Item = &'a CompressedSolid<P, C, S>>>(iter: T) -> Self {
        let mut next_idx = 16;
        let models = iter
            .into_iter()
            .map(|solid| {
                let model = PreStepModel::Solid(StepSolid::new(solid, next_idx));
                next_idx += model.step_length();
                model
            })
            .collect();
        Self {
            models,
            next_idx,
            measurement_context: StepMeasurementContext::default(),
        }
    }
}

impl<'a, P, C, S, U> FromIterator<&'a CompressedTrimmedShell<P, C, S, U>>
    for StepModels<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
    U: StepFormat + StepLength,
{
    fn from_iter<T: IntoIterator<Item = &'a CompressedTrimmedShell<P, C, S, U>>>(iter: T) -> Self {
        let mut next_idx = 16;
        let models = iter
            .into_iter()
            .map(|shell| {
                let model = PreStepModel::Shell(StepShell::new_trimmed(shell, next_idx + 1, true));
                next_idx += model.step_length();
                model
            })
            .collect();
        Self {
            models,
            next_idx,
            measurement_context: StepMeasurementContext::default(),
        }
    }
}

impl<'a, P, C, S, U> FromIterator<&'a CompressedTrimmedSolid<P, C, S, U>>
    for StepModels<'a, P, C, S>
where
    P: Copy,
    C: StepLength,
    S: StepLength,
    U: StepFormat + StepLength,
{
    fn from_iter<T: IntoIterator<Item = &'a CompressedTrimmedSolid<P, C, S, U>>>(iter: T) -> Self {
        let mut next_idx = 16;
        let models = iter
            .into_iter()
            .map(|solid| {
                let model = PreStepModel::Solid(StepSolid::new_trimmed(solid, next_idx));
                next_idx += model.step_length();
                model
            })
            .collect();
        Self {
            models,
            next_idx,
            measurement_context: StepMeasurementContext::default(),
        }
    }
}

impl<P, C, S> Display for StepModels<'_, P, C, S>
where
    P: StepFormat + Copy,
    C: StepFormat + StepLength + StepCurve,
    S: StepFormat + StepLength + StepSurface,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.pad(
"#1 = APPLICATION_PROTOCOL_DEFINITION('international standard', 'automotive_design', 2000, #2);
#2 = APPLICATION_CONTEXT('core data for automotive mechanical design processes');
#3 = SHAPE_DEFINITION_REPRESENTATION(#4, #10);
#4 = PRODUCT_DEFINITION_SHAPE('','', #5);
#5 = PRODUCT_DEFINITION('design','', #6, #9);
#6 = PRODUCT_DEFINITION_FORMATION('','', #7);
#7 = PRODUCT('','','', (#8));
#8 = PRODUCT_CONTEXT('', #2, 'mechanical');
#9 = PRODUCT_DEFINITION_CONTEXT('part definition', #2, 'design');\n")?;
        let models_slice = IndexSliceDisplay(self.models.iter().map(|model| match model {
            PreStepModel::Shell(x) => x.idx - 1,
            PreStepModel::Solid(x) => x.idx,
        }));
        f.write_fmt(format_args!(
            "#10 = ADVANCED_BREP_SHAPE_REPRESENTATION('', {models_slice}, #11);\n"
        ))?;
        let length_prefix = self.measurement_context.length_prefix;
        let accuracy = self.measurement_context.accuracy();
        f.write_fmt(format_args!("#11 = (
    GEOMETRIC_REPRESENTATION_CONTEXT(3) 
    GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#15))
    GLOBAL_UNIT_ASSIGNED_CONTEXT((#12, #13, #14))
    REPRESENTATION_CONTEXT('Context #1', '3D Context with UNIT and UNCERTAINTY')
);
#12 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT({length_prefix},.METRE.) );
#13 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );
#14 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );
#15 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({accuracy}), #12, 'distance_accuracy_value','confusion accuracy');\n"
        ))?;
        self.models
            .iter()
            .try_for_each(|model| Display::fmt(model, f))
    }
}
