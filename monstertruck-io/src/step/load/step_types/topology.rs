//! Topology entities: vertices, edges, loops, faces, shells and solids.
//!
//! Also carries the `*Holder` inherent impls that resolve a topological
//! entity's references against the entity table while loading.

use super::*;

/// `vertex_point`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = vertex_point)]
#[holder(generate_deserialize)]
pub struct VertexPoint {
    pub label: String,
    #[holder(use_place_holder)]
    pub vertex_geometry: CartesianPoint,
}

/// `edge`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum EdgeAny {
    #[holder(use_place_holder)]
    EdgeCurve(EdgeCurve),
    #[holder(use_place_holder)]
    OrientedEdge(OrientedEdge),
}

/// `edge_curve`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = edge_curve)]
#[holder(generate_deserialize)]
pub struct EdgeCurve {
    pub label: String,
    #[holder(use_place_holder)]
    pub edge_start: VertexPoint,
    #[holder(use_place_holder)]
    pub edge_end: VertexPoint,
    #[holder(use_place_holder)]
    pub edge_geometry: CurveAny,
    pub same_sense: bool,
}

impl EdgeCurve {
    pub fn parse_curve2d(&self) -> Result<Curve2D, StepConvertingError> {
        let p = Point2::from(&self.edge_start.vertex_geometry);
        let q = Point2::from(&self.edge_end.vertex_geometry);
        let (p, q) = match self.same_sense {
            true => (p, q),
            false => (q, p),
        };
        Self::sub_parse_2d(&self.edge_geometry, p, q, self.same_sense)
    }
    fn sub_parse_2d(
        curve: &CurveAny,
        p: Point2,
        q: Point2,
        same_sense: bool,
    ) -> Result<Curve2D, StepConvertingError> {
        let mut curve = match curve {
            CurveAny::Line(line) => {
                let line = geom::Line::<Point2>::from(line.as_ref());
                let p = line.projection(p);
                let q = line.projection(q);
                Curve2D::Line(Line(p, q))
            }
            CurveAny::BoundedCurve(b) => b.as_ref().try_into()?,
            CurveAny::Conic(curve) => match curve.as_ref() {
                Conic::Circle(circle) => {
                    let mat =
                        Matrix3::try_from(&circle.position)? * Matrix3::from_scale(circle.radius);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitCircle::<Point2>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                        UnitCircle::<Point2>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let circle = TrimmedCurve::new(UnitCircle::<Point2>::new(), (u, v));
                    let mut ellipse = Processor::new(circle);
                    ellipse.transform_by(mat);
                    Curve2D::Conic(Conic2D::Ellipse(ellipse))
                }
                Conic::Ellipse(ellipse) => {
                    let mat = Matrix3::try_from(&ellipse.position)?
                        * Matrix3::from_nonuniform_scale(ellipse.semi_axis_1, ellipse.semi_axis_2);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitCircle::<Point2>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                        UnitCircle::<Point2>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let circle = TrimmedCurve::new(UnitCircle::<Point2>::new(), (u, v));
                    let mut ellipse = Processor::new(circle);
                    ellipse.transform_by(mat);
                    Curve2D::Conic(Conic2D::Ellipse(ellipse))
                }
                Conic::Hyperbola(hyperbola) => {
                    let mat = Matrix3::try_from(&hyperbola.position)?
                        * Matrix3::from_nonuniform_scale(
                            hyperbola.semi_axis,
                            hyperbola.semi_imag_axis,
                        );
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Hyperbola".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, v) = (
                        UnitHyperbola::<Point2>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on hyperbola".to_string())?,
                        UnitHyperbola::<Point2>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on hyparbola".to_string())?,
                    );
                    let unit = TrimmedCurve::new(UnitHyperbola::<Point2>::new(), (u, v));
                    let mut hyperbola = Processor::new(unit);
                    hyperbola.transform_by(mat);
                    Curve2D::Conic(Conic2D::Hyperbola(hyperbola))
                }
                Conic::Parabola(parabola) => {
                    let mat = Matrix3::try_from(&parabola.position)?
                        * Matrix3::from_scale(parabola.focal_dist);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Parabola".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, v) = (
                        UnitHyperbola::<Point2>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on parabola".to_string())?,
                        UnitHyperbola::<Point2>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on parabola".to_string())?,
                    );
                    let unit = TrimmedCurve::new(UnitHyperbola::<Point2>::new(), (u, v));
                    let mut parabola = Processor::new(unit);
                    parabola.transform_by(mat);
                    Curve2D::Conic(Conic2D::Hyperbola(parabola))
                }
            },
            CurveAny::Pcurve(_) => return Err("Pcurves cannot be parsed to 2D curves.".into()),
            CurveAny::SurfaceCurve(_) => {
                return Err("Surface curves cannot be parsed to 2D curves.".into());
            }
        };
        if !same_sense {
            curve.invert();
        }
        Ok(curve)
    }
    pub fn parse_curve3d(&self) -> Result<Curve3D, StepConvertingError> {
        let p = Point3::from(&self.edge_start.vertex_geometry);
        let q = Point3::from(&self.edge_end.vertex_geometry);
        let (p, q) = match self.same_sense {
            true => (p, q),
            false => (q, p),
        };
        Self::sub_parse_curve3d(&self.edge_geometry, p, q, self.same_sense)
    }
    fn sub_parse_curve3d(
        curve: &CurveAny,
        p: Point3,
        q: Point3,
        same_sense: bool,
    ) -> Result<Curve3D, StepConvertingError> {
        let mut curve = match curve {
            CurveAny::Line(_) => Curve3D::Line(Line(p, q)),
            CurveAny::BoundedCurve(b) => b.as_ref().try_into()?,
            CurveAny::Conic(curve) => match curve.as_ref() {
                Conic::Circle(circle) => {
                    let mat =
                        Matrix4::try_from(&circle.position)? * Matrix4::from_scale(circle.radius);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitCircle::<Point3>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| format!("the point is not on circle: {p:?}"))?,
                        UnitCircle::<Point3>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| format!("the point is not on circle: {q:?}"))?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let circle = TrimmedCurve::new(UnitCircle::<Point3>::new(), (u, v));
                    let mut ellipse = Processor::new(circle);
                    ellipse.transform_by(mat);
                    Curve3D::Conic(Conic3D::Ellipse(ellipse))
                }
                Conic::Ellipse(ellipse) => {
                    let mat = Matrix4::try_from(&ellipse.position)?
                        * Matrix4::from_nonuniform_scale(
                            ellipse.semi_axis_1,
                            ellipse.semi_axis_2,
                            f64::min(ellipse.semi_axis_1, ellipse.semi_axis_2),
                        );
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitCircle::<Point3>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| format!("the point is not on circle: {p:?}"))?,
                        UnitCircle::<Point3>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| format!("the point is not on circle: {q:?}"))?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let circle = TrimmedCurve::new(UnitCircle::<Point3>::new(), (u, v));
                    let mut ellipse = Processor::new(circle);
                    ellipse.transform_by(mat);
                    Curve3D::Conic(Conic3D::Ellipse(ellipse))
                }
                Conic::Hyperbola(hyperbola) => {
                    let mat = Matrix4::try_from(&hyperbola.position)?
                        * Matrix4::from_nonuniform_scale(
                            hyperbola.semi_axis,
                            hyperbola.semi_imag_axis,
                            f64::min(hyperbola.semi_axis, hyperbola.semi_imag_axis),
                        );
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Circle".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, mut v) = (
                        UnitHyperbola::<Point3>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                        UnitHyperbola::<Point3>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on circle".to_string())?,
                    );
                    if v <= u + TOLERANCE {
                        v += 2.0 * PI;
                    }
                    let unit = TrimmedCurve::new(UnitHyperbola::<Point3>::new(), (u, v));
                    let mut hyperbola = Processor::new(unit);
                    hyperbola.transform_by(mat);
                    Curve3D::Conic(Conic3D::Hyperbola(hyperbola))
                }
                Conic::Parabola(parabola) => {
                    let mat = Matrix4::try_from(&parabola.position)?
                        * Matrix4::from_scale(parabola.focal_dist);
                    let inv_mat = mat
                        .invert()
                        .ok_or_else(|| "Failed to convert Parabola".to_string())?;
                    let (p, q) = (inv_mat.transform_point(p), inv_mat.transform_point(q));
                    let (u, v) = (
                        UnitHyperbola::<Point3>::new()
                            .search_nearest_parameter(p, None, 0)
                            .ok_or_else(|| "the point is not on parabola".to_string())?,
                        UnitHyperbola::<Point3>::new()
                            .search_nearest_parameter(q, None, 0)
                            .ok_or_else(|| "the point is not on parabola".to_string())?,
                    );
                    let unit = TrimmedCurve::new(UnitHyperbola::<Point3>::new(), (u, v));
                    let mut parabola = Processor::new(unit);
                    parabola.transform_by(mat);
                    Curve3D::Conic(Conic3D::Hyperbola(parabola))
                }
            },
            CurveAny::Pcurve(c) => {
                let surface: Surface = (&c.basis_surface).try_into()?;
                let u = surface
                    .search_nearest_parameter(p, None, 100)
                    .ok_or_else(|| "the point is not on surface".to_string())?;
                let v = surface
                    .search_nearest_parameter(q, None, 100)
                    .ok_or_else(|| "the point is not on surface".to_string())?;
                let curve2d = c
                    .reference_to_curve
                    .representation_item
                    .first()
                    .ok_or("no representation item")?;
                let curve2d = Self::sub_parse_2d(
                    curve2d,
                    Point2::new(u.0, u.1),
                    Point2::new(v.0, v.1),
                    true,
                )?;
                Curve3D::ParameterCurve(geom::ParameterCurve::new(
                    Box::new(curve2d),
                    Box::new(surface),
                ))
            }
            CurveAny::SurfaceCurve(c) => {
                if p.near(&q) {
                    return Self::sub_parse_curve3d(&c.curve_3d, p, q, same_sense);
                }
                let associated_surface = |entry: &PcurveOrSurface| -> Result<
                    step_geometry::Surface,
                    StepConvertingError,
                > {
                    match entry {
                        PcurveOrSurface::Pcurve(x) => (&x.basis_surface).try_into(),
                        PcurveOrSurface::Surface(x) => x.as_ref().try_into(),
                    }
                };
                let associated_geometry = c
                    .associated_geometry
                    .iter()
                    .map(|entry| match entry {
                        PcurveOrSurface::Pcurve(curve) => curve
                            .as_ref()
                            .try_into()
                            .map(step_geometry::SurfaceCurveAssociatedGeometry::ParameterCurve),
                        PcurveOrSurface::Surface(surface) => surface
                            .as_ref()
                            .try_into()
                            .map(Box::new)
                            .map(step_geometry::SurfaceCurveAssociatedGeometry::Surface),
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                let kind = match c.kind {
                    SurfaceCurveEntityKind::Surface => {
                        step_geometry::SurfaceCurveKind::SurfaceCurve
                    }
                    SurfaceCurveEntityKind::Seam => step_geometry::SurfaceCurveKind::SeamCurve,
                    SurfaceCurveEntityKind::Intersection => {
                        step_geometry::SurfaceCurveKind::IntersectionCurve
                    }
                };
                let master_representation = match c.master_representation {
                    PreferredSurfaceCurveRepresentation::Curve3D => {
                        step_geometry::SurfaceCurveRepresentation::Curve3D
                    }
                    PreferredSurfaceCurveRepresentation::PcurveS1 => {
                        step_geometry::SurfaceCurveRepresentation::ParameterCurve0
                    }
                    PreferredSurfaceCurveRepresentation::PcurveS2 => {
                        step_geometry::SurfaceCurveRepresentation::ParameterCurve1
                    }
                };
                let leader = Self::sub_parse_curve3d(&c.curve_3d, p, q, true)?;
                if associated_geometry.len() >= 2
                    && associated_geometry.iter().all(|entry| {
                        matches!(
                            entry,
                            step_geometry::SurfaceCurveAssociatedGeometry::Surface(_)
                        )
                    })
                {
                    let surface0 = c
                        .associated_geometry
                        .first()
                        .ok_or("The 0-indexed associated geometry is missing.")?;
                    let surface1 = c
                        .associated_geometry
                        .get(1)
                        .ok_or("The 1-indexed associated geometry is missing.")?;
                    Curve3D::IntersectionCurve(geom::IntersectionCurve::new(
                        Box::new(associated_surface(surface0)?),
                        Box::new(associated_surface(surface1)?),
                        Box::new(leader),
                    ))
                } else {
                    Curve3D::SurfaceCurve(step_geometry::SurfaceCurve3D::new(
                        kind,
                        Box::new(leader),
                        associated_geometry,
                        master_representation,
                    ))
                }
            }
        };
        if !same_sense {
            curve.invert();
        }
        Ok(curve)
    }
}

/// `oriented_edge`
///
/// `oriented_edge` has duplicated information.
/// These are not included here because they are essentially omitted.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = oriented_edge)]
#[holder(generate_deserialize)]
pub struct OrientedEdge {
    pub label: String,
    #[holder(use_place_holder)]
    pub edge_element: EdgeCurve,
    pub orientation: bool,
}

impl OrientedEdgeHolder {
    pub(crate) fn edge_element_holder(&self, table: &Table) -> Option<EdgeCurveHolder> {
        match &self.edge_element {
            PlaceHolder::Owned(holder) => Some(holder.clone()),
            PlaceHolder::Ref(Name::Entity(idx)) => table.edge_curve.get(idx).cloned(),
            _ => None,
        }
    }
    pub(crate) fn edge_element_idx(&self) -> Option<u64> {
        if let PlaceHolder::Ref(Name::Entity(idx)) = self.edge_element {
            Some(idx)
        } else {
            None
        }
    }
}

/// `edge_loop`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = edge_loop)]
#[holder(generate_deserialize)]
pub struct EdgeLoop {
    pub label: String,
    #[holder(use_place_holder)]
    pub edge_list: Vec<EdgeAny>,
}

/// `vertex_loop`
///
/// A `loop` whose entire extent is ONE vertex -- the degenerate boundary
/// exporters emit at a parameterisation singularity: a cone apex, a sphere pole,
/// or a stand-in for the (boundary-less) closed torus.
///
/// It has an arm here purely so the record is ATTRIBUTABLE. Before spec 011 T7
/// it fell into [`Table::dummy`](crate::step::load::Table::dummy), so
/// `FaceBoundHolder::bound_holder` looked the id up in `Table::edge_loop`, got
/// `None`, and the wire was discarded by a `filter_map` **with no error and no
/// message anywhere** -- the only truly silent drop the spec 011 Phase 0 census
/// found. It bit four in-repo fixtures, `boxy-with-surfacetex.step` losing 10 of
/// its 160 wires.
///
/// It is still not REPRESENTED in a `CompressedShell`: a compressed wire is a
/// sequence of edge uses and a point boundary has none, so the loop is reported
/// as [`LossReason::DegenerateVertexLoop`](crate::step::load::report::LossReason::DegenerateVertexLoop)
/// and dropped. Read that variant's docs for the measurement behind the choice.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = vertex_loop)]
#[holder(generate_deserialize)]
pub struct VertexLoop {
    pub label: String,
    #[holder(use_place_holder)]
    pub loop_vertex: VertexPoint,
}

/// `face_bound`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = face_bound)]
#[holder(generate_deserialize)]
/// `FACE_OUTER_BOUNDS` is also parsed to this struct.
pub struct FaceBound {
    pub label: String,
    // For now, we are going with the policy of accepting nothing but edgeloop.
    #[holder(use_place_holder)]
    pub bound: EdgeLoop,
    pub orientation: bool,
}

/// What a `FACE_BOUND`'s `bound` reference turns out to point at.
///
/// The declared type of `FaceBound::bound` is `EdgeLoop`, but STEP's `loop` has
/// three subtypes and the reference is untyped until it is resolved. Naming the
/// three outcomes is what lets the loader distinguish a LEGITIMATE degeneracy
/// (a vertex loop) from a genuine loss (a bound that is simply not there).
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BoundResolution {
    /// An `EDGE_LOOP`, inline or resolved from the table.
    EdgeLoop(EdgeLoopHolder),
    /// A `VERTEX_LOOP`, with its entity id.
    VertexLoop(u64),
    /// Neither: no record with that id, or a `loop` subtype with no arm. Carries
    /// the id when the reference had one.
    Unresolved(Option<u64>),
}

impl FaceBoundHolder {
    pub(crate) fn resolve_bound(&self, table: &Table) -> BoundResolution {
        match &self.bound {
            PlaceHolder::Owned(holder) => BoundResolution::EdgeLoop(holder.clone()),
            PlaceHolder::Ref(Name::Entity(idx)) => table
                .edge_loop
                .get(idx)
                .cloned()
                .map(BoundResolution::EdgeLoop)
                .unwrap_or_else(|| {
                    if table.vertex_loop.contains_key(idx) {
                        BoundResolution::VertexLoop(*idx)
                    } else {
                        BoundResolution::Unresolved(Some(*idx))
                    }
                }),
            _ => BoundResolution::Unresolved(None),
        }
    }

    pub(crate) fn bound_holder(&self, table: &Table) -> Option<EdgeLoopHolder> {
        match self.resolve_bound(table) {
            BoundResolution::EdgeLoop(holder) => Some(holder),
            _ => None,
        }
    }
}

/// `face`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum FaceAny {
    #[holder(use_place_holder)]
    FaceSurface(FaceSurface),
    #[holder(use_place_holder)]
    OrientedFace(OrientedFace),
}

/// `face_surface`
///
/// `advanced_face` is also parsed to this struct.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = face_surface)]
#[holder(generate_deserialize)]
pub struct FaceSurface {
    pub label: String,
    #[holder(use_place_holder)]
    pub bounds: Vec<FaceBound>,
    #[holder(use_place_holder)]
    pub face_geometry: SurfaceAny,
    pub same_sense: bool,
}

impl FaceSurfaceHolder {
    /// Every bound the face LISTS, paired with the entity id the face referenced.
    ///
    /// The id is carried alongside so a bound that fails to resolve can still be
    /// reported by name -- without it, a lost wire is anonymous and a human has
    /// nothing to go and look at.
    pub(crate) fn bounds_holder<'a>(
        &'a self,
        table: &'a Table,
    ) -> Vec<(Option<u64>, Option<FaceBoundHolder>)> {
        self.bounds
            .iter()
            .map(|bound| match bound {
                PlaceHolder::Owned(bound) => (None, Some(bound.clone())),
                PlaceHolder::Ref(Name::Entity(idx)) => {
                    (Some(*idx), table.face_bound.get(idx).cloned())
                }
                _ => (None, None),
            })
            .collect()
    }
}

/// `oriented_face`
///
/// `oriented_face` has duplicated information.
/// These are not included here because they are essentially omitted.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = oriented_face)]
#[holder(generate_deserialize)]
pub struct OrientedFace {
    pub label: String,
    #[holder(use_place_holder)]
    pub face_element: FaceSurface,
    pub orientation: bool,
}

impl OrientedFaceHolder {
    pub(crate) fn face_element_holder(&self, table: &Table) -> Option<FaceSurfaceHolder> {
        match &self.face_element {
            PlaceHolder::Ref(Name::Entity(idx)) => table.face_surface.get(idx).cloned(),
            PlaceHolder::Owned(x) => Some(x.clone()),
            _ => None,
        }
    }
}

/// `shell`
///
/// Includes `open_shell` and `closed_shell`.
/// Since these differences are only informal propositions, the data structure does not distinguish between the two.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shell)]
#[holder(generate_deserialize)]
pub struct Shell {
    pub label: String,
    #[holder(use_place_holder)]
    pub cfs_faces: Vec<FaceAny>,
}

impl ShellHolder {
    pub(crate) fn cfs_faces_holder<'a>(
        &'a self,
        table: &'a Table,
    ) -> impl Iterator<Item = Option<FaceAnyHolder>> + 'a {
        self.cfs_faces.iter().map(|face| match face {
            PlaceHolder::Owned(holder) => Some(holder.clone()),
            PlaceHolder::Ref(Name::Entity(idx)) => table
                .oriented_face
                .get(idx)
                .cloned()
                .map(FaceAnyHolder::OrientedFace)
                .or_else(|| {
                    table
                        .face_surface
                        .get(idx)
                        .cloned()
                        .map(FaceAnyHolder::FaceSurface)
                }),
            _ => None,
        })
    }
}

/// `oriented_shell`
///
/// Includes `oriented_open_shell` and `oriented_closed_shell`.
/// Since these differences are only informal propositions, the data structure does not distinguish between the two.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = oriented_shell)]
#[holder(generate_deserialize)]
pub struct OrientedShell {
    pub label: String,
    #[holder(use_place_holder)]
    pub shell_element: Shell,
    pub orientation: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum ShellAny {
    #[holder(use_place_holder)]
    Shell(Shell),
    #[holder(use_place_holder)]
    OrientedShell(OrientedShell),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shell_based_surface_model)]
#[holder(generate_deserialize)]
pub struct ShellBasedSurfaceModel {
    pub label: String,
    #[holder(use_place_holder)]
    pub sbsm_boundary: Vec<ShellAny>,
}

/// Also serves as `brep_with_voids`
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = manifold_solid_brep)]
#[holder(generate_deserialize)]
pub struct ManifoldSolidBrep {
    pub label: String,
    #[holder(use_place_holder)]
    pub outer: ShellAny,
    #[holder(use_place_holder)]
    pub voids: Vec<OrientedShell>,
}
