//! Conversion tests over an IN-MEMORY intermediate representation.
//!
//! No file is read here, deliberately. The decoder is cadmpeg's responsibility
//! and is measured separately (see the note on [`crate::cadmpeg`]); what is
//! unverified is this crate's mapping, and building the representation directly
//! is the only way to exercise a refusal arm for a carrier no fixture in the
//! world happens to contain.
//!
//! **These tests do not show that reading a file works.** They show that a
//! decoded document converts.
//!
//! # The oracle for orientation and shape
//!
//! Where a test asks "is this the same surface, facing the same way", the
//! answer comes from cadmpeg's OWN evaluator (`cadmpeg_ir::eval`), never from a
//! restatement of the formula the converter used. The converter and the oracle
//! then share no code, so agreement means something.
//!
//! # What cadmpeg 0.6 made unrepresentable
//!
//! Several 0.5 rows here built malformed carriers by struct literal: a NURBS net
//! with a short knot vector, a frame whose reference was parallel to its axis, a
//! non-positive radius. 0.6 admits carriers only through checked constructors,
//! so none of those can reach the converter any more, and their rows are gone
//! with the refusal arms they exercised.

use cadmpeg_ir::eval::admission::EvaluationAdmission;
use cadmpeg_ir::features::FinitePoint3;
use cadmpeg_ir::geometry::analytic::{
    CircleCurve, ConeSurface, CylinderSurface, EllipseCurve, LineCurve, PlaneSurface,
    SphereSurface, TorusSurface,
};
use cadmpeg_ir::geometry::nurbs::{
    NurbsCurve as IrNurbsCurve, NurbsPoleGrid, NurbsPoles3, NurbsSurface as IrNurbsSurface,
    NurbsSurfaceAxis,
};
use cadmpeg_ir::geometry::sampled::PolygonalSurface;
use cadmpeg_ir::geometry::{
    Curve as IrCurve, CurveGeometry, SolvedCurveGeometry, SolvedSurfaceGeometry,
    Surface as IrSurface, SurfaceGeometry,
};
use cadmpeg_ir::math::{Point3 as IrPoint3, Vector3 as IrVector3};
use cadmpeg_ir::products::{
    Occurrence, OccurrenceParent, ProductDefinition, ProductDefinitionKind, PrototypeReference,
};
use cadmpeg_ir::scalar::FiniteReal;
use cadmpeg_ir::topology::{
    Body, BodyKind, Coedge, CoedgeUseCurve, Edge as IrEdge, EdgeCarrier, Face as IrFace, FaceLoops,
    Loop as IrLoop, LoopBoundary, LoopRing, ParameterInterval, Point as IrPoint, Region, Sense,
    Shell as IrShell, Vertex as IrVertex,
};
use cadmpeg_ir::transform::Transform as IrTransform;
use monstertruck_modeling::{
    BoundedCurve, Curve, EuclideanSpace, InnerSpace, MetricSpace, ParametricCurve,
    ParametricSurface, ParametricSurface3D, Point3, SearchParameter, Surface, Transform as _,
    Vector3,
};
use monstertruck_topology::compress::{CompressedEdgeIndex, CompressedFace};

use super::*;

// ---------------------------------------------------------------------------
// Building a document by hand.
// ---------------------------------------------------------------------------

/// An identity in cadmpeg's `<format>:<scope>:<kind>#<key>` grammar.
fn ident<T>(raw: &str) -> T
where
    T: TryFrom<String>,
    T::Error: std::fmt::Debug, {
    T::try_from(raw.to_owned()).expect("a test identity is well formed")
}

fn finite(x: f64, y: f64, z: f64) -> FinitePoint3 {
    FinitePoint3::new(IrPoint3::new(x, y, z)).expect("a test point is finite")
}

/// A minimal document under construction.
///
/// Every helper returns the id it just added, so a test reads as the graph it
/// builds rather than as a wall of string literals.
struct Builder {
    ir: cadmpeg_ir::CadIr,
    counter: usize,
}

impl Default for Builder {
    fn default() -> Self {
        Self {
            ir: cadmpeg_ir::CadIr::empty(),
            counter: 0,
        }
    }
}

impl Builder {
    fn id(&mut self, kind: &str) -> String {
        self.counter += 1;
        format!("test:model:{kind}#{}", self.counter)
    }

    fn point(&mut self, x: f64, y: f64, z: f64) -> String {
        let id = self.id("point");
        self.ir
            .model
            .points
            .push(IrPoint::new(ident(&id), finite(x, y, z), None));
        id
    }

    fn vertex(&mut self, x: f64, y: f64, z: f64) -> String {
        let point = self.point(x, y, z);
        let id = self.id("vertex");
        self.ir.model.vertices.push(IrVertex {
            id: ident(&id),
            point: ident(&point),
            tolerance: None,
        });
        id
    }

    fn surface(&mut self, geometry: SurfaceGeometry) -> String {
        let id = self.id("surface");
        self.ir.model.surfaces.push(IrSurface {
            id: ident(&id),
            geometry,
            source_object: None,
        });
        id
    }

    fn curve(&mut self, geometry: CurveGeometry) -> String {
        let id = self.id("curve");
        self.ir.model.curves.push(IrCurve {
            id: ident(&id),
            geometry,
            parameter_range: None,
            source_object: None,
        });
        id
    }

    fn edge(
        &mut self,
        curve: Option<&str>,
        start: &str,
        end: &str,
        range: Option<[f64; 2]>,
    ) -> String {
        let id = self.id("edge");
        self.ir.model.edges.push(IrEdge {
            id: ident(&id),
            carrier: EdgeCarrier::new(curve.map(ident), range).expect("an admitted carrier"),
            start: ident(start),
            end: ident(end),
            tolerance: None,
        });
        id
    }

    /// A closed ring of coedges over `edges`, all forward, on one loop.
    fn ring(&mut self, face: &str, edges: &[String]) -> String {
        self.ring_with_senses(
            face,
            &edges
                .iter()
                .map(|edge| (edge.clone(), Sense::Forward))
                .collect::<Vec<_>>(),
        )
    }

    fn ring_with_senses(&mut self, face: &str, edges: &[(String, Sense)]) -> String {
        let loop_id = self.id("loop");
        let coedge_ids: Vec<String> = (0..edges.len()).map(|_| self.id("coedge")).collect();
        for (position, (edge, sense)) in edges.iter().enumerate() {
            self.ir.model.coedges.push(Coedge {
                id: ident(&coedge_ids[position]),
                owner_loop: ident(&loop_id),
                edge: ident(edge),
                // Self-referential radial ring: a laminar boundary, which is what
                // a sheet's edges are.
                radial_next: ident(&coedge_ids[position]),
                sense: *sense,
                pcurves: Vec::new(),
                use_curve: None,
            });
        }
        self.ir.model.loops.push(IrLoop {
            id: ident(&loop_id),
            face: ident(face),
            boundary: LoopBoundary::Ring(
                LoopRing::new(coedge_ids.iter().map(|id| ident(id)).collect(), Vec::new())
                    .expect("a ring of distinct coedges"),
            ),
        });
        loop_id
    }

    fn face(&mut self, id: &str, surface: &str, loops: FaceLoops) -> IrFace {
        let placeholder = self.id("shell");
        IrFace {
            id: ident(id),
            // The shell id is not read by the converter, which walks body ->
            // region -> shell -> face rather than back up.
            shell: ident(&placeholder),
            surface: ident(surface),
            sense: Sense::Forward,
            loops,
            name: None,
            color: None,
            tolerance: None,
        }
    }

    /// A body of the given kind over `faces`.
    fn body(&mut self, kind: BodyKind, faces: Vec<IrFace>) -> String {
        let body = self.id("body");
        let region = self.id("region");
        let shell = self.id("shell");
        let face_ids = faces.iter().map(|face| face.id.clone()).collect::<Vec<_>>();
        self.ir.model.faces.extend(faces);
        let shell_entity = if face_ids.is_empty() {
            // A shell must own something; a wire body's owns a free vertex.
            let vertex = self.vertex(0.0, 0.0, 0.0);
            IrShell::new(
                ident(&shell),
                ident(&region),
                Vec::new(),
                Vec::new(),
                vec![ident(&vertex)],
            )
        } else {
            IrShell::new(
                ident(&shell),
                ident(&region),
                face_ids,
                Vec::new(),
                Vec::new(),
            )
        }
        .expect("a nonempty shell");
        self.ir.model.shells.push(shell_entity);
        self.ir.model.regions.push(Region {
            id: ident(&region),
            body: ident(&body),
            shells: vec![ident(&shell)],
        });
        self.ir.model.bodies.push(Body {
            id: ident(&body),
            kind,
            regions: vec![ident(&region)],
            transform: None,
            name: None,
            color: None,
            visible: None,
        });
        body
    }

    fn convert(&self) -> Result<Vec<ImportedBody>> { convert(&self.ir, "TEST") }

    fn import(&self) -> Result<Import> { import(&self.ir, "TEST") }
}

fn solved_surface(geometry: SolvedSurfaceGeometry) -> SurfaceGeometry {
    SurfaceGeometry::Solved(geometry)
}

fn solved_curve(geometry: SolvedCurveGeometry) -> CurveGeometry { CurveGeometry::Solved(geometry) }

/// The z = 0 plane, x-aligned.
fn xy_plane() -> SurfaceGeometry {
    solved_surface(SolvedSurfaceGeometry::Plane(
        PlaneSurface::try_new(
            IrPoint3::new(0.0, 0.0, 0.0),
            IrVector3::new(0.0, 0.0, 1.0),
            IrVector3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    ))
}

/// An infinite line through `from` towards `to`, parameterised by DISTANCE, so an
/// edge on it spans `[0, |to - from|]`.
fn line_through(from: (f64, f64, f64), to: (f64, f64, f64)) -> (CurveGeometry, f64) {
    let direction = IrVector3::new(to.0 - from.0, to.1 - from.1, to.2 - from.2);
    let length =
        (direction.x * direction.x + direction.y * direction.y + direction.z * direction.z).sqrt();
    (
        solved_curve(SolvedCurveGeometry::Line(
            LineCurve::try_new(
                IrPoint3::new(from.0, from.1, from.2),
                IrVector3::new(
                    direction.x / length,
                    direction.y / length,
                    direction.z / length,
                ),
            )
            .unwrap(),
        )),
        length,
    )
}

fn nurbs_curve(
    degree: u32,
    knots: Vec<f64>,
    points: Vec<IrPoint3>,
    weights: Option<Vec<f64>>,
    periodic: bool,
) -> IrNurbsCurve {
    IrNurbsCurve::new(
        degree,
        knots,
        NurbsPoles3::from_lanes(points, weights).unwrap(),
        periodic,
    )
    .expect("an admitted NURBS curve")
}

fn nurbs_curve_geometry(curve: IrNurbsCurve) -> CurveGeometry {
    solved_curve(SolvedCurveGeometry::Nurbs(curve))
}

/// A triangular sheet face on the z = 0 plane: the smallest thing that is a real
/// face rather than a fragment.
fn triangle_sheet() -> Builder {
    let mut builder = Builder::default();
    let corners = [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0)];
    let vertices: Vec<String> = corners
        .iter()
        .map(|(x, y)| builder.vertex(*x, *y, 0.0))
        .collect();
    let surface = builder.surface(xy_plane());
    let face_id = builder.id("face");
    let mut edges = Vec::new();
    for index in 0..3 {
        let (from, to) = (corners[index], corners[(index + 1) % 3]);
        let (geometry, length) = line_through((from.0, from.1, 0.0), (to.0, to.1, 0.0));
        let curve = builder.curve(geometry);
        edges.push(builder.edge(
            Some(&curve),
            &vertices[index],
            &vertices[(index + 1) % 3],
            Some([0.0, length]),
        ));
    }
    let ring = builder.ring(&face_id, &edges);
    let face = builder.face(
        &face_id,
        &surface,
        FaceLoops::classified(ident(&ring), Vec::new()),
    );
    builder.body(BodyKind::Sheet, vec![face]);
    builder
}

fn sheet_of(bodies: &[ImportedBody]) -> &ImportedSheet {
    let ImportedBody::Sheet(sheet) = &bodies[0] else {
        panic!("a sheet body must convert to a sheet, not {:?}", bodies[0]);
    };
    sheet
}

// ---------------------------------------------------------------------------
// The whole path, for the shapes that are supposed to work.
// ---------------------------------------------------------------------------

/// A decoded sheet face becomes a compressed shell whose vertices, edges and
/// surface are the ones the document described.
#[test]
fn a_planar_sheet_face_becomes_a_compressed_shell() {
    let bodies = triangle_sheet().convert().expect("the triangle converts");
    assert_eq!(bodies.len(), 1);
    let sheet = sheet_of(&bodies);
    assert_eq!(sheet.faces.len(), 1);
    // Three edges and three vertices, each SHARED: the ring visits vertex 0
    // twice, once as an end and once as a start, and interning is what makes
    // those the same index.
    assert_eq!(sheet.edges.len(), 3);
    assert_eq!(sheet.vertices.len(), 3);
    assert_eq!(sheet.faces[0].boundaries.len(), 1);
    assert_eq!(sheet.faces[0].boundaries[0].len(), 3);
    assert!(sheet.faces[0].orientation);
    let Surface::Plane(plane) = &sheet.faces[0].surface else {
        panic!(
            "a plane must convert to a plane, not {:?}",
            sheet.faces[0].surface
        );
    };
    assert!(plane.normal().z > 0.999, "normal was {:?}", plane.normal());
    assert!(plane.origin().to_vec().magnitude() < 1.0e-12);
}

/// The invariant `Edge::try_new` will check later: an edge's curve runs from its
/// start vertex to its end vertex, at the curve's own parameter bounds.
#[test]
fn every_converted_edge_curve_runs_from_its_start_vertex_to_its_end_vertex() {
    let bodies = triangle_sheet().convert().expect("the triangle converts");
    let sheet = sheet_of(&bodies);
    for (index, edge) in sheet.edges.iter().enumerate() {
        let (front, back) = (
            sheet.vertices[edge.vertices.0],
            sheet.vertices[edge.vertices.1],
        );
        let (start, end) = edge.curve.range_tuple();
        assert!(
            edge.curve.subs(start).distance(front) < 1.0e-9,
            "edge {index} starts at {:?}, not at its start vertex {front:?}",
            edge.curve.subs(start)
        );
        assert!(
            edge.curve.subs(end).distance(back) < 1.0e-9,
            "edge {index} ends at {:?}, not at its end vertex {back:?}",
            edge.curve.subs(end)
        );
    }
}

/// The strongest oracle available here: monstertruck's OWN extraction accepts the
/// converted shell. `Shell::extract` runs `Edge::try_new` and `Face::try_new`
/// on every entity, so it checks what the assertions above only sample.
#[test]
fn the_converted_shell_is_accepted_by_monstertrucks_own_extraction() {
    use monstertruck_topology::Shell;
    let bodies = triangle_sheet().convert().expect("the triangle converts");
    let shell = Shell::extract(sheet_of(&bodies).clone()).expect("the shell extracts");
    assert_eq!(shell.len(), 1, "one face in, one face out");
    let face = &shell[0];
    assert_eq!(face.absolute_boundaries().len(), 1);
    assert_eq!(face.absolute_boundaries()[0].len(), 3);
    assert!(
        face.absolute_boundaries()[0].is_closed(),
        "the converted boundary must be a closed wire"
    );
}

/// A reversed coedge must show up as an orientation flag, NOT as a reversed curve:
/// the curve is shared between the faces that meet at the edge, and flipping it
/// would flip it for both.
#[test]
fn a_reversed_coedge_flips_the_edge_use_and_not_the_shared_curve() {
    let mut builder = Builder::default();
    let start = builder.vertex(0.0, 0.0, 0.0);
    let end = builder.vertex(2.0, 0.0, 0.0);
    let (geometry, length) = line_through((0.0, 0.0, 0.0), (2.0, 0.0, 0.0));
    let curve = builder.curve(geometry);
    let edge = builder.edge(Some(&curve), &start, &end, Some([0.0, length]));
    let surface = builder.surface(xy_plane());
    let face_id = builder.id("face");
    // A two-coedge ring over ONE edge: degenerate as a face, but it is the
    // smallest graph that carries both senses of the same edge. The ring needs
    // distinct coedges, which it has: one per use.
    let ring = builder.ring_with_senses(
        &face_id,
        &[(edge.clone(), Sense::Forward), (edge, Sense::Reversed)],
    );
    let face = builder.face(
        &face_id,
        &surface,
        FaceLoops::unspecified(vec![ident(&ring)]),
    );
    builder.body(BodyKind::Sheet, vec![face]);
    let bodies = builder.convert().expect("the two-sense ring converts");
    let sheet = sheet_of(&bodies);
    assert_eq!(sheet.edges.len(), 1, "one source edge must intern once");
    let uses = &sheet.faces[0].boundaries[0];
    assert_eq!(uses.len(), 2);
    assert!(uses[0].orientation);
    assert!(!uses[1].orientation);
    assert_eq!(uses[0].index, uses[1].index);
    let (start_parameter, _) = sheet.edges[0].curve.range_tuple();
    assert!(
        sheet.edges[0]
            .curve
            .subs(start_parameter)
            .distance(Point3::origin())
            < 1.0e-12
    );
}

/// A face whose sense is reversed becomes a face with `orientation == false`,
/// which is where `create_face` inverts it. The surface itself must NOT be
/// pre-inverted, or the two flips cancel.
#[test]
fn a_reversed_face_sense_becomes_the_faces_orientation_flag() {
    let mut builder = triangle_sheet();
    builder.ir.model.faces[0].sense = Sense::Reversed;
    let bodies = builder.convert().expect("the triangle converts");
    let sheet = sheet_of(&bodies);
    assert!(!sheet.faces[0].orientation);
    let Surface::Plane(plane) = &sheet.faces[0].surface else {
        unreachable!()
    };
    assert!(
        plane.normal().z > 0.999,
        "the surface must be untouched; the flip belongs to the face"
    );
}

/// The review's F1: the source states the ring in the FACE sense, while
/// `CompressedFace` stores it in the surface's NATURAL sense and `create_face`
/// reverses it back through the flag. Reconstructing the face-sense loop from
/// the stored one must therefore give the source ring, not its reverse.
#[test]
fn a_reversed_face_sense_reverses_the_stored_boundary_with_it() {
    let mut builder = triangle_sheet();
    builder.ir.model.faces[0].sense = Sense::Reversed;
    let bodies = builder.convert().expect("the triangle converts");
    let face = &sheet_of(&bodies).faces[0];
    assert!(!face.orientation);
    // `Face::boundaries` is what a consumer sees: the stored ring, reversed
    // with every use flipped when the flag is false.
    let mut effective: Vec<(usize, bool)> = face.boundaries[0]
        .iter()
        .map(|edge_use| (edge_use.index, edge_use.orientation))
        .collect();
    effective.reverse();
    effective
        .iter_mut()
        .for_each(|(_, orientation)| *orientation = !*orientation);
    assert_eq!(
        effective,
        vec![(0, true), (1, true), (2, true)],
        "the face-sense loop must be the source's ring"
    );
}

/// The review's F2: a negative-determinant placement flips the transformed
/// surface's natural normal, so every face flag must flip with it. Without
/// this, a mirrored body is an inside-out solid that still extracts Closed.
#[test]
fn a_mirrored_body_keeps_every_face_outward() {
    let mut ir = cadmpeg_ir::examples::unit_cube().expect("cadmpeg's cube builds");
    ir.model.bodies[0].transform = Some(
        IrTransform::affine([
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ])
        .expect("a reflection is an affine map"),
    );
    let recovered = import(&ir, "STEP").expect("the mirrored cube converts");
    let ImportedBody::Solid(solid) = &recovered.bodies[0] else {
        panic!("a cube is a solid");
    };
    let shell = &solid.boundaries[0];
    let centroid = Point3::from_vec(
        shell
            .vertices
            .iter()
            .fold(Vector3::new(0.0, 0.0, 0.0), |sum, point| {
                sum + point.to_vec()
            })
            / shell.vertices.len() as f64,
    );
    for (index, face) in shell.faces.iter().enumerate() {
        let (mut sum, mut count) = (Vector3::new(0.0, 0.0, 0.0), 0.0);
        for edge_use in face.boundaries.iter().flatten() {
            let edge = &shell.edges[edge_use.index];
            for vertex in [edge.vertices.0, edge.vertices.1] {
                sum += shell.vertices[vertex].to_vec();
                count += 1.0;
            }
        }
        let face_centre = Point3::from_vec(sum / count);
        let normal = face.surface.normal(0.0, 0.0);
        let oriented = if face.orientation { normal } else { -normal };
        assert!(
            oriented.dot(face_centre - centroid) > 0.0,
            "face {index} faces inward after a mirror"
        );
    }
}

/// The review's F5b: a singular placement must be a typed refusal, not a panic
/// in `Processor::normal` on the first consumer that asks for a normal.
#[test]
fn a_singular_body_placement_is_refused_by_name() {
    let mut ir = cadmpeg_ir::examples::unit_cube().expect("cadmpeg's cube builds");
    ir.model.bodies[0].transform = Some(
        IrTransform::affine([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ])
        .expect("a rank-deficient matrix is representable"),
    );
    let recovered = import(&ir, "STEP").expect("the call itself succeeds");
    assert!(
        recovered.bodies.is_empty(),
        "a singular placement places nothing"
    );
    assert_eq!(recovered.refused.len(), 1);
    assert!(
        matches!(
            recovered.refused[0].reason,
            Error::UnplaceableOccurrence {
                kind: "singular placement",
                ..
            }
        ),
        "got {}",
        recovered.refused[0].reason
    );
}

/// The face's stated outer loop reaches position zero, where `Face::try_new`
/// reads the outer boundary -- even when the source's loop table lists an inner
/// loop first. Since 0.6 the classification lives on the face.
#[test]
fn a_classified_outer_loop_is_put_first() {
    let mut builder = triangle_sheet();
    let inner = {
        let corners = [(1.0, 1.0), (2.0, 1.0), (1.0, 1.5)];
        let vertices: Vec<String> = corners
            .iter()
            .map(|(x, y)| builder.vertex(*x, *y, 0.0))
            .collect();
        let face_id = builder.ir.model.faces[0].id.to_string();
        let mut edges = Vec::new();
        for index in 0..3 {
            let (from, to) = (corners[index], corners[(index + 1) % 3]);
            let (geometry, length) = line_through((from.0, from.1, 0.0), (to.0, to.1, 0.0));
            let curve = builder.curve(geometry);
            edges.push(builder.edge(
                Some(&curve),
                &vertices[index],
                &vertices[(index + 1) % 3],
                Some([0.0, length]),
            ));
        }
        builder.ring(&face_id, &edges)
    };
    let outer = builder.ir.model.faces[0].loops.to_vec()[0].clone();
    // The inner ring was BUILT first in the loop table's order of discovery
    // here; the face's classification is what must decide.
    builder.ir.model.faces[0].loops = FaceLoops::classified(outer, vec![ident(&inner)]);
    let bodies = builder.convert().expect("the two-loop face converts");
    let sheet = sheet_of(&bodies);
    assert_eq!(sheet.faces[0].boundaries.len(), 2);
    let touches_origin = |boundary: &[CompressedEdgeIndex]| {
        boundary.iter().any(|use_of| {
            let edge = &sheet.edges[use_of.index];
            sheet.vertices[edge.vertices.0].to_vec().magnitude() < 1.0e-12
                || sheet.vertices[edge.vertices.1].to_vec().magnitude() < 1.0e-12
        })
    };
    assert!(
        touches_origin(&sheet.faces[0].boundaries[0]),
        "position zero must hold the loop the face classifies as outer"
    );
    assert!(!touches_origin(&sheet.faces[0].boundaries[1]));
}

// ---------------------------------------------------------------------------
// Analytic carriers.
// ---------------------------------------------------------------------------

fn convert_surface(geometry: &SurfaceGeometry) -> Result<surface::Oriented> {
    let ir = cadmpeg_ir::CadIr::empty();
    let context = context_over(&ir);
    surface::convert(geometry, &context)
}

/// A cylinder must keep its analytic form. Landing it on a spline net is the
/// silent loss of exactness this converter exists to avoid.
#[test]
fn a_cylinder_stays_a_surface_of_revolution() {
    let geometry = solved_surface(SolvedSurfaceGeometry::Cylinder(
        CylinderSurface::try_new(
            IrPoint3::new(0.0, 0.0, 0.0),
            IrVector3::new(0.0, 0.0, 1.0),
            IrVector3::new(1.0, 0.0, 0.0),
            5.0,
        )
        .unwrap(),
    ));
    let converted = convert_surface(&geometry).expect("a cylinder converts");
    let Surface::RevolutionSurface(revolution) = &converted.surface else {
        panic!("a cylinder must stay analytic, got {:?}", converted.surface);
    };
    for (u, v) in [(0.0, 0.0), (1.0, 0.5), (2.5, 1.0)] {
        let point = revolution.subs(u, v);
        let radial = (point.x * point.x + point.y * point.y).sqrt();
        assert!(
            (radial - 5.0).abs() < 1.0e-9,
            "{radial} from the axis, not 5"
        );
    }
}

/// A sphere keeps the analytic variant, which is what preserves the closed-form
/// parameter division -- spec 012 U1.2.
#[test]
fn a_sphere_stays_analytic_and_carries_its_radius() {
    let converted = convert_surface(&sphere(7.0)).expect("a sphere converts");
    let Surface::SphericalSurface(analytic) = &converted.surface else {
        panic!("a sphere must stay analytic, got {:?}", converted.surface);
    };
    assert_eq!(analytic.entity().radius(), 7.0);
    assert_eq!(analytic.entity().center(), Point3::new(1.0, -2.0, 3.0));
    assert!(!converted.reversed);
}

/// A torus's axis DOES change its point set, so unlike a sphere's the frame has
/// to survive. Measured on the surface, not on the stored transform.
#[test]
fn a_torus_keeps_the_axis_it_was_given() {
    let geometry = solved_surface(SolvedSurfaceGeometry::Torus(
        TorusSurface::try_new(
            IrPoint3::new(0.0, 0.0, 0.0),
            IrVector3::new(1.0, 0.0, 0.0),
            IrVector3::new(0.0, 1.0, 0.0),
            10.0,
            2.0,
        )
        .unwrap(),
    ));
    let converted = convert_surface(&geometry).expect("a torus converts");
    let Surface::ToroidalSurface(torus) = &converted.surface else {
        panic!("a torus must stay analytic, got {:?}", converted.surface);
    };
    for (u, v) in [(0.0, 0.0), (1.0, 2.0), (3.0, 4.5)] {
        let point = torus.subs(u, v);
        let ring_distance = (point.y * point.y + point.z * point.z).sqrt();
        let to_tube_centre = ((ring_distance - 10.0).powi(2) + point.x * point.x).sqrt();
        assert!(
            (to_tube_centre - 2.0).abs() < 1.0e-9,
            "{to_tube_centre} from the tube centre, not 2"
        );
    }
}

// ---------------------------------------------------------------------------
// Orientation: the silent traps cadmpeg 0.6 introduced.
// ---------------------------------------------------------------------------

/// The oriented normal of an IR face at carrier parameter `(u, v)`, from
/// cadmpeg's OWN evaluator: `Su x Sv`, negated for a NURBS net that says
/// `normal_reversed`, negated again for a reversed face sense. Returns the point
/// too, so the converted face can be probed at the same place.
fn ir_oriented_normal(
    geometry: &SurfaceGeometry,
    sense: Sense,
    u: f64,
    v: f64,
) -> (Point3, Vector3) {
    let partials =
        cadmpeg_ir::eval::surface_partials(EvaluationAdmission::Standard, geometry, u, v)
            .expect("the carrier evaluates")
            .into_raw();
    let du = Vector3::new(partials.du.x, partials.du.y, partials.du.z);
    let dv = Vector3::new(partials.dv.x, partials.dv.y, partials.dv.z);
    let mut normal = du.cross(dv).normalize();
    if let Some(SolvedSurfaceGeometry::Nurbs(net)) = geometry.solved()
        && net.normal_reversed()
    {
        normal = -normal;
    }
    if sense == Sense::Reversed {
        normal = -normal;
    }
    (
        Point3::new(partials.point.x, partials.point.y, partials.point.z),
        normal,
    )
}

/// The converted face's oriented normal at a model-space point on it.
fn converted_oriented_normal(face: &CompressedFace<Surface>, point: Point3) -> Vector3 {
    let (s, t) = SearchParameter::<monstertruck_modeling::D2>::search_parameter(
        &face.surface,
        point,
        None,
        100,
    )
    .unwrap_or_else(|| panic!("{point:?} must lie on the converted surface"));
    assert!(
        face.surface.subs(s, t).distance(point) < 1.0e-7,
        "the converted surface does not pass through the source point {point:?}"
    );
    let normal = face.surface.normal(s, t);
    if face.orientation { normal } else { -normal }
}

/// Put `geometry` under the triangle's face with `sense`, convert, and assert
/// the converted face faces the way cadmpeg's evaluator says the source face
/// does, at every sample.
///
/// The triangle's edges do not lie on the swapped-in surface. That does not
/// matter here: the converter maps the face's surface and sense independently
/// of its boundary, and what is under test is exactly that mapping, composed
/// through the real topology layer rather than called in isolation.
fn assert_face_faces_like_the_source(
    geometry: SurfaceGeometry,
    sense: Sense,
    samples: &[(f64, f64)],
) {
    let mut builder = triangle_sheet();
    builder.ir.model.surfaces[0].geometry = geometry.clone();
    builder.ir.model.faces[0].sense = sense;
    let bodies = builder.convert().expect("the face converts");
    let face = &sheet_of(&bodies).faces[0];
    for &(u, v) in samples {
        let (point, expected) = ir_oriented_normal(&geometry, sense, u, v);
        let got = converted_oriented_normal(face, point);
        assert!(
            got.dot(expected) > 0.999,
            "at source ({u}, {v}) = {point:?} the source face faces {expected:?}, the converted \
             face faces {got:?} -- a flipped face"
        );
    }
}

fn sphere(radius: f64) -> SurfaceGeometry {
    solved_surface(SolvedSurfaceGeometry::Sphere(
        SphereSurface::try_new(
            IrPoint3::new(1.0, -2.0, 3.0),
            IrVector3::new(0.0, 0.0, 1.0),
            IrVector3::new(1.0, 0.0, 0.0),
            radius,
        )
        .unwrap(),
    ))
}

fn torus(minor: f64) -> SurfaceGeometry {
    solved_surface(SolvedSurfaceGeometry::Torus(
        TorusSurface::try_new(
            IrPoint3::new(0.5, 0.0, -1.0),
            IrVector3::new(0.0, 1.0, 0.0),
            IrVector3::new(0.0, 0.0, 1.0),
            6.0,
            minor,
        )
        .unwrap(),
    ))
}

/// A bilinear NURBS patch over the unit square in z = 0, so `Pu x Pv = +z`.
fn flat_net(normal_reversed: bool) -> SurfaceGeometry {
    solved_surface(SolvedSurfaceGeometry::Nurbs(
        IrNurbsSurface::new(
            NurbsSurfaceAxis::new(1, vec![0.0, 0.0, 1.0, 1.0], false),
            NurbsSurfaceAxis::new(1, vec![0.0, 0.0, 1.0, 1.0], false),
            NurbsPoleGrid::from_lanes(
                vec![
                    vec![IrPoint3::new(0.0, 0.0, 0.0), IrPoint3::new(0.0, 1.0, 0.0)],
                    vec![IrPoint3::new(1.0, 0.0, 0.0), IrPoint3::new(1.0, 1.0, 0.0)],
                ],
                None,
            )
            .unwrap(),
            normal_reversed,
        )
        .unwrap(),
    ))
}

const SPHERE_SAMPLES: [(f64, f64); 4] = [(0.3, 0.2), (2.0, -0.7), (4.0, 1.1), (5.5, -1.3)];
const TORUS_SAMPLES: [(f64, f64); 4] = [(0.3, 0.2), (2.0, 2.5), (4.0, 4.0), (5.5, 5.9)];

/// TRAP 1. A NURBS net whose oriented normal is `-(Pu x Pv)` must not flip its
/// face: the flag is folded into the face's orientation exactly once.
#[test]
fn a_reversed_normal_nurbs_face_does_not_flip() {
    for (normal_reversed, sense) in [
        (false, Sense::Forward),
        (true, Sense::Forward),
        (false, Sense::Reversed),
        (true, Sense::Reversed),
    ] {
        assert_face_faces_like_the_source(
            flat_net(normal_reversed),
            sense,
            &[(0.25, 0.25), (0.5, 0.75), (0.9, 0.1)],
        );
    }
    // And the surface itself is untouched: the flip belongs to the face.
    let converted = convert_surface(&flat_net(true)).expect("the net converts");
    assert!(converted.reversed);
    assert!(
        converted.surface.normal(0.5, 0.5).z > 0.999,
        "the net must keep its own Pu x Pv"
    );
}

/// TRAP 2a. A negative sphere radius is the same sphere reached through the
/// antipodal chart, facing INWARD. It must convert to the positive-radius
/// sphere with the face flipped -- not be refused, and not face outward.
#[test]
fn a_negative_radius_sphere_is_the_same_sphere_facing_inward() {
    let converted = convert_surface(&sphere(-2.0)).expect("a negative-radius sphere converts");
    let Surface::SphericalSurface(analytic) = &converted.surface else {
        panic!("a sphere must stay analytic, got {:?}", converted.surface);
    };
    assert_eq!(analytic.entity().radius(), 2.0);
    assert!(converted.reversed, "the antipodal chart faces inward");
    for radius in [2.0, -2.0] {
        for sense in [Sense::Forward, Sense::Reversed] {
            assert_face_faces_like_the_source(sphere(radius), sense, &SPHERE_SAMPLES);
        }
    }
}

/// TRAP 2b. A negative torus tube radius shifts the tube angle by half a turn
/// and keeps the natural normal pointing out of the tube, so it converts to the
/// positive tube with NO flip. Getting this wrong in either direction flips
/// every face on a concave fillet.
#[test]
fn a_negative_tube_radius_torus_is_the_same_torus_facing_the_same_way() {
    let converted = convert_surface(&torus(-1.5)).expect("a negative-tube torus converts");
    assert!(matches!(converted.surface, Surface::ToroidalSurface(_)));
    assert!(!converted.reversed);
    for minor in [1.5, -1.5] {
        for sense in [Sense::Forward, Sense::Reversed] {
            assert_face_faces_like_the_source(torus(minor), sense, &TORUS_SAMPLES);
        }
    }
}

/// The analytic arms that were never orientation-tested before 0.6, under the
/// same oracle: a cylinder, a cone of either taper sign, and a cone posed at its
/// apex (radius zero, admitted since 0.6).
#[test]
fn cylinders_and_cones_face_like_the_source() {
    let cylinder = solved_surface(SolvedSurfaceGeometry::Cylinder(
        CylinderSurface::try_new(
            IrPoint3::new(1.0, 2.0, 3.0),
            IrVector3::new(0.0, 0.6, 0.8),
            IrVector3::new(1.0, 0.0, 0.0),
            2.5,
        )
        .unwrap(),
    ));
    let cone = |radius: f64, half_angle: f64| {
        solved_surface(SolvedSurfaceGeometry::Cone(
            ConeSurface::try_new(
                IrPoint3::new(0.0, 0.0, 1.0),
                IrVector3::new(0.0, 0.0, 1.0),
                IrVector3::new(1.0, 0.0, 0.0),
                radius,
                1.0,
                half_angle,
            )
            .unwrap(),
        ))
    };
    let samples = [(0.3, 0.4), (2.0, 1.5), (4.5, 0.9)];
    for geometry in [cylinder, cone(3.0, 0.4), cone(3.0, -0.4), cone(0.0, 0.5)] {
        for sense in [Sense::Forward, Sense::Reversed] {
            assert_face_faces_like_the_source(geometry.clone(), sense, &samples);
        }
    }
}

// ---------------------------------------------------------------------------
// Curves.
// ---------------------------------------------------------------------------

fn convert_curve(geometry: &CurveGeometry, interval: curve::Interval) -> Result<Curve> {
    // The endpoints are only consulted for a line with no interval, and for a
    // closed circular edge's full traversal.
    convert_curve_between(geometry, interval, (Point3::origin(), Point3::origin()))
}

fn convert_curve_between(
    geometry: &CurveGeometry,
    interval: curve::Interval,
    endpoints: (Point3, Point3),
) -> Result<Curve> {
    let ir = cadmpeg_ir::CadIr::empty();
    let context = context_over(&ir);
    curve::convert(geometry, interval, endpoints, &context)
}

/// A circular edge becomes an EXACT rational arc between its vertices, at the
/// angles the file stated.
#[test]
fn a_circular_edge_becomes_an_exact_arc_between_its_vertices() {
    use std::f64::consts::FRAC_PI_2;
    let geometry = solved_curve(SolvedCurveGeometry::Circle(
        CircleCurve::try_new(
            IrPoint3::new(0.0, 0.0, 0.0),
            IrVector3::new(0.0, 0.0, 1.0),
            IrVector3::new(1.0, 0.0, 0.0),
            3.0,
        )
        .unwrap(),
    ));
    let converted = convert_curve(&geometry, Some([0.0, FRAC_PI_2])).expect("a quarter arc");
    let (start, end) = converted.range_tuple();
    assert!(converted.subs(start).distance(Point3::new(3.0, 0.0, 0.0)) < 1.0e-12);
    assert!(converted.subs(end).distance(Point3::new(0.0, 3.0, 0.0)) < 1.0e-12);
    for step in 1..8 {
        let t = start + (end - start) * f64::from(step) / 8.0;
        let point = converted.subs(t);
        let radial = (point.x * point.x + point.y * point.y).sqrt();
        assert!((radial - 3.0).abs() < 1.0e-12, "off the circle at t = {t}");
        assert!(point.z.abs() < 1.0e-12);
    }
}

fn polyline_spline() -> CurveGeometry {
    nurbs_curve_geometry(nurbs_curve(
        1,
        vec![0.0, 0.0, 1.0, 2.0, 2.0],
        vec![
            IrPoint3::new(0.0, 0.0, 0.0),
            IrPoint3::new(1.0, 0.0, 0.0),
            IrPoint3::new(2.0, 0.0, 0.0),
        ],
        None,
        false,
    ))
}

/// A free-form edge is restricted to the knot interval the file stated, and the
/// restriction keeps the curve's own parameterization.
#[test]
fn a_nurbs_edge_is_restricted_to_its_stated_knot_interval() {
    let full = convert_curve(&polyline_spline(), None).expect("the polyline spline converts");
    assert_eq!(full.range_tuple(), (0.0, 2.0));
    let restricted = convert_curve(&polyline_spline(), Some([0.5, 1.5])).expect("the span");
    let (start, end) = restricted.range_tuple();
    assert!((start - 0.5).abs() < 1.0e-12, "range started at {start}");
    assert!((end - 1.5).abs() < 1.0e-12, "range ended at {end}");
    assert!(restricted.subs(start).distance(Point3::new(0.5, 0.0, 0.0)) < 1.0e-12);
    assert!(restricted.subs(end).distance(Point3::new(1.5, 0.0, 0.0)) < 1.0e-12);
}

/// An interval that runs BACKWARDS along the carrier gives a curve that still
/// starts at the edge's start vertex. 0.6 orders edge intervals, so a decoded
/// document no longer carries one; the arm is kept for direct callers.
#[test]
fn a_backwards_interval_gives_a_curve_that_still_starts_at_the_start_vertex() {
    let converted = convert_curve(&polyline_spline(), Some([1.5, 0.5])).expect("the span");
    let (start, end) = converted.range_tuple();
    assert!(converted.subs(start).distance(Point3::new(1.5, 0.0, 0.0)) < 1.0e-12);
    assert!(converted.subs(end).distance(Point3::new(0.5, 0.0, 0.0)) < 1.0e-12);
}

/// A rational free-form curve keeps its weights.
#[test]
fn a_rational_nurbs_edge_keeps_its_weights() {
    use std::f64::consts::FRAC_1_SQRT_2;
    let geometry = nurbs_curve_geometry(nurbs_curve(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            IrPoint3::new(1.0, 0.0, 0.0),
            IrPoint3::new(1.0, 1.0, 0.0),
            IrPoint3::new(0.0, 1.0, 0.0),
        ],
        Some(vec![1.0, FRAC_1_SQRT_2, 1.0]),
        false,
    ));
    let converted = convert_curve(&geometry, None).expect("the rational quarter circle");
    let point = converted.subs(0.5);
    let radial = (point.x * point.x + point.y * point.y).sqrt();
    assert!(
        (radial - 1.0).abs() < 1.0e-12,
        "the midpoint sits at radius {radial}, so the weights were lost"
    );
}

// ---------------------------------------------------------------------------
// TRAP 3: periodic and unclamped knot vectors map exactly.
// ---------------------------------------------------------------------------

/// cadmpeg's own evaluation of a curve carrier at `t`, which wraps a periodic
/// parameter modulo its period.
fn ir_curve_point(geometry: &CurveGeometry, t: f64) -> Point3 {
    let point = cadmpeg_ir::eval::decode::curve_point(EvaluationAdmission::Standard, geometry, t)
        .unwrap_or_else(|error| panic!("the source evaluates at {t}: {error:?}"))
        .get();
    Point3::new(point.x, point.y, point.z)
}

/// Assert that `converted`, over its own range, traces the source over
/// `[from, to]`, sample for sample, in the same direction and at the same rate.
fn assert_traces(converted: &Curve, geometry: &CurveGeometry, (from, to): (f64, f64)) {
    let (start, end) = converted.range_tuple();
    assert!(
        ((end - start) - (to - from)).abs() < 1.0e-9,
        "the converted range [{start}, {end}] is not the width of [{from}, {to}]"
    );
    for step in 0..=16 {
        let fraction = f64::from(step) / 16.0;
        let source = ir_curve_point(geometry, from + (to - from) * fraction);
        let got = converted.subs(start + (end - start) * fraction);
        assert!(
            got.distance(source) < 1.0e-9,
            "at fraction {fraction} the source is at {source:?}, the conversion at {got:?}"
        );
    }
}

fn is_clamped(curve: &Curve) -> bool {
    match curve {
        Curve::BsplineCurve(net) => net.is_clamped(),
        Curve::NurbsCurve(net) => net.non_rationalized().is_clamped(),
        _ => true,
    }
}

/// A quadratic spline whose knot vector is UNCLAMPED at both ends: only
/// `[knot[2], knot[4]] = [2, 4]` is the carrier. Clamping at the full extent
/// would keep the extrapolation spans `[0, 2]` and `[4, 6]`, which are not.
#[test]
fn an_unclamped_nurbs_curve_maps_onto_its_effective_domain_exactly() {
    let geometry = nurbs_curve_geometry(nurbs_curve(
        2,
        vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        vec![
            IrPoint3::new(0.0, 0.0, 0.0),
            IrPoint3::new(1.0, 2.0, 0.0),
            IrPoint3::new(3.0, 2.0, 1.0),
            IrPoint3::new(4.0, 0.0, 0.0),
        ],
        None,
        false,
    ));
    let whole = convert_curve(&geometry, None).expect("an unclamped spline converts");
    assert_eq!(whole.range_tuple(), (2.0, 4.0));
    assert!(is_clamped(&whole), "the conversion must be clamped");
    assert_traces(&whole, &geometry, (2.0, 4.0));
    // A sub-interval touching the domain's start, which `restrict` leaves
    // uncut: still clamped, still the carrier.
    let span = convert_curve(&geometry, Some([2.0, 3.5])).expect("a span");
    assert!(is_clamped(&span));
    assert_traces(&span, &geometry, (2.0, 3.5));
}

/// A closed periodic quadratic: four distinct poles, the first two repeated at
/// the end, uniform knots. Domain `[2, 6]`, period 4.
fn closed_periodic(weights: Option<Vec<f64>>) -> CurveGeometry {
    let ring = [
        IrPoint3::new(2.0, 0.0, 0.0),
        IrPoint3::new(0.0, 2.0, 0.0),
        IrPoint3::new(-2.0, 0.0, 0.5),
        IrPoint3::new(0.0, -2.0, 0.0),
    ];
    let mut poles = ring.to_vec();
    poles.extend_from_slice(&ring[..2]);
    nurbs_curve_geometry(nurbs_curve(
        2,
        (0..9).map(f64::from).collect(),
        poles,
        weights,
        true,
    ))
}

/// A periodic curve converts exactly, whole or over any interval cadmpeg can
/// state -- inside the domain, shifted by whole periods, and straddling the seam.
#[test]
fn a_periodic_nurbs_curve_maps_exactly_over_every_interval() {
    for weights in [None, Some(vec![1.0, 0.5, 2.0, 1.0, 1.0, 0.5])] {
        let geometry = closed_periodic(weights);
        let whole = convert_curve(&geometry, None).expect("a periodic curve converts");
        assert_eq!(whole.range_tuple(), (2.0, 6.0));
        assert!(is_clamped(&whole));
        assert_traces(&whole, &geometry, (2.0, 6.0));
        let (start, end) = whole.range_tuple();
        assert!(
            whole.subs(start).distance(whole.subs(end)) < 1.0e-9,
            "a closed periodic curve converts to a closed curve"
        );
        for interval in [
            (2.5, 5.0),  // inside the domain
            (6.5, 9.0),  // the same span, one period up
            (-1.5, 1.0), // the same span, one period down
            (5.0, 7.25), // straddling the seam at 6
            (1.0, 2.5),  // straddling the seam from below
            (2.0, 6.0),  // exactly one turn
        ] {
            let converted = convert_curve(&geometry, Some([interval.0, interval.1]))
                .unwrap_or_else(|error| panic!("{interval:?} converts: {error}"));
            assert!(is_clamped(&converted));
            assert_traces(&converted, &geometry, interval);
        }
    }
}

/// A periodic net that does NOT meet itself at its seam evaluates
/// discontinuously there. An edge across that seam has no exact single curve,
/// so it is refused by name rather than bridged with a jump.
#[test]
fn a_periodic_edge_across_a_seam_that_does_not_close_is_refused_by_name() {
    let geometry = nurbs_curve_geometry(nurbs_curve(
        1,
        vec![0.0, 1.0, 2.0, 3.0, 4.0],
        vec![
            IrPoint3::new(0.0, 0.0, 0.0),
            IrPoint3::new(1.0, 0.0, 0.0),
            IrPoint3::new(1.0, 1.0, 0.0),
        ],
        None,
        true,
    ));
    // Domain [knot[1], knot[3]] = [1, 3], period 2. The curve runs (0,0,0) ->
    // (1,0,0) -> (1,1,0) and jumps back to the origin at the seam.
    let error = convert_curve(&geometry, Some([2.5, 3.5])).expect_err("the seam is open");
    assert!(
        matches!(
            error,
            Error::UnsupportedCurveKind {
                kind: "periodic NURBS edge across a seam the curve does not close at",
                ..
            }
        ),
        "got {error}"
    );
    // Inside the domain the same curve converts fine.
    convert_curve(&geometry, Some([1.25, 2.75])).expect("an interval inside the domain");
}

/// cadmpeg's own evaluation of a surface carrier at `(u, v)`.
fn ir_surface_point(geometry: &SurfaceGeometry, u: f64, v: f64) -> Point3 {
    let point =
        cadmpeg_ir::eval::decode::surface_point(EvaluationAdmission::Standard, geometry, u, v)
            .unwrap_or_else(|error| panic!("the source evaluates at ({u}, {v}): {error:?}"))
            .get();
    Point3::new(point.x, point.y, point.z)
}

/// A net periodic in u (a closed quadratic ring, swept linearly in v) and
/// unclamped in v. Both must convert to a clamped net over the effective domain
/// that reproduces cadmpeg's own evaluation everywhere in it.
#[test]
fn periodic_and_unclamped_nurbs_surfaces_map_exactly() {
    let ring = [
        (2.0, 0.0),
        (0.0, 2.0),
        (-2.0, 0.0),
        (0.0, -2.0),
        (2.0, 0.0),
        (0.0, 2.0),
    ];
    let rows: Vec<Vec<IrPoint3>> = ring
        .iter()
        .map(|(x, y)| {
            (0..3)
                .map(|level| IrPoint3::new(*x, *y, f64::from(level) * 1.5))
                .collect()
        })
        .collect();
    let geometry = solved_surface(SolvedSurfaceGeometry::Nurbs(
        IrNurbsSurface::new(
            NurbsSurfaceAxis::new(2, (0..9).map(f64::from).collect::<Vec<_>>(), true),
            // Unclamped, degree 1 over 3 poles: domain [knot[1], knot[3]] = [1, 3].
            NurbsSurfaceAxis::new(1, vec![0.0, 1.0, 2.0, 3.0, 4.0], false),
            NurbsPoleGrid::from_lanes(rows, None).unwrap(),
            false,
        )
        .unwrap(),
    ));
    let converted = convert_surface(&geometry).expect("the net converts");
    let Surface::BsplineSurface(net) = &converted.surface else {
        panic!(
            "a polynomial net stays polynomial, got {:?}",
            converted.surface
        );
    };
    assert!(net.is_clamped());
    let (u_range, v_range) = monstertruck_modeling::BoundedSurface::range_tuple(net);
    assert_eq!(u_range, (2.0, 6.0));
    assert_eq!(v_range, (1.0, 3.0));
    for u in [2.0, 2.7, 4.0, 5.3, 6.0] {
        for v in [1.0, 1.4, 2.0, 3.0] {
            let source = ir_surface_point(&geometry, u, v);
            let got = net.subs(u, v);
            assert!(
                got.distance(source) < 1.0e-9,
                "at ({u}, {v}) the source is at {source:?}, the conversion at {got:?}"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Procedural carriers: the solved cache, when the construction says it is exact.
// ---------------------------------------------------------------------------

/// Put the triangle's face on a `Procedural` carrier over a `Sum` construction
/// of two of its own edge lines, with `cache` and a construction fit tolerance
/// of `fit`.
fn procedural_triangle(cache: Option<SolvedSurfaceGeometry>, fit: Option<f64>) -> Builder {
    use cadmpeg_ir::geometry::surface_payloads::SumSurfaceConstruction;
    use cadmpeg_ir::geometry::{
        CacheContract, LegacyCache, ProceduralSurface, ProceduralSurfaceDefinition,
    };
    let mut builder = triangle_sheet();
    let construction = builder.id("procedural-surface");
    let (u, v) = (
        builder.ir.model.curves[0].id.clone(),
        builder.ir.model.curves[2].id.clone(),
    );
    let mut definition = ProceduralSurfaceDefinition::Sum(
        SumSurfaceConstruction::try_new(
            u,
            v,
            IrVector3::new(0.0, 0.0, 0.0),
            CacheContract::from_form(None),
        )
        .unwrap(),
    );
    if let Some(fit) = fit {
        definition
            .set_legacy_cache(Some(LegacyCache::try_new(fit).unwrap()))
            .unwrap();
    }
    builder.ir.model.surfaces[0].geometry = SurfaceGeometry::Procedural {
        construction: ident(&construction),
        cache,
    };
    builder
        .ir
        .model
        .procedural_surfaces
        .push(ProceduralSurface::new(
            ident(&construction),
            definition,
            None,
        ));
    builder
}

fn plane_cache() -> SolvedSurfaceGeometry {
    let SurfaceGeometry::Solved(plane) = xy_plane() else {
        unreachable!()
    };
    plane
}

/// A procedural carrier with an exact cache converts AS THE CACHE: the
/// construction is not replayed, which is why this fixture's plane cache under a
/// `Sum`-of-two-lines construction still comes out a plane. That is the policy
/// the representation states (`SurfaceGeometry::solved` evaluates through the
/// cache), not evidence that a shipped decoder emits such a carrier; see
/// `super::surface` on why neither STEP nor IGES exercises this arm at 0.6.11.
#[test]
fn a_procedural_carrier_uses_its_exact_cache_and_ignores_the_construction() {
    let bodies = procedural_triangle(Some(plane_cache()), None)
        .convert()
        .expect("an exact cache converts");
    let face = &sheet_of(&bodies).faces[0];
    let Surface::Plane(plane) = &face.surface else {
        panic!("the cache is a plane, got {:?}", face.surface);
    };
    assert!(plane.normal().z > 0.999);
    assert!(face.orientation);
}

/// A procedural CURVE with an exact cache converts through it, and its
/// construction is not replayed. The surface arm had a row; this one did not.
#[test]
fn a_procedural_curve_uses_its_exact_cache_and_ignores_the_construction() {
    use cadmpeg_ir::geometry::{ProceduralCurve, ProceduralCurveDefinition};
    let mut builder = triangle_sheet();
    let construction = builder.id("procedural-curve");
    builder
        .ir
        .model
        .procedural_curves
        .push(ProceduralCurve::new(
            ident(&construction),
            ProceduralCurveDefinition::Exact { cache: None },
        ));
    let cache = SolvedCurveGeometry::Line(
        LineCurve::try_new(IrPoint3::new(0.0, 0.0, 0.0), IrVector3::new(1.0, 0.0, 0.0)).unwrap(),
    );
    let geometry = CurveGeometry::Procedural {
        construction: ident(&construction),
        cache: Some(cache),
    };
    let context = context_over(&builder.ir);
    let curve = curve::convert(
        &geometry,
        Some([0.0, 2.0]),
        (Point3::origin(), Point3::new(2.0, 0.0, 0.0)),
        &context,
    )
    .expect("an exact cache converts");
    let (start, end) = curve.range_tuple();
    assert!(curve.subs(start).distance(Point3::origin()) < 1.0e-12);
    assert!(curve.subs(end).distance(Point3::new(2.0, 0.0, 0.0)) < 1.0e-12);
}

/// A FITTED curve cache is an approximation of the construction, and a curve
/// with no cache would have to be replayed. Both are refused by name.
#[test]
fn a_procedural_curve_with_a_fitted_or_absent_cache_is_refused_by_name() {
    use cadmpeg_ir::geometry::{LegacyCache, ProceduralCurve, ProceduralCurveDefinition};
    for (cache, definition, expected) in [
        (
            None,
            ProceduralCurveDefinition::Exact { cache: None },
            "procedural (no solved cache)",
        ),
        (
            Some(SolvedCurveGeometry::Line(
                LineCurve::try_new(IrPoint3::new(0.0, 0.0, 0.0), IrVector3::new(1.0, 0.0, 0.0))
                    .unwrap(),
            )),
            ProceduralCurveDefinition::Exact {
                cache: Some(LegacyCache::try_new(1.0e-3).unwrap()),
            },
            "procedural (fitted cache)",
        ),
    ] {
        let mut builder = triangle_sheet();
        let construction = builder.id("procedural-curve");
        builder
            .ir
            .model
            .procedural_curves
            .push(ProceduralCurve::new(ident(&construction), definition));
        let geometry = CurveGeometry::Procedural {
            construction: ident(&construction),
            cache,
        };
        let context = context_over(&builder.ir);
        let error = curve::convert(
            &geometry,
            None,
            (Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            &context,
        )
        .expect_err("must be refused");
        assert!(
            matches!(error, Error::UnsupportedCurveKind { kind, .. } if kind == expected),
            "expected {expected:?}, got {error}"
        );
    }
}

/// A FITTED cache is an approximation of the construction, and a carrier with no
/// cache would have to be replayed. Both are refused by name.
#[test]
fn a_procedural_carrier_with_a_fitted_or_absent_cache_is_refused_by_name() {
    for (cache, fit, expected) in [
        (
            Some(plane_cache()),
            Some(1.0e-3),
            "procedural (fitted cache)",
        ),
        (None, None, "procedural (no solved cache)"),
    ] {
        let error = procedural_triangle(cache, fit)
            .convert()
            .expect_err("must be refused");
        assert!(
            matches!(error, Error::UnsupportedSurfaceKind { kind, .. } if kind == expected),
            "expected {expected:?}, got {error}"
        );
    }
}

// ---------------------------------------------------------------------------
// Refusals. Every arm below is a path that must FAIL, and be seen to fail.
// ---------------------------------------------------------------------------

/// The invariant the whole converter is built around: a body it cannot represent
/// fails the strict call by name. It is never dropped from the returned list.
#[test]
fn a_wire_body_is_refused_by_name_and_not_dropped() {
    let mut builder = triangle_sheet();
    builder.body(BodyKind::Wire, Vec::new());
    let error = builder
        .convert()
        .expect_err("a wire body must fail the call");
    assert!(
        matches!(error, Error::UnsupportedBodyKind { kind: "wire", .. }),
        "expected a named wire refusal, got {error}"
    );
}

#[test]
fn a_mixed_dimensional_body_is_refused_by_name() {
    let mut builder = Builder::default();
    builder.body(BodyKind::General, Vec::new());
    let error = builder.convert().expect_err("a general body must fail");
    assert!(
        matches!(
            error,
            Error::UnsupportedBodyKind {
                kind: "mixed-dimensional",
                ..
            }
        ),
        "got {error}"
    );
}

/// A 0.6 shell can own wire edges beside its faces. Converting the faces alone
/// would drop them without a word, so the shell is refused by name.
#[test]
fn a_shell_carrying_wire_edges_is_refused_rather_than_trimmed() {
    let mut builder = triangle_sheet();
    let edge = builder.ir.model.edges[0].id.clone();
    let shell = &mut builder.ir.model.shells[0];
    shell.add_wire_edge(edge);
    let error = builder
        .convert()
        .expect_err("the wire edge must not vanish");
    assert!(
        matches!(
            error,
            Error::UnsupportedBodyKind {
                kind: "shell with wire edges or free vertices",
                ..
            }
        ),
        "got {error}"
    );
}

/// A spindle torus must not reach the analytic variant: its forward map is exact
/// while `search_parameter` is wrong over roughly a third of the domain. Spec
/// 011 T1. A NEGATIVE tube radius past the major radius is the same spindle.
#[test]
fn a_spindle_torus_is_refused_rather_than_routed_analytically() {
    for minor in [4.0, -4.0] {
        let geometry = solved_surface(SolvedSurfaceGeometry::Torus(
            TorusSurface::try_new(
                IrPoint3::new(0.0, 0.0, 0.0),
                IrVector3::new(0.0, 0.0, 1.0),
                IrVector3::new(1.0, 0.0, 0.0),
                1.0,
                minor,
            )
            .unwrap(),
        ));
        let error = convert_surface(&geometry).expect_err("a spindle must be refused");
        assert!(
            matches!(
                error,
                Error::UnsupportedSurfaceKind {
                    kind: "degenerate torus",
                    ..
                }
            ),
            "minor {minor}: got {error}"
        );
    }
}

/// An elliptical cone is not a surface of revolution, so it must not silently
/// become the circular cone that `ratio == 1` would have described.
#[test]
fn an_elliptical_cone_is_refused_rather_than_rounded_to_a_circular_one() {
    let geometry = solved_surface(SolvedSurfaceGeometry::Cone(
        ConeSurface::try_new(
            IrPoint3::new(0.0, 0.0, 0.0),
            IrVector3::new(0.0, 0.0, 1.0),
            IrVector3::new(1.0, 0.0, 0.0),
            2.0,
            0.5,
            0.3,
        )
        .unwrap(),
    ));
    let error = convert_surface(&geometry).expect_err("an ellipse must be refused");
    assert!(
        matches!(
            error,
            Error::UnsupportedSurfaceKind {
                kind: "elliptical cone",
                ..
            }
        ),
        "got {error}"
    );
}

/// A tessellation with a chordal bound must not be presented as exact geometry.
#[test]
fn a_source_native_polygonal_surface_is_refused() {
    let polygonal = PolygonalSurface::new(
        vec![
            IrPoint3::new(0.0, 0.0, 0.0),
            IrPoint3::new(1.0, 0.0, 0.0),
            IrPoint3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
        0.01,
    )
    .expect("an admitted polygonal surface");
    let geometry = solved_surface(SolvedSurfaceGeometry::Polygonal(polygonal));
    let error = convert_surface(&geometry).expect_err("a mesh must be refused");
    assert!(
        matches!(
            error,
            Error::UnsupportedSurfaceKind {
                kind: "source-native polygonal",
                ..
            }
        ),
        "got {error}"
    );
}

/// A NURBS net that is a FIT to another carrier must not be presented as the
/// exact surface.
#[test]
fn a_fitted_nurbs_surface_is_refused_rather_than_called_exact() {
    let SurfaceGeometry::Solved(SolvedSurfaceGeometry::Nurbs(mut net)) = flat_net(false) else {
        unreachable!()
    };
    net.set_cache_fit_tolerance(Some(
        cadmpeg_ir::geometry::FitTolerance::try_new(1.0e-3).unwrap(),
    ));
    let error = convert_surface(&solved_surface(SolvedSurfaceGeometry::Nurbs(net)))
        .expect_err("a fit must be refused");
    assert!(
        matches!(
            error,
            Error::UnsupportedSurfaceKind {
                kind: "fitted NURBS surface",
                ..
            }
        ),
        "got {error}"
    );
}

/// An ellipse is exactly representable in principle but has no builder here, so
/// it is refused by name rather than approximated.
#[test]
fn an_elliptical_edge_is_refused_by_name() {
    let geometry = solved_curve(SolvedCurveGeometry::Ellipse(
        EllipseCurve::try_new(
            IrPoint3::new(0.0, 0.0, 0.0),
            IrVector3::new(0.0, 0.0, 1.0),
            IrVector3::new(1.0, 0.0, 0.0),
            3.0,
            2.0,
        )
        .unwrap(),
    ));
    let error = convert_curve(&geometry, Some([0.0, 1.0])).expect_err("an ellipse is refused");
    assert!(
        matches!(
            error,
            Error::UnsupportedCurveKind {
                kind: "elliptical",
                ..
            }
        ),
        "got {error}"
    );
}

/// A circle with no stated arc and DISTINCT ends is ambiguous between two arcs,
/// so guessing is refused.
#[test]
fn a_circular_edge_with_no_parameter_range_is_refused() {
    let geometry = solved_curve(SolvedCurveGeometry::Circle(
        CircleCurve::try_new(
            IrPoint3::new(0.0, 0.0, 0.0),
            IrVector3::new(0.0, 0.0, 1.0),
            IrVector3::new(1.0, 0.0, 0.0),
            1.0,
        )
        .unwrap(),
    ));
    let endpoints = (Point3::new(1.0, 0.0, 0.0), Point3::new(-1.0, 0.0, 0.0));
    let error = convert_curve_between(&geometry, None, endpoints)
        .expect_err("an untrimmed circle with distinct ends is refused");
    assert!(
        matches!(error, Error::MalformedGeometry { .. }),
        "got {error}"
    );
}

/// A circle whose ends COINCIDE is one closed traversal, and the IR supplies
/// its range. Refusing this cost whole bodies of the in-repo STEP corpus.
#[test]
fn a_closed_circular_edge_without_a_parameter_range_converts() {
    let geometry = solved_curve(SolvedCurveGeometry::Circle(
        CircleCurve::try_new(
            IrPoint3::new(0.0, 0.0, 0.0),
            IrVector3::new(0.0, 0.0, 1.0),
            IrVector3::new(1.0, 0.0, 0.0),
            1.0,
        )
        .unwrap(),
    ));
    let seam = Point3::new(1.0, 0.0, 0.0);
    let curve = convert_curve_between(&geometry, None, (seam, seam))
        .expect("a closed circular edge converts");
    let (start, end) = curve.range_tuple();
    // The rational conversion reparameterizes, so the range need not be the
    // angle sweep; what must hold is that the curve is the whole circle,
    // closed at the seam.
    assert!(curve.subs(start).distance(seam) < 1.0e-9);
    assert!(curve.subs(end).distance(seam) < 1.0e-9);
    let opposite = curve.subs(0.5 * (start + end));
    assert!(
        opposite.distance(Point3::new(-1.0, 0.0, 0.0)) < 1.0e-9,
        "the closed edge must traverse the whole circle, midpoint at {opposite:?}"
    );
}

/// An edge with no curve is a tolerant or degenerate edge. Inventing a straight
/// line between its vertices would put a boundary where the file declined to say
/// there was one.
#[test]
fn an_edge_with_no_curve_is_refused_rather_than_straightened() {
    let mut builder = triangle_sheet();
    builder.ir.model.edges[0].carrier = EdgeCarrier::Free;
    let error = builder.convert().expect_err("a curveless edge must fail");
    assert!(
        matches!(
            error,
            Error::UnsupportedCurveKind {
                kind: "absent (tolerant or degenerate edge)",
                ..
            }
        ),
        "got {error}"
    );
}

/// A coedge-local carrier means the two faces at an edge disagree about where it
/// is, and `CompressedEdge` holds one curve.
#[test]
fn a_coedge_local_carrier_is_refused() {
    let mut builder = triangle_sheet();
    let curve = builder.ir.model.curves[0].id.clone();
    builder.ir.model.coedges[0].use_curve = Some(CoedgeUseCurve {
        curve,
        parameter_range: ParameterInterval::new([0.0, 4.0]).unwrap(),
    });
    let error = builder.convert().expect_err("a tolerant coedge must fail");
    assert!(
        matches!(
            error,
            Error::UnsupportedCurveKind {
                kind: "coedge-local (tolerant edge)",
                ..
            }
        ),
        "got {error}"
    );
}

/// A pole loop is a whole boundary collapsed to one point at a surface
/// singularity. A ring of edge indices cannot express it.
#[test]
fn a_singular_pole_boundary_is_refused() {
    let mut builder = triangle_sheet();
    let vertex = builder.ir.model.vertices[0].id.clone();
    builder.ir.model.loops[0].boundary = LoopBoundary::Vertex {
        vertex,
        pcurves: Vec::new(),
    };
    let error = builder.convert().expect_err("a pole loop must fail");
    assert!(
        matches!(
            error,
            Error::UnsupportedSurfaceKind {
                kind: "singular (pole) face boundary",
                ..
            }
        ),
        "got {error}"
    );
}

/// A reference with no target means the decoded document is not closed.
#[test]
fn a_dangling_reference_names_the_table_it_dangled_out_of() {
    let mut builder = triangle_sheet();
    builder.ir.model.faces[0].surface = ident("test:model:surface#no-such-surface");
    let error = builder.convert().expect_err("a dangling surface must fail");
    assert!(
        matches!(
            error,
            Error::DanglingReference {
                kind: "surface",
                ..
            }
        ),
        "got {error}"
    );
    assert!(error.to_string().contains("no-such-surface"), "got {error}");
}

/// A document with no body at all is a fact about the file, reported as such.
#[test]
fn an_empty_document_reports_no_geometry() {
    let ir = cadmpeg_ir::CadIr::empty();
    let error = crate::cadmpeg::to_bodies(&ir, "TEST").expect_err("nothing to convert");
    assert!(matches!(error, Error::NoGeometry { .. }), "got {error}");
}

// ---------------------------------------------------------------------------
// Placement.
// ---------------------------------------------------------------------------

fn translation(x: f64, y: f64, z: f64) -> IrTransform {
    IrTransform::affine([[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z]]).unwrap()
}

/// A body's placement has to be applied to vertices AND geometry.
#[test]
fn a_body_transform_moves_the_whole_body() {
    let mut builder = triangle_sheet();
    builder.ir.model.bodies[0].transform = Some(translation(10.0, 20.0, 30.0));
    let bodies = builder.convert().expect("the placed triangle converts");
    let sheet = sheet_of(&bodies);
    assert!(sheet.vertices[0].distance(Point3::new(10.0, 20.0, 30.0)) < 1.0e-12);
    let Surface::Plane(plane) = &sheet.faces[0].surface else {
        unreachable!()
    };
    assert!((plane.origin().z - 30.0).abs() < 1.0e-12);
    let (start, _) = sheet.edges[0].curve.range_tuple();
    assert!(
        sheet.edges[0]
            .curve
            .subs(start)
            .distance(Point3::new(10.0, 20.0, 30.0))
            < 1.0e-12
    );
}

/// A row-major transform read as column-major transposes every placement, which
/// for a rotation inverts it.
#[test]
fn a_row_major_transform_is_not_read_transposed() {
    // A quarter turn about +z: x -> y, y -> -x.
    let transform = IrTransform::affine([
        [0.0, -1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ])
    .unwrap();
    let turned = frame::matrix(&transform).transform_point(Point3::new(1.0, 0.0, 0.0));
    assert!(
        turned.distance(Point3::new(0.0, 1.0, 0.0)) < 1.0e-12,
        "+x turned to {turned:?}; transposed would give (0, -1, 0)"
    );
}

// ---------------------------------------------------------------------------
// Assemblies: an instanced part appears at every instance.
// ---------------------------------------------------------------------------

fn definition(builder: &mut Builder, bodies: &[&str]) -> String {
    let id = builder.id("product");
    builder
        .ir
        .model
        .product_definitions
        .push(ProductDefinition {
            id: ident(&id),
            kind: ProductDefinitionKind::Part,
            source_name: None,
            label: None,
            description: None,
            part_number: None,
            bom_properties: Default::default(),
            bodies: bodies.iter().map(|body| ident(body)).collect(),
            sketches: Vec::new(),
            native_ref: None,
        });
    id
}

fn occurrence(
    builder: &mut Builder,
    definition: &str,
    parent: Option<&str>,
    transform: IrTransform,
    scale: f64,
) -> String {
    let id = builder.id("occurrence");
    let scale = FiniteReal::new(scale).unwrap();
    builder.ir.model.occurrences.push(Occurrence {
        id: ident(&id),
        prototype: PrototypeReference::Local {
            definition: ident(definition),
        },
        parent: match parent {
            None => OccurrenceParent::Root {},
            Some(parent) => OccurrenceParent::Occurrence {
                occurrence: ident(parent),
            },
        },
        ordinal: 0,
        transform,
        linked_prototype: None,
        scale: [scale; 3],
        name: None,
        visible: None,
        link: None,
        native_ref: None,
    });
    id
}

fn first_vertex(body: &ImportedBody) -> Point3 {
    match body {
        ImportedBody::Sheet(sheet) => sheet.vertices[0],
        ImportedBody::Solid(solid) => solid.boundaries[0].vertices[0],
    }
}

/// An assembly that uses one part twice, through a sub-assembly placed at
/// (100, 0, 0): the part's ONE body must come back twice, each at the full
/// chain of placements down to it. Reading `model.bodies` alone returned it
/// once, at its definition's origin -- the silent drop this replaces.
#[test]
fn an_instanced_part_appears_at_every_instance() {
    let mut builder = triangle_sheet();
    let body = builder.ir.model.bodies[0].id.to_string();
    let part = definition(&mut builder, &[&body]);
    let assembly = definition(&mut builder, &[]);
    let root = occurrence(
        &mut builder,
        &assembly,
        None,
        translation(100.0, 0.0, 0.0),
        1.0,
    );
    occurrence(
        &mut builder,
        &part,
        Some(&root),
        translation(10.0, 0.0, 0.0),
        1.0,
    );
    occurrence(
        &mut builder,
        &part,
        Some(&root),
        translation(0.0, 20.0, 0.0),
        1.0,
    );
    let recovered = builder.import().expect("the assembly converts");
    assert!(recovered.refused.is_empty(), "{}", recovered.summary());
    assert_eq!(recovered.bodies.len(), 2, "one body per placed instance");
    let corners: Vec<Point3> = recovered.bodies.iter().map(first_vertex).collect();
    assert!(
        corners[0].distance(Point3::new(110.0, 0.0, 0.0)) < 1.0e-12,
        "{corners:?}"
    );
    assert!(
        corners[1].distance(Point3::new(100.0, 20.0, 0.0)) < 1.0e-12,
        "{corners:?}"
    );
    // The geometry moved with the vertices.
    for (body, corner) in recovered.bodies.iter().zip(&corners) {
        let ImportedBody::Sheet(sheet) = body else {
            unreachable!()
        };
        let (start, _) = sheet.edges[0].curve.range_tuple();
        assert!(sheet.edges[0].curve.subs(start).distance(*corner) < 1.0e-12);
    }
}

/// A body's own placement inside its definition composes UNDER the
/// occurrence's: world = occurrence chain, then body.
#[test]
fn a_body_placement_composes_under_its_occurrence() {
    let mut builder = triangle_sheet();
    builder.ir.model.bodies[0].transform = Some(translation(0.0, 0.0, 5.0));
    let body = builder.ir.model.bodies[0].id.to_string();
    let part = definition(&mut builder, &[&body]);
    occurrence(&mut builder, &part, None, translation(1.0, 0.0, 0.0), 1.0);
    let recovered = builder.import().expect("the placed part converts");
    assert_eq!(recovered.bodies.len(), 1);
    assert!(first_vertex(&recovered.bodies[0]).distance(Point3::new(1.0, 0.0, 5.0)) < 1.0e-12);
}

/// The review's F6: two translations commute, so the placement row above cannot
/// tell `world.compose(local)` from `local.compose(world)`. A body scale under an
/// occurrence rotation plus translation discriminates: the body's own scale is
/// applied BEFORE the world placement.
#[test]
fn a_body_scale_composes_before_the_occurrence_rotation_and_translation() {
    let mut builder = triangle_sheet();
    builder.ir.model.bodies[0].transform = Some(
        IrTransform::affine([
            [2.0, 0.0, 0.0, 0.0],
            [0.0, 2.0, 0.0, 0.0],
            [0.0, 0.0, 2.0, 0.0],
        ])
        .expect("a uniform scale is affine"),
    );
    let body = builder.ir.model.bodies[0].id.to_string();
    let part = definition(&mut builder, &[&body]);
    occurrence(
        &mut builder,
        &part,
        None,
        // Quarter turn about z, then translate by (100, 0, 0).
        IrTransform::affine([
            [0.0, -1.0, 0.0, 100.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ])
        .expect("a rigid placement is affine"),
        1.0,
    );
    let recovered = builder.import().expect("the placed part converts");
    let ImportedBody::Sheet(sheet) = &recovered.bodies[0] else {
        panic!("the triangle is a sheet");
    };
    // Body vertex (4, 0, 0): scale 2 gives (8, 0, 0), the quarter turn gives
    // (0, 8, 0), the translation gives (100, 8, 0). Applying the body scale
    // after the world placement would give (200, 8, 0).
    assert!(
        sheet.vertices[1].distance(Point3::new(100.0, 8.0, 0.0)) < 1.0e-12,
        "got {:?}",
        sheet.vertices[1]
    );
}

/// No product structure, or a definition nothing places: the body appears once
/// at its own placement, exactly as before 0.6.
#[test]
fn an_unplaced_body_appears_once_where_it_is() {
    let recovered = triangle_sheet().import().expect("a lone body converts");
    assert_eq!(recovered.bodies.len(), 1);
    let mut builder = triangle_sheet();
    let body = builder.ir.model.bodies[0].id.to_string();
    definition(&mut builder, &[&body]);
    let recovered = builder.import().expect("an unplaced definition converts");
    assert_eq!(recovered.bodies.len(), 1);
    assert!(first_vertex(&recovered.bodies[0]).distance(Point3::origin()) < 1.0e-12);
}

/// An occurrence with a per-axis scale is refused by name: the representation
/// does not say how the scale composes with the transform, so any placement
/// would be a guess. The OTHER instance of the same part still converts.
#[test]
fn a_scaled_occurrence_is_refused_by_name_and_does_not_sink_the_others() {
    let mut builder = triangle_sheet();
    let body = builder.ir.model.bodies[0].id.to_string();
    let part = definition(&mut builder, &[&body]);
    occurrence(&mut builder, &part, None, translation(10.0, 0.0, 0.0), 1.0);
    occurrence(&mut builder, &part, None, translation(0.0, 20.0, 0.0), 2.0);
    let recovered = builder.import().expect("the assembly converts");
    assert_eq!(recovered.bodies.len(), 1, "{}", recovered.summary());
    assert_eq!(recovered.refused.len(), 1);
    assert!(
        matches!(
            recovered.refused[0].reason,
            Error::UnplaceableOccurrence {
                kind: "scaled assembly occurrence",
                ..
            }
        ),
        "got {}",
        recovered.refused[0].reason
    );
    assert!(recovered.refused[0].body.contains("occurrence"));
    assert_eq!(recovered.total(), 2, "both instances are accounted for");
}

/// A body that fails to convert is refused ONCE, naming how many instances it
/// covered, rather than vanishing from every place it was used.
#[test]
fn an_unconvertible_instanced_body_is_refused_for_all_its_instances() {
    let mut builder = triangle_sheet();
    builder.ir.model.edges[0].carrier = EdgeCarrier::Free;
    let body = builder.ir.model.bodies[0].id.to_string();
    let part = definition(&mut builder, &[&body]);
    occurrence(&mut builder, &part, None, translation(10.0, 0.0, 0.0), 1.0);
    occurrence(&mut builder, &part, None, translation(0.0, 20.0, 0.0), 1.0);
    let recovered = builder.import().expect("the call itself succeeds");
    assert!(recovered.bodies.is_empty());
    assert_eq!(recovered.refused.len(), 1);
    assert!(
        recovered.refused[0].body.contains("all 2 instances"),
        "got {}",
        recovered.refused[0].body
    );
}

/// An occurrence tree with a cycle places nothing honestly, so it fails the
/// document rather than any one body.
#[test]
fn an_occurrence_cycle_fails_the_document() {
    let mut builder = triangle_sheet();
    let body = builder.ir.model.bodies[0].id.to_string();
    let part = definition(&mut builder, &[&body]);
    let first = builder.id("occurrence");
    let second = occurrence(
        &mut builder,
        &part,
        Some(&first),
        IrTransform::identity(),
        1.0,
    );
    let mut looped = builder.ir.model.occurrences[0].clone();
    looped.id = ident(&first);
    looped.parent = OccurrenceParent::Occurrence {
        occurrence: ident(&second),
    };
    builder.ir.model.occurrences.push(looped);
    let error = builder.import().expect_err("a cycle fails the document");
    assert!(
        matches!(error, Error::MalformedGeometry { ref detail, .. } if detail.contains("occurrence tree")),
        "got {error}"
    );
}

// ---------------------------------------------------------------------------
// The decoder's loss report reaches the caller.
// ---------------------------------------------------------------------------

/// `import_reported` carries the decoder's losses onto the [`Import`], and the
/// one-line summary says so. The decode wrapper is the only producer of these,
/// and it is exercised over real files in `tests/`; this pins the plumbing.
#[test]
fn decoder_losses_ride_on_the_import_and_reach_the_summary() {
    let builder = triangle_sheet();
    let losses = vec![
        crate::cadmpeg::DecoderLoss {
            code: "test.dropped".to_owned(),
            severity: crate::cadmpeg::LossSeverity::Warning,
            message: "one entity was dropped".to_owned(),
            provenance: None,
        },
        crate::cadmpeg::DecoderLoss {
            code: "test.note".to_owned(),
            severity: crate::cadmpeg::LossSeverity::Info,
            message: "for information".to_owned(),
            provenance: None,
        },
    ];
    let recovered =
        crate::cadmpeg::import_reported(&builder.ir, "TEST", losses.clone()).expect("converts");
    assert_eq!(recovered.losses, losses);
    assert!(
        recovered.summary().contains("2 losses"),
        "{}",
        recovered.summary()
    );
    assert_eq!(
        recovered
            .losses_at_least(crate::cadmpeg::LossSeverity::Warning)
            .count(),
        1
    );
    // And a document handed in without a decode claims no losses rather than
    // inventing an empty report's worth of confidence: the field is empty.
    assert!(builder.import().unwrap().losses.is_empty());
}

// ---------------------------------------------------------------------------

/// A conversion context over a document, for the tests that exercise one carrier
/// rather than a whole body.
fn context_over(ir: &cadmpeg_ir::CadIr) -> Context<'_> { Context::new(ir, "TEST") }
